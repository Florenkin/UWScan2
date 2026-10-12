#include "scanner/reconstructor.h"
#include "pointcloud/pc.hpp"
#include "scanner/ImgProcess.h"
#include "scanner/RuiQuadricSurfaceFitting.h"
#include <fstream>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>

NAMESPACE_BEGIN { namespace scanner {
    static int mapLaserPositionToCalibrationIndex(const ReconstructionFramePtr& frame,
                                                  int surfBeginPos,
                                                  int surfEndPos,
                                                  int surfStep) {
        const int lower = std::min(surfBeginPos, surfEndPos);
        const int upper = std::max(surfBeginPos, surfEndPos);
        const int centiIndex = static_cast<int>(std::llround(frame->laserPosition * 100.0));
        if (centiIndex >= lower && centiIndex <= upper) {
            return centiIndex;
        }

        const int directIndex = static_cast<int>(std::llround(frame->laserPosition));
        if (directIndex >= lower && directIndex <= upper) {
            return directIndex;
        }

        if (std::abs(frame->scanEndPosition - frame->scanStartPosition) < 1e-9) {
            return frame->scanIndex;
        }

        const double t = (frame->laserPosition - frame->scanStartPosition) /
                         (frame->scanEndPosition - frame->scanStartPosition);
        const double rawIndex = surfBeginPos + t * (surfEndPos - surfBeginPos);
        const int step = std::max(std::abs(surfStep), 1);
        int mappedIndex = surfBeginPos + static_cast<int>(std::llround((rawIndex - surfBeginPos) / step)) * step;

        return std::clamp(mappedIndex, lower, upper);
    }

    static uchar sampleGrayBilinear(const cv::Mat& gray, const cv::Point2d& point) {
        if (gray.empty()) {
            return 0;
        }
        const double x = std::clamp(point.x, 0.0, static_cast<double>(gray.cols - 1));
        const double y = std::clamp(point.y, 0.0, static_cast<double>(gray.rows - 1));
        const int x0 = static_cast<int>(std::floor(x));
        const int y0 = static_cast<int>(std::floor(y));
        const int x1 = std::min(x0 + 1, gray.cols - 1);
        const int y1 = std::min(y0 + 1, gray.rows - 1);
        const double dx = x - x0;
        const double dy = y - y0;
        const double v00 = gray.at<uchar>(y0, x0);
        const double v10 = gray.at<uchar>(y0, x1);
        const double v01 = gray.at<uchar>(y1, x0);
        const double v11 = gray.at<uchar>(y1, x1);
        const double value = (1.0 - dx) * (1.0 - dy) * v00
                           + dx * (1.0 - dy) * v10
                           + (1.0 - dx) * dy * v01
                           + dx * dy * v11;
        return static_cast<uchar>(std::clamp(std::lround(value), 0l, 255l));
    }

    static double centerLineCoverageRatio(const std::vector<cv::Point2d>& points,
                                          const cv::Size& imageSize,
                                          bool isHorizontalScan) {
        if (points.empty() || imageSize.width <= 0 || imageSize.height <= 0) {
            return 0.0;
        }

        double minCoord = std::numeric_limits<double>::max();
        double maxCoord = std::numeric_limits<double>::lowest();
        for (const auto& point : points) {
            const double coord = isHorizontalScan ? point.x : point.y;
            minCoord = std::min(minCoord, coord);
            maxCoord = std::max(maxCoord, coord);
        }

        const double fullSpan = static_cast<double>(isHorizontalScan ? imageSize.width : imageSize.height);
        if (fullSpan <= 1.0 || maxCoord < minCoord) {
            return 0.0;
        }
        return std::clamp((maxCoord - minCoord + 1.0) / fullSpan, 0.0, 1.0);
    }

    static bool shouldUseCenterLineFallback(const std::vector<cv::Point2d>& points,
                                            const cv::Size& imageSize,
                                            bool isHorizontalScan) {
        const int expectedSpan = isHorizontalScan ? imageSize.width : imageSize.height;
        if (expectedSpan <= 0) {
            return false;
        }

        const double coverage = centerLineCoverageRatio(points, imageSize, isHorizontalScan);
        return points.size() < static_cast<size_t>(std::max(80, expectedSpan / 4)) || coverage < 0.45;
    }

    Reconstructor::Reconstructor(QObject* parent): QObject(parent){
        // 默认折射参数
        m_mu = Eigen::Vector3d(1.0, 1.5, 1.333);
        m_d0 = 20.7692;
        m_d1 = 5.0;
        m_axis = Eigen::Vector3d(-0.00046551, 0.00169085, 0.999998);
        qRegisterMetaType<pointcloud::PointCloudPtr>("pointcloud::PointCloudPtr");
        qRegisterMetaType<std::vector<cv::Point2d>>("std::vector<cv::Point2d>");
    }

    Reconstructor::~Reconstructor(){
        stopReconstruction();
        if(m_thread.joinable()){
            m_thread.join();
        }
    }

    void Reconstructor::startReconstruction(){
        m_isRunning = true;
        m_thread = std::thread(&Reconstructor::working, this);
    }

    void Reconstructor::stopReconstruction(){
        QMutexLocker lock(&m_queueMutex);
        m_isRunning = false;
        m_condition.wakeAll();
    }

    void Reconstructor::working(){
        std::cout << ">>> [Debug] Reconstructor::startReconstruction() -> Start Reconstruction" << std::endl;
        auto pointsManager = AssociatedPointsManager::getInstance();
        pointsManager->start();
        while(m_isRunning){
            QMutexLocker lock(&m_queueMutex);
            while(m_frameQueue.empty() && m_isRunning){
                m_condition.wait(&m_queueMutex);
            }
            
            if(!m_isRunning) break;

            ReconstructionFramePtr frame = m_frameQueue.front();
            m_frameQueue.pop();
            
            // 激光中心线的预设元素
            std::vector<cv::Point2d> linePoints;
            AssociatedPointsPtr associatedPoints = std::make_shared<std::vector<AssociatedPoints>>();
            auto pc = processFrame(frame, linePoints, associatedPoints);
            if(pc.has_value()){
                emit pointCloudReady(pc.value(), frame->singleScan);
                emit linePointsReady(linePoints, frame->singleScan);
                // emit associatedPointsReady(associatedPoints, frame->singleScan);
                if(!frame->singleScan && frame->savePointCloud){
                    //这里直接使用保存的路径为sessionId
                    QString sessionId = frame->sessionId;
                    pointsManager->addAssociatedPoints(sessionId, associatedPoints, frame->image.size(), frame->image);
                }
            } else if (!frame->singleScan) {
                std::cout << "[Debug] Reconstructor::working() skipped frame scanIndex="
                          << frame->scanIndex << " reason=" << m_lastError << std::endl;
                if (frame->savePointCloud && !frame->sessionId.isEmpty()) {
                    pointsManager->addAssociatedPoints(frame->sessionId,
                                                       std::make_shared<std::vector<AssociatedPoints>>(),
                                                       frame->image.size(),
                                                       frame->image);
                }
                emit reconstructionFailed(frame->scanIndex, QString::fromStdString(m_lastError), frame->singleScan);
            }
        }
        pointsManager->stop();
    }

    bool Reconstructor::init(const ReconstructionConfig& config){
        cv::FileStorage fs_read;
        //分配局部变量
        cv::Mat temp_cameraMatrix;
        cv::Mat temp_distCoeffs;
        cv::Size temp_picSize;
        Eigen::Vector3d temp_mu, temp_axis;
        double temp_d0, temp_d1;
        int temp_surfBeginPos, temp_surfEndPos, temp_surfStep, temp_surfNum;
        std::vector<Eigen::Matrix4f> temp_allTransMats;
        std::vector<Eigen::VectorXf> temp_allSurfacePars;
        std::vector<std::pair<double, double>> temp_allMaxMinZValues;

        try {
            // 读取相机内参
            fs_read.open(config.calibrationPath, cv::FileStorage::READ);
            if (!fs_read.isOpened()) {
                m_lastError = "Failed to open YAML: " + config.calibrationPath;
                std::cout << "[Debug] Reconstructor::init() -> Failed to open YAML: " << config.calibrationPath << std::endl;
                return false;
            }

            fs_read["CameraMatrix"] >> temp_cameraMatrix;
            fs_read["DistCoeffs"] >> temp_distCoeffs;
            fs_read["picSize"] >> temp_picSize;

            // 读取折射参数 (如果文件中有则覆盖默认值)
            if (!fs_read["mu"].empty()) {
                cv::Mat temp;
                fs_read["mu"] >> temp;
                Eigen::MatrixXd tempEig;      
                cv::cv2eigen(temp, tempEig);  
                temp_mu = tempEig;               
            }
            if (!fs_read["d0"].empty()) fs_read["d0"] >> temp_d0;
            if (!fs_read["d1"].empty()) fs_read["d1"] >> temp_d1;
            if (!fs_read["A"].empty()) {
                cv::Mat temp;
                fs_read["A"] >> temp;
                cv::cv2eigen(temp, temp_axis);
            }

            // 读取运动和曲面参数
            fs_read["surfBeginPos"] >> temp_surfBeginPos;
            fs_read["surfEndPos"] >> temp_surfEndPos;
            fs_read["surfStep"] >> temp_surfStep;
            fs_read["surfNum"] >> temp_surfNum;

            cv::FileNode transNode = fs_read["allTransMats"];
            for (int i = 0; i < temp_surfNum; i++) {
                cv::Mat trans;
                transNode[i]["TransMats"] >> trans;
                Eigen::Matrix4f temp;
                cv::cv2eigen(trans, temp);
                temp_allTransMats.push_back(temp);
            }

            cv::FileNode surfNode = fs_read["allSurfacePars"];
            for (int i = 0; i < temp_surfNum; i++) {
                cv::Mat surPars;
                surfNode[i]["SurfPars"] >> surPars;
                Eigen::VectorXf temp;
                cv::cv2eigen(surPars, temp);
                temp_allSurfacePars.push_back(temp);
            }

            cv::FileNode zNode = fs_read["allMaxMinZValues"];
            for (int i = 0; i < temp_surfNum; i++) {
                double maxZ, minZ;
                zNode[i]["MaxZValue"] >> maxZ;
                zNode[i]["MinZValue"] >> minZ;
                temp_allMaxMinZValues.push_back({maxZ, minZ});
            }

            fs_read.release();
        } catch (const cv::Exception& e) {
            m_lastError = "CV Exception: " + std::string(e.what());
            if(fs_read.isOpened()) fs_read.release();
            return false;
        }

        QMutexLocker lock(&m_calibrationMutex);
        m_isInitialized = true;
        m_config = config;
        m_config.thres = static_cast<int>(m_config.bwThr);
        m_config.slt_thres = static_cast<int>(m_config.sumThr);
        m_cameraMatrix = temp_cameraMatrix;
        m_distCoeffs = temp_distCoeffs;
        m_picSize = temp_picSize;
        m_mu = temp_mu;
        m_axis = temp_axis;
        m_d0 = temp_d0;
        m_d1 = temp_d1;
        m_surfBeginPos = temp_surfBeginPos;
        m_surfEndPos = temp_surfEndPos;
        m_surfStep = temp_surfStep;
        m_surfNum = temp_surfNum;
        m_allTransMats = std::move(temp_allTransMats);
        m_allSurfacePars = std::move(temp_allSurfacePars);
        m_allMaxMinZValues = std::move(temp_allMaxMinZValues);
        std::cout << "[Debug] Reconstructor::init() -> Calibration Path: " << m_config.calibrationPath << std::endl;
        std::cout << "[Debug] Reconstructor::init() -> surf range: " << m_surfBeginPos
                  << " to " << m_surfEndPos << " step " << m_surfStep << std::endl;
        assert(m_allTransMats.size() == m_allSurfacePars.size());
        assert(m_allTransMats.size() == m_allMaxMinZValues.size());
        emit loadCalibration(QString::fromStdString(m_config.calibrationPath));
        return true;        
    }

    void Reconstructor::reconstructFrame(ReconstructionFramePtr frame){
        QMutexLocker lock(&m_queueMutex);
        std::cout << "[Debug] Reconstructor::reconstructFrame() -> scanIndex: " << frame->scanIndex << std::endl;
        m_frameQueue.push(frame);
        m_condition.wakeOne();
    }

    std::vector<cv::Point2d> Reconstructor::extractPreviewCenterLine(const cv::Mat& image) {
        QMutexLocker lock(&m_calibrationMutex);
        if (image.empty()) {
            return {};
        }

        std::vector<int> currentRoi = m_config.isFullRoi
            ? std::vector<int>{0, 0, image.cols, image.rows}
            : m_config.Roi;
        if (currentRoi.size() < 4) {
            currentRoi = {0, 0, image.cols, image.rows};
        }

        cv::Rect imageRect(0, 0, image.cols, image.rows);
        cv::Rect roiRect(currentRoi[0], currentRoi[1], currentRoi[2], currentRoi[3]);
        roiRect &= imageRect;
        if (roiRect.empty()) {
            return {};
        }

        cv::Mat roiImage = image(roiRect).clone();
        std::vector<cv::Point2d> points = extractCenterLine(roiImage);
        for (auto& point : points) {
            point.x += roiRect.x;
            point.y += roiRect.y;
        }
        return points;
    }

    std::vector<cv::Point2d> Reconstructor::extractCenterLine(const cv::Mat& img){
        std::vector<cv::Point2d> linePoints;
        const int GrayCenter = 0;
        const int Steger = 1;
        const int MultiThres = 2;

        if (m_config.isHeng) {
            if (m_config.centerlineMethod == GrayCenter) {
                linePoints = findLaserCenterGrayCenter_heng(img, m_config.bwThr, m_config.sumThr);
            }
            else if (m_config.centerlineMethod == Steger) {
                linePoints = findLaserCenterSteger_v5_heng(img, m_config.bwThr, m_config.sumThr, 7, true);
            }
            else if (m_config.centerlineMethod == MultiThres) {
                linePoints = findLaserCenterMultiThres_heng(img, 0, 10, 0);
            }
        } else {
            if (m_config.centerlineMethod == GrayCenter) {
                linePoints = findLaserCenterGrayCenter_shu(img, m_config.bwThr);
            }
            else if (m_config.centerlineMethod == Steger) {
                linePoints = findLaserCenterSteger_v5_shu(img, m_config.bwThr, m_config.sumThr, 7, true);
            }
            else if (m_config.centerlineMethod == MultiThres) {
                linePoints = findLaserCenterMultiThres_shu(img, 0, 10, 0);
            }
        }

        if (shouldUseCenterLineFallback(linePoints, img.size(), m_config.isHeng)) {
            const double relaxedBwThr = std::max(5.0, m_config.bwThr * 0.55);
            const double relaxedSumThr = std::max(10.0, m_config.sumThr * 0.45);
            std::vector<cv::Point2d> fallbackPoints = m_config.isHeng
                ? findLaserCenterGrayCenter_heng(img, relaxedBwThr, relaxedSumThr)
                : findLaserCenterGrayCenter_shu(img, relaxedBwThr);

            const double currentCoverage = centerLineCoverageRatio(linePoints, img.size(), m_config.isHeng);
            const double fallbackCoverage = centerLineCoverageRatio(fallbackPoints, img.size(), m_config.isHeng);
            if (fallbackPoints.size() > linePoints.size() && fallbackCoverage > currentCoverage) {
                linePoints = std::move(fallbackPoints);
            }
        }
        return linePoints;
    }
    
    std::optional<pointcloud::PointCloudPtr> Reconstructor::processFrame(const ReconstructionFramePtr& frame, std::vector<cv::Point2d>& ret_centerline, AssociatedPointsPtr associatedPoints){
        QMutexLocker lock(&m_calibrationMutex);
        // std::cout << "[Debug] Reconstructor::processFrame() -> scanIndex: " << frame->scanIndex << std::endl;
        // 这里没有采用局部变量的形式去复制一份标定数据出来，是因为
        // 标定数据太大了，如果每次都要复制，太浪费时间了，本身锁的竞争就不是激烈
        if (!m_isInitialized) {
            // std::cout << "[Debug] Reconstructor::processFrame() -> Initialize Judge: " << m_isInitialized << std::endl;
            m_lastError = "Reconstructor not initialized.";
            return std::nullopt;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // std::cout << "[Debug] Reconstructor:processFrame() -> initialize cloud" << std::endl;
        if (frame->image.empty()) {
            m_lastError = "Input image is empty.";
            return std::nullopt;
        }
        cv::Mat grayImage;
        if (frame->image.channels() == 1) {
            grayImage = frame->image;
        } else {
            cv::cvtColor(frame->image, grayImage, cv::COLOR_BGR2GRAY);
        }

        const int calibrationIndex = mapLaserPositionToCalibrationIndex(frame, m_surfBeginPos, m_surfEndPos, m_surfStep);

        // 计算当前帧对应的索引
        // 确保 scanIndex 在有效范围内
        const int rangeLower = std::min(m_surfBeginPos, m_surfEndPos);
        const int rangeUpper = std::max(m_surfBeginPos, m_surfEndPos);
        if (calibrationIndex < rangeLower || calibrationIndex > rangeUpper) {
            // 超出范围，忽略
            std::cout << "[Debug] Reconstructor::processFrame() -> scanIndex out of bounds: "
                      << calibrationIndex << " laserPosition=" << frame->laserPosition << std::endl;
            m_lastError = "scanIndex out of bounds. laserPosition=" +
                          std::to_string(frame->laserPosition) +
                          ", calibrationIndex=" + std::to_string(calibrationIndex) +
                          ", range=[" + std::to_string(m_surfBeginPos) +
                          ", " + std::to_string(m_surfEndPos) + "]";
            return std::nullopt; 
        }
        
        // std::cout << "[Debug] Reconstructor::processFrame() -> scanIndex: " << frame->scanIndex << std::endl;
        // std::cout << "[Debug] Reconstructor::processFrame() -> m_surfBeginPos: " << m_surfBeginPos << std::endl;
        // std::cout << "[Debug] Reconstructor::processFrame() -> m_surfStep: " << m_surfStep << std::endl;
        // std::cout << "[Debug] Reconstructor::processFrame() -> m_surfEndPos: " << m_surfEndPos << std::endl;
        // std::cout << "[Debug] Reconstructor::processFrame() -> m_allTransMats.size(): " << m_allTransMats.size() << std::endl;

        int imgNo = (calibrationIndex - m_surfBeginPos) / m_surfStep;
        if (imgNo < 0 || imgNo >= m_allTransMats.size()) {
            // std::cout << "[Debug] Reconstructor::processFrame() -> imgNo: " << imgNo << std::endl;
            m_lastError = "Calculated imgNo out of bounds.";
            return std::nullopt; 
        }

        // ROI 裁剪
        cv::Mat imgSlt;
        std::vector<int> currentRoi = m_config.isFullRoi ? std::vector<int>{0, 0, frame->image.cols, frame->image.rows} : m_config.Roi;
        // 简单的边界检查
        if(currentRoi[0] + currentRoi[2] > frame->image.cols || currentRoi[1] + currentRoi[3] > frame->image.rows) {
            m_lastError = "ROI out of bounds.";
            return std::nullopt; 
        }
        imgSlt = frame->image(cv::Rect(currentRoi[0], currentRoi[1], currentRoi[2], currentRoi[3])).clone();

        // 提取中心线
        std::vector<cv::Point2d> linePoints = extractCenterLine(imgSlt);
        // std::cout << "[Debug] Reconstructor::processFrame() -> extractCenterLine() linePoints.size(): " << linePoints.size() << std::endl;
        if (linePoints.size() < 10) {
            m_lastError = "Too few line points.";
            return std::nullopt; // 点太少，不值得重建
        }

        // 还原 ROI 偏移
        for (auto& pt : linePoints) {
            pt.x += currentRoi[0];
            pt.y += currentRoi[1];
        }
        
        // 保存一份原图坐标的中心线点用于 UI 显示和深度图回填
        ret_centerline = linePoints;

        if(!associatedPoints) {
            associatedPoints = std::make_shared<std::vector<AssociatedPoints>>();
        }

        // 畸变校正。离线图片可能不是标定分辨率，重建射线必须使用标定图像坐标。
        std::vector<cv::Point2d> distortedPoints = linePoints; // 原图坐标，用于灰度采样和深度图回填
        std::vector<cv::Point2d> reconstructionPoints = linePoints;
        if (m_picSize.width > 0 && m_picSize.height > 0 &&
            (frame->image.cols != m_picSize.width || frame->image.rows != m_picSize.height)) {
            const double scaleX = static_cast<double>(m_picSize.width) / static_cast<double>(frame->image.cols);
            const double scaleY = static_cast<double>(m_picSize.height) / static_cast<double>(frame->image.rows);
            for (auto& point : reconstructionPoints) {
                point.x *= scaleX;
                point.y *= scaleY;
            }
        }
        cv::undistortPoints(reconstructionPoints, reconstructionPoints, m_cameraMatrix, m_distCoeffs, cv::Mat(), m_cameraMatrix);
        // std::cout << "[Debug] Reconstructor::processFrame() -> undistortPoints() linePoints.size(): " << linePoints.size() << std::endl;

        // 准备计算参数
        Eigen::Matrix3f TransR = m_allTransMats[imgNo].block(0, 0, 3, 3);
        Eigen::Vector3f Transt = m_allTransMats[imgNo].block(0, 3, 3, 1);
        Eigen::Matrix3f TransRInv = TransR.transpose();
        Eigen::Vector3f TranstInv = -TransR.transpose() * Transt;
        
        double maxZ = m_allMaxMinZValues[imgNo].first;
        double minZ = m_allMaxMinZValues[imgNo].second;

        Eigen::MatrixXf vpVecMat = Eigen::MatrixXf::Zero(3, reconstructionPoints.size());
        Eigen::MatrixXf vpPosMat = Eigen::MatrixXf::Zero(3, reconstructionPoints.size());
        Eigen::Matrix3d cameraMatrix_eig;
        cv::cv2eigen(m_cameraMatrix, cameraMatrix_eig);
        Eigen::Matrix3d cameraMatrixInv = cameraMatrix_eig.inverse();

        // 射线折射计算 (Air -> Glass -> Water)
        // 注意：这里只对应水中重建的参数逻辑
        // 这里暂且保持原代码的逻辑
        for (size_t k = 0; k < reconstructionPoints.size(); k++) {
            Eigen::Vector3d tempUV = { reconstructionPoints[k].x, reconstructionPoints[k].y, 1.0 };
            Eigen::Vector3d v0 = cameraMatrixInv * tempUV;
            v0.normalize();

            if (frame->isInWater) {
                Eigen::Vector3d v1, v2;
                refract_line(m_mu[0], m_mu[1], v0, m_axis, v1);
                refract_line(m_mu[0], m_mu[2], v0, m_axis, v2); // 这里原代码似乎有些奇怪，用了mu[2]和v0计算v2? 保持原样。
                
                // 计算新的起点 q2
                Eigen::Vector3d q2 = (m_d0 / v0[2]) * v0 + (m_d1 / v1[2]) * v1;
                
                vpVecMat.col(k) = v2.cast<float>();
                vpPosMat.col(k) = q2.cast<float>();
            } else {
                vpVecMat.col(k) = v0.cast<float>();
                vpPosMat.col(k) = Eigen::Vector3f::Zero(); // 相机光心
            }
        }

        // 变换到局部坐标系并求交
        vpVecMat = TransR * vpVecMat;
        vpPosMat = TransR * vpPosMat + Transt * Eigen::MatrixXf::Ones(1, reconstructionPoints.size());

        std::vector<Eigen::Vector3f> intersectPts;
        intersectPts.reserve(linePoints.size());

        // 记录有效点的索引，用于保持三种点的对应关系
        std::vector<size_t> validIndices;
        validIndices.reserve(linePoints.size());

        for (int k = 0; k < vpVecMat.cols(); k++) {
            Eigen::VectorXf vp2(6);
            vp2 << vpPosMat.col(k), vpVecMat.col(k);
            Eigen::Vector3f tempintersectPt;
            
            // TODO:调用曲面求交函数
            Rui::GetQuadricSurfIntersectPtNewForm(m_allSurfacePars[imgNo], vp2, tempintersectPt, maxZ, minZ);

            // 筛选无效点
            if (std::abs(tempintersectPt(0)) < 1e-6 && std::abs(tempintersectPt(1)) < 1e-6 && std::abs(tempintersectPt(2)) < 1e-6) {
                continue;
            }
            intersectPts.push_back(tempintersectPt);
            validIndices.push_back(k);
        }

        // 变换回世界坐标系并生成点云
        Eigen::MatrixXf reconstrcut3Dpts = Eigen::MatrixXf::Zero(3, intersectPts.size());
        for (size_t k = 0; k < intersectPts.size(); k++) {
            reconstrcut3Dpts.col(k) = intersectPts[k];
        }
        
        if (!m_config.keepLocalCoordinates) {
            reconstrcut3Dpts = TransRInv * reconstrcut3Dpts + TranstInv * Eigen::MatrixXf::Ones(1, intersectPts.size());
        }

        // 清空关联点的数据并重新填充有效点
        associatedPoints->clear();
        associatedPoints->reserve(validIndices.size());

        for (int k = 0; k < validIndices.size(); k++) {
            size_t validIndex = validIndices[k];
            pcl::PointXYZ point;
            point.x = reconstrcut3Dpts(0, k);
            point.y = reconstrcut3Dpts(1, k);
            point.z = reconstrcut3Dpts(2, k);
            cloud->push_back(point);

            const uchar gray = sampleGrayBilinear(grayImage, distortedPoints[validIndex]);
            associatedPoints->emplace_back(reconstructionPoints[validIndex], distortedPoints[validIndex], point, gray);
        }

        // 数据导出和保存
        // std::cout << ">>> [Debug] Reconstructor::reconstructCurrent() -> Reconstructed cloud size: " << cloud->size() << std::endl;
        return std::make_shared<pointcloud::PointCloudConcrete<pcl::PointXYZ>>(cloud, frame->pointcloudName);
    }
}}
