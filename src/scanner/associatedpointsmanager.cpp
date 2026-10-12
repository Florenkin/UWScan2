// associatedpointsmanager.cpp
#include "scanner/associatedpointsmanager.h"
#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <pcl/io/ply_io.h>

NAMESPACE_BEGIN { namespace scanner {

namespace {
QString normalizedImageFormat(const QString& format, const QString& fallback = "png") {
    QString ret = format.trimmed().toLower();
    if (ret.startsWith('.')) {
        ret.remove(0, 1);
    }
    return ret.isEmpty() ? fallback : ret;
}

std::vector<AssociatedPoints> flattenSessionPoints(const ScanSessionPtr& session) {
    std::vector<AssociatedPoints> points;
    if (!session) {
        return points;
    }
    size_t total = 0;
    for (const auto& frame : session->frameData) {
        if (frame) {
            total += frame->size();
        }
    }
    points.reserve(total);
    for (const auto& frame : session->frameData) {
        if (!frame) {
            continue;
        }
        points.insert(points.end(), frame->begin(), frame->end());
    }
    return points;
}

cv::Vec3b jetColor(double normalized) {
    const double v = std::clamp(normalized, 0.0, 1.0);
    auto channel = [v](double center) {
        return static_cast<uchar>(std::clamp((1.5 - std::abs(4.0 * v - center)) * 255.0, 0.0, 255.0));
    };
    return cv::Vec3b(channel(1.0), channel(2.0), channel(3.0));
}

bool writeImageFile(const QString& filePath, const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }

    const QString suffix = QFileInfo(filePath).suffix().toLower();
    const QString extension = QString(".%1").arg(suffix.isEmpty() ? QStringLiteral("png") : suffix);
    std::vector<uchar> encoded;
    if (!cv::imencode(extension.toStdString(), image, encoded)) {
        return false;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    return file.write(reinterpret_cast<const char*>(encoded.data()), static_cast<qint64>(encoded.size())) == static_cast<qint64>(encoded.size());
}

bool computeBounds(const std::vector<AssociatedPoints>& points,
                   std::array<float, 3>& minValues,
                   std::array<float, 3>& maxValues) {
    if (points.empty()) {
        return false;
    }
    minValues = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    maxValues = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
    for (const auto& point : points) {
        const std::array<float, 3> values = {
            point.reconstructedPoint.x,
            point.reconstructedPoint.y,
            point.reconstructedPoint.z
        };
        for (int i = 0; i < 3; ++i) {
            minValues[i] = std::min(minValues[i], values[i]);
            maxValues[i] = std::max(maxValues[i], values[i]);
        }
    }
    return true;
}

cv::Mat renderDepthProjection(const std::vector<AssociatedPoints>& points,
                              int depthAxis,
                              const std::array<float, 3>& minValues,
                              const std::array<float, 3>& maxValues,
                              const cv::Size& imageSize) {
    if (imageSize.width <= 0 || imageSize.height <= 0) {
        return cv::Mat();
    }
    cv::Mat image(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat filled(imageSize, CV_8UC1, cv::Scalar(0));
    const double rangeDepth = std::max(static_cast<double>(maxValues[depthAxis] - minValues[depthAxis]), 1e-9);

    for (const auto& point : points) {
        const std::array<float, 3> values = {
            point.reconstructedPoint.x,
            point.reconstructedPoint.y,
            point.reconstructedPoint.z
        };
        const int u = static_cast<int>(std::lround(point.distortedCenter.x));
        const int v = static_cast<int>(std::lround(point.distortedCenter.y));
        if (u < 0 || v < 0 || u >= image.cols || v >= image.rows) {
            continue;
        }
        image.at<cv::Vec3b>(v, u) = jetColor((values[depthAxis] - minValues[depthAxis]) / rangeDepth);
        filled.at<uchar>(v, u) = 255;
    }

    if (cv::countNonZero(filled) > 0) {
        cv::Mat dilated;
        cv::dilate(image, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        dilated.copyTo(image, filled == 0);
    }
    return image;
}

void saveDepthMaps(const ScanSessionPtr& session) {
    const auto points = flattenSessionPoints(session);
    std::array<float, 3> minValues;
    std::array<float, 3> maxValues;
    if (!computeBounds(points, minValues, maxValues) || session->imageSize.empty()) {
        return;
    }

    const QString format = normalizedImageFormat(session->depthImageFormat);
    const std::array<QString, 3> names = {"depth_x", "depth_y", "depth_z"};
    for (int axis = 0; axis < 3; ++axis) {
        const cv::Mat depth = renderDepthProjection(points, axis, minValues, maxValues, session->imageSize);
        cv::imwrite(QString("%1/%2.%3").arg(session->savePath, names[axis], format).toStdString(), depth);
    }
}

QString saveIntensityImage(const ScanSessionPtr& session) {
    const auto points = flattenSessionPoints(session);
    if (points.empty()) {
        return QString();
    }

    cv::Size imageSize = session->imageSize;
    if (imageSize.empty()) {
        int maxX = 0;
        int maxY = 0;
        for (const auto& point : points) {
            maxX = std::max(maxX, static_cast<int>(std::ceil(point.distortedCenter.x)));
            maxY = std::max(maxY, static_cast<int>(std::ceil(point.distortedCenter.y)));
        }
        imageSize = cv::Size(maxX + 1, maxY + 1);
    }
    if (imageSize.empty() || imageSize.width <= 0 || imageSize.height <= 0) {
        return QString();
    }

    cv::Mat accum(imageSize, CV_8UC1, cv::Scalar(0));
    for (const auto& point : points) {
        const int x = static_cast<int>(std::lround(point.distortedCenter.x));
        const int y = static_cast<int>(std::lround(point.distortedCenter.y));
        if (x < 0 || y < 0 || x >= accum.cols || y >= accum.rows) {
            continue;
        }
        accum.at<uchar>(y, x) = std::max(accum.at<uchar>(y, x), point.grayValue);
    }

    double maxValue = 0.0;
    cv::minMaxLoc(accum, nullptr, &maxValue);
    cv::Mat displayGray;
    if (maxValue > 0.0) {
        accum.convertTo(displayGray, CV_8UC1, 255.0 / maxValue);
    } else {
        displayGray = accum;
    }
    cv::threshold(displayGray, displayGray, 64, 255, cv::THRESH_TOZERO);

    cv::Mat displayColor(displayGray.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < displayGray.rows; ++y) {
        const uchar* row = displayGray.ptr<uchar>(y);
        for (int x = 0; x < displayGray.cols; ++x) {
            const uchar intensity = row[x];
            if (intensity == 0) {
                continue;
            }
            const int green = std::clamp(120 + static_cast<int>(intensity) / 2, 0, 255);
            displayColor.at<cv::Vec3b>(y, x) = cv::Vec3b(0, green, 0);
        }
    }

    const QString filePath = QString("%1/intensity.%2")
        .arg(session->savePath, normalizedImageFormat(session->compositeImageFormat));
    if (!writeImageFile(filePath, displayColor)) {
        return QString();
    }
    return filePath;
}

QString saveCompositeImage(const ScanSessionPtr& session) {
    if (!session || session->frameImages.empty()) {
        return QString();
    }

    cv::Size imageSize;
    for (const cv::Mat& image : session->frameImages) {
        if (!image.empty()) {
            imageSize = image.size();
            break;
        }
    }
    if (imageSize.empty()) {
        return QString();
    }

    cv::Mat sum(imageSize, CV_64FC1, cv::Scalar(0));
    int validFrameCount = 0;
    for (const cv::Mat& image : session->frameImages) {
        if (image.empty()) {
            continue;
        }
        cv::Mat gray;
        if (image.channels() == 1) {
            gray = image;
        } else {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }
        if (gray.size() != imageSize) {
            cv::resize(gray, gray, imageSize, 0, 0, cv::INTER_AREA);
        }

        cv::Mat gray64;
        gray.convertTo(gray64, CV_64FC1);
        sum += gray64;
        ++validFrameCount;
    }

    if (validFrameCount <= 0) {
        return QString();
    }

    cv::Mat ans;
    sum.convertTo(ans, CV_8UC1, 1.0 / static_cast<double>(validFrameCount));
    cv::equalizeHist(ans, ans);
    const QString format = normalizedImageFormat(session->compositeImageFormat);
    const QString filePath = QString("%1/composite.%2").arg(session->savePath, format);
    if (!writeImageFile(filePath, ans)) {
        return QString();
    }
    return filePath;
}

void saveTexturedPly(const ScanSessionPtr& session) {
    const auto points = flattenSessionPoints(session);
    if (points.empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.reserve(points.size());
    for (const auto& associated : points) {
        pcl::PointXYZRGB point;
        point.x = associated.reconstructedPoint.x;
        point.y = associated.reconstructedPoint.y;
        point.z = associated.reconstructedPoint.z;
        point.r = associated.grayValue;
        point.g = associated.grayValue;
        point.b = associated.grayValue;
        cloud.push_back(point);
    }
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = false;
    pcl::io::savePLYFileBinary(QString("%1/textured_pointcloud.ply").arg(session->savePath).toStdString(), cloud);
}
}

AssociatedPointsManager* AssociatedPointsManager::instance_ = nullptr;

AssociatedPointsManager* AssociatedPointsManager::getInstance() {
    static std::once_flag onceFlag;
    std::call_once(onceFlag, []() {
        instance_ = new AssociatedPointsManager();
    });
    return instance_;
}

AssociatedPointsManager::AssociatedPointsManager(QObject* parent) 
    : QObject(parent), isRunning_(false) {
}

AssociatedPointsManager::~AssociatedPointsManager() {
    stop();
}

void AssociatedPointsManager::start() {
    if (!isRunning_) {
        isRunning_ = true;
        workerThread_ = std::thread(&AssociatedPointsManager::workerThread, this);
    }
}

void AssociatedPointsManager::stop() {
    if (isRunning_) {
        {
            QMutexLocker lock(&mutex_);
            isRunning_ = false;
            condition_.wakeAll();
        }
        
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
        
        // 停止时保存所有活跃会话
        cancelAllSessions();
    }
}

void AssociatedPointsManager::createSession(const QString& sessionId,
                                            const QString& savePath,
                                            int totalFrames,
                                            bool saveDepthMaps,
                                            bool saveCompositeImage,
                                            bool saveTexturedPointCloud,
                                            const QString& depthImageFormat,
                                            const QString& compositeImageFormat) {
    QMutexLocker lock(&sessionMutex_);
    auto session = std::make_shared<ScanSession>(sessionId, savePath, totalFrames);
    session->saveDepthMaps = saveDepthMaps;
    session->saveCompositeImage = saveCompositeImage;
    session->saveTexturedPointCloud = saveTexturedPointCloud;
    session->depthImageFormat = normalizedImageFormat(depthImageFormat);
    session->compositeImageFormat = normalizedImageFormat(compositeImageFormat);
    activeSessions_[sessionId] = session;
    std::cout << "Created session: " << sessionId.toStdString() << std::endl;
}

void AssociatedPointsManager::cancelSession(const QString& sessionId) {
    QMutexLocker lock(&sessionMutex_);
    auto it = activeSessions_.find(sessionId);
        if (it != activeSessions_.end()) {
        it->second->isCancelled = true;
        if (it->second->processedFrames > 0) {
            // 创建最终保存任务
            auto task = std::make_shared<SaveTask>(sessionId, it->second->savePath, nullptr, true, it->second);
            {
                QMutexLocker taskLock(&mutex_);
                taskQueue_.push(task);
                condition_.wakeAll();
            }
        }
        activeSessions_.erase(it);
        std::cout << "Cancelled session: " << sessionId.toStdString() << std::endl;
    }
}

void AssociatedPointsManager::cancelAllSessions() {
    QMutexLocker lock(&sessionMutex_);
    for (auto& [sessionId, session] : activeSessions_) {
        if (session->processedFrames > 0) {
            session->isCancelled = true;
            auto task = std::make_shared<SaveTask>(sessionId, session->savePath, nullptr, true, session);
            {
                QMutexLocker taskLock(&mutex_);
                taskQueue_.push(task);
            }
        }
    }
    activeSessions_.clear();
    condition_.wakeAll();
}

void AssociatedPointsManager::addAssociatedPoints(const QString& sessionId,
                                                  AssociatedPointsPtr points,
                                                  const cv::Size& imageSize,
                                                  const cv::Mat& frameImage) {
    QMutexLocker lock(&sessionMutex_);
    auto it = activeSessions_.find(sessionId);
    if (it != activeSessions_.end() && !it->second->isCancelled) {
        it->second->frameData.push_back(points);
        if (it->second->saveCompositeImage && !frameImage.empty()) {
            cv::Mat imageCopy;
            if (frameImage.channels() == 1) {
                imageCopy = frameImage.clone();
            } else {
                cv::cvtColor(frameImage, imageCopy, cv::COLOR_BGR2GRAY);
            }
            it->second->frameImages.push_back(imageCopy);
        }
        if (it->second->imageSize.empty() && !imageSize.empty()) {
            it->second->imageSize = imageSize;
        } else if (it->second->imageSize.empty() && points && !points->empty()) {
            int maxX = 0;
            int maxY = 0;
            for (const auto& point : *points) {
                maxX = std::max(maxX, static_cast<int>(std::ceil(point.distortedCenter.x)));
                maxY = std::max(maxY, static_cast<int>(std::ceil(point.distortedCenter.y)));
            }
            it->second->imageSize = cv::Size(maxX + 1, maxY + 1);
        }
        it->second->processedFrames++;
        it->second->updateTimestamp();
        emit sessionProgress(sessionId, it->second->processedFrames, it->second->totalFrames);
        
        // 创建保存任务
        auto task = std::make_shared<SaveTask>(sessionId, it->second->savePath, points);
        {
            QMutexLocker taskLock(&mutex_);
            taskQueue_.push(task);
            condition_.wakeAll();
        }
        
        // 检查会话是否完成
        if (it->second->isComplete()) {
            auto completedSession = it->second;
            auto finalTask = std::make_shared<SaveTask>(sessionId, completedSession->savePath, nullptr, true, completedSession);
            {
                QMutexLocker taskLock(&mutex_);
                taskQueue_.push(finalTask);
                condition_.wakeAll();
            }
            emit sessionCompleted(sessionId, completedSession->processedFrames);
            activeSessions_.erase(it);
        }
    }
}

void AssociatedPointsManager::workerThread() {
    while (isRunning_) {
        SaveTaskPtr task;
        
        {
            QMutexLocker lock(&mutex_);
            while (taskQueue_.empty() && isRunning_) {
                condition_.wait(&mutex_);
            }
            
            if (!isRunning_) break;
            
            if (!taskQueue_.empty()) {
                task = taskQueue_.front();
                taskQueue_.pop();
            }
        }
        
        if (task) {
            processSaveTask(task);
        }
        
        // 定期清理超时会话（每30秒）
        static qint64 lastCleanup = QDateTime::currentMSecsSinceEpoch();
        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
        if (currentTime - lastCleanup > 30000) {
            cleanupTimeoutSessions();
            lastCleanup = currentTime;
        }
        
        // 短暂休眠
        QThread::msleep(10);
    }
}

void AssociatedPointsManager::processSaveTask(SaveTaskPtr task) {
    try {
        // 增量保存：保存单帧数据
        QDir dir(task->savePath);
        if (!dir.exists()) {
            dir.mkpath(".");
        }
        if (task->isFinal) {
            // 最终保存：保存整个会话数据
            auto session = task->session ? task->session : getSession(task->sessionId);
            if (session) {
                saveSessionData(session);
                std::cout << "Final save completed for session: " << task->sessionId.toStdString() 
                          << ", frames: " << session->processedFrames << std::endl;
            }
        } else {
            saveIncrementalFrameData(task);
            std::cout << "Incremental save for session: " << task->sessionId.toStdString() 
                      << ", points: " << (task->points ? task->points->size() : 0) << std::endl;
        }
    } catch (const std::exception& e) {
        emit saveError(task->sessionId, QString("Save error: %1").arg(e.what()));
    }
}

void AssociatedPointsManager::saveIncrementalFrameData(SaveTaskPtr task){
    if (!task->points || task->points->empty()) {
        return;
    }
    
    // 三个临时文件分别保存不同类型的数据
    QString reconFile = task->savePath + "/reconstructed_" + task->sessionId + ".tmp";
    QString distortedFile = task->savePath + "/centerline_distorted_" + task->sessionId + ".tmp";
    QString undistortedFile = task->savePath + "/centerline_undistorted_" + task->sessionId + ".tmp";
    
    {
        // 以追加模式保存数据
        std::ofstream recon(reconFile.toStdString(), std::ios::app);
        std::ofstream distorted(distortedFile.toStdString(), std::ios::app);
        std::ofstream undistorted(undistortedFile.toStdString(), std::ios::app);
        
        for (const auto& point : *task->points) {
            recon << point.reconstructedPoint.x << " " 
                << point.reconstructedPoint.y << " " 
                << point.reconstructedPoint.z << "\n";
            
            distorted << point.distortedCenter.x << " " 
                    << point.distortedCenter.y << "\n";
            
            undistorted << point.undistortedCenter.x << " " 
                        << point.undistortedCenter.y << "\n";
        }    

        recon.flush();
        distorted.flush();
        undistorted.flush();
        
        // 检查流状态
        if (recon.bad() || distorted.bad() || undistorted.bad()) {
            throw std::runtime_error("Error writing to incremental files");
        }
    }
}

static QString getSessionStatusString(const ScanSessionPtr& session) {
    if (session->isComplete()) {
        return "COMPLETED";
    } else if (session->isCancelled) {
        return QString("CANCELLED (User interrupted, %1% completed)")
               .arg(session->totalFrames > 0 ? (session->processedFrames * 100.0 / session->totalFrames) : 0.0, 0, 'f', 1);
    } else if (session->isTimeout()) {
        return QString("TIMEOUT (Session expired, %1% completed)")
               .arg(session->totalFrames > 0 ? (session->processedFrames * 100.0 / session->totalFrames) : 0.0, 0, 'f', 1);
    } else {
        return QString("PARTIAL (%1% completed)")
               .arg(session->totalFrames > 0 ? (session->processedFrames * 100.0 / session->totalFrames) : 0.0, 0, 'f', 1);
    }
}

static void addFileHeaderAndRename(const QString& tempFile, const QString& finalFile, const QString & dataType, const ScanSessionPtr& session){
    if(!QFile::exists(tempFile)) return;

    // 读取临时文件内容
    std::ifstream temp(tempFile.toStdString());
    std::stringstream buffer;
    buffer << temp.rdbuf();
    temp.close();
    
    // 创建最终文件并写入详细头信息
    std::ofstream final(finalFile.toStdString());
    
    // 详细的头信息
    final << "# ===========================================\n";
    final << "# UWScan Data File\n";
    final << "# Data Type: " << dataType.toStdString() << "\n";
    final << "# ===========================================\n";
    final << "# Session Information:\n";
    final << "#   Session ID: " << session->sessionId.toStdString() << "\n";
    final << "#   Save Path: " << session->savePath.toStdString() << "\n";
    final << "#   Creation Time: " << QDateTime::fromMSecsSinceEpoch(session->lastUpdateTime).toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString() << "\n";
    final << "#   Save Time: " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString() << "\n";
    final << "# \n";
    final << "# Scan Statistics:\n";
    final << "#   Total Frames Planned: " << session->totalFrames << "\n";
    final << "#   Frames Processed: " << session->processedFrames << "\n";
    final << "#   Completion Rate: " << std::fixed << std::setprecision(1) 
            << (session->totalFrames > 0 ? (session->processedFrames * 100.0 / session->totalFrames) : 0.0) << "%\n";
    final << "#   Status: " << getSessionStatusString(session).toStdString() << "\n";
    final << "# \n";
    final << "# Data Format:\n";
    
    // 根据数据类型添加不同的格式说明
    if (dataType.contains("Reconstructed")) {
        final << "#   Columns: X Y Z (3D coordinates in world coordinate system)\n";
        final << "#   Units: Millimeters (mm)\n";
        final << "#   Coordinate System: Right-handed Cartesian\n";
    } else if (dataType.contains("Distorted")) {
        final << "#   Columns: X Y (Image coordinates with distortion)\n";
        final << "#   Units: Pixels\n";
        final << "#   Coordinate System: Image coordinate system (origin at top-left)\n";
    } else if (dataType.contains("Undistorted")) {
        final << "#   Columns: X Y (Corrected image coordinates)\n";
        final << "#   Units: Pixels\n";
        final << "#   Coordinate System: Normalized image coordinate system\n";
    }
    
    final << "# \n";
    final << "# Processing Information:\n";
    final << "#   Software: UWScan 3D Reconstruction System\n";
    final << "#   Version: 1.0.0\n";
    final << "#   Reconstruction Method: Laser Line Scanning\n";
    final << "# \n";
    final << "# Notes:\n";
    final << "#   - This file was generated automatically by UWScan\n";
    final << "#   - Data points are ordered by frame and within-frame position\n";
    final << "#   - Missing frames indicate reconstruction failures or cancellations\n";
    final << "# ===========================================\n\n";
    
    // 写入数据内容
    final << buffer.str();
    final.close();
    
    // 删除临时文件
    QFile::remove(tempFile);
}

void AssociatedPointsManager::saveSessionData(const ScanSessionPtr& session) {
    // 创建保存目录
    QDir dir(session->savePath);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    auto sessionId = session->sessionId;
    auto savePath = session->savePath;
    QString reconTemp = savePath + "/reconstructed_" + sessionId + ".tmp";
    QString distortedTemp = savePath + "/centerline_distorted_" + sessionId + ".tmp";
    QString undistortedTemp = savePath + "/centerline_undistorted_" + sessionId + ".tmp";
    
    QString reconFinal = savePath + "/reconstructed.txt";
    QString distortedFinal = savePath + "/centerline_distorted.txt";
    QString undistortedFinal = savePath + "/centerline_undistorted.txt";
        
    addFileHeaderAndRename(reconTemp, reconFinal, "Reconstructed", session);
    addFileHeaderAndRename(distortedTemp, distortedFinal, "Distorted", session);
    addFileHeaderAndRename(undistortedTemp, undistortedFinal, "Undistorted", session);

    if (session->saveDepthMaps) {
        saveDepthMaps(session);
    }
    const QString intensityPath = saveIntensityImage(session);
    if (!intensityPath.isEmpty()) {
        emit intensityImageSaved(session->sessionId, intensityPath);
    }
    if (session->saveCompositeImage) {
        const QString compositePath = saveCompositeImage(session);
        if (!compositePath.isEmpty()) {
            emit compositeImageSaved(session->sessionId, compositePath);
        }
    }
    if (session->saveTexturedPointCloud) {
        saveTexturedPly(session);
    }
}

ScanSessionPtr AssociatedPointsManager::getSession(const QString& sessionId) {
    QMutexLocker lock(&sessionMutex_);
    auto it = activeSessions_.find(sessionId);
    return it != activeSessions_.end() ? it->second : nullptr;
}

void AssociatedPointsManager::cleanupTimeoutSessions() {
    QMutexLocker lock(&sessionMutex_);
    auto it = activeSessions_.begin();
    while (it != activeSessions_.end()) {
        if (it->second->isTimeout()) {
            if (it->second->processedFrames > 0) {
                saveSessionData(it->second);
                emit sessionCancelled(it->first, it->second->processedFrames);
            }
            it = activeSessions_.erase(it);
        } else {
            ++it;
        }
    }
}

}}
