#include "scanner/scanner.h"
#include "scanner/reconstructor.h"
#include "setting/settingsmanager.h"
#include "scanner/associatedpointsmanager.h"
#include "imagefileutils.h"
#include <QDateTime>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <filesystem>

static double getChunkFloatValue(GenApi::INodeMap& chunkDataMap, const char* nodeName) {
    GenApi::CFloatPtr ptr(chunkDataMap.GetNode(nodeName));
    if (GenApi::IsReadable(ptr)) {
        return ptr->GetValue();
    }
    return -1.0; // 读取失败时返回默认值
}

NAMESPACE_BEGIN { namespace scanner {
    static QString normalizedImageFormat(const QString& format, const QString& fallback) {
        QString ret = format.trimmed().toLower();
        if (ret.startsWith('.')) {
            ret.remove(0, 1);
        }
        return ret.isEmpty() ? fallback : ret;
    }

    static int scanPositionToFrameIndex(float position) {
        return static_cast<int>(std::round(position * 100.0f));
    }

    static std::vector<float> buildScanTargets(float startAngle, float endAngle, float signedStep) {
        std::vector<float> targets;
        if (qFuzzyCompare(startAngle, endAngle) || qFuzzyIsNull(signedStep)) {
            return targets;
        }

        const float epsilon = std::max(std::abs(signedStep) * 1e-4f, 1e-4f);
        for (float target = startAngle + signedStep;
             signedStep > 0.0f ? target <= endAngle + epsilon : target >= endAngle - epsilon;
             target += signedStep) {
            targets.push_back(signedStep > 0.0f ? std::min(target, endAngle) : std::max(target, endAngle));
        }

        if (targets.empty() || std::abs(targets.back() - endAngle) > epsilon) {
            targets.push_back(endAngle);
        } else {
            targets.back() = endAngle;
        }
        return targets;
    }

    static int centerlineMethodIndex(const QString& method) {
        if (method == "Steger") {
            return 1;
        }
        if (method == QStringLiteral("阈值法")) {
            return 2;
        }
        return 0;
    }

    static QString yamlQuote(const QString& value) {
        QString escaped = value;
        escaped.replace("\\", "\\\\");
        escaped.replace("\"", "\\\"");
        return QString("\"%1\"").arg(escaped);
    }

    static int monitorDisplayCentiDegree(float angle) {
        return static_cast<int>(std::round(angle * 100.0f));
    }

    static QString exposureValueString(double exposure) {
        return QString::number(exposure, 'f', 0);
    }

    static bool writeScanSettingsYml(const QString& savePath,
                                     float startAngle,
                                     float endAngle,
                                     float step,
                                     float speed,
                                     double sceneExposure,
                                     double scanExposure,
                                     bool saveSceneImages,
                                     bool saveOriginalImages,
                                     bool savePointCloud,
                                     bool saveDepthMaps,
                                     bool saveCompositeImage,
                                     bool saveTexturedPointCloud,
                                     const QString& sceneImageFormat,
                                     const QString& scanImageFormat,
                                     const QString& depthImageFormat,
                                     const QString& compositeImageFormat,
                                     setting::SettingsManager* settingsManager,
                                     QString* errorMessage = nullptr) {
        if (savePath.isEmpty() || savePath == "**TEMP**") {
            return true;
        }

        QDir dir(savePath);
        if (!dir.exists() && !dir.mkpath(".")) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("无法创建扫描参数目录：%1").arg(savePath);
            }
            return false;
        }

        const QString settingsPath = dir.filePath("scan_parameters.yml");
        QFile file(settingsPath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("无法写入扫描参数文件：%1").arg(settingsPath);
            }
            return false;
        }

        const QString centerlineMethod = settingsManager->getString(setting::DEFAULT_CENTERLINE_METHOD, QStringLiteral("GrayScale灰度重心法"));
        const QString calibrationFile = settingsManager->getString(setting::DEFAULT_CALIBRATE_FILE);
        QTextStream out(&file);
        out.setCodec("UTF-8");
        out << "%YAML:1.0\n";
        out << "ScanSettings:\n";
        out << "  CreatedAt: " << yamlQuote(QDateTime::currentDateTime().toString(Qt::ISODate)) << "\n";
        out << "  SceneExposureTime: " << sceneExposure << "\n";
        out << "  ScanExposureTime: " << scanExposure << "\n";
        out << "  CenterlineMethod: " << yamlQuote(centerlineMethod) << "\n";
        out << "  CenterlineMethodIndex: " << centerlineMethodIndex(centerlineMethod) << "\n";
        out << "  BwThreshold: " << settingsManager->getDouble(setting::DEFAULT_BW_THR, 55.0) << "\n";
        out << "  SumThreshold: " << settingsManager->getDouble(setting::DEFAULT_SUM_THR, 124.0) << "\n";
        out << "  IsHorizontal: " << (settingsManager->getBool(setting::DEFAULT_IS_HENG, true) ? 1 : 0) << "\n";
        out << "  CalibrationFile: " << yamlQuote(calibrationFile) << "\n";
        out << "  CalibrationName: " << yamlQuote(QFileInfo(calibrationFile).fileName()) << "\n";
        out << "  ScanStartPosition: " << startAngle << "\n";
        out << "  ScanEndPosition: " << endAngle << "\n";
        out << "  ScanStep: " << std::abs(step) << "\n";
        out << "  SignedScanStep: " << step << "\n";
        out << "  ScanSpeed: " << speed << "\n";
        out << "  SaveImages: " << ((saveSceneImages || saveOriginalImages) ? 1 : 0) << "\n";
        out << "  SaveSceneImages: " << (saveSceneImages ? 1 : 0) << "\n";
        out << "  SaveOriginalImages: " << (saveOriginalImages ? 1 : 0) << "\n";
        out << "  SavePointCloud: " << (savePointCloud ? 1 : 0) << "\n";
        out << "  SaveDepthMaps: " << (saveDepthMaps ? 1 : 0) << "\n";
        out << "  SaveCompositeImage: " << (saveCompositeImage ? 1 : 0) << "\n";
        out << "  SaveTexturedPointCloud: " << (saveTexturedPointCloud ? 1 : 0) << "\n";
        out << "  SceneImageFormat: " << yamlQuote(sceneImageFormat) << "\n";
        out << "  ScanImageFormat: " << yamlQuote(scanImageFormat) << "\n";
        out << "  DepthImageFormat: " << yamlQuote(depthImageFormat) << "\n";
        out << "  CompositeImageFormat: " << yamlQuote(compositeImageFormat) << "\n";
        file.close();
        return true;
    }

    Scanner::Scanner(QObject* parent):QObject(parent){
        connect(&camera_, &Camera::imageGrabbed, this, &Scanner::onImageGrabbed);
        connect(&motor_, &ModbusMotor::sendInfo, this, [this](const MotorFeedback& info){
            const float feedbackAngle = info.angle;
            if (hasMotorFeedback_) {
                previousMotorFeedbackAngle_ = lastMotorFeedbackAngle_;
                hasPreviousMotorFeedback_ = true;
            }
            lastMotorFeedbackAngle_ = feedbackAngle;
            hasMotorFeedback_ = true;
            scannerInfo_.angle = feedbackAngle;
            scannerInfo_.temperature = info.temperature;
            scannerInfo_.humidity = info.humidity;
            scannerInfo_.pressure = info.pressure;
            scannerInfo_.attitudeFlag = info.attitudeFlag;
            scannerInfo_.roll = info.roll;
            scannerInfo_.pitch = info.pitch;
            scannerInfo_.yaw = info.yaw;
            scannerInfo_.reserved = info.reserved;
            scannerInfo_.isOpen = camera_.isConnected();
            emit sendInfo(scannerInfo_);
            if (waitingForManualTargetAngle_ &&
                (isAtTargetAngle(manualTargetAngle_) || isAtOrPastManualTargetAngle(feedbackAngle))) {
                waitingForManualTargetAngle_ = false;
                waitingForScanStartPosition_ = false;
                motor_.stopMove();
            emit sendInformation(QStringLiteral("已到达目标位置：%1，当前位置：%2")
                .arg(manualTargetAngle_, 0, 'f', 2)
                .arg(feedbackAngle, 0, 'f', 2));
            }
            if (waitingForScanStartPosition_ &&
                (isAtTargetAngle(scanStartAngle_) || isAtOrPastStartAngle(feedbackAngle))) {
                continueScanAfterMoveToStart();
            }
            if (shouldCaptureAtCurrentFeedback()) {
                waitingForTargetAngle_ = false;
                motor_.stopMove();
                emit sendInformation(QStringLiteral("当前位置到达扫描步长目标，准备拍照：目标 %1，上次位置 %2，当前位置 %3")
                    .arg(activeTargetAngle_, 0, 'f', 2)
                    .arg(previousMotorFeedbackAngle_, 0, 'f', 2)
                    .arg(feedbackAngle, 0, 'f', 2));
                grabCurrentScanStepFrame();
            }
        });
        connect(&motor_, &ModbusMotor::sendInformation, this, &Scanner::sendInformation);
        connect(&camera_, &Camera::errorOccurred, this, [this](const QString& error) {
            emit sendInformation(error);
            if (error.contains(QStringLiteral("设备断开")) || error.contains(QStringLiteral("Device Removed"))) {
                softwareScanActive_ = false;
                waitingForTargetAngle_ = false;
                waitingForManualTargetAngle_ = false;
                waitingForScanStartPosition_ = false;
                leftReconstructionFrameCount_ = 0;
                totalFrameCount_ = 0;
                if (!sessionId_.isEmpty()) {
                    AssociatedPointsManager::getInstance()->cancelSession(sessionId_);
                    sessionId_.clear();
                }
                emit scanStopped();
            }
        });
        connect(&motor_, &ModbusMotor::errorOccurred, this, [this](const QString& error) {
            emit sendInformation(error);
            return;
            if (error.contains(QStringLiteral("断开")) ||
                error.contains(QStringLiteral("未连接")) ||
                error.contains(QStringLiteral("TCP不可写")) ||
                error.contains(QStringLiteral("not connected"), Qt::CaseInsensitive)) {
                emit scanStopped();
            }
        });
        connect(&motor_, &ModbusMotor::scanBegin, this, &Scanner::scanBegin);
        connect(&motor_, &ModbusMotor::scanStopped, this, &Scanner::scanStopped);
        connect(&motor_, &ModbusMotor::laserOpened, this, &Scanner::laserOpened);
        connect(&motor_, &ModbusMotor::laserClosed, this, &Scanner::laserClosed);
    }

    bool Scanner::connectDevices(){
        totalFrameCount_ = 0;
        leftReconstructionFrameCount_ = 0;
        const bool cameraOk = camera_.initCamera();
        const bool motorOk = motor_.connectModbus("", 9600);
        const bool ret = cameraOk && motorOk;
        if (!ret) {
            if (cameraOk && !motorOk) {
                emit sendInformation(QStringLiteral("相机已连接，但电机未连接"));
            } else if (!cameraOk && motorOk) {
                emit sendInformation(QStringLiteral("电机已连接，但相机未连接"));
            } else {
                emit sendInformation(QStringLiteral("相机和电机连接失败"));
            }
        }
        std::cout << ">>> [Debug] Scanner::connectDevices() ret: " << ret << std::endl;
        return ret;
    }

    void Scanner::disconnectDevices(){
        stopGrab();
        softwareScanActive_ = false;
        waitingForTargetAngle_ = false;
        waitingForManualTargetAngle_ = false;
        waitingForScanStartPosition_ = false;
        auto* sm = setting::SettingsManager::getInstance();
        sm->setDouble(setting::LAST_CLOSE_MOTOR_LIMIT_LOWER, getCurrentLimitLower());
        sm->setDouble(setting::LAST_CLOSE_MOTOR_LIMIT_UPPER, getCurrentLimitUpper());
        sm->sync(); 
        camera_.closeCamera();
        std::cout << ">>> [Debug] Scanner::disconnectDevices() -> Disconnect Camera" << std::endl;
        motor_.disconnectModbus();
        std::cout << ">>> [Debug] Scanner::disconnectDevices() -> Disconnect Motor" << std::endl;
        qDebug() << "Saving LAST_CLOSE limits: lower=" << getCurrentLimitLower() << "upper=" << getCurrentLimitUpper();
    }

    bool Scanner::isConnected() const {
        // std::cout << ">>> [Debug] Scanner::isConnected() ret: " << (camera_.isConnected() && motor_.isConnected()) << std::endl;
        return camera_.isConnected() && motor_.isConnected();
    }

    void Scanner::singleGrab(){
        if(leftReconstructionFrameCount_ == 0) camera_.SingleGrab();
        else return;
    }

    bool Scanner::continuousGrab(){
        bool ret = camera_.ContinuousGrab();
        std::cout << ">>> [Debug] Scanner::continuousGrab() ret: " << ret << std::endl;
        return ret;
    }

    bool Scanner::stopGrab(){
        bool ret = camera_.StopGrab();
        std::cout << ">>> [Debug] Scanner::stopGrab() ret: " << ret << std::endl;
        return ret;
    }

    void Scanner::setBrightness(int brightness){
        if (!motor_.setBrightness(brightness)) {
            emit sendInformation(QStringLiteral("激光器亮度设置指令发送失败"));
        }
    }

    bool Scanner::openLaser(){
        bool ret = motor_.openLaser();
        std::cout << ">>> [Debug] Scanner::openLaser() ret: " << ret << std::endl;
        return ret;
    }

    bool Scanner::closeLaser(){
        bool ret = motor_.closeLaser();
        std::cout << ">>> [Debug] Scanner::closeLaser() ret: " << ret << std::endl;
        return ret;
    }

    static inline QString generateSessionId(){
        QUuid uuid = QUuid::createUuid();
        return QString("scan_%1").arg(uuid.toString(QUuid::WithoutBraces));        
    }

    void Scanner::reconstructCurrent(){
        QMutexLocker lock(&m_mutex);
        isSingleScan_ = true;
        sessionId_ = generateSessionId();
    }

    void Scanner::moveMotor(float speed, float angle){
        if (!motor_.goTo(speed, angle)) {
            emit sendInformation(QStringLiteral("电机移动指令发送失败：%1").arg(angle, 0, 'f', 2));
            waitingForManualTargetAngle_ = false;
            return;
        }
        manualTargetAngle_ = angle;
        manualStartAngle_ = currentMonitorAngle();
        waitingForManualTargetAngle_ = true;
        if (hasMotorFeedback_ && isAtTargetAngle(manualTargetAngle_)) {
            waitingForManualTargetAngle_ = false;
            motor_.stopMove();
            emit sendInformation(QStringLiteral("已到达目标位置：%1，当前位置：%2")
                .arg(manualTargetAngle_, 0, 'f', 2)
                .arg(lastMotorFeedbackAngle_, 0, 'f', 2));
        } else {
            emit sendInformation(QStringLiteral("跳转到目标位置：%1").arg(angle, 0, 'f', 2));
        }
        std::cout << ">>> [Debug] Scanner::moveMotor() speed: " << speed << " angle: " << angle << std::endl;
    }

    void Scanner::setSceneExposureTime(double exposureTime) {
        camera_.setExposureTime(exposureTime);
    }

    void Scanner::setExposureTime(double exposureTime) {
        if (!softwareScanActive_) {
            sceneExposureTime_ = exposureTime;
        }
        ++previewExposureChangeSerial_;
        const int exposureChangeSerial = previewExposureChangeSerial_;
        const bool restartPreview = camera_.isConnected()
            && !softwareScanActive_
            && leftReconstructionFrameCount_ == 0;
        if (restartPreview) {
            camera_.StopGrab();
        }
        camera_.setExposureTime(exposureTime);
        if (restartPreview) {
            QTimer::singleShot(exposureSettleDelayMs(exposureTime), this, [this, exposureChangeSerial]() {
                if (exposureChangeSerial == previewExposureChangeSerial_
                    && !softwareScanActive_
                    && camera_.isConnected()) {
                    camera_.ContinuousGrab();
                }
            });
        }
    }

    void Scanner::setTriggerConfig(bool enabled, const QString& source) {
        camera_.setTriggerConfig("FrameStart",
                                 enabled,
                                 source.trimmed().isEmpty() ? "Software" : source.toStdString().c_str());
    }

    int Scanner::exposureSettleDelayMs(double exposureTime) const {
        const int exposureMs = static_cast<int>(std::ceil(std::max(0.0, exposureTime) / 1000.0));
        return std::clamp(exposureMs + 80, 120, 1200);
    }


    void Scanner::prepareCurrentScanExposureSequence() {
        scanExposureSequence_.clear();
        scanExposureSequenceIndex_ = 0;
        scanReconstructionExposureIndex_ = 0;
        waitingForScanMultiExposureFrame_ = false;
        if (!scanMultiExposureEnabled_) {
            scanExposureSequence_.push_back(scanExposureTime_);
            return;
        }
        for (double exposure = scanMultiExposureMin_;
             exposure <= scanMultiExposureMax_ + scanMultiExposureStep_ * 1e-6;
             exposure += scanMultiExposureStep_) {
            scanExposureSequence_.push_back(exposure);
        }
        if (scanExposureSequence_.empty()) {
            scanExposureSequence_.push_back(scanExposureTime_);
        }
        double bestDistance = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(scanExposureSequence_.size()); ++i) {
            const double distance = std::abs(scanExposureSequence_[i] - scanExposureTime_);
            if (distance < bestDistance) {
                bestDistance = distance;
                scanReconstructionExposureIndex_ = i;
            }
        }
    }

    void Scanner::grabCurrentScanStepFrame() {
        if (!softwareScanActive_ || scanExposureSequence_.empty()) {
            return;
        }
        if (scanExposureSequenceIndex_ < 0 || scanExposureSequenceIndex_ >= static_cast<int>(scanExposureSequence_.size())) {
            return;
        }
        waitingForScanMultiExposureFrame_ = true;
        const double exposure = scanExposureSequence_[scanExposureSequenceIndex_];
        camera_.StopGrab();
        camera_.setTriggerConfig("FrameStart", false, "Line0");
        camera_.setExposureTime(exposure);
        QTimer::singleShot(exposureSettleDelayMs(exposure), this, [this]() {
            if (softwareScanActive_ && waitingForScanMultiExposureFrame_) {
                camera_.SingleGrab();
            }
        });
    }

    QString Scanner::currentScanImageBaseName() const {
        const QString baseName = QString::number(scan_now_);
        if (!scanMultiExposureEnabled_) {
            return baseName;
        }
        return QString("%1_%2_%3_%4")
            .arg(baseName)
            .arg(exposureValueString(scanMultiExposureMin_))
            .arg(exposureValueString(scanMultiExposureMax_))
            .arg(exposureValueString(scanMultiExposureStep_));
    }

    QString Scanner::currentScanImageFileNameForExposure(double exposure) const {
        if (!scanMultiExposureEnabled_) {
            return QString::number(scan_now_);
        }
        return QString("%1_%2_%3_%4_%5")
            .arg(scan_now_)
            .arg(exposureValueString(scanMultiExposureMin_))
            .arg(exposureValueString(scanMultiExposureMax_))
            .arg(exposureValueString(scanMultiExposureStep_))
            .arg(exposureValueString(exposure));
    }

    void Scanner::saveCurrentExposureImage(const QString& suffix) {
        // 确保 savePath_ 有效
        if(savePath_.isEmpty() || savePath_ == "**TEMP**") return;

        // 设置标志位，让 onImageGrabbed() 保存当前帧
        if(suffix == "before_scan") {
            pendingBeforeScanSnapshot_ = true;
        } else if(suffix == "after_scan") {
            pendingAfterScanSnapshot_ = true;
        } else {
            qWarning() << "Unknown snapshot suffix:" << suffix;
            return;
        }

        // 触发单帧抓取
        camera_.SingleGrab();

        // 保存完成后会在 onImageGrabbed() 中自动处理
    }

    void Scanner::finishScan(bool cancelled) {
        auto pointsManager = AssociatedPointsManager::getInstance();
        const QString sessionId = sessionId_;
        const int remainingFrames = leftReconstructionFrameCount_;
        const bool shouldSaveAfterScene = saveSceneImages_ && !savePath_.isEmpty() && savePath_ != "**TEMP**" && scanExposureCaptured_;

        motor_.stopMove();
        motor_.setFeedbackInterval(500);
        softwareScanActive_ = false;
        waitingForTargetAngle_ = false;
        waitingForManualTargetAngle_ = false;
        waitingForScanStartPosition_ = false;
        leftReconstructionFrameCount_ = 0;
        totalFrameCount_ = 0;
        isSingleScan_ = false;
        scanExposureCaptured_ = false;
        activeTargetAngle_ = scanEndAngle_;
        scanTargets_.clear();
        scanExposureSequence_.clear();
        scanExposureSequenceIndex_ = 0;
        scanReconstructionExposureIndex_ = 0;
        waitingForScanMultiExposureFrame_ = false;
        scanMultiExposureEnabled_ = false;

        if (cancelled && !sessionId.isEmpty()) {
            pointsManager->cancelSession(sessionId);
            sessionId_.clear();
        }

        if (shouldSaveAfterScene) {
            pendingFinishAfterSceneSnapshot_ = true;
            auto* settingsManager = setting::SettingsManager::getInstance();
            const double sceneExposure = sceneExposureTime_ > 0.0
                ? sceneExposureTime_
                : settingsManager->getDouble(setting::LAST_CLOSE_EXPOSURETIME, camera_.exposureTime());
            QTimer::singleShot(1000, this, [this, sceneExposure]() {
                camera_.setExposureTime(sceneExposure);
                saveCurrentExposureImage("after_scan");
            });
        } else {
            if (sceneExposureTime_ > 0.0) {
                camera_.setExposureTime(sceneExposureTime_);
            }
            camera_.ContinuousGrab();
        }

        if (cancelled) {
            emit sendInformation(QStringLiteral("扫描已停止，剩余未处理帧数：%1").arg(remainingFrames));
            emit sendPorgress(0);
        } else {
            sessionId_.clear();
            emit sendInformation(QStringLiteral("扫描流程完成"));
        }
    }

    void Scanner::startScan(float startAngle,
                            float endAngle,
                            float step,
                            float speed,
                            const QString& savePath,
                            double sceneExposureTime,
                            double scanExposureTime,
                            bool saveSceneImages,
                            bool saveOriginalImages,
                            bool savePointCloud,
                            bool saveDepthMaps,
                            bool saveCompositeImage,
                            bool saveTexturedPointCloud,
                            const QString& sceneImageFormat,
                            const QString& scanImageFormat,
                            const QString& depthImageFormat,
                            const QString& compositeImageFormat,
                            bool multiExposureEnabled,
                            double multiExposureMin,
                            double multiExposureMax,
                            double multiExposureStep){
        if (qFuzzyCompare(startAngle, endAngle) || qFuzzyIsNull(step) || speed <= 0.0f) {
            emit sendInformation(QStringLiteral("扫描参数无效：开始/结束角度不能相同，步长和速度必须大于0"));
            return;
        }
        openLaser();
        camera_.StopGrab();
        motor_.stopScan();
        motor_.setFeedbackInterval(100);
        // camera_.clearBuffer();
        camera_.setTriggerConfig("FrameStart", false, "Line0");

        QTimer::singleShot(100, this, [this, startAngle, endAngle, step, speed, savePath, sceneExposureTime, scanExposureTime, saveSceneImages, saveOriginalImages, savePointCloud, saveDepthMaps, saveCompositeImage, saveTexturedPointCloud, sceneImageFormat, scanImageFormat, depthImageFormat, compositeImageFormat, multiExposureEnabled, multiExposureMin, multiExposureMax, multiExposureStep]() {  // 延时后再触发，尽量消耗掉遗留帧
            {
                QMutexLocker lock(&m_mutex);
                const float signedStep = startAngle < endAngle ? std::abs(step) : -std::abs(step);
                savePath_ = savePath;
                saveSceneImages_ = saveSceneImages;
                saveScanImages_ = saveOriginalImages;
                savePointCloud_ = savePointCloud;
                saveDepthMaps_ = saveDepthMaps;
                saveCompositeImage_ = saveCompositeImage;
                saveTexturedPointCloud_ = saveTexturedPointCloud;
                sceneImageFormat_ = normalizedImageFormat(sceneImageFormat, "png");
                scanImageFormat_ = normalizedImageFormat(scanImageFormat, "bmp");
                depthImageFormat_ = normalizedImageFormat(depthImageFormat, "png");
                compositeImageFormat_ = normalizedImageFormat(compositeImageFormat, "png");
                sessionId_ = generateSessionId();
                pendingBeforeScanSnapshot_ = false;
                pendingAfterScanSnapshot_ = false;
                pendingFinishAfterSceneSnapshot_ = false;
                scanExposureCaptured_ = false;
                
                setting::SettingsManager * settingsManager = setting::SettingsManager::getInstance();
                const double sceneExposure = sceneExposureTime > 0.0
                    ? sceneExposureTime
                    : settingsManager->getDouble(setting::LAST_CLOSE_EXPOSURETIME, camera_.exposureTime());
                sceneExposureTime_ = sceneExposure;
                scanExposureTime_ = scanExposureTime > 0.0 ? scanExposureTime : camera_.exposureTime();
                scanMultiExposureEnabled_ = multiExposureEnabled;
                scanMultiExposureMin_ = multiExposureMin;
                scanMultiExposureMax_ = multiExposureMax;
                scanMultiExposureStep_ = multiExposureStep;
                scanMultiExposureSaveAllImages_ = saveScanImages_;
                emit sendInformation(QStringLiteral("开始扫描：起点 %1，终点 %2，步长 %3，速度 %4，扫描曝光 %5")
                    .arg(startAngle, 0, 'f', 2)
                    .arg(endAngle, 0, 'f', 2)
                    .arg(std::abs(signedStep), 0, 'f', 2)
                    .arg(speed, 0, 'f', 2)
                    .arg(scanExposureTime_, 0, 'f', 0));
                if(savePath_ != "**TEMP**" && savePath_ != ""){
                    if(!std::filesystem::exists(savePath_.toStdString())){
                        std::filesystem::create_directories(savePath_.toStdString());
                    }
                }
                QString settingsWriteError;
                if (!writeScanSettingsYml(savePath_,
                                          startAngle,
                                          endAngle,
                                          signedStep,
                                          speed,
                                          sceneExposureTime_,
                                          scanExposureTime_,
                                          saveSceneImages_,
                                          saveScanImages_,
                                          savePointCloud_,
                                          saveDepthMaps_,
                                          saveCompositeImage_,
                                          saveTexturedPointCloud_,
                                          sceneImageFormat_,
                                          scanImageFormat_,
                                          depthImageFormat_,
                                          compositeImageFormat_,
                                          settingsManager,
                                          &settingsWriteError)) {
                    emit sendInformation(settingsWriteError);
                }
                
                scanTargets_ = buildScanTargets(startAngle, endAngle, signedStep);
                leftReconstructionFrameCount_ = static_cast<int>(scanTargets_.size());
                totalFrameCount_ = leftReconstructionFrameCount_;
                auto pointsManager = AssociatedPointsManager::getInstance();
                const bool needsAssociatedExports = savePointCloud_ || saveDepthMaps_ || saveCompositeImage_ || saveTexturedPointCloud_;
                if (needsAssociatedExports && !savePath_.isEmpty() && savePath_ != "**TEMP**" && totalFrameCount_ > 0) {
                    pointsManager->createSession(sessionId_,
                                                 savePath_,
                                                 totalFrameCount_,
                                                 saveDepthMaps_,
                                                 saveCompositeImage_,
                                                 saveTexturedPointCloud_,
                                                 depthImageFormat_,
                                                 compositeImageFormat_);
                }
                scan_step_ = scanPositionToFrameIndex(signedStep);
                scan_now_ = scanPositionToFrameIndex(startAngle);
                scanStartAngle_ = startAngle;
                scanEndAngle_ = endAngle;
                scanStepAngle_ = signedStep;
                scanSpeed_ = speed;
                scanFrameIndex_ = 0;
                softwareScanActive_ = false;
                waitingForTargetAngle_ = false;
                waitingForManualTargetAngle_ = false;
                scanExposureSequence_.clear();
                scanExposureSequenceIndex_ = 0;
                scanReconstructionExposureIndex_ = 0;
                waitingForScanMultiExposureFrame_ = false;
                activeTargetAngle_ = startAngle;
                if(savePath_ != "**TEMP**" && savePath_ != ""){
                    // 取最后一级目录名
                    cloudName_ = savePath_.split("/").last();
                } else {
                    QDateTime currentDateTime = QDateTime::currentDateTime();
                    cloudName_ = currentDateTime.toString("yyyyMMdd_hhmmss");
                }
                isOnEvent_ = false;
                const float monitorAngle = currentMonitorAngle();
                manualStartAngle_ = monitorAngle;
                waitingForScanStartPosition_ = true;
                if (!motor_.goTo(speed, startAngle)) {
                    waitingForScanStartPosition_ = false;
                    emit sendInformation(QStringLiteral("移动到扫描起始位置失败：%1").arg(startAngle, 0, 'f', 2));
                    finishScan(true);
                    return;
                }
                emit sendInformation(QStringLiteral("移动到扫描起始位置：%1").arg(startAngle, 0, 'f', 2));
                if (isAtTargetAngle(startAngle) || isAtOrPastStartAngle(monitorAngle)) {
                    QTimer::singleShot(0, this, [this]() {
                        continueScanAfterMoveToStart();
                    });
                    return;
                }
                QTimer::singleShot(estimateMoveDelayMs(monitorAngle, startAngle), this, [this]() {
                    continueScanAfterMoveToStart();
                });
                return;
            }
        });
    }

    void Scanner::startNextScanStep() {
        if (!softwareScanActive_ || leftReconstructionFrameCount_ <= 0) {
            if (softwareScanActive_) {
                finishScan(false);
            }
            return;
        }
        if (scanFrameIndex_ < 0 || scanFrameIndex_ >= static_cast<int>(scanTargets_.size())) {
            finishScan(false);
            return;
        }

        const float targetAngle = scanTargets_[scanFrameIndex_];
        scan_now_ = scanPositionToFrameIndex(targetAngle);
        activeTargetAngle_ = targetAngle;
        previousMotorFeedbackAngle_ = currentMonitorAngle();
        hasPreviousMotorFeedback_ = true;
        prepareCurrentScanExposureSequence();
        waitingForTargetAngle_ = true;
        if (!motor_.goTo(scanSpeed_, targetAngle)) {
            emit sendInformation(QStringLiteral("移动到扫描位置失败：%1").arg(targetAngle, 0, 'f', 2));
            finishScan(true);
            return;
        }
        emit sendInformation(QStringLiteral("移动到扫描位置：%1").arg(targetAngle, 0, 'f', 2));
    }

    int Scanner::estimateMoveDelayMs(float fromAngle, float toAngle) const {
        const float safeSpeed = std::max(std::abs(scanSpeed_), 0.1f);
        const float distance = std::abs(toAngle - fromAngle);
        const int moveMs = static_cast<int>((distance / safeSpeed) * 1000.0f);
        return std::clamp(moveMs + 300, 500, 15000);
    }

    int Scanner::plannedScanFrameCount(float startAngle, float endAngle, float step) const {
        const float signedStep = startAngle < endAngle ? std::abs(step) : -std::abs(step);
        return static_cast<int>(buildScanTargets(startAngle, endAngle, signedStep).size());
    }

    float Scanner::currentMonitorAngle() const {
        return hasMotorFeedback_ ? lastMotorFeedbackAngle_ : motor_.currentAngle();
    }

    bool Scanner::isAtOrPastTargetAngle(float targetAngle) const {
        const float currentAngle = currentMonitorAngle();
        const float tolerance = std::max(std::abs(scanStepAngle_) * 0.25f, 0.05f);
        if (scanStepAngle_ >= 0.0f) {
            return currentAngle + tolerance >= targetAngle;
        }
        return currentAngle - tolerance <= targetAngle;
    }

    bool Scanner::isAtTargetAngle(float targetAngle, float tolerance) const {
        const float currentAngle = currentMonitorAngle();
        return std::abs(currentAngle - targetAngle) <= tolerance;
    }

    bool Scanner::isAtOrPastManualTargetAngle(float currentAngle) const {
        constexpr float tolerance = 0.02f;
        if (manualTargetAngle_ >= manualStartAngle_) {
            return currentAngle + tolerance >= manualTargetAngle_;
        }
        return currentAngle - tolerance <= manualTargetAngle_;
    }

    bool Scanner::isAtOrPastStartAngle(float currentAngle) const {
        constexpr float tolerance = 0.02f;
        if (scanStartAngle_ >= manualStartAngle_) {
            return currentAngle + tolerance >= scanStartAngle_;
        }
        return currentAngle - tolerance <= scanStartAngle_;
    }

    void Scanner::continueScanAfterMoveToStart() {
        QMutexLocker lock(&m_mutex);
        if (!waitingForScanStartPosition_) {
            return;
        }
        waitingForScanStartPosition_ = false;
        motor_.stopMove();
        if (saveSceneImages_ && !savePath_.isEmpty() && savePath_ != "**TEMP**") {
            camera_.setExposureTime(sceneExposureTime_);
            saveCurrentExposureImage("before_scan");
            return;
        }
        softwareScanActive_ = true;
        emit scanBegin();
        startNextScanStep();
    }

    bool Scanner::shouldCaptureAtCurrentFeedback() const {
        if (!softwareScanActive_ || !waitingForTargetAngle_) {
            return false;
        }
        constexpr int toleranceCentiDegree = 50;
        const int current = monitorDisplayCentiDegree(currentMonitorAngle());
        const int target = monitorDisplayCentiDegree(activeTargetAngle_);
        if (std::abs(current - target) <= toleranceCentiDegree) {
            return true;
        }
        if (!hasPreviousMotorFeedback_) {
            return false;
        }

        const int previous = monitorDisplayCentiDegree(previousMotorFeedbackAngle_);
        const int lower = target - toleranceCentiDegree;
        const int upper = target + toleranceCentiDegree;
        if (scanStepAngle_ >= 0.0f) {
            return previous < lower && current > upper;
        }
        return previous > upper && current < lower;
    }

    void Scanner::stopScan(){ 
        motor_.stopScan();
        motor_.stopMove();
        softwareScanActive_ = false;
        waitingForTargetAngle_ = false;
        waitingForManualTargetAngle_ = false;
        waitingForScanMultiExposureFrame_ = false;
        scanExposureSequence_.clear();
        scanExposureSequenceIndex_ = 0;
        finishScan(true);
    }

    void Scanner::setLimit(float lowerLimit, float upperLimit){
        motor_.setLimit(lowerLimit, upperLimit);
        std::cout << ">>> [Debug] Scanner::setLimit() lowerLimit: " << lowerLimit << " upperLimit: " << upperLimit << std::endl;
    }

    void Scanner::onImageGrabbed(const Pylon::CGrabResultPtr grabResult){
        isOnEvent_ = true;
        QMutexLocker lock(&m_mutex);  // 这里加锁，保护扫描状态与帧处理
        if (grabResult->GrabSucceeded() && isOnEvent_) {
            // 读取 ChunkData
            // try {
            //     GenApi::INodeMap& chunkDataMap = grabResult->GetChunkDataNodeMap();                
            //     auto getChunk = [&](const Pylon::String_t& name) -> double {
            //         GenApi::CFloatPtr ptr(chunkDataMap.GetNode(name));
            //         return (GenApi::IsReadable(ptr)) ? ptr->GetValue() : -1.0;
            //     };
            //     double grabbedExposureTime = 0.0;
            //     grabbedExposureTime = getChunk("ChunkExposureTime");
            //     if (grabbedExposureTime <= 0) grabbedExposureTime = getChunk("ChunkExposureTimeAbs");
            //     if (grabbedExposureTime <= 0) grabbedExposureTime = getChunk("ChunkExposure");

            //     // 如果读取到了有效值
            //     if (grabbedExposureTime > 0) {
            //         scannerInfo_.exposureTime = grabbedExposureTime;
            //     } 
            // } catch (const Pylon::GenericException& e) {
            //     qWarning() << "Chunk 瑙ｆ瀽寮傚父:" << e.GetDescription();
            // }

            cv::Mat image;
            QString type = camera_.pixelFormat();
            if(type == "Mono8"){
                const uint8_t* pImageBuffer = (uint8_t*)grabResult->GetBuffer();
                image = cv::Mat(grabResult->GetHeight(), grabResult->GetWidth(), CV_8UC1, const_cast<uint8_t*>(pImageBuffer));
            } else if(type == "BayerBG8") {
                const uint8_t* pImageBuffer = (uint8_t*)grabResult->GetBuffer();
                image = cv::Mat(grabResult->GetHeight(), grabResult->GetWidth(), CV_8UC1, const_cast<uint8_t*>(pImageBuffer));
                cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
            }
            if (image.empty()) {
                qWarning() << QStringLiteral("相机返回了不支持或为空的图像格式:") << type;
                return;
            }
            image = image.clone();

            if (softwareScanActive_ && waitingForScanMultiExposureFrame_ && !scanExposureSequence_.empty()) {
                const double exposure = scanExposureSequence_.at(scanExposureSequenceIndex_);
                waitingForScanMultiExposureFrame_ = false;
                if (scanMultiExposureSaveAllImages_ && !savePath_.isEmpty() && savePath_ != "**TEMP**") {
                    const QString filename = QString("%1/%2.%3")
                        .arg(savePath_)
                        .arg(currentScanImageFileNameForExposure(exposure))
                        .arg(scanImageFormat_.isEmpty() ? "bmp" : scanImageFormat_);
                    try {
                        if (!detail::saveImageFile(filename, image)) {
                            emit sendInformation(QStringLiteral("多重曝光图像保存失败：无法写入 %1").arg(filename));
                        }
                    } catch (const cv::Exception& e) {
                        emit sendInformation(QStringLiteral("多重曝光图像保存失败：%1").arg(e.what()));
                    }
                }

                const bool useForReconstruction = !scanMultiExposureEnabled_ || scanExposureSequenceIndex_ == scanReconstructionExposureIndex_;
                if (useForReconstruction) {
                    QDateTime currentDateTime = QDateTime::currentDateTime();
                    const float monitorAngle = currentMonitorAngle();
                    const int frameScanIndex = scan_now_;
                    ReconstructionFramePtr frame(new ReconstructionFrame{
                        frameScanIndex,
                        static_cast<double>(monitorAngle),
                        static_cast<double>(scanStartAngle_),
                        static_cast<double>(scanEndAngle_),
                        true,
                        false,
                        false,
                        savePointCloud_ || saveDepthMaps_ || saveCompositeImage_ || saveTexturedPointCloud_,
                        false,
                        image.clone(),
                        leftReconstructionFrameCount_ > 0 ? savePath_ : "**TEMP**",
                        scanImageFormat_,
                        QString("%1.%2").arg(cloudName_).arg(currentScanImageBaseName()),
                        sessionId_
                    });
                    emit frameScanned(frame);
                    emit frameCaptured(frame);
                    leftReconstructionFrameCount_--;
                    const int progress = totalFrameCount_ > 0
                        ? (totalFrameCount_ - leftReconstructionFrameCount_) * 100 / totalFrameCount_
                        : 100;
                    if (leftReconstructionFrameCount_ <= 0) {
                        finishScan(false);
                        emit sendPorgress(progress);
                    } else if (softwareScanActive_) {
                        emit sendPorgress(progress);
                    }
                } else {
                    emit frameCaptured(ReconstructionFramePtr(new ReconstructionFrame{
                        scan_now_,
                        static_cast<double>(currentMonitorAngle()),
                        static_cast<double>(scanStartAngle_),
                        static_cast<double>(scanEndAngle_),
                        true,
                        false,
                        false,
                        false,
                        false,
                        image.clone(),
                        "**TEMP**",
                        scanImageFormat_,
                        currentScanImageFileNameForExposure(exposure),
                        QString()
                    }));
                }

                ++scanExposureSequenceIndex_;
                if (scanExposureSequenceIndex_ < static_cast<int>(scanExposureSequence_.size())) {
                    QTimer::singleShot(0, this, [this]() {
                        grabCurrentScanStepFrame();
                    });
                } else if (softwareScanActive_ && leftReconstructionFrameCount_ > 0) {
                    scanFrameIndex_++;
                    QTimer::singleShot(10, this, [this]() {
                        startNextScanStep();
                    });
                }
                return;
            }

            cv::Scalar meanValue = cv::mean(image);
            const double brightness = (meanValue[0] + meanValue[1] + meanValue[2]) / (image.channels() == 1 ? 1.0 : 3.0);
            if (!pendingBeforeScanSnapshot_ && !pendingAfterScanSnapshot_ && leftReconstructionFrameCount_ > 0 && brightness < 3.0) {
                emit sendInformation(QStringLiteral("相机图像亮度过低，可能激光未打开或扫描曝光过低，当前平均亮度：%1").arg(brightness, 0, 'f', 2));
            }

            QDateTime currentDateTime = QDateTime::currentDateTime();
            const float monitorAngle = currentMonitorAngle();

                // ---------------- 保存扫描前后曝光图 ----------------
            if (pendingBeforeScanSnapshot_ || pendingAfterScanSnapshot_) {
            QString suffix = pendingBeforeScanSnapshot_ ? "before_scan" : "after_scan";
                pendingBeforeScanSnapshot_ = false;
                pendingAfterScanSnapshot_ = false;

                if (!savePath_.isEmpty() && savePath_ != "**TEMP**") {
                    QString filename = QString("%1/%2_%3.%4")
                                    .arg(savePath_)
                                    .arg(currentDateTime.toString("yyyyMMdd_hhmmss"))
                                    .arg(suffix)
                                    .arg(sceneImageFormat_);
                    try {
                        if (!detail::saveImageFile(filename, image)) {
                            emit sendInformation(QStringLiteral("曝光图保存失败：无法写入 %1").arg(filename));
                        }
                        qDebug() << "Exposure snapshot saved:" << filename;
                    } catch(const cv::Exception& e) {
                        qWarning() << QStringLiteral("保存曝光图失败") << e.what();
                    }
                }

                if (suffix == "before_scan") {
                    scanExposureCaptured_ = true;
                    camera_.setExposureTime(scanExposureTime_);
                    QTimer::singleShot(50, this, [this]() {
                        softwareScanActive_ = true;
                        emit scanBegin();
                        startNextScanStep();
                    });
                }
                if (pendingFinishAfterSceneSnapshot_) {
                    pendingFinishAfterSceneSnapshot_ = false;
                    if (sceneExposureTime_ > 0.0) {
                        camera_.setExposureTime(sceneExposureTime_);
                    }
                    camera_.ContinuousGrab();
                }
                emit frameCaptured(ReconstructionFramePtr(new ReconstructionFrame{
                    scanPositionToFrameIndex(monitorAngle),
                    static_cast<double>(monitorAngle),
                    static_cast<double>(monitorAngle),
                    static_cast<double>(monitorAngle),
                    true,
                    false,
                    true,
                    false,
                    false,
                    std::move(image),
                    "**TEMP**",
                    sceneImageFormat_,
                    currentDateTime.toString("yyyyMMdd_hhmmss.scene"),
                    ""
                }));
                return;
            }

            // QMutexLocker lock(&m_mutex);  // 如有需要可在此加锁
            const int monitorScanIndex = scanPositionToFrameIndex(monitorAngle);
            const int frameScanIndex = isSingleScan_ ? monitorScanIndex : scan_now_;
            ReconstructionFramePtr frame(new ReconstructionFrame{
                    frameScanIndex,
                    static_cast<double>(monitorAngle),
                    static_cast<double>(scanStartAngle_),
                    static_cast<double>(scanEndAngle_),
                    true,
                    isSingleScan_ ? false : saveScanImages_,   // 单次扫描时不保存图像
                    isSingleScan_ ? true : false,
                    isSingleScan_ ? false : (savePointCloud_ || saveDepthMaps_ || saveCompositeImage_ || saveTexturedPointCloud_),
                    false,
                    std::move(image),
                    leftReconstructionFrameCount_ > 0 ? savePath_ : "**TEMP**",
                    scanImageFormat_,
                    QString("%1.%2").arg(isSingleScan_ ? currentDateTime.toString("yyyyMMdd_hhmmss") : cloudName_).arg(isSingleScan_ ? "singleScan" : QString::number(frameScanIndex)),
                    sessionId_
            });
            if(isSingleScan_){
                isSingleScan_ = false;
                emit frameScanned(frame);
            } else if (leftReconstructionFrameCount_ > 0){  //浜屾鍒ゆ柇
                emit frameScanned(frame);
                leftReconstructionFrameCount_--;
                const int progress = (totalFrameCount_ - leftReconstructionFrameCount_) * 100 / totalFrameCount_;
                if(leftReconstructionFrameCount_ == 0){
                    finishScan(false);
                    emit sendPorgress(progress);
                } else if (softwareScanActive_) {
                    emit sendPorgress(progress);
                    if (isAtOrPastTargetAngle(scanEndAngle_) && scanFrameIndex_ >= totalFrameCount_ - 1) {
                        finishScan(false);
                        emit sendPorgress(100);
                    } else {
                        scanFrameIndex_++;
                        QTimer::singleShot(10, this, [this]() {
                            startNextScanStep();
                        });
                    }
                }
            }
            emit frameCaptured(frame);
        } else {
            qDebug() << QStringLiteral("图像获取失败");
        }
    }
}}
