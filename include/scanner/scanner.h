#ifndef SCANNER_H
#define SCANNER_H
#include "utils.h"
#include "scanner/camera.hpp"
#include "scanner/modbusmotor.h"
#include <opencv2/opencv.hpp>
#include <QString>
#include <QUuid>
#include <QDir>
#include <opencv2/core.hpp>
#include <QTimer>
#include "setting/settingsmanager.h"
#include <vector>

NAMESPACE_BEGIN { namespace scanner {
    class Reconstructor;
    /**
     * @brief 单次扫描的重建帧
     */
    struct ReconstructionFrame{
        int scanIndex;                      ///< 扫描标记，这里通常对应电机角度或帧索引
        double laserPosition;               ///< 当前帧对应的实际激光位置
        double scanStartPosition;           ///< 当前扫描的开始位置
        double scanEndPosition;             ///< 当前扫描的结束位置
        bool isInWater;                     ///< 是否为水下扫描
        bool saveImage;                     ///< 是否保存图像
        bool singleScan;                    ///< 是否为单次扫描
        bool savePointCloud;                ///< 是否在扫描结束后保存点云/中心线数据
        bool yyy;                           ///< 预留标志位
        cv::Mat image;                      ///< 图像
        const QString savePath;             ///< 图像保存路径
        const QString imageFormat;          ///< 图像保存格式
        const QString pointcloudName;       ///< 导出的点云名称
        const QString sessionId;            ///< 会话 ID，用于标识一次连续扫描
    };

    /**
     * @brief 扫描仪信息
     */
    struct ScannerInfo{
        float angle = 0.0f;
        float humidity = 0.0f;
        float temperature = 0.0f;
        float pressure = 0.0f;
        quint8 attitudeFlag = 0;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        QByteArray reserved;
        double exposureTime = 0.0;
        float angle_max = 0.0f;
        float angle_min = 0.0f;
        bool isOpen = false;
    };

    using ReconstructionFramePtr = std::shared_ptr<ReconstructionFrame>;

    /**
     * @brief 三维扫描仪抽象实体
     */
    class Scanner: public QObject {
        Q_OBJECT
    public:
        explicit Scanner(QObject* parent = nullptr);
        virtual ~Scanner() = default;
        inline void setReconstructor(Reconstructor* reconstructor) {
                  m_reconstructor = reconstructor;}
        void saveCurrentExposureImage(const QString& suffix);
        void finishScan(bool cancelled);
        void startNextScanStep();
        void continueScanAfterMoveToStart();
        int estimateMoveDelayMs(float fromAngle, float toAngle) const;
        bool isAtOrPastTargetAngle(float targetAngle) const;
        bool isAtOrPastManualTargetAngle(float currentAngle) const;
        bool isAtOrPastStartAngle(float currentAngle) const;
        bool shouldCaptureAtCurrentFeedback() const;
        int plannedScanFrameCount(float startAngle, float endAngle, float step) const;
        bool isAtTargetAngle(float targetAngle, float tolerance = 0.02f) const;
        float currentMonitorAngle() const;
    public slots:
        /**
         * @brief 连接设备，初始化相机和电机
         */
        bool connectDevices();

        /**
         * @brief 断开设备连接，释放相机和电机资源
         */
        void disconnectDevices();

        /**
         * @brief 检查设备是否已连接
         * @return 已连接返回 true，否则返回 false
         */
        bool isConnected() const;

        /**
         * @brief 单次抓取图像
         */
        void singleGrab();

        /**
         * @brief 连续抓取图像
         */
        bool continuousGrab();

        /**
         * @brief 停止抓取图像
         */
        bool stopGrab();

        /**
         * @brief 打开激光
         */
        bool openLaser();

        /**
         * @brief 关闭激光
         */
        bool closeLaser();

        /**
         * @brief 将相机曝光切换为场景曝光
         */
        void setSceneExposureTime(double exposureTime);

        /**
         * @brief 重建当前帧，可用于单次预览重建
         */
        void reconstructCurrent();

        /**
         * @brief 移动电机到指定角度
         * @param speed 运动速度
         * @param angle 目标角度
         */
        void moveMotor(float speed, float angle);

        /**
         * @brief 开始扫描
         * @param startAngle 开始角度
         * @param endAngle 结束角度
         * @param step 扫描步长
         * @param speed 扫描速度
         * @param savePath 扫描数据保存路径
         */
        void startScan(float startAngle,
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
                       double multiExposureStep);

        /**
         * @brief 停止扫描
         */
        void stopScan();

        /**
         * @brief 设置亮度
         * @param brightness 亮度值，范围 0-100
         */
        void setBrightness(int brightness);

        /**
         * @brief 设置曝光时间
         * @param exposureTime 曝光时间，单位为 us
         */
        void setExposureTime(double exposureTime);

        /**
         * @brief 设置相机触发模式和触发源。
         */
        void setTriggerConfig(bool enabled, const QString& source);

        /**
         * @brief 获取曝光时间
         * @return 曝光时间，单位为 us
         */
        inline double getExposureTime() const {return camera_.exposureTime();}
        inline std::pair<double, double> getExposureTimeRange() const { return camera_.exposureTimeRange(); }

        /**
         * @brief 设置增益
         * @param gain 增益
         */
        inline void setGain(double gain) {camera_.setGain(gain);}

        /**
         * @brief 鑾峰彇澧炵泭
         * @return 澧炵泭
         */
        inline double getGain() const {return camera_.gain();}
        inline std::pair<double, double> getGainRange() const { return camera_.gainRange(); }


        /**
         * @brief 设置扫描范围
         * @param lowerLimit 下限角度
         * @param upperLimit 上限角度
         */
        void setLimit(float lowerLimit, float upperLimit);

        /**
         * @brief 鑾峰彇褰撳墠鐨勪笅闄愪綅
         */
        inline float getCurrentLimitLower(){ return motor_.currentLimit().first; }

        /**
         * @brief 鑾峰彇褰撳墠鐨勪笂闄愪綅
         */
        inline float getCurrentLimitUpper(){ return motor_.currentLimit().second; }
    signals:
        /**
         * @brief 信号：发送扫描到的一帧图像给重建线程处理
         * @param frame 扫描到的一帧图像
         */
        void frameScanned(ReconstructionFramePtr frame);

        /**
         * @brief 信号：扫描结束
         */
        void scanFinished();

        /**
         * @brief 信号：发送扫描到的一帧图像给预览线程处理
         * @param frame 扫描到的一帧图像
         */
        void frameCaptured(ReconstructionFramePtr frame);

        /**
         * @brief 信号：发送扫描信息
         * @param info 扫描信息
         */
        void sendInfo(ScannerInfo info);

        /**
         * @brief 信号：发送扫描进度
         * @param progress 扫描进度，范围为 0-100
         */
        void sendPorgress(int progress);

        /**
         * @brief 信号：发送通知信息
         * @param info 通知信息
         */
        void sendInformation(const QString& info);

        /**
         * @brief 信号：扫描开始
         */
        void scanBegin();

        /**
         * @brief 信号：扫描停止
         */
        void scanStopped();

        /**
         * @brief 信号：激光器打开时触发
         */
        void laserOpened();

        /**
         * @brief 信号：激光器关闭时触发
         */
        void laserClosed();

    private slots:
        /**
         * @brief 槽函数，处理相机捕获的一帧图像
         * @param grabResult 相机捕获的一帧图像
         */
        void onImageGrabbed(const Pylon::CGrabResultPtr grabResult);

    private:
        int leftReconstructionFrameCount_;  ///< 剩余需要重建的帧数
        int totalFrameCount_;               ///< 总帧数
        int scan_step_;                     ///< 扫描步长
        int scan_now_;                      ///< 当前扫描角度
        float scanStartAngle_ = 0.0f;
        float scanEndAngle_ = 0.0f;
        float scanStepAngle_ = 0.0f;
        float scanSpeed_ = 1.0f;
        int scanFrameIndex_ = 0;
        std::vector<float> scanTargets_;
        bool softwareScanActive_ = false;
        float lastMotorFeedbackAngle_ = 0.0f;
        float previousMotorFeedbackAngle_ = 0.0f;
        bool hasMotorFeedback_ = false;
        bool hasPreviousMotorFeedback_ = false;
        float activeTargetAngle_ = 0.0f;
        bool waitingForTargetAngle_ = false;
        bool waitingForManualTargetAngle_ = false;
        bool waitingForScanStartPosition_ = false;
        float manualTargetAngle_ = 0.0f;
        float manualStartAngle_ = 0.0f;

        QString cloudName_;                 ///< 生成点云的名称
        Camera camera_;                     ///< 相机对象
        ModbusMotor motor_;                 ///< 电机对象
        QString savePath_;                  ///< 扫描时图像数据的保存路径
        QString sceneImageFormat_ = "png";  ///< 场景图保存格式
        QString scanImageFormat_ = "bmp";   ///< 扫描图保存格式
        QString sessionId_;                 ///< 扫描会话 ID
        bool saveSceneImages_ = true;       ///< 扫描前后是否保存场景图
        bool saveScanImages_ = true;        ///< 扫描时是否保存原始图
        bool savePointCloud_ = true;        ///< 扫描结束后是否保存点云/中心线数据
        bool saveDepthMaps_ = false;        ///< 扫描结束后是否保存深度图
        bool saveCompositeImage_ = false;   ///< 扫描结束后是否保存合成图
        bool saveTexturedPointCloud_ = false; ///< 扫描结束后是否保存带纹理 PLY
        QString depthImageFormat_ = "png";  ///< 深度图格式
        QString compositeImageFormat_ = "png"; ///< 合成图格式
        bool isSingleScan_;                 ///< 是否为单点扫描
        bool isOnEvent_;                    ///< 是否正在 onImageGrabbed 中处理图像
        QMutex m_mutex;                     ///< 互斥锁，用于保护扫描状态修改

        ScannerInfo scannerInfo_{};         ///< 扫描仪信息

        Reconstructor* m_reconstructor = nullptr;
        bool pendingBeforeScanSnapshot_;
        bool pendingAfterScanSnapshot_;
        double sceneExposureTime_ = 0.0;
        double scanExposureTime_ = 0.0;
        bool scanExposureCaptured_ = false;
        bool pendingFinishAfterSceneSnapshot_ = false;
        int previewExposureChangeSerial_ = 0;
        bool scanMultiExposureEnabled_ = false;
        double scanMultiExposureMin_ = 0.0;
        double scanMultiExposureMax_ = 0.0;
        double scanMultiExposureStep_ = 0.0;
        std::vector<double> scanExposureSequence_;
        int scanExposureSequenceIndex_ = 0;
        int scanReconstructionExposureIndex_ = 0;
        bool waitingForScanMultiExposureFrame_ = false;
        bool scanMultiExposureSaveAllImages_ = false;

        int exposureSettleDelayMs(double exposureTime) const;
        void prepareCurrentScanExposureSequence();
        void grabCurrentScanStepFrame();
        QString currentScanImageBaseName() const;
        QString currentScanImageFileNameForExposure(double exposure) const;
    };
}}

Q_DECLARE_METATYPE(UWScan::scanner::ScannerInfo)
#endif
