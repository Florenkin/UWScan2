// associatedpointsmanager.h
#ifndef ASSOCIATEDPOINTSMANAGER_H
#define ASSOCIATEDPOINTSMANAGER_H
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include <map>
#include <memory>
#include <fstream>
#include <mutex>
#include <thread>
#include <QDateTime>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "utils.h"

NAMESPACE_BEGIN { namespace scanner {
    /**
     * @brief 关键点数据结构，包含中心点（未去畸变）、中心点（去畸变）、重建三维点一一对应
     */
    struct AssociatedPoints {
        cv::Point2d undistortedCenter;          ///< 未去畸变的中心点
        cv::Point2d distortedCenter;            ///< 去畸变的中心点
        pcl::PointXYZ reconstructedPoint;       ///< 重建的三维点坐标
        uchar grayValue = 0;                    ///< 激光中心点对应的近似灰度
        AssociatedPoints() = default;
        AssociatedPoints(const cv::Point2d& undistortedCenter, const cv::Point2d& distortedCenter, const pcl::PointXYZ& reconstructedPoint)
            : undistortedCenter(undistortedCenter), distortedCenter(distortedCenter), reconstructedPoint(reconstructedPoint) {}
        AssociatedPoints(const cv::Point2d& undistortedCenter, const cv::Point2d& distortedCenter, const pcl::PointXYZ& reconstructedPoint, uchar gray)
            : undistortedCenter(undistortedCenter), distortedCenter(distortedCenter), reconstructedPoint(reconstructedPoint), grayValue(gray) {}
    };

    using AssociatedPointsPtr = std::shared_ptr<std::vector<AssociatedPoints>>;

    /**
     * @brief 扫描会话
     */
    struct ScanSession {
        QString sessionId;                          ///< 会话ID
        QString savePath;                           ///< 保存路径
        std::vector<AssociatedPointsPtr> frameData; ///< 每一帧的关联点数据
        int totalFrames;                            ///< 总帧数
        int processedFrames;                        ///< 已处理帧数
        bool isCancelled;                           ///< 是否已取消
        qint64 lastUpdateTime;                      ///< 最后更新时间戳
        bool saveDepthMaps = false;                 ///< 是否导出X/Y/Z深度图
        bool saveCompositeImage = false;            ///< 是否导出合成图
        bool saveTexturedPointCloud = false;        ///< 是否导出带纹理PLY点云
        QString depthImageFormat = "png";           ///< 深度图格式
        QString compositeImageFormat = "png";       ///< 合成图格式
        cv::Size imageSize;                         ///< 关联图像尺寸
        std::vector<cv::Mat> frameImages;           ///< 原始扫描图，用于整图合成
        
        ScanSession(const QString& id, const QString& path, int total) 
            : sessionId(id), savePath(path), totalFrames(total), 
            processedFrames(0), isCancelled(false), 
            lastUpdateTime(QDateTime::currentMSecsSinceEpoch()) {}
        
        inline bool isComplete() const { return processedFrames >= totalFrames; }
        inline bool shouldSave() const { return processedFrames > 0 && (isComplete() || isCancelled); }
        inline void updateTimestamp() { lastUpdateTime = QDateTime::currentMSecsSinceEpoch(); }
        inline bool isTimeout(qint64 timeoutMs = 30000) const { 
            return QDateTime::currentMSecsSinceEpoch() - lastUpdateTime > timeoutMs; 
        }
    };

    using ScanSessionPtr = std::shared_ptr<ScanSession>;

    /**
     * @brief 保存任务数据结构
     */
    struct SaveTask {
        QString sessionId;
        QString savePath;
        AssociatedPointsPtr points;
        ScanSessionPtr session;
        bool isFinal; // 是否为最终保存

        SaveTask(const QString& id,
                 const QString& path,
                 AssociatedPointsPtr pts,
                 bool final = false,
                 ScanSessionPtr sessionSnapshot = nullptr)
            : sessionId(id),
              savePath(path),
              points(pts),
              session(sessionSnapshot),
              isFinal(final) {}
    };

    using SaveTaskPtr = std::shared_ptr<SaveTask>;

    /**
     * @brief 关联点管理器，负责异步保存关联点数据
     * 该类使用单例模式，确保全局只有一个实例，并可以通过instance方法获取实例
     */
    class AssociatedPointsManager : public QObject {
        Q_OBJECT

    public:
        static AssociatedPointsManager* getInstance();
        
        // 禁止拷贝和赋值
        AssociatedPointsManager(const AssociatedPointsManager&) = delete;
        AssociatedPointsManager& operator=(const AssociatedPointsManager&) = delete;

    public slots:
        void start();
        void stop();
        
        // 会话管理
        void createSession(const QString& sessionId,
                           const QString& savePath,
                           int totalFrames,
                           bool saveDepthMaps = false,
                           bool saveCompositeImage = false,
                           bool saveTexturedPointCloud = false,
                           const QString& depthImageFormat = "png",
                           const QString& compositeImageFormat = "png");
        void cancelSession(const QString& sessionId);
        void cancelAllSessions();
        
        // 添加关联点数据
        void addAssociatedPoints(const QString& sessionId,
                                 AssociatedPointsPtr points,
                                 const cv::Size& imageSize = cv::Size(),
                                 const cv::Mat& frameImage = cv::Mat());

    signals:
        void sessionProgress(const QString& sessionId, int processedFrames, int totalFrames);
        void sessionCompleted(const QString& sessionId, int savedFrames);
        void sessionCancelled(const QString& sessionId, int savedFrames);
        void saveError(const QString& sessionId, const QString& error);
        void compositeImageSaved(const QString& sessionId, const QString& filePath);
        void intensityImageSaved(const QString& sessionId, const QString& filePath);

    private:
        AssociatedPointsManager(QObject* parent = nullptr);
        ~AssociatedPointsManager();
        
        void workerThread(); // 工作线程函数
        void processSaveTask(SaveTaskPtr task); // 处理保存任务
        void saveIncrementalFrameData(SaveTaskPtr task);
        void saveSessionData(const ScanSessionPtr& session); // 保存会话数据
        
        // 会话管理
        ScanSessionPtr getSession(const QString& sessionId);
        void cleanupTimeoutSessions();
        
    private:
        static AssociatedPointsManager* instance_;
        std::thread workerThread_;
        bool isRunning_;
        
        QMutex mutex_;
        QWaitCondition condition_;
        std::queue<SaveTaskPtr> taskQueue_;
        
        std::map<QString, ScanSessionPtr> activeSessions_;
        QMutex sessionMutex_;
    };

}}

#endif // ASSOCIATEDPOINTSMANAGER_H
