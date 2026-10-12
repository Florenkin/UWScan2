#ifndef SCANNER_RECONSTRUCTOR_H
#define SCANNER_RECONSTRUCTOR_H
#include "utils.h"
#include "scanner/scanner.h"
#include "scanner/associatedpointsmanager.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "pointcloud/pc.hpp"
#include <vector>
#include <memory>
#include <QString>
#include <queue>
#include <condition_variable>
#include <thread>
#include <QMap>
#include <QMutex>
#include <QWaitCondition>

NAMESPACE_BEGIN { namespace scanner {
    /**
     * @brief 点云重建配置参数结构体
     */
    struct ReconstructionConfig {
        std::string calibrationPath;    ///< 标定文件路径
        bool isHeng = true;             ///< 横向/纵向扫描
        int centerlineMethod = 0;       ///< 0: Gray, 1: Steger, 2: MultiThres
        int thres = 50;                 ///< 兼容旧代码的二值化阈值缓存
        int slt_thres = 10;             ///< 兼容旧代码的Steger阈值缓存
        double bwThr = 55.0;            ///< 二值化阈值
        double sumThr = 124.0;          ///< Gray时为sumThr，Steger时映射为sltThr
        bool isFullRoi = true;          ///< 是否使用全ROI
        std::vector<int> Roi = {0, 0, 0, 0};    ///< ROI区域，格式为{x, y, width, height}
        bool keepLocalCoordinates = false;      ///< 是否保留局部坐标，不再旋回世界坐标
    };

    /**
     * @brief 点云重建器
     * 内部独自运行一个线程，有一个与之相符合的队列，用于接收重建帧
     * 每个重建帧包含了一张图片，以及该图片下的扫描仪参数
     */
    class Reconstructor: public QObject {
        Q_OBJECT
    public:
        Reconstructor(QObject* parent = nullptr);
        ~Reconstructor();
        inline const ReconstructionConfig& getConfig() const {
        return m_config;
        }

        /**
         * @brief 初始化点云重建器，主要是加载标定文件
         * @param config 点云重建配置参数
         * @return 如果初始化成功则返回 true
         */
        bool init(const ReconstructionConfig& config);

        /**
         * @brief 处理一帧重建数据
         * @param frame 重建帧的智能指针
         * @param linePoints 激光中心线点集
         * @param associatedPoints 关联点数据
         * @return 如果处理成功则返回包含点云的可选值，否则返回空可选值
         */
        std::optional<pointcloud::PointCloudPtr> processFrame(const ReconstructionFramePtr& frame, 
                                                              std::vector<cv::Point2d>& linePoints,
                                                              AssociatedPointsPtr associatedPoints = nullptr);

        std::vector<cv::Point2d> extractPreviewCenterLine(const cv::Mat& image);

        /**
         * @brief 获取最后一次错误信息
         * @return 最后一次错误信息字符串
         */
        inline std::string getLastError() const { return m_lastError; }
    private:
        /**
         * @brief 工具函数，提取激光中心线
         * @param img 输入图像
         * @return 提取到的激光中心线点集
         */
        std::vector<cv::Point2d> extractCenterLine(const cv::Mat& img);

        void working();

    public slots:
        void reconstructFrame(ReconstructionFramePtr frame);

        void stopReconstruction();

        void startReconstruction();
    signals:
        /**
         * @brief 点云重建完成信号
         * @param pc 重建得到的点云
         * @param isSingleScan 是否为单帧扫描
         */
        void pointCloudReady(pointcloud::PointCloudPtr pc, bool isSingleScan);

        /**
         * @brief 激光中心线提取完成信号
         * @param linePoints 提取到的激光中心线点集
         * @param isSingleScan 是否为单帧扫描
         */
        void linePointsReady(const std::vector<cv::Point2d>& linePoints, bool isSingleScan);

        void reconstructionFailed(int scanIndex, const QString& reason, bool isSingleScan);

        void associatedPointsReady(AssociatedPointsPtr associatedPoints, bool isSingleScan);

        /**
         * @brief 加载标定文件信号
         * @param path 标定文件路径
         */
        void loadCalibration(const QString& path);
    private:
        ReconstructionConfig m_config;
        std::string m_lastError;
        bool m_isInitialized = false;

        // --- 标定数据缓存 ---
        cv::Mat m_cameraMatrix;
        cv::Mat m_distCoeffs;
        cv::Size m_picSize;
        
        // 折射相关
        Eigen::Vector3d m_mu; // 折射率向量
        double m_d0;
        double m_d1;
        Eigen::Vector3d m_axis; // 接口法向量

        // 扫描范围与曲面拟合参数
        int m_surfBeginPos;
        int m_surfEndPos;
        int m_surfStep;
        int m_surfNum;
        std::vector<Eigen::Matrix4f> m_allTransMats;
        std::vector<Eigen::VectorXf> m_allSurfacePars;
        std::vector<std::pair<double, double>> m_allMaxMinZValues;

        // 线程相关
        std::queue<ReconstructionFramePtr> m_frameQueue;
        QMutex m_queueMutex;
        QMutex m_calibrationMutex;
        QWaitCondition m_condition;
        std::thread m_thread;
        std::atomic<bool> m_isRunning = false;
        QMap<QString, pointcloud::PointCloudPtr> temp_cloud_map;
    };
}}
#endif // SCANNER_RECONSTRUCTOR_H
