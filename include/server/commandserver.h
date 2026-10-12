#ifndef COMMANDSERVER_H
#define COMMANDSERVER_H

#include "protocal.h"
#include <cstdint>
#include <QtGlobal>
#include <QTcpServer>
#include <QUdpSocket>
#include <QJsonObject>
#include <QHash>
#include <QQueue>
#include <opencv2/opencv.hpp>
#include "pointcloud/pc.hpp"
#include "scanner/scanner.h"

NAMESPACE_BEGIN{ namespace server{
    class RemoteCommandServer : public QObject
    {
        Q_OBJECT
    public:
        explicit RemoteCommandServer(QObject *parent = nullptr);
        ~RemoteCommandServer();

    public slots:
        /**
         * @brief 启动服务
         */
        void startService();

        /**
         * @brief 停止服务
         */
        void stopService();

        /**
         * @brief --- 来自 MainWindow 的数据更新 (用于 UDP 推送) ---
         * 更新设备状态 (Main -> Server)
         * @param statusJson 包含设备状态的 JSON 对象
         */
        void updateDeviceStatus(const QJsonObject &statusJson);

        /**
         * @brief 发送扫描器信息 (Main -> Server)
         * @param scannerInfo 包含扫描器信息的 ScannerInfo 对象
         */
        void sendScannerInfo(const scanner::ScannerInfo &scannerInfo);
        
        /**
         * @brief 发送图片 (Main -> Server)
         * @param image 包含图片数据的 Mat 对象
         */
        void sendImage(const cv::Mat &image);
        
        /**
         * @brief 发送点云 (Main -> Server)
         * @param cloud 包含点云数据的 PointCloudPtr 对象
         */
        void sendPointCloud(pointcloud::PointCloudPtr cloud);

        /**
         * @brief 查询服务是否正在监听。
         *
         * 该槽函数供主窗口通过 BlockingQueuedConnection 调用，避免跨线程直接访问
         * QTcpServer。
         */
        bool isServiceRunning() const;

        /**
         * @brief --- 来自 MainWindow 的指令执行结果反馈 (异步响应 TCP) ---
         * 打开扫描器结果 (Main -> Server)
         * @param success 是否成功打开扫描器
         * @param msg 包含结果信息的字符串
         */
        void onScannerOpenedResult(bool success, const QString &msg);

        /**
         * @brief 关闭扫描器结果 (Main -> Server)
         * @param success 是否成功关闭扫描器
         * @param msg 包含结果信息的字符串
         */
        void onScannerClosedResult(bool success, const QString &msg);

        /**
         * @brief 开始扫描结果 (Main -> Server)
         * @param success 是否成功开始扫描
         * @param msg 包含结果信息的字符串
         */
        void onStartScanResult(bool success, const QString &msg);

        /**
         * @brief 停止扫描结果 (Main -> Server)
         * @param success 是否成功停止扫描
         * @param msg 包含结果信息的字符串
         */
        void onStopScanResult(bool success, const QString &msg);

        /**
         * @brief 打开激光结果 (Main -> Server)
         * @param success 是否成功打开激光
         * @param msg 包含结果信息的字符串
         */
        void onOpenLaserResult(bool success, const QString &msg);

        /**
         * @brief 关闭激光结果 (Main -> Server)
         * @param success 是否成功关闭激光
         */
        void onCloseLaserResult(bool success, const QString &msg);

        /**
         * @brief 改变参数结果 (Main -> Server)
         * @param success 是否成功改变参数
         * @param msg 包含结果信息的字符串
         * @param reqId 对应的请求 ID
         */
        void onParameterChangedResult(bool success, const QString &msg);

        /**
         * @brief 转到角度结果 (Main -> Server)
         * @param success 是否成功转到角度
         * @param msg 包含结果信息的字符串
         */
        void onGoToAngleResult(bool success, const QString &msg);

        /**
         * @brief 设置扫描范围限制结果 (Main -> Server)
         * @param success 是否成功设置扫描范围限制
         * @param msg 包含结果信息的字符串
         */
        void onSetLimitResult(bool success, const QString &msg);

        /**
         * @brief 设置曝光时间结果 (Main -> Server)
         * @param success 是否成功设置曝光时间
         * @param msg 包含结果信息的字符串
         */
        void onSetExposureTimeResult(bool success, const QString &msg);

        /**
         * @brief 保存图片结果 (Main -> Server)
         * @param success 是否成功保存图片
         * @param msg 包含结果信息的字符串
         */
        void onSaveImageResult(bool success, const QString &msg);

    signals:
        /**
         * @brief 向主窗口转发远程连接、命令和执行结果日志。
         */
        void logMessage(const QString &type, const QString &message);

        // --- 发送给 MainWindow 的控制指令 ---

        /**
         * @brief 请求开始扫描 (Main -> Server)
         * @param startAngle 起始角度
         * @param endAngle 结束角度
         * @param step 扫描步长
         * @param speed 扫描速度
         */
        void requestStartScan(float startAngle, float endAngle, float step, float speed, double exposureTime);

        /**
         * @brief 请求停止扫描 (Main -> Server)
         */
        void requestStopScan();

        /**
         * @brief 请求打开扫描器 (Main -> Server)
         * @param reqId 对应的请求 ID
         */
        void requestOpenScanner(uint32_t reqId); // 携带 reqId 以便回传响应

        /**
         * @brief 请求关闭扫描器 (Main -> Server)
         * @param reqId 对应的请求 ID
         */
        void requestCloseScanner(uint32_t reqId);

        /**
         * @brief 请求改变参数 (Main -> Server)
         * @param params 包含改变参数的 JSON 对象
         * @param reqId 对应的请求 ID
         */
        void requestChangeParameter(const QJsonObject &params, uint32_t reqId);

        /**
         * @brief 请求打开激光 (Main -> Server)
         */
        void requestOpenLaser();

        /**
         * @brief 请求关闭激光 (Main -> Server)
         */
        void requestCloseLaser();

        /**
         * @brief 请求转到指定角度 (Main -> Server)
         * @param angle 目标角度
         */
        void requestGoToAngle(float angle);

        /**
         * @brief 请求设置扫描范围限制 (Main -> Server)
         * @param minAngle 最小角度
         * @param maxAngle 最大角度
         */
        void requestSetLimit(float minAngle, float maxAngle);

        /**
         * @brief 请求设置曝光时间 (Main -> Server)
         * @param exposureTime 曝光时间
         */
        void requestSetExposureTime(float exposureTime);

        /**
         * @brief 请求保存图像 (Main -> Server)
         * @param filename 图像保存文件名
         */
        void requestSaveImage(const QString &filename);

    private slots:
        /**
         * @brief 处理新的 TCP 连接
         */
        void onNewTcpConnection();
        
        /**
         * @brief 处理 TCP 连接的可读数据
         */
        void onTcpReadyRead();

        /**
         * @brief 处理 TCP 连接断开的事件
         */
        void onTcpDisconnected();

        /**
         * @brief 定时器触发状态推送
         */
        void onStatusTimerTimeout();

    private:
        QTcpServer *m_tcpServer = nullptr;  ///< TCP 服务器，用于监听和接受新连接
        QUdpSocket *m_udpSocket = nullptr;  ///< UDP 套接字，用于数据上传
        
        QTcpSocket *m_currentClient = nullptr; ///< 当前连接的 TCP 客户端，假设只支持一个 TCP 控制连接
        
        QByteArray m_tcpBuffer; ///< TCP 数据缓存，用于处理粘包

        int m_udpSubscriptionMask = SubNone; ///< UDP 数据订阅掩码，默认不订阅任何数据
        QHostAddress m_targetUdpAddress; ///< 目标 UDP 地址，用于数据上传
        quint16 m_targetUdpPort = PORT_DATA_UDP; // 这里的端口通常是对方接收的端口，或者复用
        quint32 m_nextTransferId = 1;

        QTimer *m_statusTimer; ///< 用于 50ms 定时推送状态
        QJsonObject m_lastStatus; ///< 缓存最新的状态

        QHash<QString, QQueue<uint32_t>> m_reqIdMap; ///< 按命令 FIFO 保存待响应请求 ID

        // --- 内部处理函数 ---
        /**
         * @brief 处理 TCP 协议帧
         * @param header TCP 协议帧头
         * @param body TCP 协议帧体
         */
        void processTcpPacket(const TcpFrameHeader &header, const QByteArray &body);

        /**
         * @brief 发送 TCP 响应
         * @param reqId 对应的请求 ID
         * @param errorCode 错误码，成功为1，失败为其他值
         * @param body 包含响应数据的 JSON 对象
         */
        void sendTcpResponse(uint32_t reqId, uint16_t errorCode, const QJsonObject &body);

        /**
         * @brief 发送 UDP 数据包
         * @param dataType UDP 数据类型
         * @param payload 包含数据的字节数组
         */
        void sendUdpPacket(uint16_t dataType, const QByteArray &payload);
        void rememberRequest(const QString &command, uint32_t reqId);
        uint32_t takeRequestId(const QString &command);
    };
}}
#endif // COMMANDSERVER_H
