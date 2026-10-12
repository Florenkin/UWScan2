#include "server/commandserver.h"
#include <QtEndian> // 处理大小端
#include <QDataStream>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTimer>
#include <QMutexLocker>
#include <QTcpSocket>
#include <QDateTime>
#include <iostream>
#include <QBuffer>
#include <algorithm>
#include <limits>

NAMESPACE_BEGIN {
namespace server {

RemoteCommandServer::RemoteCommandServer(QObject *parent) : QObject(parent) {
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<scanner::ScannerInfo>("scanner::ScannerInfo");
    qRegisterMetaType<pointcloud::PointCloudPtr>("pointcloud::PointCloudPtr");
    qRegisterMetaType<uint32_t>("uint32_t");
    m_statusTimer = new QTimer(this);
    m_statusTimer->setInterval(50); // 20fps = 50ms
    connect(m_statusTimer, &QTimer::timeout, this, &RemoteCommandServer::onStatusTimerTimeout);
}

RemoteCommandServer::~RemoteCommandServer() {
    stopService();
}

void RemoteCommandServer::startService() {
    if (m_tcpServer && m_tcpServer->isListening()) {
        emit logMessage(QStringLiteral("Info"), QStringLiteral("远程控制服务已在运行"));
        return;
    }
    // 启动 TCP 监听
    if (!m_tcpServer) {
        m_tcpServer = new QTcpServer(this);
        connect(m_tcpServer, &QTcpServer::newConnection, this, &RemoteCommandServer::onNewTcpConnection);
    }
    if (!m_tcpServer->listen(QHostAddress::Any, PORT_CMD_TCP)) {
        qWarning() << "TCP Command Server failed to listen on" << PORT_CMD_TCP
                   << m_tcpServer->errorString();
        emit logMessage(QStringLiteral("Error"),
                        QStringLiteral("远程控制服务启动失败：%1").arg(m_tcpServer->errorString()));
        return;
    }
    qDebug() << "TCP Command Server listening on" << PORT_CMD_TCP;
    emit logMessage(QStringLiteral("Info"),
                    QStringLiteral("远程控制服务已启动，TCP端口 %1，UDP端口 %2")
                        .arg(PORT_CMD_TCP).arg(PORT_DATA_UDP));

    // 初始化 UDP Socket (用于发送)
    if (!m_udpSocket) {
        m_udpSocket = new QUdpSocket(this);
    }
    
    m_statusTimer->start();
}

bool RemoteCommandServer::isServiceRunning() const {
    return m_tcpServer && m_tcpServer->isListening();
}

void RemoteCommandServer::stopService() {
    m_statusTimer->stop();
    if (m_tcpServer) {
        m_tcpServer->close();
        delete m_tcpServer;
        m_tcpServer = nullptr;
    }
    if (m_udpSocket) {
        m_udpSocket->close();
        delete m_udpSocket;
        m_udpSocket = nullptr;
    }
    m_currentClient = nullptr;
    m_tcpBuffer.clear();
    m_udpSubscriptionMask = SubNone;
    m_targetUdpPort = PORT_DATA_UDP;
    emit logMessage(QStringLiteral("Info"), QStringLiteral("远程控制服务已停止"));
}

void RemoteCommandServer::onNewTcpConnection() {
    // 这里只是单客户端模式。后期再考虑实现多客户端
    if (m_currentClient) {
        m_currentClient->close();
        m_currentClient->deleteLater();
        m_currentClient = nullptr;
    }
    m_currentClient = m_tcpServer->nextPendingConnection();
    m_tcpBuffer.clear();
    m_reqIdMap.clear();
    connect(m_currentClient, &QTcpSocket::readyRead, this, &RemoteCommandServer::onTcpReadyRead);
    connect(m_currentClient, &QTcpSocket::disconnected, this, &RemoteCommandServer::onTcpDisconnected);
    
    // 记录对方 IP，用于 UDP 回传
    m_targetUdpAddress = m_currentClient->peerAddress();
    m_targetUdpPort = PORT_DATA_UDP;
    
    qDebug() << "Client connected:" << m_targetUdpAddress.toString();
    emit logMessage(QStringLiteral("Info"),
                    QStringLiteral("远程控制客户端已连接：%1").arg(m_targetUdpAddress.toString()));
}

void RemoteCommandServer::onTcpDisconnected() {
    auto *socket = qobject_cast<QTcpSocket *>(sender());
    if (!socket || socket != m_currentClient) {
        return;
    }
    const QString peer = socket->peerAddress().toString();
    m_currentClient = nullptr;
    m_tcpBuffer.clear();
    m_udpSubscriptionMask = SubNone; // 断开连接重置订阅
    m_reqIdMap.clear();
    emit logMessage(QStringLiteral("Info"),
                    QStringLiteral("远程控制客户端已断开%1")
                        .arg(peer.isEmpty() ? QString() : QStringLiteral("：") + peer));
}

void RemoteCommandServer::onTcpReadyRead() {
    if (!m_currentClient) return;
    
    m_tcpBuffer.append(m_currentClient->readAll());

    while (true) {
        if (m_tcpBuffer.size() < sizeof(TcpFrameHeader)) {
            return; // 等待更多数据
        }

        TcpFrameHeader header;
        memcpy(&header, m_tcpBuffer.data(), sizeof(TcpFrameHeader));
        
        // 标准做法是网络传输统一用 BigEndian
        header.magic = qFromBigEndian(header.magic);
        header.version = qFromBigEndian(header.version);
        header.error_code = qFromBigEndian(header.error_code);
        header.req_id = qFromBigEndian(header.req_id);
        header.data_length = qFromBigEndian(header.data_length); 
        
        if (header.magic != MAGIC_TCP) {
            qWarning() << "Invalid Magic Number. Closing connection.";
            m_currentClient->close();
            return;
        }
        if (header.version != PROTO_VERSION) {
            sendTcpResponse(header.req_id, 0xFFFE, QJsonObject{{"msg", "Unsupported protocol version"}});
            m_currentClient->close();
            return;
        }
        if (header.data_length > 16 * 1024 * 1024) {
            sendTcpResponse(header.req_id, 0xFFFD, QJsonObject{{"msg", "Request body is too large"}});
            m_currentClient->close();
            return;
        }

        // 检查 Body 是否完整
        int totalFrameSize = sizeof(TcpFrameHeader) + header.data_length;
        if (m_tcpBuffer.size() < totalFrameSize) {
            return; // 等待 Body数据
        }

        QByteArray bodyData = m_tcpBuffer.mid(sizeof(TcpFrameHeader), header.data_length);
        processTcpPacket(header, bodyData);

        m_tcpBuffer.remove(0, totalFrameSize);
    }
}

void RemoteCommandServer::processTcpPacket(const TcpFrameHeader &header, const QByteArray &body) {
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(body, &err);
    if (err.error != QJsonParseError::NoError) {
        sendTcpResponse(header.req_id, 0xFFFF, QJsonObject{{"msg", "JSON Parse Error"}});
        return;
    }

    if (!doc.isObject()) {
        sendTcpResponse(header.req_id, 0xFFFF, QJsonObject{{"msg", "JSON object required"}});
        return;
    }

    QJsonObject req = doc.object();
    QString cmd = req["command"].toString();
    uint32_t reqId = header.req_id;

    qDebug() << "CMD:" << cmd;
    emit logMessage(QStringLiteral("Info"),
                    QStringLiteral("收到远程命令：%1（请求 #%2）：%3")
                        .arg(cmd)
                        .arg(reqId)
                        .arg(QString::fromUtf8(QJsonDocument(req).toJson(QJsonDocument::Compact))));

    if (cmd == "START_SCAN") {
        QJsonObject params = req["params"].toObject();
        QJsonObject scan_range = params["scan_range"].toObject();
        const auto number = [&params, &scan_range](const char* rangeKey,
                                                    const char* paramsKey,
                                                    double fallback) {
            const QString rangeName = QString::fromLatin1(rangeKey);
            const QString paramsName = QString::fromLatin1(paramsKey);
            if (scan_range.contains(rangeName)) {
                return scan_range.value(rangeName).toDouble();
            }
            if (params.contains(paramsName)) {
                return params.value(paramsName).toDouble();
            }
            return fallback;
        };
        const double exposure_time = params.value("exposure_time")
            .toDouble(req.value("exposure_time").toDouble());
        const float startAngle = static_cast<float>(number("begin", "startAngle", 0.0));
        const float endAngle = static_cast<float>(number("end", "endAngle", 0.0));
        const float step = static_cast<float>(number("step", "step", 0.1));
        const float speed = static_cast<float>(number("speed", "speed",
            number("interval", "interval", 200.0)));
        std::cout << "RemoteCommandServer Recv START_SCAN: " << startAngle << " " << endAngle << " " << step << " " << speed << " exposure: " << exposure_time << std::endl;
        if (qFuzzyCompare(startAngle, endAngle) || step <= 0.0f || speed <= 0.0f) {
            sendTcpResponse(reqId, 2, QJsonObject{{"msg", "Invalid scan parameters"}});
            return;
        }
        rememberRequest(cmd, reqId);
        emit requestStartScan(startAngle, endAngle, step, speed, exposure_time);
    }
    else if (cmd == "STOP_SCAN") {
        rememberRequest(cmd, reqId);
        emit requestStopScan();
    }
    else if (cmd == "CHANGE_UDP") {
        const int subscriptionMask = req.value("type").toInt();
        const int udpPort = req.value("port").toInt(PORT_DATA_UDP);
        if (udpPort <= 0 || udpPort > 65535) {
            sendTcpResponse(reqId, 2, QJsonObject{{"msg", "Invalid UDP port"}});
            return;
        }
        m_udpSubscriptionMask = subscriptionMask;
        m_targetUdpPort = static_cast<quint16>(udpPort);
        sendTcpResponse(reqId, 1, QJsonObject{{"msg", "Subscription Updated"}});
        emit logMessage(QStringLiteral("Info"),
                        QStringLiteral("远程数据订阅已更新：%1，UDP端口：%2")
                            .arg(m_udpSubscriptionMask).arg(m_targetUdpPort));
    }
    else if (cmd == "LOG_EVENT") {
        const QString message = req.value("message").toString().trimmed();
        if (message.isEmpty()) {
            sendTcpResponse(reqId, 2, QJsonObject{{"msg", "Log message is empty"}});
            return;
        }
        emit logMessage(QStringLiteral("Info"),
                        QStringLiteral("A端操作：%1").arg(message));
        sendTcpResponse(reqId, 1, QJsonObject{{"msg", "Log event recorded"}});
    }
    else if (cmd == "OPEN_SCANNER") {
        // 这是一个耗时操作，通过信号通知 Main，Main 完成后回调 onScannerOpenedResult
        // 存储请求 ID 与指令的映射关系
        rememberRequest(cmd, reqId);
        emit requestOpenScanner(reqId);
    }
    else if (cmd == "CLOSE_SCANNER") {
        rememberRequest(cmd, reqId);
        emit requestCloseScanner(reqId);
    }
    else if (cmd == "CHANGE_PARAMETER") {
        rememberRequest(cmd, reqId);
        const QJsonObject params = req.value("params").toObject();
        emit requestChangeParameter(params.isEmpty() ? req : params, reqId);
    }
    else if( cmd == "OPEN_LASER"){
        rememberRequest(cmd, reqId);
        emit requestOpenLaser();
    }
    else if( cmd == "CLOSE_LASER"){
        rememberRequest(cmd, reqId);
        emit requestCloseLaser();
    }
    else if( cmd == "GO_TO_ANGLE"){
        rememberRequest(cmd, reqId);
        emit requestGoToAngle(req["params"].toObject()["angle"].toDouble());
    }
    else if( cmd == "SET_LIMIT"){
        rememberRequest(cmd, reqId);
        emit requestSetLimit(req["params"].toObject()["min_angle"].toDouble(), req["params"].toObject()["max_angle"].toDouble());
    }
    else if( cmd == "SET_EXPOSURE_TIME"){
        rememberRequest(cmd, reqId);
        emit requestSetExposureTime(req["params"].toObject()["exposure_time"].toDouble());
    }
    else if( cmd == "SAVE_CURRENT_IMAGE"){
        rememberRequest(cmd, reqId);
        emit requestSaveImage(req["params"].toObject()["filename"].toString());
    }
    else {
        sendTcpResponse(reqId, 0xFFFF, QJsonObject{{"msg", "Unknown command"}});
    }
    // ... 其他指令处理
}

void RemoteCommandServer::rememberRequest(const QString &command, uint32_t reqId) {
    m_reqIdMap[command].enqueue(reqId);
}

uint32_t RemoteCommandServer::takeRequestId(const QString &command) {
    auto it = m_reqIdMap.find(command);
    if (it == m_reqIdMap.end() || it->isEmpty()) {
        return 0;
    }
    return it->dequeue();
}

void RemoteCommandServer::sendTcpResponse(uint32_t reqId, uint16_t errorCode, const QJsonObject &body) {
    if (!m_currentClient || reqId == 0) return;

    QByteArray bodyData = QJsonDocument(body).toJson(QJsonDocument::Compact);
    
    TcpFrameHeader header;
    header.magic = MAGIC_TCP;
    header.version = PROTO_VERSION;
    header.error_code = errorCode; // 1 = Success
    header.req_id = reqId;
    header.data_length = bodyData.size();

    // 网络传输统一用 BigEndian
    header.magic = qToBigEndian(header.magic);
    header.version = qToBigEndian(header.version);
    header.error_code = qToBigEndian(header.error_code);
    header.req_id = qToBigEndian(header.req_id);
    header.data_length = qToBigEndian(header.data_length); 

    m_currentClient->write((const char*)&header, sizeof(header));
    m_currentClient->write(bodyData);
    m_currentClient->flush();
}

void RemoteCommandServer::sendUdpPacket(uint16_t dataType, const QByteArray &payload) {
    if (!m_udpSocket || m_targetUdpAddress.isNull()) return;

    // 留出 IP、UDP 和协议头空间，避免网络层再次分片。
    constexpr int kUdpPayloadSize = 1200;
    const int fragmentCount = std::max(
        1, (payload.size() + kUdpPayloadSize - 1) / kUdpPayloadSize);
    if (fragmentCount > std::numeric_limits<quint16>::max()) {
        emit logMessage(QStringLiteral("Error"), QStringLiteral("UDP数据过大，无法分片发送"));
        return;
    }

    quint32 transferId = m_nextTransferId++;
    if (transferId == 0) {
        transferId = m_nextTransferId++;
    }
    const quint32 timestamp = static_cast<quint32>(
        QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFFFULL);

    for (int index = 0; index < fragmentCount; ++index) {
        const QByteArray fragment = payload.mid(index * kUdpPayloadSize, kUdpPayloadSize);
        UdpFrameHeader header;
        header.magic = qToBigEndian(static_cast<uint32_t>(MAGIC_UDP));
        header.version = qToBigEndian(static_cast<uint16_t>(PROTO_VERSION));
        header.data_type = qToBigEndian(static_cast<uint16_t>(dataType));
        header.timestamp = qToBigEndian(timestamp);
        header.transfer_id = qToBigEndian(transferId);
        header.fragment_index = qToBigEndian(static_cast<uint16_t>(index));
        header.fragment_count = qToBigEndian(static_cast<uint16_t>(fragmentCount));

        QByteArray packet;
        packet.reserve(static_cast<int>(sizeof(header)) + fragment.size());
        packet.append(reinterpret_cast<const char*>(&header), sizeof(header));
        packet.append(fragment);
        m_udpSocket->writeDatagram(packet, m_targetUdpAddress, m_targetUdpPort);
    }
}

void RemoteCommandServer::onStatusTimerTimeout() {
    if (!(m_udpSubscriptionMask & SubStatus)) {
        return;
    }
    // 只有当客户端订阅了 Status (type & 1) 时才发送
    // if (m_udpSubscriptionMask & SubStatus) {

    // } 
    // m_lastStatus 由 MainWindow 通过 updateDeviceStatus 定期更新
    QByteArray json = QJsonDocument(m_lastStatus).toJson(QJsonDocument::Compact);
    sendUdpPacket(UdpTypeStatus, json);
}

void RemoteCommandServer::updateDeviceStatus(const QJsonObject &statusJson) {
    // Json 赋值是隐式共享的，copy on write，赋值操作是线程安全的
    m_lastStatus = statusJson;
}

void RemoteCommandServer::sendScannerInfo(const scanner::ScannerInfo &scannerInfo) {
    QJsonObject json;
    json["current_angle"] = scannerInfo.angle;
    json["temperature"] = scannerInfo.temperature;
    json["humidity"] = scannerInfo.humidity;
    json["pressure"] = scannerInfo.pressure;
    json["attitude_flag"] = scannerInfo.attitudeFlag;
    json["roll"] = scannerInfo.roll;
    json["pitch"] = scannerInfo.pitch;
    json["yaw"] = scannerInfo.yaw;
    QJsonArray reserved;
    for (const char byte : scannerInfo.reserved) {
        reserved.append(static_cast<unsigned char>(byte));
    }
    json["reserved"] = reserved;
    json["angle_max"] = scannerInfo.angle_max;
    json["angle_min"] = scannerInfo.angle_min;
    json["exposure_time"] = scannerInfo.exposureTime;
    json["isOpen"] = scannerInfo.isOpen;
    
    m_lastStatus = json;
    
}

void RemoteCommandServer::sendImage(const cv::Mat &image) {
    if (!(m_udpSubscriptionMask & SubImage)) {
        return;  // 客户端未订阅图片
    }
    if (image.empty()) return;
 
    cv::Mat smallMat;
    double scale = 800.0 / image.cols;
    cv::resize(image, smallMat, cv::Size(), scale, scale, cv::INTER_LINEAR);

    cv::cvtColor(smallMat, smallMat, cv::COLOR_BGR2RGB);

    // 构造 QImage，数据直接指向 smallMat.data，没有深拷贝
    QImage imgWrapper((const uchar*)smallMat.data, 
                       smallMat.cols, smallMat.rows, 
                       smallMat.step, 
                       QImage::Format_RGB888);

    QByteArray packetBytes;
    QBuffer buffer(&packetBytes);
    buffer.open(QIODevice::WriteOnly);
    
    imgWrapper.save(&buffer, "JPG", 60);

    sendUdpPacket(UdpTypeImage, packetBytes);    
}

void RemoteCommandServer::sendPointCloud(pointcloud::PointCloudPtr cloud) {
    if(!cloud || cloud->empty()) return;
    if (!(m_udpSubscriptionMask & SubPointCloud)) return;
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    
    // 设置为小端序 
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision); // float (4 bytes)

    int pointCount = 0;
    std::vector<Eigen::Vector3f> points;
    cloud->getPoints(points);

    for (const auto& pt : points) {
        if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2])) {
            continue;
        }

        stream << pt[0] << pt[1] << pt[2];
        pointCount++;

    }

    if (payload.isEmpty()) return;

    // 发送 (Type = 3)
    sendUdpPacket(UdpTypePointCloud, payload);
}

void RemoteCommandServer::onScannerOpenedResult(bool success, const QString &msg) {
    const uint32_t reqId = takeRequestId("OPEN_SCANNER");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程开启扫描仪：%1").arg(msg));
    int errorCode = success ? 1 : 0;
    sendTcpResponse(reqId, errorCode, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onScannerClosedResult(bool success, const QString &msg){
    const uint32_t reqId = takeRequestId("CLOSE_SCANNER");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程关闭扫描仪：%1").arg(msg));
    int code = success ? 1 : 0;
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onStartScanResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("START_SCAN");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程开始扫描：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onStopScanResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("STOP_SCAN");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程停止扫描：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onParameterChangedResult(bool success, const QString &msg) {
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("CHANGE_PARAMETER");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程设置参数：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onOpenLaserResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("OPEN_LASER");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程开启激光：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onCloseLaserResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("CLOSE_LASER");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程关闭激光：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onGoToAngleResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("GO_TO_ANGLE");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程转到角度：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onSetLimitResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("SET_LIMIT");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程设置扫描范围：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onSetExposureTimeResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("SET_EXPOSURE_TIME");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程设置曝光时间：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}

void RemoteCommandServer::onSaveImageResult(bool success, const QString &msg){
    uint16_t code = success ? 1 : 0;
    const uint32_t reqId = takeRequestId("SAVE_CURRENT_IMAGE");
    emit logMessage(success ? QStringLiteral("Info") : QStringLiteral("Error"),
                    QStringLiteral("远程保存图像：%1").arg(msg));
    sendTcpResponse(reqId, code, QJsonObject{{"msg", msg}});
}
}} // namespace server
