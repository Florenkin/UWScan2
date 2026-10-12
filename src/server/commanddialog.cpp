#include "server/commanddialog.h"

#include "server/protocal.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDataStream>
#include <QDateTime>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHostAddress>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QVBoxLayout>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QStandardPaths>
#include <QSettings>
#include <QTextStream>

#include <cmath>

NAMESPACE_BEGIN { namespace server {
namespace {
    constexpr int kTcpHeaderSize = 16;
    constexpr int kUdpHeaderSize = 20;
    constexpr quint32 kMaxTcpBodySize = 16 * 1024 * 1024;
    constexpr quint32 kMagicTcp = 0x55575343;
    constexpr quint32 kMagicUdp = 0x55575344;
    constexpr quint16 kProtocolVersion = 0x0001;

    QLineEdit* makeNumberEdit(QWidget* parent, const QString& value) {
        auto* edit = new QLineEdit(value, parent);
        edit->setMinimumWidth(72);
        return edit;
    }
}

RemoteCommandDialog::RemoteCommandDialog(QWidget* parent)
    : QDialog(parent),
      tcpSocket_(new QTcpSocket(this)),
      udpSocket_(new QUdpSocket(this)) {
    setWindowTitle(QStringLiteral("点云扫描"));
    setModal(false);
    resize(410, 668);
    setMinimumWidth(390);
    buildUi();

    connect(tcpSocket_, &QTcpSocket::connected, this, &RemoteCommandDialog::onTcpConnected);
    connect(tcpSocket_, &QTcpSocket::disconnected, this, &RemoteCommandDialog::onTcpDisconnected);
    connect(tcpSocket_, &QTcpSocket::readyRead, this, &RemoteCommandDialog::onTcpReadyRead);
    connect(tcpSocket_, &QTcpSocket::errorOccurred, this, &RemoteCommandDialog::onSocketError);
    connect(udpSocket_, &QUdpSocket::readyRead, this, &RemoteCommandDialog::onUdpReadyRead);
}

RemoteCommandDialog::~RemoteCommandDialog() {
    if (tcpSocket_->state() != QAbstractSocket::UnconnectedState) {
        tcpSocket_->disconnectFromHost();
    }
    udpSocket_->close();
}

void RemoteCommandDialog::buildUi() {
    setStyleSheet(QStringLiteral(
        "QDialog { background: #f5f5f5; }"
        "QGroupBox { border: 1px solid #b9b9b9; border-radius: 4px; margin-top: 9px; padding-top: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #444; }"
        "QPushButton { background: #3498c9; color: white; border: none; border-radius: 4px; padding: 6px 10px; min-height: 20px; }"
        "QPushButton:hover { background: #2584b3; }"
        "QPushButton:disabled { background: #b8c5cb; }"
        "QLineEdit, QComboBox { background: white; border: 1px solid #b5b5b5; border-radius: 3px; padding: 4px; min-height: 22px; }"
        "QPlainTextEdit { background: white; border: 1px solid #c5c5c5; }"
    ));

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 8, 10, 10);
    root->setSpacing(7);

    auto* scannerGroup = new QGroupBox(QStringLiteral("扫描仪"), this);
    auto* scannerLayout = new QGridLayout(scannerGroup);
    scannerLayout->setContentsMargins(10, 12, 10, 10);
    scannerLayout->setHorizontalSpacing(6);
    scannerLayout->setVerticalSpacing(6);

    QSettings settings(QStringLiteral("UWScan2"), QStringLiteral("RemoteControl"));
    hostEdit_ = new QLineEdit(settings.value(QStringLiteral("host"), QStringLiteral("192.168.0.17")).toString(), scannerGroup);
    tcpPortEdit_ = new QLineEdit(settings.value(QStringLiteral("tcpPort"), QStringLiteral("2356")).toString(), scannerGroup);
    udpPortEdit_ = new QLineEdit(settings.value(QStringLiteral("udpPort"), QStringLiteral("2355")).toString(), scannerGroup);
    scannerLayout->addWidget(new QLabel(QStringLiteral("IP"), scannerGroup), 0, 0);
    scannerLayout->addWidget(hostEdit_, 0, 1, 1, 5);
    scannerLayout->addWidget(new QLabel(QStringLiteral("TCP端口"), scannerGroup), 1, 0);
    scannerLayout->addWidget(tcpPortEdit_, 1, 1, 1, 2);
    scannerLayout->addWidget(new QLabel(QStringLiteral("UDP端口"), scannerGroup), 1, 3);
    scannerLayout->addWidget(udpPortEdit_, 1, 4, 1, 2);

    connectButton_ = new QPushButton(QStringLiteral("连接"), scannerGroup);
    connectionStateLabel_ = new QLabel(QStringLiteral("连接状态 ●"), scannerGroup);
    setConnectionState(QStringLiteral("连接状态 ●"), false);
    scannerLayout->addWidget(connectButton_, 2, 0, 1, 2);
    scannerLayout->addWidget(connectionStateLabel_, 2, 2, 1, 4);
    connect(connectButton_, &QPushButton::clicked, this, &RemoteCommandDialog::onConnectButtonClicked);

    auto addCommandButton = [this, scannerGroup, scannerLayout](const QString& text, int row, int column, auto callback) {
        auto* button = new QPushButton(text, scannerGroup);
        scannerLayout->addWidget(button, row, column);
        connect(button, &QPushButton::clicked, this, callback);
        return button;
    };
    addCommandButton(QStringLiteral("开启扫描仪"), 3, 0, [this]() {
        sendCommand(QJsonObject{{QStringLiteral("command"), QStringLiteral("OPEN_SCANNER")}});
    });
    addCommandButton(QStringLiteral("关闭扫描仪"), 3, 1, [this]() {
        sendCommand(QJsonObject{{QStringLiteral("command"), QStringLiteral("CLOSE_SCANNER")}});
    });
    addCommandButton(QStringLiteral("扫描开始"), 3, 2, [this]() {
        bool ok = false;
        const double begin = scanBeginEdit_->text().toDouble(&ok);
        const bool beginOk = ok;
        const double end = scanEndEdit_->text().toDouble(&ok);
        const bool endOk = ok;
        const double step = scanStepEdit_->text().toDouble(&ok);
        const bool stepOk = ok;
        const double interval = scanIntervalEdit_->text().toDouble(&ok);
        const bool intervalOk = ok;
        const double delay = scanDelayEdit_->text().toDouble(&ok);
        const bool delayOk = ok;
        const double exposure = scanExposureEdit_->text().toDouble(&ok);
        const bool exposureOk = ok;
        if (!beginOk || !endOk || !stepOk || !intervalOk || !delayOk || !exposureOk
            || step <= 0.0 || interval <= 0.0 || delay < 0.0 || exposure <= 0.0
            || qFuzzyCompare(begin, end)) {
            QMessageBox::warning(this, QStringLiteral("参数错误"), QStringLiteral("请检查扫描角度、步长和曝光时间。"));
            return;
        }
        sendCommand(QJsonObject{
            {QStringLiteral("command"), QStringLiteral("START_SCAN")},
            {QStringLiteral("params"), QJsonObject{
                {QStringLiteral("scan_range"), QJsonObject{
                    {QStringLiteral("begin"), begin},
                    {QStringLiteral("end"), end},
                    {QStringLiteral("step"), step},
                    {QStringLiteral("speed"), interval},
                    {QStringLiteral("interval"), interval},
                    {QStringLiteral("delay"), delay}
                }},
                {QStringLiteral("exposure_time"), exposure}
            }}
        });
    });
    addCommandButton(QStringLiteral("扫描停止"), 3, 3, [this]() {
        sendCommand(QJsonObject{{QStringLiteral("command"), QStringLiteral("STOP_SCAN")}});
    });

    showCloudCheck_ = new QCheckBox(QStringLiteral("显示缓存点云"), scannerGroup);
    scannerLayout->addWidget(showCloudCheck_, 4, 0, 1, 2);
    connect(showCloudCheck_, &QCheckBox::toggled, this, [this](bool checked) {
        sendLogEvent(QStringLiteral("显示缓存点云：%1")
                         .arg(checked ? QStringLiteral("开启") : QStringLiteral("关闭")));
    });
    addCommandButton(QStringLiteral("清空缓存点云"), 4, 2, [this]() { clearPointCloud(); });
    addCommandButton(QStringLiteral("打开激光"), 4, 3, [this]() {
        sendCommand(QJsonObject{{QStringLiteral("command"), QStringLiteral("OPEN_LASER")}});
    });
    addCommandButton(QStringLiteral("关闭激光"), 4, 4, [this]() {
        sendCommand(QJsonObject{{QStringLiteral("command"), QStringLiteral("CLOSE_LASER")}});
    });
    root->addWidget(scannerGroup);

    auto* tabs = new QTabWidget(this);

    auto* statusTab = new QWidget(tabs);
    auto* statusLayout = new QHBoxLayout(statusTab);
    statusLayout->setContentsMargins(8, 8, 8, 8);
    statusLayout->setSpacing(8);

    auto* statusGroup = new QGroupBox(QStringLiteral("扫描仪状态"), statusTab);
    auto* statusForm = new QFormLayout(statusGroup);
    statusAngleEdit_ = makeNumberEdit(statusGroup, QStringLiteral("0.00"));
    statusTemperatureEdit_ = makeNumberEdit(statusGroup, QStringLiteral("--"));
    statusHumidityEdit_ = makeNumberEdit(statusGroup, QStringLiteral("--"));
    statusPressureEdit_ = makeNumberEdit(statusGroup, QStringLiteral("--"));
    statusExposureEdit_ = makeNumberEdit(statusGroup, QStringLiteral("--"));
    for (QLineEdit* edit : {statusAngleEdit_, statusTemperatureEdit_, statusHumidityEdit_,
                            statusPressureEdit_, statusExposureEdit_}) {
        edit->setReadOnly(true);
    }
    statusForm->addRow(QStringLiteral("电机角度"), statusAngleEdit_);
    statusForm->addRow(QStringLiteral("温度"), statusTemperatureEdit_);
    statusForm->addRow(QStringLiteral("湿度"), statusHumidityEdit_);
    statusForm->addRow(QStringLiteral("压力"), statusPressureEdit_);
    statusForm->addRow(QStringLiteral("曝光时间"), statusExposureEdit_);
    scanStateLabel_ = new QLabel(QStringLiteral("扫描仪状态 ● 未知"), statusGroup);
    scanStateLabel_->setStyleSheet(QStringLiteral("color: #e08a00;"));
    statusForm->addRow(QStringLiteral("扫描仪状态"), scanStateLabel_);

    auto* scanGroup = new QGroupBox(QStringLiteral("开始扫描参数"), statusTab);
    auto* scanForm = new QFormLayout(scanGroup);
    scanBeginEdit_ = makeNumberEdit(scanGroup, QStringLiteral("20.00"));
    scanEndEdit_ = makeNumberEdit(scanGroup, QStringLiteral("24.00"));
    scanStepEdit_ = makeNumberEdit(scanGroup, QStringLiteral("0.01"));
    scanIntervalEdit_ = makeNumberEdit(scanGroup, QStringLiteral("200"));
    scanDelayEdit_ = makeNumberEdit(scanGroup, QStringLiteral("0"));
    scanExposureEdit_ = makeNumberEdit(scanGroup, QStringLiteral("1000"));
    scanForm->addRow(QStringLiteral("开始角度"), scanBeginEdit_);
    scanForm->addRow(QStringLiteral("结束角度"), scanEndEdit_);
    scanForm->addRow(QStringLiteral("电机步长"), scanStepEdit_);
    scanForm->addRow(QStringLiteral("扫描间隔(ms)"), scanIntervalEdit_);
    scanForm->addRow(QStringLiteral("延迟触发(ms)"), scanDelayEdit_);
    scanForm->addRow(QStringLiteral("曝光时间(us)"), scanExposureEdit_);
    auto* startScanButton = new QPushButton(QStringLiteral("开始扫描"), scanGroup);
    scanForm->addRow(QString(), startScanButton);
    connect(startScanButton, &QPushButton::clicked, this, [this]() {
        bool ok = false;
        const double begin = scanBeginEdit_->text().toDouble(&ok);
        const bool beginOk = ok;
        const double end = scanEndEdit_->text().toDouble(&ok);
        const bool endOk = ok;
        const double step = scanStepEdit_->text().toDouble(&ok);
        const bool stepOk = ok;
        const double interval = scanIntervalEdit_->text().toDouble(&ok);
        const bool intervalOk = ok;
        const double delay = scanDelayEdit_->text().toDouble(&ok);
        const bool delayOk = ok;
        const double exposure = scanExposureEdit_->text().toDouble(&ok);
        const bool exposureOk = ok;
        if (!beginOk || !endOk || !stepOk || !intervalOk || !delayOk || !exposureOk
            || step <= 0.0 || interval <= 0.0 || delay < 0.0 || exposure <= 0.0
            || qFuzzyCompare(begin, end)) {
            QMessageBox::warning(this, QStringLiteral("参数错误"), QStringLiteral("请检查扫描角度、步长、间隔、延迟和曝光时间。"));
            return;
        }
        sendCommand(QJsonObject{
            {QStringLiteral("command"), QStringLiteral("START_SCAN")},
            {QStringLiteral("params"), QJsonObject{
                {QStringLiteral("scan_range"), QJsonObject{
                    {QStringLiteral("begin"), begin},
                    {QStringLiteral("end"), end},
                    {QStringLiteral("step"), step},
                    {QStringLiteral("speed"), interval},
                    {QStringLiteral("interval"), interval},
                    {QStringLiteral("delay"), delay}
                }},
                {QStringLiteral("exposure_time"), exposure}
            }}
        });
    });
    statusLayout->addWidget(statusGroup);
    statusLayout->addWidget(scanGroup);
    tabs->addTab(statusTab, QStringLiteral("参数1"));

    auto* settingsTab = new QWidget(tabs);
    auto* settingsLayout = new QHBoxLayout(settingsTab);
    settingsLayout->setContentsMargins(8, 8, 8, 8);
    settingsLayout->setSpacing(8);

    auto* cameraGroup = new QGroupBox(QStringLiteral("扫描仪参数设置"), settingsTab);
    auto* cameraForm = new QFormLayout(cameraGroup);
    exposureEdit_ = makeNumberEdit(cameraGroup, QStringLiteral("1000"));
    gainEdit_ = makeNumberEdit(cameraGroup, QStringLiteral("0"));
    currentAngleEdit_ = makeNumberEdit(cameraGroup, QStringLiteral("0.00"));
    triggerModeCombo_ = new QComboBox(cameraGroup);
    triggerModeCombo_->addItems({QStringLiteral("ON"), QStringLiteral("OFF")});
    triggerSourceCombo_ = new QComboBox(cameraGroup);
    triggerSourceCombo_->addItems({QStringLiteral("SW"), QStringLiteral("L1"), QStringLiteral("L2"), QStringLiteral("L3")});
    cameraForm->addRow(QStringLiteral("曝光时间(us)"), exposureEdit_);
    cameraForm->addRow(QStringLiteral("增益"), gainEdit_);
    cameraForm->addRow(QStringLiteral("电机角度"), currentAngleEdit_);
    cameraForm->addRow(QStringLiteral("触发模式"), triggerModeCombo_);
    cameraForm->addRow(QStringLiteral("触发源"), triggerSourceCombo_);
    auto* applyParametersButton = new QPushButton(QStringLiteral("设置"), cameraGroup);
    cameraForm->addRow(QString(), applyParametersButton);
    connect(applyParametersButton, &QPushButton::clicked, this, [this]() {
        QJsonObject params;
        bool ok = false;
        const double exposure = exposureEdit_->text().toDouble(&ok);
        if (ok) params.insert(QStringLiteral("exposure_time"), exposure);
        const double gain = gainEdit_->text().toDouble(&ok);
        if (ok) params.insert(QStringLiteral("gain"), gain);
        const double angle = currentAngleEdit_->text().toDouble(&ok);
        if (ok) params.insert(QStringLiteral("current_angle"), angle);
        params.insert(QStringLiteral("trigger_mode"), triggerModeCombo_->currentText());
        params.insert(QStringLiteral("trigger_source"), triggerSourceCombo_->currentText());
        sendCommand(QJsonObject{
            {QStringLiteral("command"), QStringLiteral("CHANGE_PARAMETER")},
            {QStringLiteral("params"), params}
        });
    });

    auto* subscriptionGroup = new QGroupBox(QStringLiteral("扫描仪发送参数"), settingsTab);
    auto* subscriptionForm = new QFormLayout(subscriptionGroup);
    statusSubscription_ = new QCheckBox(QStringLiteral("实时状态"), subscriptionGroup);
    imageSubscription_ = new QCheckBox(QStringLiteral("实时图片"), subscriptionGroup);
    cloudSubscription_ = new QCheckBox(QStringLiteral("点云"), subscriptionGroup);
    subscriptionForm->addRow(statusSubscription_);
    subscriptionForm->addRow(imageSubscription_);
    subscriptionForm->addRow(cloudSubscription_);
    settingsLayout->addWidget(cameraGroup);
    settingsLayout->addWidget(subscriptionGroup);
    tabs->addTab(settingsTab, QStringLiteral("参数2"));
    connect(statusSubscription_, &QCheckBox::toggled, this, &RemoteCommandDialog::onSubscriptionChanged);
    connect(imageSubscription_, &QCheckBox::toggled, this, &RemoteCommandDialog::onSubscriptionChanged);
    connect(cloudSubscription_, &QCheckBox::toggled, this, &RemoteCommandDialog::onSubscriptionChanged);
    root->addWidget(tabs);

    auto* previewGroup = new QGroupBox(QStringLiteral("数据接收"), this);
    auto* previewLayout = new QVBoxLayout(previewGroup);
    previewLayout->setContentsMargins(8, 12, 8, 8);
    imagePreviewLabel_ = new QLabel(QStringLiteral("等待实时图片..."), previewGroup);
    imagePreviewLabel_->setAlignment(Qt::AlignCenter);
    imagePreviewLabel_->setMinimumHeight(80);
    imagePreviewLabel_->setStyleSheet(QStringLiteral("background: #e9eef1; color: #667;"));
    cloudStateLabel_ = new QLabel(QStringLiteral("缓存点云: 0"), previewGroup);
    statusLabel_ = new QLabel(QStringLiteral("就绪"), previewGroup);
    previewLayout->addWidget(imagePreviewLabel_);
    previewLayout->addWidget(cloudStateLabel_);
    previewLayout->addWidget(statusLabel_);
    previewGroup->setVisible(false);

    auto* bottomLayout = new QGridLayout();
    auto* saveImageButton = new QPushButton(QStringLiteral("获取图像至本地"), this);
    auto* saveCalibrationImageButton = new QPushButton(QStringLiteral("添加图像至标定数据集"), this);
    auto* saveCloudButton = new QPushButton(QStringLiteral("添加点云至文件树"), this);
    auto* saveCalibrationCloudButton = new QPushButton(QStringLiteral("添加点云至标定数据集"), this);
    bottomLayout->addWidget(saveImageButton, 0, 0);
    bottomLayout->addWidget(saveCalibrationImageButton, 0, 1);
    bottomLayout->addWidget(saveCloudButton, 1, 0);
    bottomLayout->addWidget(saveCalibrationCloudButton, 1, 1);
    connect(saveImageButton, &QPushButton::clicked, this, &RemoteCommandDialog::saveCurrentImage);
    connect(saveCalibrationImageButton, &QPushButton::clicked, this, &RemoteCommandDialog::saveCurrentImageForCalibration);
    connect(saveCloudButton, &QPushButton::clicked, this, &RemoteCommandDialog::saveCurrentPointCloud);
    connect(saveCalibrationCloudButton, &QPushButton::clicked, this, &RemoteCommandDialog::saveCurrentPointCloudForCalibration);
    root->addLayout(bottomLayout);

    logEdit_ = new QPlainTextEdit(this);
    logEdit_->setReadOnly(true);
    logEdit_->setMaximumHeight(75);
    logEdit_->setVisible(false);
    root->addWidget(logEdit_);
}

void RemoteCommandDialog::onConnectButtonClicked() {
    if (tcpSocket_->state() != QAbstractSocket::UnconnectedState) {
        tcpSocket_->disconnectFromHost();
        return;
    }

    bool tcpOk = false;
    bool udpOk = false;
    const quint16 tcpPort = tcpPortEdit_->text().toUShort(&tcpOk);
    const quint16 udpPort = udpPortEdit_->text().toUShort(&udpOk);
    if (!tcpOk || !udpOk || tcpPort == 0 || udpPort == 0) {
        setConnectionState(QStringLiteral("端口无效"), false);
        return;
    }

    udpSocket_->close();
    udpAssemblies_.clear();
    if (!udpSocket_->bind(QHostAddress::AnyIPv4, udpPort,
                          QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        setConnectionState(QStringLiteral("UDP绑定失败"), false);
        appendLog(QStringLiteral("UDP: %1").arg(udpSocket_->errorString()));
        return;
    }

    connectButton_->setEnabled(false);
    setConnectionState(QStringLiteral("连接中 ●"), false);
    tcpSocket_->connectToHost(hostEdit_->text().trimmed(), tcpPort);
    QSettings settings(QStringLiteral("UWScan2"), QStringLiteral("RemoteControl"));
    settings.setValue(QStringLiteral("host"), hostEdit_->text().trimmed());
    settings.setValue(QStringLiteral("tcpPort"), tcpPort);
    settings.setValue(QStringLiteral("udpPort"), udpPort);
}

void RemoteCommandDialog::onTcpConnected() {
    connectButton_->setEnabled(true);
    connectButton_->setText(QStringLiteral("断开"));
    setConnectionState(QStringLiteral("已连接 ●"), true);
    appendLog(QStringLiteral("TCP 已连接到 %1:%2").arg(hostEdit_->text(), tcpPortEdit_->text()));
    sendSubscription();
}

void RemoteCommandDialog::onTcpDisconnected() {
    connectButton_->setEnabled(true);
    connectButton_->setText(QStringLiteral("连接"));
    setConnectionState(QStringLiteral("连接状态 ●"), false);
    udpAssemblies_.clear();
    appendLog(QStringLiteral("TCP 连接已断开"));
}

void RemoteCommandDialog::onSocketError() {
    connectButton_->setEnabled(true);
    connectButton_->setText(QStringLiteral("连接"));
    setConnectionState(QStringLiteral("连接失败 ●"), false);
    appendLog(QStringLiteral("TCP: %1").arg(tcpSocket_->errorString()));
}

void RemoteCommandDialog::sendCommand(const QJsonObject& command) {
    if (tcpSocket_->state() != QAbstractSocket::ConnectedState) {
        setConnectionState(QStringLiteral("请先连接服务"), false);
        return;
    }

    const QByteArray body = QJsonDocument(command).toJson(QJsonDocument::Compact);
    QByteArray frame;
    QDataStream stream(&frame, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);
    stream << quint32(kMagicTcp)
           << quint16(kProtocolVersion)
           << quint16(0)
           << quint32(nextRequestId_++)
           << quint32(body.size());
    frame.append(body);
    tcpSocket_->write(frame);
    tcpSocket_->flush();
    appendLog(QStringLiteral("发送: %1").arg(command.value(QStringLiteral("command")).toString()));
}

void RemoteCommandDialog::sendLogEvent(const QString& message) {
    sendCommand(QJsonObject{
        {QStringLiteral("command"), QStringLiteral("LOG_EVENT")},
        {QStringLiteral("message"), message}
    });
}

void RemoteCommandDialog::sendSubscription() {
    const int mask = (statusSubscription_->isChecked() ? SubStatus : 0)
                   | (imageSubscription_->isChecked() ? SubImage : 0)
                   | (cloudSubscription_->isChecked() ? SubPointCloud : 0);
    sendCommand(QJsonObject{
        {QStringLiteral("command"), QStringLiteral("CHANGE_UDP")},
        {QStringLiteral("type"), mask},
        {QStringLiteral("port"), udpPortEdit_->text().toUShort()}
    });
}

void RemoteCommandDialog::onSubscriptionChanged() {
    if (tcpSocket_->state() == QAbstractSocket::ConnectedState) {
        sendSubscription();
    }
}

void RemoteCommandDialog::onTcpReadyRead() {
    tcpBuffer_.append(tcpSocket_->readAll());
    while (tcpBuffer_.size() >= kTcpHeaderSize) {
        QDataStream headerStream(tcpBuffer_.left(kTcpHeaderSize));
        headerStream.setByteOrder(QDataStream::BigEndian);
        quint32 magic = 0;
        quint16 version = 0;
        quint16 errorCode = 0;
        quint32 requestId = 0;
        quint32 bodyLength = 0;
        headerStream >> magic >> version >> errorCode >> requestId >> bodyLength;
        if (magic != kMagicTcp || version != kProtocolVersion || bodyLength > kMaxTcpBodySize) {
            appendLog(QStringLiteral("收到无效 TCP 响应头"));
            tcpSocket_->disconnectFromHost();
            return;
        }
        if (tcpBuffer_.size() < kTcpHeaderSize + static_cast<int>(bodyLength)) {
            return;
        }
        const QByteArray frame = tcpBuffer_.left(kTcpHeaderSize + static_cast<int>(bodyLength));
        tcpBuffer_.remove(0, frame.size());
        handleTcpResponse(frame);
    }
}

void RemoteCommandDialog::handleTcpResponse(const QByteArray& frame) {
    QDataStream headerStream(frame.left(kTcpHeaderSize));
    headerStream.setByteOrder(QDataStream::BigEndian);
    quint32 magic = 0;
    quint16 version = 0;
    quint16 errorCode = 0;
    quint32 requestId = 0;
    quint32 bodyLength = 0;
    headerStream >> magic >> version >> errorCode >> requestId >> bodyLength;
    const QByteArray body = frame.mid(kTcpHeaderSize, static_cast<int>(bodyLength));
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(body, &parseError);
    const QString result = parseError.error == QJsonParseError::NoError
        ? QString::fromUtf8(document.toJson(QJsonDocument::Compact))
        : QString::fromUtf8(body);
    appendLog(QStringLiteral("响应 #%1 [%2]: %3")
                  .arg(requestId)
                  .arg(errorCode == 1 ? QStringLiteral("成功") : QStringLiteral("失败 %1").arg(errorCode))
                  .arg(result));
    statusLabel_->setText(result);
}

void RemoteCommandDialog::onUdpReadyRead() {
    while (udpSocket_->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(udpSocket_->pendingDatagramSize()));
        udpSocket_->readDatagram(datagram.data(), datagram.size());
        handleUdpDatagram(datagram);
    }
}

void RemoteCommandDialog::handleUdpDatagram(const QByteArray& datagram) {
    if (datagram.size() < kUdpHeaderSize) {
        return;
    }
    QDataStream headerStream(datagram.left(kUdpHeaderSize));
    headerStream.setByteOrder(QDataStream::BigEndian);
    quint32 magic = 0;
    quint16 version = 0;
    quint16 dataType = 0;
    quint32 timestamp = 0;
    quint32 transferId = 0;
    quint16 fragmentIndex = 0;
    quint16 fragmentCount = 0;
    headerStream >> magic >> version >> dataType >> timestamp;
    headerStream >> transferId >> fragmentIndex >> fragmentCount;
    if (magic != kMagicUdp || version != kProtocolVersion) {
        return;
    }
    if (fragmentCount == 0 || fragmentIndex >= fragmentCount) {
        return;
    }

    QByteArray payload = datagram.mid(kUdpHeaderSize);
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (auto it = udpAssemblies_.begin(); it != udpAssemblies_.end();) {
        if (now - it.value().lastSeenMs > 5000) {
            it = udpAssemblies_.erase(it);
        } else {
            ++it;
        }
    }

    if (fragmentCount > 1) {
        UdpAssembly& assembly = udpAssemblies_[transferId];
        if (assembly.fragmentCount != fragmentCount || assembly.dataType != dataType) {
            assembly = UdpAssembly{};
            assembly.fragmentCount = fragmentCount;
            assembly.dataType = dataType;
            assembly.fragments.resize(fragmentCount);
            assembly.received.fill(false, fragmentCount);
        }
        assembly.lastSeenMs = now;
        if (!assembly.received[fragmentIndex]) {
            assembly.fragments[fragmentIndex] = payload;
            assembly.received[fragmentIndex] = true;
            ++assembly.receivedCount;
        }
        if (assembly.receivedCount != assembly.fragmentCount) {
            return;
        }
        payload.clear();
        for (const QByteArray& fragment : assembly.fragments) {
            payload.append(fragment);
        }
        udpAssemblies_.remove(transferId);
    }

    Q_UNUSED(timestamp);
    if (dataType == UdpTypeStatus) {
        QJsonParseError error;
        const QJsonDocument document = QJsonDocument::fromJson(payload, &error);
        if (error.error == QJsonParseError::NoError && document.isObject()) {
            const QJsonObject object = document.object();
            const double angle = object.value(QStringLiteral("current_angle")).toDouble();
            const double temperature = object.value(QStringLiteral("temperature")).toDouble();
            const double humidity = object.value(QStringLiteral("humidity")).toDouble();
            const double pressure = object.value(QStringLiteral("pressure")).toDouble();
            const double exposure = object.value(QStringLiteral("exposure_time")).toDouble();
            statusLabel_->setText(QStringLiteral("角度 %1°  温度 %2°C  湿度 %3%")
                                  .arg(angle, 0, 'f', 2)
                                  .arg(temperature, 0, 'f', 1)
                                  .arg(humidity, 0, 'f', 1));
            statusAngleEdit_->setText(QString::number(angle, 'f', 2));
            statusTemperatureEdit_->setText(QString::number(temperature, 'f', 1));
            statusHumidityEdit_->setText(QString::number(humidity, 'f', 1));
            statusPressureEdit_->setText(QString::number(pressure, 'f', 1));
            statusExposureEdit_->setText(QString::number(exposure, 'f', 1));
            currentAngleEdit_->setText(QString::number(angle, 'f', 2));
            const bool isOpen = object.value(QStringLiteral("isOpen")).toBool();
            scanStateLabel_->setText(isOpen
                ? QStringLiteral("扫描仪状态 ● 已连接")
                : QStringLiteral("扫描仪状态 ● 未连接"));
            scanStateLabel_->setStyleSheet(isOpen
                ? QStringLiteral("color: #1a9b55;")
                : QStringLiteral("color: #e33232;"));
        }
        return;
    }
    if (dataType == UdpTypeImage) {
        QImage image;
        if (image.loadFromData(payload, "JPG")) {
            latestImage_ = image;
            imagePreviewLabel_->setPixmap(QPixmap::fromImage(image).scaled(
                imagePreviewLabel_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            appendLog(QStringLiteral("收到图片: %1 x %2").arg(image.width()).arg(image.height()));
        }
        return;
    }
    if (dataType == UdpTypePointCloud) {
        if (payload.size() % (sizeof(float) * 3) != 0) {
            return;
        }
        QDataStream pointStream(payload);
        pointStream.setByteOrder(QDataStream::LittleEndian);
        pointStream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        latestPointCloud_.clear();
        while (!pointStream.atEnd()) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            pointStream >> x >> y >> z;
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                latestPointCloud_.append(QVector3D(x, y, z));
            }
        }
        cloudStateLabel_->setText(QStringLiteral("缓存点云: %1").arg(latestPointCloud_.size()));
        appendLog(QStringLiteral("收到点云: %1 个点").arg(latestPointCloud_.size()));
    }
}

void RemoteCommandDialog::clearPointCloud() {
    latestPointCloud_.clear();
    cloudStateLabel_->setText(QStringLiteral("缓存点云: 0"));
    appendLog(QStringLiteral("已清空缓存点云"));
    sendLogEvent(QStringLiteral("清空缓存点云"));
}

void RemoteCommandDialog::saveCurrentImage() {
    sendLogEvent(QStringLiteral("点击获取图像至本地"));
    if (latestImage_.isNull()) {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("当前没有可保存的图片。"));
        sendLogEvent(QStringLiteral("获取图像至本地失败：当前没有图片"));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("保存图片"), nextSavePath(QStringLiteral(".bmp")),
        QStringLiteral("8 位 BMP 图片 (*.bmp);;JPEG 图片 (*.jpg *.jpeg);;PNG 图片 (*.png)"));
    bool saved = false;
    if (!path.isEmpty()) {
        if (QFileInfo(path).suffix().compare(QStringLiteral("bmp"), Qt::CaseInsensitive) == 0) {
            const QImage indexedImage = latestImage_.convertToFormat(
                QImage::Format_Indexed8,
                Qt::DiffuseDither | Qt::PreferDither);
            saved = indexedImage.save(path, "BMP");
        } else {
            saved = latestImage_.save(path);
        }
    }
    if (saved) {
        appendLog(QStringLiteral("图片已保存: %1").arg(path));
        sendLogEvent(QStringLiteral("图片已保存至 A 端：%1").arg(path));
    } else if (!path.isEmpty()) {
        sendLogEvent(QStringLiteral("图片保存失败至 A 端：%1").arg(path));
    }
}

void RemoteCommandDialog::saveCurrentImageForCalibration() {
    sendLogEvent(QStringLiteral("点击添加图像至标定数据集"));
    saveCurrentImage();
}

void RemoteCommandDialog::saveCurrentPointCloud() {
    sendLogEvent(QStringLiteral("点击添加点云至文件树"));
    if (latestPointCloud_.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("当前没有可保存的点云。"));
        sendLogEvent(QStringLiteral("添加点云至文件树失败：当前没有点云"));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("保存点云"), nextSavePath(QStringLiteral(".txt")),
        QStringLiteral("点云文本 (*.txt);;PLY 文件 (*.ply)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, QStringLiteral("保存失败"), file.errorString());
        sendLogEvent(QStringLiteral("点云保存失败至 A 端：%1").arg(file.errorString()));
        return;
    }
    QTextStream stream(&file);
    if (path.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive)) {
        stream << "ply\nformat ascii 1.0\n";
        stream << "element vertex " << latestPointCloud_.size() << "\n";
        stream << "property float x\nproperty float y\nproperty float z\nend_header\n";
    }
    for (const QVector3D& point : latestPointCloud_) {
        stream << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
    }
    file.close();
    appendLog(QStringLiteral("点云已保存: %1").arg(path));
    sendLogEvent(QStringLiteral("点云已保存至 A 端：%1").arg(path));
}

void RemoteCommandDialog::saveCurrentPointCloudForCalibration() {
    sendLogEvent(QStringLiteral("点击添加点云至标定数据集"));
    saveCurrentPointCloud();
}

QString RemoteCommandDialog::nextSavePath(const QString& suffix) const {
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    return QDir(dir).filePath(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss")) + suffix);
}

void RemoteCommandDialog::setConnectionState(const QString& text, bool connected) {
    connectionStateLabel_->setText(text);
    connectionStateLabel_->setStyleSheet(connected
        ? QStringLiteral("color: #1a9b55;")
        : QStringLiteral("color: #e33232;"));
}

void RemoteCommandDialog::appendLog(const QString& message) {
    if (logEdit_) {
        logEdit_->appendPlainText(message);
    }
}
}}
