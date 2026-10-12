#ifndef COMMANDDIALOG_H
#define COMMANDDIALOG_H

#include "utils.h"

#include <QDialog>
#include <QHash>
#include <QImage>
#include <QJsonObject>
#include <QVector>
#include <QVector3D>

class QLabel;
class QCheckBox;
class QLineEdit;
class QPushButton;
class QComboBox;
class QTcpSocket;
class QUdpSocket;
class QPlainTextEdit;

NAMESPACE_BEGIN { namespace server {
    class RemoteCommandDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit RemoteCommandDialog(QWidget* parent = nullptr);
        ~RemoteCommandDialog() override;

    private slots:
        void onConnectButtonClicked();
        void onTcpConnected();
        void onTcpDisconnected();
        void onTcpReadyRead();
        void onUdpReadyRead();
        void onSocketError();
        void onSubscriptionChanged();
        void clearPointCloud();
        void saveCurrentImage();
        void saveCurrentImageForCalibration();
        void saveCurrentPointCloud();
        void saveCurrentPointCloudForCalibration();

    private:
        void buildUi();
        void setConnectionState(const QString& text, bool connected);
        void sendCommand(const QJsonObject& command);
        void sendSubscription();
        void handleTcpResponse(const QByteArray& frame);
        void handleUdpDatagram(const QByteArray& datagram);
        void appendLog(const QString& message);
        void sendLogEvent(const QString& message);
        QString nextSavePath(const QString& suffix) const;

        QTcpSocket* tcpSocket_ = nullptr;
        QUdpSocket* udpSocket_ = nullptr;
        QByteArray tcpBuffer_;
        quint32 nextRequestId_ = 1;
        QImage latestImage_;
        QVector<QVector3D> latestPointCloud_;

        QLineEdit* hostEdit_ = nullptr;
        QLineEdit* tcpPortEdit_ = nullptr;
        QLineEdit* udpPortEdit_ = nullptr;
        QPushButton* connectButton_ = nullptr;
        QLabel* connectionStateLabel_ = nullptr;
        QLabel* imagePreviewLabel_ = nullptr;
        QLabel* cloudStateLabel_ = nullptr;
        QLabel* statusLabel_ = nullptr;
        QPlainTextEdit* logEdit_ = nullptr;

        QLineEdit* statusAngleEdit_ = nullptr;
        QLineEdit* statusTemperatureEdit_ = nullptr;
        QLineEdit* statusHumidityEdit_ = nullptr;
        QLineEdit* statusPressureEdit_ = nullptr;
        QLineEdit* statusExposureEdit_ = nullptr;
        QLabel* scanStateLabel_ = nullptr;
        QLineEdit* scanBeginEdit_ = nullptr;
        QLineEdit* scanEndEdit_ = nullptr;
        QLineEdit* scanStepEdit_ = nullptr;
        QLineEdit* scanIntervalEdit_ = nullptr;
        QLineEdit* scanDelayEdit_ = nullptr;
        QLineEdit* scanExposureEdit_ = nullptr;

        QLineEdit* exposureEdit_ = nullptr;
        QLineEdit* gainEdit_ = nullptr;
        QLineEdit* currentAngleEdit_ = nullptr;
        QComboBox* triggerModeCombo_ = nullptr;
        QComboBox* triggerSourceCombo_ = nullptr;
        QCheckBox* statusSubscription_ = nullptr;
        QCheckBox* imageSubscription_ = nullptr;
        QCheckBox* cloudSubscription_ = nullptr;
        QCheckBox* showCloudCheck_ = nullptr;

        struct UdpAssembly {
            quint16 fragmentCount = 0;
            quint16 dataType = 0;
            QVector<QByteArray> fragments;
            QVector<bool> received;
            int receivedCount = 0;
            qint64 lastSeenMs = 0;
        };
        QHash<quint32, UdpAssembly> udpAssemblies_;
    };
}}

#endif // COMMANDDIALOG_H
