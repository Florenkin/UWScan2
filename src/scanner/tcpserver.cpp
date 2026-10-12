#include "scanner/tcpserver.h"
#include <QByteArray>
#include <setting/settingsmanager.h>
NAMESPACE_BEGIN { namespace scanner {
	TcpServer::TcpServer(QObject* parent) : QObject(parent) {
		socket_ = new QTcpSocket(this);
		connect(socket_, &QTcpSocket::connected, this, [this]() {
			emit sendInformation(tr("已连接光机：%1:%2").arg(host_).arg(port_));
			emit connected();
		});
		connect(socket_, &QTcpSocket::disconnected, this, [this]() {
			if (!closing_) {
				emit errorOccurred(tr("光机网络连接已断开，请检查光机网络连接"));
			}
		});
		connect(socket_, &QTcpSocket::readyRead, this, &TcpServer::readClientData);
	}

	TcpServer::~TcpServer() {
		responseClose();
	}

	bool TcpServer::Write(const QVector<quint8>& message) {
		if (!hasClientConnection()) {
			emit errorOccurred(tr("光机未连接或 TCP 不可写"));
			return false;
		}

		QByteArray data;
		data.reserve(message.size());
		for (quint8 value : message) {
			data.append(static_cast<char>(value));
		}

		qint64 bytesWritten = socket_->write(data);
		if (bytesWritten == -1) {
			emit errorOccurred(tr("发送光机指令失败：") + socket_->errorString());
			return false;
		}
		socket_->flush();
		return bytesWritten == data.size();
	}

	bool TcpServer::hasClientConnection() const {
		return socket_ && socket_->state() == QAbstractSocket::ConnectedState && socket_->isWritable();
	}

	bool TcpServer::connectDevice() {
		if (hasClientConnection()) {
			return true;
		}

		setting::SettingsManager* manager = setting::SettingsManager::getInstance();
		host_ = manager->getString(setting::SCANNER_HOST, QStringLiteral("192.168.0.7")).trimmed();
		const int configuredPort = manager->getInt(setting::SCANNER_PORT, 8234);
		if (host_.isEmpty()) {
			emit errorOccurred(tr("光机 IP 不能为空"));
			return false;
		}
		if (configuredPort < 1 || configuredPort > 65535) {
			emit errorOccurred(tr("光机端口必须在 1-65535 之间"));
			return false;
		}
		port_ = static_cast<quint16>(configuredPort);

		closing_ = true;
		socket_->abort();
		closing_ = false;
		socket_->connectToHost(host_, port_);
		if (socket_->waitForConnected(1500)) {
			return true;
		}

		emit errorOccurred(tr("连接光机 %1:%2 失败：%3")
			.arg(host_)
			.arg(port_)
			.arg(socket_->errorString()));
		return false;
	}

	void TcpServer::readClientData() {
		if (!socket_) {
			return;
		}

		receiveBuffer_.append(socket_->readAll());
		auto hasValidChecksum = [](const QByteArray& frame) {
			quint8 sum = 0;
			for (int i = 0; i < frame.size() - 1; ++i) {
				sum = static_cast<quint8>(sum + static_cast<quint8>(frame.at(i)));
			}
			return sum == static_cast<quint8>(frame.back());
		};
		auto emitFrame = [this](const QByteArray& frame) {
			QVector<quint8> data;
			data.reserve(frame.size());
			for (char byte : frame) {
				data.append(static_cast<quint8>(byte));
			}
			emit sendReceivedData(data);
		};

		while (receiveBuffer_.size() >= 2) {
			int frameStart = -1;
			for (int i = 0; i < receiveBuffer_.size() - 1; ++i) {
				const bool acknowledgement = receiveBuffer_.at(i) == 0x03 &&
					receiveBuffer_.at(i + 1) == 0x01;
				const bool status = receiveBuffer_.at(i) == 0x3E &&
					receiveBuffer_.at(i + 1) == static_cast<char>(0x92);
				if (acknowledgement || status) {
					frameStart = i;
					break;
				}
			}
			if (frameStart < 0) {
				const char lastByte = receiveBuffer_.back();
				receiveBuffer_.clear();
				if (lastByte == 0x03 || lastByte == 0x3E) {
					receiveBuffer_.append(lastByte);
				}
				return;
			}
			if (frameStart > 0) {
				receiveBuffer_.remove(0, frameStart);
			}

			const bool status = receiveBuffer_.at(0) == 0x3E &&
				receiveBuffer_.at(1) == static_cast<char>(0x92);
			if (status) {
				if (receiveBuffer_.size() < 4) {
					return;
				}
				if (receiveBuffer_.at(2) != 0x01 || receiveBuffer_.at(3) != 0x08) {
					receiveBuffer_.remove(0, 1);
					continue;
				}

				// Older controllers send a 24-byte status frame; newer firmware appends
				// ten reserved bytes and sends a 34-byte frame.
				if (receiveBuffer_.size() >= 34) {
					const QByteArray frame = receiveBuffer_.left(34);
					if (hasValidChecksum(frame)) {
						receiveBuffer_.remove(0, 34);
						emitFrame(frame);
						continue;
					}
				}
				if (receiveBuffer_.size() >= 24) {
					const QByteArray frame = receiveBuffer_.left(24);
					if (hasValidChecksum(frame)) {
						receiveBuffer_.remove(0, 24);
						emitFrame(frame);
						continue;
					}
				}
				if (receiveBuffer_.size() < 34) {
					return;
				}
			} else {
				if (receiveBuffer_.size() < 17) {
					return;
				}
				const QByteArray frame = receiveBuffer_.left(17);
				if (hasValidChecksum(frame)) {
					receiveBuffer_.remove(0, 17);
					emitFrame(frame);
					continue;
				}
			}

			receiveBuffer_.remove(0, 1);
		}
	}

	void TcpServer::responseOpen() {
		connectDevice();
	}

	void TcpServer::responseClose() {
		receiveBuffer_.clear();
		if (!socket_ || socket_->state() == QAbstractSocket::UnconnectedState) {
			return;
		}
		closing_ = true;
		socket_->disconnectFromHost();
		if (socket_->state() != QAbstractSocket::UnconnectedState) {
			socket_->abort();
		}
	}
}}
