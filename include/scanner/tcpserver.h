#ifndef TCPSERVER_H
#define TCPSERVER_H
#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include "utils.h"

NAMESPACE_BEGIN { namespace scanner {
	/**
	 * @brief 光机 TCP 客户端
	 * 光机在固定 IP 和端口上提供 TCP 服务，上位机需要主动建立连接后发送控制指令。
	 */
	class TcpServer : public QObject {
		Q_OBJECT
	signals:
		/**
		 * @brief 已成功连接到光机
		 */
		void connected();

		/**
		 * @brief 信号，发送从光机接收的数据
		 * @param data 接收的数据
		 */
		void sendReceivedData(QVector<quint8>);
		
		/**
		 * @brief 信号，发送通知信息
		 * @param msg 通知信息
		 */
		void sendInformation(const QString& msg);

		/**
		 * @brief 信号，发送错误信息
		 * @param error 错误信息
		 */
		void errorOccurred(const QString& error);
	public:
		/**
		 * @brief 构造函数
		 * @param parent 父对象
		 */
		TcpServer(QObject* parent = nullptr);

		/**
		 * @brief 析构函数
		 */
		~TcpServer();

		/**
		 * @brief 写入数据到电机端
		 * @details 该函数会在内部将字节序列化，编码为电机端可以识别的字节流并发送
		 * @param data 传入指令的纯字节数据
		 */
		bool Write(const QVector<quint8> & data);

		bool hasClientConnection() const;

		/**
		 * @brief 连接光机
		 * @return 成功建立 TCP 连接时返回 true
		 */
		bool connectDevice();

	private slots:
		/**
		 * @brief 读取光机返回的数据
		 */
		void readClientData();

	public slots:
		/**
		 * @brief 响应连接光机的请求
		 */
		void responseOpen();
		
		/**
		 * @brief 响应断开光机的请求
		 */
		void responseClose();

	private:
		QTcpSocket* socket_ = nullptr;
		QByteArray receiveBuffer_;
		bool closing_ = false;
		QString host_;
		quint16 port_ = 0;
	};
}}
#endif
