#ifndef MODBUSMOTOR_H
#define MODBUSMOTOR_H
#include <QModbusDevice>
#include "tcpserver.h"
#include <QTcpSocket>
#include <tuple>
#include <QTimer>
#include <QMutex>
#include <QMutexLocker>
#include <atomic>
#include "utils.h"
NAMESPACE_BEGIN { namespace scanner {
	struct MotorFeedback {
		float angle = 0.0f;
		float humidity = 0.0f;
		float temperature = 0.0f;
		float pressure = 0.0f;
		quint8 attitudeFlag = 0;
		float roll = 0.0f;
		float pitch = 0.0f;
		float yaw = 0.0f;
		QByteArray reserved;
	};

	/**
	 * @brief Modbus电机类，该类用于控制Modbus电机，包括连接、断开、发送指令、收发数据等功能
	 * 在设计上，本类没有实现任何网络通信的功能，它所做的核心工作是将用户指令根据Modbus协议翻译为具体的字节，
	 * 然后调用底层的TcpServer类来发送指令。
	 */
	class ModbusMotor: public QObject {
		Q_OBJECT
	signals:
		/**
		 * @brief 信号，电机状态改变
		 * @param state 电机状态
		 */
		void stateChanged(QModbusDevice::State);
		
		/**
		 * @brief 信号，将电机的实时角度、温度、湿度、压力信息发送出去，由上层对象处理
		 * @param info 电机信息，包含角度、温度、湿度、压力值
		 */
		void sendInfo(const MotorFeedback&);

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

		/**
		 * @brief 信号，开始扫描
		 */
		void scanBegin();

		/**
		 * @brief 信号，扫描结束时触发
		 */
		void scanStopped();

		/**
		 * @brief 信号，激光器打开时触发
		 */
		void laserOpened();

		/**
		 * @brief 信号，激光器关闭时触发
		 */
		void laserClosed();
	private slots:
		/**
		 * @brief 槽函数，处理从TcpServer类接收的数据
		 * @param data 接收的数据
		 */
		void onReceiveData(const QVector<quint8> &);

	public:
		/**
		 * @brief 构造函数
		 * @param parent 父对象
		 */
		ModbusMotor(QObject* parent = nullptr);
		~ModbusMotor();

		/**
		 * @brief 连接Modbus设备
		 * @deprecated 这里的参数没有实际作用，只是为了兼容旧版接口，后续会考虑移除
		 */
		bool connectModbus(const QString& com_name, int iBaudRate);

		/**
		 * @brief 断开光机 TCP 连接
		 */
		bool disconnectModbus();

		/**
		 * @brief 检查Modbus设备是否已连接
		 * @return 光机 TCP 连接已建立时返回 true
		 */
		bool isConnected() const {
			return server && server->hasClientConnection();
		}

		/**
		 * @brief 设置设备反馈查询间隔
		 * @param intervalMs 查询间隔，单位为毫秒
		 */
		void setFeedbackInterval(int intervalMs);

		/**
		 * @brief 中断扫描
		 */
		bool stopScan();

		/**
		 * @brief 打开激光器
		 */
		bool openLaser();

		/**
		 * @brief 设置电机的限位角度
		 * @param left 左限位角度
		 * @param right 右限位角度
		 */
		bool setLimit(float left, float right);

		/**
		 * @brief 关闭激光器
		 */
		bool closeLaser();

		/**
		 * @brief 通过网络，向电机请求一次实时信息
		 * @return 包含角度、温度、湿度、压力值的元组
		 */
		std::tuple<int, ushort, ushort, int> getInfo();

		/**
		 * @brief 控制电机做步进运动
		 * @param speed 运动速度
		 * @param isMinus 是否向负方向运动
		 */
		bool jog(float speed, bool isMinus);

		/**
		 * @brief 设置电机的亮度
		 * @param brightness 亮度值，范围为0-100
		 */
		bool setBrightness(int brightness);

		/**
		 * @brief 控制电机直接到某一角度
		 * @param speed 运动速度
		 * @param angle 目标角度
		 */
		bool goTo(float speed, float angle);

		/**
		 * @brief 停止电机运动
		 */
		bool stopMove();
		
		/**
		 * @brief 设置触发模式下的连续运动
		 * @param speed 运动速度
		 * @param startNumber 起始角度
		 * @param endNumber 结束角度
		 * @param circleNumber 循环次数
		 * @param stayNumber 保持时间
		 */
		bool continuousMoveWithTrigger(float speed, float startNumber, float endNumber, float circleNumber, float stayNumber);

		/**
		 * @brief 设置连续运动，但是不触发信号
		 * @param speed 运动速度
		 * @param startNumber 起始角度
		 * @param endNumber 结束角度
		 * @param circleNumber 循环次数
		 * @param stayNumber 保持时间
		 */
		bool continuousMoveWithoutTrigger(float speed, float startNumber, float endNumber, float circleNumber, float stayNumber);

		/**
		 * @brief 设置触发模式下的间隔运动
		 * @param startNumber 起始角度
		 * @param endNumber 结束角度
		 * @param stayNumber 保持时间
		 * @param delayNumber 延迟时间
		 * @param tinyInNumber 间隔运动步长
		 */
		bool intervalMoveWithTrigger(float startNumber, float endNumber, float stayNumber, float delayNumber, float tinyInNumber);
		
		/**
		 * @brief 设置间隔运动，但是不触发信号
		 * @param startNumber 起始角度
		 * @param endNumber 结束角度
		 * @param stayNumber 保持时间
		 * @param delayNumber 延迟时间
		 * @param tinyInNumber 间隔运动步长
		 */
		bool intervalMoveWithoutTrigger(float startNumber, float endNumber, float stayNumber, float delayNumber, float tinyInNumber);

		/**
		 * @brief 获取当前电机角度
		 * @return float 当前电机角度
		 */
		float currentAngle() const { 
			QMutexLocker locker(&stateMutex_);
			return currentAngle_; 
		}

		/**
		 * @brief 获取当前电机的扫描限位
		 * @return std::pair<float, float> 当前扫描限位
		 */
		std::pair<float, float> currentLimit() const{
			QMutexLocker locker(&stateMutex_);
			return current_limit_;
		}

	private:
		/**
		 * @brief 内部辅助函数，在原有字节流的基础上增加校验和计算
		 * @param data 原始字节流
		 * @return QVector<quint8> 包含校验和的字节流
		 */
		QVector<quint8> getWriteData(const QVector<quint8> &&);

		/**
		 * @brief 内部辅助函数，将整数转换为4字节数组
		 * @param num 整数
		 * @return QVector<quint8> 4字节数组
		 */
		QVector<quint8> numToArray(int num);

		/**
		 * @brief 将无符号32位整数编码为小端字节序
		 */
		QVector<quint8> uint32ToLittleEndianArray(quint32 value);
		
		/**
		 * @brief 内部辅助函数，将4字节数组转换为浮点数
		 * @param bytes 4字节数组
		 * @return float 浮点数
		 */
		float bytesToFloat(const QByteArray& bytes);

		TcpServer * server;  						///< 光机 TCP 客户端
		QTimer* update_timer;  						///< 定时器对象，用于定时发送请求获取电机信息
		std::atomic<bool> sendSuccess = false;   	///< 标志位，用于表示是否成功发送指令
		float currentAngle_ = 0.0f;					///< 当前电机角度
		float currentTemp_ = 0.0f;					///< 当前电机温度
		float currentHumidity_ = 0.0f;				///< 当前电机湿度
		float currentPressure_ = 0.0f;				///< 当前电机压力s
		mutable QMutex stateMutex_;					///< 状态互斥锁，用于保护状态变量的访问
		std::pair<float, float> current_limit_;     ///< 当前扫描限位
        std::pair<float, float> target_limit_;      ///< 目标扫描限位
	};
}}
#endif
