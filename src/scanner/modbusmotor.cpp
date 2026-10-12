#include "scanner/modbusmotor.h"
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <algorithm>
#include <cmath>

NAMESPACE_BEGIN { namespace scanner {
	ModbusMotor::ModbusMotor(QObject* parent) : QObject(parent) {
		this->server = new TcpServer(this);
		connect(this->server, &TcpServer::sendInformation, this, &ModbusMotor::sendInformation);
		connect(this->server, &TcpServer::errorOccurred, this, &ModbusMotor::errorOccurred);
		connect(this->server, &TcpServer::errorOccurred, this, [this](const QString& error) {
			if (error.contains(QStringLiteral("断开")) && update_timer) {
				update_timer->stop();
			}
		});
		connect(this->server, &TcpServer::connected, this, [this]() {
			emit stateChanged(QModbusDevice::ConnectedState);
			// this->setLimit(42, 57);
			update_timer->start(500);
			std::cout << "[Debug] ModbusMotor::connected() -> Timer Start " << std::endl;
			});
		connect(this->server, &TcpServer::sendReceivedData, this, &ModbusMotor::onReceiveData);

		update_timer = new QTimer(this);
		connect(update_timer, &QTimer::timeout, this, [this]() {
			this->getInfo();
			});
	}

	bool ModbusMotor::connectModbus(const QString& com_name, int iBaudRate) {
		Q_UNUSED(com_name);
		Q_UNUSED(iBaudRate);
		sendSuccess.store(false);
		return server && server->connectDevice();
	}

	bool ModbusMotor::disconnectModbus() {
		if (update_timer) {
			update_timer->stop();
		}
		sendSuccess.store(false);
		if (server) {
			server->responseClose();
		}
		emit stateChanged(QModbusDevice::UnconnectedState);
		return true;
	}

	ModbusMotor::~ModbusMotor() {
		disconnectModbus();
	}

	void ModbusMotor::setFeedbackInterval(int intervalMs) {
		if (update_timer) {
			update_timer->setInterval(std::clamp(intervalMs, 50, 1000));
		}
	}

	bool ModbusMotor::stopScan() {
		const bool ok = server->Write(getWriteData({ 0x03, 0x01, 0x0A, 0x01 }));
		std::cout << "[Debug] ModbusMotor::stopScan()" << std::endl;
		return ok;
	}

	bool ModbusMotor::openLaser() {
		const bool ok = server->Write(getWriteData({ 0x03, 0x01, 0x0B, 0x01 }));
		std::cout << "[Debug] ModbusMotor::openLaser()" << std::endl;
		return ok;
	}

	//关闭激光器
	bool ModbusMotor::closeLaser() {
		const bool ok = server->Write(getWriteData({ 0x03, 0x01, 0x0B, 0x00 }));
		std::cout << "[Debug] ModbusMotor::closeLaser()" << std::endl;
		return ok;
	}

	//获取信息
	std::tuple<int, ushort, ushort, int> ModbusMotor::getInfo() {
		server->Write(getWriteData({ 0x03, 0x01, 0x06, 0x01 }));
		int angle = 0, pressure = 0;
		ushort tempareture = 0, humity = 0;
		return std::make_tuple(angle, tempareture, humity, pressure);
	}

	//步进
	bool ModbusMotor::jog(float speed, bool isMinus) {
		int speedNumber = int(speed * 100 * (isMinus ?  -1 : 1));
		QVector<quint8> ret = numToArray(speedNumber);
		server->Write(getWriteData({ 0x03, 0x01, 0x01, ret[3], ret[2], ret[1],ret[0] }));
		std::cout << "[Debug] ModbusMotor::jog()" << std::endl;
		return true;
	}

	bool ModbusMotor::setBrightness(int brightness){
		if(brightness < 0 || brightness > 100){
			emit sendInformation("亮度值必须在0-100之间");
			return false;
		}
		const QVector<quint8> ret = uint32ToLittleEndianArray(static_cast<quint32>(brightness));
		const bool ok = server->Write(getWriteData({ 0x03, 0x01, 0x0C, ret[0], ret[1], ret[2], ret[3] }));
		std::cout << "[Debug] ModbusMotor::setBrightness()" << std::endl;
		return ok;
	}

	//位置进动
	bool ModbusMotor::goTo(float speed, float angle) {
		int speedNumber = static_cast<int>(std::lround(speed * 100.0f));
		int AngleNumber = static_cast<int>(std::lround(angle * 100.0f));
		QVector<quint8> ret = numToArray(speedNumber);
		QVector<quint8> angleret = numToArray(AngleNumber);
		const bool ok = server->Write(getWriteData({ 0x03, 0x01, 0x07, angleret[3], angleret[2], angleret[1],angleret[0], ret[3], ret[2], ret[1], ret[0] }));
		std::cout << "[Debug] ModbusMotor::goTo()" << std::endl;
		return ok;
	}

	//停止电机运动
	bool ModbusMotor::stopMove() {
		server->Write(getWriteData({ 0x03, 0x01, 0x00, 0x01 }));
		std::cout << "[Debug] ModbusMotor::stopMove()" << std::endl;
		return true;
	}

	//连续运动触发信号
	bool ModbusMotor::continuousMoveWithTrigger(float speed, float startNumber,\
		float endNumber, float circleNumber, float stayNumber) {
		QVector<quint8> speedBytes = numToArray(speed * 100);
		QVector<quint8> startBytes = numToArray(startNumber * 100);
		QVector<quint8> incrementBytes = numToArray(abs(startNumber - endNumber) * 100);  //这里有问题，有精度损失
		QVector<quint8> endNumberBytes = numToArray(endNumber * 100);
		QVector<quint8> circleBytes = numToArray(circleNumber * 100);
		QVector<quint8> stayBytes = numToArray(stayNumber * 100);
		server->Write(getWriteData({ 0x03, 0x01, 0x03, incrementBytes[3], incrementBytes[2], incrementBytes[1], \
			incrementBytes[0], speedBytes[3], speedBytes[2], speedBytes[1], speedBytes[0],\
			circleBytes[3], circleBytes[2], circleBytes[1], circleBytes[0], \
			stayBytes[3], stayBytes[2], stayBytes[1], stayBytes[0], \
			startBytes[3], startBytes[2], startBytes[1], startBytes[0], \
			endNumberBytes[3], endNumberBytes[2], endNumberBytes[1], endNumberBytes[0] }));
		//QVector<quint8> ret = 
		//if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x03 && ret[3] == 0x01) {
		//	return true;
		//}
		//else return false;
		return true;
	}

	//连续运动不触发信号
	bool ModbusMotor::continuousMoveWithoutTrigger(float speed, float startNumber, float endNumber, \
		float circleNumber, float stayNumber) {
		QVector<quint8> speedBytes = numToArray(speed * 100);
		QVector<quint8> startBytes = numToArray(startNumber * 100);
		QVector<quint8> incrementBytes = numToArray((startNumber - endNumber) * 100);
		QVector<quint8> endNumberBytes = numToArray(endNumber * 100);
		QVector<quint8> circleBytes = numToArray(circleNumber * 100);
		QVector<quint8> stayBytes = numToArray(stayNumber * 100);
		server->Write(getWriteData({ 0x03, 0x01, 0x02, incrementBytes[3], incrementBytes[2], incrementBytes[1], \
			incrementBytes[0], speedBytes[3], speedBytes[2], speedBytes[1], speedBytes[0],\
			circleBytes[3], circleBytes[2], circleBytes[1], circleBytes[0], \
			stayBytes[3], stayBytes[2], stayBytes[1], stayBytes[0], \
			startBytes[3], startBytes[2], startBytes[1], startBytes[0], \
			endNumberBytes[3], endNumberBytes[2], endNumberBytes[1], endNumberBytes[0] }));
		return true;
	}

	//间隔运动触发信号
	bool ModbusMotor::intervalMoveWithTrigger(float startNumber, float endNumber,\
		float stayNumber, float delayNumber, float tinyInNumber) {
		//std::cout << "Start:" << startNumber * 100 << " Increment:" << abs(startNumber - endNumber) * 100 << " TinyInNumber:" << tinyInNumber * 100 << " stayNumber:" << stayNumber * 100 << " delayNumber:" << delayNumber * 100 << std::endl;
		QVector<quint8> incrementBytes = numToArray(abs((int)(startNumber * 100) - (int)(endNumber * 100)));
		QVector<quint8> startBytes = numToArray(startNumber * 100);
		QVector<quint8> stayBytes = numToArray(stayNumber * 100);
		QVector<quint8> delayBytes = numToArray(delayNumber * 100);
		QVector<quint8> tinyInBytes = numToArray(tinyInNumber * 100);
		QVector<quint8> directionBytes = numToArray(endNumber > startNumber ? 1 : 0);
		server->Write(getWriteData({ 0x03, 0x01, 0x05, incrementBytes[3], incrementBytes[2], incrementBytes[1], incrementBytes[0],\
			tinyInBytes[3], tinyInBytes[2], tinyInBytes[1], tinyInBytes[0],\
			delayBytes[3], delayBytes[2], delayBytes[1], delayBytes[0],\
			stayBytes[3], stayBytes[2], stayBytes[1], stayBytes[0],\
			startBytes[3], startBytes[2], startBytes[1], startBytes[0],\
			directionBytes[3], directionBytes[2], directionBytes[1], directionBytes[0] }));
		return true;
	}

	//间隔运动不触发信号
	bool ModbusMotor::intervalMoveWithoutTrigger(float startNumber, float endNumber,\
		float stayNumber, float delayNumber, float tinyInNumber) {
		QVector<quint8> incrementBytes = numToArray(abs(startNumber - endNumber) * 100);
		QVector<quint8> startBytes = numToArray(startNumber * 100);
		QVector<quint8> stayBytes = numToArray(stayNumber * 100);
		QVector<quint8> delayBytes = numToArray(delayNumber * 100);
		QVector<quint8> tinyInBytes = numToArray(tinyInNumber * 100);
		QVector<quint8> directionBytes = numToArray(endNumber > startNumber ? 1 : 0);

		int count = 0;
		sendSuccess.store(false);
		while (!sendSuccess.load() && count < 5) {
			server->Write(getWriteData({ 0x03, 0x01, 0x04, incrementBytes[3], incrementBytes[2], incrementBytes[1], incrementBytes[0],\
			tinyInBytes[3], tinyInBytes[2], tinyInBytes[1], tinyInBytes[0],\
			delayBytes[3], delayBytes[2], delayBytes[1], delayBytes[0],\
			stayBytes[3], stayBytes[2], stayBytes[1], stayBytes[0],\
			startBytes[3], startBytes[2], startBytes[1], startBytes[0],\
			directionBytes[3], directionBytes[2], directionBytes[1], directionBytes[0] }));
			count++;
			Sleep(200);
		}
		//if(!sendSuccess.load())

		return true;
	}

	QVector<quint8> ModbusMotor::getWriteData(const QVector<quint8>&& vec) {
		QVector<quint8> writedata = vec;
		quint8 DATA_SUM = 0x00;
		for (int i = 0; i < writedata.size(); ++i) {
			DATA_SUM += writedata[i];
		}
		writedata.push_back(DATA_SUM);
		return writedata;
	}

	QVector<quint8> ModbusMotor::numToArray(int num){
		// 转换为无符号32位整数以避免符号问题
		uint32_t speed = static_cast<uint32_t>(num);
		QVector<quint8> ret;
		int right_move = 24;
		for (int i = 0; i < 4; ++i) {
			ret.push_back(static_cast<quint8>((speed >> right_move) & 0x000000FF));
			//std::cout << std::hex << (int)ret[i] << " ";
			//speed >>= 8;
			right_move -= 8;
		}
		return ret;
	}

	QVector<quint8> ModbusMotor::uint32ToLittleEndianArray(quint32 value) {
		return {
			static_cast<quint8>(value & 0xFF),
			static_cast<quint8>((value >> 8) & 0xFF),
			static_cast<quint8>((value >> 16) & 0xFF),
			static_cast<quint8>((value >> 24) & 0xFF)
		};
	}

	void ModbusMotor::onReceiveData(const QVector<quint8>& ret) {
		if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x0A && ret[3] == 0x01) {
			emit scanStopped();
			emit sendInformation("成功中断扫描");
		} 
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x0B && ret[3] == 0x01){
			emit laserOpened();
		}
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x0B && ret[3] == 0x00){
			emit sendInformation("成功关闭激光器");
			emit laserClosed();
		}
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x0C && ret[3] == 0x01) {
			emit sendInformation("成功设置激光器亮度");
		}
		// 查询反馈兼容 24 字节旧帧和 34 字节扩展帧，最后 1 字节均为 SUM 校验。
		else if ((ret.size() == 24 || ret.size() == 34) &&
				 ret[0] == 0x3E && ret[1] == 0x92 && ret[2] == 0x01 && ret[3] == 0x08) {
			quint8 sum = 0;
			for (int i = 0; i < ret.size() - 1; ++i) {
				sum = static_cast<quint8>(sum + ret[i]);
			}
			if (sum != ret.back()) {
				emit sendInformation("查询反馈校验和错误");
				return;
			}
			float angle = 0.0, pressure = 0, tempareture = 0, humity = 0.0;
			const qint32 angleRawLittle = static_cast<qint32>(
				static_cast<quint32>(ret[5]) |
				(static_cast<quint32>(ret[6]) << 8) |
				(static_cast<quint32>(ret[7]) << 16) |
				(static_cast<quint32>(ret[8]) << 24));
			const qint32 angleRawBig = static_cast<qint32>(
				static_cast<quint32>(ret[8]) |
				(static_cast<quint32>(ret[7]) << 8) |
				(static_cast<quint32>(ret[6]) << 16) |
				(static_cast<quint32>(ret[5]) << 24));
			float angleLittle = angleRawLittle / 100.0f;
			float angleBig = angleRawBig / 100.0f;
			auto currentLimits = currentLimit();
			const float lower = std::min(currentLimits.first, currentLimits.second);
			const float upper = std::max(currentLimits.first, currentLimits.second);
			const bool littleInRange = angleLittle >= lower - 10.0f && angleLittle <= upper + 10.0f;
			const bool bigInRange = angleBig >= lower - 10.0f && angleBig <= upper + 10.0f;
			if (littleInRange && !bigInRange) {
				angle = angleLittle;
			} else if (bigInRange && !littleInRange) {
				angle = angleBig;
			} else {
				angle = angleLittle;
			}
			std::cout << "[Debug] ModbusMotor::onReceiveData() angle raw little: " << angleLittle
					  << " angle raw big: " << angleBig
					  << " final: " << angle << std::endl;

			ushort num = (ret[9] << 8 | ret[10]);
			tempareture = num * 1.0f;
			if(tempareture >= 32768) tempareture = (tempareture - 65536) / 256;
			else tempareture = num / 256.0f;

			num = (ret[11] << 8 | ret[12]);
			humity = num * 1.0f;

			unsigned int temp = (ret[13] << 16 | ret[14] << 8 | ret[15]);
			pressure = (temp >= 8388608 ? (temp - 16777216) / 8388608.0 : temp / 8388608.0);
			pressure = (1000 * pressure - 150.0) / 100.0;

			if (humity > 100) humity = 100;
			if (humity < 0) humity = 0;

			auto readInt16LittleEndian = [&ret](int index) {
				return static_cast<qint16>(static_cast<quint16>(ret[index]) |
					(static_cast<quint16>(ret[index + 1]) << 8));
			};
			const float roll = readInt16LittleEndian(17) / 100.0f;
			const float pitch = readInt16LittleEndian(19) / 100.0f;
			const float yaw = readInt16LittleEndian(21) / 100.0f;

			{
				QMutexLocker locker(&stateMutex_);
				currentAngle_ = angle;
				currentTemp_ = tempareture;
				currentHumidity_ = humity;
				currentPressure_ = pressure;
			}

			MotorFeedback feedback;
			feedback.angle = angle;
			feedback.temperature = tempareture;
			feedback.humidity = humity;
			feedback.pressure = pressure;
			feedback.attitudeFlag = ret[16];
			feedback.roll = roll;
			feedback.pitch = pitch;
			feedback.yaw = yaw;
			feedback.reserved.reserve(std::max(0, ret.size() - 24));
			for (int i = 23; i < ret.size() - 1; ++i) {
				feedback.reserved.append(static_cast<char>(ret[i]));
			}
			emit sendInfo(feedback);
		}
		// 兼容旧版17字节查询反馈。
		else if (ret.size() >= 17 && ret[0] == 0x3E && ret[1] == 0x92 && ret[2] == 0x01 && ret[3] == 0x08) {
			float angle = 0.0, pressure = 0, tempareture = 0, humity = 0.0;
			const qint32 angleRawLittle = static_cast<qint32>(
				static_cast<quint32>(ret[5]) |
				(static_cast<quint32>(ret[6]) << 8) |
				(static_cast<quint32>(ret[7]) << 16) |
				(static_cast<quint32>(ret[8]) << 24));
			angle = angleRawLittle / 100.0f;
			ushort num = (ret[9] << 8 | ret[10]);
			tempareture = num >= 32768 ? (num - 65536) / 256.0f : num / 256.0f;
			num = (ret[11] << 8 | ret[12]);
			humity = std::clamp(static_cast<float>(num), 0.0f, 100.0f);
			unsigned int temp = (ret[13] << 16 | ret[14] << 8 | ret[15]);
			pressure = (temp >= 8388608 ? (temp - 16777216) / 8388608.0 : temp / 8388608.0);
			pressure = (1000 * pressure - 150.0) / 100.0;
			MotorFeedback feedback;
			feedback.angle = angle;
			feedback.temperature = tempareture;
			feedback.humidity = humity;
			feedback.pressure = pressure;
			emit sendInfo(feedback);
		}
		// 03 01 07 01
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x07 && ret[3] == 0x01) emit sendInformation("成功开始跳转到目标位置");
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x00 && ret[3] == 0x01) emit sendInformation("成功停止电机运动");
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x05 && ret[3] == 0x01) {
			sendSuccess.store(true);
			emit scanBegin();
			emit sendInformation("成功开始间隔触发采集");
		} 
		else if (ret.size() >= 4 && ret[0] == 0x03 && ret[1] == 0x01 && ret[2] == 0x09 && ret[3] == 0x01) {
			current_limit_ = target_limit_;
			emit sendInformation("成功设置软限位");
		} 
	}

	float ModbusMotor::bytesToFloat(const QByteArray& bytes) {
		QDataStream stream(bytes);
		float value;
		stream >> value;
		return value;
	}

	bool ModbusMotor::setLimit(float left, float right) {
		QVector<quint8> leftLimitBytes = numToArray(left * 100);
		QVector<quint8> rightLimitBytes = numToArray(right * 100);
		server->Write(getWriteData({ 0x03, 0x01, 0x09,\
			leftLimitBytes[3], leftLimitBytes[2], leftLimitBytes[1], leftLimitBytes[0],\
			rightLimitBytes[3], rightLimitBytes[2], rightLimitBytes[1], rightLimitBytes[0] }));
			{
        QMutexLocker locker(&stateMutex_);
        current_limit_ = std::make_pair(left, right);
    }
		target_limit_ = std::make_pair(left, right);
		return true;
	}

}}
