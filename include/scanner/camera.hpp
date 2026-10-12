#ifndef CAMERA_H
#define CAMERA_H
#include <iostream>
#include <optional>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/ConfigurationEventHandler.h>
#include <QObject>
#include <windows.h>
#include <QMutex>
#include <QDebug>
#include <QThread>
#include "utils.h"

NAMESPACE_BEGIN { namespace scanner {
    /**
     * @brief 相机类
     * 
     * 该类封装了相机的初始化、配置、采集和图像处理等功能。
     * 采用QMutex和Pylon框架自带的AutoLock机制，确保在多线程环境下的线程安全，
     * 前者用户保护相机对象的创建与销毁，后者用于保护相机配置参数的读写操作。
     */
    class Camera: public QObject, public Pylon::CImageEventHandler, public Pylon::CConfigurationEventHandler {
        Q_OBJECT

        using String_t = Pylon::String_t;
        using INodeMap = GenApi::INodeMap;
    public:
        /**
         * @brief 构造函数
         * @details Camera对象构造时不会直接初始化相机，调用initCamera()方法后才会初始化
         * @param parent 父对象指针，默认值为 nullptr
         */
        explicit Camera(QObject *parent = nullptr) : QObject(parent), camera(nullptr) {
            Pylon::PylonInitialize();
            qRegisterMetaType<Pylon::CGrabResultPtr>("Pylon::CGrabResultPtr"); // 注册智能指针类型
        }

        /**
         * @brief 析构函数
         * @details 负责释放相机资源，包括关闭相机、停止采集和释放 Pylon 运行时
         */
        virtual ~Camera() {
            closeCamera();
            if(camera){
                camera->DeregisterImageEventHandler(this);
                camera->DeregisterConfiguration(this);
            }

            Pylon::PylonTerminate();
        }

        /**
         * @brief 初始化相机
         * @details 默认返回第一个相机，后续会支持多相机下可供选择的功能
         *          本函数除了会创建具体的相机对象，还会注册图像事件处理，并打开相机
         * @return 成功返回 true
         */
        bool initCamera() {
            QMutexLocker locker(&cameraMutex);
            try {
                if (camera && !camera->IsPylonDeviceAttached()) {
                    camera.reset();
                }
                if(!camera){
                    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
                    Pylon::DeviceInfoList_t device_list;
                    // 检查是否有相机连接
                    if (tlFactory.EnumerateDevices(device_list) < 1) {
                        std::cout << ">>> [Debug] Camera::initCamera() -> No Camera Device Found" << std::endl;
                        emit errorOccurred("未找到相机设备");
                        return false;
                    }

                    // 创建相机对象
                    camera = std::make_unique<Pylon::CBaslerUniversalInstantCamera>(tlFactory.CreateFirstDevice());
                    std::cout << ">>> [Debug] Camera::initCamera() camera: " << camera->GetDeviceInfo().GetModelName().c_str() << std::endl;
                    camera->RegisterImageEventHandler(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None);
                    camera->RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None);
                }

                // 打开相机
                camera->Open();
                // setupCameraChunk(camera.get());
                QThread::msleep(100);  // 这里等待100ms，确保相机打开，提供一定的缓冲时间
                return camera->IsOpen();
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("相机初始化失败: %1").arg(e.GetDescription()));
                camera.reset();
                return false;
            }
        }

        /**
         * @brief 关闭相机
         * @details 负责关闭相机、停止采集和注销图像事件处理
         */
        void closeCamera(){
            QMutexLocker locker(&cameraMutex);
            if(!camera) return;
            try{
                if (camera->IsGrabbing()) camera->StopGrabbing();
                QThread::msleep(100);  // 这里等待100ms，确保采集停止
                if (camera->IsOpen()){
                    camera->Close();
                }
                camera->DeregisterImageEventHandler(this);
                camera->DeregisterConfiguration(this);
                camera.reset();
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("相机关闭失败: %1").arg(e.GetDescription()));
                camera.reset();
            }
        }

        /**
         * @brief 检查相机是否已连接
         * @return 如果相机已连接且打开，则返回 true
         */
        bool isConnected() const {
            return camera && camera->IsOpen();
        }

        /**
         * @brief 清空相机缓存队列
         * @details 清空相机的缓存队列，确保后续采集到的图像是最新的
         */
        void clearBuffer(){
            QMutexLocker locker(&cameraMutex);
            size_t clearedCount = 0;
            try {
                Pylon::CGrabResultPtr ptrGrabResult;
                camera->GrabOne(1, ptrGrabResult, Pylon::TimeoutHandling_Return);
                while (ptrGrabResult->GrabSucceeded()) {
                    clearedCount++;
                    camera->GrabOne(1, ptrGrabResult, Pylon::TimeoutHandling_Return);
                }
            } catch (...) {
                // 忽略异常
            }
            std::cout << ">>> [Debug] Camera::clearBuffer() -> clearedCount: " << clearedCount << std::endl;
        }

        /**
         * @brief 单帧采集
         * @details 采集一帧图像，若相机未连接或采集失败，会触发错误信号
         * @return 成功返回 true
         */
        bool SingleGrab() {
            QMutexLocker locker(&cameraMutex);
            if (!isConnected()) return false;
            try {
                if (camera->IsGrabbing()) camera->StopGrabbing();
                camera->StartGrabbing(1,
                                      Pylon::GrabStrategy_LatestImageOnly,
                                      Pylon::GrabLoop_ProvidedByInstantCamera);
                return camera->IsGrabbing();
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("单帧采集失败: %1").arg(e.GetDescription()));
                return false;
            }
        }

        /**
         * @brief 连续采集
         * @details 开始连续采集图像，若相机未连接或采集失败，会触发错误信号
         * @param changeTriggerMode 在连续采集时，是否将触发模式为OFF，默认为 true。本参数的目的主要是区分两种情况的连续采集，
         * 一种是预览状态，在进行预览时，无论原本的TriggerMode是什么，都将自动改为OFF。
         * 另一种是扫描状态，在扫描状态下，更希望TriggerMode维护为原本的触发模式，而不是自动改为OFF。
         * @return 成功返回 true
         */
        bool ContinuousGrab(bool changeTriggerMode = true) {
            QMutexLocker locker(&cameraMutex);
            if (!isConnected()) return false;
            try {
                if (changeTriggerMode) setTriggerConfig("FrameStart", false, "Line0");
                if (camera->IsGrabbing()) return true;
                
                camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByInstantCamera);
                return camera->IsGrabbing();
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("连续采集失败: %1").arg(e.GetDescription()));
                return false;
            }
        }

        /**
         * @brief 停止采集
         * @details 停止相机的连续采集，若相机未连接或停止失败，会触发错误信号
         * @return 成功返回 true
         */
        bool StopGrab() {
            QMutexLocker locker(&cameraMutex);
            if (!isConnected()) return false;
            try {
                camera->StopGrabbing();
                return true;
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("停止采集失败: %1").arg(e.GetDescription()));
                return false;
            }
        }

        /**
         * @brief 设置曝光时间
         * @details 设置相机的曝光时间，若相机未连接或设置失败，会触发错误信号
         * @param value 曝光时间，单位为秒
         */
        void setExposureTime(double value) {
            if (!isConnected()) return;
            try {
                // 如果开启了自动曝光，先关闭它，否则无法设置固定值
                if (camera->ExposureAuto.IsWritable() && 
                    camera->ExposureAuto.GetValue() != Basler_UniversalCameraParams::ExposureAuto_Off) {
                    camera->ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Off);
                }

                if (camera->ExposureTime.IsWritable()) {
                    double realValue = value;
                    if (realValue < camera->ExposureTime.GetMin()) realValue = camera->ExposureTime.GetMin();
                    if (realValue > camera->ExposureTime.GetMax()) realValue = camera->ExposureTime.GetMax();
                    camera->ExposureTime.SetValue(realValue);
                } else if (camera->ExposureTimeAbs.IsWritable()) {
                    double realValue = value;
                    if (realValue < camera->ExposureTimeAbs.GetMin()) realValue = camera->ExposureTimeAbs.GetMin();
                    if (realValue > camera->ExposureTimeAbs.GetMax()) realValue = camera->ExposureTimeAbs.GetMax();
                    camera->ExposureTimeAbs.SetValue(realValue);
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置曝光失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取曝光时间
         * @details 获取相机的当前曝光时间，若相机未连接或获取失败，会触发错误信号
         * @return 当前曝光时间，单位为秒
         */
        double exposureTime() const {
            if (!isConnected()) return 0.0;
            try {
                // 有的相机叫 ExposureTimeAbs，有的叫 ExposureTime
                if (camera->ExposureTime.IsReadable()) return camera->ExposureTime.GetValue();
                if (camera->ExposureTimeAbs.IsReadable()) return camera->ExposureTimeAbs.GetValue();
                return 0.0;
            } catch (...) { return 0.0; }
        }

        std::pair<double, double> exposureTimeRange() const {
            if (!isConnected()) return {0.0, 1000000.0};
            try {
                if (camera->ExposureTime.IsReadable()) return {camera->ExposureTime.GetMin(), camera->ExposureTime.GetMax()};
                if (camera->ExposureTimeAbs.IsReadable()) return {camera->ExposureTimeAbs.GetMin(), camera->ExposureTimeAbs.GetMax()};
            } catch (...) {}
            return {0.0, 1000000.0};
        }

        /**
         * @brief 设置增益
         * @details 设置相机的增益，若相机未连接或设置失败，会触发错误信号
         * @param value 增益值
         */
        void setGain(double value) {
            if (!isConnected()) return;
            try {
                // 如果开启了自动增益，先关闭它，否则无法设置固定值
                if (camera->GainAuto.IsWritable() && 
                    camera->GainAuto.GetValue() != Basler_UniversalCameraParams::GainAuto_Off) {
                    camera->GainAuto.SetValue(Basler_UniversalCameraParams::GainAuto_Off);
                }

                // 注意：不同相机 Gain 的名称不同 (Gain, GainRaw, GainAbs)
                // Universal 类通常会映射到 Gain，其他相机可能会映射到 GainRaw 或 GainAbs
                // 这里先检查 Gain 是否可写，否则再检查 GainRaw
                // 如果相机支持 GainAbs，Pylon 会映射到正确的那个
                if (camera->Gain.IsWritable()) {
                    camera->Gain.SetValue(value);
                } else if (camera->GainRaw.IsWritable()) {
                    camera->GainRaw.SetValue(static_cast<int64_t>(value));
                } else if (camera->GainAbs.IsWritable()) {
                    camera->GainAbs.SetValue(double(value));
                } else {
                    emit errorOccurred(QString("相机不支持增益设置"));
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置增益失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取增益
         * @details 获取相机的当前增益，若相机未连接或获取失败，会触发错误信号
         * @return 当前增益值
         */
        double gain() const {
            if (!isConnected()) return 0.0;
            try {
                if (camera->Gain.IsReadable()) return camera->Gain.GetValue();
                if (camera->GainRaw.IsReadable()) return camera->GainRaw.GetValue();
                if (camera->GainAbs.IsReadable()) return camera->GainAbs.GetValue();
                return 0.0;
            } catch (...) { return 0.0; }
        }

        std::pair<double, double> gainRange() const {
            if (!isConnected()) return {0.0, 100.0};
            try {
                if (camera->Gain.IsReadable()) return {camera->Gain.GetMin(), camera->Gain.GetMax()};
                if (camera->GainRaw.IsReadable()) return {static_cast<double>(camera->GainRaw.GetMin()), static_cast<double>(camera->GainRaw.GetMax())};
                if (camera->GainAbs.IsReadable()) return {camera->GainAbs.GetMin(), camera->GainAbs.GetMax()};
            } catch (...) {}
            return {0.0, 100.0};
        }

        /**
         * @brief 设置固定帧率
         * @details 设置相机的帧率，若相机未连接或设置失败，会触发错误信号
         * @param value 帧率值，单位为 fps
         */
        void setFrameRate(double value) {
            if (!isConnected()) return;
            try {
                if (camera->AcquisitionFrameRateEnable.IsWritable()) {
                    camera->AcquisitionFrameRateEnable.SetValue(true);
                }
                if (camera->AcquisitionFrameRate.IsWritable()) {
                    camera->AcquisitionFrameRate.SetValue(value);
                } else if (camera->AcquisitionFrameRateAbs.IsWritable()) {
                    // 兼容旧型号
                    camera->AcquisitionFrameRateAbs.SetValue(value);
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置帧率失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取设置的获取帧率
         * @details 获取相机的当前设置的获取帧率，若相机未连接或获取失败，会触发错误信号
         * @return 当前设置帧率值，单位为 fps
         */
        double frameRate() const {
            if (!isConnected()) return 0.0;
            try {
                if (camera->AcquisitionFrameRate.IsReadable()) return camera->AcquisitionFrameRate.GetValue();
                if (camera->AcquisitionFrameRateAbs.IsReadable()) return camera->AcquisitionFrameRateAbs.GetValue();
                return 0.0;
            } catch (...) { return 0.0; }
        }

        /**
         * @brief 设置像素格式
         * @details 设置相机的像素格式，若相机未连接或设置失败，会触发错误信号
         * @param formatName 像素格式名称，例如 "Mono8"、"RGB8" 等
         */
        void setPixelFormat(const String_t & formatName){
            if (!isConnected()) return;
            try {
                if(camera->PixelFormat.IsWritable()){
                    camera->PixelFormat.FromString(formatName);
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置像素格式失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取当前像素格式
         * @details 获取相机的当前像素格式，若相机未连接或获取失败，会触发错误信号
         * @return 当前像素格式名称，例如 "Mono8"、"RGB8" 等
         */
        QString pixelFormat() const {
            if (!isConnected()) return QString();
            try {
                if(camera->PixelFormat.IsReadable()) {
                    return QString(camera->PixelFormat.ToString().c_str());
                }
            } catch (const Pylon::GenericException &e) { 
                qWarning() << QString("读取像素格式失败: %1").arg(e.GetDescription());  
            }
            return QString();
        }

        /**
         * @brief 设置图像镜像
         * @details 设置相机的图像是否镜像显示，若相机未连接或设置失败，会触发错误信号
         * @param flipX 是否镜像 X 轴
         * @param flipY 是否镜像 Y 轴
         */
        void setReverse(bool flipX, bool flipY){
            if (!isConnected()) return;
            try {
                if (camera->ReverseX.IsWritable()) camera->ReverseX.SetValue(flipX);
                if (camera->ReverseY.IsWritable()) camera->ReverseY.SetValue(flipY);
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置镜像失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取当前图像镜像设置
         * @details 获取相机的当前图像是否镜像显示，若相机未连接或获取失败，会触发错误信号
         * @return 当前镜像设置，{是否镜像 X 轴, 是否镜像 Y 轴}
         */
        std::pair<bool, bool> reverse() const {
            if (!isConnected()) return {false, false};
            try {
                if (camera->ReverseX.IsReadable()) return {camera->ReverseX.GetValue(), camera->ReverseY.GetValue()};
                return {false, false};
            } catch (...) { return {false, false}; }
        }

        /**
         * @brief 设置采集模式
         * @details 设置相机的采集模式，若相机未连接或设置失败，会触发错误信号
         * @param selector 触发选择器，例如 "FrameStart"、"Line1" 等
         * @param enable 是否开启触发模式，true 表示开启，false 表示关闭
         * @param source 触发源，例如 "Line0"、"Line1" 等
         */
        void setTriggerConfig(const String_t & selector, bool enable, const String_t & source) {
            if (!isConnected()) return;
            try {
                if (camera->TriggerSelector.IsWritable()) camera->TriggerSelector.FromString(selector);
                if (camera->TriggerMode.IsWritable()) camera->TriggerMode.FromString(enable ? "On" : "Off");
                if (enable && camera->TriggerSource.IsWritable()) camera->TriggerSource.FromString(source);
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置采集模式失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 获取当前触发选择器
         * @details 获取相机的当前触发选择器，若相机未连接或获取失败，会触发错误信号
         * @return 当前触发选择器名称，例如 "FrameStart"、"Line1" 等
         */
        QString triggerSelector() const {
            if (!isConnected()) return QString();
            try {
                if (camera->TriggerSelector.IsReadable()) return QString(camera->TriggerSelector.ToString().c_str());
                return QString();
            } catch (const Pylon::GenericException &e) { 
                qWarning() << QString("读取触发选择器失败: %1").arg(e.GetDescription());  
            }
            return QString();
        }

        /**
         * @brief 获取当前触发模式
         * @details 获取相机的当前触发模式，若相机未连接或获取失败，会触发错误信号
         * @return 当前触发模式是否开启，true 表示开启，即TriggerMode On，false 表示关闭，即TriggerMode Off
         */
        bool triggerMode() const {
            if (!isConnected()) return false;
            try {
                if (camera->TriggerMode.IsReadable()) return camera->TriggerMode.ToString() == "On";
                return false;
            } catch (...) { return false; }
        }

        /**
         * @brief 获取当前触发源
         * @details 获取相机的当前触发源，若相机未连接或获取失败，会触发错误信号
         * @return 当前触发源名称，例如"Software"、"Line0"、"Line1" 等
         */
        QString triggerSource() const {
            if (!isConnected()) return QString();
            try {
                if (camera->TriggerSource.IsReadable()) return QString(camera->TriggerSource.ToString().c_str());
                return QString();
            } catch (const Pylon::GenericException &e) { 
                qWarning() << QString("读取触发源失败: %1").arg(e.GetDescription());  
            }
            return QString();
        }

        /**
         * @brief 执行软件触发
         * @details 执行相机的软件触发，若相机未连接或执行失败，会触发错误信号
         */
        void executeSoftwareTrigger(){
            if (!isConnected()) return;
            try {
                if (camera->TriggerSoftware.IsWritable()) camera->TriggerSoftware.Execute();
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("执行软件触发失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 加载用户集
         * @details 加载相机的用户集，若相机未连接或加载失败，会触发错误信号。本函数调用前应确保相机未在采集状态。
         * @param setName 用户集的名称，例如 "UserSet1"、"UserSet2" 等
         */
        void loadUserSet(const String_t & setName){
            if (!isConnected()) return;
            try {
                if (camera->IsGrabbing()){
                    emit errorOccurred(QString("当前相机正在采集，无法加载用户设置"));
                    return;
                }

                if (camera->UserSetSelector.IsWritable()){
                    camera->UserSetSelector.FromString(setName); //e.g. "UserSet1"
                    if(camera->UserSetLoad.IsWritable()) camera->UserSetLoad.Execute();
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("加载用户设置失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 保存用户集
         * @details 保存相机的用户集，若相机未连接或保存失败，会触发错误信号。
         * @param setName 用户集的名称，例如 "UserSet1"、"UserSet2" 等
         */
        void saveUserSet(const String_t & setName){
            if (!isConnected()) return;
            try {
                if (camera->UserSetSave.IsWritable()){
                    camera->UserSetSelector.FromString(setName); //e.g. "UserSet1"
                    if(camera->UserSetSave.IsWritable()) camera->UserSetSave.Execute();
                    else emit errorOccurred(QString("用户集 %1 只读").arg(setName.c_str()));
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("保存用户集失败: %1").arg(e.GetDescription()));
            }
        }

        /**
         * @brief 设置相机上电时的默认用户集
         * @param setName 用户集的名称
         */
        void setUserSetDefault(const String_t & setName){
            if (!isConnected()) return;
            try {
                if (camera->UserSetDefault.IsWritable()){
                    camera->UserSetDefault.FromString(setName); //e.g. "UserSet1"
                }
            } catch (const Pylon::GenericException &e) {
                emit errorOccurred(QString("设置用户集为默认值失败: %1").arg(e.GetDescription()));
            }
        }

    signals:
        /**
         * @brief 信号，当相机发生错误时触发
         * @details 当相机发生错误时，会触发此信号，传递错误信息。接收端可以连接此信号，在发生错误时进行相应处理（如弹窗提示）。
         * @param msg 错误信息字符串
         */
        void errorOccurred(const QString &msg);
        
        /**
         * @brief 信号，当相机成功采集到图像时触发
         * @details 当相机成功采集到图像时，会触发此信号，传递采集到的图像数据。接收端可以连接此信号，在采集到图像时进行相应处理（如显示图像）。
         * @param grabResult 采集结果智能指针，包含图像数据和元数据
         */
        void imageGrabbed(const Pylon::CGrabResultPtr grabResult);

    protected:
        std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> camera;  ///< 相机对象指针
        mutable QMutex cameraMutex;  ///< 相机对象互斥锁，用于保护相机对象的创建与销毁

        /**
         * @brief 图像采集回调函数
         * @details 当图像采集成功时调用，将采集到的图像数据传递给上层处理（扫描仪）。本函数由 Pylon 框架在子线程中调用。
         * @param camera 相机对象引用
         * @param ptrGrabResult 采集结果指针，包含图像数据和元数据
         */
        virtual void OnImageGrabbed(Pylon::CInstantCamera& x_camera, const Pylon::CGrabResultPtr& ptrGrabResult) override {
            if (ptrGrabResult->GrabSucceeded()) {
                emit imageGrabbed(ptrGrabResult); 
            } else {
                emit errorOccurred(QString("采集错误: %1 %2").arg(ptrGrabResult->GetErrorCode()).arg(ptrGrabResult->GetErrorDescription().c_str()));
                std::cout << "[Debug] Camera::OnImageGrabbed() Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription().c_str() << std::endl;
            }
        }

        /**
         * @brief 相机设备移除回调函数
         * @details 当相机设备被移除时调用，用于处理相机设备移除后的逻辑。
         * @param camera 相机对象引用
         */
        virtual void OnCameraDeviceRemoved(Pylon::CInstantCamera& camera){
            Q_UNUSED(camera);
            emit errorOccurred(QStringLiteral("相机设备断开，请重新连接硬件后再重连"));
            std::cout << ">>> [Debug] Camera::OnCameraDeviceRemoved() -> Camera Device Removed" << std::endl;
        }

        bool tryEnableChunk(INodeMap& nodeMap, const String_t& chunkName) {
            // 获取 ChunkSelector 枚举节点
            GenApi::CEnumerationPtr chunkSelector(nodeMap.GetNode("ChunkSelector"));
            // 获取 ChunkEnable 开关节点
            GenApi::CBooleanPtr chunkEnable(nodeMap.GetNode("ChunkEnable"));

            if (!IsWritable(chunkSelector) || !IsWritable(chunkEnable)) {
                return false;
            }

            // 检查这个名字是否存在于当前相机的枚举列表中
            // 例如：有的相机叫 "ExposureTime"，有的叫 "ExposureTimeAbs"
            // if (chunkSelector->GetEntryByName(chunkName)) {
            //     chunkSelector->FromString(chunkName); // 选中它
            //     chunkEnable->SetValue(true);          // 启用它
            //     qDebug() << "成功启用 Chunk:" << chunkName.c_str();
            //     return true;
            // }
            return false;
        }

        /**
         * @brief 配置相机的 Chunk 模式，Chunk模式指得是相机在采集图像时，同时采集一些其它的信息，比如曝光时间等数据，可以省去额外的同步通信时间损耗
         * @param camera 相机对象指针，指向要配置 Chunk 模式的相机
         */
        void setupCameraChunk(Pylon::CBaslerUniversalInstantCamera* camera) {
            if (!camera || !camera->IsOpen()) return;

            try {
                INodeMap& nodemap = camera->GetNodeMap();

                // 全局开启 Chunk 模式
                if (GenApi::IsWritable(nodemap.GetNode("ChunkModeActive"))) {
                    GenApi::CBooleanPtr(nodemap.GetNode("ChunkModeActive"))->SetValue(true);
                } else {
                    qWarning() << "该相机不支持 Chunk Mode Active";
                    return;
                }

                if (!tryEnableChunk(nodemap, "ExposureTime")) {
                    if (!tryEnableChunk(nodemap, "ExposureTimeAbs")) {
                        qWarning() << "未找到支持的曝光 Chunk (ExposureTime / ExposureTimeAbs)";
                    }
                }
                
                // tryEnableChunk(nodemap, "FrameCounter");

            } catch (const Pylon::GenericException& e) {
                qCritical() << "Chunk 配置异常:" << e.GetDescription();
            }
        }
    };
}}


#endif // CAMERA_H
