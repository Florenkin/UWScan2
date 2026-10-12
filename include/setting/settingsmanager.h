#ifndef SETTING_SETTINGSMANAGER_H
#define SETTING_SETTINGSMANAGER_H
#include <QSettings>
#include <QMutex>
#include "utils.h"

NAMESPACE_BEGIN{ namespace setting{
    // 一些重要的配置名称
    static const QString AUTO_CONNECT_TO_SCANNER = QStringLiteral("AutoConnectToScanner"); ///< 开启时是否自动连接到扫描仪
    static const QString AUTO_OPEN_LASER = QStringLiteral("AutoOpenLaser"); ///< 连接成功时是否自动打开激光器
    static const QString SCANNER_OPENING_BEHAVIOUR = QStringLiteral("ScannerOpeningBehaviour"); ///< 连接成功时扫描仪的打开行为
    static const QString SCANNER_OPENING_BEHAVIOUR_DEFAULT = QStringLiteral("Default"); ///< 默认打开行为，即连接成功后调整到默认配置
    static const QString SCANNER_OPENING_BEHAVIOUR_LAST_CLOSE = QStringLiteral("LastClose"); ///< 上次关闭行为，即上次关闭时的参数
    static const QString SCANNER_OPENING_BEHAVIOUR_NONE = QStringLiteral("None"); ///< 不做任何操作，扫描仪是多少就是多少
    static const QString SCANNER_HOST = QStringLiteral("ScannerHost"); ///< 光机 IP 或主机名
    static const QString SCANNER_PORT = QStringLiteral("ScannerPort"); ///< 扫描仪端口号
    static const QString SAVE_IMAGE_DIR = QStringLiteral("SaveImageDir"); ///< 扫描时保存图片的目录
    static const QString SAVE_CLOUD_DIR = QStringLiteral("SaveCloudDir"); ///< 扫描时保存点云的目录
    static const QString SAVE_SCANNER_DIR = QStringLiteral("SaveScannerDir"); ///< 扫描时保存扫描仪参数的目录
    static const QString DEFAULT_EXPOSURETIME = QStringLiteral("DefaultExposureTime"); ///< 默认曝光时间
    static const QString DEFAULT_GAIN = QStringLiteral("DefaultGain"); ///< 默认增益
    static const QString DEFAULT_MOTOR_LIMIT_UPPER = QStringLiteral("DefaultMotorLimitUpper"); ///< 默认电机限位上界
    static const QString DEFAULT_MOTOR_LIMIT_LOWER = QStringLiteral("DefaultMotorLimitLower"); ///< 默认电机限位下界

    static const QString DEFAULT_START_ANGLE = QStringLiteral("DefaultStartAngle"); ///< 默认扫描起始角度
    static const QString DEFAULT_END_ANGLE = QStringLiteral("DefaultEndAngle"); ///< 默认扫描终止角度
    static const QString DEFAULT_STEP_ANGLE = QStringLiteral("DefaultStepAngle"); ///< 默认扫描角度步长
    static const QString DEFAULT_SCAN_EXPOSURETIME = QStringLiteral("DefaultScanExposureTime"); ///< 默认扫描曝光时间
    static const QString DEFAULT_LINE = QStringLiteral("DefaultLine"); ///< 默认扫描线数
    static const QString DEFAULT_CENTERLINE_METHOD = QStringLiteral("DefaultCenterlineMethod"); ///< 默认中心线提取方法
    static const QString DEFAULT_BW_THR = QStringLiteral("DefaultBwThr"); ///< 默认二值化阈值
    static const QString DEFAULT_SUM_THR = QStringLiteral("DefaultSumThr"); ///< 默认筛选阈值
    static const QString DEFAULT_IS_HENG = QStringLiteral("DefaultIsHeng"); ///< 默认是否横向提取

    static const QString DEFAULT_SCAN_SAVE = QStringLiteral("DefaultScanSave"); ///< 默认扫描时是否保存扫描得到的图片
    static const QString DEFAULT_SCAN_CLOUD = QStringLiteral("DefaultScanCloud"); ///< 默认扫描时是否保存扫描得到的点云
    static const QString DEFAULT_CALIBRATE_FILE = QStringLiteral("DefaultCalibrateFile"); ///< 默认标定文件

    static const QString LAST_CLOSE_EXPOSURETIME = QStringLiteral("LastCloseExposureTime"); ///< 上次关闭时的曝光时间
    static const QString LAST_CLOSE_GAIN = QStringLiteral("LastCloseGain"); ///< 上次关闭时的增益
    static const QString LAST_CLOSE_MOTOR_LIMIT_UPPER = QStringLiteral("LastCloseMotorLimitUpper"); ///< 上次关闭时的电机限位上界
    static const QString LAST_CLOSE_MOTOR_LIMIT_LOWER = QStringLiteral("LastCloseMotorLimitLower"); ///< 上次关闭时的电机限位下界
    static const QString LAST_CLOSE_START_ANGLE = QStringLiteral("LastCloseStartAngle"); ///< 上次关闭时的扫描起始角度
    static const QString LAST_CLOSE_END_ANGLE = QStringLiteral("LastCloseEndAngle"); ///< 上次关闭时的扫描终止角度
    static const QString LAST_CLOSE_STEP_ANGLE = QStringLiteral("LastCloseStepAngle"); ///< 上次关闭时的扫描角度步长
    static const QString LAST_CLOSE_SCAN_SPEED = QStringLiteral("LastCloseScanSpeed"); ///< 上次关闭时的扫描移动速度
    static const QString LAST_CLOSE_LASER_BRIGHTNESS = QStringLiteral("LastCloseLaserBrightness"); ///< 上次关闭时的激光亮度
    static const QString LAST_CLOSE_LINE = QStringLiteral("LastCloseLine"); ///< 上次关闭时的扫描线号

    class SettingsManager: public QObject {
        Q_OBJECT
    public:
        ~SettingsManager() = default;

        /**
         * @brief SettingsManager设计为单例类，确保全局只有一个实例
         * @return SettingsManager* 返回该类的单例实例
         */
        static SettingsManager * getInstance();

        QVariant get(const QString& key, const QVariant& defaultValue) const;

        void set(const QString& key, const QVariant& value);

        bool contains(const QString& key) const;

        void sync() {
        QMutexLocker locker(&mutex);
        settings.sync();
    }

        inline QString getString(const QString& key, const QString& defaultValue = "") {
            auto value = get(key, defaultValue);
            return value.toString();
        }
        
        inline int getInt(const QString& key, int defaultValue = 0) {
            auto value = get(key, defaultValue);
            return value.toInt();
        }
        
        inline double getDouble(const QString& key, double defaultValue = 0.0) {
            auto value = get(key, defaultValue);
            return value.toDouble();
        }
        
        inline bool getBool(const QString& key, bool defaultValue = false) {
            auto value = get(key, defaultValue);
            return value.toBool();
        }
        
        inline void setString(const QString& key, const QString& value) {
            set(key, value);
        }
        
        inline void setInt(const QString& key, int value) {
            set(key, value);
        }
        
        inline void setDouble(const QString& key, double value) {
            set(key, value);
        }
        
        inline void setBool(const QString& key, bool value) {
            set(key, value);
        }

    signals:
        /**
         * @brief 信号，当设置项改变时触发
         * @param key 键名
         * @param value 值
         */
        void settingChanged(const QString& key, const QVariant& value);
    private:
        SettingsManager(QObject *parent = nullptr);
        
        Q_DISABLE_COPY(SettingsManager)

        QSettings settings;
        mutable QMutex mutex;
    };
}}

#endif
