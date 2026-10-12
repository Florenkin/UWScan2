#ifndef SCANNERINTERFACE_H
#define SCANNERINTERFACE_H

#include <QWidget>
#include "ui_scannerinterface.h"
#include "utils.h"
#include <QTimer>

NAMESPACE_BEGIN { namespace scanner {
	class ScannerInterface : public QWidget
	{
		Q_OBJECT

	public:
		ScannerInterface(QWidget *parent = nullptr);
		public:
		void setLimitValues(double lower, double upper);
		void setCameraParameterRanges(double exposureMin, double exposureMax, double gainMin, double gainMax);
		bool currentScanRange(float& startAngle, float& endAngle, float& stepAngle) const;
		~ScannerInterface();

	private slots:
		/**
		 * @brief 设置曝光位置的【确认】按钮按下后对应的槽函数
		 */
		void on_applyButton_clicked();

		void on_applyScanExposureButton_clicked();

		/**
		 * @brief 激光的【跳转】按钮按下后对应的槽函数
		 */
		void on_pbSkip_clicked();

		/**
		 * @brief 激光的【设为开始】按钮按下后对应的槽函数
		 */
		void on_startButton_clicked();

		void on_pushButton_laserMoveStart_clicked();

		void on_pushButton_laserMoveMiddle_clicked();

		void on_pushButton_laserMoveEnd_clicked();
		
		/**
		 * @brief 激光的【设为结束】按钮按下后对应的槽函数
		 */
		void on_endButton_clicked();
		
		/**
		 * @brief 开始扫描按钮按下后对应的槽函数
		 */
		void on_pbScan_clicked();

		/**
		 * @brief 停止扫描按钮按下后对应的槽函数
		 */
		void on_pbStopScan_clicked();

		/**
		 * @brief 保存路径选择按钮按下后对应的槽函数
		 */
		void on_pathButton_clicked();

		/**
		 * @brief 单点扫描按下后对应的槽函数
		 */
		void on_singleButton_clicked();

	public slots:
		/**
		 * @brief 曝光时间的【数值】改变后对应的槽函数
		 */
		void on_exposureTimeSpin_valueChanged(int value);
		
		/**
		 * @brief 曝光时间的【滑动条】改变后对应的槽函数
		 */
		void on_exposureTimeSlider_valueChanged(int value);

		/**
		 * @brief 激光开关按钮按下后对应的槽函数
		 */
		void on_openLaserButton_clicked();

		/**
		 * @brief 关闭激光开关按钮按下后对应的槽函数
		 */
		void on_closeLaserButton_clicked();

		/**
		 * @brief 设置限位按钮按下后对应的槽函数
		 */
		void on_limitButton_clicked();

		/**
		 * @brief 亮度的【滑动条】改变后对应的槽函数
		 */
		void on_horizontalSlider_brightness_valueChanged(int value);

		/**
		 * @brief 亮度的【数值框】改变后对应的槽函数
		 */
		void on_spinBox_brightness_valueChanged(int value);

		void on_horizontalSlider_illuminationBrightness_valueChanged(int value);

		void on_spinBox_illuminationBrightness_valueChanged(int value);

		void on_gainSpin_valueChanged(int value);

		void on_gainSlider_valueChanged(int value);

		/**
		 * @brief 校准当前的曝光时间
		 */
		void rectifyExposureTime(double exposureTime);	

		/**
		 * @brief 将部分文本框中的数值设置为默认值
		 */
		void setDefaultValues();

		/**
		 * @brief 保存激光控制和扫描参数，供下次以“上次关闭”行为打开时恢复
		 */
		void saveLastCloseParameters();

		/**
		 * @brief 更新保存路径
		 */
		void updateSavingPath();
		void previewSaveBaseDir(const QString& path);
		void setSaveBaseDir(const QString& path);

		double currentSceneExposureTime() const;

		double currentScanExposureTime() const;

		bool multiExposureEnabled() const;
		bool multiExposureSettings(double& minExposure, double& maxExposure, double& exposureStep) const;

		void updateCurrentPosition(double angle);

		void setRoiValues(int x, int y, int width, int height);
		void setCalibrationFile(const QString& path);

		bool centerPointDisplayEnabled() const;
		bool validAreaDisplayEnabled() const;
		bool lineFitEnabled() const;
		int centerlineMethodIndex() const;
		int extractionThreshold() const;
		int extractionFilterThreshold() const;
		bool horizontalExtractionEnabled() const;
		void updateCenterlineStats(int pointCount, double me, double rmse);
	signals:
		void centerlineDisplayOptionsChanged();
		void calibrationFileChanged(const QString& path);
		void warnOccurred(const QString& msg);
		void infoOccurred(const QString& msg);
		void errorOccurred(const QString& error);
		void changeExposureTime(double exposureTime);
		void changeGain(double gain);
		void changeLaserPosition(float speed, float position);
		void startScan(float startAngle,
					  float endAngle,
					  float step,
					  float speed,
					  const QString& savePath,
					  double sceneExposureTime,
					  double scanExposureTime,
					  bool saveSceneImages,
					  bool saveOriginalImages,
					  bool savePointCloud,
					  bool saveDepthMaps,
					  bool saveCompositeImage,
					  bool saveTexturedPointCloud,
					  const QString& sceneImageFormat,
					  const QString& scanImageFormat,
					  const QString& depthImageFormat,
					  const QString& compositeImageFormat,
					  bool multiExposureEnabled,
					  double multiExposureMin,
					  double multiExposureMax,
					  double multiExposureStep);
		void stopScan();
		void reconstructCurrent();
		void openLaser();
		void closeLaser();
		void changeLimit(float lowerLimit, float upperLimit);
		void setBrightness(int brightness);
		void setIlluminationBrightness(int brightness);
		void roiChanged(int x, int y, int width, int height, bool fullRoi);

	private:
		void updateSceneExposureTime(int value);
		void updateLaserBrightness(int value);
		void updateIlluminationBrightness(int value);
		void setExposureEditorValue(int value);
		int currentExposureEditorValue() const;
		QString currentMonitorPositionText() const;
		bool readRoiValues(int& x, int& y, int& width, int& height) const;
		bool readLimitValues(float& lower, float& upper) const;
		bool validateLaserMoveTarget(float targetAngle);
		Ui::ScannerInterfaceClass ui;
		double currentPosition_ = 0.0;
		bool hasCurrentPosition_ = false;
		double sceneExposureTime_ = 137.0;
		double scanExposureTime_ = 137.0;
		QTimer exposureSettingsSyncTimer_;
	};
}}
#endif // SCANNERINTERFACE_H
