#include "scanner/scannerinterface.h"
#include "setting/settingsmanager.h"
#include <QFileDialog>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSet>
#include <iostream>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QSignalBlocker>
#include <QSlider>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QTimer>
#include <QDateTime>
#include <QLayout>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <filesystem>

NAMESPACE_BEGIN { namespace scanner {
	namespace {
		constexpr float kDirectPositionSpeed = 1.0f;

		QString buildScanSavePath(const QString& baseDir) {
			const QString normalizedBaseDir = QDir::fromNativeSeparators(baseDir.trimmed().isEmpty() ? QDir::homePath() : baseDir.trimmed());
			return QDir(normalizedBaseDir).filePath(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
		}
	}

	ScannerInterface::ScannerInterface(QWidget *parent)
		: QWidget(parent)
	{
		ui.setupUi(this);
		ui.checkBox_lineFit->setText(QStringLiteral("直线拟合"));
		ui.scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		setMinimumSize(0, 0);
		setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
		ui.pathLine->setReadOnly(true);
		QDoubleValidator* doubleValidator = new QDoubleValidator(this);
		doubleValidator->setDecimals(2);  // 保留两位小数

		// 设置输入格式
		ui.startLine->setValidator(doubleValidator);
		ui.endLine->setValidator(doubleValidator);
		ui.incrementLine->setValidator(doubleValidator);

		ui.lineEdit_8->setValidator(doubleValidator);
		ui.lineEdit_3->setValidator(doubleValidator);
		ui.lineEdit_bwThr->setValidator(new QIntValidator(0, 255, this));
		ui.lineEdit_sumThr->setValidator(new QIntValidator(0, 255, this));
		ui.lineEdit_roiX->setValidator(new QIntValidator(0, 100000, this));
		ui.lineEdit_roiY->setValidator(new QIntValidator(0, 100000, this));
		ui.lineEdit_roiWidth->setValidator(new QIntValidator(0, 100000, this));
		ui.lineEdit_roiHeight->setValidator(new QIntValidator(0, 100000, this));
		ui.lineEdit_multiExposureMin->setValidator(doubleValidator);
		ui.lineEdit_multiExposureMax->setValidator(doubleValidator);
		ui.lineEdit_multiExposureStep->setValidator(doubleValidator);
		ui.lineEdit_ME->setReadOnly(true);
		ui.lineEdit_RMSE->setReadOnly(true);
		ui.lineEdit_extractPointCount->setReadOnly(true);
		ui.comboBox_cloudFormat->setEnabled(false);
		ui.radioButton_saveDepth->setText(QStringLiteral("纹理PLY"));
		ui.comboBox_depthFormat->setEnabled(false);
		ui.checkBox_scansave->setChecked(true);
		ui.checkBox_scansavecloud->setChecked(true);
		ui.radioButton_saveScene->setChecked(true);
		ui.radioButton_saveOriginal->setChecked(true);
		ui.radioButton_saveCloud->setChecked(true);
		ui.radioButton_saveDepth->setChecked(true);
		ui.radioButton_saveCompose->setChecked(true);
		ui.radioButton_saveLaserDepth->setChecked(true);
		ui.checkBox_centerPointDisplay->setChecked(false);
		ui.checkBox_validAreaDisplay->setChecked(false);
		ui.checkBox_lineFit->setChecked(false);
		ui.groupBox_multiExposure->setCheckable(true);
		ui.groupBox_multiExposure->setChecked(false);
		exposureSettingsSyncTimer_.setSingleShot(true);
		exposureSettingsSyncTimer_.setInterval(350);
		connect(&exposureSettingsSyncTimer_, &QTimer::timeout, this, [this]() {
			auto* instance = setting::SettingsManager::getInstance();
			instance->setDouble(setting::LAST_CLOSE_EXPOSURETIME, sceneExposureTime_);
			instance->sync();
		});

		auto syncThresholdLineEditToSlider = [](QLineEdit* lineEdit, QSlider* slider, int defaultValue) {
			bool ok = false;
			int value = lineEdit->text().toInt(&ok);
			if (!ok) {
				value = defaultValue;
			}
			value = std::clamp(value, slider->minimum(), slider->maximum());
			{
				QSignalBlocker blocker(slider);
				slider->setValue(value);
			}
			lineEdit->setText(QString::number(value));
		};
		connect(ui.horizontalSlider_bwThr, &QSlider::valueChanged, this, [this](int value) {
			QSignalBlocker blocker(ui.lineEdit_bwThr);
			ui.lineEdit_bwThr->setText(QString::number(value));
		});
		connect(ui.horizontalSlider_sumThr, &QSlider::valueChanged, this, [this](int value) {
			QSignalBlocker blocker(ui.lineEdit_sumThr);
			ui.lineEdit_sumThr->setText(QString::number(value));
		});
		connect(ui.lineEdit_bwThr, &QLineEdit::editingFinished, this, [this, syncThresholdLineEditToSlider]() {
			syncThresholdLineEditToSlider(ui.lineEdit_bwThr, ui.horizontalSlider_bwThr, 55);
			emit centerlineDisplayOptionsChanged();
		});
		connect(ui.lineEdit_sumThr, &QLineEdit::editingFinished, this, [this, syncThresholdLineEditToSlider]() {
			syncThresholdLineEditToSlider(ui.lineEdit_sumThr, ui.horizontalSlider_sumThr, 124);
			emit centerlineDisplayOptionsChanged();
		});
		connect(ui.checkBox_centerPointDisplay, &QCheckBox::toggled, this, &ScannerInterface::centerlineDisplayOptionsChanged);
		connect(ui.checkBox_validAreaDisplay, &QCheckBox::toggled, this, &ScannerInterface::centerlineDisplayOptionsChanged);
		connect(ui.checkBox_lineFit, &QCheckBox::toggled, this, &ScannerInterface::centerlineDisplayOptionsChanged);
		connect(ui.horizontalSlider_bwThr, &QSlider::valueChanged, this, [this](int) {
			emit centerlineDisplayOptionsChanged();
		});
		connect(ui.horizontalSlider_sumThr, &QSlider::valueChanged, this, [this](int) {
			emit centerlineDisplayOptionsChanged();
		});
		connect(ui.comboBox_centerlineMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
			emit centerlineDisplayOptionsChanged();
		});
		connect(ui.checkBox_isHeng, &QCheckBox::toggled, this, &ScannerInterface::centerlineDisplayOptionsChanged);
		connect(ui.comboBox_calibratefile, &QComboBox::currentTextChanged, this, [this](const QString& path) {
			emit calibrationFileChanged(path);
		});

		QStringList filters;
		filters << "*.yml" << "*.yaml";
		const QStringList searchDirs = {
			QCoreApplication::applicationDirPath(),
			QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("docs"),
			QDir::current().absolutePath(),
			QDir::current().absoluteFilePath("docs"),
			QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../docs")
		};
		QSet<QString> addedFiles;
		for (const QString& dirPath : searchDirs) {
			QDir dir(dirPath);
			if (!dir.exists()) {
				continue;
			}
			const QFileInfoList files = dir.entryInfoList(filters, QDir::Files | QDir::Readable, QDir::Name);
			for (const QFileInfo& fileInfo : files) {
				const QString filePath = fileInfo.absoluteFilePath();
				if (addedFiles.contains(filePath)) {
					continue;
				}
				addedFiles.insert(filePath);
				ui.comboBox_calibratefile->addItem(filePath);
			}
		}

		auto setGroupCollapsedState = [](QGroupBox* group, bool expanded) {
			if (!group || !group->layout()) {
				return;
			}
			const int itemCount = group->layout()->count();
			for (int i = 0; i < itemCount; ++i) {
				QLayoutItem* item = group->layout()->itemAt(i);
				if (!item) {
					continue;
				}
				if (QWidget* widget = item->widget()) {
					widget->setVisible(expanded);
				} else if (QLayout* layout = item->layout()) {
					for (int j = 0; j < layout->count(); ++j) {
						QLayoutItem* childItem = layout->itemAt(j);
						if (childItem && childItem->widget()) {
							childItem->widget()->setVisible(expanded);
						}
					}
				}
			}
		};
		auto makeCollapsible = [setGroupCollapsedState](QGroupBox* group) {
			if (!group) {
				return;
			}
			group->setCheckable(true);
			group->setChecked(true);
			setGroupCollapsedState(group, true);
			QObject::connect(group, &QGroupBox::toggled, group, [group, setGroupCollapsedState](bool checked) {
				setGroupCollapsedState(group, checked);
			});
		};
		makeCollapsible(ui.groupBox_extract);
		makeCollapsible(ui.groupBox_3);

		auto applyRoiFromText = [this]() {
			int x = 0;
			int y = 0;
			int width = 0;
			int height = 0;
			if (!readRoiValues(x, y, width, height)) {
				emit warnOccurred(QStringLiteral("警告：请输入正确的 ROI 参数"));
				return;
			}
			emit roiChanged(x, y, width, height, width <= 0 || height <= 0);
		};
		connect(ui.pushButton_roiCapture, &QPushButton::clicked, this, applyRoiFromText);
		connect(ui.pushButton_roiReset, &QPushButton::clicked, this, [this]() {
			setRoiValues(0, 0, 0, 0);
			emit roiChanged(0, 0, 0, 0, true);
		});
		connect(ui.lineEdit_roiX, &QLineEdit::editingFinished, this, applyRoiFromText);
		connect(ui.lineEdit_roiY, &QLineEdit::editingFinished, this, applyRoiFromText);
		connect(ui.lineEdit_roiWidth, &QLineEdit::editingFinished, this, applyRoiFromText);
		connect(ui.lineEdit_roiHeight, &QLineEdit::editingFinished, this, applyRoiFromText);
	}

	ScannerInterface::~ScannerInterface(){}

	void ScannerInterface::setLimitValues(double lower, double upper)
	{
		ui.lowerLimitLine->setText(QString::number(lower));
		ui.upperLimitLine->setText(QString::number(upper));
	}

	void ScannerInterface::setCameraParameterRanges(double exposureMin, double exposureMax, double gainMin, double gainMax)
	{
		const int minExposure = std::max(0, static_cast<int>(std::floor(exposureMin)));
		const int maxExposure = std::max(minExposure, static_cast<int>(std::ceil(exposureMax)));
		const int minGain = std::max(0, static_cast<int>(std::floor(gainMin)));
		const int maxGain = std::max(minGain, static_cast<int>(std::ceil(gainMax)));

		{
			QSignalBlocker spinBlocker(ui.exposureTimeSpin);
			QSignalBlocker sliderBlocker(ui.exposureTimeSlider);
			ui.exposureTimeSpin->setRange(minExposure, maxExposure);
			ui.exposureTimeSlider->setRange(minExposure, maxExposure);
		}
		{
			QSignalBlocker spinBlocker(ui.gainSpin);
			QSignalBlocker sliderBlocker(ui.gainSlider);
			ui.gainSpin->setRange(minGain, maxGain);
			ui.gainSlider->setRange(minGain, maxGain);
		}
	}

	bool ScannerInterface::currentScanRange(float& startAngle, float& endAngle, float& stepAngle) const
	{
		bool ok = false;
		startAngle = ui.startLine->text().toFloat(&ok);
		if (!ok) {
			return false;
		}
		endAngle = ui.endLine->text().toFloat(&ok);
		if (!ok) {
			return false;
		}
		stepAngle = ui.incrementLine->text().toFloat(&ok);
		return ok && !qFuzzyCompare(startAngle, endAngle) && stepAngle > 0.0f;
	}

	void ScannerInterface::on_applyButton_clicked() {
		updateSceneExposureTime(currentExposureEditorValue());
		exposureSettingsSyncTimer_.stop();
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		instance->setDouble(setting::LAST_CLOSE_EXPOSURETIME, sceneExposureTime_);
		instance->sync();
		emit infoOccurred(QStringLiteral("场景曝光已设置为 %1").arg(sceneExposureTime_, 0, 'f', 0));
	}

	void ScannerInterface::on_applyScanExposureButton_clicked() {
		scanExposureTime_ = static_cast<double>(currentExposureEditorValue());
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		instance->setDouble(setting::DEFAULT_SCAN_EXPOSURETIME, scanExposureTime_);
		instance->sync();
		emit infoOccurred(QStringLiteral("扫描曝光已设置为 %1").arg(scanExposureTime_, 0, 'f', 0));
	}


	void ScannerInterface::on_pbSkip_clicked() {
		bool ok = false;
		double angle = ui.lineEdit_8->text().toDouble(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的角度值"));
			return;
		}
		if (!validateLaserMoveTarget(static_cast<float>(angle))) {
			return;
		}
		emit changeLaserPosition(kDirectPositionSpeed, angle);
	}

	void ScannerInterface::on_startButton_clicked() {
		ui.startLine->setText(currentMonitorPositionText());
	}

	void ScannerInterface::on_endButton_clicked() {
		ui.endLine->setText(currentMonitorPositionText());
	}

	void ScannerInterface::on_pushButton_laserMoveStart_clicked() {
		bool ok = false;
		const float startAngle = ui.startLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的开始角度值"));
			return;
		}
		if (!validateLaserMoveTarget(startAngle)) {
			return;
		}
		emit changeLaserPosition(kDirectPositionSpeed, startAngle);
	}

	void ScannerInterface::on_pushButton_laserMoveMiddle_clicked() {
		bool ok = false;
		const float startAngle = ui.startLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的开始角度值"));
			return;
		}
		const float endAngle = ui.endLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的结束角度值"));
			return;
		}
		const float targetAngle = (startAngle + endAngle) / 2.0f;
		if (!validateLaserMoveTarget(targetAngle)) {
			return;
		}
		emit changeLaserPosition(kDirectPositionSpeed, targetAngle);
	}

	void ScannerInterface::on_pushButton_laserMoveEnd_clicked() {
		bool ok = false;
		const float endAngle = ui.endLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的结束角度值"));
			return;
		}
		if (!validateLaserMoveTarget(endAngle)) {
			return;
		}
		emit changeLaserPosition(kDirectPositionSpeed, endAngle);
	}

	void ScannerInterface::on_pbScan_clicked() {
		ui.pbScan->setEnabled(false);
		float speed, startAngle, endAngle, StepAngle;
		bool ok = false;
		speed = ui.lineEdit_3->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的扫描速度值"));
			ui.pbScan->setEnabled(true);
			return;
		}
		if (speed <= 0.0f) {
			emit warnOccurred(QStringLiteral("警告：扫描速度必须大于 0"));
			ui.pbScan->setEnabled(true);
			return;
		}

		startAngle = ui.startLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的开始角度值"));
			ui.pbScan->setEnabled(true);
			return;
		}
		endAngle = ui.endLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的结束角度值"));
			ui.pbScan->setEnabled(true);
			return;
		}
		StepAngle = ui.incrementLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的扫描步长值"));
			ui.pbScan->setEnabled(true);
			return;
		}
		if (qFuzzyCompare(startAngle, endAngle) || StepAngle <= 0.0f) {
			emit warnOccurred(QStringLiteral("警告：开始角度不能等于结束角度，扫描步长必须大于 0"));
			ui.pbScan->setEnabled(true);
			return;
		}
		const float signedStepAngle = startAngle < endAngle ? std::abs(StepAngle) : -std::abs(StepAngle);

		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		auto path = ui.pathLine->text().toStdString();
		if(!std::filesystem::exists(path)){
			std::filesystem::create_directories(path);
		}
		instance->setString(setting::LAST_CLOSE_START_ANGLE, ui.startLine->text());
		instance->setString(setting::LAST_CLOSE_END_ANGLE, ui.endLine->text());
		instance->setString(setting::LAST_CLOSE_STEP_ANGLE, ui.incrementLine->text());
		instance->setDouble(setting::LAST_CLOSE_EXPOSURETIME, sceneExposureTime_);
		instance->setDouble(setting::DEFAULT_SCAN_EXPOSURETIME, scanExposureTime_);
		instance->setString(setting::DEFAULT_CENTERLINE_METHOD, ui.comboBox_centerlineMethod->currentText());
		instance->setInt(setting::DEFAULT_BW_THR, ui.lineEdit_bwThr->text().toInt());
		instance->setInt(setting::DEFAULT_SUM_THR, ui.lineEdit_sumThr->text().toInt());
		instance->setBool(setting::DEFAULT_IS_HENG, ui.checkBox_isHeng->isChecked());
		instance->setString(setting::DEFAULT_CALIBRATE_FILE, ui.comboBox_calibratefile->currentText());
		instance->setBool(setting::DEFAULT_SCAN_SAVE, ui.checkBox_scansave->isChecked());
		instance->setBool(setting::DEFAULT_SCAN_CLOUD, ui.checkBox_scansavecloud->isChecked());
		instance->sync();

		double multiExposureMin = 0.0;
		double multiExposureMax = 0.0;
		double multiExposureStep = 0.0;
		if (multiExposureEnabled() && !multiExposureSettings(multiExposureMin, multiExposureMax, multiExposureStep)) {
			emit warnOccurred(QStringLiteral("警告：多重曝光参数无效，请检查开始曝光、结束曝光和曝光间隔"));
			ui.pbScan->setEnabled(true);
			return;
		}
		
		emit startScan(startAngle,
			endAngle,
			signedStepAngle,
			speed,
			ui.pathLine->text(),
			currentSceneExposureTime(),
			currentScanExposureTime(),
			ui.checkBox_scansave->isChecked() && ui.radioButton_saveScene->isChecked(),
			ui.checkBox_scansave->isChecked() && ui.radioButton_saveOriginal->isChecked(),
			ui.checkBox_scansavecloud->isChecked() && ui.radioButton_saveCloud->isChecked(),
			ui.checkBox_scansavecloud->isChecked() && ui.radioButton_saveLaserDepth->isChecked(),
			ui.checkBox_scansavecloud->isChecked() && ui.radioButton_saveCompose->isChecked(),
			ui.checkBox_scansavecloud->isChecked() && ui.radioButton_saveDepth->isChecked(),
			ui.comboBox_sceneFormat->currentText(),
			ui.comboBox_originalFormat->currentText(),
			ui.comboBox_laserDepthFormat->currentText(),
			ui.comboBox_composeFormat->currentText(),
			multiExposureEnabled(),
			multiExposureMin,
			multiExposureMax,
			multiExposureStep);
		QTimer::singleShot(150, this, [this]() {
			ui.pbScan->setEnabled(true);
		});
	}

	void ScannerInterface::on_pbStopScan_clicked() {
		emit stopScan();
	}

	void ScannerInterface::on_pathButton_clicked() {
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		QString path = QFileDialog::getExistingDirectory(this, tr("选择保存路径"), instance->getString(setting::SAVE_IMAGE_DIR, QDir::homePath()), \
							QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

		if (!path.isEmpty()) {
			setSaveBaseDir(path);
		}
	}

	void ScannerInterface::on_singleButton_clicked(){
		emit reconstructCurrent();
	}

	void ScannerInterface::on_exposureTimeSpin_valueChanged(int value) {
		ui.exposureTimeSlider->blockSignals(true);
		ui.exposureTimeSlider->setValue(value);
		ui.exposureTimeSlider->blockSignals(false);
		updateSceneExposureTime(value);
	}

	void ScannerInterface::on_exposureTimeSlider_valueChanged(int value) {
		ui.exposureTimeSpin->blockSignals(true);
		ui.exposureTimeSpin->setValue(value);
		ui.exposureTimeSpin->blockSignals(false);
		updateSceneExposureTime(value);
	}

	void ScannerInterface::on_openLaserButton_clicked(){
		emit openLaser();
	}

	void ScannerInterface::on_closeLaserButton_clicked(){
		emit closeLaser();
	}

	void ScannerInterface::on_limitButton_clicked(){
		float num1, num2;
		bool ok = false;
		num1 = ui.lowerLimitLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的下限角度值"));
			return;
		}
		num2 = ui.upperLimitLine->text().toFloat(&ok);
		if (!ok) {
			emit warnOccurred(QStringLiteral("警告：请输入正确的上限角度值"));
			return;
		}
		if(num1 >= num2) {
			std::swap(num1, num2);
		}
		
		emit changeLimit(num1, num2);
	}

	void ScannerInterface::on_horizontalSlider_brightness_valueChanged(int value){
		updateLaserBrightness(value);
	}

	void ScannerInterface::on_spinBox_brightness_valueChanged(int value){
		updateLaserBrightness(value);
	}

	void ScannerInterface::on_horizontalSlider_illuminationBrightness_valueChanged(int value) {
		updateIlluminationBrightness(value);
	}

	void ScannerInterface::on_spinBox_illuminationBrightness_valueChanged(int value) {
		updateIlluminationBrightness(value);
	}

	void ScannerInterface::on_gainSpin_valueChanged(int value) {
		if (ui.gainSlider->value() == value) return;
		ui.gainSlider->blockSignals(true);
		ui.gainSlider->setValue(value);
		ui.gainSlider->blockSignals(false);
		emit changeGain(static_cast<double>(value));
	}

	void ScannerInterface::on_gainSlider_valueChanged(int value) {
		if (ui.gainSpin->value() == value) return;
		ui.gainSpin->blockSignals(true);
		ui.gainSpin->setValue(value);
		ui.gainSpin->blockSignals(false);
		emit changeGain(static_cast<double>(value));
	}

	void ScannerInterface::rectifyExposureTime(double exposureTime){
		sceneExposureTime_ = exposureTime;
		setExposureEditorValue(static_cast<int>(std::lround(exposureTime)));
	}

	void ScannerInterface::updateSceneExposureTime(int value) {
		sceneExposureTime_ = static_cast<double>(value);
		emit changeExposureTime(sceneExposureTime_);
		exposureSettingsSyncTimer_.start();
	}

	void ScannerInterface::updateLaserBrightness(int value) {
		if (ui.spinBox_brightness->value() != value) {
			QSignalBlocker blocker(ui.spinBox_brightness);
			ui.spinBox_brightness->setValue(value);
		}
		if (ui.horizontalSlider_brightness->value() != value) {
			QSignalBlocker blocker(ui.horizontalSlider_brightness);
			ui.horizontalSlider_brightness->setValue(value);
		}
		ui.label_laserBrightnessValue->setText(QStringLiteral("%1%").arg(value));
		emit setBrightness(value);
	}

	void ScannerInterface::updateIlluminationBrightness(int value) {
		if (ui.spinBox_illuminationBrightness->value() != value) {
			QSignalBlocker blocker(ui.spinBox_illuminationBrightness);
			ui.spinBox_illuminationBrightness->setValue(value);
		}
		if (ui.horizontalSlider_illuminationBrightness->value() != value) {
			QSignalBlocker blocker(ui.horizontalSlider_illuminationBrightness);
			ui.horizontalSlider_illuminationBrightness->setValue(value);
		}
		ui.label_illuminationBrightnessValue->setText(QStringLiteral("%1%").arg(value));
		emit setIlluminationBrightness(value);
	}

	void ScannerInterface::setExposureEditorValue(int value) {
		const int clampedValue = std::clamp(value, ui.exposureTimeSpin->minimum(), ui.exposureTimeSpin->maximum());
		{
			QSignalBlocker spinBlocker(ui.exposureTimeSpin);
			ui.exposureTimeSpin->setValue(clampedValue);
		}
		{
			QSignalBlocker sliderBlocker(ui.exposureTimeSlider);
			ui.exposureTimeSlider->setValue(clampedValue);
		}
	}

	int ScannerInterface::currentExposureEditorValue() const {
		return ui.exposureTimeSpin->value();
	}

	void ScannerInterface::setDefaultValues(){
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		std::cout << "Opening behaviour: " << instance->getString(setting::SCANNER_OPENING_BEHAVIOUR).toStdString() << std::endl;
		updateSavingPath();
		const double currentExposure = currentSceneExposureTime();
		scanExposureTime_ = instance->getDouble(setting::DEFAULT_SCAN_EXPOSURETIME, currentExposure > 0.0 ? currentExposure : 137.0);
		ui.checkBox_scansave->setChecked(instance->getBool(setting::DEFAULT_SCAN_SAVE, true));
		ui.checkBox_scansavecloud->setChecked(instance->getBool(setting::DEFAULT_SCAN_CLOUD, true));
		ui.comboBox_centerlineMethod->setCurrentText(instance->getString(setting::DEFAULT_CENTERLINE_METHOD, QStringLiteral("灰度重心法")));
		ui.lineEdit_bwThr->setText(QString::number(instance->getInt(setting::DEFAULT_BW_THR, 55)));
		ui.lineEdit_sumThr->setText(QString::number(instance->getInt(setting::DEFAULT_SUM_THR, 124)));
		ui.horizontalSlider_bwThr->setValue(instance->getInt(setting::DEFAULT_BW_THR, 55));
		ui.horizontalSlider_sumThr->setValue(instance->getInt(setting::DEFAULT_SUM_THR, 124));
		ui.checkBox_isHeng->setChecked(instance->getBool(setting::DEFAULT_IS_HENG, true));
		const QString calibrationFile = instance->getString(setting::DEFAULT_CALIBRATE_FILE);
		if (!calibrationFile.isEmpty()) {
			if (ui.comboBox_calibratefile->findText(calibrationFile) < 0) {
				ui.comboBox_calibratefile->addItem(calibrationFile);
			}
			ui.comboBox_calibratefile->setCurrentText(calibrationFile);
		}
		ui.comboBox_cloudFormat->setCurrentText("txt");
		ui.comboBox_cloudFormat->setEnabled(false);

		if(instance->getString(setting::SCANNER_OPENING_BEHAVIOUR) == setting::SCANNER_OPENING_BEHAVIOUR_NONE) return;
		else if(instance->getString(setting::SCANNER_OPENING_BEHAVIOUR) == setting::SCANNER_OPENING_BEHAVIOUR_DEFAULT) {
			double motorLimitUpper = instance->getDouble(setting::DEFAULT_MOTOR_LIMIT_UPPER, 42.0);
			double motorLimitLower = instance->getDouble(setting::DEFAULT_MOTOR_LIMIT_LOWER, 57.0);
			if(motorLimitLower >= motorLimitUpper) std::swap(motorLimitLower, motorLimitUpper);
			if(motorLimitLower < -100) motorLimitLower = -20.0;
			if(motorLimitUpper > 100.0) motorLimitUpper = 20.0;

			ui.lowerLimitLine->setText(QString::number(motorLimitLower));
			ui.upperLimitLine->setText(QString::number(motorLimitUpper));
			on_limitButton_clicked();

			ui.startLine->setText(instance->getString(setting::DEFAULT_START_ANGLE));
			ui.endLine->setText(instance->getString(setting::DEFAULT_END_ANGLE));
			ui.incrementLine->setText(instance->getString(setting::DEFAULT_STEP_ANGLE));
		} else if(instance->getString(setting::SCANNER_OPENING_BEHAVIOUR) == setting::SCANNER_OPENING_BEHAVIOUR_LAST_CLOSE) {
			double exposureTime = instance->getDouble(setting::LAST_CLOSE_EXPOSURETIME, 100.0);
			double gain = instance->getDouble(setting::LAST_CLOSE_GAIN, 1.0);
			double motorLimitUpper = instance->getDouble(setting::LAST_CLOSE_MOTOR_LIMIT_UPPER, 42.0);
			double motorLimitLower = instance->getDouble(setting::LAST_CLOSE_MOTOR_LIMIT_LOWER, 57.0);
			if(motorLimitLower >= motorLimitUpper) std::swap(motorLimitLower, motorLimitUpper);
			// if(motorLimitLower < 42.0) motorLimitLower = 42.0;
			// if(motorLimitUpper > 57.0) motorLimitUpper = 57.0;

			std::cout << "Read LAST_CLOSE limits: lower=" << motorLimitLower << " upper=" << motorLimitUpper << std::endl;
			ui.lowerLimitLine->setText(QString::number(motorLimitLower));
			ui.upperLimitLine->setText(QString::number(motorLimitUpper));
			on_limitButton_clicked();

			ui.startLine->setText(instance->getString(setting::LAST_CLOSE_START_ANGLE, ui.startLine->text()));
			ui.endLine->setText(instance->getString(setting::LAST_CLOSE_END_ANGLE, ui.endLine->text()));
			ui.incrementLine->setText(instance->getString(setting::LAST_CLOSE_STEP_ANGLE, ui.incrementLine->text()));

			const double savedScanSpeed = instance->getDouble(
				setting::LAST_CLOSE_SCAN_SPEED, ui.lineEdit_3->text().toDouble());
			if (std::isfinite(savedScanSpeed) && savedScanSpeed > 0.0) {
				ui.lineEdit_3->setText(QString::number(savedScanSpeed));
			}

			const int savedBrightness = instance->getInt(
				setting::LAST_CLOSE_LASER_BRIGHTNESS, ui.spinBox_brightness->value());
			updateLaserBrightness(std::clamp(
				savedBrightness, ui.spinBox_brightness->minimum(), ui.spinBox_brightness->maximum()));
		}
	}

	void ScannerInterface::saveLastCloseParameters() {
		auto* instance = setting::SettingsManager::getInstance();
		const auto saveValidNumber = [instance](const QString& key, const QString& text) {
			bool ok = false;
			const double value = text.toDouble(&ok);
			if (ok && std::isfinite(value)) {
				instance->setString(key, text);
			}
		};

		saveValidNumber(setting::LAST_CLOSE_START_ANGLE, ui.startLine->text());
		saveValidNumber(setting::LAST_CLOSE_END_ANGLE, ui.endLine->text());
		saveValidNumber(setting::LAST_CLOSE_STEP_ANGLE, ui.incrementLine->text());

		bool speedOk = false;
		const double scanSpeed = ui.lineEdit_3->text().toDouble(&speedOk);
		if (speedOk && std::isfinite(scanSpeed) && scanSpeed > 0.0) {
			instance->setDouble(setting::LAST_CLOSE_SCAN_SPEED, scanSpeed);
		}

		instance->setInt(setting::LAST_CLOSE_LASER_BRIGHTNESS, ui.spinBox_brightness->value());

		float lower = 0.0f;
		float upper = 0.0f;
		if (readLimitValues(lower, upper)) {
			instance->setDouble(setting::LAST_CLOSE_MOTOR_LIMIT_LOWER, lower);
			instance->setDouble(setting::LAST_CLOSE_MOTOR_LIMIT_UPPER, upper);
		}

		instance->sync();
	}

	void ScannerInterface::updateSavingPath(){
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		previewSaveBaseDir(instance->getString(setting::SAVE_IMAGE_DIR, QDir::homePath()));
	}

	void ScannerInterface::previewSaveBaseDir(const QString& path) {
		ui.pathLine->setText(buildScanSavePath(path));
	}

	void ScannerInterface::setSaveBaseDir(const QString& path) {
		const QString trimmedPath = path.trimmed();
		if (trimmedPath.isEmpty()) {
			return;
		}
		setting::SettingsManager * instance = setting::SettingsManager::getInstance();
		instance->setString(setting::SAVE_IMAGE_DIR, trimmedPath);
		instance->sync();
		previewSaveBaseDir(trimmedPath);
	}

	double ScannerInterface::currentSceneExposureTime() const {
		return sceneExposureTime_;
	}

	double ScannerInterface::currentScanExposureTime() const {
		return scanExposureTime_;
	}

	bool ScannerInterface::multiExposureEnabled() const {
		return ui.groupBox_multiExposure->isChecked();
	}

	bool ScannerInterface::multiExposureSettings(double& minExposure, double& maxExposure, double& exposureStep) const {
		bool ok = false;
		minExposure = ui.lineEdit_multiExposureMin->text().toDouble(&ok);
		if (!ok) {
			return false;
		}
		maxExposure = ui.lineEdit_multiExposureMax->text().toDouble(&ok);
		if (!ok) {
			return false;
		}
		exposureStep = ui.lineEdit_multiExposureStep->text().toDouble(&ok);
		if (!ok) {
			return false;
		}
		return exposureStep > 0.0 && minExposure <= maxExposure;
	}

	void ScannerInterface::updateCurrentPosition(double angle) {
		currentPosition_ = angle;
		hasCurrentPosition_ = true;
	}

	void ScannerInterface::setRoiValues(int x, int y, int width, int height) {
		ui.lineEdit_roiX->setText(QString::number(std::max(0, x)));
		ui.lineEdit_roiY->setText(QString::number(std::max(0, y)));
		ui.lineEdit_roiWidth->setText(QString::number(std::max(0, width)));
		ui.lineEdit_roiHeight->setText(QString::number(std::max(0, height)));
	}

	void ScannerInterface::setCalibrationFile(const QString& path) {
		if (path.trimmed().isEmpty()) {
			return;
		}
		if (ui.comboBox_calibratefile->findText(path) < 0) {
			ui.comboBox_calibratefile->addItem(path);
		}
		ui.comboBox_calibratefile->setCurrentText(path);
	}

	bool ScannerInterface::centerPointDisplayEnabled() const {
		return ui.checkBox_centerPointDisplay->isChecked();
	}

	bool ScannerInterface::validAreaDisplayEnabled() const {
		return ui.checkBox_validAreaDisplay->isChecked();
	}

	bool ScannerInterface::lineFitEnabled() const {
		return ui.checkBox_lineFit->isChecked();
	}

	int ScannerInterface::centerlineMethodIndex() const {
		return ui.comboBox_centerlineMethod->currentIndex();
	}

	int ScannerInterface::extractionThreshold() const {
		bool ok = false;
		int value = ui.lineEdit_bwThr->text().toInt(&ok);
		return std::clamp(ok ? value : 55, 0, 255);
	}

	int ScannerInterface::extractionFilterThreshold() const {
		bool ok = false;
		int value = ui.lineEdit_sumThr->text().toInt(&ok);
		return std::clamp(ok ? value : 124, 0, 255);
	}

	bool ScannerInterface::horizontalExtractionEnabled() const {
		return ui.checkBox_isHeng->isChecked();
	}

	void ScannerInterface::updateCenterlineStats(int pointCount, double me, double rmse) {
		ui.lineEdit_extractPointCount->setText(QString::number(pointCount));
		if (!lineFitEnabled()) {
			ui.lineEdit_ME->clear();
			ui.lineEdit_RMSE->clear();
			return;
		}
		ui.lineEdit_ME->setText(std::isfinite(me) ? QString::number(me, 'f', 4) : QString());
		ui.lineEdit_RMSE->setText(std::isfinite(rmse) ? QString::number(rmse, 'f', 4) : QString());
	}

	QString ScannerInterface::currentMonitorPositionText() const {
		if (hasCurrentPosition_) {
			return QString::number(currentPosition_, 'f', 2);
		}
		return ui.lineEdit_8->text();
	}

	bool ScannerInterface::readLimitValues(float& lower, float& upper) const {
		bool okLower = false;
		bool okUpper = false;
		lower = ui.lowerLimitLine->text().toFloat(&okLower);
		upper = ui.upperLimitLine->text().toFloat(&okUpper);
		if (!okLower || !okUpper) {
			return false;
		}
		if (lower > upper) {
			std::swap(lower, upper);
		}
		return true;
	}

	bool ScannerInterface::validateLaserMoveTarget(float targetAngle) {
		float lower = 0.0f;
		float upper = 0.0f;
		if (!readLimitValues(lower, upper)) {
			emit warnOccurred(QStringLiteral("请先设置正确的激光限位"));
			return false;
		}
		if (targetAngle < lower || targetAngle > upper) {
			emit warnOccurred(
				QStringLiteral("目标角度 %1 超出限位范围 [%2, %3]")
					.arg(targetAngle, 0, 'f', 2)
					.arg(lower, 0, 'f', 2)
					.arg(upper, 0, 'f', 2));
			return false;
		}
		return true;
	}

	bool ScannerInterface::readRoiValues(int& x, int& y, int& width, int& height) const {
		bool ok = false;
		x = ui.lineEdit_roiX->text().toInt(&ok);
		if (!ok) return false;
		y = ui.lineEdit_roiY->text().toInt(&ok);
		if (!ok) return false;
		width = ui.lineEdit_roiWidth->text().toInt(&ok);
		if (!ok) return false;
		height = ui.lineEdit_roiHeight->text().toInt(&ok);
		return ok && x >= 0 && y >= 0 && width >= 0 && height >= 0;
	}
}}
