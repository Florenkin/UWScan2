#include "setting/settingwidget.h"
#include "setting/settingsmanager.h"
#include <algorithm>
#include <QFileDialog>
#include <QIcon>
#include <QIntValidator>
#include <QMessageBox>
#include <QSet>
#include <QSignalBlocker>
#include <QSlider>

NAMESPACE_BEGIN{ namespace setting{
	SettingWidget::SettingWidget(QWidget *parent): QDialog(parent){
		ui.setupUi(this);
		this->setWindowTitle("系统设置");
		this->setWindowIcon(QIcon(":/icon/Wuhan_Fuel.ico"));

		SettingsManager *settings_manager = SettingsManager::getInstance();
		connect(ui.applyButton, &QPushButton::clicked, this, &SettingWidget::onApplyButtonClicked);
		connect(ui.cancelButton, &QPushButton::clicked, this, &SettingWidget::onCancelButtonClicked);
		connect(ui.pushButton_imagesavepath, &QPushButton::clicked, this, &SettingWidget::onPushButton_imagesavepath_clicked);
		connect(ui.pushButton_cloudsavepath, &QPushButton::clicked, this, &SettingWidget::onPushButton_cloudsavepath_clicked);
		connect(ui.pushButton_scannersavepath, &QPushButton::clicked, this, &SettingWidget::onPushButton_scannersavepath_clicked);

		ui.lineEdit_bwThr->setValidator(new QIntValidator(0, 255, this));
		ui.lineEdit_sumThr->setValidator(new QIntValidator(0, 255, this));
		ui.horizontalSlider_bwThr->setRange(0, 255);
		ui.horizontalSlider_sumThr->setRange(0, 255);
	
		//软件打开时：
		ui.checkBox_autoconnectcamera->setChecked(settings_manager->getBool(AUTO_CONNECT_TO_SCANNER));
		ui.checkBox_autoopenLaser->setChecked(settings_manager->getBool(AUTO_OPEN_LASER));
		//扫描仪打开时参数：
		QString opening_behaviour = settings_manager->getString(SCANNER_OPENING_BEHAVIOUR);
		if(opening_behaviour == SCANNER_OPENING_BEHAVIOUR_NONE){
			ui.radioButton->setChecked(true);
		} else if(opening_behaviour == SCANNER_OPENING_BEHAVIOUR_DEFAULT){
			ui.radioButton_3->setChecked(true);
		} else if(opening_behaviour == SCANNER_OPENING_BEHAVIOUR_LAST_CLOSE){
			ui.radioButton_2->setChecked(true);
		}

		ui.lineEdit_exposure->setText(settings_manager->getString(DEFAULT_EXPOSURETIME));
		ui.lineEdit_gain->setText(settings_manager->getString(DEFAULT_GAIN));
		ui.lineEdit_upperlimit->setText(settings_manager->getString(DEFAULT_MOTOR_LIMIT_UPPER));
		ui.lineEdit_lowerlimit->setText(settings_manager->getString(DEFAULT_MOTOR_LIMIT_LOWER));
		ui.lineEdit_host->setText(settings_manager->getString(SCANNER_HOST, QStringLiteral("192.168.0.7")));
		ui.lineEdit_port->setText(settings_manager->getString(SCANNER_PORT, QStringLiteral("8234")));
		
		//默认保存路径：
		ui.lineEdit_imagesavepath->setText(settings_manager->getString(SAVE_IMAGE_DIR));
		ui.lineEdit_cloudsavepath->setText(settings_manager->getString(SAVE_CLOUD_DIR));
		ui.lineEdit_scannersavepath->setText(settings_manager->getString(SAVE_SCANNER_DIR));

		//扫描默认参数：
		ui.lineEdit_startangle->setText(settings_manager->getString(DEFAULT_START_ANGLE));
		ui.lineEdit_endangle->setText(settings_manager->getString(DEFAULT_END_ANGLE));
		ui.lineEdit_step->setText(settings_manager->getString(DEFAULT_STEP_ANGLE));
		ui.lineEdit_scanexposure->setText(settings_manager->getString(DEFAULT_SCAN_EXPOSURETIME));
		ui.comboBox_line->setCurrentText(settings_manager->getString(DEFAULT_LINE));
		ui.comboBox_centerlineMethod->setCurrentText(settings_manager->getString(DEFAULT_CENTERLINE_METHOD, "GrayScale灰度中心法"));
		ui.lineEdit_bwThr->setText(QString::number(settings_manager->getInt(DEFAULT_BW_THR, 55)));
		ui.lineEdit_sumThr->setText(QString::number(settings_manager->getInt(DEFAULT_SUM_THR, 124)));
		ui.horizontalSlider_bwThr->setValue(settings_manager->getInt(DEFAULT_BW_THR, 55));
		ui.horizontalSlider_sumThr->setValue(settings_manager->getInt(DEFAULT_SUM_THR, 124));
		ui.checkBox_isHeng->setChecked(settings_manager->getBool(DEFAULT_IS_HENG, true));

		//扫描默认行为：
		ui.checkBox_scansave->setChecked(settings_manager->getBool(DEFAULT_SCAN_SAVE, true));
		ui.checkBox_scansavecloud->setChecked(settings_manager->getBool(DEFAULT_SCAN_CLOUD, true));

		connect(ui.comboBox_centerlineMethod, &QComboBox::currentTextChanged, this, [this](const QString&) {
			updateCenterlineMethodUi();
		});
		connect(ui.horizontalSlider_bwThr, &QSlider::valueChanged, this, [this](int value) {
			QSignalBlocker blocker(ui.lineEdit_bwThr);
			ui.lineEdit_bwThr->setText(QString::number(value));
		});
		connect(ui.horizontalSlider_sumThr, &QSlider::valueChanged, this, [this](int value) {
			QSignalBlocker blocker(ui.lineEdit_sumThr);
			ui.lineEdit_sumThr->setText(QString::number(value));
		});
		connect(ui.lineEdit_bwThr, &QLineEdit::editingFinished, this, [this]() {
			syncThresholdLineEditToSlider(ui.lineEdit_bwThr, ui.horizontalSlider_bwThr, 55);
		});
		connect(ui.lineEdit_sumThr, &QLineEdit::editingFinished, this, [this]() {
			syncThresholdLineEditToSlider(ui.lineEdit_sumThr, ui.horizontalSlider_sumThr, 124);
		});
		updateCenterlineMethodUi();

		QString default_calibrate_file = settings_manager->getString(DEFAULT_CALIBRATE_FILE);
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
				if(filePath == default_calibrate_file){
					ui.comboBox_calibratefile->setCurrentText(filePath);
				}
			}
		}
		if (!default_calibrate_file.isEmpty() && !addedFiles.contains(default_calibrate_file)) {
			ui.comboBox_calibratefile->addItem(default_calibrate_file);
			ui.comboBox_calibratefile->setCurrentText(default_calibrate_file);
		}
		connect(ui.comboBox_calibratefile, &QComboBox::currentTextChanged, this, &SettingWidget::calibrationFileChanged);

		ui.tabWidget->setCurrentIndex(0);
	}

	SettingWidget::~SettingWidget() {
	}

	void SettingWidget::updateCenterlineMethodUi() {
		ui.label_sumThr->setText("筛选阈值：");
	}

	void SettingWidget::syncThresholdLineEditToSlider(QLineEdit* lineEdit, QSlider* slider, int defaultValue) {
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
	}

	void SettingWidget::onApplyButtonClicked() {
		//保存上述参数
		SettingsManager * manager = SettingsManager::getInstance();
		manager->setBool(AUTO_CONNECT_TO_SCANNER, ui.checkBox_autoconnectcamera->isChecked());
		manager->setBool(AUTO_OPEN_LASER, ui.checkBox_autoopenLaser->isChecked());
		manager->setString(SCANNER_OPENING_BEHAVIOUR, ui.radioButton->isChecked()? SCANNER_OPENING_BEHAVIOUR_NONE :
			ui.radioButton_3->isChecked()? SCANNER_OPENING_BEHAVIOUR_DEFAULT : SCANNER_OPENING_BEHAVIOUR_LAST_CLOSE);
		
		bool ok = false;
		const QString host = ui.lineEdit_host->text().trimmed();
		if(host.isEmpty()){
			QMessageBox::critical(this, "警告", "光机 IP 不能为空");
			return;
		}
		manager->setString(SCANNER_HOST, host);
		int port = ui.lineEdit_port->text().toInt(&ok);
		if(!ok || port < 1 || port > 65535){
			QMessageBox::critical(this, "警告", "端口号必须在1-65535之间");
			return;
		}
		if(ok) manager->setInt(SCANNER_PORT, port);
		double exposure = ui.lineEdit_exposure->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_EXPOSURETIME, exposure);
		int gain = ui.lineEdit_gain->text().toInt(&ok);
		if(ok) manager->setInt(DEFAULT_GAIN, gain);
		double upperlimit = ui.lineEdit_upperlimit->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_MOTOR_LIMIT_UPPER, upperlimit);
		double lowerlimit = ui.lineEdit_lowerlimit->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_MOTOR_LIMIT_LOWER, lowerlimit);

		manager->setString(SAVE_IMAGE_DIR, ui.lineEdit_imagesavepath->text());
		manager->setString(SAVE_CLOUD_DIR, ui.lineEdit_cloudsavepath->text());
		manager->setString(SAVE_SCANNER_DIR, ui.lineEdit_scannersavepath->text());
		emit saveImageDirChanged(ui.lineEdit_imagesavepath->text());

		double startangle = ui.lineEdit_startangle->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_START_ANGLE, startangle);
		double endangle = ui.lineEdit_endangle->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_END_ANGLE, endangle);
		double step = ui.lineEdit_step->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_STEP_ANGLE, step);
		double scanexposure = ui.lineEdit_scanexposure->text().toDouble(&ok);
		if(ok) manager->setDouble(DEFAULT_SCAN_EXPOSURETIME, scanexposure);
		manager->setString(DEFAULT_LINE, ui.comboBox_line->currentText());
		manager->setString(DEFAULT_CENTERLINE_METHOD, ui.comboBox_centerlineMethod->currentText());

		int bwThr = ui.lineEdit_bwThr->text().toInt(&ok);
		if(!ok){
			QMessageBox::critical(this, "警告", "二值化阈值必须是0-255之间的整数");
			return;
		}
		bwThr = std::clamp(bwThr, 0, 255);
		manager->setInt(DEFAULT_BW_THR, bwThr);

		int sumThr = ui.lineEdit_sumThr->text().toInt(&ok);
		if(!ok){
			QMessageBox::critical(this, "警告", "筛选阈值必须是0-255之间的整数");
			return;
		}
		sumThr = std::clamp(sumThr, 0, 255);
		manager->setInt(DEFAULT_SUM_THR, sumThr);
		manager->setBool(DEFAULT_IS_HENG, ui.checkBox_isHeng->isChecked());

		manager->setString(DEFAULT_CALIBRATE_FILE, ui.comboBox_calibratefile->currentText());
		emit calibrationFileChanged(ui.comboBox_calibratefile->currentText());
		manager->setBool(DEFAULT_SCAN_SAVE, ui.checkBox_scansave->isChecked());
		manager->setBool(DEFAULT_SCAN_CLOUD, ui.checkBox_scansavecloud->isChecked());
		this->accept();
	}
	void SettingWidget::onCancelButtonClicked() {
		this->close();
	}
	void SettingWidget::onPushButton_imagesavepath_clicked() {
		QString path = QFileDialog::getExistingDirectory(this, "选择图片保存路径", ui.lineEdit_imagesavepath->text());
		if (!path.isEmpty()) {
			ui.lineEdit_imagesavepath->setText(path);
			emit saveImageDirChanged(path);
		}
	}
	void SettingWidget::onPushButton_cloudsavepath_clicked() {
		QString path = QFileDialog::getExistingDirectory(this, "选择点云保存路径", ui.lineEdit_cloudsavepath->text());
		if (!path.isEmpty()) {
			ui.lineEdit_cloudsavepath->setText(path);
		}
	}
	void SettingWidget::onPushButton_scannersavepath_clicked() {
		QString path = QFileDialog::getExistingDirectory(this, "选择扫描数据保存路径", ui.lineEdit_scannersavepath->text());
		if (!path.isEmpty()) {
			ui.lineEdit_scannersavepath->setText(path);
		}
	}
}}
