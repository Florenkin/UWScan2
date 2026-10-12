#pragma once
#include <QDialog>
#include <QLineEdit>
#include <QSlider>
#include "ui_settingwidget.h"
#include "utils.h"

NAMESPACE_BEGIN{ namespace setting{
	class SettingWidget : public QDialog {
		Q_OBJECT

	public:
		~SettingWidget();
		SettingWidget(QWidget* parent = nullptr);
	signals:
		void calibrationFileChanged(const QString& path);
		void saveImageDirChanged(const QString& path);

	private:
		void onApplyButtonClicked();
		void onCancelButtonClicked();
		void onPushButton_imagesavepath_clicked();
		void onPushButton_cloudsavepath_clicked();
		void onPushButton_scannersavepath_clicked();
		void updateCenterlineMethodUi();
		void syncThresholdLineEditToSlider(QLineEdit* lineEdit, QSlider* slider, int defaultValue);
	private:		
		Ui::SettingWidgetClass ui;
	};
}}
