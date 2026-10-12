#include "pointcloud/importoptiondialog.h"
#include <QLabel>

NAMESPACE_BEGIN { namespace pointcloud {
    ImportOptionDialog::ImportOptionDialog(QWidget *parent) : QDialog(parent, Qt::FramelessWindowHint) {
        setWindowTitle(tr("导入选项"));
        setModal(true);

        QVBoxLayout *layout = new QVBoxLayout(this);

        // 单位选择
        QHBoxLayout *unit_layout = new QHBoxLayout();
        unit_layout->addWidget(new QLabel(tr("单位:")));
        unit_combo_ = new QComboBox();
        unit_combo_->addItem(tr("毫米"));
        unit_combo_->addItem(tr("厘米"));
        unit_combo_->addItem(tr("分米"));
        unit_combo_->addItem(tr("米"));
        
        unit_combo_->setCurrentIndex(0);

        unit_layout->addWidget(unit_combo_);
        layout->addLayout(unit_layout);

        //采样率选择
        QHBoxLayout *sample_layout = new QHBoxLayout();
        sample_layout->addWidget(new QLabel(tr("采样率:")));
        sampling_rate_combo_ = new QComboBox();
        sampling_rate_combo_->addItem(tr("100%"));
        sampling_rate_combo_->addItem(tr("75%"));
        sampling_rate_combo_->addItem(tr("50%"));
        sampling_rate_combo_->addItem(tr("25%"));
        sampling_rate_combo_->addItem(tr("10%"));
        
        sampling_rate_combo_->setCurrentIndex(0);
        sample_layout->addWidget(sampling_rate_combo_);
        layout->addLayout(sample_layout);

        // 确认按钮和取消按钮
        QHBoxLayout *button_layout = new QHBoxLayout();
        QPushButton *ok_button = new QPushButton(tr("确定"));
        QPushButton *cancel_button = new QPushButton(tr("取消"));
        button_layout->addWidget(ok_button);
        button_layout->addWidget(cancel_button);
        layout->addLayout(button_layout);

        // 连接信号槽
        connect(ok_button, &QPushButton::clicked, this, &ImportOptionDialog::onOkClicked);
        connect(cancel_button, &QPushButton::clicked, this, &ImportOptionDialog::onCancelClicked);
    }

    bool ImportOptionDialog::getImportOptions(double & point_scale, double & sampling_rate){
        static ImportOptionDialog dialog;
        if (dialog.exec() == QDialog::Accepted) {
            int unit_index = dialog.unit_combo_->currentIndex();
            int sample_index = dialog.sampling_rate_combo_->currentIndex();
            
            point_scale = pow(0.1f, unit_index);
            sampling_rate = sample_index == 0 ? 1.0f : sample_index == 1 ? \
                0.75f : sample_index == 2 ? 0.5f : sample_index == 3 ? 0.25f : 0.1f;
            
            return true;
        }
        return false;
    }

    void ImportOptionDialog::onOkClicked(){
        accept();
    }

    void ImportOptionDialog::onCancelClicked(){
        reject();
    }
}}