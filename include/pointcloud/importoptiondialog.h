#ifndef IMPORTOPTIONDIALOG_H
#define IMPORTOPTIONDIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include "utils.h"

NAMESPACE_BEGIN { namespace pointcloud {
    class ImportOptionDialog : public QDialog {
        Q_OBJECT
    public:
        /**
         * @brief 静态获取导入选项
         * @details 弹出对话框，获取用户导入选项(单位、采样率）
         * @param point_scale 点云缩放因子
         * @param sampling_rate 采样率
         * @return 是否成功获取选项
         */
        static bool getImportOptions(double & point_scale, double & sampling_rate);

        /**
         * @brief 构造函数，不推荐直接构建，建议使用静态方法 getImportOptions
         * @param parent 父窗口指针
         */
        explicit ImportOptionDialog(QWidget *parent = nullptr);
        ~ImportOptionDialog() = default;
    private slots:
        /**
         * @brief 确认按钮点击槽函数
         */
        void onOkClicked();

        /**
         * @brief 取消按钮点击槽函数
         */
        void onCancelClicked();
    private:
        QComboBox * unit_combo_;            ///< 单位选择组合框
        QComboBox * sampling_rate_combo_;   ///< 采样率选择组合框
        QPushButton *ok_button_;            ///< 确认按钮
        QPushButton *cancel_button_;        ///< 取消按钮
    };
}}

#endif // IMPORTOPTIONDIALOG_H