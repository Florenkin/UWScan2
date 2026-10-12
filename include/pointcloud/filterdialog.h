#ifndef UWSCAN_FILTERDIALOG_H
#define UWSCAN_FILTERDIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QGroupBox>
#include <QStackedWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <iostream>
#include "utils.h"
#include "pointcloud/pc.hpp"

NAMESPACE_BEGIN { namespace pointcloud {
    /**
     * @brief 点云滤波对话框
     * 用于配置点云滤波参数
     */
    class FilterDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit FilterDialog(std::shared_ptr<PointCloud> cloud, QWidget *parent = nullptr);
        ~FilterDialog();

        /**
         * @brief 设置当前选中的滤波算法
         * @param index 下拉框选中的索引
         */
        inline void setAlgorithm(int index) { comboAlgorithm->setCurrentIndex(index); }
    private slots:
        /**
         * @brief 当下拉框改变时触发，切换参数页面
         * @param index 下拉框选中的索引
         */
        void onAlgorithmChanged(int index);

        /**
         * @brief 点击应用按钮触发
         */
        void onApply();

    private:
        /**
         * @brief 设置对话框的用户界面
         */
        void setupUI();

        /**
         * @brief 创建体素网格滤波参数页面
         * @return QWidget* 体素网格滤波参数页面的指针
         */
        QWidget* createVoxelPage();

        /**
         * @brief 创建统计离群点滤波参数页面
         * @return QWidget* 统计离群点滤波参数页面的指针
         */
        QWidget* createSorPage();

        /**
         * @brief 创建直通滤波参数页面
         * @return QWidget* 直通滤波参数页面的指针
         */
        QWidget* createPassThroughPage();

        // UI 控件
        QComboBox *comboAlgorithm;      ///< 滤波算法下拉框 
        QGroupBox *paramGroupBox;       ///< 参数组框 
        QStackedWidget *stackedWidget;  ///< 参数页面栈 
        
        QDoubleSpinBox *spinVoxelLeafSize; ///< 体素网格滤波参数：体素叶子大小 

        QSpinBox *spinSorMeanK;             ///< 统计离群点滤波参数：近邻点数量 
        QDoubleSpinBox *spinSorStdDevMul;   ///< 统计离群点滤波参数：标准差倍数 

        // PassThrough 参数控件
        QDoubleSpinBox *spinPassMin;    ///< 直通滤波参数：最小值 
        QDoubleSpinBox *spinPassMax;    ///< 直通滤波参数：最大值  

        /**
         * @brief 点云指针
         */
        std::shared_ptr<PointCloud> cloud_;
    };
}}
#endif // UWSCAN_FILTERDIALOG_H
