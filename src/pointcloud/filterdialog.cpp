#include "pointcloud/filterdialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFormLayout>
#include <QDebug>

NAMESPACE_BEGIN { namespace pointcloud {
    FilterDialog::FilterDialog(std::shared_ptr<PointCloud> cloud, QWidget *parent) : QDialog(parent)
    {
        setupUI();
        cloud_ = cloud;
        
        // 设置窗口标题和大小
        setWindowTitle(tr("点云滤波设置"));
        resize(400, 300);

        // 初始化默认显示第一页
        onAlgorithmChanged(0);
    }

    FilterDialog::~FilterDialog()
    {
    }

    void FilterDialog::setupUI()
    {
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

#pragma region   算法选择区域
        QHBoxLayout *topLayout = new QHBoxLayout();
        QLabel *lblSelect = new QLabel("选择滤波算法:", this);
        comboAlgorithm = new QComboBox(this);
        comboAlgorithm->addItem("体素下采样");
        comboAlgorithm->addItem("统计滤波");
        comboAlgorithm->addItem("直通滤波");
        topLayout->addWidget(lblSelect);
        topLayout->addWidget(comboAlgorithm);
        mainLayout->addLayout(topLayout);
#pragma endregion
#pragma region   参数设置区域
        paramGroupBox = new QGroupBox("滤波参数配置", this);
        QVBoxLayout *groupLayout = new QVBoxLayout(paramGroupBox);
        
        stackedWidget = new QStackedWidget(this);
        stackedWidget->addWidget(createVoxelPage());      // Index 0
        stackedWidget->addWidget(createSorPage());        // Index 1
        stackedWidget->addWidget(createPassThroughPage());// Index 2
        
        groupLayout->addWidget(stackedWidget);
        mainLayout->addWidget(paramGroupBox);
#pragma endregion
#pragma region   底部按钮区域
        QHBoxLayout *btnLayout = new QHBoxLayout();
        btnLayout->addStretch();
        QPushButton *btnApply = new QPushButton("应用", this);
        QPushButton *btnClose = new QPushButton("关闭", this);
        btnLayout->addWidget(btnApply);
        btnLayout->addWidget(btnClose);
        mainLayout->addLayout(btnLayout);
#pragma endregion
        
        // 信号槽连接
        connect(comboAlgorithm, SIGNAL(currentIndexChanged(int)), this, SLOT(onAlgorithmChanged(int)));
        connect(btnApply, &QPushButton::clicked, this, &FilterDialog::onApply);
        connect(btnClose, &QPushButton::clicked, this, &FilterDialog::accept); // 关闭窗口
    }

    QWidget* FilterDialog::createVoxelPage()
    {
        QWidget *page = new QWidget();
        QFormLayout *layout = new QFormLayout(page);

        spinVoxelLeafSize = new QDoubleSpinBox(page);
        spinVoxelLeafSize->setRange(0.1, 1000.0);
        spinVoxelLeafSize->setValue(1.0); // 默认 1mm
        spinVoxelLeafSize->setSingleStep(1.0);
        spinVoxelLeafSize->setSuffix(" mm");

        layout->addRow("体素尺寸:", spinVoxelLeafSize);
        return page;
    }

    QWidget* FilterDialog::createSorPage()
    {
        QWidget *page = new QWidget();
        QFormLayout *layout = new QFormLayout(page);

        spinSorMeanK = new QSpinBox(page);
        spinSorMeanK->setRange(1, 200);
        spinSorMeanK->setValue(50); // 默认查询50个邻居

        spinSorStdDevMul = new QDoubleSpinBox(page);
        spinSorStdDevMul->setRange(0.1, 10.0);
        spinSorStdDevMul->setValue(1.0); // 默认 1.0 倍标准差
        spinSorStdDevMul->setSingleStep(0.1);

        layout->addRow("邻居数:", spinSorMeanK);
        layout->addRow("标准差倍数:", spinSorStdDevMul);
        return page;
    }

    QWidget* FilterDialog::createPassThroughPage()
    {
        QWidget *page = new QWidget();
        QFormLayout *layout = new QFormLayout(page);

        spinPassMin = new QDoubleSpinBox(page);
        spinPassMin->setRange(-10000.0, 10000.0);
        spinPassMin->setValue(0.0); // 默认 0mm
        spinPassMin->setSuffix(" mm");

        spinPassMax = new QDoubleSpinBox(page);
        spinPassMax->setRange(-10000.0, 10000.0);
        spinPassMax->setValue(2000.0); // 默认 2000mm
        spinPassMax->setSuffix(" mm");

        layout->addRow("滤波轴", new QLabel("Z轴")); // 假设默认针对Z轴
        layout->addRow("滤波下限:", spinPassMin);
        layout->addRow("滤波上限:", spinPassMax);
        return page;
    }

    // --- 槽函数实现 ---

    void FilterDialog::onAlgorithmChanged(int index)
    {
        // 切换 StackedWidget 到对应的页面
        stackedWidget->setCurrentIndex(index);
        
        // 可选：更新 GroupBox 标题
        QString algName = comboAlgorithm->currentText();
        paramGroupBox->setTitle(algName + " 参数");
    }

    void FilterDialog::onApply()
    {
        int index = comboAlgorithm->currentIndex();
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "[Gemini 3 Pro] Applying Filter..." << std::endl;

        if (index == 0) {
            // Voxel Grid
            double leafSize = spinVoxelLeafSize->value();
            std::cout << "Algorithm: Voxel Grid Filter" << std::endl;
            std::cout << "Parameter [Leaf Size]: " << leafSize << " mm" << std::endl;
            cloud_->voxelFilter(leafSize);            
        } else if (index == 1) {
            // SOR
            int meanK = spinSorMeanK->value();
            double stdMul = spinSorStdDevMul->value();
            std::cout << "Algorithm: Statistical Outlier Removal" << std::endl;
            std::cout << "Parameter [Mean K]: " << meanK << std::endl;
            std::cout << "Parameter [StdDev Thresh]: " << stdMul << std::endl;
            cloud_->sorFilter(meanK, stdMul);

        } else if (index == 2) {
            // PassThrough
            double minLim = spinPassMin->value();
            double maxLim = spinPassMax->value();
            std::cout << "Algorithm: PassThrough Filter (Z-Axis)" << std::endl;
            std::cout << "Parameter [Min Limit]: " << minLim << " mm" << std::endl;
            std::cout << "Parameter [Max Limit]: " << maxLim << " mm" << std::endl;
            cloud_->passThroughFilter("z", minLim, maxLim);
        }
        
        std::cout << "----------------------------------------" << std::endl;
        this->accept();
    }
}}