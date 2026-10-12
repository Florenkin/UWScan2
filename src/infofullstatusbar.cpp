#include "infofullstatusbar.h"
#include "utils.h"
#include <QHBoxLayout>
#include <QVBoxLayout>

InfofullStatusBar::InfofullStatusBar(QWidget *parent) : QStatusBar(parent){
    // 创建一个容器widget来承载自定义布局
    QWidget *container = new QWidget(this);
    
    // 设置容器的大小策略，使其能够扩展填满可用空间
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    // 创建监控组
    monitorGroup = new QGroupBox(CH("参数监控"), container);
    monitorGroup->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    
    // 设置监控组的固定宽度
    monitorGroup->setFixedWidth(200); // 根据你的需要调整
    
    // 创建激光角度部分
    QLabel *laserLeft = new QLabel(CH("激光角度"), monitorGroup);
    laserLabel = new QLabel(CH("0.0"), monitorGroup);
    QLabel *laserRight = new QLabel(CH("°"), monitorGroup);

    QHBoxLayout *laserLayout = new QHBoxLayout();
    laserLayout->addWidget(laserLeft);
    laserLayout->addWidget(laserLabel);
    laserLayout->addWidget(laserRight);

    // 创建温度部分
    QLabel *temperatureLeft = new QLabel(CH("温度"), monitorGroup);
    temperatureLabel = new QLabel(CH("0.0"), monitorGroup);
    QLabel *temperatureRight = new QLabel(CH("℃"), monitorGroup);

    QHBoxLayout *temperatureLayout = new QHBoxLayout();
    temperatureLayout->addWidget(temperatureLeft);
    temperatureLayout->addWidget(temperatureLabel);
    temperatureLayout->addWidget(temperatureRight);

    // 创建压力部分
    QLabel *pressureLeft = new QLabel(CH("压力"), monitorGroup);
    pressureLabel = new QLabel(CH("0.0"), monitorGroup);
    QLabel *pressureRight = new QLabel(CH("kPa"), monitorGroup);

    QHBoxLayout *pressureLayout = new QHBoxLayout();
    pressureLayout->addWidget(pressureLeft);
    pressureLayout->addWidget(pressureLabel);
    pressureLayout->addWidget(pressureRight);

    // 创建湿度部分
    QLabel *humidityLeft = new QLabel(CH("湿度"), monitorGroup);
    humidityLabel = new QLabel(CH("0.0"), monitorGroup);
    QLabel *humidityRight = new QLabel(CH("%"), monitorGroup);

    QHBoxLayout *humidityLayout = new QHBoxLayout();
    humidityLayout->addWidget(humidityLeft);
    humidityLayout->addWidget(humidityLabel);
    humidityLayout->addWidget(humidityRight);

    // 创建监控组的垂直布局
    QVBoxLayout *monitorLayout = new QVBoxLayout(monitorGroup);
    monitorLayout->addLayout(laserLayout);
    monitorLayout->addLayout(temperatureLayout);
    monitorLayout->addLayout(pressureLayout);
    monitorLayout->addLayout(humidityLayout);
    monitorLayout->addStretch();

    // 创建其他组件
    progressBar = new QProgressBar(container);
    progressBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    
    messageStation = new MessageStation(container);
    messageStation->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    infomationLabel = new QLabel(container);
    infomationLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // 创建进度条和信息标签的水平布局
    QHBoxLayout *progress_status_layout = new QHBoxLayout();
    progress_status_layout->addWidget(infomationLabel);
    progress_status_layout->addWidget(progressBar);
    progress_status_layout->setContentsMargins(0, 0, 0, 0);

    // 创建消息区域的垂直布局
    QVBoxLayout *message_layout = new QVBoxLayout();
    message_layout->addWidget(messageStation);
    message_layout->addLayout(progress_status_layout);
    message_layout->addStretch();
    message_layout->setContentsMargins(0, 0, 0, 0);

    // 创建主水平布局并设置给容器
    QHBoxLayout *main_layout = new QHBoxLayout(container);
    main_layout->addWidget(monitorGroup);
    
    // 方法1: 添加一个固定宽度的间隔（推荐）
    // 在monitorGroup和message_layout之间添加一个固定宽度的间隔
    QWidget *spacer = new QWidget(container);
    spacer->setFixedWidth(15); // 设置你想要的间距大小
    spacer->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    main_layout->addWidget(spacer);
    
    // 方法2: 使用布局的addSpacing方法（更简单）
    // main_layout->addSpacing(15); // 取消注释这行，并注释掉上面的spacer代码
    
    main_layout->addLayout(message_layout);
    
    // 设置布局的边距和间距
    main_layout->setContentsMargins(5, 2, 5, 2); // 容器内边距
    main_layout->setSpacing(0); // 部件之间的间距，我们已经用spacer控制了
    
    // 设置布局比例
    main_layout->setStretch(0, 0); // 监控组不拉伸
    main_layout->setStretch(1, 0); // 间隔不拉伸
    main_layout->setStretch(2, 1); // 消息区域拉伸填满剩余空间
    
    // 将容器添加到状态栏，并设置拉伸因子
    this->addWidget(container, 1);
    
    // 设置状态栏样式，移除内边距
    this->setStyleSheet("QStatusBar { padding: 0px; margin: 0px; border: none; }");
}

InfofullStatusBar::~InfofullStatusBar(){
    delete monitorGroup;
    delete messageStation;
    delete progressBar;
    delete infomationLabel;
}


