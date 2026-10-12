#include "mainwindow/advancedstatusbar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QDateTime>
#include <QDebug>
#include <QApplication>

AdvancedStatusBar::AdvancedStatusBar(QWidget *parent)
    : QStatusBar(parent)
{
    // 移除状态栏自带的抓手（右下角的小三角），为了美观
    setSizeGripEnabled(false);
    
    m_messageTimer = new QTimer(this);
    m_messageTimer->setSingleShot(true);
    connect(m_messageTimer, &QTimer::timeout, this, [this](){
        m_statusMessageLabel->clear();
    });

    setupUi();
    setupStyles();
    
    // 设置一个最小高度，因为这个控件比较复杂
    setMinimumHeight(180); 
}

AdvancedStatusBar::~AdvancedStatusBar()
{
}

QSize AdvancedStatusBar::sizeHint() const
{
    return QSize(800, 400);
}

void AdvancedStatusBar::setupUi()
{
    // 1. 创建主容器，因为QStatusBar只能addWidget，我们把所有东西包在一个Widget里
    m_mainContainer = new QWidget(this);
    // 将主容器添加到QStatusBar中，并设置拉伸因子为1，铺满
    this->addWidget(m_mainContainer, 1);

    // 主布局：水平分割（左侧参数 | 右侧复合）
    QHBoxLayout *mainLayout = new QHBoxLayout(m_mainContainer);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0); // 去掉间隙，用StyleSheet控制分割线

    // ================= 左侧：参数监控区 =================
    m_paramWidget = new QWidget(m_mainContainer);
    m_paramWidget->setObjectName("ParamWidget"); // 用于QSS
    m_paramWidget->setFixedWidth(300); // 固定左侧宽度，可按需调整
    
    QVBoxLayout *paramContainerLayout = new QVBoxLayout(m_paramWidget);
    
    // 标题
    QLabel *titleLabel = new QLabel("参数监控", m_paramWidget);
    titleLabel->setObjectName("SectionTitle");
    paramContainerLayout->addWidget(titleLabel);

    // 参数网格
    m_paramLayout = new QGridLayout();
    m_paramLayout->setColumnStretch(0, 2); // 参数名
    m_paramLayout->setColumnStretch(1, 3); // 数值 (给多点空间)
    m_paramLayout->setColumnStretch(2, 1); // 单位
    paramContainerLayout->addLayout(m_paramLayout);
    paramContainerLayout->addStretch(); // 底部顶上去

    mainLayout->addWidget(m_paramWidget);

    // ================= 右侧：日志 + 底部条 =================
    QWidget *rightWidget = new QWidget(m_mainContainer);
    QVBoxLayout *rightLayout = new QVBoxLayout(rightWidget);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(0);

    // --- 右上：日志表格 ---
    m_logTable = new QTableWidget(rightWidget);
    m_logTable->setColumnCount(3);
    m_logTable->setHorizontalHeaderLabels({"Type", "Message", "Time"});
    m_logTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents); // Type 自适应
    m_logTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);          // Message 拉伸
    m_logTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
    m_logTable->setColumnWidth(2, 150); // Time 固定宽
    
    // 表格外观设置
    m_logTable->verticalHeader()->setVisible(false); // 隐藏行号
    m_logTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_logTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_logTable->setFocusPolicy(Qt::NoFocus);
    m_logTable->setShowGrid(false); // 去掉网格线，更现代
    m_logTable->setAlternatingRowColors(true); // 隔行变色

    rightLayout->addWidget(m_logTable);

    // --- 右下：模拟的状态栏区域 ---
    m_bottomBarWidget = new QWidget(rightWidget);
    m_bottomBarWidget->setObjectName("BottomBar");
    m_bottomBarWidget->setFixedHeight(30); // 底部条高度

    QHBoxLayout *bottomLayout = new QHBoxLayout(m_bottomBarWidget);
    bottomLayout->setContentsMargins(10, 0, 10, 0); // 左右留点空隙

    // 状态消息 Label (实现 showMessage 的效果)
    m_statusMessageLabel = new QLabel(m_bottomBarWidget);
    m_statusMessageLabel->setObjectName("StatusMessage");
    bottomLayout->addWidget(m_statusMessageLabel, 1); // 拉伸因子1，占据左侧空白

    // 进度条
    m_progressBar = new QProgressBar(m_bottomBarWidget);
    m_progressBar->setFixedWidth(200);
    m_progressBar->setTextVisible(true);
    m_progressBar->setValue(0);
    bottomLayout->addWidget(m_progressBar);

    rightLayout->addWidget(m_bottomBarWidget);
    mainLayout->addWidget(rightWidget);

    // 初始化一些示例参数，防止空空的不好看
    updateParameter("当前位置", 0.0, "°");
    updateParameter("温度", 0.0, "℃");
    updateParameter("压力", 0.0, "bar");
    updateParameter("湿度", 0.0, "%");
    addAttitudeParameterRow(0.0, 0.0, 0.0);
}

void AdvancedStatusBar::setupStyles()
{
    // 使用 QSS 进行美化
    QString qss = R"(
        /* 全局字体 */
        QWidget {
            font-family: "Segoe UI", "Microsoft YaHei";
            font-size: 10pt;
        }

        /* 左侧参数区背景 */
        QWidget#ParamWidget {
            background-color: #f0f0f0;
            border-right: 1px solid #dcdcdc;
        }

        /* 标题样式 */
        QLabel#SectionTitle {
            font-weight: bold;
            color: #333333;
            padding: 5px 0;
            font-size: 11pt;
        }

        /* 参数文字颜色 */
        QLabel {
            color: #555555;
        }
        /* 参数数值高亮 */
        QLabel[role="value"] {
            color: #000000;
            font-weight: bold;
        }

        /* 表格样式 */
        QTableWidget {
            border: none;
            background-color: #ffffff;
            selection-background-color: #e1f5fe;
            selection-color: #000000;
        }
        QHeaderView::section {
            background-color: #ffffff;
            border: none;
            border-bottom: 1px solid #dcdcdc;
            padding: 4px;
            font-weight: bold;
            color: #666666;
        }

        /* 底部状态条样式 */
        QWidget#BottomBar {
            background-color: #f9f9f9;
            border-top: 1px solid #dcdcdc;
        }

        /* 状态消息文字 */
        QLabel#StatusMessage {
            color: #333333;
        }

        /* 进度条样式 */
        QProgressBar {
            border: 1px solid #cccccc;
            border-radius: 3px;
            text-align: center;
            background-color: #eeeeee;
        }
        QProgressBar::chunk {
            background-color: #4caf50; /* 绿色进度条 */
            width: 1px;
        }
    )";
    
    this->setStyleSheet(qss);
}

// 重写 showMessage
void AdvancedStatusBar::showMessage(const QString &message, int timeout)
{
    // 设置文字
    m_statusMessageLabel->setText(message);
    
    // 处理超时消失
    if (timeout > 0) {
        m_messageTimer->start(timeout);
    } else {
        m_messageTimer->stop();
    }
    
    // QStatusBar原本还会触发 messageChanged 信号，如果需要完全兼容可以 emit 它
    emit messageChanged(message);
}

void AdvancedStatusBar::setProgress(int value)
{
    m_progressBar->setValue(value);
}

void AdvancedStatusBar::updateAttitude(double roll, double pitch, double yaw, bool valid)
{
    if (!m_attitudeLabel) {
        return;
    }
    m_attitudeLabel->setText(QStringLiteral("%1  roll: %2°  pitch: %3°  yaw: %4°")
        .arg(valid ? QStringLiteral("有效") : QStringLiteral("无效"))
        .arg(roll, 0, 'f', 2)
        .arg(pitch, 0, 'f', 2)
        .arg(yaw, 0, 'f', 2));
}

void AdvancedStatusBar::updateParameter(const QString &key, double value, const QString &unit)
{
    // 如果不存在，创建新行
    if (!m_params.contains(key)) {
        int row = m_nextParamRow++;
        
        QLabel *name = new QLabel(key, m_paramWidget);
        QLabel *val = new QLabel(QString::number(value, 'f', 2), m_paramWidget);
        val->setProperty("role", "value"); // 用于QSS选择器
        QLabel *u = new QLabel(unit, m_paramWidget);
        
        m_paramLayout->addWidget(name, row, 0);
        m_paramLayout->addWidget(val, row, 1, Qt::AlignRight | Qt::AlignVCenter);
        m_paramLayout->addWidget(u, row, 2);
        
        ParamRow newRow = {name, val, u};
        m_params.insert(key, newRow);
        
        // 重置一下Stretch，保证布局不塌缩
        m_paramLayout->setRowStretch(row + 1, 1); 
        m_paramLayout->setRowStretch(row, 0);
    } else {
        // 更新数值
        m_params[key].valueLbl->setText(QString::number(value, 'f', 2));
    }
}

void AdvancedStatusBar::addAttitudeParameterRow(double roll, double pitch, double yaw)
{
    int row = m_nextParamRow++;

    m_attitudeWidget = new QWidget(m_paramWidget);
    QHBoxLayout *attitudeLayout = new QHBoxLayout(m_attitudeWidget);
    attitudeLayout->setContentsMargins(0, 0, 0, 0);
    attitudeLayout->setSpacing(12);

    m_attitudeLabel = new QLabel(m_attitudeWidget);
    m_attitudeLabel->setProperty("role", "value");
    m_attitudeLabel->setAlignment(Qt::AlignCenter);
    attitudeLayout->addWidget(m_attitudeLabel, 1);
    m_paramLayout->addWidget(m_attitudeWidget, row, 0, 1, 3);

    m_paramLayout->setRowStretch(row + 1, 1);
    m_paramLayout->setRowStretch(row, 0);
    updateAttitude(roll, pitch, yaw, false);
}

void AdvancedStatusBar::addLogMessage(const QString &type, const QString &message)
{
    int row = m_logTable->rowCount();
    m_logTable->insertRow(row);

    QTableWidgetItem *typeItem = new QTableWidgetItem("");
    QTableWidgetItem *msgItem = new QTableWidgetItem(message);
    QTableWidgetItem *timeItem = new QTableWidgetItem(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"));
    
    // 根据类型设置颜色 (可选)
    if (type == "Error") {
        typeItem->setIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxCritical));
        msgItem->setForeground(Qt::red);
    } else if (type == "Warning") {
        typeItem->setIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxWarning));
        msgItem->setForeground(QColor(255, 165, 0)); // Orange
    } else if (type == "Info"){
        typeItem->setIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation));
        msgItem->setForeground(Qt::darkGreen);
    }

    m_logTable->setItem(row, 0, typeItem);
    m_logTable->setItem(row, 1, msgItem);
    m_logTable->setItem(row, 2, timeItem);
    
    // 自动滚动到底部
    m_logTable->scrollToBottom();
}
