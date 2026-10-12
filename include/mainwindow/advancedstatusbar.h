#ifndef ADVANCEDSTATUSBAR_H
#define ADVANCEDSTATUSBAR_H

#include <QStatusBar>
#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QTableWidget>
#include <QGridLayout>
#include <QMap>
#include <QTimer>

class AdvancedStatusBar : public QStatusBar
{
    Q_OBJECT

public:
    explicit AdvancedStatusBar(QWidget *parent = nullptr);
    ~AdvancedStatusBar();
public slots:
    /**
     * @brief 更新左侧参数显示
     * @param key 参数名称
     * @param value 参数值
     * @param unit 参数单位（可选）
     */
    void updateParameter(const QString &key, double value, const QString &unit = "");
    
    /**
     * @brief 添加日志到右上角表格
     * @param type 日志类型（如 "Info", "Warning", "Error"）
     * @param message 日志内容
     */
    void addLogMessage(const QString &type, const QString &message);

    /**
     * @brief 设置进度条值
     * @param value 进度值（0-100）
     */
    void setProgress(int value);
    void updateAttitude(double roll, double pitch, double yaw, bool valid);

    /**
     * @brief 重写showMessage，使其显示在我们自定义的Label上
     * @param message 要显示的消息
     * @param timeout 显示时间（毫秒），默认0表示永久显示
     */
    void showMessage(const QString &message, int timeout = 0);

protected:
    // 确保控件有足够的高度
    QSize sizeHint() const override;

private:
    void setupUi();
    void setupStyles();
    void addAttitudeParameterRow(double roll, double pitch, double yaw);

    // UI Components
    QWidget *m_mainContainer;
    
    // Left Side
    QWidget *m_paramWidget;
    QGridLayout *m_paramLayout;
    int m_nextParamRow = 0;
    struct ParamRow {
        QLabel *nameLbl;
        QLabel *valueLbl;
        QLabel *unitLbl;
    };
    QMap<QString, ParamRow> m_params;

    // Right Side
    QTableWidget *m_logTable;
    QWidget *m_bottomBarWidget;
    QLabel *m_statusMessageLabel; // 替代原生状态栏消息
    QProgressBar *m_progressBar;
    QWidget *m_attitudeWidget = nullptr;
    QLabel *m_attitudeLabel = nullptr;

    // Timer for clearing message
    QTimer *m_messageTimer;
};

#endif // ADVANCEDSTATUSBAR_H
