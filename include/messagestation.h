#pragma once
#include <QWidget>
#include <QMessageBox>
#include <QDateTime>
#include <QResizeEvent>
#include <QMenu>
#include <memory>

namespace Ui {
class MessageStationClass;
}

class MessageStation : public QWidget
{
	Q_OBJECT

public:
	/**
	 * @brief 构造函数
	 * @param parent 父窗口指针
	 */
	MessageStation(QWidget *parent = nullptr);

	/**
	 * @brief 析构函数
	 */
	~MessageStation();

	/**
	 * @brief 清除所有消息
	 */
	void clearMessages();

public slots:
	/**
	 * @brief 接收消息槽函数
	 * @param icon 消息图标
	 * @param message 消息内容
	 */
	void onReceiveMessage(QMessageBox::Icon icon, QString message);

	/**
	 * @brief 自定义上下文菜单槽函数
	 * @param pos 鼠标点击位置
	 */
	void onCustomContextMenuRequested(const QPoint& pos);

protected slots:
	/**
	 * @brief 窗口大小改变槽函数
	 * @param re 窗口大小改变事件指针
	 */
	void resizeEvent(QResizeEvent* re);

private slots:
	/**
	 * @brief 清除消息Action触发槽函数
	 */
	void onActionClearMessagesClicked();
private:
	std::unique_ptr<Ui::MessageStationClass> ui;  	///< 消息站UI
	QAction* actionClearMessages;  	///< 清除消息Action
};
