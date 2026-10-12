#include "messagestation.h"
#include "ui_messagestation.h"

MessageStation::MessageStation(QWidget *parent)
	: QWidget(parent),
	  ui(std::make_unique<Ui::MessageStationClass>())
{
	ui->setupUi(this);
	this->setContextMenuPolicy(Qt::CustomContextMenu);
	clearMessages();
	connect(this, &MessageStation::customContextMenuRequested, this, &MessageStation::onCustomContextMenuRequested, Qt::UniqueConnection);
}

MessageStation::~MessageStation()
{}

void MessageStation::onReceiveMessage(QMessageBox::Icon icon, QString message)
{
	QString m_message = message;
	QIcon m_icon;
	if (icon == QMessageBox::Information)
	{
		m_icon = QIcon(":/icon/success.png");
	}
	else if (icon == QMessageBox::Warning)
	{
		m_icon = QIcon(":/icon/warning.png");
	}
	else if (icon == QMessageBox::Critical)
	{
		m_icon = QIcon(":/icon/error.png");
	}
	else
	{
		return;
	}

	int count = ui->tableWidget->rowCount();
	ui->tableWidget->setRowCount(count + 1);

	QFont font;
	font.setFamily(QString::fromUtf8("Times New Roman"));

	QTableWidgetItem* cube1 = new QTableWidgetItem(m_icon,"");
	cube1->setFont(font);
	ui->tableWidget->setItem(count, 0, cube1);
	

	QTableWidgetItem* cube2 = new QTableWidgetItem(message);
	cube2->setFont(font);
	ui->tableWidget->setItem(count, 1, cube2);

	QTableWidgetItem* cube3 = new QTableWidgetItem(QDateTime::currentDateTime().toString());
	cube3->setFont(font);
	ui->tableWidget->setItem(count, 2, cube3);

	auto lastModelIndex = ui->tableWidget->model()->index(count, 0);
	ui->tableWidget->scrollTo(lastModelIndex);
}

void MessageStation::resizeEvent(QResizeEvent* re)
{
	ui->tableWidget->setColumnWidth(0, 40);
	ui->tableWidget->setColumnWidth(1, (this->width() - 40) / 2 - 1);
	ui->tableWidget->setColumnWidth(2, (this->width() - 40) / 2 - 1);
}

void MessageStation::onCustomContextMenuRequested(const QPoint& Pos)
{
	QMenu menu(this);//�½�Menu
	actionClearMessages = menu.addAction(QString::fromStdWString(L"�����Ϣ"));
	connect(actionClearMessages, SIGNAL(triggered(bool)), this, SLOT(onActionClearMessagesClicked()), Qt::UniqueConnection);
	//eventPos = QCursor::pos();
	//eventPos = mapFromGlobal(eventPos);
	menu.exec(QCursor::pos());
	menu.show();
}

void MessageStation::onActionClearMessagesClicked()
{
	ui->tableWidget->clear();
	clearMessages();
}

void MessageStation::clearMessages()
{
	ui->tableWidget->setRowCount(0);
	ui->tableWidget->setColumnCount(3);
	ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	QStringList header;
	header << "Type" << "Message" << "Time";

	ui->tableWidget->setHorizontalHeaderLabels(header);
	ui->tableWidget->setColumnWidth(0, 40);
	ui->tableWidget->setColumnWidth(1, (this->width() - 40) / 2 - 1);
	ui->tableWidget->setColumnWidth(2, (this->width() - 40) / 2 - 1);
	ui->tableWidget->verticalHeader()->hide();
}
