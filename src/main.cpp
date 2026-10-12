#include <QApplication>
#include <iostream>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <QProcess>
#include <QSurfaceFormat>
#include <windows.h>
#include <QString>
#include <QVTKOpenGLNativeWidget.h>
// #include "settingwidget.h"
// #include "systemjson.h"
// #include "imageoverlay.h"
#include <opencv2/opencv.hpp>
#include "infofullstatusbar.h"
#include "mainwindow/uwscanmainwindow.h"

/**
 * @brief 重定向qdebug的打印
 * @param type
 * @param context
 * @param msg
 */
void log_out_put(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QByteArray localMsg = msg.toLocal8Bit();

    switch (type) {
    case QtDebugMsg:
        fprintf(stdout, "%s |[Debug] (%s[%u],%s)\n", localMsg.constData(), context.function, context.line, context.file);
        break;

    case QtWarningMsg:
        fprintf(stdout, "%s |[Warning] (%s[%u],%s)\n", localMsg.constData(), context.function, context.line, context.file);
        break;

    case QtCriticalMsg:
        fprintf(stdout, "%s |[Critical] (%s[%u],%s)\n", localMsg.constData(), context.function, context.line, context.file);
        break;

    case QtFatalMsg:
        fprintf(stdout, "%s |[Fatal] (%s[%u],%s)\n", localMsg.constData(), context.function, context.line, context.file);
        abort();
        break;

    default:
        fprintf(stdout, "%s |[Debug](%s[%u],%s)\n", localMsg.constData(), context.function, context.line, context.file);
        break;
    }
#ifndef QT_NO_DEBUG_OUTPUT
    fflush(stdout);
#endif
}

bool addFirewallRule() {
    QString appPath = QCoreApplication::applicationFilePath();
    appPath = appPath.replace("/", "\\");  // Windows路径格式
    
    QString ruleName = "UWScanPort8000";
    
    // 检查规则是否已存在
    QProcess process;
    process.start("netsh", QStringList() 
                  << "advfirewall" << "firewall" 
                  << "show" << "rule" << "name=" + ruleName);
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    
    if (output.contains("Enabled")) {
        qDebug() << "Firewall rule already exists";
        return true;
    }
    
    // 添加防火墙规则（需要管理员权限）
    QStringList args;
    args << "advfirewall" << "firewall" << "add" << "rule"
         << "name=" + ruleName
         << "dir=in"
         << "action=allow"
         << "protocol=TCP"
         << "localport=8000"
         << "program=" + appPath
         << "profile=private,public"
         << "enable=yes";
    
    process.start("netsh", args);
    process.waitForFinished();
    
    if (process.exitCode() == 0) {
        qDebug() << "Firewall rule added successfully";
        return true;
    } else {
        qDebug() << "Failed to add firewall rule:" << process.readAllStandardError();
        return false;
    }
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
	QApplication a(argc,argv);

    // QCoreApplication::setOrganizationName("UWScan");
    // a.setApplicationName("UWScan");

    //配置防火墙规则
    // if (!addFirewallRule()) {
    //     QMessageBox::warning(nullptr, "警告", "无法配置防火墙规则，您可能需要手动允许端口8000。");
    // }

    qInstallMessageHandler(log_out_put);
    QFile file(":/icon/style.qss"); // 注意冒号，对应qrc中的路径
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream stream(&file);
        QString qss = stream.readAll();
        // qApp->setStyleSheet(qss);
        file.close();
    }

    QFont f = a.font();
    f.setFamily(u8"微软雅黑");
    a.setFont(f);
    QElapsedTimer cost;
    cost.start();

	a.setApplicationName("UWScan");
    a.setApplicationVersion("2.0.1");

	// ConfigManager& config = ConfigManager::instance();
    
    // // 加载配置文件
    // if (!config.loadConfig()) {
    //     qDebug() << "No config file found, using defaults";
    // }

	NAMESPACE::mainwindow::UWScanMainWindow w;
    qDebug() << "window build cost:" << cost.elapsed() << " ms";
    w.resize(1600, 900);
    w.show();
    w.raise();
    w.activateWindow();
	
	int ret = a.exec();
	// 保存配置文件
    // config.saveConfig();
    
    return ret;
}
