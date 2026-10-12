#ifndef UTILS_H
#define UTILS_H

#include <QMessageBox>

#define NAMESPACE UWScan
#define NAMESPACE_BEGIN namespace NAMESPACE
#define USING_NAMESPACE using namespace NAMESPACE;

#define CH(text) QString::fromWCharArray(L##text)

#define Crit(text) emit sendMessage(QMessageBox::Critical, CH(text))
#define Warn(text) emit sendMessage(QMessageBox::Warning, CH(text))
#define Info(text) emit sendMessage(QMessageBox::Information, CH(text))

#define LOG_INFO(text) QMessageBox::information(nullptr, tr("提示"), tr(text))
#define LOG_DEBUG(text) QMessageBox::debug(nullptr, tr("调试"), tr(text))
#define LOG_WARN(text) QMessageBox::warning(nullptr, tr("警告"), tr(text))
#define LOG_ERROR(text) QMessageBox::critical(nullptr, tr("错误"), tr(text))


#define EPSILON 1e-6
#endif