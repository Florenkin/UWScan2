#ifndef INFOFULLSTATUSBAR_H
#define INFOFULLSTATUSBAR_H

#include <QStatusBar>
#include <QGroupBox>
#include <QLabel>
#include <QProgressBar>
#include "messagestation.h"

class InfofullStatusBar : public QStatusBar
{
    Q_OBJECT
public:
    explicit InfofullStatusBar(QWidget *parent = nullptr);
    ~InfofullStatusBar();

    QGroupBox *monitorGroup;
    QLabel *laserLabel;
    QLabel *temperatureLabel;
    QLabel *pressureLabel;
    QLabel *humidityLabel;

    MessageStation *messageStation;
    QProgressBar *progressBar;
    QLabel *infomationLabel;
};

#endif