#pragma once

#include <QTabWidget>
#include <QTabBar>
#include <QStylePainter>
#include <QStyleOptionTab>

class SettingTabWidget : public QTabWidget
{
	Q_OBJECT

public:
    explicit SettingTabWidget(QWidget* parent = nullptr) {
        this->setTabBar(new SettingTabBar());
    };

private:
    /*
* 自定义的TabBar与QTabWidget,它能实现bar内文字横向显示
*/

    class SettingTabBar : public QTabBar
    {
    public:
        QSize tabSizeHint(int index) const override {
            QSize s = QTabBar::tabSizeHint(index);
            s.transpose();
            s.setHeight(2 * s.height());  //把高度扩大一点
            return s;
        };
    protected:
        void paintEvent(QPaintEvent*) override {
            QStylePainter painter(this);
            QStyleOptionTab opt;
            for (int i = 0; i < count(); i++)
            {
                initStyleOption(&opt, i);
                painter.drawControl(QStyle::CE_TabBarTabShape, opt);
                painter.save();

                QSize s = opt.rect.size();
                s.transpose();
                QRect r(QPoint(), s);
                r.moveCenter(opt.rect.center());
                opt.rect = r;

                QPoint c = tabRect(i).center();
                painter.translate(c);
                painter.rotate(90);//西方向
                painter.translate(-c);
                painter.drawControl(QStyle::CE_TabBarTabLabel, opt);
                painter.restore();
            }
        };//重绘tab文字
    };
};
