#ifndef IMAGEWIDGET_H
#define IMAGEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QMutex>
#include <QWheelEvent>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/PylonIncludes.h>
#include "utils.h"

NAMESPACE_BEGIN{
class ImageWidget : public QWidget
{
    Q_OBJECT
signals:
	void sendRoi(int* roi);
    void roiSelected(int x, int y, int width, int height);
    void rightClicked(const QPoint& pos);
public:
    enum Status { NO_IMAGE = 0, AMPLIFICATION_IMAGE = 1, PREVIEW_IMAGE = 2 } image_status = NO_IMAGE;
    enum InteractMode { ViewingMode = 0, PickingMode = 1 };

    explicit ImageWidget(QWidget *parent = nullptr);

    // 设置Qt图像
    void setImage(const QImage &image);
    
    // 设置OpenCV Mat (辅助函数)
    void setMat(const cv::Mat &mat);

    void setImage(cv::Mat& mat);
	void setImage(Pylon::CGrabResultPtr mat, Basler_UniversalCameraParams::PixelFormatEnums imageType);
	void setImage(Pylon::CGrabResultPtr mat);
	void setImage(QString path);
	void autoResize();
    void setInteractMode(InteractMode);
	void setImageStatus(Status);
    void setRoiSelectionEnabled(bool enabled);
    void cropToRoi(int x, int y, int width, int height);

	cv::Mat getImage();
public slots:
	void stopContinousDisplay();
	void onReceiveDisplayImg(cv::Mat);

protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    // 核心绘图数据
    QPixmap m_pixmap;
    qreal m_scaleValue;     // 当前缩放因子
    QPointF m_offset;       // 图像左上角在控件坐标系中的偏移
    
    // 鼠标交互状态
    bool m_isDragging;
    bool m_roiSelectionEnabled = true;
    bool m_isSelectingRoi = false;
    QPoint m_lastMousePos;
    QPoint m_roiStartPos;
    QRect m_roiRubberRect;

    // 互斥锁，保护图像数据的并发访问
    QMutex m_mutex;

    InteractMode interactMode = ViewingMode;

    // 边界检查函数
    void checkBounds();
    // 重置视图以适应窗口
    void fitToWindow();
    QPointF widgetToImage(const QPoint& pos) const;
};
}
#endif // IMAGEWIDGET_H
