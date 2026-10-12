#include "ImageWidget.h"
#include <QDebug>
NAMESPACE_BEGIN{
ImageWidget::ImageWidget(QWidget *parent)
    : QWidget(parent)
    , m_scaleValue(1.0)
    , m_offset(0, 0)
    , m_isDragging(false)
{
    setMinimumSize(0, 0);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPalette pal = this->palette();
    pal.setColor(QPalette::Window, QColor(255, 255, 255));
    this->setAutoFillBackground(true);
    this->setPalette(pal);
    
    // 启用鼠标追踪（可选，如果需要hover效果）
    setMouseTracking(true);
}

void ImageWidget::setImage(const QImage &image)
{
    if (image.isNull()) return;

    // 加锁，确保线程安全
    QMutexLocker locker(&m_mutex);
    m_pixmap = QPixmap::fromImage(image);
    update();
}

void ImageWidget::setImage(cv::Mat& mat){
    setMat(mat);
}

void ImageWidget::setImage(Pylon::CGrabResultPtr ptrGrabResult, Basler_UniversalCameraParams::PixelFormatEnums type){
	cv::Mat cache;
	uint8_t* pylonImage = (uint8_t*)ptrGrabResult->GetBuffer();
	if (type == Basler_UniversalCameraParams::PixelFormat_Mono8 || type == Basler_UniversalCameraParams::PixelFormat_Mono10 || type == Basler_UniversalCameraParams::PixelFormat_Mono12 \
		|| type == Basler_UniversalCameraParams::PixelFormat_Mono16) {
		cache = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, pylonImage);
	}
	else {
		cache = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, pylonImage);
	}
	setImage(cache);
}

void ImageWidget::setImage(Pylon::CGrabResultPtr ptrGrabResult){
	uchar* test = (uchar*)(ptrGrabResult->GetBuffer());
	cv::Mat mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, test);
	setImage(mat);
}

void ImageWidget::setImage(QString path){
	std::string ch_pathname = path.toStdString();
	cv::Mat temp = cv::imread(ch_pathname);
	setImage(temp);
}

void ImageWidget::autoResize(){

}

void ImageWidget::setInteractMode(InteractMode){

}

void ImageWidget::setImageStatus(Status) {

}

void ImageWidget::setRoiSelectionEnabled(bool enabled) {
    m_roiSelectionEnabled = enabled;
    if (!enabled && m_isSelectingRoi) {
        m_isSelectingRoi = false;
        m_roiRubberRect = QRect();
        setCursor(Qt::ArrowCursor);
        update();
    }
}

void ImageWidget::cropToRoi(int x, int y, int width, int height) {
    {
        QMutexLocker locker(&m_mutex);
        if (m_pixmap.isNull() || width <= 0 || height <= 0) {
            return;
        }
        const QRect roi = QRect(x, y, width, height).intersected(m_pixmap.rect());
        if (roi.isEmpty()) {
            return;
        }
        m_pixmap = m_pixmap.copy(roi);
        m_scaleValue = 1.0;
        m_offset = QPointF(0, 0);
        m_roiRubberRect = QRect();
        m_isSelectingRoi = false;
    }
    fitToWindow();
    update();
}

cv::Mat ImageWidget::getImage(){
    // 加锁，确保线程安全
    QImage img;
    {
        QMutexLocker locker(&m_mutex);        
        // 将 QPixmap 转换为 QImage
        img = m_pixmap.toImage();
    }

    
    // 如果图像为空，返回空的 cv::Mat
    if (img.isNull()) {
        return cv::Mat();
    }
    
    // 将 QImage 转换为 cv::Mat
    cv::Mat mat;
    if (img.format() == QImage::Format_RGB888) {
        mat = cv::Mat(img.height(), img.width(), CV_8UC3, (void*)img.bits(), img.bytesPerLine()).clone();
    } else if (img.format() == QImage::Format_RGBA8888) {
        mat = cv::Mat(img.height(), img.width(), CV_8UC4, (void*)img.bits(), img.bytesPerLine()).clone();
    } else if (img.format() == QImage::Format_Grayscale8) {
        mat = cv::Mat(img.height(), img.width(), CV_8UC1, (void*)img.bits(), img.bytesPerLine()).clone();
    } else {
        // 对于其他格式，先转换为标准格式
        QImage converted = img.convertToFormat(QImage::Format_RGB888);
        mat = cv::Mat(converted.height(), converted.width(), CV_8UC3, (void*)converted.bits(), converted.bytesPerLine()).clone();
    }
    
    // 转换 BGR 到 RGB（因为 OpenCV 使用 BGR）
    if (mat.channels() == 3) {
        cv::Mat bgrMat;
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        return bgrMat;
    } else if (mat.channels() == 4) {
        cv::Mat bgraMat;
        cv::cvtColor(mat, bgraMat, cv::COLOR_RGBA2BGRA);
        return bgraMat;
    }
    
    return mat;
}

void ImageWidget::setMat(const cv::Mat &mat)
{
    if (mat.empty()) return;

    // OpenCV (BGR) -> Qt (RGB) 转换
    // 注意：这里为了演示简化了格式判断，实际使用需根据 mat.type() 做更全的判断
    QImage::Format format = QImage::Format_RGB888;
    cv::Mat rgbMat;
    
    if (mat.channels() == 3) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
    } else if (mat.channels() == 1) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_GRAY2RGB);
    } else {
        rgbMat = mat; // 假设其他情况
    }

    QImage img((const uchar*)rgbMat.data, 
               rgbMat.cols, rgbMat.rows, 
               rgbMat.step, 
               format);
               
    // 必须深拷贝，因为QImage默认引用Mat的数据，Mat释放后QImage会崩溃
    setImage(img.copy()); 
}

void ImageWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    
    QPixmap drawPixmap;
    QPointF drawOffset;
    qreal drawScale;

    {
        // 加锁，确保线程安全
        QMutexLocker locker(&m_mutex);
        if (m_pixmap.isNull()) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, "No Image Loaded");
            return;
        }

        drawPixmap = m_pixmap;
        drawOffset = m_offset;
        drawScale = m_scaleValue;
    }


    // 核心步骤：保存状态 -> 变换 -> 绘图 -> 恢复
    painter.save();

    // 1. 应用平移
    painter.translate(drawOffset);
    
    // 2. 应用缩放
    painter.scale(drawScale, drawScale);

    // 3. 设置渲染提示
    // SmoothPixmapTransform = false 意味着使用最近邻插值
    // 当放大倍数很大时，这就是“像素块”效果
    // 如果想要平滑效果，设为 true
    if (drawScale > 3.0) {
        // 放大超过3倍时，强制关闭平滑，显示像素格
        painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    } else {
        // 缩小时或倍数较小时，可以开启平滑防止锯齿，也可以一直关闭保持锐利
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true); 
        // 如果你完全想要PS那种硬边感觉，这里也可以设为 false
    }

    painter.drawPixmap(0, 0, drawPixmap);
    painter.restore();

    if (!m_roiRubberRect.isNull()) {
        painter.setRenderHint(QPainter::Antialiasing, false);
        painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
        painter.setBrush(QColor(255, 0, 0, 35));
        painter.drawRect(m_roiRubberRect.normalized());
    }
}

void ImageWidget::wheelEvent(QWheelEvent *event)
{
    if (m_pixmap.isNull()) return;

    // 计算缩放系数
    const double zoomFactor = 1.1; // 每次滚动缩放 10%
    double oldScale = m_scaleValue;
    
    if (event->angleDelta().y() > 0) {
        // 放大
        m_scaleValue *= zoomFactor;
    } else {
        // 缩小
        m_scaleValue /= zoomFactor;
    }

    // 限制最小/最大缩放倍数
    if (m_scaleValue < 0.05) m_scaleValue = 0.05;
    if (m_scaleValue > 50.0) m_scaleValue = 50.0;

    // 核心算法：以鼠标为中心缩放
    // 公式：NewOffset = Mouse - (Mouse - OldOffset) * (NewScale / OldScale)
    QPointF mousePos = event->position(); // Qt6使用position(), Qt5使用pos()
    m_offset = mousePos - (mousePos - m_offset) * (m_scaleValue / oldScale);

    checkBounds(); // 检查是否跑出边界
    update();
}

void ImageWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton && !m_pixmap.isNull()) {
        if (!m_roiSelectionEnabled) {
            emit rightClicked(event->pos());
            return;
        }
        m_isSelectingRoi = true;
        m_roiStartPos = event->pos();
        m_roiRubberRect = QRect(m_roiStartPos, m_roiStartPos);
        setCursor(Qt::CrossCursor);
        update();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        m_isDragging = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void ImageWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isSelectingRoi) {
        m_roiRubberRect = QRect(m_roiStartPos, event->pos()).normalized();
        update();
        return;
    }
    if (m_isDragging) {
        QPoint delta = event->pos() - m_lastMousePos;
        m_offset += delta;
        m_lastMousePos = event->pos();
        
        checkBounds();
        update();
    }
}

void ImageWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton && m_isSelectingRoi) {
        m_isSelectingRoi = false;
        const QRect widgetRect = m_roiRubberRect.normalized();
        m_roiRubberRect = QRect();
        setCursor(Qt::ArrowCursor);
        if (widgetRect.width() >= 2 && widgetRect.height() >= 2) {
            const QPointF imageTopLeft = widgetToImage(widgetRect.topLeft());
            const QPointF imageBottomRight = widgetToImage(widgetRect.bottomRight());
            const int x0 = std::clamp(static_cast<int>(std::floor(std::min(imageTopLeft.x(), imageBottomRight.x()))), 0, m_pixmap.width() - 1);
            const int y0 = std::clamp(static_cast<int>(std::floor(std::min(imageTopLeft.y(), imageBottomRight.y()))), 0, m_pixmap.height() - 1);
            const int x1 = std::clamp(static_cast<int>(std::ceil(std::max(imageTopLeft.x(), imageBottomRight.x()))), 0, m_pixmap.width());
            const int y1 = std::clamp(static_cast<int>(std::ceil(std::max(imageTopLeft.y(), imageBottomRight.y()))), 0, m_pixmap.height());
            const int width = std::max(0, x1 - x0);
            const int height = std::max(0, y1 - y0);
            if (width > 0 && height > 0) {
                int roi[4] = {x0, y0, width, height};
                emit sendRoi(roi);
                emit roiSelected(x0, y0, width, height);
            }
        }
        update();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        m_isDragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void ImageWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    // 如果需要窗口大小时重置图像，调用 fitToWindow();
    // 这里我们不重置，允许用户保留当前的查看状态
    checkBounds(); 
}

void ImageWidget::fitToWindow()
{
    if (m_pixmap.isNull()) return;
    
    // 计算适应窗口的缩放比例
    double winW = this->width();
    double winH = this->height();
    double imgW = m_pixmap.width();
    double imgH = m_pixmap.height();

    double scaleX = winW / imgW;
    double scaleY = winH / imgH;
    
    // 保持纵横比，取较小值
    m_scaleValue = qMin(scaleX, scaleY) * 0.9; // 留一点边距
    
    // 居中显示
    double newImgW = imgW * m_scaleValue;
    double newImgH = imgH * m_scaleValue;
    
    m_offset.setX((winW - newImgW) / 2.0);
    m_offset.setY((winH - newImgH) / 2.0);
}

void ImageWidget::checkBounds()
{
    if (m_pixmap.isNull()) return;

    // 计算当前显示的图像矩形
    double imgWidth = m_pixmap.width() * m_scaleValue;
    double imgHeight = m_pixmap.height() * m_scaleValue;

    // 规则：不要让图像完全跑出窗口
    // 只要图像的一定边缘还留在窗口内即可
    
    double w = this->width();
    double h = this->height();

    // 限制拖动范围：
    // 图像右边缘不能在窗口左边缘以左太远
    // 图像左边缘不能在窗口右边缘以右太远
    // 这里我们设定一个阈值，比如保留 50像素 在视野内
    double margin = 50.0; 

    if (m_offset.x() > w - margin) m_offset.setX(w - margin);
    if (m_offset.y() > h - margin) m_offset.setY(h - margin);
    
    if (m_offset.x() + imgWidth < margin) m_offset.setX(margin - imgWidth);
    if (m_offset.y() + imgHeight < margin) m_offset.setY(margin - imgHeight);
    
    // 可选：如果图片小于窗口，强制居中
    if (imgWidth < w) {
       // m_offset.setX((w - imgWidth) / 2);
    }
    if (imgHeight < h) {
       // m_offset.setY((h - imgHeight) / 2);
    }
}

void ImageWidget::stopContinousDisplay(){
    image_status = NO_IMAGE;
}

void ImageWidget::onReceiveDisplayImg(cv::Mat mat){
    this->setImage(mat);
}

QPointF ImageWidget::widgetToImage(const QPoint& pos) const {
    if (m_scaleValue <= 0.0) {
        return QPointF();
    }
    return (QPointF(pos) - m_offset) / m_scaleValue;
}
}
