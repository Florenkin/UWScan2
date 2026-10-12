#ifndef UWSCAN_SCANNER_IMAGEFILEUTILS_H
#define UWSCAN_SCANNER_IMAGEFILEUTILS_H

#include <QFileInfo>
#include <QImage>
#include <QString>

#include <opencv2/opencv.hpp>

#include "utils.h"

NAMESPACE_BEGIN {
namespace scanner {
namespace detail {

inline QImage matToQImage(const cv::Mat& source) {
    if (source.empty()) {
        return QImage();
    }

    cv::Mat image8;
    if (source.depth() == CV_8U) {
        image8 = source;
    } else {
        source.convertTo(image8, CV_MAKETYPE(CV_8U, source.channels()));
    }

    if (image8.channels() == 1) {
        return QImage(image8.data,
                      image8.cols,
                      image8.rows,
                      static_cast<int>(image8.step),
                      QImage::Format_Grayscale8)
            .copy();
    }

    cv::Mat rgb;
    if (image8.channels() == 3) {
        cv::cvtColor(image8, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data,
                      rgb.cols,
                      rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGB888)
            .copy();
    }
    if (image8.channels() == 4) {
        cv::cvtColor(image8, rgb, cv::COLOR_BGRA2RGBA);
        return QImage(rgb.data,
                      rgb.cols,
                      rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGBA8888)
            .copy();
    }

    return QImage();
}

inline bool saveImageFile(const QString& path, const cv::Mat& image) {
    if (QFileInfo(path).suffix().compare(QStringLiteral("bmp"), Qt::CaseInsensitive) == 0) {
        const QImage source = matToQImage(image);
        if (source.isNull()) {
            return false;
        }
        const QImage indexed = source.convertToFormat(
            QImage::Format_Indexed8,
            Qt::DiffuseDither | Qt::PreferDither);
        return indexed.save(path, "BMP");
    }

    return cv::imwrite(path.toStdString(), image);
}

} // namespace detail
} // namespace scanner
}

#endif // UWSCAN_SCANNER_IMAGEFILEUTILS_H
