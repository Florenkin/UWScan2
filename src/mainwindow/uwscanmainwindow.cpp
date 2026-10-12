#include "mainwindow/uwscanmainwindow.h"
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QCoreApplication>
#include <QDateTime>
#include <QStyle>
#include <QDesktopServices>
#include <QDialog>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QScreen>
#include <QWindow>
#include <QCollator>
#include <QInputDialog>
#include <QSet>
#include <QTextStream>
#include <QVBoxLayout>
#include <QVector>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include "pointcloud/pc.hpp"
#include "pointcloud/cloudwindow.h"
#include "pointcloud/filterdialog.h"
#include "setting/settingwidget.h"
#include "mainwindow/advancedstatusbar.h"
#include "scanner/scanner.h"
#include "scanner/associatedpointsmanager.h"
#include "setting/settingsmanager.h"
#include "server/commandserver.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

NAMESPACE_BEGIN { namespace mainwindow {
    static cv::Mat readImageFile(const QString& filePath, int flags = cv::IMREAD_UNCHANGED) {
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly)) {
            return cv::Mat();
        }

        const QByteArray data = file.readAll();
        if (data.isEmpty()) {
            return cv::Mat();
        }

        const std::vector<uchar> buffer(data.begin(), data.end());
        return cv::imdecode(buffer, flags);
    }

    static QString resolveCalibrationPath(const QString& configuredPath) {
        QFileInfo configured(configuredPath);
        if (configured.exists() && configured.isFile()) {
            return configured.absoluteFilePath();
        }

        const QStringList searchDirs = {
            QCoreApplication::applicationDirPath(),
            QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("docs"),
            QDir::current().absolutePath(),
            QDir::current().absoluteFilePath("docs"),
            QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../docs")
        };
        const QStringList filters = {"*.yml", "*.yaml"};
        for (const QString& dirPath : searchDirs) {
            QDir dir(dirPath);
            if (!dir.exists()) {
                continue;
            }
            const QFileInfoList files = dir.entryInfoList(filters, QDir::Files | QDir::Readable, QDir::Name);
            if (!files.isEmpty()) {
                return files.first().absoluteFilePath();
            }
        }
        return configuredPath;
    }

    static void populateReconstructionConfig(scanner::ReconstructionConfig& config, setting::SettingsManager* settingsManager) {
        const QString calibrationPath = resolveCalibrationPath(settingsManager->getString(setting::DEFAULT_CALIBRATE_FILE));
        config.calibrationPath = calibrationPath.toStdString();
        const QString method = settingsManager->getString(setting::DEFAULT_CENTERLINE_METHOD, "GrayScale灰度中心法");
        config.centerlineMethod = (method == "Steger") ? 1 : 0;
        config.bwThr = settingsManager->getDouble(setting::DEFAULT_BW_THR, 55.0);
        config.sumThr = settingsManager->getDouble(setting::DEFAULT_SUM_THR, 124.0);
        config.isHeng = settingsManager->getBool(setting::DEFAULT_IS_HENG, true);
        config.thres = static_cast<int>(config.bwThr);
        config.slt_thres = static_cast<int>(config.sumThr);
    }

    struct LineFitResult {
        bool ok = false;
        double vx = 0.0;
        double vy = 0.0;
        double x0 = 0.0;
        double y0 = 0.0;
        double me = 0.0;
        double rmse = 0.0;
    };

    static LineFitResult fitLineAndErrors(const std::vector<cv::Point2d>& points) {
        LineFitResult result;
        if (points.size() < 2) {
            return result;
        }
        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
        result.vx = line[0];
        result.vy = line[1];
        result.x0 = line[2];
        result.y0 = line[3];
        const double norm = std::hypot(result.vx, result.vy);
        if (norm <= 1e-9) {
            return result;
        }

        double sum = 0.0;
        double sumSq = 0.0;
        for (const auto& point : points) {
            const double distance = std::abs(result.vy * (point.x - result.x0) - result.vx * (point.y - result.y0)) / norm;
            sum += distance;
            sumSq += distance * distance;
        }
        result.me = sum / static_cast<double>(points.size());
        result.rmse = std::sqrt(sumSq / static_cast<double>(points.size()));
        result.ok = true;
        return result;
    }

    static cv::Mat createCenterlineOverlay(const cv::Mat& source,
                                           const std::vector<cv::Point2d>& points,
                                           const scanner::ReconstructionConfig& config,
                                           bool showCenters,
                                           bool showValidPixels,
                                           bool showFitLine,
                                           const LineFitResult& fit) {
        if (source.empty()) {
            return cv::Mat();
        }

        cv::Mat overlay;
        if (source.channels() == 1) {
            cv::cvtColor(source, overlay, cv::COLOR_GRAY2BGR);
        } else {
            overlay = source.clone();
        }

        const int offsetX = 0;
        const int offsetY = 0;
        const bool hasRoi = !config.isFullRoi && config.Roi.size() >= 4;
        cv::Rect roiRect(0, 0, overlay.cols, overlay.rows);
        if (hasRoi) {
            roiRect = cv::Rect(config.Roi[0], config.Roi[1], config.Roi[2], config.Roi[3]) & cv::Rect(0, 0, overlay.cols, overlay.rows);
            cv::rectangle(overlay, roiRect, cv::Scalar(255, 180, 0), 1);
        }

        if (showValidPixels) {
            cv::Mat gray;
            if (source.channels() == 1) {
                gray = source;
            } else {
                cv::cvtColor(source, gray, cv::COLOR_BGR2GRAY);
            }
            cv::Rect validRect = hasRoi ? roiRect : cv::Rect(0, 0, gray.cols, gray.rows);
            if (!validRect.empty()) {
                const double threshold = std::clamp(config.bwThr, 0.0, 255.0);
                for (int y = validRect.y; y < validRect.y + validRect.height; ++y) {
                    const uchar* srcRow = gray.ptr<uchar>(y);
                    cv::Vec3b* dstRow = overlay.ptr<cv::Vec3b>(y);
                    for (int x = validRect.x; x < validRect.x + validRect.width; ++x) {
                        if (srcRow[x] >= threshold) {
                            dstRow[x] = cv::Vec3b(0, 120, 255);
                        }
                    }
                }
            }
        }

        if (showCenters) {
            for (const auto& point : points) {
                const int x = static_cast<int>(std::lround(point.x)) - offsetX;
                const int y = static_cast<int>(std::lround(point.y)) - offsetY;
                if (x < 0 || y < 0 || x >= overlay.cols || y >= overlay.rows) {
                    continue;
                }
                overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
            }
        }

        if (showFitLine && fit.ok) {
            const double scale = std::max(overlay.cols, overlay.rows);
            cv::Point p1(static_cast<int>(std::lround(fit.x0 - fit.vx * scale)),
                         static_cast<int>(std::lround(fit.y0 - fit.vy * scale)));
            cv::Point p2(static_cast<int>(std::lround(fit.x0 + fit.vx * scale)),
                         static_cast<int>(std::lround(fit.y0 + fit.vy * scale)));
            cv::line(overlay, p1, p2, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        }

        return overlay;
    }

    static cv::Vec3b jetColor(double normalized) {
        const double v = std::clamp(normalized, 0.0, 1.0);
        const std::array<std::pair<double, cv::Vec3d>, 4> stops = {{
            {0.0, cv::Vec3d(255.0, 0.0, 0.0)},
            {1.0 / 3.0, cv::Vec3d(0.0, 255.0, 0.0)},
            {2.0 / 3.0, cv::Vec3d(0.0, 255.0, 255.0)},
            {1.0, cv::Vec3d(0.0, 0.0, 255.0)}
        }};
        for (size_t i = 1; i < stops.size(); ++i) {
            if (v <= stops[i].first) {
                const double leftPos = stops[i - 1].first;
                const double rightPos = stops[i].first;
                const double t = (v - leftPos) / std::max(rightPos - leftPos, 1e-9);
                const cv::Vec3d color = stops[i - 1].second * (1.0 - t) + stops[i].second * t;
                return cv::Vec3b(static_cast<uchar>(std::clamp(color[0], 0.0, 255.0)),
                                 static_cast<uchar>(std::clamp(color[1], 0.0, 255.0)),
                                 static_cast<uchar>(std::clamp(color[2], 0.0, 255.0)));
            }
        }
        const cv::Vec3d& color = stops.back().second;
        return cv::Vec3b(static_cast<uchar>(color[0]),
                         static_cast<uchar>(color[1]),
                         static_cast<uchar>(color[2]));
    }

    static cv::Mat createDepthPreview(const std::vector<Eigen::Vector3f>& points, int depthAxis, int imageSize = 900) {
        if (points.empty()) {
            return cv::Mat();
        }
        std::array<float, 3> minValues = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        std::array<float, 3> maxValues = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
        for (const auto& point : points) {
            const std::array<float, 3> values = {point.x(), point.y(), point.z()};
            for (int i = 0; i < 3; ++i) {
                minValues[i] = std::min(minValues[i], values[i]);
                maxValues[i] = std::max(maxValues[i], values[i]);
            }
        }

        cv::Mat image(imageSize, imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat filled(imageSize, imageSize, CV_8UC1, cv::Scalar(0));
        const int axisU = (depthAxis + 1) % 3;
        const int axisV = (depthAxis + 2) % 3;
        const double rangeU = std::max(static_cast<double>(maxValues[axisU] - minValues[axisU]), 1e-9);
        const double rangeV = std::max(static_cast<double>(maxValues[axisV] - minValues[axisV]), 1e-9);
        const double rangeDepth = std::max(static_cast<double>(maxValues[depthAxis] - minValues[depthAxis]), 1e-9);
        for (const auto& point : points) {
            const std::array<float, 3> values = {point.x(), point.y(), point.z()};
            const int u = static_cast<int>(std::lround((values[axisU] - minValues[axisU]) / rangeU * (imageSize - 1)));
            const int v = imageSize - 1 - static_cast<int>(std::lround((values[axisV] - minValues[axisV]) / rangeV * (imageSize - 1)));
            if (u < 0 || v < 0 || u >= imageSize || v >= imageSize) {
                continue;
            }
            image.at<cv::Vec3b>(v, u) = jetColor((values[depthAxis] - minValues[depthAxis]) / rangeDepth);
            filled.at<uchar>(v, u) = 255;
        }
        cv::Mat dilated;
        cv::dilate(image, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        dilated.copyTo(image, filled == 0);
        return image;
    }

    static cv::Mat createDepthPreviewFromAssociatedPoints(const cv::Mat& sourceImage,
                                                          const std::vector<scanner::AssociatedPoints>& points,
                                                          int depthAxis,
                                                          cv::Mat* depthAccumImage = nullptr) {
        if (sourceImage.empty() || points.empty()) {
            return cv::Mat();
        }

        auto buildFrontView = [&sourceImage, &points]() {
            float minX = std::numeric_limits<float>::max();
            float maxX = std::numeric_limits<float>::lowest();
            for (const auto& point : points) {
                minX = std::min(minX, point.reconstructedPoint.x);
                maxX = std::max(maxX, point.reconstructedPoint.x);
            }
            const double rangeX = std::max(static_cast<double>(maxX - minX), 1e-9);

            cv::Mat frontValue(sourceImage.size(), CV_8UC1, cv::Scalar(0));
            for (const auto& point : points) {
                const int x = static_cast<int>(std::lround(point.distortedCenter.x));
                const int y = static_cast<int>(std::lround(point.distortedCenter.y));
                if (x < 0 || y < 0 || x >= frontValue.cols || y >= frontValue.rows) {
                    continue;
                }
                const uchar value = static_cast<uchar>(std::clamp(
                    std::lround((point.reconstructedPoint.x - minX) / rangeX * 254.0) + 1l,
                    1l,
                    255l));
                frontValue.at<uchar>(y, x) = std::max(frontValue.at<uchar>(y, x), value);
            }
            cv::dilate(frontValue, frontValue, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
            cv::Mat color(frontValue.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            for (int row = 0; row < frontValue.rows; ++row) {
                const uchar* src = frontValue.ptr<uchar>(row);
                for (int col = 0; col < frontValue.cols; ++col) {
                    const uchar value = src[col];
                    if (value == 0) {
                        continue;
                    }
                    color.at<cv::Vec3b>(row, col) = jetColor(value / 255.0);
                }
            }
            return color;
        };

        if (depthAxis == 0) {
            return buildFrontView();
        }
        if (depthAxis == 1) {
            cv::Mat rotated;
            cv::rotate(buildFrontView(), rotated, cv::ROTATE_90_CLOCKWISE);
            return rotated;
        }

        std::array<float, 3> minValues = {
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()
        };
        std::array<float, 3> maxValues = {
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest()
        };
        for (const auto& point : points) {
            const std::array<float, 3> values = {
                point.reconstructedPoint.x,
                point.reconstructedPoint.y,
                point.reconstructedPoint.z
            };
            for (int i = 0; i < 3; ++i) {
                minValues[i] = std::min(minValues[i], values[i]);
                maxValues[i] = std::max(maxValues[i], values[i]);
            }
        }

        const int sideWidth = std::max(sourceImage.cols, sourceImage.rows);
        const int sideHeight = std::max(240, std::min(sourceImage.cols, sourceImage.rows));
        cv::Mat normalizedDepth(sideHeight, sideWidth, CV_8UC1, cv::Scalar(0));
        const double rangeX = std::max(static_cast<double>(maxValues[0] - minValues[0]), 1e-9);
        const double rangeY = std::max(static_cast<double>(maxValues[1] - minValues[1]), 1e-9);
        const int horizontalAxis = rangeX >= rangeY ? 0 : 1;
        const int colorAxis = horizontalAxis == 0 ? 1 : 0;
        const double rangeHorizontal = std::max(static_cast<double>(maxValues[horizontalAxis] - minValues[horizontalAxis]), 1e-9);
        const double rangeVertical = std::max(static_cast<double>(maxValues[2] - minValues[2]), 1e-9);
        const double rangeColor = std::max(static_cast<double>(maxValues[colorAxis] - minValues[colorAxis]), 1e-9);
        for (const auto& point : points) {
            const std::array<float, 3> values = {
                point.reconstructedPoint.x,
                point.reconstructedPoint.y,
                point.reconstructedPoint.z
            };
            const int x = static_cast<int>(std::lround(
                (values[horizontalAxis] - minValues[horizontalAxis]) / rangeHorizontal * (sideWidth - 1)));
            const int y = sideHeight - 1 - static_cast<int>(std::lround(
                (values[2] - minValues[2]) / rangeVertical * (sideHeight - 1)));
            if (x < 0 || y < 0 || x >= normalizedDepth.cols || y >= normalizedDepth.rows) {
                continue;
            }
            const uchar value = static_cast<uchar>(std::clamp(
                std::lround((values[colorAxis] - minValues[colorAxis]) / rangeColor * 254.0) + 1l,
                1l,
                255l));
            normalizedDepth.at<uchar>(y, x) = std::max(normalizedDepth.at<uchar>(y, x), value);
        }

        if (depthAccumImage) {
            if (depthAccumImage->empty() || depthAccumImage->size() != normalizedDepth.size()) {
                *depthAccumImage = cv::Mat(normalizedDepth.size(), CV_8UC1, cv::Scalar(0));
            }
            cv::max(*depthAccumImage, normalizedDepth, *depthAccumImage);
        }

        const cv::Mat& depthGray = (depthAccumImage && !depthAccumImage->empty()) ? *depthAccumImage : normalizedDepth;
        cv::Mat displayColor(depthGray.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < depthGray.rows; ++y) {
            const uchar* row = depthGray.ptr<uchar>(y);
            for (int x = 0; x < depthGray.cols; ++x) {
                const uchar depthValue = row[x];
                if (depthValue == 0) {
                    continue;
                }
                displayColor.at<cv::Vec3b>(y, x) = jetColor(depthValue / 255.0);
            }
        }
        cv::dilate(displayColor, displayColor, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        return displayColor;
    }

    struct FitStats {
        bool ok = false;
        QString name;
        double radius = 0.0;
        double diameter = 0.0;
        double standardDiameter = 0.0;
        double meanError = 0.0;
        double rmse = 0.0;
        double maxAbsError = 0.0;
        double errorParameter = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> errors;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
        double axisMin = 0.0;
        double axisMax = 0.0;
    };

    struct ErrorAnalysisRow {
        QString featureName;
        double standardDiameter = 0.0;
        double fittedDiameter = 0.0;
        double meanError = 0.0;
        double rmse = 0.0;
        double errorParameter = std::numeric_limits<double>::quiet_NaN();
        int sampleCount = 0;
    };

    static QRect availableScreenGeometryForWidget(const QWidget* widget) {
        if (widget) {
            if (const QScreen* screen = widget->screen()) {
                return screen->availableGeometry();
            }
            if (const QWindow* window = widget->windowHandle()) {
                if (const QScreen* screen = window->screen()) {
                    return screen->availableGeometry();
                }
            }
        }
        if (const QScreen* primary = QGuiApplication::primaryScreen()) {
            return primary->availableGeometry();
        }
        return QRect(0, 0, 1600, 900);
    }

    static QSize fitChartSizeForScreen(const QWidget* widget,
                                       int preferredWidth,
                                       int preferredHeight,
                                       int minWidth,
                                       int minHeight) {
        const QRect available = availableScreenGeometryForWidget(widget);
        const int maxWidth = std::max(minWidth, available.width() - 120);
        const int maxHeight = std::max(minHeight, available.height() - 180);
        return QSize(std::clamp(preferredWidth, minWidth, maxWidth),
                     std::clamp(preferredHeight, minHeight, maxHeight));
    }

    static void applyAdaptiveDialogSize(QDialog& dialog,
                                        const QWidget* reference,
                                        int preferredWidth,
                                        int preferredHeight,
                                        int minWidth,
                                        int minHeight) {
        const QRect available = availableScreenGeometryForWidget(reference ? reference : &dialog);
        const QSize target = fitChartSizeForScreen(reference ? reference : &dialog,
                                                   preferredWidth,
                                                   preferredHeight,
                                                   minWidth,
                                                   minHeight);
        dialog.resize(target);
        dialog.setMinimumSize(QSize(minWidth, minHeight));
        dialog.setMaximumSize(available.width() - 24, available.height() - 24);
    }

    static QHash<QString, QVector<FitStats>> g_cloud_fit_error_stats;
    static QHash<QString, FitCloudContext> g_fit_cloud_contexts;
    static const QString kLastStandardDiameterKey = QStringLiteral("LastStandardDiameter");

    static bool hasErrorParameterValue(double value) {
        return std::isfinite(value);
    }

    static bool hasErrorParameter(const FitStats& stats) {
        return hasErrorParameterValue(stats.errorParameter);
    }

    static bool hasErrorParameter(const ErrorAnalysisRow& row) {
        return hasErrorParameterValue(row.errorParameter);
    }

    struct PlaneFitStats {
        bool ok = false;
        QString name;
        Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
        float d = 0.0f;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        int inlierCount = 0;
        double meanError = 0.0;
        double rmse = 0.0;
        double maxAbsError = 0.0;
        double minDeviation = 0.0;
        double maxDeviation = 0.0;
        double flatness = 0.0;
    };

    struct PlaneDefectCropStats {
        bool ok = false;
        PlaneFitStats plane;
        Eigen::Vector3f origin = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisX = Eigen::Vector3f::UnitX();
        Eigen::Vector3f axisY = Eigen::Vector3f::UnitY();
        bool defectOnNegativeSide = true;
        double seedThreshold = 0.0;
        Eigen::Vector2f minUv = Eigen::Vector2f::Zero();
        Eigen::Vector2f maxUv = Eigen::Vector2f::Zero();
        int seedPointCount = 0;
        int retainedPointCount = 0;
        std::vector<int> retainedIndices;
    };

    struct PlaneDefectMeasurement {
        bool ok = false;
        int index = -1;
        PlaneFitStats plane;
        Eigen::Vector3f origin = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisX = Eigen::Vector3f::UnitX();
        Eigen::Vector3f axisY = Eigen::Vector3f::UnitY();
        double depth = 0.0;
        double area = 0.0;
        double convexHullArea = 0.0;
        double length = 0.0;
        double width = 0.0;
        double minDeviation = 0.0;
        double maxDeviation = 0.0;
        double corePatchDepth = 0.0;
        double profileDepth = 0.0;
        double trimmedDepth = 0.0;
        double sideDepth = 0.0;
        double bottomPlaneDepth = 0.0;
        double fusedDepth = 0.0;
        double consensusDepth = 0.0;
        Eigen::Vector2f minUv = Eigen::Vector2f::Zero();
        Eigen::Vector2f maxUv = Eigen::Vector2f::Zero();
        std::array<Eigen::Vector2f, 4> rectangleUv = {
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero()
        };
        bool hasRectangle = false;
        std::vector<int> pointIndices;
    };

    struct PlaneDefectAnalysisResult {
        bool ok = false;
        PlaneFitStats plane;
        Eigen::Vector3f origin = Eigen::Vector3f::Zero();
        Eigen::Vector3f axisX = Eigen::Vector3f::UnitX();
        Eigen::Vector3f axisY = Eigen::Vector3f::UnitY();
        bool defectOnNegativeSide = true;
        double threshold = 0.0;
        std::vector<PlaneDefectMeasurement> defects;
    };

    static double promptStandardDiameterValue(QWidget* parent,
                                              const QString& title,
                                              const QString& label,
                                              double minimumValue,
                                              bool allowZero,
                                              bool* ok,
                                              double fallbackValue = -1.0) {
        auto* settings = setting::SettingsManager::getInstance();
        double defaultValue = fallbackValue >= 0.0
            ? fallbackValue
            : settings->getDouble(kLastStandardDiameterKey, 0.0);
        if (!allowZero && defaultValue <= minimumValue) {
            defaultValue = std::max(minimumValue, 0.0001);
        }
        const double lower = allowZero ? 0.0 : minimumValue;
        const double value = QInputDialog::getDouble(parent, title, label, defaultValue, lower, 1000000.0, 4, ok);
        if (ok && *ok && value >= lower) {
            settings->setDouble(kLastStandardDiameterKey, value);
            settings->sync();
        }
        return value;
    }

    static pcl::PointCloud<pcl::PointXYZ>::Ptr toPclCloud(const std::vector<Eigen::Vector3f>& points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(points.size());
        for (const auto& point : points) {
            cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
        return cloud;
    }

    static void fillErrorStats(FitStats& stats) {
        if (stats.errors.empty()) {
            return;
        }
        double sum = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        for (double error : stats.errors) {
            sum += error;
            sumSq += error * error;
            maxAbs = std::max(maxAbs, std::abs(error));
        }
        stats.meanError = sum / static_cast<double>(stats.errors.size());
        stats.rmse = std::sqrt(sumSq / static_cast<double>(stats.errors.size()));
        stats.maxAbsError = maxAbs;
    }

    static FitStats fitSphereStats(const std::vector<Eigen::Vector3f>& points, const QString& name = "Sphere") {
        FitStats stats;
        stats.name = name;
        if (points.size() < 20) {
            return stats;
        }

        auto cloud = toPclCloud(points);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setMaxIterations(1000);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (coefficients->values.size() < 4 || inliers->indices.empty()) {
            return stats;
        }

        const Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        stats.center = center;
        stats.radius = coefficients->values[3];
        stats.diameter = stats.radius * 2.0;
        stats.errors.reserve(points.size());
        for (const auto& p : points) {
            stats.errors.push_back((p - center).norm() - stats.radius);
        }
        fillErrorStats(stats);
        stats.ok = true;
        return stats;
    }

    static FitStats fitCylinderStats(const std::vector<Eigen::Vector3f>& points) {
        FitStats stats;
        stats.name = "Cylinder";
        if (points.size() < 50) {
            return stats;
        }

        auto cloud = toPclCloud(points);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normalEstimation.setSearchMethod(tree);
        normalEstimation.setInputCloud(cloud);
        normalEstimation.setKSearch(20);
        normalEstimation.compute(*normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.1);
        seg.setRadiusLimits(0.0, 100000.0);
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.segment(*inliers, *coefficients);
        if (coefficients->values.size() < 7 || inliers->indices.empty()) {
            return stats;
        }

        const Eigen::Vector3f axisPoint(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f axis(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        if (axis.norm() < 1e-9f) {
            return stats;
        }
        axis.normalize();
        stats.axisPoint = axisPoint;
        stats.axis = axis;
        stats.radius = coefficients->values[6];
        stats.diameter = stats.radius * 2.0;
        stats.axisMin = std::numeric_limits<double>::max();
        stats.axisMax = std::numeric_limits<double>::lowest();
        stats.errors.reserve(points.size());
        for (const auto& point : points) {
            const Eigen::Vector3f delta = point - axisPoint;
            const double axisCoord = delta.dot(axis);
            const double distance = (delta - delta.dot(axis) * axis).norm();
            stats.errors.push_back(distance - stats.radius);
            stats.axisMin = std::min(stats.axisMin, axisCoord);
            stats.axisMax = std::max(stats.axisMax, axisCoord);
        }
        if (!std::isfinite(stats.axisMin) || !std::isfinite(stats.axisMax) || stats.axisMax <= stats.axisMin) {
            stats.axisMin = -stats.radius;
            stats.axisMax = stats.radius;
        }
        fillErrorStats(stats);
        stats.ok = true;
        return stats;
    }

    static PlaneFitStats fitPlaneStats(const std::vector<Eigen::Vector3f>& points, const QString& name = "Plane") {
        PlaneFitStats stats;
        stats.name = name;
        if (points.size() < 20) {
            return stats;
        }

        auto cloud = toPclCloud(points);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setMaxIterations(1000);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (coefficients->values.size() < 4 || inliers->indices.empty()) {
            return stats;
        }

        Eigen::Vector3f ransacNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        if (ransacNormal.norm() < 1e-6f) {
            return stats;
        }
        ransacNormal.normalize();

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        int validInlierCount = 0;
        for (const int index : inliers->indices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            centroid += points[static_cast<size_t>(index)];
            ++validInlierCount;
        }
        if (validInlierCount < 3) {
            return stats;
        }
        centroid /= static_cast<float>(validInlierCount);

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const int index : inliers->indices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f delta = points[static_cast<size_t>(index)] - centroid;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<float>(validInlierCount);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        if (solver.info() != Eigen::Success) {
            return stats;
        }
        Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
        if (normal.dot(ransacNormal) < 0.0f) {
            normal = -normal;
        }

        stats.normal = normal;
        stats.d = -normal.dot(centroid);
        stats.centroid = centroid;
        stats.inlierCount = validInlierCount;

        double sum = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        double minDeviation = std::numeric_limits<double>::max();
        double maxDeviation = std::numeric_limits<double>::lowest();
        for (const int index : inliers->indices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            const double distance = static_cast<double>(normal.dot(point) + stats.d);
            const double absDistance = std::abs(distance);
            sum += distance;
            sumSq += distance * distance;
            maxAbs = std::max(maxAbs, absDistance);
            minDeviation = std::min(minDeviation, distance);
            maxDeviation = std::max(maxDeviation, distance);
        }
        stats.meanError = sum / static_cast<double>(validInlierCount);
        stats.rmse = std::sqrt(sumSq / static_cast<double>(validInlierCount));
        stats.maxAbsError = maxAbs;
        stats.minDeviation = minDeviation;
        stats.maxDeviation = maxDeviation;
        stats.flatness = maxDeviation - minDeviation;
        stats.ok = true;
        return stats;
    }

    static double planeAngleDegrees(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        const double dotValue = std::clamp(std::abs(static_cast<double>(a.normalized().dot(b.normalized()))), 0.0, 1.0);
        return std::acos(dotValue) * kRadToDeg;
    }

    static std::pair<Eigen::Vector3f, Eigen::Vector3f> planeAxesFromNormal(const Eigen::Vector3f& normal) {
        Eigen::Vector3f helper = std::abs(normal.z()) < 0.9f ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
        Eigen::Vector3f axisX = helper.cross(normal);
        if (axisX.norm() < 1e-6f) {
            helper = Eigen::Vector3f::UnitY();
            axisX = helper.cross(normal);
        }
        axisX.normalize();
        Eigen::Vector3f axisY = normal.cross(axisX).normalized();
        return {axisX, axisY};
    }

    static Eigen::Vector2f projectPointToPlane2d(const Eigen::Vector3f& point,
                                                 const Eigen::Vector3f& origin,
                                                 const Eigen::Vector3f& axisX,
                                                 const Eigen::Vector3f& axisY) {
        const Eigen::Vector3f delta = point - origin;
        return Eigen::Vector2f(delta.dot(axisX), delta.dot(axisY));
    }

    static size_t percentileIndex(size_t size, double fraction) {
        if (size == 0) {
            return 0;
        }
        return std::min(size - 1,
                        static_cast<size_t>(std::lround(std::clamp(fraction, 0.0, 1.0)
                                                        * static_cast<double>(size - 1))));
    }

    static double sortedPercentile(const std::vector<float>& sortedValues, double fraction) {
        if (sortedValues.empty()) {
            return 0.0;
        }
        if (sortedValues.size() == 1) {
            return static_cast<double>(sortedValues.front());
        }
        const double clamped = std::clamp(fraction, 0.0, 1.0);
        const double scaledIndex = clamped * static_cast<double>(sortedValues.size() - 1);
        const size_t lowIndex = static_cast<size_t>(std::floor(scaledIndex));
        const size_t highIndex = static_cast<size_t>(std::ceil(scaledIndex));
        if (lowIndex >= sortedValues.size()) {
            return static_cast<double>(sortedValues.back());
        }
        if (lowIndex == highIndex || highIndex >= sortedValues.size()) {
            return static_cast<double>(sortedValues[lowIndex]);
        }
        const double weightHigh = scaledIndex - static_cast<double>(lowIndex);
        const double weightLow = 1.0 - weightHigh;
        return static_cast<double>(sortedValues[lowIndex]) * weightLow
            + static_cast<double>(sortedValues[highIndex]) * weightHigh;
    }

    static double robustDefectDepth(std::vector<float> signedDistances, bool negativeSide) {
        if (signedDistances.empty()) {
            return 0.0;
        }
        std::sort(signedDistances.begin(), signedDistances.end());
        const double fraction = negativeSide ? 0.10 : 0.90;
        return std::abs(sortedPercentile(signedDistances, fraction));
    }

    static double robustMedian(std::vector<float> values) {
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const size_t mid = values.size() / 2;
        if ((values.size() % 2) == 0 && mid > 0) {
            return (static_cast<double>(values[mid - 1]) + static_cast<double>(values[mid])) * 0.5;
        }
        return static_cast<double>(values[mid]);
    }

    static double robustMad(const std::vector<float>& values, double center) {
        if (values.empty()) {
            return 0.0;
        }
        std::vector<float> deviations;
        deviations.reserve(values.size());
        for (const float value : values) {
            deviations.push_back(static_cast<float>(std::abs(static_cast<double>(value) - center)));
        }
        return robustMedian(std::move(deviations));
    }

    static double robustMadFromResiduals(const std::vector<double>& residuals) {
        if (residuals.empty()) {
            return 0.0;
        }
        std::vector<float> absResiduals;
        absResiduals.reserve(residuals.size());
        for (double residual : residuals) {
            absResiduals.push_back(static_cast<float>(std::abs(residual)));
        }
        return robustMedian(std::move(absResiduals));
    }

    static double robustMean(std::vector<float> values, double trimFraction = 0.15) {
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const double clampedTrim = std::clamp(trimFraction, 0.0, 0.45);
        const size_t trimCount = std::min(values.size() / 2,
                                          static_cast<size_t>(std::floor(static_cast<double>(values.size()) * clampedTrim)));
        const size_t beginIndex = trimCount;
        const size_t endIndex = values.size() - trimCount;
        if (beginIndex >= endIndex) {
            return robustMedian(std::move(values));
        }

        double sum = 0.0;
        for (size_t i = beginIndex; i < endIndex; ++i) {
            sum += static_cast<double>(values[i]);
        }
        return sum / static_cast<double>(endIndex - beginIndex);
    }

    static double positivePercentile(std::vector<float> values, double fraction) {
        values.erase(std::remove_if(values.begin(), values.end(), [](float value) {
                         return !std::isfinite(value) || value <= 0.0f;
                     }),
                     values.end());
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        return sortedPercentile(values, fraction);
    }

    static double estimatePlaneDefectDepthFromBands(std::vector<float> signedDistances,
                                                    bool negativeSide,
                                                    double fallbackScale) {
        if (signedDistances.size() < 8) {
            return 0.0;
        }

        std::sort(signedDistances.begin(), signedDistances.end());
        const double medianDistance = robustMedian(signedDistances);
        const double mad = robustMad(signedDistances, medianDistance);
        const double robustScale = std::max({mad * 1.4826, fallbackScale, 0.003});
        const double referenceBand = std::max(robustScale * 1.45, fallbackScale * 0.75);
        const double separationBand = std::max(robustScale * 0.95, fallbackScale * 0.50);

        std::vector<float> referenceValues;
        std::vector<float> bottomValues;
        referenceValues.reserve(signedDistances.size());
        bottomValues.reserve(signedDistances.size());
        for (float value : signedDistances) {
            const double distance = static_cast<double>(value);
            const bool isReference = std::abs(distance - medianDistance) <= referenceBand;
            const bool isBottom = negativeSide
                ? (distance <= medianDistance - separationBand)
                : (distance >= medianDistance + separationBand);
            if (isReference) {
                referenceValues.push_back(value);
            }
            if (isBottom) {
                bottomValues.push_back(value);
            }
        }

        if (referenceValues.size() < 6) {
            for (float value : signedDistances) {
                const double distance = static_cast<double>(value);
                const bool keep = negativeSide
                    ? (distance >= sortedPercentile(signedDistances, 0.55))
                    : (distance <= sortedPercentile(signedDistances, 0.45));
                if (keep) {
                    referenceValues.push_back(value);
                }
            }
        }
        if (bottomValues.size() < 6) {
            for (float value : signedDistances) {
                const double distance = static_cast<double>(value);
                const bool keep = negativeSide
                    ? (distance <= sortedPercentile(signedDistances, 0.22))
                    : (distance >= sortedPercentile(signedDistances, 0.78));
                if (keep) {
                    bottomValues.push_back(value);
                }
            }
        }

        if (referenceValues.empty() || bottomValues.empty()) {
            return 0.0;
        }

        const double referenceLevel = robustMean(referenceValues, 0.18);
        const double bottomLevel = robustMean(bottomValues, 0.12);
        const double shoulderLevel = static_cast<double>(sortedPercentile(bottomValues, negativeSide ? 0.26 : 0.74));
        const double extremeLevel = static_cast<double>(sortedPercentile(bottomValues, negativeSide ? 0.10 : 0.90));
        const double bottomDepth = negativeSide
            ? (referenceLevel - bottomLevel)
            : (bottomLevel - referenceLevel);
        const double shoulderDepth = negativeSide
            ? (referenceLevel - shoulderLevel)
            : (shoulderLevel - referenceLevel);
        const double extremeDepth = negativeSide
            ? (referenceLevel - extremeLevel)
            : (extremeLevel - referenceLevel);
        return std::max({bottomDepth,
                         shoulderDepth * 0.92,
                         extremeDepth * 0.80,
                         0.0});
    }

    static double estimatePlaneDefectDepthFromLocalProfile(const std::vector<Eigen::Vector3f>& points,
                                                           const PlaneDefectCropStats& cropStats,
                                                           const std::vector<int>& defectPointIndices,
                                                           const std::vector<Eigen::Vector2f>& defectUvs,
                                                           const Eigen::Vector2f& defectMinUv,
                                                           const Eigen::Vector2f& defectMaxUv,
                                                           const PlaneFitStats& referencePlane,
                                                           bool negativeSide) {
        if (!referencePlane.ok || defectPointIndices.empty() || defectPointIndices.size() != defectUvs.size()) {
            return 0.0;
        }

        const Eigen::Vector2f defectSpan = defectMaxUv - defectMinUv;
        const Eigen::Vector2f globalSpan = cropStats.maxUv - cropStats.minUv;
        const float outerPadU = std::max(defectSpan.x() * 0.85f, std::max(globalSpan.x() * 0.025f, 0.05f));
        const float outerPadV = std::max(defectSpan.y() * 0.85f, std::max(globalSpan.y() * 0.025f, 0.05f));
        const float innerPadU = std::max(defectSpan.x() * 0.12f, std::max(globalSpan.x() * 0.008f, 0.015f));
        const float innerPadV = std::max(defectSpan.y() * 0.12f, std::max(globalSpan.y() * 0.008f, 0.015f));
        const float outerMinU = defectMinUv.x() - outerPadU;
        const float outerMaxU = defectMaxUv.x() + outerPadU;
        const float outerMinV = defectMinUv.y() - outerPadV;
        const float outerMaxV = defectMaxUv.y() + outerPadV;
        const float innerMinU = defectMinUv.x() - innerPadU;
        const float innerMaxU = defectMaxUv.x() + innerPadU;
        const float innerMinV = defectMinUv.y() - innerPadV;
        const float innerMaxV = defectMaxUv.y() + innerPadV;

        std::vector<char> excluded(points.size(), 0);
        for (int pointIndex : defectPointIndices) {
            if (pointIndex >= 0 && pointIndex < static_cast<int>(points.size())) {
                excluded[static_cast<size_t>(pointIndex)] = 1;
            }
        }

        std::vector<float> referenceRingDistances;
        referenceRingDistances.reserve(cropStats.retainedIndices.size());
        for (int pointIndex : cropStats.retainedIndices) {
            if (pointIndex < 0 || pointIndex >= static_cast<int>(points.size())) {
                continue;
            }
            if (excluded[static_cast<size_t>(pointIndex)] != 0) {
                continue;
            }
            const Eigen::Vector2f uv = projectPointToPlane2d(points[static_cast<size_t>(pointIndex)],
                                                             cropStats.origin,
                                                             cropStats.axisX,
                                                             cropStats.axisY);
            if (uv.x() < outerMinU || uv.x() > outerMaxU || uv.y() < outerMinV || uv.y() > outerMaxV) {
                continue;
            }
            const bool insideInner = uv.x() >= innerMinU && uv.x() <= innerMaxU
                && uv.y() >= innerMinV && uv.y() <= innerMaxV;
            if (insideInner) {
                continue;
            }
            referenceRingDistances.push_back(referencePlane.normal.dot(points[static_cast<size_t>(pointIndex)]) + referencePlane.d);
        }
        if (referenceRingDistances.size() < 12) {
            return 0.0;
        }

        const double ringMedian = robustMedian(referenceRingDistances);
        const double ringMad = robustMad(referenceRingDistances, ringMedian);
        const double ringScale = std::max({ringMad * 1.4826, referencePlane.rmse * 1.5, 0.003});
        std::vector<float> filteredReferenceRing;
        filteredReferenceRing.reserve(referenceRingDistances.size());
        for (float distance : referenceRingDistances) {
            if (std::abs(static_cast<double>(distance) - ringMedian) <= ringScale * 1.8) {
                filteredReferenceRing.push_back(distance);
            }
        }
        if (filteredReferenceRing.size() < 8) {
            filteredReferenceRing = std::move(referenceRingDistances);
        }
        const double referenceLevel = robustMean(filteredReferenceRing, 0.18);

        std::vector<float> pointDepths;
        pointDepths.reserve(defectPointIndices.size());
        for (int pointIndex : defectPointIndices) {
            if (pointIndex < 0 || pointIndex >= static_cast<int>(points.size())) {
                continue;
            }
            const double signedDistance = static_cast<double>(referencePlane.normal.dot(points[static_cast<size_t>(pointIndex)]) + referencePlane.d);
            const double depth = negativeSide ? (referenceLevel - signedDistance) : (signedDistance - referenceLevel);
            pointDepths.push_back(static_cast<float>(std::max(depth, 0.0)));
        }
        if (pointDepths.size() < 8) {
            return 0.0;
        }

        const bool useUAxis = defectSpan.x() >= defectSpan.y();
        const float axisMin = useUAxis ? defectMinUv.x() : defectMinUv.y();
        const float axisMax = useUAxis ? defectMaxUv.x() : defectMaxUv.y();
        const float axisSpan = std::max(axisMax - axisMin, 1e-4f);
        const int binCount = std::clamp(static_cast<int>(std::lround(axisSpan / std::max(0.08f, axisSpan / 10.0f))), 6, 18);
        std::vector<std::vector<float>> bins(static_cast<size_t>(binCount));
        std::vector<float> coreDepths;
        coreDepths.reserve(pointDepths.size());
        const float coreMinU = defectMinUv.x() + defectSpan.x() * 0.18f;
        const float coreMaxU = defectMaxUv.x() - defectSpan.x() * 0.18f;
        const float coreMinV = defectMinUv.y() + defectSpan.y() * 0.18f;
        const float coreMaxV = defectMaxUv.y() - defectSpan.y() * 0.18f;

        for (size_t i = 0; i < defectUvs.size() && i < pointDepths.size(); ++i) {
            const Eigen::Vector2f& uv = defectUvs[i];
            const float axisValue = useUAxis ? uv.x() : uv.y();
            const int binIndex = std::clamp(static_cast<int>(std::floor((axisValue - axisMin) / axisSpan * binCount)),
                                            0,
                                            binCount - 1);
            bins[static_cast<size_t>(binIndex)].push_back(pointDepths[i]);
            const bool inCore = uv.x() >= coreMinU && uv.x() <= coreMaxU
                && uv.y() >= coreMinV && uv.y() <= coreMaxV;
            if (inCore) {
                coreDepths.push_back(pointDepths[i]);
            }
        }

        std::vector<float> profileDepths;
        profileDepths.reserve(static_cast<size_t>(binCount));
        for (auto& bin : bins) {
            if (bin.size() < 3) {
                continue;
            }
            std::sort(bin.begin(), bin.end());
            profileDepths.push_back(static_cast<float>(sortedPercentile(bin, 0.82)));
        }
        if (profileDepths.empty()) {
            return 0.0;
        }

        std::sort(profileDepths.begin(), profileDepths.end(), std::greater<float>());
        const size_t keepProfiles = std::max<size_t>(2, profileDepths.size() / 3);
        std::vector<float> strongestProfiles(profileDepths.begin(), profileDepths.begin() + keepProfiles);
        const double profileDepth = robustMean(strongestProfiles, 0.10);

        double coreDepth = 0.0;
        if (!coreDepths.empty()) {
            std::sort(coreDepths.begin(), coreDepths.end());
            coreDepth = sortedPercentile(coreDepths, 0.72);
        }

        const double bandDepth = estimatePlaneDefectDepthFromBands(filteredReferenceRing,
                                                                   false,
                                                                   std::max(referencePlane.rmse, 0.003));
        Q_UNUSED(bandDepth);
        const double deepProfile = positivePercentile(pointDepths, 0.84);
        const double conservativeDepth = std::max(profileDepth, deepProfile * 0.96);
        return std::max({conservativeDepth * 0.72 + coreDepth * 0.28,
                         conservativeDepth,
                         coreDepth,
                         0.0});
    }

    static PlaneFitStats fitPlaneWeightedLeastSquaresStats(const std::vector<Eigen::Vector3f>& points,
                                                           const std::vector<double>& weights,
                                                           const QString& name,
                                                           const Eigen::Vector3f* referenceNormal = nullptr) {
        PlaneFitStats stats;
        stats.name = name;
        if (points.size() < 3 || points.size() != weights.size()) {
            return stats;
        }

        double weightSum = 0.0;
        Eigen::Vector3d weightedCentroid = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < points.size(); ++i) {
            const double weight = std::max(weights[i], 0.0);
            if (weight <= 1e-9) {
                continue;
            }
            weightedCentroid += points[i].cast<double>() * weight;
            weightSum += weight;
        }
        if (weightSum <= 1e-9) {
            return stats;
        }
        weightedCentroid /= weightSum;

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        int effectiveCount = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            const double weight = std::max(weights[i], 0.0);
            if (weight <= 1e-9) {
                continue;
            }
            const Eigen::Vector3d delta = points[i].cast<double>() - weightedCentroid;
            covariance += weight * (delta * delta.transpose());
            ++effectiveCount;
        }
        if (effectiveCount < 3) {
            return stats;
        }
        covariance /= weightSum;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) {
            return stats;
        }

        Eigen::Vector3d normal64 = solver.eigenvectors().col(0);
        if (normal64.norm() < 1e-9) {
            return stats;
        }
        normal64.normalize();
        Eigen::Vector3f normal = normal64.cast<float>();
        if (referenceNormal && referenceNormal->norm() >= 1e-6f && normal.dot(*referenceNormal) < 0.0f) {
            normal = -normal;
        }

        stats.normal = normal;
        stats.centroid = weightedCentroid.cast<float>();
        stats.d = -normal.dot(stats.centroid);
        stats.inlierCount = effectiveCount;

        double sum = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        double minDeviation = std::numeric_limits<double>::max();
        double maxDeviation = std::numeric_limits<double>::lowest();
        for (const auto& point : points) {
            const double distance = static_cast<double>(normal.dot(point) + stats.d);
            const double absDistance = std::abs(distance);
            sum += distance;
            sumSq += distance * distance;
            maxAbs = std::max(maxAbs, absDistance);
            minDeviation = std::min(minDeviation, distance);
            maxDeviation = std::max(maxDeviation, distance);
        }

        stats.meanError = sum / static_cast<double>(points.size());
        stats.rmse = std::sqrt(sumSq / static_cast<double>(points.size()));
        stats.maxAbsError = maxAbs;
        stats.minDeviation = minDeviation;
        stats.maxDeviation = maxDeviation;
        stats.flatness = maxDeviation - minDeviation;
        stats.ok = true;
        return stats;
    }

    static PlaneFitStats fitPlaneWeightedLeastSquaresStatsRobust(const std::vector<Eigen::Vector3f>& points,
                                                                 const std::vector<double>& weights,
                                                                 const QString& name,
                                                                 const Eigen::Vector3f* referenceNormal = nullptr) {
        if (points.size() < 12 || points.size() != weights.size()) {
            return fitPlaneWeightedLeastSquaresStats(points, weights, name, referenceNormal);
        }

        std::vector<double> currentWeights = weights;
        PlaneFitStats bestPlane = fitPlaneWeightedLeastSquaresStats(points, currentWeights, name, referenceNormal);
        if (!bestPlane.ok) {
            return bestPlane;
        }

        for (int iteration = 0; iteration < 4; ++iteration) {
            std::vector<double> residuals;
            residuals.reserve(points.size());
            for (const auto& point : points) {
                residuals.push_back(std::abs(static_cast<double>(bestPlane.normal.dot(point) + bestPlane.d)));
            }

            const double residualScale = std::max({robustMadFromResiduals(residuals) * 1.4826,
                                                   bestPlane.rmse,
                                                   0.003});
            const double huberBand = std::max(residualScale * 2.2, 0.006);

            std::vector<double> refinedWeights(weights.size(), 0.0);
            bool hasEffectiveWeights = false;
            for (size_t i = 0; i < weights.size(); ++i) {
                const double baseWeight = std::max(weights[i], 0.0);
                if (baseWeight <= 1e-9) {
                    continue;
                }
                const double residual = residuals[i];
                const double robustFactor = residual <= huberBand
                    ? 1.0
                    : std::clamp(huberBand / std::max(residual, 1e-9), 0.08, 1.0);
                refinedWeights[i] = baseWeight * robustFactor * robustFactor;
                hasEffectiveWeights = hasEffectiveWeights || refinedWeights[i] > 1e-9;
            }
            if (!hasEffectiveWeights) {
                break;
            }

            PlaneFitStats refinedPlane = fitPlaneWeightedLeastSquaresStats(points, refinedWeights, name, referenceNormal);
            if (!refinedPlane.ok) {
                break;
            }
            bestPlane = refinedPlane;
            currentWeights = std::move(refinedWeights);
        }

        return bestPlane;
    }

    static PlaneFitStats fitPlaneLeastSquaresStats(const std::vector<Eigen::Vector3f>& points,
                                                   const QString& name,
                                                   const Eigen::Vector3f* referenceNormal = nullptr) {
        PlaneFitStats stats;
        stats.name = name;
        if (points.size() < 3) {
            return stats;
        }

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& point : points) {
            centroid += point.cast<double>();
        }
        centroid /= static_cast<double>(points.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto& point : points) {
            const Eigen::Vector3d delta = point.cast<double>() - centroid;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(points.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) {
            return stats;
        }

        Eigen::Vector3d normal64 = solver.eigenvectors().col(0);
        if (normal64.norm() < 1e-9) {
            return stats;
        }
        normal64.normalize();
        Eigen::Vector3f normal = normal64.cast<float>();
        if (referenceNormal && referenceNormal->norm() >= 1e-6f && normal.dot(*referenceNormal) < 0.0f) {
            normal = -normal;
        }

        stats.normal = normal;
        stats.d = -normal.dot(centroid.cast<float>());
        stats.centroid = centroid.cast<float>();
        stats.inlierCount = static_cast<int>(points.size());

        double sum = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        double minDeviation = std::numeric_limits<double>::max();
        double maxDeviation = std::numeric_limits<double>::lowest();
        for (const auto& point : points) {
            const double distance = static_cast<double>(normal.dot(point) + stats.d);
            const double absDistance = std::abs(distance);
            sum += distance;
            sumSq += distance * distance;
            maxAbs = std::max(maxAbs, absDistance);
            minDeviation = std::min(minDeviation, distance);
            maxDeviation = std::max(maxDeviation, distance);
        }

        stats.meanError = sum / static_cast<double>(points.size());
        stats.rmse = std::sqrt(sumSq / static_cast<double>(points.size()));
        stats.maxAbsError = maxAbs;
        stats.minDeviation = minDeviation;
        stats.maxDeviation = maxDeviation;
        stats.flatness = maxDeviation - minDeviation;
        stats.ok = true;
        return stats;
    }

    static PlaneFitStats fitPlaneLeastSquaresStatsRobust(const std::vector<Eigen::Vector3f>& points,
                                                         const QString& name,
                                                         const Eigen::Vector3f* referenceNormal = nullptr) {
        PlaneFitStats bestPlane = fitPlaneLeastSquaresStats(points, name, referenceNormal);
        if (!bestPlane.ok || points.size() < 12) {
            return bestPlane;
        }

        std::vector<Eigen::Vector3f> currentPoints = points;
        for (int iteration = 0; iteration < 4; ++iteration) {
            std::vector<float> residuals;
            residuals.reserve(currentPoints.size());
            for (const auto& point : currentPoints) {
                residuals.push_back(std::abs(bestPlane.normal.dot(point) + bestPlane.d));
            }

            const double residualMedian = robustMedian(residuals);
            const double residualMad = robustMad(residuals, residualMedian);
            const double residualBand = std::max({bestPlane.rmse * 2.5, residualMedian + residualMad * 2.8, 0.006});
            std::vector<Eigen::Vector3f> inliers;
            inliers.reserve(currentPoints.size());
            for (size_t i = 0; i < currentPoints.size(); ++i) {
                if (static_cast<double>(residuals[i]) <= residualBand) {
                    inliers.push_back(currentPoints[i]);
                }
            }

            if (inliers.size() < 12) {
                break;
            }

            PlaneFitStats refinedPlane = fitPlaneLeastSquaresStats(inliers, name, referenceNormal);
            if (!refinedPlane.ok) {
                break;
            }

            bestPlane = refinedPlane;
            if (inliers.size() == currentPoints.size()) {
                break;
            }
            currentPoints = std::move(inliers);
        }

        return bestPlane;
    }

    static double robustDefectDepthTrimmed(std::vector<float> signedDistances,
                                           bool negativeSide,
                                           double fallbackScale) {
        if (signedDistances.empty()) {
            return 0.0;
        }
        std::sort(signedDistances.begin(), signedDistances.end());
        const double medianDistance = robustMedian(signedDistances);
        const double mad = robustMad(signedDistances, medianDistance);
        const double robustBand = std::max({mad * 2.5, fallbackScale, 0.005});

        std::vector<float> selected;
        selected.reserve(signedDistances.size());
        for (float value : signedDistances) {
            const bool onDefectSide = negativeSide
                ? static_cast<double>(value) <= medianDistance - robustBand * 0.35
                : static_cast<double>(value) >= medianDistance + robustBand * 0.35;
            if (onDefectSide) {
                selected.push_back(value);
            }
        }
        if (selected.size() < 6) {
            selected = std::move(signedDistances);
        }

        std::sort(selected.begin(), selected.end());
        const double deep = std::abs(sortedPercentile(selected, negativeSide ? 0.14 : 0.86));
        const double shoulder = std::abs(sortedPercentile(selected, negativeSide ? 0.28 : 0.72));
        const double bottom = std::abs(sortedPercentile(selected, negativeSide ? 0.08 : 0.92));
        return std::max({deep * 0.55 + shoulder * 0.35 + bottom * 0.10, 0.0});
    }

    static double trimmedDefectSideDepth(std::vector<float> signedDistances,
                                         bool negativeSide,
                                         double trimFraction = 0.18) {
        if (signedDistances.size() < 4) {
            return 0.0;
        }

        std::sort(signedDistances.begin(), signedDistances.end());
        const size_t count = signedDistances.size();
        const size_t keepCount = std::max<size_t>(3,
                                                  static_cast<size_t>(std::ceil(static_cast<double>(count) * 0.45)));
        std::vector<float> selected;
        selected.reserve(keepCount);
        if (negativeSide) {
            selected.insert(selected.end(), signedDistances.begin(), signedDistances.begin() + keepCount);
        } else {
            selected.insert(selected.end(), signedDistances.end() - keepCount, signedDistances.end());
        }

        const double level = robustMean(std::move(selected), trimFraction);
        return std::max(negativeSide ? -level : level, 0.0);
    }

    static double fuseDepthEstimates(double primary,
                                     double secondary,
                                     double tertiary,
                                     double agreementTolerance,
                                     bool preferPrimary) {
        std::vector<double> validValues;
        validValues.reserve(3);
        if (primary > 0.0) {
            validValues.push_back(primary);
        }
        if (secondary > 0.0) {
            validValues.push_back(secondary);
        }
        if (tertiary > 0.0) {
            validValues.push_back(tertiary);
        }
        if (validValues.empty()) {
            return 0.0;
        }
        if (validValues.size() == 1) {
            return validValues.front();
        }

        if (primary > 0.0) {
            std::vector<double> closeToPrimary;
            closeToPrimary.reserve(3);
            closeToPrimary.push_back(primary);
            if (secondary > 0.0 && std::abs(secondary - primary) <= agreementTolerance) {
                closeToPrimary.push_back(secondary);
            }
            if (tertiary > 0.0 && std::abs(tertiary - primary) <= agreementTolerance) {
                closeToPrimary.push_back(tertiary);
            }
            if (closeToPrimary.size() >= 2) {
                const double sum = std::accumulate(closeToPrimary.begin(), closeToPrimary.end(), 0.0);
                return sum / static_cast<double>(closeToPrimary.size());
            }
            if (preferPrimary) {
                return primary;
            }
        }

        std::sort(validValues.begin(), validValues.end());
        const size_t mid = validValues.size() / 2;
        if ((validValues.size() % 2) == 0) {
            return (validValues[mid - 1] + validValues[mid]) * 0.5;
        }
        return validValues[mid];
    }

    static double conservativeDepthConsensus(std::vector<double> values,
                                             double spreadTolerance) {
        values.erase(std::remove_if(values.begin(), values.end(), [](double value) {
                         return !std::isfinite(value) || value <= 0.0;
                     }),
                     values.end());
        if (values.empty()) {
            return 0.0;
        }
        if (values.size() == 1) {
            return values.front();
        }

        std::sort(values.begin(), values.end());
        const double spread = values.back() - values.front();
        const size_t mid = values.size() / 2;
        if (spread > spreadTolerance) {
            return values.size() >= 3
                ? values[std::max<size_t>(0, mid - 1)]
                : values.front();
        }
        if ((values.size() % 2) == 0) {
            return (values[mid - 1] + values[mid]) * 0.5;
        }
        return values[mid];
    }

    static double biasReducedDepthConsensus(std::vector<double> values,
                                            double agreementTolerance,
                                            double preferredLowValue = 0.0) {
        values.erase(std::remove_if(values.begin(), values.end(), [](double value) {
                         return !std::isfinite(value) || value <= 0.0;
                     }),
                     values.end());
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        if (values.size() == 1) {
            return values.front();
        }

        if (preferredLowValue > 0.0) {
            std::vector<double> closeToPreferred;
            for (double value : values) {
                if (std::abs(value - preferredLowValue) <= agreementTolerance) {
                    closeToPreferred.push_back(value);
                }
            }
            if (!closeToPreferred.empty()) {
                closeToPreferred.push_back(preferredLowValue);
                std::sort(closeToPreferred.begin(), closeToPreferred.end());
                const size_t end = std::min<size_t>(closeToPreferred.size(), 3);
                double sum = 0.0;
                for (size_t i = 0; i < end; ++i) {
                    sum += closeToPreferred[i];
                }
                return sum / static_cast<double>(end);
            }
        }

        double bestPairMean = 0.0;
        double bestPairDiff = std::numeric_limits<double>::max();
        for (size_t i = 1; i < values.size(); ++i) {
            const double diff = std::abs(values[i] - values[i - 1]);
            if (diff < bestPairDiff) {
                bestPairDiff = diff;
                bestPairMean = (values[i] + values[i - 1]) * 0.5;
            }
        }
        if (bestPairDiff <= agreementTolerance) {
            return bestPairMean;
        }

        return values.size() >= 3 ? values[1] : values.front();
    }

    static double estimatePlaneDefectDepthFromCorePatch(const std::vector<Eigen::Vector2f>& defectUvs,
                                                        const std::vector<float>& localSignedDistances,
                                                        const Eigen::Vector2f& defectMinUv,
                                                        const Eigen::Vector2f& defectMaxUv,
                                                        bool negativeSide,
                                                        double fallbackScale) {
        if (defectUvs.size() != localSignedDistances.size() || defectUvs.size() < 8) {
            return 0.0;
        }

        const Eigen::Vector2f span = defectMaxUv - defectMinUv;
        const float coreMinU = defectMinUv.x() + span.x() * 0.18f;
        const float coreMaxU = defectMaxUv.x() - span.x() * 0.18f;
        const float coreMinV = defectMinUv.y() + span.y() * 0.18f;
        const float coreMaxV = defectMaxUv.y() - span.y() * 0.18f;

        std::vector<float> coreDepths;
        coreDepths.reserve(localSignedDistances.size());
        for (size_t i = 0; i < defectUvs.size(); ++i) {
            const Eigen::Vector2f& uv = defectUvs[i];
            if (uv.x() < coreMinU || uv.x() > coreMaxU || uv.y() < coreMinV || uv.y() > coreMaxV) {
                continue;
            }
            const double depth = negativeSide
                ? -static_cast<double>(localSignedDistances[i])
                : static_cast<double>(localSignedDistances[i]);
            if (depth > 0.0) {
                coreDepths.push_back(static_cast<float>(depth));
            }
        }

        if (coreDepths.size() < 6) {
            for (float distance : localSignedDistances) {
                const double depth = negativeSide ? -static_cast<double>(distance) : static_cast<double>(distance);
                if (depth > 0.0) {
                    coreDepths.push_back(static_cast<float>(depth));
                }
            }
        }
        if (coreDepths.size() < 6) {
            return 0.0;
        }

        std::sort(coreDepths.begin(), coreDepths.end());
        const double coreMedian = robustMedian(coreDepths);
        const double coreMad = robustMad(coreDepths, coreMedian);
        const double clipBand = std::max({coreMad * 2.0, fallbackScale, 0.005});
        std::vector<float> filteredCoreDepths;
        filteredCoreDepths.reserve(coreDepths.size());
        for (float depth : coreDepths) {
            if (std::abs(static_cast<double>(depth) - coreMedian) <= clipBand) {
                filteredCoreDepths.push_back(depth);
            }
        }
        if (filteredCoreDepths.size() < 4) {
            filteredCoreDepths = std::move(coreDepths);
        }

        std::sort(filteredCoreDepths.begin(), filteredCoreDepths.end());
        const double q35 = sortedPercentile(filteredCoreDepths, 0.35);
        const double q50 = sortedPercentile(filteredCoreDepths, 0.50);
        const double q65 = sortedPercentile(filteredCoreDepths, 0.65);
        return std::max((q35 * 0.25 + q50 * 0.50 + q65 * 0.25), 0.0);
    }

    static PlaneFitStats fitPlaneFromBottomRegion(const std::vector<Eigen::Vector3f>& componentPoints,
                                                  const std::vector<float>& baseDistances,
                                                  const PlaneFitStats& baselinePlane,
                                                  bool defectOnNegativeSide,
                                                  const QString& name,
                                                  double* depth) {
        PlaneFitStats bottomPlane;
        if (depth) {
            *depth = 0.0;
        }
        if (componentPoints.size() != baseDistances.size() || componentPoints.size() < 20 || !baselinePlane.ok) {
            return bottomPlane;
        }

        std::vector<float> sortedDistances = baseDistances;
        std::sort(sortedDistances.begin(), sortedDistances.end());
        const float medianDistance = static_cast<float>(robustMedian(sortedDistances));
        const float deepQuantile = static_cast<float>(sortedPercentile(sortedDistances, defectOnNegativeSide ? 0.12 : 0.88));
        const float shoulderQuantile = static_cast<float>(sortedPercentile(sortedDistances, defectOnNegativeSide ? 0.22 : 0.78));
        const float margin = static_cast<float>(std::max(baselinePlane.rmse * 2.5, 0.01));

        std::vector<Eigen::Vector3f> bottomPoints;
        std::vector<float> bottomDistances;
        bottomPoints.reserve(componentPoints.size());
        bottomDistances.reserve(componentPoints.size());
        for (size_t i = 0; i < componentPoints.size(); ++i) {
            const float distance = baseDistances[i];
            const bool inBottomBand = defectOnNegativeSide
                ? (distance <= deepQuantile + margin && distance <= shoulderQuantile)
                : (distance >= deepQuantile - margin && distance >= shoulderQuantile);
            const bool sufficientlySeparated = defectOnNegativeSide
                ? (distance <= medianDistance - margin)
                : (distance >= medianDistance + margin);
            if (!inBottomBand || !sufficientlySeparated) {
                continue;
            }
            bottomPoints.push_back(componentPoints[i]);
            bottomDistances.push_back(distance);
        }

        if (bottomPoints.size() < 20) {
            for (size_t i = 0; i < componentPoints.size(); ++i) {
                const float distance = baseDistances[i];
                const bool keep = defectOnNegativeSide
                    ? (distance <= shoulderQuantile)
                    : (distance >= shoulderQuantile);
                if (!keep) {
                    continue;
                }
                bottomPoints.push_back(componentPoints[i]);
                bottomDistances.push_back(distance);
            }
        }

        if (bottomPoints.size() < 20) {
            if (depth) {
                *depth = robustDefectDepth(baseDistances, defectOnNegativeSide);
            }
            return bottomPlane;
        }

        std::vector<double> bottomWeights;
        bottomWeights.reserve(bottomDistances.size());
        std::vector<float> sortedBottomDistances = bottomDistances;
        std::sort(sortedBottomDistances.begin(), sortedBottomDistances.end());
        const double deepestLevel = sortedPercentile(sortedBottomDistances, defectOnNegativeSide ? 0.10 : 0.90);
        const double shoulderLevel = sortedPercentile(sortedBottomDistances, defectOnNegativeSide ? 0.28 : 0.72);
        const double depthScale = std::max({std::abs(deepestLevel - shoulderLevel),
                                            baselinePlane.rmse * 1.8,
                                            0.01});
        for (float distance : bottomDistances) {
            const double separation = defectOnNegativeSide
                ? (shoulderLevel - static_cast<double>(distance))
                : (static_cast<double>(distance) - shoulderLevel);
            const double normalized = std::clamp(separation / depthScale, 0.0, 1.0);
            const double weight = 0.20 + 0.80 * normalized * normalized;
            bottomWeights.push_back(weight);
        }

        bottomPlane = fitPlaneWeightedLeastSquaresStatsRobust(bottomPoints,
                                                              bottomWeights,
                                                              name,
                                                              &baselinePlane.normal);
        if (!bottomPlane.ok) {
            bottomPlane = fitPlaneLeastSquaresStatsRobust(bottomPoints, name, &baselinePlane.normal);
        }

        if (depth) {
            const double planeDepth = bottomPlane.ok
                ? std::abs(static_cast<double>(baselinePlane.normal.dot(bottomPlane.centroid) + baselinePlane.d))
                : 0.0;
            const double trimmedDepth = trimmedDefectSideDepth(bottomDistances, defectOnNegativeSide, 0.12);
            const double bandDepth = robustDefectDepthTrimmed(baseDistances,
                                                              defectOnNegativeSide,
                                                              std::max(baselinePlane.rmse, 0.003));
            *depth = conservativeDepthConsensus({trimmedDepth, planeDepth, bandDepth},
                                                std::max(baselinePlane.rmse * 2.0, 0.02));
            if (*depth <= 0.0) {
                *depth = planeDepth > 0.0 ? planeDepth : std::abs(robustMedian(bottomDistances));
            }
        }
        return bottomPlane;
    }

    static float cross2d(const Eigen::Vector2f& origin,
                         const Eigen::Vector2f& a,
                         const Eigen::Vector2f& b) {
        const Eigen::Vector2f oa = a - origin;
        const Eigen::Vector2f ob = b - origin;
        return oa.x() * ob.y() - oa.y() * ob.x();
    }

    static std::vector<Eigen::Vector2f> convexHull2d(std::vector<Eigen::Vector2f> points) {
        if (points.size() <= 1) {
            return points;
        }
        std::sort(points.begin(), points.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
            if (!qFuzzyCompare(a.x(), b.x())) {
                return a.x() < b.x();
            }
            return a.y() < b.y();
        });
        points.erase(std::unique(points.begin(), points.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
            return std::abs(a.x() - b.x()) < 1e-5f && std::abs(a.y() - b.y()) < 1e-5f;
        }), points.end());
        if (points.size() <= 1) {
            return points;
        }

        std::vector<Eigen::Vector2f> hull;
        hull.reserve(points.size() * 2);
        for (const auto& point : points) {
            while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), point) <= 0.0f) {
                hull.pop_back();
            }
            hull.push_back(point);
        }
        const size_t lowerSize = hull.size();
        for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
            while (hull.size() > lowerSize && cross2d(hull[hull.size() - 2], hull.back(), *it) <= 0.0f) {
                hull.pop_back();
            }
            hull.push_back(*it);
        }
        if (!hull.empty()) {
            hull.pop_back();
        }
        return hull;
    }

    static double polygonArea2d(const std::vector<Eigen::Vector2f>& polygon) {
        if (polygon.size() < 3) {
            return 0.0;
        }
        double area = 0.0;
        for (size_t i = 0; i < polygon.size(); ++i) {
            const Eigen::Vector2f& a = polygon[i];
            const Eigen::Vector2f& b = polygon[(i + 1) % polygon.size()];
            area += static_cast<double>(a.x()) * b.y() - static_cast<double>(a.y()) * b.x();
        }
        return std::abs(area) * 0.5;
    }

    static double closedContourArea2d(const std::vector<Eigen::Vector2f>& points) {
        if (points.size() < 3) {
            return 0.0;
        }

        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (const auto& point : points) {
            minU = std::min(minU, point.x());
            maxU = std::max(maxU, point.x());
            minV = std::min(minV, point.y());
            maxV = std::max(maxV, point.y());
        }

        const double spanU = std::max<double>(maxU - minU, 1e-6);
        const double spanV = std::max<double>(maxV - minV, 1e-6);
        const double densityCell = std::sqrt((spanU * spanV) / std::max<double>(points.size(), 1.0)) * 0.9;
        const double spanCell = std::max(spanU, spanV) / 700.0;
        const double cellSize = std::max({densityCell, spanCell, 1e-6});
        const int cols = std::clamp(static_cast<int>(std::ceil(spanU / cellSize)) + 7, 8, 1200);
        const int rows = std::clamp(static_cast<int>(std::ceil(spanV / cellSize)) + 7, 8, 1200);
        const double pixelU = spanU / std::max(cols - 7, 1);
        const double pixelV = spanV / std::max(rows - 7, 1);
        const double pixelSize = std::max({pixelU, pixelV, 1e-6});

        cv::Mat mask(rows, cols, CV_8UC1, cv::Scalar(0));
        for (const auto& point : points) {
            const int x = 3 + static_cast<int>(std::lround((point.x() - minU) / pixelSize));
            const int y = 3 + static_cast<int>(std::lround((point.y() - minV) / pixelSize));
            if (x >= 0 && y >= 0 && x < cols && y < rows) {
                cv::circle(mask, cv::Point(x, y), 1, cv::Scalar(255), -1, cv::LINE_8);
            }
        }

        const int closeRadius = std::clamp(static_cast<int>(std::lround(densityCell / pixelSize * 1.8)), 2, 9);
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                         cv::Size(closeRadius * 2 + 1, closeRadius * 2 + 1));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);
        cv::erode(mask, mask, kernel, cv::Point(-1, -1), 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double maxAreaPixels = 0.0;
        for (const auto& contour : contours) {
            maxAreaPixels = std::max(maxAreaPixels, cv::contourArea(contour));
        }
        return maxAreaPixels * pixelSize * pixelSize;
    }

    struct MinimumRectangle2d {
        bool ok = false;
        double length = 0.0;
        double width = 0.0;
        std::array<Eigen::Vector2f, 4> corners = {
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero()
        };
    };

    static MinimumRectangle2d minimumAreaRectangle2d(const std::vector<Eigen::Vector2f>& hull) {
        MinimumRectangle2d result;
        if (hull.size() < 2) {
            return result;
        }
        double bestArea = std::numeric_limits<double>::max();
        Eigen::Vector2f bestU = Eigen::Vector2f::UnitX();
        Eigen::Vector2f bestV = Eigen::Vector2f::UnitY();
        double bestMinU = 0.0;
        double bestMaxU = 0.0;
        double bestMinV = 0.0;
        double bestMaxV = 0.0;
        for (size_t i = 0; i < hull.size(); ++i) {
            const Eigen::Vector2f edge = hull[(i + 1) % hull.size()] - hull[i];
            if (edge.norm() < 1e-6f) {
                continue;
            }
            const Eigen::Vector2f u = edge.normalized();
            const Eigen::Vector2f v(-u.y(), u.x());
            double minU = std::numeric_limits<double>::max();
            double maxU = std::numeric_limits<double>::lowest();
            double minV = std::numeric_limits<double>::max();
            double maxV = std::numeric_limits<double>::lowest();
            for (const auto& point : hull) {
                const double pu = point.dot(u);
                const double pv = point.dot(v);
                minU = std::min(minU, pu);
                maxU = std::max(maxU, pu);
                minV = std::min(minV, pv);
                maxV = std::max(maxV, pv);
            }
            const double sideA = maxU - minU;
            const double sideB = maxV - minV;
            const double area = sideA * sideB;
            if (area < bestArea) {
                bestArea = area;
                bestU = u;
                bestV = v;
                bestMinU = minU;
                bestMaxU = maxU;
                bestMinV = minV;
                bestMaxV = maxV;
            }
        }
        if (!std::isfinite(bestArea) || bestArea == std::numeric_limits<double>::max()) {
            return result;
        }

        const double sideU = bestMaxU - bestMinU;
        const double sideV = bestMaxV - bestMinV;
        result.length = std::max(sideU, sideV);
        result.width = std::min(sideU, sideV);
        result.corners = {
            bestU * static_cast<float>(bestMinU) + bestV * static_cast<float>(bestMinV),
            bestU * static_cast<float>(bestMaxU) + bestV * static_cast<float>(bestMinV),
            bestU * static_cast<float>(bestMaxU) + bestV * static_cast<float>(bestMaxV),
            bestU * static_cast<float>(bestMinU) + bestV * static_cast<float>(bestMaxV)
        };
        result.ok = true;
        return result;
    }

    static std::vector<std::vector<int>> connectedComponents2d(const std::vector<Eigen::Vector2f>& uvs,
                                                               double cellSize,
                                                               int minPointCount) {
        std::vector<std::vector<int>> components;
        if (uvs.empty() || cellSize <= 0.0) {
            return components;
        }

        struct Cell {
            int x = 0;
            int y = 0;
            bool operator==(const Cell& other) const {
                return x == other.x && y == other.y;
            }
        };
        struct CellHash {
            size_t operator()(const Cell& cell) const {
                const size_t hx = std::hash<int>{}(cell.x);
                const size_t hy = std::hash<int>{}(cell.y);
                return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
            }
        };

        std::unordered_map<Cell, std::vector<int>, CellHash> grid;
        grid.reserve(uvs.size() * 2);
        for (int i = 0; i < static_cast<int>(uvs.size()); ++i) {
            const Cell cell{
                static_cast<int>(std::floor(static_cast<double>(uvs[static_cast<size_t>(i)].x()) / cellSize)),
                static_cast<int>(std::floor(static_cast<double>(uvs[static_cast<size_t>(i)].y()) / cellSize))
            };
            grid[cell].push_back(i);
        }

        std::vector<char> visited(uvs.size(), 0);
        const double neighborDistance = cellSize * 1.75;
        const double neighborDistanceSq = neighborDistance * neighborDistance;
        for (int seed = 0; seed < static_cast<int>(uvs.size()); ++seed) {
            if (visited[static_cast<size_t>(seed)]) {
                continue;
            }
            std::vector<int> component;
            std::deque<int> queue;
            visited[static_cast<size_t>(seed)] = 1;
            queue.push_back(seed);
            while (!queue.empty()) {
                const int current = queue.front();
                queue.pop_front();
                component.push_back(current);
                const Eigen::Vector2f& uv = uvs[static_cast<size_t>(current)];
                const Cell center{
                    static_cast<int>(std::floor(static_cast<double>(uv.x()) / cellSize)),
                    static_cast<int>(std::floor(static_cast<double>(uv.y()) / cellSize))
                };
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const Cell neighborCell{center.x + dx, center.y + dy};
                        const auto found = grid.find(neighborCell);
                        if (found == grid.end()) {
                            continue;
                        }
                        for (int candidate : found->second) {
                            if (visited[static_cast<size_t>(candidate)]) {
                                continue;
                            }
                            const double distSq = (uvs[static_cast<size_t>(candidate)] - uv).squaredNorm();
                            if (distSq > neighborDistanceSq) {
                                continue;
                            }
                            visited[static_cast<size_t>(candidate)] = 1;
                            queue.push_back(candidate);
                        }
                    }
                }
            }
            if (static_cast<int>(component.size()) >= minPointCount) {
                components.push_back(std::move(component));
            }
        }
        return components;
    }

    static std::vector<std::vector<int>> connectedComponents2dPeriodic(const std::vector<Eigen::Vector2f>& uvs,
                                                                       double cellSize,
                                                                       int minPointCount,
                                                                       double wrapPeriodX) {
        if (uvs.empty()) {
            return {};
        }
        if (!(wrapPeriodX > 0.0)) {
            return connectedComponents2d(uvs, cellSize, minPointCount);
        }

        std::vector<Eigen::Vector2f> periodicUvs;
        std::vector<int> originalIndices;
        periodicUvs.reserve(uvs.size() * 3);
        originalIndices.reserve(uvs.size() * 3);
        for (int i = 0; i < static_cast<int>(uvs.size()); ++i) {
            const Eigen::Vector2f& uv = uvs[static_cast<size_t>(i)];
            periodicUvs.push_back(uv);
            originalIndices.push_back(i);
            periodicUvs.emplace_back(uv.x() - static_cast<float>(wrapPeriodX), uv.y());
            originalIndices.push_back(i);
            periodicUvs.emplace_back(uv.x() + static_cast<float>(wrapPeriodX), uv.y());
            originalIndices.push_back(i);
        }

        auto periodicComponents = connectedComponents2d(periodicUvs, cellSize, 1);
        std::vector<std::vector<int>> components;
        std::vector<char> dedupFlags(uvs.size(), 0);
        for (const auto& periodicComponent : periodicComponents) {
            std::vector<int> component;
            for (int periodicIndex : periodicComponent) {
                if (periodicIndex < 0 || periodicIndex >= static_cast<int>(originalIndices.size())) {
                    continue;
                }
                const int originalIndex = originalIndices[static_cast<size_t>(periodicIndex)];
                if (originalIndex < 0 || originalIndex >= static_cast<int>(uvs.size())) {
                    continue;
                }
                if (dedupFlags[static_cast<size_t>(originalIndex)]) {
                    continue;
                }
                dedupFlags[static_cast<size_t>(originalIndex)] = 1;
                component.push_back(originalIndex);
            }
            for (int originalIndex : component) {
                dedupFlags[static_cast<size_t>(originalIndex)] = 0;
            }
            if (static_cast<int>(component.size()) >= minPointCount) {
                components.push_back(std::move(component));
            }
        }
        return components;
    }

    static pointcloud::PointCloudPtr createPlaneDefectCropCloud(const std::vector<Eigen::Vector3f>& points,
                                                                const PlaneDefectCropStats& stats,
                                                                const QString& name) {
        if (stats.retainedIndices.empty()) {
            return nullptr;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(stats.retainedIndices.size());
        for (int index : stats.retainedIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            pcl::PointXYZ point;
            point.x = points[static_cast<size_t>(index)].x();
            point.y = points[static_cast<size_t>(index)].y();
            point.z = points[static_cast<size_t>(index)].z();
            cloud->push_back(point);
        }
        if (cloud->empty()) {
            return nullptr;
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
        return std::make_shared<pointcloud::PointCloudConcrete<pcl::PointXYZ>>(cloud, name);
    }

    static pointcloud::ColorRGB makeColorRgb(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
        pointcloud::ColorRGB color;
        color.components.r = r;
        color.components.g = g;
        color.components.b = b;
        color.components.a = a;
        return color;
    }

    static pointcloud::PointCloudPtr createPlaneDefectMeasurementCloud(const std::vector<Eigen::Vector3f>& points,
                                                                       const PlaneDefectMeasurement& defect,
                                                                       const QString& name,
                                                                       const pointcloud::ColorRGB& color) {
        if (defect.pointIndices.empty()) {
            return nullptr;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(defect.pointIndices.size());
        for (int index : defect.pointIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }
        if (cloud->empty()) {
            return nullptr;
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
        auto result = std::make_shared<pointcloud::PointCloudConcrete<pcl::PointXYZ>>(cloud, name);
        result->setColor(color);
        return result;
    }

    static std::array<pointcloud::ColorRGB, 7> planeDefectPalette() {
        return {
            makeColorRgb(230, 57, 70),
            makeColorRgb(29, 128, 185),
            makeColorRgb(42, 157, 143),
            makeColorRgb(245, 166, 35),
            makeColorRgb(132, 94, 194),
            makeColorRgb(0, 168, 150),
            makeColorRgb(255, 99, 146)
        };
    }

    static void removePlaneDefectRectangleOverlays(const pcl::visualization::PCLVisualizer::Ptr& viewer,
                                                   const QString& baseId) {
        if (!viewer || baseId.isEmpty()) {
            return;
        }
        for (int edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
            viewer->removeShape(QString("%1_plane_edge_%2")
                                    .arg(baseId)
                                    .arg(edgeIndex)
                                    .toStdString());
        }
        for (int gridIndex = 1; gridIndex <= 3; ++gridIndex) {
            viewer->removeShape(QString("%1_plane_u_%2")
                                    .arg(baseId)
                                    .arg(gridIndex)
                                    .toStdString());
            viewer->removeShape(QString("%1_plane_v_%2")
                                    .arg(baseId)
                                    .arg(gridIndex)
                                    .toStdString());
        }
        viewer->removeShape((baseId + "_plane_normal").toStdString());
        for (int defectIndex = 1; defectIndex <= 7; ++defectIndex) {
            for (int edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
                viewer->removeShape(QString("%1_S%2_rect_%3")
                                        .arg(baseId)
                                        .arg(defectIndex)
                                        .arg(edgeIndex)
                                        .toStdString());
            }
        }
    }

    static Eigen::Vector3f planeUvToPoint(const Eigen::Vector2f& uv,
                                          const Eigen::Vector3f& origin,
                                          const Eigen::Vector3f& axisX,
                                          const Eigen::Vector3f& axisY,
                                          const Eigen::Vector3f& normal,
                                          float offset) {
        return origin + axisX * uv.x() + axisY * uv.y() + normal * offset;
    }

    static void addRegisteredLine(pointcloud::CloudWindow* cloudWindow,
                                  const Eigen::Vector3f& p0,
                                  const Eigen::Vector3f& p1,
                                  double r,
                                  double g,
                                  double b,
                                  double lineWidth,
                                  const QString& id) {
        if (!cloudWindow || id.isEmpty()) {
            return;
        }
        const auto viewer = cloudWindow->getViewer();
        if (!viewer) {
            return;
        }
        viewer->removeShape(id.toStdString());
        viewer->addLine(pcl::PointXYZ(p0.x(), p0.y(), p0.z()),
                        pcl::PointXYZ(p1.x(), p1.y(), p1.z()),
                        r,
                        g,
                        b,
                        id.toStdString());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                             lineWidth,
                                             id.toStdString());
        cloudWindow->registerFitOverlayId(id);
    }

    static void showPlaneDefectBasePlaneOverlay(pointcloud::CloudWindow* cloudWindow,
                                                const PlaneDefectCropStats& stats,
                                                const QString& baseId) {
        if (!cloudWindow || !stats.ok || !stats.plane.ok || baseId.isEmpty()) {
            return;
        }
        const auto viewer = cloudWindow->getViewer();
        if (!viewer) {
            return;
        }

        const float spanU = std::max(stats.maxUv.x() - stats.minUv.x(), 1.0f);
        const float spanV = std::max(stats.maxUv.y() - stats.minUv.y(), 1.0f);
        const float marginU = std::max(spanU * 0.06f, 0.2f);
        const float marginV = std::max(spanV * 0.06f, 0.2f);
        const float minU = stats.minUv.x() - marginU;
        const float maxU = stats.maxUv.x() + marginU;
        const float minV = stats.minUv.y() - marginV;
        const float maxV = stats.maxUv.y() + marginV;
        const float offset = static_cast<float>(std::max(stats.plane.rmse * 2.0, 0.01));

        const std::array<Eigen::Vector2f, 4> corners = {
            Eigen::Vector2f(minU, minV),
            Eigen::Vector2f(maxU, minV),
            Eigen::Vector2f(maxU, maxV),
            Eigen::Vector2f(minU, maxV)
        };
        for (int edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
            const Eigen::Vector3f p0 = planeUvToPoint(corners[static_cast<size_t>(edgeIndex)],
                                                      stats.origin,
                                                      stats.axisX,
                                                      stats.axisY,
                                                      stats.plane.normal,
                                                      offset);
            const Eigen::Vector3f p1 = planeUvToPoint(corners[static_cast<size_t>((edgeIndex + 1) % 4)],
                                                      stats.origin,
                                                      stats.axisX,
                                                      stats.axisY,
                                                      stats.plane.normal,
                                                      offset);
            addRegisteredLine(cloudWindow,
                              p0,
                              p1,
                              0.95,
                              0.95,
                              0.25,
                              3.0,
                              QString("%1_plane_edge_%2").arg(baseId).arg(edgeIndex));
        }

        for (int gridIndex = 1; gridIndex <= 3; ++gridIndex) {
            const float t = static_cast<float>(gridIndex) / 4.0f;
            const float u = minU + (maxU - minU) * t;
            const float v = minV + (maxV - minV) * t;
            addRegisteredLine(cloudWindow,
                              planeUvToPoint(Eigen::Vector2f(u, minV), stats.origin, stats.axisX, stats.axisY, stats.plane.normal, offset),
                              planeUvToPoint(Eigen::Vector2f(u, maxV), stats.origin, stats.axisX, stats.axisY, stats.plane.normal, offset),
                              0.8,
                              0.8,
                              0.35,
                              1.0,
                              QString("%1_plane_u_%2").arg(baseId).arg(gridIndex));
            addRegisteredLine(cloudWindow,
                              planeUvToPoint(Eigen::Vector2f(minU, v), stats.origin, stats.axisX, stats.axisY, stats.plane.normal, offset),
                              planeUvToPoint(Eigen::Vector2f(maxU, v), stats.origin, stats.axisX, stats.axisY, stats.plane.normal, offset),
                              0.8,
                              0.8,
                              0.35,
                              1.0,
                              QString("%1_plane_v_%2").arg(baseId).arg(gridIndex));
        }

        const Eigen::Vector2f centerUv((minU + maxU) * 0.5f, (minV + maxV) * 0.5f);
        const Eigen::Vector3f center = planeUvToPoint(centerUv,
                                                     stats.origin,
                                                     stats.axisX,
                                                     stats.axisY,
                                                     stats.plane.normal,
                                                     offset);
        const float normalLength = std::max(std::max(spanU, spanV) * 0.12f, 0.5f);
        addRegisteredLine(cloudWindow,
                          center,
                          center + stats.plane.normal.normalized() * normalLength,
                          1.0,
                          0.2,
                          0.2,
                          3.0,
                          baseId + "_plane_normal");
    }

    static void showPlaneDefectRectangleOverlays(pointcloud::CloudWindow* cloudWindow,
                                                 const PlaneDefectAnalysisResult& analysis,
                                                 const QString& baseId) {
        if (!cloudWindow) {
            return;
        }
        const auto viewer = cloudWindow->getViewer();
        if (!viewer || baseId.isEmpty()) {
            return;
        }
        removePlaneDefectRectangleOverlays(viewer, baseId);
        const auto palette = planeDefectPalette();
        const float offset = static_cast<float>(std::max(analysis.plane.rmse * 4.0, 0.02));
        for (const auto& defect : analysis.defects) {
            if (!defect.hasRectangle) {
                continue;
            }
            const pointcloud::ColorRGB color = palette[static_cast<size_t>((defect.index - 1) % palette.size())];
            for (int edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
                const Eigen::Vector3f p0 = planeUvToPoint(defect.rectangleUv[static_cast<size_t>(edgeIndex)],
                                                          analysis.origin,
                                                          analysis.axisX,
                                                          analysis.axisY,
                                                          analysis.plane.normal,
                                                          offset);
                const Eigen::Vector3f p1 = planeUvToPoint(defect.rectangleUv[static_cast<size_t>((edgeIndex + 1) % 4)],
                                                          analysis.origin,
                                                          analysis.axisX,
                                                          analysis.axisY,
                                                          analysis.plane.normal,
                                                          offset);
                const std::string lineId = QString("%1_S%2_rect_%3")
                                               .arg(baseId)
                                               .arg(defect.index)
                                               .arg(edgeIndex)
                                               .toStdString();
                viewer->addLine(pcl::PointXYZ(p0.x(), p0.y(), p0.z()),
                                pcl::PointXYZ(p1.x(), p1.y(), p1.z()),
                                color.components.r / 255.0,
                                color.components.g / 255.0,
                                color.components.b / 255.0,
                                lineId);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                                     4.0,
                                                     lineId);
                cloudWindow->registerFitOverlayId(QString::fromStdString(lineId));
            }
        }
    }

    static void refinePlaneDefectBaseline(const std::vector<Eigen::Vector3f>& points,
                                          PlaneDefectCropStats& stats,
                                          const QString& name) {
        if (!stats.plane.ok || stats.retainedIndices.size() < 50) {
            return;
        }

        std::vector<float> distances;
        distances.reserve(stats.retainedIndices.size());
        for (int index : stats.retainedIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            distances.push_back(stats.plane.normal.dot(points[static_cast<size_t>(index)]) + stats.plane.d);
        }
        if (distances.size() < 50) {
            return;
        }

        std::sort(distances.begin(), distances.end());
        const float lowTail = static_cast<float>(sortedPercentile(distances, 0.18));
        const float highTail = static_cast<float>(sortedPercentile(distances, 0.82));
        const float centralLow = static_cast<float>(sortedPercentile(distances, 0.02));
        const float centralHigh = static_cast<float>(sortedPercentile(distances, 0.98));
        stats.defectOnNegativeSide = std::abs(lowTail) >= std::abs(highTail);

        std::vector<Eigen::Vector3f> basePoints;
        basePoints.reserve(stats.retainedIndices.size());
        for (int index : stats.retainedIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            const float distance = stats.plane.normal.dot(point) + stats.plane.d;
            const bool keep = stats.defectOnNegativeSide
                ? (distance >= lowTail && distance <= centralHigh)
                : (distance <= highTail && distance >= centralLow);
            if (keep) {
                basePoints.push_back(point);
            }
        }
        if (basePoints.size() < 50 || basePoints.size() < stats.retainedIndices.size() / 3) {
            return;
        }

        PlaneFitStats refinedPlane = fitPlaneLeastSquaresStats(basePoints, name + "_Base", &stats.plane.normal);
        if (!refinedPlane.ok) {
            return;
        }

        stats.plane = refinedPlane;
        stats.origin = stats.plane.centroid;
        const auto [axisX, axisY] = planeAxesFromNormal(stats.plane.normal);
        stats.axisX = axisX;
        stats.axisY = axisY;

        distances.clear();
        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (int index : stats.retainedIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            distances.push_back(stats.plane.normal.dot(point) + stats.plane.d);
            const Eigen::Vector2f uv = projectPointToPlane2d(point, stats.origin, stats.axisX, stats.axisY);
            minU = std::min(minU, uv.x());
            maxU = std::max(maxU, uv.x());
            minV = std::min(minV, uv.y());
            maxV = std::max(maxV, uv.y());
        }
        if (distances.empty()) {
            return;
        }

        std::sort(distances.begin(), distances.end());
        const float lowThreshold = static_cast<float>(sortedPercentile(distances, 0.06));
        const float highThreshold = static_cast<float>(sortedPercentile(distances, 0.94));
        stats.defectOnNegativeSide = std::abs(lowThreshold) >= std::abs(highThreshold);
        stats.seedThreshold = stats.defectOnNegativeSide ? std::abs(lowThreshold) : std::abs(highThreshold);
        stats.retainedPointCount = static_cast<int>(stats.retainedIndices.size());
        stats.minUv = Eigen::Vector2f(minU, minV);
        stats.maxUv = Eigen::Vector2f(maxU, maxV);
    }

    static PlaneDefectCropStats cropCenterPlaneDefect(const std::vector<Eigen::Vector3f>& points,
                                                      const QString& name = "PlaneDefect") {
        PlaneDefectCropStats stats;
        stats.plane = fitPlaneStats(points, name);
        if (!stats.plane.ok || points.size() < 50) {
            return stats;
        }

        stats.origin = stats.plane.centroid;
        const auto [axisX, axisY] = planeAxesFromNormal(stats.plane.normal);
        stats.axisX = axisX;
        stats.axisY = axisY;

        std::vector<float> signedDistances;
        signedDistances.reserve(points.size());
        std::vector<Eigen::Vector2f> uvs;
        uvs.reserve(points.size());
        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (const auto& point : points) {
            const float distance = stats.plane.normal.dot(point) + stats.plane.d;
            signedDistances.push_back(distance);
            const Eigen::Vector2f uv = projectPointToPlane2d(point, stats.origin, stats.axisX, stats.axisY);
            uvs.push_back(uv);
            minU = std::min(minU, uv.x());
            maxU = std::max(maxU, uv.x());
            minV = std::min(minV, uv.y());
            maxV = std::max(maxV, uv.y());
        }

        const float centerU = (minU + maxU) * 0.5f;
        const float centerV = (minV + maxV) * 0.5f;
        const float spanU = maxU - minU;
        const float spanV = maxV - minV;
        const float halfCentralU = spanU * 0.32f;
        const float halfCentralV = spanV * 0.32f;

        std::vector<float> centralDistances;
        centralDistances.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector2f& uv = uvs[i];
            if (std::abs(uv.x() - centerU) > halfCentralU || std::abs(uv.y() - centerV) > halfCentralV) {
                continue;
            }
            centralDistances.push_back(signedDistances[i]);
        }
        if (centralDistances.size() < 100) {
            return stats;
        }

        std::sort(centralDistances.begin(), centralDistances.end());
        const size_t medianIndex = centralDistances.size() / 2;
        const size_t lowIndex = static_cast<size_t>(std::clamp<int>(static_cast<int>(centralDistances.size() * 0.08), 0, static_cast<int>(centralDistances.size() - 1)));
        const size_t highIndex = static_cast<size_t>(std::clamp<int>(static_cast<int>(centralDistances.size() * 0.92), 0, static_cast<int>(centralDistances.size() - 1)));
        const float medianDistance = centralDistances[medianIndex];
        const float lowThreshold = centralDistances[lowIndex];
        const float highThreshold = centralDistances[highIndex];
        stats.defectOnNegativeSide = std::abs(lowThreshold) >= std::abs(highThreshold);
        stats.seedThreshold = stats.defectOnNegativeSide ? std::abs(lowThreshold) : std::abs(highThreshold);

        std::vector<int> seedIndices;
        seedIndices.reserve(points.size() / 8);
        float seedMinU = std::numeric_limits<float>::max();
        float seedMaxU = std::numeric_limits<float>::lowest();
        float seedMinV = std::numeric_limits<float>::max();
        float seedMaxV = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector2f& uv = uvs[i];
            if (std::abs(uv.x() - centerU) > halfCentralU || std::abs(uv.y() - centerV) > halfCentralV) {
                continue;
            }
            const float distance = signedDistances[i];
            const bool onDefectSide = stats.defectOnNegativeSide ? (distance <= lowThreshold) : (distance >= highThreshold);
            if (!onDefectSide) {
                continue;
            }
            seedIndices.push_back(static_cast<int>(i));
            seedMinU = std::min(seedMinU, uv.x());
            seedMaxU = std::max(seedMaxU, uv.x());
            seedMinV = std::min(seedMinV, uv.y());
            seedMaxV = std::max(seedMaxV, uv.y());
        }

        stats.seedPointCount = static_cast<int>(seedIndices.size());
        if (seedIndices.empty()) {
            return stats;
        }

        const float seedSpanU = std::max(seedMaxU - seedMinU, spanU * 0.02f);
        const float seedSpanV = std::max(seedMaxV - seedMinV, spanV * 0.02f);
        const float expandU = std::max(seedSpanU * 0.40f, spanU * 0.03f);
        const float expandV = std::max(seedSpanV * 0.40f, spanV * 0.03f);
        const float keepMinU = seedMinU - expandU;
        const float keepMaxU = seedMaxU + expandU;
        const float keepMinV = seedMinV - expandV;
        const float keepMaxV = seedMaxV + expandV;

        const float distanceBand = std::max(
            std::abs((stats.defectOnNegativeSide ? lowThreshold : highThreshold) - medianDistance) * 1.8f,
            static_cast<float>(std::max(stats.plane.rmse * 6.0, 0.05)));
        const float distanceMin = medianDistance - distanceBand;
        const float distanceMax = medianDistance + distanceBand;

        float retainedMinU = std::numeric_limits<float>::max();
        float retainedMaxU = std::numeric_limits<float>::lowest();
        float retainedMinV = std::numeric_limits<float>::max();
        float retainedMaxV = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector2f& uv = uvs[i];
            if (uv.x() < keepMinU || uv.x() > keepMaxU || uv.y() < keepMinV || uv.y() > keepMaxV) {
                continue;
            }

            const float distance = signedDistances[i];
            if (distance < distanceMin || distance > distanceMax) {
                continue;
            }

            stats.retainedIndices.push_back(static_cast<int>(i));
            retainedMinU = std::min(retainedMinU, uv.x());
            retainedMaxU = std::max(retainedMaxU, uv.x());
            retainedMinV = std::min(retainedMinV, uv.y());
            retainedMaxV = std::max(retainedMaxV, uv.y());
        }

        if (stats.retainedIndices.size() < seedIndices.size()) {
            stats.retainedIndices = seedIndices;
            retainedMinU = seedMinU;
            retainedMaxU = seedMaxU;
            retainedMinV = seedMinV;
            retainedMaxV = seedMaxV;
        }

        if (!stats.retainedIndices.empty()) {
            std::vector<Eigen::Vector2f> retainedUvs;
            retainedUvs.reserve(stats.retainedIndices.size());
            float componentMinU = std::numeric_limits<float>::max();
            float componentMaxU = std::numeric_limits<float>::lowest();
            float componentMinV = std::numeric_limits<float>::max();
            float componentMaxV = std::numeric_limits<float>::lowest();
            for (int retainedIndex : stats.retainedIndices) {
                const Eigen::Vector2f& uv = uvs[static_cast<size_t>(retainedIndex)];
                retainedUvs.push_back(uv);
                componentMinU = std::min(componentMinU, uv.x());
                componentMaxU = std::max(componentMaxU, uv.x());
                componentMinV = std::min(componentMinV, uv.y());
                componentMaxV = std::max(componentMaxV, uv.y());
            }
            const double componentSpanU = std::max<double>(componentMaxU - componentMinU, 1e-6);
            const double componentSpanV = std::max<double>(componentMaxV - componentMinV, 1e-6);
            const double densityCell = std::sqrt((componentSpanU * componentSpanV) / std::max<double>(retainedUvs.size(), 1.0)) * 2.6;
            const double spanCell = std::max(componentSpanU, componentSpanV) / 240.0;
            const double noiseCell = std::max(stats.plane.rmse * 4.0, 0.03);
            const double maxCellSize = std::max(0.03, std::max(componentSpanU, componentSpanV) / 16.0);
            const double cellSize = std::clamp(std::max({densityCell, spanCell, noiseCell}), 0.03, maxCellSize);
            const int minComponentPoints = std::max(30, static_cast<int>(retainedUvs.size() * 0.004));
            auto components = connectedComponents2d(retainedUvs, cellSize, minComponentPoints);
            if (!components.empty()) {
                const auto largestIt = std::max_element(components.begin(), components.end(), [](const auto& a, const auto& b) {
                    return a.size() < b.size();
                });
                if (largestIt != components.end() && !largestIt->empty()) {
                    std::vector<int> filteredIndices;
                    filteredIndices.reserve(largestIt->size());
                    retainedMinU = std::numeric_limits<float>::max();
                    retainedMaxU = std::numeric_limits<float>::lowest();
                    retainedMinV = std::numeric_limits<float>::max();
                    retainedMaxV = std::numeric_limits<float>::lowest();
                    for (int localIndex : *largestIt) {
                        const int originalIndex = stats.retainedIndices[static_cast<size_t>(localIndex)];
                        filteredIndices.push_back(originalIndex);
                        const Eigen::Vector2f& uv = uvs[static_cast<size_t>(originalIndex)];
                        retainedMinU = std::min(retainedMinU, uv.x());
                        retainedMaxU = std::max(retainedMaxU, uv.x());
                        retainedMinV = std::min(retainedMinV, uv.y());
                        retainedMaxV = std::max(retainedMaxV, uv.y());
                    }
                    stats.retainedIndices = std::move(filteredIndices);
                }
            }
        }

        stats.retainedPointCount = static_cast<int>(stats.retainedIndices.size());
        stats.minUv = Eigen::Vector2f(retainedMinU, retainedMinV);
        stats.maxUv = Eigen::Vector2f(retainedMaxU, retainedMaxV);
        refinePlaneDefectBaseline(points, stats, name);
        stats.ok = true;
        return stats;
    }

    static PlaneDefectCropStats makePlaneDefectCropFromRoi(const std::vector<Eigen::Vector3f>& points,
                                                           const QString& name = "PlaneDefectRoi") {
        PlaneDefectCropStats stats;
        stats.plane = fitPlaneStats(points, name);
        if (!stats.plane.ok || points.size() < 50) {
            return stats;
        }

        stats.origin = stats.plane.centroid;
        const auto [axisX, axisY] = planeAxesFromNormal(stats.plane.normal);
        stats.axisX = axisX;
        stats.axisY = axisY;
        stats.retainedIndices.reserve(points.size());

        std::vector<float> distances;
        distances.reserve(points.size());
        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < points.size(); ++i) {
            stats.retainedIndices.push_back(static_cast<int>(i));
            distances.push_back(stats.plane.normal.dot(points[i]) + stats.plane.d);
            const Eigen::Vector2f uv = projectPointToPlane2d(points[i], stats.origin, stats.axisX, stats.axisY);
            minU = std::min(minU, uv.x());
            maxU = std::max(maxU, uv.x());
            minV = std::min(minV, uv.y());
            maxV = std::max(maxV, uv.y());
        }
        std::sort(distances.begin(), distances.end());
        const size_t lowIndex = static_cast<size_t>(std::clamp<int>(static_cast<int>(distances.size() * 0.08), 0, static_cast<int>(distances.size() - 1)));
        const size_t highIndex = static_cast<size_t>(std::clamp<int>(static_cast<int>(distances.size() * 0.92), 0, static_cast<int>(distances.size() - 1)));
        stats.defectOnNegativeSide = std::abs(distances[lowIndex]) >= std::abs(distances[highIndex]);
        stats.seedThreshold = stats.defectOnNegativeSide ? std::abs(distances[lowIndex]) : std::abs(distances[highIndex]);
        stats.seedPointCount = static_cast<int>(points.size());
        stats.retainedPointCount = static_cast<int>(points.size());
        stats.minUv = Eigen::Vector2f(minU, minV);
        stats.maxUv = Eigen::Vector2f(maxU, maxV);
        refinePlaneDefectBaseline(points, stats, name);
        stats.ok = true;
        return stats;
    }

    static PlaneFitStats fitLocalReferencePlaneForDefect(const std::vector<Eigen::Vector3f>& points,
                                                         const PlaneDefectCropStats& cropStats,
                                                         const std::vector<int>& defectPointIndices,
                                                         const Eigen::Vector2f& defectMinUv,
                                                         const Eigen::Vector2f& defectMaxUv,
                                                         const QString& name) {
        if (!cropStats.plane.ok || cropStats.retainedIndices.size() < 50 || defectPointIndices.empty()) {
            return cropStats.plane;
        }

        std::vector<char> excluded(points.size(), 0);
        for (int pointIndex : defectPointIndices) {
            if (pointIndex >= 0 && pointIndex < static_cast<int>(points.size())) {
                excluded[static_cast<size_t>(pointIndex)] = 1;
            }
        }

        const Eigen::Vector2f globalSpan = cropStats.maxUv - cropStats.minUv;
        const Eigen::Vector2f defectSpan = defectMaxUv - defectMinUv;
        const float expandU = std::max(defectSpan.x() * 1.1f, std::max(globalSpan.x() * 0.05f, 0.05f));
        const float expandV = std::max(defectSpan.y() * 1.1f, std::max(globalSpan.y() * 0.05f, 0.05f));
        const float innerPadU = std::max(defectSpan.x() * 0.18f, std::max(globalSpan.x() * 0.01f, 0.02f));
        const float innerPadV = std::max(defectSpan.y() * 0.18f, std::max(globalSpan.y() * 0.01f, 0.02f));
        const float outerMinU = defectMinUv.x() - expandU;
        const float outerMaxU = defectMaxUv.x() + expandU;
        const float outerMinV = defectMinUv.y() - expandV;
        const float outerMaxV = defectMaxUv.y() + expandV;
        const float innerMinU = defectMinUv.x() - innerPadU;
        const float innerMaxU = defectMaxUv.x() + innerPadU;
        const float innerMinV = defectMinUv.y() - innerPadV;
        const float innerMaxV = defectMaxUv.y() + innerPadV;

        struct CandidatePoint {
            Eigen::Vector3f point = Eigen::Vector3f::Zero();
            float distance = 0.0f;
            bool inInner = false;
        };

        std::vector<CandidatePoint> candidates;
        candidates.reserve(cropStats.retainedIndices.size());
        std::vector<float> candidateDistances;
        candidateDistances.reserve(cropStats.retainedIndices.size());
        for (int pointIndex : cropStats.retainedIndices) {
            if (pointIndex < 0 || pointIndex >= static_cast<int>(points.size())) {
                continue;
            }
            if (excluded[static_cast<size_t>(pointIndex)] != 0) {
                continue;
            }

            const Eigen::Vector3f& point = points[static_cast<size_t>(pointIndex)];
            const Eigen::Vector2f uv = projectPointToPlane2d(point, cropStats.origin, cropStats.axisX, cropStats.axisY);
            if (uv.x() < outerMinU || uv.x() > outerMaxU || uv.y() < outerMinV || uv.y() > outerMaxV) {
                continue;
            }

            const float distance = cropStats.plane.normal.dot(point) + cropStats.plane.d;
            const bool inInner = uv.x() >= innerMinU && uv.x() <= innerMaxU
                && uv.y() >= innerMinV && uv.y() <= innerMaxV;
            candidates.push_back(CandidatePoint{point, distance, inInner});
            candidateDistances.push_back(distance);
        }

        if (candidates.size() < 40) {
            return cropStats.plane;
        }

        std::sort(candidateDistances.begin(), candidateDistances.end());
        const float medianDistance = static_cast<float>(robustMedian(candidateDistances));
        const float lowBand = static_cast<float>(sortedPercentile(candidateDistances, 0.18));
        const float highBand = static_cast<float>(sortedPercentile(candidateDistances, 0.82));
        const float centralBand = static_cast<float>(std::max(cropStats.plane.rmse * 2.5, 0.01));

        std::vector<Eigen::Vector3f> referencePoints;
        referencePoints.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            const bool onBaseSide = cropStats.defectOnNegativeSide
                ? (candidate.distance >= lowBand)
                : (candidate.distance <= highBand);
            const bool closeToLocalBase = std::abs(candidate.distance - medianDistance) <= centralBand;
            if (!onBaseSide) {
                continue;
            }
            if (candidate.inInner && !closeToLocalBase) {
                continue;
            }
            referencePoints.push_back(candidate.point);
        }

        if (referencePoints.size() < 30) {
            referencePoints.clear();
            referencePoints.reserve(candidates.size());
            for (const auto& candidate : candidates) {
                const bool onBaseSide = cropStats.defectOnNegativeSide
                    ? (candidate.distance >= medianDistance - centralBand)
                    : (candidate.distance <= medianDistance + centralBand);
                if (onBaseSide && !candidate.inInner) {
                    referencePoints.push_back(candidate.point);
                }
            }
        }

        if (referencePoints.size() < 20) {
            return cropStats.plane;
        }

        PlaneFitStats localPlane = fitPlaneLeastSquaresStatsRobust(referencePoints, name, &cropStats.plane.normal);
        return localPlane.ok ? localPlane : cropStats.plane;
    }

    static QString formatPlaneDefectCropSummary(const QString& cloudName, const PlaneDefectCropStats& stats) {
        QString info;
        info += cloudName + "\n";
        info += QObject::tr("基准平面法向量: (%1, %2, %3)\n")
            .arg(stats.plane.normal.x(), 0, 'f', 6)
            .arg(stats.plane.normal.y(), 0, 'f', 6)
            .arg(stats.plane.normal.z(), 0, 'f', 6);
        info += QObject::tr("平面方程: %1x + %2y + %3z + %4 = 0\n")
            .arg(stats.plane.normal.x(), 0, 'f', 6)
            .arg(stats.plane.normal.y(), 0, 'f', 6)
            .arg(stats.plane.normal.z(), 0, 'f', 6)
            .arg(stats.plane.d, 0, 'f', 6);
        info += QObject::tr("平面内点数: %1\n平面RMSE: %2\n平面度(最大-最小偏差): %3\n最小偏差: %4\n最大偏差: %5\n候选凹槽阈值: %6\n")
            .arg(stats.plane.inlierCount)
            .arg(stats.plane.rmse, 0, 'f', 4)
            .arg(stats.plane.flatness, 0, 'f', 4)
            .arg(stats.plane.minDeviation, 0, 'f', 4)
            .arg(stats.plane.maxDeviation, 0, 'f', 4)
            .arg(stats.seedThreshold, 0, 'f', 4);
        info += QObject::tr("凹槽方向: %1\n候选点数: %2\n保留点数: %3\n")
            .arg(stats.defectOnNegativeSide ? QObject::tr("沿法向负方向") : QObject::tr("沿法向正方向"))
            .arg(stats.seedPointCount)
            .arg(stats.retainedPointCount);
        info += QObject::tr("二维范围: U[%1, %2], V[%3, %4]")
            .arg(stats.minUv.x(), 0, 'f', 4)
            .arg(stats.maxUv.x(), 0, 'f', 4)
            .arg(stats.minUv.y(), 0, 'f', 4)
            .arg(stats.maxUv.y(), 0, 'f', 4);
        return info;
    }

    static PlaneDefectAnalysisResult analyzePlaneDefects(const std::vector<Eigen::Vector3f>& points,
                                                         const PlaneDefectCropStats& cropStats,
                                                         const QString& name = "PlaneDefectAnalysis") {
        PlaneDefectAnalysisResult result;
        if (cropStats.retainedIndices.size() < 100) {
            return result;
        }

        result.plane = cropStats.plane;
        if (!result.plane.ok) {
            return result;
        }
        result.origin = cropStats.origin;
        result.axisX = cropStats.axisX;
        result.axisY = cropStats.axisY;
        result.defectOnNegativeSide = cropStats.defectOnNegativeSide;

        std::vector<float> distances;
        distances.reserve(cropStats.retainedIndices.size());
        for (int index : cropStats.retainedIndices) {
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            distances.push_back(result.plane.normal.dot(point) + result.plane.d);
        }
        if (distances.empty()) {
            return result;
        }

        std::sort(distances.begin(), distances.end());
        const auto percentileIndex = [&distances](double fraction) {
            return static_cast<size_t>(std::clamp<int>(
                static_cast<int>(distances.size() * fraction),
                0,
                static_cast<int>(distances.size() - 1)));
        };
        const float medianDistance = distances[percentileIndex(0.50)];
        const float seedThresholdRaw = distances[percentileIndex(result.defectOnNegativeSide ? 0.22 : 0.78)];
        const float growThresholdRaw = distances[percentileIndex(result.defectOnNegativeSide ? 0.36 : 0.64)];
        const float noiseBand = static_cast<float>(std::max(result.plane.rmse * 1.2, 0.005));
        float seedThreshold = seedThresholdRaw;
        float growThreshold = growThresholdRaw;
        if (result.defectOnNegativeSide) {
            growThreshold = std::min(growThresholdRaw, medianDistance - noiseBand);
            seedThreshold = std::min(seedThresholdRaw, growThreshold);
        } else {
            growThreshold = std::max(growThresholdRaw, medianDistance + noiseBand);
            seedThreshold = std::max(seedThresholdRaw, growThreshold);
        }
        result.threshold = growThreshold;

        std::vector<int> defectIndices;
        std::vector<Eigen::Vector2f> defectUvs;
        std::vector<float> defectDistances;
        std::vector<char> defectSeedFlags;
        defectIndices.reserve(cropStats.retainedIndices.size());
        defectUvs.reserve(cropStats.retainedIndices.size());
        defectDistances.reserve(cropStats.retainedIndices.size());
        defectSeedFlags.reserve(cropStats.retainedIndices.size());

        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (int index : cropStats.retainedIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            const float distance = result.plane.normal.dot(point) + result.plane.d;
            const bool isSeedPoint = result.defectOnNegativeSide
                ? distance <= seedThreshold
                : distance >= seedThreshold;
            const bool isGrowPoint = result.defectOnNegativeSide
                ? distance <= growThreshold
                : distance >= growThreshold;
            if (!isGrowPoint) {
                continue;
            }
            const Eigen::Vector2f uv = projectPointToPlane2d(point, result.origin, result.axisX, result.axisY);
            defectIndices.push_back(index);
            defectUvs.push_back(uv);
            defectDistances.push_back(distance);
            defectSeedFlags.push_back(isSeedPoint ? 1 : 0);
            minU = std::min(minU, uv.x());
            maxU = std::max(maxU, uv.x());
            minV = std::min(minV, uv.y());
            maxV = std::max(maxV, uv.y());
        }
        if (defectIndices.size() < 10) {
            return result;
        }

        const double spanU = std::max<double>(maxU - minU, 1e-6);
        const double spanV = std::max<double>(maxV - minV, 1e-6);
        const double densityCell = std::sqrt((spanU * spanV) / std::max<double>(defectUvs.size(), 1.0)) * 2.3;
        const double spanCell = std::max(spanU, spanV) / 260.0;
        const double noiseCell = std::max(result.plane.rmse * 3.0, 0.02);
        const double maxCellSize = std::max(0.02, std::max(spanU, spanV) / 18.0);
        const double cellSize = std::clamp(std::max({densityCell, spanCell, noiseCell}), 0.02, maxCellSize);
        const int minComponentPoints = std::max(6, static_cast<int>(defectUvs.size() * 0.003));
        auto components = connectedComponents2d(defectUvs, cellSize, minComponentPoints);
        if (components.empty()) {
            return result;
        }

        std::vector<std::vector<int>> seededComponents;
        seededComponents.reserve(components.size());
        for (auto& component : components) {
            int seedCount = 0;
            for (int localIndex : component) {
                if (localIndex >= 0 && localIndex < static_cast<int>(defectSeedFlags.size())
                    && defectSeedFlags[static_cast<size_t>(localIndex)]) {
                    ++seedCount;
                }
            }
            const int minSeedPoints = std::max(2, static_cast<int>(component.size() * 0.03));
            if (seedCount >= minSeedPoints) {
                seededComponents.push_back(std::move(component));
            }
        }
        components = std::move(seededComponents);
        if (components.empty()) {
            return result;
        }

        std::sort(components.begin(), components.end(), [&defectUvs](const std::vector<int>& a, const std::vector<int>& b) {
            auto centroidOf = [&defectUvs](const std::vector<int>& component) {
                Eigen::Vector2d center = Eigen::Vector2d::Zero();
                for (int localIndex : component) {
                    const auto& uv = defectUvs[static_cast<size_t>(localIndex)];
                    center += Eigen::Vector2d(uv.x(), uv.y());
                }
                center /= std::max<double>(component.size(), 1.0);
                return center;
            };
            const Eigen::Vector2d ca = centroidOf(a);
            const Eigen::Vector2d cb = centroidOf(b);
            if (std::abs(ca.y() - cb.y()) > std::max(1.0, std::abs(ca.x() - cb.x())) * 0.05) {
                return ca.y() < cb.y();
            }
            return ca.x() < cb.x();
        });

        int defectIndex = 1;
        for (const auto& component : components) {
            PlaneDefectMeasurement measurement;
            measurement.index = defectIndex++;
            measurement.origin = result.origin;
            measurement.axisX = result.axisX;
            measurement.axisY = result.axisY;
            measurement.pointIndices.reserve(component.size());

            std::vector<Eigen::Vector3f> componentPoints;
            std::vector<Eigen::Vector2f> componentUvs;
            componentPoints.reserve(component.size());
            componentUvs.reserve(component.size());
            float minCompU = std::numeric_limits<float>::max();
            float maxCompU = std::numeric_limits<float>::lowest();
            float minCompV = std::numeric_limits<float>::max();
            float maxCompV = std::numeric_limits<float>::lowest();

            for (int localIndex : component) {
                const size_t local = static_cast<size_t>(localIndex);
                const int pointIndex = defectIndices[local];
                const Eigen::Vector3f& point = points[static_cast<size_t>(pointIndex)];
                const Eigen::Vector2f& uv = defectUvs[local];
                measurement.pointIndices.push_back(pointIndex);
                componentPoints.push_back(point);
                componentUvs.push_back(uv);
                minCompU = std::min(minCompU, uv.x());
                maxCompU = std::max(maxCompU, uv.x());
                minCompV = std::min(minCompV, uv.y());
                maxCompV = std::max(maxCompV, uv.y());
            }

            const PlaneFitStats localReferencePlane = fitLocalReferencePlaneForDefect(points,
                                                                                      cropStats,
                                                                                      measurement.pointIndices,
                                                                                      Eigen::Vector2f(minCompU, minCompV),
                                                                                      Eigen::Vector2f(maxCompU, maxCompV),
                                                                                      QString("%1_S%2_LocalBase").arg(name).arg(measurement.index));
            std::vector<float> componentLocalDistances;
            componentLocalDistances.reserve(componentPoints.size());
            for (const auto& point : componentPoints) {
                componentLocalDistances.push_back(localReferencePlane.normal.dot(point) + localReferencePlane.d);
            }
            const double minLocalDeviation = componentLocalDistances.empty()
                ? 0.0
                : static_cast<double>(*std::min_element(componentLocalDistances.begin(), componentLocalDistances.end()));
            const double maxLocalDeviation = componentLocalDistances.empty()
                ? 0.0
                : static_cast<double>(*std::max_element(componentLocalDistances.begin(), componentLocalDistances.end()));
            const double profileDepth = localReferencePlane.ok
                ? estimatePlaneDefectDepthFromLocalProfile(points,
                                                           cropStats,
                                                           measurement.pointIndices,
                                                           componentUvs,
                                                           Eigen::Vector2f(minCompU, minCompV),
                                                           Eigen::Vector2f(maxCompU, maxCompV),
                                                           localReferencePlane,
                                                           result.defectOnNegativeSide)
                : 0.0;
            measurement.depth = 0.0;
            measurement.plane = fitPlaneFromBottomRegion(componentPoints,
                                                         componentLocalDistances,
                                                         localReferencePlane.ok ? localReferencePlane : result.plane,
                                                         result.defectOnNegativeSide,
                                                         QString("%1_S%2").arg(name).arg(measurement.index),
                                                         &measurement.depth);
            const double trimmedDepth = robustDefectDepthTrimmed(componentLocalDistances,
                                                                 result.defectOnNegativeSide,
                                                                 std::max(localReferencePlane.ok ? localReferencePlane.rmse : result.plane.rmse, 0.003));
            const double sideDepth = trimmedDefectSideDepth(componentLocalDistances,
                                                            result.defectOnNegativeSide,
                                                            0.15);
            const double corePatchDepth = estimatePlaneDefectDepthFromCorePatch(componentUvs,
                                                                                componentLocalDistances,
                                                                                Eigen::Vector2f(minCompU, minCompV),
                                                                                Eigen::Vector2f(maxCompU, maxCompV),
                                                                                result.defectOnNegativeSide,
                                                                                std::max(localReferencePlane.ok ? localReferencePlane.rmse : result.plane.rmse, 0.003));
            measurement.profileDepth = profileDepth;
            measurement.trimmedDepth = trimmedDepth;
            measurement.sideDepth = sideDepth;
            measurement.corePatchDepth = corePatchDepth;
            measurement.bottomPlaneDepth = measurement.depth;
            if (measurement.plane.ok) {
                measurement.minDeviation = measurement.plane.minDeviation;
                measurement.maxDeviation = measurement.plane.maxDeviation;
            } else {
                measurement.minDeviation = 0.0;
                measurement.maxDeviation = 0.0;
            }

            const double fusedDepth = fuseDepthEstimates(profileDepth,
                                                         trimmedDepth,
                                                         measurement.depth,
                                                         std::max((localReferencePlane.ok ? localReferencePlane.rmse : result.plane.rmse) * 2.4, 0.02),
                                                         false);
            measurement.fusedDepth = fusedDepth;
            const double consensusDepth = conservativeDepthConsensus({corePatchDepth,
                                                                      profileDepth,
                                                                      trimmedDepth,
                                                                      sideDepth,
                                                                      measurement.depth,
                                                                      fusedDepth},
                                                                     std::max((localReferencePlane.ok ? localReferencePlane.rmse : result.plane.rmse) * 2.5, 0.025));
            measurement.consensusDepth = consensusDepth;
            const double biasReducedDepth = biasReducedDepthConsensus({corePatchDepth,
                                                                       profileDepth,
                                                                       trimmedDepth,
                                                                       sideDepth,
                                                                       measurement.bottomPlaneDepth,
                                                                       fusedDepth,
                                                                       consensusDepth},
                                                                      std::max((localReferencePlane.ok ? localReferencePlane.rmse : result.plane.rmse) * 2.0, 0.02),
                                                                      corePatchDepth);
            if (biasReducedDepth > 0.0) {
                measurement.depth = biasReducedDepth;
            } else if (consensusDepth > 0.0) {
                measurement.depth = consensusDepth;
            } else if (fusedDepth > 0.0) {
                measurement.depth = fusedDepth;
            } else if (sideDepth > 0.0) {
                measurement.depth = sideDepth;
            } else if (profileDepth > 0.0) {
                measurement.depth = profileDepth;
            }
            if (measurement.depth <= 0.0) {
                measurement.depth = trimmedDepth > 0.0
                    ? trimmedDepth
                    : robustDefectDepth(componentLocalDistances, result.defectOnNegativeSide);
            }
            if (measurement.depth <= 0.0) {
                measurement.depth = result.defectOnNegativeSide
                    ? std::abs(minLocalDeviation)
                    : std::abs(maxLocalDeviation);
            }
            measurement.minUv = Eigen::Vector2f(minCompU, minCompV);
            measurement.maxUv = Eigen::Vector2f(maxCompU, maxCompV);
            const auto hull = convexHull2d(componentUvs);
            measurement.convexHullArea = polygonArea2d(hull);
            measurement.area = closedContourArea2d(componentUvs);
            if (measurement.area <= 0.0) {
                measurement.area = measurement.convexHullArea;
            }
            const MinimumRectangle2d rect = minimumAreaRectangle2d(hull);
            if (rect.ok) {
                measurement.length = rect.length;
                measurement.width = rect.width;
                measurement.rectangleUv = rect.corners;
                measurement.hasRectangle = true;
            }
            measurement.ok = true;
            result.defects.push_back(std::move(measurement));
        }

        if (result.defects.size() > 7) {
            std::sort(result.defects.begin(), result.defects.end(), [](const PlaneDefectMeasurement& a,
                                                                       const PlaneDefectMeasurement& b) {
                if (!qFuzzyCompare(a.area, b.area)) {
                    return a.area > b.area;
                }
                return a.pointIndices.size() > b.pointIndices.size();
            });
            result.defects.resize(7);
        }
        std::sort(result.defects.begin(), result.defects.end(), [](const PlaneDefectMeasurement& a,
                                                                   const PlaneDefectMeasurement& b) {
            const Eigen::Vector2f ca = (a.minUv + a.maxUv) * 0.5f;
            const Eigen::Vector2f cb = (b.minUv + b.maxUv) * 0.5f;
            if (std::abs(ca.y() - cb.y()) > std::max(1.0f, std::abs(ca.x() - cb.x())) * 0.05f) {
                return ca.y() < cb.y();
            }
            return ca.x() < cb.x();
        });
        for (int i = 0; i < static_cast<int>(result.defects.size()); ++i) {
            result.defects[static_cast<size_t>(i)].index = i + 1;
        }
        result.ok = !result.defects.empty();
        return result;
    }

    static QString formatPlaneDefectAnalysisSummary(const QString& cloudName,
                                                    const PlaneDefectCropStats& cropStats,
                                                    const PlaneDefectAnalysisResult& analysis) {
        QString info = formatPlaneDefectCropSummary(cloudName, cropStats);
        info += QObject::tr("\n\n缺陷阈值: %1\n凹槽数量: %2")
            .arg(analysis.threshold, 0, 'f', 4)
            .arg(analysis.defects.size());
        for (const auto& defect : analysis.defects) {
            info += QObject::tr(
                "\n\nS%1\n"
                "点数: %2\n"
                "深度D: %3\n"
                "长度L: %4\n"
                "宽度W: %5\n"
                "投影封闭面积: %6\n"
                "凸包面积: %7\n"
                "底面内点数: %8\n"
                "底面平面方程: %9x + %10y + %11z + %12 = 0\n"
                "底面平面度(最大-最小偏差): %13\n"
                "底面最小偏差: %14\n"
                "底面最大偏差: %15")
                .arg(defect.index)
                .arg(defect.pointIndices.size())
                .arg(defect.depth, 0, 'f', 4)
                .arg(defect.length, 0, 'f', 4)
                .arg(defect.width, 0, 'f', 4)
                .arg(defect.area, 0, 'f', 4)
                .arg(defect.convexHullArea, 0, 'f', 4)
                .arg(defect.plane.inlierCount)
                .arg(defect.plane.normal.x(), 0, 'f', 6)
                .arg(defect.plane.normal.y(), 0, 'f', 6)
                .arg(defect.plane.normal.z(), 0, 'f', 6)
                .arg(defect.plane.d, 0, 'f', 6)
                .arg(defect.plane.flatness, 0, 'f', 4)
                .arg(defect.minDeviation, 0, 'f', 4)
                .arg(defect.maxDeviation, 0, 'f', 4);
            info += QObject::tr("\n  深度分量 core=%1 profile=%2 trimmed=%3 side=%4 plane=%5 fused=%6 consensus=%7")
                .arg(defect.corePatchDepth, 0, 'f', 4)
                .arg(defect.profileDepth, 0, 'f', 4)
                .arg(defect.trimmedDepth, 0, 'f', 4)
                .arg(defect.sideDepth, 0, 'f', 4)
                .arg(defect.bottomPlaneDepth, 0, 'f', 4)
                .arg(defect.fusedDepth, 0, 'f', 4)
                .arg(defect.consensusDepth, 0, 'f', 4);
        }
        return info;
    }

    static QString formatPlanePropertyInfo(const QString& cloudName,
                                           const PlaneFitStats& stats,
                                           const std::optional<double>& angleBetweenPlanes = std::nullopt) {
        QString info;
        info += cloudName + "\n";
        info += QObject::tr("拟合平面法向量: (%1, %2, %3)\n")
            .arg(stats.normal.x(), 0, 'f', 6)
            .arg(stats.normal.y(), 0, 'f', 6)
            .arg(stats.normal.z(), 0, 'f', 6);
        info += QObject::tr("平面方程: %1x + %2y + %3z + %4 = 0\n")
            .arg(stats.normal.x(), 0, 'f', 6)
            .arg(stats.normal.y(), 0, 'f', 6)
            .arg(stats.normal.z(), 0, 'f', 6)
            .arg(stats.d, 0, 'f', 6);
        info += QObject::tr("平均误差(ME): %1\nRMSE: %2\n最大绝对误差: %3\n")
            .arg(stats.meanError, 0, 'f', 4)
            .arg(stats.rmse, 0, 'f', 4)
            .arg(stats.maxAbsError, 0, 'f', 4);
        info += QObject::tr("与 XY 平面夹角: %1°\n与 XZ 平面夹角: %2°\n与 YZ 平面夹角: %3°")
            .arg(planeAngleDegrees(stats.normal, Eigen::Vector3f::UnitZ()), 0, 'f', 4)
            .arg(planeAngleDegrees(stats.normal, Eigen::Vector3f::UnitY()), 0, 'f', 4)
            .arg(planeAngleDegrees(stats.normal, Eigen::Vector3f::UnitX()), 0, 'f', 4);
        if (angleBetweenPlanes.has_value()) {
            info += QObject::tr("\n与另一拟合平面夹角: %1°").arg(*angleBetweenPlanes, 0, 'f', 4);
        }
        return info;
    }

    struct CylinderDefectMetrics {
        bool ok = false;
        double radius = 0.0;
        double minDeviation = 0.0;
        double maxDeviation = 0.0;
        double threshold = 0.0;
        bool positiveSide = true;
        double depth = 0.0;
        double axialLength = 0.0;
        double circumferentialArc = 0.0;
        double area = 0.0;
        double volume = 0.0;
        double thetaSpan = 0.0;
        int defectPointCount = 0;
        std::vector<int> pointIndices;
        std::vector<Eigen::Vector2f> surfaceUvs;
    };

    struct CylinderDefectThreshold {
        double signedThreshold = 0.0;
        bool positiveSide = true;
    };

    static std::pair<Eigen::Vector3f, Eigen::Vector3f> cylinderRadialBasis(const Eigen::Vector3f& axis) {
        Eigen::Vector3f base = std::abs(axis.z()) < 0.9f ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
        Eigen::Vector3f xAxis = axis.cross(base);
        if (xAxis.norm() < 1e-6f) {
            base = Eigen::Vector3f::UnitY();
            xAxis = axis.cross(base);
        }
        xAxis.normalize();
        Eigen::Vector3f yAxis = axis.cross(xAxis).normalized();
        return {xAxis, yAxis};
    }

    static double effectiveAngularSpan(std::vector<double> angles) {
        if (angles.size() < 2) {
            return 0.0;
        }
        constexpr double twoPi = 6.28318530717958647692;
        for (double& angle : angles) {
            angle = std::fmod(angle, twoPi);
            if (angle < 0.0) {
                angle += twoPi;
            }
        }
        std::sort(angles.begin(), angles.end());
        double maxGap = 0.0;
        for (size_t i = 1; i < angles.size(); ++i) {
            maxGap = std::max(maxGap, angles[i] - angles[i - 1]);
        }
        maxGap = std::max(maxGap, angles.front() + twoPi - angles.back());
        return std::max(0.0, twoPi - maxGap);
    }

    static CylinderDefectThreshold robustCylinderDefectThreshold(const FitStats& stats) {
        CylinderDefectThreshold result;
        std::vector<double> errors = stats.errors;
        if (errors.empty()) {
            return result;
        }
        std::sort(errors.begin(), errors.end());
        const auto valueAt = [&errors](double fraction) {
            const size_t index = std::min(errors.size() - 1,
                                          static_cast<size_t>(std::lround(fraction * static_cast<double>(errors.size() - 1))));
            return errors[index];
        };
        const double median = valueAt(0.50);
        std::vector<double> absDeviations;
        absDeviations.reserve(errors.size());
        for (double error : errors) {
            absDeviations.push_back(std::abs(error - median));
        }
        std::sort(absDeviations.begin(), absDeviations.end());
        const double mad = absDeviations[absDeviations.size() / 2];
        const double robustSigma = std::max(1.4826 * mad, stats.rmse);
        const double maxPositive = std::max(0.0, errors.back());
        const double maxNegative = std::max(0.0, -errors.front());
        double positiveThreshold = std::max({median + 3.0 * robustSigma, stats.rmse * 3.0, maxPositive * 0.25});
        double negativeThreshold = std::max({-median + 3.0 * robustSigma, stats.rmse * 3.0, maxNegative * 0.25});
        if (positiveThreshold >= maxPositive && maxPositive > 0.0) {
            positiveThreshold = maxPositive * 0.55;
        }
        if (negativeThreshold >= maxNegative && maxNegative > 0.0) {
            negativeThreshold = maxNegative * 0.55;
        }

        int positiveCount = 0;
        int negativeCount = 0;
        for (double error : stats.errors) {
            if (error >= positiveThreshold) {
                ++positiveCount;
            }
            if (-error >= negativeThreshold) {
                ++negativeCount;
            }
        }

        const int minCandidateCount = std::max(8, static_cast<int>(stats.errors.size() * 0.0005));
        const double positiveFraction = static_cast<double>(positiveCount) / std::max<size_t>(stats.errors.size(), 1);
        const double negativeFraction = static_cast<double>(negativeCount) / std::max<size_t>(stats.errors.size(), 1);
        const bool positiveLooksGlobal = positiveFraction > 0.25;
        const bool negativeLooksGlobal = negativeFraction > 0.25;
        if (negativeCount >= minCandidateCount && (positiveLooksGlobal || negativeFraction < positiveFraction * 0.75 || maxNegative > maxPositive * 1.15)) {
            result.positiveSide = false;
            result.signedThreshold = -negativeThreshold;
            return result;
        }
        if (positiveCount >= minCandidateCount && !positiveLooksGlobal) {
            result.positiveSide = true;
            result.signedThreshold = positiveThreshold;
            return result;
        }
        if (negativeCount >= minCandidateCount && !negativeLooksGlobal) {
            result.positiveSide = false;
            result.signedThreshold = -negativeThreshold;
            return result;
        }
        result.positiveSide = maxPositive >= maxNegative;
        result.signedThreshold = result.positiveSide ? std::max(0.0, positiveThreshold) : -std::max(0.0, negativeThreshold);
        return result;
    }

    static FitStats cylinderStatsForMeasuredPoints(const std::vector<Eigen::Vector3f>& points,
                                                   const FitStats& fittedStats) {
        FitStats stats = fittedStats;
        stats.errors.clear();
        stats.errors.reserve(points.size());
        stats.axisMin = std::numeric_limits<double>::max();
        stats.axisMax = std::numeric_limits<double>::lowest();
        for (const auto& point : points) {
            const Eigen::Vector3f delta = point - fittedStats.axisPoint;
            const double axisCoord = delta.dot(fittedStats.axis);
            const double distance = (delta - static_cast<float>(axisCoord) * fittedStats.axis).norm();
            stats.errors.push_back(distance - fittedStats.radius);
            stats.axisMin = std::min(stats.axisMin, axisCoord);
            stats.axisMax = std::max(stats.axisMax, axisCoord);
        }
        if (!std::isfinite(stats.axisMin) || !std::isfinite(stats.axisMax) || stats.axisMax <= stats.axisMin) {
            stats.axisMin = fittedStats.axisMin;
            stats.axisMax = fittedStats.axisMax;
        }
        stats.meanError = 0.0;
        stats.rmse = 0.0;
        stats.maxAbsError = 0.0;
        fillErrorStats(stats);
        stats.ok = fittedStats.ok && !points.empty();
        return stats;
    }

    static double quantizedErrorValue(double errorValue) {
        const double clamped = std::clamp(errorValue, -1.0, 1.0);
        return std::round(clamped * 10.0) / 10.0;
    }

    static cv::Vec3b errorPaletteColor(double quantizedError) {
        const std::array<std::pair<double, cv::Vec3d>, 4> stops = {{
            {-1.0, cv::Vec3d(255.0, 0.0, 0.0)},
            {0.0, cv::Vec3d(0.0, 255.0, 0.0)},
            {0.5, cv::Vec3d(0.0, 255.0, 255.0)},
            {1.0, cv::Vec3d(0.0, 0.0, 255.0)}
        }};
        const double value = std::clamp(quantizedError, -1.0, 1.0);
        for (size_t i = 1; i < stops.size(); ++i) {
            if (value <= stops[i].first) {
                const double leftPos = stops[i - 1].first;
                const double rightPos = stops[i].first;
                const double t = (value - leftPos) / std::max(rightPos - leftPos, 1e-9);
                const cv::Vec3d color = stops[i - 1].second * (1.0 - t) + stops[i].second * t;
                return cv::Vec3b(static_cast<uchar>(std::clamp(color[0], 0.0, 255.0)),
                                 static_cast<uchar>(std::clamp(color[1], 0.0, 255.0)),
                                 static_cast<uchar>(std::clamp(color[2], 0.0, 255.0)));
            }
        }
        const cv::Vec3d& color = stops.back().second;
        return cv::Vec3b(static_cast<uchar>(color[0]),
                         static_cast<uchar>(color[1]),
                         static_cast<uchar>(color[2]));
    }

    static cv::Vec3b errorColor(double errorValue) {
        const double quantized = quantizedErrorValue(errorValue);
        return errorPaletteColor(quantized);
    }

    static CylinderDefectMetrics computeCylinderDefectMetrics(const std::vector<Eigen::Vector3f>& points,
                                                              const FitStats& stats,
                                                              double defectThreshold) {
        CylinderDefectMetrics metrics;
        if (points.empty() || stats.errors.size() != points.size()) {
            return metrics;
        }

        double minAxis = std::numeric_limits<double>::max();
        double maxAxis = std::numeric_limits<double>::lowest();
        std::vector<double> defectAngles;
        std::vector<Eigen::Vector2f> growCandidateUvs;
        std::vector<int> growCandidatePointIndices;
        std::vector<double> growCandidateAngles;
        std::vector<double> growCandidateDeviations;
        std::vector<char> growCandidateSeeds;
        double deviationSum = 0.0;
        const bool positiveSide = defectThreshold >= 0.0;
        const double absThreshold = std::abs(defectThreshold);
        const double growThreshold = std::max(0.0,
                                              std::min(absThreshold,
                                                       absThreshold - std::max(stats.rmse * 2.0, absThreshold * 0.18)));
        metrics.radius = stats.radius;
        metrics.threshold = defectThreshold;
        metrics.positiveSide = positiveSide;
        metrics.minDeviation = *std::min_element(stats.errors.begin(), stats.errors.end());
        metrics.maxDeviation = *std::max_element(stats.errors.begin(), stats.errors.end());
        const auto [radialX, radialY] = cylinderRadialBasis(stats.axis);
        for (size_t i = 0; i < points.size(); ++i) {
            const double signedDepth = positiveSide ? stats.errors[i] : -stats.errors[i];
            if (signedDepth < growThreshold) {
                continue;
            }
            const Eigen::Vector3f delta = points[i] - stats.axisPoint;
            const double axisCoord = delta.dot(stats.axis);
            const Eigen::Vector3f radialVec = delta - static_cast<float>(axisCoord) * stats.axis;
            const double theta = std::atan2(radialVec.dot(radialY), radialVec.dot(radialX));
            growCandidateUvs.emplace_back(static_cast<float>(stats.radius * theta), static_cast<float>(axisCoord));
            growCandidatePointIndices.push_back(static_cast<int>(i));
            growCandidateAngles.push_back(theta);
            growCandidateDeviations.push_back(signedDepth);
            growCandidateSeeds.push_back(signedDepth >= absThreshold ? 1 : 0);
        }
        if (growCandidatePointIndices.empty()) {
            return metrics;
        }

        float minU = std::numeric_limits<float>::max();
        float maxU = std::numeric_limits<float>::lowest();
        float minV = std::numeric_limits<float>::max();
        float maxV = std::numeric_limits<float>::lowest();
        for (const Eigen::Vector2f& uv : growCandidateUvs) {
            minU = std::min(minU, uv.x());
            maxU = std::max(maxU, uv.x());
            minV = std::min(minV, uv.y());
            maxV = std::max(maxV, uv.y());
        }
        const double spanU = std::max<double>(maxU - minU, 1e-6);
        const double spanV = std::max<double>(maxV - minV, 1e-6);
        const double densityCell = std::sqrt((spanU * spanV) / std::max<double>(growCandidateUvs.size(), 1.0)) * 2.4;
        const double spanCell = std::max(spanU, spanV) / 220.0;
        const double noiseCell = std::max(stats.rmse * 4.0, 0.02);
        const double maxCellSize = std::max(0.02, std::max(spanU, spanV) / 14.0);
        const double cellSize = std::clamp(std::max({densityCell, spanCell, noiseCell}), 0.02, maxCellSize);
        const int minComponentPoints = std::max(6, static_cast<int>(growCandidateUvs.size() * 0.002));
        const double wrapPeriod = std::max(stats.radius, 1e-6) * 6.28318530717958647692;
        auto components = connectedComponents2dPeriodic(growCandidateUvs, cellSize, minComponentPoints, wrapPeriod);
        if (components.empty()) {
            return metrics;
        }

        std::vector<int> mergedComponent;
        mergedComponent.reserve(growCandidateUvs.size());
        std::vector<char> selectedFlags(growCandidateUvs.size(), 0);
        for (const auto& component : components) {
            bool hasStrongSeed = false;
            for (int localIndex : component) {
                if (localIndex >= 0
                    && localIndex < static_cast<int>(growCandidateSeeds.size())
                    && growCandidateSeeds[static_cast<size_t>(localIndex)]) {
                    hasStrongSeed = true;
                    break;
                }
            }
            if (!hasStrongSeed) {
                continue;
            }
            for (int localIndex : component) {
                if (localIndex < 0 || localIndex >= static_cast<int>(selectedFlags.size())) {
                    continue;
                }
                if (selectedFlags[static_cast<size_t>(localIndex)]) {
                    continue;
                }
                selectedFlags[static_cast<size_t>(localIndex)] = 1;
                mergedComponent.push_back(localIndex);
            }
        }
        if (mergedComponent.empty()) {
            return metrics;
        }

        std::sort(mergedComponent.begin(), mergedComponent.end(), [&](int a, int b) {
            return growCandidateDeviations[static_cast<size_t>(a)] > growCandidateDeviations[static_cast<size_t>(b)];
        });

        if (mergedComponent.empty()) {
            return metrics;
        }

        metrics.pointIndices.reserve(mergedComponent.size());
        metrics.surfaceUvs.reserve(mergedComponent.size());
        defectAngles.reserve(mergedComponent.size());
        for (int localIndex : mergedComponent) {
            if (localIndex < 0 || localIndex >= static_cast<int>(growCandidatePointIndices.size())) {
                continue;
            }
            const int pointIndex = growCandidatePointIndices[static_cast<size_t>(localIndex)];
            const Eigen::Vector3f delta = points[static_cast<size_t>(pointIndex)] - stats.axisPoint;
            const double axisCoord = delta.dot(stats.axis);
            minAxis = std::min(minAxis, axisCoord);
            maxAxis = std::max(maxAxis, axisCoord);
            defectAngles.push_back(growCandidateAngles[static_cast<size_t>(localIndex)]);
            metrics.surfaceUvs.push_back(growCandidateUvs[static_cast<size_t>(localIndex)]);
            metrics.pointIndices.push_back(pointIndex);
            metrics.depth = std::max(metrics.depth, growCandidateDeviations[static_cast<size_t>(localIndex)]);
            deviationSum += growCandidateDeviations[static_cast<size_t>(localIndex)];
        }

        metrics.defectPointCount = static_cast<int>(metrics.pointIndices.size());
        if (metrics.defectPointCount == 0) {
            return metrics;
        }
        metrics.axialLength = maxAxis - minAxis;
        metrics.thetaSpan = effectiveAngularSpan(defectAngles);
        metrics.circumferentialArc = stats.radius * metrics.thetaSpan;
        metrics.area = closedContourArea2d(metrics.surfaceUvs);
        if (metrics.area <= 0.0) {
            const auto hull = convexHull2d(metrics.surfaceUvs);
            metrics.area = polygonArea2d(hull);
        }
        if (metrics.area <= 0.0) {
            metrics.area = metrics.axialLength * metrics.circumferentialArc;
        }
        metrics.volume = metrics.area * (deviationSum / static_cast<double>(metrics.defectPointCount));
        metrics.ok = true;
        return metrics;
    }

    static pointcloud::PointCloudPtr createCylinderDefectCloud(const std::vector<Eigen::Vector3f>& points,
                                                               const CylinderDefectMetrics& metrics,
                                                               const QString& name) {
        if (metrics.pointIndices.empty()) {
            return nullptr;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(metrics.pointIndices.size());
        for (int index : metrics.pointIndices) {
            if (index < 0 || index >= static_cast<int>(points.size())) {
                continue;
            }
            const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
            cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }
        if (cloud->empty()) {
            return nullptr;
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
        auto result = std::make_shared<pointcloud::PointCloudConcrete<pcl::PointXYZ>>(cloud, name);
        result->setColor(makeColorRgb(255, 80, 40, 255));
        return result;
    }

    static QString formatCylinderDefectReport(const QString& cloudName,
                                              const FitStats& stats,
                                              const CylinderDefectMetrics& metrics,
                                              bool manualThreshold) {
        QString info;
        info += cloudName + "\n";
        info += QObject::tr("拟合半径R: %1\n拟合直径: %2\n")
            .arg(stats.radius, 0, 'f', 4)
            .arg(stats.diameter, 0, 'f', 4);
        info += QObject::tr("转轴一点: (%1, %2, %3)\n转轴方向: (%4, %5, %6)\n")
            .arg(stats.axisPoint.x(), 0, 'f', 6)
            .arg(stats.axisPoint.y(), 0, 'f', 6)
            .arg(stats.axisPoint.z(), 0, 'f', 6)
            .arg(stats.axis.x(), 0, 'f', 6)
            .arg(stats.axis.y(), 0, 'f', 6)
            .arg(stats.axis.z(), 0, 'f', 6);
        info += QObject::tr("最小偏差: %1\n最大偏差: %2\n阈值(%3): %4\n缺陷点数: %5\n")
            .arg(metrics.minDeviation, 0, 'f', 4)
            .arg(metrics.maxDeviation, 0, 'f', 4)
            .arg(manualThreshold ? QObject::tr("手动") : QObject::tr("自适应"))
            .arg(metrics.threshold, 0, 'f', 4)
            .arg(metrics.defectPointCount);
        info += QObject::tr("缺陷方向: %1\n深度D: %2\n轴向长度: %3\n周向弧长: %4\n展开角度Δθeff(rad): %5\n投影面积: %6\n体积估算: %7")
            .arg(metrics.positiveSide ? QObject::tr("外凸/ρ-R为正") : QObject::tr("内凹/R-ρ为正"))
            .arg(metrics.depth, 0, 'f', 4)
            .arg(metrics.axialLength, 0, 'f', 4)
            .arg(metrics.circumferentialArc, 0, 'f', 4)
            .arg(metrics.thetaSpan, 0, 'f', 4)
            .arg(metrics.area, 0, 'f', 4)
            .arg(metrics.volume, 0, 'f', 4);
        return info;
    }

    static Eigen::Vector3f cylinderSurfaceUvToPoint(const Eigen::Vector2f& uv,
                                                    const FitStats& stats,
                                                    const Eigen::Vector3f& radialX,
                                                    const Eigen::Vector3f& radialY,
                                                    double radialOffset) {
        const double theta = uv.x() / std::max(stats.radius, 1e-9);
        const double radius = stats.radius + radialOffset;
        return stats.axisPoint
            + static_cast<float>(uv.y()) * stats.axis
            + static_cast<float>(radius * std::cos(theta)) * radialX
            + static_cast<float>(radius * std::sin(theta)) * radialY;
    }

    static Eigen::Vector2f bilerpUv(const std::array<Eigen::Vector2f, 4>& corners, float s, float t) {
        const Eigen::Vector2f bottom = corners[0] * (1.0f - s) + corners[1] * s;
        const Eigen::Vector2f top = corners[3] * (1.0f - s) + corners[2] * s;
        return bottom * (1.0f - t) + top * t;
    }

    static void addRegisteredCylinderUvPolyline(pointcloud::CloudWindow* cloudWindow,
                                                const FitStats& stats,
                                                const Eigen::Vector3f& radialX,
                                                const Eigen::Vector3f& radialY,
                                                const Eigen::Vector2f& uvStart,
                                                const Eigen::Vector2f& uvEnd,
                                                double radialOffset,
                                                double r,
                                                double g,
                                                double b,
                                                double lineWidth,
                                                const QString& idBase,
                                                int minSegments = 12) {
        if (!cloudWindow || idBase.isEmpty()) {
            return;
        }
        const double radius = std::max(stats.radius, 1e-9);
        const double thetaSpan = std::abs((uvEnd.x() - uvStart.x()) / radius);
        const int segments = std::clamp(std::max(minSegments,
                                                 static_cast<int>(std::ceil(thetaSpan / 0.10))),
                                        12,
                                        72);
        for (int seg = 0; seg < segments; ++seg) {
            const float t0 = static_cast<float>(seg) / static_cast<float>(segments);
            const float t1 = static_cast<float>(seg + 1) / static_cast<float>(segments);
            const Eigen::Vector2f uv0 = uvStart * (1.0f - t0) + uvEnd * t0;
            const Eigen::Vector2f uv1 = uvStart * (1.0f - t1) + uvEnd * t1;
            const Eigen::Vector3f p0 = cylinderSurfaceUvToPoint(uv0, stats, radialX, radialY, radialOffset);
            const Eigen::Vector3f p1 = cylinderSurfaceUvToPoint(uv1, stats, radialX, radialY, radialOffset);
            addRegisteredLine(cloudWindow,
                              p0,
                              p1,
                              r,
                              g,
                              b,
                              lineWidth,
                              QString("%1_seg_%2").arg(idBase).arg(seg));
        }
    }

    static void showCylinderDefectProjectionMesh(pointcloud::CloudWindow* cloudWindow,
                                                 const FitStats& stats,
                                                 const CylinderDefectMetrics& metrics,
                                                 const QString& baseId) {
        if (!cloudWindow || !stats.ok || !metrics.ok || metrics.surfaceUvs.size() < 2 || baseId.isEmpty()) {
            return;
        }
        const MinimumRectangle2d rect = minimumAreaRectangle2d(convexHull2d(metrics.surfaceUvs));
        if (!rect.ok) {
            return;
        }
        const auto [radialX, radialY] = cylinderRadialBasis(stats.axis);
        const double radialOffset = std::max(std::min(metrics.depth * 0.03, 0.08),
                                             std::max(stats.rmse * 0.75, 0.006));
        const int uDivisions = std::clamp(static_cast<int>(std::ceil(rect.length / std::max(stats.radius * 0.12, 0.8))), 4, 14);
        const int vDivisions = std::clamp(static_cast<int>(std::ceil(rect.width /  std::max((stats.axisMax - stats.axisMin) * 0.08, 0.8))), 4, 12);

        for (int i = 0; i <= uDivisions; ++i) {
            const float s = uDivisions > 0 ? static_cast<float>(i) / static_cast<float>(uDivisions) : 0.0f;
            addRegisteredCylinderUvPolyline(cloudWindow,
                                            stats,
                                            radialX,
                                            radialY,
                                            bilerpUv(rect.corners, s, 0.0f),
                                            bilerpUv(rect.corners, s, 1.0f),
                                            radialOffset,
                                            0.05,
                                            1.0,
                                            0.95,
                                            (i == 0 || i == uDivisions) ? 4.5 : 2.0,
                                            QString("%1_cylinder_defect_mesh_u_%2").arg(baseId).arg(i));
        }

        for (int j = 0; j <= vDivisions; ++j) {
            const float t = vDivisions > 0 ? static_cast<float>(j) / static_cast<float>(vDivisions) : 0.0f;
            addRegisteredCylinderUvPolyline(cloudWindow,
                                            stats,
                                            radialX,
                                            radialY,
                                            bilerpUv(rect.corners, 0.0f, t),
                                            bilerpUv(rect.corners, 1.0f, t),
                                            radialOffset,
                                            0.05,
                                            1.0,
                                            0.95,
                                            (j == 0 || j == vDivisions) ? 4.5 : 2.0,
                                            QString("%1_cylinder_defect_mesh_v_%2").arg(baseId).arg(j));
        }
    }

    static pointcloud::PointCloudPtr createErrorCloud(const std::vector<Eigen::Vector3f>& points,
                                                      const FitStats& stats,
                                                      const QString& name) {
        if (points.empty() || stats.errors.size() != points.size()) {
            return nullptr;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            const cv::Vec3b color = errorColor(stats.errors[i]);
            pcl::PointXYZRGB p;
            p.x = points[i].x();
            p.y = points[i].y();
            p.z = points[i].z();
            const std::uint32_t red = static_cast<std::uint32_t>(color[2]);
            const std::uint32_t green = static_cast<std::uint32_t>(color[1]);
            const std::uint32_t blue = static_cast<std::uint32_t>(color[0]);
            p.rgba = (255u << 24) | (red << 16) | (green << 8) | blue;
            cloud->push_back(p);
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
        return std::make_shared<pointcloud::PointCloudConcrete<pcl::PointXYZRGB>>(cloud, name);
    }

    static pointcloud::PointCloudPtr createCylinderErrorCloud(const std::vector<Eigen::Vector3f>& points,
                                                              const FitStats& stats,
                                                              const QString& name) {
        return createErrorCloud(points, stats, name);
    }

    static pointcloud::PointCloudPtr createSphereErrorCloud(const std::vector<Eigen::Vector3f>& points,
                                                            const FitStats& stats,
                                                            const QString& name) {
        return createErrorCloud(points, stats, name);
    }

    static FitStats cylinderStatsWithFixedDiameter(const std::vector<Eigen::Vector3f>& points, double standardDiameter) {
        FitStats stats = fitCylinderStats(points);
        if (!stats.ok || standardDiameter <= 0.0) {
            return stats;
        }
        stats.standardDiameter = standardDiameter;
        stats.errors.clear();
        stats.errors.reserve(points.size());
        const double standardRadius = standardDiameter * 0.5;
        for (const auto& point : points) {
            const Eigen::Vector3f delta = point - stats.axisPoint;
            const double distance = (delta - delta.dot(stats.axis) * stats.axis).norm();
            stats.errors.push_back(distance - standardRadius);
        }
        stats.meanError = 0.0;
        stats.rmse = 0.0;
        stats.maxAbsError = 0.0;
        fillErrorStats(stats);
        return stats;
    }

    static FitStats sphereErrorStatsAgainstStandard(const std::vector<Eigen::Vector3f>& points,
                                                    const FitStats& fittedStats,
                                                    double standardDiameter,
                                                    const QString& name = "SphereStandard") {
        FitStats stats = fittedStats;
        stats.name = name;
        if (!stats.ok || standardDiameter <= 0.0) {
            return stats;
        }
        stats.standardDiameter = standardDiameter;
        const double standardRadius = standardDiameter * 0.5;
        stats.errors.clear();
        stats.errors.reserve(points.size());
        const bool hasFittedErrors = fittedStats.errors.size() == points.size();
        double errorParameterSum = 0.0;
        int errorParameterCount = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            const double standardError = (points[i] - stats.center).norm() - standardRadius;
            stats.errors.push_back(standardError);
            if (hasFittedErrors) {
                errorParameterSum += standardError - fittedStats.errors[i];
                ++errorParameterCount;
            }
        }
        stats.meanError = 0.0;
        stats.rmse = 0.0;
        stats.maxAbsError = 0.0;
        if (errorParameterCount > 0) {
            stats.errorParameter = errorParameterSum / static_cast<double>(errorParameterCount);
        } else {
            stats.errorParameter = fittedStats.radius - standardRadius;
        }
        fillErrorStats(stats);
        return stats;
    }

    static void writeFitErrorsCsv(const QString& path, const std::vector<FitStats>& statsList) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            return;
        }
        QTextStream out(&file);
        out << "feature,index,error\n";
        for (const auto& stats : statsList) {
            for (int i = 0; i < static_cast<int>(stats.errors.size()); ++i) {
                out << stats.name << "," << i << "," << stats.errors[i] << "\n";
            }
        }
    }

    static bool writeFitErrorsTable(const QString& path, const std::vector<FitStats>& statsList) {
        QFile file(path);
        const bool fileExists = QFileInfo::exists(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | (fileExists ? QIODevice::Append : QIODevice::Truncate))) {
            return false;
        }

        const QString suffix = QFileInfo(path).suffix().toLower();
        const bool csv = (suffix == "csv");
        QTextStream out(&file);
        const QChar sep = csv ? QChar(',') : QChar('\t');
        if (!fileExists || file.size() == 0) {
            out << "feature" << sep
                << "standard_diameter" << sep
                << "fitted_diameter" << sep
                << "mean_error" << sep
                << "rmse" << sep
                << "error_parameter" << sep
                << "sample_count\n";
        } else if (file.pos() > 0) {
            out << "\n";
        }
        for (const auto& stats : statsList) {
            out << stats.name << sep
                << stats.standardDiameter << sep
                << stats.diameter << sep
                << stats.meanError << sep
                << stats.rmse << sep
                << (hasErrorParameter(stats) ? QString::number(stats.errorParameter, 'f', 6) : QString()) << sep
                << static_cast<int>(stats.errors.size()) << "\n";
        }
        return true;
    }

    static std::vector<FitStats> toStdFitStats(const QVector<FitStats>& statsList) {
        std::vector<FitStats> result;
        result.reserve(statsList.size());
        for (const auto& stats : statsList) {
            result.push_back(stats);
        }
        return result;
    }

    static bool saveFitErrorsWithDialog(QWidget* parent, const QString& defaultName, const QVector<FitStats>& statsList) {
        if (!parent || statsList.isEmpty()) {
            return false;
        }

        QString suggestedName = defaultName.trimmed();
        if (suggestedName.isEmpty()) {
            suggestedName = "fit_errors";
        }
        {
            QFileDialog dialog(
                parent,
                QObject::tr("保存误差数据"),
                QDir::homePath() + "/" + suggestedName + ".csv",
                QObject::tr("CSV文件(*.csv);;Excel文件(*.xlx *.xls *.xlsx)")
            );
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            dialog.setFileMode(QFileDialog::AnyFile);
            dialog.setOption(QFileDialog::DontConfirmOverwrite, true);
            if (dialog.exec() != QDialog::Accepted) {
                return false;
            }
            const QStringList selectedFiles = dialog.selectedFiles();
            if (selectedFiles.isEmpty()) {
                return false;
            }
            const QString path = selectedFiles.first();
            if (path.isEmpty()) {
                return false;
            }
            return writeFitErrorsTable(path, toStdFitStats(statsList));
        }
        const QString path = QFileDialog::getSaveFileName(
            parent,
            QObject::tr("保存误差数据"),
            QDir::homePath() + "/" + suggestedName + ".csv",
            QObject::tr("CSV文件(*.csv);;Excel文件(*.xls *.xlsx)")
        );
        if (path.isEmpty()) {
            return false;
        }

        return writeFitErrorsTable(path, toStdFitStats(statsList));
    }

    static bool readErrorTable(const QString& path, QVector<ErrorAnalysisRow>* rows) {
        if (!rows) {
            return false;
        }
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            return false;
        }

        const QString suffix = QFileInfo(path).suffix().toLower();
        const bool csv = (suffix == "csv");
        QTextStream in(&file);
        bool firstLine = true;
        while (!in.atEnd()) {
            const QString line = in.readLine().trimmed();
            if (line.isEmpty()) {
                continue;
            }
            if (firstLine) {
                firstLine = false;
                if (line.startsWith("feature", Qt::CaseInsensitive)) {
                    continue;
                }
            }
            const QStringList parts = csv ? line.split(',') : line.split('\t');
            if (parts.size() < 6) {
                continue;
            }
            bool okStandard = false;
            bool okFitted = false;
            bool okMean = false;
            bool okRmse = false;
            bool okCount = false;
            ErrorAnalysisRow row;
            row.featureName = parts.at(0).trimmed();
            row.standardDiameter = parts.at(1).toDouble(&okStandard);
            row.fittedDiameter = parts.at(2).toDouble(&okFitted);
            row.meanError = parts.at(3).toDouble(&okMean);
            row.rmse = parts.at(4).toDouble(&okRmse);
            int sampleCountIndex = 5;
            if (parts.size() > 6) {
                const QString errorParameterText = parts.at(5).trimmed();
                if (!errorParameterText.isEmpty()) {
                    bool okErrorParameter = false;
                    const double parsedErrorParameter = errorParameterText.toDouble(&okErrorParameter);
                    if (okErrorParameter) {
                        row.errorParameter = parsedErrorParameter;
                    }
                }
                sampleCountIndex = 6;
            }
            if (parts.size() <= sampleCountIndex) {
                continue;
            }
            row.sampleCount = parts.at(sampleCountIndex).toInt(&okCount);
            if (!okStandard || !okFitted || !okMean || !okRmse || !okCount) {
                continue;
            }
            rows->push_back(row);
        }
        return !rows->isEmpty();
    }

    static bool readErrorTable(const QString& path, QVector<double>* errors, QString* featureName = nullptr) {
        if (!errors) {
            return false;
        }

        QVector<ErrorAnalysisRow> rows;
        if (!readErrorTable(path, &rows)) {
            return false;
        }

        errors->clear();
        errors->reserve(rows.size());
        for (const auto& row : rows) {
            errors->push_back(row.meanError);
        }

        if (featureName && !rows.isEmpty()) {
            *featureName = rows.first().featureName;
        }
        return !errors->isEmpty();
    }

    static QPixmap createErrorLineChart(const QVector<double>& errors, const QString& title, const QSize& size = QSize(960, 540)) {
        QPixmap pixmap(size);
        pixmap.fill(Qt::white);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRect chartRect(70, 40, size.width() - 110, size.height() - 110);
        painter.setPen(QPen(QColor(210, 210, 210), 1));
        for (int i = 0; i <= 5; ++i) {
            const int y = chartRect.top() + i * chartRect.height() / 5;
            painter.drawLine(chartRect.left(), y, chartRect.right(), y);
        }
        painter.setPen(QPen(Qt::black, 1.5));
        painter.drawRect(chartRect);

        double minValue = 0.0;
        double maxValue = 0.0;
        for (double value : errors) {
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
        }
        if (qFuzzyCompare(minValue, maxValue)) {
            minValue -= 1.0;
            maxValue += 1.0;
        }
        const double range = std::max(maxValue - minValue, 1e-9);

        QPainterPath path;
        for (int i = 0; i < errors.size(); ++i) {
            const double xRatio = errors.size() > 1 ? static_cast<double>(i) / static_cast<double>(errors.size() - 1) : 0.0;
            const double yRatio = (errors.at(i) - minValue) / range;
            const QPointF point(chartRect.left() + xRatio * chartRect.width(),
                                chartRect.bottom() - yRatio * chartRect.height());
            if (i == 0) {
                path.moveTo(point);
            } else {
                path.lineTo(point);
            }
        }
        painter.setPen(QPen(QColor(28, 110, 164), 2.0));
        painter.drawPath(path);

        painter.setPen(Qt::black);
        painter.drawText(QRect(0, 8, size.width(), 24), Qt::AlignCenter, title);
        painter.drawText(QRect(chartRect.left(), size.height() - 50, chartRect.width(), 24), Qt::AlignCenter, "Index");
        painter.save();
        painter.translate(18, chartRect.center().y());
        painter.rotate(-90);
        painter.drawText(QRect(-chartRect.height() / 2, -20, chartRect.height(), 24), Qt::AlignCenter, "Error");
        painter.restore();
        painter.drawText(10, chartRect.top() + 4, QString::number(maxValue, 'f', 4));
        painter.drawText(10, chartRect.bottom(), QString::number(minValue, 'f', 4));

        return pixmap;
    }

    static QString formatErrorSummary(const QVector<double>& errors) {
        if (errors.isEmpty()) {
            return QString();
        }

        double sum = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        for (double value : errors) {
            sum += value;
            sumSq += value * value;
            maxAbs = std::max(maxAbs, std::abs(value));
        }

        const double mean = sum / static_cast<double>(errors.size());
        const double rmse = std::sqrt(sumSq / static_cast<double>(errors.size()));
        return QObject::tr("样本数：%1    平均误差：%2    RMSE：%3    最大绝对误差：%4")
            .arg(errors.size())
            .arg(mean, 0, 'f', 4)
            .arg(rmse, 0, 'f', 4)
            .arg(maxAbs, 0, 'f', 4);
    }

    static QPixmap createSummaryAnalysisChart(const QVector<ErrorAnalysisRow>& rows, const QString& title, const QSize& size = QSize(1100, 560)) {
        QPixmap pixmap(size);
        pixmap.fill(Qt::white);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRect chartRect(80, 40, size.width() - 260, size.height() - 110);
        painter.setPen(QPen(QColor(210, 210, 210), 1));
        for (int i = 0; i <= 5; ++i) {
            const int y = chartRect.top() + i * chartRect.height() / 5;
            painter.drawLine(chartRect.left(), y, chartRect.right(), y);
        }
        painter.setPen(QPen(Qt::black, 1.5));
        painter.drawRect(chartRect);

        QVector<QPair<QString, QVector<double>>> displaySeries = {
            {QObject::tr("拟合直径"), {}},
            {QObject::tr("标准直径 + 平均误差"), {}},
            {QObject::tr("标准直径 + RMSE"), {}},
            {QObject::tr("标准直径 - RMSE"), {}}
        };
        displaySeries[1].first = QObject::tr("拟合直径 + 平均误差");
        displaySeries[2].first = QObject::tr("拟合直径 + RMSE");
        displaySeries[3].first = QObject::tr("拟合直径 - RMSE");
        const bool includeErrorParameter = std::any_of(rows.begin(), rows.end(), [](const ErrorAnalysisRow& row) {
            return hasErrorParameter(row);
        });
        if (includeErrorParameter) {
            displaySeries.push_back({QObject::tr("拟合直径 + 误差参数"), {}});
        }
        for (const auto& row : rows) {
            displaySeries[0].second.push_back(row.fittedDiameter);
            displaySeries[1].second.push_back(row.fittedDiameter + row.meanError);
            displaySeries[2].second.push_back(row.fittedDiameter + row.rmse);
            displaySeries[3].second.push_back(row.fittedDiameter - row.rmse);
            if (includeErrorParameter) {
                displaySeries[4].second.push_back(hasErrorParameter(row) ? row.fittedDiameter + row.errorParameter : row.fittedDiameter);
            }
        }

        QVector<QColor> displayColors = {
            QColor(0, 114, 188),
            QColor(34, 139, 34),
            QColor(255, 140, 0),
            QColor(160, 82, 45)
        };
        if (includeErrorParameter) {
            displayColors.push_back(QColor(138, 43, 226));
        }

        const double axisCenter = rows.isEmpty() ? 0.0 : rows.first().standardDiameter;
        double minValue = std::numeric_limits<double>::max();
        double maxValue = std::numeric_limits<double>::lowest();
        for (const auto& series : displaySeries) {
            for (double value : series.second) {
                minValue = std::min(minValue, value);
                maxValue = std::max(maxValue, value);
            }
        }
        if (!rows.isEmpty()) {
            minValue = std::min(minValue, axisCenter);
            maxValue = std::max(maxValue, axisCenter);
        }
        if (minValue == std::numeric_limits<double>::max() || maxValue == std::numeric_limits<double>::lowest()) {
            minValue = 0.0;
            maxValue = 1.0;
        }
        if (qFuzzyCompare(minValue, maxValue)) {
            minValue -= 1.0;
            maxValue += 1.0;
        } else {
            const double padding = std::max((maxValue - minValue) * 0.08, 1e-6);
            minValue -= padding;
            maxValue += padding;
        }
        const double range = std::max(maxValue - minValue, 1e-9);
        const double centerRatio = (axisCenter - minValue) / range;
        const int centerY = static_cast<int>(std::lround(chartRect.bottom() - centerRatio * chartRect.height()));

        painter.setPen(QPen(QColor(220, 20, 60), 1.5, Qt::DashLine));
        painter.drawLine(chartRect.left(), centerY, chartRect.right(), centerY);

        for (int s = 0; s < displaySeries.size(); ++s) {
            const auto& values = displaySeries.at(s).second;
            QPainterPath path;
            for (int i = 0; i < values.size(); ++i) {
                const double xRatio = values.size() > 1 ? static_cast<double>(i) / static_cast<double>(values.size() - 1) : 0.0;
                const double yRatio = (values.at(i) - minValue) / range;
                const QPointF point(chartRect.left() + xRatio * chartRect.width(),
                                    chartRect.bottom() - yRatio * chartRect.height());
                if (i == 0) {
                    path.moveTo(point);
                } else {
                    path.lineTo(point);
                }
            }
            painter.setPen(QPen(displayColors.at(s), 2.0));
            painter.drawPath(path);
        }

        painter.setPen(Qt::black);
        painter.drawText(QRect(0, 8, size.width(), 24), Qt::AlignCenter, title);
        painter.drawText(QRect(chartRect.left(), size.height() - 50, chartRect.width(), 24), Qt::AlignCenter, "Index");
        painter.save();
        painter.translate(18, chartRect.center().y());
        painter.rotate(-90);
        painter.drawText(QRect(-chartRect.height() / 2, -20, chartRect.height(), 24), Qt::AlignCenter, "Diameter");
        painter.restore();
        painter.drawText(10, chartRect.top() + 4, QString::number(maxValue, 'f', 4));
        painter.drawText(10, centerY + 4, QString::number(axisCenter, 'f', 4));
        painter.drawText(10, chartRect.bottom(), QString::number(minValue, 'f', 4));

        const int legendX = chartRect.right() + 24;
        int legendY = chartRect.top() + 16;
        painter.setPen(Qt::black);
        painter.drawText(legendX, legendY - 10, QObject::tr("图例"));
        painter.setPen(QPen(QColor(220, 20, 60), 2.0, Qt::DashLine));
        painter.drawLine(legendX, legendY, legendX + 24, legendY);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 34, legendY + 5, QObject::tr("标准直径"));
        legendY += 28;
        for (int i = 0; i < displaySeries.size(); ++i) {
            painter.setPen(QPen(displayColors.at(i), 3.0));
            painter.drawLine(legendX, legendY + i * 28, legendX + 24, legendY + i * 28);
            painter.setPen(Qt::black);
            painter.drawText(legendX + 34, legendY + 5 + i * 28, displaySeries.at(i).first);
        }

        return pixmap;

#if 0
        QVector<QPair<QString, QVector<double>>> series = {
            {QObject::tr("标准直径"), {}},
            {QObject::tr("拟合直径"), {}},
            {QObject::tr("平均误差"), {}},
            {QObject::tr("RMSE"), {}},
            {QObject::tr("样本数"), {}}
        };
        for (const auto& row : rows) {
            series[0].second.push_back(row.standardDiameter);
            series[1].second.push_back(row.fittedDiameter);
            series[2].second.push_back(row.meanError);
            series[3].second.push_back(row.rmse);
            series[4].second.push_back(static_cast<double>(row.sampleCount));
        }

        const QVector<QColor> colors = {
            QColor(220, 20, 60),
            QColor(0, 114, 188),
            QColor(34, 139, 34),
            QColor(255, 140, 0),
            QColor(138, 43, 226)
        };

        double minValue = std::numeric_limits<double>::max();
        double maxValue = std::numeric_limits<double>::lowest();
        for (const auto& item : series) {
            for (double value : item.second) {
                minValue = std::min(minValue, value);
                maxValue = std::max(maxValue, value);
            }
        }
        if (minValue == std::numeric_limits<double>::max()) {
            minValue = 0.0;
            maxValue = 1.0;
        }
        if (qFuzzyCompare(minValue, maxValue)) {
            minValue -= 1.0;
            maxValue += 1.0;
        }
        const double range = std::max(maxValue - minValue, 1e-9);

        for (int s = 0; s < series.size(); ++s) {
            const auto& values = series.at(s).second;
            QPainterPath path;
            for (int i = 0; i < values.size(); ++i) {
                const double xRatio = values.size() > 1 ? static_cast<double>(i) / static_cast<double>(values.size() - 1) : 0.0;
                const double yRatio = (values.at(i) - minValue) / range;
                const QPointF point(chartRect.left() + xRatio * chartRect.width(),
                                    chartRect.bottom() - yRatio * chartRect.height());
                if (i == 0) {
                    path.moveTo(point);
                } else {
                    path.lineTo(point);
                }
            }
            painter.setPen(QPen(colors.at(s), 2.0));
            painter.drawPath(path);
        }

        painter.setPen(Qt::black);
        painter.drawText(QRect(0, 8, size.width(), 24), Qt::AlignCenter, title);
        painter.drawText(QRect(chartRect.left(), size.height() - 50, chartRect.width(), 24), Qt::AlignCenter, "Index");
        painter.save();
        painter.translate(18, chartRect.center().y());
        painter.rotate(-90);
        painter.drawText(QRect(-chartRect.height() / 2, -20, chartRect.height(), 24), Qt::AlignCenter, "Value");
        painter.restore();
        painter.drawText(10, chartRect.top() + 4, QString::number(maxValue, 'f', 4));
        painter.drawText(10, chartRect.bottom(), QString::number(minValue, 'f', 4));

        const int legendX = chartRect.right() + 24;
        int legendY = chartRect.top() + 16;
        painter.setPen(Qt::black);
        painter.drawText(legendX, legendY - 10, QObject::tr("图例"));
        for (int i = 0; i < series.size(); ++i) {
            painter.setPen(QPen(colors.at(i), 3.0));
            painter.drawLine(legendX, legendY + i * 28, legendX + 24, legendY + i * 28);
            painter.setPen(Qt::black);
            painter.drawText(legendX + 34, legendY + 5 + i * 28, series.at(i).first);
        }

        return pixmap;
#endif
    }

    static QString formatSummaryAnalysisText(const QVector<ErrorAnalysisRow>& rows) {
        if (rows.isEmpty()) {
            return QString();
        }
        bool hasAnyErrorParameter = false;
        double minErrorParameter = std::numeric_limits<double>::max();
        double maxErrorParameter = std::numeric_limits<double>::lowest();
        for (const auto& row : rows) {
            if (!hasErrorParameter(row)) {
                continue;
            }
            hasAnyErrorParameter = true;
            minErrorParameter = std::min(minErrorParameter, row.errorParameter);
            maxErrorParameter = std::max(maxErrorParameter, row.errorParameter);
        }
        QString summary = QObject::tr("共 %1 组数据，图中绘制拟合直径、拟合直径基准下的平均误差与 RMSE 上下界，纵轴会根据数据自动覆盖全部点。")
            .arg(rows.size());
        if (hasAnyErrorParameter) {
            summary += QObject::tr("\n误差参数(标准误差-拟合误差)范围：%1 ~ %2")
                .arg(minErrorParameter, 0, 'f', 4)
                .arg(maxErrorParameter, 0, 'f', 4);
        }
        return summary;
        return QStringLiteral("共 %1 组数据，图中绘制拟合直径、标准直径基准下的平均误差和 RMSE 上下界，纵轴固定为标准直径 ±1。")
            .arg(rows.size());
        return QObject::tr("共 %1 组数据，不同颜色分别表示标准直径、拟合直径、平均误差、RMSE 和样本数。")
            .arg(rows.size());
    }

    static QString formatFitReport(const FitStats& stats, double standardDiameter) {
        const double deviation = standardDiameter > 0.0 ? stats.diameter - standardDiameter : 0.0;
        QString report = QString("%1\n直径：%2\n半径：%3\n标准值：%4\n偏差：%5\n平均误差：%6\nRMSE：%7\n最大绝对误差：%8\n样本数：%9")
            .arg(stats.name)
            .arg(stats.diameter, 0, 'f', 4)
            .arg(stats.radius, 0, 'f', 4)
            .arg(standardDiameter > 0.0 ? QString::number(standardDiameter, 'f', 4) : QObject::tr("未输入"))
            .arg(standardDiameter > 0.0 ? QString::number(deviation, 'f', 4) : QObject::tr("未计算"))
            .arg(stats.meanError, 0, 'f', 4)
            .arg(stats.rmse, 0, 'f', 4)
            .arg(stats.maxAbsError, 0, 'f', 4)
            .arg(stats.errors.size());
        if (hasErrorParameter(stats)) {
            report += QObject::tr("\n误差参数(标准误差-拟合误差)：%1")
                .arg(stats.errorParameter, 0, 'f', 4);
        }
        return report;
        return QString("%1\n直径：%2\n半径：%3\n标准值：%4\n偏差：%5\n平均误差：%6\nRMSE：%7\n最大绝对误差：%8\n样本数：%9")
            .arg(stats.name)
            .arg(stats.diameter, 0, 'f', 4)
            .arg(stats.radius, 0, 'f', 4)
            .arg(standardDiameter > 0.0 ? QString::number(standardDiameter, 'f', 4) : QString("未输入"))
            .arg(standardDiameter > 0.0 ? QString::number(deviation, 'f', 4) : QString("未计算"))
            .arg(stats.meanError, 0, 'f', 4)
            .arg(stats.rmse, 0, 'f', 4)
            .arg(stats.maxAbsError, 0, 'f', 4)
            .arg(stats.errors.size());
    }

    static QString formatFitFooterMessage(const QString& name, const FitStats& stats, bool standardReference) {
        return QString("%1 误差：ME=%2  RMSE=%3  MAX=%4%5")
            .arg(name)
            .arg(stats.meanError, 0, 'f', 4)
            .arg(stats.rmse, 0, 'f', 4)
            .arg(stats.maxAbsError, 0, 'f', 4)
            .arg(standardReference ? QString("  [标准]") : QString());
    }

    static QString formatFitPropertyInfo(const QString& title,
                                         const FitStats& fitStats,
                                         const FitStats& errorStats,
                                         double standardDiameter,
                                         bool standardReference) {
        QString info;
        info += title + "\n";
        info += QString("拟合直径: %1\n拟合半径: %2\n")
            .arg(fitStats.diameter, 0, 'f', 4)
            .arg(fitStats.radius, 0, 'f', 4);
        if (standardDiameter > 0.0) {
            info += QString("标准直径: %1\n标准半径: %2\n")
                .arg(standardDiameter, 0, 'f', 4)
                .arg(standardDiameter * 0.5, 0, 'f', 4);
        }
        info += QString("平均误差(ME): %1\nRMSE: %2\n最大绝对误差: %3\n样本数: %4")
            .arg(errorStats.meanError, 0, 'f', 4)
            .arg(errorStats.rmse, 0, 'f', 4)
            .arg(errorStats.maxAbsError, 0, 'f', 4)
            .arg(errorStats.errors.size());
        if (hasErrorParameter(errorStats)) {
            info += QObject::tr("\n误差参数(标准误差-拟合误差): %1")
                .arg(errorStats.errorParameter, 0, 'f', 4);
        }
        if (standardReference) {
            info += "\n误差参考: 标准球";
        } else {
            info += "\n误差参考: 拟合球";
        }
        return info;
    }

    struct StaticScanSettings {
        QString settingsPath;
        QString calibrationFile;
        QString scanImageFormat = "bmp";
        QString centerlineMethod;
        int centerlineMethodIndex = 0;
        double bwThreshold = 55.0;
        double sumThreshold = 124.0;
        bool isHorizontal = true;
        float startPosition = 0.0f;
        float endPosition = 0.0f;
        float signedStep = 0.0f;
    };

    static QString normalizedImageFormat(const QString& format, const QString& fallback = "bmp") {
        QString ret = format.trimmed().toLower();
        if (ret.startsWith('.')) {
            ret.remove(0, 1);
        }
        return ret.isEmpty() ? fallback : ret;
    }

    static int scanPositionToFrameIndex(float position) {
        return static_cast<int>(std::round(position * 100.0f));
    }

    static std::optional<double> scanFileBaseNameToPosition(const QString& baseName, double rangeLower, double rangeUpper) {
        bool ok = false;
        const double rawValue = baseName.toDouble(&ok);
        QString leadingToken = baseName.section('_', 0, 0);
        if (!ok && leadingToken != baseName) {
            const double tokenValue = leadingToken.toDouble(&ok);
            if (ok) {
                const double rawToken = tokenValue;
                if (rawToken >= rangeLower && rawToken <= rangeUpper) {
                    return rawToken;
                }
                const double centiToken = rawToken / 100.0;
                if (centiToken >= rangeLower && centiToken <= rangeUpper) {
                    return centiToken;
                }
            }
            ok = false;
        }
        if (!ok) {
            return std::nullopt;
        }

        if (rawValue >= rangeLower && rawValue <= rangeUpper) {
            return rawValue;
        }

        const double centiPosition = rawValue / 100.0;
        if (centiPosition >= rangeLower && centiPosition <= rangeUpper) {
            return centiPosition;
        }

        return std::nullopt;
    }

    static std::optional<double> scanFileBaseNameToFreePosition(const QString& baseName) {
        bool ok = false;
        QString valueText = baseName;
        double rawValue = valueText.toDouble(&ok);
        if (!ok) {
            const QString leadingToken = baseName.section('_', 0, 0);
            if (leadingToken == baseName) {
                return std::nullopt;
            }
            valueText = leadingToken;
            rawValue = valueText.toDouble(&ok);
        }
        if (!ok) {
            return std::nullopt;
        }

        if (std::abs(rawValue) > 100000.0) {
            return std::nullopt;
        }

        if (valueText.contains('.') || valueText.contains(',')) {
            return rawValue;
        }

        return rawValue / 100.0;
    }

    static bool inferStaticScanSettingsFromImageNames(const QFileInfoList& imageFiles,
                                                      StaticScanSettings& settings) {
        std::vector<double> positions;
        positions.reserve(imageFiles.size());
        for (const QFileInfo& fileInfo : imageFiles) {
            const auto position = scanFileBaseNameToFreePosition(fileInfo.completeBaseName());
            if (position.has_value()) {
                positions.push_back(position.value());
            }
        }
        if (positions.empty()) {
            return false;
        }

        std::sort(positions.begin(), positions.end());
        positions.erase(std::unique(positions.begin(), positions.end(), [](double lhs, double rhs) {
            return std::abs(lhs - rhs) < 1e-6;
        }), positions.end());

        settings.startPosition = static_cast<float>(positions.front());
        settings.endPosition = static_cast<float>(positions.back());
        settings.signedStep = positions.size() >= 2
            ? static_cast<float>(std::abs(positions[1] - positions[0]))
            : 1.0f;

        for (size_t i = 1; i < positions.size(); ++i) {
            const double step = std::abs(positions[i] - positions[i - 1]);
            if (step > 1e-6 && step < std::abs(settings.signedStep)) {
                settings.signedStep = static_cast<float>(step);
            }
        }
        if (settings.endPosition < settings.startPosition) {
            settings.signedStep = -std::abs(settings.signedStep);
        }
        return !qFuzzyCompare(settings.startPosition, settings.endPosition) && !qFuzzyIsNull(settings.signedStep);
    }

    static std::vector<float> buildStaticScanTargets(float startAngle, float endAngle, float signedStep) {
        std::vector<float> targets;
        if (qFuzzyCompare(startAngle, endAngle) || qFuzzyIsNull(signedStep)) {
            return targets;
        }

        const float epsilon = std::max(std::abs(signedStep) * 1e-4f, 1e-4f);
        for (float target = startAngle + signedStep;
             signedStep > 0.0f ? target <= endAngle + epsilon : target >= endAngle - epsilon;
             target += signedStep) {
            targets.push_back(signedStep > 0.0f ? std::min(target, endAngle) : std::max(target, endAngle));
        }

        if (targets.empty() || std::abs(targets.back() - endAngle) > epsilon) {
            targets.push_back(endAngle);
        } else {
            targets.back() = endAngle;
        }
        return targets;
    }

    static double offlineCenterlineOrientationScore(const std::vector<cv::Point2d>& points,
                                                    const cv::Size& imageSize,
                                                    bool isHorizontal) {
        if (points.empty() || imageSize.width <= 0 || imageSize.height <= 0) {
            return 0.0;
        }

        double minCoord = std::numeric_limits<double>::max();
        double maxCoord = std::numeric_limits<double>::lowest();
        for (const auto& point : points) {
            const double coord = isHorizontal ? point.x : point.y;
            minCoord = std::min(minCoord, coord);
            maxCoord = std::max(maxCoord, coord);
        }

        const double fullSpan = static_cast<double>(isHorizontal ? imageSize.width : imageSize.height);
        if (fullSpan <= 1.0 || maxCoord < minCoord) {
            return 0.0;
        }

        const double coverage = std::clamp((maxCoord - minCoord + 1.0) / fullSpan, 0.0, 1.0);
        return static_cast<double>(points.size()) * coverage;
    }

    static int centerlineMethodIndexFromName(const QString& method) {
        if (method == "Steger") {
            return 1;
        }
        if (method == "阈值法") {
            return 2;
        }
        return 0;
    }

    static QString readYamlString(const cv::FileNode& node, const QString& fallback = QString()) {
        if (node.empty()) {
            return fallback;
        }
        std::string value;
        node >> value;
        return QString::fromUtf8(value.c_str());
    }

    static double readYamlDouble(const cv::FileNode& node, double fallback = 0.0) {
        return node.empty() ? fallback : static_cast<double>(node);
    }

    static int readYamlInt(const cv::FileNode& node, int fallback = 0) {
        return node.empty() ? fallback : static_cast<int>(node);
    }

    static QString findStaticSettingsFile(const QDir& dir) {
        const QString preferred = dir.filePath("scan_parameters.yml");
        if (QFileInfo::exists(preferred)) {
            return preferred;
        }

        const QString legacy = dir.filePath("SETTINGS.YML");
        if (QFileInfo::exists(legacy)) {
            return legacy;
        }

        const QFileInfoList ymlFiles = dir.entryInfoList({"*.yml", "*.yaml"}, QDir::Files | QDir::Readable, QDir::Name);
        return ymlFiles.isEmpty() ? QString() : ymlFiles.first().absoluteFilePath();
    }

    static QString resolveStaticCalibrationPath(const QString& configuredPath, const QDir& scanDir) {
        QFileInfo configured(configuredPath);
        if (configured.exists() && configured.isFile()) {
            return configured.absoluteFilePath();
        }

        if (configured.isRelative() && !configuredPath.isEmpty()) {
            QFileInfo relativeToScan(scanDir.filePath(configuredPath));
            if (relativeToScan.exists() && relativeToScan.isFile()) {
                return relativeToScan.absoluteFilePath();
            }

            QFileInfo basenameInScan(scanDir.filePath(configured.fileName()));
            if (basenameInScan.exists() && basenameInScan.isFile()) {
                return basenameInScan.absoluteFilePath();
            }
        }

        return resolveCalibrationPath(configuredPath);
    }

    static bool readStaticScanSettings(const QString& settingsPath, const QDir& scanDir, StaticScanSettings& settings, QString& errorMessage) {
        cv::FileStorage storage(settingsPath.toStdString(), cv::FileStorage::READ);
        if (!storage.isOpened()) {
            errorMessage = QString("无法读取参数文件：%1").arg(settingsPath);
            return false;
        }

        cv::FileNode root = storage["ScanSettings"];
        if (root.empty()) {
            errorMessage = QString("参数文件缺少 ScanSettings 节点：%1").arg(settingsPath);
            return false;
        }

        settings.settingsPath = settingsPath;
        settings.centerlineMethod = readYamlString(root["CenterlineMethod"], "GrayScale灰度中心法");
        settings.centerlineMethodIndex = readYamlInt(root["CenterlineMethodIndex"], centerlineMethodIndexFromName(settings.centerlineMethod));
        settings.bwThreshold = readYamlDouble(root["BwThreshold"], settings.bwThreshold);
        settings.sumThreshold = readYamlDouble(root["SumThreshold"], settings.sumThreshold);
        settings.isHorizontal = readYamlInt(root["IsHorizontal"], settings.isHorizontal ? 1 : 0) != 0;
        settings.calibrationFile = resolveStaticCalibrationPath(readYamlString(root["CalibrationFile"]), scanDir);
        settings.startPosition = static_cast<float>(readYamlDouble(root["ScanStartPosition"], 0.0));
        settings.endPosition = static_cast<float>(readYamlDouble(root["ScanEndPosition"], 0.0));
        const double signedStep = readYamlDouble(root["SignedScanStep"], 0.0);
        const double scanStep = readYamlDouble(root["ScanStep"], 0.0);
        if (std::abs(signedStep) > 1e-9) {
            settings.signedStep = static_cast<float>(signedStep);
        } else if (std::abs(scanStep) > 1e-9) {
            settings.signedStep = static_cast<float>(settings.endPosition >= settings.startPosition ? std::abs(scanStep) : -std::abs(scanStep));
        }
        settings.scanImageFormat = normalizedImageFormat(readYamlString(root["ScanImageFormat"], settings.scanImageFormat));

        if (settings.calibrationFile.isEmpty()) {
            errorMessage = QString("参数文件未包含有效标定文件：%1").arg(settingsPath);
            return false;
        }
        if (qFuzzyCompare(settings.startPosition, settings.endPosition) || qFuzzyIsNull(settings.signedStep)) {
            errorMessage = QString("参数文件中的扫描起点、终点或步长无效：%1").arg(settingsPath);
            return false;
        }
        if ((settings.endPosition - settings.startPosition) * settings.signedStep < 0.0f) {
            errorMessage = QString("参数文件中的扫描方向与步长符号不一致：%1").arg(settingsPath);
            return false;
        }

        return true;
    }

    UWScanMainWindow::UWScanMainWindow(QWidget *parent): SARibbonMainWindow(parent){
        this->setWindowTitle("UWScan");
        this->setWindowIcon(QIcon(":/icon/Wuhan_Fuel.ico.png"));
        this->setRibbonTheme(SARibbonTheme::RibbonThemeOffice2013);

        ribbon = ribbonBar();
        ribbon->setTitleBarHeight(40);
        ribbon->setTabBarHeight(40);

        // ribbon->setWindowTitleBackgroundBrush(QColor(0, 83, 157));
        // ribbon->setWindowTitleTextColor(QColor(255, 255, 255));
        ribbon->setRibbonStyle(SARibbonBar::RibbonStyleCompactThreeRow);
        
        auto* mainSplitter = new QSplitter(Qt::Horizontal, this);
        mainSplitter->setObjectName("mainSplitter");
        mainSplitter->setChildrenCollapsible(true);
        mainSplitter->setHandleWidth(6);
        mainSplitter->setMinimumSize(0, 0);
        mainSplitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setCentralWidget(mainSplitter);

        sideTabWidget = new QTabWidget(this);
        sideTabWidget->setObjectName("sideTabWidget");
        sideTabWidget->setMinimumSize(0, 0);
        sideTabWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

        centerTabWidget = new QTabWidget(this);
        centerTabWidget->setObjectName("centerTabWidget");
        centerTabWidget->setMinimumSize(0, 0);
        centerTabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        mainSplitter->addWidget(sideTabWidget);
        mainSplitter->addWidget(centerTabWidget);
        mainSplitter->setStretchFactor(0, 0);
        mainSplitter->setStretchFactor(1, 1);
        mainSplitter->setCollapsible(0, true);
        mainSplitter->setCollapsible(1, true);
        mainSplitter->setSizes({320, 1200});
        setting::SettingsManager * settingManager = setting::SettingsManager::getInstance();

#pragma region 状态栏
        //将状态条设置为自定义状态条
        statusBar = new AdvancedStatusBar(this);
        statusBar->setObjectName("statusBar");
        this->setStatusBar(statusBar);

        statusBar->showMessage("系统初始化完成...", 2000);
#pragma endregion 状态栏

#pragma region 点云树与点云显示窗口
        treeWidget = new pointcloud::CloudTreeWidget(this);
        treeWidget->setObjectName("treeWidget");
        sideTabWidget->addTab(treeWidget, tr("点云"));

        cloudWidget = new pointcloud::CloudWindow(treeWidget, this);
        cloudWidget->setObjectName("cloudWidget");
        centerTabWidget->addTab(cloudWidget, tr("点云"));
        
        connect(treeWidget, &pointcloud::CloudTreeWidget::refreshViewRequested, this, &UWScanMainWindow::refreshAllClouds);
        // connect(cloudWidget, &pointcloud::CloudWindow::selectionFinished, this, &UWScanMainWindow::onSelectionFinished);
        connect(treeWidget, &pointcloud::CloudTreeWidget::focusOnCloudRequest, cloudWidget, &pointcloud::CloudWindow::onFocusCloud);
        connect(treeWidget, &pointcloud::CloudTreeWidget::saveErrorDataRequested, this, [this](const QString& cloudId) {
            const auto cachedStats = g_cloud_fit_error_stats.value(cloudId);
            if (cachedStats.isEmpty()) {
                QMessageBox::warning(this, tr("警告"), tr("当前点云没有可保存的误差数据"));
                return;
            }

            QString defaultName = "fit_errors";
            for (const auto& cloud : treeWidget->getAllClouds()) {
                if (cloud && QString::fromStdString(cloud->getId()) == cloudId && !cloud->getName().trimmed().isEmpty()) {
                    defaultName = cloud->getName();
                    break;
                }
            }

            saveFitErrorsWithDialog(this, defaultName, cachedStats);
        });
#pragma endregion 点云树与点云显示窗口
#pragma region 文件Tab
        /*******文件Tab*******/
        fileCategory = ribbon->addCategoryPage(tr("文件"));
        fileCategory->setObjectName("fileCategory");
        filePanel = fileCategory->addPanel(tr("文件"));
        filePanel->setObjectName("filePanel");

        newAction = new QAction(tr("新建"), this);
        newAction->setIcon(QIcon(":/icon/开始-新建.png"));
        newAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_N));
        newAction->setObjectName("newAction");
        filePanel->addLargeAction(newAction);
        connect(newAction, &QAction::triggered, this, &UWScanMainWindow::onNewActionTriggered);

        openAction = new QAction(QIcon(":/icon/开始-打开.png"), tr("打开"), this);
        openAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_O));
        openAction->setObjectName("openAction");
        filePanel->addLargeAction(openAction);
        connect(openAction, &QAction::triggered, this, &UWScanMainWindow::onOpenActionTriggered);
        
        saveMenu = new QMenu(tr("保存"), this);
        saveMenu->setIcon(QIcon(":/icon/开始-保存.png"));
        saveMenu->setObjectName("saveMenu");
        filePanel->addLargeMenu(saveMenu);

        savePointcloudAction = new QAction(QIcon(":/icon/开始-保存.png"), tr("保存点云"), this);
        savePointcloudAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
        savePointcloudAction->setObjectName("savePointcloudAction");
        saveMenu->addAction(savePointcloudAction);
        connect(savePointcloudAction, &QAction::triggered, this, &UWScanMainWindow::onSavePointcloudActionTriggered);

        QAction* saveFitErrorAction = new QAction(QIcon(":/icon/开始-保存.png"), tr("保存误差"), this);
        saveFitErrorAction->setObjectName("saveFitErrorAction");
        saveMenu->addAction(saveFitErrorAction);
        connect(saveFitErrorAction, &QAction::triggered, this, [this]() {
            auto currentCloud = treeWidget->getCurrentCloud();
            if (!currentCloud) {
                QMessageBox::warning(this, tr("警告"), tr("请先选中误差点云"));
                return;
            }
            const QString cloudId = QString::fromStdString(currentCloud->getId());
            if (!g_cloud_fit_error_stats.contains(cloudId) || g_cloud_fit_error_stats.value(cloudId).isEmpty()) {
                QMessageBox::warning(this, tr("警告"), tr("当前点云没有可保存的误差数据"));
                return;
            }
            const QString defaultName = currentCloud->getName().isEmpty() ? QString("fit_errors") : currentCloud->getName();
            saveFitErrorsWithDialog(this, defaultName, g_cloud_fit_error_stats.value(cloudId));
            return;
            const QString csvPath = QFileDialog::getSaveFileName(
                this,
                tr("保存误差分布"),
                QDir::homePath() + "/" + defaultName + ".csv",
                tr("CSV文件(*.csv)")
            );
            if (csvPath.isEmpty()) {
                return;
            }
            std::vector<FitStats> statsList;
            const auto cachedStats = g_cloud_fit_error_stats.value(cloudId);
            statsList.reserve(cachedStats.size());
            for (const auto& stats : cachedStats) {
                statsList.push_back(stats);
            }
            writeFitErrorsCsv(csvPath, statsList);
        });

        saveImageAction = new QAction(QIcon(":/icon/开始-保存.png"), tr("保存图像"), this);
        saveImageAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_I));
        saveImageAction->setObjectName("saveImageAction");
        saveMenu->addAction(saveImageAction);
        connect(saveImageAction, &QAction::triggered, this, &UWScanMainWindow::onSaveImageActionTriggered);

        saveDepthAction = new QAction(QIcon(":/icon/开始-保存.png"), tr("保存深度"), this);
        saveDepthAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_D));
        saveDepthAction->setObjectName("saveDepthAction");
        saveMenu->addAction(saveDepthAction);
        connect(saveDepthAction, &QAction::triggered, this, &UWScanMainWindow::onSaveDepthActionTriggered);
        
        saveCenterLineAction = new QAction(QIcon(":/icon/开始-保存.png"), tr("保存中心线"), this);
        saveCenterLineAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_L));
        saveCenterLineAction->setObjectName("saveCenterLineAction");
        saveMenu->addAction(saveCenterLineAction);
        connect(saveCenterLineAction, &QAction::triggered, this, &UWScanMainWindow::onSaveCenterLineActionTriggered);
        
#pragma endregion 文件Tab   

#pragma region 扫描Tab
        scannerWidget = new scanner::ScannerInterface(this);
        scannerWidget->setObjectName("scannerWidget");
        scannerWidget->setEnabled(true);
        scannerWidget->setDefaultValues();
        sideTabWidget->addTab(scannerWidget, tr("扫描"));

        imageWidget = new ImageWidget(this);
        imageWidget->setObjectName("imageWidget");
        centerTabWidget->addTab(imageWidget, tr("图像"));

        centerLineWidget = new ImageWidget(this);
        centerLineWidget->setObjectName("centerLineWidget");
        centerTabWidget->addTab(centerLineWidget, tr("强度图"));

        depthWidget = new ImageWidget(this);
        depthWidget->setObjectName("depthWidget");
        depthWidget->setRoiSelectionEnabled(false);
        centerTabWidget->addTab(depthWidget, tr("深度图"));
        connect(depthWidget, &ImageWidget::rightClicked, this, [this](const QPoint&) {
            depthPreviewAxis = (depthPreviewAxis + 1) % 3;
            depthAccumImage.release();
            updateDepthPreview();
            static const char* axisNames[] = {"X", "Y", "Z"};
            statusBar->showMessage(tr("深度图切换到 %1 轴").arg(axisNames[depthPreviewAxis]), 1200);
        });

        compositeWidget = new ImageWidget(this);
        compositeWidget->setObjectName("compositeWidget");
        compositeWidget->setRoiSelectionEnabled(false);
        centerTabWidget->addTab(compositeWidget, tr("合成图"));

        scanner = new scanner::Scanner(this);
        scanner->setObjectName("scanner");

        reconstructor = new scanner::Reconstructor(this);
        reconstructor->setObjectName("reconstructor");
        scanner->setReconstructor(reconstructor);
        imageThread = new scanner::ImageThread(this);
        imageThread->setObjectName("imageThread");
        imageThread->start();

        scanner::ReconstructionConfig temp_rc;
        connect(reconstructor, &scanner::Reconstructor::loadCalibration, this, [this](const QString& path){
            statusBar->addLogMessage("Info", "加载标定文件为" + path);
        });
        populateReconstructionConfig(temp_rc, settingManager);
        if (!reconstructor->init(temp_rc)) {
            statusBar->addLogMessage("Error", QString("标定文件加载失败：%1；原因：%2")
                .arg(QString::fromStdString(temp_rc.calibrationPath))
                .arg(QString::fromStdString(reconstructor->getLastError())));
        }
        reconstructor->startReconstruction();
        

        //捕捉到一帧图像时，更新图像显示
        connect(scanner, &scanner::Scanner::frameCaptured, [this](scanner::ReconstructionFramePtr frame){
            latestReconstructionImage = frame->image.clone();
            const bool needsPreviewCenterline = scannerWidget &&
                (scannerWidget->centerPointDisplayEnabled() ||
                 scannerWidget->validAreaDisplayEnabled() ||
                 scannerWidget->lineFitEnabled());
            if (needsPreviewCenterline) {
                linePoints = reconstructor->extractPreviewCenterLine(latestReconstructionImage);
                if (scannerWidget->lineFitEnabled()) {
                    const LineFitResult fit = fitLineAndErrors(linePoints);
                    scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), fit.me, fit.rmse);
                } else {
                    scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
                }
            }
            refreshDisplayedImage();
            QMetaObject::invokeMethod(commandServer, "sendImage", Qt::QueuedConnection, Q_ARG(cv::Mat, frame->image));
        });
        connect(scannerWidget, &scanner::ScannerInterface::centerlineDisplayOptionsChanged, this, [this]() {
            scanner::ReconstructionConfig config = reconstructor->getConfig();
            config.centerlineMethod = scannerWidget->centerlineMethodIndex();
            config.bwThr = scannerWidget->extractionThreshold();
            config.sumThr = scannerWidget->extractionFilterThreshold();
            config.thres = static_cast<int>(config.bwThr);
            config.slt_thres = static_cast<int>(config.sumThr);
            config.isHeng = scannerWidget->horizontalExtractionEnabled();
            if (!reconstructor->init(config)) {
                statusBar->addLogMessage("Error", QString("中心线参数更新失败：%1").arg(QString::fromStdString(reconstructor->getLastError())));
                return;
            }
            const bool showOverlay = scannerWidget->centerPointDisplayEnabled() ||
                scannerWidget->validAreaDisplayEnabled() ||
                scannerWidget->lineFitEnabled();
            if (!showOverlay) {
                linePoints.clear();
                scannerWidget->updateCenterlineStats(0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
                refreshDisplayedImage();
                return;
            }
            if (!latestReconstructionImage.empty()) {
                linePoints = reconstructor->extractPreviewCenterLine(latestReconstructionImage);
                if (scannerWidget->lineFitEnabled()) {
                    const LineFitResult fit = fitLineAndErrors(linePoints);
                    scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), fit.me, fit.rmse);
                } else {
                    scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
                }
            }
            refreshDisplayedImage();
        });
        connect(scannerWidget, &scanner::ScannerInterface::calibrationFileChanged, this, [this](const QString& path) {
            if (path.trimmed().isEmpty()) {
                return;
            }
            auto* settingsManager = setting::SettingsManager::getInstance();
            settingsManager->setString(setting::DEFAULT_CALIBRATE_FILE, path);
            settingsManager->sync();
            scanner::ReconstructionConfig config = reconstructor->getConfig();
            config.calibrationPath = resolveCalibrationPath(path).toStdString();
            if (!reconstructor->init(config)) {
                statusBar->addLogMessage("Error", QString("标定文件加载失败：%1；原因：%2")
                    .arg(path)
                    .arg(QString::fromStdString(reconstructor->getLastError())));
                return;
            }
            statusBar->addLogMessage("Info", tr("标定文件已切换：%1").arg(path));
        });
        connect(treeWidget, &pointcloud::CloudTreeWidget::modifyStandardDiameterRequested,
                this, &UWScanMainWindow::updateFitCloudStandardDiameter);

        connect(scannerWidget, &scanner::ScannerInterface::changeExposureTime, scanner, &scanner::Scanner::setExposureTime);
        connect(scannerWidget, &scanner::ScannerInterface::changeGain, scanner, &scanner::Scanner::setGain);
        connect(scannerWidget, &scanner::ScannerInterface::changeLaserPosition, scanner, &scanner::Scanner::moveMotor);
        connect(scannerWidget, &scanner::ScannerInterface::startScan, scanner, &scanner::Scanner::startScan);
        connect(treeWidget, &QTreeWidget::itemSelectionChanged, this, [this]() {
            auto allClouds = treeWidget->getAllClouds();
            auto currentCloud = treeWidget->getCurrentCloud();
            for (const auto& cloud : allClouds) {
                if (!cloud) {
                    continue;
                }
                cloud->setHighlighted(cloud == currentCloud);
            }
            refreshAllClouds();
            if (!currentCloud) {
                depthPreviewPoints.clear();
                depthImage.release();
                cloudWidget->clearSelectedCloudCoordinates();
                cloudWidget->clearErrorLegend();
                return;
            }
            currentCloud->getPoints(depthPreviewPoints);
            const QString cloudId = QString::fromStdString(currentCloud->getId());
            if (cloudInfoMessages.contains(cloudId)) {
                cloudWidget->setSelectedCloudMessage(cloudInfoMessages.value(cloudId));
            } else {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                currentCloud->getCentroid(x, y, z);
                cloudWidget->setSelectedCloudCoordinates(currentCloud->getName(), x, y, z);
            }
            if (cloudLegendInfos.contains(cloudId)) {
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(cloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            } else {
                cloudWidget->clearErrorLegend();
            }
            if (!suppressDepthPreviewFromCloudSelection) {
                offlineDepthAssociatedPoints.clear();
                offlineDepthImageSize = cv::Size();
                updateDepthPreview();
            }
        });
        connect(treeWidget, &pointcloud::CloudTreeWidget::cloudDeleted, this, [this](const QString& cloudId) {
            cloudInfoMessages.remove(cloudId);
            cloudLegendInfos.remove(cloudId);
            g_cloud_fit_error_stats.remove(cloudId);
            g_fit_cloud_contexts.remove(cloudId);
        });
        connect(scannerWidget, &scanner::ScannerInterface::roiChanged, this, [this](int x, int y, int width, int height, bool fullRoi) {
            scanner::ReconstructionConfig config = reconstructor->getConfig();
            config.isFullRoi = fullRoi;
            config.Roi = std::vector<int>{x, y, width, height};
            if (!reconstructor->init(config)) {
                statusBar->addLogMessage("Error", QString("ROI更新失败：%1").arg(QString::fromStdString(reconstructor->getLastError())));
            }
            imageWidget->cropToRoi(x, y, width, height);
        });
        connect(imageWidget, &ImageWidget::roiSelected, scannerWidget, [this](int x, int y, int width, int height) {
            scannerWidget->setRoiValues(x, y, width, height);
            scanner::ReconstructionConfig config = reconstructor->getConfig();
            config.isFullRoi = false;
            config.Roi = std::vector<int>{x, y, width, height};
            if (!reconstructor->init(config)) {
                statusBar->addLogMessage("Error", QString("ROI更新失败：%1").arg(QString::fromStdString(reconstructor->getLastError())));
            }
        });
        connect(scannerWidget, &scanner::ScannerInterface::infoOccurred, this, [this](const QString& info){
            statusBar->addLogMessage("Info", info);
        });
        connect(scannerWidget, &scanner::ScannerInterface::warnOccurred, this, [this](const QString& warning){
            statusBar->addLogMessage("Warning", warning);
        });
        connect(scannerWidget, &scanner::ScannerInterface::errorOccurred, this, [this](const QString& error){
            statusBar->addLogMessage("Error", error);
        });
        connect(scannerWidget, &scanner::ScannerInterface::startScan, this, [this](){
            this->stopScanAction->setEnabled(true);
            this->previewAction->setEnabled(false);
            statusBar->setProgress(0);
        });

        connect(scannerWidget, &scanner::ScannerInterface::stopScan, this, &UWScanMainWindow::onStopScanActionTriggered);
        connect(scannerWidget, &scanner::ScannerInterface::reconstructCurrent, scanner, &scanner::Scanner::reconstructCurrent);
        // connect(scannerWidget, &scanner::ScannerInterface::changeLaserPosition, scanner, &scanner::Scanner::moveMotor);

        connect(scanner, &scanner::Scanner::frameScanned, reconstructor, &scanner::Reconstructor::reconstructFrame);
        connect(scanner, &scanner::Scanner::frameScanned, imageThread, &scanner::ImageThread::enqueueFrame);
        connect(scanner, &scanner::Scanner::sendInfo, this, [this](scanner::ScannerInfo info){
            // std::cout << "[Debug] UWScanMainWindow::onReceiveInfo() angle: " << angle << " tempareture: " << tempareture << " humity: " << humity << " pressure: " << pressure << std::endl;
            statusBar->updateParameter("当前位置", info.angle);
            statusBar->updateParameter("温度", info.temperature);
            statusBar->updateParameter("湿度", info.humidity);
            statusBar->updateParameter("压力", info.pressure);
            statusBar->updateAttitude(
                info.roll,
                info.pitch,
                info.yaw,
                (info.attitudeFlag & 0x01) != 0);
            scannerWidget->updateCurrentPosition(info.angle);
            info.angle_max = scanner->getCurrentLimitUpper();
            info.angle_min = scanner->getCurrentLimitLower();
            QMetaObject::invokeMethod(commandServer, "sendScannerInfo", Qt::QueuedConnection, Q_ARG(scanner::ScannerInfo, info));
        });

        connect(scanner, &scanner::Scanner::sendPorgress, this, [this](int progress){
            statusBar->setProgress(progress);
            if(progress == 100){
                statusBar->addLogMessage("Info", "扫描完成");
                this->stopScanAction->setEnabled(false);
                this->previewAction->setEnabled(true);
                scannerWidget->updateSavingPath();
            }
        });

        connect(scanner, &scanner::Scanner::scanStopped, this, [this](){
            this->stopScanAction->setEnabled(false);
            this->previewAction->setEnabled(true);
            scannerWidget->updateSavingPath();
        });

        connect(scannerWidget, &scanner::ScannerInterface::setBrightness, scanner, &scanner::Scanner::setBrightness);

        connect(scanner, &scanner::Scanner::sendInformation, this, [this](const QString& info){
            statusBar->addLogMessage("Info", info);
            return;
            const bool isDisconnectMessage = info.contains(QStringLiteral("断开")) ||
                                             info.contains(QStringLiteral("未连接或TCP不可写")) ||
                                             info.contains(QStringLiteral("Device Removed"));
            if (scannerConnectionEstablished && isDisconnectMessage && !scannerDisconnectWarningShown) {
                scannerDisconnectWarningShown = true;
                scannerConnectionEstablished = false;
                scannerWidget->setEnabled(true);
                connAction->setIcon(QIcon(":/icon/checkbox_unchecked.png"));
                previewAction->setEnabled(false);
                stopPreviewAction->setEnabled(false);
                scanAction->setEnabled(true);
                stopScanAction->setEnabled(false);
                staticReconstructionAction->setEnabled(true);
                QMessageBox::warning(this, tr("设备连接提示"), tr("扫描设备连接已断开，请检查相机或网口连接后重新连接。"));
            }
        });

        #if 0
        connect(scanner, &scanner::Scanner::deviceDisconnected, this, [this](const QString& reason) {
            if (!scannerConnectionEstablished || scannerDisconnectWarningShown) {
                return;
            }
            scannerDisconnectWarningShown = true;
            scannerConnectionEstablished = false;
            scannerWidget->setEnabled(false);
            connAction->setIcon(QIcon(":/icon/checkbox_unchecked.png"));
            previewAction->setEnabled(false);
            stopPreviewAction->setEnabled(false);
            scanAction->setEnabled(false);
            stopScanAction->setEnabled(false);
            staticReconstructionAction->setEnabled(true);
            const QString detail = reason.isEmpty()
                ? tr("扫描设备连接已断开，请检查相机或电机连接后重新连接。")
                : tr("扫描设备连接已断开：%1\n请检查相机或电机连接后重新连接。").arg(reason);
            QMessageBox::warning(this, tr("设备连接提示"), detail);
        });

        #endif
        connect(scannerWidget, &scanner::ScannerInterface::changeLimit, scanner, &scanner::Scanner::setLimit);

        connect(scannerWidget, &scanner::ScannerInterface::openLaser, scanner, &scanner::Scanner::openLaser);

        connect(scannerWidget, &scanner::ScannerInterface::closeLaser, scanner, &scanner::Scanner::closeLaser);

        connect(reconstructor, &scanner::Reconstructor::pointCloudReady, this, [this](pointcloud::PointCloudPtr pc, bool isSingleScan){
            // std::cout << ">>> [Debug] UWScanMainWindow::onPointCloudReady() pc->size(): " << pc->getPointCount() << std::endl;
            if(!pc){
                statusBar->addLogMessage("Error", "该次扫描未捕获到任何激光点");
                return;
            }

            bool shouldResetCamera = isSingleScan;
            if(isSingleScan){
                treeWidget->addCloud(pc);
            } else {
                QString fullname = pc->getName();
                int pos = fullname.lastIndexOf('.');
                treeWidget->appendCloud(pc, fullname.left(pos));
                shouldResetCamera = true;
            }

            refreshAllClouds();
            if (shouldResetCamera) {
                auto focusCloud = treeWidget->getCurrentCloud();
                if (!focusCloud) {
                    focusCloud = pc;
                }
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                focusCloud->getCentroid(x, y, z);
                cloudWidget->onFocusCloudOrthographic(x, y, z);
            }
            cloudWidget->refreshView();
            auto currentCloud = treeWidget->getCurrentCloud();
            if (currentCloud) {
                currentCloud->getPoints(depthPreviewPoints);
                updateDepthPreview();
            }
            statusBar->showMessage(QString("点云实时更新：%1").arg(pc->getName()), 1000);
            QMetaObject::invokeMethod(commandServer, "sendPointCloud", Qt::QueuedConnection, Q_ARG(pointcloud::PointCloudPtr, pc));
        });

        connect(reconstructor, &scanner::Reconstructor::reconstructionFailed, this, [this](int scanIndex, const QString& reason, bool){
            statusBar->addLogMessage("Warning", QString("重建跳过：重建索引 %1，原因：%2").arg(scanIndex).arg(reason));
        });

        connect(scanner, &scanner::Scanner::frameScanned, this, [this](scanner::ReconstructionFramePtr frame){
            latestReconstructionImage = frame->image.clone();
            statusBar->addLogMessage("Info", QString("提交重建帧，激光位置：%1，重建索引：%2")
                .arg(frame->laserPosition, 0, 'f', 2)
                .arg(frame->scanIndex));
        });

        connect(reconstructor, &scanner::Reconstructor::linePointsReady, this, [this](const std::vector<cv::Point2d>& linePoints, bool isSingleScan){
            Q_UNUSED(isSingleScan);
            if (scannerWidget->lineFitEnabled()) {
                const LineFitResult fit = fitLineAndErrors(linePoints);
                scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), fit.me, fit.rmse);
            } else {
                scannerWidget->updateCenterlineStats(static_cast<int>(linePoints.size()), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
            }
            this->linePoints = linePoints;
            updateCenterIntensityPreview();
            refreshDisplayedImage();
        });

        connect(scanner::AssociatedPointsManager::getInstance(), &scanner::AssociatedPointsManager::compositeImageSaved, this, [this](const QString&, const QString& filePath) {
            cv::Mat image = readImageFile(filePath, cv::IMREAD_GRAYSCALE);
            if (!image.empty()) {
                compositeWidget->setImage(image);
                statusBar->addLogMessage("Info", tr("合成图已保存：%1").arg(filePath));
            }
        });

        connect(scanner::AssociatedPointsManager::getInstance(), &scanner::AssociatedPointsManager::intensityImageSaved, this, [this](const QString&, const QString& filePath) {
            cv::Mat image = readImageFile(filePath, cv::IMREAD_COLOR);
            if (!image.empty()) {
                centerLineWidget->setImage(image);
                statusBar->addLogMessage("Info", tr("强度图已保存：%1").arg(filePath));
            }
        });

        connect(scanner::AssociatedPointsManager::getInstance(), &scanner::AssociatedPointsManager::sessionProgress, this, [this](const QString&, int processedFrames, int totalFrames) {
            if (totalFrames <= 0) {
                return;
            }
            const int progress = std::clamp(static_cast<int>(std::lround(processedFrames * 100.0 / totalFrames)), 0, 100);
            statusBar->setProgress(progress);
            statusBar->showMessage(tr("离线重建进度：%1/%2").arg(processedFrames).arg(totalFrames), 0);
        });

        connect(scanner::AssociatedPointsManager::getInstance(), &scanner::AssociatedPointsManager::sessionCompleted, this, [this](const QString& sessionId, int savedFrames) {
            statusBar->setProgress(100);
            statusBar->showMessage(tr("离线重建完成，共处理 %1 帧").arg(savedFrames), 5000);
            statusBar->addLogMessage("Info", tr("离线重建完成，共处理 %1 帧").arg(savedFrames));
            if (!offlineReconstructionSessionId.isEmpty() && offlineReconstructionSessionId == sessionId) {
                QMessageBox::information(this,
                                         tr("提示"),
                                         tr("离线重建完成，共处理 %1 张图片，点云已保存为 reconstructed.txt").arg(savedFrames));
                offlineReconstructionSessionId.clear();
            }
        });

        connect(scanner::AssociatedPointsManager::getInstance(), &scanner::AssociatedPointsManager::sessionCancelled, this, [this](const QString& sessionId, int savedFrames) {
            statusBar->addLogMessage("Warning", tr("离线重建提前结束，已处理 %1 帧").arg(savedFrames));
            if (!offlineReconstructionSessionId.isEmpty() && offlineReconstructionSessionId == sessionId) {
                offlineReconstructionSessionId.clear();
            }
        });

        scanCategory = ribbon->addCategoryPage(tr("扫描"));
        scanCategory->setObjectName("scanCategory");
        connPanel = scanCategory->addPanel(tr("通信"));
        connPanel->setObjectName("connPanel");
        scanPanel = scanCategory->addPanel(tr("扫描"));
        scanPanel->setObjectName("scanPanel");

        connAction = new QAction(QIcon(":/icon/checkbox_unchecked.png"), tr("连接"), this);
        connAction->setObjectName("connAction");
        connPanel->addLargeAction(connAction);
        connect(connAction, &QAction::triggered, this, &UWScanMainWindow::onConnActionTriggered);

        previewAction = new QAction(QIcon(":/icon/视频.png"), tr("视频预览"), this);
        previewAction->setObjectName("previewAction");
        scanPanel->addSmallAction(previewAction);
        connect(previewAction, &QAction::triggered, this, &UWScanMainWindow::onPreviewActionTriggered);

        stopPreviewAction = new QAction(QIcon(":/icon/停止(1).png"), tr("停止预览"), this);
        stopPreviewAction->setObjectName("stopPreviewAction");
        scanPanel->addSmallAction(stopPreviewAction);
        connect(stopPreviewAction, &QAction::triggered, this, &UWScanMainWindow::onStopPreviewActionTriggered);

        scanAction = new QAction(QIcon(":/icon/扫描测量.png"), tr("扫描"), this);
        scanAction->setObjectName("scanAction");
        scanPanel->addSmallAction(scanAction);
        connect(scanAction, &QAction::triggered, this, &UWScanMainWindow::onScanActionTriggered);

        stopScanAction = new QAction(QIcon(":/icon/停止(1).png"), tr("停止扫描"), this);
        stopScanAction->setObjectName("stopScanAction");
        scanPanel->addSmallAction(stopScanAction);
        connect(stopScanAction, &QAction::triggered, this, &UWScanMainWindow::onStopScanActionTriggered);

        staticReconstructionAction = new QAction(QIcon(":/icon/静态重建.png"), tr("离线重建"), this);
        staticReconstructionAction->setObjectName("staticReconstructionAction");
        scanPanel->addLargeAction(staticReconstructionAction);
        connect(staticReconstructionAction, &QAction::triggered, this, &UWScanMainWindow::onStaticReconstructionActionTriggered);

        previewAction->setEnabled(false);
        stopPreviewAction->setEnabled(false);
        scanAction->setEnabled(true);
        stopScanAction->setEnabled(false);
        staticReconstructionAction->setEnabled(true);
#pragma endregion

#pragma region 点云菜单Tab
        /*** 点云菜单Tab ***/
        pointcloudCategory = ribbon->addCategoryPage(tr("点云"));
        pointcloudCategory->setObjectName("pointcloudCategory");

        selectPanel = pointcloudCategory->addPanel(tr("选择"));
        selectPanel->setObjectName("selectPanel");

        guideInAction = new QAction(QIcon(":/icon/开始-导入.png"), tr("导入"), this);
        guideInAction->setObjectName("guideInAction");
        selectPanel->addLargeAction(guideInAction);
        connect(guideInAction, &QAction::triggered, this, &UWScanMainWindow::onGuideInActionTriggered);

        selectAllAction = new QAction(QIcon(":/icon/全选-全选.png"), tr("全选"), this);
        selectAllAction->setObjectName("selectAllAction");
        selectPanel->addLargeAction(selectAllAction);
        connect(selectAllAction, &QAction::triggered, this, &UWScanMainWindow::onSelectAllActionTriggered);

        selectNoneAction = new QAction(QIcon(":/icon/取消选择.png"), tr("取消选择"), this);
        selectNoneAction->setObjectName("selectNoneAction");
        selectPanel->addLargeAction(selectNoneAction);
        connect(selectNoneAction, &QAction::triggered, this, &UWScanMainWindow::onSelectNoneActionTriggered);

        reverseAction = new QAction(QIcon(":/icon/反选.png"), tr("反选"), this);
        reverseAction->setObjectName("reverseAction");
        selectPanel->addLargeAction(reverseAction);
        connect(reverseAction, &QAction::triggered, this, &UWScanMainWindow::onReverseActionTriggered);

        deleteAction = new QAction(QIcon(":/icon/删除(1).png"), tr("删除"), this);
        deleteAction->setObjectName("deleteAction");
        selectPanel->addLargeAction(deleteAction);
        connect(deleteAction, &QAction::triggered, this, &UWScanMainWindow::onDeleteActionTriggered);

        rotateCenterAction = new QAction(QIcon(":/icon/陀螺仪.png"), tr("旋转中心"), this);
        rotateCenterAction->setObjectName("rotateCenterAction");
        selectPanel->addLargeAction(rotateCenterAction);
        connect(rotateCenterAction, &QAction::triggered, this, &UWScanMainWindow::onRotateCenterActionTriggered);

        selectToolMenu = new QMenu(tr("选择工具"), this);
        selectToolMenu->setIcon(QIcon(":/icon/选取测量位置.png"));
        selectToolMenu->setObjectName("selectToolMenu");
        selectToolMenu->setIcon(QIcon(":/icon/开始-选择.png"));
        selectPanel->addLargeMenu(selectToolMenu);

        rectToolAction = new QAction(QIcon(":/icon/UI_icon2_矩形框选2.png"), tr("矩形选择"), this);
        rectToolAction->setObjectName("rectToolAction");
        rectToolAction->setIcon(QIcon(":/icon/UI_icon2_矩形框选2.png"));
        selectToolMenu->addAction(rectToolAction);
        connect(rectToolAction, &QAction::triggered, this->cloudWidget, &pointcloud::CloudWindow::onRectToolActionTriggered);
        connect(rectToolAction, &QAction::triggered, this, [this](){
            selectToolMenu->setIcon(rectToolAction->icon());
        });

        lassoToolAction = new QAction(QIcon(":/icon/套索.png"), tr("lasso选择"), this);
        lassoToolAction->setObjectName("lassoToolAction");
        selectToolMenu->addAction(lassoToolAction);
        connect(lassoToolAction, &QAction::triggered, this->cloudWidget, &pointcloud::CloudWindow::onLassoToolActionTriggered);
        connect(lassoToolAction, &QAction::triggered, this, [this](){
            selectToolMenu->setIcon(lassoToolAction->icon());
        });

        polyToolAction = new QAction(QIcon(":/icon/点击选多边形.png"), tr("多边形选择"), this);
        polyToolAction->setObjectName("polyToolAction");
        selectToolMenu->addAction(polyToolAction);
        connect(polyToolAction, &QAction::triggered, this->cloudWidget, &pointcloud::CloudWindow::onPolyToolActionTriggered);
        connect(polyToolAction, &QAction::triggered, this, [this](){
            selectToolMenu->setIcon(polyToolAction->icon());
        });

        ellipseToolAction = new QAction(QIcon(":/icon/UI_icon2_鐭╁舰妗嗛€?.png"), tr("椭圆选择"), this);
        ellipseToolAction->setObjectName("ellipseToolAction");
        ellipseToolAction->setIcon(QIcon(":/icon/圆形选框.png"));
        selectToolMenu->addAction(ellipseToolAction);
        connect(ellipseToolAction, &QAction::triggered, this, [this](){
            cloudWidget->setInteractionMode(static_cast<pointcloud::InteractionMode>(4));
            selectToolMenu->setIcon(ellipseToolAction->icon());
        });

        filterPanel = pointcloudCategory->addPanel(tr("滤波"));
        filterPanel->setObjectName("filterPanel");

        VoxelFilterAction = new QAction(QIcon(":/icon/点云数据处理--均匀精简.png"), tr("体素滤波"), this);
        VoxelFilterAction->setObjectName("VoxelFilterAction");
        filterPanel->addLargeAction(VoxelFilterAction);
        connect(VoxelFilterAction, &QAction::triggered, this, &UWScanMainWindow::onVoxelFilterActionTriggered);

        sorFilterAction = new QAction(QIcon(":/icon/点云数据处理-随机精简.png"), tr("SOR滤波"), this);
        sorFilterAction->setObjectName("sorFilterAction");
        filterPanel->addLargeAction(sorFilterAction);
        connect(sorFilterAction, &QAction::triggered, this, &UWScanMainWindow::onSorFilterActionTriggered);

        passFilterAction = new QAction(QIcon(":/icon/点云数据处理-曲率精简.png"), tr("直通滤波"), this);
        passFilterAction->setObjectName("passFilterAction");
        filterPanel->addLargeAction(passFilterAction);
        connect(passFilterAction, &QAction::triggered, this, &UWScanMainWindow::onPassFilterActionTriggered);

        featurePanel = pointcloudCategory->addPanel(tr("特征尺寸"));
        featurePanel->setObjectName("featurePanel");
        defectPanel = pointcloudCategory->addPanel(tr("缺陷测量"));
        defectPanel->setObjectName("defectPanel");

        cylinderAction = new QAction(QIcon(":/icon/对象属性Object Attributes (14).png"), tr("拟合圆柱"), this);
        cylinderAction->setObjectName("cylinderAction");
        featurePanel->addLargeAction(cylinderAction);
        connect(cylinderAction, &QAction::triggered, this, &UWScanMainWindow::onCylinderFeatureActionTriggered);

        cylinderErrorAction = new QAction(QIcon(":/icon/统计计算(1).png"), tr("圆柱面误差分析"), this);
        cylinderErrorAction->setObjectName("cylinderErrorAction");
        cylinderErrorAction->setText(tr("误差分析"));
        featurePanel->addLargeAction(cylinderErrorAction);
        connect(cylinderErrorAction, &QAction::triggered, this, &UWScanMainWindow::onCylinderErrorAnalysisActionTriggered);

        sphereAction = new QAction(QIcon(":/icon/balls.png"), tr("拟合球"), this);
        sphereAction->setObjectName("sphereAction");
        featurePanel->addLargeAction(sphereAction);
        connect(sphereAction, &QAction::triggered, this, &UWScanMainWindow::onSphereFeatureActionTriggered);

        planeDefectAction = new QAction(QIcon(":/icon/error.png"), tr("平面缺陷测量"), this);
        planeDefectAction->setObjectName("planeDefectAction");
        defectPanel->addLargeAction(planeDefectAction);
        connect(planeDefectAction, SIGNAL(triggered()), this, SLOT(onPlaneDefectMeasurementActionTriggered()));

        cylinderDefectAction = new QAction(QIcon(":/icon/error.png"), tr("圆柱缺陷测量"), this);
        cylinderDefectAction->setObjectName("cylinderDefectAction");
        defectPanel->addLargeAction(cylinderDefectAction);
        connect(cylinderDefectAction, SIGNAL(triggered()), this, SLOT(onCylinderDefectMeasurementActionTriggered()));
        ballAction = new QAction(QIcon(":/icon/balls.png"), tr("拟合双球"), this);
        planeAction = new QAction(QIcon(":/icon/瀵硅薄灞炴€bject Attributes (14).png"), tr("骞抽潰鎷熷悎"), this);
        planeAction->setObjectName("planeAction");
        planeAction->setIcon(QIcon(":/icon/planes.png"));
        planeAction->setText(tr("平面拟合"));
        featurePanel->addLargeAction(planeAction);
        connect(planeAction, &QAction::triggered, this, &UWScanMainWindow::onPlaneFeatureActionTriggered);

        ballAction->setObjectName("ballAction");
        featurePanel->addLargeAction(ballAction);
        connect(ballAction, &QAction::triggered, this, &UWScanMainWindow::onDoubleSphereFeatureActionTriggered);

#pragma region 系统设置Tab
        /*** 系统设置Tab ***/
        settingCategory = ribbon->addCategoryPage(tr("设置"));
        settingCategory->setObjectName("settingCategory");
        settingPanel = settingCategory->addPanel(tr("设置"));
        settingPanel->setObjectName("settingPanel");

        settingAction = new QAction(QIcon(":/icon/设置.png"), tr("设置"), this);
        settingAction->setObjectName("settingAction");
        settingPanel->addLargeAction(settingAction);
        connect(settingAction, &QAction::triggered, this, &UWScanMainWindow::onSettingActionTriggered);
#pragma endregion

#pragma region 帮助Tab
        /*** 帮助Tab ***/
        helpCategory = ribbon->addCategoryPage(tr("帮助"));
        helpCategory->setObjectName("helpCategory");
        helpPanel = helpCategory->addPanel(tr("帮助"));
        helpPanel->setObjectName("helpPanel");

        helpAction = new QAction(QIcon(":/icon/帮助(1).png"), tr("帮助"), this);
        helpAction->setObjectName("helpAction");
        helpPanel->addAction(helpAction);
        connect(helpAction, &QAction::triggered, this, &UWScanMainWindow::onHelpActionTriggered);
#pragma endregion 帮助Tab
#pragma region 服务器
        //服务器线程的装配
        serverThread = new QThread(this);
        commandServer = new server::RemoteCommandServer(nullptr);
        commandServer->moveToThread(serverThread);

        connect(serverThread, &QThread::finished, commandServer, &server::RemoteCommandServer::deleteLater);
        serverThread->start();
        connect(serverThread, &QThread::finished, serverThread, &QThread::deleteLater);
        connect(commandServer,
                &server::RemoteCommandServer::logMessage,
                this,
                [this](const QString &type, const QString &message) {
                    if (statusBar) {
                        statusBar->addLogMessage(type, message);
                    }
                });
        connect(commandServer, &server::RemoteCommandServer::requestStartScan, this, [this](float startAngle, float endAngle, float step, float speed, double exposureTime){
            qDebug() << "requestStartScan: " << startAngle << " " << endAngle << " " << step << " " << speed << " exposure: " << exposureTime;
            if(scanner->isConnected()){
                if(qFuzzyCompare(startAngle, endAngle) || step <= 0 || speed <= 0){
                    QMetaObject::invokeMethod(commandServer, "onStartScanResult", Qt::QueuedConnection,
                                              Q_ARG(bool, false),
                                              Q_ARG(QString, "开始角度不能等于结束角度，步长和速度必须大于0"));
                    return;
                }
                const float signedStep = startAngle < endAngle ? std::abs(step) : -std::abs(step);
                const QString baseDir = setting::SettingsManager::getInstance()->getString(setting::SAVE_IMAGE_DIR, QDir::homePath());
                const QString savePath = baseDir + "/" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
                const double effectiveExposure = exposureTime > 0.0 ? exposureTime : scannerWidget->currentScanExposureTime();
                scanner->startScan(startAngle,
                                   endAngle,
                                   signedStep,
                                   speed,
                                   savePath,
                                   scannerWidget->currentSceneExposureTime(),
                                   effectiveExposure,
                                   true,
                                   true,
                                   true,
                                   true,
                                   true,
                                   true,
                                   "png",
                                   "bmp",
                                   "png",
                                   "png",
                                   false,
                                   0.0,
                                   0.0,
                                   0.0);
            } else {
                QMetaObject::invokeMethod(commandServer, "onStartScanResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestGoToAngle, this, [this](float angle){
            if(scanner->isConnected()){
                scanner->moveMotor(1, angle);
                QMetaObject::invokeMethod(commandServer, "onGoToAngleResult", Qt::QueuedConnection,
                                          Q_ARG(bool, true),
                                          Q_ARG(QString, "转到角度" + QString::number(angle)));
            } else {
                QMetaObject::invokeMethod(commandServer, "onGoToAngleResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        connect(scanner, &scanner::Scanner::scanBegin, this, [this](){
            QMetaObject::invokeMethod(commandServer, "onStartScanResult", Qt::QueuedConnection,
                                      Q_ARG(bool, true), Q_ARG(QString, "扫描开始"));
            auto ret = imageWidget->getImage();
            
            //生成和ret一样大小的黑色图像
            centerLineImage = cv::Mat(ret.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            centerIntensityAccumImage.release();
            depthAccumImage.release();
            centerLineWidget->setImage(centerLineImage);

            linePoints.clear();
        });

        connect(commandServer, &server::RemoteCommandServer::requestStopScan, this, [this](){
            if(scanner->isConnected()){
                scanner->stopScan();
                QMetaObject::invokeMethod(commandServer, "onStopScanResult", Qt::QueuedConnection,
                                          Q_ARG(bool, true), Q_ARG(QString, "扫描停止"));
            } else {
                QMetaObject::invokeMethod(commandServer, "onStopScanResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestOpenScanner, this, [this](uint32_t){
            if(scanner->isConnected()){
                QMetaObject::invokeMethod(commandServer, "onScannerOpenedResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备已连接"));
            } else {
                onConnActionTriggered();
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestCloseScanner, this, [this](uint32_t){
            if(!scanner->isConnected()){
                QMetaObject::invokeMethod(commandServer, "onScannerClosedResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
                return;
            }
            onConnActionTriggered();
        });

        connect(commandServer, &server::RemoteCommandServer::requestOpenLaser, this, [this](){
            if(!scanner->isConnected()){
                QMetaObject::invokeMethod(commandServer, "onOpenLaserResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
                return;
            }
            if (!scanner->openLaser()) {
                QMetaObject::invokeMethod(commandServer, "onOpenLaserResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "激光器打开指令发送失败"));
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestCloseLaser, this, [this](){
            if(!scanner->isConnected()){
                QMetaObject::invokeMethod(commandServer, "onCloseLaserResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
                return;
            }
            if (!scanner->closeLaser()) {
                QMetaObject::invokeMethod(commandServer, "onCloseLaserResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "激光器关闭指令发送失败"));
            }
        });

        connect(scanner, &scanner::Scanner::laserOpened, this, [this](){
            QMetaObject::invokeMethod(commandServer, "onOpenLaserResult", Qt::QueuedConnection,
                                      Q_ARG(bool, true), Q_ARG(QString, "激光器打开"));
        });

        connect(scanner, &scanner::Scanner::laserClosed, this, [this](){
            QMetaObject::invokeMethod(commandServer, "onCloseLaserResult", Qt::QueuedConnection,
                                      Q_ARG(bool, true), Q_ARG(QString, "激光器关闭"));
        });

        connect(commandServer, &server::RemoteCommandServer::requestChangeParameter, this,
                [this](const QJsonObject &params, uint32_t){
            // 处理参数变化
            if(!scanner->isConnected()){
                QMetaObject::invokeMethod(commandServer, "onParameterChangedResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
                return;
            }
            if(params.contains("exposure_time")) scanner->setExposureTime(params["exposure_time"].toDouble());
            else if(params.contains("exposureTime")) scanner->setExposureTime(params["exposureTime"].toDouble());
            if(params.contains("gain"))scanner->setGain(params["gain"].toDouble());

            if(params.contains("isOpen")){
                onConnActionTriggered();
            }

            if(params.contains("current_angle")){
                scanner->moveMotor(1, params["current_angle"].toDouble());
            }
            if (params.contains("trigger_mode") || params.contains("triggerMode")
                || params.contains("trigger_source") || params.contains("triggerSource")) {
                const QJsonValue modeValue = params.contains("trigger_mode")
                    ? params.value("trigger_mode") : params.value("triggerMode");
                const QJsonValue sourceValue = params.contains("trigger_source")
                    ? params.value("trigger_source") : params.value("triggerSource");
                bool enabled = modeValue.isBool()
                    ? modeValue.toBool()
                    : modeValue.toString().compare("ON", Qt::CaseInsensitive) == 0;
                QString source = sourceValue.toString().trimmed();
                if (source.compare("SW", Qt::CaseInsensitive) == 0) source = "Software";
                else if (source.compare("L1", Qt::CaseInsensitive) == 0) source = "Line1";
                else if (source.compare("L2", Qt::CaseInsensitive) == 0) source = "Line2";
                else if (source.compare("L3", Qt::CaseInsensitive) == 0) source = "Line3";
                scanner->setTriggerConfig(enabled, source);
            }
            QMetaObject::invokeMethod(commandServer, "onParameterChangedResult", Qt::QueuedConnection,
                                      Q_ARG(bool, true), Q_ARG(QString, "参数设置成功"));
        });

        connect(commandServer, &server::RemoteCommandServer::requestSetLimit, this, [this](float minAngle, float maxAngle){
            if(scanner->isConnected()){
                if(minAngle >= maxAngle){
                    QMetaObject::invokeMethod(commandServer, "onSetLimitResult", Qt::QueuedConnection,
                                              Q_ARG(bool, false), Q_ARG(QString, "最小角度不能大于最大角度"));
                    return;
                }
                scanner->setLimit(minAngle, maxAngle);
                QMetaObject::invokeMethod(commandServer, "onSetLimitResult", Qt::QueuedConnection,
                                          Q_ARG(bool, true), Q_ARG(QString, "扫描范围限制设置成功"));
            } else {
                QMetaObject::invokeMethod(commandServer, "onSetLimitResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestSetExposureTime, this, [this](float exposureTime){
            if(scanner->isConnected()){
                scanner->setExposureTime(exposureTime);
                QMetaObject::invokeMethod(commandServer, "onSetExposureTimeResult", Qt::QueuedConnection,
                                          Q_ARG(bool, true), Q_ARG(QString, "曝光时间设置成功"));
            } else {
                QMetaObject::invokeMethod(commandServer, "onSetExposureTimeResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        connect(commandServer, &server::RemoteCommandServer::requestSaveImage, this, [this](const QString &filename){
            if(scanner->isConnected()){
                auto img = imageWidget->getImage();
                if(img.empty()){
                    QMetaObject::invokeMethod(commandServer, "onSaveImageResult", Qt::QueuedConnection,
                                              Q_ARG(bool, false), Q_ARG(QString, "没有图片可供保存"));
                    return;
                }
                bool ret = cv::imwrite(filename.toStdString(), img);
                QMetaObject::invokeMethod(commandServer, "onSaveImageResult", Qt::QueuedConnection,
                                          Q_ARG(bool, ret),
                                          Q_ARG(QString, ret ? "图像保存成功" : "图像保存失败"));
            } else {
                QMetaObject::invokeMethod(commandServer, "onSaveImageResult", Qt::QueuedConnection,
                                          Q_ARG(bool, false), Q_ARG(QString, "设备未连接"));
            }
        });

        serverPanel = scanCategory->addPanel(tr("服务器"));
        serverPanel->setObjectName("serverPanel");

        startServerAction = new QAction(QIcon(":/icon/通讯面板-启动服务器.png"), tr("启动服务器"), this);
        startServerAction->setObjectName("startServerAction");
        serverPanel->addLargeAction(startServerAction);
        connect(startServerAction, &QAction::triggered, this, &UWScanMainWindow::onStartServerActionTriggered);

#pragma endregion 服务器
        if(settingManager->getBool(setting::AUTO_CONNECT_TO_SCANNER)) onConnActionTriggered();
        lassoToolAction->trigger();
    }

    UWScanMainWindow::~UWScanMainWindow() noexcept {
        if(scannerWidget) scannerWidget->saveLastCloseParameters();
        if(scanner) scanner->disconnectDevices();
        if(serverDialog){
            serverDialog->close();
            serverDialog->deleteLater();
            serverDialog = nullptr;
        }
        if(serverThread){
            serverThread->quit();
            serverThread->wait();
        }
    }

    void UWScanMainWindow::onNewActionTriggered(){
        qDebug() << "New project";
    }

    void UWScanMainWindow::onOpenActionTriggered(){
        QStringList filePaths = QFileDialog::getOpenFileNames(
            this,
            tr("选择文件"),
            QDir::homePath(),
            tr("点云文件(*.ply *.pcd *.xyz *.las *.txt *.asc) ;; 图片文件(*.png *.jpg *.jpeg *.bmp)")
        );
        for (auto & filePath : filePaths) {
            if (filePath.isEmpty()) {
                return;
            }

            if (filePath.endsWith(".ply") || filePath.endsWith(".pcd") || filePath.endsWith(".xyz") || filePath.endsWith(".las") || filePath.endsWith(".txt") || filePath.endsWith(".asc")) {
                auto cloud = pointcloud::createFromFile(filePath);
                if (cloud && !cloud->empty()) {
                    treeWidget->addCloud(cloud);
                    refreshAllClouds();
                    float x = 0.0f;
                    float y = 0.0f;
                    float z = 0.0f;
                    cloud->getCentroid(x, y, z);
                    cloudWidget->onFocusCloudOrthographic(x, y, z);
                    centerTabWidget->setCurrentIndex(0);
                }
            } else if (filePath.endsWith(".png") || filePath.endsWith(".jpg") || filePath.endsWith(".jpeg") || filePath.endsWith(".bmp")) {
                cv::Mat mat = cv::imread(filePath.toStdString());
                imageWidget->setMat(mat);
                centerTabWidget->setCurrentIndex(1);
            }
        }
    }

    void UWScanMainWindow::onSavePointcloudActionTriggered(){
        auto selectedClouds = treeWidget->getSelectedClouds();
        if (selectedClouds.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("请先选择要保存的点云"));
            return;
        }
        for (auto & cloud : selectedClouds) {
            auto cloudName = cloud->getName();
            QString savePath = QFileDialog::getSaveFileName(
                this,
                tr("保存点云"),
                QDir::homePath() + "/" + cloudName + ".txt",
                tr("点云文本文件(*.txt)")
            );
            if (savePath.isEmpty()) {
                return;
            }
            if (!savePath.endsWith(".txt", Qt::CaseInsensitive)) {
                savePath += ".txt";
            }
            cloud->saveToFile(savePath);
        }
        QMessageBox::information(this, tr("提示"), tr("点云保存完成"));
    }
    void UWScanMainWindow::onSaveImageActionTriggered(){
        auto img = imageWidget->getImage();
        if(img.empty()){
            QMessageBox::warning(this, tr("警告"), tr("没有图片可供保存"));
            return;
        }
        
        auto ret = QFileDialog::getSaveFileName(
            this,
            tr("保存图像"),
            QDir::homePath(),
            tr("图片文件(*.png *.jpg *.jpeg *.bmp)")
        );
        if(ret.isEmpty()) return;

        if(!cv::imwrite(ret.toStdString(), img)){
            QMessageBox::warning(this, tr("警告"), tr("图片保存失败"));
            return;
        } else QMessageBox::information(this, tr("提示"), tr("图片保存成功"));
    }

    void UWScanMainWindow::onSaveCenterLineActionTriggered(){
        if(linePoints.empty()){
            QMessageBox::warning(this, tr("警告"), tr("没有中心线可供保存，请先扫描"));
            return;
        }

        auto ret = QFileDialog::getSaveFileName(
            this,
            tr("保存中心线"),
            QDir::homePath(),
            tr("中心线文件(*.txt)")
        );
        if(ret.isEmpty()) return;
        std::ofstream file(ret.toStdString());
        if(!file.is_open()){
            QMessageBox::warning(this, tr("警告"), tr("文件打开失败"));
            return;
        }
        for(auto& point : linePoints){
            file << point.x << " " << point.y << std::endl;
        }
        file.close();
        QMessageBox::information(this, tr("提示"), tr("中心线保存成功"));
    }

    void UWScanMainWindow::onSaveDepthActionTriggered(){
        if(depthImage.empty()){
            QMessageBox::warning(this, tr("警告"), tr("没有深度图可供保存"));
            return;
        }
        auto ret = QFileDialog::getSaveFileName(
            this,
            tr("保存深度图"),
            QDir::homePath(),
            tr("图片文件(*.png *.bmp *.jpg)")
        );
        if(ret.isEmpty()) return;
        if(!cv::imwrite(ret.toStdString(), depthImage)){
            QMessageBox::warning(this, tr("警告"), tr("深度图保存失败"));
            return;
        }
        QMessageBox::information(this, tr("提示"), tr("深度图保存成功"));
    }
    void UWScanMainWindow::onConnActionTriggered(){
        if(scanner->isConnected()){
            scannerWidget->saveLastCloseParameters();
            scanner->disconnectDevices();
            scannerWidget->setEnabled(true);
            connAction->setIcon(QIcon(":/icon/checkbox_unchecked.png"));
            previewAction->setEnabled(false);
            stopPreviewAction->setEnabled(false);
            scanAction->setEnabled(true);
            stopScanAction->setEnabled(false);
            staticReconstructionAction->setEnabled(true);
            scannerDisconnectWarningShown = false;
            scannerConnectionEstablished = false;
            QMetaObject::invokeMethod(commandServer, "onScannerClosedResult", Qt::QueuedConnection, Q_ARG(bool, true), Q_ARG(QString, "Scanner closed successfully"));
            return;
        }

        bool ret = scanner->connectDevices();
        if (ret) {
            const auto exposureRange = scanner->getExposureTimeRange();
            const auto gainRange = scanner->getGainRange();
            scannerWidget->setCameraParameterRanges(exposureRange.first, exposureRange.second, gainRange.first, gainRange.second);
            scannerWidget->setEnabled(true);
            connAction->setIcon(QIcon(":/icon/checkbox_checked.png"));
            previewAction->setEnabled(true);
            stopPreviewAction->setEnabled(false);
            scanAction->setEnabled(true);
            stopScanAction->setEnabled(false);
            staticReconstructionAction->setEnabled(true);
            scannerDisconnectWarningShown = false;
            scannerConnectionEstablished = scanner->isConnected();
        } else {
            scannerWidget->setEnabled(true);
            connAction->setIcon(QIcon(":/icon/checkbox_unchecked.png"));
            previewAction->setEnabled(false);
            stopPreviewAction->setEnabled(false);
            scanAction->setEnabled(true);
            stopScanAction->setEnabled(false);
            staticReconstructionAction->setEnabled(true);
            scannerDisconnectWarningShown = false;
            scannerConnectionEstablished = false;
            QMetaObject::invokeMethod(commandServer, "onScannerOpenedResult", Qt::QueuedConnection, Q_ARG(bool, false), Q_ARG(QString, "Failed to connect scanner"));
            return;
        }
        stopPreviewAction->setEnabled(false);
        stopScanAction->setEnabled(false);
        previewAction->setEnabled(true);
        
        scanner->continuousGrab();
        this->centerTabWidget->setCurrentIndex(1);

        setting::SettingsManager * instance = setting::SettingsManager::getInstance();
        if(instance->getBool(setting::AUTO_OPEN_LASER)){
            scanner->openLaser();
            QTimer::singleShot(1000, this, [=](){
                scanner->openLaser();
            });
        }

        

        auto scanner_opening_behavior = instance->getString(setting::SCANNER_OPENING_BEHAVIOUR);
        if(scanner_opening_behavior == setting::SCANNER_OPENING_BEHAVIOUR_DEFAULT){
            // 默认参数（原来的逻辑，可以继续保留）
            double exposureTime = instance->getDouble(setting::DEFAULT_EXPOSURETIME);
            scanner->setExposureTime(exposureTime);
            // 补充：设置默认限位值
            double upper = instance->getDouble(setting::DEFAULT_MOTOR_LIMIT_UPPER, 57.0);
            double lower = instance->getDouble(setting::DEFAULT_MOTOR_LIMIT_LOWER, 42.0);
            scanner->setLimit(lower, upper);
            scannerWidget->setLimitValues(lower, upper);   // 需要在 ScannerInterface 中实现
            // 其他默认参数 ...
        }
        else if(scanner_opening_behavior == setting::SCANNER_OPENING_BEHAVIOUR_LAST_CLOSE){
            // 与上次关闭时一致
            double exposureTime = instance->getDouble(setting::LAST_CLOSE_EXPOSURETIME);
            scanner->setExposureTime(exposureTime);
            double upper = instance->getDouble(setting::LAST_CLOSE_MOTOR_LIMIT_UPPER, 57.0);
            double lower = instance->getDouble(setting::LAST_CLOSE_MOTOR_LIMIT_LOWER, 42.0);
            scanner->setLimit(lower, upper);
            scannerWidget->setLimitValues(lower, upper);
            // 恢复其他上次关闭时的参数（增益、开始/结束角度等）...
        }
        QMetaObject::invokeMethod(commandServer, "onScannerOpenedResult", Qt::QueuedConnection, Q_ARG(bool, true), Q_ARG(QString, "Scanner opened successfully"));
        onScanActionTriggered();
    }

    void UWScanMainWindow::onPreviewActionTriggered(){
        if(scanner->continuousGrab()){
            previewAction->setEnabled(false);
            stopPreviewAction->setEnabled(true);
            this->centerTabWidget->setCurrentIndex(1);
        } else {
            previewAction->setEnabled(true);
            stopPreviewAction->setEnabled(false);
        }
    }

    void UWScanMainWindow::onScanActionTriggered(){
        if (sideTabWidget) {
            sideTabWidget->setCurrentWidget(scannerWidget);
        }
        if (scanner->isConnected()) {
            double exposureTime = scanner->getExposureTime();
            scannerWidget->rectifyExposureTime(exposureTime);
            scannerWidget->setDefaultValues();
        } else {
            scannerWidget->updateSavingPath();
        }
    }

    void UWScanMainWindow::onStopScanActionTriggered(){
        scanner->stopScan();
        onPreviewActionTriggered();
    }

    void UWScanMainWindow::onStaticReconstructionActionTriggered(){
        const QString folderPath = QFileDialog::getExistingDirectory(
            this,
            tr("选择离线重建图片文件夹"),
            setting::SettingsManager::getInstance()->getString(setting::SAVE_IMAGE_DIR, QDir::homePath()),
            QFileDialog::ShowDirsOnly
        );
        if(folderPath.isEmpty()){
            return;
        }

        QDir dir(folderPath);
        auto* settingsManager = setting::SettingsManager::getInstance();
        StaticScanSettings scanSettings;
        scanSettings.centerlineMethod = settingsManager->getString(setting::DEFAULT_CENTERLINE_METHOD, QStringLiteral("GrayScale灰度重心法"));
        scanSettings.centerlineMethodIndex = centerlineMethodIndexFromName(scanSettings.centerlineMethod);
        scanSettings.bwThreshold = settingsManager->getDouble(setting::DEFAULT_BW_THR, 55.0);
        scanSettings.sumThreshold = settingsManager->getDouble(setting::DEFAULT_SUM_THR, 124.0);
        scanSettings.isHorizontal = settingsManager->getBool(setting::DEFAULT_IS_HENG, true);
        QString configuredCalibration = settingsManager->getString(setting::DEFAULT_CALIBRATE_FILE);
        if (configuredCalibration.trimmed().isEmpty()) {
            configuredCalibration = QString::fromStdString(reconstructor->getConfig().calibrationPath);
        }
        scanSettings.calibrationFile = resolveStaticCalibrationPath(configuredCalibration, dir);
        if (scanSettings.calibrationFile.trimmed().isEmpty() || !QFileInfo::exists(scanSettings.calibrationFile)) {
            QMessageBox::warning(this,
                                 tr("警告"),
                                 tr("当前没有可用标定文件，请先在设置中选择正确标定文件。"));
            return;
        }

        scanner::ReconstructionConfig temp_rc;
        temp_rc.calibrationPath = scanSettings.calibrationFile.toStdString();
        temp_rc.centerlineMethod = scanSettings.centerlineMethodIndex;
        temp_rc.bwThr = scanSettings.bwThreshold;
        temp_rc.sumThr = scanSettings.sumThreshold;
        temp_rc.thres = static_cast<int>(temp_rc.bwThr);
        temp_rc.slt_thres = static_cast<int>(temp_rc.sumThr);
        temp_rc.isHeng = scanSettings.isHorizontal;
        temp_rc.keepLocalCoordinates = false;
        const scanner::ReconstructionConfig currentConfig = reconstructor->getConfig();
        temp_rc.isFullRoi = currentConfig.isFullRoi;
        temp_rc.Roi = currentConfig.Roi;

        QStringList imageFilters = {
            "*.bmp", "*.png", "*.jpg", "*.jpeg",
            "*.BMP", "*.PNG", "*.JPG", "*.JPEG"
        };
        imageFilters.removeDuplicates();
        QFileInfoList imageFiles = dir.entryInfoList(imageFilters, QDir::Files | QDir::Readable, QDir::Name);
        if(imageFiles.isEmpty()){
            QMessageBox::warning(this, tr("警告"), tr("所选文件夹中未找到可用于重建的图片"));
            return;
        }

        QCollator collator;
        collator.setNumericMode(true);
        std::sort(imageFiles.begin(), imageFiles.end(), [&collator](const QFileInfo& lhs, const QFileInfo& rhs) {
            return collator.compare(lhs.completeBaseName(), rhs.completeBaseName()) < 0;
        });

        std::vector<std::pair<QFileInfo, double>> framesToSubmit;
        for (const QFileInfo& fileInfo : imageFiles) {
            const std::optional<double> laserPosition = scanFileBaseNameToFreePosition(fileInfo.completeBaseName());
            if (laserPosition.has_value()) {
                framesToSubmit.emplace_back(fileInfo, laserPosition.value());
            }
        }
        std::sort(framesToSubmit.begin(), framesToSubmit.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
        });

        if (framesToSubmit.empty()) {
            QMessageBox::warning(this,
                                 tr("警告"),
                                 tr("无法从图片文件名解析扫描角度，请使用“角度x100”方式命名，例如 0.1 对应 10。"));
            return;
        }

        scanSettings.startPosition = static_cast<float>(framesToSubmit.front().second);
        scanSettings.endPosition = static_cast<float>(framesToSubmit.back().second);
        scanSettings.signedStep = 1.0f;
        if (framesToSubmit.size() >= 2) {
            double minStep = std::numeric_limits<double>::max();
            for (size_t i = 1; i < framesToSubmit.size(); ++i) {
                const double step = std::abs(framesToSubmit[i].second - framesToSubmit[i - 1].second);
                if (step > 1e-6) {
                    minStep = std::min(minStep, step);
                }
            }
            if (minStep < std::numeric_limits<double>::max()) {
                scanSettings.signedStep = static_cast<float>(minStep);
            }
        }
        statusBar->addLogMessage("Info",
                                 tr("离线重建按文件名角度提交 %1 张图片，角度范围：%2 到 %3")
                                     .arg(framesToSubmit.size())
                                     .arg(framesToSubmit.front().second, 0, 'f', 2)
                                     .arg(framesToSubmit.back().second, 0, 'f', 2));

        cv::Mat previewImage = readImageFile(framesToSubmit.front().first.absoluteFilePath(), cv::IMREAD_COLOR);
        auto tryOfflineOrientation = [this, &temp_rc](bool isHorizontal, const cv::Mat& image, double* score = nullptr) -> bool {
            scanner::ReconstructionConfig testConfig = temp_rc;
            testConfig.isHeng = isHorizontal;
            if (!reconstructor->init(testConfig)) {
                return false;
            }
            if (image.empty()) {
                if (score) {
                    *score = 0.0;
                }
                return true;
            }
            const std::vector<cv::Point2d> points = reconstructor->extractPreviewCenterLine(image);
            if (score) {
                *score = offlineCenterlineOrientationScore(points, image.size(), isHorizontal);
            }
            return true;
        };

        if (!previewImage.empty()) {
            double horizontalScore = 0.0;
            double verticalScore = 0.0;
            const bool horizontalOk = tryOfflineOrientation(true, previewImage, &horizontalScore);
            const bool verticalOk = tryOfflineOrientation(false, previewImage, &verticalScore);
            if (!horizontalOk && !verticalOk) {
                QMessageBox::warning(this, tr("警告"), tr("标定文件加载失败：%1\n%2")
                    .arg(QString::fromStdString(temp_rc.calibrationPath))
                    .arg(QString::fromStdString(reconstructor->getLastError())));
                return;
            }

            const bool useHorizontal = !verticalOk || (horizontalOk && horizontalScore >= verticalScore);
            temp_rc.isHeng = useHorizontal;
            if (!reconstructor->init(temp_rc)) {
                QMessageBox::warning(this, tr("警告"), tr("标定文件加载失败：%1\n%2")
                    .arg(QString::fromStdString(temp_rc.calibrationPath))
                    .arg(QString::fromStdString(reconstructor->getLastError())));
                return;
            }
            statusBar->addLogMessage("Info",
                                     tr("离线重建中心线方向：%1（水平评分 %2，竖直评分 %3），点云/深度图保持局部方向")
                                         .arg(useHorizontal ? tr("横向") : tr("纵向"))
                                         .arg(horizontalScore, 0, 'f', 2)
                                         .arg(verticalScore, 0, 'f', 2));
        } else {
            if (!reconstructor->init(temp_rc)) {
                QMessageBox::warning(this, tr("警告"), tr("标定文件加载失败：%1\n%2")
                    .arg(QString::fromStdString(temp_rc.calibrationPath))
                    .arg(QString::fromStdString(reconstructor->getLastError())));
                return;
            }
        }

        if (!previewImage.empty()) {
            imageWidget->setMat(previewImage);
            latestReconstructionImage = previewImage.clone();
            centerLineImage = cv::Mat(previewImage.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            centerIntensityAccumImage.release();
            depthAccumImage.release();
            offlineDepthAssociatedPoints.clear();
            offlineDepthImageSize = cv::Size();
            depthPreviewAxis = 0;
            centerLineWidget->setImage(centerLineImage);
            linePoints.clear();
            centerTabWidget->setCurrentIndex(1);
        }

        const QString sessionId = QString("static_%1").arg(QDateTime::currentMSecsSinceEpoch());
        offlineReconstructionSessionId = sessionId;
        statusBar->setProgress(0);
        statusBar->showMessage(tr("离线重建准备中..."), 0);
        scanner::AssociatedPointsManager::getInstance()->createSession(sessionId,
                                                                       folderPath,
                                                                       static_cast<int>(framesToSubmit.size()),
                                                                       true,
                                                                       true,
                                                                       true,
                                                                       "png",
                                                                       "png");

        int reconstructedCount = 0;
        int debugLoggedFrames = 0;
        std::vector<Eigen::Vector3f> offlineMergedPoints;
        std::vector<scanner::AssociatedPoints> offlineMergedAssociatedPoints;
        const QString offlineCloudName = QString("%1_离线重建点云_%2").arg(dir.dirName(), sessionId);
        pointcloud::PointCloudPtr offlineDisplayPointCloud;
        auto submitOfflineFrame = [this,
                                   &dir,
                                   &folderPath,
                                   &scanSettings,
                                   &reconstructedCount,
                                   &debugLoggedFrames,
                                   &offlineMergedPoints,
                                   &offlineMergedAssociatedPoints,
                                   &offlineCloudName,
                                   &offlineDisplayPointCloud](const QFileInfo& fileInfo, double laserPosition, const QString& sessionId) {
            cv::Mat image = readImageFile(fileInfo.absoluteFilePath(), cv::IMREAD_COLOR);
            if (image.empty()) {
                statusBar->addLogMessage("Warning", tr("离线图片读取失败：%1").arg(fileInfo.absoluteFilePath()));
                return false;
            }
            scanner::ReconstructionFramePtr frame(new scanner::ReconstructionFrame{
                scanPositionToFrameIndex(static_cast<float>(laserPosition)),
                laserPosition,
                static_cast<double>(scanSettings.startPosition),
                static_cast<double>(scanSettings.endPosition),
                true,
                false,
                false,
                true,
                false,
                image.clone(),
                folderPath,
                fileInfo.suffix().toLower(),
                QString("%1.%2").arg(dir.dirName()).arg(scanPositionToFrameIndex(static_cast<float>(laserPosition))),
                sessionId
            });

            if (debugLoggedFrames < 8) {
                statusBar->addLogMessage("Info",
                                         tr("离线帧映射：%1 -> laserPosition=%2, scanIndex=%3, scanStart=%4, scanEnd=%5")
                                             .arg(fileInfo.fileName())
                                             .arg(frame->laserPosition, 0, 'f', 4)
                                             .arg(frame->scanIndex)
                                             .arg(frame->scanStartPosition, 0, 'f', 4)
                                             .arg(frame->scanEndPosition, 0, 'f', 4));
                ++debugLoggedFrames;
            }

            latestReconstructionImage = frame->image.clone();
            std::vector<cv::Point2d> offlineLinePoints;
            scanner::AssociatedPointsPtr associatedPoints = std::make_shared<std::vector<scanner::AssociatedPoints>>();
            auto pointCloud = reconstructor->processFrame(frame, offlineLinePoints, associatedPoints);

            if (pointCloud.has_value()) {
                ++reconstructedCount;
                std::vector<Eigen::Vector3f> framePoints;
                pointCloud.value()->getPoints(framePoints);
                offlineMergedPoints.insert(offlineMergedPoints.end(), framePoints.begin(), framePoints.end());
                offlineMergedAssociatedPoints.insert(offlineMergedAssociatedPoints.end(),
                                                     associatedPoints->begin(),
                                                     associatedPoints->end());
                if (!offlineDisplayPointCloud) {
                    pointCloud.value()->setName(offlineCloudName);
                    offlineDisplayPointCloud = pointCloud.value();
                    suppressDepthPreviewFromCloudSelection = true;
                    treeWidget->addCloud(offlineDisplayPointCloud);
                    suppressDepthPreviewFromCloudSelection = false;
                } else {
                    offlineDisplayPointCloud->merge(pointCloud.value());
                }
                depthPreviewPoints = offlineMergedPoints;
                this->linePoints = offlineLinePoints;
                updateCenterIntensityPreview();
                refreshDisplayedImage();
                refreshAllClouds();
                cloudWidget->refreshView();
                depthImage = createDepthPreviewFromAssociatedPoints(latestReconstructionImage,
                                                                    offlineMergedAssociatedPoints,
                                                                    depthPreviewAxis,
                                                                    &depthAccumImage);
                if (!depthImage.empty()) {
                    depthWidget->setImage(depthImage);
                }

                scanner::AssociatedPointsManager::getInstance()->addAssociatedPoints(sessionId,
                                                                                     associatedPoints,
                                                                                     frame->image.size(),
                                                                                     frame->image);
            } else {
                scanner::AssociatedPointsManager::getInstance()->addAssociatedPoints(sessionId,
                                                                                     std::make_shared<std::vector<scanner::AssociatedPoints>>(),
                                                                                     frame->image.size(),
                                                                                     frame->image);
                statusBar->addLogMessage("Warning",
                                         tr("离线重建跳过：%1，位置 %2，原因：%3")
                                             .arg(fileInfo.fileName())
                                             .arg(laserPosition, 0, 'f', 2)
                                             .arg(QString::fromStdString(reconstructor->getLastError())));
            }
            return true;
        };

        int submittedCount = 0;
        for (const auto& frameInfo : framesToSubmit) {
            if (submitOfflineFrame(frameInfo.first, frameInfo.second, sessionId)) {
                ++submittedCount;
            }
            QCoreApplication::processEvents();
        }

        if (submittedCount == 0) {
            offlineReconstructionSessionId.clear();
            scanner::AssociatedPointsManager::getInstance()->cancelSession(sessionId);
            QMessageBox::warning(this, tr("警告"), tr("图片读取失败，未提交静态重建任务"));
            return;
        }

        if (!offlineMergedPoints.empty()) {
            depthPreviewPoints = offlineMergedPoints;
            offlineDepthAssociatedPoints = offlineMergedAssociatedPoints;
            offlineDepthImageSize = latestReconstructionImage.size();
            if (offlineDisplayPointCloud) {
                offlineDisplayPointCloud->setName(QString("%1_离线重建点云").arg(dir.dirName()));
            }
            depthImage = createDepthPreviewFromAssociatedPoints(
                latestReconstructionImage.empty() ? cv::Mat(offlineDepthImageSize, CV_8UC3, cv::Scalar(0, 0, 0)) : latestReconstructionImage,
                offlineDepthAssociatedPoints,
                depthPreviewAxis,
                &depthAccumImage);
            if (!depthImage.empty()) {
                depthWidget->setImage(depthImage);
            }
            refreshAllClouds();
            cloudWidget->refreshView();
        }

        statusBar->addLogMessage("Info", tr("离线重建使用当前重建参数与标定文件：%1")
            .arg(scanSettings.calibrationFile));
        statusBar->addLogMessage("Info", tr("离线重建角度范围：%1 到 %2，最小步长：%3")
            .arg(scanSettings.startPosition, 0, 'f', 2)
            .arg(scanSettings.endPosition, 0, 'f', 2)
            .arg(scanSettings.signedStep, 0, 'f', 2));
        statusBar->addLogMessage("Info", tr("离线重建任务已提交，共 %1 张图片，成功重建 %2 张").arg(submittedCount).arg(reconstructedCount));
    }

    void UWScanMainWindow::updateDepthPreview() {
        if (!offlineDepthAssociatedPoints.empty() && !offlineDepthImageSize.empty()) {
            cv::Mat source = latestReconstructionImage;
            if (source.empty() || source.size() != offlineDepthImageSize) {
                source = cv::Mat(offlineDepthImageSize, CV_8UC3, cv::Scalar(0, 0, 0));
            }
            depthImage = createDepthPreviewFromAssociatedPoints(source, offlineDepthAssociatedPoints, depthPreviewAxis, &depthAccumImage);
            if (!depthImage.empty()) {
                depthWidget->setImage(depthImage);
            }
            return;
        }
        if (!depthPreviewPoints.empty()) {
            depthImage = createDepthPreview(depthPreviewPoints, depthPreviewAxis);
            if (!depthImage.empty()) {
                depthWidget->setImage(depthImage);
            }
            return;
        }
    }

    void UWScanMainWindow::updateCenterIntensityPreview() {
        if (latestReconstructionImage.empty() || linePoints.empty()) {
            return;
        }

        cv::Mat gray;
        if (latestReconstructionImage.channels() == 1) {
            gray = latestReconstructionImage;
        } else {
            cv::cvtColor(latestReconstructionImage, gray, cv::COLOR_BGR2GRAY);
        }

        if (centerIntensityAccumImage.empty() || centerIntensityAccumImage.size() != gray.size()) {
            centerIntensityAccumImage = cv::Mat(gray.size(), CV_8UC1, cv::Scalar(0));
        }

        cv::Mat frameIntensity(gray.size(), CV_8UC1, cv::Scalar(0));
        for (const auto& point : linePoints) {
            const int x = static_cast<int>(std::lround(point.x));
            const int y = static_cast<int>(std::lround(point.y));
            if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) {
                continue;
            }

            const uchar value = gray.at<uchar>(y, x);
            uchar& dst = frameIntensity.at<uchar>(y, x);
            dst = std::max(dst, value);
        }

        cv::max(centerIntensityAccumImage, frameIntensity, centerIntensityAccumImage);

        double maxValue = 0.0;
        cv::minMaxLoc(centerIntensityAccumImage, nullptr, &maxValue);
        cv::Mat displayGray;
        if (maxValue > 0.0) {
            centerIntensityAccumImage.convertTo(displayGray, CV_8UC1, 255.0 / maxValue);
        } else {
            displayGray = centerIntensityAccumImage;
        }
        const int minVisibleIntensity = 64;
        cv::threshold(displayGray, displayGray, minVisibleIntensity, 255, cv::THRESH_TOZERO);

        cv::Mat displayColor(displayGray.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < displayGray.rows; ++y) {
            const uchar* row = displayGray.ptr<uchar>(y);
            for (int x = 0; x < displayGray.cols; ++x) {
                const uchar intensity = row[x];
                if (intensity == 0) {
                    continue;
                }
                const int green = std::clamp(120 + static_cast<int>(intensity) / 2, 0, 255);
                displayColor.at<cv::Vec3b>(y, x) = cv::Vec3b(0, green, 0);
            }
        }

        centerLineImage = displayColor;
        centerLineWidget->setImage(centerLineImage);
    }

    void UWScanMainWindow::refreshCenterlineOverlayPreview() {
        if (latestReconstructionImage.empty()) {
            return;
        }
        centerLineImage = latestReconstructionImage.clone();
        centerLineWidget->setImage(centerLineImage);
    }

    void UWScanMainWindow::refreshDisplayedImage() {
        if (latestReconstructionImage.empty()) {
            return;
        }

        const bool showOverlay = scannerWidget &&
            (scannerWidget->centerPointDisplayEnabled() ||
             scannerWidget->validAreaDisplayEnabled() ||
             scannerWidget->lineFitEnabled());

        if (!showOverlay || linePoints.empty()) {
            imageWidget->setMat(latestReconstructionImage);
            return;
        }

        const auto config = reconstructor->getConfig();
        const LineFitResult fit = fitLineAndErrors(linePoints);
        cv::Mat overlay = createCenterlineOverlay(latestReconstructionImage,
                                                  linePoints,
                                                  config,
                                                  scannerWidget->centerPointDisplayEnabled(),
                                                  scannerWidget->validAreaDisplayEnabled(),
                                                  scannerWidget->lineFitEnabled(),
                                                  fit);
        if (overlay.empty()) {
            imageWidget->setMat(latestReconstructionImage);
            return;
        }
        imageWidget->setMat(overlay);
    }

    double UWScanMainWindow::promptStandardDiameter(const QString& title,
                                                    const QString& label,
                                                    double minimumValue,
                                                    bool allowZero,
                                                    bool* ok,
                                                    double fallbackValue) const {
        return promptStandardDiameterValue(const_cast<UWScanMainWindow*>(this),
                                           title,
                                           label,
                                           minimumValue,
                                           allowZero,
                                           ok,
                                           fallbackValue);
    }

    void UWScanMainWindow::updateFitCloudStandardDiameter(const QString& cloudId) {
        if (!g_cloud_fit_error_stats.contains(cloudId) || g_cloud_fit_error_stats.value(cloudId).isEmpty()) {
            QMessageBox::warning(this, tr("警告"), tr("当前点云没有可修改的标准直径"));
            return;
        }

        const FitCloudContext context = g_fit_cloud_contexts.value(cloudId);
        if (!context.standardDiameterEditable) {
            QMessageBox::warning(this, tr("警告"), tr("当前点云不支持修改标准直径"));
            return;
        }

        std::shared_ptr<pointcloud::PointCloud> targetCloud;
        for (const auto& cloud : treeWidget->getAllClouds()) {
            if (cloud && QString::fromStdString(cloud->getId()) == cloudId) {
                targetCloud = cloud;
                break;
            }
        }
        if (!targetCloud) {
            QMessageBox::warning(this, tr("警告"), tr("未找到目标点云"));
            return;
        }

        bool ok = false;
        const double standardDiameter = promptStandardDiameter(tr("标准直径"),
                                                               tr("请输入标准直径："),
                                                               0.0,
                                                               false,
                                                               &ok,
                                                               context.standardDiameter > 0.0 ? context.standardDiameter : -1.0);
        if (!ok || standardDiameter <= 0.0) {
            return;
        }

        std::vector<Eigen::Vector3f> points;
        targetCloud->getPoints(points);
        if (points.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("点云为空，无法更新标准直径"));
            return;
        }

        QVector<FitStats> updatedStats;
        if (context.kind == FitFeatureKind::Cylinder) {
            updatedStats.push_back(cylinderStatsWithFixedDiameter(points, standardDiameter));
        } else if (context.kind == FitFeatureKind::Sphere) {
            FitStats fitted = fitSphereStats(points);
            updatedStats.push_back(sphereErrorStatsAgainstStandard(points, fitted, standardDiameter));
        } else {
            QMessageBox::warning(this, tr("警告"), tr("当前拟合类型不支持修改标准直径"));
            return;
        }

        if (updatedStats.isEmpty() || !updatedStats.first().ok) {
            QMessageBox::warning(this, tr("警告"), tr("重新计算标准直径失败"));
            return;
        }

        g_cloud_fit_error_stats.insert(cloudId, updatedStats);
        FitCloudContext updatedContext = context;
        updatedContext.standardDiameter = standardDiameter;
        g_fit_cloud_contexts.insert(cloudId, updatedContext);

        const FitStats& stats = updatedStats.first();
        const QString errorMessage = formatFitFooterMessage(targetCloud->getName(), stats, true);
        cloudInfoMessages.insert(cloudId, errorMessage);
        treeWidget->setCloudExtraInfo(cloudId,
                                      formatFitPropertyInfo(targetCloud->getName(), stats, stats, standardDiameter, true));
        if (treeWidget->getCurrentCloud() == targetCloud) {
            cloudWidget->setSelectedCloudMessage(errorMessage);
        }
        treeWidget->updateCloudItemAppearance(cloudId);
    }

    void UWScanMainWindow::onSelectAllActionTriggered(){
        cloudWidget->selectAllPoints();
    }

    void UWScanMainWindow::onSelectNoneActionTriggered(){
        cloudWidget->clearSelection();
    }

    void UWScanMainWindow::onDeleteActionTriggered(){
        auto currentCloud = treeWidget->getCurrentCloud();
        if (currentCloud && currentCloud->getSelectedCount() > 0) {
            cloudWidget->deleteSelectedPoints();
            refreshAllClouds();
            currentCloud->getPoints(depthPreviewPoints);
            updateDepthPreview();
            return;
        }
        treeWidget->deleteSelectedClouds();
    }

    void UWScanMainWindow::onReverseActionTriggered(){
        cloudWidget->reverseSelect();
    }

    void UWScanMainWindow::onRotateCenterActionTriggered(){

    }

    void UWScanMainWindow::onSelectToolActionTriggered(){

    }

    void UWScanMainWindow::onStopPreviewActionTriggered(){
        if(scanner->stopGrab()){
            previewAction->setEnabled(true);
            stopPreviewAction->setEnabled(false);
        }
    }

    void UWScanMainWindow::onGuideInActionTriggered(){
        QStringList filePaths = QFileDialog::getOpenFileNames(
            this,
            tr("导入点云"),
            QDir::homePath(),
            tr("点云文件(*.ply *.pcd *.xyz *.las *.txt *.asc)")
        );
        for (auto & filePath : filePaths) {
            if (filePath.isEmpty()) {
                return;
            }

            if (filePath.endsWith(".ply") || filePath.endsWith(".pcd") || filePath.endsWith(".xyz") || filePath.endsWith(".las") || filePath.endsWith(".txt") || filePath.endsWith(".asc")) {
                auto cloud = pointcloud::createFromFile(filePath);
                if (cloud && !cloud->empty()) {
                    treeWidget->addCloud(cloud);
                    refreshAllClouds();
                    float x = 0.0f;
                    float y = 0.0f;
                    float z = 0.0f;
                    cloud->getCentroid(x, y, z);
                    cloudWidget->onFocusCloudOrthographic(x, y, z);
                    centerTabWidget->setCurrentIndex(0);
                }
            } else {
                QMessageBox::warning(this, tr("警告"), tr("导入操作仅支持点云文件"));
            }
        }
    }

    void UWScanMainWindow::refreshAllClouds(){
        auto viewer = cloudWidget->getViewer();
        auto clouds = treeWidget->getAllClouds();
        size_t pointCount = 0;
        for (auto & cloud : clouds) {
            cloud->render(viewer);
            if (cloud->isVisible()) {
                pointCount += cloud->getPointCount();
            }
        }
        cloudWidget->setPointCount(pointCount);
        cloudWidget->refreshView();    
    }

    void UWScanMainWindow::hideCloudsByNames(const QStringList& names) {
        if (names.isEmpty()) {
            return;
        }
        const QSet<QString> targets(names.begin(), names.end());
        const auto clouds = treeWidget->getAllClouds();
        for (const auto& cloud : clouds) {
            if (!cloud || !targets.contains(cloud->getName()) || !cloud->isVisible()) {
                continue;
            }
            treeWidget->setCloudVisibility(QString::fromStdString(cloud->getId()), false);
        }
    }

    void UWScanMainWindow::onSelectionFinished(const std::vector<QPoint>& polygon){
        auto cur_cloud = treeWidget->getCurrentCloud();
        auto viewer = cloudWidget->getViewer();
        if (cur_cloud && cur_cloud->isVisible()) {
            bool is_shift = QApplication::keyboardModifiers() & Qt::ShiftModifier;
            cur_cloud->selectByScreenPolygon(polygon, viewer, is_shift);

            refreshAllClouds();
        }
    }

    void UWScanMainWindow::onHelpActionTriggered(){
        QDir docDir = QCoreApplication::applicationDirPath();
        docDir.cdUp();
        docDir.cdUp();
        docDir.cd("docs/html");
        QString docPath = docDir.absoluteFilePath("index.html");

        QFileInfo fi(docPath);
        if (!fi.exists()) {
            QMessageBox::warning(this, tr("文档不存在"),
                                tr("没有找到文档文件：\n%1").arg(docPath));
            return;
        }

        // 转成 file:// URL，用默认浏览器打开
        QUrl url = QUrl::fromLocalFile(fi.absoluteFilePath());
        if (!QDesktopServices::openUrl(url)) {
            QMessageBox::warning(this, tr("无法打开浏览器"),
                                tr("系统无法用默认浏览器打开：\n%1").arg(url.toString()));
        }
    }

    void UWScanMainWindow::onVoxelFilterActionTriggered(){
        NAMESPACE::pointcloud::PointCloudPtr cur_cloud = treeWidget->getCurrentCloud();
        if (!cur_cloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        NAMESPACE::pointcloud::FilterDialog filterDialog(cur_cloud, this);
        filterDialog.setAlgorithm(0);
        filterDialog.exec();

        float x, y, z;
        cur_cloud->getCentroid(x, y, z);
        this->cloudWidget->onFocusCloud(x, y, z);
        refreshAllClouds();
    }
    void UWScanMainWindow::onSorFilterActionTriggered(){
        NAMESPACE::pointcloud::PointCloudPtr cur_cloud = treeWidget->getCurrentCloud();
        if (!cur_cloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        NAMESPACE::pointcloud::FilterDialog filterDialog(cur_cloud, this);
        filterDialog.setAlgorithm(1);
        filterDialog.exec();

        float x, y, z;
        cur_cloud->getCentroid(x, y, z);
        this->cloudWidget->onFocusCloud(x, y, z);
        refreshAllClouds();
    }
    void UWScanMainWindow::onPassFilterActionTriggered(){
        NAMESPACE::pointcloud::PointCloudPtr cur_cloud = treeWidget->getCurrentCloud();
        if (!cur_cloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        NAMESPACE::pointcloud::FilterDialog filterDialog(cur_cloud, this);
        filterDialog.setAlgorithm(2);
        filterDialog.exec();

        float x, y, z;
        cur_cloud->getCentroid(x, y, z);
        this->cloudWidget->onFocusCloud(x, y, z);
        refreshAllClouds();
    }

    void UWScanMainWindow::onCylinderFeatureActionTriggered(){
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        std::vector<Eigen::Vector3f> points;
        curCloud->getPoints(points);
        FitStats stats = fitCylinderStats(points);
        if (!stats.ok) {
            QMessageBox::warning(this, tr("警告"), tr("圆柱拟合失败，请检查点云是否包含足够的圆柱面点"));
            return;
        }
        bool ok = false;
        const double standard = promptStandardDiameter(tr("标准直径"),
                                                       tr("请输入标准直径，0表示不对比："),
                                                       0.0,
                                                       true,
                                                       &ok);
        if (!ok) {
            return;
        }
        FitStats displayStats = standard > 0.0 ? cylinderStatsWithFixedDiameter(points, standard) : stats;
        hideCloudsByNames(QStringList{
            curCloud->getName() + "_圆柱标准误差",
            curCloud->getName() + "_圆柱拟合误差"
        });
        cloudWidget->clearFitOverlays();
        const QString fitGroupKey = QString::fromStdString(curCloud->getId()) + "_fit";
        cloudWidget->showCylinderFitOverlay(fitGroupKey,
                                            displayStats.axisPoint,
                                            displayStats.axis,
                                            displayStats.radius,
                                            displayStats.axisMin,
                                            displayStats.axisMax);
        const QString currentCloudId = QString::fromStdString(curCloud->getId());
        const QString currentMessage = formatFitFooterMessage(curCloud->getName(), displayStats, standard > 0.0);
        cloudInfoMessages.insert(currentCloudId, currentMessage);
        treeWidget->clearCloudExtraInfo(currentCloudId);
        cloudWidget->setSelectedCloudMessage(currentMessage);
        auto errorCloud = createCylinderErrorCloud(points, displayStats,
                                                   curCloud->getName() + (standard > 0.0 ? "_圆柱标准误差" : "_圆柱拟合误差"));
        if (errorCloud) {
            treeWidget->setCloudVisibility(currentCloudId, false);
            treeWidget->addCloud(errorCloud);
            const QString errorCloudId = QString::fromStdString(errorCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(errorCloudId, fitGroupKey);
            treeWidget->setCloudGroupForCloud(errorCloudId, fitGroupKey);
            const QString errorMessage = formatFitFooterMessage(errorCloud->getName(), displayStats, standard > 0.0);
            cloudInfoMessages.insert(errorCloudId, errorMessage);
            treeWidget->setCloudExtraInfo(errorCloudId,
                                          formatFitPropertyInfo(errorCloud->getName(), displayStats, displayStats, standard, standard > 0.0));
            g_cloud_fit_error_stats.insert(errorCloudId, QVector<FitStats>{displayStats});
            g_fit_cloud_contexts.insert(errorCloudId, FitCloudContext{FitFeatureKind::Cylinder, true, standard});
            cloudLegendInfos.insert(errorCloudId, CloudLegendInfo{
                standard > 0.0 ? tr("圆柱标准误差") : tr("圆柱拟合误差"),
                1.0
            });
            if (treeWidget->getCurrentCloud() == errorCloud) {
                cloudWidget->setSelectedCloudMessage(errorMessage);
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(errorCloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            }
            refreshAllClouds();
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            errorCloud->getCentroid(x, y, z);
            cloudWidget->onFocusCloudOrthographic(x, y, z);
            centerTabWidget->setCurrentIndex(0);
        }
        QMessageBox::information(this, tr("圆柱拟合结果"), formatFitReport(displayStats, standard));
    }

    void UWScanMainWindow::onCylinderErrorAnalysisActionTriggered(){
        {
            const QString summaryPath = QFileDialog::getOpenFileName(
                this,
                tr("选择误差数据文件"),
                QDir::homePath(),
                tr("CSV文件(*.csv);;Excel文件(*.xls *.xlsx)")
            );
            if (summaryPath.isEmpty()) {
                return;
            }

            QVector<ErrorAnalysisRow> rows;
            if (!readErrorTable(summaryPath, &rows)) {
                QMessageBox::warning(this, tr("警告"), tr("无法读取误差数据，请确认文件格式是否正确"));
                return;
            }

            QString featureName;
            if (!rows.isEmpty()) {
                featureName = rows.first().featureName;
            }
            const QString summaryTitle = featureName.isEmpty()
                ? QFileInfo(summaryPath).completeBaseName() + tr(" 误差分析")
                : featureName + tr(" 误差分析");

            QDialog summaryDialog(this);
            summaryDialog.setWindowTitle(summaryTitle);
            applyAdaptiveDialogSize(summaryDialog, this, 1360, 860, 900, 620);

            auto* summaryLayout = new QVBoxLayout(&summaryDialog);
            summaryLayout->setContentsMargins(12, 12, 12, 12);
            summaryLayout->setSpacing(8);

            auto* summaryTextLabel = new QLabel(formatSummaryAnalysisText(rows), &summaryDialog);
            summaryTextLabel->setWordWrap(true);
            summaryLayout->addWidget(summaryTextLabel);

            auto* summaryChartLabel = new QLabel(&summaryDialog);
            summaryChartLabel->setAlignment(Qt::AlignCenter);
            const QSize summaryChartSize = fitChartSizeForScreen(&summaryDialog, 1240, 640, 760, 420);
            summaryChartLabel->setPixmap(createSummaryAnalysisChart(rows, summaryTitle, summaryChartSize));
            summaryLayout->addWidget(summaryChartLabel, 1);

            summaryDialog.exec();
            return;
        }
        {
            const QString path = QFileDialog::getOpenFileName(
                this,
                tr("选择误差数据文件"),
                QDir::homePath(),
                tr("CSV文件(*.csv);;Excel文件(*.xls *.xlsx)")
            );
            if (path.isEmpty()) {
                return;
            }

            QVector<double> errors;
            QString featureName;
            if (!readErrorTable(path, &errors, &featureName)) {
                QMessageBox::warning(this, tr("警告"), tr("无法读取误差数据，请确认文件格式是否正确"));
                return;
            }

            const QString title = featureName.isEmpty()
                ? QFileInfo(path).completeBaseName() + tr(" 误差分析")
                : featureName + tr(" 误差分析");

            QDialog dialog(this);
            dialog.setWindowTitle(title);
            dialog.resize(1000, 680);

            auto* layout = new QVBoxLayout(&dialog);
            layout->setContentsMargins(12, 12, 12, 12);
            layout->setSpacing(8);

            auto* summaryLabel = new QLabel(formatErrorSummary(errors), &dialog);
            summaryLabel->setWordWrap(true);
            layout->addWidget(summaryLabel);

            auto* chartLabel = new QLabel(&dialog);
            chartLabel->setAlignment(Qt::AlignCenter);
            chartLabel->setPixmap(createErrorLineChart(errors, title, QSize(960, 540)));
            layout->addWidget(chartLabel, 1);

            dialog.exec();
            return;
        }
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个含缺陷的点云"));
            return;
        }
        bool ok = false;
        const double standardDiameter = promptStandardDiameter(tr("圆柱直径"),
                                                               tr("请输入圆柱标准直径："),
                                                               0.0001,
                                                               false,
                                                               &ok);
        if (!ok || standardDiameter <= 0.0) {
            return;
        }

        std::vector<Eigen::Vector3f> points;
        curCloud->getPoints(points);
        FitStats stats = cylinderStatsWithFixedDiameter(points, standardDiameter);
        if (!stats.ok) {
            QMessageBox::warning(this, tr("警告"), tr("圆柱拟合失败，请检查点云是否包含足够的圆柱面点"));
            return;
        }
        hideCloudsByNames(QStringList{curCloud->getName() + "_圆柱误差"});

        const double defectThreshold = std::max(stats.rmse * 3.0, stats.maxAbsError * 0.2);
        const CylinderDefectMetrics metrics = computeCylinderDefectMetrics(points, stats, defectThreshold);
        cloudWidget->clearFitOverlays();
        const QString fitGroupKey = QString::fromStdString(curCloud->getId()) + "_fit";
        cloudWidget->showCylinderFitOverlay(fitGroupKey,
                                            stats.axisPoint,
                                            stats.axis,
                                            stats.radius,
                                            stats.axisMin,
                                            stats.axisMax);
        const QString currentCloudId = QString::fromStdString(curCloud->getId());
        const QString currentMessage = formatFitFooterMessage(curCloud->getName(), stats, true);
        cloudInfoMessages.insert(currentCloudId, currentMessage);
        treeWidget->clearCloudExtraInfo(currentCloudId);
        cloudWidget->setSelectedCloudMessage(currentMessage);
        auto errorCloud = createCylinderErrorCloud(points, stats, curCloud->getName() + "_圆柱误差");
        if (errorCloud) {
            treeWidget->setCloudVisibility(currentCloudId, false);
            treeWidget->addCloud(errorCloud);
            const QString errorCloudId = QString::fromStdString(errorCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(errorCloudId, fitGroupKey);
            treeWidget->setCloudGroupForCloud(errorCloudId, fitGroupKey);
            const QString errorMessage = formatFitFooterMessage(errorCloud->getName(), stats, true);
            cloudInfoMessages.insert(errorCloudId, errorMessage);
            treeWidget->setCloudExtraInfo(errorCloudId,
                                          formatFitPropertyInfo(errorCloud->getName(), stats, stats, standardDiameter, true));
            g_cloud_fit_error_stats.insert(errorCloudId, QVector<FitStats>{stats});
            g_fit_cloud_contexts.insert(errorCloudId, FitCloudContext{FitFeatureKind::Cylinder, true, standardDiameter});
            cloudLegendInfos.insert(errorCloudId, CloudLegendInfo{tr("圆柱误差"), 1.0});
            if (treeWidget->getCurrentCloud() == errorCloud) {
                cloudWidget->setSelectedCloudMessage(errorMessage);
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(errorCloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            }
            refreshAllClouds();
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            errorCloud->getCentroid(x, y, z);
            cloudWidget->onFocusCloudOrthographic(x, y, z);
            centerTabWidget->setCurrentIndex(0);
        }

        QMessageBox::information(this, tr("圆柱面误差分析"),
            formatFitReport(stats, standardDiameter) +
            tr("\n\n缺陷轴向长度：%1\n缺陷周向弧长：%2\n缺陷深度：%3\n缺陷面积估算：%4\n缺陷点数：%5")
                .arg(metrics.axialLength, 0, 'f', 4)
                .arg(metrics.circumferentialArc, 0, 'f', 4)
                .arg(metrics.depth, 0, 'f', 4)
                .arg(metrics.area, 0, 'f', 4)
                .arg(metrics.defectPointCount));
    }

    void UWScanMainWindow::onSphereFeatureActionTriggered(){
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        std::vector<Eigen::Vector3f> points;
        curCloud->getPoints(points);
        FitStats fittedStats = fitSphereStats(points);
        if (!fittedStats.ok) {
            QMessageBox::warning(this, tr("警告"), tr("球拟合失败，请检查点云是否包含足够的球面点"));
            return;
        }
        bool ok = false;
        const double standard = promptStandardDiameter(tr("标准直径"),
                                                       tr("请输入标准直径，0表示不对比："),
                                                       0.0,
                                                       true,
                                                       &ok);
        if (!ok) {
            return;
        }
        FitStats errorStats = standard > 0.0
            ? sphereErrorStatsAgainstStandard(points, fittedStats, standard, "SphereStandard")
            : fittedStats;
        hideCloudsByNames(QStringList{
            curCloud->getName() + "_球标准误差",
            curCloud->getName() + "_球拟合误差"
        });
        cloudWidget->clearFitOverlays();
        const QString fitGroupKey = QString::fromStdString(curCloud->getId()) + "_fit";
        cloudWidget->showSphereFitOverlay(fitGroupKey,
                                          fittedStats.center,
                                          fittedStats.radius);
        const QString currentCloudId = QString::fromStdString(curCloud->getId());
        const QString currentMessage = formatFitFooterMessage(curCloud->getName(), errorStats, standard > 0.0);
        cloudInfoMessages.insert(currentCloudId, currentMessage);
        treeWidget->clearCloudExtraInfo(currentCloudId);
        cloudWidget->setSelectedCloudMessage(currentMessage);
        auto errorCloud = createSphereErrorCloud(points, errorStats,
                                                 curCloud->getName() + (standard > 0.0 ? "_球标准误差" : "_球拟合误差"));
        if (errorCloud) {
            treeWidget->setCloudVisibility(currentCloudId, false);
            treeWidget->addCloud(errorCloud);
            const QString errorCloudId = QString::fromStdString(errorCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(errorCloudId, fitGroupKey);
            treeWidget->setCloudGroupForCloud(errorCloudId, fitGroupKey);
            const QString errorMessage = formatFitFooterMessage(errorCloud->getName(), errorStats, standard > 0.0);
            cloudInfoMessages.insert(errorCloudId, errorMessage);
            treeWidget->setCloudExtraInfo(errorCloudId,
                                          formatFitPropertyInfo(errorCloud->getName(), fittedStats, errorStats, standard, standard > 0.0));
            g_cloud_fit_error_stats.insert(errorCloudId, QVector<FitStats>{errorStats});
            g_fit_cloud_contexts.insert(errorCloudId, FitCloudContext{FitFeatureKind::Sphere, true, standard});
            cloudLegendInfos.insert(errorCloudId, CloudLegendInfo{
                standard > 0.0 ? tr("球标准误差") : tr("球拟合误差"),
                1.0
            });
            if (treeWidget->getCurrentCloud() == errorCloud) {
                cloudWidget->setSelectedCloudMessage(errorMessage);
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(errorCloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            }
            refreshAllClouds();
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            errorCloud->getCentroid(x, y, z);
            cloudWidget->onFocusCloudOrthographic(x, y, z);
            centerTabWidget->setCurrentIndex(0);
        }
        QMessageBox::information(this, tr("球拟合结果"),
            formatFitReport(fittedStats, standard) +
            (standard > 0.0
                ? tr("\n\n标准球误差\n平均误差：%1\nRMSE：%2\n最大绝对误差：%3")
                    .arg(errorStats.meanError, 0, 'f', 4)
                    .arg(errorStats.rmse, 0, 'f', 4)
                    .arg(errorStats.maxAbsError, 0, 'f', 4)
                : QString()));
    }

    void UWScanMainWindow::onDoubleSphereFeatureActionTriggered(){
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }
        std::vector<Eigen::Vector3f> points;
        curCloud->getPoints(points);
        if (points.size() < 40) {
            QMessageBox::warning(this, tr("警告"), tr("双球拟合需要更多点"));
            return;
        }

        std::vector<float> xs;
        xs.reserve(points.size());
        for (const auto& point : points) {
            xs.push_back(point.x());
        }
        std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
        const float medianX = xs[xs.size() / 2];
        std::vector<Eigen::Vector3f> leftPoints;
        std::vector<Eigen::Vector3f> rightPoints;
        for (const auto& point : points) {
            (point.x() <= medianX ? leftPoints : rightPoints).push_back(point);
        }

        FitStats firstFit = fitSphereStats(leftPoints, "SphereA");
        FitStats secondFit = fitSphereStats(rightPoints, "SphereB");
        if (!firstFit.ok || !secondFit.ok) {
            QMessageBox::warning(this, tr("警告"), tr("双球拟合失败，请检查点云是否能按X方向分成两个球面"));
            return;
        }
        bool ok = false;
        const double standard = promptStandardDiameter(tr("标准直径"),
                                                       tr("请输入标准直径，0表示不对比："),
                                                       0.0,
                                                       true,
                                                       &ok);
        if (!ok) {
            return;
        }
        FitStats firstError = standard > 0.0
            ? sphereErrorStatsAgainstStandard(leftPoints, firstFit, standard, "SphereAStandard")
            : firstFit;
        FitStats secondError = standard > 0.0
            ? sphereErrorStatsAgainstStandard(rightPoints, secondFit, standard, "SphereBStandard")
            : secondFit;
        hideCloudsByNames(QStringList{
            curCloud->getName() + "_SphereA标准误差",
            curCloud->getName() + "_SphereA误差",
            curCloud->getName() + "_SphereB标准误差",
            curCloud->getName() + "_SphereB误差"
        });
        if (!firstError.ok || !secondError.ok) {
            QMessageBox::warning(this, tr("警告"), tr("双球拟合失败，请检查点云是否能按X方向分成两个球面"));
            return;
        }
        cloudWidget->clearFitOverlays();
        const QString fitBaseId = QString::fromStdString(curCloud->getId()) + "_fit";
        cloudWidget->showSphereFitOverlay(fitBaseId + "_a", firstFit.center, firstFit.radius);
        cloudWidget->showSphereFitOverlay(fitBaseId + "_b", secondFit.center, secondFit.radius);
        const QString currentCloudId = QString::fromStdString(curCloud->getId());
        const QString currentMessage = formatFitFooterMessage(curCloud->getName(),
                                                              firstError.maxAbsError >= secondError.maxAbsError ? firstError : secondError,
                                                              standard > 0.0);
        cloudInfoMessages.insert(currentCloudId, currentMessage);
        treeWidget->clearCloudExtraInfo(currentCloudId);
        cloudWidget->setSelectedCloudMessage(currentMessage);

        auto firstErrorCloud = createSphereErrorCloud(leftPoints, firstError,
                                                      curCloud->getName() + (standard > 0.0 ? "_SphereA标准误差" : "_SphereA误差"));
        auto secondErrorCloud = createSphereErrorCloud(rightPoints, secondError,
                                                       curCloud->getName() + (standard > 0.0 ? "_SphereB标准误差" : "_SphereB误差"));
        bool hasErrorCloud = false;
        if (firstErrorCloud) {
            treeWidget->addCloud(firstErrorCloud);
            const QString errorCloudId = QString::fromStdString(firstErrorCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(errorCloudId, fitBaseId);
            treeWidget->setCloudGroupForCloud(errorCloudId, fitBaseId);
            const QString errorMessage = formatFitFooterMessage(firstErrorCloud->getName(), firstError, standard > 0.0);
            cloudInfoMessages.insert(errorCloudId, errorMessage);
            treeWidget->setCloudExtraInfo(errorCloudId,
                                          formatFitPropertyInfo(firstErrorCloud->getName(), firstFit, firstError, standard, standard > 0.0));
            g_cloud_fit_error_stats.insert(errorCloudId, QVector<FitStats>{firstError});
            g_fit_cloud_contexts.insert(errorCloudId, FitCloudContext{FitFeatureKind::Sphere, true, standard});
            cloudLegendInfos.insert(errorCloudId, CloudLegendInfo{
                standard > 0.0 ? tr("SphereA标准误差") : tr("SphereA误差"),
                1.0
            });
            if (treeWidget->getCurrentCloud() == firstErrorCloud) {
                cloudWidget->setSelectedCloudMessage(errorMessage);
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(errorCloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            }
            hasErrorCloud = true;
        }
        if (secondErrorCloud) {
            treeWidget->addCloud(secondErrorCloud);
            const QString errorCloudId = QString::fromStdString(secondErrorCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(errorCloudId, fitBaseId);
            treeWidget->setCloudGroupForCloud(errorCloudId, fitBaseId);
            const QString errorMessage = formatFitFooterMessage(secondErrorCloud->getName(), secondError, standard > 0.0);
            cloudInfoMessages.insert(errorCloudId, errorMessage);
            treeWidget->setCloudExtraInfo(errorCloudId,
                                          formatFitPropertyInfo(secondErrorCloud->getName(), secondFit, secondError, standard, standard > 0.0));
            g_cloud_fit_error_stats.insert(errorCloudId, QVector<FitStats>{secondError});
            g_fit_cloud_contexts.insert(errorCloudId, FitCloudContext{FitFeatureKind::Sphere, true, standard});
            cloudLegendInfos.insert(errorCloudId, CloudLegendInfo{
                standard > 0.0 ? tr("SphereB标准误差") : tr("SphereB误差"),
                1.0
            });
            if (treeWidget->getCurrentCloud() == secondErrorCloud) {
                cloudWidget->setSelectedCloudMessage(errorMessage);
                const CloudLegendInfo legendInfo = cloudLegendInfos.value(errorCloudId);
                cloudWidget->setErrorLegend(legendInfo.title, legendInfo.maxAbsError);
            }
            hasErrorCloud = true;
        }
        if (hasErrorCloud) {
            treeWidget->setCloudVisibility(currentCloudId, false);
            refreshAllClouds();
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            curCloud->getCentroid(x, y, z);
            cloudWidget->onFocusCloudOrthographic(x, y, z);
            centerTabWidget->setCurrentIndex(0);
        }
        QString report = formatFitReport(firstFit, standard) + "\n\n" + formatFitReport(secondFit, standard);
        if (standard > 0.0) {
            report += tr("\n\nSphereA 标准球误差\n平均误差：%1\nRMSE：%2\n最大绝对误差：%3")
                .arg(firstError.meanError, 0, 'f', 4)
                .arg(firstError.rmse, 0, 'f', 4)
                .arg(firstError.maxAbsError, 0, 'f', 4);
            report += tr("\n\nSphereB 标准球误差\n平均误差：%1\nRMSE：%2\n最大绝对误差：%3")
                .arg(secondError.meanError, 0, 'f', 4)
                .arg(secondError.rmse, 0, 'f', 4)
                .arg(secondError.maxAbsError, 0, 'f', 4);
        }
        QMessageBox::information(this, tr("双球拟合结果"), report);
    }

    void UWScanMainWindow::onPlaneDefectMeasurementActionTriggered() {
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }

        const QString sourceCloudId = QString::fromStdString(curCloud->getId());
        const QString originalCloudName = curCloud->getName();
        const size_t originalPointCount = curCloud->getPointCount();
        const bool useSelectedRoi = curCloud->getSelectedCount() >= 50;
        size_t selectedPointCount = curCloud->getSelectedCount();
        bool retainedCurrentCloud = false;
        QString roiSourceText;
        if (useSelectedRoi) {
            retainedCurrentCloud = curCloud->keepSelectedOnly();
            roiSourceText = tr("当前框选点");
        } else {
            std::vector<Eigen::Vector3f> originalPoints;
            curCloud->getPoints(originalPoints);
            const PlaneDefectCropStats autoCrop = cropCenterPlaneDefect(originalPoints, curCloud->getName());
            const bool cropIsEffective = autoCrop.ok
                && autoCrop.retainedIndices.size() >= 50
                && autoCrop.retainedIndices.size() < static_cast<size_t>(originalPoints.size() * 0.80);
            if (cropIsEffective) {
                retainedCurrentCloud = curCloud->retainIndices(autoCrop.retainedIndices);
                roiSourceText = tr("自动裁剪");
            }
        }

        if (!retainedCurrentCloud) {
            QMessageBox::warning(this,
                                 tr("警告"),
                                 tr("未能从原始点云中稳定裁剪出缺陷区域。\n\n请先用矩形或套索框选缺陷所示工件区域，然后再点击“平面缺陷测量”。\n当前点数: %1\n当前框选点数: %2")
                                     .arg(originalPointCount)
                                     .arg(selectedPointCount));
            return;
        }

        std::vector<Eigen::Vector3f> points;
        curCloud->getPoints(points);
        PlaneDefectCropStats result = makePlaneDefectCropFromRoi(points, curCloud->getName() + "_ROI");
        if (!result.ok) {
            QMessageBox::warning(this,
                                 tr("警告"),
                                 tr("当前保留区域无法拟合平面，请重新框选缺陷所示区域"));
            return;
        }

        const PlaneDefectAnalysisResult analysis = analyzePlaneDefects(points, result, curCloud->getName() + "_Defects");
        QString report = formatPlaneDefectCropSummary(curCloud->getName(), result);
        const QString defectGroupKey = sourceCloudId + "_plane_defects";
        const auto defectColors = planeDefectPalette();
        removePlaneDefectRectangleOverlays(cloudWidget->getViewer(), defectGroupKey);
        showPlaneDefectBasePlaneOverlay(cloudWidget, result, defectGroupKey);
        cloudWidget->setFitOverlayGroupForCloud(sourceCloudId, defectGroupKey);
        treeWidget->setCloudGroupForCloud(sourceCloudId, defectGroupKey);
        if (analysis.ok) {
            report = formatPlaneDefectAnalysisSummary(curCloud->getName(), result, analysis);
            for (const auto& defect : analysis.defects) {
                const pointcloud::ColorRGB color = defectColors[static_cast<size_t>((defect.index - 1) % defectColors.size())];
                auto defectCloud = createPlaneDefectMeasurementCloud(points,
                                                                     defect,
                                                                     QString("%1_S%2")
                                                                         .arg(originalCloudName)
                                                                         .arg(defect.index),
                                                                     color);
                if (!defectCloud) {
                    continue;
                }
                defectCloud->setHighlighted(true);
                treeWidget->addCloud(defectCloud);
                const QString defectCloudId = QString::fromStdString(defectCloud->getId());
                cloudWidget->setFitOverlayGroupForCloud(defectCloudId, defectGroupKey);
                treeWidget->setCloudGroupForCloud(defectCloudId, defectGroupKey);
                treeWidget->setCloudExtraInfo(defectCloudId,
                                              formatPlaneDefectAnalysisSummary(defectCloud->getName(), result, PlaneDefectAnalysisResult{
                                                  true,
                                                  analysis.plane,
                                                  analysis.origin,
                                                  analysis.axisX,
                                                  analysis.axisY,
                                                  analysis.defectOnNegativeSide,
                                                  analysis.threshold,
                                                  std::vector<PlaneDefectMeasurement>{defect}
                                              }));
            }
            showPlaneDefectRectangleOverlays(cloudWidget, analysis, defectGroupKey);
        } else {
            report += tr("\n\n当前点云已裁剪为平面缺陷ROI。当前样本的后续凹槽分割未稳定完成，因此本次仅执行ROI保留。");
        }
        report += tr("\n\nROI来源: %1\n裁剪前点数: %2\n裁剪后点数: %3")
            .arg(roiSourceText)
            .arg(originalPointCount)
            .arg(points.size());

        if (!curCloud->getName().endsWith("_ROI")) {
            curCloud->setName(originalCloudName + "_ROI");
        }
        treeWidget->setCloudExtraInfo(sourceCloudId, report);
        cloudInfoMessages.insert(sourceCloudId, report);
        curCloud->render(cloudWidget->getViewer());
        refreshAllClouds();
        treeWidget->updateCloudItemAppearance(sourceCloudId);

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        curCloud->getCentroid(x, y, z);
        cloudWidget->onFocusCloudOrthographic(x, y, z);
        cloudWidget->setPointCount(curCloud->getPointCount());
        cloudWidget->refreshView();
        centerTabWidget->setCurrentIndex(0);

        QMessageBox::information(this, tr("平面缺陷测量结果"), report);
    }

    void UWScanMainWindow::onCylinderDefectMeasurementActionTriggered() {
        auto curCloud = treeWidget->getCurrentCloud();
        if (!curCloud) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个点云"));
            return;
        }

        const QString sourceCloudId = QString::fromStdString(curCloud->getId());
        const QString originalCloudName = curCloud->getName();
        std::vector<Eigen::Vector3f> allPoints;
        curCloud->getPoints(allPoints);
        std::vector<Eigen::Vector3f> searchPoints;
        curCloud->getSelectedPoints(searchPoints);
        QString roiSourceText = tr("当前框选点");
        if (searchPoints.size() < 50) {
            searchPoints = allPoints;
            roiSourceText = tr("当前点云");
        }
        if (allPoints.size() < 50 || searchPoints.size() < 10) {
            QMessageBox::warning(this, tr("警告"), tr("圆柱缺陷测量至少需要 50 个点"));
            return;
        }

        FitStats fittedStats = fitCylinderStats(allPoints);
        if (!fittedStats.ok) {
            QMessageBox::warning(this, tr("警告"), tr("圆柱拟合失败，请检查点云是否包含足够的圆柱面点"));
            return;
        }
        FitStats searchStats = cylinderStatsForMeasuredPoints(searchPoints, fittedStats);

        const CylinderDefectThreshold autoThreshold = robustCylinderDefectThreshold(fittedStats);
        bool ok = false;
        const double inputThreshold = QInputDialog::getDouble(this,
                                                              tr("圆柱缺陷阈值"),
                                                              tr("请输入阈值：正值检测外凸(ρ-R)，负值检测内凹(R-ρ)，0表示自适应："),
                                                              autoThreshold.signedThreshold,
                                                              -1000000.0,
                                                              1000000.0,
                                                              4,
                                                              &ok);
        if (!ok) {
            return;
        }
        const bool manualThreshold = std::abs(inputThreshold) > 1e-9;
        const double defectThreshold = manualThreshold ? inputThreshold : autoThreshold.signedThreshold;
        CylinderDefectMetrics metrics = computeCylinderDefectMetrics(searchPoints, searchStats, defectThreshold);
        if (!metrics.ok) {
            QMessageBox::warning(this,
                                 tr("警告"),
                                 tr("未找到满足阈值的局部圆柱缺陷点。\n\n最小偏差: %1\n最大偏差: %2\n当前阈值: %3\n提示: 外凸缺陷用正阈值，内凹缺陷用负阈值。")
                                     .arg(searchStats.errors.empty() ? 0.0 : *std::min_element(searchStats.errors.begin(), searchStats.errors.end()), 0, 'f', 4)
                                     .arg(searchStats.errors.empty() ? 0.0 : *std::max_element(searchStats.errors.begin(), searchStats.errors.end()), 0, 'f', 4)
                                     .arg(defectThreshold, 0, 'f', 4));
            return;
        }

        const QString defectGroupKey = sourceCloudId + "_cylinder_defects";
        hideCloudsByNames(QStringList{originalCloudName + "_圆柱缺陷"});
        cloudWidget->clearFitOverlays();
        cloudWidget->showCylinderFitOverlay(defectGroupKey,
                                            fittedStats.axisPoint,
                                            fittedStats.axis,
                                            fittedStats.radius,
                                            fittedStats.axisMin,
                                            fittedStats.axisMax);
        showCylinderDefectProjectionMesh(cloudWidget, fittedStats, metrics, defectGroupKey);
        cloudWidget->setFitOverlayGroupForCloud(sourceCloudId, defectGroupKey);
        treeWidget->setCloudGroupForCloud(sourceCloudId, defectGroupKey);

        const QString report = formatCylinderDefectReport(originalCloudName,
                                                          fittedStats,
                                                          metrics,
                                                          manualThreshold)
            + tr("\n\nROI来源: %1\n圆柱拟合点数: %2\n缺陷搜索点数: %3")
                  .arg(roiSourceText)
                  .arg(allPoints.size())
                  .arg(searchPoints.size());
        auto defectCloud = createCylinderDefectCloud(searchPoints, metrics, originalCloudName + "_圆柱缺陷");
        if (defectCloud) {
            defectCloud->setHighlighted(true);
            treeWidget->addCloud(defectCloud);
            const QString defectCloudId = QString::fromStdString(defectCloud->getId());
            cloudWidget->setFitOverlayGroupForCloud(defectCloudId, defectGroupKey);
            treeWidget->setCloudGroupForCloud(defectCloudId, defectGroupKey);
            treeWidget->setCloudExtraInfo(defectCloudId, report);
            cloudInfoMessages.insert(defectCloudId, report);
        }

        treeWidget->setCloudExtraInfo(sourceCloudId, report);
        cloudInfoMessages.insert(sourceCloudId, report);
        refreshAllClouds();
        treeWidget->updateCloudItemAppearance(sourceCloudId);

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        curCloud->getCentroid(x, y, z);
        cloudWidget->onFocusCloudOrthographic(x, y, z);
        cloudWidget->setSelectedCloudMessage(report);
        cloudWidget->refreshView();
        centerTabWidget->setCurrentIndex(0);

        QMessageBox::information(this, tr("圆柱缺陷测量结果"), report);
    }

    void UWScanMainWindow::onPlaneFeatureActionTriggered() {
        auto selectedClouds = treeWidget->getSelectedClouds();
        if (selectedClouds.empty()) {
            auto currentCloud = treeWidget->getCurrentCloud();
            if (currentCloud) {
                selectedClouds.push_back(currentCloud);
            }
        }
        if (selectedClouds.empty()) {
            QMessageBox::warning(this, tr("警告"), tr("请先选中一个或两个点云"));
            return;
        }

        auto formatPlaneReport = [](const QString& cloudName, const PlaneFitStats& stats) {
            return QObject::tr(
                "%1\n法向量: (%2, %3, %4)\n平面方程: %5x + %6y + %7z + %8 = 0\n平均误差: %9\nRMSE: %10\n最大绝对误差: %11")
                .arg(cloudName)
                .arg(stats.normal.x(), 0, 'f', 6)
                .arg(stats.normal.y(), 0, 'f', 6)
                .arg(stats.normal.z(), 0, 'f', 6)
                .arg(stats.normal.x(), 0, 'f', 6)
                .arg(stats.normal.y(), 0, 'f', 6)
                .arg(stats.normal.z(), 0, 'f', 6)
                .arg(stats.d, 0, 'f', 6)
                .arg(stats.meanError, 0, 'f', 4)
                .arg(stats.rmse, 0, 'f', 4)
                .arg(stats.maxAbsError, 0, 'f', 4);
        };
        auto angleBetweenNormals = [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
            const double dotValue = std::clamp(std::abs(static_cast<double>(a.normalized().dot(b.normalized()))), 0.0, 1.0);
            return std::acos(dotValue) * kRadToDeg;
        };

        if (selectedClouds.size() >= 2) {
            std::vector<Eigen::Vector3f> firstPoints;
            std::vector<Eigen::Vector3f> secondPoints;
            selectedClouds[0]->getPoints(firstPoints);
            selectedClouds[1]->getPoints(secondPoints);

            const PlaneFitStats firstPlane = fitPlaneStats(firstPoints, selectedClouds[0]->getName());
            const PlaneFitStats secondPlane = fitPlaneStats(secondPoints, selectedClouds[1]->getName());
            if (!firstPlane.ok || !secondPlane.ok) {
                QMessageBox::warning(this, tr("警告"), tr("平面拟合失败，请确认选中的点云包含足够且有效的平面点"));
                return;
            }

            const double includedAngle = planeAngleDegrees(firstPlane.normal, secondPlane.normal);
            treeWidget->setCloudExtraInfo(QString::fromStdString(selectedClouds[0]->getId()),
                                          formatPlanePropertyInfo(selectedClouds[0]->getName(), firstPlane, includedAngle));
            treeWidget->setCloudExtraInfo(QString::fromStdString(selectedClouds[1]->getId()),
                                          formatPlanePropertyInfo(selectedClouds[1]->getName(), secondPlane, includedAngle));

            QString report = formatPlaneReport(selectedClouds[0]->getName(), firstPlane);
            report += tr("\n\n");
            report += formatPlaneReport(selectedClouds[1]->getName(), secondPlane);
            report += tr("\n\n两平面夹角: %1°")
                .arg(includedAngle, 0, 'f', 4);
            QMessageBox::information(this, tr("平面拟合结果"), report);
            return;
        }

        std::vector<Eigen::Vector3f> points;
        selectedClouds.front()->getPoints(points);
        const PlaneFitStats plane = fitPlaneStats(points, selectedClouds.front()->getName());
        if (!plane.ok) {
            QMessageBox::warning(this, tr("警告"), tr("平面拟合失败，请确认点云包含足够且有效的平面点"));
            return;
        }

        treeWidget->setCloudExtraInfo(QString::fromStdString(selectedClouds.front()->getId()),
                                      formatPlanePropertyInfo(selectedClouds.front()->getName(), plane));

        QString report = formatPlaneReport(selectedClouds.front()->getName(), plane);
        report += tr("\n\n与 XY 平面夹角: %1°\n与 XZ 平面夹角: %2°\n与 YZ 平面夹角: %3°")
            .arg(planeAngleDegrees(plane.normal, Eigen::Vector3f::UnitZ()), 0, 'f', 4)
            .arg(planeAngleDegrees(plane.normal, Eigen::Vector3f::UnitY()), 0, 'f', 4)
            .arg(planeAngleDegrees(plane.normal, Eigen::Vector3f::UnitX()), 0, 'f', 4);
        QMessageBox::information(this, tr("平面拟合结果"), report);
        return;
    }

    void UWScanMainWindow::onSettingActionTriggered(){
        setting::SettingWidget settingWidget(this);
        settingWidget.setWindowModality(Qt::ApplicationModal);
        const QString originalSaveDir = setting::SettingsManager::getInstance()->getString(setting::SAVE_IMAGE_DIR, QDir::homePath());
        connect(&settingWidget, &setting::SettingWidget::saveImageDirChanged, this, [this](const QString& path) {
            if (!path.trimmed().isEmpty()) {
                scannerWidget->previewSaveBaseDir(path);
            }
        });
        connect(&settingWidget, &setting::SettingWidget::calibrationFileChanged, this, [this](const QString& path) {
            if (!path.trimmed().isEmpty()) {
                scannerWidget->setCalibrationFile(path);
            }
        });
        auto ret = settingWidget.exec();
        if(ret == QDialog::Accepted){
            //更新设置
            scanner::ReconstructionConfig temp_rc;
            auto * settingManager = setting::SettingsManager::getInstance();
            populateReconstructionConfig(temp_rc, settingManager);
            if (!reconstructor->init(temp_rc)) {
                QMessageBox::warning(this, tr("警告"), tr("标定文件加载失败：%1\n%2")
                    .arg(QString::fromStdString(temp_rc.calibrationPath))
                    .arg(QString::fromStdString(reconstructor->getLastError())));
            }
            scannerWidget->setDefaultValues();
            scannerWidget->setSaveBaseDir(settingManager->getString(setting::SAVE_IMAGE_DIR, QDir::homePath()));
            scannerWidget->setCalibrationFile(settingManager->getString(setting::DEFAULT_CALIBRATE_FILE));
        } else {
            scannerWidget->previewSaveBaseDir(originalSaveDir);
        }
    }

    void UWScanMainWindow::onStartServerActionTriggered(){
        bool isRunning = false;
        QMetaObject::invokeMethod(commandServer, "isServiceRunning", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, isRunning));
        if(isRunning){
            QMetaObject::invokeMethod(commandServer, "stopService", Qt::BlockingQueuedConnection);
            if(serverDialog){
                serverDialog->close();
                serverDialog->deleteLater();
                serverDialog = nullptr;
            }
            QMessageBox::information(this, tr("提示"), tr("服务已停止"));
        } else {
            QMetaObject::invokeMethod(commandServer, "startService", Qt::BlockingQueuedConnection);
            bool serviceStarted = false;
            QMetaObject::invokeMethod(commandServer, "isServiceRunning", Qt::BlockingQueuedConnection,
                                      Q_RETURN_ARG(bool, serviceStarted));
            if(serviceStarted){
                if(!serverDialog){
                    serverDialog = new server::RemoteCommandDialog(this);
                    connect(serverDialog, &QObject::destroyed, this, [this](){
                        serverDialog = nullptr;
                    });
                }
                serverDialog->show();
                serverDialog->raise();
                serverDialog->activateWindow();
                QMessageBox::information(this, tr("提示"), tr("服务已启动"));
            } else {
                QMessageBox::warning(this, tr("警告"), tr("远程控制服务启动失败，请检查端口 %1 是否被占用。")
                    .arg(server::PORT_CMD_TCP));
            }
        }

        QMetaObject::invokeMethod(commandServer, "isServiceRunning", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, isRunning));
        if(isRunning){
            //改变文字和icon
            startServerAction->setText(tr("停止服务器"));
            startServerAction->setIcon(QIcon(":/icon/通讯面板-通讯面板.png"));
        } else {
            startServerAction->setText(tr("启动服务器"));
            startServerAction->setIcon(QIcon(":/icon/通讯面板-启动服务器.png"));
        }
    }
}}
