#include "scanner/uwscan3d_algorithms.h"

#include "scanner/ImgProcess.h"
#include "scanner/RuiQuadricSurfaceFitting.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace {
struct CalibrationData {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size pictureSize;
    Eigen::Vector3d mu = Eigen::Vector3d(1.0, 1.5, 1.333);
    double d0 = 20.7692;
    double d1 = 5.0;
    Eigen::Vector3d axis = Eigen::Vector3d(-0.00046551, 0.00169085, 0.999998);
    int surfBeginPos = 0;
    int surfEndPos = 0;
    int surfStep = 1;
    std::vector<Eigen::Matrix4f> transforms;
    std::vector<Eigen::VectorXf> surfaceParameters;
    std::vector<std::pair<double, double>> zRanges;
};

cv::Mat makeImageView(const std::uint8_t* imageData, int width, int height, int stride, int channels) {
    const int type = channels == 1 ? CV_8UC1 : CV_8UC3;
    return cv::Mat(height, width, type, const_cast<std::uint8_t*>(imageData), stride);
}

cv::Mat makeOutputGrayView(std::uint8_t* imageData, int width, int height, int stride) {
    return cv::Mat(height, width, CV_8UC1, imageData, stride);
}

cv::Mat makeOutputBgrView(std::uint8_t* imageData, int width, int height, int stride) {
    return cv::Mat(height, width, CV_8UC3, imageData, stride);
}

cv::Mat toGray(const cv::Mat& image) {
    if (image.channels() == 1) {
        return image;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

int copyPoints(const std::vector<cv::Point2d>& points, UWScan3DPoint2d* outPoints, int maxPoints) {
    if (!outPoints || maxPoints <= 0) {
        return static_cast<int>(points.size());
    }
    if (static_cast<int>(points.size()) > maxPoints) {
        return UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT;
    }
    for (size_t i = 0; i < points.size(); ++i) {
        outPoints[i].x = points[i].x;
        outPoints[i].y = points[i].y;
    }
    return static_cast<int>(points.size());
}

Eigen::MatrixXf makePointMatrix(const float* xyzPoints, int pointCount) {
    Eigen::MatrixXf points(pointCount, 3);
    for (int i = 0; i < pointCount; ++i) {
        points(i, 0) = xyzPoints[i * 3 + 0];
        points(i, 1) = xyzPoints[i * 3 + 1];
        points(i, 2) = xyzPoints[i * 3 + 2];
    }
    return points;
}

std::vector<Eigen::Vector3f> makePointVector(const float* xyzPoints, int pointCount) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(pointCount);
    for (int i = 0; i < pointCount; ++i) {
        points.emplace_back(xyzPoints[i * 3 + 0], xyzPoints[i * 3 + 1], xyzPoints[i * 3 + 2]);
    }
    return points;
}

void copyMatrix4RowMajor(const Eigen::Matrix4f& matrix, float* outValues) {
    if (!outValues) {
        return;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            outValues[row * 4 + col] = matrix(row, col);
        }
    }
}

double computeMeanError(const std::vector<double>& errors) {
    if (errors.empty()) {
        return 0.0;
    }
    return std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
}

int mapLaserPositionToCalibrationIndex(double laserPosition,
                                       double scanStartPosition,
                                       double scanEndPosition,
                                       int scanIndex,
                                       int surfBeginPos,
                                       int surfEndPos,
                                       int surfStep) {
    const int lower = std::min(surfBeginPos, surfEndPos);
    const int upper = std::max(surfBeginPos, surfEndPos);
    const int centiIndex = static_cast<int>(std::llround(laserPosition * 100.0));
    if (centiIndex >= lower && centiIndex <= upper) {
        return centiIndex;
    }

    const int directIndex = static_cast<int>(std::llround(laserPosition));
    if (directIndex >= lower && directIndex <= upper) {
        return directIndex;
    }

    if (std::abs(scanEndPosition - scanStartPosition) < 1e-9) {
        return scanIndex;
    }

    const double t = (laserPosition - scanStartPosition) / (scanEndPosition - scanStartPosition);
    const double rawIndex = surfBeginPos + t * (surfEndPos - surfBeginPos);
    const int step = std::max(std::abs(surfStep), 1);
    int mappedIndex = surfBeginPos + static_cast<int>(std::llround((rawIndex - surfBeginPos) / step)) * step;
    return std::clamp(mappedIndex, lower, upper);
}

uchar sampleGrayBilinear(const cv::Mat& gray, const cv::Point2d& point) {
    if (gray.empty()) {
        return 0;
    }
    const double x = std::clamp(point.x, 0.0, static_cast<double>(gray.cols - 1));
    const double y = std::clamp(point.y, 0.0, static_cast<double>(gray.rows - 1));
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, gray.cols - 1);
    const int y1 = std::min(y0 + 1, gray.rows - 1);
    const double dx = x - x0;
    const double dy = y - y0;
    const double v00 = gray.at<uchar>(y0, x0);
    const double v10 = gray.at<uchar>(y0, x1);
    const double v01 = gray.at<uchar>(y1, x0);
    const double v11 = gray.at<uchar>(y1, x1);
    const double value = (1.0 - dx) * (1.0 - dy) * v00
        + dx * (1.0 - dy) * v10
        + (1.0 - dx) * dy * v01
        + dx * dy * v11;
    return static_cast<uchar>(std::clamp(std::lround(value), 0l, 255l));
}

double centerLineCoverageRatio(const std::vector<cv::Point2d>& points,
                               const cv::Size& imageSize,
                               bool isHorizontalScan) {
    if (points.empty() || imageSize.width <= 0 || imageSize.height <= 0) {
        return 0.0;
    }

    double minCoord = std::numeric_limits<double>::max();
    double maxCoord = std::numeric_limits<double>::lowest();
    for (const auto& point : points) {
        const double coord = isHorizontalScan ? point.x : point.y;
        minCoord = std::min(minCoord, coord);
        maxCoord = std::max(maxCoord, coord);
    }

    const double fullSpan = static_cast<double>(isHorizontalScan ? imageSize.width : imageSize.height);
    if (fullSpan <= 1.0 || maxCoord < minCoord) {
        return 0.0;
    }
    return std::clamp((maxCoord - minCoord + 1.0) / fullSpan, 0.0, 1.0);
}

bool shouldUseCenterLineFallback(const std::vector<cv::Point2d>& points,
                                 const cv::Size& imageSize,
                                 bool isHorizontalScan) {
    const int expectedSpan = isHorizontalScan ? imageSize.width : imageSize.height;
    if (expectedSpan <= 0) {
        return false;
    }

    const double coverage = centerLineCoverageRatio(points, imageSize, isHorizontalScan);
    return points.size() < static_cast<size_t>(std::max(80, expectedSpan / 4)) || coverage < 0.45;
}

std::vector<cv::Point2d> extractCenterLine(const cv::Mat& image,
                                           int horizontalScan,
                                           int centerlineMethod,
                                           double bwThreshold,
                                           double sumThreshold) {
    const bool isHorizontal = horizontalScan != 0;
    std::vector<cv::Point2d> linePoints;
    if (isHorizontal) {
        if (centerlineMethod == 0) {
            linePoints = findLaserCenterGrayCenter_heng(image, bwThreshold, sumThreshold);
        } else if (centerlineMethod == 1) {
            linePoints = findLaserCenterSteger_v5_heng(image, bwThreshold, sumThreshold, 7.0, true);
        } else {
            linePoints = findLaserCenterMultiThres_heng(image, 0.0, 10, 0.0);
        }
    } else {
        if (centerlineMethod == 0) {
            linePoints = findLaserCenterGrayCenter_shu(image, bwThreshold);
        } else if (centerlineMethod == 1) {
            linePoints = findLaserCenterSteger_v5_shu(image, bwThreshold, sumThreshold, 7.0, true);
        } else {
            linePoints = findLaserCenterMultiThres_shu(image, 0.0, 10, 0.0);
        }
    }

    if (shouldUseCenterLineFallback(linePoints, image.size(), isHorizontal)) {
        const double relaxedBwThr = std::max(5.0, bwThreshold * 0.55);
        const double relaxedSumThr = std::max(10.0, sumThreshold * 0.45);
        std::vector<cv::Point2d> fallbackPoints = isHorizontal
            ? findLaserCenterGrayCenter_heng(image, relaxedBwThr, relaxedSumThr)
            : findLaserCenterGrayCenter_shu(image, relaxedBwThr);

        const double currentCoverage = centerLineCoverageRatio(linePoints, image.size(), isHorizontal);
        const double fallbackCoverage = centerLineCoverageRatio(fallbackPoints, image.size(), isHorizontal);
        if (fallbackPoints.size() > linePoints.size() && fallbackCoverage > currentCoverage) {
            linePoints = std::move(fallbackPoints);
        }
    }

    return linePoints;
}

int copyFloatVector(const Eigen::VectorXf& values, float* outValues, int maxValues) {
    if (!outValues || maxValues <= 0) {
        return static_cast<int>(values.size());
    }
    if (values.size() > maxValues) {
        return UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT;
    }
    std::memcpy(outValues, values.data(), sizeof(float) * static_cast<size_t>(values.size()));
    return static_cast<int>(values.size());
}

int copyDoubleMat(const cv::Mat& values, double* outValues, int maxValues) {
    const int total = static_cast<int>(values.total());
    if (!outValues || maxValues <= 0) {
        return total;
    }
    if (total > maxValues) {
        return UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT;
    }

    cv::Mat asDouble;
    values.convertTo(asDouble, CV_64F);
    std::memcpy(outValues, asDouble.ptr<double>(), sizeof(double) * static_cast<size_t>(total));
    return total;
}

bool loadCalibrationData(const char* pathUtf8, CalibrationData& outData) {
    if (!pathUtf8 || pathUtf8[0] == '\0') {
        return false;
    }

    cv::FileStorage fsRead;
    fsRead.open(pathUtf8, cv::FileStorage::READ);
    if (!fsRead.isOpened()) {
        return false;
    }

    fsRead["CameraMatrix"] >> outData.cameraMatrix;
    fsRead["DistCoeffs"] >> outData.distCoeffs;
    fsRead["picSize"] >> outData.pictureSize;

    if (!fsRead["mu"].empty()) {
        cv::Mat temp;
        fsRead["mu"] >> temp;
        Eigen::MatrixXd tempEig;
        cv::cv2eigen(temp, tempEig);
        outData.mu = tempEig;
    }
    if (!fsRead["d0"].empty()) {
        fsRead["d0"] >> outData.d0;
    }
    if (!fsRead["d1"].empty()) {
        fsRead["d1"] >> outData.d1;
    }
    if (!fsRead["A"].empty()) {
        cv::Mat temp;
        fsRead["A"] >> temp;
        cv::cv2eigen(temp, outData.axis);
    }

    fsRead["surfBeginPos"] >> outData.surfBeginPos;
    fsRead["surfEndPos"] >> outData.surfEndPos;
    fsRead["surfStep"] >> outData.surfStep;

    int surfNum = 0;
    fsRead["surfNum"] >> surfNum;
    cv::FileNode transNode = fsRead["allTransMats"];
    cv::FileNode surfNode = fsRead["allSurfacePars"];
    cv::FileNode zNode = fsRead["allMaxMinZValues"];
    if (surfNum <= 0) {
        surfNum = static_cast<int>(transNode.size());
    }

    outData.transforms.clear();
    outData.surfaceParameters.clear();
    outData.zRanges.clear();
    outData.transforms.reserve(surfNum);
    outData.surfaceParameters.reserve(surfNum);
    outData.zRanges.reserve(surfNum);

    for (int i = 0; i < surfNum; ++i) {
        cv::Mat trans;
        transNode[i]["TransMats"] >> trans;
        Eigen::Matrix4f transform;
        cv::cv2eigen(trans, transform);
        outData.transforms.push_back(transform);
    }

    for (int i = 0; i < surfNum; ++i) {
        cv::Mat pars;
        surfNode[i]["SurfPars"] >> pars;
        Eigen::VectorXf parameters;
        cv::cv2eigen(pars, parameters);
        outData.surfaceParameters.push_back(parameters);
    }

    for (int i = 0; i < surfNum; ++i) {
        double maxZ = 0.0;
        double minZ = 0.0;
        zNode[i]["MaxZValue"] >> maxZ;
        zNode[i]["MinZValue"] >> minZ;
        outData.zRanges.emplace_back(maxZ, minZ);
    }

    fsRead.release();
    return !outData.cameraMatrix.empty() && !outData.distCoeffs.empty() &&
        outData.transforms.size() == outData.surfaceParameters.size() &&
        outData.transforms.size() == outData.zRanges.size();
}
}

struct UWScan3DCalibrationHandle {
    CalibrationData data;
};

const char* uwscan3d_version() {
    return "0.2.0";
}

int uwscan3d_default_basler_config(UWScan3DBaslerConfig* out_config) {
    if (!out_config) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    out_config->exposure_time = 20000.0;
    out_config->gain = 0.0;
    out_config->frame_rate = 10.0;
    out_config->pixel_format = 0;
    out_config->reverse_x = 0;
    out_config->reverse_y = 0;
    out_config->trigger_selector = 0;
    out_config->trigger_mode = 0;
    out_config->trigger_source = 0;
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_clamp_basler_config(UWScan3DBaslerConfig* io_config,
                                 double exposure_min,
                                 double exposure_max,
                                 double gain_min,
                                 double gain_max) {
    if (!io_config) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    if (exposure_min <= exposure_max) {
        io_config->exposure_time = std::clamp(io_config->exposure_time, exposure_min, exposure_max);
    }
    if (gain_min <= gain_max) {
        io_config->gain = std::clamp(io_config->gain, gain_min, gain_max);
    }
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_load(const char* path_utf8, UWScan3DCalibrationHandle** out_handle) {
    if (!path_utf8 || !out_handle) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    try {
        std::unique_ptr<UWScan3DCalibrationHandle> handle(new UWScan3DCalibrationHandle());
        if (!loadCalibrationData(path_utf8, handle->data)) {
            return UWSCAN3D_STATUS_EXCEPTION;
        }
        *out_handle = handle.release();
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_calibration_free(UWScan3DCalibrationHandle* handle) {
    delete handle;
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_get_picture_size(const UWScan3DCalibrationHandle* handle, int* out_width, int* out_height) {
    if (!handle || !out_width || !out_height) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    *out_width = handle->data.pictureSize.width;
    *out_height = handle->data.pictureSize.height;
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_get_camera_matrix(const UWScan3DCalibrationHandle* handle, double* out_camera_matrix_9) {
    if (!handle) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return copyDoubleMat(handle->data.cameraMatrix, out_camera_matrix_9, 9);
}

int uwscan3d_calibration_get_dist_coeffs(const UWScan3DCalibrationHandle* handle, double* out_dist_coeffs, int max_values) {
    if (!handle) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return copyDoubleMat(handle->data.distCoeffs, out_dist_coeffs, max_values);
}

int uwscan3d_calibration_get_mu(const UWScan3DCalibrationHandle* handle, double* out_mu_3) {
    if (!handle || !out_mu_3) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    out_mu_3[0] = handle->data.mu.x();
    out_mu_3[1] = handle->data.mu.y();
    out_mu_3[2] = handle->data.mu.z();
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_get_axis(const UWScan3DCalibrationHandle* handle, double* out_axis_3) {
    if (!handle || !out_axis_3) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    out_axis_3[0] = handle->data.axis.x();
    out_axis_3[1] = handle->data.axis.y();
    out_axis_3[2] = handle->data.axis.z();
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_get_refraction_distances(const UWScan3DCalibrationHandle* handle, double* out_d0, double* out_d1) {
    if (!handle || !out_d0 || !out_d1) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    *out_d0 = handle->data.d0;
    *out_d1 = handle->data.d1;
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_get_surface_count(const UWScan3DCalibrationHandle* handle) {
    if (!handle) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return static_cast<int>(handle->data.transforms.size());
}

int uwscan3d_calibration_get_surface_transform(const UWScan3DCalibrationHandle* handle, int surface_index, float* out_transform_16) {
    if (!handle || surface_index < 0 || surface_index >= static_cast<int>(handle->data.transforms.size())) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    copyMatrix4RowMajor(handle->data.transforms[static_cast<size_t>(surface_index)], out_transform_16);
    return 16;
}

int uwscan3d_calibration_get_surface_parameters(const UWScan3DCalibrationHandle* handle,
                                                int surface_index,
                                                float* out_parameters,
                                                int max_values) {
    if (!handle || surface_index < 0 || surface_index >= static_cast<int>(handle->data.surfaceParameters.size())) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return copyFloatVector(handle->data.surfaceParameters[static_cast<size_t>(surface_index)], out_parameters, max_values);
}

int uwscan3d_calibration_get_surface_z_range(const UWScan3DCalibrationHandle* handle,
                                             int surface_index,
                                             double* out_max_z,
                                             double* out_min_z) {
    if (!handle || !out_max_z || !out_min_z ||
        surface_index < 0 || surface_index >= static_cast<int>(handle->data.zRanges.size())) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    *out_max_z = handle->data.zRanges[static_cast<size_t>(surface_index)].first;
    *out_min_z = handle->data.zRanges[static_cast<size_t>(surface_index)].second;
    return UWSCAN3D_STATUS_OK;
}

int uwscan3d_calibration_map_laser_position(const UWScan3DCalibrationHandle* handle,
                                            double laser_position,
                                            double scan_start_position,
                                            double scan_end_position,
                                            int scan_index) {
    if (!handle) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return mapLaserPositionToCalibrationIndex(
        laser_position,
        scan_start_position,
        scan_end_position,
        scan_index,
        handle->data.surfBeginPos,
        handle->data.surfEndPos,
        handle->data.surfStep
    );
}

int uwscan3d_calibration_slot(const UWScan3DCalibrationHandle* handle, int calibration_index) {
    if (!handle || handle->data.surfStep == 0) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }
    return (calibration_index - handle->data.surfBeginPos) / handle->data.surfStep;
}

int uwscan3d_extract_centerline_gray(const std::uint8_t* image_data,
                                     int width,
                                     int height,
                                     int stride,
                                     int channels,
                                     int horizontal_scan,
                                     double bw_threshold,
                                     double sum_threshold,
                                     UWScan3DPoint2d* out_points,
                                     int max_points) {
    if (!image_data || width <= 0 || height <= 0 || stride <= 0 || (channels != 1 && channels != 3)) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        const cv::Mat image = makeImageView(image_data, width, height, stride, channels).clone();
        const std::vector<cv::Point2d> points = horizontal_scan
            ? findLaserCenterGrayCenter_heng(image, bw_threshold, sum_threshold)
            : findLaserCenterGrayCenter_shu(image, bw_threshold);
        return copyPoints(points, out_points, max_points);
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_extract_centerline_steger(const std::uint8_t* image_data,
                                       int width,
                                       int height,
                                       int stride,
                                       int channels,
                                       int horizontal_scan,
                                       double bw_threshold,
                                       double slt_threshold,
                                       double stripe_width,
                                       int filter_points,
                                       UWScan3DPoint2d* out_points,
                                       int max_points) {
    if (!image_data || width <= 0 || height <= 0 || stride <= 0 || (channels != 1 && channels != 3)) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        const cv::Mat image = makeImageView(image_data, width, height, stride, channels).clone();
        const bool filter = filter_points != 0;
        const std::vector<cv::Point2d> points = horizontal_scan
            ? findLaserCenterSteger_v5_heng(image, bw_threshold, slt_threshold, stripe_width, filter)
            : findLaserCenterSteger_v5_shu(image, bw_threshold, slt_threshold, stripe_width, filter);
        return copyPoints(points, out_points, max_points);
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_build_composite_image(const std::uint8_t* const* image_data_array,
                                   int image_count,
                                   int width,
                                   int height,
                                   int stride,
                                   int channels,
                                   std::uint8_t* out_gray_image) {
    if (!image_data_array || image_count <= 0 || width <= 0 || height <= 0 || stride <= 0 ||
        (channels != 1 && channels != 3) || !out_gray_image) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        cv::Mat accum(height, width, CV_64F, cv::Scalar(0));
        for (int i = 0; i < image_count; ++i) {
            if (!image_data_array[i]) {
                return UWSCAN3D_STATUS_INVALID_ARGUMENT;
            }
            const cv::Mat image = makeImageView(image_data_array[i], width, height, stride, channels);
            const cv::Mat gray = toGray(image);
            accum += gray;
        }

        cv::Mat composite;
        accum.convertTo(composite, CV_8UC1, 1.0 / static_cast<double>(image_count));
        cv::equalizeHist(composite, composite);
        std::memcpy(out_gray_image, composite.data, static_cast<size_t>(width * height));
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_build_intensity_image(const std::uint8_t* image_data,
                                   int width,
                                   int height,
                                   int stride,
                                   int channels,
                                   const UWScan3DPoint2d* line_points,
                                   int point_count,
                                   const std::uint8_t* previous_accum_gray,
                                   int previous_stride,
                                   std::uint8_t* out_accum_gray,
                                   int out_accum_stride,
                                   std::uint8_t* out_display_bgr,
                                   int out_display_stride) {
    if (!image_data || width <= 0 || height <= 0 || stride <= 0 || (channels != 1 && channels != 3) ||
        !line_points || point_count < 0 || !out_accum_gray || out_accum_stride <= 0 ||
        !out_display_bgr || out_display_stride <= 0) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        const cv::Mat image = makeImageView(image_data, width, height, stride, channels);
        const cv::Mat gray = toGray(image);
        cv::Mat accum = makeOutputGrayView(out_accum_gray, width, height, out_accum_stride);
        if (previous_accum_gray) {
            makeOutputGrayView(const_cast<std::uint8_t*>(previous_accum_gray), width, height, previous_stride).copyTo(accum);
        } else {
            accum.setTo(cv::Scalar(0));
        }

        cv::Mat frameIntensity(height, width, CV_8UC1, cv::Scalar(0));
        for (int i = 0; i < point_count; ++i) {
            const int x = static_cast<int>(std::lround(line_points[i].x));
            const int y = static_cast<int>(std::lround(line_points[i].y));
            if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) {
                continue;
            }
            const uchar value = gray.at<uchar>(y, x);
            uchar& dst = frameIntensity.at<uchar>(y, x);
            dst = std::max(dst, value);
        }

        cv::max(accum, frameIntensity, accum);

        double maxValue = 0.0;
        cv::minMaxLoc(accum, nullptr, &maxValue);
        cv::Mat displayGray;
        if (maxValue > 0.0) {
            accum.convertTo(displayGray, CV_8UC1, 255.0 / maxValue);
        } else {
            displayGray = accum.clone();
        }
        cv::threshold(displayGray, displayGray, 64, 255, cv::THRESH_TOZERO);

        cv::Mat displayColor = makeOutputBgrView(out_display_bgr, width, height, out_display_stride);
        displayColor.setTo(cv::Scalar(0, 0, 0));
        for (int y = 0; y < displayGray.rows; ++y) {
            const uchar* row = displayGray.ptr<uchar>(y);
            for (int x = 0; x < displayGray.cols; ++x) {
                const uchar intensity = row[x];
                if (intensity == 0) {
                    continue;
                }
                const int green = std::clamp(120 + static_cast<int>(intensity) / 2, 0, 255);
                displayColor.at<cv::Vec3b>(y, x) = cv::Vec3b(0, static_cast<uchar>(green), 0);
            }
        }
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_refract_line(double n0,
                          double n1,
                          const double* ray0_xyz,
                          const double* normal_xyz,
                          double* out_ray1_xyz) {
    if (!ray0_xyz || !normal_xyz || !out_ray1_xyz) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        Eigen::Vector3d ray0(ray0_xyz[0], ray0_xyz[1], ray0_xyz[2]);
        Eigen::Vector3d normal(normal_xyz[0], normal_xyz[1], normal_xyz[2]);
        Eigen::Vector3d ray1;
        if (!refract_line(n0, n1, ray0, normal, ray1)) {
            return UWSCAN3D_STATUS_EXCEPTION;
        }
        out_ray1_xyz[0] = ray1.x();
        out_ray1_xyz[1] = ray1.y();
        out_ray1_xyz[2] = ray1.z();
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_fit_quadric_surface(const float* xyz_points,
                                 int point_count,
                                 float* out_coefficients_10,
                                 float* out_transform_16,
                                 double* out_max_z,
                                 double* out_min_z,
                                 double* out_mean_error) {
    if (!xyz_points || point_count < 6 || !out_coefficients_10) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        Eigen::VectorXf coefficients;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        std::vector<double> errors;
        double maxZ = 0.0;
        double minZ = 0.0;

        Rui::GetOptimalQuadricSurfCoeffs(makePointVector(xyz_points, point_count), coefficients, errors, transform, maxZ, minZ);
        std::memcpy(out_coefficients_10, coefficients.data(), sizeof(float) * 10);
        if (out_transform_16) {
            copyMatrix4RowMajor(transform, out_transform_16);
        }
        if (out_max_z) {
            *out_max_z = maxZ;
        }
        if (out_min_z) {
            *out_min_z = minZ;
        }
        if (out_mean_error) {
            *out_mean_error = computeMeanError(errors);
        }
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_fit_quartic_surface(const float* xyz_points,
                                 int point_count,
                                 float* out_coefficients_35,
                                 float* out_transform_16,
                                 double* out_max_z,
                                 double* out_min_z,
                                 double* out_mean_error) {
    if (!xyz_points || point_count < 10 || !out_coefficients_35) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        const Eigen::MatrixXf points = makePointMatrix(xyz_points, point_count);
        Eigen::VectorXf coefficients;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        std::vector<double> errors;
        double maxZ = 0.0;
        double minZ = 0.0;

        Rui::QuarticSurfFittingAll_newForm(points, coefficients, errors, transform, maxZ, minZ);
        std::memcpy(out_coefficients_35, coefficients.data(), sizeof(float) * 35);
        if (out_transform_16) {
            copyMatrix4RowMajor(transform, out_transform_16);
        }
        if (out_max_z) {
            *out_max_z = maxZ;
        }
        if (out_min_z) {
            *out_min_z = minZ;
        }
        if (out_mean_error) {
            *out_mean_error = computeMeanError(errors);
        }
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_intersect_quadric_surface(const float* coefficients_10,
                                       const float* ray_6,
                                       double max_z,
                                       double min_z,
                                       float* out_xyz) {
    if (!coefficients_10 || !ray_6 || !out_xyz) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        Eigen::VectorXf coefficients(10);
        Eigen::VectorXf ray(6);
        for (int i = 0; i < 10; ++i) {
            coefficients(i) = coefficients_10[i];
        }
        for (int i = 0; i < 6; ++i) {
            ray(i) = ray_6[i];
        }

        Eigen::Vector3f point = Eigen::Vector3f::Zero();
        Rui::GetQuadricSurfIntersectPtNewForm(coefficients, ray, point, max_z, min_z);
        out_xyz[0] = point.x();
        out_xyz[1] = point.y();
        out_xyz[2] = point.z();
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_intersect_quartic_surface(const float* coefficients_35,
                                       const float* ray_6,
                                       double max_z,
                                       double min_z,
                                       float* out_xyz) {
    if (!coefficients_35 || !ray_6 || !out_xyz) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        Eigen::VectorXf coefficients(35);
        Eigen::VectorXf ray(6);
        for (int i = 0; i < 35; ++i) {
            coefficients(i) = coefficients_35[i];
        }
        for (int i = 0; i < 6; ++i) {
            ray(i) = ray_6[i];
        }

        Eigen::Vector3f point = Eigen::Vector3f::Zero();
        Rui::GetQuarticSurfIntersectPtNewForm(coefficients, ray, point, max_z, min_z);
        out_xyz[0] = point.x();
        out_xyz[1] = point.y();
        out_xyz[2] = point.z();
        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}

int uwscan3d_reconstruct_frame(const std::uint8_t* image_data,
                               int width,
                               int height,
                               int stride,
                               int channels,
                               const UWScan3DCalibrationHandle* calibration,
                               const UWScan3DReconstructionConfig* config,
                               double laser_position,
                               double scan_start_position,
                               double scan_end_position,
                               int scan_index,
                               int is_in_water,
                               int use_quartic_surface,
                               UWScan3DPoint2d* out_distorted_points,
                               int max_distorted_points,
                               int* out_distorted_count,
                               float* out_cloud_xyz,
                               int max_cloud_points,
                               int* out_cloud_count) {
    if (!image_data || width <= 0 || height <= 0 || stride <= 0 || (channels != 1 && channels != 3) ||
        !calibration || !config || !out_distorted_count || !out_cloud_count) {
        return UWSCAN3D_STATUS_INVALID_ARGUMENT;
    }

    try {
        const cv::Mat image = makeImageView(image_data, width, height, stride, channels).clone();
        cv::Mat grayImage = toGray(image);

        cv::Rect roiRect(0, 0, width, height);
        if (config->use_full_roi == 0) {
            roiRect = cv::Rect(config->roi_x, config->roi_y, config->roi_width, config->roi_height);
            roiRect &= cv::Rect(0, 0, width, height);
            if (roiRect.empty()) {
                return UWSCAN3D_STATUS_INVALID_ARGUMENT;
            }
        }

        const cv::Mat roiImage = image(roiRect).clone();
        std::vector<cv::Point2d> linePoints = extractCenterLine(
            roiImage,
            config->horizontal_scan,
            config->centerline_method,
            config->bw_threshold,
            config->sum_threshold
        );
        if (linePoints.size() < 10) {
            return UWSCAN3D_STATUS_EXCEPTION;
        }

        for (auto& point : linePoints) {
            point.x += roiRect.x;
            point.y += roiRect.y;
        }

        const std::vector<cv::Point2d> distortedPoints = linePoints;
        *out_distorted_count = static_cast<int>(distortedPoints.size());
        if (out_distorted_points && max_distorted_points > 0) {
            if (*out_distorted_count > max_distorted_points) {
                return UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT;
            }
            const int copied = copyPoints(distortedPoints, out_distorted_points, max_distorted_points);
            if (copied < 0) {
                return copied;
            }
        }

        cv::undistortPoints(linePoints, linePoints, calibration->data.cameraMatrix, calibration->data.distCoeffs, cv::Mat(), calibration->data.cameraMatrix);

        const int calibrationIndex = mapLaserPositionToCalibrationIndex(
            laser_position,
            scan_start_position,
            scan_end_position,
            scan_index,
            calibration->data.surfBeginPos,
            calibration->data.surfEndPos,
            calibration->data.surfStep
        );
        const int slot = (calibrationIndex - calibration->data.surfBeginPos) / calibration->data.surfStep;
        if (slot < 0 || slot >= static_cast<int>(calibration->data.transforms.size())) {
            return UWSCAN3D_STATUS_EXCEPTION;
        }

        const Eigen::Matrix4f& transform = calibration->data.transforms[static_cast<size_t>(slot)];
        const Eigen::VectorXf& surfaceParameters = calibration->data.surfaceParameters[static_cast<size_t>(slot)];
        const double maxZ = calibration->data.zRanges[static_cast<size_t>(slot)].first;
        const double minZ = calibration->data.zRanges[static_cast<size_t>(slot)].second;

        const Eigen::Matrix3f transR = transform.block<3, 3>(0, 0);
        const Eigen::Vector3f transT = transform.block<3, 1>(0, 3);
        const Eigen::Matrix3f transRInv = transR.transpose();
        const Eigen::Vector3f transTInv = -transR.transpose() * transT;

        Eigen::Matrix3d cameraMatrixEig;
        cv::cv2eigen(calibration->data.cameraMatrix, cameraMatrixEig);
        const Eigen::Matrix3d cameraMatrixInv = cameraMatrixEig.inverse();

        Eigen::MatrixXf vpVecMat = Eigen::MatrixXf::Zero(3, static_cast<int>(linePoints.size()));
        Eigen::MatrixXf vpPosMat = Eigen::MatrixXf::Zero(3, static_cast<int>(linePoints.size()));

        for (size_t i = 0; i < linePoints.size(); ++i) {
            const Eigen::Vector3d tempUv(linePoints[i].x, linePoints[i].y, 1.0);
            Eigen::Vector3d v0 = cameraMatrixInv * tempUv;
            v0.normalize();

            if (is_in_water != 0) {
                Eigen::Vector3d v1;
                Eigen::Vector3d v2;
                if (!refract_line(calibration->data.mu[0], calibration->data.mu[1], v0, calibration->data.axis, v1) ||
                    !refract_line(calibration->data.mu[0], calibration->data.mu[2], v0, calibration->data.axis, v2)) {
                    continue;
                }
                const Eigen::Vector3d q2 = (calibration->data.d0 / v0[2]) * v0 + (calibration->data.d1 / v1[2]) * v1;
                vpVecMat.col(static_cast<int>(i)) = v2.cast<float>();
                vpPosMat.col(static_cast<int>(i)) = q2.cast<float>();
            } else {
                vpVecMat.col(static_cast<int>(i)) = v0.cast<float>();
                vpPosMat.col(static_cast<int>(i)) = Eigen::Vector3f::Zero();
            }
        }

        vpVecMat = transR * vpVecMat;
        vpPosMat = transR * vpPosMat + transT * Eigen::MatrixXf::Ones(1, static_cast<int>(linePoints.size()));

        std::vector<Eigen::Vector3f> intersectPoints;
        intersectPoints.reserve(linePoints.size());
        std::vector<size_t> validIndices;
        validIndices.reserve(linePoints.size());

        for (int i = 0; i < vpVecMat.cols(); ++i) {
            Eigen::VectorXf ray(6);
            ray << vpPosMat.col(i), vpVecMat.col(i);
            Eigen::Vector3f point = Eigen::Vector3f::Zero();
            if (use_quartic_surface != 0) {
                Rui::GetQuarticSurfIntersectPtNewForm(surfaceParameters, ray, point, maxZ, minZ);
            } else {
                Rui::GetQuadricSurfIntersectPtNewForm(surfaceParameters, ray, point, maxZ, minZ);
            }
            if (std::abs(point.x()) < 1e-6f && std::abs(point.y()) < 1e-6f && std::abs(point.z()) < 1e-6f) {
                continue;
            }
            intersectPoints.push_back(point);
            validIndices.push_back(static_cast<size_t>(i));
        }

        *out_cloud_count = static_cast<int>(intersectPoints.size());
        if (out_cloud_xyz && max_cloud_points > 0) {
            if (*out_cloud_count > max_cloud_points) {
                return UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT;
            }
        }

        for (size_t i = 0; i < intersectPoints.size(); ++i) {
            const Eigen::Vector3f worldPoint = transRInv * intersectPoints[i] + transTInv;
            if (out_cloud_xyz && max_cloud_points > 0) {
                out_cloud_xyz[i * 3 + 0] = worldPoint.x();
                out_cloud_xyz[i * 3 + 1] = worldPoint.y();
                out_cloud_xyz[i * 3 + 2] = worldPoint.z();
            }
            (void)sampleGrayBilinear(grayImage, distortedPoints[validIndices[i]]);
        }

        return UWSCAN3D_STATUS_OK;
    } catch (const std::exception&) {
        return UWSCAN3D_STATUS_EXCEPTION;
    }
}
