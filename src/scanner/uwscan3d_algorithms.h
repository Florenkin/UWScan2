#pragma once

#include <cstdint>

#ifdef _WIN32
#define UWSCAN3D_API extern "C" __declspec(dllexport)
#else
#define UWSCAN3D_API extern "C"
#endif

struct UWScan3DPoint2d {
    double x;
    double y;
};

struct UWScan3DBaslerConfig {
    double exposure_time;
    double gain;
    double frame_rate;
    int pixel_format;
    int reverse_x;
    int reverse_y;
    int trigger_selector;
    int trigger_mode;
    int trigger_source;
};

struct UWScan3DReconstructionConfig {
    int horizontal_scan;
    int centerline_method;
    double bw_threshold;
    double sum_threshold;
    int use_full_roi;
    int roi_x;
    int roi_y;
    int roi_width;
    int roi_height;
};

struct UWScan3DCalibrationHandle;

enum UWScan3DStatus {
    UWSCAN3D_STATUS_OK = 0,
    UWSCAN3D_STATUS_INVALID_ARGUMENT = -1,
    UWSCAN3D_STATUS_EXCEPTION = -2,
    UWSCAN3D_STATUS_INSUFFICIENT_OUTPUT = -3
};

UWSCAN3D_API const char* uwscan3d_version();

UWSCAN3D_API int uwscan3d_default_basler_config(UWScan3DBaslerConfig* out_config);

UWSCAN3D_API int uwscan3d_clamp_basler_config(
    UWScan3DBaslerConfig* io_config,
    double exposure_min,
    double exposure_max,
    double gain_min,
    double gain_max
);

UWSCAN3D_API int uwscan3d_calibration_load(
    const char* path_utf8,
    UWScan3DCalibrationHandle** out_handle
);

UWSCAN3D_API int uwscan3d_calibration_free(UWScan3DCalibrationHandle* handle);

UWSCAN3D_API int uwscan3d_calibration_get_picture_size(
    const UWScan3DCalibrationHandle* handle,
    int* out_width,
    int* out_height
);

UWSCAN3D_API int uwscan3d_calibration_get_camera_matrix(
    const UWScan3DCalibrationHandle* handle,
    double* out_camera_matrix_9
);

UWSCAN3D_API int uwscan3d_calibration_get_dist_coeffs(
    const UWScan3DCalibrationHandle* handle,
    double* out_dist_coeffs,
    int max_values
);

UWSCAN3D_API int uwscan3d_calibration_get_mu(
    const UWScan3DCalibrationHandle* handle,
    double* out_mu_3
);

UWSCAN3D_API int uwscan3d_calibration_get_axis(
    const UWScan3DCalibrationHandle* handle,
    double* out_axis_3
);

UWSCAN3D_API int uwscan3d_calibration_get_refraction_distances(
    const UWScan3DCalibrationHandle* handle,
    double* out_d0,
    double* out_d1
);

UWSCAN3D_API int uwscan3d_calibration_get_surface_count(
    const UWScan3DCalibrationHandle* handle
);

UWSCAN3D_API int uwscan3d_calibration_get_surface_transform(
    const UWScan3DCalibrationHandle* handle,
    int surface_index,
    float* out_transform_16
);

UWSCAN3D_API int uwscan3d_calibration_get_surface_parameters(
    const UWScan3DCalibrationHandle* handle,
    int surface_index,
    float* out_parameters,
    int max_values
);

UWSCAN3D_API int uwscan3d_calibration_get_surface_z_range(
    const UWScan3DCalibrationHandle* handle,
    int surface_index,
    double* out_max_z,
    double* out_min_z
);

UWSCAN3D_API int uwscan3d_calibration_map_laser_position(
    const UWScan3DCalibrationHandle* handle,
    double laser_position,
    double scan_start_position,
    double scan_end_position,
    int scan_index
);

UWSCAN3D_API int uwscan3d_calibration_slot(
    const UWScan3DCalibrationHandle* handle,
    int calibration_index
);

UWSCAN3D_API int uwscan3d_extract_centerline_gray(
    const std::uint8_t* image_data,
    int width,
    int height,
    int stride,
    int channels,
    int horizontal_scan,
    double bw_threshold,
    double sum_threshold,
    UWScan3DPoint2d* out_points,
    int max_points
);

UWSCAN3D_API int uwscan3d_extract_centerline_steger(
    const std::uint8_t* image_data,
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
    int max_points
);

UWSCAN3D_API int uwscan3d_build_composite_image(
    const std::uint8_t* const* image_data_array,
    int image_count,
    int width,
    int height,
    int stride,
    int channels,
    std::uint8_t* out_gray_image
);

UWSCAN3D_API int uwscan3d_build_intensity_image(
    const std::uint8_t* image_data,
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
    int out_display_stride
);

UWSCAN3D_API int uwscan3d_refract_line(
    double n0,
    double n1,
    const double* ray0_xyz,
    const double* normal_xyz,
    double* out_ray1_xyz
);

UWSCAN3D_API int uwscan3d_fit_quadric_surface(
    const float* xyz_points,
    int point_count,
    float* out_coefficients_10,
    float* out_transform_16,
    double* out_max_z,
    double* out_min_z,
    double* out_mean_error
);

UWSCAN3D_API int uwscan3d_fit_quartic_surface(
    const float* xyz_points,
    int point_count,
    float* out_coefficients_35,
    float* out_transform_16,
    double* out_max_z,
    double* out_min_z,
    double* out_mean_error
);

UWSCAN3D_API int uwscan3d_intersect_quadric_surface(
    const float* coefficients_10,
    const float* ray_6,
    double max_z,
    double min_z,
    float* out_xyz
);

UWSCAN3D_API int uwscan3d_intersect_quartic_surface(
    const float* coefficients_35,
    const float* ray_6,
    double max_z,
    double min_z,
    float* out_xyz
);

UWSCAN3D_API int uwscan3d_reconstruct_frame(
    const std::uint8_t* image_data,
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
    int* out_cloud_count
);
