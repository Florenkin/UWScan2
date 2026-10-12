from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import cv2
import numpy as np

from .calibration import CalibrationData
from .image_processing import find_laser_center_gray
from .surface import intersect_quadric_surface, intersect_quartic_surface
from .underwater import refract_line


@dataclass(slots=True)
class ReconstructionConfig:
    horizontal_scan: bool = True
    centerline_method: str = "gray"
    bw_threshold: float = 55.0
    sum_threshold: float = 124.0
    use_full_roi: bool = True
    roi: Sequence[int] = field(default_factory=lambda: (0, 0, 0, 0))


def _extract_centerline(image: np.ndarray, config: ReconstructionConfig) -> np.ndarray:
    if config.centerline_method != "gray":
        raise NotImplementedError("pure Python path currently implements gray centerline only")
    return find_laser_center_gray(
        image,
        horizontal_scan=config.horizontal_scan,
        bw_threshold=config.bw_threshold,
        sum_threshold=config.sum_threshold,
    )


def reconstruct_frame(
    image: np.ndarray,
    calibration: CalibrationData,
    config: ReconstructionConfig,
    laser_position: float,
    scan_start_position: float,
    scan_end_position: float,
    scan_index: int,
    is_in_water: bool = True,
    use_quartic_surface: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    if image.size == 0:
        raise ValueError("image is empty")

    roi_image = image
    roi_offset = np.zeros(2, dtype=np.float64)
    if not config.use_full_roi and len(config.roi) >= 4:
        x, y, w, h = map(int, config.roi[:4])
        roi_image = image[y : y + h, x : x + w]
        roi_offset[:] = (x, y)

    line_points = _extract_centerline(roi_image, config)
    if line_points.shape[0] < 10:
        raise ValueError("too few line points")
    line_points = line_points + roi_offset

    distorted_points = line_points.astype(np.float64)
    undistorted = cv2.undistortPoints(
        distorted_points.reshape(-1, 1, 2),
        calibration.camera_matrix,
        calibration.dist_coeffs,
        P=calibration.camera_matrix,
    ).reshape(-1, 2)

    calibration_index = calibration.map_laser_position(
        laser_position=laser_position,
        scan_start=scan_start_position,
        scan_end=scan_end_position,
        scan_index=scan_index,
    )
    slot = calibration.calibration_slot(calibration_index)
    transform = calibration.transforms[slot]
    surface_params = calibration.surface_parameters[slot]
    max_z, min_z = calibration.z_ranges[slot]

    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    rotation_inv = rotation.T
    translation_inv = -rotation.T @ translation
    camera_inv = np.linalg.inv(calibration.camera_matrix)

    cloud = []
    for point in undistorted:
        uv = np.array([point[0], point[1], 1.0], dtype=np.float64)
        ray0 = camera_inv @ uv
        ray0 = ray0 / np.linalg.norm(ray0)

        if is_in_water:
            ray1 = refract_line(calibration.mu[0], calibration.mu[1], ray0, calibration.axis)
            ray2 = refract_line(calibration.mu[0], calibration.mu[2], ray0, calibration.axis)
            origin = (calibration.d0 / ray0[2]) * ray0 + (calibration.d1 / ray1[2]) * ray1
        else:
            ray2 = ray0
            origin = np.zeros(3, dtype=np.float64)

        local_origin = rotation @ origin + translation
        local_ray = rotation @ ray2
        local_line = np.concatenate([local_origin, local_ray]).astype(np.float64)
        if use_quartic_surface:
            local_point = intersect_quartic_surface(surface_params, local_line, max_z, min_z)
        else:
            local_point = intersect_quadric_surface(surface_params, local_line, max_z, min_z)
        if np.allclose(local_point, 0.0):
            continue
        world_point = rotation_inv @ local_point + translation_inv
        cloud.append(world_point)

    return distorted_points, np.asarray(cloud, dtype=np.float64)
