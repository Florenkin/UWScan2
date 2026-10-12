from __future__ import annotations

from importlib import import_module

__all__ = [
    "BaslerCameraConfig",
    "CalibrationData",
    "ReconstructionConfig",
    "UWScan3DDll",
    "build_composite_image",
    "build_intensity_image",
    "find_laser_center_gray",
    "fit_quadric_surface",
    "fit_quartic_surface",
    "intersect_quadric_surface",
    "intersect_quartic_surface",
    "preprocess_laser_mask",
    "reconstruct_frame",
    "refract_line",
]

_EXPORT_MAP = {
    "BaslerCameraConfig": (".basler", "BaslerCameraConfig"),
    "CalibrationData": (".calibration", "CalibrationData"),
    "ReconstructionConfig": (".reconstruction", "ReconstructionConfig"),
    "UWScan3DDll": (".dll", "UWScan3DDll"),
    "build_composite_image": (".image_processing", "build_composite_image"),
    "build_intensity_image": (".image_processing", "build_intensity_image"),
    "find_laser_center_gray": (".image_processing", "find_laser_center_gray"),
    "fit_quadric_surface": (".surface", "fit_quadric_surface"),
    "fit_quartic_surface": (".surface", "fit_quartic_surface"),
    "intersect_quadric_surface": (".surface", "intersect_quadric_surface"),
    "intersect_quartic_surface": (".surface", "intersect_quartic_surface"),
    "preprocess_laser_mask": (".image_processing", "preprocess_laser_mask"),
    "reconstruct_frame": (".reconstruction", "reconstruct_frame"),
    "refract_line": (".underwater", "refract_line"),
}


def __getattr__(name: str):
    if name not in _EXPORT_MAP:
        raise AttributeError(name)
    module_name, attr_name = _EXPORT_MAP[name]
    module = import_module(module_name, __name__)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value
