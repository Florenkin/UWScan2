from __future__ import annotations

import ctypes
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np


class _Point2D(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class _BaslerConfig(ctypes.Structure):
    _fields_ = [
        ("exposure_time", ctypes.c_double),
        ("gain", ctypes.c_double),
        ("frame_rate", ctypes.c_double),
        ("pixel_format", ctypes.c_int),
        ("reverse_x", ctypes.c_int),
        ("reverse_y", ctypes.c_int),
        ("trigger_selector", ctypes.c_int),
        ("trigger_mode", ctypes.c_int),
        ("trigger_source", ctypes.c_int),
    ]


class _ReconstructionConfig(ctypes.Structure):
    _fields_ = [
        ("horizontal_scan", ctypes.c_int),
        ("centerline_method", ctypes.c_int),
        ("bw_threshold", ctypes.c_double),
        ("sum_threshold", ctypes.c_double),
        ("use_full_roi", ctypes.c_int),
        ("roi_x", ctypes.c_int),
        ("roi_y", ctypes.c_int),
        ("roi_width", ctypes.c_int),
        ("roi_height", ctypes.c_int),
    ]


@dataclass(slots=True)
class DllSurfaceFitResult:
    coefficients: np.ndarray
    transform: np.ndarray
    max_z: float
    min_z: float
    mean_error: float


@dataclass(slots=True)
class UWScan3DCalibration:
    _dll: "UWScan3DDll"
    handle: ctypes.c_void_p
    path: Path

    def close(self) -> None:
        if self.handle:
            self._dll._dll.uwscan3d_calibration_free(self.handle)
            self.handle = ctypes.c_void_p()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    @property
    def picture_size(self) -> tuple[int, int]:
        width = ctypes.c_int()
        height = ctypes.c_int()
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_picture_size(self.handle, ctypes.byref(width), ctypes.byref(height))
        )
        return int(width.value), int(height.value)

    @property
    def camera_matrix(self) -> np.ndarray:
        values = np.zeros(9, dtype=np.float64)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_camera_matrix(
                self.handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        return values.reshape(3, 3)

    @property
    def dist_coeffs(self) -> np.ndarray:
        count = int(self._dll._dll.uwscan3d_calibration_get_dist_coeffs(self.handle, None, 0))
        values = np.zeros(count, dtype=np.float64)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_dist_coeffs(
                self.handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                count,
            )
        )
        return values

    @property
    def mu(self) -> np.ndarray:
        values = np.zeros(3, dtype=np.float64)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_mu(
                self.handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        return values

    @property
    def axis(self) -> np.ndarray:
        values = np.zeros(3, dtype=np.float64)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_axis(
                self.handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        return values

    @property
    def d0(self) -> float:
        d0 = ctypes.c_double()
        d1 = ctypes.c_double()
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_refraction_distances(
                self.handle,
                ctypes.byref(d0),
                ctypes.byref(d1),
            )
        )
        return float(d0.value)

    @property
    def d1(self) -> float:
        d0 = ctypes.c_double()
        d1 = ctypes.c_double()
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_refraction_distances(
                self.handle,
                ctypes.byref(d0),
                ctypes.byref(d1),
            )
        )
        return float(d1.value)

    @property
    def surf_num(self) -> int:
        return int(self._dll._check_status(self._dll._dll.uwscan3d_calibration_get_surface_count(self.handle)))

    def transform(self, surface_index: int) -> np.ndarray:
        values = np.zeros(16, dtype=np.float32)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_surface_transform(
                self.handle,
                int(surface_index),
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            )
        )
        return values.reshape(4, 4)

    def surface_parameters(self, surface_index: int) -> np.ndarray:
        count = int(
            self._dll._check_status(
                self._dll._dll.uwscan3d_calibration_get_surface_parameters(self.handle, int(surface_index), None, 0)
            )
        )
        values = np.zeros(count, dtype=np.float32)
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_surface_parameters(
                self.handle,
                int(surface_index),
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                count,
            )
        )
        return values

    def z_range(self, surface_index: int) -> tuple[float, float]:
        max_z = ctypes.c_double()
        min_z = ctypes.c_double()
        self._dll._check_status(
            self._dll._dll.uwscan3d_calibration_get_surface_z_range(
                self.handle,
                int(surface_index),
                ctypes.byref(max_z),
                ctypes.byref(min_z),
            )
        )
        return float(max_z.value), float(min_z.value)

    def map_laser_position(
        self,
        laser_position: float,
        scan_start: float,
        scan_end: float,
        scan_index: int,
    ) -> int:
        return int(
            self._dll._check_status(
                self._dll._dll.uwscan3d_calibration_map_laser_position(
                    self.handle,
                    float(laser_position),
                    float(scan_start),
                    float(scan_end),
                    int(scan_index),
                )
            )
        )

    def calibration_slot(self, calibration_index: int) -> int:
        return int(
            self._dll._check_status(
                self._dll._dll.uwscan3d_calibration_slot(self.handle, int(calibration_index))
            )
        )


@dataclass(slots=True)
class UWScan3DDll:
    path: Path
    _dll: ctypes.CDLL = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._dll = ctypes.CDLL(str(self.path))

        self._dll.uwscan3d_version.argtypes = []
        self._dll.uwscan3d_version.restype = ctypes.c_char_p

        self._dll.uwscan3d_default_basler_config.argtypes = [ctypes.POINTER(_BaslerConfig)]
        self._dll.uwscan3d_default_basler_config.restype = ctypes.c_int
        self._dll.uwscan3d_clamp_basler_config.argtypes = [
            ctypes.POINTER(_BaslerConfig),
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
        ]
        self._dll.uwscan3d_clamp_basler_config.restype = ctypes.c_int

        self._dll.uwscan3d_calibration_load.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        self._dll.uwscan3d_calibration_load.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_free.argtypes = [ctypes.c_void_p]
        self._dll.uwscan3d_calibration_free.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_picture_size.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        self._dll.uwscan3d_calibration_get_picture_size.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_camera_matrix.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_calibration_get_camera_matrix.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_dist_coeffs.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_int,
        ]
        self._dll.uwscan3d_calibration_get_dist_coeffs.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_mu.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
        self._dll.uwscan3d_calibration_get_mu.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_axis.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
        self._dll.uwscan3d_calibration_get_axis.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_refraction_distances.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_calibration_get_refraction_distances.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_surface_count.argtypes = [ctypes.c_void_p]
        self._dll.uwscan3d_calibration_get_surface_count.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_surface_transform.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._dll.uwscan3d_calibration_get_surface_transform.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_surface_parameters.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
        ]
        self._dll.uwscan3d_calibration_get_surface_parameters.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_get_surface_z_range.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_calibration_get_surface_z_range.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_map_laser_position.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int,
        ]
        self._dll.uwscan3d_calibration_map_laser_position.restype = ctypes.c_int
        self._dll.uwscan3d_calibration_slot.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._dll.uwscan3d_calibration_slot.restype = ctypes.c_int

        self._dll.uwscan3d_extract_centerline_gray.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(_Point2D),
            ctypes.c_int,
        ]
        self._dll.uwscan3d_extract_centerline_gray.restype = ctypes.c_int
        self._dll.uwscan3d_extract_centerline_steger.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int,
            ctypes.POINTER(_Point2D),
            ctypes.c_int,
        ]
        self._dll.uwscan3d_extract_centerline_steger.restype = ctypes.c_int
        self._dll.uwscan3d_build_composite_image.argtypes = [
            ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self._dll.uwscan3d_build_composite_image.restype = ctypes.c_int
        self._dll.uwscan3d_build_intensity_image.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(_Point2D),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
        ]
        self._dll.uwscan3d_build_intensity_image.restype = ctypes.c_int

        self._dll.uwscan3d_refract_line.argtypes = [
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_refract_line.restype = ctypes.c_int

        self._dll.uwscan3d_fit_quadric_surface.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_fit_quadric_surface.restype = ctypes.c_int
        self._dll.uwscan3d_fit_quartic_surface.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self._dll.uwscan3d_fit_quartic_surface.restype = ctypes.c_int
        self._dll.uwscan3d_intersect_quadric_surface.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._dll.uwscan3d_intersect_quadric_surface.restype = ctypes.c_int
        self._dll.uwscan3d_intersect_quartic_surface.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._dll.uwscan3d_intersect_quartic_surface.restype = ctypes.c_int

        self._dll.uwscan3d_reconstruct_frame.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.POINTER(_ReconstructionConfig),
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(_Point2D),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]
        self._dll.uwscan3d_reconstruct_frame.restype = ctypes.c_int

    @classmethod
    def discover(cls, root: str | Path) -> "UWScan3DDll":
        root_path = Path(root)
        candidates = [
            root_path / "bin" / "UWScan3DAlgorithms.dll",
            root_path / "lib" / "UWScan3DAlgorithms.dll",
            root_path / "build" / "src" / "UWScan3DAlgorithms.dll",
        ]
        for candidate in candidates:
            if candidate.exists():
                return cls(candidate)
        raise FileNotFoundError("UWScan3DAlgorithms.dll not found")

    def _check_status(self, status: int) -> int:
        if status < 0:
            raise RuntimeError(f"dll call failed: {status}")
        return status

    def version(self) -> str:
        return self._dll.uwscan3d_version().decode("utf-8")

    def default_basler_config(self) -> dict[str, float | int]:
        config = _BaslerConfig()
        self._check_status(self._dll.uwscan3d_default_basler_config(ctypes.byref(config)))
        return {
            "exposure_time": float(config.exposure_time),
            "gain": float(config.gain),
            "frame_rate": float(config.frame_rate),
            "pixel_format": int(config.pixel_format),
            "reverse_x": int(config.reverse_x),
            "reverse_y": int(config.reverse_y),
            "trigger_selector": int(config.trigger_selector),
            "trigger_mode": int(config.trigger_mode),
            "trigger_source": int(config.trigger_source),
        }

    def clamp_basler_config(
        self,
        config: dict[str, float | int],
        exposure_range: tuple[float, float] | None = None,
        gain_range: tuple[float, float] | None = None,
    ) -> dict[str, float | int]:
        native = _BaslerConfig(
            exposure_time=float(config.get("exposure_time", 20000.0)),
            gain=float(config.get("gain", 0.0)),
            frame_rate=float(config.get("frame_rate", 10.0)),
            pixel_format=int(config.get("pixel_format", 0)),
            reverse_x=int(config.get("reverse_x", 0)),
            reverse_y=int(config.get("reverse_y", 0)),
            trigger_selector=int(config.get("trigger_selector", 0)),
            trigger_mode=int(config.get("trigger_mode", 0)),
            trigger_source=int(config.get("trigger_source", 0)),
        )
        exp_min, exp_max = exposure_range or (1.0, 1.0)
        gain_min, gain_max = gain_range or (0.0, 0.0)
        self._check_status(
            self._dll.uwscan3d_clamp_basler_config(
                ctypes.byref(native),
                float(exp_min),
                float(exp_max),
                float(gain_min),
                float(gain_max),
            )
        )
        return {
            "exposure_time": float(native.exposure_time),
            "gain": float(native.gain),
            "frame_rate": float(native.frame_rate),
            "pixel_format": int(native.pixel_format),
            "reverse_x": int(native.reverse_x),
            "reverse_y": int(native.reverse_y),
            "trigger_selector": int(native.trigger_selector),
            "trigger_mode": int(native.trigger_mode),
            "trigger_source": int(native.trigger_source),
        }

    def load_calibration(self, path: str | Path) -> UWScan3DCalibration:
        handle = ctypes.c_void_p()
        self._check_status(
            self._dll.uwscan3d_calibration_load(
                str(path).encode("utf-8"),
                ctypes.byref(handle),
            )
        )
        return UWScan3DCalibration(self, handle, Path(path))

    def _as_image_u8(self, image: np.ndarray) -> tuple[np.ndarray, int, int, int]:
        image_u8 = np.ascontiguousarray(image, dtype=np.uint8)
        if image_u8.ndim == 2:
            height, width = image_u8.shape
            channels = 1
        elif image_u8.ndim == 3 and image_u8.shape[2] in (1, 3):
            height, width, channels = image_u8.shape
        else:
            raise ValueError("image must be grayscale or BGR uint8")
        return image_u8, height, width, channels

    def extract_centerline_gray(
        self,
        image: np.ndarray,
        horizontal_scan: bool = True,
        bw_threshold: float = 55.0,
        sum_threshold: float = 124.0,
        max_points: Optional[int] = None,
    ) -> np.ndarray:
        image_u8, height, width, channels = self._as_image_u8(image)
        capacity = max_points or (width if horizontal_scan else height) * 4
        out_points = (_Point2D * capacity)()
        status = self._dll.uwscan3d_extract_centerline_gray(
            image_u8.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            width,
            height,
            int(image_u8.strides[0]),
            channels,
            1 if horizontal_scan else 0,
            bw_threshold,
            sum_threshold,
            out_points,
            capacity,
        )
        count = self._check_status(status)
        return np.asarray([(out_points[i].x, out_points[i].y) for i in range(count)], dtype=np.float64)

    def extract_centerline_steger(
        self,
        image: np.ndarray,
        horizontal_scan: bool = True,
        bw_threshold: float = 55.0,
        slt_threshold: float = 124.0,
        stripe_width: float = 7.0,
        filter_points: bool = True,
        max_points: Optional[int] = None,
    ) -> np.ndarray:
        image_u8, height, width, channels = self._as_image_u8(image)
        capacity = max_points or (width if horizontal_scan else height) * 4
        out_points = (_Point2D * capacity)()
        status = self._dll.uwscan3d_extract_centerline_steger(
            image_u8.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            width,
            height,
            int(image_u8.strides[0]),
            channels,
            1 if horizontal_scan else 0,
            bw_threshold,
            slt_threshold,
            stripe_width,
            1 if filter_points else 0,
            out_points,
            capacity,
        )
        count = self._check_status(status)
        return np.asarray([(out_points[i].x, out_points[i].y) for i in range(count)], dtype=np.float64)

    def build_composite_image(self, images: Sequence[np.ndarray]) -> np.ndarray:
        if not images:
            raise ValueError("images is empty")
        normalized = [self._as_image_u8(image)[0] for image in images]
        first = normalized[0]
        if any(image.shape != first.shape for image in normalized):
            raise ValueError("all images must have the same shape for DLL composite")
        height, width = first.shape[:2]
        channels = 1 if first.ndim == 2 else first.shape[2]
        ptrs = (ctypes.POINTER(ctypes.c_uint8) * len(normalized))(
            *[image.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)) for image in normalized]
        )
        out_gray = np.zeros((height, width), dtype=np.uint8)
        self._check_status(
            self._dll.uwscan3d_build_composite_image(
                ptrs,
                len(normalized),
                width,
                height,
                int(first.strides[0]),
                channels,
                out_gray.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            )
        )
        return out_gray

    def build_intensity_image(
        self,
        image: np.ndarray,
        line_points: np.ndarray,
        previous: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        image_u8, height, width, channels = self._as_image_u8(image)
        points = np.ascontiguousarray(line_points, dtype=np.float64).reshape(-1, 2)
        native_points = (_Point2D * len(points))(*[_Point2D(float(x), float(y)) for x, y in points])
        previous_accum = None
        previous_stride = 0
        if previous is not None:
            previous_accum = np.ascontiguousarray(previous, dtype=np.uint8)
            previous_stride = int(previous_accum.strides[0])
        out_accum = np.zeros((height, width), dtype=np.uint8)
        out_display = np.zeros((height, width, 3), dtype=np.uint8)
        self._check_status(
            self._dll.uwscan3d_build_intensity_image(
                image_u8.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                width,
                height,
                int(image_u8.strides[0]),
                channels,
                native_points,
                len(points),
                None if previous_accum is None else previous_accum.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                previous_stride,
                out_accum.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                int(out_accum.strides[0]),
                out_display.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                int(out_display.strides[0]),
            )
        )
        return out_accum, out_display

    def refract_line(self, n0: float, n1: float, ray0_xyz: Iterable[float], normal_xyz: Iterable[float]) -> np.ndarray:
        ray0 = np.ascontiguousarray(list(ray0_xyz), dtype=np.float64).reshape(3)
        normal = np.ascontiguousarray(list(normal_xyz), dtype=np.float64).reshape(3)
        out = np.zeros(3, dtype=np.float64)
        self._check_status(
            self._dll.uwscan3d_refract_line(
                float(n0),
                float(n1),
                ray0.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                normal.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            )
        )
        return out

    def fit_quadric_surface(self, points: np.ndarray) -> DllSurfaceFitResult:
        pts = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
        coeffs = np.zeros(10, dtype=np.float32)
        transform = np.zeros(16, dtype=np.float32)
        max_z = ctypes.c_double()
        min_z = ctypes.c_double()
        mean_error = ctypes.c_double()
        self._check_status(
            self._dll.uwscan3d_fit_quadric_surface(
                pts.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                pts.shape[0],
                coeffs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                transform.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ctypes.byref(max_z),
                ctypes.byref(min_z),
                ctypes.byref(mean_error),
            )
        )
        return DllSurfaceFitResult(coeffs, transform.reshape(4, 4), float(max_z.value), float(min_z.value), float(mean_error.value))

    def fit_quartic_surface(self, points: np.ndarray) -> DllSurfaceFitResult:
        pts = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
        coeffs = np.zeros(35, dtype=np.float32)
        transform = np.zeros(16, dtype=np.float32)
        max_z = ctypes.c_double()
        min_z = ctypes.c_double()
        mean_error = ctypes.c_double()
        self._check_status(
            self._dll.uwscan3d_fit_quartic_surface(
                pts.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                pts.shape[0],
                coeffs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                transform.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ctypes.byref(max_z),
                ctypes.byref(min_z),
                ctypes.byref(mean_error),
            )
        )
        return DllSurfaceFitResult(coeffs, transform.reshape(4, 4), float(max_z.value), float(min_z.value), float(mean_error.value))

    def intersect_quadric_surface(self, coefficients: np.ndarray, ray: np.ndarray, max_z: float, min_z: float) -> np.ndarray:
        coeffs = np.ascontiguousarray(coefficients, dtype=np.float32).reshape(10)
        ray6 = np.ascontiguousarray(ray, dtype=np.float32).reshape(6)
        out = np.zeros(3, dtype=np.float32)
        self._check_status(
            self._dll.uwscan3d_intersect_quadric_surface(
                coeffs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ray6.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                float(max_z),
                float(min_z),
                out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            )
        )
        return out.astype(np.float64)

    def intersect_quartic_surface(self, coefficients: np.ndarray, ray: np.ndarray, max_z: float, min_z: float) -> np.ndarray:
        coeffs = np.ascontiguousarray(coefficients, dtype=np.float32).reshape(35)
        ray6 = np.ascontiguousarray(ray, dtype=np.float32).reshape(6)
        out = np.zeros(3, dtype=np.float32)
        self._check_status(
            self._dll.uwscan3d_intersect_quartic_surface(
                coeffs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ray6.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                float(max_z),
                float(min_z),
                out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            )
        )
        return out.astype(np.float64)

    def reconstruct_frame(
        self,
        image: np.ndarray,
        calibration: UWScan3DCalibration,
        config: dict[str, object],
        laser_position: float,
        scan_start_position: float,
        scan_end_position: float,
        scan_index: int,
        is_in_water: bool = True,
        use_quartic_surface: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        image_u8, height, width, channels = self._as_image_u8(image)
        roi = tuple(config.get("roi", (0, 0, 0, 0))) if isinstance(config, dict) else (0, 0, 0, 0)
        native_config = _ReconstructionConfig(
            horizontal_scan=1 if bool(config.get("horizontal_scan", True)) else 0,
            centerline_method={"gray": 0, "steger": 1, "multi_thres": 2}.get(str(config.get("centerline_method", "gray")), 0),
            bw_threshold=float(config.get("bw_threshold", 55.0)),
            sum_threshold=float(config.get("sum_threshold", 124.0)),
            use_full_roi=1 if bool(config.get("use_full_roi", True)) else 0,
            roi_x=int(roi[0]) if len(roi) > 0 else 0,
            roi_y=int(roi[1]) if len(roi) > 1 else 0,
            roi_width=int(roi[2]) if len(roi) > 2 else 0,
            roi_height=int(roi[3]) if len(roi) > 3 else 0,
        )

        distorted_count = ctypes.c_int()
        cloud_count = ctypes.c_int()
        self._check_status(
            self._dll.uwscan3d_reconstruct_frame(
                image_u8.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                width,
                height,
                int(image_u8.strides[0]),
                channels,
                calibration.handle,
                ctypes.byref(native_config),
                float(laser_position),
                float(scan_start_position),
                float(scan_end_position),
                int(scan_index),
                1 if is_in_water else 0,
                1 if use_quartic_surface else 0,
                None,
                0,
                ctypes.byref(distorted_count),
                None,
                0,
                ctypes.byref(cloud_count),
            )
        )

        distorted_native = (_Point2D * distorted_count.value)()
        cloud = np.zeros((cloud_count.value, 3), dtype=np.float32)
        self._check_status(
            self._dll.uwscan3d_reconstruct_frame(
                image_u8.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                width,
                height,
                int(image_u8.strides[0]),
                channels,
                calibration.handle,
                ctypes.byref(native_config),
                float(laser_position),
                float(scan_start_position),
                float(scan_end_position),
                int(scan_index),
                1 if is_in_water else 0,
                1 if use_quartic_surface else 0,
                distorted_native,
                distorted_count.value,
                ctypes.byref(distorted_count),
                cloud.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                cloud_count.value,
                ctypes.byref(cloud_count),
            )
        )
        distorted = np.asarray([(distorted_native[i].x, distorted_native[i].y) for i in range(distorted_count.value)], dtype=np.float64)
        return distorted, cloud[: cloud_count.value].astype(np.float64)
