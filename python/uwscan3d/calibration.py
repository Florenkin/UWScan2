from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, List, Sequence

import cv2
import numpy as np

if TYPE_CHECKING:
    from .dll import UWScan3DCalibration, UWScan3DDll


def _read_matrix(node: cv2.FileNode) -> np.ndarray:
    value = node.mat()
    if value is None:
        return np.empty((0, 0), dtype=np.float64)
    return np.asarray(value)


def _read_scalar(node: cv2.FileNode, default: float = 0.0) -> float:
    if node.empty():
        return default
    return float(node.real())


def _read_sequence(node: cv2.FileNode, key: str) -> List[np.ndarray]:
    values: List[np.ndarray] = []
    for i in range(node.size()):
        child = node.at(i)
        values.append(_read_matrix(child.getNode(key)))
    return values


@dataclass(slots=True)
class CalibrationData:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    picture_size: tuple[int, int]
    mu: np.ndarray
    d0: float
    d1: float
    axis: np.ndarray
    surf_begin_pos: int
    surf_end_pos: int
    surf_step: int
    transforms: List[np.ndarray]
    surface_parameters: List[np.ndarray]
    z_ranges: List[tuple[float, float]]

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationData":
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(path)
        try:
            picture_size = tuple(int(x) for x in _read_matrix(fs.getNode("picSize")).reshape(-1)[:2])
            transforms = _read_sequence(fs.getNode("allTransMats"), "TransMats")
            surface_parameters = _read_sequence(fs.getNode("allSurfacePars"), "SurfPars")
            z_ranges = []
            z_node = fs.getNode("allMaxMinZValues")
            for i in range(z_node.size()):
                child = z_node.at(i)
                z_ranges.append(
                    (
                        _read_scalar(child.getNode("MaxZValue")),
                        _read_scalar(child.getNode("MinZValue")),
                    )
                )
            return cls(
                camera_matrix=_read_matrix(fs.getNode("CameraMatrix")),
                dist_coeffs=_read_matrix(fs.getNode("DistCoeffs")),
                picture_size=picture_size,
                mu=_read_matrix(fs.getNode("mu")).reshape(-1),
                d0=_read_scalar(fs.getNode("d0")),
                d1=_read_scalar(fs.getNode("d1")),
                axis=_read_matrix(fs.getNode("A")).reshape(-1),
                surf_begin_pos=int(_read_scalar(fs.getNode("surfBeginPos"))),
                surf_end_pos=int(_read_scalar(fs.getNode("surfEndPos"))),
                surf_step=int(_read_scalar(fs.getNode("surfStep"), 1.0)),
                transforms=[np.asarray(matrix, dtype=np.float32) for matrix in transforms],
                surface_parameters=[np.asarray(matrix, dtype=np.float32).reshape(-1) for matrix in surface_parameters],
                z_ranges=z_ranges,
            )
        finally:
            fs.release()

    @property
    def surf_num(self) -> int:
        return len(self.transforms)

    def map_laser_position(self, laser_position: float, scan_start: float, scan_end: float, scan_index: int) -> int:
        lower = min(self.surf_begin_pos, self.surf_end_pos)
        upper = max(self.surf_begin_pos, self.surf_end_pos)
        centi_index = int(round(laser_position * 100.0))
        if lower <= centi_index <= upper:
            return centi_index

        direct_index = int(round(laser_position))
        if lower <= direct_index <= upper:
            return direct_index

        if abs(scan_end - scan_start) < 1e-9:
            return scan_index

        t = (laser_position - scan_start) / (scan_end - scan_start)
        raw_index = self.surf_begin_pos + t * (self.surf_end_pos - self.surf_begin_pos)
        step = max(abs(self.surf_step), 1)
        mapped = self.surf_begin_pos + round((raw_index - self.surf_begin_pos) / step) * step
        return int(min(max(mapped, lower), upper))

    def calibration_slot(self, calibration_index: int) -> int:
        return int((calibration_index - self.surf_begin_pos) / self.surf_step)

    @classmethod
    def from_dll(cls, calibration: "UWScan3DCalibration") -> "CalibrationData":
        transforms = [calibration.transform(i).astype(np.float32) for i in range(calibration.surf_num)]
        surface_parameters = [calibration.surface_parameters(i).astype(np.float32) for i in range(calibration.surf_num)]
        z_ranges = [calibration.z_range(i) for i in range(calibration.surf_num)]
        d0 = calibration.d0
        d1 = calibration.d1
        return cls(
            camera_matrix=calibration.camera_matrix,
            dist_coeffs=calibration.dist_coeffs,
            picture_size=calibration.picture_size,
            mu=calibration.mu,
            d0=d0,
            d1=d1,
            axis=calibration.axis,
            surf_begin_pos=calibration.calibration_slot(0) * calibration.path.exists() if False else 0,
            surf_end_pos=0,
            surf_step=1,
            transforms=transforms,
            surface_parameters=surface_parameters,
            z_ranges=z_ranges,
        )


def load_calibration_via_dll(dll: "UWScan3DDll", path: str | Path) -> "UWScan3DCalibration":
    return dll.load_calibration(path)
