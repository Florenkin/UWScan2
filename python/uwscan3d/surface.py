from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(slots=True)
class SurfaceFitResult:
    coefficients: np.ndarray
    transform: np.ndarray
    max_z: float
    min_z: float
    mean_error: float


def _center_and_rotate(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = points.mean(axis=0)
    normalized = points - center
    _, _, vh = np.linalg.svd(normalized, full_matrices=False)
    rotation = vh.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation.T
    transform[:3, 3] = -(rotation.T @ center)
    rotated = (rotation.T @ normalized.T).T
    return rotated, transform


def fit_quadric_surface(points: np.ndarray) -> SurfaceFitResult:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (n, 3)")

    local_pts, transform = _center_and_rotate(pts)
    x = local_pts[:, 0]
    y = local_pts[:, 1]
    z = local_pts[:, 2]
    a = np.column_stack([np.ones_like(x), x, y, x * x, x * y, y * y])
    coeffs6, _, _, _ = np.linalg.lstsq(a, z, rcond=None)
    residual = a @ coeffs6 - z
    coeffs10 = np.array(
        [coeffs6[3], coeffs6[5], 0.0, coeffs6[4], 0.0, 0.0, coeffs6[1], coeffs6[2], -1.0, coeffs6[0]],
        dtype=np.float64,
    )
    return SurfaceFitResult(
        coefficients=coeffs10,
        transform=transform,
        max_z=float(z.max()),
        min_z=float(z.min()),
        mean_error=float(np.mean(np.abs(residual))),
    )


def fit_quartic_surface(points: np.ndarray) -> SurfaceFitResult:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (n, 3)")

    local_pts, transform = _center_and_rotate(pts)
    x = local_pts[:, 0]
    y = local_pts[:, 1]
    z = local_pts[:, 2]
    a = np.column_stack(
        [
            np.ones_like(x),
            x,
            y,
            x * x,
            x * y,
            y * y,
            x ** 3,
            (x ** 2) * y,
            x * (y ** 2),
            y ** 3,
            x ** 4,
            (x ** 3) * y,
            (x ** 2) * (y ** 2),
            x * (y ** 3),
            y ** 4,
        ]
    )
    coeffs15, _, _, _ = np.linalg.lstsq(a, z, rcond=None)
    residual = a @ coeffs15 - z
    coeffs35 = np.zeros(35, dtype=np.float64)
    coeffs35[[34, 31, 32, 28, 25, 29, 15, 16, 18, 19, 0, 1, 2, 9, 6]] = coeffs15
    coeffs35[33] = -1.0
    return SurfaceFitResult(
        coefficients=coeffs35,
        transform=transform,
        max_z=float(z.max()),
        min_z=float(z.min()),
        mean_error=float(np.mean(np.abs(residual))),
    )


def intersect_quadric_surface(coefficients: np.ndarray, ray: np.ndarray, max_z: float, min_z: float) -> np.ndarray:
    c = np.asarray(coefficients, dtype=np.float64).reshape(-1)
    p = np.asarray(ray, dtype=np.float64).reshape(-1)
    if c.size != 10 or p.size != 6:
        raise ValueError("invalid quadric inputs")

    k1 = p[3] / p[5]
    k2 = p[4] / p[5]
    b1 = p[0] - k1 * p[2]
    b2 = p[1] - k2 * p[2]
    a2 = c[0] * k1 * k1 + c[3] * k1 * k2 + c[5] * k1 + c[1] * k2 * k2 + c[4] * k2 + c[2]
    a1 = c[8] + b1 * c[5] + b2 * c[4] + c[6] * k1 + c[7] * k2 + 2 * b1 * c[0] * k1 + 2 * b2 * c[1] * k2 + b1 * c[3] * k2 + b2 * c[3] * k1
    a0 = c[0] * b1 * b1 + c[3] * b1 * b2 + c[6] * b1 + c[1] * b2 * b2 + c[7] * b2 + c[9]
    roots = np.roots([a2, a1, a0]) if abs(a2) > 1e-10 else np.roots([a1, a0])
    roots = [float(np.real(root)) for root in roots if abs(np.imag(root)) < 1e-6]
    if not roots:
        return np.zeros(3, dtype=np.float64)
    z = min(roots, key=abs)
    if z < min_z - 10 or z > max_z + 10 or abs(z) > 1e2:
        return np.zeros(3, dtype=np.float64)
    return np.asarray([k1 * z + b1, k2 * z + b2, z], dtype=np.float64)


def intersect_quartic_surface(coefficients: np.ndarray, ray: np.ndarray, max_z: float, min_z: float) -> np.ndarray:
    c = np.asarray(coefficients, dtype=np.float64).reshape(-1)
    p = np.asarray(ray, dtype=np.float64).reshape(-1)
    if c.size != 35 or p.size != 6:
        raise ValueError("invalid quartic inputs")

    samples = np.linspace(min_z - 10, max_z + 10, 1024)
    k1 = p[3] / p[5]
    k2 = p[4] / p[5]
    b1 = p[0] - k1 * p[2]
    b2 = p[1] - k2 * p[2]
    x = k1 * samples + b1
    y = k2 * samples + b2
    z = samples
    values = (
        c[0] * x**4 + c[1] * x**3 * y + c[2] * x**2 * y**2 + c[3] * x**3 * z + c[4] * x**2 * z**2 +
        c[5] * x**2 * y * z + c[6] * y**4 + c[7] * y**3 * z + c[8] * y**2 * z**2 + c[9] * x * y**3 +
        c[10] * x * y**2 * z + c[11] * z**4 + c[12] * x * z**3 + c[13] * y * z**3 + c[14] * x * y * z**2 +
        c[15] * x**3 + c[16] * x**2 * y + c[17] * x**2 * z + c[18] * y**3 + c[19] * y**2 * z +
        c[20] * x * y**2 + c[21] * z**3 + c[22] * x * z**2 + c[23] * y * z**2 + c[24] * x * y * z +
        c[25] * y**2 + c[26] * x * y + c[27] * z + c[28] * x**2 + c[29] * y**2 + c[30] * z**2 +
        c[31] * x + c[32] * y + c[33] * z + c[34]
    )
    idx = int(np.argmin(np.abs(values)))
    candidate = float(samples[idx])
    if candidate < min_z - 10 or candidate > max_z + 10 or abs(candidate) > 1e2:
        return np.zeros(3, dtype=np.float64)
    return np.asarray([k1 * candidate + b1, k2 * candidate + b2, candidate], dtype=np.float64)
