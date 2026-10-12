from __future__ import annotations

import numpy as np


def refract_line(n0: float, n1: float, ray0_xyz: np.ndarray, normal_xyz: np.ndarray) -> np.ndarray:
    ray0 = np.asarray(ray0_xyz, dtype=np.float64)
    normal = np.asarray(normal_xyz, dtype=np.float64)
    ray0 = ray0 / np.linalg.norm(ray0)
    normal = normal / np.linalg.norm(normal)

    ratio = n0 / n1
    c1 = float(np.dot(normal, ray0))
    c2 = 1.0 - ratio * ratio * (1.0 - c1 * c1)
    if c2 <= 0.0:
        raise ValueError("total internal reflection")
    ray1 = ratio * ray0 - normal * (ratio * c1 - np.sqrt(c2))
    return ray1 / np.linalg.norm(ray1)
