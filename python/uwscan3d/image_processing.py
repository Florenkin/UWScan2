from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np


def _to_gray(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image.astype(np.uint8, copy=False)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def preprocess_laser_mask(image: np.ndarray, threshold_value: float, area_threshold: float = 50.0) -> tuple[np.ndarray, list[tuple[int, int, int, int]]]:
    gray = _to_gray(image)
    gray = cv2.GaussianBlur(gray, (0, 0), 2, 2)
    _, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    mask = np.zeros_like(gray)
    ranges: list[tuple[int, int, int, int]] = []
    for contour in contours:
        if cv2.contourArea(contour) <= area_threshold:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        ranges.append((y, y + h, x, x + w))
    return cv2.bitwise_and(gray, mask), ranges


def find_laser_center_gray(
    image: np.ndarray,
    horizontal_scan: bool = True,
    bw_threshold: float = 55.0,
    sum_threshold: float = 124.0,
    pixel_threshold: float = 30.0,
) -> np.ndarray:
    masked, ranges = preprocess_laser_mask(image, bw_threshold)
    points: list[tuple[float, float]] = []
    for top, bottom, left, right in ranges:
        if horizontal_scan:
            for x in range(left, right):
                column = masked[top:bottom, x].astype(np.float64)
                if x >= left + 2 and x < right - 2:
                    column = column + masked[top:bottom, x - 2] + masked[top:bottom, x - 1] + masked[top:bottom, x + 1] + masked[top:bottom, x + 2]
                total = float(column[column > pixel_threshold].sum())
                if total < sum_threshold:
                    continue
                y_coords = np.arange(top, bottom, dtype=np.float64)
                y = float((column * y_coords).sum() / column.sum())
                points.append((float(x), y))
        else:
            for y in range(top, bottom):
                row = masked[y, left:right].astype(np.float64)
                if y >= top + 2 and y < bottom - 2:
                    row = row + masked[y - 2, left:right] + masked[y - 1, left:right] + masked[y + 1, left:right] + masked[y + 2, left:right]
                total = float(row[row > pixel_threshold].sum())
                if total < 100.0:
                    continue
                x_coords = np.arange(left, right, dtype=np.float64)
                x = float((row * x_coords).sum() / row.sum())
                points.append((x, float(y)))
    return np.asarray(points, dtype=np.float64)


def build_composite_image(images: Sequence[np.ndarray]) -> np.ndarray:
    valid = [image for image in images if image is not None and image.size > 0]
    if not valid:
        raise ValueError("images is empty")
    gray_images = [_to_gray(image) for image in valid]
    target_shape = gray_images[0].shape[::-1]
    accum = np.zeros(gray_images[0].shape, dtype=np.float64)
    for gray in gray_images:
        if gray.shape[::-1] != target_shape:
            gray = cv2.resize(gray, target_shape, interpolation=cv2.INTER_AREA)
        accum += gray.astype(np.float64)
    composite = np.clip(accum / len(gray_images), 0, 255).astype(np.uint8)
    return cv2.equalizeHist(composite)


def build_intensity_image(image: np.ndarray, line_points: np.ndarray, previous: np.ndarray | None = None) -> np.ndarray:
    gray = _to_gray(image)
    accum = np.zeros_like(gray) if previous is None else previous.copy()
    frame = np.zeros_like(gray)
    rounded = np.rint(line_points).astype(np.int32)
    for x, y in rounded:
        if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
            frame[y, x] = max(frame[y, x], gray[y, x])
    accum = np.maximum(accum, frame)

    max_value = float(accum.max())
    display = accum if max_value <= 0.0 else np.clip(accum.astype(np.float64) * (255.0 / max_value), 0, 255).astype(np.uint8)
    display = cv2.threshold(display, 64, 255, cv2.THRESH_TOZERO)[1]
    color = np.zeros((display.shape[0], display.shape[1], 3), dtype=np.uint8)
    non_zero = display > 0
    color[..., 1][non_zero] = np.clip(120 + display[non_zero] // 2, 0, 255)
    return color
