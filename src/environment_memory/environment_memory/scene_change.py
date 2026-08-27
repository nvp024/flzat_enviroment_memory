from __future__ import annotations

import cv2
import numpy as np


def hsv_histogram(bgr_image: np.ndarray) -> np.ndarray:
    if bgr_image is None or bgr_image.size == 0:
        raise ValueError("image cannot be empty")
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
    cv2.normalize(histogram, histogram, alpha=1.0, norm_type=cv2.NORM_L1)
    return histogram


def histogram_distance(first: np.ndarray, second: np.ndarray) -> float:
    if first.shape != second.shape:
        raise ValueError("histograms must have the same shape")
    return float(cv2.compareHist(first, second, cv2.HISTCMP_BHATTACHARYYA))
