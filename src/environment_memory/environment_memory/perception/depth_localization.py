"""Project robust bounding-box depth into camera coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


class LocalizationError(ValueError):
    """A detection cannot be localized without violating a quality gate."""


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if self.width < 1 or self.height < 1:
            raise LocalizationError("camera dimensions must be positive")
        if not all(math.isfinite(value) for value in values):
            raise LocalizationError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise LocalizationError("camera focal lengths must be positive")


@dataclass(frozen=True)
class DepthLocalizationConfig:
    central_fraction: float = 0.60
    minimum_depth_m: float = 0.20
    maximum_depth_m: float = 10.0
    minimum_valid_samples: int = 30
    minimum_valid_ratio: float = 0.30
    mad_scale: float = 3.0
    minimum_outlier_band_m: float = 0.02
    maximum_normalized_dispersion: float = 0.10

    def __post_init__(self) -> None:
        if not 0.0 < self.central_fraction <= 1.0:
            raise ValueError("central_fraction must be in (0, 1]")
        if not 0.0 <= self.minimum_depth_m < self.maximum_depth_m:
            raise ValueError("invalid depth range")
        if self.minimum_valid_samples < 1:
            raise ValueError("minimum_valid_samples must be positive")
        if not 0.0 <= self.minimum_valid_ratio <= 1.0:
            raise ValueError("minimum_valid_ratio must be in [0, 1]")
        if self.mad_scale <= 0.0 or self.maximum_normalized_dispersion <= 0.0:
            raise ValueError("dispersion scales must be positive")


@dataclass(frozen=True)
class DepthLocalizationResult:
    x: float
    y: float
    z: float
    u: float
    v: float
    valid_depth_ratio: float
    depth_mad_m: float
    localization_quality: float
    clamped_bbox: tuple[int, int, int, int]


def intrinsics_from_camera_matrix(
    width: int, height: int, camera_matrix: list[float] | tuple[float, ...]
) -> CameraIntrinsics:
    if len(camera_matrix) != 9:
        raise LocalizationError("CameraInfo.k must contain nine values")
    intrinsics = CameraIntrinsics(
        width=int(width),
        height=int(height),
        fx=float(camera_matrix[0]),
        fy=float(camera_matrix[4]),
        cx=float(camera_matrix[2]),
        cy=float(camera_matrix[5]),
    )
    intrinsics.validate()
    return intrinsics


def localize_detection(
    depth_m: np.ndarray,
    bbox: tuple[float, float, float, float],
    intrinsics: CameraIntrinsics,
    config: DepthLocalizationConfig = DepthLocalizationConfig(),
) -> DepthLocalizationResult:
    intrinsics.validate()
    if not isinstance(depth_m, np.ndarray) or depth_m.ndim != 2:
        raise LocalizationError("depth image must be a two-dimensional NumPy array")
    if not np.issubdtype(depth_m.dtype, np.floating):
        raise LocalizationError("depth image must contain floating-point metres")
    if depth_m.shape != (intrinsics.height, intrinsics.width):
        raise LocalizationError("depth image and CameraInfo dimensions differ")

    x_min, y_min, x_max, y_max = _clamp_bbox(
        bbox, intrinsics.width, intrinsics.height
    )
    inset_fraction = (1.0 - config.central_fraction) / 2.0
    roi_x_min = max(x_min, int(math.ceil(x_min + (x_max - x_min) * inset_fraction)))
    roi_y_min = max(y_min, int(math.ceil(y_min + (y_max - y_min) * inset_fraction)))
    roi_x_max = min(x_max, int(math.floor(x_max - (x_max - x_min) * inset_fraction)))
    roi_y_max = min(y_max, int(math.floor(y_max - (y_max - y_min) * inset_fraction)))
    if roi_x_max <= roi_x_min or roi_y_max <= roi_y_min:
        raise LocalizationError("central depth region is empty")

    roi = depth_m[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
    valid_mask = (
        np.isfinite(roi)
        & (roi >= config.minimum_depth_m)
        & (roi <= config.maximum_depth_m)
    )
    valid_count = int(np.count_nonzero(valid_mask))
    valid_ratio = valid_count / float(roi.size)
    if valid_count < config.minimum_valid_samples:
        raise LocalizationError("too few valid depth samples")
    if valid_ratio < config.minimum_valid_ratio:
        raise LocalizationError("valid-depth ratio is below threshold")

    valid_depths = roi[valid_mask].astype(np.float64, copy=False)
    initial_median = float(np.median(valid_depths))
    absolute_deviations = np.abs(valid_depths - initial_median)
    mad = float(np.median(absolute_deviations))
    robust_sigma = 1.4826 * mad
    outlier_band = max(
        config.minimum_outlier_band_m, config.mad_scale * robust_sigma
    )
    inlier_values = absolute_deviations <= outlier_band
    if int(np.count_nonzero(inlier_values)) < config.minimum_valid_samples:
        raise LocalizationError("too few depth inliers after MAD filtering")

    valid_rows, valid_columns = np.nonzero(valid_mask)
    inlier_depths = valid_depths[inlier_values]
    inlier_rows = valid_rows[inlier_values]
    inlier_columns = valid_columns[inlier_values]
    z = float(np.median(inlier_depths))
    u = float(np.median(inlier_columns + roi_x_min))
    v = float(np.median(inlier_rows + roi_y_min))
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy

    normalized_dispersion = mad / max(z, np.finfo(float).eps)
    dispersion_score = max(
        0.0,
        1.0 - normalized_dispersion / config.maximum_normalized_dispersion,
    )
    quality = min(1.0, max(0.0, 0.5 * valid_ratio + 0.5 * dispersion_score))
    return DepthLocalizationResult(
        x=x,
        y=y,
        z=z,
        u=u,
        v=v,
        valid_depth_ratio=valid_ratio,
        depth_mad_m=mad,
        localization_quality=quality,
        clamped_bbox=(x_min, y_min, x_max, y_max),
    )


def _clamp_bbox(
    bbox: tuple[float, float, float, float], width: int, height: int
) -> tuple[int, int, int, int]:
    if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
        raise LocalizationError("bounding box must contain four finite values")
    raw_x_min, raw_y_min, raw_x_max, raw_y_max = bbox
    if raw_x_max <= raw_x_min or raw_y_max <= raw_y_min:
        raise LocalizationError("bounding box has no positive area")
    x_min = max(0, min(width, int(math.floor(raw_x_min))))
    y_min = max(0, min(height, int(math.floor(raw_y_min))))
    x_max = max(0, min(width, int(math.ceil(raw_x_max))))
    y_max = max(0, min(height, int(math.ceil(raw_y_max))))
    if x_max <= x_min or y_max <= y_min:
        raise LocalizationError("bounding box is outside the image")
    return x_min, y_min, x_max, y_max
