import math

import numpy as np
import pytest

from environment_memory.depth_localization import (
    CameraIntrinsics,
    DepthLocalizationConfig,
    LocalizationError,
    intrinsics_from_camera_matrix,
    localize_detection,
)


INTRINSICS = CameraIntrinsics(
    width=100, height=80, fx=100.0, fy=100.0, cx=50.0, cy=40.0
)


def test_localizes_central_depth_with_pinhole_projection_and_mad_filtering():
    depth = np.full((80, 100), np.nan, dtype=np.float32)
    depth[20:60, 20:80] = 2.0
    depth[35, 45] = 9.0
    depth[36, 46] = np.inf

    result = localize_detection(depth, (20.0, 20.0, 80.0, 60.0), INTRINSICS)

    assert result.clamped_bbox == (20, 20, 80, 60)
    assert result.z == pytest.approx(2.0)
    assert result.u == pytest.approx(50.0)
    assert result.v == pytest.approx(40.0)
    assert result.x == pytest.approx(0.0)
    assert result.y == pytest.approx(0.0)
    assert result.depth_mad_m == pytest.approx(0.0)
    assert 0.99 < result.valid_depth_ratio <= 1.0
    assert 0.99 < result.localization_quality <= 1.0


def test_uses_only_central_sixty_percent_of_bbox():
    depth = np.full((80, 100), 8.0, dtype=np.float32)
    depth[28:52, 32:68] = 1.5

    result = localize_detection(depth, (20.0, 20.0, 80.0, 60.0), INTRINSICS)

    assert result.z == pytest.approx(1.5)
    assert result.valid_depth_ratio == pytest.approx(1.0)


def test_rejects_low_valid_ratio_and_invalid_boxes():
    depth = np.full((80, 100), np.nan, dtype=np.float32)
    depth[30:34, 40:50] = 2.0

    with pytest.raises(LocalizationError, match="valid-depth ratio"):
        localize_detection(
            depth,
            (20.0, 20.0, 80.0, 60.0),
            INTRINSICS,
            DepthLocalizationConfig(minimum_valid_samples=1),
        )
    with pytest.raises(LocalizationError, match="positive area"):
        localize_detection(depth, (20.0, 20.0, 20.0, 60.0), INTRINSICS)
    with pytest.raises(LocalizationError, match="outside the image"):
        localize_detection(depth, (120.0, 20.0, 140.0, 60.0), INTRINSICS)


def test_rejects_non_float_depth_and_bad_intrinsics():
    integer_depth = np.ones((80, 100), dtype=np.uint16)
    with pytest.raises(LocalizationError, match="floating-point metres"):
        localize_detection(integer_depth, (20.0, 20.0, 80.0, 60.0), INTRINSICS)

    with pytest.raises(LocalizationError, match="focal lengths"):
        intrinsics_from_camera_matrix(
            100, 80, [0.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
        )


def test_localization_quality_decreases_with_depth_dispersion():
    depth = np.full((80, 100), 2.0, dtype=np.float32)
    alternating = np.indices((24, 36)).sum(axis=0) % 2 == 0
    depth[28:52, 32:68][alternating] = 2.1

    result = localize_detection(depth, (20.0, 20.0, 80.0, 60.0), INTRINSICS)

    assert math.isfinite(result.localization_quality)
    assert 0.5 <= result.localization_quality < 1.0
