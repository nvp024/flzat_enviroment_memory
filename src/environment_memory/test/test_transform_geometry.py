import math

import pytest

from environment_memory.transform_geometry import (
    RigidTransform,
    TransformGeometryError,
    transform_point,
    validate_transform_contract,
)


def test_applies_rotation_then_translation():
    half_sqrt = math.sqrt(0.5)
    transform = RigidTransform(
        translation=(10.0, -2.0, 0.5),
        quaternion_xyzw=(0.0, 0.0, half_sqrt, half_sqrt),
    )

    result = transform_point((1.0, 0.0, 2.0), transform)

    assert result == pytest.approx((10.0, -1.0, 2.5))


def test_normalizes_valid_quaternion():
    transform = RigidTransform(
        translation=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 2.0)
    )
    assert transform_point((1.0, 2.0, 3.0), transform) == pytest.approx(
        (1.0, 2.0, 3.0)
    )


def test_rejects_zero_quaternion_and_non_finite_input():
    with pytest.raises(TransformGeometryError, match="zero norm"):
        transform_point(
            (1.0, 2.0, 3.0),
            RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        )
    with pytest.raises(TransformGeometryError, match="finite"):
        transform_point(
            (math.nan, 2.0, 3.0),
            RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )


def test_requires_exact_timestamp_and_frame_contract():
    validate_transform_contract(
        "map",
        "camera_optical_frame",
        123,
        expected_target_frame="map",
        expected_source_frame="camera_optical_frame",
        expected_stamp_ns=123,
    )
    with pytest.raises(TransformGeometryError, match="timestamp"):
        validate_transform_contract(
            "map",
            "camera_optical_frame",
            124,
            expected_target_frame="map",
            expected_source_frame="camera_optical_frame",
            expected_stamp_ns=123,
        )
    with pytest.raises(TransformGeometryError, match="frame contract"):
        validate_transform_contract(
            "map",
            "camera_link",
            123,
            expected_target_frame="map",
            expected_source_frame="camera_optical_frame",
            expected_stamp_ns=123,
        )
