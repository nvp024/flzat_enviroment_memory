"""Validate timestamped transforms and transform 3D points."""

from __future__ import annotations

from dataclasses import dataclass
import math


class TransformGeometryError(ValueError):
    pass


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def validate_transform_contract(
    target_frame: str,
    source_frame: str,
    stamp_ns: int,
    *,
    expected_target_frame: str,
    expected_source_frame: str,
    expected_stamp_ns: int,
) -> None:
    if not target_frame or not source_frame:
        raise TransformGeometryError("transform frame IDs must not be empty")
    if (
        target_frame != expected_target_frame
        or source_frame != expected_source_frame
    ):
        raise TransformGeometryError(
            "transform frame contract mismatch; expected "
            f"{expected_target_frame} <- {expected_source_frame}"
        )
    if stamp_ns != expected_stamp_ns:
        raise TransformGeometryError(
            "transform timestamp does not match RGB timestamp"
        )


def transform_point(
    point: tuple[float, float, float], transform: RigidTransform
) -> tuple[float, float, float]:
    values = point + transform.translation + transform.quaternion_xyzw
    if not all(math.isfinite(value) for value in values):
        raise TransformGeometryError("point and transform must be finite")
    qx, qy, qz, qw = transform.quaternion_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise TransformGeometryError("transform quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    px, py, pz = point

    # Quaternion-vector rotation: v' = v + 2 * (qw * (q x v) + q x (q x v)).
    cross_x = qy * pz - qz * py
    cross_y = qz * px - qx * pz
    cross_z = qx * py - qy * px
    second_x = qy * cross_z - qz * cross_y
    second_y = qz * cross_x - qx * cross_z
    second_z = qx * cross_y - qy * cross_x
    rotated = (
        px + 2.0 * (qw * cross_x + second_x),
        py + 2.0 * (qw * cross_y + second_y),
        pz + 2.0 * (qw * cross_z + second_z),
    )
    return tuple(
        rotated[index] + transform.translation[index] for index in range(3)
    )
