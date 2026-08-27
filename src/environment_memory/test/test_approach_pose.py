import math

import pytest

from environment_memory.approach_pose import (
    OccupancyGrid2D,
    Pose2D,
    generate_approach_candidates,
)


def grid(data=None, threshold=50):
    values = [0] * 10_000 if data is None else data
    return OccupancyGrid2D(
        width=100,
        height=100,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=tuple(values),
        occupied_threshold=threshold,
    )


def test_candidates_are_safe_standoff_poses_oriented_toward_object():
    candidates = generate_approach_candidates(
        5.0, 5.0, Pose2D(0.0, 5.0, 0.0), grid(), grid(threshold=1)
    )

    assert candidates
    first = candidates[0]
    radius = math.hypot(first.x - 5.0, first.y - 5.0)
    assert 0.8 - 1e-9 <= radius <= 1.2 + 1e-9
    assert radius != pytest.approx(0.0)
    assert first.yaw == pytest.approx(math.atan2(5.0 - first.y, 5.0 - first.x))


def test_occupied_inflated_cells_and_outside_map_are_rejected():
    blocked = [100] * 10_000
    assert generate_approach_candidates(
        5.0,
        5.0,
        Pose2D(0.0, 0.0, 0.0),
        grid(),
        grid(blocked, threshold=1),
    ) == []
    assert generate_approach_candidates(
        -5.0,
        -5.0,
        Pose2D(0.0, 0.0, 0.0),
        grid(),
        grid(threshold=1),
    ) == []


def test_invalid_standoff_radius_is_rejected():
    with pytest.raises(ValueError, match="0.8 to 1.2"):
        generate_approach_candidates(
            5.0,
            5.0,
            Pose2D(0.0, 0.0, 0.0),
            grid(),
            grid(threshold=1),
            radii_m=(0.5,),
        )
