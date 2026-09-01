"""Generate safe standoff poses around remembered objects."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class OccupancyGrid2D:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]
    occupied_threshold: int
    reject_unknown: bool = True

    def is_free(self, x: float, y: float) -> bool:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return False
        column, row = cell
        value = self.data[row * self.width + column]
        if value < 0:
            return not self.reject_unknown
        return value < self.occupied_threshold

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        dx = x - self.origin_x
        dy = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        return column, row


def validate_grid(grid: OccupancyGrid2D) -> None:
    if grid.width <= 0 or grid.height <= 0:
        raise ValueError("occupancy grid dimensions must be positive")
    if not math.isfinite(grid.resolution) or grid.resolution <= 0.0:
        raise ValueError("occupancy grid resolution must be positive")
    if len(grid.data) != grid.width * grid.height:
        raise ValueError("occupancy grid data length does not match dimensions")
    if not 1 <= grid.occupied_threshold <= 101:
        raise ValueError("occupied threshold must be between 1 and 101")


def generate_approach_candidates(
    object_x: float,
    object_y: float,
    robot: Pose2D,
    static_map: OccupancyGrid2D,
    inflated_costmap: OccupancyGrid2D,
    radii_m: Sequence[float] = (0.8, 1.0, 1.2),
    samples_per_radius: int = 16,
) -> list[Pose2D]:
    validate_grid(static_map)
    validate_grid(inflated_costmap)
    if not all(math.isfinite(value) for value in (object_x, object_y, robot.x, robot.y)):
        raise ValueError("object and robot positions must be finite")
    if samples_per_radius < 4:
        raise ValueError("at least four angular samples are required")
    if not radii_m or any(
        not math.isfinite(radius) or not 0.8 <= radius <= 1.2
        for radius in radii_m
    ):
        raise ValueError("approach radii must remain within 0.8 to 1.2 metres")

    candidates = []
    for radius in radii_m:
        for index in range(samples_per_radius):
            angle = 2.0 * math.pi * index / samples_per_radius
            x = object_x + radius * math.cos(angle)
            y = object_y + radius * math.sin(angle)
            if not static_map.is_free(x, y) or not inflated_costmap.is_free(x, y):
                continue
            yaw = math.atan2(object_y - y, object_x - x)
            candidates.append(Pose2D(x, y, yaw))
    candidates.sort(
        key=lambda pose: (
            math.hypot(pose.x - robot.x, pose.y - robot.y),
            pose.x,
            pose.y,
        )
    )
    return candidates
