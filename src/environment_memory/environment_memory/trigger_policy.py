from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class TriggerReason(str, Enum):
    FIRST_VALID = "first_valid"
    WAYPOINT = "waypoint"
    SCENE_CHANGE = "scene_change"
    ROTATION = "rotation"
    TRANSLATION = "translation"
    TIMED_REFRESH = "timed_refresh"


TRIGGER_PRIORITY = {
    TriggerReason.FIRST_VALID: 60,
    TriggerReason.WAYPOINT: 50,
    TriggerReason.SCENE_CHANGE: 40,
    TriggerReason.ROTATION: 30,
    TriggerReason.TRANSLATION: 20,
    TriggerReason.TIMED_REFRESH: 10,
}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TriggerConfig:
    translation_m: float = 1.0
    rotation_rad: float = math.radians(45.0)
    scene_distance: float = 0.35
    max_interval_s: float = 20.0
    min_interval_s: float = 8.0
    waypoint_settle_s: float = 0.75
    preferred_linear_speed_mps: float = 0.10
    preferred_angular_speed_rps: float = 0.15


@dataclass(frozen=True)
class TriggerDecision:
    eligible: bool
    reason: Optional[TriggerReason] = None
    priority: int = -1
    detail: str = ""


class ObservationTriggerPolicy:
    def __init__(self, config: TriggerConfig = TriggerConfig()) -> None:
        if config.min_interval_s < 0.0 or config.max_interval_s <= 0.0:
            raise ValueError("trigger intervals must be positive")
        self.config = config
        self._last_accepted_time: Optional[float] = None
        self._last_accepted_pose: Optional[Pose2D] = None
        self._last_histogram = None
        self._previous_pose: Optional[Pose2D] = None
        self._accumulated_yaw = 0.0
        self._waypoint_ready_at: Optional[float] = None

    def mark_waypoint_completed(self, completion_time_s: float) -> None:
        self._waypoint_ready_at = completion_time_s + self.config.waypoint_settle_s

    def evaluate(
        self,
        now_s: float,
        pose: Pose2D,
        scene_distance: Optional[float],
        linear_speed_mps: float,
        angular_speed_rps: float,
    ) -> TriggerDecision:
        self._update_accumulated_yaw(pose)
        if self._last_accepted_time is None:
            return self._decision(TriggerReason.FIRST_VALID)

        elapsed = max(0.0, now_s - self._last_accepted_time)
        if elapsed < self.config.min_interval_s:
            return TriggerDecision(False, detail="minimum interval has not elapsed")

        candidates = []
        if self._waypoint_ready_at is not None and now_s >= self._waypoint_ready_at:
            candidates.append(TriggerReason.WAYPOINT)
        if scene_distance is not None and scene_distance >= self.config.scene_distance:
            candidates.append(TriggerReason.SCENE_CHANGE)
        if self._accumulated_yaw >= self.config.rotation_rad:
            candidates.append(TriggerReason.ROTATION)
        if self._translation_from_last_accept(pose) >= self.config.translation_m:
            candidates.append(TriggerReason.TRANSLATION)
        if elapsed >= self.config.max_interval_s:
            candidates.append(TriggerReason.TIMED_REFRESH)
        if not candidates:
            return TriggerDecision(False, detail="no trigger threshold reached")

        reason = max(candidates, key=lambda item: TRIGGER_PRIORITY[item])
        stable = (
            abs(linear_speed_mps) <= self.config.preferred_linear_speed_mps
            and abs(angular_speed_rps) <= self.config.preferred_angular_speed_rps
        )
        if not stable and reason != TriggerReason.TIMED_REFRESH:
            return TriggerDecision(
                False, reason=reason, detail="waiting for stable frame"
            )
        return self._decision(reason)

    def accept(self, now_s: float, pose: Pose2D, histogram) -> None:
        self._last_accepted_time = now_s
        self._last_accepted_pose = pose
        self._last_histogram = None if histogram is None else histogram.copy()
        self._accumulated_yaw = 0.0
        self._previous_pose = pose
        self._waypoint_ready_at = None

    @property
    def last_histogram(self):
        return self._last_histogram

    def _translation_from_last_accept(self, pose: Pose2D) -> float:
        if self._last_accepted_pose is None:
            return math.inf
        return math.hypot(
            pose.x - self._last_accepted_pose.x,
            pose.y - self._last_accepted_pose.y,
        )

    def _update_accumulated_yaw(self, pose: Pose2D) -> None:
        if self._previous_pose is not None:
            delta = math.atan2(
                math.sin(pose.yaw - self._previous_pose.yaw),
                math.cos(pose.yaw - self._previous_pose.yaw),
            )
            self._accumulated_yaw += abs(delta)
        self._previous_pose = pose

    @staticmethod
    def _decision(reason: TriggerReason) -> TriggerDecision:
        return TriggerDecision(True, reason, TRIGGER_PRIORITY[reason])
