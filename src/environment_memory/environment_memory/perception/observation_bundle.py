"""Define frozen sensor and transform observation bundles."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ObservationBundle:
    observation_id: str
    rgb: Any
    depth: Any
    camera_info: Any
    camera_transform: Any
    robot_transform: Any
    scan_stamp_ns: int
    trigger_reason: str
    sync_delta_ns: int
