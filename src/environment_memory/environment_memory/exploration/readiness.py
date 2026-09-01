"""Represent Mode 1 runtime readiness gates."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReadinessSnapshot:
    map_received: bool = False
    scan_received: bool = False
    nav_action_ready: bool = False
    lifecycle_active: bool = False
    map_to_odom_tf_ready: bool = False
    odom_to_base_tf_ready: bool = False
    base_to_base_link_tf_ready: bool = False
    base_link_to_camera_tf_ready: bool = False
    explorer_control_ready: bool = False
    memory_manager_ready: bool = False
    semantic_pipeline_ready: bool = False

    @property
    def ready(self) -> bool:
        return not self.missing

    @property
    def missing(self) -> tuple[str, ...]:
        checks = (
            ("map", self.map_received),
            ("scan", self.scan_received),
            ("navigate_to_pose", self.nav_action_ready),
            ("Nav2 lifecycle", self.lifecycle_active),
            ("map->odom TF", self.map_to_odom_tf_ready),
            ("odom->base_footprint TF", self.odom_to_base_tf_ready),
            ("base_footprint->base_link TF", self.base_to_base_link_tf_ready),
            ("base_link->camera_optical_frame TF", self.base_link_to_camera_tf_ready),
            ("frontier control service", self.explorer_control_ready),
            ("writable Memory Manager", self.memory_manager_ready),
            ("structured VLM pipeline", self.semantic_pipeline_ready),
        )
        return tuple(name for name, available in checks if not available)
