"""Build a Mode 1-only Nav2 speed profile from the OpenArm configuration."""

from __future__ import annotations

import tempfile
from pathlib import Path

import yaml


def create_mode1_nav2_params(
    source: Path, output_directory: Path | None = None
) -> Path:
    """Copy Nav2 parameters with slower exploration motion; leave source untouched."""
    with source.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    try:
        controller = params["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        smoother = params["velocity_smoother"]["ros__parameters"]
        progress_checker = controller["progress_checker"]
        for name in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
            if len(smoother[name]) != 3:
                raise ValueError(f"velocity_smoother.{name} must have three axes")
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"Unsupported OpenArm Nav2 parameter structure: {exc}"
        ) from exc

    follow_path["desired_linear_vel"] = 0.10
    follow_path["rotate_to_heading_angular_vel"] = 0.30
    follow_path["max_angular_accel"] = 0.60
    progress_checker["movement_time_allowance"] = 30.0

    smoother["max_velocity"] = [0.10, 0.0, 0.30]
    smoother["min_velocity"] = [-0.10, 0.0, -0.30]
    smoother["max_accel"] = [0.30, 0.0, 0.60]
    smoother["max_decel"] = [-0.40, 0.0, -0.80]

    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="flzat_mode1_nav2_",
        suffix=".yaml",
        dir=output_directory,
        delete=False,
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return Path(stream.name)
