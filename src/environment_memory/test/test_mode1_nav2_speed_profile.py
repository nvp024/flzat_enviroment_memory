"""The slower Nav2 settings must apply to exploration only."""

from pathlib import Path

import yaml

from environment_memory.exploration.nav2_speed_profile import (
    create_mode1_nav2_params,
)


def test_mode1_speed_profile_preserves_openarm_source(tmp_path: Path) -> None:
    source = tmp_path / "nav2_params.yaml"
    original = {
        "controller_server": {
            "ros__parameters": {
                "FollowPath": {
                    "desired_linear_vel": 0.25,
                    "rotate_to_heading_angular_vel": 0.50,
                    "max_angular_accel": 1.0,
                    "lookahead_dist": 0.50,
                },
                "progress_checker": {"movement_time_allowance": 15.0},
            }
        },
        "velocity_smoother": {
            "ros__parameters": {
                "max_velocity": [0.30, 0.0, 0.60],
                "min_velocity": [-0.15, 0.0, -0.60],
                "max_accel": [0.80, 0.0, 1.0],
                "max_decel": [-1.00, 0.0, -1.20],
            }
        },
        "planner_server": {"ros__parameters": {"expected_planner_frequency": 1.0}},
    }
    source.write_text(yaml.safe_dump(original), encoding="utf-8")

    profile = create_mode1_nav2_params(source, tmp_path)

    assert profile != source
    assert yaml.safe_load(source.read_text(encoding="utf-8")) == original
    updated = yaml.safe_load(profile.read_text(encoding="utf-8"))
    controller = updated["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    smoother = updated["velocity_smoother"]["ros__parameters"]
    assert follow_path["desired_linear_vel"] == 0.10
    assert follow_path["rotate_to_heading_angular_vel"] == 0.30
    assert follow_path["max_angular_accel"] == 0.60
    assert follow_path["lookahead_dist"] == 0.50
    assert controller["progress_checker"]["movement_time_allowance"] == 30.0
    assert smoother["max_velocity"] == [0.10, 0.0, 0.30]
    assert smoother["min_velocity"] == [-0.10, 0.0, -0.30]
    assert smoother["max_accel"] == [0.30, 0.0, 0.60]
    assert smoother["max_decel"] == [-0.40, 0.0, -0.80]
    assert updated["planner_server"] == original["planner_server"]


def test_mode1_launch_passes_its_own_nav2_parameters() -> None:
    launch_path = (
        Path(__file__).parents[1] / "launch" / "exploration_observation.launch.py"
    )
    launch_source = launch_path.read_text(encoding="utf-8")

    assert '"params_file": str(mode1_nav2_params)' in launch_source
