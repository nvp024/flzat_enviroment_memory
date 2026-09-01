from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_external_frontier_dependency_is_commit_pinned():
    repos = (WORKSPACE_ROOT / "environment_memory.repos").read_text(encoding="utf-8")

    assert "b0fad500e5c81ad3154f0469ca283b2702a3f90c" in repos
    assert "src/frontier_exploration_ros2" in repos


def test_integration_launch_does_not_bypass_motion_safety_or_add_phase8():
    launch = (PACKAGE_ROOT / "launch" / "exploration_observation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "all_in_one.launch.py" in launch
    assert '"slam": "true"' in launch
    assert '"autostart": "false"' in launch
    assert '"control_service_enabled": "true"' in launch
    assert "memory_build_manager" in launch
    assert "observation_manager" in launch
    assert "/cmd_vel" not in launch
    assert 'executable="memory_manager"' in launch
    assert "memory_command_manager" not in launch
    assert "/environment_memory/query" not in launch
    assert "object_detector" not in launch


def test_frontier_configuration_uses_nav2_and_completion_hook():
    config = (PACKAGE_ROOT / "config" / "frontier_openarm.yaml").read_text(
        encoding="utf-8"
    )

    assert "navigate_to_pose_action_name: /navigate_to_pose" in config
    assert "completion_event_enabled: true" in config
    assert "control_service_enabled: true" in config
    assert "autostart: false" in config


def test_observation_manager_locks_phase4_sensor_contracts():
    source = (
        PACKAGE_ROOT
        / "environment_memory"
        / "perception"
        / "observation_manager.py"
    ).read_text(encoding="utf-8")
    package = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert 'declare_parameter("sync_queue_size", 10)' in source
    assert 'declare_parameter("sync_slop_s", 0.08)' in source
    assert 'declare_parameter("camera_info_max_age_s", 1.0)' in source
    assert 'declare_parameter("sensor_max_age_s", 0.5)' in source
    assert 'declare_parameter("tf_timeout_s", 0.5)' in source
    assert 'if depth.encoding != "32FC1"' in source
    assert "Time.from_msg(rgb.header.stamp)" in source
    assert "lookup_transform" in source
    assert "image_to_bgr" in source
    assert "image_to_depth_32fc1" in source
    assert "cv_bridge" not in source
    assert "<exec_depend>cv_bridge</exec_depend>" not in package
