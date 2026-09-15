from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
INTERFACE_ROOT = WORKSPACE_ROOT / "src" / "environment_memory_interfaces"


def test_mode1_is_vlm_grounding_only() -> None:
    launch = (
        PACKAGE_ROOT / "launch" / "autonomous_memory_build.launch.py"
    ).read_text(encoding="utf-8")
    internal = (
        PACKAGE_ROOT / "launch" / "exploration_observation.launch.py"
    ).read_text(encoding="utf-8")
    observation = (
        PACKAGE_ROOT
        / "environment_memory"
        / "perception"
        / "observation_manager.py"
    ).read_text(encoding="utf-8")

    assert 'default_value="Qwen/Qwen3-VL-2B-Instruct"' in launch
    assert "vlm_grounding_geometry.yaml" in launch
    assert '"/vlm/ground_objects"' in observation
    assert "send_goal_async" in observation
    assert "get_result_async" in observation
    assert '"TIMED_OUT"' in observation
    assert "detection_backend" not in launch
    assert "detection_backend" not in internal
    assert "Ultralytics" not in observation
    assert "semantic_observation_manager" not in internal


def test_legacy_detector_and_semantic_handoff_are_removed() -> None:
    package = PACKAGE_ROOT / "environment_memory"
    cmake = (INTERFACE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert not (package / "perception" / "detector.py").exists()
    assert not (package / "perception" / "model_asset.py").exists()
    semantic_manager = package / "semantics" / "semantic_observation_manager.py"
    assert not semantic_manager.exists()
    assert not (WORKSPACE_ROOT / "requirements-detector.txt").exists()
    assert not (WORKSPACE_ROOT / "tools" / "fetch_yolov8n.py").exists()
    assert "GeometricObjectObservation.msg" not in cmake
    assert "VlmObservation.msg" not in cmake


def test_vlm_grounding_geometry_config_keeps_depth_contract() -> None:
    config = (
        PACKAGE_ROOT / "config" / "vlm_grounding_geometry.yaml"
    ).read_text(encoding="utf-8")

    assert "grounding_max_detections: 20" in config
    assert "depth_central_fraction: 0.60" in config
    assert "depth_minimum_m: 0.20" in config
    assert "depth_maximum_m: 10.0" in config
    assert "depth_minimum_valid_samples: 30" in config
    assert "depth_minimum_valid_ratio: 0.30" in config
    assert "detector_model" not in config


def test_coordinator_uses_frozen_bundle_and_direct_memory_handoff() -> None:
    source = (
        PACKAGE_ROOT
        / "environment_memory"
        / "perception"
        / "observation_manager.py"
    ).read_text(encoding="utf-8")

    assert "copy.deepcopy(rgb)" in source
    assert "copy.deepcopy(depth)" in source
    assert "copy.deepcopy(camera_info)" in source
    assert "copy.deepcopy(camera_transform)" in source
    assert "copy.deepcopy(robot_transform)" in source
    assert "result.observation_id != bundle.observation_id" in source
    assert "localize_detection(" in source
    assert '"/environment_memory/localized_observations"' in source
    assert '"/environment_memory/persistence_events"' in source
    assert "observations.jsonl" in source


def test_grounding_drain_waits_for_persistence_ack() -> None:
    observation = (
        PACKAGE_ROOT
        / "environment_memory"
        / "perception"
        / "observation_manager.py"
    ).read_text(encoding="utf-8")
    memory = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "memory_manager.py"
    ).read_text(encoding="utf-8")

    assert "not self._pending_persistence" in observation
    assert "self._persistence_bundles.pop" in observation
    assert '"/environment_memory/persistence_events"' in memory
    assert 'state="STORED"' in memory
