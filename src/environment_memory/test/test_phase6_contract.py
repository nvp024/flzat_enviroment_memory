from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
INTERFACE_ROOT = WORKSPACE_ROOT / "src" / "environment_memory_interfaces"


def test_phase6_internal_evidence_interface_is_detector_linked():
    message = (INTERFACE_ROOT / "msg" / "VlmObservation.msg").read_text(
        encoding="utf-8"
    )
    cmake = (INTERFACE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "string observation_id" in message
    assert "sensor_msgs/CompressedImage image" in message
    assert "robot_interfaces/ObjectDetection2D[] detections" in message
    assert '"msg/VlmObservation.msg"' in cmake


def test_semantic_manager_uses_shared_action_and_publishes_missing_endpoint():
    source = (
        PACKAGE_ROOT
        / "environment_memory"
        / "semantics"
        / "semantic_observation_manager.py"
    ).read_text(encoding="utf-8")

    assert '"/vlm/analyze_environment"' in source
    assert '"/environment_memory/geometric_observations"' in source
    assert '"/environment_memory/vlm_observations"' in source
    assert '"/environment_memory/localized_observations"' in source
    assert "join_semantics" in source
    assert "NavigateToPose" not in source
    assert "Chroma" not in source
    assert ".upsert(" not in source


def test_build_launch_restores_phase6_manager():
    launch = (
        PACKAGE_ROOT / "launch" / "exploration_observation.launch.py"
    ).read_text(encoding="utf-8")
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert 'executable="semantic_observation_manager"' in launch
    assert "semantic_observation_manager =" in setup


def test_phase5_handoff_and_finalization_wait_for_semantic_drain():
    observation = (
        PACKAGE_ROOT
        / "environment_memory"
        / "perception"
        / "observation_manager.py"
    ).read_text(encoding="utf-8")
    memory = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "memory_manager.py"
    ).read_text(encoding="utf-8")
    build = (
        PACKAGE_ROOT
        / "environment_memory"
        / "exploration"
        / "memory_build_manager.py"
    ).read_text(encoding="utf-8")
    readiness = (
        PACKAGE_ROOT / "environment_memory" / "exploration" / "readiness.py"
    ).read_text(encoding="utf-8")

    assert '"/environment_memory/vlm_observations"' in observation
    assert "vlm_bgr" in observation
    assert "vlm_annotation" in observation
    assert '"/environment_memory/semantic_status"' in memory
    assert "waiting for structured VLM pipeline to drain" in memory
    assert '"/environment_memory/semantic_status"' in build
    assert "semantic_pipeline_ready" in readiness
