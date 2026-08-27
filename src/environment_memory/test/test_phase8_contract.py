from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def source(name):
    return (PACKAGE_ROOT / "environment_memory" / name).read_text(encoding="utf-8")


def test_query_server_is_read_only_and_uses_existing_interface():
    query_server = source("memory_query_server.py")
    retrieval = source("retrieval.py")

    assert '"/environment_memory/query"' in query_server
    assert "ReadOnlyChromaMemoryStore" in query_server
    assert ".upsert(" not in query_server
    assert ".add(" not in query_server
    assert ".delete(" not in query_server
    assert "get_collection(COLLECTION_NAME)" in retrieval
    assert "MAX_RESULTS = 5" in retrieval


def test_command_manager_reuses_speech_and_routes_motion_only_through_nav2():
    manager = source("memory_command_manager.py")

    assert '"/audio_events"' in manager
    assert '"/stt_action"' in manager
    assert '"/tts_action"' in manager
    assert '"/compute_path_to_pose"' in manager
    assert '"/navigate_to_pose"' in manager
    assert "NavigateToPose.Goal()" in manager
    assert "cmd_vel" not in manager
    assert "RunVlm" not in manager

    launch = (
        PACKAGE_ROOT / "launch" / "assistant_runtime.launch.py"
    ).read_text(encoding="utf-8")
    assert "speech_services.launch.py" in launch
    assert "vlm_server.launch.py" in launch
    assert "voice_pipeline.launch.py" not in launch
    assert "audio_loopback_node" not in launch


def test_v1_approach_limits_and_no_deferred_features():
    approach = source("approach_pose.py")
    retrieval = source("retrieval.py")
    manager = source("memory_command_manager.py")

    assert "(0.8, 1.0, 1.2)" in approach
    combined = approach + retrieval + manager
    assert "agentic" not in combined.lower()
    assert "scene_graph" not in combined.lower()
    assert "visual_embedding" not in combined.lower()
