from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def source(group, name):
    return (
        PACKAGE_ROOT / "environment_memory" / group / name
    ).read_text(encoding="utf-8")


def test_query_server_is_read_only_and_uses_existing_interface():
    query_server = source("retrieval", "memory_query_server.py")
    retrieval = source("retrieval", "search.py")

    assert '"/environment_memory/query"' in query_server
    assert "ReadOnlyChromaMemoryStore" in query_server
    assert ".upsert(" not in query_server
    assert ".add(" not in query_server
    assert ".delete(" not in query_server
    assert "get_collection(COLLECTION_NAME)" in retrieval
    assert "MAX_RESULTS = 5" in retrieval


def test_completed_memory_coordinates_are_published_for_rviz():
    markers = source("retrieval", "memory_marker_publisher.py")
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    launch = (
        PACKAGE_ROOT / "launch" / "assistant_runtime.launch.py"
    ).read_text(encoding="utf-8")

    assert "visualization_msgs.msg import Marker, MarkerArray" in markers
    assert 'MARKER_TOPIC = "/environment_memory/object_markers"' in markers
    assert 'clear.action = Marker.DELETEALL' in markers
    assert 'point.header.frame_id = "map"' in markers
    assert "ReadOnlyChromaMemoryStore" in markers
    assert ".upsert(" not in markers
    assert "memory_marker_publisher" in setup
    assert 'executable="memory_marker_publisher"' in launch


def test_command_manager_reuses_speech_and_routes_motion_only_through_nav2():
    manager = source("assistant", "memory_command_manager.py")

    assert '"/audio_events"' in manager
    assert '"/environment_memory/text_command"' in manager
    assert "def _on_text_command(" in manager
    assert "self._handle_transcript(message.data)" in manager
    assert "self._handle_transcript(transcript)" in manager
    assert "self._log_query_ranking(query_text, matches)" in manager
    assert 'f"rank={rank}, score={match.score:.4f}' in manager
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
    approach = source("assistant", "approach_pose.py")
    retrieval = source("retrieval", "search.py")
    manager = source("assistant", "memory_command_manager.py")

    assert "(0.8, 1.0, 1.2)" in approach
    combined = approach + retrieval + manager
    assert "agentic" not in combined.lower()
    assert "scene_graph" not in combined.lower()
    assert "visual_embedding" not in combined.lower()
