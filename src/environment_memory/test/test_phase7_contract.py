from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
INTERFACE_ROOT = WORKSPACE_ROOT / "src" / "environment_memory_interfaces"


def test_phase7_dependencies_and_model_revision_are_pinned():
    requirements = (WORKSPACE_ROOT / "requirements-memory.txt").read_text(
        encoding="utf-8"
    )
    record = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "memory_record.py"
    ).read_text(encoding="utf-8")

    assert "numpy==1.26.4" in requirements
    assert "chromadb==1.5.9" in requirements
    assert "sentence-transformers==6.0.0" in requirements
    assert "paraphrase-multilingual-MiniLM-L12-v2" in record
    assert "e8f8c211226b894fcb81acc59f3b34ba3efd5f42" in record
    assert "EMBEDDING_DIMENSION = 384" in record


def test_localized_observation_carries_keyframe_evidence():
    interface = (
        INTERFACE_ROOT / "msg" / "LocalizedObjectObservation.msg"
    ).read_text(encoding="utf-8")
    cmake = (INTERFACE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "sensor_msgs/CompressedImage image" in interface
    assert "sensor_msgs" in cmake


def test_memory_manager_is_write_only_phase7_component():
    source = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "memory_manager.py"
    ).read_text(encoding="utf-8")
    launch = (
        PACKAGE_ROOT / "launch" / "exploration_observation.launch.py"
    ).read_text(encoding="utf-8")

    assert '"/environment_memory/localized_observations"' in source
    assert '"/environment_memory/status"' in source
    assert 'executable="memory_manager"' in launch
    assert "semantic.detection_id != message.detection.detection_id" in source
    assert "QueryMemory" not in source
    assert "memory_command_manager" not in source
    assert "ActionClient" not in source

    build_manager = (
        PACKAGE_ROOT
        / "environment_memory"
        / "exploration"
        / "memory_build_manager.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("storage_root", "")' in build_manager
    assert '"/environment_memory/status"' in build_manager
    readiness = (
        PACKAGE_ROOT / "environment_memory" / "exploration" / "readiness.py"
    ).read_text(encoding="utf-8")
    assert "memory_manager_ready" in readiness


def test_planned_thresholds_collection_and_manifest_schema_are_locked():
    dedup = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "deduplication.py"
    ).read_text(encoding="utf-8")
    store = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "memory_store.py"
    ).read_text(encoding="utf-8")
    manifest = (
        PACKAGE_ROOT / "environment_memory" / "storage" / "manifest.py"
    ).read_text(encoding="utf-8")

    assert "SPATIAL_THRESHOLD_M = 0.60" in dedup
    assert "SEMANTIC_THRESHOLD = 0.80" in dedup
    assert 'COLLECTION_NAME = "environment_objects_v1"' in store
    assert 'MANIFEST_SCHEMA = "environment_memory.manifest.v1"' in manifest
