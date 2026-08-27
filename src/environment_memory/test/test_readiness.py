from environment_memory.readiness import ReadinessSnapshot


def test_readiness_lists_every_missing_contract():
    snapshot = ReadinessSnapshot(map_received=True, scan_received=True)

    assert not snapshot.ready
    assert "map" not in snapshot.missing
    assert "navigate_to_pose" in snapshot.missing
    assert "base_link->camera_optical_frame TF" in snapshot.missing


def test_readiness_is_true_only_when_all_contracts_are_available():
    snapshot = ReadinessSnapshot(
        True, True, True, True, True, True, True, True, True, True, True
    )

    assert snapshot.ready
    assert snapshot.missing == ()


def test_memory_manager_is_part_of_build_readiness():
    snapshot = ReadinessSnapshot(
        True, True, True, True, True, True, True, True, True, False, True
    )

    assert not snapshot.ready
    assert "writable Memory Manager" in snapshot.missing


def test_structured_vlm_pipeline_is_part_of_build_readiness():
    snapshot = ReadinessSnapshot(
        True, True, True, True, True, True, True, True, True, True, False
    )

    assert not snapshot.ready
    assert "structured VLM pipeline" in snapshot.missing
