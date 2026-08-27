from types import SimpleNamespace

import pytest

from environment_memory.semantic_batch import (
    SemanticBatchAssembler,
    SemanticBatchError,
    join_semantics,
)


def stamp(value=123):
    return SimpleNamespace(sec=0, nanosec=value)


def detection(detection_id, observation_id="obs-1"):
    return SimpleNamespace(
        detection_id=detection_id,
        observation_id=observation_id,
    )


def geometry(detection_id, observation_id="obs-1"):
    return SimpleNamespace(
        observation_id=observation_id,
        observation_stamp=stamp(),
        detection=detection(detection_id, observation_id),
        map_position=SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(), frame_id="map")
        ),
        robot_pose=SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(), frame_id="map")
        ),
    )


def evidence(ids=(1, 2), observation_id="obs-1"):
    return SimpleNamespace(
        observation_id=observation_id,
        observation_stamp=stamp(),
        image=SimpleNamespace(
            header=SimpleNamespace(stamp=stamp()), format="jpeg", data=b"x"
        ),
        detections=[detection(value, observation_id) for value in ids],
    )


def semantic(detection_id, useful=True):
    return SimpleNamespace(detection_id=detection_id, useful=useful)


def test_batch_joins_evidence_and_geometry_in_either_arrival_order():
    assembler = SemanticBatchAssembler()
    assembler.add_evidence(evidence())
    assembler.add_geometry(geometry(2))
    assert assembler.pop_ready() == []
    assembler.add_geometry(geometry(1))

    batches = assembler.pop_ready()

    assert len(batches) == 1
    assert batches[0].observation_id == "obs-1"
    assert [item.detection.detection_id for item in batches[0].geometries] == [1, 2]
    assert assembler.pending_count == 0


def test_join_requires_detection_id_and_discards_not_useful_semantics():
    assembler = SemanticBatchAssembler()
    assembler.add_geometry(geometry(1))
    assembler.add_geometry(geometry(2))
    assembler.add_evidence(evidence())
    batch = assembler.pop_ready()[0]

    joined = join_semantics(batch, [semantic(2, False), semantic(1, True)])

    assert len(joined) == 1
    assert joined[0][0].detection.detection_id == 1
    with pytest.raises(SemanticBatchError, match="no matching geometry"):
        join_semantics(batch, [semantic(99)])


def test_mismatched_ids_timestamps_duplicates_and_expiration_are_rejected():
    assembler = SemanticBatchAssembler()
    bad_geometry = geometry(1)
    bad_geometry.detection.observation_id = "wrong"
    with pytest.raises(SemanticBatchError, match="observation_id"):
        assembler.add_geometry(bad_geometry)

    assembler.add_geometry(geometry(1), now=1.0)
    with pytest.raises(SemanticBatchError, match="duplicate geometry"):
        assembler.add_geometry(geometry(1), now=1.1)
    assert assembler.expire(2.0, now=4.0) == 1
    assert assembler.pending_count == 0
