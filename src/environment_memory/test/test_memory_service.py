from dataclasses import replace

import pytest

from environment_memory.memory_service import MemoryService
from environment_memory.memory_store import InMemoryStore
from environment_memory.memory_record import MapPosition
from test_memory_record import observation


JPEG = b"\xff\xd8phase-seven-fixture\xff\xd9"


class ConstantEmbedder:
    model_name = "sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2"
    revision = "test"
    dimension = 384

    def encode(self, _text):
        return (1.0,) + (0.0,) * 383


class FailingStore(InMemoryStore):
    def upsert(self, record, embedding, document):
        raise RuntimeError("simulated Chroma failure")


def service(tmp_path, store=None):
    return MemoryService(
        tmp_path,
        "hotel_demo",
        "map-session",
        store or InMemoryStore(),
        ConstantEmbedder(),
    )


def test_create_merge_separate_and_checkpoint(tmp_path):
    memory = service(tmp_path)
    first = memory.upsert(observation(), JPEG)
    merged = memory.upsert(
        observation(
            observation_id="observation-2",
            observed_ros_ns=124_000_000_000,
            map_position=MapPosition("map", 5.6, 3.2, 0.9),
        ),
        JPEG,
    )
    separate = memory.upsert(
        observation(
            observation_id="observation-3",
            observed_ros_ns=125_000_000_000,
            map_position=MapPosition("map", 7.0, 3.2, 0.9),
        ),
        JPEG,
    )

    assert first.created is True
    assert merged.created is False
    assert merged.record.object_id == first.record.object_id
    assert merged.record.seen_count == 2
    assert separate.created is True
    assert memory.store.count() == 2
    assert memory.manifest.status == "incomplete"
    assert memory.manifest.object_count == 2


def test_low_confidence_merge_reuses_existing_keyframe(tmp_path):
    memory = service(tmp_path)
    first = memory.upsert(observation(), JPEG)
    weaker = replace(
        observation(
            observation_id="observation-2", observed_ros_ns=124_000_000_000
        ),
        detector_confidence=0.4,
        semantic_confidence=0.4,
        localization_quality=0.4,
    )

    merged = memory.upsert(weaker, b"not-needed")

    assert merged.record.image_ref == first.record.image_ref
    assert len(list((tmp_path / "images").glob("*.jpg"))) == 1


def test_replayed_observation_does_not_increment_seen_count(tmp_path):
    memory = service(tmp_path)
    first = memory.upsert(observation(), JPEG)

    with pytest.raises(ValueError, match="already processed"):
        memory.upsert(observation(), JPEG)

    assert memory.store.count() == 1
    assert memory.store.all()[0].record.object_id == first.record.object_id
    assert memory.store.all()[0].record.seen_count == 1


def test_failed_upsert_rolls_back_new_keyframe(tmp_path):
    memory = service(tmp_path, FailingStore())

    with pytest.raises(RuntimeError, match="simulated Chroma"):
        memory.upsert(observation(), JPEG)

    assert list((tmp_path / "images").glob("*.jpg")) == []


def test_invalid_observation_never_writes_keyframe(tmp_path):
    memory = service(tmp_path)

    with pytest.raises(ValueError, match="map_position frame"):
        memory.upsert(
            observation(map_position=MapPosition("odom", 1.0, 2.0, 3.0)), JPEG
        )

    assert memory.store.count() == 0
    assert list((tmp_path / "images").glob("*.jpg")) == []


def test_restart_recovery_and_finalization(tmp_path):
    store = InMemoryStore()
    memory = service(tmp_path, store)
    memory.upsert(observation(), JPEG)

    recovered = service(tmp_path, store)
    assert recovered.store.count() == 1
    recovered.stop_accepting_and_flush()
    map_directory = tmp_path / "maps"
    map_directory.mkdir()
    map_yaml = map_directory / "hotel.yaml"
    map_yaml.write_text("image: hotel.pgm\n", encoding="utf-8")
    manifest = recovered.finalize(map_yaml)

    assert manifest.status == "complete"
    assert manifest.object_count == 1
    with pytest.raises(RuntimeError, match="stopped"):
        recovered.upsert(observation(), JPEG)
