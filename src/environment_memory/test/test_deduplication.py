from dataclasses import replace

import pytest

from environment_memory.storage.deduplication import merge_record, select_duplicate
from environment_memory.storage.memory_record import MapPosition, new_record
from test_memory_record import observation


def unit_vector(index):
    values = [0.0] * 384
    values[index] = 1.0
    return tuple(values)


def test_selects_closest_matching_same_class_record():
    incoming = observation(map_position=MapPosition("map", 0.0, 0.0, 0.0))
    far_match = new_record(
        replace(
            incoming,
            observation_id="observation-2",
            map_position=MapPosition("map", 0.50, 0.0, 0.0),
        ),
        "object-far",
        "images/far.jpg",
    )
    close_match = new_record(
        replace(
            incoming,
            observation_id="observation-3",
            map_position=MapPosition("map", 0.20, 0.0, 0.0),
        ),
        "object-close",
        "images/close.jpg",
    )

    result = select_duplicate(
        incoming,
        unit_vector(0),
        [(far_match, unit_vector(0)), (close_match, unit_vector(0))],
    )

    assert result.object_id == "object-close"


def test_separates_far_low_similarity_and_different_class_objects():
    incoming = observation(map_position=MapPosition("map", 0.0, 0.0, 0.0))
    far = new_record(
        replace(incoming, map_position=MapPosition("map", 0.61, 0.0, 0.0)),
        "far",
        "images/far.jpg",
    )
    low_similarity = new_record(incoming, "other", "images/other.jpg")
    other_class = replace(low_similarity, detector_class="cup")

    assert select_duplicate(
        incoming,
        unit_vector(0),
        [
            (far, unit_vector(0)),
            (low_similarity, unit_vector(1)),
            (other_class, unit_vector(0)),
        ],
    ) is None


def test_merge_updates_counts_weighted_position_and_best_evidence():
    existing = new_record(
        observation(
            map_position=MapPosition("map", 0.0, 0.0, 0.0),
            detector_confidence=0.6,
            semantic_confidence=0.6,
            localization_quality=0.6,
        ),
        "object-1",
        "images/old.jpg",
    )
    incoming = observation(
        observation_id="observation-2",
        description="A better description.",
        attributes=(("color", "navy"), ("material", "plastic")),
        relationships=("on the counter", "beside the sink"),
        map_position=MapPosition("map", 0.4, 0.0, 0.0),
        observed_ros_ns=124_000_000_000,
        detector_confidence=0.9,
        semantic_confidence=0.9,
        localization_quality=0.9,
    )

    merged = merge_record(existing, incoming, "images/new.jpg")

    assert merged.object_id == existing.object_id
    assert merged.seen_count == 2
    assert merged.map_position.x == pytest.approx(0.24)
    assert merged.description == "A better description."
    assert dict(merged.attributes) == {"color": "navy", "material": "plastic"}
    assert merged.image_ref == "images/new.jpg"
    assert merged.confidence == pytest.approx(0.9)
