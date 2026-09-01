"""Merge spatially and semantically duplicate object records."""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Sequence

from environment_memory.storage.memory_record import (
    IncomingMemoryObservation,
    MapPosition,
    MemoryRecord,
    with_confidences,
)


SPATIAL_THRESHOLD_M = 0.60
SEMANTIC_THRESHOLD = 0.80


def cosine_similarity(left: Sequence[float], right: Sequence[float]) -> float:
    if len(left) != len(right) or not left:
        raise ValueError("embeddings must have equal nonzero dimensions")
    dot = sum(float(a) * float(b) for a, b in zip(left, right))
    left_norm = math.sqrt(sum(float(value) ** 2 for value in left))
    right_norm = math.sqrt(sum(float(value) ** 2 for value in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise ValueError("embeddings must have nonzero norm")
    return max(-1.0, min(1.0, dot / (left_norm * right_norm)))


def map_distance(
    left: MapPosition, right: MapPosition
) -> float:
    if left.frame_id != "map" or right.frame_id != "map":
        raise ValueError("deduplication requires map-frame positions")
    return math.sqrt(
        (left.x - right.x) ** 2
        + (left.y - right.y) ** 2
        + (left.z - right.z) ** 2
    )


def select_duplicate(
    observation: IncomingMemoryObservation,
    observation_embedding: Sequence[float],
    candidates: Sequence[tuple[MemoryRecord, Sequence[float]]],
) -> MemoryRecord | None:
    matches = []
    for record, embedding in candidates:
        if (
            record.environment_id != observation.environment_id
            or record.map_id != observation.map_id
            or record.detector_class != observation.detector_class
        ):
            continue
        distance = map_distance(record.map_position, observation.map_position)
        if distance > SPATIAL_THRESHOLD_M:
            continue
        if cosine_similarity(embedding, observation_embedding) < SEMANTIC_THRESHOLD:
            continue
        matches.append((distance, record.object_id, record))
    return None if not matches else min(matches)[2]


def merge_record(
    existing: MemoryRecord,
    incoming: IncomingMemoryObservation,
    incoming_image_ref: str,
) -> MemoryRecord:
    old_weight = max(existing.confidence, 1e-6) * existing.seen_count
    new_weight = max(incoming.confidence, 1e-6)
    total_weight = old_weight + new_weight
    position = MapPosition(
        frame_id="map",
        x=(existing.map_position.x * old_weight + incoming.map_position.x * new_weight)
        / total_weight,
        y=(existing.map_position.y * old_weight + incoming.map_position.y * new_weight)
        / total_weight,
        z=(existing.map_position.z * old_weight + incoming.map_position.z * new_weight)
        / total_weight,
    )
    attributes = dict(existing.attributes)
    attributes.update(dict(incoming.attributes))
    relationships = tuple(
        dict.fromkeys(existing.relationships + incoming.relationships)
    )[:5]
    replace_evidence = incoming.confidence > existing.confidence
    merged = replace(
        existing,
        label=incoming.label if replace_evidence else existing.label,
        description=(
            incoming.description if replace_evidence else existing.description
        ),
        attributes=tuple(attributes.items())[:8],
        relationships=relationships,
        scene=incoming.scene if replace_evidence else existing.scene,
        map_position=position,
        robot_pose=incoming.robot_pose,
        last_seen_utc=incoming.observed_utc,
        last_seen_ros_ns=max(existing.last_seen_ros_ns, incoming.observed_ros_ns),
        seen_count=existing.seen_count + 1,
        detector_confidence=(
            incoming.detector_confidence
            if replace_evidence
            else existing.detector_confidence
        ),
        semantic_confidence=(
            incoming.semantic_confidence
            if replace_evidence
            else existing.semantic_confidence
        ),
        localization_quality=(
            incoming.localization_quality
            if replace_evidence
            else existing.localization_quality
        ),
        image_ref=incoming_image_ref if replace_evidence else existing.image_ref,
    )
    return with_confidences(merged)
