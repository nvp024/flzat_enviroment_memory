"""Assemble geometry and image evidence into semantic batches."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any


class SemanticBatchError(ValueError):
    pass


@dataclass(frozen=True)
class SemanticObservationBatch:
    observation_id: str
    observation_stamp_ns: int
    evidence: Any
    geometries: tuple[Any, ...]


@dataclass
class _PartialBatch:
    observation_id: str
    observation_stamp_ns: int
    updated_monotonic: float
    evidence: Any = None
    expected_detection_ids: set[int] = field(default_factory=set)
    geometries: dict[int, Any] = field(default_factory=dict)


class SemanticBatchAssembler:
    """Join Phase 5 evidence and geometry without relying on topic arrival order."""

    def __init__(self, maximum_batches: int = 4) -> None:
        if maximum_batches < 2:
            raise ValueError("maximum_batches must be at least two")
        self.maximum_batches = maximum_batches
        self._partials: dict[str, _PartialBatch] = {}

    @property
    def pending_count(self) -> int:
        return len(self._partials)

    def add_geometry(self, message: Any, now: float | None = None) -> None:
        observation_id = _observation_id(message)
        stamp_ns = _stamp_ns(message.observation_stamp)
        detection = message.detection
        if detection.observation_id != observation_id:
            raise SemanticBatchError("geometry detector observation_id does not match")
        if _stamp_ns(message.map_position.header.stamp) != stamp_ns:
            raise SemanticBatchError("geometry map timestamp does not match observation")
        if _stamp_ns(message.robot_pose.header.stamp) != stamp_ns:
            raise SemanticBatchError("geometry robot timestamp does not match observation")
        if message.map_position.header.frame_id != "map":
            raise SemanticBatchError("geometry must use the map frame")
        partial = self._partial(observation_id, stamp_ns, now)
        detection_id = int(detection.detection_id)
        if (
            partial.expected_detection_ids
            and detection_id not in partial.expected_detection_ids
        ):
            raise SemanticBatchError("geometry detection_id was not supplied as evidence")
        if detection_id in partial.geometries:
            raise SemanticBatchError("duplicate geometry detection_id")
        partial.geometries[detection_id] = message

    def add_evidence(self, message: Any, now: float | None = None) -> None:
        observation_id = _observation_id(message)
        stamp_ns = _stamp_ns(message.observation_stamp)
        if _stamp_ns(message.image.header.stamp) != stamp_ns:
            raise SemanticBatchError("evidence image timestamp does not match observation")
        if not message.image.data or "jpeg" not in message.image.format.lower():
            raise SemanticBatchError("VLM evidence must be a JPEG image")
        detection_ids = set()
        for detection in message.detections:
            if detection.observation_id != observation_id:
                raise SemanticBatchError("evidence detector observation_id does not match")
            detection_id = int(detection.detection_id)
            if detection_id in detection_ids:
                raise SemanticBatchError("duplicate evidence detection_id")
            detection_ids.add(detection_id)
        if not detection_ids or len(detection_ids) > 8:
            raise SemanticBatchError("evidence requires one to eight detections")
        partial = self._partial(observation_id, stamp_ns, now)
        if partial.evidence is not None:
            raise SemanticBatchError("duplicate VLM evidence")
        unknown_geometry = set(partial.geometries) - detection_ids
        if unknown_geometry:
            raise SemanticBatchError("geometry detection IDs do not match evidence")
        partial.evidence = message
        partial.expected_detection_ids = detection_ids

    def pop_ready(self) -> list[SemanticObservationBatch]:
        ready = []
        for observation_id, partial in list(self._partials.items()):
            if partial.evidence is None:
                continue
            if set(partial.geometries) != partial.expected_detection_ids:
                continue
            ready.append(
                SemanticObservationBatch(
                    observation_id=observation_id,
                    observation_stamp_ns=partial.observation_stamp_ns,
                    evidence=partial.evidence,
                    geometries=tuple(
                        partial.geometries[detection_id]
                        for detection_id in sorted(partial.geometries)
                    ),
                )
            )
            del self._partials[observation_id]
        ready.sort(key=lambda item: (item.observation_stamp_ns, item.observation_id))
        return ready

    def expire(self, maximum_age_s: float, now: float | None = None) -> int:
        if maximum_age_s <= 0.0:
            raise ValueError("maximum age must be positive")
        current = time.monotonic() if now is None else now
        expired = [
            observation_id
            for observation_id, partial in self._partials.items()
            if current - partial.updated_monotonic > maximum_age_s
        ]
        for observation_id in expired:
            del self._partials[observation_id]
        return len(expired)

    def _partial(
        self, observation_id: str, stamp_ns: int, now: float | None
    ) -> _PartialBatch:
        current = time.monotonic() if now is None else now
        partial = self._partials.get(observation_id)
        if partial is None:
            if len(self._partials) >= self.maximum_batches:
                raise SemanticBatchError("semantic batch assembler is full")
            partial = _PartialBatch(observation_id, stamp_ns, current)
            self._partials[observation_id] = partial
        elif partial.observation_stamp_ns != stamp_ns:
            raise SemanticBatchError("observation_id was reused with another timestamp")
        partial.updated_monotonic = current
        return partial


def join_semantics(
    batch: SemanticObservationBatch, semantic_objects: tuple[Any, ...] | list[Any]
) -> list[tuple[Any, Any]]:
    geometry_by_id = {
        int(message.detection.detection_id): message for message in batch.geometries
    }
    joined = []
    seen_ids = set()
    for semantic in semantic_objects:
        detection_id = int(semantic.detection_id)
        if detection_id in seen_ids:
            raise SemanticBatchError("duplicate semantic detection_id")
        seen_ids.add(detection_id)
        geometry = geometry_by_id.get(detection_id)
        if geometry is None:
            raise SemanticBatchError("semantic detection_id has no matching geometry")
        if semantic.useful:
            joined.append((geometry, semantic))
    return joined


def _observation_id(message: Any) -> str:
    value = str(message.observation_id)
    if not value:
        raise SemanticBatchError("observation_id cannot be empty")
    return value


def _stamp_ns(stamp: Any) -> int:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if value <= 0:
        raise SemanticBatchError("observation timestamp must be positive")
    return value
