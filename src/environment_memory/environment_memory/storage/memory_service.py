"""Coordinate single-writer memory transactions and checkpoints."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import uuid

from environment_memory.storage.deduplication import merge_record, select_duplicate
from environment_memory.storage.embedding import Embedder
from environment_memory.storage.keyframe_store import KeyframeStore
from environment_memory.storage.manifest import ManifestManager, ManifestState
from environment_memory.storage.memory_record import (
    IncomingMemoryObservation,
    MemoryRecord,
    embedding_text,
    new_record,
    validate_incoming,
    validate_record,
)
from environment_memory.storage.memory_store import MemoryStore


@dataclass(frozen=True)
class UpsertResult:
    record: MemoryRecord
    created: bool


class MemoryService:
    """Single-writer canonical memory transaction coordinator."""

    def __init__(
        self,
        environment_root: Path,
        environment_id: str,
        map_id: str,
        store: MemoryStore,
        embedder: Embedder,
    ) -> None:
        self._lock = threading.Lock()
        self.environment_root = environment_root.resolve()
        self.environment_id = environment_id
        self.map_id = map_id
        self.store = store
        self.embedder = embedder
        self.keyframes = KeyframeStore(self.environment_root)
        self.manifests = ManifestManager(self.environment_root)
        self.accepting = True
        self.active_upserts = 0
        self._validate_recovery()
        self.manifest = self.manifests.start_or_resume(
            environment_id, map_id, self.store.count()
        )

    def _validate_recovery(self) -> None:
        for item in self.store.all():
            validate_record(item.record)
            if item.record.environment_id != self.environment_id:
                raise RuntimeError("recovered record environment_id does not match")
            if item.record.map_id != self.map_id:
                raise RuntimeError("recovered record map_id does not match")
            image_path = (self.environment_root / item.record.image_ref).resolve()
            if self.environment_root not in image_path.parents or not image_path.is_file():
                raise RuntimeError(
                    f"recovered record {item.record.object_id} keyframe is missing"
                )

    def upsert(
        self, observation: IncomingMemoryObservation, jpeg_data: bytes
    ) -> UpsertResult:
        with self._lock:
            if not self.accepting:
                raise RuntimeError("memory finalization has stopped new observations")
            if observation.environment_id != self.environment_id:
                raise ValueError("observation environment_id does not match session")
            if observation.map_id != self.map_id:
                raise ValueError("observation map_id does not match session")
            self.active_upserts += 1
            try:
                return self._upsert_locked(observation, jpeg_data)
            finally:
                self.active_upserts -= 1

    def _upsert_locked(
        self, observation: IncomingMemoryObservation, jpeg_data: bytes
    ) -> UpsertResult:
        validate_incoming(observation)
        incoming_text = embedding_text(observation)
        incoming_embedding = self.embedder.encode(incoming_text)
        candidates = self.store.same_class(
            observation.environment_id,
            observation.map_id,
            observation.detector_class,
        )
        duplicate = select_duplicate(
            observation,
            incoming_embedding,
            [(item.record, item.embedding) for item in candidates],
        )
        if (
            duplicate is not None
            and observation.observed_ros_ns <= duplicate.last_seen_ros_ns
        ):
            raise ValueError("duplicate observation is stale or already processed")
        needs_new_evidence = duplicate is None or observation.confidence > duplicate.confidence
        image_ref = ""
        image_created = False
        if needs_new_evidence:
            image_ref, image_created = self.keyframes.write_jpeg(
                observation.observation_id, jpeg_data
            )
        else:
            image_ref = duplicate.image_ref

        try:
            if duplicate is None:
                record = new_record(observation, str(uuid.uuid4()), image_ref)
                created = True
            else:
                record = merge_record(duplicate, observation, image_ref)
                created = False
            validate_record(record)
            document = embedding_text(record)
            final_embedding = self.embedder.encode(document)
            self.store.upsert(record, final_embedding, document)
        except Exception:
            self.keyframes.remove_if_created(image_ref, image_created)
            raise
        self.manifest = self.manifests.checkpoint(
            self.manifest, self.store.count()
        )
        return UpsertResult(record=record, created=created)

    def stop_accepting_and_flush(self) -> None:
        with self._lock:
            self.accepting = False
            self.store.flush()
            self.manifest = self.manifests.checkpoint(
                self.manifest, self.store.count()
            )

    def finalize(self, map_yaml: Path) -> ManifestState:
        with self._lock:
            if self.accepting or self.active_upserts:
                raise RuntimeError("memory must be drained before finalization")
            self.store.flush()
            self.manifest = self.manifests.complete(
                self.manifest, map_yaml, self.store.count()
            )
            return self.manifest
