from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Protocol, Sequence

from environment_memory.memory_record import (
    EMBEDDING_DIMENSION,
    MemoryRecord,
    embedding_text,
)


COLLECTION_NAME = "environment_objects_v1"


@dataclass(frozen=True)
class StoredMemory:
    record: MemoryRecord
    embedding: tuple[float, ...]
    document: str


class MemoryStore(Protocol):
    def all(self) -> list[StoredMemory]: ...

    def same_class(
        self, environment_id: str, map_id: str, detector_class: str
    ) -> list[StoredMemory]: ...

    def upsert(
        self, record: MemoryRecord, embedding: Sequence[float], document: str
    ) -> None: ...

    def count(self) -> int: ...

    def flush(self) -> None: ...


class InMemoryStore:
    def __init__(self) -> None:
        self._items: dict[str, StoredMemory] = {}

    def all(self) -> list[StoredMemory]:
        return list(self._items.values())

    def same_class(
        self, environment_id: str, map_id: str, detector_class: str
    ) -> list[StoredMemory]:
        return [
            item
            for item in self._items.values()
            if item.record.environment_id == environment_id
            and item.record.map_id == map_id
            and item.record.detector_class == detector_class
        ]

    def upsert(
        self, record: MemoryRecord, embedding: Sequence[float], document: str
    ) -> None:
        self._items[record.object_id] = StoredMemory(
            record=record,
            embedding=tuple(float(value) for value in embedding),
            document=document,
        )

    def count(self) -> int:
        return len(self._items)

    def flush(self) -> None:
        return None


class ChromaMemoryStore:
    """Embedded Chroma adapter; the canonical record is JSON metadata."""

    def __init__(self, path: Path) -> None:
        try:
            import chromadb
        except ImportError as exc:
            raise RuntimeError(
                "chromadb is unavailable; install requirements-memory.txt"
            ) from exc
        path.mkdir(parents=True, exist_ok=True)
        self._client = chromadb.PersistentClient(path=str(path))
        self._collection = self._client.get_or_create_collection(
            name=COLLECTION_NAME,
            metadata={"hnsw:space": "cosine", "schema_version": "environment_memory.v1"},
        )
        self._cache: dict[str, StoredMemory] = {}
        self._recover()

    def _recover(self) -> None:
        result = self._collection.get(
            include=["embeddings", "metadatas", "documents"]
        )
        ids = result.get("ids") or []
        embeddings = result.get("embeddings")
        metadatas = result.get("metadatas") or []
        documents = result.get("documents") or []
        if embeddings is None:
            embeddings = []
        if not (len(ids) == len(embeddings) == len(metadatas) == len(documents)):
            raise RuntimeError("Chroma collection returned inconsistent record arrays")
        for object_id, embedding, metadata, document in zip(
            ids, embeddings, metadatas, documents
        ):
            if not metadata or "record_json" not in metadata:
                raise RuntimeError(f"stored object {object_id} lacks canonical record")
            record = MemoryRecord.from_json(str(metadata["record_json"]))
            if record.object_id != object_id:
                raise RuntimeError(f"stored object ID mismatch for {object_id}")
            vector = tuple(float(value) for value in embedding)
            if len(vector) != EMBEDDING_DIMENSION or not all(
                math.isfinite(value) for value in vector
            ):
                raise RuntimeError(
                    f"stored object {object_id} has an invalid embedding"
                )
            if str(document) != embedding_text(record):
                raise RuntimeError(
                    f"stored object {object_id} document does not match record"
                )
            self._cache[object_id] = StoredMemory(
                record=record,
                embedding=vector,
                document=str(document),
            )

    def all(self) -> list[StoredMemory]:
        return list(self._cache.values())

    def same_class(
        self, environment_id: str, map_id: str, detector_class: str
    ) -> list[StoredMemory]:
        return [
            item
            for item in self._cache.values()
            if item.record.environment_id == environment_id
            and item.record.map_id == map_id
            and item.record.detector_class == detector_class
        ]

    def upsert(
        self, record: MemoryRecord, embedding: Sequence[float], document: str
    ) -> None:
        vector = [float(value) for value in embedding]
        metadata = {
            "record_json": record.to_json(),
            "environment_id": record.environment_id,
            "map_id": record.map_id,
            "detector_class": record.detector_class,
            "scene": record.scene,
            "last_seen_ros_ns": record.last_seen_ros_ns,
        }
        self._collection.upsert(
            ids=[record.object_id],
            embeddings=[vector],
            metadatas=[metadata],
            documents=[document],
        )
        self._cache[record.object_id] = StoredMemory(
            record=record, embedding=tuple(vector), document=document
        )

    def count(self) -> int:
        return len(self._cache)

    def flush(self) -> None:
        # PersistentClient upserts synchronously; retained as a finalization seam.
        return None
