"""Checkpoint and finalize environment-memory manifests atomically."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile


MANIFEST_SCHEMA = "environment_memory.manifest.v1"


class ManifestError(RuntimeError):
    pass


@dataclass(frozen=True)
class ManifestState:
    environment_id: str
    map_id: str
    status: str
    map_yaml: str
    map_checksum: str
    database_path: str
    object_count: int
    created_at_utc: str


class ManifestManager:
    def __init__(self, environment_root: Path) -> None:
        self.root = environment_root.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / "manifest.json"

    def load(self) -> ManifestState | None:
        if not self.path.exists():
            return None
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if payload.pop("schema_version") != MANIFEST_SCHEMA:
                raise ManifestError("unsupported manifest schema")
            state = ManifestState(**payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ManifestError(f"invalid manifest: {exc}") from exc
        if state.status not in {"incomplete", "complete"}:
            raise ManifestError("manifest status must be incomplete or complete")
        return state

    def start_or_resume(
        self, environment_id: str, map_id: str, object_count: int
    ) -> ManifestState:
        existing = self.load()
        if existing is not None:
            if existing.environment_id != environment_id:
                raise ManifestError("manifest environment_id does not match")
            if existing.status == "complete":
                raise ManifestError("completed memory cannot be reopened writable")
            if existing.map_id != map_id:
                raise ManifestError("incomplete manifest map_id does not match")
            state = ManifestState(
                environment_id=environment_id,
                map_id=map_id,
                status="incomplete",
                map_yaml=existing.map_yaml,
                map_checksum=existing.map_checksum,
                database_path="chroma",
                object_count=object_count,
                created_at_utc=existing.created_at_utc,
            )
        else:
            state = ManifestState(
                environment_id=environment_id,
                map_id=map_id,
                status="incomplete",
                map_yaml="",
                map_checksum="",
                database_path="chroma",
                object_count=object_count,
                created_at_utc=datetime.now(timezone.utc).isoformat(),
            )
        self.write(state)
        return state

    def checkpoint(self, state: ManifestState, object_count: int) -> ManifestState:
        updated = ManifestState(
            **{**state.__dict__, "status": "incomplete", "object_count": object_count}
        )
        self.write(updated)
        return updated

    def complete(
        self, state: ManifestState, map_yaml: Path, object_count: int
    ) -> ManifestState:
        map_yaml = map_yaml.expanduser().resolve()
        if not map_yaml.is_file():
            raise ManifestError(f"saved map YAML does not exist: {map_yaml}")
        try:
            relative = map_yaml.relative_to(self.root).as_posix()
        except ValueError as exc:
            raise ManifestError("saved map YAML must be inside environment root") from exc
        checksum = hashlib.sha256(map_yaml.read_bytes()).hexdigest()
        completed = ManifestState(
            environment_id=state.environment_id,
            map_id=state.map_id,
            status="complete",
            map_yaml=relative,
            map_checksum=checksum,
            database_path="chroma",
            object_count=object_count,
            created_at_utc=state.created_at_utc,
        )
        self.write(completed)
        return completed

    def write(self, state: ManifestState) -> None:
        payload = {"schema_version": MANIFEST_SCHEMA, **state.__dict__}
        temporary_name = ""
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", newline="\n", prefix=".manifest-",
                suffix=".tmp", dir=self.root, delete=False,
            ) as temporary:
                json.dump(payload, temporary, ensure_ascii=False, indent=2, sort_keys=True)
                temporary.write("\n")
                temporary.flush()
                os.fsync(temporary.fileno())
                temporary_name = temporary.name
            os.replace(temporary_name, self.path)
            temporary_name = ""
        finally:
            if temporary_name:
                Path(temporary_name).unlink(missing_ok=True)
