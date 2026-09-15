"""Append traceable observation lifecycle events as JSON Lines."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import threading
from typing import Any


OBSERVATION_LOG_SCHEMA = "environment_memory.observation_lifecycle.v1"


class ObservationLifecycleLog:
    """Small append-only writer shared by coordinator callbacks."""

    def __init__(
        self,
        path: Path,
        environment_id: str,
        map_id: str,
    ) -> None:
        self.path = path.expanduser().resolve()
        self.environment_id = environment_id
        self.map_id = map_id
        self._lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(
        self,
        observation_id: str,
        state: str,
        **details: Any,
    ) -> None:
        """Persist one state transition before its in-memory data is released."""
        payload = {
            "schema_version": OBSERVATION_LOG_SCHEMA,
            "logged_utc": datetime.now(timezone.utc).isoformat(),
            "environment_id": self.environment_id,
            "map_id": self.map_id,
            "observation_id": observation_id,
            "state": state,
            **details,
        }
        line = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        with self._lock:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
                stream.flush()
