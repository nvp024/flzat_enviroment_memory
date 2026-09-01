"""Persist JPEG observation evidence atomically."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile


class KeyframeError(ValueError):
    pass


class KeyframeStore:
    JPEG_MAGIC = b"\xff\xd8"

    def __init__(self, environment_root: Path) -> None:
        self._root = environment_root.resolve()
        self._images = self._root / "images"
        self._images.mkdir(parents=True, exist_ok=True)

    def write_jpeg(self, observation_id: str, data: bytes) -> tuple[str, bool]:
        if not observation_id or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
            for character in observation_id
        ):
            raise KeyframeError("observation_id is not safe for a keyframe name")
        if len(data) < 4 or not data.startswith(self.JPEG_MAGIC) or not data.endswith(b"\xff\xd9"):
            raise KeyframeError("keyframe must be a nonempty JPEG image")
        relative = Path("images") / f"observation_{observation_id}.jpg"
        destination = (self._root / relative).resolve()
        if self._root not in destination.parents:
            raise KeyframeError("keyframe destination escapes environment root")
        if destination.exists():
            if destination.read_bytes() != data:
                raise KeyframeError("observation keyframe already exists with other bytes")
            return relative.as_posix(), False

        temporary_name = ""
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", prefix=".keyframe-", suffix=".tmp", dir=self._images,
                delete=False,
            ) as temporary:
                temporary.write(data)
                temporary.flush()
                os.fsync(temporary.fileno())
                temporary_name = temporary.name
            os.replace(temporary_name, destination)
            temporary_name = ""
        finally:
            if temporary_name:
                Path(temporary_name).unlink(missing_ok=True)
        return relative.as_posix(), True

    def remove_if_created(self, image_ref: str, created: bool) -> None:
        if not created:
            return
        destination = (self._root / image_ref).resolve()
        if self._root not in destination.parents:
            raise KeyframeError("rollback path escapes environment root")
        destination.unlink(missing_ok=True)
