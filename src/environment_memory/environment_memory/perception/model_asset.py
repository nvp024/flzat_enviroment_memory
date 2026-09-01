"""Resolve and verify pinned perception model assets."""

from __future__ import annotations

import hashlib
from pathlib import Path


class ModelAssetError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_verified_model(model_path: str, expected_sha256: str) -> Path:
    path = Path(model_path).expanduser()
    if not path.is_file():
        raise ModelAssetError(
            f"detector model not found at {path}; run tools/fetch_yolov8n.py"
        )
    expected = expected_sha256.strip().lower()
    if len(expected) != 64 or any(character not in "0123456789abcdef" for character in expected):
        raise ModelAssetError("detector_model_sha256 must be a 64-character hex digest")
    actual = sha256_file(path)
    if actual != expected:
        raise ModelAssetError(
            f"detector model SHA-256 mismatch for {path}: expected {expected}, got {actual}"
        )
    return path.resolve()
