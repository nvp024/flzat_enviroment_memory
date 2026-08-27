#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
from urllib.request import urlopen


MODEL_URL = (
    "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt"
)
MODEL_SHA256 = "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36"
DEFAULT_OUTPUT = (
    Path.home()
    / ".local"
    / "share"
    / "flzat"
    / "environment_memory"
    / "models"
    / "yolov8n-v8.3.0.pt"
)


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fetch and verify the Phase 5 pinned YOLOv8n weights."
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists() and not args.force:
        if _sha256(output) == MODEL_SHA256:
            print(f"Model already verified: {output}")
            return 0
        parser.error(f"existing file has the wrong checksum: {output}; use --force")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            prefix="yolov8n-", suffix=".pt.part", dir=output.parent, delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            with urlopen(MODEL_URL, timeout=60) as response:
                shutil.copyfileobj(response, temporary)
        actual = _sha256(temporary_path)
        if actual != MODEL_SHA256:
            raise RuntimeError(
                f"download checksum mismatch: expected {MODEL_SHA256}, got {actual}"
            )
        temporary_path.replace(output)
        print(f"Verified model installed: {output}")
        return 0
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
