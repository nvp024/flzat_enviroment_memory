import hashlib
import json

import pytest

from environment_memory.storage.manifest import ManifestError, ManifestManager


def test_incomplete_checkpoint_and_atomic_completion(tmp_path):
    manager = ManifestManager(tmp_path)
    state = manager.start_or_resume("hotel_demo", "map-1", 0)
    state = manager.checkpoint(state, 3)
    map_directory = tmp_path / "maps"
    map_directory.mkdir()
    map_yaml = map_directory / "hotel.yaml"
    map_yaml.write_text("image: hotel.pgm\n", encoding="utf-8")

    complete = manager.complete(state, map_yaml, 3)
    payload = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))

    assert complete.status == "complete"
    assert payload["schema_version"] == "environment_memory.manifest.v1"
    assert payload["map_yaml"] == "maps/hotel.yaml"
    assert payload["map_checksum"] == hashlib.sha256(map_yaml.read_bytes()).hexdigest()
    assert payload["object_count"] == 3
    assert list(tmp_path.glob(".manifest-*.tmp")) == []


def test_resumes_incomplete_but_refuses_completed_or_other_map(tmp_path):
    manager = ManifestManager(tmp_path)
    state = manager.start_or_resume("hotel_demo", "map-1", 2)
    assert manager.start_or_resume("hotel_demo", "map-1", 4).object_count == 4
    with pytest.raises(ManifestError, match="map_id"):
        manager.start_or_resume("hotel_demo", "map-2", 4)

    map_directory = tmp_path / "maps"
    map_directory.mkdir()
    map_yaml = map_directory / "hotel.yaml"
    map_yaml.write_text("image: hotel.pgm\n", encoding="utf-8")
    manager.complete(state, map_yaml, 4)
    with pytest.raises(ManifestError, match="completed"):
        manager.start_or_resume("hotel_demo", "map-1", 4)
