import json

from environment_memory.perception.observation_log import (
    OBSERVATION_LOG_SCHEMA,
    ObservationLifecycleLog,
)


def test_observation_log_is_append_only_and_traceable(tmp_path) -> None:
    path = tmp_path / "hotel" / "observations.jsonl"
    log = ObservationLifecycleLog(path, "hotel", "map-1")

    log.append("obs-1", "CAPTURED", rgb_stamp_ns=123)
    log.append("obs-1", "STORED", detection_id=0, object_id="object-1")

    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["state"] for row in rows] == ["CAPTURED", "STORED"]
    assert all(row["schema_version"] == OBSERVATION_LOG_SCHEMA for row in rows)
    assert all(row["environment_id"] == "hotel" for row in rows)
    assert all(row["map_id"] == "map-1" for row in rows)
    assert rows[1]["object_id"] == "object-1"
