"""Exercise the Step 2 numeric and persistence path with a seeded SDF oracle."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path

import numpy as np

from environment_memory.perception.depth_localization import (
    CameraIntrinsics,
    localize_detection,
)
from environment_memory.perception.grounding_contract import (
    validate_grounding_detections,
)
from environment_memory.perception.transform_geometry import (
    RigidTransform,
    transform_point,
)
from environment_memory.storage.memory_record import (
    IncomingMemoryObservation,
    MapPosition,
    RobotPose,
)
from environment_memory.storage.memory_service import MemoryService
from environment_memory.storage.memory_store import InMemoryStore
from vlm_pipeline.grounding_schema import parse_grounding_response


JPEG = b"\xff\xd8step-two-replay\xff\xd9"


@dataclass
class DetectionMessage:
    observation_id: str
    detection_id: int
    detector_class: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int


class ConstantEmbedder:
    """Return stable vectors so this test isolates spatial deduplication."""

    model_name = "sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2"
    revision = "test"
    dimension = 384

    @staticmethod
    def encode(_text):
        return (1.0,) + (0.0,) * 383


def test_bbox_depth_tf_and_three_view_dedup_replay(tmp_path) -> None:
    oracle_path = Path(__file__).parent / "fixtures" / "hotel_demo_14_seeded_oracle.json"
    oracle = json.loads(oracle_path.read_text(encoding="utf-8"))
    target = tuple(oracle["objects"]["fan"])
    parsed = parse_grounding_response(
        '{"objects":[{"label":"floor_fan","bbox_2d":[200,200,800,800]}]}',
        100,
        80,
    )
    grounded = parsed.objects[0]
    message = DetectionMessage(
        observation_id="replay-1",
        detection_id=0,
        detector_class=grounded.label,
        confidence=grounded.confidence,
        x_min=grounded.pixel_bbox[0],
        y_min=grounded.pixel_bbox[1],
        x_max=grounded.pixel_bbox[2],
        y_max=grounded.pixel_bbox[3],
    )
    detection = validate_grounding_detections(
        "replay-1", 100, 80, [message]
    )[0]
    depth = np.full((80, 100), 0.5, dtype=np.float32)
    localized = localize_detection(
        depth,
        (detection.x_min, detection.y_min, detection.x_max, detection.y_max),
        CameraIntrinsics(100, 80, 100.0, 100.0, 49.5, 39.5),
    )
    camera_point = (localized.x, localized.y, localized.z)
    camera_tf = RigidTransform(
        translation=(
            target[0] - camera_point[0],
            target[1] - camera_point[1],
            target[2] - camera_point[2],
        ),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    map_point = transform_point(camera_point, camera_tf)
    later_robot_camera_tf = RigidTransform(
        translation=(50.0, 50.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    wrong_late_point = transform_point(camera_point, later_robot_camera_tf)

    assert math.dist(map_point, target) <= 0.20
    assert math.dist(wrong_late_point, target) > 1.0
    assert detection.detector_class == "fan"

    service = MemoryService(
        tmp_path,
        "step2_replay",
        "map-replay",
        InMemoryStore(),
        ConstantEmbedder(),
    )
    started = datetime(2026, 9, 10, tzinfo=timezone.utc)
    for index, dx in enumerate((0.0, 0.04, -0.03), start=1):
        service.upsert(
            IncomingMemoryObservation(
                environment_id="step2_replay",
                map_id="map-replay",
                observation_id=f"replay-{index}",
                detector_class="fan",
                label="fan",
                description="visually grounded fan",
                attributes=(),
                relationships=(),
                scene="indoor_environment",
                map_position=MapPosition(
                    "map", map_point[0] + dx, map_point[1], map_point[2]
                ),
                robot_pose=RobotPose(0.0, 0.0, 0.0, 0.0),
                observed_utc=(started + timedelta(seconds=index)).isoformat(),
                observed_ros_ns=index * 1_000_000_000,
                detector_confidence=0.5,
                semantic_confidence=0.5,
                localization_quality=localized.localization_quality,
            ),
            JPEG,
        )

    records = service.store.all()
    assert len(records) == 1
    assert records[0].record.seen_count == 3
    assert records[0].record.environment_id == "step2_replay"
