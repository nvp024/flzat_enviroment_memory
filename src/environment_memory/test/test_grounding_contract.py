from dataclasses import dataclass

import pytest

from environment_memory.perception.grounding_contract import (
    GroundingContractError,
    validate_grounding_detections,
)


@dataclass
class DetectionMessage:
    observation_id: str = "obs-1"
    detection_id: int = 0
    detector_class: str = "floor_fan"
    confidence: float = 0.5
    x_min: int = 10
    y_min: int = 20
    x_max: int = 100
    y_max: int = 120


def test_grounding_result_keeps_integer_pixel_xyxy() -> None:
    result = validate_grounding_detections(
        "obs-1", 640, 480, [DetectionMessage()]
    )

    assert (result[0].x_min, result[0].y_min) == (10, 20)
    assert (result[0].x_max, result[0].y_max) == (100, 120)


def test_late_result_from_another_observation_is_rejected() -> None:
    with pytest.raises(GroundingContractError, match="observation_id mismatch"):
        validate_grounding_detections(
            "obs-new",
            640,
            480,
            [DetectionMessage(observation_id="obs-old")],
        )


@pytest.mark.parametrize(
    "message",
    (
        DetectionMessage(x_max=700),
        DetectionMessage(x_min=100, x_max=100),
        DetectionMessage(detector_class=""),
        DetectionMessage(confidence=float("nan")),
    ),
)
def test_malformed_grounding_detection_is_rejected(message) -> None:
    with pytest.raises(GroundingContractError):
        validate_grounding_detections("obs-1", 640, 480, [message])


def test_duplicate_detection_id_is_rejected() -> None:
    with pytest.raises(GroundingContractError, match="duplicate detection_id"):
        validate_grounding_detections(
            "obs-1",
            640,
            480,
            [DetectionMessage(), DetectionMessage(detector_class="bench")],
        )
