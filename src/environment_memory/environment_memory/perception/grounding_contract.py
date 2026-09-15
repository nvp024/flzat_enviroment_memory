"""Validate GroundObjects detections against their frozen RGB observation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class GroundedDetection:
    """Validated VLM detection expressed in frozen-image pixel coordinates."""

    detection_id: int
    detector_class: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int


class GroundingContractError(ValueError):
    """A grounding result cannot safely be paired with frozen sensor data."""


def validate_grounding_detections(
    observation_id: str,
    width: int,
    height: int,
    messages: Iterable[object],
    maximum_detections: int = 20,
) -> list[GroundedDetection]:
    """Require exact ID, unique detections and in-image integer pixel xyxy."""
    values = list(messages)
    if maximum_detections < 1:
        raise GroundingContractError("maximum_detections must be positive")
    if len(values) > maximum_detections:
        raise GroundingContractError("GroundObjects result exceeds detection limit")
    if width < 1 or height < 1:
        raise GroundingContractError("frozen RGB dimensions must be positive")
    detections = []
    seen_ids = set()
    for message in values:
        detection_id = int(message.detection_id)
        if detection_id in seen_ids:
            raise GroundingContractError(
                "GroundObjects result contains duplicate detection_id"
            )
        seen_ids.add(detection_id)
        if message.observation_id != observation_id:
            raise GroundingContractError(
                "grounded detection observation_id mismatch"
            )
        bounds = (
            int(message.x_min),
            int(message.y_min),
            int(message.x_max),
            int(message.y_max),
        )
        if not (
            0 <= bounds[0] < bounds[2] <= width
            and 0 <= bounds[1] < bounds[3] <= height
        ):
            raise GroundingContractError(
                "grounded detection bbox is outside the frozen RGB"
            )
        confidence = float(message.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise GroundingContractError(
                "grounded detection confidence is invalid"
            )
        label = str(message.detector_class).strip()
        if not label:
            raise GroundingContractError("grounded detection label is empty")
        detections.append(
            GroundedDetection(
                detection_id=detection_id,
                detector_class=label,
                confidence=confidence,
                x_min=bounds[0],
                y_min=bounds[1],
                x_max=bounds[2],
                y_max=bounds[3],
            )
        )
    return detections
