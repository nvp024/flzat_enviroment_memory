"""Expose the configured YOLO object-detection boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, Sequence

import numpy as np


@dataclass(frozen=True)
class Detection2D:
    detection_id: int
    detector_class: str
    class_id: int
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


@dataclass(frozen=True)
class DetectorConfig:
    confidence_threshold: float = 0.35
    nms_iou_threshold: float = 0.50
    max_detections: int = 8
    ignored_classes: tuple[str, ...] = ("person",)

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not 0.0 <= self.nms_iou_threshold <= 1.0:
            raise ValueError("nms_iou_threshold must be in [0, 1]")
        if self.max_detections < 1:
            raise ValueError("max_detections must be positive")


class Detector(Protocol):
    def detect(self, bgr_image: np.ndarray) -> Sequence[Detection2D]: ...


class UltralyticsYoloDetector:
    """Small adapter that keeps Ultralytics-specific objects out of ROS code."""

    def __init__(
        self,
        model_path: str,
        config: DetectorConfig,
        *,
        model: Any | None = None,
    ) -> None:
        self._config = config
        if model is None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "Ultralytics is unavailable; install requirements-detector.txt"
                ) from exc
            model = YOLO(model_path)
        self._model = model

    def detect(self, bgr_image: np.ndarray) -> list[Detection2D]:
        if not isinstance(bgr_image, np.ndarray) or bgr_image.ndim != 3:
            raise ValueError("detector input must be an HxWxC NumPy image")
        results = self._model.predict(
            source=bgr_image,
            conf=self._config.confidence_threshold,
            iou=self._config.nms_iou_threshold,
            max_det=self._config.max_detections,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]
        if result.boxes is None:
            return []

        xyxy = _as_numpy(result.boxes.xyxy)
        confidences = _as_numpy(result.boxes.conf).reshape(-1)
        class_ids = _as_numpy(result.boxes.cls).reshape(-1).astype(int)
        ignored = {name.strip().lower() for name in self._config.ignored_classes}
        candidates: list[tuple[str, int, float, np.ndarray]] = []
        for bounds, confidence, class_id in zip(xyxy, confidences, class_ids):
            class_name = str(result.names[int(class_id)]).strip().lower()
            score = float(confidence)
            if class_name in ignored or score < self._config.confidence_threshold:
                continue
            candidates.append((class_name, int(class_id), score, bounds))

        candidates.sort(key=lambda item: item[2], reverse=True)
        detections: list[Detection2D] = []
        for detection_id, (class_name, class_id, score, bounds) in enumerate(
            candidates[: self._config.max_detections]
        ):
            detections.append(
                Detection2D(
                    detection_id=detection_id,
                    detector_class=class_name,
                    class_id=class_id,
                    confidence=score,
                    x_min=float(bounds[0]),
                    y_min=float(bounds[1]),
                    x_max=float(bounds[2]),
                    y_max=float(bounds[3]),
                )
            )
        return detections


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)
