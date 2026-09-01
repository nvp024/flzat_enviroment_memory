from types import SimpleNamespace

import numpy as np

from environment_memory.perception.detector import DetectorConfig, UltralyticsYoloDetector


class FakeModel:
    def __init__(self):
        self.arguments = None

    def predict(self, **kwargs):
        self.arguments = kwargs
        boxes = SimpleNamespace(
            xyxy=np.array(
                [[1.0, 2.0, 20.0, 30.0], [4.0, 5.0, 25.0, 35.0]],
                dtype=np.float32,
            ),
            conf=np.array([0.70, 0.95], dtype=np.float32),
            cls=np.array([39.0, 0.0], dtype=np.float32),
        )
        return [SimpleNamespace(boxes=boxes, names={0: "person", 39: "bottle"})]


def test_yolo_adapter_forwards_thresholds_and_filters_people():
    model = FakeModel()
    config = DetectorConfig()
    detector = UltralyticsYoloDetector("unused.pt", config, model=model)
    image = np.zeros((40, 50, 3), dtype=np.uint8)

    detections = detector.detect(image)

    assert model.arguments["conf"] == 0.35
    assert model.arguments["iou"] == 0.50
    assert model.arguments["max_det"] == 8
    assert model.arguments["verbose"] is False
    assert len(detections) == 1
    assert detections[0].detection_id == 0
    assert detections[0].detector_class == "bottle"
    assert detections[0].class_id == 39
    assert detections[0].confidence == np.float32(0.70)


def test_detector_config_rejects_invalid_thresholds():
    for kwargs in (
        {"confidence_threshold": -0.1},
        {"nms_iou_threshold": 1.1},
        {"max_detections": 0},
    ):
        try:
            DetectorConfig(**kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid config was accepted: {kwargs}")
