from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
INTERFACE_ROOT = WORKSPACE_ROOT / "src" / "environment_memory_interfaces"


def test_phase5_detector_and_geometry_defaults_match_plan():
    config = (PACKAGE_ROOT / "config" / "yolov8n_geometry.yaml").read_text(
        encoding="utf-8"
    )

    assert "detector_confidence_threshold: 0.35" in config
    assert "detector_nms_iou_threshold: 0.50" in config
    assert "detector_max_detections: 8" in config
    assert 'detector_ignored_classes: ["person"]' in config
    assert "depth_central_fraction: 0.60" in config
    assert "depth_minimum_m: 0.20" in config
    assert "depth_maximum_m: 10.0" in config
    assert "depth_minimum_valid_samples: 30" in config
    assert "depth_minimum_valid_ratio: 0.30" in config


def test_geometric_interface_keeps_phase5_separate_from_semantics_and_storage():
    interface = (
        INTERFACE_ROOT / "msg" / "GeometricObjectObservation.msg"
    ).read_text(encoding="utf-8")

    assert "robot_interfaces/ObjectDetection2D detection" in interface
    assert "geometry_msgs/PointStamped camera_position" in interface
    assert "geometry_msgs/PointStamped map_position" in interface
    assert "geometry_msgs/PoseStamped robot_pose" in interface
    assert "localization_quality" in interface
    assert "SemanticObject" not in interface
    assert "description" not in interface
    assert "embedding" not in interface


def test_observation_manager_requires_exact_rgb_timestamped_transforms():
    source = (
        PACKAGE_ROOT / "environment_memory" / "observation_manager.py"
    ).read_text(encoding="utf-8")

    assert "_require_exact_transform" in source
    assert "validate_transform_contract" in source
    assert "lookup_transform" in source
    assert "Time()" not in source
    assert "RGB-D timestamps must be nonzero for exact TF lookup" in source
    assert "geometric_confidence = min(" in source


def test_detector_config_is_applied_to_observation_manager():
    launch = (
        PACKAGE_ROOT / "launch" / "exploration_observation.launch.py"
    ).read_text(encoding="utf-8")
    observation_node = launch.index('executable="observation_manager"')
    detector_parameters = launch.index(
        'LaunchConfiguration("detector_config")', observation_node
    )

    assert detector_parameters > observation_node


def test_model_version_and_weight_are_pinned():
    requirements = (WORKSPACE_ROOT / "requirements-detector.txt").read_text(
        encoding="utf-8"
    )
    fetcher = (WORKSPACE_ROOT / "tools" / "fetch_yolov8n.py").read_text(
        encoding="utf-8"
    )

    assert "numpy==1.26.4" in requirements
    assert "ultralytics==8.3.0" in requirements
    assert "ultralytics-thop==2.0.0" in requirements
    assert "releases/download/v8.3.0/yolov8n.pt" in fetcher
    assert "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36" in fetcher
