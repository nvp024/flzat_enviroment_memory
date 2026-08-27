from __future__ import annotations

import copy
import json
import math
import threading
import uuid
from typing import Optional

import cv2
import message_filters
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from environment_memory.depth_localization import (
    DepthLocalizationConfig,
    DepthLocalizationResult,
    LocalizationError,
    intrinsics_from_camera_matrix,
    localize_detection,
)
from environment_memory.detector import (
    Detection2D,
    DetectorConfig,
    UltralyticsYoloDetector,
)
from environment_memory.model_asset import resolve_verified_model
from environment_memory.observation_bundle import ObservationBundle
from environment_memory.observation_queue import LatestObservationQueue
from environment_memory.scene_change import histogram_distance, hsv_histogram
from environment_memory.trigger_policy import (
    ObservationTriggerPolicy,
    Pose2D,
    TriggerConfig,
)
from environment_memory.transform_geometry import (
    RigidTransform,
    transform_point,
    validate_transform_contract,
)
from environment_memory_interfaces.msg import (
    ExplorationStatus,
    GeometricObjectObservation,
    VlmObservation,
)
from robot_interfaces.msg import ObjectDetection2D


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(rotation) -> float:
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObservationManager(Node):
    """Capture triggered RGB-D bundles and localize YOLO detections in map."""

    def __init__(self) -> None:
        super().__init__("observation_manager")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._camera_info: Optional[CameraInfo] = None
        self._scan: Optional[LaserScan] = None
        self._odometry: Optional[Odometry] = None
        self._nav_goal_states: dict[bytes, int] = {}
        self._queue = LatestObservationQueue[ObservationBundle]()
        self._accepted = 0
        self._rejected = 0
        self._replaced = 0
        self._detections = 0
        self._localized = 0
        self._localization_rejected = 0
        self._last_observation_id = ""
        self._last_reason = "waiting for synchronized RGB-D"
        self._exploring = False
        self._map_frame = self._string_parameter("map_frame")
        self._base_frame = self._string_parameter("base_frame")
        self._camera_frame = self._string_parameter("camera_frame")
        self._max_sync_delta_ns = int(
            self._float_parameter("sync_slop_s") * 1_000_000_000
        )
        self._camera_info_max_age_ns = int(
            self._float_parameter("camera_info_max_age_s") * 1_000_000_000
        )
        self._sensor_max_age_ns = int(
            self._float_parameter("sensor_max_age_s") * 1_000_000_000
        )
        self._tf_timeout_s = self._float_parameter("tf_timeout_s")
        self._jpeg_quality = self._integer_parameter("debug_jpeg_quality")
        detector_config = DetectorConfig(
            confidence_threshold=self._float_parameter(
                "detector_confidence_threshold"
            ),
            nms_iou_threshold=self._float_parameter("detector_nms_iou_threshold"),
            max_detections=self._integer_parameter("detector_max_detections"),
            ignored_classes=tuple(
                str(value).strip().lower()
                for value in self.get_parameter("detector_ignored_classes").value
            ),
        )
        verified_model = resolve_verified_model(
            self._string_parameter("detector_model_path"),
            self._string_parameter("detector_model_sha256"),
        )
        self._detector = UltralyticsYoloDetector(
            str(verified_model), detector_config
        )
        self._depth_config = DepthLocalizationConfig(
            central_fraction=self._float_parameter("depth_central_fraction"),
            minimum_depth_m=self._float_parameter("depth_minimum_m"),
            maximum_depth_m=self._float_parameter("depth_maximum_m"),
            minimum_valid_samples=self._integer_parameter(
                "depth_minimum_valid_samples"
            ),
            minimum_valid_ratio=self._float_parameter(
                "depth_minimum_valid_ratio"
            ),
            mad_scale=self._float_parameter("depth_mad_scale"),
            minimum_outlier_band_m=self._float_parameter(
                "depth_minimum_outlier_band_m"
            ),
            maximum_normalized_dispersion=self._float_parameter(
                "depth_maximum_normalized_dispersion"
            ),
        )
        self._policy = ObservationTriggerPolicy(
            TriggerConfig(
                translation_m=self._float_parameter("translation_trigger_m"),
                rotation_rad=math.radians(
                    self._float_parameter("rotation_trigger_deg")
                ),
                scene_distance=self._float_parameter("scene_change_threshold"),
                max_interval_s=self._float_parameter("max_interval_s"),
                min_interval_s=self._float_parameter("min_interval_s"),
                waypoint_settle_s=self._float_parameter("waypoint_settle_s"),
                preferred_linear_speed_mps=self._float_parameter(
                    "preferred_linear_speed_mps"
                ),
                preferred_angular_speed_rps=self._float_parameter(
                    "preferred_angular_speed_rps"
                ),
            )
        )

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        exploration_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/environment_memory/observation_status", status_qos
        )
        self._debug_pub = self.create_publisher(
            CompressedImage,
            "/environment_memory/debug_image",
            qos_profile_sensor_data,
        )
        self._geometry_pub = self.create_publisher(
            GeometricObjectObservation,
            "/environment_memory/geometric_observations",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._vlm_observation_pub = self.create_publisher(
            VlmObservation,
            "/environment_memory/vlm_observations",
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            CameraInfo,
            self._string_parameter("camera_info_topic"),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self._string_parameter("odom_topic"),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_goal_status,
            10,
        )
        self.create_subscription(
            ExplorationStatus,
            "/exploration/status",
            self._on_exploration_status,
            exploration_qos,
        )
        self._rgb_sub = message_filters.Subscriber(
            self,
            Image,
            self._string_parameter("rgb_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        self._depth_sub = message_filters.Subscriber(
            self,
            Image,
            self._string_parameter("depth_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=self._integer_parameter("sync_queue_size"),
            slop=self._float_parameter("sync_slop_s"),
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._on_rgb_depth)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.1, self._process_next)
        self.create_timer(1.0, self._publish_status)

    def _declare_parameters(self) -> None:
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_s", 0.08)
        self.declare_parameter("camera_info_max_age_s", 1.0)
        self.declare_parameter("sensor_max_age_s", 0.5)
        self.declare_parameter("tf_timeout_s", 0.5)
        self.declare_parameter("translation_trigger_m", 1.0)
        self.declare_parameter("rotation_trigger_deg", 45.0)
        self.declare_parameter("scene_change_threshold", 0.35)
        self.declare_parameter("max_interval_s", 20.0)
        self.declare_parameter("min_interval_s", 8.0)
        self.declare_parameter("waypoint_settle_s", 0.75)
        self.declare_parameter("preferred_linear_speed_mps", 0.10)
        self.declare_parameter("preferred_angular_speed_rps", 0.15)
        self.declare_parameter("debug_jpeg_quality", 85)
        self.declare_parameter(
            "detector_model_path",
            "~/.local/share/flzat/environment_memory/models/yolov8n-v8.3.0.pt",
        )
        self.declare_parameter(
            "detector_model_sha256",
            "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36",
        )
        self.declare_parameter("detector_confidence_threshold", 0.35)
        self.declare_parameter("detector_nms_iou_threshold", 0.50)
        self.declare_parameter("detector_max_detections", 8)
        self.declare_parameter("detector_ignored_classes", ["person"])
        self.declare_parameter("depth_central_fraction", 0.60)
        self.declare_parameter("depth_minimum_m", 0.20)
        self.declare_parameter("depth_maximum_m", 10.0)
        self.declare_parameter("depth_minimum_valid_samples", 30)
        self.declare_parameter("depth_minimum_valid_ratio", 0.30)
        self.declare_parameter("depth_mad_scale", 3.0)
        self.declare_parameter("depth_minimum_outlier_band_m", 0.02)
        self.declare_parameter("depth_maximum_normalized_dispersion", 0.10)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = message

    def _on_scan(self, message: LaserScan) -> None:
        with self._lock:
            self._scan = message

    def _on_odometry(self, message: Odometry) -> None:
        with self._lock:
            self._odometry = message

    def _on_goal_status(self, message: GoalStatusArray) -> None:
        now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
        with self._lock:
            for entry in message.status_list:
                goal_id = bytes(entry.goal_info.goal_id.uuid)
                previous = self._nav_goal_states.get(goal_id)
                self._nav_goal_states[goal_id] = entry.status
                if (
                    entry.status == GoalStatus.STATUS_SUCCEEDED
                    and previous != GoalStatus.STATUS_SUCCEEDED
                ):
                    self._policy.mark_waypoint_completed(now_s)

    def _on_exploration_status(self, message: ExplorationStatus) -> None:
        with self._lock:
            self._exploring = message.state == ExplorationStatus.EXPLORING
            if not self._exploring:
                self._last_reason = f"exploration state {message.state}; capture paused"

    def _on_rgb_depth(self, rgb: Image, depth: Image) -> None:
        with self._lock:
            if not self._exploring:
                return
        rgb_stamp_ns = _stamp_ns(rgb.header.stamp)
        depth_stamp_ns = _stamp_ns(depth.header.stamp)
        sync_delta_ns = abs(rgb_stamp_ns - depth_stamp_ns)
        if sync_delta_ns > self._max_sync_delta_ns:
            self._reject("RGB-D timestamp delta exceeds configured slop")
            return
        with self._lock:
            camera_info = self._camera_info
            scan = self._scan
            odometry = self._odometry
        invalid = self._validate_sensor_bundle(
            rgb,
            depth,
            camera_info,
            scan,
            rgb_stamp_ns,
            self.get_clock().now().nanoseconds,
        )
        if invalid:
            self._reject(invalid)
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
            histogram = hsv_histogram(bgr)
        except Exception as exc:
            self._reject(f"RGB conversion failed: {exc}")
            return

        rgb_time = Time.from_msg(rgb.header.stamp)
        camera_source = rgb.header.frame_id or self._camera_frame
        try:
            camera_transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                camera_source,
                rgb_time,
                timeout=Duration(seconds=self._tf_timeout_s),
            )
            robot_transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rgb_time,
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        except TransformException as exc:
            self._reject(f"timestamped TF unavailable: {exc}")
            return

        pose = Pose2D(
            robot_transform.transform.translation.x,
            robot_transform.transform.translation.y,
            _yaw_from_quaternion(robot_transform.transform.rotation),
        )
        scene_distance = None
        with self._lock:
            if self._policy.last_histogram is not None:
                scene_distance = histogram_distance(
                    self._policy.last_histogram, histogram
                )
            linear_speed, angular_speed = self._speeds(odometry, rgb_stamp_ns)
            decision = self._policy.evaluate(
                rgb_stamp_ns / 1_000_000_000.0,
                pose,
                scene_distance,
                linear_speed,
                angular_speed,
            )
            if not decision.eligible or decision.reason is None:
                self._last_reason = decision.detail or "no trigger"
                return

            observation_id = str(uuid.uuid4())
            bundle = ObservationBundle(
                observation_id=observation_id,
                rgb=copy.deepcopy(rgb),
                depth=copy.deepcopy(depth),
                camera_info=copy.deepcopy(camera_info),
                camera_transform=copy.deepcopy(camera_transform),
                robot_transform=copy.deepcopy(robot_transform),
                scan_stamp_ns=_stamp_ns(scan.header.stamp),
                trigger_reason=decision.reason.value,
                sync_delta_ns=sync_delta_ns,
            )
            queued = self._queue.submit(bundle, decision.priority)
            if not queued.accepted:
                self._rejected += 1
                self._last_reason = "higher-priority observation already pending"
                return
            if queued.replaced is not None:
                self._replaced += 1
            self._policy.accept(
                rgb_stamp_ns / 1_000_000_000.0, pose, histogram
            )
            self._accepted += 1
            self._last_observation_id = observation_id
            self._last_reason = decision.reason.value

    def _validate_sensor_bundle(
        self,
        rgb: Image,
        depth: Image,
        camera_info: Optional[CameraInfo],
        scan: Optional[LaserScan],
        rgb_stamp_ns: int,
        now_ns: int,
    ) -> str:
        if rgb_stamp_ns <= 0 or _stamp_ns(depth.header.stamp) <= 0:
            return "RGB-D timestamps must be nonzero for exact TF lookup"
        if camera_info is None:
            return "CameraInfo has not arrived"
        if scan is None:
            return "LiDAR scan has not arrived"
        if depth.encoding != "32FC1":
            return f"depth encoding must be 32FC1, got {depth.encoding}"
        if rgb.width != depth.width or rgb.height != depth.height:
            return "RGB and depth resolutions differ"
        if camera_info.width != rgb.width or camera_info.height != rgb.height:
            return "CameraInfo resolution does not match RGB"
        if not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id:
            return "RGB/depth frame IDs are empty or different"
        if camera_info.header.frame_id != rgb.header.frame_id:
            return "CameraInfo frame ID does not match RGB"
        if (
            abs(rgb_stamp_ns - _stamp_ns(camera_info.header.stamp))
            > self._camera_info_max_age_ns
        ):
            return "CameraInfo is stale"
        if abs(rgb_stamp_ns - _stamp_ns(scan.header.stamp)) > self._sensor_max_age_ns:
            return "LiDAR health is stale"
        if now_ns > 0 and abs(now_ns - rgb_stamp_ns) > self._sensor_max_age_ns:
            return "RGB-D bundle is stale"
        if (
            now_ns > 0
            and abs(now_ns - _stamp_ns(scan.header.stamp)) > self._sensor_max_age_ns
        ):
            return "LiDAR scan is stale relative to current time"
        return ""

    def _speeds(
        self, odometry: Optional[Odometry], observation_stamp_ns: int
    ) -> tuple[float, float]:
        if odometry is None:
            return math.inf, math.inf
        if (
            abs(observation_stamp_ns - _stamp_ns(odometry.header.stamp))
            > self._sensor_max_age_ns
        ):
            return math.inf, math.inf
        return odometry.twist.twist.linear.x, odometry.twist.twist.angular.z

    def _process_next(self) -> None:
        with self._lock:
            bundle = self._queue.begin_next()
        if bundle is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(bundle.rgb, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(
                bundle.depth, desired_encoding="32FC1"
            )
            self._validate_transform_bundle(bundle)
            intrinsics = intrinsics_from_camera_matrix(
                bundle.camera_info.width,
                bundle.camera_info.height,
                list(bundle.camera_info.k),
            )
            detections = list(self._detector.detect(bgr))
            vlm_bgr = bgr.copy()
            geometric_messages = []
            with self._lock:
                self._detections += len(detections)
            for detection in detections:
                try:
                    localization = localize_detection(
                        depth,
                        (
                            detection.x_min,
                            detection.y_min,
                            detection.x_max,
                            detection.y_max,
                        ),
                        intrinsics,
                        self._depth_config,
                    )
                    message = self._geometric_observation(
                        bundle, detection, localization
                    )
                    geometric_messages.append(message)
                    with self._lock:
                        self._localized += 1
                    annotation = (
                        f"{detection.detection_id}:{detection.detector_class} "
                        f"{detection.confidence:.2f} map="
                        f"({message.map_position.point.x:.2f},"
                        f"{message.map_position.point.y:.2f},"
                        f"{message.map_position.point.z:.2f})"
                    )
                    self._draw_detection(
                        bgr, localization.clamped_bbox, annotation, (0, 200, 0)
                    )
                    vlm_annotation = (
                        f"{detection.detection_id}:{detection.detector_class} "
                        f"{detection.confidence:.2f}"
                    )
                    self._draw_detection(
                        vlm_bgr,
                        localization.clamped_bbox,
                        vlm_annotation,
                        (0, 200, 0),
                    )
                except LocalizationError as exc:
                    with self._lock:
                        self._localization_rejected += 1
                    bounds = self._display_bbox(detection, bgr.shape[1], bgr.shape[0])
                    annotation = (
                        f"{detection.detection_id}:{detection.detector_class} "
                        f"rejected: {exc}"
                    )
                    self._draw_detection(bgr, bounds, annotation, (0, 0, 255))
            self._publish_debug(bundle, bgr, len(detections))
            for message in geometric_messages:
                self._geometry_pub.publish(message)
            if geometric_messages:
                vlm_observation = VlmObservation()
                vlm_observation.observation_id = bundle.observation_id
                vlm_observation.observation_stamp = bundle.rgb.header.stamp
                vlm_observation.image = self._compressed_image(bundle, vlm_bgr)
                vlm_observation.detections = [
                    copy.deepcopy(message.detection)
                    for message in geometric_messages
                ]
                self._vlm_observation_pub.publish(vlm_observation)
        except Exception as exc:
            with self._lock:
                self._localization_rejected += 1
                self._last_reason = f"Phase 5 processing failed: {exc}"
            self.get_logger().error(
                f"Observation {bundle.observation_id} processing failed: {exc}"
            )
        finally:
            with self._lock:
                self._queue.complete()
            self._publish_status()

    def _reject(self, reason: str) -> None:
        with self._lock:
            self._rejected += 1
            self._last_reason = reason

    def _publish_status(self) -> None:
        with self._lock:
            payload = {
                "accepted": self._accepted,
                "rejected": self._rejected,
                "replaced": self._replaced,
                "detections": self._detections,
                "localized": self._localized,
                "localization_rejected": self._localization_rejected,
                "last_observation_id": self._last_observation_id,
                "last_reason": self._last_reason,
                "active": self._queue.active,
                "pending": self._queue.has_pending,
            }
        self._status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )

    def _validate_transform_bundle(self, bundle: ObservationBundle) -> None:
        stamp_ns = _stamp_ns(bundle.rgb.header.stamp)
        camera_source = bundle.rgb.header.frame_id
        self._require_exact_transform(
            bundle.camera_transform, self._map_frame, camera_source, stamp_ns
        )
        self._require_exact_transform(
            bundle.robot_transform, self._map_frame, self._base_frame, stamp_ns
        )

    @staticmethod
    def _require_exact_transform(transform, target: str, source: str, stamp_ns: int) -> None:
        validate_transform_contract(
            transform.header.frame_id,
            transform.child_frame_id,
            _stamp_ns(transform.header.stamp),
            expected_target_frame=target,
            expected_source_frame=source,
            expected_stamp_ns=stamp_ns,
        )

    def _geometric_observation(
        self,
        bundle: ObservationBundle,
        detection: Detection2D,
        localization: DepthLocalizationResult,
    ) -> GeometricObjectObservation:
        camera_transform = bundle.camera_transform.transform
        map_xyz = transform_point(
            (localization.x, localization.y, localization.z),
            RigidTransform(
                translation=(
                    camera_transform.translation.x,
                    camera_transform.translation.y,
                    camera_transform.translation.z,
                ),
                quaternion_xyzw=(
                    camera_transform.rotation.x,
                    camera_transform.rotation.y,
                    camera_transform.rotation.z,
                    camera_transform.rotation.w,
                ),
            ),
        )
        detection_message = ObjectDetection2D()
        detection_message.observation_id = bundle.observation_id
        detection_message.detection_id = detection.detection_id
        detection_message.detector_class = detection.detector_class
        detection_message.confidence = detection.confidence
        (
            detection_message.x_min,
            detection_message.y_min,
            detection_message.x_max,
            detection_message.y_max,
        ) = localization.clamped_bbox

        camera_position = PointStamped()
        camera_position.header = copy.deepcopy(bundle.rgb.header)
        camera_position.point.x = localization.x
        camera_position.point.y = localization.y
        camera_position.point.z = localization.z
        map_position = PointStamped()
        map_position.header.stamp = bundle.rgb.header.stamp
        map_position.header.frame_id = self._map_frame
        map_position.point.x, map_position.point.y, map_position.point.z = map_xyz

        robot_pose = PoseStamped()
        robot_pose.header.stamp = bundle.rgb.header.stamp
        robot_pose.header.frame_id = self._map_frame
        robot_pose.pose.position.x = bundle.robot_transform.transform.translation.x
        robot_pose.pose.position.y = bundle.robot_transform.transform.translation.y
        robot_pose.pose.position.z = bundle.robot_transform.transform.translation.z
        robot_pose.pose.orientation = copy.deepcopy(
            bundle.robot_transform.transform.rotation
        )

        message = GeometricObjectObservation()
        message.observation_id = bundle.observation_id
        message.observation_stamp = bundle.rgb.header.stamp
        message.depth_stamp = bundle.depth.header.stamp
        message.detection = detection_message
        message.camera_position = camera_position
        message.map_position = map_position
        message.robot_pose = robot_pose
        message.valid_depth_ratio = localization.valid_depth_ratio
        message.depth_mad_m = localization.depth_mad_m
        message.localization_quality = localization.localization_quality
        message.geometric_confidence = min(
            detection.confidence, localization.localization_quality
        )
        return message

    def _publish_debug(
        self, bundle: ObservationBundle, bgr, detection_count: int
    ) -> CompressedImage:
        label = (
            f"{bundle.trigger_reason} id={bundle.observation_id[:8]} "
            f"dt={bundle.sync_delta_ns / 1_000_000.0:.1f}ms "
            f"detections={detection_count}"
        )
        cv2.putText(
            bgr,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        message = self._compressed_image(bundle, bgr)
        self._debug_pub.publish(message)
        return message

    def _compressed_image(
        self, bundle: ObservationBundle, bgr
    ) -> CompressedImage:
        success, encoded = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        )
        if not success:
            raise RuntimeError("OpenCV could not encode debug JPEG")
        message = CompressedImage()
        message.header = bundle.rgb.header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    @staticmethod
    def _display_bbox(
        detection: Detection2D, width: int, height: int
    ) -> tuple[int, int, int, int]:
        x_min = max(0, min(width - 1, int(detection.x_min)))
        y_min = max(0, min(height - 1, int(detection.y_min)))
        x_max = max(0, min(width - 1, int(detection.x_max)))
        y_max = max(0, min(height - 1, int(detection.y_max)))
        return x_min, y_min, x_max, y_max

    @staticmethod
    def _draw_detection(bgr, bounds, label: str, color) -> None:
        x_min, y_min, x_max, y_max = bounds
        cv2.rectangle(bgr, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.putText(
            bgr,
            label[:100],
            (x_min, max(16, y_min - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[ObservationManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = ObservationManager()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("ObservationManager shutting down …")
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
