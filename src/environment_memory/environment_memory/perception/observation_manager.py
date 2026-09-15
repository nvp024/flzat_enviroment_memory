"""Ground objects in frozen RGB-D observations and persist map geometry."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import threading
import time
import uuid
from typing import Optional

import cv2
import message_filters
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
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

from environment_memory.perception.depth_localization import (
    DepthLocalizationConfig,
    DepthLocalizationResult,
    LocalizationError,
    intrinsics_from_camera_matrix,
    localize_detection,
)
from environment_memory.perception.grounding_contract import (
    GroundedDetection,
    validate_grounding_detections,
)
from environment_memory.perception.observation_bundle import ObservationBundle
from environment_memory.perception.observation_log import ObservationLifecycleLog
from environment_memory.perception.observation_queue import LatestObservationQueue
from environment_memory.perception.ros_image import image_to_bgr, image_to_depth_32fc1
from environment_memory.perception.scene_change import histogram_distance, hsv_histogram
from environment_memory.perception.trigger_policy import (
    ObservationTriggerPolicy,
    Pose2D,
    TriggerConfig,
)
from environment_memory.perception.transform_geometry import (
    RigidTransform,
    transform_point,
    validate_transform_contract,
)
from environment_memory_interfaces.msg import (
    ExplorationStatus,
    LocalizedObjectObservation,
)
from robot_interfaces.action import GroundObjects
from robot_interfaces.msg import ObjectDetection2D, SemanticObject


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(rotation) -> float:
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class LocalizedGeometry:
    """Internal geometry produced from one grounded detection and frozen TF."""

    observation_stamp: TimeMessage
    depth_stamp: TimeMessage
    detection: ObjectDetection2D
    map_position: PointStamped
    robot_pose: PoseStamped
    localization_quality: float


class ObservationManager(Node):
    """Coordinate frozen RGB-D observations through VLM grounding and memory."""

    def __init__(self) -> None:
        super().__init__("observation_manager")
        self._declare_parameters()
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
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
        self._finalizing = False
        self._accepting = True
        self._exploration_started_ns = 0
        self._active_bundle: ObservationBundle | None = None
        self._active_goal = None
        self._active_started = 0.0
        self._persistence_bundles: dict[str, ObservationBundle] = {}
        self._pending_persistence: set[tuple[str, int]] = set()
        self._environment_id = self._string_parameter("environment_id")
        self._map_id = self._string_parameter("map_id")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self._environment_id):
            raise ValueError("environment_id may contain letters, numbers, _ and -")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self._map_id):
            raise ValueError("map_id may contain letters, numbers, _ and -")
        storage_value = self._string_parameter("storage_root").strip()
        storage_root = (
            Path(storage_value).expanduser()
            if storage_value
            else Path.home() / ".local" / "share" / "flzat" / "environment_memory"
        )
        self._lifecycle_log = ObservationLifecycleLog(
            storage_root / self._environment_id / "observations.jsonl",
            self._environment_id,
            self._map_id,
        )
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
        self._initial_settle_ns = int(
            self._float_parameter("initial_settle_s") * 1_000_000_000
        )
        self._grounding_timeout_s = self._float_parameter(
            "grounding_action_timeout_s"
        )
        self._grounding_max_detections = self._integer_parameter(
            "grounding_max_detections"
        )
        if self._grounding_timeout_s <= 0.0:
            raise ValueError("grounding_action_timeout_s must be positive")
        if self._initial_settle_ns < 0:
            raise ValueError("initial_settle_s cannot be negative")
        if self._grounding_max_detections < 1:
            raise ValueError("grounding_max_detections must be positive")
        self._grounding_client = ActionClient(
            self,
            GroundObjects,
            "/vlm/ground_objects",
            callback_group=self._callback_group,
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
        self._localized_pub = self.create_publisher(
            LocalizedObjectObservation,
            "/environment_memory/localized_observations",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._semantic_status_pub = self.create_publisher(
            String,
            "/environment_memory/semantic_status",
            exploration_qos,
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
        self.create_subscription(
            String,
            "/environment_memory/persistence_events",
            self._on_persistence_event,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._callback_group,
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
        self.create_timer(
            0.1, self._process_next, callback_group=self._callback_group
        )
        self.create_timer(
            1.0, self._publish_status, callback_group=self._callback_group
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "mapping-session")
        self.declare_parameter("storage_root", "")
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
        self.declare_parameter("initial_settle_s", 1.0)
        self.declare_parameter("preferred_linear_speed_mps", 0.10)
        self.declare_parameter("preferred_angular_speed_rps", 0.15)
        self.declare_parameter("debug_jpeg_quality", 85)
        self.declare_parameter("grounding_action_timeout_s", 60.0)
        self.declare_parameter("grounding_max_detections", 20)
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
        goal_to_cancel = None
        with self._lock:
            was_exploring = self._exploring
            self._exploring = message.state == ExplorationStatus.EXPLORING
            if self._exploring and not was_exploring:
                stamp_ns = _stamp_ns(message.stamp)
                self._exploration_started_ns = (
                    stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds
                )
                self._last_reason = "exploration started; waiting for stable capture"
            elif message.state == ExplorationStatus.FINALIZING:
                self._finalizing = True
                self._last_reason = "exploration finalizing; draining observations"
            elif message.state in {
                ExplorationStatus.COMPLETED,
                ExplorationStatus.FAILED,
            }:
                self._accepting = False
                goal_to_cancel = self._active_goal
                self._last_reason = f"exploration state {message.state}; stopped"
            elif not self._exploring:
                self._last_reason = f"exploration state {message.state}; capture paused"
        if goal_to_cancel is not None:
            goal_to_cancel.cancel_goal_async()

    def _on_rgb_depth(self, rgb: Image, depth: Image) -> None:
        replaced_bundle = None
        with self._lock:
            if not self._exploring or not self._accepting:
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
            bgr = image_to_bgr(rgb)
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
            if (
                self._policy.last_histogram is None
                and rgb_stamp_ns - self._exploration_started_ns
                < self._initial_settle_ns
            ):
                self._last_reason = "waiting for initial settle interval"
                return
            if (
                self._policy.last_histogram is None
                and (
                    abs(linear_speed)
                    > self._policy.config.preferred_linear_speed_mps
                    or abs(angular_speed)
                    > self._policy.config.preferred_angular_speed_rps
                )
            ):
                self._last_reason = "waiting for stable first frame"
                return
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
                self._log_bundle(bundle, "CAPTURED")
                self._lifecycle_log.append(
                    bundle.observation_id,
                    "REJECTED",
                    reason="higher-priority observation already pending",
                )
                return
            if queued.replaced is not None:
                self._replaced += 1
                replaced_bundle = queued.replaced
            self._policy.accept(
                rgb_stamp_ns / 1_000_000_000.0, pose, histogram
            )
            self._accepted += 1
            self._last_observation_id = observation_id
            self._last_reason = decision.reason.value
        if replaced_bundle is not None:
            self._lifecycle_log.append(
                replaced_bundle.observation_id,
                "REJECTED",
                reason="replaced by newer pending observation",
            )
        self._log_bundle(bundle, "CAPTURED")
        self._lifecycle_log.append(
            observation_id,
            "VLM_QUEUED",
            trigger_reason=bundle.trigger_reason,
        )

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
        self._process_grounding_tick()

    def _process_grounding_tick(self) -> None:
        timed_out_bundle = None
        goal_to_cancel = None
        bundle_to_start = None
        with self._lock:
            if (
                self._active_bundle is not None
                and self._active_started > 0.0
                and time.monotonic() - self._active_started
                > self._grounding_timeout_s
            ):
                timed_out_bundle = self._active_bundle
                goal_to_cancel = self._active_goal
            elif self._active_bundle is None:
                if not self._grounding_client.server_is_ready():
                    self._last_reason = "waiting for GroundObjects action server"
                    return
                bundle_to_start = self._queue.begin_next()
                if bundle_to_start is not None:
                    self._active_bundle = bundle_to_start
                    self._active_started = time.monotonic()
                    self._last_reason = "grounding request active"
        if timed_out_bundle is not None:
            if goal_to_cancel is not None:
                goal_to_cancel.cancel_goal_async()
            self._finish_active_grounding(
                timed_out_bundle,
                "TIMED_OUT",
                "GroundObjects action timeout",
                rejected=True,
            )
        elif bundle_to_start is not None:
            self._send_grounding(bundle_to_start)

    def _send_grounding(self, bundle: ObservationBundle) -> None:
        try:
            bgr = image_to_bgr(bundle.rgb)
            goal = GroundObjects.Goal()
            goal.observation_id = bundle.observation_id
            goal.stamp = bundle.rgb.header.stamp
            goal.image = self._compressed_image(bundle, bgr)
            self._lifecycle_log.append(
                bundle.observation_id,
                "VLM_ACTIVE",
            )
            future = self._grounding_client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, expected=bundle: self._on_grounding_goal(
                    expected, completed
                )
            )
        except Exception as exc:
            self._finish_active_grounding(
                bundle,
                "REJECTED",
                f"GroundObjects goal send failed: {exc}",
                rejected=True,
            )

    def _on_grounding_goal(self, bundle: ObservationBundle, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("GroundObjects goal was rejected")
            with self._lock:
                if self._active_bundle is not bundle:
                    goal_handle.cancel_goal_async()
                    return
                self._active_goal = goal_handle
            goal_handle.get_result_async().add_done_callback(
                lambda completed, expected=bundle: self._on_grounding_result(
                    expected, completed
                )
            )
        except Exception as exc:
            self._finish_active_grounding(
                bundle,
                "REJECTED",
                f"GroundObjects goal failed: {exc}",
                rejected=True,
            )

    def _on_grounding_result(self, bundle: ObservationBundle, future) -> None:
        with self._lock:
            if self._active_bundle is not bundle:
                late_result = True
            else:
                late_result = False
                # The action completed within its deadline. Disable the action
                # timeout while deterministic local post-processing runs.
                self._active_started = 0.0
        if late_result:
            self._lifecycle_log.append(
                bundle.observation_id,
                "REJECTED",
                reason="late GroundObjects result arrived after bundle release",
            )
            return
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"GroundObjects action status {response.status}")
            result = response.result
            if not result.success:
                raise RuntimeError(result.error_message or "grounding failed")
            if result.observation_id != bundle.observation_id:
                raise RuntimeError("GroundObjects result observation_id mismatch")
            detections = self._validated_grounding_detections(
                bundle, result.detections
            )
            self._lifecycle_log.append(
                bundle.observation_id,
                "BBOX_VALIDATED",
                model_id=result.model_id,
                model_revision=result.model_revision,
                prompt_version=result.prompt_version,
                raw_response=result.raw_response,
                detections=[self._detection_payload(item) for item in detections],
            )
            published = self._localize_and_publish(
                bundle,
                detections,
            )
            if not detections:
                self._lifecycle_log.append(
                    bundle.observation_id,
                    "STORED",
                    empty_observation=True,
                )
            elif published == 0:
                raise RuntimeError("all grounded boxes failed depth localization")
            self._finish_active_grounding(
                bundle,
                "",
                f"grounded {len(detections)} objects; published {published}",
                rejected=False,
            )
        except Exception as exc:
            self._finish_active_grounding(
                bundle,
                "REJECTED",
                f"grounding result rejected: {exc}",
                rejected=True,
            )

    def _validated_grounding_detections(
        self,
        bundle: ObservationBundle,
        messages,
    ) -> list[GroundedDetection]:
        return validate_grounding_detections(
            bundle.observation_id,
            int(bundle.rgb.width),
            int(bundle.rgb.height),
            messages,
            self._grounding_max_detections,
        )

    def _localize_and_publish(
        self,
        bundle: ObservationBundle,
        detections: list[GroundedDetection],
    ) -> int:
        bgr = image_to_bgr(bundle.rgb)
        depth = image_to_depth_32fc1(bundle.depth)
        self._validate_transform_bundle(bundle)
        intrinsics = intrinsics_from_camera_matrix(
            bundle.camera_info.width,
            bundle.camera_info.height,
            list(bundle.camera_info.k),
        )
        evidence_bgr = bgr.copy()
        localized_geometry = []
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
                message = self._localized_geometry(
                    bundle, detection, localization
                )
                localized_geometry.append(message)
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
                    evidence_bgr,
                    localization.clamped_bbox,
                    vlm_annotation,
                    (0, 200, 0),
                )
                self._lifecycle_log.append(
                    bundle.observation_id,
                    "LOCALIZED",
                    **self._localization_payload(message, localization),
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
                self._lifecycle_log.append(
                    bundle.observation_id,
                    "REJECTED",
                    detection_id=detection.detection_id,
                    label=detection.detector_class,
                    reason=f"depth localization failed: {exc}",
                )
        self._publish_debug(bundle, bgr, len(detections))
        if not localized_geometry:
            return 0
        evidence = self._compressed_image(bundle, evidence_bgr)
        with self._lock:
            self._persistence_bundles[bundle.observation_id] = bundle
            for message in localized_geometry:
                self._pending_persistence.add(
                    (bundle.observation_id, int(message.detection.detection_id))
                )
        for geometry in localized_geometry:
            self._lifecycle_log.append(
                bundle.observation_id,
                "PERSISTING",
                detection_id=int(geometry.detection.detection_id),
                label=geometry.detection.detector_class,
            )
            self._localized_pub.publish(
                self._minimal_localized_observation(bundle, geometry, evidence)
            )
        return len(localized_geometry)

    def _finish_active_grounding(
        self,
        bundle: ObservationBundle,
        state: str,
        reason: str,
        *,
        rejected: bool,
    ) -> None:
        with self._lock:
            if self._active_bundle is not bundle:
                return
            if state:
                self._lifecycle_log.append(
                    bundle.observation_id, state, reason=reason
                )
            self._queue.complete()
            self._active_bundle = None
            self._active_goal = None
            self._active_started = 0.0
            if rejected:
                self._rejected += 1
            self._last_reason = reason
        self._publish_status()

    def _on_persistence_event(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            observation_id = str(payload["observation_id"])
            detection_id = int(payload["detection_id"])
            state = str(payload["state"])
            if state not in {"STORED", "REJECTED"}:
                raise ValueError("unsupported persistence state")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            with self._lock:
                self._last_reason = f"invalid persistence event: {exc}"
            return
        key = (observation_id, detection_id)
        with self._lock:
            if key not in self._pending_persistence:
                return
            self._lifecycle_log.append(
                observation_id,
                state,
                detection_id=detection_id,
                object_id=str(payload.get("object_id", "")),
                created=payload.get("created"),
                seen_count=payload.get("seen_count"),
                reason=str(payload.get("reason", "")),
            )
            self._pending_persistence.remove(key)
            if not any(item[0] == observation_id for item in self._pending_persistence):
                self._persistence_bundles.pop(observation_id, None)
        self._publish_status()

    def _minimal_localized_observation(
        self,
        bundle: ObservationBundle,
        geometry: LocalizedGeometry,
        evidence: CompressedImage,
    ) -> LocalizedObjectObservation:
        semantic = SemanticObject()
        semantic.detection_id = geometry.detection.detection_id
        semantic.label = geometry.detection.detector_class
        semantic.description = (
            "visually grounded "
            + geometry.detection.detector_class.replace("_", " ")
        )
        semantic.attribute_keys = []
        semantic.attribute_values = []
        semantic.relationships = []
        semantic.useful = True
        semantic.confidence = geometry.detection.confidence

        message = LocalizedObjectObservation()
        message.environment_id = self._environment_id
        message.map_id = self._map_id
        message.observation_id = bundle.observation_id
        message.observation_stamp = geometry.observation_stamp
        message.depth_stamp = geometry.depth_stamp
        message.semantic = semantic
        message.detection = copy.deepcopy(geometry.detection)
        message.map_position = copy.deepcopy(geometry.map_position)
        message.robot_pose = copy.deepcopy(geometry.robot_pose)
        message.scene = "indoor_environment"
        message.localization_quality = geometry.localization_quality
        message.image_ref = ""
        message.image = copy.deepcopy(evidence)
        return message

    def _log_bundle(self, bundle: ObservationBundle, state: str) -> None:
        camera = bundle.camera_transform.transform
        robot = bundle.robot_transform.transform
        self._lifecycle_log.append(
            bundle.observation_id,
            state,
            rgb_stamp_ns=_stamp_ns(bundle.rgb.header.stamp),
            depth_stamp_ns=_stamp_ns(bundle.depth.header.stamp),
            camera_info_stamp_ns=_stamp_ns(bundle.camera_info.header.stamp),
            scan_stamp_ns=bundle.scan_stamp_ns,
            sync_delta_ns=bundle.sync_delta_ns,
            trigger_reason=bundle.trigger_reason,
            rgb_frame=bundle.rgb.header.frame_id,
            map_frame=bundle.camera_transform.header.frame_id,
            camera_translation=[
                camera.translation.x,
                camera.translation.y,
                camera.translation.z,
            ],
            camera_quaternion_xyzw=[
                camera.rotation.x,
                camera.rotation.y,
                camera.rotation.z,
                camera.rotation.w,
            ],
            robot_translation=[
                robot.translation.x,
                robot.translation.y,
                robot.translation.z,
            ],
            robot_quaternion_xyzw=[
                robot.rotation.x,
                robot.rotation.y,
                robot.rotation.z,
                robot.rotation.w,
            ],
            tf_status="exact_timestamp",
        )

    @staticmethod
    def _detection_payload(detection: GroundedDetection) -> dict:
        return {
            "detection_id": detection.detection_id,
            "label": detection.detector_class,
            "confidence": detection.confidence,
            "bbox_pixel_xyxy": [
                int(detection.x_min),
                int(detection.y_min),
                int(detection.x_max),
                int(detection.y_max),
            ],
        }

    @staticmethod
    def _localization_payload(
        message: LocalizedGeometry,
        localization: DepthLocalizationResult,
    ) -> dict:
        return {
            "detection_id": int(message.detection.detection_id),
            "label": message.detection.detector_class,
            "bbox_pixel_xyxy": list(localization.clamped_bbox),
            "valid_depth_ratio": localization.valid_depth_ratio,
            "depth_mad_m": localization.depth_mad_m,
            "localization_quality": localization.localization_quality,
            "camera_xyz": [localization.x, localization.y, localization.z],
            "map_xyz": [
                message.map_position.point.x,
                message.map_position.point.y,
                message.map_position.point.z,
            ],
            "tf_status": "exact_timestamp",
        }

    def _reject(self, reason: str) -> None:
        with self._lock:
            self._rejected += 1
            self._last_reason = reason

    def _publish_status(self) -> None:
        with self._lock:
            payload = {
                "backend": "vlm_grounding",
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
                "pending_persistence": len(self._pending_persistence),
            }
            semantic_payload = {
                "ready": (
                    self._grounding_client.server_is_ready()
                    and self._accepting
                ),
                "accepting": self._accepting,
                "active": self._active_bundle is not None,
                "pending": self._queue.has_pending,
                "observation_drained": (
                    not self._queue.active and not self._queue.has_pending
                ),
                "drained": (
                    self._finalizing
                    and self._active_bundle is None
                    and not self._queue.active
                    and not self._queue.has_pending
                    and not self._pending_persistence
                ),
                "pending_persistence": len(self._pending_persistence),
                "last_reason": self._last_reason,
            }
        self._status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )
        self._semantic_status_pub.publish(
            String(
                data=json.dumps(
                    semantic_payload,
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )
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

    def _localized_geometry(
        self,
        bundle: ObservationBundle,
        detection: GroundedDetection,
        localization: DepthLocalizationResult,
    ) -> LocalizedGeometry:
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

        return LocalizedGeometry(
            observation_stamp=copy.deepcopy(bundle.rgb.header.stamp),
            depth_stamp=copy.deepcopy(bundle.depth.header.stamp),
            detection=detection_message,
            map_position=map_position,
            robot_pose=robot_pose,
            localization_quality=localization.localization_quality,
        )

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
        detection: GroundedDetection, width: int, height: int
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

    def _boolean_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value


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
