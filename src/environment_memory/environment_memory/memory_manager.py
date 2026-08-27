from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import threading
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from environment_memory.embedding import SentenceTransformerEmbedder
from environment_memory.memory_record import (
    EMBEDDING_MODEL,
    EMBEDDING_MODEL_REVISION,
    IncomingMemoryObservation,
    MapPosition,
    RobotPose,
)
from environment_memory.memory_service import MemoryService
from environment_memory.memory_store import ChromaMemoryStore
from environment_memory_interfaces.msg import (
    ExplorationStatus,
    LocalizedObjectObservation,
)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(rotation) -> float:
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _snake_case(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")


class MemoryManager(Node):
    """The sole writable owner of Version 1 long-term environment memory."""

    def __init__(self) -> None:
        super().__init__("memory_manager")
        self._declare_parameters()
        self._status_lock = threading.Lock()
        self._finalization_lock = threading.Lock()
        self._accepted = 0
        self._created = 0
        self._merged = 0
        self._rejected = 0
        self._last_reason = "initializing persistent memory"
        self._finalized = False
        self._finalization_requested = False
        self._semantic_drained = False

        environment_id = self._safe_id("environment_id")
        map_id = self._safe_id("map_id")
        root_value = self._string_parameter("storage_root").strip()
        if root_value:
            storage_root = Path(root_value).expanduser()
        else:
            storage_root = (
                Path.home() / ".local" / "share" / "flzat" / "environment_memory"
            )
        self._environment_root = (storage_root / environment_id).resolve()
        map_output = self._string_parameter("map_output_path").strip()
        if map_output:
            self._map_yaml = Path(map_output).expanduser().with_suffix(".yaml")
        else:
            self._map_yaml = (
                self._environment_root / "maps" / environment_id
            ).with_suffix(".yaml")

        embedder = SentenceTransformerEmbedder(
            EMBEDDING_MODEL,
            EMBEDDING_MODEL_REVISION,
            self._string_parameter("embedding_device"),
            self._boolean_parameter("embedding_local_files_only"),
        )
        store = ChromaMemoryStore(self._environment_root / "chroma")
        self._service = MemoryService(
            self._environment_root,
            environment_id,
            map_id,
            store,
            embedder,
        )
        self._last_reason = f"writable memory ready; recovered {store.count()} objects"

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        exploration_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/environment_memory/status", reliable
        )
        self.create_subscription(
            LocalizedObjectObservation,
            "/environment_memory/localized_observations",
            self._on_observation,
            reliable,
        )
        self.create_subscription(
            ExplorationStatus,
            "/exploration/status",
            self._on_exploration_status,
            exploration_qos,
        )
        self.create_subscription(
            String,
            "/environment_memory/semantic_status",
            self._on_semantic_status,
            exploration_qos,
        )
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "mapping-session")
        self.declare_parameter("storage_root", "")
        self.declare_parameter("map_output_path", "")
        self.declare_parameter("embedding_device", "cpu")
        self.declare_parameter("embedding_local_files_only", False)

    def _on_observation(self, message: LocalizedObjectObservation) -> None:
        try:
            observation = self._convert_observation(message)
            result = self._service.upsert(observation, bytes(message.image.data))
        except Exception as exc:
            with self._status_lock:
                self._rejected += 1
                self._last_reason = f"observation rejected: {exc}"
            self.get_logger().warn(self._last_reason)
            self._publish_status()
            return
        with self._status_lock:
            self._accepted += 1
            if result.created:
                self._created += 1
            else:
                self._merged += 1
            operation = "created" if result.created else "merged"
            self._last_reason = f"{operation} object {result.record.object_id}"
        self._publish_status()

    def _convert_observation(
        self, message: LocalizedObjectObservation
    ) -> IncomingMemoryObservation:
        stamp_ns = _stamp_ns(message.observation_stamp)
        if stamp_ns <= 0:
            raise ValueError("observation timestamp must be positive")
        if message.detection.observation_id != message.observation_id:
            raise ValueError("detector observation_id does not match")
        if message.semantic.detection_id != message.detection.detection_id:
            raise ValueError("semantic detection_id does not match detection")
        if message.map_position.header.frame_id != "map":
            raise ValueError("object position must use map frame")
        if message.robot_pose.header.frame_id != "map":
            raise ValueError("robot pose must use map frame")
        if _stamp_ns(message.map_position.header.stamp) != stamp_ns:
            raise ValueError("object map position timestamp does not match observation")
        if _stamp_ns(message.robot_pose.header.stamp) != stamp_ns:
            raise ValueError("robot pose timestamp does not match observation")
        if not message.semantic.useful:
            raise ValueError("semantic object is not marked useful")
        if len(message.semantic.attribute_keys) != len(
            message.semantic.attribute_values
        ):
            raise ValueError("semantic attribute keys and values differ in length")
        if not message.image.data or "jpeg" not in message.image.format.lower():
            raise ValueError("accepted object requires JPEG keyframe evidence")
        if _stamp_ns(message.image.header.stamp) != stamp_ns:
            raise ValueError("keyframe timestamp does not match observation")
        rotation = message.robot_pose.pose.orientation
        quaternion_norm = math.sqrt(
            rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
            + rotation.w * rotation.w
        )
        if not math.isfinite(quaternion_norm) or quaternion_norm <= 1e-9:
            raise ValueError("robot pose orientation is invalid")
        return IncomingMemoryObservation(
            environment_id=message.environment_id,
            map_id=message.map_id,
            observation_id=message.observation_id,
            detector_class=_snake_case(message.detection.detector_class),
            label=message.semantic.label,
            description=message.semantic.description,
            attributes=tuple(
                zip(
                    message.semantic.attribute_keys,
                    message.semantic.attribute_values,
                )
            ),
            relationships=tuple(message.semantic.relationships),
            scene=message.scene,
            map_position=MapPosition(
                frame_id="map",
                x=message.map_position.point.x,
                y=message.map_position.point.y,
                z=message.map_position.point.z,
            ),
            robot_pose=RobotPose(
                x=message.robot_pose.pose.position.x,
                y=message.robot_pose.pose.position.y,
                z=message.robot_pose.pose.position.z,
                yaw=_yaw_from_quaternion(rotation),
            ),
            observed_utc=datetime.now(timezone.utc).isoformat(),
            observed_ros_ns=stamp_ns,
            detector_confidence=message.detection.confidence,
            semantic_confidence=message.semantic.confidence,
            localization_quality=message.localization_quality,
        )

    def _on_exploration_status(self, message: ExplorationStatus) -> None:
        if message.state == ExplorationStatus.FINALIZING:
            self._finalization_requested = True
            self._last_reason = "waiting for structured VLM pipeline to drain"
            self._try_stop_for_finalization()
        elif message.state == ExplorationStatus.COMPLETED and not self._finalized:
            try:
                manifest = self._service.finalize(self._map_yaml)
                self._finalized = True
                self._last_reason = (
                    f"manifest complete with {manifest.object_count} objects"
                )
            except Exception as exc:
                self._last_reason = f"manifest finalization failed: {exc}"
                self.get_logger().error(self._last_reason)
        elif message.state == ExplorationStatus.FAILED and self._service.accepting:
            try:
                self._service.stop_accepting_and_flush()
                self._last_reason = "build failed; incomplete memory checkpoint saved"
            except Exception as exc:
                self._last_reason = f"failed-build checkpoint failed: {exc}"
                self.get_logger().error(self._last_reason)
        self._publish_status()

    def _on_semantic_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._semantic_drained = payload.get("drained") is True
        except (TypeError, json.JSONDecodeError):
            self._semantic_drained = False
        self._try_stop_for_finalization()
        self._publish_status()

    def _try_stop_for_finalization(self) -> None:
        with self._finalization_lock:
            if not (
                self._finalization_requested
                and self._semantic_drained
                and self._service.accepting
            ):
                return
            try:
                self._service.stop_accepting_and_flush()
                self._last_reason = "memory drained; waiting for saved map"
            except Exception as exc:
                self._last_reason = f"memory drain failed: {exc}"
                self.get_logger().error(self._last_reason)

    def _publish_status(self) -> None:
        with self._status_lock:
            payload = {
                "accepted": self._accepted,
                "created": self._created,
                "merged": self._merged,
                "rejected": self._rejected,
                "object_count": self._service.store.count(),
                "accepting": self._service.accepting,
                "active_upserts": self._service.active_upserts,
                "manifest_status": self._service.manifest.status,
                "finalized": self._finalized,
                "semantic_drained": self._semantic_drained,
                "last_reason": self._last_reason,
            }
        self._status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )

    def _safe_id(self, name: str) -> str:
        value = self._string_parameter(name)
        if not re.fullmatch(r"[A-Za-z0-9_-]+", value):
            raise ValueError(f"{name} may contain letters, numbers, _ and -")
        return value

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _boolean_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def destroy_node(self) -> None:
        if hasattr(self, "_service") and self._service.accepting:
            try:
                self._service.stop_accepting_and_flush()
            except Exception as exc:
                self.get_logger().error(f"shutdown checkpoint failed: {exc}")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[MemoryManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = MemoryManager()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("MemoryManager shutting down …")
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
