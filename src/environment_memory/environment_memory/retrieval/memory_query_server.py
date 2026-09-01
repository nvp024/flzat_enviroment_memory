"""Serve filtered semantic queries over completed memory."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from environment_memory.storage.embedding import SentenceTransformerEmbedder
from environment_memory.storage.memory_record import (
    EMBEDDING_MODEL,
    EMBEDDING_MODEL_REVISION,
    MapPosition,
)
from environment_memory.storage.readonly_memory import load_completed_manifest
from environment_memory.retrieval.search import (
    MAX_RESULTS,
    MemoryQuery,
    ReadOnlyChromaMemoryStore,
    SemanticRetriever,
)
from environment_memory_interfaces.msg import MemoryObject
from environment_memory_interfaces.srv import QueryMemory


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _time_message(timestamp_ns: int) -> TimeMessage:
    message = TimeMessage()
    message.sec = timestamp_ns // 1_000_000_000
    message.nanosec = timestamp_ns % 1_000_000_000
    return message


class ReadOnlyMemoryManager(Node):
    """Serve semantic queries against one completed, immutable build session."""

    def __init__(self) -> None:
        super().__init__("memory_manager")
        self._declare_parameters()
        environment_id = self._safe_id("environment_id")
        map_id = self._string_parameter("map_id").strip()
        if map_id and not re.fullmatch(r"[A-Za-z0-9_-]+", map_id):
            raise ValueError("map_id may contain letters, numbers, _ and -")
        storage_root_value = self._string_parameter("storage_root").strip()
        storage_root = (
            Path(storage_root_value).expanduser()
            if storage_root_value
            else Path.home()
            / ".local"
            / "share"
            / "flzat"
            / "environment_memory"
        )
        environment_root = (storage_root / environment_id).resolve()
        self._manifest = load_completed_manifest(
            environment_root, environment_id, map_id
        )
        embedder = SentenceTransformerEmbedder(
            EMBEDDING_MODEL,
            EMBEDDING_MODEL_REVISION,
            self._string_parameter("embedding_device"),
            self._boolean_parameter("embedding_local_files_only"),
        )
        store = ReadOnlyChromaMemoryStore(self._manifest.database_path)
        self._retriever = SemanticRetriever(store, embedder, self._manifest)

        self._service = self.create_service(
            QueryMemory,
            "/environment_memory/query",
            self._on_query,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/environment_memory/status", status_qos
        )
        self._publish_status()
        self.get_logger().info(
            "Read-only memory ready: "
            f"environment={self._manifest.environment_id}, "
            f"map={self._manifest.map_id}, objects={store.count()}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "")
        self.declare_parameter("storage_root", "")
        self.declare_parameter("embedding_device", "cpu")
        self.declare_parameter("embedding_local_files_only", False)

    def _on_query(self, request, response):
        try:
            start_ns = _stamp_ns(request.start_time)
            end_ns = _stamp_ns(request.end_time)
            radius = float(request.radius_m)
            if not math.isfinite(radius) or radius < 0.0:
                raise ValueError("radius must be finite and nonnegative")
            center = None
            radius_value = None
            if radius > 0.0:
                center = MapPosition(
                    "map",
                    float(request.center.x),
                    float(request.center.y),
                    float(request.center.z),
                )
                radius_value = radius
            query = MemoryQuery(
                text=request.query,
                top_k=int(request.top_k) if request.top_k else MAX_RESULTS,
                environment_id=request.environment_id,
                map_id=request.map_id,
                scene=request.scene,
                start_ros_ns=start_ns or None,
                end_ros_ns=end_ns or None,
                center=center,
                radius_m=radius_value,
            )
            hits = self._retriever.query(query)
            response.objects = [self._to_message(hit.stored.record) for hit in hits]
            response.cosine_scores = [float(hit.cosine_score) for hit in hits]
            response.success = True
            response.error_message = ""
        except Exception as exc:
            response.success = False
            response.objects = []
            response.cosine_scores = []
            response.error_message = str(exc)
            self.get_logger().warn(f"Memory query rejected: {exc}")
        return response

    def _to_message(self, record) -> MemoryObject:
        message = MemoryObject()
        message.object_id = record.object_id
        message.environment_id = record.environment_id
        message.map_id = record.map_id
        message.detector_class = record.detector_class
        message.label = record.label
        message.description = record.description
        message.attribute_keys = [key for key, _ in record.attributes]
        message.attribute_values = [value for _, value in record.attributes]
        message.relationships = list(record.relationships)
        message.scene = record.scene
        message.map_position = PointStamped()
        message.map_position.header.frame_id = "map"
        message.map_position.header.stamp = _time_message(record.last_seen_ros_ns)
        message.map_position.point = Point(
            x=record.map_position.x,
            y=record.map_position.y,
            z=record.map_position.z,
        )
        message.robot_pose = PoseStamped()
        message.robot_pose.header = message.map_position.header
        message.robot_pose.pose.position = Point(
            x=record.robot_pose.x,
            y=record.robot_pose.y,
            z=record.robot_pose.z,
        )
        half_yaw = record.robot_pose.yaw / 2.0
        message.robot_pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw)
        )
        message.first_seen = _time_message(record.first_seen_ros_ns)
        message.last_seen = _time_message(record.last_seen_ros_ns)
        message.seen_count = record.seen_count
        message.detector_confidence = record.detector_confidence
        message.semantic_confidence = record.semantic_confidence
        message.localization_quality = record.localization_quality
        message.confidence = record.confidence
        message.image_ref = record.image_ref
        return message

    def _publish_status(self) -> None:
        payload = {
            "mode": "read_only",
            "environment_id": self._manifest.environment_id,
            "map_id": self._manifest.map_id,
            "manifest_status": "complete",
            "object_count": self._retriever.store.count(),
            "query_service": "/environment_memory/query",
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


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[ReadOnlyMemoryManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = ReadOnlyMemoryManager()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
