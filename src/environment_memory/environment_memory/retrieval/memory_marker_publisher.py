"""Publish completed semantic-memory object coordinates for RViz."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from environment_memory.retrieval.search import ReadOnlyChromaMemoryStore
from environment_memory.storage.memory_record import MemoryRecord
from environment_memory.storage.readonly_memory import load_completed_manifest


MARKER_TOPIC = "/environment_memory/object_markers"
POINT_NAMESPACE = "rag_object_points"
LABEL_NAMESPACE = "rag_object_coordinates"
POINT_SCALE_M = 0.24
LABEL_HEIGHT_M = 0.24
LABEL_OFFSET_M = 0.38

SCENE_COLORS = {
    "bedroom": (0.72, 0.36, 0.92),
    "dining_area": (0.96, 0.70, 0.22),
    "entrance": (0.26, 0.78, 0.42),
    "kitchen": (0.96, 0.48, 0.18),
    "living_room": (0.20, 0.62, 0.96),
}


def build_marker_array(
    records: Sequence[MemoryRecord], stamp
) -> MarkerArray:
    """Build one point and coordinate label for every memory record."""
    message = MarkerArray()
    clear = Marker()
    clear.header.frame_id = "map"
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    message.markers.append(clear)

    ordered_records = sorted(records, key=lambda item: item.object_id)
    for index, record in enumerate(ordered_records):
        position = record.map_position
        color = SCENE_COLORS.get(record.scene, (0.20, 0.85, 0.82))

        point = Marker()
        point.header.frame_id = "map"
        point.header.stamp = stamp
        point.ns = POINT_NAMESPACE
        point.id = index
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.position.x = float(position.x)
        point.pose.position.y = float(position.y)
        point.pose.position.z = float(position.z)
        point.pose.orientation.w = 1.0
        point.scale.x = POINT_SCALE_M
        point.scale.y = POINT_SCALE_M
        point.scale.z = POINT_SCALE_M
        point.color.r, point.color.g, point.color.b = color
        point.color.a = 0.95
        point.frame_locked = True
        message.markers.append(point)

        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = stamp
        label.ns = LABEL_NAMESPACE
        label.id = index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(position.x)
        label.pose.position.y = float(position.y)
        label.pose.position.z = float(position.z) + LABEL_OFFSET_M
        label.pose.orientation.w = 1.0
        label.scale.z = LABEL_HEIGHT_M
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = (
            f"{record.label} "
            f"({position.x:.2f}, {position.y:.2f}, {position.z:.2f})"
        )
        label.frame_locked = True
        message.markers.append(label)

    return message


class MemoryMarkerPublisher(Node):
    """Load one completed RAG database and expose its coordinates to RViz."""

    def __init__(self) -> None:
        """Load the completed memory and publish its persistent marker set."""
        super().__init__("memory_marker_publisher")
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "")
        self.declare_parameter("storage_root", "")

        environment_id = self._safe_id("environment_id", required=True)
        map_id = self._safe_id("map_id", required=False)
        storage_value = self._string_parameter("storage_root").strip()
        storage_root = (
            Path(storage_value).expanduser()
            if storage_value
            else Path.home()
            / ".local"
            / "share"
            / "flzat"
            / "environment_memory"
        )
        manifest = load_completed_manifest(
            storage_root / environment_id,
            expected_environment_id=environment_id,
            expected_map_id=map_id,
        )
        self._store = ReadOnlyChromaMemoryStore(manifest.database_path)
        if self._store.count() != manifest.object_count:
            raise RuntimeError(
                "database object count does not match completed manifest"
            )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)
        records = self._store.records()
        message = build_marker_array(records, self.get_clock().now().to_msg())
        self._publisher.publish(message)
        self.get_logger().info(
            "Published "
            f"{len(records)} RAG object coordinates on {MARKER_TOPIC}"
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _safe_id(self, name: str, required: bool) -> str:
        value = self._string_parameter(name).strip()
        if not value and not required:
            return ""
        if not re.fullmatch(r"[A-Za-z0-9_-]+", value):
            raise ValueError(f"{name} may contain letters, numbers, _ and -")
        return value


def main(args=None) -> None:
    """Run the read-only RViz marker publisher."""
    rclpy.init(args=args)
    node: Optional[MemoryMarkerPublisher] = None
    try:
        node = MemoryMarkerPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
