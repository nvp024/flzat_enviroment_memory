"""Tests for read-only RViz object-coordinate markers."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from environment_memory.retrieval.memory_marker_publisher import (
    LABEL_NAMESPACE,
    MARKER_TOPIC,
    POINT_NAMESPACE,
    build_marker_array,
)


def _record(object_id, label, scene, x, y, z):
    return SimpleNamespace(
        object_id=object_id,
        label=label,
        scene=scene,
        map_position=SimpleNamespace(x=x, y=y, z=z),
    )


def test_marker_array_contains_one_point_and_label_per_record():
    """Every database record produces one point and one text label."""
    records = (
        _record("tv", "tv", "living_room", 5.95, 6.90, 0.25),
        _record("bed", "bed", "bedroom", 11.15, 2.20, 0.25),
    )

    message = build_marker_array(records, Time(sec=12, nanosec=34))

    assert MARKER_TOPIC == "/environment_memory/object_markers"
    assert len(message.markers) == 5
    assert message.markers[0].action == Marker.DELETEALL
    point, label = message.markers[1:3]
    assert point.ns == POINT_NAMESPACE
    assert point.header.frame_id == "map"
    assert point.type == Marker.SPHERE
    assert point.pose.position.x == 11.15
    assert point.pose.position.y == 2.20
    assert point.pose.position.z == 0.25
    assert label.ns == LABEL_NAMESPACE
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == "bed (11.15, 2.20, 0.25)"


def test_marker_records_are_sorted_for_stable_ids():
    """Marker IDs remain stable even if Chroma changes return ordering."""
    records = (
        _record("z-object", "z", "unknown", 2.0, 0.0, 0.0),
        _record("a-object", "a", "unknown", 1.0, 0.0, 0.0),
    )

    message = build_marker_array(records, Time())

    assert message.markers[1].pose.position.x == 1.0
    assert message.markers[3].pose.position.x == 2.0
