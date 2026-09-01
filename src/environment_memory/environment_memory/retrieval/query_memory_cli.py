"""Expose completed-memory queries as a command-line client."""

from __future__ import annotations

import argparse
import json
from typing import Optional

import rclpy
from rclpy.node import Node

from environment_memory_interfaces.srv import QueryMemory


class QueryMemoryClient(Node):
    def __init__(self) -> None:
        super().__init__("query_memory_cli")
        self.client = self.create_client(QueryMemory, "/environment_memory/query")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Query completed environment memory")
    parser.add_argument("query", help="semantic query text")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--environment-id", default="")
    parser.add_argument("--map-id", default="")
    parser.add_argument("--scene", default="")
    parser.add_argument("--start-ros-ns", type=int)
    parser.add_argument("--end-ros-ns", type=int)
    parser.add_argument("--center-x", type=float)
    parser.add_argument("--center-y", type=float)
    parser.add_argument("--center-z", type=float)
    parser.add_argument("--radius-m", type=float)
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser


def main(args: Optional[list[str]] = None) -> None:
    parsed, ros_args = _parser().parse_known_args(args)
    radius_values = (
        parsed.center_x,
        parsed.center_y,
        parsed.center_z,
        parsed.radius_m,
    )
    if any(value is not None for value in radius_values) and not all(
        value is not None for value in radius_values
    ):
        raise ValueError(
            "--center-x, --center-y, --center-z, and --radius-m must be used together"
        )
    rclpy.init(args=ros_args)
    node = QueryMemoryClient()
    try:
        if not node.client.wait_for_service(timeout_sec=parsed.timeout):
            raise RuntimeError("/environment_memory/query is unavailable")
        request = QueryMemory.Request()
        request.query = parsed.query
        request.top_k = parsed.top_k
        request.environment_id = parsed.environment_id
        request.map_id = parsed.map_id
        request.scene = parsed.scene
        if parsed.start_ros_ns is not None:
            request.start_time.sec = parsed.start_ros_ns // 1_000_000_000
            request.start_time.nanosec = parsed.start_ros_ns % 1_000_000_000
        if parsed.end_ros_ns is not None:
            request.end_time.sec = parsed.end_ros_ns // 1_000_000_000
            request.end_time.nanosec = parsed.end_ros_ns % 1_000_000_000
        if parsed.radius_m is not None:
            request.center.x = parsed.center_x
            request.center.y = parsed.center_y
            request.center.z = parsed.center_z
            request.radius_m = parsed.radius_m
        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=parsed.timeout)
        if not future.done():
            raise RuntimeError("memory query timed out")
        response = future.result()
        if response is None or not response.success:
            error = "no response" if response is None else response.error_message
            raise RuntimeError(f"memory query failed: {error}")
        payload = []
        for item, score in zip(response.objects, response.cosine_scores):
            payload.append(
                {
                    "object_id": item.object_id,
                    "label": item.label,
                    "description": item.description,
                    "scene": item.scene,
                    "map_position": {
                        "x": item.map_position.point.x,
                        "y": item.map_position.point.y,
                        "z": item.map_position.point.z,
                    },
                    "image_ref": item.image_ref,
                    "cosine_score": score,
                }
            )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
