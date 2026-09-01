"""Supervise autonomous exploration and map finalization."""

from __future__ import annotations

import json
import math
import re
import time
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from environment_memory.exploration.readiness import ReadinessSnapshot
from environment_memory_interfaces.msg import ExplorationStatus


class MemoryBuildManager(Node):
    """Start frontier exploration after readiness and save the completed map."""

    def __init__(self) -> None:
        super().__init__("memory_build_manager")
        self._declare_parameters()
        self._environment_id = self._string_parameter("environment_id")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self._environment_id):
            raise ValueError("environment_id may contain letters, numbers, _ and -")
        self._map_frame = self._string_parameter("map_frame")
        self._odom_frame = self._string_parameter("odom_frame")
        self._base_frame = self._string_parameter("base_frame")
        self._base_link_frame = self._string_parameter("base_link_frame")
        self._camera_frame = self._string_parameter("camera_frame")
        self._readiness_timeout_s = self._float_parameter("readiness_timeout_s")
        self._finalization_timeout_s = self._float_parameter(
            "finalization_timeout_s"
        )
        self._started_monotonic = time.monotonic()
        self._finalization_started: Optional[float] = None
        self._map_received = False
        self._scan_received = False
        self._control_requested = False
        self._control_future = None
        self._map_save_future = None
        self._memory_drained = False
        self._memory_ready = False
        self._semantic_ready = False
        self._state = ExplorationStatus.WAITING
        self._reason = "waiting for map, scan, Nav2, TF, and explorer"
        self._current_goal = PoseStamped()
        self._goal_states: dict[bytes, int] = {}
        self._goals_seen = 0
        self._goals_succeeded = 0
        self._goals_failed = 0

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            ExplorationStatus, "/exploration/status", status_qos
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Empty,
            self._string_parameter("completion_topic"),
            self._on_exploration_complete,
            status_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/explore/selected_frontier",
            self._on_selected_frontier,
            10,
        )
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(
            String,
            "/environment_memory/status",
            self._on_memory_status,
            10,
        )
        self.create_subscription(
            String,
            "/environment_memory/semantic_status",
            self._on_semantic_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_goal_status,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._control_client = self.create_client(
            ControlExploration, self._string_parameter("control_service")
        )
        self._save_map_client = self.create_client(
            SaveMap, self._string_parameter("map_save_service")
        )
        lifecycle_names = self.get_parameter("lifecycle_nodes").value
        self._lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in lifecycle_names
        }
        self._lifecycle_futures = {name: None for name in lifecycle_names}
        self._lifecycle_states = {name: "unknown" for name in lifecycle_names}

        output = self._string_parameter("map_output_path").strip()
        if output:
            self._map_output_path = Path(output).expanduser()
        else:
            storage_root = self._string_parameter("storage_root").strip()
            if storage_root:
                root = Path(storage_root).expanduser()
            else:
                root = (
                    Path.home()
                    / ".local"
                    / "share"
                    / "flzat"
                    / "environment_memory"
                )
            self._map_output_path = (
                root
                / self._environment_id
                / "maps"
                / self._environment_id
            )

        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("completion_topic", "/exploration/complete")
        self.declare_parameter("control_service", "/control_exploration")
        self.declare_parameter("map_save_service", "/slam_toolbox/save_map")
        self.declare_parameter("map_output_path", "")
        self.declare_parameter("storage_root", "")
        self.declare_parameter("readiness_timeout_s", 120.0)
        self.declare_parameter("finalization_timeout_s", 30.0)
        self.declare_parameter(
            "lifecycle_nodes",
            ["controller_server", "planner_server", "bt_navigator"],
        )

    def _tick(self) -> None:
        self._refresh_lifecycle_states()
        if self._state == ExplorationStatus.WAITING:
            self._tick_waiting()
        elif self._state == ExplorationStatus.EXPLORING:
            self._tick_control_response()
        elif self._state == ExplorationStatus.FINALIZING:
            self._tick_map_save()

    def _tick_waiting(self) -> None:
        if time.monotonic() - self._started_monotonic > self._readiness_timeout_s:
            self._fail("readiness timeout expired")
            return
        readiness = self._readiness()
        if not readiness.ready:
            self._reason = "waiting for " + ", ".join(readiness.missing)
            return
        if not self._control_requested:
            request = ControlExploration.Request()
            request.action = ControlExploration.Request.ACTION_START
            request.delay_seconds = 0.0
            request.quit_after_stop = False
            self._control_future = self._control_client.call_async(request)
            self._control_requested = True
            self._reason = "requesting frontier exploration start"
        self._tick_control_response()

    def _tick_control_response(self) -> None:
        if self._state != ExplorationStatus.WAITING or self._control_future is None:
            return
        if not self._control_future.done():
            return
        try:
            response = self._control_future.result()
        except Exception as exc:
            self._fail(f"frontier start service failed: {exc}")
            return
        if response is None or not response.accepted:
            message = "no response" if response is None else response.message
            self._fail(f"frontier start rejected: {message}")
            return
        self._state = ExplorationStatus.EXPLORING
        self._reason = response.message or "frontier exploration running"
        self._publish_status()

    def _readiness(self) -> ReadinessSnapshot:
        return ReadinessSnapshot(
            map_received=self._map_received,
            scan_received=self._scan_received,
            nav_action_ready=self._nav_client.server_is_ready(),
            lifecycle_active=all(
                state == "active" for state in self._lifecycle_states.values()
            ),
            map_to_odom_tf_ready=self._can_transform(
                self._map_frame, self._odom_frame
            ),
            odom_to_base_tf_ready=self._can_transform(
                self._odom_frame, self._base_frame
            ),
            base_to_base_link_tf_ready=self._can_transform(
                self._base_frame, self._base_link_frame
            ),
            base_link_to_camera_tf_ready=self._can_transform(
                self._base_link_frame, self._camera_frame
            ),
            explorer_control_ready=self._control_client.service_is_ready(),
            memory_manager_ready=self._memory_ready,
            semantic_pipeline_ready=self._semantic_ready,
        )

    def _refresh_lifecycle_states(self) -> None:
        for name, client in self._lifecycle_clients.items():
            future = self._lifecycle_futures[name]
            if future is not None and future.done():
                try:
                    response = future.result()
                    self._lifecycle_states[name] = response.current_state.label
                except Exception:
                    self._lifecycle_states[name] = "unknown"
                self._lifecycle_futures[name] = None
            if self._lifecycle_futures[name] is None and client.service_is_ready():
                self._lifecycle_futures[name] = client.call_async(GetState.Request())

    def _can_transform(self, target_frame: str, source_frame: str) -> bool:
        try:
            return self._tf_buffer.can_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return False

    def _on_map(self, _message: OccupancyGrid) -> None:
        self._map_received = True

    def _on_scan(self, message: LaserScan) -> None:
        if message.header.frame_id and math.isfinite(message.range_min):
            self._scan_received = True

    def _on_selected_frontier(self, message: PoseStamped) -> None:
        self._current_goal = message

    def _on_plan(self, message: NavPath) -> None:
        if message.poses:
            self._current_goal = message.poses[-1]

    def _on_goal_status(self, message: GoalStatusArray) -> None:
        terminal = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        for entry in message.status_list:
            goal_id = bytes(entry.goal_info.goal_id.uuid)
            previous = self._goal_states.get(goal_id)
            if previous is None:
                self._goals_seen += 1
            if previous != entry.status:
                if entry.status == GoalStatus.STATUS_SUCCEEDED:
                    self._goals_succeeded += 1
                elif entry.status in {
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                }:
                    self._goals_failed += 1
            self._goal_states[goal_id] = entry.status
            if entry.status in terminal:
                self._current_goal = PoseStamped()

    def _on_exploration_complete(self, _message: Empty) -> None:
        if self._state != ExplorationStatus.EXPLORING:
            return
        self._state = ExplorationStatus.FINALIZING
        self._reason = "frontiers exhausted; saving SLAM map"
        self._finalization_started = time.monotonic()
        self._publish_status()

    def _tick_map_save(self) -> None:
        if self._finalization_started is None:
            self._finalization_started = time.monotonic()
        if time.monotonic() - self._finalization_started > self._finalization_timeout_s:
            self._fail("map save finalization timeout expired")
            return
        if not self._memory_drained:
            self._reason = "waiting for Memory Manager to drain and checkpoint"
            return
        if self._map_save_future is None:
            if not self._save_map_client.service_is_ready():
                self._reason = "waiting for SLAM Toolbox save_map service"
                return
            self._map_output_path.parent.mkdir(parents=True, exist_ok=True)
            request = SaveMap.Request()
            request.name = String(data=str(self._map_output_path))
            self._map_save_future = self._save_map_client.call_async(request)
            self._reason = f"saving map to {self._map_output_path}"
            return
        if not self._map_save_future.done():
            return
        try:
            response = self._map_save_future.result()
        except Exception as exc:
            self._fail(f"map save service failed: {exc}")
            return
        if response is None or int(response.result) != 0:
            result = "no response" if response is None else str(response.result)
            self._fail(f"map save failed with result {result}")
            return
        self._state = ExplorationStatus.COMPLETED
        self._reason = f"exploration complete; map saved to {self._map_output_path}"
        self._publish_status()

    def _on_memory_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._memory_ready = (
                payload.get("accepting") is True
                and payload.get("manifest_status") == "incomplete"
            )
            self._memory_drained = (
                payload.get("accepting") is False
                and int(payload.get("active_upserts", -1)) == 0
                and payload.get("manifest_status") == "incomplete"
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            self._memory_ready = False
            self._memory_drained = False

    def _on_semantic_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._semantic_ready = payload.get("ready") is True
        except (TypeError, json.JSONDecodeError):
            self._semantic_ready = False

    def _fail(self, reason: str) -> None:
        self._state = ExplorationStatus.FAILED
        self._reason = reason
        self.get_logger().error(reason)
        self._publish_status()

    def _publish_status(self) -> None:
        message = ExplorationStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self._state
        message.goals_seen = self._goals_seen
        message.goals_succeeded = self._goals_succeeded
        message.goals_failed = self._goals_failed
        message.current_goal = self._current_goal
        message.reason = self._reason
        self._status_pub.publish(message)

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[MemoryBuildManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = MemoryBuildManager()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("MemoryBuildManager shutting down …")
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
