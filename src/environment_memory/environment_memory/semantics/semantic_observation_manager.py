"""Join shared VLM semantics to detector-linked map geometry."""

from __future__ import annotations

import copy
import json
import re
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from environment_memory.perception.observation_queue import LatestObservationQueue
from environment_memory.semantics.semantic_batch import (
    SemanticBatchError,
    SemanticObservationBatch,
    SemanticBatchAssembler,
    join_semantics,
)
from environment_memory_interfaces.msg import (
    ExplorationStatus,
    GeometricObjectObservation,
    LocalizedObjectObservation,
    VlmObservation,
)
from robot_interfaces.action import AnalyzeEnvironment


class SemanticObservationManager(Node):
    """Schedule frozen observations and join VLM semantics to Phase 5 geometry."""

    def __init__(self) -> None:
        super().__init__("semantic_observation_manager")
        self._declare_parameters()
        self._environment_id = self._safe_id("environment_id")
        self._map_id = self._safe_id("map_id")
        self._batch_timeout_s = self._float_parameter("batch_timeout_s")
        self._action_timeout_s = self._float_parameter("action_timeout_s")
        if self._batch_timeout_s <= 0.0 or self._action_timeout_s <= 0.0:
            raise ValueError("Phase 6 timeout parameters must be positive")

        self._lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()
        self._assembler = SemanticBatchAssembler(
            self._integer_parameter("maximum_partial_batches")
        )
        self._queue = LatestObservationQueue[SemanticObservationBatch]()
        self._active_batch: SemanticObservationBatch | None = None
        self._active_goal = None
        self._active_started = 0.0
        self._finalizing = False
        self._observation_drained = False
        self._accepting = True
        self._submitted = 0
        self._replaced = 0
        self._completed = 0
        self._published = 0
        self._rejected = 0
        self._expired = 0
        self._last_reason = "waiting for shared VLM server"

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._localized_pub = self.create_publisher(
            LocalizedObjectObservation,
            "/environment_memory/localized_observations",
            reliable,
        )
        self._status_pub = self.create_publisher(
            String, "/environment_memory/semantic_status", latched
        )
        self.create_subscription(
            GeometricObjectObservation,
            "/environment_memory/geometric_observations",
            self._on_geometry,
            reliable,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            VlmObservation,
            "/environment_memory/vlm_observations",
            self._on_evidence,
            reliable,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String,
            "/environment_memory/observation_status",
            self._on_observation_status,
            reliable,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            ExplorationStatus,
            "/exploration/status",
            self._on_exploration_status,
            latched,
            callback_group=self._cb_group,
        )
        self._client = ActionClient(
            self,
            AnalyzeEnvironment,
            "/vlm/analyze_environment",
            callback_group=self._cb_group,
        )
        self.create_timer(0.1, self._tick, callback_group=self._cb_group)
        self.create_timer(1.0, self._publish_status, callback_group=self._cb_group)
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "mapping-session")
        self.declare_parameter("batch_timeout_s", 5.0)
        self.declare_parameter("action_timeout_s", 60.0)
        self.declare_parameter("maximum_partial_batches", 4)

    def _on_geometry(self, message: GeometricObjectObservation) -> None:
        with self._lock:
            if not self._accepting:
                return
            try:
                self._assembler.add_geometry(message)
            except SemanticBatchError as exc:
                self._reject_locked(f"geometry rejected: {exc}")

    def _on_evidence(self, message: VlmObservation) -> None:
        with self._lock:
            if not self._accepting:
                return
            try:
                self._assembler.add_evidence(message)
            except SemanticBatchError as exc:
                self._reject_locked(f"evidence rejected: {exc}")

    def _on_observation_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            drained = payload.get("active") is False and payload.get("pending") is False
        except (TypeError, json.JSONDecodeError):
            drained = False
        with self._lock:
            self._observation_drained = drained

    def _on_exploration_status(self, message: ExplorationStatus) -> None:
        with self._lock:
            if message.state == ExplorationStatus.FINALIZING:
                self._finalizing = True
                self._last_reason = "finalizing after observation and VLM drain"
            elif message.state in {
                ExplorationStatus.FAILED,
                ExplorationStatus.COMPLETED,
            }:
                self._accepting = False
                if self._active_goal is not None:
                    self._active_goal.cancel_goal_async()
                self._last_reason = "semantic processing stopped"

    def _tick(self) -> None:
        batch_to_start = None
        goal_to_cancel = None
        with self._lock:
            self._expired += self._assembler.expire(self._batch_timeout_s)
            for batch in self._assembler.pop_ready():
                outcome = self._queue.submit(batch, 1)
                if outcome.replaced is not None:
                    self._replaced += 1
                self._submitted += 1
            if (
                self._active_batch is None
                and self._client.server_is_ready()
            ):
                batch_to_start = self._queue.begin_next()
                if batch_to_start is not None:
                    self._active_batch = batch_to_start
                    self._active_started = time.monotonic()
            if (
                self._active_batch is not None
                and self._active_started > 0.0
                and time.monotonic() - self._active_started > self._action_timeout_s
            ):
                goal_to_cancel = self._active_goal
                self._last_reason = "environment VLM action timed out"
            if self._drained_locked():
                self._accepting = False
        if goal_to_cancel is not None:
            goal_to_cancel.cancel_goal_async()
        if batch_to_start is not None:
            self._send_batch(batch_to_start)

    def _send_batch(self, batch: SemanticObservationBatch) -> None:
        goal = AnalyzeEnvironment.Goal()
        goal.observation_id = batch.observation_id
        goal.stamp = batch.evidence.observation_stamp
        goal.image = copy.deepcopy(batch.evidence.image)
        goal.detections = [copy.deepcopy(item) for item in batch.evidence.detections]
        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, expected=batch: self._on_goal(expected, completed)
            )
        except Exception as exc:
            self._finish_batch(batch, f"VLM goal send failed: {exc}", rejected=True)

    def _on_goal(self, batch: SemanticObservationBatch, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("environment VLM goal was rejected")
            with self._lock:
                if self._active_batch is not batch:
                    goal_handle.cancel_goal_async()
                    return
                self._active_goal = goal_handle
            goal_handle.get_result_async().add_done_callback(
                lambda completed, expected=batch: self._on_result(
                    expected, completed
                )
            )
        except Exception as exc:
            self._finish_batch(batch, f"VLM goal failed: {exc}", rejected=True)

    def _on_result(self, batch: SemanticObservationBatch, future) -> None:
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"environment VLM action status {response.status}")
            if not response.result.success:
                raise RuntimeError(response.result.error_message or "semantic analysis failed")
            joined = join_semantics(batch, response.result.objects)
            for geometry, semantic in joined:
                self._localized_pub.publish(
                    self._localized_message(
                        batch, geometry, semantic, response.result.scene
                    )
                )
            with self._lock:
                self._published += len(joined)
            self._finish_batch(
                batch,
                f"published {len(joined)} useful semantic objects",
                rejected=False,
            )
        except Exception as exc:
            self._finish_batch(batch, f"semantic result rejected: {exc}", rejected=True)

    def _localized_message(self, batch, geometry, semantic, scene):
        if geometry.observation_id != batch.observation_id:
            raise SemanticBatchError("geometry observation_id changed before publish")
        message = LocalizedObjectObservation()
        message.environment_id = self._environment_id
        message.map_id = self._map_id
        message.observation_id = batch.observation_id
        message.observation_stamp = geometry.observation_stamp
        message.depth_stamp = geometry.depth_stamp
        message.semantic = copy.deepcopy(semantic)
        message.detection = copy.deepcopy(geometry.detection)
        message.map_position = copy.deepcopy(geometry.map_position)
        message.robot_pose = copy.deepcopy(geometry.robot_pose)
        message.scene = scene
        message.localization_quality = geometry.localization_quality
        message.image_ref = ""
        message.image = copy.deepcopy(batch.evidence.image)
        return message

    def _finish_batch(
        self, batch: SemanticObservationBatch, reason: str, rejected: bool
    ) -> None:
        with self._lock:
            if self._active_batch is not batch:
                return
            self._queue.complete()
            self._active_batch = None
            self._active_goal = None
            self._active_started = 0.0
            self._completed += 1
            if rejected:
                self._rejected += 1
            self._last_reason = reason

    def _drained_locked(self) -> bool:
        return (
            self._finalizing
            and self._observation_drained
            and self._assembler.pending_count == 0
            and self._active_batch is None
            and not self._queue.active
            and not self._queue.has_pending
        )

    def _publish_status(self) -> None:
        with self._lock:
            payload = {
                "ready": self._client.server_is_ready() and self._accepting,
                "accepting": self._accepting,
                "active": self._active_batch is not None,
                "pending": self._queue.has_pending,
                "partial_batches": self._assembler.pending_count,
                "observation_drained": self._observation_drained,
                "drained": self._drained_locked(),
                "submitted": self._submitted,
                "replaced": self._replaced,
                "completed": self._completed,
                "published": self._published,
                "rejected": self._rejected,
                "expired": self._expired,
                "last_reason": self._last_reason,
            }
        self._status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )

    def _reject_locked(self, reason: str) -> None:
        self._rejected += 1
        self._last_reason = reason

    def _safe_id(self, name: str) -> str:
        value = self._string_parameter(name)
        if not re.fullmatch(r"[A-Za-z0-9_-]+", value):
            raise ValueError(f"{name} may contain letters, numbers, _ and -")
        return value

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[SemanticObservationManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = SemanticObservationManager()
        executor = MultiThreadedExecutor(num_threads=4)
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
