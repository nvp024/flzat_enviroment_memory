from __future__ import annotations

import math
import threading
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from environment_memory.approach_pose import (
    OccupancyGrid2D,
    Pose2D,
    generate_approach_candidates,
)
from environment_memory.command_intent import (
    CommandError,
    CommandMatch,
    Intent,
    ambiguous_navigation_matches,
    clarification_prompt,
    format_query_answer,
    parse_command,
    resolve_clarification,
)
from environment_memory_interfaces.srv import QueryMemory
from robot_interfaces.action import SpeechToText, TextToSpeech
from robot_interfaces.msg import SpeechAudio


class MemoryCommandManager(Node):
    """Voice-command state machine; only this node may dispatch assistant Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("memory_command_manager")
        self._declare_parameters()
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._clarification: tuple[CommandMatch, ...] = ()
        self._static_map: OccupancyGrid2D | None = None
        self._costmap: OccupancyGrid2D | None = None
        self._path_candidates: list[Pose2D] = []
        self._selected_object: CommandMatch | None = None
        self._active_navigation_goal = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SpeechAudio,
            "/audio_events",
            self._on_audio,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            map_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._on_costmap,
            map_qos,
            callback_group=self._cb_group,
        )
        self._stt = ActionClient(
            self,
            SpeechToText,
            "/stt_action",
            callback_group=self._cb_group,
        )
        self._tts = ActionClient(
            self,
            TextToSpeech,
            "/tts_action",
            callback_group=self._cb_group,
        )
        self._query = self.create_client(
            QueryMemory,
            "/environment_memory/query",
            callback_group=self._cb_group,
        )
        self._planner = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            callback_group=self._cb_group,
        )
        self._navigator = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self._cb_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            "Memory command manager ready; waiting for VAD audio events."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("environment_id", "hotel_demo")
        self.declare_parameter("map_id", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("ambiguity_score_margin", 0.05)
        self.declare_parameter("static_occupied_threshold", 50)
        self.declare_parameter("inflated_occupied_threshold", 1)
        self.declare_parameter("approach_samples_per_radius", 16)

    def _on_audio(self, message: SpeechAudio) -> None:
        with self._lock:
            if self._busy:
                self.get_logger().warn("Ignoring audio while another command is active.")
                return
            self._busy = True
        if not self._stt.server_is_ready():
            self._speak_and_release("Speech recognition is not available.")
            return
        goal = SpeechToText.Goal()
        goal.audio_packet = message
        try:
            self._stt.send_goal_async(goal).add_done_callback(self._on_stt_goal)
        except Exception as exc:
            self._speak_and_release(f"I could not start speech recognition. {exc}")

    def _on_stt_goal(self, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("speech recognition goal was rejected")
            goal_handle.get_result_async().add_done_callback(self._on_stt_result)
        except Exception as exc:
            self._speak_and_release(f"I could not recognize that command. {exc}")

    def _on_stt_result(self, future) -> None:
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"speech recognition status {result.status}")
            transcript = result.result.transcript.strip()
            if not transcript:
                raise CommandError("speech transcript is empty")
            self.get_logger().info(f"Memory command transcript: {transcript!r}")
            if self._clarification:
                selected = resolve_clarification(transcript, self._clarification)
                if selected is None:
                    prompt = clarification_prompt(self._clarification)
                    self._speak_and_release("I did not understand the choice. " + prompt)
                    return
                self._clarification = ()
                self._begin_navigation(selected)
                return
            command = parse_command(transcript)
            self._request_memory(command.intent, command.query_text)
        except Exception as exc:
            self._speak_and_release(f"I could not understand that command. {exc}")

    def _request_memory(self, intent: Intent, query_text: str) -> None:
        if not self._query.service_is_ready():
            self._speak_and_release("Environment memory is not available.")
            return
        request = QueryMemory.Request()
        request.query = query_text
        request.top_k = 5
        request.environment_id = self._string_parameter("environment_id")
        request.map_id = self._string_parameter("map_id")
        try:
            future = self._query.call_async(request)
            future.add_done_callback(
                lambda completed, requested_intent=intent: self._on_query_result(
                    requested_intent, completed
                )
            )
        except Exception as exc:
            self._speak_and_release(f"The memory query could not be sent. {exc}")

    def _on_query_result(self, intent: Intent, future) -> None:
        try:
            response = future.result()
            if response is None or not response.success:
                error = "no response" if response is None else response.error_message
                raise RuntimeError(error)
            if len(response.objects) != len(response.cosine_scores):
                raise RuntimeError("memory service returned inconsistent results")
            map_frame = self._string_parameter("map_frame")
            if any(
                item.map_position.header.frame_id != map_frame
                or not all(
                    math.isfinite(value)
                    for value in (
                        item.map_position.point.x,
                        item.map_position.point.y,
                        item.map_position.point.z,
                    )
                )
                for item in response.objects
            ):
                raise RuntimeError("memory service returned an invalid map position")
            matches = tuple(
                CommandMatch(
                    object_id=item.object_id,
                    label=item.label,
                    description=item.description,
                    scene=item.scene,
                    score=float(score),
                    x=float(item.map_position.point.x),
                    y=float(item.map_position.point.y),
                    z=float(item.map_position.point.z),
                )
                for item, score in zip(response.objects, response.cosine_scores)
            )
            if intent == Intent.QUERY_MEMORY:
                self._speak_and_release(format_query_answer(matches))
                return
            selected, ambiguous = ambiguous_navigation_matches(
                matches,
                self._float_parameter("ambiguity_score_margin"),
            )
            if ambiguous:
                self._clarification = ambiguous[:3]
                self._speak_and_release(clarification_prompt(self._clarification))
                return
            if selected is None:
                self._speak_and_release(
                    "I could not find that object, so I will not move."
                )
                return
            self._begin_navigation(selected)
        except Exception as exc:
            self._speak_and_release(f"The memory query failed. {exc}")

    def _begin_navigation(self, selected: CommandMatch) -> None:
        if self._static_map is None or self._costmap is None:
            self._speak_and_release(
                "Navigation maps are not ready, so I will not move."
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._string_parameter("map_frame"),
                self._string_parameter("base_frame"),
                Time(),
            )
            translation = transform.transform.translation
            robot = Pose2D(float(translation.x), float(translation.y), 0.0)
            candidates = generate_approach_candidates(
                selected.x,
                selected.y,
                robot,
                self._static_map,
                self._costmap,
                samples_per_radius=self._integer_parameter(
                    "approach_samples_per_radius"
                ),
            )
        except (TransformException, ValueError) as exc:
            self._speak_and_release(
                f"I cannot calculate a safe approach position. {exc}"
            )
            return
        if not candidates:
            self._speak_and_release(
                "There is no free approach position near that object, so I will not move."
            )
            return
        self._selected_object = selected
        self._path_candidates = candidates
        self._try_next_candidate()

    def _try_next_candidate(self) -> None:
        if not self._path_candidates:
            self._speak_and_release(
                "Nav2 could not find a safe path near that object, so I will not move."
            )
            return
        if not self._planner.server_is_ready():
            self._speak_and_release("The Nav2 path planner is not available.")
            return
        candidate = self._path_candidates.pop(0)
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_message(candidate)
        goal.use_start = False
        try:
            self._planner.send_goal_async(goal).add_done_callback(
                lambda completed, pose=candidate: self._on_planner_goal(pose, completed)
            )
        except Exception as exc:
            self._speak_and_release(f"Nav2 path validation failed. {exc}")

    def _on_planner_goal(self, candidate: Pose2D, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._try_next_candidate()
                return
            goal_handle.get_result_async().add_done_callback(
                lambda completed, pose=candidate: self._on_planner_result(
                    pose, completed
                )
            )
        except Exception:
            self._try_next_candidate()

    def _on_planner_result(self, candidate: Pose2D, future) -> None:
        try:
            result = future.result()
            valid = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result is not None
                and bool(result.result.path.poses)
            )
        except Exception:
            valid = False
        if not valid:
            self._try_next_candidate()
            return
        self._send_navigation(candidate)

    def _send_navigation(self, candidate: Pose2D) -> None:
        if not self._navigator.server_is_ready():
            self._speak_and_release("The Nav2 navigator is not available.")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_message(candidate)
        try:
            self._navigator.send_goal_async(goal).add_done_callback(
                self._on_navigation_goal
            )
        except Exception as exc:
            self._speak_and_release(f"The navigation goal could not be sent. {exc}")

    def _on_navigation_goal(self, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("Nav2 rejected the approach goal")
            self._active_navigation_goal = goal_handle
            goal_handle.get_result_async().add_done_callback(self._on_navigation_result)
        except Exception as exc:
            self._speak_and_release(f"Navigation did not start. {exc}")

    def _on_navigation_result(self, future) -> None:
        selected = self._selected_object
        label = "the remembered object" if selected is None else selected.label.replace("_", " ")
        self._active_navigation_goal = None
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                speech = f"I reached a safe approach position near {label}."
            elif result.status == GoalStatus.STATUS_CANCELED:
                speech = f"Navigation to {label} was cancelled."
            else:
                speech = f"I could not reach {label}. Nav2 reported a failure."
        except Exception as exc:
            speech = f"Navigation to {label} failed. {exc}"
        self._speak_and_release(speech)

    def _pose_message(self, pose: Pose2D) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._string_parameter("map_frame")
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.position.z = 0.0
        message.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(pose.yaw / 2.0),
            w=math.cos(pose.yaw / 2.0),
        )
        return message

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            if message.header.frame_id != self._string_parameter("map_frame"):
                raise ValueError("static map must use the configured map frame")
            self._static_map = self._grid_from_message(
                message, self._integer_parameter("static_occupied_threshold")
            )
        except ValueError as exc:
            self.get_logger().warn(f"Rejected static map: {exc}")

    def _on_costmap(self, message: OccupancyGrid) -> None:
        try:
            if message.header.frame_id != self._string_parameter("map_frame"):
                raise ValueError("global costmap must use the configured map frame")
            self._costmap = self._grid_from_message(
                message, self._integer_parameter("inflated_occupied_threshold")
            )
        except ValueError as exc:
            self.get_logger().warn(f"Rejected global costmap: {exc}")

    @staticmethod
    def _grid_from_message(
        message: OccupancyGrid, occupied_threshold: int
    ) -> OccupancyGrid2D:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        grid = OccupancyGrid2D(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=yaw,
            data=tuple(int(value) for value in message.data),
            occupied_threshold=occupied_threshold,
            reject_unknown=True,
        )
        # is_free validates bounds lazily; dimensions are validated by generation.
        return grid

    def _speak_and_release(self, text: str) -> None:
        self._speak(text, self._release_command)

    def _speak(self, text: str, done: Callable[[], None]) -> None:
        self.get_logger().info(f"Assistant response: {text}")
        if not self._tts.server_is_ready():
            self.get_logger().error("TTS action is unavailable; response was logged only.")
            done()
            return
        goal = TextToSpeech.Goal()
        goal.text = text
        try:
            future = self._tts.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, callback=done: self._on_tts_goal(
                    callback, completed
                )
            )
        except Exception:
            done()

    def _on_tts_goal(self, done: Callable[[], None], future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                done()
                return
            goal_handle.get_result_async().add_done_callback(
                lambda _completed, callback=done: callback()
            )
        except Exception:
            done()

    def _release_command(self) -> None:
        self._selected_object = None
        self._path_candidates = []
        with self._lock:
            self._busy = False

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[MemoryCommandManager] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = MemoryCommandManager()
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
