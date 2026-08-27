"""Public Version 1 entry point for autonomous map and memory building."""

from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    memory_share = Path(get_package_share_directory("environment_memory"))
    build_runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(memory_share / "launch" / "exploration_observation.launch.py")
        ),
        launch_arguments={
            "environment_id": LaunchConfiguration("environment_id"),
            "map_id": LaunchConfiguration("map_id"),
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "transport_partition": LaunchConfiguration("transport_partition"),
            "map_output_path": LaunchConfiguration("map_output_path"),
            "storage_root": LaunchConfiguration("storage_root"),
            "embedding_device": LaunchConfiguration("embedding_device"),
            "embedding_local_files_only": LaunchConfiguration(
                "embedding_local_files_only"
            ),
            "frontier_log_level": LaunchConfiguration("frontier_log_level"),
            "readiness_timeout_s": LaunchConfiguration("readiness_timeout_s"),
            "finalization_timeout_s": LaunchConfiguration(
                "finalization_timeout_s"
            ),
            "semantic_action_timeout_s": LaunchConfiguration(
                "semantic_action_timeout_s"
            ),
            "detector_config": LaunchConfiguration("detector_config"),
            "enable_vlm": "true",
            "vlm_backend": LaunchConfiguration("vlm_backend"),
            "vlm_model_id": LaunchConfiguration("vlm_model_id"),
            "vlm_device": LaunchConfiguration("vlm_device"),
            "vlm_local_files_only": LaunchConfiguration("vlm_local_files_only"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("environment_id", default_value="hotel_demo"),
            DeclareLaunchArgument(
                "map_id",
                default_value=str(uuid.uuid4()),
                description=(
                    "Unique mapping-session ID. Reuse it only when resuming the "
                    "same incomplete build."
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("transport_partition", default_value=""),
            DeclareLaunchArgument("map_output_path", default_value=""),
            DeclareLaunchArgument("storage_root", default_value=""),
            DeclareLaunchArgument("embedding_device", default_value="cpu"),
            DeclareLaunchArgument(
                "embedding_local_files_only", default_value="false"
            ),
            DeclareLaunchArgument("frontier_log_level", default_value="info"),
            DeclareLaunchArgument("readiness_timeout_s", default_value="300.0"),
            DeclareLaunchArgument("finalization_timeout_s", default_value="75.0"),
            DeclareLaunchArgument("semantic_action_timeout_s", default_value="60.0"),
            DeclareLaunchArgument(
                "detector_config",
                default_value=str(memory_share / "config" / "yolov8n_geometry.yaml"),
            ),
            DeclareLaunchArgument("vlm_backend", default_value="smolvlm2"),
            DeclareLaunchArgument(
                "vlm_model_id",
                default_value="HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
            ),
            DeclareLaunchArgument("vlm_device", default_value="auto"),
            DeclareLaunchArgument("vlm_local_files_only", default_value="false"),
            LogInfo(
                msg=(
                    "Autonomous memory-build mode: SLAM, Nav2, frontier "
                    "exploration, frozen RGB-D observations, shared VLM analysis, "
                    "and writable persistent memory. No speech services are started."
                )
            ),
            build_runtime,
        ]
    )
