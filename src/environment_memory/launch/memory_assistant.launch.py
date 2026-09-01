"""Public Version 1 entry point for read-only memory assistance."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from environment_memory.storage.readonly_memory import load_completed_manifest


def _assistant_actions(context):
    environment_id = LaunchConfiguration("environment_id").perform(context).strip()
    requested_map_id = LaunchConfiguration("map_id").perform(context).strip()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", environment_id):
        raise RuntimeError("environment_id may contain letters, numbers, _ and -")
    if requested_map_id and not re.fullmatch(r"[A-Za-z0-9_-]+", requested_map_id):
        raise RuntimeError("map_id may contain letters, numbers, _ and -")
    storage_value = LaunchConfiguration("storage_root").perform(context).strip()
    storage_root = (
        Path(storage_value).expanduser()
        if storage_value
        else Path.home() / ".local" / "share" / "flzat" / "environment_memory"
    )
    manifest = load_completed_manifest(
        storage_root / environment_id,
        environment_id,
        requested_map_id,
    )

    navigation_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )
    memory_share = Path(get_package_share_directory("environment_memory"))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "all_in_one.launch.py")
        ),
        launch_arguments={
            "slam": "false",
            "map": str(manifest.map_yaml),
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "transport_partition": LaunchConfiguration("transport_partition"),
        }.items(),
    )
    assistant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(memory_share / "launch" / "assistant_runtime.launch.py")
        ),
        launch_arguments={
            "environment_id": manifest.environment_id,
            "map_id": manifest.map_id,
            "storage_root": str(storage_root),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "embedding_device": LaunchConfiguration("embedding_device"),
            "embedding_local_files_only": LaunchConfiguration(
                "embedding_local_files_only"
            ),
            "vad_silence_ms": LaunchConfiguration("vad_silence_ms"),
            "whisper_language": LaunchConfiguration("whisper_language"),
            "enable_shared_vlm": "true",
            "vlm_backend": LaunchConfiguration("vlm_backend"),
            "vlm_model_id": LaunchConfiguration("vlm_model_id"),
            "vlm_device": LaunchConfiguration("vlm_device"),
            "vlm_local_files_only": LaunchConfiguration("vlm_local_files_only"),
        }.items(),
    )
    return [
        LogInfo(
            msg=(
                "Memory-assistant mode: verified completed manifest "
                f"environment={manifest.environment_id}, map={manifest.map_id}; "
                "starting saved-map localization and read-only memory."
            )
        ),
        navigation,
        assistant,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("environment_id", default_value="hotel_demo"),
            DeclareLaunchArgument(
                "map_id",
                default_value="",
                description=(
                    "Optional expected mapping-session ID. Empty uses the completed "
                    "manifest value."
                ),
            ),
            DeclareLaunchArgument("storage_root", default_value=""),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("transport_partition", default_value=""),
            DeclareLaunchArgument("embedding_device", default_value="cpu"),
            DeclareLaunchArgument(
                "embedding_local_files_only", default_value="false"
            ),
            DeclareLaunchArgument("vad_silence_ms", default_value="500"),
            DeclareLaunchArgument("whisper_language", default_value="en"),
            DeclareLaunchArgument("vlm_backend", default_value="smolvlm2"),
            DeclareLaunchArgument(
                "vlm_model_id",
                default_value="HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
            ),
            DeclareLaunchArgument("vlm_device", default_value="auto"),
            DeclareLaunchArgument("vlm_local_files_only", default_value="false"),
            OpaqueFunction(function=_assistant_actions),
        ]
    )
