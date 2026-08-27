"""Phase 7 integration foundation including the writable memory owner."""

from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    memory_share = Path(get_package_share_directory("environment_memory"))
    navigation_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )
    frontier_share = Path(get_package_share_directory("frontier_exploration_ros2"))
    vlm_share = Path(get_package_share_directory("vlm_pipeline"))

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "all_in_one.launch.py")
        ),
        launch_arguments={
            "slam": "true",
            "map": "",
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "transport_partition": LaunchConfiguration("transport_partition"),
        }.items(),
    )
    frontier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(frontier_share / "launch" / "frontier_explorer.launch.py")
        ),
        launch_arguments={
            "params_file": str(memory_share / "config" / "frontier_openarm.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "false",
            "control_service_enabled": "true",
            "log_level": LaunchConfiguration("frontier_log_level"),
            "map_qos_durability": "transient_local",
            "costmap_qos_reliability": "reliable",
        }.items(),
    )
    vlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vlm_share / "launch" / "vlm_server.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_vlm")),
        launch_arguments={
            "backend": LaunchConfiguration("vlm_backend"),
            "model_id": LaunchConfiguration("vlm_model_id"),
            "device": LaunchConfiguration("vlm_device"),
            "local_files_only": LaunchConfiguration("vlm_local_files_only"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("environment_id", default_value="hotel_demo"),
            DeclareLaunchArgument("map_id", default_value=str(uuid.uuid4())),
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
                description="YOLO model asset and RGB-D geometry parameters.",
            ),
            DeclareLaunchArgument(
                "enable_vlm",
                default_value="true",
                description=(
                    "Load the shared VLM server. The Phase 7 Memory Manager only "
                    "consumes completed localized semantic observations."
                ),
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
                    "Starting the Phase 7 integration foundation: exploration, "
                    "triggered geometry, and the writable persistent Memory Manager. "
                    "No retrieval or assistant behavior is launched."
                )
            ),
            navigation,
            frontier,
            Node(
                package="environment_memory",
                executable="memory_build_manager",
                name="memory_build_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "environment_id": ParameterValue(
                            LaunchConfiguration("environment_id"), value_type=str
                        ),
                        "map_output_path": ParameterValue(
                            LaunchConfiguration("map_output_path"), value_type=str
                        ),
                        "storage_root": ParameterValue(
                            LaunchConfiguration("storage_root"), value_type=str
                        ),
                        "readiness_timeout_s": ParameterValue(
                            LaunchConfiguration("readiness_timeout_s"),
                            value_type=float,
                        ),
                        "finalization_timeout_s": ParameterValue(
                            LaunchConfiguration("finalization_timeout_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="environment_memory",
                executable="observation_manager",
                name="observation_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("detector_config"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    }
                ],
            ),
            Node(
                package="environment_memory",
                executable="memory_manager",
                name="memory_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "environment_id": ParameterValue(
                            LaunchConfiguration("environment_id"), value_type=str
                        ),
                        "map_id": ParameterValue(
                            LaunchConfiguration("map_id"), value_type=str
                        ),
                        "storage_root": ParameterValue(
                            LaunchConfiguration("storage_root"), value_type=str
                        ),
                        "map_output_path": ParameterValue(
                            LaunchConfiguration("map_output_path"), value_type=str
                        ),
                        "embedding_device": ParameterValue(
                            LaunchConfiguration("embedding_device"), value_type=str
                        ),
                        "embedding_local_files_only": ParameterValue(
                            LaunchConfiguration("embedding_local_files_only"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="environment_memory",
                executable="semantic_observation_manager",
                name="semantic_observation_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "environment_id": ParameterValue(
                            LaunchConfiguration("environment_id"), value_type=str
                        ),
                        "map_id": ParameterValue(
                            LaunchConfiguration("map_id"), value_type=str
                        ),
                        "action_timeout_s": ParameterValue(
                            LaunchConfiguration("semantic_action_timeout_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            vlm,
        ]
    )
