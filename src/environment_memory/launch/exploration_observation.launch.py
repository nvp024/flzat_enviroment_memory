"""Phase 7 integration foundation including the writable memory owner."""

from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from environment_memory.exploration.nav2_speed_profile import (
    create_mode1_nav2_params,
)


def _simulator_navigation(context, navigation_share, mode1_nav2_params):
    simulator = LaunchConfiguration("simulator").perform(context).strip().lower()
    common_arguments = {
        "headless": LaunchConfiguration("headless"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "autostart": "true",
        "params_file": str(mode1_nav2_params),
    }
    if simulator == "gazebo":
        return [
            LogInfo(msg="Mode 1 simulator: Gazebo Harmonic"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(navigation_share / "launch" / "all_in_one.launch.py")
                ),
                launch_arguments={
                    **common_arguments,
                    "slam": "true",
                    "map": "",
                    "transport_partition": LaunchConfiguration(
                        "transport_partition"
                    ),
                }.items(),
            ),
        ]
    if simulator == "isaac":
        isaac_share = Path(
            get_package_share_directory("openarm_skeleton_v1_2_isaac")
        )
        return [
            LogInfo(
                msg=(
                    "Mode 1 simulator: Isaac Sim. The current Isaac package "
                    "provides SLAM/Nav2 sensors but not RGB-D topics; semantic "
                    "observations require the future Isaac RGB-D bridge."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(isaac_share / "launch" / "isaac_nav2.launch.py")
                ),
                launch_arguments={
                    **common_arguments,
                    "start_isaac": LaunchConfiguration("start_isaac"),
                    "scene": LaunchConfiguration("scene"),
                    "isaac_sim_path": LaunchConfiguration("isaac_sim_path"),
                    "startup_timeout": LaunchConfiguration("startup_timeout"),
                    "lidar_config": LaunchConfiguration("lidar_config"),
                    "max_frames": LaunchConfiguration("max_frames"),
                    "slam": "true",
                    "map": "",
                }.items(),
            ),
        ]
    raise RuntimeError(
        f"Unknown simulator {simulator!r}; use simulator:=gazebo or "
        "simulator:=isaac"
    )


def generate_launch_description():
    memory_share = Path(get_package_share_directory("environment_memory"))
    navigation_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )
    frontier_share = Path(get_package_share_directory("frontier_exploration_ros2"))
    vlm_share = Path(get_package_share_directory("vlm_pipeline"))
    mode1_nav2_params = create_mode1_nav2_params(
        navigation_share / "config" / "nav2_params.yaml"
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
            "model_revision": LaunchConfiguration("vlm_model_revision"),
            "device": LaunchConfiguration("vlm_device"),
            "local_files_only": LaunchConfiguration("vlm_local_files_only"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("environment_id", default_value="hotel_demo"),
            DeclareLaunchArgument("map_id", default_value=str(uuid.uuid4())),
            DeclareLaunchArgument(
                "simulator",
                default_value="gazebo",
                description="Simulation backend: gazebo or isaac",
            ),
            DeclareLaunchArgument("scene", default_value="hotel"),
            DeclareLaunchArgument("start_isaac", default_value="true"),
            DeclareLaunchArgument(
                "isaac_sim_path",
                default_value=EnvironmentVariable(
                    "ISAAC_SIM_PATH", default_value=""
                ),
            ),
            DeclareLaunchArgument("startup_timeout", default_value="600.0"),
            DeclareLaunchArgument(
                "lidar_config", default_value="Example_Rotary_2D"
            ),
            DeclareLaunchArgument("max_frames", default_value="0"),
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
            DeclareLaunchArgument("grounding_action_timeout_s", default_value="60.0"),
            DeclareLaunchArgument("initial_observation_settle_s", default_value="1.0"),
            DeclareLaunchArgument(
                "observation_config",
                default_value=str(
                    memory_share / "config" / "vlm_grounding_geometry.yaml"
                ),
                description="VLM grounding and RGB-D geometry parameters.",
            ),
            DeclareLaunchArgument(
                "enable_vlm",
                default_value="true",
                description=(
                    "Load the shared VLM server. The Phase 7 Memory Manager only "
                    "consumes completed localized semantic observations."
                ),
            ),
            DeclareLaunchArgument("vlm_backend", default_value="qwen3_vl"),
            DeclareLaunchArgument(
                "vlm_model_id",
                default_value="Qwen/Qwen3-VL-2B-Instruct",
            ),
            DeclareLaunchArgument("vlm_model_revision", default_value="main"),
            DeclareLaunchArgument("vlm_device", default_value="auto"),
            DeclareLaunchArgument("vlm_local_files_only", default_value="false"),
            LogInfo(
                msg=(
                    "Starting the Phase 7 integration foundation: exploration, "
                    "triggered geometry, and the writable persistent Memory Manager. "
                    "No retrieval or assistant behavior is launched."
                )
            ),
            LogInfo(msg=f"Mode 1 slow Nav2 parameters: {mode1_nav2_params}"),
            OpaqueFunction(
                function=_simulator_navigation,
                args=[navigation_share, mode1_nav2_params],
            ),
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
                    LaunchConfiguration("observation_config"),
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
                        "grounding_action_timeout_s": ParameterValue(
                            LaunchConfiguration("grounding_action_timeout_s"),
                            value_type=float,
                        ),
                        "initial_settle_s": ParameterValue(
                            LaunchConfiguration("initial_observation_settle_s"),
                            value_type=float,
                        ),
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
            vlm,
        ]
    )
