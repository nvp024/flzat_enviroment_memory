"""Internal Phase 8 assistant services; Phase 9 owns the public scenario launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    audio_share = Path(get_package_share_directory("audio_pipeline"))
    vlm_share = Path(get_package_share_directory("vlm_pipeline"))
    speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(audio_share / "launch" / "speech_services.launch.py")
        ),
        launch_arguments={
            "vad_silence_ms": LaunchConfiguration("vad_silence_ms"),
            "whisper_language": LaunchConfiguration("whisper_language"),
        }.items(),
    )
    shared_vlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vlm_share / "launch" / "vlm_server.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_shared_vlm")),
        launch_arguments={
            "backend": LaunchConfiguration("vlm_backend"),
            "model_id": LaunchConfiguration("vlm_model_id"),
            "device": LaunchConfiguration("vlm_device"),
            "local_files_only": LaunchConfiguration("vlm_local_files_only"),
        }.items(),
    )
    common = {
        "environment_id": ParameterValue(
            LaunchConfiguration("environment_id"), value_type=str
        ),
        "map_id": ParameterValue(LaunchConfiguration("map_id"), value_type=str),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("environment_id", default_value="hotel_demo"),
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("storage_root", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("embedding_device", default_value="cpu"),
            DeclareLaunchArgument(
                "embedding_local_files_only", default_value="false"
            ),
            DeclareLaunchArgument("vad_silence_ms", default_value="500"),
            DeclareLaunchArgument("whisper_language", default_value="en"),
            DeclareLaunchArgument("enable_shared_vlm", default_value="true"),
            DeclareLaunchArgument("vlm_backend", default_value="smolvlm2"),
            DeclareLaunchArgument(
                "vlm_model_id",
                default_value="HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
            ),
            DeclareLaunchArgument("vlm_device", default_value="auto"),
            DeclareLaunchArgument("vlm_local_files_only", default_value="false"),
            speech,
            shared_vlm,
            Node(
                package="environment_memory",
                executable="memory_query_server",
                name="memory_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        **common,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "storage_root": ParameterValue(
                            LaunchConfiguration("storage_root"), value_type=str
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
                executable="memory_command_manager",
                name="memory_command_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        **common,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="environment_memory",
                executable="memory_marker_publisher",
                name="memory_marker_publisher",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        **common,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "storage_root": ParameterValue(
                            LaunchConfiguration("storage_root"), value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
