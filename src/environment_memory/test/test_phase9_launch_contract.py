from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / "launch"


def launch_source(name):
    return (LAUNCH_ROOT / name).read_text(encoding="utf-8")


def test_exactly_two_documented_public_scenario_launches_exist():
    expected = {
        "autonomous_memory_build.launch.py",
        "memory_assistant.launch.py",
    }
    assert all((LAUNCH_ROOT / name).is_file() for name in expected)

    readme = (PACKAGE_ROOT.parents[1] / "README.md").read_text(encoding="utf-8")
    assert "exactly two public scenario entry points" in readme
    assert all(name in readme for name in expected)


def test_autonomous_build_is_slam_vlm_and_write_mode_without_speech():
    source = launch_source("autonomous_memory_build.launch.py")
    internal = launch_source("exploration_observation.launch.py")
    combined = source + internal

    assert "exploration_observation.launch.py" in source
    assert '"enable_vlm": "true"' in source
    assert '"slam": "true"' in internal
    assert 'executable="memory_manager"' in internal
    assert 'executable="observation_manager"' in internal
    assert "frontier_explorer.launch.py" in internal
    assert "speech_services.launch.py" not in combined
    assert "voice_pipeline.launch.py" not in combined
    assert "audio_loopback_node" not in combined
    assert "memory_command_manager" not in combined


def test_assistant_binds_completed_manifest_to_amcl_and_read_only_runtime():
    source = launch_source("memory_assistant.launch.py")
    runtime = launch_source("assistant_runtime.launch.py")

    assert "load_completed_manifest" in source
    assert '"slam": "false"' in source
    assert '"map": str(manifest.map_yaml)' in source
    assert "assistant_runtime.launch.py" in source
    assert 'executable="memory_query_server"' in runtime
    assert 'executable="memory_command_manager"' in runtime
    assert "speech_services.launch.py" in runtime
    assert "vlm_server.launch.py" in runtime
    combined = source + runtime
    assert "frontier_explorer.launch.py" not in combined
    assert "observation_manager" not in combined
    assert 'executable="memory_manager"' not in combined
    assert "voice_pipeline.launch.py" not in combined
    assert "audio_loopback_node" not in combined


def test_assistant_can_disable_speech_without_disabling_text_commands():
    source = launch_source("memory_assistant.launch.py")
    runtime = launch_source("assistant_runtime.launch.py")

    assert '"enable_speech": LaunchConfiguration("enable_speech")' in source
    assert re.search(
        r'DeclareLaunchArgument\(\s*"enable_speech",\s*default_value="true"',
        source,
    )
    assert 'condition=IfCondition(LaunchConfiguration("enable_speech"))' in runtime
    assert 'DeclareLaunchArgument("enable_speech", default_value="true")' in runtime
    assert 'executable="memory_command_manager"' in runtime
    assert 'executable="memory_query_server"' in runtime


def test_both_public_modes_select_gazebo_by_default_or_isaac_explicitly():
    autonomous = launch_source("autonomous_memory_build.launch.py")
    exploration = launch_source("exploration_observation.launch.py")
    assistant = launch_source("memory_assistant.launch.py")

    assert re.search(
        r'DeclareLaunchArgument\(\s*"simulator",\s*default_value="gazebo"',
        autonomous,
    )
    assert '"simulator": LaunchConfiguration("simulator")' in autonomous
    for source in (exploration, assistant):
        assert 'simulator == "gazebo"' in source
        assert 'simulator == "isaac"' in source
        assert "all_in_one.launch.py" in source
        assert "openarm_skeleton_v1_2_isaac" in source
        assert "isaac_nav2.launch.py" in source
        assert '"scene": LaunchConfiguration("scene")' in source
        assert '"start_isaac": LaunchConfiguration("start_isaac")' in source
        assert '"isaac_sim_path": LaunchConfiguration("isaac_sim_path")' in source
        assert "use simulator:=gazebo or " in source
        assert '"simulator",\n                default_value="gazebo"' in source

    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>openarm_skeleton_v1_2_isaac</exec_depend>" in package_xml


def test_mode1_and_mode2_pass_the_correct_mapping_mode_to_each_simulator():
    exploration = launch_source("exploration_observation.launch.py")
    assistant = launch_source("memory_assistant.launch.py")

    assert exploration.count('"slam": "true"') == 2
    assert exploration.count('"map": ""') == 2
    assert assistant.count('"slam": "false"') == 1
    assert assistant.count('"map": str(manifest.map_yaml)') == 1


def test_public_launches_keep_phase10_acceptance_out_of_runtime():
    combined = launch_source("autonomous_memory_build.launch.py") + launch_source(
        "memory_assistant.launch.py"
    )
    assert "rosbag" not in combined
    assert "ground_truth" not in combined
    assert "acceptance_evaluator" not in combined
