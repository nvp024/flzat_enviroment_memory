"""Static contract for the standalone Isaac Sim host installer."""

from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
INSTALLER = REPOSITORY_ROOT / "tools" / "setup_isaac_sim.sh"
GENERAL_INSTALLER = REPOSITORY_ROOT / "tools" / "setup_new_machine.sh"


def test_isaac_installer_is_valid_and_separate_from_general_setup():
    assert INSTALLER.is_file()
    subprocess.run(["bash", "-n", str(INSTALLER)], check=True)
    help_result = subprocess.run(
        ["bash", str(INSTALLER), "--help"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "Isaac Sim 5.0.0" in help_result.stdout

    source = INSTALLER.read_text(encoding="utf-8")
    assert 'isaac_version="5.0.0"' in source
    assert 'driver_branch="580"' in source
    assert "ubuntu-drivers install" in source
    assert "download.isaacsim.omniverse.nvidia.com" in source
    assert "setup_new_machine.sh" not in source
    assert "colcon build" not in source
    assert "conda create" not in source
    assert "pip install" not in source
    assert "ros-jazzy" not in source


def test_general_installer_builds_isaac_package_without_installing_runtime():
    subprocess.run(["bash", "-n", str(GENERAL_INSTALLER)], check=True)
    source = GENERAL_INSTALLER.read_text(encoding="utf-8")

    assert "openarm_skeleton_v1_2_isaac/package.xml" in source
    assert "isaac_sdf_scene.py" in source
    assert "hotel_lobby_demo.sdf" in source
    assert "openarm_skeleton_v1_2_isaac" in source
    assert "run_isaac_nav2.sh" in source
    assert "check_isaac_host.sh" in source
    assert "python3-lark" in source
    assert "'lark==1.3.1'" in source
    assert "import lark" in source
    assert 'bash %q\\n\' "$memory_ws/tools/setup_isaac_sim.sh"' in source
    assert "nvidia-driver-" not in source
    assert "download.isaacsim.omniverse.nvidia.com" not in source
