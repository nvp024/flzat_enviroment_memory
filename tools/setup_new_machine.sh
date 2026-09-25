#!/usr/bin/env bash
# Install and build the three FLZAT ROS workspaces on Ubuntu 24.04 x86_64.
# Run as a normal user. System package commands inside the script request sudo.
set -euo pipefail

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
step() { printf '\n==> %s\n' "$*"; }
source_ros_setup() {
  # ROS-generated setup files use optional variables without nounset guards.
  set +u
  source "$1"
  set -u
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "$script_dir/../.." && pwd -P)"
openarm_ws="$project_root/openarm_skeleton_v1.2_ws"
robot_ws="$project_root/flzat_robot_ws"
memory_ws="$project_root/flzat_enviroment_memory"
conda_dir="${FLZAT_MINICONDA_DIR:-$HOME/miniconda3}"
conda_env="py312"
prefetch_models="${FLZAT_PREFETCH_MODELS:-1}"
setup_state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/flzat"
setup_state_file="$setup_state_dir/setup_new_machine_$(printf '%s' "$project_root" | sha256sum | cut -d ' ' -f 1).state"

[[ $EUID -ne 0 ]] || fail "Run as your normal user; do not prefix this script with sudo."
[[ -r /etc/os-release ]] || fail "Cannot identify the operating system."
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_CODENAME:-} == noble ]] ||
  fail "This setup supports Ubuntu 24.04 (noble) only; found ${PRETTY_NAME:-unknown}."
[[ $(uname -m) == x86_64 ]] || fail "This setup supports x86_64 (amd64) only."
[[ -f "$memory_ws/environment_memory.repos" ]] ||
  fail "Memory workspace not found at $memory_ws. Keep the three repositories side by side."

step "Install Git and fetch any missing sibling workspaces"
sudo -v
sudo apt-get update
sudo apt-get install -y git curl ca-certificates software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y git-lfs
git lfs install --skip-repo

if [[ ! -d "$openarm_ws" ]]; then
  git clone --branch main https://github.com/nvp024/flzat_nav_ws.git "$openarm_ws"
fi
if [[ ! -d "$robot_ws" ]]; then
  git clone --branch feature/multimodel-pipeline-v0.1 \
    https://github.com/nvp024/flzat-voice-ros2.git "$robot_ws"
fi
[[ -f "$openarm_ws/src/openarm_skeleton_v1_2_gazebo/package.xml" ]] ||
  fail "OpenArm workspace is incomplete: $openarm_ws"
[[ -f "$openarm_ws/src/openarm_skeleton_v1_2_isaac/package.xml" ]] ||
  fail "OpenArm workspace does not contain the Isaac package. Pull the current main branch in $openarm_ws."
[[ -f "$openarm_ws/src/openarm_skeleton_v1_2_isaac/scripts/isaac_sdf_scene.py" ]] ||
  fail "OpenArm Isaac package is outdated: scripts/isaac_sdf_scene.py is missing. Pull the current main branch."
[[ -f "$openarm_ws/src/openarm_skeleton_v1_2_gazebo/worlds/hotel_lobby_demo.sdf" ]] ||
  fail "The shared Gazebo/Isaac hotel world is missing from $openarm_ws."
[[ -f "$openarm_ws/scripts/run_isaac_nav2.sh" ]] ||
  fail "OpenArm workspace is missing scripts/run_isaac_nav2.sh."
[[ -f "$openarm_ws/scripts/check_isaac_host.sh" ]] ||
  fail "OpenArm workspace is missing scripts/check_isaac_host.sh."
[[ -f "$robot_ws/src/vlm_pipeline/package.xml" ]] ||
  fail "Robot workspace is incomplete: $robot_ws"
[[ -f "$memory_ws/tools/setup_isaac_sim.sh" ]] ||
  fail "The separate Isaac Sim installer is missing: $memory_ws/tools/setup_isaac_sim.sh"

# Colcon-generated paths contain machine-specific prefixes. Only resume a build
# started by this script on the same machine and at the same absolute path.
[[ -r /etc/machine-id ]] || fail "Cannot identify this machine to validate existing build artifacts."
setup_identity="$(< /etc/machine-id) $project_root"
for workspace_dir in "$openarm_ws" "$robot_ws" "$memory_ws"; do
  for generated_dir in build install log; do
    if [[ -e "$workspace_dir/$generated_dir" || -L "$workspace_dir/$generated_dir" ]]; then
      if [[ ! -f "$setup_state_file" || $(< "$setup_state_file") != "$setup_identity" ]]; then
        fail "Found $workspace_dir/$generated_dir without a matching setup marker for this machine and path. Move old build/install/log folders aside before running on the new machine."
      fi
    fi
  done
done

step "Install ROS 2 Jazzy, Gazebo Harmonic, Nav2 and SLAM Toolbox"
ros_apt_version="$(curl --fail --silent --show-error \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | awk -F'"' '/"tag_name"/ {version=$4} END {print version}')"
[[ -n "$ros_apt_version" ]] || fail "Could not determine the ROS apt-source version."
ros_apt_deb="$(mktemp --suffix=.deb)"
trap 'rm -f -- "$ros_apt_deb"' EXIT
curl --fail --location --show-error --output "$ros_apt_deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.noble_all.deb"
sudo dpkg -i "$ros_apt_deb"
rm -f -- "$ros_apt_deb"
trap - EXIT
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-dev-tools python3-colcon-common-extensions python3-lark \
  python3-pytest python3-rosdep
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
source_ros_setup /opt/ros/jazzy/setup.bash
ros2 --help >/dev/null
gz sim --versions

step "Fetch tracked Whisper model files"
if [[ -d "$robot_ws/.git" ]]; then
  git -C "$robot_ws" lfs pull
fi
[[ -f "$robot_ws/models/tiny.pt" ]] || fail "Missing Whisper model: $robot_ws/models/tiny.pt"
[[ $(stat -c %s "$robot_ws/models/tiny.pt") -gt 1000000 ]] ||
  fail "Whisper tiny.pt is a Git LFS pointer, not model weights. Run git lfs pull in $robot_ws."

step "Install system libraries for audio and VLM"
sudo apt-get install -y ffmpeg libportaudio2 portaudio19-dev espeak-ng libespeak1

step "Install Miniconda and create the py312 environment"
if [[ ! -f "$conda_dir/etc/profile.d/conda.sh" ]]; then
  [[ ! -e "$conda_dir" ]] || fail "$conda_dir exists but is not a usable Miniconda installation."
  conda_installer="$(mktemp --suffix=.sh)"
  trap 'rm -f -- "$conda_installer"' EXIT
  curl --fail --location --show-error \
    --output "$conda_installer" \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
  bash "$conda_installer" -b -p "$conda_dir"
  rm -f -- "$conda_installer"
  trap - EXIT
fi
source "$conda_dir/etc/profile.d/conda.sh"
if [[ ! -d "$conda_dir/envs/$conda_env" ]]; then
  conda create -y --override-channels -c conda-forge -n "$conda_env" python=3.12 pip
fi
conda activate "$conda_dir/envs/$conda_env"
python -c 'import sys; assert sys.version_info[:2] == (3, 12), "py312 must use Python 3.12"'

step "Install Python packages used by speech, VLM and environment memory"
python -m pip install --upgrade pip wheel 'setuptools==79.0.1' colcon-common-extensions
if [[ -n ${FLZAT_TORCH_INDEX_URL:-} ]]; then
  python -m pip install --index-url "$FLZAT_TORCH_INDEX_URL" \
    'torch==2.13.0' 'torchvision==0.28.0'
else
  python -m pip install 'torch==2.13.0' 'torchvision==0.28.0'
fi
python -m pip install \
  'transformers==5.14.1' \
  'qwen-vl-utils==0.0.14' \
  'num2words==0.5.14' \
  'openai-whisper==20250625' \
  'sounddevice==0.5.5' \
  'pyttsx3==2.99' \
  'opencv-python==5.0.0.93' \
  'numpy==2.4.6' \
  'chromadb==1.5.9' \
  'sentence-transformers==6.0.0' \
  'Pillow==12.3.0' \
  'lark==1.3.1' \
  typeguard \
  pytest

step "Fetch the pinned frontier-exploration source and ROS dependencies"
if [[ ! -d "$memory_ws/src/frontier_exploration_ros2" ]]; then
  vcs import "$memory_ws" < "$memory_ws/environment_memory.repos"
fi
[[ -f "$memory_ws/src/frontier_exploration_ros2/package.xml" ]] ||
  fail "Frontier package was not fetched into $memory_ws/src/frontier_exploration_ros2."
source_ros_setup /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  "$openarm_ws/src" "$robot_ws/src" "$memory_ws/src" \
  --ignore-src -r -y --rosdistro jazzy --skip-keys ament_python

step "Build OpenArm, robot and environment-memory workspaces"
mkdir -p -- "$setup_state_dir"
printf '%s\n' "$setup_identity" > "$setup_state_file"
(
  cd "$openarm_ws"
  colcon build --symlink-install
)
source_ros_setup "$openarm_ws/install/setup.bash"
(
  cd "$robot_ws"
  colcon build --symlink-install
)
source_ros_setup "$robot_ws/install/setup.bash"
(
  cd "$memory_ws"
  colcon build --symlink-install
)
source_ros_setup "$memory_ws/install/setup.bash"

if [[ $prefetch_models == 1 ]]; then
  step "Download the default VLM and embedding models (several GB)"
  python - <<'PY'
from huggingface_hub import snapshot_download

models = (
    ("Qwen/Qwen3-VL-2B-Instruct", "main"),
    ("HuggingFaceTB/SmolVLM2-500M-Video-Instruct", "main"),
    (
        "sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2",
        "e8f8c211226b894fcb81acc59f3b34ba3efd5f42",
    ),
)
for model_id, revision in models:
    print(f"Downloading {model_id} ({revision})", flush=True)
    snapshot_download(repo_id=model_id, revision=revision)
PY
fi

step "Verify installation"
python -m pip check
python - "$prefetch_models" <<'PY'
import sys

import chromadb
import cv2
import lark
import num2words
import qwen_vl_utils
import rclpy
import sentence_transformers
import sounddevice
import torch
import torchvision
import transformers
import whisper
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

assert (
    getattr(transformers, "AutoModelForImageTextToText", None)
    or getattr(transformers, "AutoModelForMultimodalLM", None)
), "Installed transformers has no SmolVLM2-compatible model class"

device = "cuda" if torch.cuda.is_available() else "cpu"
boxes = torch.tensor([[0.0, 0.0, 1.0, 1.0]], device=device)
scores = torch.tensor([1.0], device=device)
assert torchvision.ops.nms(boxes, scores, 0.5).tolist() == [0]

if sys.argv[1] == "1":
    for model_id in (
        "Qwen/Qwen3-VL-2B-Instruct",
        "HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
    ):
        AutoProcessor.from_pretrained(model_id, local_files_only=True)
        print(f"Local processor ready: {model_id}")

print("ROS Python, OpenCV, audio, Chroma and VLM imports: OK")
print(f"PyTorch: {torch.__version__}; torchvision: {torchvision.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA GPU: {torch.cuda.get_device_name(0)}")
else:
    print("WARNING: CUDA is unavailable; VLM device:=cuda will not work.")
PY
ros2 --help >/dev/null
gz sim --versions
for package_name in \
  openarm_skeleton_v1_2_navigation openarm_skeleton_v1_2_isaac vlm_pipeline \
  environment_memory frontier_exploration_ros2; do
  ros2 pkg prefix "$package_name"
done

printf '\nInstallation complete. In each new terminal, run:\n'
printf 'source %q\n' "$conda_dir/etc/profile.d/conda.sh"
printf 'conda activate %q\n' "$conda_dir/envs/$conda_env"
printf 'source /opt/ros/jazzy/setup.bash\n'
printf 'source %q\n' "$openarm_ws/install/setup.bash"
printf 'source %q\n' "$robot_ws/install/setup.bash"
printf 'source %q\n' "$memory_ws/install/setup.bash"
printf '\nExisting maps and Chroma data are separate from Git and must be copied manually.\n'
printf '\nNVIDIA driver and Isaac Sim are intentionally separate. Install them with:\n'
printf 'bash %q\n' "$memory_ws/tools/setup_isaac_sim.sh"
