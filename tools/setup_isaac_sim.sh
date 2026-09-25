#!/usr/bin/env bash
# Install the NVIDIA host dependencies and Isaac Sim used by the FLZAT stack.
# This script intentionally does not install ROS, Gazebo, Conda, VLM models,
# or build any of the three ROS workspaces.
set -euo pipefail

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
step() { printf '\n==> %s\n' "$*"; }

isaac_version="5.0.0"
driver_branch="580"
install_dir="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
download_dir="${XDG_CACHE_HOME:-$HOME/.cache}/flzat/isaac_sim"
archive=""
driver_profile="${FLZAT_NVIDIA_PROFILE:-auto}"
skip_driver=false
allow_driver_switch=false

usage() {
  cat <<'EOF'
Install only NVIDIA/Isaac Sim dependencies for the FLZAT project.

Usage:
  bash tools/setup_isaac_sim.sh [options]

Options:
  --archive PATH           Use an existing Isaac Sim 5.0.0 zip.
  --install-dir PATH       Install directory (default: $ISAAC_SIM_PATH or ~/isaacsim).
  --driver-profile VALUE   auto, desktop, or server (default: auto).
  --skip-driver            Keep the currently installed NVIDIA driver.
  --allow-driver-switch    Permit switching an existing non-580 driver to branch 580.
  -h, --help               Show this help.

Environment equivalents:
  ISAAC_SIM_PATH           Default install directory.
  FLZAT_NVIDIA_PROFILE     auto, desktop, or server.

The project is pinned to Isaac Sim 5.0.0 and NVIDIA driver branch 580.
The script never removes an existing Isaac directory. It is safe to rerun.
EOF
}

while (($#)); do
  case "$1" in
    --archive)
      (($# >= 2)) || fail "--archive requires a path."
      archive="$2"
      shift 2
      ;;
    --install-dir)
      (($# >= 2)) || fail "--install-dir requires a path."
      install_dir="$2"
      shift 2
      ;;
    --driver-profile)
      (($# >= 2)) || fail "--driver-profile requires auto, desktop, or server."
      driver_profile="$2"
      shift 2
      ;;
    --skip-driver)
      skip_driver=true
      shift
      ;;
    --allow-driver-switch)
      allow_driver_switch=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown option: $1 (use --help)."
      ;;
  esac
done

[[ $EUID -ne 0 ]] || fail "Run as your normal user; do not prefix the script with sudo."
[[ -r /etc/os-release ]] || fail "Cannot identify the operating system."
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_CODENAME:-} == noble ]] ||
  fail "This installer supports Ubuntu 24.04 (noble) only; found ${PRETTY_NAME:-unknown}."
[[ $(uname -m) == x86_64 ]] || fail "Isaac Sim 5.0.0 standalone requires x86_64."
[[ $driver_profile =~ ^(auto|desktop|server)$ ]] ||
  fail "--driver-profile must be auto, desktop, or server."

install_dir="$(realpath -m -- "$install_dir")"
if [[ -n $archive ]]; then
  archive="$(realpath -m -- "$archive")"
  [[ -f $archive ]] || fail "Isaac Sim archive not found: $archive"
fi

step "Install Isaac Sim host utilities"
sudo -v
sudo apt-get update
sudo apt-get install -y \
  ca-certificates curl unzip pciutils ubuntu-drivers-common \
  libgl1 libglu1-mesa libvulkan1 vulkan-tools \
  libx11-6 libxcb1 libxcursor1 libxi6 libxinerama1 libxrandr2

if ! lspci -nn | grep -Eqi 'NVIDIA|3D controller.*NVIDIA|VGA.*NVIDIA'; then
  fail "No NVIDIA GPU was detected by lspci. Isaac Sim requires an RTX-capable NVIDIA GPU."
fi

if [[ $driver_profile == auto ]]; then
  vendor="$(cat /sys/devices/virtual/dmi/id/sys_vendor 2>/dev/null || true)"
  product="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null || true)"
  if [[ "$vendor $product" == *Amazon* || "$vendor $product" == *EC2* ]]; then
    driver_profile="server"
  else
    driver_profile="desktop"
  fi
fi

reboot_required=false
current_driver=""
if command -v nvidia-smi >/dev/null 2>&1; then
  current_driver="$(
    nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null |
      head -1 | tr -d '[:space:]'
  )"
fi
current_branch="${current_driver%%.*}"

if [[ $skip_driver == true ]]; then
  step "Keep the existing NVIDIA driver (--skip-driver)"
elif [[ $current_branch == "$driver_branch" ]]; then
  step "NVIDIA driver branch $driver_branch is already active ($current_driver)"
else
  if [[ -n $current_driver && $allow_driver_switch != true ]]; then
    fail "NVIDIA driver $current_driver is active. Rerun with --allow-driver-switch to replace it with the project-validated branch $driver_branch, or use --skip-driver to keep it."
  fi

  step "Install NVIDIA driver branch $driver_branch ($driver_profile profile)"
  if [[ $driver_profile == server ]]; then
    available_drivers="$(ubuntu-drivers list --gpgpu 2>/dev/null || true)"
    grep -q "nvidia-driver-${driver_branch}-server" <<<"$available_drivers" || {
      printf '%s\n' "$available_drivers" >&2
      fail "Ubuntu does not currently offer nvidia-driver-${driver_branch}-server for this GPU."
    }
    sudo ubuntu-drivers install --gpgpu "nvidia:${driver_branch}-server"
  else
    available_drivers="$(ubuntu-drivers list 2>/dev/null || true)"
    grep -q "nvidia-driver-${driver_branch}" <<<"$available_drivers" || {
      printf '%s\n' "$available_drivers" >&2
      fail "Ubuntu does not currently offer nvidia-driver-${driver_branch} for this GPU."
    }
    sudo ubuntu-drivers install "nvidia:${driver_branch}"
  fi
  reboot_required=true
fi

if [[ -f "$install_dir/python.sh" && -f "$install_dir/isaac-sim.sh" ]]; then
  step "Isaac Sim is already installed at $install_dir"
else
  if [[ -e $install_dir ]] &&
     [[ -n $(find "$install_dir" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null) ]]; then
    fail "$install_dir exists and is not an Isaac Sim installation. Move it aside or choose --install-dir."
  fi

  install_parent="$(dirname -- "$install_dir")"
  mkdir -p -- "$install_parent" "$download_dir"
  available_kib="$(df -Pk -- "$install_parent" | awk 'NR == 2 {print $4}')"
  required_kib=$((50 * 1024 * 1024))
  (( available_kib >= required_kib )) ||
    fail "Isaac Sim installation needs at least 50 GB free; only $((available_kib / 1024 / 1024)) GB is available on the target filesystem."

  if [[ -z $archive ]]; then
    archive="$download_dir/isaac-sim-standalone-${isaac_version}-linux-x86_64.zip"
    archive_part="${archive}.part"
    if [[ ! -f $archive ]]; then
      step "Download Isaac Sim $isaac_version (large file; resume is enabled)"
      download_url="https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-${isaac_version}-linux-x86_64.zip"
      curl --fail --location --show-error --retry 5 --retry-delay 5 \
        --continue-at - --output "$archive_part" "$download_url"
      mv -- "$archive_part" "$archive"
    fi
  fi

  step "Validate and extract Isaac Sim $isaac_version"
  unzip -tq "$archive" >/dev/null || fail "Invalid or incomplete zip archive: $archive"
  mkdir -p -- "$install_dir"
  unzip -q "$archive" -d "$install_dir"
  [[ -f "$install_dir/python.sh" ]] || fail "Extraction completed but python.sh is missing in $install_dir."
  [[ -f "$install_dir/post_install.sh" ]] || fail "Extraction completed but post_install.sh is missing."
  [[ -f "$install_dir/isaac-sim.sh" ]] || fail "Extraction completed but isaac-sim.sh is missing."

  step "Run Isaac Sim post-install setup"
  (
    cd "$install_dir"
    bash ./post_install.sh
  )
fi

step "Record the Isaac Sim environment"
environment_dir="${XDG_CONFIG_HOME:-$HOME/.config}/flzat"
environment_file="$environment_dir/isaac_sim_env.sh"
mkdir -p -- "$environment_dir"
printf 'export ISAAC_SIM_PATH=%q\n' "$install_dir" > "$environment_file"
printf 'Created %s\n' "$environment_file"

[[ -f "$install_dir/python.sh" ]] || fail "Missing $install_dir/python.sh"
[[ -f "$install_dir/isaac-sim.sh" ]] || fail "Missing $install_dir/isaac-sim.sh"

printf '\nIsaac Sim %s files are ready at: %s\n' "$isaac_version" "$install_dir"
if [[ $reboot_required == true || -f /var/run/reboot-required ]]; then
  cat <<EOF

The NVIDIA driver was installed or the system reports a pending reboot.
Reboot before running Isaac Sim:

  sudo reboot

After reconnecting, verify and load the environment:

  nvidia-smi
  source "$environment_file"
  "\$ISAAC_SIM_PATH/python.sh" --version
EOF
else
  if [[ $skip_driver != true ]]; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
  fi
  "$install_dir/python.sh" --version
  cat <<EOF

Load the environment in each new terminal with:

  source "$environment_file"

Then verify the OpenArm host from its workspace:

  ./scripts/check_isaac_host.sh
EOF
fi

printf '\nThis installer did not install ROS/Gazebo/Conda or build any workspace.\n'
