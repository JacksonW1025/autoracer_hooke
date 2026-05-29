#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

mkdir -p src/external

copy_from_pilot_repo() {
  local pilot_repo="${PILOT_REPO:-/home/corage/workspace/project/pilot-auto.x1}"
  if [[ ! -d "${pilot_repo}" ]]; then
    echo "PILOT_REPO does not exist: ${pilot_repo}" >&2
    exit 1
  fi
  if ! command -v rsync >/dev/null 2>&1; then
    echo "rsync is required for IMPORT_FROM_PILOT=true." >&2
    exit 1
  fi

  copy_dir() {
    local src_dir="$1"
    local dst_dir="$2"
    if [[ ! -d "${src_dir}" ]]; then
      echo "Missing source directory: ${src_dir}" >&2
      exit 1
    fi
    mkdir -p "$(dirname "${dst_dir}")"
    rsync -a --delete \
      --exclude ".git" \
      --exclude "build" \
      --exclude "install" \
      --exclude "log" \
      "${src_dir}/" "${dst_dir}/"
  }

  copy_dir "${pilot_repo}/src/autoware/autoware_cmake" \
    "src/external/autoware/autoware_cmake"
  copy_dir "${pilot_repo}/src/autoware/autoware_utils" \
    "src/external/autoware/autoware_utils"
  copy_dir "${pilot_repo}/src/autoware/autoware_msgs" \
    "src/external/autoware/autoware_msgs"
  copy_dir "${pilot_repo}/src/autoware/autoware_internal_msgs" \
    "src/external/autoware/autoware_internal_msgs"
  copy_dir "${pilot_repo}/src/autoware/tier4_autoware_msgs" \
    "src/external/autoware/tier4_autoware_msgs"
  copy_dir "${pilot_repo}/src/autoware/core/common/autoware_vehicle_info_utils" \
    "src/external/autoware/core/common/autoware_vehicle_info_utils"
  copy_dir "${pilot_repo}/src/autoware/universe/common/tier4_api_utils" \
    "src/external/autoware/universe/common/tier4_api_utils"
  copy_dir "${pilot_repo}/src/vendor/nebula" \
    "src/external/vendor/nebula"
  copy_dir "${pilot_repo}/src/vendor/sync_tooling_msgs" \
    "src/external/vendor/sync_tooling_msgs"
  copy_dir "${pilot_repo}/src/vendor/boost_transport_drivers" \
    "src/external/vendor/boost_transport_drivers"
  copy_dir "${pilot_repo}/src/vendor/generate_parameter_library" \
    "src/external/vendor/generate_parameter_library"
  copy_dir "${pilot_repo}/src/whale_components/hardware_drivers/gnss/fixposition_driver" \
    "src/external/whale_components/hardware_drivers/gnss/fixposition_driver"
}

if [[ "${IMPORT_FROM_PILOT:-false}" == "true" ]]; then
  copy_from_pilot_repo
else
  if ! command -v vcs >/dev/null 2>&1; then
    echo "vcs is required. Install python3-vcstool first." >&2
    exit 1
  fi
  vcs import src < autoracer.repos
fi

# The Hooke2 chassis control chain is vendored in this repository under src/.
# Keep the external whale_components checkout available for other drivers, but
# hide duplicate package names from colcon if the full repo is imported.
for package_dir in \
  src/external/whale_components/hardware_drivers/can_driver \
  src/external/whale_components/hardware_drivers/gnss/fixposition_driver/fixposition_driver_ros1 \
  src/external/whale_components/hardware_drivers/gnss/fixposition_driver/fixposition-sdk/examples/ros1_fpsdk_demo \
  src/external/whale_components/hardware_drivers/gnss/fixposition_driver/fixposition-sdk/fpsdk_ros1 \
  src/external/whale_components/vehicle/interfaces/hooke2_interface/hooke2_interface \
  src/external/whale_components/vehicle/vehicle_launcher/hooke2_launch/hooke2_description \
  src/external/whale_components/vehicle/vehicle_launcher/hooke2_launch/hooke2_launch \
  src/external/whale_components/wd_msgs/vehicle_interface_msgs/hooke2_msgs \
  src/external/whale_components/wd_msgs/vehicle_interface_msgs/wd_byte
do
  if [[ -d "${package_dir}" ]]; then
    touch "${package_dir}/COLCON_IGNORE"
  fi
done
