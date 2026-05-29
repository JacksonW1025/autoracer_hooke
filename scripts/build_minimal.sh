#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AUTORACER_SOURCE_LOCAL_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

PACKAGES=(
  autoracer_description
  autoracer_localization
  autoracer_sensing
  autoracer_planning
  autoracer_control
  autoracer_safety
  autoracer_bringup
  autoware_control_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  tier4_api_msgs
  tier4_debug_msgs
  tier4_external_api_msgs
  tier4_vehicle_msgs
  tier4_api_utils
  autoware_vehicle_info_utils
  autoware_map_projection_loader
  autoware_map_loader
  autoware_ndt_scan_matcher
  autoware_gnss_poser
  autoware_downsample_filters
  nebula_msgs
  nebula_hesai
  nebula_hesai_decoders
  wd_byte
  hooke2_msgs
  can_driver
  fixposition_driver_msgs
  fixposition_driver_lib
  rtcm_msgs
  fpsdk_common
  fpsdk_ros2
  fixposition_driver_ros2
  hooke2_description
  hooke2_launch
  hooke2_interface
)

colcon build --symlink-install --packages-up-to "${PACKAGES[@]}"
