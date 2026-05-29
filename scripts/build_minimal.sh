#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/humble/setup.bash

PACKAGES=(
  autoracer_description
  autoracer_localization
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
  fixposition_driver_ros2
  hooke2_description
  hooke2_sensor_kit_description
  hooke2_interface
)

colcon build --symlink-install --packages-up-to "${PACKAGES[@]}"
