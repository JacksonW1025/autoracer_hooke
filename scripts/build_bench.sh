#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AUTORACER_SOURCE_LOCAL_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

COLCON_WORKER_ARGS=()
if [[ -n "${COLCON_PARALLEL_WORKERS:-}" ]]; then
  COLCON_WORKER_ARGS+=(--parallel-workers "$COLCON_PARALLEL_WORKERS")
fi

PACKAGES_UP_TO=(
  autoracer_sensing
  autoracer_vehicle_interface
  hipnuc_imu
  lslidar_driver
  nebula_hesai
  fixposition_driver_ros2
  autoware_localization_rviz_plugin
  autoware_planning_rviz_plugin
  tier4_control_mode_rviz_plugin
  tier4_state_rviz_plugin
  tier4_vehicle_rviz_plugin
  tier4_planning_factor_rviz_plugin
)

PACKAGES_SELECT=(
  autoracer_description
  autoracer_bringup
)

OVERRIDE_PACKAGES=(
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_lanelet2_extension
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_utils_math
  autoware_utils_system
  autoware_utils_visualization
  autoware_vehicle_msgs
)

colcon build --symlink-install \
  "${COLCON_WORKER_ARGS[@]}" \
  --packages-up-to "${PACKAGES_UP_TO[@]}" \
  --allow-overriding "${OVERRIDE_PACKAGES[@]}"
colcon build --symlink-install "${COLCON_WORKER_ARGS[@]}" --packages-select "${PACKAGES_SELECT[@]}"
