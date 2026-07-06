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

colcon build --symlink-install "${COLCON_WORKER_ARGS[@]}" --packages-up-to "${PACKAGES_UP_TO[@]}"
colcon build --symlink-install "${COLCON_WORKER_ARGS[@]}" --packages-select "${PACKAGES_SELECT[@]}"
