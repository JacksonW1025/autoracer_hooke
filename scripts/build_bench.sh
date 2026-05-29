#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AUTORACER_SOURCE_LOCAL_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

PACKAGES_UP_TO=(
  autoracer_sensing
  nebula_hesai
  fixposition_driver_ros2
  hooke2_interface
  can_driver
)

PACKAGES_SELECT=(
  autoracer_description
  autoracer_bringup
)

colcon build --symlink-install --packages-up-to "${PACKAGES_UP_TO[@]}"
colcon build --symlink-install --packages-select "${PACKAGES_SELECT[@]}"
