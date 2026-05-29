#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

ros2 service call /control/control_mode_request \
  autoware_vehicle_msgs/srv/ControlModeCommand \
  "{mode: 1}"
