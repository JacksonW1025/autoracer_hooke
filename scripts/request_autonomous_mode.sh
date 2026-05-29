#!/usr/bin/env bash
set -euo pipefail

ros2 service call /control/control_mode_request \
  autoware_vehicle_msgs/srv/ControlModeCommand \
  "{mode: 1}"

