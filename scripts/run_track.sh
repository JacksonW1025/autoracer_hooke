#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"

if [[ -f "$ROOT_DIR/defaults.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/defaults.env"
  set +a
fi

ros2 launch autoracer_bringup track.launch.py \
  map_path:="${MAP_PATH}" \
  launch_sensing:="${LAUNCH_SENSING}" \
  launch_localization:="${LAUNCH_LOCALIZATION}" \
  launch_vehicle:="${LAUNCH_VEHICLE}" \
  launch_rviz:="${LAUNCH_RVIZ}" \
  enable_drive_commands:="${ENABLE_DRIVE_COMMANDS}" \
  max_speed_mps:="${MAX_SPEED_MPS}" \
  can_channel_id:="${CAN_CHANNEL_ID}" \
  can_baudrate:="${CAN_BAUDRATE}"

