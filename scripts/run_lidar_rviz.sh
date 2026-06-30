#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -f "$ROOT_DIR/defaults.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/defaults.env"
  set +a
fi

# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

LAUNCH_ARGS=(
  extrinsics_file:="${EXTRINSICS_FILE}"
  lidar_driver:="${LIDAR_DRIVER}"
  lidar_param_file:="${LIDAR_PARAM_FILE}"
  lidar_sensor_ip:="${LIDAR_SENSOR_IP}"
  lidar_data_port:="${LIDAR_DATA_PORT}"
  lidar_sensor_model:="${LIDAR_SENSOR_MODEL}"
)
if [[ -n "${LIDAR_HOST_IP:-}" ]]; then
  LAUNCH_ARGS+=(lidar_host_ip:="${LIDAR_HOST_IP}")
fi

ros2 launch autoracer_bringup lidar_rviz.launch.py \
  "${LAUNCH_ARGS[@]}"
