#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

if [[ -f "$ROOT_DIR/defaults.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/defaults.env"
  set +a
fi

is_true() {
  case "${1,,}" in
    1 | true | yes | on) return 0 ;;
    *) return 1 ;;
  esac
}

lidar_route_ready() {
  local route
  route="$(ip route get "$LIDAR_SENSOR_IP" 2>/dev/null || true)"
  [[ "$route" == *" dev ${LIDAR_IFACE} "* && "$route" == *" src ${LIDAR_HOST_IP} "* ]]
}

lidar_carrier_ready() {
  [[ -r "/sys/class/net/${LIDAR_IFACE}/carrier" ]] || return 1
  [[ "$(cat "/sys/class/net/${LIDAR_IFACE}/carrier")" == "1" ]]
}

try_configure_lidar_link() {
  if ! is_true "${RC_AUTO_CONFIGURE_LIDAR_LINK:-true}"; then
    return 0
  fi

  if [[ "$EUID" -eq 0 ]]; then
    ./scripts/rc/rc_configure_lidar.sh
    return 0
  fi

  if sudo -n true 2>/dev/null; then
    sudo -n -E ./scripts/rc/rc_configure_lidar.sh
    return 0
  fi

  echo "[rc-lidar] sudo is not available non-interactively." >&2
  echo "[rc-lidar] If the link is not ready, run: sudo -E ./scripts/rc/rc_configure_lidar.sh" >&2
}

require_lidar_link_ready() {
  if ! is_true "${RC_REQUIRE_LIDAR_LINK:-true}"; then
    return 0
  fi
  if ! is_true "${LAUNCH_SENSING}" || [[ "${LIDAR_DRIVER}" != "lslidar_c32" ]]; then
    return 0
  fi

  try_configure_lidar_link

  local deadline=$((SECONDS + ${LIDAR_LINK_WAIT_SEC:-20}))
  while (( SECONDS <= deadline )); do
    if lidar_route_ready && lidar_carrier_ready; then
      return 0
    fi
    sleep 1
  done

  echo "ERROR: C32 LiDAR link is not ready." >&2
  echo "Expected ${LIDAR_IFACE} ${LIDAR_HOST_IP}/32 with route to ${LIDAR_SENSOR_IP}/32." >&2
  echo "Run: sudo -E ./scripts/rc/rc_configure_lidar.sh" >&2
  echo "Then check LiDAR power/cable if carrier is still 0/down." >&2
  ip -brief addr show dev "$LIDAR_IFACE" >&2 || true
  ip route get "$LIDAR_SENSOR_IP" >&2 || true
  if [[ -r "/sys/class/net/${LIDAR_IFACE}/carrier" ]]; then
    echo "carrier=$(cat "/sys/class/net/${LIDAR_IFACE}/carrier")" >&2
  fi
  if [[ -r "/sys/class/net/${LIDAR_IFACE}/operstate" ]]; then
    echo "operstate=$(cat "/sys/class/net/${LIDAR_IFACE}/operstate")" >&2
  fi
  exit 1
}

require_lidar_link_ready

LAUNCH_ARGS=(
  map_path:="${MAP_PATH}"
  launch_sensing:="${LAUNCH_SENSING}"
  launch_imu:="${LAUNCH_IMU}"
  launch_localization:="${LAUNCH_LOCALIZATION}"
  launch_planning:="${LAUNCH_PLANNING}"
  launch_control:="${LAUNCH_CONTROL}"
  launch_safety:="${LAUNCH_SAFETY}"
  launch_vehicle:="${LAUNCH_VEHICLE}"
  launch_rviz:="${LAUNCH_RVIZ}"
  extrinsics_file:="${EXTRINSICS_FILE}"
  enable_drive_commands:="${ENABLE_DRIVE_COMMANDS}"
  max_speed_mps:="${MAX_SPEED_MPS}"
  control_min_lookahead_m:="${CONTROL_MIN_LOOKAHEAD_M}"
  control_lookahead_gain:="${CONTROL_LOOKAHEAD_GAIN}"
  control_goal_tolerance_m:="${CONTROL_GOAL_TOLERANCE_M}"
  control_max_steer_rate_radps:="${CONTROL_MAX_STEER_RATE_RADPS}"
  serial_baudrate:="${SERIAL_BAUDRATE}"
  imu_serial_port:="${IMU_SERIAL_PORT}"
  imu_baudrate:="${IMU_BAUDRATE}"
  wheel_base_m:="${WHEEL_BASE_M}"
  max_steer_rad:="${MAX_STEER_RAD}"
  lidar_driver:="${LIDAR_DRIVER}"
  lidar_param_file:="${LIDAR_PARAM_FILE}"
  lidar_sensor_ip:="${LIDAR_SENSOR_IP}"
  lidar_data_port:="${LIDAR_DATA_PORT}"
  lidar_sensor_model:="${LIDAR_SENSOR_MODEL}"
  launch_pointcloud_filter:="${LAUNCH_POINTCLOUD_FILTER}"
  pointcloud_filter_input_topic:="${POINTCLOUD_FILTER_INPUT_TOPIC}"
  pointcloud_filter_output_topic:="${POINTCLOUD_FILTER_OUTPUT_TOPIC}"
  pointcloud_filter_leaf_size_m:="${POINTCLOUD_FILTER_LEAF_SIZE_M}"
  pointcloud_filter_min_range_m:="${POINTCLOUD_FILTER_MIN_RANGE_M}"
  pointcloud_filter_max_range_m:="${POINTCLOUD_FILTER_MAX_RANGE_M}"
  pointcloud_filter_max_points:="${POINTCLOUD_FILTER_MAX_POINTS}"
  localization_pointcloud_topic:="${LOCALIZATION_POINTCLOUD_TOPIC}"
  launch_fixposition:="${LAUNCH_FIXPOSITION}"
  launch_fixposition_seed:="${LAUNCH_FIXPOSITION_SEED}"
  launch_manual_seed:="${LAUNCH_MANUAL_SEED}"
  launch_map_projection_loader:="${LAUNCH_MAP_PROJECTION_LOADER}"
  manual_seed_input_topic:="${MANUAL_SEED_INPUT_TOPIC}"
  manual_seed_require_input_pose:="${MANUAL_SEED_REQUIRE_INPUT_POSE}"
  manual_seed_x:="${MANUAL_SEED_X}"
  manual_seed_y:="${MANUAL_SEED_Y}"
  manual_seed_z:="${MANUAL_SEED_Z}"
  manual_seed_yaw:="${MANUAL_SEED_YAW}"
  manual_seed_xy_variance:="${MANUAL_SEED_XY_VARIANCE}"
  manual_seed_z_variance:="${MANUAL_SEED_Z_VARIANCE}"
  manual_seed_yaw_variance:="${MANUAL_SEED_YAW_VARIANCE}"
  ndt_param_file:="${NDT_PARAM_FILE}"
  ndt_initial_pose_stamp_offset_sec:="${NDT_INITIAL_POSE_STAMP_OFFSET_SEC}"
  fixposition_stream:="${FIXPOSITION_STREAM}"
)
if [[ -n "${LIDAR_HOST_IP:-}" ]]; then
  LAUNCH_ARGS+=(lidar_host_ip:="${LIDAR_HOST_IP}")
fi
if [[ -n "${SERIAL_PORT:-}" ]]; then
  LAUNCH_ARGS+=(serial_port:="${SERIAL_PORT}")
fi

ros2 launch autoracer_bringup track.launch.py \
  "${LAUNCH_ARGS[@]}"
