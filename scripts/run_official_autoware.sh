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

: "${AUTORACER_VEHICLE_MODEL:=autoracer_rc}"
: "${AUTORACER_SENSOR_MODEL:=autoracer_rc_sensor_kit}"
: "${AUTOWARE_DATA_PATH:=${HOME}/autoware_data}"
: "${POINTCLOUD_CONTAINER_NAME:=pointcloud_container}"
: "${LANELET2_MAP_FILE:=lanelet2_map.osm}"
: "${POINTCLOUD_MAP_FILE:=pointcloud_map.pcd}"
: "${LAUNCH_VEHICLE:=true}"
: "${LAUNCH_SENSING:=true}"
: "${LAUNCH_LOCALIZATION:=true}"
: "${LAUNCH_PLANNING:=true}"
: "${LAUNCH_CONTROL:=true}"
: "${LAUNCH_RVIZ:=false}"
: "${LAUNCH_MAP:=true}"
: "${LAUNCH_SYSTEM:=true}"
: "${LAUNCH_SYSTEM_MONITOR:=true}"
: "${LAUNCH_API:=true}"
: "${LAUNCH_PERCEPTION:=false}"
: "${LAUNCH_SENSING_DRIVER:=${LAUNCH_SENSING:-true}}"
: "${LAUNCH_VEHICLE_INTERFACE:=${LAUNCH_VEHICLE:-true}}"
: "${RVIZ_CONFIG_NAME:=autoware.rviz}"
: "${ENABLE_ALL_MODULES_AUTO_MODE:=false}"

ACTIVE_AUTORACER_VEHICLE_MODEL="autoracer_rc"
ACTIVE_AUTORACER_SENSOR_MODEL="autoracer_rc_sensor_kit"

is_true() {
  case "${1,,}" in
    1 | true | yes | on) return 0 ;;
    *) return 1 ;;
  esac
}

require_active_profile_pair() {
  if [[ "${AUTORACER_VEHICLE_MODEL}" == "${ACTIVE_AUTORACER_VEHICLE_MODEL}" ]] &&
    [[ "${AUTORACER_SENSOR_MODEL}" == "${ACTIVE_AUTORACER_SENSOR_MODEL}" ]]; then
    return 0
  fi

  cat >&2 <<EOF
ERROR: Only the RC official profile is enabled in this branch.
Requested vehicle_model=${AUTORACER_VEHICLE_MODEL}
Requested sensor_model=${AUTORACER_SENSOR_MODEL}

Hooke is currently a disabled_placeholder guarded by COLCON_IGNORE. Use
scripts/hooke/hooke_start_autoware.sh for the Hooke handoff message until the
real Hooke profile is complete.
EOF
  exit 2
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
  if ! is_true "${LAUNCH_SENSING:-true}" || ! is_true "${LAUNCH_SENSING_DRIVER:-true}"; then
    return 0
  fi
  if [[ "${AUTORACER_SENSOR_MODEL}" != "autoracer_rc_sensor_kit" ]]; then
    return 0
  fi

  if lidar_route_ready && lidar_carrier_ready; then
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

if [[ -z "${MAP_PATH:-}" ]]; then
  echo "ERROR: MAP_PATH is required for official Autoware startup." >&2
  exit 1
fi

require_active_profile_pair

if is_true "${LAUNCH_VEHICLE_INTERFACE}" && [[ -z "${SERIAL_PORT:-}" ]]; then
  echo "ERROR: SERIAL_PORT is required when LAUNCH_VEHICLE_INTERFACE=true." >&2
  echo "Set SERIAL_PORT=/dev/<actual_chassis_tty> or LAUNCH_VEHICLE_INTERFACE=false." >&2
  exit 1
fi

require_lidar_link_ready

# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

LAUNCH_ARGS=(
  map_path:="${MAP_PATH}"
  vehicle_model:="${AUTORACER_VEHICLE_MODEL}"
  sensor_model:="${AUTORACER_SENSOR_MODEL}"
  pointcloud_container_name:="${POINTCLOUD_CONTAINER_NAME}"
  data_path:="${AUTOWARE_DATA_PATH}"
  lanelet2_map_file:="${LANELET2_MAP_FILE}"
  pointcloud_map_file:="${POINTCLOUD_MAP_FILE}"
  launch_vehicle:="${LAUNCH_VEHICLE}"
  launch_vehicle_interface:="${LAUNCH_VEHICLE_INTERFACE}"
  launch_system:="${LAUNCH_SYSTEM}"
  launch_system_monitor:="${LAUNCH_SYSTEM_MONITOR}"
  launch_map:="${LAUNCH_MAP}"
  launch_sensing:="${LAUNCH_SENSING}"
  launch_sensing_driver:="${LAUNCH_SENSING_DRIVER}"
  launch_localization:="${LAUNCH_LOCALIZATION}"
  launch_perception:="${LAUNCH_PERCEPTION}"
  launch_planning:="${LAUNCH_PLANNING}"
  launch_control:="${LAUNCH_CONTROL}"
  launch_api:="${LAUNCH_API}"
  enable_all_modules_auto_mode:="${ENABLE_ALL_MODULES_AUTO_MODE}"
  rviz:="${LAUNCH_RVIZ}"
  rviz_config_name:="${RVIZ_CONFIG_NAME}"
)

ros2 launch autoware_launch autoware.launch.xml \
  "${LAUNCH_ARGS[@]}"
