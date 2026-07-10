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

RUN_LAUNCH="${RUN_LAUNCH:-1}"
LAUNCH_LIDAR="${LAUNCH_LIDAR:-true}"
LAUNCH_FIXPOSITION="${LAUNCH_FIXPOSITION:-true}"
LAUNCH_VEHICLE="${LAUNCH_VEHICLE:-true}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
WARMUP_SEC="${WARMUP_SEC:-12}"
SAMPLE_TIMEOUT_SEC="${SAMPLE_TIMEOUT_SEC:-8}"
HZ_TIMEOUT_SEC="${HZ_TIMEOUT_SEC:-8}"
CAN_INTERFACE="${CAN_INTERFACE:-can${CAN_CHANNEL_ID:-0}}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${RUN_DIR:-$ROOT_DIR/log/bench_verify_${STAMP}}"
SUMMARY_FILE="$RUN_DIR/summary.txt"
mkdir -p "$RUN_DIR"

FAIL_COUNT=0
LAUNCH_PID=""

log() {
  printf '%s\n' "$*" | tee -a "$SUMMARY_FILE"
}

mark_fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  log "FAIL $*"
}

mark_ok() {
  log "OK   $*"
}

mark_warn() {
  log "WARN $*"
}

is_true() {
  [[ "$1" == "true" || "$1" == "1" || "$1" == "yes" ]]
}

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "-${LAUNCH_PID}" >/dev/null 2>&1 || kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "-${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

require_pkg() {
  local package="$1"
  local prefix
  prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  if [[ -z "$prefix" ]]; then
    mark_fail "ROS package missing in this workspace: ${package}"
    return 1
  fi
  if [[ "$prefix" == "${AUTORACER_OLD_REPO:-/home/corage/workspace/project/pilot-auto.x1}"* ]]; then
    mark_fail "Package ${package} resolves to old repo: ${prefix}"
    return 1
  fi
  mark_ok "package ${package} -> ${prefix}"
}

probe_ping() {
  local label="$1"
  local host="$2"
  if ping -c 1 -W 1 "$host" >/dev/null 2>&1; then
    mark_ok "${label} ping ${host}"
  else
    mark_fail "${label} ping ${host}"
  fi
}

parse_tcp_stream() {
  local stream="$1"
  if [[ "$stream" =~ ^tcpcli://([^/:]+):([0-9]+)$ ]]; then
    printf '%s %s\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}"
    return 0
  fi
  return 1
}

probe_tcp_stream() {
  local stream="$1"
  local host_port
  host_port="$(parse_tcp_stream "$stream" || true)"
  if [[ -z "$host_port" ]]; then
    log "SKIP TCP probe for non-tcpcli stream: ${stream}"
    return 0
  fi

  local host port
  read -r host port <<<"$host_port"
  if timeout 2 bash -c "</dev/tcp/${host}/${port}" >/dev/null 2>&1; then
    mark_ok "Fixposition TCP ${host}:${port}"
  else
    mark_fail "Fixposition TCP ${host}:${port}"
  fi
}

probe_can() {
  if ! ip -details link show "$CAN_INTERFACE" >"$RUN_DIR/${CAN_INTERFACE}_link.txt" 2>&1; then
    mark_fail "CAN interface ${CAN_INTERFACE} not found"
    return 0
  fi
  if grep -q "state UP" "$RUN_DIR/${CAN_INTERFACE}_link.txt" &&
    grep -q "bitrate ${CAN_BAUDRATE:-500000}" "$RUN_DIR/${CAN_INTERFACE}_link.txt"; then
    mark_ok "${CAN_INTERFACE} UP at ${CAN_BAUDRATE:-500000}"
  else
    mark_fail "${CAN_INTERFACE} is not UP at ${CAN_BAUDRATE:-500000}"
  fi

  if command -v candump >/dev/null 2>&1; then
    timeout 2 bash -c "candump -L '$CAN_INTERFACE' 2>/dev/null | head -n 5" \
      >"$RUN_DIR/${CAN_INTERFACE}_frames.txt" || true
    if [[ -s "$RUN_DIR/${CAN_INTERFACE}_frames.txt" ]]; then
      mark_ok "${CAN_INTERFACE} received raw chassis CAN frames"
    else
      mark_fail "${CAN_INTERFACE} received no CAN frames within 2s"
    fi
  else
    log "SKIP candump: can-utils is not installed"
  fi
}

check_topic_info() {
  local topic="$1"
  local label="$2"
  local output="$RUN_DIR/topic_info_${label}.txt"
  local attempt
  for attempt in 1 2 3 4 5; do
    if ros2 topic info "$topic" >"$output" 2>&1; then
      mark_ok "topic exists ${topic}"
      return 0
    fi
    sleep 1
  done
  mark_fail "topic missing ${topic}"
}

check_topic_sample() {
  local topic="$1"
  local label="$2"
  local field="${3:-}"
  local output="$RUN_DIR/sample_${label}.yaml"
  local cmd=(ros2 topic echo "$topic" --once)
  if [[ -n "$field" ]]; then
    cmd+=(--field "$field")
  fi

  if timeout "$SAMPLE_TIMEOUT_SEC" "${cmd[@]}" >"$output" 2>&1 && [[ -s "$output" ]]; then
    mark_ok "sample ${topic}"
  else
    mark_fail "no sample from ${topic} within ${SAMPLE_TIMEOUT_SEC}s"
  fi
}

check_topic_sample_optional() {
  local topic="$1"
  local label="$2"
  local field="${3:-}"
  local output="$RUN_DIR/sample_${label}.yaml"
  local cmd=(ros2 topic echo "$topic" --once)
  if [[ -n "$field" ]]; then
    cmd+=(--field "$field")
  fi

  if timeout "$SAMPLE_TIMEOUT_SEC" "${cmd[@]}" >"$output" 2>&1 && [[ -s "$output" ]]; then
    mark_ok "sample ${topic}"
  else
    mark_warn "optional sample missing ${topic} within ${SAMPLE_TIMEOUT_SEC}s"
  fi
}

check_topic_hz() {
  local topic="$1"
  local label="$2"
  local output="$RUN_DIR/hz_${label}.txt"
  timeout "$HZ_TIMEOUT_SEC" ros2 topic hz "$topic" >"$output" 2>&1 || true
  if grep -q "average rate" "$output"; then
    mark_ok "rate ${topic}: $(grep 'average rate' "$output" | tail -n 1)"
  else
    mark_fail "no rate estimate for ${topic} within ${HZ_TIMEOUT_SEC}s"
  fi
}

log "Autoracer Hooke bench verification"
log "run dir: ${RUN_DIR}"
log "old underlay guard: ${AUTORACER_OLD_REPO:-/home/corage/workspace/project/pilot-auto.x1}"
log ""
log "Preflight"
probe_ping "LiDAR" "${LIDAR_SENSOR_IP:-192.168.1.130}"
if is_true "$LAUNCH_FIXPOSITION"; then
  probe_ping "Fixposition" "${FIXPOSITION_IP:-192.168.1.200}"
  probe_tcp_stream "${FIXPOSITION_STREAM:-tcpcli://192.168.1.200:21000}"
fi
if is_true "$LAUNCH_VEHICLE"; then
  probe_can
fi

if ! is_true "$RUN_LAUNCH"; then
  log ""
  log "RUN_LAUNCH=false, skipped ROS driver launch."
  exit "$((FAIL_COUNT == 0 ? 0 : 1))"
fi

log ""
log "Package resolution"
require_pkg autoracer_bringup || true
require_pkg autoracer_description || true
if is_true "$LAUNCH_LIDAR"; then
  require_pkg nebula_hesai || true
fi
if is_true "$LAUNCH_FIXPOSITION"; then
  require_pkg fixposition_driver_ros2 || true
  require_pkg autoracer_sensing || true
fi
if is_true "$LAUNCH_VEHICLE"; then
  require_pkg can_driver || true
  require_pkg hooke2_interface || true
fi

if (( FAIL_COUNT > 0 )); then
  log ""
  log "Pre-launch checks failed; fix these before starting ROS drivers."
  exit 1
fi

log ""
log "Starting ROS bench launch"
setsid ros2 launch autoracer_bringup bench_verification.launch.py \
  launch_lidar:="$LAUNCH_LIDAR" \
  launch_fixposition:="$LAUNCH_FIXPOSITION" \
  launch_vehicle:="$LAUNCH_VEHICLE" \
  launch_rviz:="$LAUNCH_RVIZ" \
  lidar_host_ip:="${LIDAR_HOST_IP:-192.168.1.120}" \
  lidar_sensor_ip:="${LIDAR_SENSOR_IP:-192.168.1.130}" \
  lidar_data_port:="${LIDAR_DATA_PORT:-2368}" \
  lidar_sensor_model:="${LIDAR_SENSOR_MODEL:-Pandar40P}" \
  fixposition_stream:="${FIXPOSITION_STREAM:-tcpcli://192.168.1.200:21000}" \
  can_channel_id:="${CAN_CHANNEL_ID:-0}" \
  can_baudrate:="${CAN_BAUDRATE:-500000}" \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID="$!"

sleep 3
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  mark_fail "bench launch exited early; see ${RUN_DIR}/launch.log"
  exit 1
fi

log "Waiting ${WARMUP_SEC}s for sensors and ROS graph discovery..."
sleep "$WARMUP_SEC"

log ""
log "ROS topic checks"
if is_true "$LAUNCH_LIDAR"; then
  check_topic_info /sensing/lidar/concatenated/pointcloud lidar_pointcloud
  check_topic_hz /sensing/lidar/concatenated/pointcloud lidar_pointcloud
  check_topic_sample /sensing/lidar/concatenated/pointcloud lidar_pointcloud_header header
fi

if is_true "$LAUNCH_FIXPOSITION"; then
  check_topic_sample /fixposition/fix fix
  check_topic_sample /fixposition/rawimu rawimu
  check_topic_sample /fixposition/autoware_orientation autoware_orientation
  check_topic_sample /fixposition/odometry_enu odometry_enu
  check_topic_sample /fixposition/speed speed
  check_topic_info /fixposition/fpa/odomstatus odomstatus
  check_topic_sample_optional /fixposition/fpa/odomstatus odomstatus
fi

if is_true "$LAUNCH_VEHICLE"; then
  check_topic_sample /vehicle/status/velocity_status velocity_status
  check_topic_sample /vehicle/status/steering_status steering_status
  check_topic_sample /vehicle/status/gear_status gear_status
  check_topic_sample /vehicle/status/control_mode control_mode
  check_topic_sample /hooke2/wheel_speed_rpt hooke2_wheel_speed
  check_topic_sample /hooke2/steering_rpt hooke2_steering
  check_topic_sample /hooke2/global_rpt hooke2_global
fi

log ""
if (( FAIL_COUNT == 0 )); then
  log "PASS bench verification completed."
else
  log "FAIL bench verification completed with ${FAIL_COUNT} failed check(s)."
fi
log "Artifacts: ${RUN_DIR}"
exit "$((FAIL_COUNT == 0 ? 0 : 1))"
