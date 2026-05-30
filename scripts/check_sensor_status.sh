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

LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.1.120}"
LIDAR_SENSOR_IP="${LIDAR_SENSOR_IP:-192.168.1.130}"
LIDAR_DATA_PORT="${LIDAR_DATA_PORT:-2368}"
LIDAR_SENSOR_MODEL="${LIDAR_SENSOR_MODEL:-Pandar40P}"
FIXPOSITION_IP="${FIXPOSITION_IP:-192.168.1.200}"
FIXPOSITION_STREAM="${FIXPOSITION_STREAM:-tcpcli://${FIXPOSITION_IP}:21000}"
CAN_CHANNEL_ID="${CAN_CHANNEL_ID:-0}"
CAN_BAUDRATE="${CAN_BAUDRATE:-500000}"
CAN_INTERFACE="${CAN_INTERFACE:-can${CAN_CHANNEL_ID}}"
WARMUP_SEC="${WARMUP_SEC:-12}"
SAMPLE_TIMEOUT_SEC="${SAMPLE_TIMEOUT_SEC:-8}"
HZ_TIMEOUT_SEC="${HZ_TIMEOUT_SEC:-8}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${RUN_DIR:-$ROOT_DIR/log/sensor_status_${STAMP}}"
SUMMARY_FILE="$RUN_DIR/summary.txt"
mkdir -p "$RUN_DIR"

FAIL_COUNT=0
WARN_COUNT=0
LAUNCH_PID=""

log() {
  printf '%s\n' "$*" | tee -a "$SUMMARY_FILE"
}

ok() {
  log "[ OK ] $*"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  log "[WARN] $*"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  log "[FAIL] $*"
}

collect_launched_pids() {
  {
    pgrep -f "autoracer_bringup bench_verification.launch.py" || true
    pgrep -f "fixposition_driver_ros2_exec" || true
    pgrep -f "velocity_to_fixposition_speed" || true
    pgrep -f "component_container_mt.*autoracer_lidar_container" || true
    pgrep -f "hooke2_interface/lib/hooke2_interface/hooke2_interface" || true
    pgrep -f "hooke2_diag_publisher" || true
    pgrep -f "can_driver/lib/can_driver/can_driver" || true
    pgrep -f "static_tf_.*base_link" || true
    pgrep -f "static_tf_lidar_top_base_link_to_lidar_top" || true
  } | sort -u | while read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "${BASHPID}" ]] && continue
    echo "$pid"
  done
}

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "-${LAUNCH_PID}" >/dev/null 2>&1 || kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "-${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi

  local pids
  pids="$(collect_launched_pids | tr '\n' ' ')"
  if [[ -n "${pids// }" ]]; then
    # shellcheck disable=SC2086
    kill -TERM ${pids} 2>/dev/null || true
    sleep 1
    pids="$(collect_launched_pids | tr '\n' ' ')"
    if [[ -n "${pids// }" ]]; then
      # shellcheck disable=SC2086
      kill -KILL ${pids} 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT

ping_sensor() {
  local label="$1"
  local host="$2"
  if ping -c 1 -W 1 "$host" >"$RUN_DIR/ping_${label}.txt" 2>&1; then
    ok "${label} ${host} ping 通"
  else
    fail "${label} ${host} ping 不通"
  fi
}

probe_can_interface() {
  if ! ip -details link show "$CAN_INTERFACE" >"$RUN_DIR/${CAN_INTERFACE}_link.txt" 2>&1; then
    fail "底盘 CAN 接口 ${CAN_INTERFACE} 不存在"
    return 1
  fi

  if grep -q "state UP" "$RUN_DIR/${CAN_INTERFACE}_link.txt" &&
    grep -q "bitrate ${CAN_BAUDRATE}" "$RUN_DIR/${CAN_INTERFACE}_link.txt"; then
    ok "底盘 CAN 接口 ${CAN_INTERFACE} UP，bitrate=${CAN_BAUDRATE}"
    return 0
  fi

  fail "底盘 CAN 接口 ${CAN_INTERFACE} 未 UP 或 bitrate 不是 ${CAN_BAUDRATE}"
  return 1
}

require_pkg() {
  local package="$1"
  local prefix
  prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  if [[ -z "$prefix" ]]; then
    fail "ROS package 缺失：${package}"
    return 1
  fi
  if [[ "$prefix" == "${AUTORACER_OLD_REPO:-/home/corage/workspace/project/pilot-auto.x1}"* ]]; then
    fail "Package ${package} 解析到了旧仓：${prefix}"
    return 1
  fi
  ok "package ${package} -> ${prefix}"
}

topic_exists() {
  local topic="$1"
  local label="$2"
  local output="$RUN_DIR/topic_info_${label}.txt"
  local attempt
  for attempt in 1 2 3 4 5; do
    if ros2 topic info "$topic" >"$output" 2>&1; then
      ok "topic 存在 ${topic}"
      return 0
    fi
    sleep 1
  done
  fail "topic 不存在 ${topic}"
  return 1
}

topic_hz() {
  local topic="$1"
  local label="$2"
  local output="$RUN_DIR/hz_${label}.txt"
  timeout "$HZ_TIMEOUT_SEC" ros2 topic hz "$topic" >"$output" 2>&1 || true
  if grep -q "average rate" "$output"; then
    ok "topic 频率 ${topic}: $(grep 'average rate' "$output" | tail -n 1)"
    return 0
  fi
  fail "${topic} 在 ${HZ_TIMEOUT_SEC}s 内没有频率估计"
  return 1
}

topic_sample() {
  local topic="$1"
  local label="$2"
  local field="${3:-}"
  local output="$RUN_DIR/sample_${label}.yaml"
  local cmd=(ros2 topic echo "$topic" --once)
  if [[ -n "$field" ]]; then
    cmd+=(--field "$field")
  fi

  if timeout "$SAMPLE_TIMEOUT_SEC" "${cmd[@]}" >"$output" 2>&1 && [[ -s "$output" ]]; then
    ok "topic 有样本 ${topic}"
    sed -n '1,80p' "$output" | tee -a "$SUMMARY_FILE"
    return 0
  fi

  fail "${topic} 在 ${SAMPLE_TIMEOUT_SEC}s 内没有样本"
  return 1
}

fixposition_quality_hint() {
  local fix_sample="$RUN_DIR/sample_fix.yaml"
  local rawimu_sample="$RUN_DIR/sample_rawimu.yaml"
  local orientation_sample="$RUN_DIR/sample_autoware_orientation.yaml"

  if [[ ! -s "$fix_sample" ]]; then
    return 0
  fi

  local latitude longitude altitude status
  latitude="$(awk '/^latitude:/ {print $2; exit}' "$fix_sample")"
  longitude="$(awk '/^longitude:/ {print $2; exit}' "$fix_sample")"
  altitude="$(awk '/^altitude:/ {print $2; exit}' "$fix_sample")"
  status="$(awk '/^[[:space:]]+status:/ {print $2; exit}' "$fix_sample")"

  if [[ "${latitude:-}" == "0.0" && "${longitude:-}" == "0.0" && "${altitude:-}" == "0.0" ]]; then
    warn "Fixposition 话题在流动，但 NavSatFix 位置为 0/0/0。若这是室内测试，GNSS 未初始化通常正常；若是室外开阔环境，则组合导航状态异常，需要检查天线、配置或差分。"
    return 0
  fi

  if [[ -n "${status:-}" && "${status}" != "0" ]]; then
    warn "Fixposition NavSatFix status=${status}，数据在流动但定位质量需要进一步确认。"
    return 0
  fi

  if [[ -s "$rawimu_sample" && -s "$orientation_sample" ]]; then
    ok "Fixposition 数据流和基础定位字段看起来正常。"
  fi
}

chassis_feedback_hint() {
  ok "底盘反馈话题均有样本。静止检查时速度、转角、加速度等字段接近 0 属正常；本检查关注反馈链路是否有数据。"
}

log "LiDAR + Fixposition + 底盘反馈状态检查"
log "run dir: ${RUN_DIR}"
log ""
log "1) 硬件连接检查"
ping_sensor "lidar" "$LIDAR_SENSOR_IP"
ping_sensor "fixposition" "$FIXPOSITION_IP"
probe_can_interface || true

if (( FAIL_COUNT == 0 )); then
  log "[ OK ] 连接状态正常：LiDAR ${LIDAR_SENSOR_IP} 和 Fixposition ${FIXPOSITION_IP} 都可 ping 通，${CAN_INTERFACE} 可用。"
else
  log "[FAIL] 连接状态异常：请先检查网线、电源、IP 网段、CAN 接口和设备配置。"
  log "Artifacts: ${RUN_DIR}"
  exit 1
fi

log ""
log "2) ROS package 检查"
require_pkg autoracer_bringup || true
require_pkg nebula_hesai || true
require_pkg fixposition_driver_ros2 || true
require_pkg can_driver || true
require_pkg hooke2_interface || true

if (( FAIL_COUNT > 0 )); then
  log "[FAIL] ROS package 检查失败，跳过 launch。"
  log "Artifacts: ${RUN_DIR}"
  exit 1
fi

log ""
log "3) 启动 LiDAR + Fixposition + Hooke2 底盘反馈 launch/node"
setsid ros2 launch autoracer_bringup bench_verification.launch.py \
  launch_static_tf:=true \
  launch_lidar:=true \
  launch_fixposition:=true \
  launch_vehicle:=true \
  launch_rviz:=false \
  lidar_host_ip:="$LIDAR_HOST_IP" \
  lidar_sensor_ip:="$LIDAR_SENSOR_IP" \
  lidar_data_port:="$LIDAR_DATA_PORT" \
  lidar_sensor_model:="$LIDAR_SENSOR_MODEL" \
  fixposition_stream:="$FIXPOSITION_STREAM" \
  can_channel_id:="$CAN_CHANNEL_ID" \
  can_baudrate:="$CAN_BAUDRATE" \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID="$!"

sleep 3
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  fail "bench launch 提前退出，请查看 ${RUN_DIR}/launch.log"
  log "Artifacts: ${RUN_DIR}"
  exit 1
fi

log "等待 ${WARMUP_SEC}s 让传感器和 ROS graph 稳定..."
sleep "$WARMUP_SEC"

log ""
log "4) ROS node/topic 列表"
ros2 node list | sort >"$RUN_DIR/node_list.txt"
ros2 topic list | sort >"$RUN_DIR/topic_list.txt"
sed -n '1,120p' "$RUN_DIR/node_list.txt" | tee -a "$SUMMARY_FILE"
log "--- sensor/chassis topics ---"
grep -E '^/(fixposition|sensing/lidar|vehicle/status|hooke2)' "$RUN_DIR/topic_list.txt" |
  tee -a "$SUMMARY_FILE" || true

log ""
log "5) LiDAR 点云检查"
topic_exists /sensing/lidar/concatenated/pointcloud lidar_pointcloud || true
topic_hz /sensing/lidar/concatenated/pointcloud lidar_pointcloud || true
topic_sample /sensing/lidar/concatenated/pointcloud lidar_pointcloud_header header || true

log ""
log "6) Fixposition 组合导航话题检查"
topic_sample /fixposition/fix fix || true
topic_sample /fixposition/rawimu rawimu || true
topic_sample /fixposition/autoware_orientation autoware_orientation || true
topic_sample /fixposition/odometry_enu odometry_enu || true
fixposition_quality_hint

log ""
log "7) Hooke2 底盘反馈检查"
topic_sample /vehicle/status/velocity_status vehicle_velocity_status || true
topic_sample /vehicle/status/steering_status vehicle_steering_status || true
topic_sample /vehicle/status/gear_status vehicle_gear_status || true
topic_sample /vehicle/status/control_mode vehicle_control_mode || true
topic_sample /hooke2/wheel_speed_rpt hooke2_wheel_speed || true
topic_sample /hooke2/steering_rpt hooke2_steering || true
topic_sample /hooke2/global_rpt hooke2_global || true
if [[ -s "$RUN_DIR/sample_vehicle_velocity_status.yaml" &&
  -s "$RUN_DIR/sample_vehicle_steering_status.yaml" ]]; then
  chassis_feedback_hint
fi

log ""
if (( FAIL_COUNT == 0 )); then
  if (( WARN_COUNT == 0 )); then
    log "传感器与底盘反馈测试通过。LiDAR 点云、Fixposition 数据和 Hooke2 底盘反馈均正常发布。"
  else
    log "传感器与底盘反馈测试通过，但有 ${WARN_COUNT} 条 Fixposition 数据质量提示。"
  fi
  result=0
else
  log "传感器或底盘反馈状态异常：有 ${FAIL_COUNT} 项检查失败。"
  result=1
fi
log "Artifacts: ${RUN_DIR}"
exit "$result"
