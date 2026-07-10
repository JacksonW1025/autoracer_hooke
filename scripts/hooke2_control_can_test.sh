#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${ROOT_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

IFACE="${IFACE:-can0}"
BITRATE="${BITRATE:-500000}"
CAN_RX_TOPIC="${CAN_RX_TOPIC:-/can_rx_from_autoware}"
CAN_TX_TOPIC="${CAN_TX_TOPIC:-/can_tx_to_autoware}"
CONTROL_TOPIC="${CONTROL_TOPIC:-/control/command/control_cmd}"
GEAR_TOPIC="${GEAR_TOPIC:-/control/command/gear_cmd}"
TURN_TOPIC="${TURN_TOPIC:-/control/command/turn_indicators_cmd}"
HAZARD_TOPIC="${HAZARD_TOPIC:-/control/command/hazard_lights_cmd}"
EMERGENCY_TOPIC="${EMERGENCY_TOPIC:-/control/command/emergency_cmd}"
CONTROL_MODE_SERVICE="${CONTROL_MODE_SERVICE:-/control/control_mode_request}"
READY_TOPIC="${READY_TOPIC:-/vehicle/status/steering_status}"

STEER_RATIO="${STEER_RATIO:-16.0}"
VEHICLE_INFO_MAX_TIRE_STEER_RAD="${VEHICLE_INFO_MAX_TIRE_STEER_RAD:-0.488}"
CAN_STEER_WHEEL_MAX_DEG="${CAN_STEER_WHEEL_MAX_DEG:-440}"
MAX_SPEED_MPS="${MAX_SPEED_MPS:-5.0}"
PUB_RATE_HZ="${PUB_RATE_HZ:-20}"
ACTION_SECONDS="${ACTION_SECONDS:-5}"
STOP_SECONDS="${STOP_SECONDS:-3}"
STOP_DECEL_MPS2="${STOP_DECEL_MPS2:--0.60}"
TEST_ACCEL_MPS2="${TEST_ACCEL_MPS2:-0.20}"
STEER_SPEED_DEG_PER_SEC="${STEER_SPEED_DEG_PER_SEC:-80}"
PERIOD_SECONDS="${PERIOD_SECONDS:-0.05}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-20}"
FEEDBACK_TIMEOUT_SEC="${FEEDBACK_TIMEOUT_SEC:-30}"
PUB_ONCE_TIMEOUT_SEC="${PUB_ONCE_TIMEOUT_SEC:-8}"
COUNTDOWN_SEC="${COUNTDOWN_SEC:-5}"
WAIT_FOR_CAN_FEEDBACK="${WAIT_FOR_CAN_FEEDBACK:-1}"
LAUNCH_INTERFACE="${LAUNCH_INTERFACE:-auto}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-autoracer_bringup}"
LAUNCH_FILE="${LAUNCH_FILE:-vehicle.launch.py}"
KEEP_LAUNCH_ALIVE="${KEEP_LAUNCH_ALIVE:-0}"
SHOW_LAUNCH_OUTPUT="${SHOW_LAUNCH_OUTPUT:-0}"
KEEP_CAN_UP="${KEEP_CAN_UP:-0}"
LAUNCH_LOG_FILE="${LAUNCH_LOG_FILE:-/tmp/hooke2_control_can_test_$(date +%Y%m%d_%H%M%S).log}"

GEAR_DRIVE=2
TURN_DISABLE=1
HAZARD_DISABLE=1
MODE_AUTONOMOUS=1
MODE_MANUAL=4

FRAME_MODE_SPEED_DRIVE="105#0001000000000000"
FRAME_BRAKE_ENABLE_ZERO="101#0119000000000000"
FRAME_BRAKE_DISABLE_RESET="101#0019000000000000"
FRAME_GEAR_DRIVE="103#0104000000000000"
FRAME_PARK_RELEASE="104#0100000000000000"

LAUNCH_PID=""
MONITOR_PID=""
MONITOR_FILE=""
CAN_STARTED_BY_SCRIPT="0"
ACTIVE_PATH="none"
STOP_SENT="0"
SETUP_DONE="0"
SPEED_MPS=""
TIRE_DEG=""
TIRE_RAD=""
METHOD=""

log() {
  echo "[hooke2-control-can] $*"
}

warn() {
  echo "[hooke2-control-can] WARN: $*" >&2
}

fail() {
  echo "[hooke2-control-can] FAIL: $*" >&2
  exit 1
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    fail "缺少命令: $1"
  fi
}

is_number() {
  [[ "$1" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

calc() {
  awk "BEGIN { printf \"%.10f\", $* }"
}

calc3() {
  awk "BEGIN { printf \"%.3f\", $* }"
}

lt() {
  awk -v a="$1" -v b="$2" 'BEGIN { exit !(a < b) }'
}

gt() {
  awk -v a="$1" -v b="$2" 'BEGIN { exit !(a > b) }'
}

abs_gt() {
  awk -v a="$1" -v b="$2" 'BEGIN { if (a < 0) a = -a; exit !(a > b) }'
}

round_int() {
  awk -v v="$1" 'BEGIN { printf "%.0f", v }'
}

normalize_int() {
  if [[ "$1" == "-0" ]]; then
    printf '0'
  else
    printf '%s' "$1"
  fi
}

trunc_int() {
  awk -v v="$1" 'BEGIN { printf "%d", v }'
}

require_integer() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    fail "${name} 必须是非负整数，当前为 '${value}'。"
  fi
}

clamp_float() {
  awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    printf "%.10f", v;
  }'
}

clamp_int() {
  awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    printf "%d", v;
  }'
}

deg_to_rad() {
  awk -v d="$1" 'BEGIN { printf "%.10f", d * atan2(0, -1) / 180.0 }'
}

effective_max_tire_deg() {
  awk \
    -v ratio="${STEER_RATIO}" \
    -v can_max="${CAN_STEER_WHEEL_MAX_DEG}" \
    -v info_rad="${VEHICLE_INFO_MAX_TIRE_STEER_RAD}" \
    'BEGIN {
      can_deg = can_max / ratio;
      info_deg = info_rad * 180.0 / atan2(0, -1);
      max_deg = can_deg < info_deg ? can_deg : info_deg;
      printf "%.6f", max_deg;
    }'
}

effective_max_tire_rad() {
  awk \
    -v ratio="${STEER_RATIO}" \
    -v can_max="${CAN_STEER_WHEEL_MAX_DEG}" \
    -v info_rad="${VEHICLE_INFO_MAX_TIRE_STEER_RAD}" \
    'BEGIN {
      can_rad = (can_max / ratio) * atan2(0, -1) / 180.0;
      max_rad = can_rad < info_rad ? can_rad : info_rad;
      printf "%.6f", max_rad;
    }'
}

source_workspace() {
  if [[ "${SETUP_DONE}" == "1" ]]; then
    return 0
  fi

  set +u
  # shellcheck source=scripts/ros_env.sh
  source "${ROOT_DIR}/scripts/ros_env.sh"
  set -u
  SETUP_DONE="1"
}

can_state_line() {
  ip -details link show "${IFACE}" 2>/dev/null | awk '/can </ {gsub(/^ +/, ""); print; exit}'
}

tx_packets() {
  ip -s link show dev "${IFACE}" 2>/dev/null | awk '/TX:/{getline; print $2; exit}'
}

bring_up_can() {
  need_cmd ip
  need_cmd sudo

  if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    echo "未找到 ${IFACE}。当前 CAN 接口:"
    ip -o link show | awk -F': ' '/can[0-9]+/ {print "  " $2}'
    return 1
  fi

  if ip -details link show "${IFACE}" | grep -q "state UP" &&
    ip -details link show "${IFACE}" | grep -q "bitrate ${BITRATE}"; then
    log "${IFACE} 已经 UP 且波特率为 ${BITRATE}。"
    return 0
  fi

  log "启动 ${IFACE}，bitrate=${BITRATE} ..."
  sudo ip link set "${IFACE}" down >/dev/null 2>&1 || true
  sudo ip link set "${IFACE}" type can bitrate "${BITRATE}" loopback off listen-only off fd off restart-ms 100
  sudo ip link set "${IFACE}" txqueuelen 100 || true
  sudo ip link set "${IFACE}" up
  CAN_STARTED_BY_SCRIPT="1"
  sleep 0.2
}

check_can_feedback() {
  if [[ "${WAIT_FOR_CAN_FEEDBACK}" != "1" ]]; then
    return 0
  fi

  need_cmd candump
  need_cmd timeout

  log "检查 ${IFACE} 是否有底盘上行 CAN 帧，最多等待 2 秒 ..."
  local line
  line="$(timeout 2 candump -L "${IFACE}" 2>/dev/null | head -n 1 || true)"
  if [[ -n "${line}" ]]; then
    log "检测到 CAN 帧: ${line}"
    return 0
  fi

  warn "没有检测到 ${IFACE} 上行帧。当前状态: $(can_state_line)"
  warn "请确认 CAN-H/CAN-L/GND 已接好，或用 WAIT_FOR_CAN_FEEDBACK=0 跳过这一步。"
  return 1
}

ensure_can_ready() {
  bring_up_can
  check_can_feedback
}

send_frame() {
  cansend "${IFACE}" "$1" >/dev/null
}

speed_frame() {
  local speed="$1"
  local enable="$2"
  local clamped
  local raw
  local byte5
  local byte6

  clamped="$(clamp_float "${speed}" "0.0" "10.23")"
  raw="$(round_int "$(calc "${clamped} * 25.0")")"
  byte5=$(((raw >> 2) & 0xFF))
  byte6=$(((raw & 0x03) << 6))
  printf '100#%02X00000000%02X%02X00' "${enable}" "${byte5}" "${byte6}"
}

steer_frame() {
  local target_deg="$1"
  local speed_deg="$2"
  local enable="$3"
  local target
  local speed
  local raw
  local hi
  local lo

  target="$(clamp_int "${target_deg}" "-${CAN_STEER_WHEEL_MAX_DEG}" "${CAN_STEER_WHEEL_MAX_DEG}")"
  speed="$(clamp_int "${speed_deg}" "0" "250")"
  raw=$((target + 500))
  hi=$(((raw >> 8) & 0xFF))
  lo=$((raw & 0xFF))
  printf '102#%02X%02X00%02X%02X000000' "${enable}" "${speed}" "${hi}" "${lo}"
}

raw_can_wheel_deg_from_tire_deg() {
  local tire_deg="$1"
  local wheel
  local rounded
  wheel="$(calc "-1.0 * (${tire_deg}) * ${STEER_RATIO}")"
  rounded="$(round_int "${wheel}")"
  normalize_int "${rounded}"
}

control_can_wheel_deg_from_tire_rad() {
  local tire_rad="$1"
  local wheel_rad
  local wheel_deg
  wheel_rad="$(clamp_float "$(calc "-1.0 * (${tire_rad}) * ${STEER_RATIO}")" "-8.72" "8.72")"
  wheel_deg="$(calc "${wheel_rad} * 57.0")"
  trunc_int "${wheel_deg}"
}

print_semantics() {
  local info_deg
  local can_deg
  local eff_deg
  local eff_rad

  info_deg="$(calc3 "${VEHICLE_INFO_MAX_TIRE_STEER_RAD} * 180.0 / atan2(0, -1)")"
  can_deg="$(calc3 "${CAN_STEER_WHEEL_MAX_DEG} / ${STEER_RATIO}")"
  eff_deg="$(calc3 "$(effective_max_tire_deg)")"
  eff_rad="$(effective_max_tire_rad)"

  cat <<EOF

Hooke2 控制/CAN 单次驱动验证

已按当前代码确认的单位和含义：
  - Autoware Control: lateral.steering_tire_angle 是轮胎转角，单位 rad，正值左转，负值右转。
  - hooke2_interface: 将轮胎角 rad 按 ${STEER_RATIO}:1 转成方向盘角，并取反后写入 CAN 0x102。
  - CAN 0x102: Steer_ANGLE_Target 是方向盘角，单位 deg，协议限制约 -${CAN_STEER_WHEEL_MAX_DEG} 到 +${CAN_STEER_WHEEL_MAX_DEG} deg。
  - 车辆配置 max_steer_angle=${VEHICLE_INFO_MAX_TIRE_STEER_RAD} rad (${info_deg} deg)；
    受 CAN 方向盘角限制，本脚本实际接受轮胎角范围为 -${eff_deg} 到 +${eff_deg} deg (${eff_rad} rad)。

安全要求：
  1. 车辆周围无人，车轮和转向机构附近无人。
  2. 急停可用，安全员在场。
  3. 本测试会发送挡位 D、驻车释放、低速速度/转向命令。
EOF
}

read_number() {
  local prompt="$1"
  local value

  while true; do
    if ! read -r -p "${prompt}" value; then
      fail "未读取到输入，取消测试。"
    fi
    if is_number "${value}"; then
      printf '%s' "${value}"
      return 0
    fi
    echo "请输入数字。" >&2
  done
}

collect_inputs() {
  local max_tire_deg
  local max_tire_deg_show
  max_tire_deg="$(effective_max_tire_deg)"
  max_tire_deg_show="$(calc3 "${max_tire_deg}")"

  if [[ -n "${TEST_SPEED_MPS:-}" ]]; then
    SPEED_MPS="${TEST_SPEED_MPS}"
  else
    SPEED_MPS="$(read_number "请输入目标速度 m/s [0-${MAX_SPEED_MPS}]: ")"
  fi

  while lt "${SPEED_MPS}" "0.0" || gt "${SPEED_MPS}" "${MAX_SPEED_MPS}"; do
    echo "速度超出范围，请输入 0 到 ${MAX_SPEED_MPS} m/s。"
    SPEED_MPS="$(read_number "请输入目标速度 m/s [0-${MAX_SPEED_MPS}]: ")"
  done

  if [[ -n "${TEST_TIRE_STEER_DEG:-}" ]]; then
    TIRE_DEG="${TEST_TIRE_STEER_DEG}"
  else
    TIRE_DEG="$(read_number "请输入轮胎转角 deg，正左负右 [-${max_tire_deg_show}, +${max_tire_deg_show}]: ")"
  fi

  while abs_gt "${TIRE_DEG}" "${max_tire_deg}"; do
    echo "轮胎转角超出范围；当前限制为 +/- ${max_tire_deg_show} deg。"
    TIRE_DEG="$(read_number "请输入轮胎转角 deg，正左负右 [-${max_tire_deg_show}, +${max_tire_deg_show}]: ")"
  done

  TIRE_RAD="$(deg_to_rad "${TIRE_DEG}")"

  if [[ -n "${TEST_METHOD:-}" ]]; then
    METHOD="${TEST_METHOD}"
  else
    echo
    echo "请选择驱动路径："
    echo "  1) 发布 Autoware Control 话题，验证 hooke2_interface -> ${CAN_RX_TOPIC} -> can_driver"
    echo "  2) 直接发送原始 CAN 帧到底盘，绕过 ROS 控制链路"
    read -r -p "输入选项 [1/2]: " METHOD
  fi

  case "${METHOD}" in
    1|control|Control|topic|Topic)
      METHOD="control"
      ;;
    2|raw|Raw|can|CAN)
      METHOD="raw"
      ;;
    *)
      fail "无效驱动路径: ${METHOD}"
      ;;
  esac
}

confirm_arm() {
  local raw_wheel_deg
  local control_wheel_deg
  local raw_steer
  local control_steer
  local speed

  raw_wheel_deg="$(raw_can_wheel_deg_from_tire_deg "${TIRE_DEG}")"
  control_wheel_deg="$(control_can_wheel_deg_from_tire_rad "${TIRE_RAD}")"
  raw_steer="$(steer_frame "${raw_wheel_deg}" "${STEER_SPEED_DEG_PER_SEC}" 1)"
  control_steer="$(steer_frame "${control_wheel_deg}" 220 1)"
  speed="$(speed_frame "${SPEED_MPS}" 1)"

  echo
  echo "本次命令预览："
  echo "  速度命令: ${SPEED_MPS} m/s"
  echo "  轮胎角命令: ${TIRE_DEG} deg (${TIRE_RAD} rad)，正左负右"
  echo "  原始 CAN 路径方向盘角: ${raw_wheel_deg} deg，steer frame=${raw_steer}"
  echo "  Control 路径按当前 hooke2_interface 预计方向盘角: ${control_wheel_deg} deg，steer frame≈${control_steer}"
  echo "  speed frame=${speed}"
  echo "  持续发送 ${ACTION_SECONDS}s，随后发送 ${STOP_SECONDS}s 速度 0/回正/退出自动。"
  echo

  local arm
  read -r -p "确认安全条件满足后，输入 ARM 开始: " arm
  if [[ "${arm}" != "ARM" ]]; then
    fail "未确认 ARM，取消测试。"
  fi
}

node_exists() {
  local node_name="$1"
  ros2 node list 2>/dev/null | grep -qx "${node_name}"
}

can_driver_exists() {
  ros2 node list 2>/dev/null | grep -Eq '^/(can_driver|can_driver0|can_driver_node)$'
}

wait_for_node() {
  local node_name="$1"
  local timeout_sec="$2"
  local start_time="${SECONDS}"

  log "等待节点 ${node_name} ..."
  while (( SECONDS - start_time < timeout_sec )); do
    if node_exists "${node_name}"; then
      log "节点就绪: ${node_name}"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_can_driver_node() {
  local timeout_sec="$1"
  local start_time="${SECONDS}"

  log "等待 can_driver 节点 ..."
  while (( SECONDS - start_time < timeout_sec )); do
    if can_driver_exists; then
      log "can_driver 节点就绪。"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start_time="${SECONDS}"

  log "等待服务 ${service_name} ..."
  while (( SECONDS - start_time < timeout_sec )); do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      log "服务就绪: ${service_name}"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_subscribers() {
  local topic_name="$1"
  local min_count="$2"
  local timeout_sec="$3"
  local start_time="${SECONDS}"
  local count

  log "等待 ${topic_name} 至少 ${min_count} 个订阅者 ..."
  while (( SECONDS - start_time < timeout_sec )); do
    count="$(ros2 topic info "${topic_name}" 2>/dev/null | awk '/Subscription count:/ {print $3}')"
    if [[ -n "${count}" && "${count}" -ge "${min_count}" ]]; then
      log "Topic 就绪: ${topic_name} 有 ${count} 个订阅者。"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_message() {
  local topic_name="$1"
  local timeout_sec="$2"

  log "等待 ${topic_name} 底盘反馈 ..."
  if timeout "${timeout_sec}" ros2 topic echo --once "${topic_name}" >/dev/null; then
    log "收到反馈: ${topic_name}"
    return 0
  fi
  return 1
}

pub_once() {
  local topic_name="$1"
  local msg_type="$2"
  local msg_yaml="$3"

  timeout "${PUB_ONCE_TIMEOUT_SEC}" \
    ros2 topic pub -p 0 --once -w 1 "${topic_name}" "${msg_type}" "${msg_yaml}" >/dev/null
}

call_control_mode() {
  local mode="$1"
  local label="$2"
  local output

  log "请求控制模式: ${label} (${mode})"
  output="$(
    timeout "${PUB_ONCE_TIMEOUT_SEC}" \
      ros2 service call "${CONTROL_MODE_SERVICE}" \
        autoware_vehicle_msgs/srv/ControlModeCommand \
        "{stamp: {sec: 0, nanosec: 0}, mode: ${mode}}" 2>&1
  )" || {
    echo "${output}" >&2
    return 1
  }

  if grep -Eq "success[=:] ?[Tt]rue|success=True" <<< "${output}"; then
    return 0
  fi

  echo "${output}" >&2
  return 1
}

control_yaml() {
  local velocity="$1"
  local acceleration="$2"
  local steering="$3"
  local steering_rate="$4"

  cat <<EOF
{stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, lateral: {stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, steering_tire_angle: ${steering}, steering_tire_rotation_rate: ${steering_rate}, is_defined_steering_tire_rotation_rate: true}, longitudinal: {stamp: {sec: 0, nanosec: 0}, control_time: {sec: 0, nanosec: 0}, velocity: ${velocity}, acceleration: ${acceleration}, jerk: 0.0, is_defined_acceleration: true, is_defined_jerk: true}}
EOF
}

publish_control_for() {
  local label="$1"
  local velocity="$2"
  local acceleration="$3"
  local steering="$4"
  local steering_rate="$5"
  local duration_sec="$6"
  local times=$((duration_sec * PUB_RATE_HZ))
  local timeout_sec=$((duration_sec + 10))

  if (( times <= 0 )); then
    return 0
  fi

  log "发布 Control: ${label}, velocity=${velocity} m/s, tire_angle=${steering} rad, duration=${duration_sec}s"
  timeout "${timeout_sec}" \
    ros2 topic pub -p 0 -r "${PUB_RATE_HZ}" -t "${times}" \
      "${CONTROL_TOPIC}" autoware_control_msgs/msg/Control \
      "$(control_yaml "${velocity}" "${acceleration}" "${steering}" "${steering_rate}")" >/dev/null
}

publish_static_commands() {
  log "发布辅助命令: gear=D, turn=DISABLE, hazard=DISABLE, emergency=false"
  pub_once "${GEAR_TOPIC}" autoware_vehicle_msgs/msg/GearCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${GEAR_DRIVE}}"
  pub_once "${TURN_TOPIC}" autoware_vehicle_msgs/msg/TurnIndicatorsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${TURN_DISABLE}}"
  pub_once "${HAZARD_TOPIC}" autoware_vehicle_msgs/msg/HazardLightsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${HAZARD_DISABLE}}"
  pub_once "${EMERGENCY_TOPIC}" tier4_vehicle_msgs/msg/VehicleEmergencyStamped \
    "{stamp: {sec: 0, nanosec: 0}, emergency: false}"
}

launch_interface_stack() {
  local should_launch="0"

  case "${LAUNCH_INTERFACE}" in
    0|false|False|no|No)
      log "LAUNCH_INTERFACE=${LAUNCH_INTERFACE}，使用已运行的车辆接口。"
      return 0
      ;;
    1|true|True|yes|Yes)
      should_launch="1"
      ;;
    auto|AUTO)
      if node_exists "/hooke2_interface" && can_driver_exists; then
        log "检测到 /hooke2_interface 和 can_driver 已运行，复用现有进程。"
        return 0
      fi
      should_launch="1"
      ;;
    *)
      fail "LAUNCH_INTERFACE 仅支持 auto/1/0。"
      ;;
  esac

  if [[ "${should_launch}" != "1" ]]; then
    return 0
  fi

  log "启动 ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ..."
  if [[ "${SHOW_LAUNCH_OUTPUT}" == "1" ]]; then
    ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" &
  else
    mkdir -p "$(dirname "${LAUNCH_LOG_FILE}")"
    log "Launch 日志: ${LAUNCH_LOG_FILE}"
    ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" >"${LAUNCH_LOG_FILE}" 2>&1 &
  fi
  LAUNCH_PID="$!"
  sleep 2

  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    wait "${LAUNCH_PID}" || true
    fail "ros2 launch 提前退出。"
  fi
}

countdown() {
  local seconds="$1"
  if (( seconds <= 0 )); then
    return 0
  fi

  log "${seconds}s 后请求 AUTONOMOUS，Ctrl-C 可中止。"
  while (( seconds > 0 )); do
    log "${seconds} ..."
    sleep 1
    seconds=$((seconds - 1))
  done
}

start_can_topic_monitor() {
  local duration_sec="$1"
  MONITOR_FILE="$(mktemp /tmp/hooke2_can_rx_monitor.XXXXXX)"
  timeout "${duration_sec}" ros2 topic echo "${CAN_RX_TOPIC}" can_msgs/msg/Frame >"${MONITOR_FILE}" 2>/dev/null &
  MONITOR_PID="$!"
  sleep 1
}

finish_can_topic_monitor() {
  if [[ -n "${MONITOR_PID}" ]]; then
    wait "${MONITOR_PID}" 2>/dev/null || true
    MONITOR_PID=""
  fi
}

verify_can_topic_monitor() {
  if [[ -z "${MONITOR_FILE}" || ! -f "${MONITOR_FILE}" ]]; then
    warn "没有 CAN topic 监控文件，跳过 topic 帧检查。"
    return 0
  fi

  local ok="1"
  if grep -q '^id: 256$' "${MONITOR_FILE}"; then
    log "已在 ${CAN_RX_TOPIC} 看到 0x100 速度/油门 CAN 帧。"
  else
    warn "未在 ${CAN_RX_TOPIC} 看到 0x100 帧。"
    ok="0"
  fi

  if grep -q '^id: 258$' "${MONITOR_FILE}"; then
    log "已在 ${CAN_RX_TOPIC} 看到 0x102 转向 CAN 帧。"
  else
    warn "未在 ${CAN_RX_TOPIC} 看到 0x102 帧。"
    ok="0"
  fi

  rm -f "${MONITOR_FILE}"
  MONITOR_FILE=""

  [[ "${ok}" == "1" ]]
}

send_control_stop_and_manual() {
  if [[ "${STOP_SENT}" == "1" || "${SETUP_DONE}" != "1" ]]; then
    return 0
  fi
  STOP_SENT="1"

  if ! node_exists "/hooke2_interface"; then
    return 0
  fi

  set +e
  log "发送 Control 停止命令并请求 MANUAL。"
  publish_control_for "Stop" "0.0" "${STOP_DECEL_MPS2}" "0.0" "0.0" "${STOP_SECONDS}"
  pub_once "${TURN_TOPIC}" autoware_vehicle_msgs/msg/TurnIndicatorsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${TURN_DISABLE}}"
  pub_once "${HAZARD_TOPIC}" autoware_vehicle_msgs/msg/HazardLightsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${HAZARD_DISABLE}}"
  pub_once "${EMERGENCY_TOPIC}" tier4_vehicle_msgs/msg/VehicleEmergencyStamped \
    "{stamp: {sec: 0, nanosec: 0}, emergency: false}"
  call_control_mode "${MODE_MANUAL}" "MANUAL"
  set -e
}

stop_launch_if_needed() {
  if [[ "${KEEP_LAUNCH_ALIVE}" == "1" || -z "${LAUNCH_PID}" ]]; then
    return 0
  fi

  log "停止本脚本启动的车辆接口。"
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  for _ in {1..10}; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    pkill -TERM -P "${LAUNCH_PID}" 2>/dev/null || true
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
  LAUNCH_PID=""
}

raw_stop() {
  if [[ "${STOP_SENT}" == "1" ]]; then
    return 0
  fi
  STOP_SENT="1"

  set +e
  local speed_zero
  local speed_disable
  local steer_center
  local steer_disable
  local end_time

  speed_zero="$(speed_frame 0 1)"
  speed_disable="$(speed_frame 0 0)"
  steer_center="$(steer_frame 0 "${STEER_SPEED_DEG_PER_SEC}" 1)"
  steer_disable="$(steer_frame 0 0 0)"

  log "发送原始 CAN 停止帧：速度 0 + 转向回正 ${STOP_SECONDS}s。"
  end_time=$((SECONDS + STOP_SECONDS))
  while (( SECONDS < end_time )); do
    cansend "${IFACE}" "${speed_zero}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${FRAME_BRAKE_ENABLE_ZERO}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${steer_center}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${FRAME_GEAR_DRIVE}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${FRAME_PARK_RELEASE}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${FRAME_MODE_SPEED_DRIVE}" >/dev/null 2>&1 || true
    sleep "${PERIOD_SECONDS}"
  done

  for _ in $(seq 1 10); do
    cansend "${IFACE}" "${speed_disable}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${FRAME_BRAKE_DISABLE_RESET}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${steer_disable}" >/dev/null 2>&1 || true
    sleep "${PERIOD_SECONDS}"
  done
  set -e
}

bring_down_can_if_needed() {
  if [[ "${CAN_STARTED_BY_SCRIPT}" == "1" && "${KEEP_CAN_UP}" != "1" ]]; then
    sudo ip link set "${IFACE}" down >/dev/null 2>&1 || true
    log "${IFACE} 已置 DOWN。"
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  finish_can_topic_monitor

  if [[ "${ACTIVE_PATH}" == "control" ]]; then
    send_control_stop_and_manual || true
    stop_launch_if_needed || true
  elif [[ "${ACTIVE_PATH}" == "raw" ]]; then
    raw_stop || true
  fi

  bring_down_can_if_needed || true
  exit "${status}"
}

run_control_path() {
  need_cmd ros2
  need_cmd timeout
  source_workspace
  ensure_can_ready

  local tx_before
  local tx_after
  tx_before="$(tx_packets || true)"

  launch_interface_stack
  wait_for_node "/hooke2_interface" "${DISCOVERY_TIMEOUT_SEC}" || fail "等待 /hooke2_interface 超时。"
  wait_for_can_driver_node "${DISCOVERY_TIMEOUT_SEC}" || fail "等待 can_driver 超时。"
  wait_for_service "${CONTROL_MODE_SERVICE}" "${DISCOVERY_TIMEOUT_SEC}" || fail "等待控制模式服务超时。"
  wait_for_subscribers "${CONTROL_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${CONTROL_TOPIC} 没有订阅者。"
  wait_for_subscribers "${GEAR_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${GEAR_TOPIC} 没有订阅者。"
  wait_for_subscribers "${TURN_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${TURN_TOPIC} 没有订阅者。"
  wait_for_subscribers "${HAZARD_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${HAZARD_TOPIC} 没有订阅者。"
  wait_for_subscribers "${EMERGENCY_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${EMERGENCY_TOPIC} 没有订阅者。"
  wait_for_subscribers "${CAN_RX_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}" || fail "${CAN_RX_TOPIC} 没有 can_driver 订阅者。"
  wait_for_message "${READY_TOPIC}" "${FEEDBACK_TIMEOUT_SEC}" || fail "没有收到 ${READY_TOPIC}，底盘反馈未就绪。"

  publish_static_commands
  countdown "${COUNTDOWN_SEC}"
  call_control_mode "${MODE_AUTONOMOUS}" "AUTONOMOUS"

  ACTIVE_PATH="control"
  STOP_SENT="0"
  start_can_topic_monitor "$((ACTION_SECONDS + STOP_SECONDS + 3))"
  publish_control_for "User command" "${SPEED_MPS}" "${TEST_ACCEL_MPS2}" "${TIRE_RAD}" "0.10" "${ACTION_SECONDS}"
  send_control_stop_and_manual
  finish_can_topic_monitor
  verify_can_topic_monitor || fail "Control -> CAN topic 检查未通过。"

  tx_after="$(tx_packets || true)"
  if [[ -n "${tx_before}" && -n "${tx_after}" && "${tx_after}" =~ ^[0-9]+$ && "${tx_before}" =~ ^[0-9]+$ ]]; then
    log "${IFACE} TX packets 增量: $((tx_after - tx_before))"
  fi

  stop_launch_if_needed
  ACTIVE_PATH="none"
}

run_raw_path() {
  need_cmd cansend
  need_cmd candump
  need_cmd timeout
  ensure_can_ready

  local wheel_deg
  local speed
  local steer
  local tx_before
  local tx_after
  local end_time

  wheel_deg="$(raw_can_wheel_deg_from_tire_deg "${TIRE_DEG}")"
  speed="$(speed_frame "${SPEED_MPS}" 1)"
  steer="$(steer_frame "${wheel_deg}" "${STEER_SPEED_DEG_PER_SEC}" 1)"
  tx_before="$(tx_packets || true)"

  ACTIVE_PATH="raw"
  STOP_SENT="0"

  log "直接发送原始 CAN，持续 ${ACTION_SECONDS}s。"
  log "  speed=${speed}"
  log "  steer=${steer}"
  log "  brake=${FRAME_BRAKE_ENABLE_ZERO}"
  log "  gear=${FRAME_GEAR_DRIVE}"
  log "  park=${FRAME_PARK_RELEASE}"
  log "  mode=${FRAME_MODE_SPEED_DRIVE}"

  end_time=$((SECONDS + ACTION_SECONDS))
  while (( SECONDS < end_time )); do
    send_frame "${speed}"
    send_frame "${FRAME_BRAKE_ENABLE_ZERO}"
    send_frame "${steer}"
    send_frame "${FRAME_GEAR_DRIVE}"
    send_frame "${FRAME_PARK_RELEASE}"
    send_frame "${FRAME_MODE_SPEED_DRIVE}"
    sleep "${PERIOD_SECONDS}"
  done

  raw_stop
  tx_after="$(tx_packets || true)"
  if [[ -n "${tx_before}" && -n "${tx_after}" && "${tx_after}" =~ ^[0-9]+$ && "${tx_before}" =~ ^[0-9]+$ ]]; then
    log "${IFACE} TX packets 增量: $((tx_after - tx_before))"
  fi

  ACTIVE_PATH="none"
}

main() {
  cd "${ROOT_DIR}" || exit 1

  require_integer "PUB_RATE_HZ" "${PUB_RATE_HZ}"
  require_integer "ACTION_SECONDS" "${ACTION_SECONDS}"
  require_integer "STOP_SECONDS" "${STOP_SECONDS}"
  require_integer "DISCOVERY_TIMEOUT_SEC" "${DISCOVERY_TIMEOUT_SEC}"
  require_integer "FEEDBACK_TIMEOUT_SEC" "${FEEDBACK_TIMEOUT_SEC}"
  require_integer "PUB_ONCE_TIMEOUT_SEC" "${PUB_ONCE_TIMEOUT_SEC}"
  require_integer "COUNTDOWN_SEC" "${COUNTDOWN_SEC}"

  print_semantics
  collect_inputs
  confirm_arm

  trap cleanup EXIT INT TERM

  case "${METHOD}" in
    control) run_control_path ;;
    raw) run_raw_path ;;
    *) fail "内部错误: unknown method ${METHOD}" ;;
  esac

  trap - EXIT INT TERM
  bring_down_can_if_needed
  log "测试完成。"
}

main "$@"
