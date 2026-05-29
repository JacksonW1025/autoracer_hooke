#!/usr/bin/env bash

set -u

IFACE="can0"
BITRATE="500000"
ACTION_SECONDS="5"
CENTER_SECONDS="2"
PERIOD_SECONDS="0.05"
CURRENT_ACTION="idle"
CAN_STARTED_BY_SCRIPT="0"

# Test magnitudes. Keep these conservative for real-vehicle checks.
FORWARD_SPEED_CMPS="20"       # 0.20 m/s. Hooke2 speed mode encoding mirrors repo code: vel_target / 4.0.
STEER_TIRE_DEG="10"           # Hooke2 code converts tire angle to steering wheel angle by *16.
STEER_RATIO="16"
STEER_SPEED_DEG_PER_SEC="80"

# Hooke2 raw frame notes from src/common/*_command_*.cpp:
# 0x100 throttle/speed:
#   byte0 bit0 = throttle enable
#   byte5-6    = speed target in repo speed mode: command_mps / 4.0 / 0.01
# 0x101 brake:
#   standard AUTO_DRIVE keeps brake wire enabled with 0 pedal target
# 0x103 gear:
#   byte0 bit0 = gear enable, byte1 bits0-2 = gear target
# 0x104 park:
#   byte0 bit0 = park enable, byte1 bit0 = park target
# 0x102 steering:
#   byte0 bit0 = steering enable
#   byte1      = steering speed deg/s
#   byte3-4    = steering wheel target + 500, 1 deg precision, big endian
# 0x105 vehicle mode:
#   drive mode 0 = throttle pedal drive, 1 = speed drive; steer mode 0 = standard steer
FRAME_MODE_DEFAULT="105#0000000000000000"
FRAME_MODE_SPEED_DRIVE="105#0001000000000000"
FRAME_BRAKE_ENABLE_ZERO="101#0119000000000000"
FRAME_BRAKE_DISABLE_RESET="101#0019000000000000"
FRAME_GEAR_DRIVE="103#0104000000000000"
FRAME_PARK_RELEASE="104#0100000000000000"

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少命令: $1"
    exit 1
  fi
}

run_sudo() {
  sudo "$@"
}

show_header() {
  if [[ -n "${TERM:-}" ]]; then
    clear
  fi
  echo "Hooke2 CAN Chassis Test"
  echo "接口: ${IFACE}, 波特率: ${BITRATE}"
  echo
  echo "安全检查:"
  echo "  1. 车辆静止，周围无人靠近车轮和转向机构"
  echo "  2. 急停可用，驾驶员/安全员在场"
  echo "  3. 前进测试会发送低速、挡位D、驻车释放、制动0和转向回正帧"
  echo
}

bring_up_can() {
  if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    echo "未找到 ${IFACE}。当前 CAN 接口:"
    ip -o link show | awk -F': ' '/can[0-9]+/ {print "  " $2}'
    return 1
  fi

  if ip -details link show "${IFACE}" | grep -q "state UP" &&
    ip -details link show "${IFACE}" | grep -q "bitrate ${BITRATE}"; then
    echo "${IFACE} 已经 UP 且波特率为 ${BITRATE}，直接使用现有 CAN 连接。"
    return 0
  fi

  echo "启动 ${IFACE} ..."
  run_sudo ip link set "${IFACE}" down >/dev/null 2>&1 || true
  run_sudo ip link set "${IFACE}" type can bitrate "${BITRATE}" loopback off listen-only off fd off restart-ms 100 || return 1
  run_sudo ip link set "${IFACE}" txqueuelen 100 || true
  run_sudo ip link set "${IFACE}" up || return 1
  CAN_STARTED_BY_SCRIPT="1"
  sleep 0.2
}

can_state_line() {
  ip -details link show "${IFACE}" | awk '/can </ {gsub(/^ +/, ""); print; exit}'
}

check_can_feedback() {
  echo "检查 ${IFACE} 是否有底盘上行 CAN 帧，最多等待 2 秒 ..."

  local line
  line="$(timeout 2 candump -L "${IFACE}" 2>/dev/null | head -n 1 || true)"
  if [[ -n "${line}" ]]; then
    echo "检测到 CAN 帧: ${line}"
    return 0
  fi

  echo "没有检测到 ${IFACE} 上行帧。"
  echo "当前状态: $(can_state_line)"
  echo
  echo "请确认同一组 CAN-H/CAN-L/GND 已接到本机 ${IFACE}，并且本机 CAN 口针脚定义正确。"
  return 1
}

ensure_can_ready() {
  bring_up_can || return 1
  check_can_feedback || return 1
}

send_frame() {
  cansend "${IFACE}" "$1" >/dev/null
}

speed_frame() {
  local speed_cmps="$1"
  local enable="$2"

  if (( speed_cmps < 0 )); then
    speed_cmps=0
  elif (( speed_cmps > 400 )); then
    speed_cmps=400
  fi

  # Repository formula: physical m/s command is divided by 4.0 before encoding at 0.01.
  # With speed_cmps = command_mps * 100, encoded_raw = speed_cmps / 4.
  local raw=$(((speed_cmps + 2) / 4))
  local byte5=$(((raw >> 2) & 0xFF))
  local byte6=$(((raw & 0x03) << 6))
  printf '100#%02X00000000%02X%02X00' "${enable}" "${byte5}" "${byte6}"
}

steer_frame() {
  local target_deg="$1"
  local speed_deg="$2"
  local enable="$3"

  if (( target_deg < -440 )); then
    target_deg=-440
  elif (( target_deg > 440 )); then
    target_deg=440
  fi

  if (( speed_deg < 0 )); then
    speed_deg=0
  elif (( speed_deg > 250 )); then
    speed_deg=250
  fi

  local raw=$((target_deg + 500))
  local hi=$(((raw >> 8) & 0xFF))
  local lo=$((raw & 0xFF))
  printf '102#%02X%02X00%02X%02X000000' "${enable}" "${speed_deg}" "${hi}" "${lo}"
}

send_steer_for_seconds() {
  local label="$1"
  local target_deg="$2"
  local seconds="$3"
  local end_time
  local frame

  echo
  frame="$(steer_frame "${target_deg}" "${STEER_SPEED_DEG_PER_SEC}" 1)"
  echo "开始: ${label}，持续 ${seconds}s，frame=${frame}"
  end_time=$((SECONDS + seconds))
  while (( SECONDS < end_time )); do
    send_frame "${FRAME_MODE_DEFAULT}" || return 1
    send_frame "${frame}" || return 1
    sleep "${PERIOD_SECONDS}"
  done
}

send_forward_for_seconds() {
  local label="$1"
  local speed_cmps="$2"
  local seconds="$3"
  local end_time
  local speed
  local steer_center

  echo
  speed="$(speed_frame "${speed_cmps}" 1)"
  steer_center="$(steer_frame 0 "${STEER_SPEED_DEG_PER_SEC}" 1)"
  echo "开始: ${label}，持续 ${seconds}s"
  echo "  speed=${speed}"
  echo "  brake=${FRAME_BRAKE_ENABLE_ZERO}"
  echo "  steer=${steer_center}"
  echo "  gear=${FRAME_GEAR_DRIVE}"
  echo "  park=${FRAME_PARK_RELEASE}"
  echo "  mode=${FRAME_MODE_SPEED_DRIVE}"
  end_time=$((SECONDS + seconds))
  while (( SECONDS < end_time )); do
    send_frame "${speed}" || return 1
    send_frame "${FRAME_BRAKE_ENABLE_ZERO}" || return 1
    send_frame "${steer_center}" || return 1
    send_frame "${FRAME_GEAR_DRIVE}" || return 1
    send_frame "${FRAME_PARK_RELEASE}" || return 1
    send_frame "${FRAME_MODE_SPEED_DRIVE}" || return 1
    sleep "${PERIOD_SECONDS}"
  done
}

stop_forward() {
  local end_time
  local speed_zero
  local speed_disable
  local steer_center
  local steer_disable

  echo
  echo "前进停止: 发送速度 0 ${CENTER_SECONDS}s，然后 disable 油门/转向"

  speed_zero="$(speed_frame 0 1)"
  speed_disable="$(speed_frame 0 0)"
  steer_center="$(steer_frame 0 "${STEER_SPEED_DEG_PER_SEC}" 1)"
  steer_disable="$(steer_frame 0 0 0)"

  end_time=$((SECONDS + CENTER_SECONDS))
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
}

safe_stop() {
  echo
  echo "停止动作: 转向回正 ${CENTER_SECONDS}s，然后 disable 转向控制"

  local end_time
  local steer_center
  local steer_disable

  steer_center="$(steer_frame 0 "${STEER_SPEED_DEG_PER_SEC}" 1)"
  steer_disable="$(steer_frame 0 0 0)"

  end_time=$((SECONDS + CENTER_SECONDS))
  while (( SECONDS < end_time )); do
    cansend "${IFACE}" "${FRAME_MODE_DEFAULT}" >/dev/null 2>&1 || true
    cansend "${IFACE}" "${steer_center}" >/dev/null 2>&1 || true
    sleep "${PERIOD_SECONDS}"
  done

  for _ in $(seq 1 10); do
    cansend "${IFACE}" "${steer_disable}" >/dev/null 2>&1 || true
    sleep "${PERIOD_SECONDS}"
  done
}

run_steer_action() {
  local label="$1"
  local target_deg="$2"

  ensure_can_ready || {
    echo
    echo "CAN 未通，已取消本次动作。按回车返回菜单。"
    read -r _
    return
  }

  CURRENT_ACTION="steer"
  send_steer_for_seconds "${label}" "${target_deg}" "${ACTION_SECONDS}"
  safe_stop
  CURRENT_ACTION="idle"

  echo
  echo "动作完成。按回车返回菜单。"
  read -r _
}

run_forward_action() {
  local label="$1"
  local speed_cmps="$2"

  ensure_can_ready || {
    echo
    echo "CAN 未通，已取消本次动作。按回车返回菜单。"
    read -r _
    return
  }

  CURRENT_ACTION="forward"
  send_forward_for_seconds "${label}" "${speed_cmps}" "${ACTION_SECONDS}"
  stop_forward
  CURRENT_ACTION="idle"

  echo
  echo "动作完成。按回车返回菜单。"
  read -r _
}

on_exit() {
  echo
  echo "收到退出信号，发送安全停止帧 ..."
  if [[ "${CURRENT_ACTION}" == "forward" ]]; then
    stop_forward
  else
    safe_stop
  fi
}

main_menu() {
  local steer_wheel_deg=$((STEER_TIRE_DEG * STEER_RATIO))
  local left_target_deg=$((-steer_wheel_deg))
  local right_target_deg="${steer_wheel_deg}"

  while true; do
    show_header
    echo "请选择测试动作，每个动作发送 ${ACTION_SECONDS}s:"
    echo "  1. 标准低速前进 0.20m/s"
    echo "  2. 左转，约 ${STEER_TIRE_DEG}deg 轮胎角"
    echo "  3. 右转，约 ${STEER_TIRE_DEG}deg 轮胎角"
    echo "  4. 只检查 CAN 连通"
    echo "  0. 退出"
    echo
    read -r -p "输入数字: " choice

    case "${choice}" in
      1) run_forward_action "标准低速前进 0.20m/s" "${FORWARD_SPEED_CMPS}" ;;
      2) run_steer_action "左转，方向盘命令 ${left_target_deg}deg" "${left_target_deg}" ;;
      3) run_steer_action "右转，方向盘命令 +${right_target_deg}deg" "${right_target_deg}" ;;
      4)
        ensure_can_ready
        echo
        echo "检查完成。按回车返回菜单。"
        read -r _
        ;;
      0)
        safe_stop
        if [[ "${CAN_STARTED_BY_SCRIPT}" == "1" ]]; then
          run_sudo ip link set "${IFACE}" down >/dev/null 2>&1 || true
          echo "已退出，${IFACE} 已置 DOWN。"
        else
          echo "已退出，${IFACE} 保持当前 UP 状态。"
        fi
        exit 0
        ;;
      *)
        echo "无效选项。按回车返回菜单。"
        read -r _
        ;;
    esac
  done
}

need_cmd ip
need_cmd sudo
need_cmd cansend
need_cmd candump
need_cmd timeout

show_header
echo "如果 ${IFACE} 尚未启动，脚本会请求 sudo 权限来配置 CAN。"
echo
read -r -p "确认安全条件满足后，输入 ARM 继续: " arm
if [[ "${arm}" != "ARM" ]]; then
  echo "未确认 ARM，退出。"
  exit 1
fi

trap 'on_exit; exit 130' INT
trap 'on_exit; exit 143' TERM
main_menu
