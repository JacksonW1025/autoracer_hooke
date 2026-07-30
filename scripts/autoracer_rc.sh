#!/usr/bin/env bash
set -Eeuo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(dirname "${PRODUCT_ROOT}")"
MAP_ASSETS_ROOT="${RC_MAP_ASSETS_ROOT:-${WORKSPACE_ROOT}/rc-map-assets}"
MAPPING_RECORDING_HELPER="${PRODUCT_ROOT}/scripts/autoracer_rc_recording.sh"
RUNTIME_STATE_WATCH_HELPER="${PRODUCT_ROOT}/scripts/autoracer_rc_runtime_state_watch.py"
DRIVING_PROFILES_FILE="${RC_DRIVING_PROFILES_FILE:-${PRODUCT_ROOT}/src/platform/rc/autoracer_rc_bringup/config/rc/driving_profiles.yaml}"

G90_DEVICE="${RC_G90_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA6079369-if00}"
IMU_DEVICE="${RC_IMU_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0003-if00-port0}"
CHASSIS_DEVICE="${RC_CHASSIS_DEVICE:-/dev/autoracer_rc_chassis}"
LIDAR_IP="${RC_LIDAR_IP:-192.168.1.200}"

OBSERVE_SEC="${RC_OBSERVE_SEC:-4}"
SAMPLE_TIMEOUT_SEC="${RC_SAMPLE_TIMEOUT_SEC:-4}"
TOPIC_WAIT_SEC="${RC_TOPIC_WAIT_SEC:-10}"
RATE_WAIT_SEC="${RC_RATE_WAIT_SEC:-8}"

ACTIVE_PID=""
ACTIVE_LABEL=""
ACTIVE_LOG=""
RUNTIME_STATE_WATCH_PID=""
RUNTIME_STATE_FILE=""
RUNTIME_STATE_WATCH_LOG=""
SESSION_ROOT=""
TASK_INDEX=0
ENV_READY=0
INTERACTIVE=0
VERBOSE=0

LAST_RATE=""
LAST_FIELD_OUTPUT=""
LAST_VALUE=""
G90_GGA_COUNT=0
G90_GST_COUNT=0
G90_THS_COUNT=0
G90_QUALITY="unknown"
G90_GST_STATE="未知"
G90_THS_MODE="unknown"
G90_POSITION_READY=0

AUTONOMY_MAP_ID=""
AUTONOMY_MAP_PATH=""
AUTONOMY_COURSE_PATH=""
AUTONOMY_COURSE_DIR=""
AUTONOMY_PROJECTOR=""
AUTONOMY_ROUTE_MAX_SPEED=""
AUTONOMY_ROUTE_POINTS=""
AUTONOMY_ROUTE_LENGTH=""
AUTONOMY_PROFILE_ID=""
AUTONOMY_PROFILE_NAME=""
AUTONOMY_PROFILE_MAX_SPEED=""
AUTONOMY_PROFILE_MAX_ACCEL=""
AUTONOMY_PROFILE_MAX_DECEL=""
AUTONOMY_PROFILE_LATENCY=""
AUTONOMY_PROFILE_STOPPING_MARGIN=""

RUNTIME_STATE=""
RUNTIME_READY=0
RUNTIME_REASON=""
RUNTIME_CONTROL_ENABLED=0
RUNTIME_CONTROL_MODE="null"
RUNTIME_GEAR="null"
RUNTIME_ENGAGED=0

info() {
  printf '[INFO] %s\n' "$*"
}

ok() {
  printf '[ OK ] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
}

fail() {
  printf '[FAIL] %s\n' "$*" >&2
}

is_dry_run() {
  [[ "${RC_DRY_RUN:-0}" == "1" ]]
}

is_verbose() {
  (( VERBOSE == 1 ))
}

verbose_info() {
  if is_verbose; then
    info "$@"
  fi
}

verbose_ok() {
  if is_verbose; then
    ok "$@"
  fi
}

print_command() {
  if ! is_verbose && ! is_dry_run; then
    return 0
  fi
  printf '       '
  printf '%q ' "$@"
  printf '\n'
}

ensure_environment() {
  if (( ENV_READY == 1 )) || is_dry_run; then
    return 0
  fi

  # shellcheck source=scripts/ros_env.sh
  source "${PRODUCT_ROOT}/scripts/ros_env.sh"
  ENV_READY=1
}

ensure_session_root() {
  if [[ -n "${SESSION_ROOT}" ]] || is_dry_run; then
    return 0
  fi
  SESSION_ROOT="$(mktemp -d /tmp/autoracer-rc.XXXXXX)"
  export ROS_LOG_DIR="${SESSION_ROOT}/ros-cli"
  mkdir -p "${ROS_LOG_DIR}"
  verbose_info "本次入口日志：${SESSION_ROOT}"
}

device_available() {
  local label="$1"
  local path="$2"
  if is_dry_run; then
    return 0
  fi
  if [[ ! -e "${path}" ]]; then
    fail "${label} 设备路径未枚举：${path}"
    return 1
  fi
  if [[ ! -r "${path}" || ! -w "${path}" ]]; then
    fail "${label} 设备缺少当前用户读写权限：${path}"
    return 1
  fi
  verbose_ok "${label} 设备：${path} -> $(readlink -f "${path}")"
}

report_lidar_network() {
  local route
  if is_dry_run; then
    return 0
  fi
  if ping -c 1 -W 1 "${LIDAR_IP}" >/dev/null 2>&1; then
    verbose_ok "LiDAR ${LIDAR_IP} 网络可达"
  else
    warn "LiDAR ${LIDAR_IP} ping 无响应；最终以真实点云是否到达为准"
  fi
  route="$(ip -4 route get "${LIDAR_IP}" 2>/dev/null | head -n 1 || true)"
  if [[ -n "${route}" ]]; then
    if is_verbose; then
      printf '       %s\n' "${route}"
    fi
  else
    warn "没有解析到 LiDAR IPv4 路由"
  fi
}

start_launch() {
  local label="$1"
  shift

  if [[ -n "${ACTIVE_PID}" ]]; then
    fail "已有任务运行：${ACTIVE_LABEL}"
    return 1
  fi

  verbose_info "启动：${label}"
  print_command "$@"
  if is_dry_run; then
    return 0
  fi

  ensure_environment
  ensure_session_root
  TASK_INDEX=$((TASK_INDEX + 1))
  local task_name
  task_name="${label//[^[:alnum:]_-]/_}"
  ACTIVE_LOG="${SESSION_ROOT}/${TASK_INDEX}-${task_name}.log"
  local ros_log_dir="${SESSION_ROOT}/${TASK_INDEX}-${task_name}-ros"
  mkdir -p "${ros_log_dir}"

  ROS_LOG_DIR="${ros_log_dir}" setsid "$@" </dev/null >"${ACTIVE_LOG}" 2>&1 &
  ACTIVE_PID="$!"
  ACTIVE_LABEL="${label}"

  sleep 1
  if ! kill -0 "${ACTIVE_PID}" >/dev/null 2>&1; then
    fail "${label} 启动后立即退出"
    tail -n 40 "${ACTIVE_LOG}" >&2 || true
    wait "${ACTIVE_PID}" 2>/dev/null || true
    ACTIVE_PID=""
    ACTIVE_LABEL=""
    return 1
  fi
  verbose_ok "${label} 已启动，PID/PGID=${ACTIVE_PID}"
}

start_runtime_state_watch() {
  if [[ -n "${RUNTIME_STATE_WATCH_PID}" ]]; then
    fail "runtime 状态订阅器已在运行"
    return 1
  fi

  ensure_environment
  ensure_session_root
  RUNTIME_STATE_FILE="${SESSION_ROOT}/${TASK_INDEX}-runtime-state.json"
  RUNTIME_STATE_WATCH_LOG="${SESSION_ROOT}/${TASK_INDEX}-runtime-state-watch.log"
  local ros_log_dir="${SESSION_ROOT}/${TASK_INDEX}-runtime-state-watch-ros"
  mkdir -p "${ros_log_dir}"

  ROS_LOG_DIR="${ros_log_dir}" setsid \
    python3 "${RUNTIME_STATE_WATCH_HELPER}" \
    --output "${RUNTIME_STATE_FILE}" \
    </dev/null >"${RUNTIME_STATE_WATCH_LOG}" 2>&1 &
  RUNTIME_STATE_WATCH_PID="$!"

  sleep 1
  if ! kill -0 "${RUNTIME_STATE_WATCH_PID}" >/dev/null 2>&1; then
    fail "runtime 状态订阅器启动后立即退出"
    tail -n 40 "${RUNTIME_STATE_WATCH_LOG}" >&2 || true
    wait "${RUNTIME_STATE_WATCH_PID}" 2>/dev/null || true
    RUNTIME_STATE_WATCH_PID=""
    return 1
  fi
}

stop_runtime_state_watch() {
  if [[ -z "${RUNTIME_STATE_WATCH_PID}" ]]; then
    return 0
  fi

  local pid="${RUNTIME_STATE_WATCH_PID}"
  kill -INT -- "-${pid}" >/dev/null 2>&1 || kill -INT "${pid}" >/dev/null 2>&1 || true

  local attempt
  for attempt in {1..20}; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    for attempt in {1..20}; do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
  fi
  if kill -0 "${pid}" >/dev/null 2>&1; then
    warn "runtime 状态订阅器未在 SIGTERM 后退出，仅清理本入口拥有的进程组"
    kill -KILL -- "-${pid}" >/dev/null 2>&1 || true
  fi

  wait "${pid}" 2>/dev/null || true
  RUNTIME_STATE_WATCH_PID=""
}

stop_active() {
  stop_runtime_state_watch
  if [[ -z "${ACTIVE_PID}" ]]; then
    return 0
  fi

  local pid="${ACTIVE_PID}"
  local label="${ACTIVE_LABEL}"
  verbose_info "停止：${label}"
  kill -INT -- "-${pid}" >/dev/null 2>&1 || kill -INT "${pid}" >/dev/null 2>&1 || true

  local attempt
  for attempt in {1..40}; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done

  if kill -0 "${pid}" >/dev/null 2>&1; then
    warn "${label} 未在 SIGINT 后退出，发送 SIGTERM"
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    for attempt in {1..20}; do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.2
    done
  fi

  if kill -0 "${pid}" >/dev/null 2>&1; then
    warn "${label} 未在 SIGTERM 后退出，仅清理本入口拥有的进程组"
    kill -KILL -- "-${pid}" >/dev/null 2>&1 || true
  fi

  wait "${pid}" 2>/dev/null || true
  ACTIVE_PID=""
  ACTIVE_LABEL=""
  verbose_ok "${label} 已停止"
}

on_exit() {
  stop_active
  if [[ -n "${SESSION_ROOT}" ]]; then
    info "运行日志保留在 ${SESSION_ROOT}"
  fi
}

on_signal() {
  printf '\n' >&2
  warn "收到退出信号"
  exit 130
}

trap on_exit EXIT
trap on_signal INT TERM

wait_for_topic() {
  local topic="$1"
  local attempts=$((TOPIC_WAIT_SEC * 2))
  local topic_type=""
  local attempt
  for ((attempt = 1; attempt <= attempts; attempt++)); do
    topic_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
    if [[ -n "${topic_type}" ]]; then
      verbose_ok "${topic} (${topic_type})"
      return 0
    fi
    sleep 0.5
  done
  fail "${TOPIC_WAIT_SEC}s 内未发现 ${topic}"
  return 1
}

observe_rate() {
  local topic="$1"
  local rate
  LAST_RATE=""
  rate="$(
    timeout "${RATE_WAIT_SEC}s" \
      env PYTHONUNBUFFERED=1 \
      ros2 topic hz --window 200 --wall-time "${topic}" 2>&1 |
      awk '/average rate:/ {print; exit}' || true
  )"
  if [[ -z "${rate}" ]]; then
    fail "${topic} 在最大等待 ${RATE_WAIT_SEC}s 内没有真实消息频率"
    return 1
  fi
  LAST_RATE="${rate##* }"
  verbose_ok "${topic}: ${rate}"
}

observe_field() {
  local topic="$1"
  local field="$2"
  local output
  LAST_FIELD_OUTPUT=""
  output="$(
    timeout "${SAMPLE_TIMEOUT_SEC}s" \
      ros2 topic echo "${topic}" --once --field "${field}" 2>&1 || true
  )"
  if [[ -z "${output}" ]] || grep -Eq \
    'Could not determine|Unknown topic|Traceback|usage:' <<<"${output}"
  then
    fail "${topic}.${field} 在 ${SAMPLE_TIMEOUT_SEC}s 内没有真实样本"
    return 1
  fi
  LAST_FIELD_OUTPUT="${output}"
  if is_verbose; then
    printf '%s\n' "----- ${topic}.${field} -----"
    sed -n '1,24p' <<<"${output}"
  fi
}

observe_positive_field() {
  local topic="$1"
  local field="$2"
  local output
  local value
  LAST_VALUE=""
  output="$(
    timeout "${SAMPLE_TIMEOUT_SEC}s" \
      ros2 topic echo "${topic}" --once --field "${field}" 2>&1 || true
  )"
  value="$(awk '/^[[:space:]]*[0-9]+([.][0-9]+)?[[:space:]]*$/ {print $1; exit}' <<<"${output}")"
  if [[ -z "${value}" ]] || ! awk -v value="${value}" 'BEGIN {exit !(value > 0)}'; then
    fail "${topic}.${field} 没有观测到大于零的真实样本"
    return 1
  fi
  LAST_VALUE="${value}"
  verbose_ok "${topic}.${field}=${value}"
}

launch_sensing() {
  local label="$1"
  local launch_lidar="$2"
  local launch_imu="$3"
  local launch_g90="$4"
  start_launch \
    "${label}" \
    ros2 launch autoracer_rc_bringup sensing.launch.py \
    launch_static_tf:=true \
    launch_lidar:="${launch_lidar}" \
    launch_imu:="${launch_imu}" \
    launch_g90:="${launch_g90}" \
    launch_g90_driver:="${launch_g90}" \
    imu_device:="${IMU_DEVICE}" \
    g90_device:="${G90_DEVICE}"
}

finish_test() {
  local result="$1"
  stop_active
  if (( result != 0 )); then
    fail "真实设备检查失败；启动日志：${ACTIVE_LOG}"
    return "${result}"
  fi
  return 0
}

test_lidar() {
  local result=0
  local raw_rate="unknown"
  local formal_rate="unknown"
  local frame_id="unknown"
  local width="unknown"
  info "正在检查 LiDAR 数据..."
  report_lidar_network
  if ! launch_sensing "LiDAR-test" true false false; then
    return 1
  fi
  if is_dry_run; then
    return 0
  fi

  wait_for_topic /sensing/lidar/raw/pointcloud || result=1
  wait_for_topic /sensing/lidar/concatenated/pointcloud || result=1
  if observe_rate /sensing/lidar/raw/pointcloud; then
    raw_rate="${LAST_RATE}"
  else
    result=1
  fi
  if observe_rate /sensing/lidar/concatenated/pointcloud; then
    formal_rate="${LAST_RATE}"
  else
    result=1
  fi
  if observe_field /sensing/lidar/concatenated/pointcloud header; then
    frame_id="$(
      awk '/^frame_id:/ {print $2; exit}' <<<"${LAST_FIELD_OUTPUT}"
    )"
    frame_id="${frame_id:-unknown}"
  else
    result=1
  fi
  if observe_positive_field /sensing/lidar/concatenated/pointcloud width; then
    width="${LAST_VALUE}"
  else
    result=1
  fi
  observe_positive_field /sensing/lidar/concatenated/pointcloud height || result=1
  observe_positive_field /sensing/lidar/concatenated/pointcloud point_step || result=1
  observe_positive_field /sensing/lidar/concatenated/pointcloud row_step || result=1
  finish_test "${result}" || return 1
  ok "LiDAR：raw ${raw_rate} Hz，正式点云 ${formal_rate} Hz，frame=${frame_id}，points=${width}"
}

test_imu() {
  local result=0
  local raw_rate="unknown"
  local formal_rate="unknown"
  local frame_id="unknown"
  info "正在检查 IMU 数据..."
  device_available "IMU" "${IMU_DEVICE}" || return 1
  if ! launch_sensing "IMU-test" false true false; then
    return 1
  fi
  if is_dry_run; then
    return 0
  fi

  wait_for_topic /sensing/imu/raw/imu_data || result=1
  wait_for_topic /sensing/imu/imu_data || result=1
  if observe_rate /sensing/imu/raw/imu_data; then
    raw_rate="${LAST_RATE}"
  else
    result=1
  fi
  if observe_rate /sensing/imu/imu_data; then
    formal_rate="${LAST_RATE}"
  else
    result=1
  fi
  if observe_field /sensing/imu/imu_data header; then
    frame_id="$(
      awk '/^frame_id:/ {print $2; exit}' <<<"${LAST_FIELD_OUTPUT}"
    )"
    frame_id="${frame_id:-unknown}"
  else
    result=1
  fi
  observe_field /sensing/imu/imu_data angular_velocity || result=1
  observe_field /sensing/imu/imu_data linear_acceleration || result=1
  observe_field /sensing/imu/imu_data angular_velocity_covariance || result=1
  finish_test "${result}" || return 1
  ok "IMU：raw ${raw_rate} Hz，正式数据 ${formal_rate} Hz，frame=${frame_id}"
}

g90_state_name() {
  case "$1" in
    4) printf 'RTK Fixed' ;;
    5) printf 'RTK Float' ;;
    2) printf 'DGPS' ;;
    1) printf 'Single' ;;
    0) printf 'No fix' ;;
    *) printf 'Unknown quality %s' "$1" ;;
  esac
}

nmea_sentence_count() {
  local capture="$1"
  local kind="$2"
  awk -v kind="${kind}," \
    'index($0, "sentence: $") && index($0, kind) {count++}
     END {print count+0}' \
    "${capture}"
}

latest_nmea_sentence() {
  local capture="$1"
  local kind="$2"
  awk -v kind="${kind}," \
    'index($0, "sentence: $") && index($0, kind) {latest=$0}
     END {print latest}' \
    "${capture}"
}

observe_g90_nmea() {
  local capture="${SESSION_ROOT}/${TASK_INDEX}-g90-nmea.txt"
  timeout "${OBSERVE_SEC}s" \
    ros2 topic echo /g90/raw/nmea_sentence >"${capture}" 2>&1 || true

  G90_GGA_COUNT="$(nmea_sentence_count "${capture}" GGA)"
  G90_GST_COUNT="$(nmea_sentence_count "${capture}" GST)"
  G90_THS_COUNT="$(nmea_sentence_count "${capture}" THS)"
  G90_QUALITY="unknown"
  G90_GST_STATE="未知"
  G90_THS_MODE="unknown"
  G90_POSITION_READY=0
  verbose_info \
    "G90 ${OBSERVE_SEC}s 原始报文：GGA=${G90_GGA_COUNT} GST=${G90_GST_COUNT} THS=${G90_THS_COUNT}"

  local kind
  local line
  for kind in GGA GST THS; do
    line="$(latest_nmea_sentence "${capture}" "${kind}")"
    if [[ -n "${line}" ]] && is_verbose; then
      printf '%s\n' "----- latest ${kind} -----"
      printf '%s\n' "${line#*sentence: }"
    fi
  done

  local gga_line
  local gga_sentence
  local gga_payload
  local -a fields=()
  gga_line="$(latest_nmea_sentence "${capture}" GGA)"
  if [[ -n "${gga_line}" ]]; then
    gga_sentence="${gga_line#*\$}"
    gga_payload="${gga_sentence%%\**}"
    IFS=',' read -r -a fields <<<"${gga_payload}"
    G90_QUALITY="${fields[6]:-unknown}"
    verbose_info \
      "G90 当前质量：$(g90_state_name "${G90_QUALITY}") (GGA quality=${G90_QUALITY})"
  fi

  local gst_line
  local gst_sentence
  local gst_payload
  local gst_nonempty=0
  local index
  gst_line="$(latest_nmea_sentence "${capture}" GST)"
  if [[ -n "${gst_line}" ]]; then
    gst_sentence="${gst_line#*\$}"
    gst_payload="${gst_sentence%%\**}"
    fields=()
    IFS=',' read -r -a fields <<<"${gst_payload}"
    for index in {1..8}; do
      if [[ -n "${fields[index]:-}" ]]; then
        gst_nonempty=$((gst_nonempty + 1))
      fi
    done
    if (( gst_nonempty == 8 )); then
      G90_GST_STATE="完整"
    elif (( gst_nonempty == 0 )); then
      G90_GST_STATE="空"
    else
      G90_GST_STATE="不完整"
    fi
  fi

  local ths_line
  local ths_sentence
  local ths_payload
  local ths_heading=""
  ths_line="$(latest_nmea_sentence "${capture}" THS)"
  if [[ -n "${ths_line}" ]]; then
    ths_sentence="${ths_line#*\$}"
    ths_payload="${ths_sentence%%\**}"
    fields=()
    IFS=',' read -r -a fields <<<"${ths_payload}"
    ths_heading="${fields[1]:-}"
    G90_THS_MODE="${fields[2]:-unknown}"
  fi

  if (( G90_GGA_COUNT == 0 || G90_GST_COUNT == 0 || G90_THS_COUNT == 0 )); then
    fail "G90 未同时收到真实 GGA/GST/THS"
    return 1
  fi

  if [[ "${G90_QUALITY}" =~ ^(4|5)$ ]] &&
    [[ "${G90_GST_STATE}" == "完整" ]] &&
    [[ "${G90_THS_MODE}" == "A" ]] &&
    [[ -n "${ths_heading}" ]]
  then
    G90_POSITION_READY=1
  fi
  return 0
}

observe_g90_fix_sample() {
  local capture="${SESSION_ROOT}/${TASK_INDEX}-g90-fix.txt"
  timeout "${TOPIC_WAIT_SEC}s" \
    ros2 topic echo /g90/fix --once >"${capture}" 2>&1 || true

  local required_pattern
  for required_pattern in \
    '^header:$' \
    '^[[:space:]]+stamp:$' \
    '^[[:space:]]+sec: [1-9][0-9]*$' \
    '^[[:space:]]+frame_id: gnss_link$' \
    '^status:$' \
    '^latitude:' \
    '^longitude:'
  do
    if ! grep -Eq "${required_pattern}" "${capture}"; then
      fail \
        "/g90/fix 在 ${TOPIC_WAIT_SEC}s 内没有收到完整样本；采样输出：${capture}"
      return 1
    fi
  done

  if is_verbose; then
    printf '%s\n' "----- /g90/fix sample -----"
    sed -n '1,40p' "${capture}"
  fi
}

test_g90() {
  local result=0
  local fix_rate="unknown"
  G90_POSITION_READY=0
  info "正在检查 G90 数据..."
  device_available "G90" "${G90_DEVICE}" || return 1
  if ! launch_sensing "G90-test" false false true; then
    return 1
  fi
  if is_dry_run; then
    return 0
  fi

  wait_for_topic /g90/raw/nmea_sentence || result=1
  wait_for_topic /g90/fix || result=1
  observe_g90_nmea || result=1
  if observe_rate /g90/fix; then
    fix_rate="${LAST_RATE}"
  else
    result=1
  fi
  observe_g90_fix_sample || result=1
  finish_test "${result}" || return 1

  if (( G90_POSITION_READY == 1 )); then
    ok "G90：NMEA/适配链 ${fix_rate} Hz，接收机数据有效（$(g90_state_name "${G90_QUALITY}")，GST完整，THS=A）"
  else
    warn \
      "G90：设备链路与 NMEA/适配链 ${fix_rate} Hz 正常；定位未就绪（quality=${G90_QUALITY}，GST${G90_GST_STATE}，THS=${G90_THS_MODE}）"
  fi
  return 0
}

telemetry_authorized() {
  if is_dry_run; then
    return 0
  fi
  if (( INTERACTIVE == 1 )); then
    return 0
  fi
  if [[ "${RC_TELEMETRY_AUTHORIZED:-0}" == "1" ]]; then
    return 0
  fi

  fail "非交互底盘检查需要显式设置 RC_TELEMETRY_AUTHORIZED=1"
  return 1
}

start_chassis_telemetry() {
  telemetry_authorized || {
    fail "未获得本次 telemetry 授权"
    return 1
  }
  device_available "底盘" "${CHASSIS_DEVICE}" || return 1
  start_launch \
    "chassis-telemetry-test" \
    ros2 launch autoracer_rc_bringup vehicle.launch.py \
    serial_port:="${CHASSIS_DEVICE}" \
    telemetry_only:=true \
    use_sim_time:=false
}

test_chassis() {
  local result=0
  local feedback_rate="unknown"
  local velocity="unknown"
  local steering="unknown"
  local mode="unknown"
  info "正在检查底盘串口数据..."
  if ! start_chassis_telemetry; then
    return 1
  fi
  if is_dry_run; then
    return 0
  fi

  wait_for_topic /vehicle/status/velocity_status || result=1
  wait_for_topic /vehicle/status/steering_status || result=1
  wait_for_topic /vehicle/status/control_mode || result=1
  if observe_rate /vehicle/status/velocity_status; then
    feedback_rate="${LAST_RATE}"
  else
    result=1
  fi
  if observe_field /vehicle/status/velocity_status longitudinal_velocity; then
    velocity="$(
      awk '/^[[:space:]]*-?[0-9]+([.][0-9]+)?[[:space:]]*$/ {
        gsub(/[[:space:]]/, ""); print; exit
      }' <<<"${LAST_FIELD_OUTPUT}"
    )"
    velocity="${velocity:-unknown}"
  else
    result=1
  fi
  if observe_field /vehicle/status/steering_status steering_tire_angle; then
    steering="$(
      awk '/^[[:space:]]*-?[0-9]+([.][0-9]+)?[[:space:]]*$/ {
        gsub(/[[:space:]]/, ""); print; exit
      }' <<<"${LAST_FIELD_OUTPUT}"
    )"
    steering="${steering:-unknown}"
  else
    result=1
  fi
  if observe_field /vehicle/status/control_mode mode; then
    mode="$(
      awk '/^[[:space:]]*-?[0-9]+([.][0-9]+)?[[:space:]]*$/ {
        gsub(/[[:space:]]/, ""); print; exit
      }' <<<"${LAST_FIELD_OUTPUT}"
    )"
    mode="${mode:-unknown}"
  else
    result=1
  fi
  finish_test "${result}" || return 1
  ok "底盘：反馈 ${feedback_rate} Hz，速度=${velocity}，转角=${steering}，模式=${mode}"
}

test_all_devices() {
  local result=0
  test_lidar || result=1
  test_imu || result=1
  test_g90 || result=1
  test_chassis || result=1
  if is_dry_run; then
    return "${result}"
  fi
  if (( result == 0 )); then
    if (( G90_POSITION_READY == 1 )); then
      ok "设备链路 4/4 正常，G90 接收机数据有效"
    else
      warn "设备链路 4/4 正常；G90 定位结果未就绪"
    fi
  else
    fail "全部设备检查存在链路或真实数据故障"
  fi
  return "${result}"
}

record_mapping() {
  local site="${RC_MAPPING_SITE:-}"
  local label="${RC_MAPPING_LABEL:-}"
  local operator_name="${RC_MAPPING_OPERATOR:-$(id -un)}"
  local entered=""

  if (( INTERACTIVE == 1 )); then
    while [[ -z "${site}" ]]; do
      read -r -p '场地（必填）：' entered || return 1
      site="${entered}"
      if [[ -z "${site}" ]]; then
        warn "场地不能为空"
      fi
    done
    read -r -p '会话标签（可选，留空则只用时间戳）：' entered || return 1
    if [[ -n "${entered}" ]]; then
      label="${entered}"
    fi
  fi

  if [[ -z "${site}" ]]; then
    fail "非交互录制需要设置 RC_MAPPING_SITE"
    return 1
  fi
  if [[ ! -x "${MAPPING_RECORDING_HELPER}" ]]; then
    fail "建图录制组件不存在或不可执行：${MAPPING_RECORDING_HELPER}"
    return 1
  fi

  local command=(
    "${MAPPING_RECORDING_HELPER}"
    --site "${site}"
    --operator "${operator_name}"
  )
  if [[ -n "${label}" ]]; then
    command+=(--label "${label}")
  fi
  if is_verbose; then
    command+=(--verbose)
  fi
  if is_dry_run; then
    command+=(--dry-run)
  fi
  "${command[@]}"
}

discover_autonomy_assets() {
  python3 - "${MAP_ASSETS_ROOT}" "${VERBOSE}" <<'PY'
import csv
import json
import math
from pathlib import Path
import sys

import yaml


root = Path(sys.argv[1])
verbose = sys.argv[2] == "1"
supported_projectors = {
    "LocalCartesian",
    "LocalCartesianUTM",
    "MGRS",
    "TransverseMercator",
}


def rejected(map_id, reason):
    if verbose:
        print(f"[INFO] 自动驾驶资产跳过 {map_id}：{reason}", file=sys.stderr)


if not root.is_dir():
    raise SystemExit(f"地图资产根目录不存在：{root}")

for map_path in sorted(root.iterdir()):
    if not map_path.is_dir() or map_path.name == "courses":
        continue
    map_id = map_path.name
    course_dir = root / "courses" / map_id
    course_path = course_dir / "course.csv"
    course_manifest_path = course_dir / "manifest.json"
    release_manifest_path = map_path / "release_manifest.json"
    projector_path = map_path / "map_projector_info.yaml"
    required = (
        course_path,
        course_manifest_path,
        release_manifest_path,
        projector_path,
        map_path / "input_contract.json",
        map_path / "pointcloud_map_metadata.yaml",
    )
    if not all(path.is_file() for path in required) or not any(
        map_path.glob("*.pcd")
    ):
        rejected(map_id, "PCD、CSV 或 sidecar 不完整")
        continue

    try:
        projector_document = yaml.safe_load(projector_path.read_text(encoding="utf-8"))
        projector = str(projector_document.get("projector_type", ""))
        course_manifest = json.loads(
            course_manifest_path.read_text(encoding="utf-8")
        )
        release_manifest = json.loads(
            release_manifest_path.read_text(encoding="utf-8")
        )
        if projector not in supported_projectors:
            rejected(map_id, f"projector={projector or 'missing'}")
            continue
        if (
            course_manifest.get("schema_version") != 2
            or course_manifest.get("map_id") != map_id
            or course_manifest.get("map", {}).get("id") != map_id
            or course_manifest.get("publication_status") != "PUBLISHED"
            or course_manifest.get("speed_profile", {}).get("status")
            != "USER_APPROVED_LOW_SPEED_VALIDATION"
            or course_manifest.get("validation", {}).get("status") != "PASS"
        ):
            rejected(map_id, "路线未发布、速度未获批、校验未通过或 map_id 不一致")
            continue
        if (
            release_manifest.get("status") != "PASS"
            or release_manifest.get("map_id") != map_id
            or release_manifest.get("publication_status") != "PUBLISHED"
        ):
            rejected(map_id, "地图未发布、release manifest 未通过或 map_id 不一致")
            continue

        route_max_speed = float(
            course_manifest.get("speed_profile", {}).get("max_speed_mps")
        )
        route_points = int(
            course_manifest.get("validation", {})
            .get("metrics", {})
            .get("point_count")
        )
        route_length = float(
            course_manifest.get("geometry", {}).get("course_length_m")
        )
        if (
            not math.isfinite(route_max_speed)
            or route_max_speed <= 0.0
            or route_points < 2
            or not math.isfinite(route_length)
            or route_length <= 0.0
        ):
            rejected(map_id, "路线统计无效")
            continue

        with course_path.open("r", encoding="utf-8", newline="") as stream:
            header = next(csv.reader(stream), [])
        if "target_velocity" not in header:
            rejected(map_id, "course.csv 缺少 target_velocity")
            continue
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        rejected(map_id, str(error))
        continue

    values = (
        map_id,
        str(map_path),
        str(course_path),
        projector,
        f"{route_max_speed:.6f}",
        str(route_points),
        f"{route_length:.6f}",
    )
    print("\t".join(values))
PY
}

validate_autonomy_asset() {
  local map_path="$1"
  local course_path="$2"
  python3 - \
    "${map_path}" \
    "${course_path}" \
    "${PRODUCT_ROOT}/src/core/autoracer_planning" <<'PY'
import math
from pathlib import Path
import sys

import yaml


map_path = Path(sys.argv[1])
course_path = Path(sys.argv[2])
sys.path.insert(0, sys.argv[3])

from autoracer_planning.fixed_course import (  # noqa: E402
    load_course_asset,
    validate_course_map_contract,
)


supported_projectors = {
    "LocalCartesian",
    "LocalCartesianUTM",
    "MGRS",
    "TransverseMercator",
}
projector_document = yaml.safe_load(
    (map_path / "map_projector_info.yaml").read_text(encoding="utf-8")
)
projector = str(projector_document.get("projector_type", ""))
if projector not in supported_projectors:
    raise SystemExit(f"地图投影不能用于 G90 自动驾驶：{projector or 'missing'}")

manifest, samples = load_course_asset(course_path.parent)
validate_course_map_contract(manifest, map_path)
if len(samples) < 2:
    raise SystemExit("路线点数不足")

route_max_speed = max(abs(sample.target_velocity) for sample in samples)
route_length = float(samples[-1].s)
if (
    not math.isfinite(route_max_speed)
    or route_max_speed <= 0.0
    or not math.isfinite(route_length)
    or route_length <= 0.0
):
    raise SystemExit("路线速度或长度无效")

print(
    "\t".join(
        (
            projector,
            f"{route_max_speed:.6f}",
            str(len(samples)),
            f"{route_length:.6f}",
        )
    )
)
PY
}

discover_driving_profiles() {
  python3 - "${DRIVING_PROFILES_FILE}" <<'PY'
import math
from pathlib import Path
import re
import sys

import yaml


path = Path(sys.argv[1])
document = yaml.safe_load(path.read_text(encoding="utf-8"))
if not isinstance(document, dict) or document.get("schema_version") != 1:
    raise SystemExit("运行方案 schema_version 必须为 1")
profiles = document.get("profiles")
if not isinstance(profiles, dict):
    raise SystemExit("运行方案 profiles 必须是映射")


def number(profile_id, profile, key):
    raw = profile.get(key)
    if isinstance(raw, bool):
        raise SystemExit(f"运行方案 {profile_id} 的 {key} 不是数值")
    value = float(raw)
    if not math.isfinite(value):
        raise SystemExit(f"运行方案 {profile_id} 的 {key} 不是有限值")
    return value


for profile_id, profile in profiles.items():
    if not isinstance(profile, dict) or profile.get("approved") is not True:
        continue
    if not isinstance(profile_id, str) or not re.fullmatch(
        r"[a-z0-9][a-z0-9_-]*", profile_id
    ):
        raise SystemExit(f"获批运行方案 ID 无效：{profile_id!r}")
    display_name = profile.get("display_name")
    if (
        not isinstance(display_name, str)
        or not display_name.strip()
        or "\t" in display_name
        or "\n" in display_name
    ):
        raise SystemExit(f"运行方案 {profile_id} 的 display_name 无效")

    max_speed = number(profile_id, profile, "max_speed_mps")
    max_accel = number(profile_id, profile, "max_accel_mps2")
    max_decel = number(profile_id, profile, "max_decel_mps2")
    latency = number(profile_id, profile, "command_latency_sec")
    stopping_margin = number(profile_id, profile, "stopping_margin_m")
    if (
        max_speed <= 0.0
        or max_accel <= 0.0
        or max_decel >= 0.0
        or latency < 0.0
        or stopping_margin < 0.0
    ):
        raise SystemExit(f"运行方案 {profile_id} 的边界值无效")

    print(
        "\t".join(
            (
                profile_id,
                display_name.strip(),
                f"{max_speed:.6f}",
                f"{max_accel:.6f}",
                f"{max_decel:.6f}",
                f"{latency:.6f}",
                f"{stopping_margin:.6f}",
            )
        )
    )
PY
}

select_autonomy_asset() {
  local listing
  if ! listing="$(discover_autonomy_assets)"; then
    fail "自动驾驶资产扫描失败"
    return 1
  fi
  if [[ -z "${listing}" ]]; then
    fail "没有可用于自动驾驶的正式地图：需要同名 PCD+CSV、PASS sidecar 和非 Local 地理投影"
    return 1
  fi

  local -a records=()
  mapfile -t records <<<"${listing}"
  local selected=""
  local requested="${RC_MAP_ID:-}"
  local index

  if [[ -n "${requested}" ]]; then
    for selected in "${records[@]}"; do
      if [[ "${selected%%$'\t'*}" == "${requested}" ]]; then
        break
      fi
      selected=""
    done
    if [[ -z "${selected}" ]]; then
      fail "指定 map_id 不在可用自动驾驶资产中：${requested}"
      return 1
    fi
  elif (( INTERACTIVE == 1 )); then
    printf '可用场景与地图：\n'
    for index in "${!records[@]}"; do
      local map_id map_path course_path projector route_speed points length
      IFS=$'\t' read -r \
        map_id map_path course_path projector route_speed points length \
        <<<"${records[index]}"
      printf '  %d. %s（%s，%s 点，%.1f m）\n' \
        "$((index + 1))" "${map_id}" "${projector}" "${points}" "${length}"
    done
    local choice
    read -r -p '请选择地图：' choice || return 1
    if [[ ! "${choice}" =~ ^[0-9]+$ ]] \
      || (( choice < 1 || choice > ${#records[@]} )); then
      fail "无效地图选项：${choice}"
      return 1
    fi
    selected="${records[choice - 1]}"
  else
    fail "非交互自动驾驶需要设置 RC_MAP_ID"
    return 1
  fi

  IFS=$'\t' read -r \
    AUTONOMY_MAP_ID AUTONOMY_MAP_PATH AUTONOMY_COURSE_PATH \
    AUTONOMY_PROJECTOR AUTONOMY_ROUTE_MAX_SPEED AUTONOMY_ROUTE_POINTS \
    AUTONOMY_ROUTE_LENGTH <<<"${selected}"
  AUTONOMY_COURSE_DIR="${AUTONOMY_COURSE_PATH%/course.csv}"
  if [[ "${AUTONOMY_COURSE_DIR}" == "${AUTONOMY_COURSE_PATH}" ]]; then
    fail "正式路线必须以 course.csv 结尾：${AUTONOMY_COURSE_PATH}"
    return 1
  fi

  info "校验地图与路线资产：${AUTONOMY_MAP_ID}"
  local validation
  if ! validation="$(
    validate_autonomy_asset "${AUTONOMY_MAP_PATH}" "${AUTONOMY_COURSE_PATH}"
  )"; then
    fail "地图与路线哈希合同校验失败：${AUTONOMY_MAP_ID}"
    return 1
  fi
  IFS=$'\t' read -r \
    AUTONOMY_PROJECTOR AUTONOMY_ROUTE_MAX_SPEED AUTONOMY_ROUTE_POINTS \
    AUTONOMY_ROUTE_LENGTH <<<"${validation}"
  ok "资产校验通过：${AUTONOMY_MAP_ID}"
}

select_driving_profile() {
  local listing
  if ! listing="$(discover_driving_profiles)"; then
    fail "运行方案配置无效：${DRIVING_PROFILES_FILE}"
    return 1
  fi
  if [[ -z "${listing}" ]]; then
    fail "没有获批的自动驾驶运行方案：${DRIVING_PROFILES_FILE}"
    return 1
  fi

  local -a records=()
  mapfile -t records <<<"${listing}"
  local selected=""
  local requested="${RC_DRIVING_PROFILE:-}"

  if [[ -n "${requested}" ]]; then
    for selected in "${records[@]}"; do
      if [[ "${selected%%$'\t'*}" == "${requested}" ]]; then
        break
      fi
      selected=""
    done
    if [[ -z "${selected}" ]]; then
      fail "指定运行方案不存在或未获批：${requested}"
      return 1
    fi
  elif (( ${#records[@]} == 1 )); then
    selected="${records[0]}"
  else
    fail "正式入口要求恰好一个默认获批运行方案；当前为 ${#records[@]} 个"
    return 1
  fi

  IFS=$'\t' read -r \
    AUTONOMY_PROFILE_ID AUTONOMY_PROFILE_NAME AUTONOMY_PROFILE_MAX_SPEED \
    AUTONOMY_PROFILE_MAX_ACCEL AUTONOMY_PROFILE_MAX_DECEL \
    AUTONOMY_PROFILE_LATENCY AUTONOMY_PROFILE_STOPPING_MARGIN <<<"${selected}"
  info "自动使用运行方案：${AUTONOMY_PROFILE_NAME}"
}

autonomy_authorized() {
  if (( INTERACTIVE == 1 )); then
    return 0
  fi
  if [[ "${RC_AUTONOMY_AUTHORIZED:-0}" == "1" ]]; then
    return 0
  fi
  fail "非交互自动驾驶需要 RC_AUTONOMY_AUTHORIZED=1 授权本次打开控制串口"
  return 1
}

runtime_state_json() {
  local output
  [[ -n "${RUNTIME_STATE_FILE}" && -r "${RUNTIME_STATE_FILE}" ]] || return 1
  IFS= read -r output <"${RUNTIME_STATE_FILE}" || return 1
  [[ -n "${output}" ]] || return 1
  printf '%s\n' "${output}"
}

read_runtime_snapshot() {
  local document
  document="$(runtime_state_json)" || return 1
  [[ -n "${document}" ]] || return 1

  local parsed
  parsed="$(
    python3 - "${document}" <<'PY'
import json
import sys


document = json.loads(sys.argv[1])


def boolean(name):
    return "1" if bool(document.get(name, False)) else "0"


reason = str(document.get("reason", "unknown")).replace("\t", " ").replace(
    "\n", " "
)
values = (
    str(document.get("state", "UNKNOWN")),
    boolean("ready"),
    reason,
    boolean("control_enabled"),
    "null"
    if document.get("control_mode") is None
    else str(document.get("control_mode")),
    "null" if document.get("gear") is None else str(document.get("gear")),
    boolean("engaged"),
)
print("\t".join(values))
PY
  )" || return 1

  IFS=$'\t' read -r \
    RUNTIME_STATE RUNTIME_READY RUNTIME_REASON RUNTIME_CONTROL_ENABLED \
    RUNTIME_CONTROL_MODE RUNTIME_GEAR RUNTIME_ENGAGED <<<"${parsed}"
}

wait_for_q_acknowledgement() {
  local prompt="$1"
  local key=""
  printf '%s\n' "${prompt}"
  while true; do
    key=""
    if ! IFS= read -r -s -n 1 key; then
      warn "终端输入已关闭，无法继续等待 Q；开始安全清理"
      return 0
    fi
    if [[ "${key}" =~ ^[qQ]$ ]]; then
      return 0
    fi
  done
}

acknowledge_autonomy_failure() {
  local conclusion="$1"
  fail "${conclusion}"
  if is_dry_run || [[ ! -t 0 ]]; then
    return 0
  fi
  wait_for_q_acknowledgement "按 Q 确认上述结论并清理返回菜单。"
}

wait_for_autonomy_ready() {
  local last_reason=""
  local key=""
  info "完整运行图正在准备；不设超时，按 Q 取消。"

  while kill -0 "${ACTIVE_PID}" >/dev/null 2>&1; do
    key=""
    if IFS= read -r -s -n 1 -t 0.2 key; then
      case "${key}" in
        q|Q) return 2 ;;
      esac
    fi

    if read_runtime_snapshot; then
      if [[ "${RUNTIME_STATE}" == "FAULT" ]]; then
        fail "准备阶段进入 FAULT：${RUNTIME_REASON}"
        return 1
      fi
      if (( RUNTIME_READY == 1 )) && [[ "${RUNTIME_STATE}" == "IDLE" ]]; then
        return 0
      fi
      if [[ "${RUNTIME_REASON}" != "${last_reason}" ]]; then
        info "准备状态：${RUNTIME_REASON}"
        last_reason="${RUNTIME_REASON}"
      fi
    fi
  done

  fail "自动驾驶运行图在 READY 前退出"
  return 1
}

wait_for_autonomy_start_decision() {
  local key=""
  local last_status="${RUNTIME_STATE}:${RUNTIME_READY}:${RUNTIME_REASON}"
  local current_status=""

  while kill -0 "${ACTIVE_PID}" >/dev/null 2>&1; do
    key=""
    if IFS= read -r -s -n 1 -t 0.2 key; then
      case "${key}" in
        q|Q) return 2 ;;
        s|S)
          if read_runtime_snapshot \
            && (( RUNTIME_READY == 1 )) \
            && [[ "${RUNTIME_STATE}" == "IDLE" ]]; then
            return 0
          fi
          warn "当前状态不是 READY，未执行开始请求；继续等待或按 Q 取消"
          ;;
      esac
    fi

    if read_runtime_snapshot; then
      if [[ "${RUNTIME_STATE}" == "FAULT" ]]; then
        fail "等待开始期间进入 FAULT：${RUNTIME_REASON}"
        return 1
      fi
      current_status="${RUNTIME_STATE}:${RUNTIME_READY}:${RUNTIME_REASON}"
      if [[ "${current_status}" != "${last_status}" ]]; then
        if (( RUNTIME_READY == 1 )) && [[ "${RUNTIME_STATE}" == "IDLE" ]]; then
          info "等待开始状态：READY / ${RUNTIME_REASON}"
        else
          info "等待开始状态：NOT_READY / ${RUNTIME_STATE} / ${RUNTIME_REASON}"
        fi
        last_status="${current_status}"
      fi
    fi
  done

  fail "自动驾驶运行图在等待 S/Q 时退出"
  return 1
}

call_race_service() {
  local action="$1"
  local output
  if ! output="$(
    timeout 5s ros2 service call \
      "/autoracer/race/${action}" \
      std_srvs/srv/Trigger \
      '{}' 2>&1
  )"; then
    fail "调用 /autoracer/race/${action} 失败"
    verbose_info "${output}"
    return 1
  fi
  if [[ "${output}" != *"success=True"* ]]; then
    fail "/autoracer/race/${action} 拒绝请求"
    verbose_info "${output}"
    return 1
  fi
  verbose_info "${output}"
}

runtime_fault_is_stopped() {
  (( RUNTIME_ENGAGED == 0 )) \
    && [[ "${RUNTIME_CONTROL_MODE}" =~ ^(0|4)$ ]] \
    && [[ "${RUNTIME_GEAR}" =~ ^(1|22)$ ]]
}

monitor_autonomy_run() {
  local key=""
  local last_state=""
  local last_reason=""
  local stop_requested=0
  local fault_reported=0

  info "自动驾驶运行中；按 Q 请求正常停车。"
  while kill -0 "${ACTIVE_PID}" >/dev/null 2>&1; do
    key=""
    if IFS= read -r -s -n 1 -t 0.2 key; then
      if [[ "${key}" =~ ^[qQ]$ ]] && (( stop_requested == 0 )); then
        if ! call_race_service stop; then
          return 1
        fi
        stop_requested=1
        info "已请求停车，等待 Hall 零速并退出自动模式。"
      fi
    fi

    if ! read_runtime_snapshot; then
      continue
    fi
    if [[ "${RUNTIME_STATE}" != "${last_state}" ]] \
      || [[ "${RUNTIME_REASON}" != "${last_reason}" ]]; then
      info "运行状态：${RUNTIME_STATE} / ${RUNTIME_REASON}"
      last_state="${RUNTIME_STATE}"
      last_reason="${RUNTIME_REASON}"
    fi

    case "${RUNTIME_STATE}" in
      FINISHED)
        ok "车辆已确认停止并退出自动模式"
        return 0
        ;;
      FAULT)
        if (( fault_reported == 0 )); then
          fail "自动驾驶进入 FAULT：${RUNTIME_REASON}"
          call_race_service stop || true
          fault_reported=1
        fi
        if runtime_fault_is_stopped; then
          ok "FAULT 后已确认停止并退出自动模式"
          return 1
        fi
        ;;
    esac
  done

  fail "自动驾驶运行图在确认停车前退出"
  return 1
}

start_autonomy() {
  if [[ ! -t 0 ]] && ! is_dry_run; then
    fail "自动驾驶入口必须在交互式终端运行，以便使用 S/Q"
    return 1
  fi

  if ! select_autonomy_asset; then
    acknowledge_autonomy_failure "自动驾驶资产检查未通过"
    return 1
  fi
  if ! select_driving_profile; then
    acknowledge_autonomy_failure "自动驾驶运行方案检查未通过"
    return 1
  fi
  if ! autonomy_authorized; then
    acknowledge_autonomy_failure "自动驾驶本次授权检查未通过"
    return 1
  fi

  local effective_speed
  effective_speed="$(
    awk \
      -v route="${AUTONOMY_ROUTE_MAX_SPEED}" \
      -v profile="${AUTONOMY_PROFILE_MAX_SPEED}" \
      'BEGIN {printf "%.3f", route < profile ? route : profile}'
  )"

  printf '\n'
  printf '场景：%s\n' "${AUTONOMY_MAP_ID}"
  printf '地图：%s\n' "${AUTONOMY_MAP_PATH}"
  printf '路线：%s\n' "${AUTONOMY_COURSE_PATH}"
  printf '运行方案：%s\n' "${AUTONOMY_PROFILE_NAME}"
  printf '路线最高目标速度：%.3f m/s\n' "${AUTONOMY_ROUTE_MAX_SPEED}"
  printf '实际速度上限：%s m/s\n' "${effective_speed}"
  printf '资产校验：PASS\n'
  printf '选择 7 已授权本次打开底盘控制串口；READY 前不会开始自动驾驶。\n'

  if ! device_available "底盘" "${CHASSIS_DEVICE}" \
    || ! device_available "IMU" "${IMU_DEVICE}" \
    || ! device_available "G90" "${G90_DEVICE}"; then
    acknowledge_autonomy_failure "自动驾驶设备路径检查未通过"
    return 1
  fi
  report_lidar_network

  if ! start_launch \
    "autonomy-${AUTONOMY_MAP_ID}-${AUTONOMY_PROFILE_ID}" \
    ros2 launch autoracer_rc_bringup race.launch.py \
    localization_map_path:="${AUTONOMY_MAP_PATH}" \
    course_path:="${AUTONOMY_COURSE_DIR}" \
    max_speed_mps:="${AUTONOMY_PROFILE_MAX_SPEED}" \
    max_accel_mps2:="${AUTONOMY_PROFILE_MAX_ACCEL}" \
    max_decel_mps2:="${AUTONOMY_PROFILE_MAX_DECEL}" \
    command_latency_sec:="${AUTONOMY_PROFILE_LATENCY}" \
    stopping_margin_m:="${AUTONOMY_PROFILE_STOPPING_MARGIN}" \
    chassis_serial_port:="${CHASSIS_DEVICE}" \
    imu_device:="${IMU_DEVICE}" \
    g90_device:="${G90_DEVICE}"; then
    acknowledge_autonomy_failure "自动驾驶运行图启动失败"
    return 1
  fi

  if is_dry_run; then
    info "dry-run 完成：未打开设备、未启动节点、未调用自动驾驶服务"
    return 0
  fi

  if ! start_runtime_state_watch; then
    acknowledge_autonomy_failure \
      "自动驾驶 runtime 状态订阅器启动失败；日志：${RUNTIME_STATE_WATCH_LOG}"
    stop_active
    return 1
  fi

  local ready_result
  if wait_for_autonomy_ready; then
    ready_result=0
  else
    ready_result=$?
  fi
  if (( ready_result != 0 )); then
    if (( ready_result == 2 )); then
      info "已取消自动驾驶准备"
      stop_active
      return 0
    fi
    acknowledge_autonomy_failure \
      "自动驾驶未达到 READY；启动日志：${ACTIVE_LOG}"
    stop_active
    return 1
  fi

  printf '\n'
  printf '定位：READY\n'
  printf 'Planning：READY\n'
  printf '底盘反馈：READY\n'
  printf '[S] 开始自动驾驶  [Q] 取消\n'

  local decision_result
  if wait_for_autonomy_start_decision; then
    decision_result=0
  else
    decision_result=$?
  fi
  if (( decision_result != 0 )); then
    if (( decision_result == 2 )); then
      info "已取消自动驾驶"
      stop_active
      return 0
    fi
    acknowledge_autonomy_failure \
      "自动驾驶在等待开始时失去可运行状态；启动日志：${ACTIVE_LOG}"
    stop_active
    return 1
  fi

  if ! call_race_service start; then
    acknowledge_autonomy_failure \
      "自动驾驶开始请求未执行；启动日志：${ACTIVE_LOG}"
    stop_active
    return 1
  fi
  ok "已接受本次自动驾驶开始请求"

  local run_result
  if monitor_autonomy_run; then
    run_result=0
  else
    run_result=$?
  fi
  if (( run_result != 0 )); then
    acknowledge_autonomy_failure \
      "自动驾驶运行未正常完成；启动日志：${ACTIVE_LOG}"
  fi
  stop_active
  return "${run_result}"
}

run_and_report() {
  local label="$1"
  shift
  if "$@"; then
    verbose_ok "${label} 完成"
  else
    fail "${label} 未通过"
  fi
}

print_menu() {
  printf '\n'
  printf 'Autoracer RC\n'
  printf '检查项会采样真实数据，完成后自动停止并返回菜单。\n'
  printf '建图录制会停在 READY 等待 S；录制中按 Q 封存并返回菜单。\n'
  printf '自动驾驶会加载获批资产并停在 READY；按 S 开始，运行中按 Q 停车。\n'
  printf '自动驾驶准备不设超时；故障结论显示后按 Q 清理返回。\n'
  if is_verbose; then
    printf '当前使用全量输出模式。\n'
  fi
  printf '  1. 检查 LiDAR 数据\n'
  printf '  2. 检查 IMU 数据\n'
  printf '  3. 检查 G90 数据\n'
  printf '  4. 检查底盘串口数据\n'
  printf '  5. 检查全部设备\n'
  printf '  6. 录制建图数据\n'
  printf '  7. 启动自动驾驶\n'
  printf '  0. 退出并清理\n'
}

interactive_menu() {
  INTERACTIVE=1
  local choice
  while true; do
    print_menu
    read -r -p '请选择：' choice || return 0
    case "${choice}" in
      1) run_and_report "LiDAR 检查" test_lidar ;;
      2) run_and_report "IMU 检查" test_imu ;;
      3) run_and_report "G90 检查" test_g90 ;;
      4) run_and_report "底盘串口检查" test_chassis ;;
      5) run_and_report "全部设备检查" test_all_devices ;;
      6) run_and_report "建图数据录制" record_mapping ;;
      7) run_and_report "自动驾驶" start_autonomy ;;
      0) return 0 ;;
      *) warn "无效选项：${choice}" ;;
    esac
  done
}

usage() {
  cat <<'EOF'
Usage:
  scripts/autoracer_rc.sh [-v|--verbose]
  scripts/autoracer_rc.sh [-v|--verbose] check lidar|imu|g90|chassis|devices
  scripts/autoracer_rc.sh [-v|--verbose] record mapping
  scripts/autoracer_rc.sh [-v|--verbose] start autonomy

No arguments opens the repeatable interactive menu. Checks start the real
production driver path, sample real topics, stop their owned process group,
and return. Mapping recording waits for READY, requires S to start, and uses
Q to stop, seal, validate, and hash the raw bag. Automatic driving lists
only hash-bound geographic map/course pairs, then automatically applies the
single approved named profile. It waits for READY without a menu timeout,
requires S to call the start service, and uses Q to request a confirmed stop.
A confirmed failure remains on screen until Q acknowledges cleanup.
Output is concise by default. Put -v or --verbose before the command to show
launch commands, topic types, field samples, and raw G90 NMEA observations.
Set RC_DRY_RUN=1 to print commands without opening devices.

Environment overrides:
  RC_G90_DEVICE, RC_IMU_DEVICE, RC_CHASSIS_DEVICE, RC_LIDAR_IP
  RC_MAP_ID, RC_MAP_ASSETS_ROOT, RC_OBSERVE_SEC, RC_RATE_WAIT_SEC
  RC_MAPPING_SITE, RC_MAPPING_LABEL, RC_MAPPING_OPERATOR
  RC_MAPPING_ALLOW_DEGRADED_LIDAR=1
    Authorize one mapping session when only lidar_rate/lidar_maximum_gap fail
  RC_DRIVING_PROFILE, RC_DRIVING_PROFILES_FILE
  RC_TELEMETRY_AUTHORIZED=1  Explicit authorization for one non-interactive run
  RC_AUTONOMY_AUTHORIZED=1   Authorize one non-interactive control-port open
EOF
}

main() {
  while (( $# > 0 )); do
    case "$1" in
      -v|--verbose)
        VERBOSE=1
        shift
        ;;
      --)
        shift
        break
        ;;
      *)
        break
        ;;
    esac
  done

  if (( $# == 0 )); then
    interactive_menu
    return
  fi

  case "$1" in
    -h|--help|help)
      usage
      ;;
    menu)
      interactive_menu
      ;;
    check)
      case "${2:-}" in
        lidar) test_lidar ;;
        imu) test_imu ;;
        g90) test_g90 ;;
        chassis) test_chassis ;;
        devices) test_all_devices ;;
        *) usage; return 2 ;;
      esac
      ;;
    record)
      case "${2:-}" in
        mapping) record_mapping ;;
        *) usage; return 2 ;;
      esac
      ;;
    start)
      case "${2:-}" in
        autonomy) start_autonomy ;;
        *) usage; return 2 ;;
      esac
      ;;
    *)
      usage
      return 2
      ;;
  esac
}

main "$@"
