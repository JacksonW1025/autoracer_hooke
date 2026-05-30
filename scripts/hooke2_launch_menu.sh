#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOCAL_SETUP="${ROOT_DIR}/install/local_setup.bash"
LOG_DIR="${ROOT_DIR}/log/hooke2_launch_menu"

cd "${ROOT_DIR}" || exit 1

source_or_exit() {
  local label="$1"
  local setup_file="$2"

  if [[ ! -f "${setup_file}" ]]; then
    echo "[FAIL] ${label}: missing ${setup_file}" >&2
    exit 1
  fi

  local had_nounset=0
  case $- in
    *u*) had_nounset=1 ;;
  esac

  # shellcheck disable=SC1090
  set +u
  if source "${setup_file}"; then
    if [[ "${had_nounset}" == "1" ]]; then
      set -u
    fi
    echo "[ OK ] ${label}: ${setup_file}"
  else
    if [[ "${had_nounset}" == "1" ]]; then
      set -u
    fi
    echo "[FAIL] ${label}: source failed: ${setup_file}" >&2
    exit 1
  fi
}

load_defaults() {
  if [[ -f "${ROOT_DIR}/defaults.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/defaults.env"
    set +a
    echo "[ OK ] Loaded defaults.env"
  fi

  LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.1.120}"
  LIDAR_SENSOR_IP="${LIDAR_SENSOR_IP:-192.168.1.130}"
  LIDAR_DATA_PORT="${LIDAR_DATA_PORT:-2368}"
  LIDAR_SENSOR_MODEL="${LIDAR_SENSOR_MODEL:-Pandar40P}"
}

collect_ros_pids() {
  {
    pgrep -f -- "ros2 launch" || true
    pgrep -f -- "ros2 run" || true
    pgrep -f -- "--ros-args" || true
    pgrep -f -- "rviz2" || true
    pgrep -f -- "rqt" || true
    pgrep -f -- "robot_state_publisher" || true
    pgrep -f -- "static_transform_publisher" || true
    pgrep -f -- "component_container" || true
    pgrep -f -- "hesai_ros_wrapper" || true
    pgrep -f -- "fixposition_driver" || true
  } | sort -u | while read -r pid; do
    [[ -z "${pid}" ]] && continue
    [[ "${pid}" == "$$" ]] && continue
    [[ "${pid}" == "${BASHPID}" ]] && continue
    echo "${pid}"
  done
}

send_signal_to_ros() {
  local signal="$1"
  local pids
  pids="$(collect_ros_pids | tr '\n' ' ')"

  if [[ -z "${pids// }" ]]; then
    return 1
  fi

  echo "[INFO] Sending ${signal} to: ${pids}"
  # shellcheck disable=SC2086
  kill "-${signal}" ${pids} 2>/dev/null || true
  return 0
}

cleanup_ros() {
  echo "[INFO] Cleaning ROS 2 launch/node/RViz/RQT processes..."
  if ! send_signal_to_ros INT; then
    echo "[INFO] No matching ROS 2 processes found."
    return 0
  fi

  sleep 2
  send_signal_to_ros TERM || true
  sleep 1
  send_signal_to_ros KILL || true
  ros2 daemon stop >/dev/null 2>&1 || true
  echo "[ OK ] Cleanup requested."
}

start_launch() {
  local name="$1"
  shift

  mkdir -p "${LOG_DIR}"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local log_file="${LOG_DIR}/${name}_${stamp}.log"

  echo "[INFO] Starting ${name}"
  echo "[INFO] Log: ${log_file}"
  (
    cd "${ROOT_DIR}" || exit 1
    "$@"
  ) >"${log_file}" 2>&1 &

  local pid=$!
  echo "[ OK ] ${name} started, launcher pid=${pid}"
}

print_menu() {
  cat <<'EOF'

请选择操作：
  0) 清理当前 ROS 2 launch/node/RViz2/RQT 进程
  1) 在 RViz 中展示 Hooke2 完整车辆 URDF
  2) 启动 LiDAR 点云 + Hooke2 车辆 URDF 的统一 RViz 展示
  3) 控制话题/CAN 发送链路单次驱动验证

按 Ctrl-C 退出脚本；启动后的进程可用选项 0 清理。
EOF
}

main_loop() {
  while true; do
    print_menu
    printf "输入选项 [0/1/2/3]: "
    if ! IFS= read -r choice; then
      echo
      exit 0
    fi

    case "${choice}" in
      0)
        cleanup_ros
        ;;
      1)
        start_launch "vehicle_urdf_rviz" \
          ros2 launch hooke2_description vehicle_rviz.launch.py
        ;;
      2)
        start_launch "lidar_vehicle_rviz" \
          ros2 launch autoracer_bringup lidar_vehicle_rviz.launch.py \
            lidar_host_ip:="${LIDAR_HOST_IP}" \
            lidar_sensor_ip:="${LIDAR_SENSOR_IP}" \
            lidar_data_port:="${LIDAR_DATA_PORT}" \
            lidar_sensor_model:="${LIDAR_SENSOR_MODEL}"
        ;;
      3)
        "${ROOT_DIR}/scripts/hooke2_control_can_test.sh"
        ;;
      *)
        echo "[WARN] Unknown option: ${choice}"
        ;;
    esac
  done
}

trap 'echo; echo "[INFO] Script exited. Use option 0 next time if ROS processes need cleanup."; exit 0' INT

source_or_exit "ROS Humble environment" "${ROS_SETUP}"
source_or_exit "Autoware Hooke local setup" "${LOCAL_SETUP}"
load_defaults
main_loop
