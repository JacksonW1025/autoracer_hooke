#!/usr/bin/env bash
set -euo pipefail

# One-button Hooke2 interface smoke test.
# It launches the Hooke2 vehicle interface stack, waits for real chassis feedback,
# then publishes Autoware-format control commands that Hooke2 converts to CAN.
#
# Defaults are intentionally low speed. Override with environment variables, e.g.:
#   TEST_SPEED_MPS=0.15 MOVE_DURATION_SEC=3 ./scripts/hooke2_autoware_control_smoke_test.sh 2
#
# Useful switches:
#   LAUNCH_INTERFACE=0    Use an already-running vehicle interface.
#   PUBLISH_FAKE_ODOM=1   Publish /localization/kinematic_state for legacy raw converters.
#   KEEP_LAUNCH_ALIVE=1   Do not stop the launched nodes on script exit.
#   SHOW_LAUNCH_OUTPUT=1  Show raw launch/node output instead of writing it to a log file.

usage() {
  cat <<'EOF'
Usage:
  scripts/hooke2_autoware_control_smoke_test.sh [1|2|3]

Test options:
  1                              Forward.
  2                              Left turn.
  3                              Right turn.

Environment overrides:
  WORKSPACE_ROOT                 Workspace root. Default: parent of this script directory.
  LAUNCH_INTERFACE               1 to launch autoracer_bringup vehicle.launch.py. Default: 1.
  SHOW_LAUNCH_OUTPUT             1 to show raw launch/node output. Default: 0.
  LAUNCH_LOG_FILE                Raw launch/node output path. Default: /tmp/hooke2_autoware_control_*.log.
  KEEP_LAUNCH_ALIVE              1 to leave launched nodes running after test. Default: 0.
  PUBLISH_FAKE_ODOM              1 to publish simple odometry for legacy raw converters. Default: 0.
  WAIT_FOR_HOOKE2_FEEDBACK        1 to wait for /vehicle/status/steering_status before AUTO. Default: 1.
  FEEDBACK_TIMEOUT_SEC           Wait time for real chassis feedback. Default: 30.
  DISCOVERY_TIMEOUT_SEC          Wait time for ROS graph discovery. Default: 20.
  COUNTDOWN_SEC                  Delay before requesting AUTONOMOUS. Default: 5.
  TEST_OPTION                    Same as positional [1|2|3]. Prompts when omitted.
  PUB_RATE_HZ                    Control command publish rate. Default: 20.
  TEST_SPEED_MPS                 Low-speed command. Default: 0.25.
  TEST_ACCEL_MPS2                Acceleration command. Default: 0.20.
  TEST_STEER_LEFT_RAD            Left steering tire angle. Default: 0.10.
  TEST_STEER_RIGHT_RAD           Right steering tire angle. Default: -0.10.
  MOVE_DURATION_SEC              Straight motion duration. Default: 5.
  STEER_DURATION_SEC             Each steering phase duration. Default: 3.
  STOP_DURATION_SEC              Stop command duration. Default: 3.
  STOP_DECEL_MPS2                Stop deceleration command. Default: -0.60.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# > 1 )); then
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-}"

LAUNCH_INTERFACE="${LAUNCH_INTERFACE:-1}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-autoracer_bringup}"
LAUNCH_FILE="${LAUNCH_FILE:-vehicle.launch.py}"
KEEP_LAUNCH_ALIVE="${KEEP_LAUNCH_ALIVE:-0}"
SHOW_LAUNCH_OUTPUT="${SHOW_LAUNCH_OUTPUT:-0}"
LAUNCH_LOG_FILE="${LAUNCH_LOG_FILE:-/tmp/hooke2_autoware_control_$(date +%Y%m%d_%H%M%S).log}"

PUBLISH_FAKE_ODOM="${PUBLISH_FAKE_ODOM:-0}"
WAIT_FOR_HOOKE2_FEEDBACK="${WAIT_FOR_HOOKE2_FEEDBACK:-1}"
FEEDBACK_TIMEOUT_SEC="${FEEDBACK_TIMEOUT_SEC:-30}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-20}"
PUB_ONCE_TIMEOUT_SEC="${PUB_ONCE_TIMEOUT_SEC:-8}"
COUNTDOWN_SEC="${COUNTDOWN_SEC:-5}"

PUB_RATE_HZ="${PUB_RATE_HZ:-20}"
CONTROL_CMD_MATCHING_SUBS="${CONTROL_CMD_MATCHING_SUBS:-1}"
TEST_OPTION="${1:-${TEST_OPTION:-}}"

TEST_SPEED_MPS="${TEST_SPEED_MPS:-0.25}"
TEST_ACCEL_MPS2="${TEST_ACCEL_MPS2:-0.20}"
TEST_STEER_LEFT_RAD="${TEST_STEER_LEFT_RAD:-0.10}"
TEST_STEER_RIGHT_RAD="${TEST_STEER_RIGHT_RAD:--0.10}"
MOVE_DURATION_SEC="${MOVE_DURATION_SEC:-5}"
STEER_DURATION_SEC="${STEER_DURATION_SEC:-3}"
STOP_DURATION_SEC="${STOP_DURATION_SEC:-3}"
STOP_DECEL_MPS2="${STOP_DECEL_MPS2:--0.60}"

CONTROL_TOPIC="${CONTROL_TOPIC:-/control/command/control_cmd}"
GEAR_TOPIC="${GEAR_TOPIC:-/control/command/gear_cmd}"
TURN_TOPIC="${TURN_TOPIC:-/control/command/turn_indicators_cmd}"
HAZARD_TOPIC="${HAZARD_TOPIC:-/control/command/hazard_lights_cmd}"
EMERGENCY_TOPIC="${EMERGENCY_TOPIC:-/control/command/emergency_cmd}"
CONTROL_MODE_SERVICE="${CONTROL_MODE_SERVICE:-/control/control_mode_request}"
FAKE_ODOM_TOPIC="${FAKE_ODOM_TOPIC:-/localization/kinematic_state}"
HOOKE2_READY_TOPIC="${HOOKE2_READY_TOPIC:-/vehicle/status/steering_status}"
HOOKE2_NODE_NAME="${HOOKE2_NODE_NAME:-/hooke2_interface}"

# Autoware message constants used by the current Hooke2 interface source.
GEAR_DRIVE=2
TURN_DISABLE=1
HAZARD_DISABLE=1
MODE_AUTONOMOUS=1
MODE_MANUAL=4

LAUNCH_PID=""
ODOM_PID=""
SETUP_DONE=0
STOP_SENT=0
TEST_OPTION_LABEL=""

require_integer() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[hooke2-test] ${name} must be a non-negative integer, got '${value}'." >&2
    exit 2
  fi
}

require_integer "PUB_RATE_HZ" "${PUB_RATE_HZ}"
require_integer "MOVE_DURATION_SEC" "${MOVE_DURATION_SEC}"
require_integer "STEER_DURATION_SEC" "${STEER_DURATION_SEC}"
require_integer "STOP_DURATION_SEC" "${STOP_DURATION_SEC}"
require_integer "COUNTDOWN_SEC" "${COUNTDOWN_SEC}"
require_integer "FEEDBACK_TIMEOUT_SEC" "${FEEDBACK_TIMEOUT_SEC}"
require_integer "DISCOVERY_TIMEOUT_SEC" "${DISCOVERY_TIMEOUT_SEC}"
require_integer "PUB_ONCE_TIMEOUT_SEC" "${PUB_ONCE_TIMEOUT_SEC}"

select_test_option() {
  local option="${TEST_OPTION}"

  if [[ -z "${option}" ]]; then
    if [[ ! -t 0 ]]; then
      echo "[hooke2-test] Choose a test option with argument 1/2/3 or TEST_OPTION=1/2/3." >&2
      exit 2
    fi

    echo "[hooke2-test] Select test maneuver:"
    echo "[hooke2-test]   1) Forward"
    echo "[hooke2-test]   2) Left turn"
    echo "[hooke2-test]   3) Right turn"
    read -r -p "[hooke2-test] Enter option [1-3]: " option
  fi

  case "${option}" in
    1|forward|Forward|straight|Straight)
      TEST_OPTION=1
      TEST_OPTION_LABEL="Forward"
      ;;
    2|left|Left)
      TEST_OPTION=2
      TEST_OPTION_LABEL="Left turn"
      ;;
    3|right|Right)
      TEST_OPTION=3
      TEST_OPTION_LABEL="Right turn"
      ;;
    *)
      echo "[hooke2-test] Invalid test option '${option}'. Use 1, 2, or 3." >&2
      exit 2
      ;;
  esac

  echo "[hooke2-test] Selected test: ${TEST_OPTION_LABEL} (${TEST_OPTION})"
}

source_workspace() {
  set +u
  if [[ -n "${SETUP_FILE}" ]]; then
    if [[ ! -f "${SETUP_FILE}" ]]; then
      echo "[hooke2-test] Cannot find setup file: ${SETUP_FILE}" >&2
      exit 1
    fi
    # shellcheck source=/dev/null
    source "${SETUP_FILE}"
  else
    ROOT_DIR="${WORKSPACE_ROOT}"
    # shellcheck source=scripts/ros_env.sh
    source "${WORKSPACE_ROOT}/scripts/ros_env.sh"
  fi
  set -u
  SETUP_DONE=1
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start_time="${SECONDS}"

  echo "[hooke2-test] Waiting for service ${service_name} ..."
  while (( SECONDS - start_time < timeout_sec )); do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      echo "[hooke2-test] Service ready: ${service_name}"
      return 0
    fi
    sleep 1
  done

  echo "[hooke2-test] Timed out waiting for service: ${service_name}" >&2
  return 1
}

wait_for_subscribers() {
  local topic_name="$1"
  local min_count="$2"
  local timeout_sec="$3"
  local start_time="${SECONDS}"
  local count=""

  echo "[hooke2-test] Waiting for at least ${min_count} subscriber(s) on ${topic_name} ..."
  while (( SECONDS - start_time < timeout_sec )); do
    count="$(
      ros2 topic info "${topic_name}" 2>/dev/null |
        awk '/Subscription count:/ {print $3}'
    )"
    if [[ -n "${count}" && "${count}" -ge "${min_count}" ]]; then
      echo "[hooke2-test] Topic ready: ${topic_name} has ${count} subscriber(s)."
      return 0
    fi
    sleep 1
  done

  echo "[hooke2-test] Timed out waiting for subscribers on ${topic_name}." >&2
  return 1
}

node_exists() {
  local node_name="$1"
  ros2 node list 2>/dev/null | grep -qx "${node_name}"
}

wait_for_node() {
  local node_name="$1"
  local timeout_sec="$2"
  local start_time="${SECONDS}"

  echo "[hooke2-test] Waiting for node ${node_name} ..."
  while (( SECONDS - start_time < timeout_sec )); do
    if node_exists "${node_name}"; then
      echo "[hooke2-test] Node ready: ${node_name}"
      return 0
    fi
    sleep 1
  done

  echo "[hooke2-test] Timed out waiting for node: ${node_name}" >&2
  return 1
}

ensure_hooke2_alive() {
  if node_exists "${HOOKE2_NODE_NAME}"; then
    return 0
  fi

  echo "[hooke2-test] ${HOOKE2_NODE_NAME} is not alive; stopping test." >&2
  return 1
}

wait_for_message() {
  local topic_name="$1"
  local timeout_sec="$2"

  echo "[hooke2-test] Waiting for one message on ${topic_name} ..."
  if timeout "${timeout_sec}" ros2 topic echo --once "${topic_name}" >/dev/null; then
    echo "[hooke2-test] Received feedback on ${topic_name}."
    return 0
  fi

  echo "[hooke2-test] Timed out waiting for ${topic_name}." >&2
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
  local output=""

  echo "[hooke2-test] Command: control_mode=${label} (${mode})"
  if ! output="$(
    timeout "${PUB_ONCE_TIMEOUT_SEC}" \
    ros2 service call "${CONTROL_MODE_SERVICE}" \
      autoware_vehicle_msgs/srv/ControlModeCommand \
      "{stamp: {sec: 0, nanosec: 0}, mode: ${mode}}" 2>&1
  )"; then
    echo "${output}" >&2
    return 1
  fi

  if ! grep -q "success=True" <<< "${output}"; then
    echo "[hooke2-test] Control mode request did not report success:" >&2
    echo "${output}" >&2
    return 1
  fi
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

  ensure_hooke2_alive
  echo "[hooke2-test] Command: ${label}, velocity=${velocity} m/s, acceleration=${acceleration} m/s^2, steering_tire_angle=${steering} rad, steering_rate=${steering_rate} rad/s, duration=${duration_sec}s"
  timeout "${timeout_sec}" \
    ros2 topic pub -p 0 -r "${PUB_RATE_HZ}" -t "${times}" \
      "${CONTROL_TOPIC}" autoware_control_msgs/msg/Control \
      "$(control_yaml "${velocity}" "${acceleration}" "${steering}" "${steering_rate}")" >/dev/null
  ensure_hooke2_alive
}

run_selected_maneuver() {
  case "${TEST_OPTION}" in
    1)
      publish_control_for "Forward" "${TEST_SPEED_MPS}" "${TEST_ACCEL_MPS2}" "0.0" "0.0" "${MOVE_DURATION_SEC}"
      ;;
    2)
      publish_control_for "Left turn" "${TEST_SPEED_MPS}" "${TEST_ACCEL_MPS2}" "${TEST_STEER_LEFT_RAD}" "0.10" "${STEER_DURATION_SEC}"
      ;;
    3)
      publish_control_for "Right turn" "${TEST_SPEED_MPS}" "${TEST_ACCEL_MPS2}" "${TEST_STEER_RIGHT_RAD}" "-0.10" "${STEER_DURATION_SEC}"
      ;;
    *)
      echo "[hooke2-test] Internal error: invalid test option '${TEST_OPTION}'." >&2
      exit 2
      ;;
  esac
}

publish_static_commands() {
  echo "[hooke2-test] Command: gear=D, turn=DISABLE, hazard=DISABLE, emergency=false"
  pub_once "${GEAR_TOPIC}" autoware_vehicle_msgs/msg/GearCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${GEAR_DRIVE}}"
  pub_once "${TURN_TOPIC}" autoware_vehicle_msgs/msg/TurnIndicatorsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${TURN_DISABLE}}"
  pub_once "${HAZARD_TOPIC}" autoware_vehicle_msgs/msg/HazardLightsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${HAZARD_DISABLE}}"
  pub_once "${EMERGENCY_TOPIC}" tier4_vehicle_msgs/msg/VehicleEmergencyStamped \
    "{stamp: {sec: 0, nanosec: 0}, emergency: false}"
}

start_fake_odometry() {
  if [[ "${PUBLISH_FAKE_ODOM}" != "1" ]]; then
    return 0
  fi

  echo "[hooke2-test] Publishing simple odometry on ${FAKE_ODOM_TOPIC} for legacy raw converters."
  ros2 topic pub -p 0 -r 20 "${FAKE_ODOM_TOPIC}" nav_msgs/msg/Odometry \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: map}, child_frame_id: base_link, pose: {pose: {orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0}}}}" >/dev/null &
  ODOM_PID="$!"
}

launch_interface() {
  if [[ "${LAUNCH_INTERFACE}" != "1" ]]; then
    echo "[hooke2-test] LAUNCH_INTERFACE=0, assuming Hooke2 interface is already running."
    return 0
  fi

  echo "[hooke2-test] Launching ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ..."
  if [[ "${SHOW_LAUNCH_OUTPUT}" == "1" ]]; then
    ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" &
  else
    mkdir -p "$(dirname "${LAUNCH_LOG_FILE}")"
    echo "[hooke2-test] Raw launch output: ${LAUNCH_LOG_FILE}"
    ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" >"${LAUNCH_LOG_FILE}" 2>&1 &
  fi
  LAUNCH_PID="$!"
  sleep 2

  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[hooke2-test] ros2 launch exited early." >&2
    wait "${LAUNCH_PID}" || true
    exit 1
  fi
}

countdown() {
  local seconds="$1"
  if (( seconds <= 0 )); then
    return 0
  fi

  echo "[hooke2-test] Requesting AUTONOMOUS in ${seconds}s. Press Ctrl-C to abort."
  while (( seconds > 0 )); do
    echo "[hooke2-test] ${seconds} ..."
    sleep 1
    seconds=$((seconds - 1))
  done
}

send_stop_and_manual() {
  if [[ "${STOP_SENT}" == "1" || "${SETUP_DONE}" != "1" ]]; then
    return 0
  fi

  set +e
  STOP_SENT=1

  if ! node_exists "${HOOKE2_NODE_NAME}"; then
    echo "[hooke2-test] ${HOOKE2_NODE_NAME} is already down; skip stop/MANUAL commands."
    set -e
    return 0
  fi

  echo "[hooke2-test] Sending stop command and requesting MANUAL."
  publish_control_for "Stop" "0.0" "${STOP_DECEL_MPS2}" "0.0" "0.0" "${STOP_DURATION_SEC}"
  pub_once "${TURN_TOPIC}" autoware_vehicle_msgs/msg/TurnIndicatorsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${TURN_DISABLE}}"
  pub_once "${HAZARD_TOPIC}" autoware_vehicle_msgs/msg/HazardLightsCommand \
    "{stamp: {sec: 0, nanosec: 0}, command: ${HAZARD_DISABLE}}"
  pub_once "${EMERGENCY_TOPIC}" tier4_vehicle_msgs/msg/VehicleEmergencyStamped \
    "{stamp: {sec: 0, nanosec: 0}, emergency: false}"
  call_control_mode "${MODE_MANUAL}" "MANUAL"
  set -e
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  send_stop_and_manual || true

  if [[ -n "${ODOM_PID}" ]]; then
    kill "${ODOM_PID}" 2>/dev/null || true
    wait "${ODOM_PID}" 2>/dev/null || true
  fi

  if [[ "${LAUNCH_INTERFACE}" == "1" && "${KEEP_LAUNCH_ALIVE}" != "1" && -n "${LAUNCH_PID}" ]]; then
    echo "[hooke2-test] Stopping launched Hooke2 interface stack."
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
  fi

  exit "${status}"
}

trap cleanup EXIT INT TERM

select_test_option
source_workspace
launch_interface
start_fake_odometry

wait_for_node "${HOOKE2_NODE_NAME}" "${DISCOVERY_TIMEOUT_SEC}"
wait_for_service "${CONTROL_MODE_SERVICE}" "${DISCOVERY_TIMEOUT_SEC}"
wait_for_subscribers "${CONTROL_TOPIC}" "${CONTROL_CMD_MATCHING_SUBS}" "${DISCOVERY_TIMEOUT_SEC}"
wait_for_subscribers "${GEAR_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}"
wait_for_subscribers "${TURN_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}"
wait_for_subscribers "${HAZARD_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}"
wait_for_subscribers "${EMERGENCY_TOPIC}" 1 "${DISCOVERY_TIMEOUT_SEC}"

if [[ "${WAIT_FOR_HOOKE2_FEEDBACK}" == "1" ]]; then
  wait_for_message "${HOOKE2_READY_TOPIC}" "${FEEDBACK_TIMEOUT_SEC}"
fi

publish_static_commands
countdown "${COUNTDOWN_SEC}"
call_control_mode "${MODE_AUTONOMOUS}" "AUTONOMOUS"

run_selected_maneuver

send_stop_and_manual

echo "[hooke2-test] Done."
