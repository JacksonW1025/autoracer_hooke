#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'EOF'
Usage:
  rc_stop_mapping_bag.sh

Environment:
  RC_RUNTIME_DIR          runtime state directory, default: /tmp/autoracer_rc
  RC_MAPPING_STATE_FILE   state file from rc_start_mapping_bag.sh

Stops the active mapping bag gracefully, stops the RC sensing stack, prints
rosbag metadata, and removes the runtime state file.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "ERROR: unknown argument: $1" >&2
  usage >&2
  exit 1
fi

RC_RUNTIME_DIR="${RC_RUNTIME_DIR:-/tmp/autoracer_rc}"
STATE_FILE="${RC_MAPPING_STATE_FILE:-${RC_RUNTIME_DIR}/mapping_bag.env}"

if [[ ! -f "$STATE_FILE" ]]; then
  echo "ERROR: no active mapping bag state file: ${STATE_FILE}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$STATE_FILE"

if [[ -n "${REC_PID:-}" ]] && kill -0 "$REC_PID" 2>/dev/null; then
  echo "[rc-bag] stopping recorder pid ${REC_PID}"
  kill -INT -- "-${REC_PID}" 2>/dev/null || kill -INT "$REC_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$REC_PID" 2>/dev/null || break
    sleep 0.5
  done
fi

if [[ -n "${SENSOR_PID:-}" ]] && kill -0 "$SENSOR_PID" 2>/dev/null; then
  echo "[rc-bag] stopping sensor stack pid ${SENSOR_PID}"
  kill -TERM -- "-${SENSOR_PID}" 2>/dev/null || kill -TERM "$SENSOR_PID" 2>/dev/null || true
fi

./scripts/rc/rc_stop.sh >/dev/null 2>&1 || true

# shellcheck source=scripts/ros_env.sh
source "$ROOT_DIR/scripts/ros_env.sh"

if [[ -n "${BAG_PATH:-}" && -d "$BAG_PATH" ]]; then
  echo "[rc-bag] bag info: ${BAG_PATH}"
  ros2 bag info "$BAG_PATH"
else
  echo "[rc-bag] warning: bag path was not found: ${BAG_PATH:-unset}" >&2
fi

rm -f "$STATE_FILE"
echo "[rc-bag] sensor log: ${SENSOR_LOG:-unknown}"
echo "[rc-bag] record log: ${RECORD_LOG:-unknown}"
echo "[rc-bag] stopped"
