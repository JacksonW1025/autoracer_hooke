#!/usr/bin/env bash
set -euo pipefail

TOOL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${TOOL_DIR}/../.." && pwd)"
BAG_PATH="${1:?usage: run_super_lio_course_replay.sh <bag_path> <map_id>}"
MAP_ID="${2:?usage: run_super_lio_course_replay.sh <bag_path> <map_id>}"
MAPPING_WS="${MAPPING_WS:-$(dirname "${REPO_ROOT}")/rc_mapping_ws}"
MAPPING_DATA_DIR="${MAPPING_DATA_DIR:-$(dirname "${REPO_ROOT}")/rc_mapping_data}"
CONFIG="${CONFIG:-${MAPPING_DATA_DIR}/runs/${MAP_ID}/rc_c32_super_lio.yaml}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
REPLAY_DIR="${MAPPING_DATA_DIR}/course_replays/${MAP_ID}"
ODOM_BAG="${REPLAY_DIR}/lio_odom"
SUPER_LIO_REPO="${MAPPING_WS}/src/Super-LIO"

for path in "${BAG_PATH}" "${CONFIG}" "${MAPPING_WS}/install/setup.bash" "${SUPER_LIO_REPO}"; do
  [[ -e "${path}" ]] || { echo "ERROR: missing required path: ${path}" >&2; exit 1; }
done
[[ ! -e "${REPLAY_DIR}" ]] || {
  echo "ERROR: replay directory already exists: ${REPLAY_DIR}" >&2
  exit 1
}

mkdir -p "${REPLAY_DIR}/ros_log"
OUTPUT_ENV_FILE="${REPLAY_DIR}/selected_topics.env" \
  "${TOOL_DIR}/inspect_bag_topics.sh" "${BAG_PATH}" \
  | tee "${REPLAY_DIR}/bag_inspection.txt"
source "${REPLAY_DIR}/selected_topics.env"
cp "${CONFIG}" "${REPLAY_DIR}/rc_c32_super_lio.yaml"
git -C "${SUPER_LIO_REPO}" rev-parse HEAD > "${REPLAY_DIR}/super_lio_commit.txt"
git -C "${SUPER_LIO_REPO}" status --short > "${REPLAY_DIR}/super_lio_status.txt"
sha256sum "${BAG_PATH}/metadata.yaml" "${CONFIG}" \
  > "${REPLAY_DIR}/source_sha256.txt"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${MAPPING_WS}/install/setup.bash"
set -u
export ROS_LOG_DIR="${REPLAY_DIR}/ros_log"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"

lio_pid=""
rec_pid=""

stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  pgrep -g "${pid}" >/dev/null 2>&1 || return 0
  kill -INT "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    pgrep -g "${pid}" >/dev/null 2>&1 || return 0
    sleep 1
  done
  kill -TERM "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    pgrep -g "${pid}" >/dev/null 2>&1 || return 0
    sleep 1
  done
  kill -KILL "-${pid}" 2>/dev/null || true
}

cleanup() {
  set +e
  stop_group "${rec_pid}"
  stop_group "${lio_pid}"
}
trap cleanup EXIT

cd "${REPLAY_DIR}"
setsid ros2 run super_lio super_lio_node --ros-args \
  --params-file "${REPLAY_DIR}/rc_c32_super_lio.yaml" \
  -p "lio.map.save_map:=false" \
  -p "lio.ros.lidar_topic:=${LIDAR_TOPIC}" \
  -p "lio.ros.imu_topic:=${IMU_TOPIC}" > super_lio.log 2>&1 &
lio_pid=$!

sleep 2
setsid ros2 bag record -o "${ODOM_BAG}" /lio/odom > odom_record.log 2>&1 &
rec_pid=$!
sleep 2
ros2 bag play "${BAG_PATH}" --clock --rate "${PLAYBACK_RATE}" > bag_play.log 2>&1
sleep 2
stop_group "${rec_pid}"
rec_pid=""
stop_group "${lio_pid}"
lio_pid=""
trap - EXIT

info="$(ros2 bag info "${ODOM_BAG}")"
printf '%s\n' "${info}" | tee "${REPLAY_DIR}/odom_bag_info.txt"
grep -Eq 'Topic: /lio/odom \|.*Count: [1-9][0-9]*' <<< "${info}" || {
  echo "ERROR: replay generated no /lio/odom messages" >&2
  exit 1
}

echo "[course-replay] completed: ${REPLAY_DIR}"
