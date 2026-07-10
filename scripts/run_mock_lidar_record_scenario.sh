#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=scripts/ros_env.sh
source "${ROOT_DIR}/scripts/ros_env.sh"

if [[ -f "${ROOT_DIR}/defaults.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/defaults.env"
  set +a
fi

MAP_PATH="${MAP_PATH:-${ROOT_DIR}/maps/whale_map_20251107}"
MOCK_LIDAR_SCENARIO_DIR="${MOCK_LIDAR_SCENARIO_DIR:-${MAP_PATH}/mock_lidar_scenarios}"
MAP_RVIZ_LEAF_SIZE="${MAP_RVIZ_LEAF_SIZE:-0.5}"

ros2 launch autoracer_bringup mock_lidar_record_scenario.launch.py \
  map_path:="${MAP_PATH}" \
  scenario_dir:="${MOCK_LIDAR_SCENARIO_DIR}" \
  map_leaf_size:="${MAP_RVIZ_LEAF_SIZE}" \
  "$@"
