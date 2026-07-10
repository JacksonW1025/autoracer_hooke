#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'EOF'
Usage:
  MAP_PATH=<autoware_map_dir> rc_start_autoware.sh

Environment:
  MAP_PATH                required Autoware map directory
  ENABLE_DRIVE_COMMANDS   default: false
  LAUNCH_RVIZ             default: false

Starts the official Autoware launch path with the autoracer_rc vehicle,
autoracer_rc_sensor_kit sensors, safety gate, and RC serial interface.
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

if [[ -z "${MAP_PATH:-}" ]]; then
  echo "ERROR: MAP_PATH is required for Autoware startup." >&2
  usage >&2
  exit 1
fi

export LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
export ENABLE_DRIVE_COMMANDS="${ENABLE_DRIVE_COMMANDS:-false}"

exec ./scripts/run_official_autoware.sh
