#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'EOF'
Usage:
  rc_start_sensors.sh

Starts only C32 LiDAR, Hipnuc IMU, pointcloud filter, and static TF. This is the
vehicle-side sensor entry point for mapping bag capture and input diagnosis.
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

export LAUNCH_LOCALIZATION=false
export LAUNCH_PLANNING=false
export LAUNCH_CONTROL=false
export LAUNCH_SAFETY=false
export LAUNCH_VEHICLE=false
export LAUNCH_RVIZ=false
export ENABLE_DRIVE_COMMANDS=false

exec ./scripts/run_track.sh
