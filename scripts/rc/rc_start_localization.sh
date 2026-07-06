#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'EOF'
Usage:
  MAP_PATH=<map_dir> rc_start_localization.sh

Environment:
  MAP_PATH       required map directory
  LAUNCH_RVIZ    default: true

Starts sensing + NDT localization only. Planning, control, safety, and vehicle
interface are disabled. If map_projector_info.yaml is absent, the map projection
loader is disabled for PCD-only localization checks.
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
  echo "ERROR: MAP_PATH is required." >&2
  usage >&2
  exit 1
fi

export LAUNCH_PLANNING="${LAUNCH_PLANNING:-false}"
export LAUNCH_CONTROL="${LAUNCH_CONTROL:-false}"
export LAUNCH_SAFETY="${LAUNCH_SAFETY:-false}"
export LAUNCH_VEHICLE="${LAUNCH_VEHICLE:-false}"
export LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
export ENABLE_DRIVE_COMMANDS="${ENABLE_DRIVE_COMMANDS:-false}"

if [[ ! -f "${MAP_PATH}/map_projector_info.yaml" ]]; then
  export LAUNCH_MAP_PROJECTION_LOADER="${LAUNCH_MAP_PROJECTION_LOADER:-false}"
fi

exec ./scripts/run_track.sh
