#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

cat >&2 <<EOF
ERROR: Hooke profile is disabled and not runtime ready.

The reserved Hooke official profile directories live under:
  ${ROOT_DIR}/src/autoracer_hooke_description
  ${ROOT_DIR}/src/autoracer_hooke_launch
  ${ROOT_DIR}/src/autoracer_hooke_sensor_kit_description
  ${ROOT_DIR}/src/autoracer_hooke_sensor_kit_launch

Each directory is guarded by COLCON_IGNORE. Remove COLCON_IGNORE only after the
real Hooke vehicle, sensor, CAN adapter, and sensing launch files are complete.
EOF

exit 2
