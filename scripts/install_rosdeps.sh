#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${ROOT_DIR}/vendor_ws}"
cd "${ROOT_DIR}"

AUTORACER_SOURCE_VENDOR_SETUP=false
AUTORACER_SOURCE_PRODUCT_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "${ROOT_DIR}/scripts/ros_env.sh"
rosdep update
rosdep install --from-paths "${VENDOR_WS}/src" "${ROOT_DIR}/src" --ignore-src -y -r
