#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

AUTORACER_SOURCE_VENDOR_SETUP=true
AUTORACER_SOURCE_PRODUCT_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "${PRODUCT_ROOT}/scripts/ros_env.sh"

cd "${PRODUCT_ROOT}"
colcon build \
  --base-paths src/core src/platform \
  --symlink-install \
  --parallel-workers "${COLCON_PARALLEL_WORKERS:-4}" \
  --packages-up-to autoracer_bringup autoracer_hooke2_bringup autoracer_rc_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
