#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

profile="full"
case "${1:-}" in
  "") ;;
  --rc) profile="rc" ;;
  *)
    echo "Usage: $0 [--rc]" >&2
    exit 2
    ;;
esac
[[ $# -le 1 ]] || {
  echo "Usage: $0 [--rc]" >&2
  exit 2
}

export PYTHONNOUSERSITE=1
AUTORACER_SOURCE_VENDOR_SETUP=true
AUTORACER_SOURCE_PRODUCT_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "${PRODUCT_ROOT}/scripts/ros_env.sh"

cd "${PRODUCT_ROOT}"
if [[ "${profile}" == "rc" ]]; then
  colcon build \
    --base-paths src/core src/platform/rc \
    --symlink-install \
    --parallel-workers "${COLCON_PARALLEL_WORKERS:-4}" \
    --packages-up-to autoracer_bringup autoracer_rc_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
else
  colcon build \
    --base-paths src/core src/platform \
    --symlink-install \
    --parallel-workers "${COLCON_PARALLEL_WORKERS:-4}" \
    --packages-up-to autoracer_bringup autoracer_hooke2_bringup autoracer_rc_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
fi
