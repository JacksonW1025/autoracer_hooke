#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${PRODUCT_ROOT}/vendor_ws}"

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

PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages.tsv"
if [[ "${profile}" == "rc" ]]; then
  PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages-rc.tsv"
fi

export PYTHONNOUSERSITE=1
AUTORACER_SOURCE_VENDOR_SETUP=false
AUTORACER_SOURCE_PRODUCT_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "${PRODUCT_ROOT}/scripts/ros_env.sh"

mapfile -t packages < <(cut -f1 "${PACKAGE_MANIFEST}")
if ((${#packages[@]} == 0)); then
  echo "No packages listed in ${PACKAGE_MANIFEST}" >&2
  exit 1
fi

underlay_overrides=(
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_lanelet2_extension
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_geometry
)

cmake_build_type="${CMAKE_BUILD_TYPE:-Release}"
cmake_cxx_flags="${CMAKE_CXX_FLAGS:-} -I${VENDOR_WS}/install/autoware_lanelet2_extension/include"

cd "${VENDOR_WS}"
colcon build \
  --symlink-install \
  --parallel-workers "${COLCON_PARALLEL_WORKERS:-4}" \
  --allow-overriding "${underlay_overrides[@]}" \
  --packages-select "${packages[@]}" \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE="${cmake_build_type}" \
    -DCMAKE_CXX_FLAGS="${cmake_cxx_flags}"
