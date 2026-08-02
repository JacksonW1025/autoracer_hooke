#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${ROOT_DIR}/vendor_ws}"
cd "${ROOT_DIR}"

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

AUTORACER_SOURCE_VENDOR_SETUP=false
AUTORACER_SOURCE_PRODUCT_SETUP=false
# shellcheck source=scripts/ros_env.sh
source "${ROOT_DIR}/scripts/ros_env.sh"

if [[ "${profile}" == "rc" ]]; then
  if ! rosdep update; then
    echo "rosdep update failed; validating the existing local cache for the locked RC profile." >&2
    rosdep db >/dev/null
  fi
  rc_unused_localization_keys=(
    autoware_ar_tag_based_localizer
    autoware_geo_pose_projector
    autoware_lidar_marker_localizer
    autoware_pose_estimator_arbiter
    eagleye_geo_pose_fusion
    eagleye_gnss_converter
    eagleye_rt
    yabloc_common
    yabloc_image_processing
    yabloc_monitor
    yabloc_particle_filter
    yabloc_pose_initializer
  )
  rc_skip_keys="ament_python ${rc_unused_localization_keys[*]}"
  rosdep install \
    --from-paths "${VENDOR_WS}/src" "${ROOT_DIR}/src/core" "${ROOT_DIR}/src/platform/rc" \
    --ignore-src \
    --default-yes \
    --dependency-types build \
    --dependency-types build_export \
    --dependency-types buildtool \
    --dependency-types buildtool_export \
    --dependency-types exec \
    --skip-keys "${rc_skip_keys}"
  rosdep install \
    --from-paths "${ROOT_DIR}/src/core" "${ROOT_DIR}/src/platform/rc" \
    --ignore-src \
    --default-yes \
    --dependency-types test \
    --skip-keys pytest
else
  rosdep update
  rosdep install --from-paths "${VENDOR_WS}/src" "${ROOT_DIR}/src" --ignore-src -y -r
fi
