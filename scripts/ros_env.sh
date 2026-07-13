#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source scripts/ros_env.sh from another script or shell"
  exit 0
fi

_autoracer_filter_path_var() {
  local var_name="$1"
  local blocked_prefix="$2"
  local value="${!var_name:-}"
  local filtered=""
  local entry

  [[ -n "${value}" ]] || return 0
  while IFS= read -r -d ':' entry; do
    [[ -n "${entry}" ]] || continue
    [[ "${entry}" == "${blocked_prefix}"* ]] && continue
    filtered="${filtered:+${filtered}:}${entry}"
  done < <(printf '%s:' "${value}")
  export "${var_name}=${filtered}"
}

_autoracer_filter_prefix() {
  local blocked_prefix="$1"
  local var_name
  for var_name in \
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
    LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH PATH
  do
    _autoracer_filter_path_var "${var_name}" "${blocked_prefix}"
  done
}

_autoracer_filter_conda_paths() {
  local prefix
  for prefix in "/opt/anaconda3" "${HOME:-}/anaconda3" "${HOME:-}/miniconda3"; do
    [[ -n "${prefix}" ]] && _autoracer_filter_prefix "${prefix}"
  done
  export CMAKE_IGNORE_PREFIX_PATH="/opt/anaconda3${CMAKE_IGNORE_PREFIX_PATH:+:${CMAKE_IGNORE_PREFIX_PATH}}"
  export CMAKE_IGNORE_PATH="/opt/anaconda3/bin:/opt/anaconda3/include:/opt/anaconda3/lib${CMAKE_IGNORE_PATH:+:${CMAKE_IGNORE_PATH}}"
}

_autoracer_source_setup() {
  local setup_file="$1"
  local label="$2"
  if [[ ! -f "${setup_file}" ]]; then
    echo "[autoracer-env] Missing ${label} setup: ${setup_file}" >&2
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  if [[ "${_autoracer_had_nounset}" == "1" ]]; then
    set -u
  fi
}

AUTORACER_PRODUCT_ROOT="${AUTORACER_PRODUCT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
AUTORACER_ROOT_DIR="$(dirname "${AUTORACER_PRODUCT_ROOT}")"
AUTORACER_VENDOR_WS="${AUTORACER_VENDOR_WS:-${AUTORACER_PRODUCT_ROOT}/vendor_ws}"
AUTORACER_OLD_REPO="${AUTORACER_OLD_REPO:-${AUTORACER_ROOT_DIR}/pilot-auto.x1}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

_autoracer_had_nounset=0
case $- in
  *u*) _autoracer_had_nounset=1 ;;
esac

unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
_autoracer_filter_prefix "${AUTORACER_OLD_REPO}"
if [[ "${AUTORACER_FILTER_CONDA:-true}" == "true" ]]; then
  _autoracer_filter_conda_paths
fi
if [[ -x /usr/bin/protoc ]]; then
  export PATH="/usr/bin:${PATH}"
fi

_autoracer_source_setup "${ROS_SETUP}" "ROS"
_autoracer_filter_prefix "${AUTORACER_OLD_REPO}"

if [[ "${AUTORACER_SOURCE_VENDOR_SETUP:-true}" == "true" ]]; then
  _autoracer_source_setup "${AUTORACER_VENDOR_WS}/install/local_setup.bash" "vendor underlay"
fi

source_product="${AUTORACER_SOURCE_PRODUCT_SETUP:-${AUTORACER_SOURCE_LOCAL_SETUP:-true}}"
if [[ "${source_product}" == "true" ]]; then
  _autoracer_source_setup "${AUTORACER_PRODUCT_ROOT}/install/local_setup.bash" "product overlay"
fi

_autoracer_filter_prefix "${AUTORACER_OLD_REPO}"
if [[ "${AUTORACER_FILTER_CONDA:-true}" == "true" ]]; then
  _autoracer_filter_conda_paths
fi

if [[ "${AUTORACER_FORBID_OLD_UNDERLAY:-true}" == "true" ]]; then
  for var_name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
    if [[ ":${!var_name:-}:" == *":${AUTORACER_OLD_REPO}"* ]]; then
      echo "[autoracer-env] Refusing old pilot underlay in ${var_name}" >&2
      return 1
    fi
  done
fi
