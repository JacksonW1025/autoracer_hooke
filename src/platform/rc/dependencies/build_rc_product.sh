#!/usr/bin/env bash
# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

fail() {
  echo "RC product build failed: $*" >&2
  exit 2
}

if (($# != 2)) || [[ "$1" != "--workspace" ]]; then
  fail "usage: $0 --workspace /absolute/path/to/rc-product-workspace"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PRODUCT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)"
WORKSPACE_INPUT="$2"

[[ "${WORKSPACE_INPUT}" == /* ]] || fail "RC workspace must be an explicit absolute path"
[[ -d "${WORKSPACE_INPUT}" && ! -L "${WORKSPACE_INPUT}" ]] || \
  fail "RC workspace must be an existing real directory"
WORKSPACE="$(cd "${WORKSPACE_INPUT}" && pwd -P)"
[[ "${WORKSPACE}" != "${PRODUCT_ROOT}" && "${WORKSPACE}" != "${PRODUCT_ROOT}/"* ]] || \
  fail "RC workspace must be outside the product source tree"
workspace_name="${WORKSPACE##*/}"
[[ "${workspace_name,,}" =~ (^|[-_.])rc([-_.]|$) ]] || \
  fail "RC workspace name must contain an explicit RC token"
workspace_lower="${WORKSPACE,,}"
[[ "${workspace_lower}" != *hooke* && "${workspace_lower}" != *autoware* && \
  "${workspace_name}" != "vendor_ws" ]] || fail "RC workspace path is reserved"

shopt -s nullglob dotglob
workspace_entries=("${WORKSPACE}"/*)
shopt -u nullglob dotglob
((${#workspace_entries[@]} == 0)) || fail "RC product build requires a fresh empty workspace"

command -v colcon >/dev/null 2>&1 || fail "required RC colcon tool is unavailable"
cd "${WORKSPACE}"
colcon build \
  --base-paths \
    "${PRODUCT_ROOT}/src/core" \
    "${PRODUCT_ROOT}/src/platform/rc" \
  --symlink-install \
  --parallel-workers "${COLCON_PARALLEL_WORKERS:-4}" \
  --packages-up-to autoracer_rc_bringup \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
