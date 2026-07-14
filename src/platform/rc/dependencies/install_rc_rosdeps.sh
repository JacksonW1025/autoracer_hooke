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
  echo "RC rosdep install failed: $*" >&2
  exit 2
}

if (($# != 2)) || [[ "$1" != "--workspace" ]]; then
  fail "usage: $0 --workspace /absolute/path/to/rc-vendor-workspace"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PRODUCT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)"
RESOLVER="${SCRIPT_DIR}/resolve_rc_vendor.py"
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

SOURCE_ROOT="${WORKSPACE}/src"
[[ -d "${SOURCE_ROOT}" && ! -L "${SOURCE_ROOT}" ]] || \
  fail "fresh RC vendor workspace has no source directory"
[[ -f "${WORKSPACE}/rc-vendor-filtered.repos" ]] || \
  fail "fresh RC vendor workspace has no filtered repository record"
shopt -s nullglob dotglob
workspace_entries=("${WORKSPACE}"/*)
shopt -u nullglob dotglob
((${#workspace_entries[@]} == 2)) || fail "RC vendor workspace is not a fresh import"

if ! records_output="$(python3 "${RESOLVER}" --format records)"; then
  fail "RC vendor resolution did not succeed"
fi
[[ -n "${records_output}" ]] || fail "resolved RC vendor selection is empty"
mapfile -t records <<< "${records_output}"
((${#records[@]} == 82)) || fail "resolved RC vendor selection must contain exactly 82 packages"

vendor_paths=()
for record in "${records[@]}"; do
  [[ "${record}" == *$'\t'* && "${record#*$'\t'}" != *$'\t'* ]] || \
    fail "resolver returned a malformed RC package record"
  package_name="${record%%$'\t'*}"
  package_path="${record#*$'\t'}"
  [[ "${package_name}" =~ ^[a-z][a-z0-9_]*$ ]] || \
    fail "resolver returned an invalid RC package name"
  [[ -n "${package_path}" && "${package_path}" != /* && "${package_path}" != *..* ]] || \
    fail "resolver returned an unsafe RC package path"
  vendor_path="${SOURCE_ROOT}/${package_path}"
  [[ -d "${vendor_path}" && ! -L "${vendor_path}" && \
    -f "${vendor_path}/package.xml" && ! -L "${vendor_path}/package.xml" ]] || \
    fail "resolved RC package is missing from the fresh workspace: ${package_name}"
  vendor_paths+=("${vendor_path}")
done

command -v rosdep >/dev/null 2>&1 || fail "required RC rosdep tool is unavailable"
rosdep install \
  --from-paths \
  "${vendor_paths[@]}" \
  "${PRODUCT_ROOT}/src/core" \
  "${PRODUCT_ROOT}/src/platform/rc" \
  --ignore-src \
  -y \
  -r
