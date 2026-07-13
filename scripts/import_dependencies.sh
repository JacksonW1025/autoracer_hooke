#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(dirname "${PRODUCT_ROOT}")"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${PRODUCT_ROOT}/vendor_ws}"
PILOT_REPO="${PILOT_REPO:-${ROOT_DIR}/pilot-auto.x1}"
PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages.tsv"
REPOSITORY_MANIFEST="${PRODUCT_ROOT}/dependencies/autoracer.repos"
PATCH_DIR="${PRODUCT_ROOT}/dependencies/patches"

mode="reuse"
case "${1:-}" in
  "") ;;
  --refresh) mode="pilot" ;;
  --network) mode="network" ;;
  --verify-only) mode="verify" ;;
  *)
    echo "Usage: $0 [--refresh|--network|--verify-only]" >&2
    exit 2
    ;;
esac

if [[ ! -s "${PACKAGE_MANIFEST}" ]]; then
  echo "Missing package manifest: ${PACKAGE_MANIFEST}" >&2
  exit 1
fi

copy_curated_packages() {
  local source_root="$1"
  local package relative_path source_dir destination_dir

  while IFS=$'\t' read -r package relative_path; do
    [[ -n "${package}" && -n "${relative_path}" ]] || continue
    source_dir="${source_root}/${relative_path}"
    destination_dir="${VENDOR_WS}/src/${relative_path}"
    if [[ ! -f "${source_dir}/package.xml" ]]; then
      echo "Missing ${package} in dependency source: ${source_dir}" >&2
      exit 1
    fi
    mkdir -p "$(dirname "${destination_dir}")"
    rsync -a --delete \
      --exclude='.git' \
      --exclude='build' \
      --exclude='install' \
      --exclude='log' \
      --exclude='COLCON_IGNORE' \
      --exclude='*.db3' \
      "${source_dir}/" "${destination_dir}/"
  done < "${PACKAGE_MANIFEST}"
}

apply_patch_once() {
  local patch_file="$1"
  if patch --batch --forward --dry-run --silent -d "${VENDOR_WS}" -p1 \
    < "${patch_file}" >/dev/null 2>&1
  then
    patch --batch --forward --silent -d "${VENDOR_WS}" -p1 < "${patch_file}"
  elif patch --batch --reverse --dry-run --silent -d "${VENDOR_WS}" -p1 \
    < "${patch_file}" >/dev/null 2>&1
  then
    printf 'already applied: %s\n' "$(basename "${patch_file}")"
  else
    echo "Patch does not apply cleanly: ${patch_file}" >&2
    exit 1
  fi
}

verify_package_set() {
  local expected actual
  expected="$(mktemp)"
  actual="$(mktemp)"
  cut -f1 "${PACKAGE_MANIFEST}" | sort -u > "${expected}"
  colcon list --base-paths "${VENDOR_WS}/src" --names-only | sort -u > "${actual}"
  if ! diff -u "${expected}" "${actual}"; then
    rm -f "${expected}" "${actual}"
    echo "Vendor package set differs from ${PACKAGE_MANIFEST}" >&2
    return 1
  fi
  printf 'verified vendor package set: %s packages\n' "$(wc -l < "${actual}")"
  rm -f "${expected}" "${actual}"
}

if [[ "${mode}" == "pilot" || "${mode}" == "network" ]]; then
  if [[ -z "${VENDOR_WS}" || "${VENDOR_WS}" == "/" || "${VENDOR_WS}" == "${HOME}" ]]; then
    echo "Refusing unsafe vendor workspace path: ${VENDOR_WS}" >&2
    exit 1
  fi
  rm -rf "${VENDOR_WS}/src"
  mkdir -p "${VENDOR_WS}/src"

  if [[ "${mode}" == "pilot" ]]; then
    if [[ ! -d "${PILOT_REPO}/src" ]]; then
      echo "Missing real-vehicle source repository: ${PILOT_REPO}" >&2
      exit 1
    fi
    copy_curated_packages "${PILOT_REPO}/src"
  else
    command -v vcs >/dev/null || {
      echo "vcs is required for --network" >&2
      exit 1
    }
    temporary_checkout="$(mktemp -d)"
    trap 'rm -rf "${temporary_checkout}"' EXIT
    mkdir -p "${temporary_checkout}/src"
    vcs import "${temporary_checkout}/src" < "${REPOSITORY_MANIFEST}"
    copy_curated_packages "${temporary_checkout}/src"
  fi
elif [[ ! -d "${VENDOR_WS}/src" ]]; then
  echo "Missing vendor workspace. Run $0 --refresh." >&2
  exit 1
fi

if [[ "${mode}" != "verify" ]]; then
  while IFS= read -r patch_file; do
    apply_patch_once "${patch_file}"
  done < <(find "${PATCH_DIR}" -maxdepth 1 -type f -name '*.patch' -print | sort)
fi

verify_package_set
