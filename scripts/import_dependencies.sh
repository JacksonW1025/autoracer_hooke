#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(dirname "${PRODUCT_ROOT}")"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${PRODUCT_ROOT}/vendor_ws}"
PILOT_REPO="${PILOT_REPO:-${ROOT_DIR}/pilot-auto.x1}"
FULL_PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages.tsv"
FULL_REPOSITORY_MANIFEST="${PRODUCT_ROOT}/dependencies/autoracer.repos"
RC_PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages-rc.tsv"
RC_REPOSITORY_MANIFEST="${PRODUCT_ROOT}/dependencies/autoracer-rc.repos"
PATCH_DIR="${PRODUCT_ROOT}/dependencies/patches"

mode="reuse"
profile="full"
case "${1:-}" in
  "") ;;
  --refresh) mode="pilot" ;;
  --network) mode="network" ;;
  --verify-only) mode="verify" ;;
  --network-rc) mode="network"; profile="rc" ;;
  --verify-only-rc) mode="verify"; profile="rc" ;;
  *)
    echo "Usage: $0 [--refresh|--network|--verify-only|--network-rc|--verify-only-rc]" >&2
    exit 2
    ;;
esac

PACKAGE_MANIFEST="${FULL_PACKAGE_MANIFEST}"
REPOSITORY_MANIFEST="${FULL_REPOSITORY_MANIFEST}"
if [[ "${profile}" == "rc" ]]; then
  PACKAGE_MANIFEST="${RC_PACKAGE_MANIFEST}"
  REPOSITORY_MANIFEST="${RC_REPOSITORY_MANIFEST}"
fi

if [[ ! -s "${PACKAGE_MANIFEST}" ]]; then
  echo "Missing package manifest: ${PACKAGE_MANIFEST}" >&2
  exit 1
fi

copy_curated_packages() {
  local source_root="$1"
  local destination_workspace="$2"
  local package relative_path source_dir destination_dir

  while IFS=$'\t' read -r package relative_path; do
    [[ -n "${package}" && -n "${relative_path}" ]] || continue
    source_dir="${source_root}/${relative_path}"
    destination_dir="${destination_workspace}/src/${relative_path}"
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
  local target_workspace="$2"
  if patch --batch --forward --dry-run --silent -d "${target_workspace}" -p1 \
    < "${patch_file}" >/dev/null 2>&1
  then
    patch --batch --forward --silent -d "${target_workspace}" -p1 < "${patch_file}"
  elif patch --batch --reverse --dry-run --silent -d "${target_workspace}" -p1 \
    < "${patch_file}" >/dev/null 2>&1
  then
    printf 'already applied: %s\n' "$(basename "${patch_file}")"
  else
    echo "Patch does not apply cleanly: ${patch_file}" >&2
    exit 1
  fi
}

verify_package_set() {
  local target_workspace="$1"
  local expected actual
  expected="$(mktemp)"
  actual="$(mktemp)"
  cut -f1 "${PACKAGE_MANIFEST}" | sort -u > "${expected}"
  colcon list --base-paths "${target_workspace}/src" --names-only | sort -u > "${actual}"
  if ! diff -u "${expected}" "${actual}"; then
    rm -f "${expected}" "${actual}"
    echo "Vendor package set differs from ${PACKAGE_MANIFEST}" >&2
    return 1
  fi
  printf 'verified vendor package set: %s packages\n' "$(wc -l < "${actual}")"
  rm -f "${expected}" "${actual}"
}

staging_workspace=""
temporary_checkout=""
previous_source=""

cleanup() {
  if [[ -n "${previous_source}" && -d "${previous_source}/src" && ! -e "${VENDOR_WS}/src" ]]; then
    mv "${previous_source}/src" "${VENDOR_WS}/src"
  fi
  if [[ -n "${previous_source}" && -d "${previous_source}" ]]; then
    rm -rf -- "${previous_source}"
  fi
  if [[ -n "${staging_workspace}" && -d "${staging_workspace}" ]]; then
    rm -rf -- "${staging_workspace}"
  fi
  if [[ -n "${temporary_checkout}" && -d "${temporary_checkout}" ]]; then
    rm -rf -- "${temporary_checkout}"
  fi
}

trap cleanup EXIT

target_workspace="${VENDOR_WS}"
replace_source=false

if [[ "${mode}" == "pilot" || "${mode}" == "network" ]]; then
  if [[ -z "${VENDOR_WS}" || "${VENDOR_WS}" == "/" || "${VENDOR_WS}" == "${HOME}" ]]; then
    echo "Refusing unsafe vendor workspace path: ${VENDOR_WS}" >&2
    exit 1
  fi
  mkdir -p "${VENDOR_WS}"
  staging_workspace="$(mktemp -d "${VENDOR_WS}/.import.XXXXXX")"
  mkdir -p "${staging_workspace}/src"
  target_workspace="${staging_workspace}"
  replace_source=true

  if [[ "${mode}" == "pilot" ]]; then
    if [[ ! -d "${PILOT_REPO}/src" ]]; then
      echo "Missing real-vehicle source repository: ${PILOT_REPO}" >&2
      exit 1
    fi
    copy_curated_packages "${PILOT_REPO}/src" "${target_workspace}"
  else
    command -v vcs >/dev/null || {
      echo "vcs is required for --network" >&2
      exit 1
    }
    temporary_checkout="$(mktemp -d)"
    mkdir -p "${temporary_checkout}/src"
    if [[ "${profile}" == "rc" ]]; then
      vcs import --shallow "${temporary_checkout}/src" < "${REPOSITORY_MANIFEST}"
    else
      vcs import "${temporary_checkout}/src" < "${REPOSITORY_MANIFEST}"
    fi
    copy_curated_packages "${temporary_checkout}/src" "${target_workspace}"
  fi
elif [[ ! -d "${VENDOR_WS}/src" ]]; then
  echo "Missing vendor workspace. Run $0 --refresh." >&2
  exit 1
fi

if [[ "${mode}" != "verify" ]]; then
  while IFS= read -r patch_file; do
    apply_patch_once "${patch_file}" "${target_workspace}"
  done < <(find "${PATCH_DIR}" -maxdepth 1 -type f -name '*.patch' -print | sort)
fi

verify_package_set "${target_workspace}"

if [[ "${replace_source}" == true ]]; then
  previous_source="$(mktemp -d "${VENDOR_WS}/.previous.XXXXXX")"
  if [[ -d "${VENDOR_WS}/src" ]]; then
    mv "${VENDOR_WS}/src" "${previous_source}/src"
  fi
  mv "${staging_workspace}/src" "${VENDOR_WS}/src"
  rm -rf -- "${previous_source}"
  previous_source=""
  printf 'installed verified vendor source: %s\n' "${VENDOR_WS}/src"
fi
