#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(dirname "${PRODUCT_ROOT}")"
VENDOR_WS="${AUTORACER_VENDOR_WS:-${PRODUCT_ROOT}/vendor_ws}"
PILOT_REPO="${PILOT_REPO:-${ROOT_DIR}/pilot-auto.x1}"
PACKAGE_MANIFEST="${PRODUCT_ROOT}/dependencies/vendor-packages.tsv"
REPOSITORY_MANIFEST="${PRODUCT_ROOT}/dependencies/autoracer.repos"
VERSION_LOCK="${PRODUCT_ROOT}/dependencies/versions.lock.yaml"
PATCH_STACK_STAMP="${VENDOR_WS}/.autoracer_dependency_patch_stack.sha256"

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

patch_stack_already_applied() {
  local snapshot relative_path patch_file index
  local -a patch_files=("$@")
  local -a touched_paths

  snapshot="$(mktemp -d)"
  mapfile -t touched_paths < <(
    sed -nE \
      's#^(\+\+\+ b/|--- a/)([^[:space:]]+).*$#\2#p' \
      "${patch_files[@]}" | sort -u
  )
  for relative_path in "${touched_paths[@]}"; do
    if [[ ! -e "${VENDOR_WS}/${relative_path}" ]]; then
      rm -rf "${snapshot}"
      return 1
    fi
    mkdir -p "${snapshot}/$(dirname "${relative_path}")"
    cp -a "${VENDOR_WS}/${relative_path}" "${snapshot}/${relative_path}"
  done

  for ((index = ${#patch_files[@]} - 1; index >= 0; --index)); do
    patch_file="${patch_files[index]}"
    if ! patch --batch --reverse --silent -d "${snapshot}" -p1 \
      < "${patch_file}" >/dev/null 2>&1
    then
      rm -rf "${snapshot}"
      return 1
    fi
  done
  rm -rf "${snapshot}"
  return 0
}

patch_stack_matches_clean_reconstruction() {
  local snapshot relative_path patch_file source_path
  local -a patch_files=("$@")
  local -a touched_paths

  if [[ ! -d "${PILOT_REPO}/src" ]]; then
    return 1
  fi
  snapshot="$(mktemp -d)"
  mapfile -t touched_paths < <(
    sed -nE \
      's#^(\+\+\+ b/|--- a/)([^[:space:]]+).*$#\2#p' \
      "${patch_files[@]}" | sort -u
  )
  for relative_path in "${touched_paths[@]}"; do
    source_path="${PILOT_REPO}/${relative_path}"
    if [[ -e "${source_path}" ]]; then
      mkdir -p "${snapshot}/$(dirname "${relative_path}")"
      cp -a "${source_path}" "${snapshot}/${relative_path}"
    fi
  done
  for patch_file in "${patch_files[@]}"; do
    if ! patch --batch --forward --silent -d "${snapshot}" -p1 \
      < "${patch_file}" >/dev/null 2>&1
    then
      rm -rf "${snapshot}"
      return 1
    fi
  done
  for relative_path in "${touched_paths[@]}"; do
    if ! cmp --silent \
      "${snapshot}/${relative_path}" "${VENDOR_WS}/${relative_path}"
    then
      rm -rf "${snapshot}"
      return 1
    fi
  done
  rm -rf "${snapshot}"
  return 0
}

patch_stack_fingerprint() {
  local patch_file
  for patch_file in "$@"; do
    sha256sum "${patch_file}" | cut -d' ' -f1
  done | sha256sum | cut -d' ' -f1
}

recorded_patch_stack_prefix_length() {
  local recorded_hash="$1"
  shift
  local -a patch_files=("$@")
  local count prefix_hash

  for ((count = ${#patch_files[@]} - 1; count >= 1; --count)); do
    prefix_hash="$(patch_stack_fingerprint "${patch_files[@]:0:count}")"
    if [[ "${recorded_hash}" == "${prefix_hash}" ]]; then
      printf '%s\n' "${count}"
      return 0
    fi
  done
  return 1
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
  rm -f "${PATCH_STACK_STAMP}"
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
    vcs import "${temporary_checkout}/src" < "${REPOSITORY_MANIFEST}"
    copy_curated_packages "${temporary_checkout}/src"
  fi
elif [[ ! -d "${VENDOR_WS}/src" ]]; then
  echo "Missing vendor workspace. Run $0 --refresh." >&2
  exit 1
fi

if [[ "${mode}" != "verify" ]]; then
  mapfile -t declared_patches < <(
    sed -nE \
      's#^[[:space:]]*-[[:space:]]+(dependencies/patches/[^[:space:]]+\.patch)[[:space:]]*$#\1#p' \
      "${VERSION_LOCK}"
  )
  if [[ "${#declared_patches[@]}" -eq 0 ]]; then
    echo "No dependency patches declared in ${VERSION_LOCK}" >&2
    exit 1
  fi
  declared_patch_files=()
  for relative_patch in "${declared_patches[@]}"; do
    patch_file="${PRODUCT_ROOT}/${relative_patch}"
    if [[ ! -f "${patch_file}" ]]; then
      echo "Missing declared dependency patch: ${patch_file}" >&2
      exit 1
    fi
    declared_patch_files+=("${patch_file}")
  done
  patch_stack_hash="$(patch_stack_fingerprint "${declared_patch_files[@]}")"
  recorded_patch_stack_hash=""
  if [[ -f "${PATCH_STACK_STAMP}" ]]; then
    recorded_patch_stack_hash="$(<"${PATCH_STACK_STAMP}")"
  fi
  if [[ "${recorded_patch_stack_hash}" == "${patch_stack_hash}" ]]; then
    printf 'verified dependency patch-stack stamp: %s\n' "${patch_stack_hash}"
  elif prefix_length="$(
    recorded_patch_stack_prefix_length \
      "${recorded_patch_stack_hash}" "${declared_patch_files[@]}"
  )"; then
    remaining_patch_files=("${declared_patch_files[@]:prefix_length}")
    if patch_stack_already_applied "${remaining_patch_files[@]}"; then
      for patch_file in "${remaining_patch_files[@]}"; do
        printf 'already applied in appended stack: %s\n' "$(basename "${patch_file}")"
      done
    else
      for patch_file in "${remaining_patch_files[@]}"; do
        apply_patch_once "${patch_file}"
      done
    fi
    printf '%s\n' "${patch_stack_hash}" > "${PATCH_STACK_STAMP}"
    printf \
      'extended dependency patch-stack from %s to %s patches: %s\n' \
      "${prefix_length}" "${#declared_patch_files[@]}" "${patch_stack_hash}"
  elif patch_stack_already_applied "${declared_patch_files[@]}"; then
    for patch_file in "${declared_patch_files[@]}"; do
      printf 'already applied in declared stack: %s\n' "$(basename "${patch_file}")"
    done
    printf '%s\n' "${patch_stack_hash}" > "${PATCH_STACK_STAMP}"
  elif patch_stack_matches_clean_reconstruction "${declared_patch_files[@]}"; then
    for patch_file in "${declared_patch_files[@]}"; do
      printf 'verified against clean reconstruction: %s\n' "$(basename "${patch_file}")"
    done
    printf '%s\n' "${patch_stack_hash}" > "${PATCH_STACK_STAMP}"
  else
    for patch_file in "${declared_patch_files[@]}"; do
      apply_patch_once "${patch_file}"
    done
    printf '%s\n' "${patch_stack_hash}" > "${PATCH_STACK_STAMP}"
  fi
fi

verify_package_set
