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

if [[ "${profile}" == "rc" ]]; then
  "${PRODUCT_ROOT}/scripts/import_dependencies.sh" --verify-only-rc
  "${PRODUCT_ROOT}/scripts/build_vendor.sh" --rc
  "${PRODUCT_ROOT}/scripts/build_product.sh" --rc
else
  "${PRODUCT_ROOT}/scripts/import_dependencies.sh" --verify-only
  "${PRODUCT_ROOT}/scripts/build_vendor.sh"
  "${PRODUCT_ROOT}/scripts/build_product.sh"
fi
