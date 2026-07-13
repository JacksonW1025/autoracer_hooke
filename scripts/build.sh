#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${PRODUCT_ROOT}/scripts/import_dependencies.sh" --verify-only
"${PRODUCT_ROOT}/scripts/build_vendor.sh"
"${PRODUCT_ROOT}/scripts/build_product.sh"
