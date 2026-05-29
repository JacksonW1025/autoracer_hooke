#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if ! command -v vcs >/dev/null 2>&1; then
  echo "vcs is required. Install python3-vcstool first." >&2
  exit 1
fi

mkdir -p src/external
vcs import src < autoracer.repos

