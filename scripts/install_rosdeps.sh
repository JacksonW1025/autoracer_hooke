#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y -r

