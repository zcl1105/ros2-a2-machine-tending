#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
xvfb-run -a -s '-screen 0 1440x900x24' python3 scripts/capture_video.py
