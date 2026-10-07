#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 scripts/integration.py
