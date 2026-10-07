#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "${ROS_DISTRO:-}" != humble ]]; then
  echo 'This project targets ROS 2 Humble.' >&2
  exit 1
fi
colcon build --symlink-install --event-handlers console_direct+
python3 -m unittest discover -s tests -v
echo 'Build complete. Run: source install/setup.bash'
