#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
mkdir -p results/bags
ros2 bag record -o "results/bags/a2-$(date +%Y%m%d-%H%M%S)-$$" \
  /joint_states /tf /tf_static /robot_description \
  /a2/arm/state /a2/machine/state /a2/workpiece /a2/task/status /a2/scene
