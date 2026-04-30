#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

if [[ -f "/workspace/ros2_ws/install/setup.bash" ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

exec ros2 run foxglove_bridge foxglove_bridge
