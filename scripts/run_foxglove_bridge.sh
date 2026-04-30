#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

if [[ -f "/workspace/ros2_ws/install/setup.bash" ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

# Port must match docker/compose.yaml (8765:8765). Address 0.0.0.0 keeps Docker port publish working on macOS.
exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0
