#!/usr/bin/env bash
# Run INSIDE mosaic-dev after sourcing ROS + workspace (or rely on paths below).
# Confirms the pipeline is publishing before debugging Foxglove.
# Do not use `set -u`: ROS setup.bash reads optional vars (e.g. AMENT_TRACE_SETUP_FILES).
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

echo "Waiting up to 15s for one message per topic (pipeline must be running)..."
for t in /mosaic/camera/image_raw /mosaic/lidar/points; do
  echo "--- $t ---"
  if timeout 15 ros2 topic echo "$t" --once >/dev/null 2>&1; then
    echo "OK: received a message"
  else
    echo "FAIL: no message (wrong dataset path, launch not running, or replay stuck)"
  fi
done

echo "--- ros2 topic list (mosaic) ---"
ros2 topic list 2>/dev/null | grep '^/mosaic/' || echo "(none — graph empty / DDS issue)"
