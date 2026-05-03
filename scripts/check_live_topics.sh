#!/usr/bin/env bash
# Run INSIDE mosaic-dev; confirms the pipeline is publishing before debugging Foxglove.
# Do not use `set -u`: ROS setup.bash reads optional vars (e.g. AMENT_TRACE_SETUP_FILES).

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Quick check that camera and LiDAR topics publish (pipeline must be running).

Usage:
  ./scripts/check_live_topics.sh

Expects the usual mosaic-dev layout: sources /opt/ros/humble and /workspace/ros2_ws/install
if present. Waits up to 15s per topic for one message, then lists /mosaic/* topics.
EOF
  exit 0
fi

set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "check_live_topics: ros2 not on PATH after sourcing setup.bash (unexpected)." >&2
  exit 1
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
