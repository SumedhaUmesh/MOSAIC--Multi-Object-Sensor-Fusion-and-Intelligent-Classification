#!/usr/bin/env bash
# Probe core MOSAIC topics while the pipeline is running (stdlib-friendly for orchestrators).
# Exit 0 if every topic delivers at least one message within the per-topic timeout.
# Usage (inside container after sourcing ROS + workspace):
#   source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
#   ./scripts/health_check.sh [timeout_seconds_per_topic]
set -eo pipefail

timeout_sec="${1:-8}"
topics=(
  /mosaic/camera/image_raw
  /mosaic/lidar/points
  /mosaic/tracks
)

failures=0
for t in "${topics[@]}"; do
  if ! timeout "${timeout_sec}" ros2 topic echo "$t" --once >/dev/null 2>&1; then
    echo "health_check: FAILED no message on ${t} within ${timeout_sec}s" >&2
    failures=$((failures + 1))
  else
    echo "health_check: OK ${t}"
  fi
done

exit "${failures}"
