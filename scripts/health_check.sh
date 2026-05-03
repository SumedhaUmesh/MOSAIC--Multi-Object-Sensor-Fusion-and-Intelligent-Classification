#!/usr/bin/env bash
# Probe core MOSAIC topics while the pipeline is running (stdlib-friendly for orchestrators).
# Exit 0 if every topic delivers at least one message within the per-topic timeout.
# Exit code equals the number of topics that failed (max 3 with default topic list).
set -eo pipefail

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Probe core MOSAIC topics while the pipeline is running.

Usage (inside container after sourcing ROS + workspace):
  source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
  scripts/health_check.sh [timeout_seconds_per_topic]

Default timeout per topic: 8 seconds.
Exit 0 only if every listed topic receives at least one message within its timeout.
Otherwise exit with the number of failed topics.

Topics: /mosaic/camera/image_raw, /mosaic/lidar/points, /mosaic/tracks
EOF
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "health_check: ros2 not on PATH; source /opt/ros/humble/setup.bash and your workspace install/setup.bash first." >&2
  exit 1
fi

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
