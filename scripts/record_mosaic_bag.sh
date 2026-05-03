#!/usr/bin/env bash
# Record MOSAIC topics to a ros2 bag (run inside the dev container with ROS sourced).
set -euo pipefail

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Record MOSAIC topics to a ros2 bag (run inside the dev container with ROS sourced).

Usage:
  source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
  scripts/record_mosaic_bag.sh [output_dir]

If output_dir is omitted, writes under <repo>/bags/mosaic_YYYYMMDD_HHMMSS.

Recorded topics include replay frame index, camera image and calibration, LiDAR,
camera/LiDAR detections, fused tracks, lane JSON state, and ADAS warnings.
EOF
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "record_mosaic_bag: ros2 not on PATH; source /opt/ros/humble/setup.bash and your workspace install/setup.bash first." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
default_name="mosaic_$(date +%Y%m%d_%H%M%S)"
out_path="${1:-${repo_root}/bags/${default_name}}"

mkdir -p "$(dirname "${out_path}")"

echo "Recording MOSAIC topics to: ${out_path}" >&2

exec ros2 bag record -o "${out_path}" \
  /mosaic/replay/frame_index \
  /mosaic/camera/image_raw \
  /mosaic/camera/calibration \
  /mosaic/lidar/points \
  /mosaic/detections/camera \
  /mosaic/detections/lidar \
  /mosaic/tracks \
  /mosaic/lanes/state \
  /mosaic/adas/warnings
