#!/usr/bin/env bash
# Record MOSAIC topics to a ros2 bag (run inside the dev container with ROS sourced).
# Usage:
#   source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
#   /workspace/scripts/record_mosaic_bag.sh [output_path]
# Default output: <repo>/bags/mosaic_YYYYMMDD_HHMMSS
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
default_name="mosaic_$(date +%Y%m%d_%H%M%S)"
out_path="${1:-${repo_root}/bags/${default_name}}"

mkdir -p "$(dirname "${out_path}")"

exec ros2 bag record -o "${out_path}" \
  /mosaic/camera/image_raw \
  /mosaic/camera/calibration \
  /mosaic/lidar/points \
  /mosaic/detections/camera \
  /mosaic/detections/lidar \
  /mosaic/tracks \
  /mosaic/lanes/state \
  /mosaic/adas/warnings
