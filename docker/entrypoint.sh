#!/usr/bin/env bash
set -e

if [ -f "/workspace/docker/fastdds_disable_shm.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/docker/fastdds_disable_shm.xml"
fi

source /opt/ros/humble/setup.bash

if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
  source /workspace/ros2_ws/install/setup.bash
fi

exec "$@"
