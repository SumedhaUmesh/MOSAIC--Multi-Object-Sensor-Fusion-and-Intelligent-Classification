# MOSAIC: Multi-Object Sensor Fusion and Intelligent Classification

End-to-end ROS2 ADAS scaffold that combines camera perception, LiDAR perception,
multi-object tracking, and ADAS warning logic.

## Stack

- ROS2 Humble (Dockerized Ubuntu 22.04 workflow)
- Python: dataset replay + camera perception
- C++17: LiDAR perception + fusion tracker + ADAS app
- Eigen, OpenCV/PCL-ready integration points

## Repository Layout

- `docker/`: containerized dev/runtime setup
- `ros2_ws/src/mosaic_msgs`: custom ROS2 interfaces
- `ros2_ws/src/dataset_replay_py`: KITTI replay node
- `ros2_ws/src/perception_camera_py`: camera perception node
- `ros2_ws/src/lane_detection_py`: classical lane detection node
- `ros2_ws/src/perception_lidar_cpp`: LiDAR perception node
- `ros2_ws/src/fusion_tracker_cpp`: association + EKF tracking core
- `ros2_ws/src/adas_app_cpp`: FCW + lane-departure warnings from fused tracks and lane JSON
- `ros2_ws/src/mosaic_bringup`: launch files and RViz config
- `scripts/evaluate_fusion.py`: RMSE comparison utility
- `scripts/record_mosaic_bag.sh`: record core `/mosaic/*` topics with `ros2 bag`

## Quick Start (Docker)

```bash
cd docker
docker compose up --build -d
docker compose exec mosaic-dev bash
```

Inside container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build --merge-install
source install/setup.bash
ros2 launch mosaic_bringup mosaic_pipeline.launch.py dataset_root:=/workspace/data/kitti sequence:=0
```

Always use `--merge-install` for this workspace so Python and C++ packages share one `install/` layout.

### ADAS node parameters (`adas_app_cpp`)

Set on the `adas_app_node` (for example in a custom launch or via `ros2 param set`):

| Parameter | Default | Role |
|-----------|---------|------|
| `ttc_threshold` | `2.5` | FCW: publish when time-to-collision (seconds) is below this while closing speed is significant. |
| `publish_fcw_on_rising_edge_only` | `true` | If true, emit one FCW per track when it **enters** the TTC zone; clears when the track leaves the zone. Set `false` for a warning every tracker tick while in zone. |
| `publish_ldw_on_rising_edge_only` | `true` | Same idea for lane departure: warn on transition into departure, not on every message while latched. |

### Docker Desktop / Fast DDS shared memory noise

If you see `RTPS_TRANSPORT_SHM Error` spam, the dev container sets `FASTRTPS_DEFAULT_PROFILES_FILE` to
`docker/fastdds_disable_shm.xml` to force UDP-only transport (more reliable in Docker Desktop).

If you still see SHM errors, rebuild/recreate the container so the updated `docker/compose.yaml` + `entrypoint.sh` take effect:

```bash
cd docker
docker compose up --build -d --force-recreate
```

## Visualization (Foxglove Studio)

Foxglove gives you a real desktop UI for images, point clouds, and plots without relying on RViz inside Docker.

1. Install **Foxglove Studio** on macOS.
2. Rebuild the dev container so `ros-humble-foxglove-bridge` is present:

```bash
cd docker
docker compose up --build -d
```

3. Run your pipeline in the container (same as Quick Start).
4. Start the bridge (from your Mac host, this runs detached in the container):

```bash
cd docker
docker compose exec -d mosaic-dev bash -lc "chmod +x /workspace/scripts/run_foxglove_bridge.sh && /workspace/scripts/run_foxglove_bridge.sh"
```

5. In Foxglove Studio, create a **Foxglove WebSocket** connection to:

`ws://127.0.0.1:8765`

Suggested starter subscriptions:

- `/mosaic/camera/image_raw`
- `/mosaic/lidar/points`
- `/mosaic/detections/camera`
- `/mosaic/detections/lidar`
- `/mosaic/tracks`
- `/mosaic/lanes/state` (raw JSON string)
- `/mosaic/adas/warnings`

## Recording a trace (`ros2 bag`)

With the pipeline running in the container and both setups sourced, start a bag in a **second** shell (Ctrl+C stops recording):

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
chmod +x /workspace/scripts/record_mosaic_bag.sh
/workspace/scripts/record_mosaic_bag.sh
```

Optional path: `/workspace/scripts/record_mosaic_bag.sh /workspace/bags/my_run`. Bags default under `bags/` at the repo root; that directory is listed in `.gitignore`.

Including the camera image makes bags **large** quickly; edit the script’s topic list if you only need tracks, lanes, or ADAS messages.

## Metrics Workflow

Use `scripts/evaluate_fusion.py` with JSON arrays for `ground_truth`, `camera_only`,
`lidar_only`, and `fused` to compare RMSE.

Example:

```bash
python3 scripts/evaluate_fusion.py --input scripts/sample_eval_input.json --output outputs/fusion_metrics.json
```

The report includes:

- overall RMSE for camera, LiDAR, and fused
- per-axis RMSE (`x`, `y`, `z`)
- fused improvement percentage vs camera-only and lidar-only baselines

## Notes

This implementation is an extensible scaffold with working message contracts,
node wiring, EKF primitives, and baseline ADAS warning flow. Replace stubs with
full KITTI/CARLA loaders and model-backed detections for production-grade results.
