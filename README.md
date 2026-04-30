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

`mosaic_pipeline.launch.py` forwards these as launch arguments (same defaults): `adas_ttc_threshold`, `adas_publish_fcw_on_rising_edge_only`, `adas_publish_ldw_on_rising_edge_only`. Example:

`ros2 launch mosaic_bringup mosaic_pipeline.launch.py adas_ttc_threshold:=3.0 adas_publish_fcw_on_rising_edge_only:=false`

### Docker Desktop / Fast DDS shared memory noise

If you see `RTPS_TRANSPORT_SHM Error` spam, the dev container sets `FASTRTPS_DEFAULT_PROFILES_FILE` to
`docker/fastdds_disable_shm.xml` to force UDP-only transport (more reliable in Docker Desktop).

If you still see SHM errors, rebuild/recreate the container so the updated `docker/compose.yaml` + `entrypoint.sh` take effect:

```bash
cd docker
docker compose up --build -d --force-recreate
```

## Visualization (Foxglove Studio)

Foxglove gives you a live desktop UI for the camera, LiDAR, and topics—no RViz inside Docker.

### One-time setup

1. Install **[Foxglove Studio](https://foxglove.dev/download)** on your Mac.
2. Ensure the dev image includes the bridge (already in `docker/Dockerfile`). Rebuild if you pulled an older tree:

```bash
cd docker
docker compose up --build -d
```

### Every session (order matters)

1. **Start the pipeline** in the container (Quick Start: `ros2 launch mosaic_bringup mosaic_pipeline.launch.py ...`). Wait until nodes are publishing (optional: `ros2 topic hz /mosaic/camera/image_raw`).
2. **Start the bridge** from the Mac host (uses port **8765**, forwarded by Compose):

```bash
cd docker
docker compose exec -d mosaic-dev bash -lc "chmod +x /workspace/scripts/run_foxglove_bridge.sh && /workspace/scripts/run_foxglove_bridge.sh"
```

3. Open **Foxglove Studio** → **Open data source** / **Connect** (wording depends on version) → choose **Foxglove WebSocket** → URL:

`ws://127.0.0.1:8765`

After it connects, you should see ROS topics in the sidebar (e.g. `/mosaic/...`). If the topic list is empty, confirm the pipeline is running and restart the bridge command above.

### Panels to see camera + LiDAR

| Goal | What to do |
|------|------------|
| **Camera** | **Add panel** → **Image** → set image topic to `/mosaic/camera/image_raw`. |
| **LiDAR** | **Add panel** → **3D** → enable `/mosaic/lidar/points` under point clouds (or pick it from the topic picker). Set **Fixed frame** / display frame to **`base_link`** (replay uses this frame for image + cloud). |
| **Tracks / ADAS / lanes** | **Add panel** → **Raw Messages** (or **Log**) → choose `/mosaic/tracks`, `/mosaic/adas/warnings`, or `/mosaic/lanes/state`. Custom `mosaic_msgs` types are easiest to read here unless you add a Plot extension. |

Other useful topics: `/mosaic/detections/camera`, `/mosaic/detections/lidar`.

### Layout import (optional)

You can save your own workspace once and reuse it: **Layouts** → export JSON; teammates **Import from file**. Foxglove’s layout format changes occasionally—building once locally is the most reliable.

### If Mac cannot connect

- Confirm Compose maps **8765** (`docker compose.yaml`).
- Confirm nothing else on the Mac is bound to 8765.
- Run the bridge **after** sourcing ROS inside the container (the script does this); keep **one** bridge instance (avoid launching twice).

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
