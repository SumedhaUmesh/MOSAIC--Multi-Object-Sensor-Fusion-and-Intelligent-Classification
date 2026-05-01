# MOSAIC: Multi-Object Sensor Fusion and Intelligent Classification

End-to-end ROS2 ADAS scaffold that combines camera perception, LiDAR perception,
multi-object tracking, and ADAS warning logic.

## Stack

- ROS2 Humble (Dockerized Ubuntu 22.04 workflow)
- Python: dataset replay + camera perception
- C++17: LiDAR perception + fusion tracker + ADAS app
- Eigen, OpenCV/PCL-ready integration points

## Repository Layout

- `docker/`: Dockerfile, Compose, Fast DDS profile, pinned **`docker/requirements.txt`** (YOLO/scientific stack)
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
- `scripts/check_live_topics.sh`: quick check that camera/LiDAR topics publish (run in container)
- `scripts/evaluate_kitti_tracks.py`: starter KITTI-label vs fused-tracks metrics (stdlib-only; see Metrics + `docs/roadmap.md`)
- `docs/roadmap.md`: what is “done” for demo vs research vs shipping
- `docs/production.md`: production-oriented baseline vs external certification work
- `CHANGELOG.md`: what changed, written for humans
- `scripts/health_check.sh`: topic liveness probe for orchestration (run while pipeline is up)

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

To start **Foxglove’s bridge in the same launch** (WebSocket on port **8765**), add `launch_foxglove_bridge:=true` (optional):

```bash
ros2 launch mosaic_bringup mosaic_pipeline.launch.py dataset_root:=/workspace/data/kitti sequence:=0 launch_foxglove_bridge:=true
```

Always use `--merge-install` for this workspace so Python and C++ packages share one `install/` layout.

Compose builds from the **repo root**; a root **`.dockerignore`** skips KITTI data, bags, colcon outputs, and `.git` so the daemon upload stays small. Edit **`docker/requirements.txt`** only when you mean to change pinned NumPy/SciPy/Ultralytics versions.

### Production orientation

Defaults for replay rate, perception, fusion, ADAS, and Foxglove live in **`ros2_ws/src/mosaic_bringup/config/mosaic_defaults.yaml`** (loaded by `mosaic_pipeline.launch.py`; launch arguments still override).

See **`docs/production.md`** for what that covers versus real safety qualification (ISO-style process, HW sign-off, cybersecurity, etc.). To probe a running stack:

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
chmod +x /workspace/scripts/health_check.sh
/workspace/scripts/health_check.sh
```

### ADAS node parameters (`adas_app_cpp`)

Set on the `adas_app_node` (for example in a custom launch or via `ros2 param set`):

| Parameter | Default | Role |
|-----------|---------|------|
| `ttc_threshold` | `2.5` | FCW: publish when time-to-collision (seconds) is below this while closing speed is significant. |
| `publish_fcw_on_rising_edge_only` | `true` | If true, emit one FCW per track when it **enters** the TTC zone; clears when the track leaves the zone. Set `false` for a warning every tracker tick while in zone. |
| `publish_ldw_on_rising_edge_only` | `true` | Same idea for lane departure: warn on transition into departure, not on every message while latched. |

`mosaic_pipeline.launch.py` forwards these as launch arguments (same defaults): `adas_ttc_threshold`, `adas_publish_fcw_on_rising_edge_only`, `adas_publish_ldw_on_rising_edge_only`. Example:

`ros2 launch mosaic_bringup mosaic_pipeline.launch.py adas_ttc_threshold:=3.0 adas_publish_fcw_on_rising_edge_only:=false`

Fusion tuning uses the same launch file: arguments **`fusion_prediction_dt`**, **`fusion_max_assignment_cost`**, **`fusion_mahalanobis_gate`**, **`fusion_iou_weight`**, **`fusion_confirm_hits`**, **`fusion_tentative_max_misses`**, **`fusion_confirmed_max_misses`** (defaults match `fusion_tracker_node`). List everything with:

`ros2 launch mosaic_bringup mosaic_pipeline.launch.py --show-args`

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

**Option A — one launch (recommended):** start the pipeline **with** the bridge:

```bash
ros2 launch mosaic_bringup mosaic_pipeline.launch.py dataset_root:=/workspace/data/kitti sequence:=0 launch_foxglove_bridge:=true
```

Optional: `foxglove_port:=8765` if you change the port (must match `docker/compose.yaml`).

**Option B — bridge separately:** launch the pipeline **without** `launch_foxglove_bridge`, then from the Mac host:

```bash
cd docker
docker compose exec -d mosaic-dev bash -lc "chmod +x /workspace/scripts/run_foxglove_bridge.sh && /workspace/scripts/run_foxglove_bridge.sh"
```

Wait until topics are flowing (optional: second shell + `/workspace/scripts/check_live_topics.sh`).

**Then** open **Foxglove Studio** → **Open data source** / **Connect** → **Foxglove WebSocket** → URL:

`ws://127.0.0.1:8765`

After it connects, you should see ROS topics in the sidebar (e.g. `/mosaic/...`). If the topic list is empty, confirm the pipeline is running and reconnect (or restart the bridge if you use Option B).

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
- Run **at most one** bridge on **8765**: do not use `launch_foxglove_bridge:=true` **and** the `exec -d` script at the same time.

### Foxglove connected but you see nothing

Work through these in order:

1. **Confirm ROS is actually publishing** (second shell in the container while `mosaic_pipeline` runs):

```bash
chmod +x /workspace/scripts/check_live_topics.sh
/workspace/scripts/check_live_topics.sh
```

You should see `OK: received a message` for camera and LiDAR. If both fail, fix the pipeline or `dataset_root` first—not Foxglove.

2. **Confirm the bridge is running** (on the Mac): start it only after the pipeline is up, then in Foxglove **disconnect and reconnect** `ws://127.0.0.1:8765` so the topic list refreshes.

3. **Wait ~30s after launch** the first time: the camera node downloads/loads YOLO before images flow regularly.

4. **Panel topic names**: open each panel’s **settings (gear)** and set exactly **`/mosaic/camera/image_raw`** and **`/mosaic/lidar/points`** (an old saved layout can point at wrong topics).

5. **3D view**: set **fixed frame** to **`base_link`**, then use **reset view** / fit (cloud can start outside the default camera frustum).

6. **Sidebar**: click `/mosaic/camera/image_raw`—if **no messages** appear there while the check script passes, the WebSocket connection is wrong or stale; reconnect.

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

### Starter KITTI tracking benchmark (v1)

This is a **diagnostic** path only: GT labels are in **camera** coordinates while MOSAIC tracks use the replay fusion frame—see `docs/roadmap.md` before quoting numbers in a report.

1. With the stack sourced, terminal A:

```bash
ros2 launch mosaic_bringup mosaic_pipeline.launch.py dataset_root:=/workspace/data/kitti sequence:=0
```

2. Terminal B (records fused tracks keyed by `/mosaic/replay/frame_index`):

```bash
ros2 run dataset_replay_py dump_tracks_eval_node --ros-args -p output_path:=/workspace/outputs/tracks_dump.json
```

Let it run through part or all of the sequence, then **Ctrl+C** (the node writes JSON on exit).

3. Offline metrics:

```bash
python3 scripts/evaluate_kitti_tracks.py \
  --kitti-root /workspace/data/kitti \
  --sequence 0 \
  --predictions /workspace/outputs/tracks_dump.json \
  --calib /workspace/data/kitti/training/calib/0000.txt \
  --output /workspace/outputs/kitti_track_metrics.json
```

## Notes

This implementation is an extensible scaffold with working message contracts,
node wiring, EKF primitives, and baseline ADAS warning flow. Replace stubs with
full KITTI/CARLA loaders and model-backed detections for production-grade results.
