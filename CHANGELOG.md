# Changelog

All notable changes to MOSAIC are described here. This file is meant for humans skim-reading release notes, not machine parsing.

## Unreleased

- README Quick Start: notes `.dockerignore` (smaller Docker build context) and points to `docker/requirements.txt` for intentional dependency bumps; repository layout lists those paths under `docker/`.
- Docker: `docker/requirements.txt` pins `numpy`, `scipy`, and `ultralytics` so `mosaic-dev` image builds stay reproducible across weeks (bump versions deliberately when you upgrade YOLO).
- Repo-root `.dockerignore` so `docker compose build` (context `..` from `docker/`) does not send KITTI data, bags, colcon outputs, or `.git` to the daemon—faster rebuilds on typical laptops.
- `mosaic_pipeline.launch.py`: every launch argument now has a short `description` so `ros2 launch ... --show-args` is self-documenting.
- Production-ish baseline: shared defaults in `mosaic_bringup/config/mosaic_defaults.yaml`, `docs/production.md` spelling out what certification still needs from you, and `scripts/health_check.sh` for cheap topic liveness checks while the stack is running.
- Removed the old `config/tracker.yaml` stub—it wasn’t wired into launch and drifted from the real fusion parameters (everything tunable lives in `mosaic_defaults.yaml` and launch args now).

## Earlier highlights (summary)

- End-to-end ROS 2 Humble pipeline in Docker: KITTI replay, YOLO camera perception, PCL LiDAR clustering, fusion with Hungarian assignment, ADAS FCW/LDW, lane JSON, Foxglove bridge (optional from launch).
- Starter KITTI eval: replay frame index topic, track dump node, offline label comparison script with optional calib transform, CI smoke for that script.
- Ops helpers: bag recorder including frame index, live topic check script, Fast DDS UDP profile for Docker Desktop.

When you tag a release, drop the date under **Unreleased** and rename the section to the version (e.g. `## 0.2.0 — 2026-05-01`).
