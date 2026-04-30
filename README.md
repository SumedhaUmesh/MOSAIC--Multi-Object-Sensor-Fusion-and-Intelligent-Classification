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
- `ros2_ws/src/perception_lidar_cpp`: LiDAR perception node
- `ros2_ws/src/fusion_tracker_cpp`: association + EKF tracking core
- `ros2_ws/src/adas_app_cpp`: FCW and warning node
- `ros2_ws/src/mosaic_bringup`: launch files and RViz config
- `scripts/evaluate_fusion.py`: RMSE comparison utility

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
colcon build
source install/setup.bash
ros2 launch mosaic_bringup mosaic_pipeline.launch.py
```

## Metrics Workflow

Use `scripts/evaluate_fusion.py` with JSON arrays for `ground_truth`, `camera_only`,
`lidar_only`, and `fused` to compare RMSE.

## Notes

This implementation is an extensible scaffold with working message contracts,
node wiring, EKF primitives, and baseline ADAS warning flow. Replace stubs with
full KITTI/CARLA loaders and model-backed detections for production-grade results.
