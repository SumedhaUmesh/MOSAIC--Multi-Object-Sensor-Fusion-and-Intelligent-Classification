# MOSAIC Architecture

## Pipeline Layers

1. Dataset replay publishes image and LiDAR topics.
2. Camera and LiDAR perception nodes publish normalized `Detection3DArray`.
3. Fusion tracker performs Mahalanobis gating, minimum-cost (Hungarian) assignment with explicit unmatched track/detection slots, EKF updates, then publishes `TrackArray`.
4. ADAS node consumes tracks and lane state JSON, then emits warnings.

See `docs/roadmap.md` for demo vs research vs product scope.

## Timing and asynchronous sensors

KITTI replay drives **camera and LiDAR from the same timer** (`publish_rate_hz` in `mosaic_defaults.yaml`): each tick publishes one image and one point cloud for the same frame index, so in this stack there is **no real-world clock skew** between sensors—only whatever jitter your OS / DDS adds between those two publishes.

The fusion node does **not** maintain a separate hardware-sync buffer with interpolation. It processes messages **as they arrive** on each subscription: camera detections and LiDAR detections update tracks when their callbacks run. If one modality were delayed (e.g. LiDAR processing finished 10 ms after the camera branch), the **EKF prediction step** (`prediction_dt`) still advances on the fusion node’s processing cadence, and the **late update** applies when that detection message is handled—association uses the **current** gated Mahalanobis / cost against predicted track state. For a thesis or design review, call out explicitly that **true asynchronous multi-rate fusion** (explicit timestamps, out-of-sequence handling, per-sensor delays) is **not** implemented here; this pipeline assumes replay-time alignment suitable for KITTI demo and research ablations.

## Topic Interfaces

- `/mosaic/replay/frame_index` (`std_msgs/UInt32`, 0-based frame index aligned with KITTI files for this sequence)
- `/mosaic/camera/image_raw` (`sensor_msgs/Image`)
- `/mosaic/camera/calibration` (`std_msgs/String`, KITTI calib text for projection / eval)
- `/mosaic/lanes/state` (`std_msgs/String`, JSON lane metrics + departure flag)
- `/mosaic/lidar/points` (`sensor_msgs/PointCloud2`)
- `/mosaic/detections/camera` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/detections/lidar` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/tracks` (`mosaic_msgs/TrackArray`)
- `/mosaic/adas/warnings` (`mosaic_msgs/AdasWarning`)
