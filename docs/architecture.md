# MOSAIC Architecture

## Pipeline Layers

1. Dataset replay publishes image and LiDAR topics.
2. Camera and LiDAR perception nodes publish normalized `Detection3DArray`.
3. Fusion tracker performs Mahalanobis gating, minimum-cost (Hungarian) assignment with explicit unmatched track/detection slots, EKF updates, then publishes `TrackArray`.
4. ADAS node consumes tracks and lane state JSON, then emits warnings.

See `docs/roadmap.md` for demo vs research vs product scope.

## Topic Interfaces

- `/mosaic/replay/frame_index` (`std_msgs/UInt32`, 0-based frame index aligned with KITTI files for this sequence)
- `/mosaic/camera/image_raw` (`sensor_msgs/Image`)
- `/mosaic/lanes/state` (`std_msgs/String`, JSON lane metrics + departure flag)
- `/mosaic/lidar/points` (`sensor_msgs/PointCloud2`)
- `/mosaic/detections/camera` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/detections/lidar` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/tracks` (`mosaic_msgs/TrackArray`)
- `/mosaic/adas/warnings` (`mosaic_msgs/AdasWarning`)
