# MOSAIC Architecture

## Pipeline Layers

1. Dataset replay publishes image and LiDAR topics.
2. Camera and LiDAR perception nodes publish normalized `Detection3DArray`.
3. Fusion tracker performs association and EKF updates, then publishes `TrackArray`.
4. ADAS node consumes tracks and emits warnings.

## Topic Interfaces

- `/mosaic/camera/image_raw` (`sensor_msgs/Image`)
- `/mosaic/lidar/points` (`sensor_msgs/PointCloud2`)
- `/mosaic/detections/camera` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/detections/lidar` (`mosaic_msgs/Detection3DArray`)
- `/mosaic/tracks` (`mosaic_msgs/TrackArray`)
- `/mosaic/adas/warnings` (`mosaic_msgs/AdasWarning`)
