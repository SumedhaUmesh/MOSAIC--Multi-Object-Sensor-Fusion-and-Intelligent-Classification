from pathlib import Path
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String, UInt32


class KittiReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("kitti_replay_node")
        self.declare_parameter("dataset_root", "/workspace/data/kitti")
        self.declare_parameter("sequence", 0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("loop", True)

        self.dataset_root = Path(self.get_parameter("dataset_root").value)
        self.sequence = f"{int(self.get_parameter('sequence').value):04d}"
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.loop = bool(self.get_parameter("loop").value)

        self.image_pub = self.create_publisher(Image, "/mosaic/camera/image_raw", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/mosaic/lidar/points", 10)
        self.frame_index_pub = self.create_publisher(UInt32, "/mosaic/replay/frame_index", 10)
        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.calib_pub = self.create_publisher(String, "/mosaic/camera/calibration", qos_transient)

        self.image_files: List[Path] = []
        self.lidar_files: List[Path] = []
        self.timestamps: List[str] = []
        self.frame_id = 0
        self._load_sequence()
        if not self.image_files or not self.lidar_files:
            raise RuntimeError("No KITTI files found. Check dataset_root and sequence parameters.")

        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 0.1), self._publish_frame)
        self.get_logger().info(
            f"KITTI replay ready: seq={self.sequence}, frames={min(len(self.image_files), len(self.lidar_files))}"
        )

    def _load_sequence(self) -> None:
        img_dir = self.dataset_root / "training" / "image_02" / self.sequence
        lidar_dir = self.dataset_root / "training" / "velodyne" / self.sequence
        ts_file = self.dataset_root / "training" / "oxts" / self.sequence / "timestamps.txt"
        calib_file = self.dataset_root / "training" / "calib" / f"{self.sequence}.txt"

        self.image_files = sorted(img_dir.glob("*.png"))
        self.lidar_files = sorted(lidar_dir.glob("*.bin"))
        frame_count = min(len(self.image_files), len(self.lidar_files))
        self.image_files = self.image_files[:frame_count]
        self.lidar_files = self.lidar_files[:frame_count]

        if ts_file.exists():
            self.timestamps = [line.strip() for line in ts_file.read_text().splitlines() if line.strip()]
            self.timestamps = self.timestamps[:frame_count]

        if calib_file.exists():
            msg = String()
            msg.data = calib_file.read_text()
            self.calib_pub.publish(msg)
        else:
            self.get_logger().warn(f"Calibration file not found: {calib_file}")

    def _publish_frame(self) -> None:
        if self.frame_id >= len(self.image_files):
            if not self.loop:
                self.get_logger().info("Finished KITTI replay. Stopping timer.")
                self.timer.cancel()
                return
            self.frame_id = 0

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id="base_link")
        image_file = self.image_files[self.frame_id]
        lidar_file = self.lidar_files[self.frame_id]

        image_bgr = cv2.imread(str(image_file), cv2.IMREAD_COLOR)
        if image_bgr is None:
            self.get_logger().warn(f"Failed to read image: {image_file}")
            self.frame_id += 1
            return
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_msg = Image()
        image_msg.height = int(image_rgb.shape[0])
        image_msg.width = int(image_rgb.shape[1])
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = False
        image_msg.step = image_msg.width * 3
        image_msg.data = image_rgb.tobytes()
        image_msg.header = header

        points_flat = np.fromfile(lidar_file, dtype=np.float32)
        if points_flat.size % 4 != 0:
            self.get_logger().warn(f"Invalid LiDAR frame shape: {lidar_file}")
            self.frame_id += 1
            return
        points = points_flat.reshape(-1, 4)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, points.tolist())

        idx_msg = UInt32()
        idx_msg.data = int(self.frame_id)
        self.frame_index_pub.publish(idx_msg)
        self.image_pub.publish(image_msg)
        self.lidar_pub.publish(cloud)
        self.frame_id += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KittiReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
