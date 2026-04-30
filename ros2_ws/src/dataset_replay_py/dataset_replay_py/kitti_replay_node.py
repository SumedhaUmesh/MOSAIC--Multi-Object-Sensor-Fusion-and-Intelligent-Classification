import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header


class KittiReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("kitti_replay_node")
        self.image_pub = self.create_publisher(Image, "/mosaic/camera/image_raw", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/mosaic/lidar/points", 10)
        self.timer = self.create_timer(0.1, self._publish_stub_frame)
        self.frame_id = 0
        self.get_logger().info("KITTI replay node started in stub mode.")

    def _publish_stub_frame(self) -> None:
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id="base_link")

        img = Image()
        img.header = header
        img.height = 375
        img.width = 1242
        img.encoding = "rgb8"
        img.step = img.width * 3
        img.data = bytes([0] * (img.height * img.step))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = 0
        cloud.point_step = 16
        cloud.row_step = 0
        cloud.is_dense = False
        cloud.data = b""

        self.image_pub.publish(img)
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
