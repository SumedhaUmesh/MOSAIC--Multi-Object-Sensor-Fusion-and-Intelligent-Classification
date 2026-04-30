import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mosaic_msgs.msg import Detection3D, Detection3DArray


class CameraPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_perception_node")
        self.subscription = self.create_subscription(
            Image, "/mosaic/camera/image_raw", self._image_cb, 10
        )
        self.publisher = self.create_publisher(Detection3DArray, "/mosaic/detections/camera", 10)
        self.declare_parameter("confidence_threshold", 0.3)
        self.get_logger().info("Camera perception node started (YOLOv8 stub mode).")

    def _image_cb(self, msg: Image) -> None:
        det_arr = Detection3DArray()
        det_arr.header = msg.header

        det = Detection3D()
        det.header = msg.header
        det.id = 1
        det.source = "camera"
        det.class_label = "car"
        det.center.x = 10.0
        det.center.y = 0.0
        det.center.z = 0.0
        det.size.x = 4.0
        det.size.y = 1.8
        det.size.z = 1.6
        det.confidence = 0.9
        det_arr.detections = [det]

        self.publisher.publish(det_arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
