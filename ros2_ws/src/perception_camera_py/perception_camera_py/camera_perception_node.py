from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from mosaic_msgs.msg import Detection3D, Detection3DArray

from perception_camera_py.kitti_calib import KittiCalibration, bbox_to_velo_center_from_height, parse_kitti_calib


class CameraPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_perception_node")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("device", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_detections", 25)
        self.declare_parameter("calibration_topic", "/mosaic/camera/calibration")

        self._confidence = float(self.get_parameter("confidence_threshold").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._device = str(self.get_parameter("device").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._max_det = int(self.get_parameter("max_detections").value)
        calib_topic = str(self.get_parameter("calibration_topic").value)

        self._calib: Optional[KittiCalibration] = None
        self._warned_missing_calib = False
        self._warned_bad_encoding = False

        self._model = YOLO(self._model_name)
        self._vehicle_classes = {2, 3, 5, 7}  # car, motorcycle, bus, truck (COCO)

        self._default_sizes: Dict[str, Tuple[float, float, float]] = {
            "car": (4.5, 1.8, 1.6),
            "truck": (7.0, 2.4, 2.6),
            "bus": (11.0, 2.5, 3.2),
            "motorcycle": (2.1, 0.8, 1.4),
        }

        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, calib_topic, self._calib_cb, qos_transient)
        self.create_subscription(Image, "/mosaic/camera/image_raw", self._image_cb, 10)
        self.publisher = self.create_publisher(Detection3DArray, "/mosaic/detections/camera", 10)
        self.get_logger().info(
            f"Camera perception started: model={self._model_name}, conf>={self._confidence}"
        )

    def _calib_cb(self, msg: String) -> None:
        try:
            self._calib = parse_kitti_calib(msg.data)
            self._warned_missing_calib = False
            self.get_logger().info("Received KITTI calibration; 3D projection enabled.")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to parse calibration: {exc}")

    def _coco_class_name(self, cls_id: int) -> str:
        names = getattr(self._model, "names", {}) or {}
        return str(names.get(int(cls_id), f"class_{int(cls_id)}"))

    def _vehicle_label(self, cls_id: int) -> Optional[str]:
        if int(cls_id) not in self._vehicle_classes:
            return None
        name = self._coco_class_name(cls_id).lower()
        if "truck" in name:
            return "truck"
        if "bus" in name:
            return "bus"
        if "motor" in name:
            return "motorcycle"
        return "car"

    def _image_cb(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            if not self._warned_bad_encoding:
                self.get_logger().warn(f"Unsupported image encoding '{msg.encoding}' (expected rgb8/bgr8)")
                self._warned_bad_encoding = True
            return

        if self._calib is None:
            if not self._warned_missing_calib:
                self.get_logger().warn("Waiting for KITTI calibration on /mosaic/camera/calibration ...")
                self._warned_missing_calib = True
            return

        height = int(msg.height)
        width = int(msg.width)
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            frame = data.reshape((height, width, 3))
            frame_bgr = frame[:, :, ::-1].copy()
        else:
            frame_bgr = data.reshape((height, width, 3))

        device = None if not self._device else self._device
        results = self._model.predict(
            source=frame_bgr,
            imgsz=self._imgsz,
            conf=self._confidence,
            device=device,
            verbose=False,
        )
        if not results:
            return

        det_arr = Detection3DArray()
        det_arr.header = msg.header

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self.publisher.publish(det_arr)
            return

        xyxy = boxes.xyxy.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)
        conf = boxes.conf.cpu().numpy()

        detections: List[Detection3D] = []
        det_id = 0
        for i in range(min(len(xyxy), self._max_det)):
            label = self._vehicle_label(int(cls[i]))
            if label is None:
                continue

            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            bbox_w = max(1.0, x2 - x1)
            bbox_h = max(1.0, y2 - y1)
            u = 0.5 * (x1 + x2)
            v_bottom = y2

            size = self._default_sizes[label]
            try:
                center, _ = bbox_to_velo_center_from_height(
                    self._calib, u=u, v_bottom=v_bottom, bbox_h_px=bbox_h, object_height_m=size[2]
                )
            except Exception:  # noqa: BLE001
                continue

            det = Detection3D()
            det.header = msg.header
            det.id = det_id
            det.source = "camera"
            det.class_label = label
            det.center.x = float(center[0])
            det.center.y = float(center[1])
            det.center.z = float(center[2])
            det.size.x = float(size[0])
            det.size.y = float(size[1])
            det.size.z = float(size[2])
            det.confidence = float(conf[i])
            detections.append(det)
            det_id += 1

        det_arr.detections = detections
        self.publisher.publish(det_arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
