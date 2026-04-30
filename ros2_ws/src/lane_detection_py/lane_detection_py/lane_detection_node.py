from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass
class LaneFit:
    left_x: float
    right_x: float
    lane_center_x: float
    lane_width_px: float


class LaneDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("lane_detection_node")

        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("hough_threshold", 40)
        self.declare_parameter("min_line_length", 40)
        self.declare_parameter("max_line_gap", 150)
        self.declare_parameter("roi_top_ratio", 0.55)
        self.declare_parameter("departure_threshold_px", 60.0)
        self.declare_parameter("min_lane_width_px", 250.0)

        self.create_subscription(Image, "/mosaic/camera/image_raw", self._image_cb, 10)
        self.state_pub = self.create_publisher(String, "/mosaic/lanes/state", 10)

        self._warned_bad_encoding = False
        self.get_logger().info("Lane detection node started (classical OpenCV pipeline).")

    def _decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding not in ("rgb8", "bgr8"):
            if not self._warned_bad_encoding:
                self.get_logger().warn(f"Unsupported image encoding '{msg.encoding}' (expected rgb8/bgr8)")
                self._warned_bad_encoding = True
            return None

        height = int(msg.height)
        width = int(msg.width)
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            rgb = data.reshape((height, width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return data.reshape((height, width, 3))

    def _fit_lanes(self, bgr: np.ndarray) -> Optional[LaneFit]:
        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(
            blur,
            int(self.get_parameter("canny_low").value),
            int(self.get_parameter("canny_high").value),
        )

        roi_top = int(float(self.get_parameter("roi_top_ratio").value) * h)
        mask = np.zeros_like(edges)
        polygon = np.array([[(0, h), (w, h), (w, int(h * 0.85)), (int(w * 0.55), roi_top), (int(w * 0.45), roi_top), (0, int(h * 0.85))]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            masked,
            rho=1,
            theta=np.pi / 180,
            threshold=int(self.get_parameter("hough_threshold").value),
            minLineLength=int(self.get_parameter("min_line_length").value),
            maxLineGap=int(self.get_parameter("max_line_gap").value),
        )
        if lines is None:
            return None

        left_xs = []
        right_xs = []
        mid_y = int(h * 0.9)
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / float(x2 - x1)
            if abs(slope) < 0.4:
                continue
            intercept = y1 - slope * x1
            x_at_mid = (mid_y - intercept) / slope
            if slope < 0:
                left_xs.append(x_at_mid)
            else:
                right_xs.append(x_at_mid)

        if not left_xs or not right_xs:
            return None

        left_x = float(np.median(left_xs))
        right_x = float(np.median(right_xs))
        if right_x <= left_x:
            return None

        lane_center = 0.5 * (left_x + right_x)
        lane_width = right_x - left_x
        return LaneFit(left_x=left_x, right_x=right_x, lane_center_x=lane_center, lane_width_px=lane_width)

    def _image_cb(self, msg: Image) -> None:
        bgr = self._decode_image(msg)
        if bgr is None:
            return

        fit = self._fit_lanes(bgr)
        if fit is None:
            payload = {
                "ok": False,
                "frame_id": msg.header.frame_id,
                "image_width": int(msg.width),
                "image_height": int(msg.height),
            }
            out = String()
            out.data = json.dumps(payload)
            self.state_pub.publish(out)
            return

        image_center_x = 0.5 * float(msg.width)
        offset = float(fit.lane_center_x - image_center_x)

        min_width = float(self.get_parameter("min_lane_width_px").value)
        width_ok = fit.lane_width_px >= min_width

        thresh = float(self.get_parameter("departure_threshold_px").value)
        departure = width_ok and abs(offset) > thresh

        payload = {
            "ok": True,
            "frame_id": msg.header.frame_id,
            "image_width": int(msg.width),
            "image_height": int(msg.height),
            "left_x_at_bottom": fit.left_x,
            "right_x_at_bottom": fit.right_x,
            "lane_center_x_at_bottom": fit.lane_center_x,
            "lane_width_px": fit.lane_width_px,
            "offset_from_image_center_px": offset,
            "lane_departure": bool(departure),
        }

        out = String()
        out.data = json.dumps(payload)
        self.state_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
