"""Subscribe to replay frame index + fused tracks and write JSON for evaluate_kitti_tracks.py."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from mosaic_msgs.msg import TrackArray
from std_msgs.msg import UInt32


class DumpTracksEvalNode(Node):
    def __init__(self) -> None:
        super().__init__("dump_tracks_eval_node")
        self.declare_parameter("output_path", "/workspace/tracks_dump.json")

        self._last_frame_idx: int = -1
        self._by_frame: Dict[int, List[Dict[str, Any]]] = {}

        self.create_subscription(UInt32, "/mosaic/replay/frame_index", self._on_frame_idx, 10)
        self.create_subscription(TrackArray, "/mosaic/tracks", self._on_tracks, 10)

        self._output_path = Path(self.get_parameter("output_path").value)

    def _on_frame_idx(self, msg: UInt32) -> None:
        self._last_frame_idx = int(msg.data)

    def _on_tracks(self, msg: TrackArray) -> None:
        if self._last_frame_idx < 0:
            return
        tracks = [
            {
                "track_id": int(t.track_id),
                "x": float(t.position.x),
                "y": float(t.position.y),
                "z": float(t.position.z),
                "status": t.status,
            }
            for t in msg.tracks
        ]
        self._by_frame[self._last_frame_idx] = tracks

    def _write_output(self) -> None:
        frames = [{"frame_index": k, "tracks": v} for k, v in sorted(self._by_frame.items())]
        payload = {"frames": frames}
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_path.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(f"Wrote {len(frames)} frames to {self._output_path}")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DumpTracksEvalNode()
    try:
        rclpy.spin(node)
    finally:
        node._write_output()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
