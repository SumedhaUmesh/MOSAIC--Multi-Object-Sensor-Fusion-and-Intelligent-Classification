#!/usr/bin/env python3
"""Synthetic round-trip smoke test for evaluate_kitti_tracks.py (no ROS)."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> None:
    repo = Path(__file__).resolve().parents[1]
    script = repo / "scripts" / "evaluate_kitti_tracks.py"

    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        kitti = root / "kitti"
        seq_dir = kitti / "training" / "label_02" / "0000"
        seq_dir.mkdir(parents=True)
        # track_id type truncated occluded alpha x1 y1 x2 y2 h w l x y z ry
        seq_dir.joinpath("000000.txt").write_text(
            "1 Car 0 0 0 0 0 10 10 1.5 1.5 4 10 0 30 0\n",
            encoding="utf-8",
        )
        calib = root / "calib.txt"
        calib.write_text(
            "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            "Tr_velo_to_cam: 1 0 0 0 0 1 0 0 0 0 1 0\n",
            encoding="utf-8",
        )
        pred = root / "pred.json"
        pred.write_text(
            json.dumps(
                {
                    "frames": [
                        {
                            "frame_index": 0,
                            "tracks": [{"track_id": 99, "x": 10.0, "y": 0.0, "z": 30.0}],
                        }
                    ]
                }
            ),
            encoding="utf-8",
        )
        out = root / "metrics.json"
        subprocess.check_call(
            [
                sys.executable,
                str(script),
                "--kitti-root",
                str(kitti),
                "--sequence",
                "0",
                "--predictions",
                str(pred),
                "--calib",
                str(calib),
                "--gate-m",
                "50",
                "--output",
                str(out),
            ],
            cwd=str(repo),
        )
        report = json.loads(out.read_text(encoding="utf-8"))
        assert report.get("matched_pairs") == 1, report
        assert report.get("mean_translation_error_m") is not None
        assert report["mean_translation_error_m"] < 1e-6

    print("evaluate_kitti_tracks smoke OK")


if __name__ == "__main__":
    main()
