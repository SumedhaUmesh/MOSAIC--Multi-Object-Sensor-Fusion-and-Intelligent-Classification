#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Path to JSON with camera/lidar/fused arrays")
    args = parser.parse_args()

    payload = json.loads(Path(args.input).read_text())
    gt = np.array(payload["ground_truth"], dtype=float)
    cam = np.array(payload["camera_only"], dtype=float)
    lidar = np.array(payload["lidar_only"], dtype=float)
    fused = np.array(payload["fused"], dtype=float)

    results = {
        "camera_rmse": rmse(cam, gt),
        "lidar_rmse": rmse(lidar, gt),
        "fused_rmse": rmse(fused, gt),
    }
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
