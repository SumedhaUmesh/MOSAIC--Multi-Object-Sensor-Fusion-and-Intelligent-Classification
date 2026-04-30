#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import Dict

import numpy as np


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))


def rmse_per_axis(pred: np.ndarray, gt: np.ndarray) -> Dict[str, float]:
    out = {}
    axis_names = ["x", "y", "z"]
    for i in range(pred.shape[1]):
        name = axis_names[i] if i < len(axis_names) else f"axis_{i}"
        out[name] = float(np.sqrt(np.mean((pred[:, i] - gt[:, i]) ** 2)))
    return out


def pct_improvement(baseline: float, candidate: float) -> float:
    if baseline <= 1e-12:
        return 0.0
    return float(100.0 * (baseline - candidate) / baseline)


def load_array(payload: dict, key: str) -> np.ndarray:
    arr = np.array(payload[key], dtype=float)
    if arr.ndim != 2:
        raise ValueError(f"'{key}' must be a 2D array of shape [N, D]")
    return arr


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Path to JSON with ground_truth/camera_only/lidar_only/fused arrays")
    parser.add_argument("--output", default="", help="Optional output JSON path for metrics report")
    args = parser.parse_args()

    payload = json.loads(Path(args.input).read_text())
    required = ["ground_truth", "camera_only", "lidar_only", "fused"]
    missing = [k for k in required if k not in payload]
    if missing:
        raise KeyError(f"Missing keys in input JSON: {missing}")

    gt = load_array(payload, "ground_truth")
    cam = load_array(payload, "camera_only")
    lidar = load_array(payload, "lidar_only")
    fused = load_array(payload, "fused")

    if not (cam.shape == lidar.shape == fused.shape == gt.shape):
        raise ValueError(
            "Array shape mismatch: all arrays must share [N, D] shape. "
            f"got gt={gt.shape}, cam={cam.shape}, lidar={lidar.shape}, fused={fused.shape}"
        )

    camera_rmse = rmse(cam, gt)
    lidar_rmse = rmse(lidar, gt)
    fused_rmse = rmse(fused, gt)

    results = {
        "num_samples": int(gt.shape[0]),
        "num_dims": int(gt.shape[1]),
        "overall_rmse": {
            "camera": camera_rmse,
            "lidar": lidar_rmse,
            "fused": fused_rmse,
        },
        "axis_rmse": {
            "camera": rmse_per_axis(cam, gt),
            "lidar": rmse_per_axis(lidar, gt),
            "fused": rmse_per_axis(fused, gt),
        },
        "fused_improvement_percent": {
            "vs_camera": pct_improvement(camera_rmse, fused_rmse),
            "vs_lidar": pct_improvement(lidar_rmse, fused_rmse),
        },
    }

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(results, indent=2))

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
