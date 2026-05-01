#!/usr/bin/env python3
"""
Offline comparison of MOSAIC fused tracks (JSON from dump_tracks_eval_node) vs KITTI tracking labels.

KITTI label rows (training label_02 per-frame files) are parsed as:
  track_id type truncated occluded alpha x1 y1 x2 y2 h w l x y z ry [score]

Coordinate frames: labels use camera rectified coordinates for (x,y,z). MOSAIC tracks are
published in the node's world frame (replay uses frame_id base_link). Treat metrics below as
**approximate** unless you transform both sides with calibration — see docs/roadmap.md.

Greedy nearest-neighbor matching in 3D with a distance gate (default 5 m) counts TP-style pairs
for diagnostic RMSE; this is not full MOTSA/MOTA.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


def parse_kitti_label_objects(label_path: Path) -> List[Tuple[int, str, np.ndarray]]:
    """Return list of (track_id, type, xyz camera coords). Skips DontCare and malformed lines."""
    if not label_path.is_file():
        return []
    rows: List[Tuple[int, str, np.ndarray]] = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 16:
            continue
        try:
            tid = int(float(parts[0]))
            obj_type = parts[1]
            if obj_type == "DontCare":
                continue
            x, y, z = float(parts[12]), float(parts[13]), float(parts[14])
        except ValueError:
            continue
        rows.append((tid, obj_type, np.array([x, y, z], dtype=float)))
    return rows


def load_predictions(path: Path) -> Dict[int, List[Tuple[int, np.ndarray]]]:
    """Map frame_index -> list of (track_id, xyz)."""
    data = json.loads(path.read_text())
    frames = data.get("frames", [])
    out: Dict[int, List[Tuple[int, np.ndarray]]] = {}
    for entry in frames:
        idx = int(entry["frame_index"])
        tracks = []
        for t in entry.get("tracks", []):
            tid = int(t["track_id"])
            xyz = np.array([float(t["x"]), float(t["y"]), float(t["z"])], dtype=float)
            tracks.append((tid, xyz))
        out[idx] = tracks
    return out


def greedy_match(
    gt_xyz: np.ndarray,
    pr_xyz: np.ndarray,
    gate: float,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Return matched (gt_i, pr_j) pairs and unmatched gt/pr indices."""
    if gt_xyz.size == 0 or pr_xyz.size == 0:
        return [], list(range(gt_xyz.shape[0])), list(range(pr_xyz.shape[0]))
    cost = np.linalg.norm(gt_xyz[:, None, :] - pr_xyz[None, :, :], axis=-1)
    pairs: List[Tuple[int, int]] = []
    used_g = set()
    used_p = set()
    flat = [(float(cost[i, j]), i, j) for i in range(cost.shape[0]) for j in range(cost.shape[1])]
    flat.sort(key=lambda x: x[0])
    for d, i, j in flat:
        if d > gate:
            break
        if i in used_g or j in used_p:
            continue
        used_g.add(i)
        used_p.add(j)
        pairs.append((i, j))
    unmatched_g = [i for i in range(gt_xyz.shape[0]) if i not in used_g]
    unmatched_p = [j for j in range(pr_xyz.shape[0]) if j not in used_p]
    return pairs, unmatched_g, unmatched_p


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare tracks JSON to KITTI tracking labels.")
    parser.add_argument("--kitti-root", type=Path, required=True, help="KITTI tracking root (contains training/).")
    parser.add_argument("--sequence", type=int, default=0, help="Sequence id (e.g. 0 -> 0000).")
    parser.add_argument("--predictions", type=Path, required=True, help="JSON written by dump_tracks_eval_node.")
    parser.add_argument("--gate-m", type=float, default=5.0, help="Max L3 distance for a greedy match pair.")
    parser.add_argument(
        "--types",
        nargs="*",
        default=["Car", "Pedestrian", "Cyclist"],
        help="KITTI types kept in GT (subset).",
    )
    parser.add_argument("--output", type=Path, default=None, help="Optional metrics JSON path.")
    args = parser.parse_args()

    seq = f"{args.sequence:04d}"
    label_dir = args.kitti_root / "training" / "label_02" / seq
    if not label_dir.is_dir():
        raise FileNotFoundError(f"Missing label dir: {label_dir}")

    allowed = set(args.types)
    preds_by_frame = load_predictions(args.predictions)

    all_errors: List[float] = []
    total_gt = 0
    total_pr = 0
    total_pairs = 0

    for frame_idx in sorted(preds_by_frame.keys()):
        lbl = label_dir / f"{frame_idx:06d}.txt"
        gt_objs = parse_kitti_label_objects(lbl)
        gt_objs = [(tid, t, xyz) for tid, t, xyz in gt_objs if t in allowed]
        gt_xyz = np.stack([x[2] for x in gt_objs], axis=0) if gt_objs else np.zeros((0, 3))
        pr_list = preds_by_frame[frame_idx]
        pr_xyz = np.stack([x[1] for x in pr_list], axis=0) if pr_list else np.zeros((0, 3))

        total_gt += gt_xyz.shape[0]
        total_pr += pr_xyz.shape[0]

        pairs, ug, up = greedy_match(gt_xyz, pr_xyz, args.gate_m)
        total_pairs += len(pairs)
        for gi, pj in pairs:
            err = float(np.linalg.norm(gt_xyz[gi] - pr_xyz[pj]))
            all_errors.append(err)

    report = {
        "sequence": seq,
        "note": "Greedy L3 matching vs KITTI label camera-frame GT; coordinate mismatch vs MOSAIC base_link is expected — interpret as diagnostic only.",
        "frames_with_predictions": len(preds_by_frame),
        "association_gate_m": args.gate_m,
        "total_gt_objects_over_frames": total_gt,
        "total_predicted_tracks_over_frames": total_pr,
        "matched_pairs": total_pairs,
        "mean_translation_error_m": float(np.mean(all_errors)) if all_errors else None,
        "rmse_translation_error_m": float(np.sqrt(np.mean(np.square(all_errors)))) if all_errors else None,
    }

    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)


if __name__ == "__main__":
    main()
