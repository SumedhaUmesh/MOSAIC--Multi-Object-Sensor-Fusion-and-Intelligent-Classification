#!/usr/bin/env python3
"""
Offline comparison of MOSAIC fused tracks (JSON from dump_tracks_eval_node) vs KITTI tracking labels.

KITTI label rows (training label_02 per-frame files) are parsed as:
  track_id type truncated occluded alpha x1 y1 x2 y2 h w l x y z ry [score]

Coordinate frames: KITTI labels use **camera rectified** (x,y,z). Replay LiDAR/pipeline tracks are
typically in **Velodyne** coordinates (replay header may say `base_link`, but coords match velodyne
`.bin`). Pass **--calib** `training/calib/<seq>.txt` to map predictions into the same
camera-rect frame before matching (standard `R0_rect @ Tr_velo_to_cam`).

Greedy nearest-neighbor matching in 3D with a distance gate (default 5 m) counts TP-style pairs
for diagnostic RMSE; this is not full MOTSA/MOTA.

Stdlib only (no NumPy) so this runs on minimal Python installs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat4 = List[List[float]]


def _eye4() -> Mat4:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _matmul44(a: Mat4, b: Mat4) -> Mat4:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def _matvec44(m: Mat4, v: Sequence[float]) -> List[float]:
    return [sum(m[i][k] * v[k] for k in range(4)) for i in range(4)]


def _parse_calib_matrices(calib_text: str) -> Tuple[Mat4, Mat4]:
    """Return (R_rect 4x4, Tr_velo_to_cam 4x4) from KITTI calib file text."""
    lines = [line.strip() for line in calib_text.splitlines() if line.strip()]
    fields: Dict[str, List[float]] = {}
    for line in lines:
        if ":" in line:
            key, rest = line.split(":", 1)
            key, payload = key.strip(), rest.strip()
        else:
            parts = line.split()
            if len(parts) < 2:
                continue
            key, payload = parts[0].strip(), " ".join(parts[1:])
        try:
            fields[key] = [float(x) for x in payload.split()]
        except ValueError:
            continue

    r_vals = None
    for k in ("R0_rect", "R_rect_00", "R_rect"):
        if k in fields and len(fields[k]) >= 9:
            r_vals = fields[k][:9]
            break
    if r_vals is None:
        raise KeyError("Calib missing R0_rect / R_rect")

    tr_vals = None
    for k in ("Tr_velo_to_cam", "Tr_velo_cam"):
        if k in fields and len(fields[k]) >= 12:
            tr_vals = fields[k][:12]
            break
    if tr_vals is None:
        raise KeyError("Calib missing Tr_velo_to_cam / Tr_velo_cam")

    r_rect = [r_vals[i * 3 + j] for i in range(3) for j in range(3)]
    tr = [tr_vals[i * 4 + j] for i in range(3) for j in range(4)]

    r4 = _eye4()
    for i in range(3):
        for j in range(3):
            r4[i][j] = r_rect[i * 3 + j]

    tr4 = _eye4()
    for i in range(3):
        for j in range(4):
            tr4[i][j] = tr[i * 4 + j]

    return r4, tr4


def velodyne_to_cam_rect(points: List[Vec3], calib_path: Path) -> List[Vec3]:
    """Transform points from Velodyne frame to camera-2 rectified coordinates."""
    if not points:
        return []
    r4, tr4 = _parse_calib_matrices(calib_path.read_text())
    t = _matmul44(r4, tr4)
    out: List[Vec3] = []
    for x, y, z in points:
        hom = _matvec44(t, [x, y, z, 1.0])
        out.append((hom[0], hom[1], hom[2]))
    return out


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def parse_kitti_label_objects(label_path: Path) -> List[Tuple[int, str, Vec3]]:
    """Return list of (track_id, type, xyz camera coords). Skips DontCare and malformed lines."""
    if not label_path.is_file():
        return []
    rows: List[Tuple[int, str, Vec3]] = []
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
        rows.append((tid, obj_type, (x, y, z)))
    return rows


def load_predictions(path: Path) -> Dict[int, List[Tuple[int, Vec3]]]:
    """Map frame_index -> list of (track_id, xyz)."""
    data = json.loads(path.read_text())
    frames = data.get("frames", [])
    out: Dict[int, List[Tuple[int, Vec3]]] = {}
    for entry in frames:
        idx = int(entry["frame_index"])
        tracks = []
        for t in entry.get("tracks", []):
            tid = int(t["track_id"])
            xyz = (float(t["x"]), float(t["y"]), float(t["z"]))
            tracks.append((tid, xyz))
        out[idx] = tracks
    return out


def greedy_match(
    gt_xyz: List[Vec3],
    pr_xyz: List[Vec3],
    gate: float,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Return matched (gt_i, pr_j) pairs and unmatched gt/pr indices."""
    ng, np_ = len(gt_xyz), len(pr_xyz)
    if ng == 0 or np_ == 0:
        return [], list(range(ng)), list(range(np_))

    flat: List[Tuple[float, int, int]] = []
    for i in range(ng):
        for j in range(np_):
            flat.append((_dist3(gt_xyz[i], pr_xyz[j]), i, j))
    flat.sort(key=lambda x: x[0])

    pairs: List[Tuple[int, int]] = []
    used_g = set()
    used_p = set()
    for d, i, j in flat:
        if d > gate:
            break
        if i in used_g or j in used_p:
            continue
        used_g.add(i)
        used_p.add(j)
        pairs.append((i, j))

    unmatched_g = [i for i in range(ng) if i not in used_g]
    unmatched_p = [j for j in range(np_) if j not in used_p]
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
    parser.add_argument(
        "--calib",
        type=Path,
        default=None,
        help="KITTI calib file (e.g. training/calib/0000.txt). If set, transforms predictions "
        "Velodyne→cam rect (R0_rect @ Tr_velo_to_cam) before matching label (x,y,z).",
    )
    args = parser.parse_args()

    seq = f"{args.sequence:04d}"
    label_dir = args.kitti_root / "training" / "label_02" / seq
    if not label_dir.is_dir():
        raise FileNotFoundError(f"Missing label dir: {label_dir}")

    calib_path = args.calib
    if calib_path is not None and not calib_path.is_file():
        raise FileNotFoundError(f"Calib not found: {calib_path}")

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
        gt_xyz = [x[2] for x in gt_objs]
        pr_list = preds_by_frame[frame_idx]
        pr_xyz = [x[1] for x in pr_list]
        if calib_path is not None and pr_xyz:
            pr_xyz = velodyne_to_cam_rect(pr_xyz, calib_path)

        total_gt += len(gt_xyz)
        total_pr += len(pr_xyz)

        pairs, _, _ = greedy_match(gt_xyz, pr_xyz, args.gate_m)
        total_pairs += len(pairs)
        for gi, pj in pairs:
            all_errors.append(_dist3(gt_xyz[gi], pr_xyz[pj]))

    note = (
        "Greedy L3 matching vs KITTI label (camera rect). Predictions transformed Velodyne→cam rect "
        f"via {calib_path} (best when track centers are near LiDAR/Velo frame)."
        if calib_path is not None
        else "Greedy L3 matching vs KITTI label (camera rect); predictions NOT transformed — "
        "use --calib training/calib/<seq>.txt when your dumped tracks are in Velodyne coordinates."
    )
    mean_e = sum(all_errors) / len(all_errors) if all_errors else None
    rmse_e = math.sqrt(sum(e * e for e in all_errors) / len(all_errors)) if all_errors else None

    report = {
        "sequence": seq,
        "calib_used": str(calib_path) if calib_path is not None else None,
        "prediction_coord_frame": "camera_rectified" if calib_path is not None else "raw_pipeline",
        "note": note,
        "frames_with_predictions": len(preds_by_frame),
        "association_gate_m": args.gate_m,
        "total_gt_objects_over_frames": total_gt,
        "total_predicted_tracks_over_frames": total_pr,
        "matched_pairs": total_pairs,
        "mean_translation_error_m": float(mean_e) if mean_e is not None else None,
        "rmse_translation_error_m": float(rmse_e) if rmse_e is not None else None,
    }

    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)


if __name__ == "__main__":
    main()
