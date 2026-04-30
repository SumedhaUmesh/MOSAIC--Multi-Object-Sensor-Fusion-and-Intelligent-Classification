from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class KittiCalibration:
    p_rect_02: np.ndarray  # 3x4
    r_rect_00: np.ndarray  # 3x3
    velo_to_cam: np.ndarray  # 3x4


def _get_field(fields: Dict[str, np.ndarray], *keys: str) -> np.ndarray:
    for key in keys:
        if key in fields:
            return fields[key]
    raise KeyError(", ".join(keys))


def parse_kitti_calib(text: str) -> KittiCalibration:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    fields: Dict[str, np.ndarray] = {}
    for line in lines:
        if ":" in line:
            key, rest = line.split(":", 1)
            key = key.strip()
            payload = rest.strip()
        else:
            parts = line.split()
            if len(parts) < 2:
                continue
            key = parts[0].strip()
            payload = " ".join(parts[1:])

        try:
            values = np.array([float(x) for x in payload.split()], dtype=np.float64)
        except ValueError:
            continue
        fields[key] = values

    p_rect_02 = _get_field(fields, "P2").reshape(3, 4)
    r_rect_00 = _get_field(fields, "R0_rect", "R_rect_00", "R_rect").reshape(3, 3)
    velo_to_cam = _get_field(fields, "Tr_velo_to_cam", "Tr_velo_cam").reshape(3, 4)
    return KittiCalibration(p_rect_02=p_rect_02, r_rect_00=r_rect_00, velo_to_cam=velo_to_cam)


def bbox_to_velo_center_from_height(
    calib: KittiCalibration,
    u: float,
    v_bottom: float,
    bbox_h_px: float,
    object_height_m: float,
) -> Tuple[np.ndarray, float]:
    if bbox_h_px <= 1.0:
        raise ValueError("bbox height too small")

    k = calib.p_rect_02[:, :3]
    fx = float(k[0, 0])
    fy = float(k[1, 1])
    cx = float(k[0, 2])
    cy = float(k[1, 2])

    z_c = (fy * object_height_m) / bbox_h_px
    x_c = (u - cx) * z_c / fx
    y_c = (v_bottom - cy) * z_c / fy

    x_rect = np.array([x_c, y_c, z_c, 1.0], dtype=np.float64)

    r_rect = calib.r_rect_00
    tr = calib.velo_to_cam
    tr4 = np.eye(4, dtype=np.float64)
    tr4[:3, :4] = tr

    r4 = np.eye(4, dtype=np.float64)
    r4[:3, :3] = r_rect

    # X_velo = inv(Tr) * inv(R_rect) * X_rect
    x_velo_hom = np.linalg.solve(tr4, np.linalg.solve(r4, x_rect))
    center_velo = x_velo_hom[:3]

    return center_velo, z_c
