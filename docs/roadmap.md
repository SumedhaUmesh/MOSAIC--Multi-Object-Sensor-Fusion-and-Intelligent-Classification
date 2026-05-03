# MOSAIC roadmap: demo vs research vs product

For deployable defaults, health probes, and honest scope vs automotive certification, see **`production.md`**.

The repository is **complete as an integrated ROS 2 scaffold and demo**: Dockerized Humble stack, KITTI replay, camera and LiDAR perception, fusion with Hungarian assignment and EKF-style updates, ADAS warnings, lane JSON, Foxglove, bags, and CI.

It is **not complete in a shipping-product sense**: there is no full benchmark harness against KITTI labels (beyond the starter script in `scripts/evaluate_kitti_tracks.py`), no systematic tuning or ablation study, no simulator hook (for example CARLA), and lane departure remains **baseline heuristics**, not road-certified logic.

Use this page to prioritize what “done” means for your goal.

## Near term (research / portfolio hardening)

1. **KITTI tracking evaluation** — Run `dump_tracks_eval_node` while the pipeline plays a sequence, then `evaluate_kitti_tracks.py` against `training/label_02/<seq>/*.txt`. Use `--calib training/calib/<seq>.txt` when comparing in camera rectified coordinates (Velodyne→cam via `R0_rect @ Tr_velo_to_cam`). Fusion-heavy tracks may still bias away from pure LiDAR frame—then refine transforms or match in Velodyne. Next: IoU-based association, MOT metrics.
2. **Tuning / ablation** — Sweep fusion gates (`fusion_mahalanobis_gate`, `fusion_max_assignment_cost`), detector confidence, and replay rate; log configs next to metrics JSON.
3. **CI smoke** — Extend CI with `launch --show-args` (already present) plus optional headless topic checks if runtime permits.

## Medium term

4. **Simulation** — Bridge CARLA or another sim to publish `sensor_msgs/Image` / `PointCloud2` (or MOSAIC-specific topics) so regression tests do not depend on KITTI disk layout.
5. **Lanes / ADAS** — Replace classical lane JSON with a metric aligned to GT or sim lanes; separate FCW tuning from perception latency.

## Product / safety (long term)

6. **Safety case** — Formal requirements, ODD definition, fault handling, and independent validation are out of scope for this scaffold unless explicitly funded.

If you only need a **working demo and thesis screenshots**, you can stop after item 1–2 at a level you are willing to defend in writing.
