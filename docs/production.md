# Production orientation

“Production-ready” for robotics stacks usually spans **product safety** (requirements, validation, homologation), **operations** (monitoring, rollback, SLOs), and **software engineering** (config control, supply-chain, testing). MOSAIC is an **integration scaffold**: this document defines what that means here and what this repo adds versus what your organization must still provide.

## What this repository adds (engineering baseline)

| Area | Artifact |
|------|-----------|
| **Versioned defaults** | `mosaic_bringup/config/mosaic_defaults.yaml` loaded by `mosaic_pipeline.launch.py`; launch arguments still override per run. |
| **Health probing** | `scripts/health_check.sh` — waits briefly for core `/mosaic/*` topics (run while the pipeline is up). |
| **Offline metrics smoke** | `scripts/test_evaluate_kitti_tracks_smoke.py` in CI for the KITTI eval script. |
| **Operational docs** | Docker + Fast DDS profile for Desktop, Foxglove, bags, eval scripts. |

Pin this YAML file in reviews when you change tuning; treat it like an internal “release manifest” for perception/fusion/ADAS defaults.

## What “production” still requires externally

These are **not** implemented as a certified product inside this repo:

1. **Safety case** — ODD definition, hazard analysis, fault reactions, independent verification (often aligned with ISO 26262 / UL 4600–style process for ADAS-adjacent systems).
2. **Qualification** — HW platform sign-off, sensor calibration traceability, EMC/temperature, and verification and validation on target ECU.
3. **Cybersecurity** — SBOM, signed images, update mechanism, threat modeling (ISO 21434 etc., as applicable).
4. **Lifecycle** — staged rollout, feature flags, telemetry with privacy constraints, incident response.
5. **Performance guarantees** — WCET, memory ceilings, deterministic replay on the **deployment** CPU/GPU, not only the dev laptop.

## Recommended next steps for a real program

1. **Freeze dependencies** — Pin Docker base image digest, apt versions where feasible, and Ultralytics model weights checksum.
2. **Split images** — Multi-stage build: compile in CI, ship a minimal runtime image with only `install/` + runtime deps (no compiler chain).
3. **Inject config at deploy time** — Mount `mosaic_defaults.yaml` from a config repo or Kubernetes ConfigMap; avoid baking secrets into images.
4. **Expand automated tests** — ROS launch smoke under `pytest` + `launch_testing`, bag regression on golden sequences.
5. **Observability** — Add standard diagnostics (`diagnostic_msgs`) or a thin `/mosaic/metrics` publisher with latency and drop counts.

If you need a **single milestone** before claiming “production pilot”, aim for: pinned image + launch_testing smoke + health_check in your orchestrator + incident runbook—not more features.
