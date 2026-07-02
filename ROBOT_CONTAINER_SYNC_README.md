# Robot Container Sync

This document tracks the staged sync from:

- Source: `/home/adeel/BARNS/services/robot_container`
- Target: `/home/adeel/barns_ws`
- Target branch: `main`
- Baseline pushed first: `1f302ad Fix DI service status handling`

## Goals

- Preserve the target repository's current ROS workspace layout.
- Bring over useful robot-container updates in small, reviewable commits.
- Avoid blindly overwriting local fixes or vendored upstream packages.
- Verify each stage before pushing it.

## Initial Findings

- The target repository is a ROS workspace rooted at `/home/adeel/barns_ws`.
- The source container keeps its ROS workspace under `ros_ws/`.
- The source contains container runtime files at its root: Dockerfiles, Compose config,
  requirements, and container entrypoint scripts.
- The source contains packages that are not present in the target:
  `OrbbecSDK_ROS2`, `dobot_bringup_v4`, and `shared`.
- The target contains packages/assets that are not present in the source:
  `barns_flowcharts` and `motion_debug_v3`.
- The target had untracked DI backup files before this sync. They are preserved locally
  and intentionally not part of the baseline commit.

## Stage Log

| Stage | Commit | Status | Notes |
| --- | --- | --- | --- |
| 0 | `1f302ad` | Pushed | Baseline DI service fix pushed before sync work. |
| 1 | `2266f0c` | Pushed | Added this sync README and comparison trail. |
| 2 | Current stage | Ready | Merge `dobot_bringup_v3` communication hardening while preserving the target DI fix. |

## Verification Log

- `python3 -m py_compile src/dobot_bringup_v3/dobot_bringup_v3/dobot_api.py src/dobot_bringup_v3/dobot_bringup_v3/dobot_bringup.py`
  passed before the baseline push.
- `python3 -m py_compile src/dobot_bringup_v3/dobot_bringup_v3/dobot_api.py src/dobot_bringup_v3/dobot_bringup_v3/dobot_bringup.py src/dobot_bringup_v3/dobot_bringup_v3/feedback.py`
  passed after merging the bringup hardening stage.
