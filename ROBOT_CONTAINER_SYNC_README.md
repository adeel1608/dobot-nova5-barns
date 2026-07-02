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
| 2 | `0254b6f` | Pushed | Merged `dobot_bringup_v3` communication hardening while preserving the target DI fix. |
| 3 | `a1c1873` | Pushed | Added the `oms_v1` production GUI package, docs, and ROS console entry point. |
| 4 | `92b7cd1` | Pushed | Added source `shared` RabbitMQ/logging helpers used by `oms_v1.app` and GUI fallback mode. |
| 5 | `c159046` | Pushed | Synced `oms_v1` motion core and parameters from the robot container. |
| 6 | `ea4edb7` | Pushed | Synced `oms_v1` espresso sequence implementation from the robot container. |
| 7 | `1060416` | Pushed | Synced `oms_v1` milk frothing and cleaning sequence implementations. |
| 8 | `951d458` | Pushed | Synced `oms_v1` paper cup, plastic cup, and slush sequence implementations. |
| 9 | Current stage | Ready | Sync remaining `oms_v1` home, test, and computer vision sequence support modules. |

## Verification Log

- `python3 -m py_compile src/dobot_bringup_v3/dobot_bringup_v3/dobot_api.py src/dobot_bringup_v3/dobot_bringup_v3/dobot_bringup.py`
  passed before the baseline push.
- `python3 -m py_compile src/dobot_bringup_v3/dobot_bringup_v3/dobot_api.py src/dobot_bringup_v3/dobot_bringup_v3/dobot_bringup.py src/dobot_bringup_v3/dobot_bringup_v3/feedback.py`
  passed after merging the bringup hardening stage.
- `python3 -m py_compile src/oms_v1/setup.py src/oms_v1/oms_v1/gui/*.py`
  passed after adding the `oms_v1` GUI stage.
- `python3 -m py_compile src/shared/__init__.py src/shared/logger.py src/shared/rabbitmq_client.py`
  passed after adding the shared helper stage.
- `python3 -m py_compile src/oms_v1/oms_v1/params.py src/oms_v1/oms_v1/manipulate_node.py`
  passed after syncing the `oms_v1` motion core stage.
- `python3 -m py_compile src/oms_v1/oms_v1/sequences/espresso.py`
  passed after syncing the espresso sequence stage.
- `python3 -m py_compile src/oms_v1/oms_v1/sequences/milk_frothing.py src/oms_v1/oms_v1/sequences/cleaning.py`
  passed after syncing the milk frothing and cleaning sequence stage.
- `python3 -m py_compile src/oms_v1/oms_v1/sequences/paper_cups.py src/oms_v1/oms_v1/sequences/plastic_cups.py src/oms_v1/oms_v1/sequences/slush.py`
  passed after syncing the cup and slush sequence stage.
- `python3 -m py_compile src/oms_v1/oms_v1/sequences/home.py src/oms_v1/oms_v1/sequences/test.py src/oms_v1/oms_v1/sequences/computer_vision.py`
  passed after syncing the remaining `oms_v1` sequence support stage.
