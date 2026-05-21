# `home.py` flowcharts

- Sequence/exported functions: 10
- Support/helper functions: 5

## Sequence/exported functions

### `home`

Move robot to a predefined home position.

- **Mermaid file:** [../mermaid/home/home.mmd](../mermaid/home/home.mmd)
- **Parameter scenarios observed:** `position`
- **Branch/decision scenarios:**
  - `not position`
  - `not angles`
  - `not ok(run_skill("gotoJ_deg", *angles))`

```mermaid
flowchart TD
  N0(["START home(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Parameter/normalization: position = params.get('position', 'north')"]
  N3{"IF not position?"}
  N4(["RETURN _fail('missing home position')"])
  N5{"IF not angles?"}
  N6(["RETURN _fail(f'unknown home position=(position)')"])
  N7{"IF not ok(run_skill('gotoJ_deg', *angles))?"}
  N8(["RETURN False"])
  N9(["RETURN True"])
  N10(["END home"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 -- "yes" --> N4
  N3 --> N5
  N5 -- "yes" --> N6
  N5 --> N7
  N7 -- "yes" --> N8
  N7 --> N9
  N0 --> N10
```

### `return_back_to_home`

Return the robot to a safe home position based on current angle.

- **Mermaid file:** [../mermaid/home/return_back_to_home.mmd](../mermaid/home/return_back_to_home.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not ok(release_result)`
  - `not ok(angles) or len(angles) < 6`
  - `-22.49 <= a1 <= 22.49`
  - `j1_val is None`
  - `not ok(run_skill("gotoJ_deg", j1_val, *home_j2_j6))`
  - `not ok(run_skill("toggle_drag_mode"))`
  - `22.51 <= a1 <= 67.49`
  - `67.51 <= a1 <= 112.49`
  - `112.51 <= a1 <= 157.49`
  - `157.51 <= a1 <= 202.49`
  - `202.51 <= a1 <= 247.49`
  - `247.51 <= a1 <= 292.49`
  - `292.51 <= a1 <= 337.49`
  - `337.51 <= a1 <= 360.0`
  - `-67.49 <= a1 <= -22.51`
  - `-112.49 <= a1 <= -67.51`
  - `-157.49 <= a1 <= -112.51`
  - `-202.49 <= a1 <= -157.51`
  - `-247.49 <= a1 <= -202.51`
  - `-292.49 <= a1 <= -247.51`
  - `-337.49 <= a1 <= -292.51`
  - `-360.0 <= a1 <= -337.51`

```mermaid
flowchart TD
  N0(["START return_back_to_home(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Robot call: release_result = run_skill('release_tension')"]
  N3{"IF not ok(release_result)?"}
  N4{"IF not ok(run_skill('toggle_drag_mode'))?"}
  N5(["RETURN False"])
  N6["Robot call: run_skill('set_speed_factor', 100)"]
  N7["Robot call: run_skill('sync')"]
  N8["Robot call: run_skill('set_gripper_position', 255,0,255)"]
  N9["State/cache: angles = run_skill('current_angles')"]
  N10{"IF not ok(angles) or len(angles) < 6?"}
  N11(["RETURN False"])
  N12{"IF -22.49 <= a1 <= 22.49?"}
  N13{"IF 22.51 <= a1 <= 67.49?"}
  N14{"IF 67.51 <= a1 <= 112.49?"}
  N15{"IF 112.51 <= a1 <= 157.49?"}
  N16{"IF 157.51 <= a1 <= 202.49?"}
  N17{"IF 202.51 <= a1 <= 247.49?"}
  N18{"IF 247.51 <= a1 <= 292.49?"}
  N19{"IF 292.51 <= a1 <= 337.49?"}
  N20{"IF 337.51 <= a1 <= 360.0?"}
  N21{"IF -67.49 <= a1 <= -22.51?"}
  N22{"IF -112.49 <= a1 <= -67.51?"}
  N23{"IF -157.49 <= a1 <= -112.51?"}
  N24{"IF -202.49 <= a1 <= -157.51?"}
  N25{"IF -247.49 <= a1 <= -202.51?"}
  N26{"IF -292.49 <= a1 <= -247.51?"}
  N27{"IF -337.49 <= a1 <= -292.51?"}
  N28{"IF -360.0 <= a1 <= -337.51?"}
  N29{"IF j1_val is None?"}
  N30(["RETURN _fail(f'current joint1 outside return-home compass ranges: (a1)')"])
  N31{"IF not ok(run_skill('gotoJ_deg', j1_val, *home_j2_j6))?"}
  N32(["RETURN False"])
  N33(["RETURN True"])
  N34(["END return_back_to_home"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 -- "yes" --> N4
  N4 -- "yes" --> N5
  N4 --> N6
  N3 --> N6
  N6 --> N7
  N7 --> N8
  N8 --> N9
  N9 --> N10
  N10 -- "yes" --> N11
  N10 --> N12
  N12 -- "no" --> N13
  N13 -- "no" --> N14
  N14 -- "no" --> N15
  N15 -- "no" --> N16
  N16 -- "no" --> N17
  N17 -- "no" --> N18
  N18 -- "no" --> N19
  N19 -- "no" --> N20
  N20 -- "no" --> N21
  N21 -- "no" --> N22
  N22 -- "no" --> N23
  N23 -- "no" --> N24
  N24 -- "no" --> N25
  N25 -- "no" --> N26
  N26 -- "no" --> N27
  N27 -- "no" --> N28
  N12 --> N29
  N13 --> N29
  N14 --> N29
  N15 --> N29
  N16 --> N29
  N17 --> N29
  N18 --> N29
  N19 --> N29
  N20 --> N29
  N21 --> N29
  N22 --> N29
  N23 --> N29
  N24 --> N29
  N25 --> N29
  N26 --> N29
  N27 --> N29
  N28 --> N29
  N28 --> N29
  N29 -- "yes" --> N30
  N29 --> N31
  N31 -- "yes" --> N32
  N31 --> N33
  N0 --> N34
```

### `get_machine_position`

Calibrate and record machine positions for all coffee equipment.

- **Mermaid file:** [../mermaid/home/get_machine_position.mmd](../mermaid/home/get_machine_position.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not return_back_to_home()`
  - `not _calibrate_marker("portafilter_cleaner", _prep_cleaner, ok)`
  - `not _calibrate_marker("espresso_grinder", _prep_grinder, ok)`
  - `not _calibrate_marker("three_group_espresso", _prep_espresso, ok)`
  - `not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME))`
  - `not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['portafilter_cleaner']['prep_position']))`
  - `not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep1']))`
  - `not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep2']))`
  - `not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['three_group_espresso_calibration']['prep1']))`
  - `not ok(run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0))`
  - `not ok(run_skill("move_to", "portafilter_cleaner", 0.22))`
  - `not ok(run_skill("move_to", "espresso_grinder", 0.22))`
  - `not ok(run_skill("move_to", "three_group_espresso", 0.22))`

```mermaid
flowchart TD
  N0(["START get_machine_position(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Call: invalidate_port_cache()"]
  N3["Call: invalidate_cleaning_cache()"]
  N4["Call: angled_invalidate_cleaning_cache()"]
  N5["Call: angled_invalidate_port_cache()"]
  N6["Robot call: run_skill('set_speed_factor', SPEED_FAST)"]
  N7{"IF not return_back_to_home()?"}
  N8(["RETURN False"])
  N9["Nested helper defined: _prep_cleaner()"]
  N10{"IF not _calibrate_marker('portafilter_cleaner', _prep_cleaner, ok)?"}
  N11(["RETURN False"])
  N12["Nested helper defined: _prep_grinder()"]
  N13{"IF not _calibrate_marker('espresso_grinder', _prep_grinder, ok)?"}
  N14(["RETURN False"])
  N15["Nested helper defined: _prep_espresso()"]
  N16{"IF not _calibrate_marker('three_group_espresso', _prep_espresso, ok)?"}
  N17(["RETURN False"])
  N18{"IF not ok(run_skill('gotoJ_deg', *ESPRESSO_HOME))?"}
  N19(["RETURN False"])
  N20(["RETURN True"])
  N21(["END get_machine_position"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 --> N6
  N6 --> N7
  N7 -- "yes" --> N8
  N7 --> N9
  N9 --> N10
  N10 -- "yes" --> N11
  N10 --> N12
  N12 --> N13
  N13 -- "yes" --> N14
  N13 --> N15
  N15 --> N16
  N16 -- "yes" --> N17
  N16 --> N18
  N18 -- "yes" --> N19
  N18 --> N20
  N0 --> N21
```

### `check_saved_data`

Check and display currently saved machine position data.

- **Mermaid file:** [../mermaid/home/check_saved_data.mmd](../mermaid/home/check_saved_data.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not os.path.exists(mem_path)`

```mermaid
flowchart TD
  N0(["START check_saved_data(**params)"])
  N1(["END check_saved_data"])
  N0 --> N1
```

### `check_aruco_status`

Check current ArUco marker detection status and help diagnose calibration issues.

- **Mermaid file:** [../mermaid/home/check_aruco_status.mmd](../mermaid/home/check_aruco_status.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START check_aruco_status(**params)"])
  N1(["RETURN True"])
  N2(["END check_aruco_status"])
  N0 --> N1
  N0 --> N2
```

### `open_gripper`

- **Mermaid file:** [../mermaid/home/open_gripper.mmd](../mermaid/home/open_gripper.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START open_gripper(**params)"])
  N1["Robot call: run_skill('sync')"]
  N2["Robot call: run_skill('set_gripper_position', 255, 0, 255)"]
  N3(["RETURN True"])
  N4(["END open_gripper"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N0 --> N4
```

### `close_gripper`

- **Mermaid file:** [../mermaid/home/close_gripper.mmd](../mermaid/home/close_gripper.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START close_gripper(**params)"])
  N1["Robot call: run_skill('sync')"]
  N2["Robot call: run_skill('set_gripper_position', 255, 255, 255)"]
  N3(["RETURN True"])
  N4(["END close_gripper"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N0 --> N4
```

### `toggle_drag_mode`

- **Mermaid file:** [../mermaid/home/toggle_drag_mode.mmd](../mermaid/home/toggle_drag_mode.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START toggle_drag_mode(**params)"])
  N1["Robot call: run_skill('toggle_drag_mode')"]
  N2(["RETURN True"])
  N3(["END toggle_drag_mode"])
  N0 --> N1
  N1 --> N2
  N0 --> N3
```

### `reset_robot1`

Restart robot1 deployment via kubectl on NUC (qss@192.168.200.254).

- **Mermaid file:** [../mermaid/home/reset_robot1.mmd](../mermaid/home/reset_robot1.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START reset_robot1(**params)"])
  N1(["RETURN _run_kubectl_rollout_restart_on_nuc('robot1')"])
  N2(["END reset_robot1"])
  N0 --> N1
  N0 --> N2
```

### `reset_robot2`

Restart robot2 deployment via kubectl on NUC (qss@192.168.200.254).

- **Mermaid file:** [../mermaid/home/reset_robot2.mmd](../mermaid/home/reset_robot2.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START reset_robot2(**params)"])
  N1(["RETURN _run_kubectl_rollout_restart_on_nuc('robot2')"])
  N2(["END reset_robot2"])
  N0 --> N1
  N0 --> N2
```

## Support/helper functions

### `run_skill`

- **Mermaid file:** [../mermaid/home/run_skill.mmd](../mermaid/home/run_skill.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START run_skill(**params)"])
  N1["Robot call: _trace_step('run_skill', f'(skill_name) START args=(_trace_format_value(skill_args)) kwargs=(_trace_forma..."]
  N2["Robot call: result = _raw_run_skill(*args, **kwargs)"]
  N3["Robot call: _trace_step('run_skill', f'(skill_name) DONE result=(_trace_format_value(result))')"]
  N4(["RETURN result"])
  N5(["END run_skill"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N0 --> N5
```

### `_calibrate_marker`

Retry a single marker calibration up to MAX_CALIBRATION_RETRIES times. prep_fn must move the arm into the correct approach pose and call sync. Returns True on the first successful get_machine_position, False if all attempts are exhausted.

- **Mermaid file:** [../mermaid/home/_calibrate_marker.mmd](../mermaid/home/_calibrate_marker.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not prep_fn()`
  - `ok(result)`

```mermaid
flowchart TD
  N0(["START _calibrate_marker(**params)"])
  N1{"LOOP attempt in range(1, MAX_CALIBRATION_RETRIES + 1)"}
  N2{"IF not prep_fn()?"}
  N3["Call: _log.warning(f'(CALIBRATION) (marker_name) prep failed (attempt (attempt)/(MAX_CALIBRATION_RETRIES))')"]
  N4["Call: time.sleep(1.0)"]
  N5["CONTINUE"]
  N6["Robot call: result = run_skill('get_machine_position', marker_name)"]
  N7{"IF ok(result)?"}
  N8(["RETURN True"])
  N9["Call: _log.warning(f'(CALIBRATION) (marker_name) failed (attempt (attempt)/(MAX_CALIBRATION_RETRIES))')"]
  N10["Call: time.sleep(1.0)"]
  N11["Call: _log.error(f'(CALIBRATION) (marker_name) failed after (MAX_CALIBRATION_RETRIES) attempts')"]
  N12(["RETURN False"])
  N13(["END _calibrate_marker"])
  N0 --> N1
  N1 -- "iterate" --> N2
  N2 -- "yes" --> N3
  N3 --> N4
  N4 --> N5
  N5 --> N6
  N2 --> N6
  N6 --> N7
  N7 -- "yes" --> N8
  N7 --> N9
  N9 --> N10
  N10 -- "next/retry" --> N1
  N1 --> N11
  N11 --> N12
  N0 --> N13
```

### `solution`

Convert joint values to cartesian, apply offsets, and convert back to joints.

- **Mermaid file:** [../mermaid/home/solution.mmd](../mermaid/home/solution.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not pos_result or not hasattr(pos_result, 'pose')`
  - `inv_result and hasattr(inv_result, 'angle')`

```mermaid
flowchart TD
  N0(["START solution(**params)"])
  N1["Robot call: pos_result = run_skill('positive_solution', j1, j2, j3, j4, j5, j6)"]
  N2{"IF not pos_result or not hasattr(pos_result, 'pose')?"}
  N3(["RETURN None"])
  N4["Robot call: inv_result = run_skill('inverse_solution', new_x, new_y, new_z, new_rx, new_ry, new_rz)"]
  N5{"IF inv_result and hasattr(inv_result, 'angle')?"}
  N6(["RETURN None"])
  N7(["RETURN inv_result"])
  N8(["END solution"])
  N0 --> N1
  N1 --> N2
  N2 -- "yes" --> N3
  N2 --> N4
  N4 --> N5
  N5 -- "no" --> N6
  N5 --> N7
  N0 --> N8
```

### `solution_interactive`

Interactive wrapper for solution function that prompts for input.

- **Mermaid file:** [../mermaid/home/solution_interactive.mmd](../mermaid/home/solution_interactive.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START solution_interactive(**params)"])
  N1(["END solution_interactive"])
  N0 --> N1
```

### `_run_kubectl_rollout_restart_on_nuc`

Run kubectl rollout restart on the NUC via SSH. Returns True on success.

- **Mermaid file:** [../mermaid/home/_run_kubectl_rollout_restart_on_nuc.mmd](../mermaid/home/_run_kubectl_rollout_restart_on_nuc.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `result.returncode == 0`

```mermaid
flowchart TD
  N0(["START _run_kubectl_rollout_restart_on_nuc(**params)"])
  N1(["END _run_kubectl_rollout_restart_on_nuc"])
  N0 --> N1
```
