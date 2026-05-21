# `milk_frothing.py` flowcharts

- Sequence/exported functions: 10
- Support/helper functions: 6

## Sequence/exported functions

### `get_frother_position`

Calibrate and record the milk frother position for future operations. After first successful calibration, skip re-reading machine position on later runs.

- **Mermaid file:** [../mermaid/milk_frothing/get_frother_position.mmd](../mermaid/milk_frothing/get_frother_position.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not return_back_to_home()`
  - `not home(position="north_east")`
  - `not _get_frother_position_done`
  - `not home(position="north_east")`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_EXTRA_POSES['frother_calibration_transition']))`
  - `not _milk_frother_position_done`
  - `not home(position="north")`
  - `not steam_calibrated`
  - `not frother_calibrated`
  - `not prep_ok`
  - `ok(run_skill("get_machine_position", "left_steam_wand"))`
  - `not prep_ok`
  - `ok(run_skill("get_machine_position", "milk_frother_2"))`
  - `not ok(run_skill("move_to", "left_steam_wand", 0.29))`
  - `not ok(run_skill("move_to", "milk_frother_2", 0.29))`

```mermaid
flowchart TD
  N0(["START get_frother_position(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Call: invalidate_plastic_cup_cache()"]
  N3["Call: invalidate_milk_frothing_cache()"]
  N4["Robot call: run_skill('set_speed_factor', 100)"]
  N5{"IF not return_back_to_home()?"}
  N6(["RETURN False"])
  N7{"IF not home(position='north_east')?"}
  N8(["RETURN False"])
  N9{"IF not _get_frother_position_done?"}
  N10["State/cache: steam_calibrated = False"]
  N11{"LOOP attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1)"}
  N12{"LOOP _ in range(cycles)"}
  N13["Call: time.sleep(CALIBRATION_SETTLE_TIME)"]
  N14{"IF not ok(run_skill('move_to', 'left_steam_wand', 0.29))?"}
  N15["BREAK"]
  N16{"IF not prep_ok?"}
  N17["Call: _log.warning(f'(CALIBRATION) left_steam_wand prep failed (attempt (attempt)/(MAX_FROTHER_CALIBRATION_RETRIES))')"]
  N18["Call: time.sleep(1.0)"]
  N19["CONTINUE"]
  N20{"IF ok(run_skill('get_machine_position', 'left_steam_wand'))?"}
  N21["State/cache: steam_calibrated = True"]
  N22["BREAK"]
  N23["Call: _log.warning(f'(CALIBRATION) left_steam_wand failed (attempt (attempt)/(MAX_FROTHER_CALIBRATION_RETRIES))')"]
  N24["Call: time.sleep(1.0)"]
  N25{"IF not steam_calibrated?"}
  N26["Call: _log.error(f'(CALIBRATION) left_steam_wand failed after (MAX_FROTHER_CALIBRATION_RETRIES) attempts')"]
  N27(["RETURN False"])
  N28{"IF not home(position='north_east')?"}
  N29(["RETURN False"])
  N30{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_EXTRA_POSES('frother_calibration_transition')))?"}
  N31(["RETURN False"])
  N32{"IF not _milk_frother_position_done?"}
  N33["State/cache: frother_calibrated = False"]
  N34{"LOOP attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1)"}
  N35{"LOOP _ in range(cycles)"}
  N36["Call: time.sleep(CALIBRATION_SETTLE_TIME)"]
  N37{"IF not ok(run_skill('move_to', 'milk_frother_2', 0.29))?"}
  N38["BREAK"]
  N39{"IF not prep_ok?"}
  N40["Call: _log.warning(f'(CALIBRATION) milk_frother_2 prep failed (attempt (attempt)/(MAX_FROTHER_CALIBRATION_RETRIES))')"]
  N41["Call: time.sleep(1.0)"]
  N42["CONTINUE"]
  N43{"IF ok(run_skill('get_machine_position', 'milk_frother_2'))?"}
  N44["State/cache: frother_calibrated = True"]
  N45["BREAK"]
  N46["Call: _log.warning(f'(CALIBRATION) milk_frother_2 failed (attempt (attempt)/(MAX_FROTHER_CALIBRATION_RETRIES))')"]
  N47["Call: time.sleep(1.0)"]
  N48{"IF not frother_calibrated?"}
  N49["Call: _log.error(f'(CALIBRATION) milk_frother_2 failed after (MAX_FROTHER_CALIBRATION_RETRIES) attempts')"]
  N50(["RETURN False"])
  N51{"IF not home(position='north')?"}
  N52(["RETURN False"])
  N53(["RETURN True"])
  N54(["END get_frother_position"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 -- "yes" --> N6
  N5 --> N7
  N7 -- "yes" --> N8
  N7 --> N9
  N9 -- "yes" --> N10
  N10 --> N11
  N11 -- "iterate" --> N12
  N12 -- "iterate" --> N13
  N13 --> N14
  N14 -- "yes" --> N15
  N15 -- "next/retry" --> N12
  N14 -- "next/retry" --> N12
  N12 --> N16
  N16 -- "yes" --> N17
  N17 --> N18
  N18 --> N19
  N19 --> N20
  N16 --> N20
  N20 -- "yes" --> N21
  N21 --> N22
  N22 --> N23
  N20 --> N23
  N23 --> N24
  N24 -- "next/retry" --> N11
  N11 --> N25
  N25 -- "yes" --> N26
  N26 --> N27
  N25 --> N28
  N9 --> N28
  N28 -- "yes" --> N29
  N28 --> N30
  N30 -- "yes" --> N31
  N30 --> N32
  N32 -- "yes" --> N33
  N33 --> N34
  N34 -- "iterate" --> N35
  N35 -- "iterate" --> N36
  N36 --> N37
  N37 -- "yes" --> N38
  N38 -- "next/retry" --> N35
  N37 -- "next/retry" --> N35
  N35 --> N39
  N39 -- "yes" --> N40
  N40 --> N41
  N41 --> N42
  N42 --> N43
  N39 --> N43
  N43 -- "yes" --> N44
  N44 --> N45
  N45 --> N46
  N43 --> N46
  N46 --> N47
  N47 -- "next/retry" --> N34
  N34 --> N48
  N48 -- "yes" --> N49
  N49 --> N50
  N48 --> N51
  N32 --> N51
  N51 -- "yes" --> N52
  N51 --> N53
  N0 --> N54
```

### `pick_frother`

Pick up the milk frother for milk frothing operations. Live/no-cache pickup verification: - After the secure gripper close, the reported gripper position must be within PICK_FROTHER_GRIP_VERIFY_TARGET +/- PICK_FROTHER_GRIP_VERIFY_TOLERANCE. - If verification fails, reverse back to the pickup area, refresh only the milk_frother_2 machine position, and retry the whole live pickup routine. Important: do NOT call the full get_frother_position() inside this recovery. That function runs home/return...

- **Mermaid file:** [../mermaid/milk_frothing/pick_frother.mmd](../mermaid/milk_frothing/pick_frother.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not success`
  - `not frother_calibrated`
  - `not ok(run_skill("mount_machine", 'milk_frother_2', 'milk_frother_1'))`
  - `not ok(run_skill( "set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'], ))`
  - `not ok(run_skill("approach_machine", 'milk_frother_2', 'milk_frother_1'))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area']))`
  - `not _refresh_milk_frother_position_only(attempt_idx)`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area']))`
  - `not ok(run_skill("approach_machine", 'milk_frother_2', 'milk_frother_1'))`
  - `not ok(run_skill( "set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'], ))`
  - `not ok(run_skill("mount_machine", 'milk_frother_2', 'milk_frother_1'))`
  - `gripper_verified`
  - `attempt_idx >= total_attempts`
  - `not _reverse_to_pickup_area_and_recalibrate(attempt_idx)`
  - `not prep_ok`
  - `ok(run_skill("get_machine_position", "milk_frother_2"))`
  - `not ok(run_skill("move_to", "milk_frother_2", 0.29))`

```mermaid
flowchart TD
  N0(["START pick_frother(**params)"])
  N1["Call: _trace_step('pick_frother', 'START')"]
  N2["Nested helper defined: ok()"]
  N3["Nested helper defined: _secure_gripper_and_verify()"]
  N4["Nested helper defined: _refresh_milk_frother_position_only()"]
  N5["Nested helper defined: _reverse_to_pickup_area_and_recalibrate()"]
  N6["State/cache: total_attempts = PICK_FROTHER_GRIP_VERIFY_RETRIES + 1"]
  N7{"LOOP attempt_idx in range(1, total_attempts + 1)"}
  N8["Call: _log.info( f'(PICK-FROTHER-GRIP) pickup attempt (attempt_idx)/(total_attempts)' )"]
  N9{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('pickup')('area')))?"}
  N10(["RETURN False"])
  N11{"IF not ok(run_skill('approach_machine', 'milk_frother_2', 'milk_frother_1'))?"}
  N12(["RETURN False"])
  N13["Robot call: run_skill('sync')"]
  N14{"IF not ok(run_skill( 'set_gripper_position', GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS('pickup_initial'), ))?"}
  N15(["RETURN False"])
  N16{"IF not ok(run_skill('mount_machine', 'milk_frother_2', 'milk_frother_1'))?"}
  N17(["RETURN False"])
  N18["Robot call: run_skill('sync')"]
  N19{"IF gripper_verified?"}
  N20(["RETURN True"])
  N21["Call: _log.warning( f'(PICK-FROTHER-GRIP) secure grip verify failed on attempt ' f'(attempt_idx)/(total_attempts): po..."]
  N22{"IF attempt_idx >= total_attempts?"}
  N23["Call: _log.error( f'(PICK-FROTHER-GRIP) FINAL FAIL after (total_attempts) attempts: ' f'last_pos=(gripper_pos), requi..."]
  N24(["RETURN False"])
  N25{"IF not _reverse_to_pickup_area_and_recalibrate(attempt_idx)?"}
  N26(["RETURN False"])
  N27(["RETURN False"])
  N28(["END pick_frother"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 --> N6
  N6 --> N7
  N7 -- "iterate" --> N8
  N8 --> N9
  N9 -- "yes" --> N10
  N9 --> N11
  N11 -- "yes" --> N12
  N11 --> N13
  N13 --> N14
  N14 -- "yes" --> N15
  N14 --> N16
  N16 -- "yes" --> N17
  N16 --> N18
  N18 --> N19
  N19 -- "yes" --> N20
  N19 --> N21
  N21 --> N22
  N22 -- "yes" --> N23
  N23 --> N24
  N22 --> N25
  N25 -- "yes" --> N26
  N25 -- "next/retry" --> N7
  N7 --> N27
  N0 --> N28
```

### `unmount_and_swirl_milk`

Swirl frothed milk in a circular motion for latte art preparation.

- **Mermaid file:** [../mermaid/milk_frothing/unmount_and_swirl_milk.mmd](../mermaid/milk_frothing/unmount_and_swirl_milk.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_EXTRA_POSES['deep_froth_approach']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos']))`

```mermaid
flowchart TD
  N0(["START unmount_and_swirl_milk(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Call: time.sleep(MILK_FROTHING_DELAYS('swirl_delay'))"]
  N3{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_EXTRA_POSES('deep_froth_approach')))?"}
  N4(["RETURN False"])
  N5["Robot call: run_skill('set_speed_factor', MILK_FROTHER_SPEEDS('swirl'))"]
  N6{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('swirling')('intermediate1')))?"}
  N7(["RETURN False"])
  N8{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('swirling')('swirl_pos')))?"}
  N9(["RETURN False"])
  N10["Robot call: run_skill('sync')"]
  N11["Robot call: run_skill( 'move_circle', MILK_SWIRL_CIRCLE_PARAMS('cycles'), MILK_SWIRL_CIRCLE_PARAMS('point1_offset'), ..."]
  N12(["RETURN True"])
  N13(["END unmount_and_swirl_milk"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 -- "yes" --> N4
  N3 --> N5
  N5 --> N6
  N6 -- "yes" --> N7
  N6 --> N8
  N8 -- "yes" --> N9
  N8 --> N10
  N10 --> N11
  N11 --> N12
  N0 --> N13
```

### `pour_milk_cup_station`

- **Mermaid file:** [../mermaid/milk_frothing/pour_milk_cup_station.mmd](../mermaid/milk_frothing/pour_milk_cup_station.mmd)
- **Parameter scenarios observed:** `position.cup_position / stage`
- **Branch/decision scenarios:**
  - `not ok(run_skill("gotoJ_deg", *stage_cfg['position']))`
  - `not ok(run_skill("gotoJ_deg", *stage_cfg['adjust1']))`
  - `_is_valid_angles(stage_cache.get('forward'))`
  - `_is_valid_angles(stage_cache.get('up'))`
  - `not ok(run_skill("gotoJ_deg", *stage_cfg['position']))`
  - `not ok(run_skill("gotoJ_deg", *stage_cache['forward']))`
  - `not ok(run_skill("moveEE_movJ", *stage_offsets['move_forward']))`
  - `not _is_valid_angles(forward_pose)`
  - `not ok(run_skill("gotoJ_deg", *stage_cache['up']))`
  - `not ok(run_skill("moveEE_movJ", *stage_offsets['move_up']))`
  - `not _is_valid_angles(up_pose)`

```mermaid
flowchart TD
  N0(["START pour_milk_cup_station(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Parameter/normalization: cup_position = _extract_cup_position(params)"]
  N3["State/cache: stage_cache = _pour_milk_cup_station_cache.setdefault(stage, ())"]
  N4["Robot call: run_skill('set_speed_factor', MILK_FROTHER_SPEEDS('pour_approach'))"]
  N5{"IF not ok(run_skill('gotoJ_deg', *stage_cfg('position')))?"}
  N6(["RETURN False"])
  N7["Robot call: run_skill('sync')"]
  N8["Robot call: run_skill('set_speed_factor', MILK_FROTHER_SPEEDS('pour'))"]
  N9{"IF not ok(run_skill('gotoJ_deg', *stage_cfg('adjust1')))?"}
  N10(["RETURN False"])
  N11{"IF _is_valid_angles(stage_cache.get('forward'))?"}
  N12{"IF not ok(run_skill('gotoJ_deg', *stage_cache('forward')))?"}
  N13(["RETURN False"])
  N14["Robot call: run_skill('sync')"]
  N15{"IF not ok(run_skill('moveEE_movJ', *stage_offsets('move_forward')))?"}
  N16(["RETURN False"])
  N17["State/cache: forward_pose = _capture_current_angles()"]
  N18{"IF not _is_valid_angles(forward_pose)?"}
  N19(["RETURN False"])
  N20["State/cache: stage_cache('forward') = forward_pose"]
  N21{"IF _is_valid_angles(stage_cache.get('up'))?"}
  N22{"IF not ok(run_skill('gotoJ_deg', *stage_cache('up')))?"}
  N23(["RETURN False"])
  N24{"IF not ok(run_skill('moveEE_movJ', *stage_offsets('move_up')))?"}
  N25(["RETURN False"])
  N26["State/cache: up_pose = _capture_current_angles()"]
  N27{"IF not _is_valid_angles(up_pose)?"}
  N28(["RETURN False"])
  N29["State/cache: stage_cache('up') = up_pose"]
  N30{"IF not ok(run_skill('gotoJ_deg', *stage_cfg('position')))?"}
  N31(["RETURN False"])
  N32["Robot call: run_skill('set_speed_factor', MILK_FROTHER_SPEEDS('return'))"]
  N33(["RETURN True"])
  N34(["END pour_milk_cup_station"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 -- "yes" --> N6
  N5 --> N7
  N7 --> N8
  N8 --> N9
  N9 -- "yes" --> N10
  N9 --> N11
  N11 -- "yes" --> N12
  N12 -- "yes" --> N13
  N12 --> N14
  N11 -- "no" --> N15
  N15 -- "yes" --> N16
  N15 --> N17
  N17 --> N18
  N18 -- "yes" --> N19
  N18 --> N20
  N14 --> N21
  N20 --> N21
  N21 -- "yes" --> N22
  N22 -- "yes" --> N23
  N21 -- "no" --> N24
  N24 -- "yes" --> N25
  N24 --> N26
  N26 --> N27
  N27 -- "yes" --> N28
  N27 --> N29
  N22 --> N30
  N29 --> N30
  N30 -- "yes" --> N31
  N30 --> N32
  N32 --> N33
  N0 --> N34
```

### `mount_frother`

Mount the milk frother to the steam wand for frothing preparation. Applies a Z adjustment based on milk volume and cup size.

- **Mermaid file:** [../mermaid/milk_frothing/mount_frother.mmd](../mermaid/milk_frothing/mount_frother.mmd)
- **Parameter scenarios observed:** `ingredients`, `ingredients.cups / cups / size / cup_size`, `milk`
- **Branch/decision scenarios:**
  - `not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth"))`
  - `not ok(run_skill("mount_machine", "left_steam_wand", "deep_froth"))`
  - `not cup_size`
  - `not cup_size`
  - `not ok(run_skill("moveEE_movJ", 12.5, 12.5, -z_adjustment, 7.5, 0, 0))`

```mermaid
flowchart TD
  N0(["START mount_frother(**params)"])
  N1["Nested helper defined: ok()"]
  N2["Robot call: run_skill('set_speed_factor', MILK_FROTHER_SPEEDS('mount'))"]
  N3{"IF not ok(run_skill('approach_machine', 'left_steam_wand', 'deep_froth'))?"}
  N4(["RETURN False"])
  N5{"IF not ok(run_skill('mount_machine', 'left_steam_wand', 'deep_froth'))?"}
  N6(["RETURN False"])
  N7["Parameter/normalization: cups_dict = _extract_cups_dict(params)"]
  N8["Parameter/normalization: cup_size = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')"]
  N9{"IF not cup_size?"}
  N10["Parameter/normalization: cup_size = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')"]
  N11{"IF not cup_size?"}
  N12["Parameter/normalization: milk_data = params.get('milk') or params.get('ingredients', ()).get('milk', ()) or ()"]
  N13{"IF not ok(run_skill('moveEE_movJ', 12.5, 12.5, -z_adjustment, 7.5, 0, 0))?"}
  N14(["RETURN False"])
  N15(["RETURN True"])
  N16(["END mount_frother"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 -- "yes" --> N4
  N3 --> N5
  N5 -- "yes" --> N6
  N5 --> N7
  N7 --> N8
  N8 --> N9
  N9 -- "yes" --> N10
  N10 --> N11
  N9 --> N11
  N11 --> N12
  N11 --> N12
  N12 --> N13
  N13 -- "yes" --> N14
  N13 --> N15
  N0 --> N16
```

### `clean_milk_pitcher`

- **Mermaid file:** [../mermaid/milk_frothing/clean_milk_pitcher.mmd](../mermaid/milk_frothing/clean_milk_pitcher.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3']))`
  - `_is_valid_angles(_clean_milk_pitcher_cache)`
  - `not ok(run_skill("gotoJ_deg", *_clean_milk_pitcher_cache))`
  - `not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion']))`
  - `not _is_valid_angles(clean_pose)`

```mermaid
flowchart TD
  N0(["START clean_milk_pitcher(**params)"])
  N1["Nested helper defined: ok()"]
  N2{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('cleaning')('pose1')))?"}
  N3(["RETURN False"])
  N4{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('cleaning')('pose2')))?"}
  N5(["RETURN False"])
  N6{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('cleaning')('pose3')))?"}
  N7(["RETURN False"])
  N8{"IF _is_valid_angles(_clean_milk_pitcher_cache)?"}
  N9{"IF not ok(run_skill('gotoJ_deg', *_clean_milk_pitcher_cache))?"}
  N10(["RETURN False"])
  N11["Robot call: run_skill('sync')"]
  N12{"IF not ok(run_skill('moveEE_movJ', *MILK_FROTHER_MOVEMENT_OFFSETS('cleaning_motion')))?"}
  N13(["RETURN False"])
  N14["State/cache: clean_pose = _capture_current_angles()"]
  N15{"IF not _is_valid_angles(clean_pose)?"}
  N16(["RETURN False"])
  N17["State/cache: _clean_milk_pitcher_cache = clean_pose"]
  N18(["RETURN True"])
  N19(["END clean_milk_pitcher"])
  N0 --> N1
  N1 --> N2
  N2 -- "yes" --> N3
  N2 --> N4
  N4 -- "yes" --> N5
  N4 --> N6
  N6 -- "yes" --> N7
  N6 --> N8
  N8 -- "yes" --> N9
  N9 -- "yes" --> N10
  N9 --> N11
  N8 -- "no" --> N12
  N12 -- "yes" --> N13
  N12 --> N14
  N14 --> N15
  N15 -- "yes" --> N16
  N15 --> N17
  N11 --> N18
  N17 --> N18
  N0 --> N19
```

### `return_frother`

Return the frother to its original location using recorded approach/grab angles.

- **Mermaid file:** [../mermaid/milk_frothing/return_frother.mmd](../mermaid/milk_frothing/return_frother.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4']))`
  - `not _is_valid_position(cached_lift)`
  - `not ok(run_skill("gotoEE", *cached_lift))`
  - `not ok(run_skill("moveEE", 0, 0, -145, 0, 0, 0))`
  - `not ok(run_skill("set_gripper_position", 75, 165, 255))`
  - `not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach']))`
  - `not ok(run_skill("approach_machine", "milk_frother_2", "milk_frother_1"))`
  - `not ok(home(position="north"))`
  - `not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open']))`

```mermaid
flowchart TD
  N0(["START return_frother(**params)"])
  N1["Nested helper defined: ok()"]
  N2{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('return')('pre_return1')))?"}
  N3(["RETURN False"])
  N4{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('return')('pre_return2')))?"}
  N5(["RETURN False"])
  N6{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('return')('pre_return3')))?"}
  N7(["RETURN False"])
  N8{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('return')('pre_return4')))?"}
  N9(["RETURN False"])
  N10["State/cache: cached_lift = _place_frother_milk_station_cache.get('lift_after_place')"]
  N11{"IF not _is_valid_position(cached_lift)?"}
  N12(["RETURN False"])
  N13{"IF not ok(run_skill('gotoEE', *cached_lift))?"}
  N14(["RETURN False"])
  N15{"IF not ok(run_skill('moveEE', 0, 0, -145, 0, 0, 0))?"}
  N16(["RETURN False"])
  N17["Call: time.sleep(0.5)"]
  N18{"IF not ok(run_skill('set_gripper_position', 75, 165, 255))?"}
  N19(["RETURN False"])
  N20["Call: time.sleep(0.5)"]
  N21{"IF not ok(run_skill('moveEE', *MILK_FROTHER_MOVEMENT_OFFSETS('final_approach')))?"}
  N22(["RETURN False"])
  N23{"IF not ok(run_skill('approach_machine', 'milk_frother_2', 'milk_frother_1'))?"}
  N24(["RETURN False"])
  N25{"IF not ok(home(position='north'))?"}
  N26(["RETURN False"])
  N27["Robot call: run_skill('sync')"]
  N28{"IF not ok(run_skill('set_gripper_position', GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS('open')))?"}
  N29(["RETURN False"])
  N30(["RETURN True"])
  N31(["END return_frother"])
  N0 --> N1
  N1 --> N2
  N2 -- "yes" --> N3
  N2 --> N4
  N4 -- "yes" --> N5
  N4 --> N6
  N6 -- "yes" --> N7
  N6 --> N8
  N8 -- "yes" --> N9
  N8 --> N10
  N10 --> N11
  N11 -- "yes" --> N12
  N11 --> N13
  N13 -- "yes" --> N14
  N13 --> N15
  N15 -- "yes" --> N16
  N15 --> N17
  N17 --> N18
  N18 -- "yes" --> N19
  N18 --> N20
  N20 --> N21
  N21 -- "yes" --> N22
  N21 --> N23
  N23 -- "yes" --> N24
  N23 --> N25
  N25 -- "yes" --> N26
  N25 --> N27
  N27 --> N28
  N28 -- "yes" --> N29
  N28 --> N30
  N0 --> N31
```

### `place_frother_milk_station`

- **Mermaid file:** [../mermaid/milk_frothing/place_frother_milk_station.mmd](../mermaid/milk_frothing/place_frother_milk_station.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `_is_valid_position(cached_lift)`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final']))`
  - `_is_valid_angles(cached_nudge)`
  - `not ok(run_skill("set_gripper_position", 25, 220, 255))`
  - `not ok(run_skill("gotoEE", *cached_lift))`
  - `not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place']))`
  - `not _is_valid_position(lift_pose)`
  - `not ok(run_skill("gotoJ_deg", *cached_nudge))`
  - `not ok(run_skill("moveEE_movJ", 5, 0, 0, 0, 0, 0))`
  - `not _is_valid_angles(nudge_pose)`

```mermaid
flowchart TD
  N0(["START place_frother_milk_station(**params)"])
  N1["Call: _trace_step('place_frother_milk_station', 'START')"]
  N2["Nested helper defined: ok()"]
  N3["State/cache: cached_lift = _place_frother_milk_station_cache.get('lift_after_place')"]
  N4["State/cache: cached_nudge = _place_frother_milk_station_cache.get('final_nudge')"]
  N5{"IF _is_valid_position(cached_lift)?"}
  N6{"IF not ok(run_skill('gotoEE', *cached_lift))?"}
  N7(["RETURN False"])
  N8{"IF not ok(run_skill('moveEE', *MILK_FROTHER_MOVEMENT_OFFSETS('lift_after_place')))?"}
  N9(["RETURN False"])
  N10["Call: lift_pose = _capture_current_position()"]
  N11{"IF not _is_valid_position(lift_pose)?"}
  N12(["RETURN False"])
  N13["State/cache: _place_frother_milk_station_cache('lift_after_place') = lift_pose"]
  N14{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('place_pre1')))?"}
  N15(["RETURN False"])
  N16{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('place_pre2')))?"}
  N17(["RETURN False"])
  N18{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('place_approach')))?"}
  N19(["RETURN False"])
  N20{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('place_final')))?"}
  N21(["RETURN False"])
  N22{"IF _is_valid_angles(cached_nudge)?"}
  N23{"IF not ok(run_skill('gotoJ_deg', *cached_nudge))?"}
  N24(["RETURN False"])
  N25["Robot call: run_skill('sync')"]
  N26{"IF not ok(run_skill('moveEE_movJ', 5, 0, 0, 0, 0, 0))?"}
  N27(["RETURN False"])
  N28["State/cache: nudge_pose = _capture_current_angles()"]
  N29{"IF not _is_valid_angles(nudge_pose)?"}
  N30(["RETURN False"])
  N31["State/cache: _place_frother_milk_station_cache('final_nudge') = nudge_pose"]
  N32{"IF not ok(run_skill('set_gripper_position', 25, 220, 255))?"}
  N33(["RETURN False"])
  N34(["RETURN True"])
  N35(["END place_frother_milk_station"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 -- "yes" --> N6
  N6 -- "yes" --> N7
  N5 -- "no" --> N8
  N8 -- "yes" --> N9
  N8 --> N10
  N10 --> N11
  N11 -- "yes" --> N12
  N11 --> N13
  N6 --> N14
  N13 --> N14
  N14 -- "yes" --> N15
  N14 --> N16
  N16 -- "yes" --> N17
  N16 --> N18
  N18 -- "yes" --> N19
  N18 --> N20
  N20 -- "yes" --> N21
  N20 --> N22
  N22 -- "yes" --> N23
  N23 -- "yes" --> N24
  N23 --> N25
  N22 -- "no" --> N26
  N26 -- "yes" --> N27
  N26 --> N28
  N28 --> N29
  N29 -- "yes" --> N30
  N29 --> N31
  N25 --> N32
  N31 --> N32
  N32 -- "yes" --> N33
  N32 --> N34
  N0 --> N35
```

### `pick_frother_milk_station`

- **Mermaid file:** [../mermaid/milk_frothing/pick_frother_milk_station.mmd](../mermaid/milk_frothing/pick_frother_milk_station.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not ok(run_skill("set_gripper_position", 255, 255, 255))`
  - `_is_valid_angles(_pick_frother_milk_station_cache)`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1']))`
  - `not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2']))`
  - `not ok(run_skill("gotoJ_deg", *_pick_frother_milk_station_cache))`
  - `not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick']))`
  - `not _is_valid_angles(lift_pose)`

```mermaid
flowchart TD
  N0(["START pick_frother_milk_station(**params)"])
  N1["Call: _trace_step('pick_frother_milk_station', 'START')"]
  N2["Nested helper defined: ok()"]
  N3{"IF not ok(run_skill('set_gripper_position', 255, 255, 255))?"}
  N4(["RETURN False"])
  N5{"IF _is_valid_angles(_pick_frother_milk_station_cache)?"}
  N6{"IF not ok(run_skill('gotoJ_deg', *_pick_frother_milk_station_cache))?"}
  N7(["RETURN False"])
  N8["Robot call: run_skill('sync')"]
  N9{"IF not ok(run_skill('moveEE_movJ', *MILK_FROTHER_MOVEMENT_OFFSETS('lift_after_pick')))?"}
  N10(["RETURN False"])
  N11["State/cache: lift_pose = _capture_current_angles()"]
  N12{"IF not _is_valid_angles(lift_pose)?"}
  N13(["RETURN False"])
  N14["State/cache: _pick_frother_milk_station_cache = lift_pose"]
  N15{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('pick_retreat1')))?"}
  N16(["RETURN False"])
  N17{"IF not ok(run_skill('gotoJ_deg', *MILK_FROTHING_PARAMS('milk_station')('pick_retreat2')))?"}
  N18(["RETURN False"])
  N19(["RETURN True"])
  N20(["END pick_frother_milk_station"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 -- "yes" --> N4
  N3 --> N5
  N5 -- "yes" --> N6
  N6 -- "yes" --> N7
  N6 --> N8
  N5 -- "no" --> N9
  N9 -- "yes" --> N10
  N9 --> N11
  N11 --> N12
  N12 -- "yes" --> N13
  N12 --> N14
  N8 --> N15
  N14 --> N15
  N15 -- "yes" --> N16
  N15 --> N17
  N17 -- "yes" --> N18
  N17 --> N19
  N0 --> N20
```

### `invalidate_milk_frothing_cache`

- **Mermaid file:** [../mermaid/milk_frothing/invalidate_milk_frothing_cache.mmd](../mermaid/milk_frothing/invalidate_milk_frothing_cache.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START invalidate_milk_frothing_cache(**params)"])
  N1["Call: _trace_step('invalidate_milk_frothing_cache', 'START')"]
  N2["Call: _place_frother_milk_station_cache.clear()"]
  N3["State/cache: _pick_frother_milk_station_cache = None"]
  N4["Call: _mount_frother_cache.clear()"]
  N5["Call: _pour_milk_cup_station_cache.clear()"]
  N6["State/cache: _clean_milk_pitcher_cache = None"]
  N7["Call: _return_frother_cache.clear()"]
  N8["Call: _pick_frother_cache.clear()"]
  N9(["END invalidate_milk_frothing_cache"])
  N0 --> N1
  N1 --> N2
  N2 --> N3
  N3 --> N4
  N4 --> N5
  N5 --> N6
  N6 --> N7
  N7 --> N8
  N8 --> N9
```

## Support/helper functions

### `run_skill`

Trace wrapper around manipulate_node.run_skill.

- **Mermaid file:** [../mermaid/milk_frothing/run_skill.mmd](../mermaid/milk_frothing/run_skill.mmd)
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

### `_is_valid_angles`

- **Mermaid file:** [../mermaid/milk_frothing/_is_valid_angles.mmd](../mermaid/milk_frothing/_is_valid_angles.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START _is_valid_angles(**params)"])
  N1(["RETURN bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6"])
  N2(["END _is_valid_angles"])
  N0 --> N1
  N0 --> N2
```

### `_capture_current_angles`

- **Mermaid file:** [../mermaid/milk_frothing/_capture_current_angles.mmd](../mermaid/milk_frothing/_capture_current_angles.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not _is_valid_angles(angles)`

```mermaid
flowchart TD
  N0(["START _capture_current_angles(**params)"])
  N1["State/cache: angles = run_skill('current_angles')"]
  N2{"IF not _is_valid_angles(angles)?"}
  N3(["RETURN None"])
  N4(["RETURN tuple(angles)"])
  N5(["END _capture_current_angles"])
  N0 --> N1
  N1 --> N2
  N2 -- "yes" --> N3
  N2 --> N4
  N0 --> N5
```

### `_capture_current_position`

- **Mermaid file:** [../mermaid/milk_frothing/_capture_current_position.mmd](../mermaid/milk_frothing/_capture_current_position.mmd)
- **Parameter scenarios observed:** none directly read
- **Branch/decision scenarios:**
  - `not _is_valid_position(position)`

```mermaid
flowchart TD
  N0(["START _capture_current_position(**params)"])
  N1["State/cache: position = run_skill('current_pose')"]
  N2{"IF not _is_valid_position(position)?"}
  N3(["RETURN None"])
  N4(["RETURN tuple(position)"])
  N5(["END _capture_current_position"])
  N0 --> N1
  N1 --> N2
  N2 -- "yes" --> N3
  N2 --> N4
  N0 --> N5
```

### `_is_valid_position`

- **Mermaid file:** [../mermaid/milk_frothing/_is_valid_position.mmd](../mermaid/milk_frothing/_is_valid_position.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START _is_valid_position(**params)"])
  N1(["RETURN bool(position) and isinstance(position, (tuple, list)) and len(position) == 6"])
  N2(["END _is_valid_position"])
  N0 --> N1
  N0 --> N2
```

### `_cache_key_from_z_adjustment`

- **Mermaid file:** [../mermaid/milk_frothing/_cache_key_from_z_adjustment.mmd](../mermaid/milk_frothing/_cache_key_from_z_adjustment.mmd)
- **Parameter scenarios observed:** none directly read

```mermaid
flowchart TD
  N0(["START _cache_key_from_z_adjustment(**params)"])
  N1(["RETURN f'(round(float(z_adjustment), 3):.3f)'"])
  N2(["END _cache_key_from_z_adjustment"])
  N0 --> N1
  N0 --> N2
```
