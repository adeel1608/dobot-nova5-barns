# Function Reference README

This README documents the functions in the main BARNS robot-control files currently provided for handover:

- `manipulate_node_v4.py` / `Pasted text.txt`: low-level robot motion, TF, MoveIt, and Dobot service wrappers.
- `testing_v1.py`: high-level coffee automation sequences, training/test routines, and CLI helpers.
- `params.py`: shared constants, request parsing helpers, validators, and logging helpers.

`Pasted text.txt` is byte-for-byte identical to `manipulate_node_v4.py`, so it is documented once under `manipulate_node_v4.py`.

## Function conventions

- Most high-level sequence functions in `testing_v1.py` accept `**params` and return `True` on success or `False` on failure.
- Most high-level sequence functions call `run_skill(...)`, which forwards to the selected motion implementation. The current selector is `USE_VERSION = "v4"`.
- Low-level motion functions in `manipulate_node_v4.py` talk to Dobot ROS 2 services under `/dobot_bringup_v3/srv/...`, use TF frames, and sometimes use MoveIt2.
- Functions whose names start with `_` are internal helpers. They are documented because they are part of the file, but they should not normally be called directly from external code.
- `testing_v1.py` contains repeated helper names such as `_is_valid_angles` and `_capture_current_angles` in different sections. In Python, the later definition replaces the earlier global name at runtime. They currently have similar behavior, but this is worth keeping in mind during refactoring.

## `manipulate_node_v4.py`

This file is the low-level motion layer. It defines TF acquisition helpers, the `robot_perception` node, the `robot_motion` node, Dobot service clients, MoveIt2 primitives, portafilter-specific helpers, and the generic `run_skill` wrapper.

### `RobotMotionError`

- **Base:** `Exception`
- **Purpose:** Base exception for robot motion failures

### `SyncFailureError`

- **Base:** `RobotMotionError`
- **Purpose:** Raised when sync operation fails after all retries

### `MovementFailureError`

- **Base:** `RobotMotionError`
- **Purpose:** Raised when a movement command fails after all retries

### Module-level functions

#### `get_transform_list(tf_stamped)`

- **Location:** `manipulate_node_v4.py:74`
- **Purpose:** Converts a geometry_msgs TransformStamped message into a list:  [tx, ty, tz, qx, qy, qz, qw].
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `init_motion_node()`

- **Location:** `manipulate_node_v4.py:3856`
- **Purpose:** Initialize the global motion node for sequence execution.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `init`, `robot_motion`.

#### `cleanup_motion_node()`

- **Location:** `manipulate_node_v4.py:3867`
- **Purpose:** Cleanup the global motion node.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `destroy_node`, `shutdown`.
  - Side effects: uses executor/thread cleanup.

#### `run_skill_with_node(motion_node, fn_name, *args)`

- **Location:** `manipulate_node_v4.py:3877`
- **Purpose:** Execute a robot skill using the provided motion_node instance.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `reset_robot_connection`, `is_robot_likely_responsive`, `health_check`, `fn`.
- **Additional notes from docstring:**
  - This function calls a method on the motion_node and handles different return types. If the operation fails, it logs an error and returns False.

#### `run_skill(fn_name, *args)`

- **Location:** `manipulate_node_v4.py:3950`
- **Purpose:** Execute a robot skill using the global motion node (for backward compatibility).
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `init_motion_node`, `run_skill_with_node`.
- **Additional notes from docstring:**
  - This function automatically manages the motion node lifecycle and provides the same interface as before while being more efficient for sequence execution.

#### `execute_sequence(sequence_func, **params)`

- **Location:** `manipulate_node_v4.py:3968`
- **Purpose:** Execute a sequence function with proper motion node management.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `init_motion_node`, `sequence_func`, `destroy_node`.
- **Additional notes from docstring:**
  - This function creates a motion node, executes the sequence, and cleans up properly.

### Class `robot_perception`

ROS 2 node class used by the motion stack.

#### `robot_perception.__init__(self)`

- **Location:** `manipulate_node_v4.py:98`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `Buffer`, `TransformListener`, `SingleThreadedExecutor`, `add_node`, `Thread`, `start`.
  - Side effects: uses TF lookup/broadcasting, uses executor/thread cleanup.

#### `robot_perception._safe_spin(self)`

- **Location:** `manipulate_node_v4.py:118`
- **Purpose:** Safely spin the executor with exception handling
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `robot_perception._cleanup(self)`

- **Location:** `manipulate_node_v4.py:130`
- **Purpose:** Internal cleanup method
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `shutdown`, `is_alive`.
  - Side effects: uses executor/thread cleanup.

#### `robot_perception.destroy_node(self)`

- **Location:** `manipulate_node_v4.py:148`
- **Purpose:** Override destroy_node to properly cleanup threads
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `destroy_node`.

#### `robot_perception.get_tf(self, target_frame, reference_frame='base_link', max_retries=3, sleep_time=0.035)`

- **Location:** `manipulate_node_v4.py:156`
- **Purpose:** Look up TF once (with limited retries).  If the TF infrastructure has not been set up yet, create it on-demand so other methods remain untouched.
- **Returns:** `tuple[list[float], float] | tuple[None, None]`
- **Dependencies / side effects:**
  - Important helper/API calls: `Buffer`, `TransformListener`, `lookup_transform`, `Time`.
  - Side effects: uses TF lookup/broadcasting, contains timed waits.

#### `robot_perception.acquire_target_transform(self, target_frame, *, max_wait=25.0, trans_thresh=0.001, rot_thresh=2.5, num_samples=10, warmup_sec=0.3)`

- **Location:** `manipulate_node_v4.py:193`
- **Purpose:** Obtain one *stable* transform for `target_frame`. Samples until `num_samples` unique poses are collected; if the max spread across the window is within thresholds, return the averaged pose. Otherwise clear and retry until `max_wait` elapses.
- **Returns:** `list[float] | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `Buffer`, `TransformListener`, `inv`, `from_quat`, `magnitude`, `spread`, `eigh`, `tolist`, `concatenate`, `monotonic`, `get_tf`, `max_spread`, `average_pose`.
  - Key constants/config used: `SAMPLE_DELAY`, `Q`, `M`.
  - Side effects: uses TF lookup/broadcasting, contains timed waits.

### Class `robot_motion`

ROS 2 node class used by the motion stack.

#### Lifecycle, health, and logging

##### `robot_motion.__init__(self)`

- **Location:** `manipulate_node_v4.py:330`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `create_client`, `MoveIt2`, `Buffer`, `TransformListener`.
  - Key constants/config used: `REFERENCE_FRAME`, `PLANNER_ID`, `CARTESIAN`, `CARTESIAN_MAX_STEP`, `CARTESIAN_FRACTION_THRESHOLD`, `CARTESIAN_JUMP_THRESHOLD`, `CARTESIAN_AVOID_COLLISIONS`, `VELOCITY_SCALING`, `ACCELERATION_SCALING`, `SYNCHRONOUS`, `END_EFFECTOR_NAME`, `GROUP_NAME`.
  - Side effects: uses TF lookup/broadcasting, uses MoveIt2 planning/execution, uses executor/thread cleanup.

##### `robot_motion.health_check(self)`

- **Location:** `manipulate_node_v4.py:425`
- **Purpose:** Perform a quick health check of critical robot services. Returns True if robot appears to be responsive, False otherwise.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

##### `robot_motion.is_robot_likely_responsive(self)`

- **Location:** `manipulate_node_v4.py:445`
- **Purpose:** Check if the robot is likely responsive based on recent successful operations. This can be used to bypass intensive health checks when robot appears to be working.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

##### `robot_motion.log_robot_status(self)`

- **Location:** `manipulate_node_v4.py:465`
- **Purpose:** Log current robot status for diagnostic purposes.
- **Returns:** `None`
- **Dependencies / side effects:**
  - Side effects: uses executor/thread cleanup.

##### `robot_motion.reset_robot_connection(self)`

- **Location:** `manipulate_node_v4.py:497`
- **Purpose:** Attempt to reset robot connection when services become unresponsive.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `create_client`.
  - Side effects: contains timed waits.

##### `robot_motion.safe_log(self, level, message)`

- **Location:** `manipulate_node_v4.py:537`
- **Purpose:** Safely log a message, handling ROS context issues
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

##### `robot_motion.destroy_node(self)`

- **Location:** `manipulate_node_v4.py:560`
- **Purpose:** Override destroy_node to mark node as invalid
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `destroy_node`.

#### Perception-assisted target acquisition

##### `robot_motion.get_machine_position(self, target_tf, required_samples=10, *, acq_timeout=10.0, debug=False)`

- **Location:** `manipulate_node_v4.py:606`
- **Purpose:** Acquire a stable averaged pose for `target_tf` using a single perception node, write the result to machine_pose_data_memory.yaml, and return both the YAML entry and a datalog dictionary. Returns *None* on any failure.
- **Returns:** `tuple[dict, dict] | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_perception`, `acquire_target_transform`, `suppress`, `destroy_node`, `get_package_share_directory`, `safe_load`, `safe_dump`.
  - YAML/config files referenced: `machine_pose_data_memory.yaml`.
  - Side effects: reads YAML/config data, writes YAML/config data, uses executor/thread cleanup.

##### `robot_motion.move_to(self, target_tf, distance, speed=100, acceleration=100, offset_x_mm=0.0, offset_y_mm=0.0, offset_z_mm=0.0, offset_rx_deg=0.0, offset_ry_deg=0.0, offset_rz_deg=0.0)`

- **Location:** `manipulate_node_v4.py:977`
- **Purpose:** Approach *target_tf* along its +Z axis, stop `distance` m short, apply XYZ offsets (mm) and RPY offsets (deg), orient the tool so +Z points at the object, and execute a single MovL. Returns True on success.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_perception`, `acquire_target_transform`, `suppress`, `destroy_node`, `abs`, `map`, `euler_from_matrix`, `Request`, `astype`, `sync`.
  - Key constants/config used: `R`.
  - Side effects: contains timed waits.

#### Dobot services and state access

##### `robot_motion.release_tension(self, settling_time=0.5)`

- **Location:** `manipulate_node_v4.py:710`
- **Purpose:** • Enable drag-mode (StartDrag) → let the arm "relax". • Wait `settling_time` s to dissipate any spring-back. • Disable drag-mode (StopDrag). Retries driver-error responses up to `max_attempts`, but aborts on transport time-outs.  Returns **True** on full success. Ensures robot is NOT in drag mode before starting by calling StopDrag first.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.

##### `robot_motion.inverse_solution(self, x, y, z, rx, ry, rz, user=0, tool=0)`

- **Location:** `manipulate_node_v4.py:778`
- **Purpose:** Calculate the inverse solution for the given pose.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.positive_solution(self, j1, j2, j3, j4, j5, j6)`

- **Location:** `manipulate_node_v4.py:786`
- **Purpose:** Calculate the positive solution for the given joint angles.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.sync(self, raise_on_failure=False, max_retries=20)`

- **Location:** `manipulate_node_v4.py:794`
- **Purpose:** Wait for the Dobot motion to complete by calling the /dobot_bringup_v3/srv/Sync service.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `SyncFailureError`, `Request`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - Parameters ---------- raise_on_failure : bool     If True, raises SyncFailureError instead of returning False max_retries : int     Maximum number of retry attempts (default: 3, reduced from 20 for faster failure)
  - Returns ------- True if the service returns res == 0 (motion done), False otherwise.
  - Raises ------ SyncFailureError     If raise_on_failure=True and sync fails after all attempts

##### `robot_motion.set_gripper_position(self, speed=255, position=255, force=255, wait_finish=True)`

- **Location:** `manipulate_node_v4.py:892`
- **Purpose:** Command the gripper and, if `wait_finish`, block until two identical GetGripperPosition readings are observed.
- **Returns:** `tuple[bool, int | None]`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `monotonic`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - Returns (True, final_position) on success; (False, None) otherwise.

##### `robot_motion.set_DO(self, index, status)`

- **Location:** `manipulate_node_v4.py:2597`
- **Purpose:** Execute a digital output on the Dobot via the DOExecute service.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `sync`.

##### `robot_motion.toggle_drag_mode(self, max_attempts=5, retry_pause=0.25)`

- **Location:** `manipulate_node_v4.py:2641`
- **Purpose:** Alternate StartDrag and StopDrag until one succeeds (res == 0). Returns True on success, False if both calls fail after max_attempts each.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.

##### `robot_motion.set_speed_factor(self, ratio)`

- **Location:** `manipulate_node_v4.py:2778`
- **Purpose:** Set the robot speed factor using the SpeedFactor service.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - Parameters: • ratio: scaling factor (0–100)
  - Returns True on success (res==0), False on timeout or driver error.

##### `robot_motion.use_tool(self, index, timeout=5.0)`

- **Location:** `manipulate_node_v4.py:2843`
- **Purpose:** Call the /dobot_bringup_v3/srv/Tool service to select a tool. Returns True on success (res == 0), False otherwise.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.set_tool(self, index, table, timeout=5.0)`

- **Location:** `manipulate_node_v4.py:2876`
- **Purpose:** Call the /dobot_bringup_v3/srv/SetTool service to configure a tool. table should be a string like "{0,0,177.5,0,0,0}". Returns True on success (res == 0), False otherwise.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.current_angles(self)`

- **Location:** `manipulate_node_v4.py:3066`
- **Purpose:** Read the current joint angles immediately and return them in degrees.
- **Returns:** `tuple[float, ...] | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `wait_for_joint_state`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - This function is designed to be used with run_skill to save/restore robot positions:
  - Example usage:     # Save current position     saved_angles = run_skill("current_angles")
  - # Do some movements...     run_skill("moveJ_deg", 10, 0, 0, 0, 0, 0)
  - # Return to saved position     if saved_angles:         run_skill("gotoJ_deg", *saved_angles)

##### `robot_motion.current_pose(self)`

- **Location:** `manipulate_node_v4.py:3117`
- **Purpose:** Read the current Cartesian pose immediately and return it.
- **Returns:** `tuple[float, ...] | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - This function is designed to be used with run_skill to save/restore robot positions:
  - Example usage:     # Save current pose     saved_pose = run_skill("current_pose")
  - # Do some movements...     run_skill("moveEE", 10, 0, 0, 0, 0, 0)
  - # Return to saved pose     if saved_pose:         run_skill("gotoEE", *saved_pose)

#### General movement primitives

##### `robot_motion.moveEE(self, offset_x_mm, offset_y_mm, offset_z_mm, offset_rx_deg, offset_ry_deg, offset_rz_deg)`

- **Location:** `manipulate_node_v4.py:2019`
- **Purpose:** Execute a relative Cartesian move by calling the RelMovL service instead of fetching current pose and adding offsets manually. Accepts both int and float for offsets.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `sync`, `Request`.

##### `robot_motion.moveJ_deg(self, offset1, offset2, offset3, offset4, offset5, offset6)`

- **Location:** `manipulate_node_v4.py:2080`
- **Purpose:** Execute a relative joint move by calling the RelMovJ service instead of fetching current joint angles and adding offsets manually.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.gotoEE(self, abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg, speed=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:2126`
- **Purpose:** Move to absolute Cartesian position using MovL. Returns True on success, False on failure.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.

##### `robot_motion.gotoJ_deg(self, angle1, angle2, angle3, angle4, angle5, angle6, velocity=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:2173`
- **Purpose:** Command an *absolute* set of six joint angles (deg) via JointMovJ. Retries on driver‑error; aborts on transport failure.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `verify_joint_positions`.
  - Side effects: contains timed waits.

##### `robot_motion.move_arc(self, count, offset1, offset2, param_value=None)`

- **Location:** `manipulate_node_v4.py:2693`
- **Purpose:** Execute an arc motion by specifying two offset poses relative to the current pose via Arc service.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `sync`, `Request`.
- **Additional notes from docstring:**
  - Parameters: • count: number of arc segments • offset1: (dx1, dy1, dz1, drx1, dry1, drz1) offsets in mm and deg from current pose for first point • offset2: (dx2, dy2, dz2, drx2, dry2, drz2) offsets in mm and deg from current pose for second point
  - Returns True on success (res==0), False on timeout or driver error.

##### `robot_motion.move_circle(self, count, offset1, offset2, param_value=None)`

- **Location:** `manipulate_node_v4.py:2912`
- **Purpose:** Execute a circular motion by specifying two offset poses relative to current pose via Circle3 service.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `sync`, `Request`.
- **Additional notes from docstring:**
  - Parameters: • count: number of circle segments • offset1: (dx1, dy1, dz1, drx1, dry1, drz1) offsets in mm and deg from current pose for first point • offset2: (dx2, dy2, dz2, drx2, dry2, drz2) offsets in mm and deg from current pose for second point • param_value: list of Speed/AccL strings, e.g. ["SpeedL=100,AccL=100"]
  - Returns True on success, False on timeout or driver error.

##### `robot_motion.moveEE_movJ(self, offset_x_mm, offset_y_mm, offset_z_mm, offset_rx_deg, offset_ry_deg, offset_rz_deg, speed=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:3198`
- **Purpose:** Execute a relative Cartesian move by getting current pose, adding offsets, and using MovJ service for motion execution (joint space planning). Accepts both int and float for offsets.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `sync`, `Request`.
  - Side effects: contains timed waits.

##### `robot_motion.gotoEE_movJ(self, abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg, speed=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:3799`
- **Purpose:** Drive Link-6 to an absolute Cartesian pose (mm/deg) using MovJ.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `sync`.
  - Side effects: contains timed waits.

#### Taught tool/machine offset movement

##### `robot_motion.approach_tool(self, target_tf, speed=100, acceleration=100, *, offset_x_mm=0.0, offset_y_mm=0.0, offset_z_mm=-128)`

- **Location:** `manipulate_node_v4.py:2263`
- **Purpose:** Go to the YAML-defined "approach_pose" of `target_tf`, then apply local offsets along the tool frame.  Returns True on success.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_perception`, `acquire_target_transform`, `suppress`, `destroy_node`, `get_package_share_directory`, `safe_load`, `quaternion_matrix`, `map`, `euler_from_matrix`, `Request`, `sync`.
  - Key constants/config used: `M_obj`, `M_off`, `M_goal`.
  - YAML/config files referenced: `tool_offset_points.yaml`.
  - Side effects: reads YAML/config data, uses executor/thread cleanup.

##### `robot_motion.approach_machine(self, machine_name, point_name, *, speed=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:2351`
- **Purpose:** Drive to <machine>/<point>.approach_pose (world frame) via MovL.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `get_package_share_directory`, `safe_load`, `quaternion_matrix`, `map`, `euler_from_matrix`, `Request`, `sync`.
  - Key constants/config used: `M_base`, `M_off`, `M_goal`.
  - YAML/config files referenced: `machine_pose_data_memory.yaml`, `machine_offset_points.yaml`.
  - Side effects: reads YAML/config data, uses executor/thread cleanup.

##### `robot_motion.mount_machine(self, machine_name, point_name, *, speed=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:2420`
- **Purpose:** Drive to <machine>/<point>.mount_pose via MovL.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `get_package_share_directory`, `safe_load`, `quaternion_matrix`, `map`, `euler_from_matrix`, `Request`, `sync`.
  - Key constants/config used: `M_base`, `M_off`, `M_goal`.
  - YAML/config files referenced: `machine_pose_data_memory.yaml`, `machine_offset_points.yaml`.
  - Side effects: reads YAML/config data, uses executor/thread cleanup.

##### `robot_motion.grab_tool(self, target_tf, speed=100, acceleration=100, offset_x_mm=0.0, offset_y_mm=0.0, offset_z_mm=-135.0)`

- **Location:** `manipulate_node_v4.py:2489`
- **Purpose:** Drive Link6 to the pre-configured "grab" pose for *target_tf* and close in with optional local offsets.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_perception`, `acquire_target_transform`, `suppress`, `destroy_node`, `get_package_share_directory`, `safe_load`, `quaternion_matrix`, `quaternion_from_matrix`, `map`, `euler_from_quaternion`, `Request`, `sync`.
  - Key constants/config used: `M_obj`, `M_off`, `M_goal`, `R`.
  - YAML/config files referenced: `tool_offset_points.yaml`.
  - Side effects: reads YAML/config data, uses executor/thread cleanup.
- **Additional notes from docstring:**
  - Mirrors approach_tool structure: single wait-for-service, retries confined to call_async.

#### Portafilter orientation and arc helpers

##### `robot_motion.enforce_rxry(self)`

- **Location:** `manipulate_node_v4.py:1077`
- **Purpose:** Override Link6's Rx→90°, Ry→0° (keep current Rz) while freezing the world‐space position of portafilter_link to ±0.5 mm. Uses the /dobot_bringup_v3/srv/MovJ service (Link6 planning) instead of MoveIt2. Retries any failed service call every 0.25 s up to 10 attempts; returns False on total failure. Returns True on success, False on any failure.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `now`, `get_clock`, `euler_matrix`, `tolist`, `create_client`.
  - Key constants/config used: `R6_curr`, `M_goal`, `R6_goal`.
  - Side effects: contains timed waits.

##### `robot_motion.enforce_rxry_angled(self)`

- **Location:** `manipulate_node_v4.py:1207`
- **Purpose:** Keep the attached tool origin fixed in world space while forcing Link6 to a known-good grasp orientation.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `now`, `get_clock`, `euler_matrix`, `abs`, `minimum`, `all`, `create_client`.
  - Key constants/config used: `R6_curr`, `R6_goal`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - Tool definition:     0.0, 0.0, 275.0, -17.5, 0.0, 0.0
  - Known-good pose orientation:     Rx = 74.452705     Ry = 0.328027     Rz = 84.781235
  - Uses MovJ to move the flange while compensating XYZ so the tool origin stays in the same world position.
  - Tolerances:     Position: 1 mm     Rotation: 0.1 deg

##### `robot_motion._get_link6_pose_with_retries(self, max_attempts=3)`

- **Location:** `manipulate_node_v4.py:1417`
- **Purpose:** Get current Link6 pose with retry logic.
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.

##### `robot_motion._compute_portafilter_world_position(self, p_link6, rads, d_rel)`

- **Location:** `manipulate_node_v4.py:1452`
- **Purpose:** Compute world position of portafilter_link given Link6 position and orientation.
- **Returns:** `np.ndarray`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`.
  - Key constants/config used: `R6_curr`.

##### `robot_motion._compute_link6_goal_pose(self, rz_curr, p_pf_world, d_rel)`

- **Location:** `manipulate_node_v4.py:1461`
- **Purpose:** Compute goal pose for Link6 to achieve desired orientation while keeping portafilter position fixed.
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`, `quaternion_from_matrix`, `tolist`.
  - Key constants/config used: `M_goal`, `R6_goal`.

##### `robot_motion._execute_enforce_motion(self, goal_pose)`

- **Location:** `manipulate_node_v4.py:1482`
- **Purpose:** Execute the enforce motion using MoveIt2.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `MoveIt2`, `move_to_pose`, `wait_until_executed`, `query_state`.
  - Key constants/config used: `GROUP_NAME`.
  - Side effects: uses MoveIt2 planning/execution, contains timed waits.

##### `robot_motion._verify_portafilter_position(self, p_pf_world, d_rel, tolerance, max_attempts=10)`

- **Location:** `manipulate_node_v4.py:1523`
- **Purpose:** Verify that portafilter_link stayed within tolerance.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`.
  - Key constants/config used: `R6_new`.
  - Side effects: contains timed waits.

##### `robot_motion._get_link6_pose_with_retries_v1(self, max_attempts=3)`

- **Location:** `manipulate_node_v4.py:1561`
- **Purpose:** Get current Link6 pose with retry logic (v1 implementation).
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`.
  - Side effects: contains timed waits.

##### `robot_motion._compute_portafilter_world_position_v1(self, p_link6, rads, d_rel)`

- **Location:** `manipulate_node_v4.py:1596`
- **Purpose:** Compute world position of portafilter_link given Link6 position and orientation (v1 implementation).
- **Returns:** `np.ndarray`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`.
  - Key constants/config used: `R6_curr`.

##### `robot_motion._compute_link6_goal_pose_v1(self, rz_curr, p_pf_world, d_rel)`

- **Location:** `manipulate_node_v4.py:1605`
- **Purpose:** Compute goal pose for Link6 to achieve desired orientation while keeping portafilter position fixed (v1 implementation).
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`, `quaternion_from_matrix`, `tolist`.
  - Key constants/config used: `M_goal`, `R6_goal`.

##### `robot_motion._execute_enforce_motion_v1(self, goal_pose)`

- **Location:** `manipulate_node_v4.py:1626`
- **Purpose:** Execute the enforce motion using MoveIt2 (v1 implementation).
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `MoveIt2`, `move_to_pose`, `wait_until_executed`, `query_state`.
  - Key constants/config used: `GROUP_NAME`.
  - Side effects: uses MoveIt2 planning/execution, contains timed waits.

##### `robot_motion._verify_portafilter_position_v1(self, p_pf_world, d_rel, tolerance, max_attempts=10)`

- **Location:** `manipulate_node_v4.py:1667`
- **Purpose:** Verify that portafilter_link stayed within tolerance (v1 implementation).
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `euler_matrix`.
  - Key constants/config used: `R6_new`.
  - Side effects: contains timed waits.

##### `robot_motion.move_portafilter_arc(self, angle_deg)`

- **Location:** `manipulate_node_v4.py:1701`
- **Purpose:** Rotate the portafilter_link by the specified angle about its local Y axis.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `abs`.

##### `robot_motion._get_portafilter_current_pose_v1(self)`

- **Location:** `manipulate_node_v4.py:1747`
- **Purpose:** Get current portafilter pose from TF (v1 implementation).
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `lookup_transform`, `Time`, `Duration`, `get_transform_list`.
  - Side effects: uses TF lookup/broadcasting.

##### `robot_motion._compute_arc_pose_v1(self, current_pose, angle_deg)`

- **Location:** `manipulate_node_v4.py:1767`
- **Purpose:** Compute new pose after rotating about local Y axis (v1 implementation).
- **Returns:** `tuple | None`
- **Dependencies / side effects:**
  - Important helper/API calls: `quaternion_matrix`, `quaternion_about_axis`, `quaternion_multiply`, `tolist`.
  - Key constants/config used: `R_current`.

##### `robot_motion._verify_arc_completion_v1(self, tolerance_deg=1.0, batch_size=10, max_attempts=5)`

- **Location:** `manipulate_node_v4.py:1833`
- **Purpose:** Verify that the arc motion has completed with stable joint positions (v1 implementation).
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `wait_for_joint_state`, `all`, `verify_joint_positions`.
  - Side effects: contains timed waits.

##### `robot_motion.move_portafilter_arc_movL(self, angle_deg, velocity=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:1879`
- **Purpose:** Move the portafilter_link in an arc by angle_deg about its local Y axis, using the /dobot_bringup_v3/srv/MovL service (Link6 linear planning). Allows specifying SpeedL and AccL via the velocity and acceleration arguments. Retries any failed service call every 0.25 s up to 10 attempts; returns False on total failure. Returns True on success, False on any failure.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `now`, `get_clock`, `sync`, `euler_matrix`, `quaternion_about_axis`, `quaternion_from_matrix`, `quaternion_multiply`, `quaternion_matrix`, `tolist`, `euler_from_matrix`.
  - Key constants/config used: `R6_curr`, `R6_goal`, `R_goal_full`.

##### `robot_motion.move_portafilter_arc_movJ(self, angle_deg, d_rel_z=287.5, velocity=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:3313`
- **Purpose:** Rotate the portafilter_link about its local Y axis by `angle_deg` while keeping its pivot (d_rel_z ahead of Link-6) fixed. • Reads /GetPose ONCE, then iteratively computes each ≤ 5 ° goal. • Queues one /MovJ per chunk, no retries. • Returns True only if every chunk's /MovJ succeeds.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `abs`, `Request`, `map`, `euler_matrix`, `from_rotvec`, `apply`, `as_matrix`, `eye`, `euler_from_matrix`, `sync`.
  - Key constants/config used: `R6`, `R6_goal`, `M_goal`.

##### `robot_motion.move_portafilter_arc_movJ_angled(self, angle_deg, d_rel_z=287.5, tcp_rx_deg=-17.5, tcp_ry_deg=0.0, tcp_rz_deg=0.0, velocity=100, acceleration=100)`

- **Location:** `manipulate_node_v4.py:3414`
- **Purpose:** Rotate the ANGLED portafilter about its own local Y axis by `angle_deg` while keeping its TCP pivot fixed.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `abs`, `Request`, `map`, `euler_matrix`, `from_rotvec`, `apply`, `as_matrix`, `eye`, `euler_from_matrix`, `sync`.
  - Key constants/config used: `R6`, `R_tcp`, `R_tool`, `R_tool_goal`, `R6_goal`, `M_goal`.
- **Additional notes from docstring:**
  - This treats the angled tool as having TCP:     (0, 0, d_rel_z, tcp_rx_deg, tcp_ry_deg, tcp_rz_deg)
  - So compared to the normal function, the TCP orientation is built into: - pivot computation - rotation axis selection - flange goal orientation solving
  - Returns True only if every MovJ succeeds.

##### `robot_motion.move_portafilter_arc_tool(self, arc_size_deg=45.0, axis='z', tcp_table='{0,0,287.5,0,0,0}')`

- **Location:** `manipulate_node_v4.py:3555`
- **Purpose:** 1) Configure TCP via SetTool (tool index 1, tcp_table) 2) Pivot the portafilter_link by arc_size_deg around the given axis    via RelMovL (relative linear move) using Tool=1 3) Reset TCP to default (tool 0, zero table) Returns True on success, False otherwise.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `use_tool`.
  - Side effects: contains timed waits.

##### `robot_motion.move_portafilter_arc_tool_angled(self, arc_size_deg=30.0, axis='z', tcp_table='{0,0,275.0,-17.5,0,0}')`

- **Location:** `manipulate_node_v4.py:3637`
- **Purpose:** 1) Configure TCP via SetTool (tool index 1, tcp_table) 2) Pivot the portafilter_link by arc_size_deg around the given axis    via RelMovL (relative linear move) using Tool=1 3) Reset TCP to default (tool 0, zero table) Returns True on success, False otherwise.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `Request`, `use_tool`.
  - Side effects: contains timed waits.

##### `robot_motion.enforce_rxry_moveit(self, d_rel_z=0.2825)`

- **Location:** `manipulate_node_v4.py:3719`
- **Purpose:** Freeze the portafilter_link origin (±0.5 mm) while forcing Link-6 to Rx = 90 °, Ry = 0 ° (retain the current Rz).
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `wait_for_servo_ready`, `Request`, `map`, `euler_matrix`, `quaternion_from_matrix`, `MoveIt2`, `move_to_pose`, `tolist`, `wait_until_executed`, `query_state`.
  - Key constants/config used: `R6_now`, `R6_goal`.
  - Side effects: uses MoveIt2 planning/execution, contains timed waits.

#### Servo/feedback waiting utilities

##### `robot_motion.wait_for_joint_state(self, topic_name, timeout_sec=2.0)`

- **Location:** `manipulate_node_v4.py:3005`
- **Purpose:** Synchronously waits for a JointState message from the given topic.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `create_subscription`, `now`, `get_clock`, `destroy_subscription`.

##### `robot_motion.wait_for_servo_ready(self, timeout=15.0)`

- **Location:** `manipulate_node_v4.py:3026`
- **Purpose:** Poll /get_servo_status (Trigger) until status == 0 (READY) or timeout.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `create_client`, `monotonic`, `Request`.
  - Side effects: contains timed waits.

##### `robot_motion._wait_for_servo_ready_with_timeout(self, timeout=15.0)`

- **Location:** `manipulate_node_v4.py:3052`
- **Purpose:** Wait for servo to be ready with timeout handling.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `monotonic`, `wait_for_servo_ready`.
  - Side effects: contains timed waits.

#### Other robot motion helpers

##### `robot_motion.verify_joint_positions(self, expected_joints, tolerance_deg=1.0)`

- **Location:** `manipulate_node_v4.py:568`
- **Purpose:** Verify that the current joint positions match the expected values within tolerance.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `current_angles`, `abs`.

##### `robot_motion._execute_arc_motion_v1(self, new_pose, angle_deg)`

- **Location:** `manipulate_node_v4.py:1790`
- **Purpose:** Execute the arc motion using MoveIt2 (v1 implementation).
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `MoveIt2`, `move_to_pose`, `wait_until_executed`, `query_state`.
  - Side effects: uses MoveIt2 planning/execution, contains timed waits.

## `testing_v1.py`

This file is the high-level behavior layer. It loads a motion-node implementation, exposes the persistent `run_skill` bridge, then defines the actual coffee automation workflows: home/calibration, cup handling, espresso, angled espresso, cleaning, milk frothing, plastic cups/ice, slush, service calls, training routines, and tests.

### Runtime, version selection, and persistent motion-node helpers

#### `_get_local_run_skill()`

- **Location:** `testing_v1.py:35`
- **Purpose:** Get run_skill function from the selected manipulate_node version
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `dirname`, `abspath`, `spec_from_file_location`, `ImportError`, `module_from_spec`, `exec_module`.
  - Key constants/config used: `USE_VERSION`.
  - Side effects: uses executor/thread cleanup.

#### `get_motion_node()`

- **Location:** `testing_v1.py:105`
- **Purpose:** Get or create the global motion node instance
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `init`, `robot_motion_class`.
  - Key constants/config used: `USE_VERSION`.

#### `run_skill(fn_name, *args)`

- **Location:** `testing_v1.py:125`
- **Purpose:** Efficient run_skill that reuses a persistent motion node. Falls back to old approach if there are issues.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_motion_node`, `fn`, `cleanup_motion_node`, `run_skill_old`.
  - Key constants/config used: `_DATA_RETURN_SKILLS`.
- **Additional notes from docstring:**
  - Skills like set_gripper_position return a tuple (success, payload). The raw tuple is truthy even when success is False, which used to bypass every `if not ok(run_skill(...))` check downstream and let sequences keep marching past a failed gripper command. Unwrap such returns to a bool here so a single source of truth handles failure detection, and keep data-returning skills (current_angles, current_pose, get_machine_position) returning their raw payload.

#### `cleanup_motion_node()`

- **Location:** `testing_v1.py:166`
- **Purpose:** Clean up the global motion node with proper thread management
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `is_alive`, `destroy_node`, `shutdown`.
  - Side effects: uses executor/thread cleanup.

#### `show_version_info()`

- **Location:** `testing_v1.py:202`
- **Purpose:** Display current version and how to switch
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Key constants/config used: `USE_VERSION`.

#### `switch_version()`

- **Location:** `testing_v1.py:231`
- **Purpose:** Interactive version switcher
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Key constants/config used: `USE_VERSION`.
  - Side effects: requires operator input.

### Home, calibration, kinematics, gripper, and robot reset helpers

#### `_calibrate_marker(marker_name, prep_fn, ok)`

- **Location:** `testing_v1.py:288`
- **Purpose:** Retry a single marker calibration up to MAX_CALIBRATION_RETRIES times.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `get_machine_position`.
  - Important helper/API calls: `prep_fn`, `run_skill`.
  - Key constants/config used: `MAX_CALIBRATION_RETRIES`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - prep_fn must move the arm into the correct approach pose and call sync. Returns True on the first successful get_machine_position, False if all attempts are exhausted.

#### `home(**params)`

- **Location:** `testing_v1.py:308`
- **Purpose:** Move robot to a predefined home position.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `HOME_ANGLES`.

#### `return_back_to_home()`

- **Location:** `testing_v1.py:328`
- **Purpose:** Return the robot to a safe home position based on current angle.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `release_tension`, `toggle_drag_mode`, `set_speed_factor`, `sync`, `set_gripper_position`, `current_angles`, `gotoJ_deg`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `HOME_CALIBRATION_PARAMS`.

#### `get_machine_position(**params)`

- **Location:** `testing_v1.py:395`
- **Purpose:** Calibrate and record machine positions for all coffee equipment.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `move_to`, `sync`, `moveJ_deg`.
  - Important helper/API calls: `invalidate_port_cache`, `invalidate_cleaning_cache`, `angled_invalidate_cleaning_cache`, `angled_invalidate_port_cache`, `run_skill`, `return_back_to_home`, `check_saved_data`.
  - Key constants/config used: `SPEED_FAST`, `HOME_CALIBRATION_PARAMS`, `HOME_CALIBRATION_CONSTANTS`, `ESPRESSO_HOME`.
  - Side effects: contains timed waits.

#### `check_saved_data()`

- **Location:** `testing_v1.py:470`
- **Purpose:** Check and display currently saved machine position data.
- **Returns:** `Dict[str, Any]`
- **Dependencies / side effects:**
  - Important helper/API calls: `get_package_share_directory`, `exists`, `safe_load`.
  - YAML/config files referenced: `machine_pose_data_memory.yaml`.
  - Side effects: reads YAML/config data, uses executor/thread cleanup.

#### `check_aruco_status(**params)`

- **Location:** `testing_v1.py:493`
- **Purpose:** Check current ArUco marker detection status and help diagnose calibration issues.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `check_saved_data`.

#### `solution(j1, j2, j3, j4, j5, j6, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)`

- **Location:** `testing_v1.py:500`
- **Purpose:** Convert joint values to cartesian, apply offsets, and convert back to joints.
- **Returns:** May return data or `None` on failure.
- **Dependencies / side effects:**
  - Robot skills used: `positive_solution`, `inverse_solution`.
  - Important helper/API calls: `run_skill`.

#### `solution_interactive(*params)`

- **Location:** `testing_v1.py:547`
- **Purpose:** Interactive wrapper for solution function that prompts for input.
- **Returns:** May return data or `None` on failure.
- **Dependencies / side effects:**
  - Important helper/API calls: `solution`.
  - Side effects: requires operator input.

#### `open_gripper(**params)`

- **Location:** `testing_v1.py:581`
- **Purpose:** Synchronizes motion, then opens the gripper fully using the configured gripper command.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`.

#### `close_gripper(**params)`

- **Location:** `testing_v1.py:586`
- **Purpose:** Synchronizes motion, then closes the gripper fully using the configured gripper command.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`.

#### `toggle_drag_mode(**params)`

- **Location:** `testing_v1.py:591`
- **Purpose:** Calls the low-level drag-mode toggle skill. Used when the operator needs the robot to enter/exit manual drag mode.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `toggle_drag_mode`.
  - Important helper/API calls: `run_skill`.

#### `_run_kubectl_rollout_restart_on_nuc(deployment)`

- **Location:** `testing_v1.py:599`
- **Purpose:** Run kubectl rollout restart on the NUC via SSH. Returns True on success.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Key constants/config used: `_RESET_SSH_PASS`, `_RESET_SSH_USER`, `_RESET_SSH_HOST`.
  - Side effects: runs an external subprocess/SSH command.

#### `reset_robot1(**params)`

- **Location:** `testing_v1.py:631`
- **Purpose:** Restart robot1 deployment via kubectl on NUC (qss@192.168.200.254).
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `reset_robot2(**params)`

- **Location:** `testing_v1.py:635`
- **Purpose:** Restart robot2 deployment via kubectl on NUC (qss@192.168.200.254).
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

### Paper cup handling

#### `_normalize_paper_cup_size(cups_dict)`

- **Location:** `testing_v1.py:661`
- **Purpose:** Universal cup size normalizer for paper cup operations. Accepts BOTH H-codes AND C-codes regardless of prefix. Extracts the numeric size and returns standardized format.
- **Returns:** `str`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `grab_paper_cup(**params)`

- **Location:** `testing_v1.py:722`
- **Purpose:** Grab a paper cup of specified size from the paper cup dispenser.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE`, `set_gripper_position`, `sync`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `GRAB_PAPER_CUP_PARAMS`, `ESPRESSO_HOME`, `PAPER_CUPS_NAVIGATION_PARAMS`, `GRIPPER_FULL`, `GRIPPER_OPEN`.

#### `place_paper_cup(**params)`

- **Location:** `testing_v1.py:790`
- **Purpose:** Place a paper cup at the specified staging area.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveJ_deg`, `sync`, `set_gripper_position`, `moveEE`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `PLACE_PAPER_CUP_PARAMS`, `PAPER_CUPS_NAVIGATION_PARAMS`, `PAPER_CUP_MOVEMENT_OFFSETS`.

#### `grab_paper_cup_arm1(**params)`

- **Location:** `testing_v1.py:836`
- **Purpose:** Grab a paper cup of specified size from the paper cup dispenser.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE_movJ`.
  - Important helper/API calls: `home`, `run_skill`, `detect_cup_gripper`.
- **Additional notes from docstring:**
  - First attempt does the full size-specific approach. If detection fails, retries only:   1) open gripper   2) move back up   3) close gripper with size-specific width   4) move back down

#### `place_paper_cup_arm1(**params)`

- **Location:** `testing_v1.py:922`
- **Purpose:** Place a paper cup at the specified staging area.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE`, `set_speed_factor`.
  - Important helper/API calls: `run_skill`, `home`.
  - Key constants/config used: `PLACE_PAPER_CUP_PARAMS`, `PAPER_CUP_MOVEMENT_OFFSETS`.

#### `dispense_paper_arm1_cup_station(**params)`

- **Location:** `testing_v1.py:992`
- **Purpose:** Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grab_paper_cup_arm1`, `place_paper_cup_arm1`.

#### `grab_paper_arm2_cup_station(**params)`

- **Location:** `testing_v1.py:1002`
- **Purpose:** Grab a paper cup of specified size from the paper cup dispenser.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE_movJ`, `sync`, `set_gripper_position`, `moveEE`.
  - Important helper/API calls: `home`, `run_skill`, `detect_cup_gripper`.
- **Additional notes from docstring:**
  - First attempt does the full size-specific approach. If detection fails, retries only:   1) open gripper   2) move back up   3) close gripper with size-specific width   4) move back down

#### `place_paper_arm2_cup_station(**params)`

- **Location:** `testing_v1.py:1096`
- **Purpose:** Place a paper cup at specified staging area.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE`, `set_speed_factor`.
  - Important helper/API calls: `home`, `run_skill`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`, `PAPER_CUP_MOVEMENT_OFFSETS`.

#### `dispense_paper_arm2_cup_station(**params)`

- **Location:** `testing_v1.py:1153`
- **Purpose:** Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grab_paper_arm2_cup_station`, `place_paper_arm2_cup_station`.

#### `dispense_paper_cup_station(**params)`

- **Location:** `testing_v1.py:1163`
- **Purpose:** Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grab_paper_cup`, `place_paper_cup`.

#### `pick_paper_cup_station(**params)`

- **Location:** `testing_v1.py:1173`
- **Purpose:** Pick up a paper cup from a specific stage.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE`, `set_gripper_position`, `moveEE_movJ`.
  - Important helper/API calls: `home`, `run_skill`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`, `PAPER_CUP_GRIPPER_POSITIONS`, `PAPER_CUP_MOVEMENT_OFFSETS`, `GRIPPER_FULL`.

#### `place_paper_cup_station(**params)`

- **Location:** `testing_v1.py:1229`
- **Purpose:** Place a paper cup at specified staging area.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE`.
  - Important helper/API calls: `home`, `run_skill`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`, `GRIPPER_RELEASE`, `GRIPPER_OPEN`, `PAPER_CUP_MOVEMENT_OFFSETS`.

#### `place_paper_cup_sauces(**params)`

- **Location:** `testing_v1.py:1268`
- **Purpose:** Place the paper cup at the sauces station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`.

#### `pick_paper_cup_sauces(**params)`

- **Location:** `testing_v1.py:1289`
- **Purpose:** Pick the paper cup from the sauces station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `moveEE`, `set_gripper_position`, `gotoJ_deg`.
  - Important helper/API calls: `detect_cup_gripper`, `run_skill`.
  - Key constants/config used: `GRIPPER_FULL`, `PAPER_CUPS_STATION_PARAMS`.

#### `place_paper_cup_milk(**params)`

- **Location:** `testing_v1.py:1319`
- **Purpose:** Place the paper cup at the milk station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`.

#### `pick_paper_cup_milk(**params)`

- **Location:** `testing_v1.py:1340`
- **Purpose:** Pick the paper cup from the milk station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `moveEE`, `set_gripper_position`, `gotoJ_deg`.
  - Important helper/API calls: `detect_cup_gripper`, `run_skill`.
  - Key constants/config used: `GRIPPER_FULL`, `PAPER_CUPS_STATION_PARAMS`.

#### `pick_cup_for_hot_water(**params)`

- **Location:** `testing_v1.py:1370`
- **Purpose:** Pick up a paper cup from a specific stage for hot water.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE`, `set_gripper_position`, `sync`, `set_speed_factor`, `moveEE_movJ`, `approach_machine`, `mount_machine`.
  - Important helper/API calls: `home`, `run_skill`.
  - Key constants/config used: `PAPER_CUPS_STATION_PARAMS`, `PAPER_CUP_GRIPPER_POSITIONS`, `PAPER_CUP_MOVEMENT_OFFSETS`.

#### `return_cup_with_hot_water(**params)`

- **Location:** `testing_v1.py:1441`
- **Purpose:** Complete hot water dispensing sequence and return to holding position.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `moveEE`, `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `home`.
  - Key constants/config used: `PLACE_PAPER_CUP_PARAMS`, `ESPRESSO_MOVEMENT_OFFSETS`, `GRIPPER_RELEASE`, `GRIPPER_OPEN`, `PAPER_CUP_MOVEMENT_OFFSETS`, `PAPER_CUPS_NAVIGATION_PARAMS`.

### Espresso workflow

#### `_portafilter_clear_up_offset(port)`

- **Location:** `testing_v1.py:1607`
- **Purpose:** Use per-port learned Z from last live unmount, else params default.
- **Returns:** `Tuple[float, float, float, float, float, float]`
- **Dependencies / side effects:**
  - Key constants/config used: `ESPRESSO_MOVEMENT_OFFSETS`.

#### `_portafilter_clear_up_angled_offset(port)`

- **Location:** `testing_v1.py:1615`
- **Purpose:** Angled mount clear-up: learned Z from last live angled unmount, else params default.
- **Returns:** `Tuple[float, float, float, float, float, float]`
- **Dependencies / side effects:**
  - Key constants/config used: `ESPRESSO_MOVEMENT_OFFSETS`.

#### `_angled_unmount_grab_tool_name(port)`

- **Location:** `testing_v1.py:1623`
- **Purpose:** Portafilter tool frame for angled unmount grab (per robot teach).
- **Returns:** `str`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_open_gripper_with_verify(speed=255, force=255)`

- **Location:** `testing_v1.py:1629`
- **Purpose:** Send gripper-open (position 0) and verify it actually reached a low position.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_motion_node`, `set_gripper_position`.
  - Key constants/config used: `_GRIPPER_OPEN_RETRIES`, `_GRIPPER_OPEN_MAX_POS`.
  - Side effects: contains timed waits.
- **Additional notes from docstring:**
  - Retries up to _GRIPPER_OPEN_RETRIES times if the reported position is above _GRIPPER_OPEN_MAX_POS (gripper did not physically open).

#### `invalidate_port_cache()`

- **Location:** `testing_v1.py:1652`
- **Purpose:** Clears all espresso and portafilter runtime caches so the next operation re-learns live poses instead of replaying cached joints.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_is_valid_angles(angles)`

- **Location:** `testing_v1.py:1669`
- **Purpose:** Internal validator that checks whether a joint-angle payload is a six-value tuple/list.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_run_cached_machine_approach(cache_key, machine_name, target_name)`

- **Location:** `testing_v1.py:1672`
- **Purpose:** Replay a cached pose captured immediately after a successful approach_machine(...). If no cache exists yet, run the live approach, sync, capture current_angles, and cache them.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `approach_machine`, `sync`, `current_angles`.
  - Important helper/API calls: `run_skill`.

#### `_run_cached_machine_mount(cache_key, machine_name, target_name)`

- **Location:** `testing_v1.py:1693`
- **Purpose:** Replay a cached pose captured immediately after a successful mount_machine(...). If no cache exists yet, run the live mount, sync, capture current_angles, and cache them.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `mount_machine`, `sync`, `current_angles`.
  - Important helper/API calls: `run_skill`.

#### `_normalize_espresso_shot(espresso_dict)`

- **Location:** `testing_v1.py:1714`
- **Purpose:** Parses an espresso request dictionary and maps it to the correct port, tool, shot timing, and angled/non-angled flow.
- **Returns:** `Optional[Dict[str, Any]]`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `unmount(**params)`

- **Location:** `testing_v1.py:1774`
- **Purpose:** Unmounts a portafilter from the espresso machine, grips it, releases mechanical tension, rotates/clears it safely, and caches the return poses.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `grab_tool`, `moveEE_movJ`, `release_tension`, `enforce_rxry`, `current_pose`, `current_angles`, `move_portafilter_arc_movJ`.
  - Important helper/API calls: `angled_unmount`, `run_skill`, `get_motion_node`, `set_gripper_position`, `ceil`.
  - Key constants/config used: `PULL_ESPRESSO_PARAMS`, `_PORTAFILTER_GRIP_POS_MIN`, `_PORTAFILTER_GRIP_POS_MAX`, `_UNMOUNT_POST_TENSION_Z_TARGET_MM`, `_UNMOUNT_POST_TENSION_Z_TOL_MM`, `ESPRESSO_MOVEMENT_OFFSETS`, `ESPRESSO_GRINDER_PARAMS`.

#### `grinder(**params)`

- **Location:** `testing_v1.py:2018`
- **Purpose:** Moves the mounted portafilter through grinder and tamper station positions, using cached machine approach/mount poses when available.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `current_angles`, `sync`, `set_gripper_position`, `moveEE_movJ`.
  - Important helper/API calls: `angled_grinder`, `run_skill`.
  - Key constants/config used: `ESPRESSO_GRINDER_HOME`.
  - Side effects: contains timed waits.

#### `single_grinder(**params)`

- **Location:** `testing_v1.py:2126`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `grinder` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grinder`.

#### `double_grinder(**params)`

- **Location:** `testing_v1.py:2130`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `grinder` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grinder`.

#### `tamper(**params)`

- **Location:** `testing_v1.py:2134`
- **Purpose:** Moves the portafilter to the tamper station, executes the tamping path, and prepares it for remounting.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `move_to`, `approach_tool`, `grab_tool`, `current_angles`, `set_gripper_position`, `moveEE`.
  - Important helper/API calls: `angled_tamper`, `run_skill`.
  - Key constants/config used: `ESPRESSO_GRINDER_HOME`.

#### `single_tamper(**params)`

- **Location:** `testing_v1.py:2203`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `tamper` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `tamper`.

#### `double_tamper(**params)`

- **Location:** `testing_v1.py:2207`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `tamper` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_tamper`, `tamper`.

#### `mount(**params)`

- **Location:** `testing_v1.py:2214`
- **Purpose:** Mounts the portafilter back into the espresso group using the cached unmount/mount poses and learned arc command.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE_movJ`, `enforce_rxry`, `sync`, `move_portafilter_arc_movJ`, `release_tension`.
  - Important helper/API calls: `angled_mount`, `run_skill`.
  - Key constants/config used: `PULL_ESPRESSO_PARAMS`, `ESPRESSO_GRINDER_PARAMS`.

#### `grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:2298`
- **Purpose:** Grab the espresso pitcher and stop right after closing the gripper.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `current_angles`, `set_gripper_position`.
  - Important helper/API calls: `angled_grab_espresso_pitcher`, `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:2448`
- **Purpose:** Complete pitcher pickup after grab_espresso_pitcher().
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `set_speed_factor`, `gotoJ_deg`, `current_angles`.
  - Important helper/API calls: `angled_pick_espresso_pitcher`, `run_skill`.
  - Key constants/config used: `ESPRESSO_SPEEDS`, `ESPRESSO_PITCHER_PARAMS`.

#### `pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:2535`
- **Purpose:** Moves the pitcher to the selected cup station and executes the pour sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_speed_factor`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_PITCHER_PARAMS`, `SPEED_SLOW_POURING`.

#### `get_hot_water(**params)`

- **Location:** `testing_v1.py:2619`
- **Purpose:** Moves a cup/pitcher to the hot-water point on the espresso machine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_MOVEMENT_OFFSETS`.

#### `with_hot_water(**params)`

- **Location:** `testing_v1.py:2646`
- **Purpose:** Runs the hot-water dispensing portion of the espresso-machine workflow.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_SPEEDS`, `ESPRESSO_MOVEMENT_OFFSETS`.

#### `return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:2657`
- **Purpose:** Returns the espresso pitcher back to its machine/station holder.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `current_angles`, `set_gripper_position`.
  - Important helper/API calls: `angled_return_espresso_pitcher`, `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:2840`
- **Purpose:** Returns the pitcher after cleaning-related handling, using stored pitcher return poses.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE_movJ`, `current_angles`, `moveJ_deg`.
  - Important helper/API calls: `angled_return_cleaned_espresso_pitcher`, `run_skill`, `all`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `unmount_single(**params)`

- **Location:** `testing_v1.py:3026`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `unmount`.

#### `unmount_double(**params)`

- **Location:** `testing_v1.py:3030`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_unmount`, `unmount`.

#### `mount_single(**params)`

- **Location:** `testing_v1.py:3037`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `mount`.

#### `mount_double(**params)`

- **Location:** `testing_v1.py:3041`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_mount`, `mount`.

#### `single_grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3048`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `grab_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `grab_espresso_pitcher`.

#### `double_grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3052`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `grab_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_grab_espresso_pitcher`, `grab_espresso_pitcher`.

#### `single_pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3059`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `pick_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `pick_espresso_pitcher`.

#### `double_pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3063`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `pick_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pick_espresso_pitcher`, `pick_espresso_pitcher`.

#### `single_pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:3070`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `pour_espresso_pitcher_cup_station` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `pour_espresso_pitcher_cup_station`.

#### `double_pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:3074`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `pour_espresso_pitcher_cup_station` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pour_espresso_pitcher_cup_station`, `pour_espresso_pitcher_cup_station`.

#### `single_return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3081`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `return_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `return_espresso_pitcher`.

#### `double_return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3085`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `return_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_espresso_pitcher`, `return_espresso_pitcher`.

#### `single_return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3092`
- **Purpose:** Convenience wrapper that selects the single variant and delegates to `return_cleaned_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `return_cleaned_espresso_pitcher`.

#### `double_return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:3096`
- **Purpose:** Convenience wrapper that selects the double variant and delegates to `return_cleaned_espresso_pitcher` or the shared espresso routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_cleaned_espresso_pitcher`, `return_cleaned_espresso_pitcher`.

### Angled espresso workflow

#### `angled_invalidate_port_cache()`

- **Location:** `testing_v1.py:3128`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled__is_valid_angles(angles)`

- **Location:** `testing_v1.py:3152`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled__normalize_espresso_shot(espresso_dict)`

- **Location:** `testing_v1.py:3155`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `Optional[Dict[str, Any]]`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled_unmount(**params)`

- **Location:** `testing_v1.py:3200`
- **Purpose:** Angled-portafilter variant of unmount with angled-specific tool names, arc commands, and grip checks.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `grab_tool`, `release_tension`, `current_pose`, `moveEE_movJ`, `current_angles`, `move_portafilter_arc_tool_angled`, `moveJ_deg`, `set_speed_factor`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `get_motion_node`, `set_gripper_position`, `trace_grip`, `angled__is_valid_angles`, `ceil`.
  - Key constants/config used: `PULL_ESPRESSO_PARAMS`, `ANGLED_UNMOUNT_GRIP_POS_MIN`, `ANGLED_UNMOUNT_GRIP_POS_MAX`, `ANGLED_UNMOUNT_GRIP_RETRIES`, `_UNMOUNT_POST_TENSION_Z_TARGET_ANGL_MM`, `_UNMOUNT_POST_TENSION_Z_TOL_MM`, `_ANGLED_PORTAFILTER_ARC_TARGET_RZ`, `_ANGLED_PORTAFILTER_MOUNT_ARC_EXTRA`, `ESPRESSO_MOVEMENT_OFFSETS`.

#### `angled_grinder(**params)`

- **Location:** `testing_v1.py:3606`
- **Purpose:** Angled-portafilter variant of the grinder sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `current_angles`, `sync`, `set_gripper_position`, `moveEE_movJ`, `approach_tool`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`.
  - Key constants/config used: `ESPRESSO_GRINDER_HOME`.
  - Side effects: contains timed waits.

#### `angled_single_grinder(**params)`

- **Location:** `testing_v1.py:3698`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_grinder`.

#### `angled_double_grinder(**params)`

- **Location:** `testing_v1.py:3703`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_grinder`.

#### `angled_tamper(**params)`

- **Location:** `testing_v1.py:3708`
- **Purpose:** Angled-portafilter variant of the tamper sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `move_to`, `approach_tool`, `grab_tool`, `set_gripper_position`, `current_angles`, `moveEE`, `release_tension`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`, `get_motion_node`, `set_gripper_position`.
  - Key constants/config used: `ESPRESSO_GRINDER_HOME`, `ANGLED_TAMPER_GRIP_POS_MIN`, `ANGLED_TAMPER_GRIP_POS_MAX`, `ANGLED_TAMPER_GRIP_RETRIES`.

#### `angled_single_tamper(**params)`

- **Location:** `testing_v1.py:3880`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_tamper`.

#### `angled_double_tamper(**params)`

- **Location:** `testing_v1.py:3884`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_tamper`.

#### `angled_mount(**params)`

- **Location:** `testing_v1.py:3889`
- **Purpose:** Angled-portafilter variant of the remount sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_speed_factor`, `move_portafilter_arc_tool_angled`, `release_tension`.
  - Important helper/API calls: `globals`, `trace`, `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`.
  - Key constants/config used: `PULL_ESPRESSO_PARAMS`.

#### `angled_grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4072`
- **Purpose:** Grab the espresso pitcher and stop right after closing the gripper.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `current_angles`, `set_gripper_position`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `angled_pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4197`
- **Purpose:** Complete pitcher pickup after angled_grab_espresso_pitcher().
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `set_speed_factor`, `gotoJ_deg`, `current_angles`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`.
  - Key constants/config used: `ESPRESSO_SPEEDS`, `ESPRESSO_PITCHER_PARAMS`.

#### `angled_pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:4270`
- **Purpose:** Angled-flow variant for pouring espresso into a selected cup station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_speed_factor`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_PITCHER_PARAMS`, `SPEED_SLOW_POURING`.

#### `angled_get_hot_water(**params)`

- **Location:** `testing_v1.py:4353`
- **Purpose:** Angled-flow variant for moving to the hot-water point.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_MOVEMENT_OFFSETS`.

#### `angled_with_hot_water(**params)`

- **Location:** `testing_v1.py:4380`
- **Purpose:** Angled-flow hot-water dispensing routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_SPEEDS`, `ESPRESSO_MOVEMENT_OFFSETS`.

#### `angled_return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4391`
- **Purpose:** Angled-flow variant for returning the espresso pitcher.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `current_angles`, `set_gripper_position`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled__is_valid_angles`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `angled_return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4554`
- **Purpose:** Angled-flow variant for returning the cleaned espresso pitcher.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE_movJ`, `current_angles`, `moveJ_deg`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `all`, `angled__is_valid_angles`.
  - Key constants/config used: `ESPRESSO_HOME`.

#### `angled_unmount_single(**params)`

- **Location:** `testing_v1.py:4739`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_unmount`.

#### `angled_unmount_double(**params)`

- **Location:** `testing_v1.py:4743`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_unmount`.

#### `angled_mount_single(**params)`

- **Location:** `testing_v1.py:4747`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_mount`.

#### `angled_mount_double(**params)`

- **Location:** `testing_v1.py:4751`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_mount`.

#### `angled_single_grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4755`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_grab_espresso_pitcher`.

#### `angled_double_grab_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4759`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_grab_espresso_pitcher`.

#### `angled_single_pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4763`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pick_espresso_pitcher`.

#### `angled_double_pick_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4767`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pick_espresso_pitcher`.

#### `angled_single_pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:4771`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pour_espresso_pitcher_cup_station`.

#### `angled_double_pour_espresso_pitcher_cup_station(**params)`

- **Location:** `testing_v1.py:4775`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_pour_espresso_pitcher_cup_station`.

#### `angled_single_return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4779`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_espresso_pitcher`.

#### `angled_double_return_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4783`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_espresso_pitcher`.

#### `angled_single_return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4787`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_cleaned_espresso_pitcher`.

#### `angled_double_return_cleaned_espresso_pitcher(**params)`

- **Location:** `testing_v1.py:4791`
- **Purpose:** Convenience wrapper for the angled espresso flow that sets the correct port/tool variant before delegating to the shared angled routine.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_return_cleaned_espresso_pitcher`.

### Portafilter cleaning

#### `invalidate_cleaning_cache()`

- **Location:** `testing_v1.py:4814`
- **Purpose:** Clears cached hard-brush and soft-brush cleaning poses.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_is_valid_angles(angles)`

- **Location:** `testing_v1.py:4818`
- **Purpose:** Internal validator that checks whether a joint-angle payload is a six-value tuple/list.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_capture_and_cache_current_angles(cache_list)`

- **Location:** `testing_v1.py:4821`
- **Purpose:** Internal helper that reads current joint angles and appends them to a cache list.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `current_angles`.
  - Important helper/API calls: `run_skill`.

#### `clean_portafilter(**params)`

- **Location:** `testing_v1.py:4828`
- **Purpose:** Keep approach/mount live, cache only the post-mount motion sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `approach_machine`, `mount_machine`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `angled_clean_portafilter`, `run_skill`.
  - Key constants/config used: `DEFAULT_PORT`, `CLEANING_PARAMS`, `ESPRESSO_GRINDER_HOME`.

#### `clean_portafilter_single(**params)`

- **Location:** `testing_v1.py:4956`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `clean_portafilter`.

#### `clean_portafilter_double(**params)`

- **Location:** `testing_v1.py:4960`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `clean_portafilter`.

### Angled portafilter cleaning

#### `angled_invalidate_cleaning_cache()`

- **Location:** `testing_v1.py:4973`
- **Purpose:** Clears cached angled-cleaning hard-brush and soft-brush poses.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled_cleaning_is_valid_angles(angles)`

- **Location:** `testing_v1.py:4977`
- **Purpose:** Internal angled-cleaning validator for six joint angles.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled_cleaning_capture_current_angles(cache_list)`

- **Location:** `testing_v1.py:4980`
- **Purpose:** Captures current angles into the angled-cleaning cache list.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `current_angles`.
  - Important helper/API calls: `run_skill`, `angled_cleaning_is_valid_angles`.

#### `angled_clean_portafilter(**params)`

- **Location:** `testing_v1.py:4987`
- **Purpose:** Keep approach/mount live, cache only the post-mount motion sequence.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `approach_machine`, `mount_machine`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `angled__normalize_espresso_shot`, `run_skill`, `angled_cleaning_capture_current_angles`.
  - Key constants/config used: `DEFAULT_PORT`, `CLEANING_PARAMS`, `ESPRESSO_GRINDER_HOME`.

#### `angled_clean_portafilter_single(**params)`

- **Location:** `testing_v1.py:5089`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_clean_portafilter`.

#### `angled_clean_portafilter_double(**params)`

- **Location:** `testing_v1.py:5093`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_clean_portafilter`.

### Milk frothing

#### `invalidate_milk_frothing_cache()`

- **Location:** `testing_v1.py:5127`
- **Purpose:** Clears all cached poses related to milk frother pick, mount, pour, clean, and return flows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_is_valid_angles(angles)`

- **Location:** `testing_v1.py:5141`
- **Purpose:** Internal validator that checks whether a joint-angle payload is a six-value tuple/list.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_capture_current_angles()`

- **Location:** `testing_v1.py:5144`
- **Purpose:** Internal helper that reads current joint angles and returns them as a tuple for caching.
- **Returns:** `Optional[Tuple[float, ...]]`
- **Dependencies / side effects:**
  - Robot skills used: `current_angles`.
  - Important helper/API calls: `run_skill`.

#### `_capture_current_position()`

- **Location:** `testing_v1.py:5150`
- **Purpose:** Internal helper that reads current Cartesian pose and returns it as a tuple for caching.
- **Returns:** `Optional[Tuple[float, ...]]`
- **Dependencies / side effects:**
  - Robot skills used: `current_pose`.
  - Important helper/API calls: `run_skill`.

#### `_is_valid_position(position)`

- **Location:** `testing_v1.py:5156`
- **Purpose:** Internal validator for Cartesian pose payloads.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_cache_key_from_z_adjustment(z_adjustment)`

- **Location:** `testing_v1.py:5159`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `str`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `get_frother_position(**params)`

- **Location:** `testing_v1.py:5162`
- **Purpose:** Calibrate and record the milk frother position for future operations. After first successful calibration, skip re-reading machine position on later runs.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `move_to`, `get_machine_position`, `gotoJ_deg`.
  - Important helper/API calls: `invalidate_plastic_cup_cache`, `invalidate_milk_frothing_cache`, `run_skill`, `return_back_to_home`, `home`.
  - Key constants/config used: `MAX_FROTHER_CALIBRATION_RETRIES`, `CALIBRATION_SETTLE_TIME`.
  - Side effects: contains timed waits.

#### `pick_frother(**params)`

- **Location:** `testing_v1.py:5242`
- **Purpose:** Pick up the milk frother for milk frothing operations.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `approach_machine`, `sync`, `set_gripper_position`, `mount_machine`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHING_PARAMS`, `GRIPPER_FULL`, `MILK_FROTHER_GRIPPER_POSITIONS`.

#### `place_frother_milk_station(**params)`

- **Location:** `testing_v1.py:5265`
- **Purpose:** Places the frother at the milk station and caches the placement pose.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoEE`, `moveEE`, `gotoJ_deg`, `sync`, `moveEE_movJ`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHER_MOVEMENT_OFFSETS`, `MILK_FROTHING_PARAMS`, `MILK_FROTHER_GRIPPER_POSITIONS`.

#### `pick_frother_milk_station(**params)`

- **Location:** `testing_v1.py:5308`
- **Purpose:** Picks the frother back up from the milk station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_gripper_position`, `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHER_MOVEMENT_OFFSETS`, `MILK_FROTHING_PARAMS`.

#### `mount_frother(**params)`

- **Location:** `testing_v1.py:5335`
- **Purpose:** Mount the milk frother to the steam wand for frothing preparation. Applies a Z adjustment based on milk volume and cup size.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `approach_machine`, `mount_machine`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHER_SPEEDS`, `MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE`.

#### `unmount_and_swirl_milk(**params)`

- **Location:** `testing_v1.py:5373`
- **Purpose:** Swirl frothed milk in a circular motion for latte art preparation.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `set_speed_factor`, `sync`, `move_circle`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHING_DELAYS`, `MILK_FROTHER_SPEEDS`, `MILK_FROTHING_PARAMS`, `MILK_SWIRL_CIRCLE_PARAMS`.
  - Side effects: contains timed waits.

#### `pour_milk_cup_station(**params)`

- **Location:** `testing_v1.py:5406`
- **Purpose:** Pours milk into the selected cup station using configured cup/stage parameters.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHING_PARAMS`, `MILK_POURING_OFFSETS`, `MILK_FROTHER_SPEEDS`.

#### `clean_milk_pitcher(**params)`

- **Location:** `testing_v1.py:5454`
- **Purpose:** Moves through the milk pitcher cleaning path and caches the cleaned pose.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `MILK_FROTHING_PARAMS`, `MILK_FROTHER_MOVEMENT_OFFSETS`.

#### `return_frother(**params)`

- **Location:** `testing_v1.py:5481`
- **Purpose:** Return the frother to its original location using recorded approach/grab angles.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `gotoEE`, `moveEE`, `set_gripper_position`, `approach_machine`, `sync`.
  - Important helper/API calls: `run_skill`, `home`.
  - Key constants/config used: `MILK_FROTHING_PARAMS`, `MILK_FROTHER_MOVEMENT_OFFSETS`, `GRIPPER_FULL`, `MILK_FROTHER_GRIPPER_POSITIONS`.
  - Side effects: contains timed waits.

### Plastic cups and ice

#### `invalidate_plastic_cup_cache()`

- **Location:** `testing_v1.py:5529`
- **Purpose:** Clears cached poses related to plastic-cup, ice, sauces, and milk-station handling.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_is_valid_angles(angles)`

- **Location:** `testing_v1.py:5537`
- **Purpose:** Internal validator that checks whether a joint-angle payload is a six-value tuple/list.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_capture_current_angles()`

- **Location:** `testing_v1.py:5540`
- **Purpose:** Internal helper that reads current joint angles and returns them as a tuple for caching.
- **Returns:** `Optional[Tuple[float, ...]]`
- **Dependencies / side effects:**
  - Robot skills used: `current_angles`.
  - Important helper/API calls: `run_skill`.

#### `_normalize_plastic_cup_size(cups_dict)`

- **Location:** `testing_v1.py:5546`
- **Purpose:** Parses a plastic-cup request and returns the standard size string used by plastic-cup flows.
- **Returns:** `str`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `dispense_plastic_cup(**params)`

- **Location:** `testing_v1.py:5580`
- **Purpose:** Dispenses a plastic cup of the requested size and sets the internal cup-dispensed state flag.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_gripper_position`, `gotoJ_deg`, `moveEE`, `sync`, `set_DO`.
  - Important helper/API calls: `validate_cup_size`, `home`, `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `DEFAULT_PLASTIC_CUP_SIZE`, `CUP_CONFIG`, `PLASTIC_CUPS_PARAMS`, `DISPENSE_PARAMS`.
  - Side effects: contains timed waits.

#### `go_to_ice(**params)`

- **Location:** `testing_v1.py:5631`
- **Purpose:** Moves a plastic cup to the ice dispenser and performs the ice dispensing pose.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `PLASTIC_CUPS_PARAMS`, `GRIPPER_RELEASE_GENTLE`, `GRIPPER_HOLD_LOOSE`.

#### `go_home_with_ice(**params)`

- **Location:** `testing_v1.py:5650`
- **Purpose:** Returns from the ice dispenser to a safe home/transfer pose while holding the iced cup.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveEE_movJ`, `set_gripper_position`.
  - Important helper/API calls: `validate_cup_size`, `run_skill`, `home`.
  - Key constants/config used: `DEFAULT_PLASTIC_CUP_SIZE`, `GRIPPER_FULL`, `PLASTIC_CUPS_PARAMS`.

#### `place_plastic_cup_station(**params)`

- **Location:** `testing_v1.py:5686`
- **Purpose:** Places a plastic cup at a selected station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `set_gripper_position`, `moveEE`.
  - Important helper/API calls: `run_skill`, `home`.
  - Key constants/config used: `SPEED_NORMAL`, `PLASTIC_CUPS_PARAMS`, `SPEED_FAST`, `PLASTIC_CUP_MOVEMENT_OFFSETS`.

#### `pick_plastic_cup_station(**params)`

- **Location:** `testing_v1.py:5744`
- **Purpose:** Picks a plastic cup back up from a selected station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `moveEE`, `set_gripper_position`, `set_speed_factor`.
  - Important helper/API calls: `home`, `run_skill`.
  - Key constants/config used: `PLASTIC_CUPS_PARAMS`, `PLASTIC_CUP_MOVEMENT_OFFSETS`, `GRIPPER_FULL`, `SPEED_NORMAL`.

#### `place_plastic_cup_sauces(**params)`

- **Location:** `testing_v1.py:5803`
- **Purpose:** Places a plastic cup at the sauces station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `moveEE`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `SPEED_NORMAL`, `PLASTIC_CUPS_PARAMS`, `GRIPPER_RELEASE_GENTLE`, `GRIPPER_HOLD_LOOSE`.

#### `pick_plastic_cup_sauces(**params)`

- **Location:** `testing_v1.py:5842`
- **Purpose:** Picks a plastic cup from the sauces station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `moveEE`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `SPEED_NORMAL`, `GRIPPER_FULL`, `PLASTIC_CUPS_PARAMS`.

#### `place_plastic_cup_milk(**params)`

- **Location:** `testing_v1.py:5876`
- **Purpose:** Places a plastic cup at the milk station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `SPEED_NORMAL`, `PLASTIC_CUPS_PARAMS`, `GRIPPER_RELEASE_GENTLE`, `GRIPPER_HOLD_LOOSE`.

#### `pick_plastic_cup_milk(**params)`

- **Location:** `testing_v1.py:5900`
- **Purpose:** Picks a plastic cup from the milk station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `moveEE`, `set_gripper_position`.
  - Important helper/API calls: `run_skill`, `detect_cup_gripper`.
  - Key constants/config used: `SPEED_NORMAL`, `GRIPPER_FULL`, `PLASTIC_CUPS_PARAMS`.

### Slush handling

#### `_normalize_slush_cup_size(cups_dict)`

- **Location:** `testing_v1.py:5943`
- **Purpose:** Normalizes slush cup size requests from paper/plastic cup dictionaries.
- **Returns:** `str`
- **Dependencies / side effects:**
  - Key constants/config used: `DEFAULT_PLASTIC_CUP_SIZE`.

#### `get_slush(**params)`

- **Location:** `testing_v1.py:5970`
- **Purpose:** Dispenses a plastic cup and moves it to the selected slush dispenser.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_gripper_position`.
  - Important helper/API calls: `dispense_plastic_cup`, `run_skill`.
  - Key constants/config used: `SLUSH_PARAMS`, `GRIPPER_RELEASE_GENTLE`, `GRIPPER_HOLD_LOOSE`.

#### `place_slush(**params)`

- **Location:** `testing_v1.py:6011`
- **Purpose:** Returns from the slush dispenser and places the filled cup at the selected station.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `set_gripper_position`, `sync`, `gotoJ_deg`.
  - Important helper/API calls: `run_skill`, `home`, `place_plastic_cup_station`.
  - Key constants/config used: `SPEED_NORMAL`, `GRIPPER_RELEASE_GENTLE`, `SLUSH_PARAMS`.

### External machine/service calls

#### `call_tamper(**params)`

- **Location:** `testing_v1.py:6053`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_coffee_machine(**params)`

- **Location:** `testing_v1.py:6072`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_hot_water(**params)`

- **Location:** `testing_v1.py:6094`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_coffee_purge(**params)`

- **Location:** `testing_v1.py:6114`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_frother(**params)`

- **Location:** `testing_v1.py:6134`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_grinder(**params)`

- **Location:** `testing_v1.py:6161`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_ice(**params)`

- **Location:** `testing_v1.py:6181`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_milk_syrup(**params)`

- **Location:** `testing_v1.py:6204`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

#### `call_slush(**params)`

- **Location:** `testing_v1.py:6233`
- **Purpose:** External equipment trigger/helper used to call the named machine function or actuator service.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `run`.
  - Side effects: runs an external subprocess/SSH command.

### Composite drink workflows

#### `espresso(**params)`

- **Location:** `testing_v1.py:6261`
- **Purpose:** Complete espresso preparation sequence using port_3.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `unmount`, `grinder`, `tamper`, `mount`, `grab_paper_cup`, `place_paper_cup`, `pick_espresso_pitcher`, `pour_espresso_pitcher_cup_station`, `return_espresso_pitcher`, `clean_portafilter`.
- **Additional notes from docstring:**
  - Workflow: 3. Unmount portafilter from port_3 2. Grind and tamp coffee 3. Mount portafilter back to port_3 4. Prepare paper cup at stage_3 5. Pour espresso from port_3 6. Return pitcher

#### `espresso_angled(**params)`

- **Location:** `testing_v1.py:6355`
- **Purpose:** Angled toolhead espresso: machine port key angled_portafilter_1, tool double_portafilter_angled, then paper cup arm1 7oz at stage 1, standard pitcher on port_1, pour and return at stage 1.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `angled_unmount`, `clean_portafilter`, `angled_mount`.
- **Additional notes from docstring:**
  - Steps: angled_unmount, clean_portafilter, angled_grinder, return_cleaned_espresso_pitcher, angled_tamper, angled_mount, dispense_paper_arm1_cup_station, grab/pick/pour/return pitcher.

#### `americano(**params)`

- **Location:** `testing_v1.py:6434`
- **Purpose:** Complete americano preparation sequence using port_1.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `unmount`, `grinder`, `tamper`, `mount`, `grab_paper_cup`, `place_paper_cup`, `pick_espresso_pitcher`, `get_hot_water`, `with_hot_water`, `pour_espresso_pitcher_cup_station`, `return_espresso_pitcher`.
- **Additional notes from docstring:**
  - Workflow: 1. Unmount portafilter from port_1 2. Grind and tamp coffee 3. Mount portafilter back to port_1 4. Prepare paper cup at stage_1 5. Get hot water and prepare 6. Pour espresso from port_1 7. Return pitcher

#### `multi_espresso(**params)`

- **Location:** `testing_v1.py:6526`
- **Purpose:** Batch espresso workflow that repeats espresso preparation across multiple cup stations.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `unmount`, `grinder`, `tamper`, `mount`, `grab_paper_cup`, `place_paper_cup`, `pick_espresso_pitcher`, `pour_espresso_pitcher_cup_station`, `return_espresso_pitcher`.

#### `milk_frothing(**params)`

- **Location:** `testing_v1.py:6555`
- **Purpose:** Composite milk-frothing workflow with logging, batching, frother handling, milk station actions, and cup placement.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `getLogger`, `setLevel`, `FileHandler`, `setFormatter`, `Formatter`, `addHandler`, `isoformat`, `now`, `now_iso`, `dumps`, `log_event`, `robot_say`, `perf_counter`, `fn`, `type`, `format_exc`, `validate_config`, `timed_step`, `print_cup_summary`, `print_batch_summary`.
  - Side effects: requires operator input, contains timed waits.

#### `milk_1(**params)`

- **Location:** `testing_v1.py:7089`
- **Purpose:** Pour milk at cup position 1 - simplified raw commands
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `set_speed_factor`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Side effects: contains timed waits.

#### `milk_2(**params)`

- **Location:** `testing_v1.py:7104`
- **Purpose:** Pour milk at cup position 2 - simplified raw commands
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Side effects: contains timed waits.

#### `milk_3(**params)`

- **Location:** `testing_v1.py:7120`
- **Purpose:** Pour milk at cup position 3 - simplified raw commands
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Side effects: contains timed waits.

#### `milk_4(**params)`

- **Location:** `testing_v1.py:7136`
- **Purpose:** Pour milk at cup position 4 - simplified raw commands
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `gotoJ_deg`, `sync`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Side effects: contains timed waits.

#### `slushie(**params)`

- **Location:** `testing_v1.py:7152`
- **Purpose:** Complete slushie preparation sequence.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_slush`, `place_slush`.
- **Additional notes from docstring:**
  - Workflow: 1. Dispense plastic cup (if needed, handled by get_slush internally) 2. Get slush from dispenser 3. Place slush cup at designated position

### Training, diagnostics, tests, and CLI

#### `espresso_training(**params)`

- **Location:** `testing_v1.py:7198`
- **Purpose:** Manual training helper for espresso machine, grinder, and cleaner points using guided moves and operator prompts.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `moveEE_movJ`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`, `ESPRESSO_GRINDER_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `milk_training(**params)`

- **Location:** `testing_v1.py:7285`
- **Purpose:** Manual training helper for milk frother and steam-wand related points.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `sync`, `approach_tool`, `set_gripper_position`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `GRIPPER_FULL`, `MILK_FROTHER_GRIPPER_POSITIONS`.
  - Side effects: requires operator input, contains timed waits.

#### `angled_espresso_training(**params)`

- **Location:** `testing_v1.py:7317`
- **Purpose:** Manual training helper for angled espresso/angled portafilter points.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `sync`, `approach_tool`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `espresso_port_1_training(**params)`

- **Location:** `testing_v1.py:7375`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `sync`, `approach_tool`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `espresso_port_2_training(**params)`

- **Location:** `testing_v1.py:7394`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `sync`, `approach_tool`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `angled_espresso_port_1_training(**params)`

- **Location:** `testing_v1.py:7413`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `sync`, `approach_tool`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `angled_espresso_port_2_training(**params)`

- **Location:** `testing_v1.py:7431`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`, `moveEE_movJ`, `sync`, `approach_tool`, `grab_tool`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `angled_grinder_training(**params)`

- **Location:** `testing_v1.py:7452`
- **Purpose:** Manual training helper for angled grinder/tamper points.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `ESPRESSO_GRINDER_HOME`.
  - Side effects: requires operator input, contains timed waits.

#### `angled_cleaner_training(**params)`

- **Location:** `testing_v1.py:7477`
- **Purpose:** Manual training helper for angled cleaning station points.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `get_machine_position`.
  - Important helper/API calls: `home`, `run_skill`.
  - Side effects: requires operator input, contains timed waits.

#### `test(**params)`

- **Location:** `testing_v1.py:7500`
- **Purpose:** General manual test routine for current robot setup.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_machine_position`, `perf_counter`, `unmount`, `clean_portafilter`, `Thread`, `start`, `tamper`, `mount`, `grab_espresso_pitcher`, `pick_espresso_pitcher`, `pour_espresso_pitcher_cup_station`, `return_espresso_pitcher`.
  - Side effects: requires operator input, uses executor/thread cleanup.

#### `test_arm1(**params)`

- **Location:** `testing_v1.py:7566`
- **Purpose:** Manual test routine focused on arm 1 motions.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`.
  - Important helper/API calls: `get_machine_position`, `perf_counter`, `angled_unmount`, `angled_clean_portafilter`, `Thread`, `start`, `angled_tamper`, `angled_mount`, `run_skill`.
  - Side effects: uses executor/thread cleanup.

#### `test_both_port(**params)`

- **Location:** `testing_v1.py:7633`
- **Purpose:** Manual test routine for both portafilter/port paths.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`.
  - Important helper/API calls: `step_fn`, `get_machine_position`, `perf_counter`, `run_skill`, `angled_unmount`, `angled_clean_portafilter`, `angled_grinder`, `call_tamper`, `angled_tamper`, `angled_mount`.

#### `test_plastic_cup(**params)`

- **Location:** `testing_v1.py:7774`
- **Purpose:** Manual test routine for plastic cup flows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`.
  - Important helper/API calls: `invalidate_plastic_cup_cache`, `run_skill`, `perf_counter`, `dispense_plastic_cup`, `go_to_ice`, `call_ice`, `go_home_with_ice`, `place_plastic_cup_milk`, `call_milk_syrup`, `pick_plastic_cup_milk`, `place_plastic_cup_sauces`, `pick_plastic_cup_sauces`, `place_plastic_cup_station`.

#### `test_paper_cup(**params)`

- **Location:** `testing_v1.py:7817`
- **Purpose:** Manual test routine for paper cup flows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`, `move_portafilter_arc_tool_angled`, `moveJ_deg`.
  - Important helper/API calls: `run_skill`.

#### `now_ms()`

- **Location:** `testing_v1.py:7860`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `float`
- **Dependencies / side effects:**
  - Important helper/API calls: `perf_counter`.

#### `summarize(values, label, skip_first=False)`

- **Location:** `testing_v1.py:7863`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** May return data or `None` on failure.
- **Dependencies / side effects:**
  - Important helper/API calls: `pstdev`, `median`.

#### `print_chunk_stats(values, label, chunk_size=10)`

- **Location:** `testing_v1.py:7911`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `pstdev`.

#### `safe_run_skill(skill_name, *args)`

- **Location:** `testing_v1.py:7925`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `skill_name`.
  - Important helper/API calls: `now_ms`, `run_skill`, `repr`.

#### `go_to_start()`

- **Location:** `testing_v1.py:7937`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `safe_run_skill`.
  - Key constants/config used: `START_JOINTS`.

#### `warmup()`

- **Location:** `testing_v1.py:7944`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `safe_run_skill`.
  - Key constants/config used: `START_JOINTS`, `GRIPPER_OPEN`, `GRIPPER_CLOSE`.

#### `run_motion_series(motion_name, dx_mm, repeats, writer)`

- **Location:** `testing_v1.py:7953`
- **Purpose:** motion_name: 'moveEE' or 'moveEE_movJ' dx_mm: step size in +X repeats: number of repetitions
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `go_to_start`, `safe_run_skill`, `writerow`, `summarize`, `print_chunk_stats`.

#### `run_gripper_series(cycles, writer)`

- **Location:** `testing_v1.py:8000`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `go_to_start`, `now_ms`, `safe_run_skill`, `writerow`, `summarize`.
  - Key constants/config used: `GRIPPER_OPEN`, `GRIPPER_CLOSE`.

#### `test_arm2(**params)`

- **Location:** `testing_v1.py:8077`
- **Purpose:** Manual test routine focused on arm 2 motions.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Important helper/API calls: `mkdir`, `strftime`, `DictWriter`, `writeheader`, `warmup`, `run_motion_series`, `go_to_start`, `run_gripper_series`, `pstdev`.
  - Key constants/config used: `OUT_DIR`, `STEP_SIZES_MM`, `TOTAL_TRAVEL_MM`.

#### `robot_arm_test(**params)`

- **Location:** `testing_v1.py:8147`
- **Purpose:** Deterministic timing test: - Go to zero position - Move joint 4: +90 / -90 (3 cycles) - Repeat 10 times - Log timing for each run
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `set_speed_factor`, `sync`, `gotoJ_deg`, `moveJ_deg`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `TOTAL_RUNS`, `CYCLES_PER_RUN`.
  - Side effects: contains timed waits.

#### `hello(**params)`

- **Location:** `testing_v1.py:8226`
- **Purpose:** Small connectivity/smoke-test function.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `sync`.
  - Important helper/API calls: `perf_counter`, `run_skill`.

#### `_print_kinematics_srv_response(res)`

- **Location:** `testing_v1.py:8647`
- **Purpose:** Formats and prints Dobot kinematics service responses.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `run_kinematics_tools_menu()`

- **Location:** `testing_v1.py:8661`
- **Purpose:** Interactive 1-10 menu mirroring kinemtaics_solutions.py; returns to sequence CLI on q.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `init`, `create_node`, `create_client`, `Request`, `call`, `destroy_node`.
  - Key constants/config used: `CP`.
  - Side effects: requires operator input, contains timed waits.

#### `signal_handler(signum, frame)`

- **Location:** `testing_v1.py:8840`
- **Purpose:** Handle shutdown signals gracefully
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `cleanup_motion_node`, `exit`.

#### `_main()`

- **Location:** `testing_v1.py:8846`
- **Purpose:** Entry point for the interactive testing CLI/menu.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `signal`, `register`, `run_kinematics_tools_menu`, `index`, `solution`, `cleanup_motion_node`.
  - Key constants/config used: `USE_VERSION`, `SEQUENCES`.
  - Side effects: requires operator input.

### Nested helper functions inside `testing_v1.py`

These functions are defined inside another function. They are not importable directly, but they matter for understanding the parent workflow.

#### `home.ok(r)`

- **Location:** `testing_v1.py:312`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `return_back_to_home.ok(r)`

- **Location:** `testing_v1.py:332`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `get_machine_position.ok(r)`

- **Location:** `testing_v1.py:406`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `_prep_cleaner()`

- **Location:** `testing_v1.py:417`
- **Scope:** nested inside `get_machine_position`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `sync`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `HOME_CALIBRATION_PARAMS`, `HOME_CALIBRATION_CONSTANTS`.
  - Side effects: contains timed waits.

#### `_prep_grinder()`

- **Location:** `testing_v1.py:431`
- **Scope:** nested inside `get_machine_position`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `move_to`, `sync`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `HOME_CALIBRATION_PARAMS`, `HOME_CALIBRATION_CONSTANTS`.
  - Side effects: contains timed waits.

#### `_prep_espresso()`

- **Location:** `testing_v1.py:447`
- **Scope:** nested inside `get_machine_position`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Robot skills used: `gotoJ_deg`, `moveJ_deg`, `move_to`, `sync`.
  - Important helper/API calls: `run_skill`.
  - Key constants/config used: `HOME_CALIBRATION_PARAMS`, `HOME_CALIBRATION_CONSTANTS`.
  - Side effects: contains timed waits.

#### `grab_paper_cup.ok(r)`

- **Location:** `testing_v1.py:726`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_cup.ok(r)`

- **Location:** `testing_v1.py:794`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `grab_paper_cup_arm1.ok(r)`

- **Location:** `testing_v1.py:847`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_cup_arm1.ok(r)`

- **Location:** `testing_v1.py:926`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `grab_paper_arm2_cup_station.ok(r)`

- **Location:** `testing_v1.py:1013`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_arm2_cup_station.ok(r)`

- **Location:** `testing_v1.py:1100`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_paper_cup_station.ok(r)`

- **Location:** `testing_v1.py:1177`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_cup_station.ok(r)`

- **Location:** `testing_v1.py:1234`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_cup_sauces.ok(r)`

- **Location:** `testing_v1.py:1272`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_paper_cup_sauces.ok(r)`

- **Location:** `testing_v1.py:1293`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_paper_cup_milk.ok(r)`

- **Location:** `testing_v1.py:1323`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_paper_cup_milk.ok(r)`

- **Location:** `testing_v1.py:1344`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_cup_for_hot_water.ok(r)`

- **Location:** `testing_v1.py:1374`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `return_cup_with_hot_water.ok(r)`

- **Location:** `testing_v1.py:1445`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `unmount.ok(r)`

- **Location:** `testing_v1.py:1777`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `_close_and_verify_grip()`

- **Location:** `testing_v1.py:1813`
- **Scope:** nested inside `unmount`
- **Purpose:** Close gripper and return (gripped_ok, reported_position).
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_motion_node`, `set_gripper_position`.
  - Key constants/config used: `_PORTAFILTER_GRIP_POS_MIN`, `_PORTAFILTER_GRIP_POS_MAX`.

#### `_grab_then_close()`

- **Location:** `testing_v1.py:1825`
- **Scope:** nested inside `unmount`
- **Purpose:** Perform grab_tool + close + read. Returns (gripped_ok, pos).
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `grab_tool`.
  - Important helper/API calls: `run_skill`.

#### `grinder.ok(r)`

- **Location:** `testing_v1.py:2019`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `tamper.ok(r)`

- **Location:** `testing_v1.py:2135`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `mount.ok(r)`

- **Location:** `testing_v1.py:2217`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `grab_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:2302`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:2452`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pour_espresso_pitcher_cup_station.ok(r)`

- **Location:** `testing_v1.py:2536`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `get_hot_water.ok(r)`

- **Location:** `testing_v1.py:2620`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `with_hot_water.ok(r)`

- **Location:** `testing_v1.py:2647`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `return_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:2660`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `return_cleaned_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:2841`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_unmount.ok(r)`

- **Location:** `testing_v1.py:3203`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `trace_grip(msg)`

- **Location:** `testing_v1.py:3250`
- **Scope:** nested inside `angled_unmount`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_close_and_verify_grip_angled()`

- **Location:** `testing_v1.py:3253`
- **Scope:** nested inside `angled_unmount`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_motion_node`, `set_gripper_position`.
  - Key constants/config used: `ANGLED_UNMOUNT_GRIP_POS_MIN`, `ANGLED_UNMOUNT_GRIP_POS_MAX`.

#### `_grab_then_close_angled()`

- **Location:** `testing_v1.py:3265`
- **Scope:** nested inside `angled_unmount`
- **Purpose:** Grab sequence for the named object/tool/station.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `grab_tool`.
  - Important helper/API calls: `run_skill`.

#### `_rerun_approach_for_retry(attempt_idx)`

- **Location:** `testing_v1.py:3274`
- **Scope:** nested inside `angled_unmount`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** `bool`
- **Dependencies / side effects:**
  - Robot skills used: `set_gripper_position`, `sync`.
  - Important helper/API calls: `trace_grip`, `run_skill`.

#### `angled_grinder.ok(r)`

- **Location:** `testing_v1.py:3607`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_tamper.ok(r)`

- **Location:** `testing_v1.py:3709`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `_close_and_verify_grip_angled()`

- **Location:** `testing_v1.py:3743`
- **Scope:** nested inside `angled_tamper`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `get_motion_node`, `set_gripper_position`.
  - Key constants/config used: `ANGLED_TAMPER_GRIP_POS_MIN`, `ANGLED_TAMPER_GRIP_POS_MAX`.

#### `_grab_then_close_angled()`

- **Location:** `testing_v1.py:3755`
- **Scope:** nested inside `angled_tamper`
- **Purpose:** Grab sequence for the named object/tool/station.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Robot skills used: `sync`, `grab_tool`.
  - Important helper/API calls: `run_skill`.

#### `angled_mount.ok(r)`

- **Location:** `testing_v1.py:3892`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `trace(msg)`

- **Location:** `testing_v1.py:3897`
- **Scope:** nested inside `angled_mount`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `angled_grab_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:4076`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_pick_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:4201`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_pour_espresso_pitcher_cup_station.ok(r)`

- **Location:** `testing_v1.py:4271`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_get_hot_water.ok(r)`

- **Location:** `testing_v1.py:4354`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_with_hot_water.ok(r)`

- **Location:** `testing_v1.py:4381`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_return_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:4394`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_return_cleaned_espresso_pitcher.ok(r)`

- **Location:** `testing_v1.py:4555`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `clean_portafilter.ok(r)`

- **Location:** `testing_v1.py:4842`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `angled_clean_portafilter.ok(r)`

- **Location:** `testing_v1.py:4997`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `get_frother_position.ok(r)`

- **Location:** `testing_v1.py:5167`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_frother.ok(r)`

- **Location:** `testing_v1.py:5247`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_frother_milk_station.ok(r)`

- **Location:** `testing_v1.py:5266`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_frother_milk_station.ok(r)`

- **Location:** `testing_v1.py:5309`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `mount_frother.ok(r)`

- **Location:** `testing_v1.py:5340`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `unmount_and_swirl_milk.ok(r)`

- **Location:** `testing_v1.py:5377`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pour_milk_cup_station.ok(r)`

- **Location:** `testing_v1.py:5407`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `clean_milk_pitcher.ok(r)`

- **Location:** `testing_v1.py:5455`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `return_frother.ok(r)`

- **Location:** `testing_v1.py:5486`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `go_to_ice.ok(r)`

- **Location:** `testing_v1.py:5632`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `go_home_with_ice.ok(r)`

- **Location:** `testing_v1.py:5651`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_plastic_cup_station.ok(r)`

- **Location:** `testing_v1.py:5687`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_plastic_cup_station.ok(r)`

- **Location:** `testing_v1.py:5745`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_plastic_cup_sauces.ok(r)`

- **Location:** `testing_v1.py:5804`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_plastic_cup_sauces.ok(r)`

- **Location:** `testing_v1.py:5843`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_plastic_cup_milk.ok(r)`

- **Location:** `testing_v1.py:5877`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `pick_plastic_cup_milk.ok(r)`

- **Location:** `testing_v1.py:5901`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `get_slush.ok(r)`

- **Location:** `testing_v1.py:5971`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `place_slush.ok(r)`

- **Location:** `testing_v1.py:6012`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `now_iso()`

- **Location:** `testing_v1.py:6610`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `isoformat`, `now`.

#### `robot_say(message, mood='robot')`

- **Location:** `testing_v1.py:6613`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_event(event_name, level='info', **data)`

- **Location:** `testing_v1.py:6634`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Formatted console logging helper.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `now_iso`, `dumps`.

#### `validate_config()`

- **Location:** `testing_v1.py:6660`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Validation helper for checking that a parameter is within supported values.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `log_event`, `robot_say`.

#### `timed_step(step_name, fn, *args, **kwargs)`

- **Location:** `testing_v1.py:6691`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `perf_counter`, `log_event`, `robot_say`, `fn`, `type`, `format_exc`.
  - Side effects: contains timed waits.

#### `print_cup_summary(cup_summary)`

- **Location:** `testing_v1.py:6740`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_say`.

#### `print_batch_summary(batch_summary)`

- **Location:** `testing_v1.py:6752`
- **Scope:** nested inside `milk_frothing`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `robot_say`.

#### `_run_sequence(label_steps, iteration_number)`

- **Location:** `testing_v1.py:7644`
- **Scope:** nested inside `test_both_port`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:**
  - Important helper/API calls: `step_fn`.

#### `robot_arm_test.ok(r)`

- **Location:** `testing_v1.py:8156`
- **Purpose:** Local success-check helper used by the parent function. It treats `False` and `None` as failure.
- **Returns:** `bool`

#### `call(name, req)`

- **Location:** `testing_v1.py:8708`
- **Scope:** nested inside `run_kinematics_tools_menu`
- **Purpose:** Internal/helper function used by the robot automation workflows.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

## `params.py`

This file is the shared configuration layer. Most of the file is constants: speeds, gripper positions, home positions, calibration parameters, station poses, cup maps, milk/slush/espresso settings, and offsets. The functions below parse request dictionaries, validate values, and print consistent logs.

### Helper functions

#### `_set_cup_dispensed()`

- **Location:** `params.py:77`
- **Purpose:** Set flag indicating cup was just dispensed or came from ice.
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_check_and_clear_cup_dispensed()`

- **Location:** `params.py:82`
- **Purpose:** Check if cup was just dispensed, then clear the flag.
- **Returns:** `bool`
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `_extract_cup_position(params)`

- **Location:** `params.py:90`
- **Purpose:** Extract cup_position from new parameter format.
- **Returns:** `int`
- **Dependencies / side effects:**
  - Key constants/config used: `DEFAULT_CUP_POSITION`.
- **Additional notes from docstring:**
  - New format: {'position': {'cup_position': 1.0}} Also supports legacy: {'stage': '1'} or {'stage': 1}

#### `_extract_cups_dict(params)`

- **Location:** `params.py:128`
- **Purpose:** Extract cups dictionary from various parameter formats.
- **Returns:** `dict`
- **Dependencies / side effects:** None significant beyond normal Python execution.
- **Additional notes from docstring:**
  - Handles multiple input formats: - New nested format: {'ingredients': {'cups': {'cup_H12': 1.0}}} - Direct format: {'cups': {'cup_H12': 1.0}} - Array format: [{'ingredients': {'cups': ...}}] - Legacy format: {'size': '12oz'} or {'cup_size': '12oz'}

#### `_normalize_cup_size(cups_dict, cup_type='paper', default_size=None)`

- **Location:** `params.py:172`
- **Purpose:** Unified cup size normalizer for paper (H-codes) and plastic (C-codes) cups.
- **Returns:** `str`
- **Dependencies / side effects:**
  - Key constants/config used: `DEFAULT_PAPER_CUP_SIZE`, `DEFAULT_PLASTIC_CUP_SIZE`.

#### `validate_port(port)`

- **Location:** `params.py:236`
- **Purpose:** Validate port parameter
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Key constants/config used: `VALID_PORTS`.
  - Side effects: uses executor/thread cleanup.

#### `validate_stage(stage)`

- **Location:** `params.py:243`
- **Purpose:** Validate stage parameter
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Key constants/config used: `VALID_STAGES`.
  - Side effects: uses executor/thread cleanup.

#### `validate_cup_size(cup_size)`

- **Location:** `params.py:250`
- **Purpose:** Validate cup size parameter
- **Returns:** `bool` success/failure value.
- **Dependencies / side effects:**
  - Key constants/config used: `VALID_CUP_SIZES`.
  - Side effects: uses executor/thread cleanup.

#### `get_param_with_default(params, key, default)`

- **Location:** `params.py:257`
- **Purpose:** Get parameter with default value
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_step(step_num, total_steps, description)`

- **Location:** `params.py:261`
- **Purpose:** Log a formatted step
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_success(message, indent=0)`

- **Location:** `params.py:265`
- **Purpose:** Log success message
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_error(message, indent=0)`

- **Location:** `params.py:270`
- **Purpose:** Log error message
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_warning(message, indent=0)`

- **Location:** `params.py:275`
- **Purpose:** Log warning message
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

#### `log_info(message, indent=0)`

- **Location:** `params.py:280`
- **Purpose:** Log info message
- **Returns:** Return value follows the underlying operation; see code/docstring for exact payload.
- **Dependencies / side effects:** None significant beyond normal Python execution.

## Clarifications / follow-up items

The current reference is complete for the files provided. The only areas that may need confirmation later are:

1. Which of the manual `*_training` and `test_*` routines are still actively used versus historical tuning scripts.
2. Whether duplicated helper definitions in `testing_v1.py` should be consolidated during cleanup.
3. Which external machine `call_*` functions are mapped to production hardware endpoints, since their names show intent but deployment wiring should be verified on the target system.
4. Whether `manipulate_node_v4.py` should have its header updated from “V3 hybrid” to “V4 complete”, since the code and selector describe it as v4 elsewhere.
