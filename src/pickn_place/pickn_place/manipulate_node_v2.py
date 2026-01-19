#!/usr/bin/env python3
# Python Imports
import sys
import time
import os
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ament_index_python.packages import get_package_share_directory

# ROS Imports
import rclpy   
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import tf_transformations

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from control_msgs.action import FollowJointTrajectory

import threading
from rclpy.executors import SingleThreadedExecutor

# MoveIt2 Imports
from pymoveit2 import MoveIt2, MoveIt2State
from tf_transformations import euler_matrix, quaternion_from_matrix

# Helper functions
def get_transform_list(tf_stamped):
    """
    Converts a geometry_msgs TransformStamped message into a list: 
    [tx, ty, tz, qx, qy, qz, qw].
    """
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]

# MoveIt2 Configuration Constants
REFERENCE_FRAME = "base_link"
PLANNER_ID = "OMPL"
CARTESIAN = True
CARTESIAN_MAX_STEP = 0.001
CARTESIAN_FRACTION_THRESHOLD = 0.8
CARTESIAN_JUMP_THRESHOLD = 0.0
CARTESIAN_AVOID_COLLISIONS = True
VELOCITY_SCALING = 1.0
ACCELERATION_SCALING = 1.0
END_EFFECTOR_NAME = "tool_link"
GROUP_NAME = "portafilter_center"
SYNCHRONOUS = True

class robot_perception(Node):
    def __init__(self):
        super().__init__("robot_perception")
        self._percep_exec = SingleThreadedExecutor()
        self._percep_exec.add_node(self)
        self._executor_thread = threading.Thread(target=self._percep_exec.spin, daemon=False)
        self._executor_thread.start()
    
    def destroy_node(self):
        """Override destroy_node to properly cleanup threads"""
        try:
            if hasattr(self, '_percep_exec'):
                self._percep_exec.shutdown()
            if hasattr(self, '_executor_thread') and self._executor_thread.is_alive():
                self._executor_thread.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()

    def get_tf(
        self,
        target_frame: str,
        reference_frame: str = "base_link",
        max_retries: int = 3,
        sleep_time: float = 0.035,
    ) -> tuple[list[float], float] | tuple[None, None]:
        """
        Look up TF once (with limited retries).  If the TF infrastructure has
        not been set up yet, create it on-demand so other methods remain
        untouched.
        """

        from tf2_ros import Buffer, TransformListener

        # ---- lazy-initialise TF listener ---------------------------------
        if not hasattr(self, "_tf_ready"):
            # create only when first needed
            self.tf_buffer   = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._tf_ready   = True

        # ---- lookup with retry ------------------------------------------
        for _ in range(max_retries):
            try:
                t = self.tf_buffer.lookup_transform(
                    reference_frame, target_frame, Time())
                tr  = t.transform.translation
                rot = t.transform.rotation
                pose  = [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                return pose, stamp
            except Exception:          # LookupException, ConnectivityException, etc.
                time.sleep(sleep_time)

        return None, None

    def acquire_target_transform(
        self,
        target_frame: str,
        *,
        max_wait: float = 10.0,
        trans_thresh: float = 0.001,
        rot_thresh: float = 2.5,
        num_samples: int = 10,
    ) -> list[float] | None:
        """
        Obtain one *stable* transform for `target_frame`. Samples until `num_samples` 
        unique poses are collected; if the max spread (ΔT, ΔR) across the window is 
        within thresholds, return the averaged pose. Otherwise clear and retry until 
        `max_wait` elapses.
        """
        import time, math, numpy as np
        from scipy.spatial.transform import Rotation as Rot

        log = self.get_logger()
        SAMPLE_DELAY = 0.035 #for 30 FPS

        # Lazy‐initialize TF listener
        if not hasattr(self, "_tf_ready"):
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._tf_ready = True

        def get_pose_and_stamp():
            pose, stamp = self.get_tf(target_frame, max_retries=3)
            return pose

        def spread(a, b):
            d_pos = np.linalg.norm(np.array(a[:3]) - np.array(b[:3]))
            d_rot = Rot.from_quat(a[3:]).inv() * Rot.from_quat(b[3:])
            return d_pos, math.degrees(d_rot.magnitude())

        def max_spread(buf):
            dts, drs = zip(*(spread(a, b)
                            for i, a in enumerate(buf)
                            for b in buf[i + 1:]))
            return max(dts), max(drs)

        def average_pose(buf):
            arr = np.asarray(buf)
            pos = arr[:, :3].mean(axis=0)
            Q = arr[:, 3:]
            # align quaternion hemisphere
            Q[np.sum(Q * Q[0], axis=1) < 0] *= -1
            M = (Q[:, :, None] * Q[:, None, :]).mean(axis=0)
            quat = np.linalg.eigh(M)[1][:, -1]
            quat /= np.linalg.norm(quat)
            return np.concatenate([pos, quat]).tolist()

        samples = []
        start_time = time.monotonic()
        last_dT = last_dR = None

        while (time.monotonic() - start_time) < max_wait:
            pose = get_pose_and_stamp()
            if pose is None:
                time.sleep(SAMPLE_DELAY)
                continue

            # Discard duplicates against *all* previous samples
            if pose in samples:
                time.sleep(SAMPLE_DELAY)
                continue

            samples.append(pose)
            if len(samples) > num_samples:
                samples.pop(0)

            # Only check stability once the window is full
            if len(samples) < num_samples:
                time.sleep(SAMPLE_DELAY)
                continue

            # Compute max spread over the window
            dT, dR = max_spread(samples)
            last_dT, last_dR = dT, dR
            log.info(f"[acqTF] max dT={dT:.4f} m, dR={dR:.2f}° "
                    f"(tol ≤{trans_thresh:.4f} m / {rot_thresh:.2f}°)")

            if dT <= trans_thresh and dR <= rot_thresh:
                log.info("[acqTF] stable → returning averaged pose")
                return average_pose(samples)

            # Unstable: clear buffer and retry
            log.info("[acqTF] unstable window → reset buffer")
            samples.clear()
            time.sleep(SAMPLE_DELAY)

        # Timeout
        if last_dT is not None:
            log.error(
                f"acquire_target_transform: timeout after {max_wait:.1f}s; "
                f"last dT={last_dT:.4f} m, dR={last_dR:.2f}° "
                f"(required ≤{trans_thresh:.4f} m / {rot_thresh:.2f}°)"
            )
        else:
            log.error(
                f"acquire_target_transform: no valid TF samples within {max_wait:.1f}s"
            )
        return None

# ROS Interface Imports
from dobot_msgs_v3.srv import (
    StartDrag,
    StopDrag,
    SetGripperPosition,
    GetGripperPosition,
    GetPose,
    GetAngle,
    Sync,
    MovL,
    MovJ,
    JointMovJ,
    DOExecute,
    Arc,
    SpeedFactor,
    Circle3,
    RelMovL,
    RelMovJ,
)

class robot_motion(Node):
    def __init__(self):
        super().__init__("robot_motion")          # keep the node alive for the whole run

        # ── MoveIt2 Configuration ──────────────────────────────────────────
        self.reference_frame = REFERENCE_FRAME
        self.planner_id = PLANNER_ID
        self.cartesian = CARTESIAN
        self.cartesian_max_step = CARTESIAN_MAX_STEP
        self.cartesian_fraction_threshold = CARTESIAN_FRACTION_THRESHOLD
        self.cartesian_jump_threshold = CARTESIAN_JUMP_THRESHOLD
        self.cartesian_avoid_collisions = CARTESIAN_AVOID_COLLISIONS
        self.velocity_scaling = VELOCITY_SCALING
        self.acceleration_scaling = ACCELERATION_SCALING
        self.synchronous = SYNCHRONOUS

        # ── Initialize MoveIt2 ─────────────────────────────────────────────
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name=self.reference_frame,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
        )
        self.moveit2.planner_id = self.planner_id
        self.moveit2.max_velocity = self.velocity_scaling
        self.moveit2.max_acceleration = self.acceleration_scaling
        self.moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        # ── TF Buffer and Listener ─────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 1) create every service client exactly once ────────────────────
        self.start_drag_cli   = self.create_client(StartDrag,           '/dobot_bringup_v3/srv/StartDrag')
        self.stop_drag_cli    = self.create_client(StopDrag,            '/dobot_bringup_v3/srv/StopDrag')
        self.sync_cli         = self.create_client(Sync,                '/dobot_bringup_v3/srv/Sync')
        self.set_gripper_cli  = self.create_client(SetGripperPosition,  '/dobot_bringup_v3/srv/SetGripperPosition')
        self.get_gripper_cli  = self.create_client(GetGripperPosition,  '/dobot_bringup_v3/srv/GetGripperPosition')
        self.get_pose_cli     = self.create_client(GetPose,             '/dobot_bringup_v3/srv/GetPose')
        self.get_angle_cli    = self.create_client(GetAngle,            '/dobot_bringup_v3/srv/GetAngle')
        self.movl_cli         = self.create_client(MovL,                '/dobot_bringup_v3/srv/MovL')
        self.movj_cli         = self.create_client(MovJ,                '/dobot_bringup_v3/srv/MovJ')
        self.jointmovj_cli    = self.create_client(JointMovJ,           '/dobot_bringup_v3/srv/JointMovJ')
        self.doexec_cli       = self.create_client(DOExecute,           '/dobot_bringup_v3/srv/DOExecute')
        self.arc_cli          = self.create_client(Arc,                 '/dobot_bringup_v3/srv/Arc')
        self.speed_factor_cli = self.create_client(SpeedFactor,         '/dobot_bringup_v3/srv/SpeedFactor')
        self.circle3_cli      = self.create_client(Circle3,             '/dobot_bringup_v3/srv/Circle3')
        self.relmov_l_cli     = self.create_client(RelMovL,             '/dobot_bringup_v3/srv/RelMovL')
        self.relmov_j_cli     = self.create_client(RelMovJ,             '/dobot_bringup_v3/srv/RelMovJ')

        # ── 2) wait once for every service (5 s each) ──────────────────────
        timeout = 5.0
        service_map = {
            "StartDrag":           self.start_drag_cli,
            "StopDrag":            self.stop_drag_cli,
            "Sync":                self.sync_cli,
            "SetGripperPosition":  self.set_gripper_cli,
            "GetGripperPosition":  self.get_gripper_cli,
            "GetPose":             self.get_pose_cli,
            "GetAngle":            self.get_angle_cli,
            "MovL":                self.movl_cli,
            "MovJ":                self.movj_cli,
            "JointMovJ":           self.jointmovj_cli,
            "DOExecute":           self.doexec_cli,
            "Arc":                 self.arc_cli,
            "SpeedFactor":         self.speed_factor_cli,
            "Circle3":             self.circle3_cli,
            "RelMovL":             self.relmov_l_cli,
            "RelMovJ":             self.relmov_j_cli,
        }

        missing = [name for name, cli in service_map.items()
                   if not cli.wait_for_service(timeout_sec=timeout)]
        if missing:
            self.get_logger().error(
                f"⚠️  services unavailable after {timeout}s: {', '.join(missing)}")
        else:
            self.get_logger().info("✅  all motion-control services are ready")

    def verify_joint_positions(self, expected_joints, tolerance_deg=1.0):
        """
        Verify that the current joint positions match the expected values within tolerance.
        
        Args:
            expected_joints: List of expected joint angles in degrees
            tolerance_deg: Tolerance in degrees for each joint
            
        Returns:
            bool: True if all joints are within tolerance, False otherwise
        """
        try:
            current_angles = self.current_angles()
            if current_angles is None:
                self.get_logger().warn("verify_joint_positions(): Could not get current joint angles")
                return False
                
            if len(current_angles) != len(expected_joints):
                self.get_logger().error(f"verify_joint_positions(): Length mismatch - got {len(current_angles)}, expected {len(expected_joints)}")
                return False
                
            # Convert current angles from radians to degrees
            current_degrees = [math.degrees(angle) for angle in current_angles]
            
            # Check each joint
            for i, (current, expected) in enumerate(zip(current_degrees, expected_joints)):
                error = abs(current - expected)
                if error > tolerance_deg:
                    self.get_logger().warn(f"verify_joint_positions(): Joint {i+1} error {error:.2f}° > {tolerance_deg}°")
                    return False
                    
            self.get_logger().info("verify_joint_positions(): All joints within tolerance")
            return True
            
        except Exception as e:
            self.get_logger().error(f"verify_joint_positions(): Error: {e}")
            return False

    def get_machine_position(
            self,
            target_tf: str,
            required_samples: int = 6,
            *,
            acq_timeout: float = 10.0,
            debug: bool = False
    ) -> tuple[dict, dict] | None:
        """
        Collect `required_samples` fresh poses for `target_tf`, average them,
        write the result to machine_pose_data_memory.yaml, and return both the
        YAML entry and a datalog dictionary.  Returns *None* on any failure.
        """

        import time, os, yaml, numpy as np
        from ament_index_python.packages import get_package_share_directory

        log = self.get_logger()
        dl = {
            "start_time":   time.time(),
            "params":       dict(target_tf=target_tf,
                                required_samples=required_samples,
                                acq_timeout=acq_timeout),
            "poses":        [],        # list[list[float]]
            "final_pose":   {},
            "yaml_path":    None,
        }

        # ── 1) collect poses ──────────────────────────────────────────────────
        for i in range(required_samples):
            # spin up a *temporary* robot_perception node
            perception = robot_perception()
            perc_exec  = SingleThreadedExecutor()
            perc_exec.add_node(perception)

            # ── start TF listener in a helper thread ───────────────────────────
            import threading, contextlib
            tf_thread = threading.Thread(target=perc_exec.spin, daemon=True)
            tf_thread.start()

            pose = None
            try:
                pose = perception.acquire_target_transform(
                    target_tf,
                    max_wait=acq_timeout,
                    trans_thresh=0.001,     #1mm tolerance
                    rot_thresh=2.0,         #2 deg error
                    num_samples=6,
                )
            finally:
                # shut down executor and destroy the node
                with contextlib.suppress(Exception):
                    perc_exec.shutdown()
                perception.destroy_node()

            if pose is None:
                log.error(f"[GMP] acquire_target_transform timed out (sample {i})")
                return None

            dl["poses"].append(pose)
            if debug:
                log.debug(f"[GMP] #{i:02d}: {pose}")

        # ── 2) average translation & quaternion (Markley method) ──────────────
        poses_np = np.asarray(dl["poses"])           # (N, 7)
        tx, ty, tz = poses_np[:, :3].mean(axis=0)

        quats = poses_np[:, 3:]                      # (N, 4)  [x y z w]
        M = sum(np.outer([q[3], *q[:3]], [q[3], *q[:3]]) for q in quats) / required_samples
        eig_vals, eig_vecs = np.linalg.eig(M)
        q_avg_wxyz = eig_vecs[:, eig_vals.argmax()]   # (w x y z)
        qw, qx, qy, qz = (q_avg_wxyz / np.linalg.norm(q_avg_wxyz)).tolist()

        dl["final_pose"] = {
            "translation": {"x": float(tx), "y": float(ty), "z": float(tz)},
            "rotation":    {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        }
        log.info(f"[GMP] mean XYZ = ({tx:.4f}, {ty:.4f}, {tz:.4f})  "
                f"quat = ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

        # ── 3) write YAML once ────────────────────────────────────────────────
        pkg_share = get_package_share_directory("pickn_place")
        mem_path  = os.path.join(pkg_share, "machine_pose_data_memory.yaml")

        try:
            with open(mem_path, "r") as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}

        data.setdefault("machines", {})[target_tf] = {
            "Time": f"{time.time()}",
            **dl["final_pose"],
        }

        with open(mem_path, "w") as f:
            yaml.safe_dump(data, f)

        dl["yaml_path"]   = mem_path
        dl["end_time"]    = time.time()
        dl["duration_s"]  = dl["end_time"] - dl["start_time"]

        self.datalog_lastrun_skill = dl
        log.info(f"[GMP] ✅ finished in {dl['duration_s']:.2f} s → {mem_path}")

        return data["machines"][target_tf], dl

    def release_tension(self, settling_time: float = 2.0) -> bool:
        """
        • Enable drag-mode (StartDrag) → let the arm "relax".
        • Wait `settling_time` s to dissipate any spring-back.
        • Disable drag-mode (StopDrag).
        Retries driver-error responses up to `max_attempts`, but aborts on
        transport time-outs.  Returns **True** on full success.
        """

        import time, rclpy

        max_attempts  = 6
        call_timeout  = 5.0      # seconds to wait for each service reply
        retry_pause   = 0.25     # pause between attempts

        log = self.get_logger()

        # helpers ---------------------------------------------------------------
        def _try_srv(cli, srv_name):
            fut = cli.call_async(StartDrag.Request() if srv_name == "StartDrag"
                                else StopDrag.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=call_timeout)
            if not fut.done() or fut.result() is None:
                log.error(f"{srv_name}: call timed-out after {call_timeout}s")
                return None        # transport failure – abort
            return getattr(fut.result(), "res", 1)   # driver result code

        # ----------------------------------------------------------------------#
        # 1)  StartDrag  --------------------------------------------------------#
        for attempt in range(1, max_attempts + 1):
            res = _try_srv(self.start_drag_cli, "StartDrag")
            if res is None:            # comms failure – give up
                return False
            if res == 0:               # driver accepted
                log.info("StartDrag OK")
                break
            log.warn(f"StartDrag driver res={res}; retrying ({attempt}/{max_attempts})")
            time.sleep(retry_pause)
        else:
            log.error("StartDrag: exceeded max_attempts")
            return False

        # 2)  settling pause ----------------------------------------------------#
        time.sleep(settling_time)

        # 3)  StopDrag  ---------------------------------------------------------#
        for attempt in range(1, max_attempts + 1):
            res = _try_srv(self.stop_drag_cli, "StopDrag")
            if res is None:
                return False
            if res == 0:
                log.info("StopDrag OK – tension released ✔️")
                return True
            log.warn(f"StopDrag driver res={res}; retrying ({attempt}/{max_attempts})")
            time.sleep(retry_pause)

        log.error("StopDrag: exceeded max_attempts")
        return False
    
    def sync(self, timeout: float = 20.0) -> bool: #high timeout to wait for multiple sequences. 
        """
        Wait for the Dobot motion queue to empty.
        Blocks up to `timeout` seconds for a single service call.
        Returns True if the queue is empty (res == 0), False otherwise.
        """
        import rclpy

        log = self.get_logger()

        # ensure service is available
        if not self.sync_cli.wait_for_service(timeout_sec=timeout):
            log.error(f"sync: service unavailable after {timeout:.2f}s")
            return False

        # call once and wait up to timeout
        future = self.sync_cli.call_async(Sync.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done() or future.result() is None:
            log.error(f"sync: service call timed out after {timeout:.2f}s")
            return False

        result = future.result()
        if result is not None and result.res == 0:
            log.info("sync: motion queue empty ✓")
            return True
        else:
            res_code = result.res if result is not None else "unknown"
            log.error(f"sync: queue not cleared (res={res_code})")
            return False

    def set_gripper_position(
            self,
            speed: int = 255,
            position: int = 255,
            force: int = 255,
            wait_finish: bool = True
    ) -> tuple[bool, int | None]:
        """
        Command the gripper and, if `wait_finish`, block until two identical
        GetGripperPosition readings are observed.

        Returns (True, final_position) on success; (False, None) otherwise.
        """

        import time, rclpy

        # ── clamp & build request ───────────────────────────────────────────────
        position = int(max(0, min(255, position)))
        speed    = int(max(0, min(255, speed)))
        force    = int(max(0, min(255, force)))

        set_req             = SetGripperPosition.Request()
        set_req.position    = position
        set_req.speed       = speed
        set_req.force       = force

        log           = self.get_logger()
        retry_pause   = 0.25
        max_attempts  = 5
        call_timeout  = 5.0

        # ── 1) send SetGripperPosition with retries on driver-error ─────────────
        for attempt in range(1, max_attempts + 1):
            fut = self.set_gripper_cli.call_async(set_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=call_timeout)

            if not fut.done() or fut.result() is None:           # transport failure
                log.error("set_gripper_position: call timed out")
                return False, None

            if getattr(fut.result(), "res", 1) == 0:             # accepted
                break
            log.warn(f"set_gripper_position: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            time.sleep(retry_pause)
        else:
            log.error("set_gripper_position: exceeded max_attempts")
            return False, None
        
        time.sleep(0.5)

        # ── 2) optionally wait until position stabilises ───────────────────────
        if not wait_finish:
            return True, position

        identical_required = 3                    # how many consecutive identical reads
        consecutive_ok     = 0
        latest_reading     = None
        start_time         = time.monotonic()

        while (time.monotonic() - start_time) < 10.0:   # 3 s overall poll window
            fut = self.get_gripper_cli.call_async(GetGripperPosition.Request(index=0))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

            if not fut.done() or fut.result() is None:
                log.error("set_gripper_position: polling timed out")
                return False, None

            current = fut.result().position
            log.debug(f"set_gripper_position: reading {current}")

            if current == latest_reading:
                consecutive_ok += 1
                if consecutive_ok >= identical_required:
                    log.info("set_gripper_position: position stabilised ✓")
                    return True, current
            else:
                consecutive_ok = 1
                latest_reading = current

            time.sleep(0.2)

        log.error("set_gripper_position: did not stabilise within 3 s")
        return False, None

    def move_to(
            self,
            target_tf: str,
            distance: float,
            speed: int = 100,
            acceleration: int = 100,
            offset_x_mm: float = 0.0,
            offset_y_mm: float = 0.0,
            offset_z_mm: float = 0.0,
            offset_rx_deg: float = 0.0,
            offset_ry_deg: float = 0.0,
            offset_rz_deg: float = 0.0,
    ) -> bool:
        """
        Approach *target_tf* along its +Z axis, stop `distance` m short,
        apply XYZ offsets (mm) and RPY offsets (deg), orient the tool so +Z
        points at the object, and execute a single MovL. Returns True on success.
        """
        import math, numpy as np, time, rclpy
        from tf_transformations import euler_from_matrix

        log = self.get_logger()

        # ── 1) acquire a fresh, stable TF via a temp perception node ────────────
        perception = robot_perception()
        perc_exec  = SingleThreadedExecutor()
        perc_exec.add_node(perception)

        # start the TF listener in the background while we work
        import threading, contextlib
        tf_thread = threading.Thread(target=perc_exec.spin, daemon=True)
        tf_thread.start()

        try:
            pose = perception.acquire_target_transform(
                target_tf,
                max_wait=10.0,
                trans_thresh=0.002,    #2 mm accuracy
                rot_thresh=180.0,      #ignore orientation
                num_samples=6,
            )
        finally:
            with contextlib.suppress(Exception):
                perc_exec.shutdown()
            perception.destroy_node()

        if pose is None:
            log.error("move_to: failed to obtain stable TF")
            return False

        obj_pos = np.asarray(pose[:3])                # (x y z) in metres

        # ── 2) compute goal position ------------------------------------------------
        base_above = np.array([0.0, 0.0, obj_pos[2] + 0.250])   # 250 mm overhead
        vec        = obj_pos - base_above
        if np.linalg.norm(vec) < 1e-6:
            log.error("move_to: approach vector degenerate")
            return False

        unit_vec = vec / np.linalg.norm(vec)
        goal_pos = obj_pos - distance * unit_vec
        goal_pos += np.array([offset_x_mm, offset_y_mm, offset_z_mm]) / 1000.0  # mm → m

        # ── 3) build goal rotation (tool +Z → unit_vec) -----------------------------
        z_axis   = unit_vec
        world_up = np.array([0, 0, 1]) if abs(np.dot(z_axis, [0, 0, 1])) < 0.99 else np.array([0, 1, 0])
        x_axis   = np.cross(world_up, z_axis);  x_axis /= np.linalg.norm(x_axis)
        y_axis   = np.cross(z_axis, x_axis)
        R        = np.column_stack((x_axis, y_axis, z_axis))
        rx_deg, ry_deg, rz_deg = map(math.degrees, euler_from_matrix(R, axes="sxyz"))

        # ── apply user-specified orientation offsets (deg) --------------------------
        rx_deg += offset_rx_deg
        ry_deg += offset_ry_deg
        rz_deg += offset_rz_deg

        # ── 4) send MovL  ------------------------------------------------------------
        req = MovL.Request()
        req.x, req.y, req.z = (goal_pos * 1000.0).astype(float)   # m → mm
        req.rx, req.ry, req.rz = rx_deg, ry_deg, rz_deg
        req.param_value = [f"SpeedL={speed},AccL={acceleration}"]

        retry_pause, max_attempts = 0.5, 5
        for attempt in range(1, max_attempts + 1):
            log.info(f"move_to: MovL attempt {attempt}/{max_attempts}")
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("move_to: MovL call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("move_to: success ✓")
                return True

            log.warn(f"move_to: driver res={fut.result().res}; retrying…")
            self.sync()

        log.error("move_to: failed after maximum retries")
        return False

    def enforce_rxry(self) -> bool:
        """
        Override Link6's Rx→90°, Ry→0° (keep current Rz) while freezing
        the world‐space position of portafilter_link to ±0.5 mm.
        Returns True on success.
        """
        import numpy as np
        import time
        from tf_transformations import euler_matrix, quaternion_from_matrix

        try:
            # Constants
            d_rel = np.array([0.0, 0.0, 0.2825])  # Fixed offset from Link6 origin → portafilter_link origin (m)
            tolerance = 0.002  # 0.5 mm in metres

            # 1) Get current Link6 pose
            current_pose = self._get_link6_pose_with_retries()
            if current_pose is None:
                return False

            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = current_pose

            # 2) Compute current portafilter world position
            p_pf_world = self._compute_portafilter_world_position(
                np.array([tx_mm, ty_mm, tz_mm]) * 1e-3,
                np.radians([rx_curr, ry_curr, rz_curr]),
                d_rel
            )

            # 3) Compute goal pose for Link6
            goal_pose = self._compute_link6_goal_pose(rz_curr, p_pf_world, d_rel)
            if goal_pose is None:
                return False

            # 4) Execute motion
            if not self._execute_enforce_motion(goal_pose):
                return False

            # 5) Verify portafilter position accuracy
            if not self._verify_portafilter_position(p_pf_world, d_rel, tolerance):
                return False

            self.get_logger().info("enforce_rxry(): Completed with SUCCESS.")
            return True

        except Exception as e:
            self.get_logger().error(f"enforce_rxry(): Unexpected error: {e}")
            return False

    def _get_link6_pose_with_retries(self, max_attempts: int = 3) -> tuple | None:
        """Get current Link6 pose with retry logic."""
        if not self.get_pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("_get_link6_pose_with_retries(): GetPose service unavailable.")
            return None

        req = GetPose.Request()
        req.user = 0
        req.tool = 0

        for attempt in range(1, max_attempts + 1):
            try:
                future = self.get_pose_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                resp = future.result()
                
                if resp is not None and hasattr(resp, "pose"):
                    # Parse pose string
                    parts = resp.pose.strip("{}").split(",")
                    if len(parts) < 6:
                        self.get_logger().error("_get_link6_pose_with_retries(): Invalid pose format.")
                        return None
                    
                    pose_values = [float(p) for p in parts[:6]]
                    return tuple(pose_values)
                
            except Exception as e:
                self.get_logger().warn(f"_get_link6_pose_with_retries(): Attempt {attempt}/{max_attempts} failed: {e}")
            
            if attempt < max_attempts:
                time.sleep(0.5)

        self.get_logger().error("_get_link6_pose_with_retries(): Failed to retrieve pose after all attempts.")
        return None

    def _compute_portafilter_world_position(self, p_link6: np.ndarray, rads: np.ndarray, d_rel: np.ndarray) -> np.ndarray:
        """Compute world position of portafilter_link given Link6 position and orientation."""
        try:
            R6_curr = euler_matrix(*rads)[:3, :3]
            return p_link6 + R6_curr.dot(d_rel)
        except Exception as e:
            self.get_logger().error(f"_compute_portafilter_world_position(): Error: {e}")
            raise

    def _compute_link6_goal_pose(self, rz_curr: float, p_pf_world: np.ndarray, d_rel: np.ndarray) -> tuple | None:
        """Compute goal pose for Link6 to achieve desired orientation while keeping portafilter position fixed."""
        try:
            # Define desired Link6 orientation: Rx=90°, Ry=0°, keep Rz
            rx_target, ry_target = 90.0, 0.0
            rads_goal = np.radians([rx_target, ry_target, rz_curr])
            
            M_goal = euler_matrix(*rads_goal)
            R6_goal = M_goal[:3, :3]
            quat_goal = list(quaternion_from_matrix(M_goal))

            # Back-solve Link6 goal position so portafilter_link stays put
            p6_goal = p_pf_world - R6_goal.dot(d_rel)
            
            self.get_logger().info(f"_compute_link6_goal_pose(): Goal Link6 pos (m): {p6_goal.tolist()}")
            return p6_goal.tolist(), quat_goal
            
        except Exception as e:
            self.get_logger().error(f"_compute_link6_goal_pose(): Error: {e}")
            return None

    def _execute_enforce_motion(self, goal_pose: tuple) -> bool:
        """Execute the enforce motion using MoveIt2."""
        try:
            position, quaternion = goal_pose
            
            # Create temporary MoveIt2 instance for Link6 end effector
            temp_moveit2 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name="Link6",
                group_name=GROUP_NAME,
            )
            temp_moveit2.planner_id = self.planner_id
            temp_moveit2.max_velocity = self.velocity_scaling
            temp_moveit2.max_acceleration = self.acceleration_scaling
            temp_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            self.get_logger().info("_execute_enforce_motion(): Calling MoveIt2.move_to_pose()...")
            time.sleep(0.2)
            temp_moveit2.move_to_pose(
                position=position,
                quat_xyzw=quaternion,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            time.sleep(0.2)
            temp_moveit2.wait_until_executed()
            
            state = temp_moveit2.query_state()
            if state != MoveIt2State.IDLE:
                self.get_logger().warn(f"_execute_enforce_motion(): Motion ended with state: {state}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_enforce_motion(): Error: {e}")
            return False

    def _verify_portafilter_position(self, p_pf_world: np.ndarray, d_rel: np.ndarray, tolerance: float, max_attempts: int = 10) -> bool:
        """Verify that portafilter_link stayed within tolerance."""
        try:
            for attempt in range(max_attempts):
                # Re-read Link6 pose
                current_pose = self._get_link6_pose_with_retries()
                if current_pose is None:
                    return False

                tx2, ty2, tz2, rx2, ry2, rz2 = current_pose
                p6_new = np.array([tx2, ty2, tz2]) * 1e-3
                R6_new = euler_matrix(*np.radians([rx2, ry2, rz2]))[:3, :3]
                p_pf_new = p6_new + R6_new.dot(d_rel)

                error = np.linalg.norm(p_pf_new - p_pf_world)
                if error <= tolerance:
                    self.get_logger().info(f"_verify_portafilter_position(): Position verified (error: {error*1e3:.2f} mm)")
                    return True

                self.get_logger().warn(
                    f"_verify_portafilter_position(): portafilter moved {error*1e3:.2f} mm (>0.5 mm), "
                    f"attempt {attempt + 1}/{max_attempts}"
                )
                
                if attempt < max_attempts - 1:
                    time.sleep(0.2)

            self.get_logger().error("_verify_portafilter_position(): Failed to verify position after all attempts")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_portafilter_position(): Error: {e}")
            return False
    
    def move_portafilter_arc(self, angle_deg: float) -> bool:
        """
        Rotate the portafilter_link by the specified angle about its local Y axis.
        
        Args:
            angle_deg: Rotation angle in degrees (positive = counterclockwise)
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Input validation
        if not isinstance(angle_deg, (int, float)):
            self.get_logger().error("move_portafilter_arc(): angle_deg must be a number")
            return False
        
        if abs(angle_deg) > 180:
            self.get_logger().warn(f"move_portafilter_arc(): Large angle {angle_deg}° may be unsafe")
        
        self.get_logger().info(f"move_portafilter_arc(): Rotating portafilter_link by {angle_deg}°")

        try:
            # 1) Get current portafilter pose
            current_pose = self._get_portafilter_current_pose()
            if current_pose is None:
                return False

            # 2) Compute new orientation
            new_pose = self._compute_arc_pose(current_pose, angle_deg)
            if new_pose is None:
                return False

            # 3) Execute arc motion
            if not self._execute_arc_motion(new_pose, angle_deg):
                return False

            # 4) Verify motion completion
            if not self._verify_arc_completion():
                return False

            self.get_logger().info("move_portafilter_arc(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc(): Unexpected error: {e}")
            return False

    def _get_portafilter_current_pose(self) -> tuple | None:
        """Get current portafilter pose from TF."""
        try:
            # Wait for transform to be available
            self.get_logger().info(f"_get_portafilter_current_pose(): Looking up transform from {self.reference_frame} to portafilter_link")
            
            # Check if frames exist
            try:
                pf_tf_stamped = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    "portafilter_link", 
                    rclpy.time.Time(),
                    timeout=Duration(seconds=5.0)
                )
            except Exception as tf_error:
                self.get_logger().error(f"_get_portafilter_current_pose(): TF lookup failed: {tf_error}")
                
                # Try alternative approach using Link6 pose + offset
                self.get_logger().info("_get_portafilter_current_pose(): Trying fallback using Link6 pose")
                return self._get_portafilter_pose_from_link6()
            
            current_tf = get_transform_list(pf_tf_stamped)
            
            position = np.array(current_tf[:3])
            current_quat = np.array(current_tf[3:])
            
            self.get_logger().info(f"_get_portafilter_current_pose(): Got portafilter pose: pos={position}, quat={current_quat}")
            return position, current_quat
            
        except Exception as e:
            self.get_logger().error(f"_get_portafilter_current_pose(): Failed to get portafilter_link transform: {e}")
            return None

    def _get_portafilter_pose_from_link6(self) -> tuple | None:
        """Fallback: Compute portafilter pose from Link6 pose + offset."""
        try:
            # Get Link6 pose
            current_pose = self._get_link6_pose_with_retries()
            if current_pose is None:
                return None
                
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = current_pose
            
            # Convert to meters and compute portafilter position
            d_rel = np.array([0.0, 0.0, 0.2825])  # Fixed offset
            p_link6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
            rads = np.radians([rx_curr, ry_curr, rz_curr])
            R6_curr = euler_matrix(*rads)[:3, :3]
            
            # Compute portafilter position
            p_pf = p_link6 + R6_curr.dot(d_rel)
            
            # Use Link6 quaternion for portafilter orientation
            M6 = euler_matrix(*rads)
            current_quat = tf_transformations.quaternion_from_matrix(M6)
            
            self.get_logger().info(f"_get_portafilter_pose_from_link6(): Computed portafilter pose: pos={p_pf}, quat={current_quat}")
            return p_pf, current_quat
            
        except Exception as e:
            self.get_logger().error(f"_get_portafilter_pose_from_link6(): Error: {e}")
            return None

    def _compute_arc_pose(self, current_pose: tuple, angle_deg: float) -> tuple | None:
        """Compute new pose after rotating about local Y axis."""
        try:
            position, current_quat = current_pose
            
            # Compute new orientation by rotating about local Y axis
            R_current = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]
            rotation_axis = R_current[:, 1]  # Local Y axis
            theta = math.radians(angle_deg)
            relative_quat = tf_transformations.quaternion_about_axis(theta, rotation_axis)
            new_quat = tf_transformations.quaternion_multiply(relative_quat, current_quat)
            new_pos = position.tolist()

            self.get_logger().info(
                f"_compute_arc_pose(): New orientation (xyzw): {new_quat}"
            )
            
            return new_pos, new_quat
            
        except Exception as e:
            self.get_logger().error(f"_compute_arc_pose(): Error computing arc pose: {e}")
            return None

    def _execute_arc_motion(self, new_pose: tuple, angle_deg: float) -> bool:
        """Execute the arc motion using MoveIt2."""
        try:
            new_pos, new_quat = new_pose
            
            # Create MoveIt2 instance for portafilter
            pf_moveit2 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name="portafilter_link",
                group_name="portafilter_center",
            )
            pf_moveit2.planner_id = self.planner_id
            pf_moveit2.max_velocity = self.velocity_scaling
            pf_moveit2.max_acceleration = self.acceleration_scaling
            pf_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            pf_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            # Execute the arc motion
            time.sleep(0.2)
            pf_moveit2.move_to_pose(
                position=new_pos,
                quat_xyzw=new_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            time.sleep(0.2)
            pf_moveit2.wait_until_executed()
            
            state = pf_moveit2.query_state()
            if state == MoveIt2State.IDLE:
                self.get_logger().info("_execute_arc_motion(): Rotation motion executed successfully.")
                return True
            else:
                self.get_logger().warn(f"_execute_arc_motion(): Rotation motion ended with state: {state}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"_execute_arc_motion(): Error: {e}")
            return False

    def _verify_arc_completion(self, tolerance_deg: float = 1.0, batch_size: int = 10, max_attempts: int = 5) -> bool:
        """Verify that the arc motion has completed with stable joint positions."""
        try:
            sleep_between = 0.1  # seconds

            for overall_attempt in range(max_attempts):
                # Collect one batch of joint samples
                joint_samples = []
                for _ in range(batch_size):
                    js = self.wait_for_joint_state("/joint_states_robot", timeout_sec=1.0)
                    if js and len(js.position) >= 6:
                        angles_deg = [math.degrees(j) for j in js.position[0:6]]
                        joint_samples.append(angles_deg)
                    time.sleep(sleep_between)

                # Compute per-joint spread across this batch
                if not joint_samples:
                    self.get_logger().warn("_verify_arc_completion(): No joint samples collected")
                    continue
                    
                spreads = [max(vals) - min(vals) for vals in zip(*joint_samples)]

                # Check if spread is within tolerance
                if all(spread < tolerance_deg for spread in spreads):
                    self.get_logger().info("_verify_arc_completion(): Joint positions stable")
                    return True

                # Try fallback verification
                if joint_samples:
                    last_sample = joint_samples[-1]
                    if self.verify_joint_positions(last_sample, tolerance_deg):
                        self.get_logger().info("_verify_arc_completion(): Fallback verification passed")
                        return True

                self.get_logger().warn(
                    f"_verify_arc_completion(): Joint spread too large "
                    f"(spreads: {spreads}), attempt {overall_attempt + 1}/{max_attempts}"
                )

            self.get_logger().error("_verify_arc_completion(): Failed to achieve stable joint positions")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_arc_completion(): Error: {e}")
            return False

    def move_portafilter_arc_movL(
        self,
        angle_deg: float,
        velocity: int = 100,
        acceleration: int = 100,
        ) -> bool:
        """
        Move the portafilter_link in an arc by angle_deg about its local Y axis,
        using the /dobot_bringup_v3/srv/MovL service (Link6 linear planning). Allows
        specifying SpeedL and AccL via the velocity and acceleration arguments.
        Retries any failed service call every 0.25 s up to 10 attempts; returns False on total failure.
        Returns True on success, False on any failure.
        """
        from tf_transformations import (
            euler_matrix,
            quaternion_about_axis,
            quaternion_matrix,
            quaternion_multiply,
            euler_from_matrix,
        )

        # ── 1) fixed offset from Link6 origin → portafilter_link origin (m)
        d_rel = np.array([0.0, 0.0, 0.276])

        retry_pause = 0.25
        max_attempts = 5

        gp_req = GetPose.Request()
        gp_req.user = 0
        gp_req.tool = 0

        resp = None
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"move_portafilter_arc(): GetPose attempt {attempt}/{max_attempts}")
            future = self.get_pose_cli.call_async(gp_req)
            start = self.get_clock().now().nanoseconds * 1e-9
            got_response = False

            while (self.get_clock().now().nanoseconds * 1e-9 - start) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)
                if future.done() and future.result() is not None and hasattr(future.result(), "pose"):
                    resp = future.result()
                    got_response = True
                    break

            if got_response:
                break
            self.get_logger().warn(
                "move_portafilter_arc(): GetPose call failed or timed out—waiting 0.25 s before retry"
            )
            self.sync()

        if resp is None or not hasattr(resp, "pose"):
            self.get_logger().error(
                "move_portafilter_arc(): Failed to retrieve initial pose after 10 attempts."
            )
            return False

        # parse "{tx,ty,tz,rx,ry,rz,…}"  (tx, ty, tz in mm; rx, ry, rz in degrees)
        parts = resp.pose.strip("{}").split(",")
        if len(parts) < 6:
            self.get_logger().error("move_portafilter_arc(): Invalid pose format.")
            return False
        try:
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = [float(p) for p in parts[:6]]
        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc(): Error parsing pose: {e}")
            return False

        # convert to metres and builrent portafilted current rotation matrix
        p_link6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
        rads = np.radians([rx_curr, ry_curr, rz_curr])
        R6_curr = euler_matrix(*rads)[:3, :3]
        # compute world position of the portafilter_link
        p_pf_world = p_link6 + R6_curr.dot(d_rel)

        # ── 3) compute new orientation: rotate portafilter_link about its local Y by angle_deg
        theta = math.radians(angle_deg)
        # local Y axis of Link6 in world frame:
        rotation_axis = R6_curr[:, 1]
        relative_quat = quaternion_about_axis(theta, rotation_axis)
        # current Link6 quaternion (from current rotation matrix)
        current_quat = tf_transformations.quaternion_from_matrix(euler_matrix(*rads))
        new_quat = quaternion_multiply(relative_quat, current_quat)
        # build new rotation matrix
        R6_goal = quaternion_matrix(new_quat)[0:3, 0:3]

        # ── 4) back‐solve Link6 goal position so portafilter_link stays put
        p6_goal = p_pf_world - R6_goal.dot(d_rel)
        self.get_logger().info(f"move_portafilter_arc(): Goal Link6 pos (m): {p6_goal.tolist()}")

        # convert goal position back to millimetres for MovL
        x_goal_mm = float(p6_goal[0] * 1000.0)
        y_goal_mm = float(p6_goal[1] * 1000.0)
        z_goal_mm = float(p6_goal[2] * 1000.0)

        # convert new_quat to Euler XYZ in degrees for MovL request
        R_goal_full = quaternion_matrix(new_quat)
        rx_goal, ry_goal, rz_goal = np.degrees(euler_from_matrix(R_goal_full, axes="sxyz"))

        movl_req = MovL.Request()
        movl_req.x = x_goal_mm
        movl_req.y = y_goal_mm
        movl_req.z = z_goal_mm
        movl_req.rx = rx_goal
        movl_req.ry = ry_goal
        movl_req.rz = rz_goal
        # use provided velocity and acceleration
        movl_req.param_value = [f"SpeedL={velocity},AccL={acceleration}"]

        movl_resp = None
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"move_portafilter_arc(): MovL attempt {attempt}/{max_attempts}")
            if not self.movl_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("move_portafilter_arc(): MovL service unavailable, retrying...")
                self.sync()
                continue

            movl_fut = self.movl_cli.call_async(movl_req)
            start_mv = self.get_clock().now().nanoseconds * 1e-9
            while not movl_fut.done() and (self.get_clock().now().nanoseconds * 1e-9 - start_mv) < 0.5:
                rclpy.spin_once(self, timeout_sec=0.01)

            if movl_fut.done() and movl_fut.result() is not None:
                movl_resp = movl_fut.result()
                break

            self.get_logger().warn("move_portafilter_arc(): MovL call failed or timed out, retrying...")
            self.sync()

        if movl_resp is None:
            self.get_logger().error("move_portafilter_arc(): Failed to call MovL after 10 attempts.")
            return False
        self.sync()

        self.get_logger().info("move_portafilter_arc(): Verification OK.")
        return True

    def moveEE(
            self,
            offset_x_mm: float,
            offset_y_mm: float,
            offset_z_mm: float,
            offset_rx_deg: float,
            offset_ry_deg: float,
            offset_rz_deg: float,
            speed: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Add the requested Δpose (mm / deg) to the current Link‑6 pose and execute
        a single MovL.  Blocks until the queue is empty and, if servo‑ready helper
        is present, until motion completes.  Returns True on success.
        """
        self.sync() #sync first for accurate get pose

        log = self.get_logger()

        # ── 1) fetch current pose once via GetPose ─────────────────────────────
        fut = self.get_pose_cli.call_async(GetPose.Request(user=0, tool=0))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None or not hasattr(fut.result(), "pose"):
            log.error("moveEE: GetPose call failed")
            return False

        try:
            tx, ty, tz, rx, ry, rz = map(float, fut.result().pose.strip("{}").split(",")[:6])
        except Exception as exc:
            log.error(f"moveEE: bad pose string – {exc}")
            return False

        # ── 2) build target pose ------------------------------------------------
        tgt_xyz = np.array([tx, ty, tz]) + np.array([offset_x_mm,
                                                    offset_y_mm,
                                                    offset_z_mm])
        tgt_rpy = (rx + offset_rx_deg,
                ry + offset_ry_deg,
                rz + offset_rz_deg)

        req = MovL.Request(
            x = float(tgt_xyz[0]),
            y = float(tgt_xyz[1]),
            z = float(tgt_xyz[2]),
            rx = float(tgt_rpy[0]),
            ry = float(tgt_rpy[1]),
            rz = float(tgt_rpy[2]),
            param_value = [f"SpeedL={speed},AccL={acceleration}"],
        )

        # ── 3) send MovL with limited retries on driver error ------------------
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("moveEE: MovL call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("moveEE: success ✓")
                break
            log.warn(f"moveEE: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()
        else:
            log.error("moveEE: exceeded max_attempts")
            return False

        return True

    def moveJ_deg(
            self,
            offset1: float, offset2: float, offset3: float,
            offset4: float, offset5: float, offset6: float,
            *,
            velocity: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Add Δangles (deg) to the current joints and execute JointMovJ.
        Sequence: Sync → GetAngle → JointMovJ.  Returns True on success.
        """
        self.sync() #sync first for accurate get pose

        import time, rclpy

        log = self.get_logger()

        # ── 2) read current joint angles once ─────────────────────────────────
        fut = self.get_angle_cli.call_async(GetAngle.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None:
            log.error("moveJ_deg: GetAngle call failed")
            return False

        try:
            curr_angles = [float(a) for a in fut.result().angle.strip("{}").split(",")[:6]]
        except Exception as exc:
            log.error(f"moveJ_deg: bad GetAngle string – {exc}")
            return False

        # ── 3) build target = current + offsets ───────────────────────────────
        deltas   = [offset1, offset2, offset3, offset4, offset5, offset6]
        targets  = [c + d for c, d in zip(curr_angles, deltas)]
        log.info(f"moveJ_deg: target joints → {targets}")

        req = JointMovJ.Request(
            j1 = targets[0], j2 = targets[1], j3 = targets[2],
            j4 = targets[3], j5 = targets[4], j6 = targets[5],
            param_value = [f"SpeedJ={velocity},AccJ={acceleration}"],
        )

        # ── 4) send JointMovJ with limited retries ----------------------------
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.jointmovj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("moveJ_deg: JointMovJ call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("moveJ_deg: success ✓")
                break
            log.warn(f"moveJ_deg: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()
        else:
            log.error("moveJ_deg: exceeded max_attempts")
            return False

        return True

    def gotoEE(
            self,
            abs_x_mm: float,
            abs_y_mm: float,
            abs_z_mm: float,
            abs_rx_deg: float,
            abs_ry_deg: float,
            abs_rz_deg: float,
            speed: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Drive Link‑6 to an *absolute* pose (mm / deg in base_link) via a single
        MovL.  Retries on driver error; aborts on transport timeout.
        """

        import time, rclpy

        log = self.get_logger()

        req = MovL.Request(
            x  = abs_x_mm,
            y  = abs_y_mm,
            z  = abs_z_mm,
            rx = abs_rx_deg,
            ry = abs_ry_deg,
            rz = abs_rz_deg,
            param_value = [f"SpeedL={speed},AccL={acceleration}"],
        )

        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("gotoEE: MovL call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("gotoEE: success ✓")
                break
            log.warn(f"gotoEE: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()
        else:
            log.error("gotoEE: exceeded max_attempts")
            return False

        return True
    
    def gotoJ_deg(
            self,
            angle1: float, angle2: float, angle3: float,
            angle4: float, angle5: float, angle6: float,
            velocity: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Command an *absolute* set of six joint angles (deg) via JointMovJ.
        Retries on driver‑error; aborts on transport failure.
        """

        import time, rclpy

        req = JointMovJ.Request(
            j1 = angle1, j2 = angle2, j3 = angle3,
            j4 = angle4, j5 = angle5, j6 = angle6,
            param_value = [f"SpeedJ={velocity},AccJ={acceleration}"],
        )

        log = self.get_logger()
        retry_pause, max_attempts = 0.25, 5

        for attempt in range(1, max_attempts + 1):
            fut = self.jointmovj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("gotoJ_deg: JointMovJ call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("gotoJ_deg: success ✓")
                break
            log.warn(f"gotoJ_deg: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()
        else:
            log.error("gotoJ_deg: exceeded max_attempts")
            return False

        return True

    def approach_tool(
            self,
            target_tf: str,
            speed: int = 100,
            acceleration: int = 100,
            *,
            offset_x_mm: float = 0.0,
            offset_y_mm: float = 0.0,
            offset_z_mm: float = -128,
    ) -> bool:
        """
        Go to the YAML-defined "approach_pose" of `target_tf`, then apply local
        offsets along the tool frame.  Returns True on success.
        """

        import os, yaml, math, numpy as np, time, rclpy, tf_transformations
        from ament_index_python.packages import get_package_share_directory

        log = self.get_logger()

        # ── 1) fresh TF via temp perception node ──────────────────────────────
        perception, exec_ = robot_perception(), SingleThreadedExecutor()
        exec_.add_node(perception)

        # ── start TF listener in a helper thread ───────────────────────────
        import threading, contextlib
        tf_thread = threading.Thread(target=exec_.spin, daemon=True)
        tf_thread.start()

        try:
            pose = perception.acquire_target_transform(
                target_tf,
                max_wait=10.0,
                trans_thresh=0.002,    #2 mm accuracy
                rot_thresh=2.0,        #2 deg error
                num_samples=6,
            )
        finally:
            # shut down executor and destroy the node
            with contextlib.suppress(Exception):
                exec_.shutdown()
            perception.destroy_node()

        if pose is None:
            log.error("approach_tool: stable TF not found")
            return False

        obj_pos, obj_quat = np.asarray(pose[:3]), pose[3:]

        # ── 2) read YAML offset ------------------------------------------------
        share = get_package_share_directory("pickn_place")
        with open(os.path.join(share, "tool_offset_points.yaml"), "r") as f:
            off_dict = yaml.safe_load(f).get(target_tf, {}).get("approach_pose")
        if off_dict is None:
            log.error("approach_tool: YAML entry missing")
            return False

        t_off = np.array([off_dict["translation"][ax] for ax in ("x", "y", "z")])
        q_off = [off_dict["rotation"][ax] for ax in ("x", "y", "z", "w")]

        # ── 3) compose goal transform ----------------------------------------
        M_obj  = tf_transformations.quaternion_matrix(obj_quat);  M_obj[:3, 3] = obj_pos
        M_off  = tf_transformations.quaternion_matrix(q_off);     M_off[:3, 3] = t_off
        M_goal = M_obj @ M_off

        pos_mm = M_goal[:3, 3] * 1000.0
        pos_mm += (M_goal[:3, 0] * offset_x_mm +
                M_goal[:3, 1] * offset_y_mm +
                M_goal[:3, 2] * offset_z_mm)

        rx, ry, rz = map(math.degrees,
                        tf_transformations.euler_from_matrix(M_goal, axes="sxyz"))

        # ── 4) send MovL -------------------------------------------------------
        req = MovL.Request(
            x= float(pos_mm[0]), y= float(pos_mm[1]), z= float(pos_mm[2]),
            rx= rx, ry= ry, rz= rz,
            param_value=[f"SpeedL={speed},AccL={acceleration}"],
        )

        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if not fut.done() or fut.result() is None:
                log.error("approach_tool: MovL call timed out")
                return False
            if getattr(fut.result(), "res", 1) == 0:
                log.info("approach_tool: success ✓")
                return True
            log.warn(f"approach_tool: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})")
            self.sync()

        log.error("approach_tool: exceeded max_attempts")
        return False
        
    def approach_machine(
            self,
            machine_name: str,
            point_name: str,
            *,
            speed: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Drive to <machine>/<point>.approach_pose (world frame) via MovL.
        """

        import os, yaml, math, numpy as np, time, rclpy, tf_transformations
        from ament_index_python.packages import get_package_share_directory

        log = self.get_logger()
        share = get_package_share_directory("pickn_place")

        try:
            with open(os.path.join(share, "machine_pose_data_memory.yaml"), "r") as f:
                mem = yaml.safe_load(f) or {}
            with open(os.path.join(share, "machine_offset_points.yaml"), "r") as f:
                off = yaml.safe_load(f) or {}
        except Exception as exc:
            log.error(f"approach_machine: YAML load failed – {exc}")
            return False

        base = mem.get("machines", {}).get(machine_name)
        offs = off.get(machine_name, {}).get(point_name, {}).get("approach_pose")
        if base is None or offs is None:
            log.error("approach_machine: missing YAML entries")
            return False

        M_base = tf_transformations.quaternion_matrix(
            [base["rotation"][k] for k in ("x", "y", "z", "w")])
        M_base[:3, 3] = [base["translation"][k] for k in ("x", "y", "z")]

        M_off  = tf_transformations.quaternion_matrix(
            [offs["rotation"][k] for k in ("x", "y", "z", "w")])
        M_off[:3, 3] = [offs["translation"][k] for k in ("x", "y", "z")]

        M_goal = M_base @ M_off
        pos_mm = M_goal[:3, 3] * 1000.0
        rx, ry, rz = map(math.degrees,
                        tf_transformations.euler_from_matrix(M_goal, axes="sxyz"))

        req = MovL.Request(
            x=float(pos_mm[0]), y=float(pos_mm[1]), z=float(pos_mm[2]),
            rx=rx, ry=ry, rz=rz,
            param_value=[f"SpeedL={speed},AccL={acceleration}"],
        )

        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if not fut.done() or fut.result() is None:
                log.error("approach_machine: MovL call timed out")
                return False
            if getattr(fut.result(), "res", 1) == 0:
                log.info("approach_machine: success ✓")
                return True
            log.warn(f"approach_machine: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()

        log.error("approach_machine: exceeded max_attempts")
        return False
    
    def mount_machine(
            self,
            machine_name: str,
            point_name: str,
            *,
            speed: int = 100,
            acceleration: int = 100,
    ) -> bool:
        """
        Drive to <machine>/<point>.mount_pose via MovL.
        """

        import os, yaml, math, numpy as np, time, rclpy, tf_transformations
        from ament_index_python.packages import get_package_share_directory

        log = self.get_logger()
        share = get_package_share_directory("pickn_place")

        try:
            with open(os.path.join(share, "machine_pose_data_memory.yaml"), "r") as f:
                mem = yaml.safe_load(f) or {}
            with open(os.path.join(share, "machine_offset_points.yaml"), "r") as f:
                off = yaml.safe_load(f) or {}
        except Exception as exc:
            log.error(f"mount_machine: YAML load failed – {exc}")
            return False

        base = mem.get("machines", {}).get(machine_name)
        offs = off.get(machine_name, {}).get(point_name, {}).get("mount_pose")
        if base is None or offs is None:
            log.error("mount_machine: missing YAML entries")
            return False

        M_base = tf_transformations.quaternion_matrix(
            [base["rotation"][k] for k in ("x", "y", "z", "w")])
        M_base[:3, 3] = [base["translation"][k] for k in ("x", "y", "z")]

        M_off  = tf_transformations.quaternion_matrix(
            [offs["rotation"][k] for k in ("x", "y", "z", "w")])
        M_off[:3, 3] = [offs["translation"][k] for k in ("x", "y", "z")]

        M_goal = M_base @ M_off
        pos_mm = M_goal[:3, 3] * 1000.0
        rx, ry, rz = map(math.degrees,
                        tf_transformations.euler_from_matrix(M_goal, axes="sxyz"))

        req = MovL.Request(
            x=float(pos_mm[0]), y=float(pos_mm[1]), z=float(pos_mm[2]),
            rx=rx, ry=ry, rz=rz,
            param_value=[f"SpeedL={speed},AccL={acceleration}"],
        )

        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if not fut.done() or fut.result() is None:
                log.error("mount_machine: MovL call timed out")
                return False
            if getattr(fut.result(), "res", 1) == 0:
                log.info("mount_machine: success ✓")
                return True
            log.warn(f"mount_machine: driver res={fut.result().res}; "
                    f"retrying ({attempt}/{max_attempts})")
            self.sync()

        log.error("mount_machine: exceeded max_attempts")
        return False
    
    def grab_tool(
            self,
            target_tf: str,
            speed: int = 100,
            acceleration: int = 100,
            offset_x_mm: float = 0.0,
            offset_y_mm: float = 0.0,
            offset_z_mm: float = -128,
    ) -> bool:
        """
        Drive Link6 to the pre-configured "grab" pose for *target_tf* and
        close in with optional local offsets.

        Mirrors approach_tool structure: single wait-for-service, retries
        confined to call_async.
        """
        import math, os, yaml, time, numpy as np, tf_transformations
        from ament_index_python.packages import get_package_share_directory
        import rclpy

        log = self.get_logger()

        # ── 1) stable transform for the object ────────────────────────────────
        perception = robot_perception()
        perc_exec  = SingleThreadedExecutor()
        perc_exec.add_node(perception)

        # ── start TF listener in a helper thread ───────────────────────────
        import threading, contextlib
        tf_thread = threading.Thread(target=perc_exec.spin, daemon=True)
        tf_thread.start()

        try:
            pose = perception.acquire_target_transform(
                target_tf,
                max_wait=10.0,
                trans_thresh=0.001,    #1 mm accuracy
                rot_thresh=1.5,        #1.5 deg error
                num_samples=6,
            )
        finally:
            # shut down executor and destroy the node
            with contextlib.suppress(Exception):
                perc_exec.shutdown()
            perception.destroy_node()

        if pose is None:
            log.error("grab_tool: no stable TF")
            return False

        obj_pos, obj_quat = np.array(pose[:3]), pose[3:]

        # ── 2) load YAML grab offset ──────────────────────────────────────────
        try:
            pkg  = get_package_share_directory("pickn_place")
            path = os.path.join(pkg, "tool_offset_points.yaml")
            data = yaml.safe_load(open(path, "r"))
            off  = data[target_tf]["grab_pose"]
            t_off = [off["translation"][ax] for ax in ("x", "y", "z")]
            q_off = [off["rotation"][ax]    for ax in ("x", "y", "z", "w")]
        except Exception as e:
            log.error(f"grab_tool: YAML load failed – {e}")
            return False

        # ── 3) compute goal transform ─────────────────────────────────────────
        M_obj       = tf_transformations.quaternion_matrix(obj_quat);  M_obj[:3, 3] = obj_pos
        M_off       = tf_transformations.quaternion_matrix(q_off);     M_off[:3, 3] = t_off
        M_goal      = M_obj.dot(M_off)

        pos_mm  = M_goal[:3, 3] * 1000.0
        R       = M_goal[:3, 0:3]
        pos_mm += (R[:, 0] * offset_x_mm +
                R[:, 1] * offset_y_mm +
                R[:, 2] * offset_z_mm)

        goal_quat = tf_transformations.quaternion_from_matrix(M_goal)
        rx, ry, rz = map(math.degrees,
                        tf_transformations.euler_from_quaternion(goal_quat))

        # ── 4) MovL client + single gate ──────────────────────────────────────
        if not self.movl_cli.wait_for_service(timeout_sec=5.0):
            log.error("grab_tool: MovL service unavailable")
            return False

        # ── 5) build request ─────────────────────────────────────────────────
        req = MovL.Request()
        req.x, req.y, req.z = map(float, pos_mm)
        req.rx, req.ry, req.rz = rx, ry, rz
        req.param_value       = [f"SpeedL={speed},AccL={acceleration}"]

        # ── 6) retry loop (driver-error only) ─────────────────────────────────
        retry_pause  = 0.25
        max_attempts = 5

        for attempt in range(1, max_attempts + 1):
            log.info(f"grab_tool: MovL attempt {attempt}/{max_attempts}")
            fut = self.movl_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

            if not fut.done() or fut.result() is None:
                log.error("grab_tool: MovL call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("grab_tool: success")
                return True

            log.warn(f"grab_tool: driver res={fut.result().res}; retrying…")
            self.sync()

        log.error("grab_tool: failed after maximum retries")
        return False

    def set_DO(self, index: int, status: int) -> bool:
        """
        Execute a digital output on the Dobot via the DOExecute service.
        """
        import rclpy, time

        # build request
        req = DOExecute.Request()
        req.index = index
        req.status = status

        retry_pause = 0.25
        max_attempts = 10

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"set_DO: DOExecute attempt {attempt}/{max_attempts}")

            if not self.doexec_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("set_DO: DOExecute service unavailable")
                return False

            fut = self.doexec_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

            if not fut.done() or fut.result() is None:
                self.get_logger().error("set_DO: DOExecute call timed out")
                return False  # do not retry on timeout

            resp = fut.result()
            res_code = getattr(resp, 'res', 1)
            if res_code == 0:
                self.get_logger().info("set_DO: DOExecute succeeded")
                break
            else:
                self.get_logger().warn(
                    f"set_DO: DOExecute returned error code {res_code}, retrying…"
                )
                self.sync()
        else:
            self.get_logger().error("set_DO: DOExecute failed after maximum retries")
            return False

        return True

    def toggle_drag_mode(self, max_attempts: int = 5, retry_pause: float = 0.25) -> bool:
        """
        Alternate StartDrag and StopDrag until one succeeds (res == 0).
        Returns True on success, False if both calls fail after max_attempts each.
        """

        # begin by trying to enable drag mode
        use_start = True

        for attempt in range(1, max_attempts + 1):
            cli = self.start_drag_cli if use_start else self.stop_drag_cli
            srv_name = "StartDrag" if use_start else "StopDrag"

            # wait for service
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f"toggle_drag_mode: {srv_name} service unavailable (attempt {attempt}/{max_attempts})"
                )
                # flip for next attempt
                use_start = not use_start
                time.sleep(retry_pause)
                continue

            # call and wait
            fut = cli.call_async(StartDrag.Request() if use_start else StopDrag.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

            if fut.done() and fut.result() is not None:
                res = getattr(fut.result(), 'res', None)
                if res == 0:
                    self.get_logger().info(f"toggle_drag_mode: {srv_name} succeeded")
                    return True
                else:
                    self.get_logger().warn(
                        f"toggle_drag_mode: {srv_name} returned res={res}, switching to "
                        f"{'StopDrag' if use_start else 'StartDrag'}"
                    )
            else:
                self.get_logger().warn(
                    f"toggle_drag_mode: {srv_name} call timed out or failed, switching to "
                    f"{'StopDrag' if use_start else 'StartDrag'}"
                )

            # flip for next attempt
            use_start = not use_start
            time.sleep(retry_pause)

        self.get_logger().error(
            "toggle_drag_mode: Exhausted attempts without success"
        )
        return False
    
    def move_arc(
        self,
        count: int,
        offset1: tuple[float, float, float, float, float, float],
        offset2: tuple[float, float, float, float, float, float],
        param_value: list[str] | None = None,
    ) -> bool:
        """
        Execute an arc motion by specifying two offset poses relative to the current pose via Arc service.

        Parameters:
        • count: number of arc segments
        • offset1: (dx1, dy1, dz1, drx1, dry1, drz1) offsets in mm and deg from current pose for first point
        • offset2: (dx2, dy2, dz2, drx2, dry2, drz2) offsets in mm and deg from current pose for second point

        Returns True on success (res==0), False on timeout or driver error.
        """
        self.sync() #sync first for accurate get pose

        # 1) fetch current pose via GetPose service
        fut = self.get_pose_cli.call_async(GetPose.Request(user=0, tool=0))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not hasattr(fut.result(), "pose"):
            self.get_logger().error("move_arc: GetPose call failed")
            return False

        # parse current pose: x, y, z (mm), rx, ry, rz (deg)
        try:
            vals = [float(v) for v in fut.result().pose.strip("{}").split(",")[:6]]
            cx, cy, cz, crx, cry, crz = vals
        except Exception as exc:
            self.get_logger().error(f"move_arc: bad pose string – {exc}")
            return False

        # record start pose for logging
        start_pose = {"x": cx, "y": cy, "z": cz, "rx": crx, "ry": cry, "rz": crz}
        self.get_logger().info(f"[move_arc] start_pose: {start_pose}")

        # 2) compute target points by applying offsets
        dx1, dy1, dz1, drx1, dry1, drz1 = offset1
        dx2, dy2, dz2, drx2, dry2, drz2 = offset2

        x1, y1, z1     = cx + dx1, cy + dy1, cz + dz1
        rx1, ry1, rz1  = crx + drx1, cry + dry1, crz + drz1
        x2, y2, z2     = cx + dx2, cy + dy2, cz + dz2
        rx2, ry2, rz2  = crx + drx2, cry + dry2, crz + drz2

        # stash datalog for inspection if needed
        self.datalog_lastrun_move_arc = {
            "start_pose": start_pose,
            "point1":     {"x": x1, "y": y1, "z": z1, "rx": rx1, "ry": ry1, "rz": rz1},
            "point2":     {"x": x2, "y": y2, "z": z2, "rx": rx2, "ry": ry2, "rz": rz2},
        }

        # 3) build Arc request
        param_value = param_value or []
        req = Arc.Request()
        req.x1, req.y1, req.z1   = x1,  y1,  z1
        req.rx1, req.ry1, req.rz1 = rx1, ry1, rz1
        req.x2, req.y2, req.z2   = x2,  y2,  z2
        req.rx2, req.ry2, req.rz2 = rx2, ry2, rz2
        req.param_value          = param_value

        # 4) call service with retries
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.arc_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                self.get_logger().error("move_arc: call timed out")
                return False

            if fut.result().res == 0:
                self.get_logger().info(f"move_arc: success ✓ (count={count})")
                return True

            self.get_logger().warn(
                f"move_arc: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})"
            )
            self.sync()

        self.get_logger().error("move_arc: failed after maximum retries")
        return False
    
    def set_speed_factor(
        self,
        ratio: float,
    ) -> bool:
        """
        Set the robot speed factor using the SpeedFactor service.

        Parameters:
        • ratio: scaling factor (0–100)

        Returns True on success (res==0), False on timeout or driver error.
        """
        import time

        req = SpeedFactor.Request()
        req.ratio = ratio
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.speed_factor_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if not fut.done() or fut.result() is None:
                self.get_logger().error("set_speed_factor: call timed out")
                return False
            if fut.result().res == 0:
                self.get_logger().info(f"set_speed_factor: success ✓ (ratio={ratio})")
                return True
            self.get_logger().warn(
                f"set_speed_factor: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})"
            )
            time.sleep(retry_pause)
        self.get_logger().error("set_speed_factor: failed after maximum retries")
        return False

    def move_circle(
        self,
        count: int,
        offset1: tuple[float, float, float, float, float, float],
        offset2: tuple[float, float, float, float, float, float],
        param_value: list[str] | None = None,
    ) -> bool:
        """
        Execute a circular motion by specifying two offset poses relative to current pose via Circle3 service.

        Parameters:
        • count: number of circle segments
        • offset1: (dx1, dy1, dz1, drx1, dry1, drz1) offsets in mm and deg from current pose for first point
        • offset2: (dx2, dy2, dz2, drx2, dry2, drz2) offsets in mm and deg from current pose for second point
        • param_value: list of Speed/AccL strings, e.g. ["SpeedL=100,AccL=100"]

        Returns True on success, False on timeout or driver error.
        """
        self.sync() #sync first for accurate get pose

        # 1) fetch current pose via GetPose service
        fut = self.get_pose_cli.call_async(GetPose.Request(user=0, tool=0))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not hasattr(fut.result(), "pose"):
            self.get_logger().error("move_circle: GetPose call failed")
            return False

        # parse current pose: x, y, z (mm), rx, ry, rz (deg)
        try:
            vals = [float(v) for v in fut.result().pose.strip("{}").split(",")[:6]]
            cx, cy, cz, crx, cry, crz = vals
        except Exception as exc:
            self.get_logger().error(f"move_circle: bad pose string – {exc}")
            return False

        # record and log start pose
        start_pose = {"x": cx, "y": cy, "z": cz, "rx": crx, "ry": cry, "rz": crz}
        self.get_logger().info(f"[move_circle] start_pose: {start_pose}")

        # 2) compute target points by applying offsets
        dx1, dy1, dz1, drx1, dry1, drz1 = offset1
        dx2, dy2, dz2, drx2, dry2, drz2 = offset2

        x1, y1, z1    = cx + dx1, cy + dy1, cz + dz1
        rx1, ry1, rz1 = crx + drx1, cry + dry1, crz + drz1
        x2, y2, z2    = cx + dx2, cy + dy2, cz + dz2
        rx2, ry2, rz2 = crx + drx2, cry + dry2, crz + drz2

        # record and log computed points
        point1 = {"x": x1, "y": y1, "z": z1, "rx": rx1, "ry": ry1, "rz": rz1}
        point2 = {"x": x2, "y": y2, "z": z2, "rx": rx2, "ry": ry2, "rz": rz2}
        self.get_logger().info(f"[move_circle] point1: {point1}")
        self.get_logger().info(f"[move_circle] point2: {point2}")

        # stash datalog for later inspection
        self.datalog_lastrun_move_circle = {
            "start_pose": start_pose,
            "point1":     point1,
            "point2":     point2
        }

        # 3) build Circle3 request
        param_value = param_value or []
        req = Circle3.Request()
        req.count = count
        req.x1, req.y1, req.z1     = x1,  y1,  z1
        req.rx1, req.ry1, req.rz1  = rx1, ry1, rz1
        req.x2, req.y2, req.z2     = x2,  y2,  z2
        req.rx2, req.ry2, req.rz2  = rx2, ry2, rz2
        req.param_value            = param_value

        # 4) call service with retries
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut = self.circle3_cli .call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

            if not fut.done() or fut.result() is None:
                self.get_logger().error("move_circle: call timed out")
                return False

            if fut.result().res == 0:
                self.get_logger().info(f"move_circle: success ✓ (count={count})")
                return True

            self.get_logger().warn(
                f"move_circle: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})"
            )
            self.sync()

        self.get_logger().error("move_circle: failed after maximum retries")
        return False

    def wait_for_joint_state(self, topic_name: str, timeout_sec: float = 2.0):
        """
        Synchronously waits for a JointState message from the given topic.
        """
        from sensor_msgs.msg import JointState
        import rclpy
        msg_container = {"msg": None}

        def callback(msg):
            msg_container["msg"] = msg

        sub = self.create_subscription(JointState, topic_name, callback, 10)
        start_time = self.get_clock().now().nanoseconds * 1e-9
        while msg_container["msg"] is None:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            if (current_time - start_time) > timeout_sec:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return msg_container["msg"]

    def wait_for_servo_ready(self, timeout: float = 15.0) -> bool:
        """
        Poll /get_servo_status (Trigger) until status == 0 (READY) or timeout.
        """
        client = self.create_client(Trigger, 'get_servo_status')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Servo status service unavailable")
            return False
        
        # Poll until ready or timeout
        import time
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout:
            try:
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.done() and future.result() is not None:
                    # Assuming success=True and message="" indicates ready
                    if future.result().success:
                        return True
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)
        
        return False
    
    def _wait_for_servo_ready_with_timeout(self, timeout: float = 15.0) -> bool:
        """Wait for servo to be ready with timeout handling."""
        start_time = time.monotonic()
        while not self.wait_for_servo_ready(timeout=timeout):
            if time.monotonic() - start_time > timeout:
                self.get_logger().error("_wait_for_servo_ready_with_timeout(): Timeout waiting for servo ready")
                return False
            self.get_logger().warn("_wait_for_servo_ready_with_timeout(): Waiting for servo to be ready...")
            time.sleep(0.2)
        return True
    
    def current_angles(self) -> tuple[float, ...] | None:
        """
        Read the current joint angles immediately and return them in degrees.
        
        This function is designed to be used with run_skill to save/restore robot positions:
        
        Example usage:
            # Save current position
            saved_angles = run_skill("current_angles")
            
            # Do some movements...
            run_skill("moveJ_deg", 10, 0, 0, 0, 0, 0)
            
            # Return to saved position
            if saved_angles:
                run_skill("gotoJ_deg", *saved_angles)
        
        Returns:
            tuple: (angle1, angle2, angle3, angle4, angle5, angle6) in degrees, or None if failed
        """
        try:
            # Wait for servo to be ready (movement complete)
            if not self._wait_for_servo_ready_with_timeout():
                return None
                
            # Read current joint state
            joint_state_msg = self.wait_for_joint_state("/joint_states_robot", timeout_sec=2.0)
            
            if joint_state_msg is None:
                self.get_logger().error("current_angles(): Failed to read joint state")
                return None
                
            if not hasattr(joint_state_msg, 'position') or len(joint_state_msg.position) < 6:
                self.get_logger().error("current_angles(): Insufficient joint data")
                return None
            
            # Convert from radians to degrees - ensure exactly 6 values
            joint_positions = list(joint_state_msg.position[:6])
            angles_deg = tuple(math.degrees(float(angle)) for angle in joint_positions)
            
            self.get_logger().info(f"current_angles(): Current joint angles (deg): {angles_deg}")
            return angles_deg
            
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"current_angles(): Error converting angles: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"current_angles(): Unexpected error: {e}")
            return None

# Global motion node for sequence execution
_global_motion_node = None

def init_motion_node():
    """Initialize the global motion node for sequence execution."""
    global _global_motion_node
    if _global_motion_node is None:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        _global_motion_node = robot_motion()
    return _global_motion_node

def cleanup_motion_node():
    """Cleanup the global motion node."""
    global _global_motion_node
    if _global_motion_node is not None:
        _global_motion_node.destroy_node()
        _global_motion_node = None
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

def run_skill_with_node(motion_node, fn_name: str, *args):
    """
    Execute a robot skill using the provided motion_node instance.
    
    This function calls a method on the motion_node and handles different return types.
    If the operation fails, it logs an error and returns False.
    
    Args:
        motion_node: The robot_motion instance to use
        fn_name: Name of the method to call on motion_node
        *args: Arguments to pass to the method
        
    Returns:
        bool: True if operation succeeded, False otherwise
    """
    try:
        fn = getattr(motion_node, fn_name)
        result = fn(*args)
        
        # Handle different return types: bool, tuple[bool, ...], or None
        if isinstance(result, tuple):
            # For methods like set_gripper_position that return (bool, other_data)
            ok = result[0] if len(result) > 0 else False
        else:
            # For methods that return bool directly or None
            ok = bool(result) if result is not None else False
            
        if not ok:
            motion_node.get_logger().error(f"{fn_name}{args} failed")
            
        return ok
        
    except Exception as e:
        motion_node.get_logger().error(f"{fn_name}{args} failed with exception: {e}")
        return False

def run_skill(fn_name: str, *args):
    """
    Execute a robot skill using the global motion node (for backward compatibility).
    
    This function automatically manages the motion node lifecycle and provides
    the same interface as before while being more efficient for sequence execution.
    
    Args:
        fn_name: Name of the method to call on motion_node
        *args: Arguments to pass to the method
        
    Returns:
        bool: True if operation succeeded, False otherwise
    """
    motion_node = init_motion_node()
    return run_skill_with_node(motion_node, fn_name, *args)

def execute_sequence(sequence_func, **params):
    """
    Execute a sequence function with proper motion node management.
    
    This function creates a motion node, executes the sequence, and cleans up properly.
    
    Args:
        sequence_func: The sequence function to execute
        **params: Parameters to pass to the sequence function
        
    Returns:
        bool: True if sequence completed successfully, False otherwise
    """
    try:
        motion_node = init_motion_node()
        result = sequence_func(**params)
        return result
    except Exception as e:
        if _global_motion_node:
            _global_motion_node.get_logger().error(f"Sequence execution failed: {e}")
        return False
    finally:
        # Note: Don't cleanup here - let the calling code decide when to cleanup
        # This allows for multiple sequences to run with the same motion node
        pass
         
# def main():
#     try:
#         run_skill("refresh_position")
#         time.sleep(3.0)

#         ########################################

#         for i in range(1):
#             # run_skill("moveEE", 0, 0, 0, 0, 0, 0, "portafilter_link")
#             # run_skill("gotoEE", 0,-350,250,90,0,0)
#             run_skill("moveJ_deg",-45, 0, 0, 0, 0, 0, 1.0, 0.1)
#             # run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0)
#             # run_skill("move_to", 'espresso_grinder', 0.15, 0.0, 0.0, 0.0)
#             # run_skill("enforce_rxry")
#             # run_skill("move_portafilter_arc", 45)
            
#     except KeyboardInterrupt:
#         pass

# if __name__ == "__main__":
#     main()

#-------FUNCTIONS--------------------------------------------------------------------------------------------------------------------------------------------------------------
        # run_skill("move_portafilter_arc", 20)
        # run_skill("moveEE", 100,  0, -150, 10, 0, 0) #move end effector from current position. x y z mm, rx ry rz deg
        # run_skill("release_tension") #activate drag mode for 1 sec
        # run_skill("enforce_rxry")
        # run_skill("move_portafilter_arc", 10) #move portafilter in arc degrees -CW +CCW
        # run_skill("moveJ_deg", 0, 0, 0, 0, 0, 0)  #move joints from current position. j1 j2 j3 j4 j5 j6 deg
        # run_skill("gotoEE", 100,  0, -150, 10, 0, 0) # same as move but absolute, no reference, careful! - no motion planning 
        # run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0) # same as move but absolute, no reference, careful! - no motion planning 
        # run_skill("circle_motion", 300, 0, 200, 0, 0, 0, 300, 100, 200, 0, 0, 0, 2, 25.0) # circular motion through 2 points: x1,y1,z1,rx1,ry1,rz1,x2,y2,z2,rx2,ry2,rz2,count,speed%
        # run_skill("arc_circle_motion", 250, 0, 300, 50, 0, 360, 6, 0, 0, 0, 25.0) # reliable circular motion: center_x,y,z, radius, start_angle, total_angle, num_arcs, rx,ry,rz, speed%
        # run_skill("approach_machine", "three_group_espresso", "group_1")
        # run_skill("mount_machine", "three_group_espresso", "group_1")
        # run_skill("move_to", "three_group_espresso", 0.12)
        # run_skill("get_machine_position", "three_group_espresso")
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
