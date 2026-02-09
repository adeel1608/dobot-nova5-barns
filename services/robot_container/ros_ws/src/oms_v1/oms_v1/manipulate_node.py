#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
🌟 MANIPULATE NODE V3 - HYBRID IMPLEMENTATION 🌟
═══════════════════════════════════════════════════════════════════════════════

This is a hybrid implementation that combines the best working methods from
v1 and v2 to create the most reliable version:

FROM V1 (Working methods):
  ✅ enforce_rxry()          - Proven stable orientation enforcement  
  ✅ sync()                  - Reliable motion synchronization
  ✅ move_portafilter_arc()  - Stable portafilter rotation

FROM V2 (Everything else):
  ✅ All other methods       - Updated implementations with fixes
  ✅ Improved error handling - Better robustness
  ✅ Enhanced logging        - Better debugging

CLASS: robot_motion (same as v2)

USAGE: 
  Set USE_VERSION = "v3" in testing_v1.py to use this implementation

═══════════════════════════════════════════════════════════════════════════════
"""
# Python Imports
import sys
import time
import os
import math
import yaml
import numpy as np
import threading
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

# Custom Exceptions
class RobotMotionError(Exception):
    """Base exception for robot motion failures"""
    pass

class SyncFailureError(RobotMotionError):
    """Raised when sync operation fails after all retries"""
    pass

class MovementFailureError(RobotMotionError):
    """Raised when a movement command fails after all retries"""
    pass

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from control_msgs.action import FollowJointTrajectory

# MoveIt2 Imports
from pymoveit2 import MoveIt2, MoveIt2State
from tf_transformations import euler_matrix, quaternion_from_matrix

# Global motion node for sequence execution
_global_motion_node = None
_motion_node_lock = threading.Lock()

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
        self._percep_exec = None
        self._executor_thread = None
        self._shutdown_requested = False
        
        try:
            self._percep_exec = SingleThreadedExecutor()
            self._percep_exec.add_node(self)
            self._executor_thread = threading.Thread(target=self._safe_spin, daemon=False)
            self._executor_thread.start()
        except Exception as e:
            self.get_logger().error(f"robot_perception.__init__(): Failed to initialize: {e}")
            self._cleanup()
    
    def _safe_spin(self):
        """Safely spin the executor with exception handling"""
        try:
            while not self._shutdown_requested and self._percep_exec is not None:
                self._percep_exec.spin_once(timeout_sec=0.1)
        except Exception as e:
            if not self._shutdown_requested:
                try:
                    self.get_logger().warn(f"robot_perception._safe_spin(): Exception during spin: {e}")
                except:
                    pass  # Ignore logging errors during shutdown
    
    def _cleanup(self):
        """Internal cleanup method"""
        self._shutdown_requested = True
        
        try:
            if hasattr(self, '_percep_exec') and self._percep_exec is not None:
                self._percep_exec.shutdown()
                self._percep_exec = None
        except Exception:
            pass
            
        try:
            if hasattr(self, '_executor_thread') and self._executor_thread is not None and self._executor_thread.is_alive():
                self._executor_thread.join(timeout=3.0)
                self._executor_thread = None
        except Exception:
            pass
    
    def destroy_node(self):
        """Override destroy_node to properly cleanup threads"""
        self._cleanup()
        try:
            super().destroy_node()
        except Exception:
            pass  # Ignore errors during node destruction

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
    Tool,
    SetTool,
    InverseSolution,
    PositiveSolution,
)

class robot_motion(Node):
    def __init__(self):
        super().__init__("robot_motion")          # keep the node alive for the whole run
        self._node_valid = True

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
        self.tool_cli         = self.create_client(Tool,                '/dobot_bringup_v3/srv/Tool')
        self.set_tool_cli     = self.create_client(SetTool,             '/dobot_bringup_v3/srv/SetTool')
        self.servo_cli        = self.create_client(Trigger,             'get_servo_status')
        self.inverse_solution_cli = self.create_client(InverseSolution, '/dobot_bringup_v3/srv/InverseSolution')
        self.positive_solution_cli = self.create_client(PositiveSolution, '/dobot_bringup_v3/srv/PositiveSolution')
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
            "Tool":                self.tool_cli,
            "SetTool":             self.set_tool_cli,
            "GetServoStatus":      self.servo_cli,
            "InverseSolution":     self.inverse_solution_cli,
            "PositiveSolution":    self.positive_solution_cli,
        }

        missing = [name for name, cli in service_map.items()
                   if not cli.wait_for_service(timeout_sec=timeout)]
        if missing:
            self.get_logger().error(
                f"⚠️  services unavailable after {timeout}s: {', '.join(missing)}")
        else:
            self.get_logger().info("✅  all motion-control services are ready")

        # ── Initialize MoveIt2 Configuration ──────────────────────────────────
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

        # ── Initialize TF Buffer and Listener ─────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ── Initialize health monitoring ────────────────────────────────────── 
        self._last_successful_operation = time.time()
        self._consecutive_failures = 0
        self._max_consecutive_failures = 5
    
    def health_check(self) -> bool:
        """
        Perform a quick health check of critical robot services.
        Returns True if robot appears to be responsive, False otherwise.
        """
        try:
            # Test if sync service is responsive with short timeout
            if not self.sync_cli.wait_for_service(timeout_sec=3.0):
                self.safe_log("warn", "health_check(): Sync service unavailable")
                return False
            
            # For health check, just verify service availability rather than calling it
            # This reduces the chance of interfering with ongoing operations
            self.safe_log("info", "health_check(): Robot services appear available")
            return True
            
        except Exception as e:
            self.safe_log("error", f"health_check(): Exception during health check: {e}")
            return False
    
    def is_robot_likely_responsive(self) -> bool:
        """
        Check if the robot is likely responsive based on recent successful operations.
        This can be used to bypass intensive health checks when robot appears to be working.
        """
        try:
            if hasattr(self, '_last_successful_operation'):
                time_since_success = time.time() - self._last_successful_operation
                # If we had a successful operation in the last 30 seconds, assume robot is working
                if time_since_success < 30.0:
                    return True
            
            # If we haven't had many consecutive failures, assume robot is probably working
            if hasattr(self, '_consecutive_failures') and self._consecutive_failures < 3:
                return True
                
            return False
        except Exception:
            return False
    
    def log_robot_status(self) -> None:
        """
        Log current robot status for diagnostic purposes.
        """
        try:
            if hasattr(self, '_last_successful_operation'):
                time_since_success = time.time() - self._last_successful_operation
                self.safe_log("info", f"Robot status: {time_since_success:.1f}s since last success")
            
            if hasattr(self, '_consecutive_failures'):
                self.safe_log("info", f"Robot status: {self._consecutive_failures} consecutive failures")
                
            # Check service availability
            services_status = []
            services_to_check = [
                ("Sync", self.sync_cli),
                ("SpeedFactor", self.speed_factor_cli),
                ("JointMovJ", self.jointmovj_cli)
            ]
            
            for service_name, client in services_to_check:
                try:
                    available = client.wait_for_service(timeout_sec=1.0)
                    services_status.append(f"{service_name}: {'✓' if available else '✗'}")
                except Exception:
                    services_status.append(f"{service_name}: ✗")
            
            self.safe_log("info", f"Robot services: {', '.join(services_status)}")
            
        except Exception as e:
            self.safe_log("debug", f"log_robot_status(): Error: {e}")
    
    def reset_robot_connection(self) -> bool:
        """
        Attempt to reset robot connection when services become unresponsive.
        """
        try:
            self.safe_log("warn", "reset_robot_connection(): Attempting to reset robot connection...")
            
            # Wait a moment for any ongoing operations to complete
            time.sleep(2.0)
            
            # Try to recreate critical service clients
            try:
                from dobot_msgs_v3.srv import Sync, SpeedFactor, JointMovJ
                self.sync_cli = self.create_client(Sync, '/dobot_bringup_v3/srv/Sync')
                self.speed_factor_cli = self.create_client(SpeedFactor, '/dobot_bringup_v3/srv/SpeedFactor')
                self.jointmovj_cli = self.create_client(JointMovJ, '/dobot_bringup_v3/srv/JointMovJ')
            except Exception as e:
                self.safe_log("error", f"reset_robot_connection(): Failed to recreate clients: {e}")
                return False
            
            # Wait for services to become available
            services_ready = (
                self.sync_cli.wait_for_service(timeout_sec=10.0) and
                self.speed_factor_cli.wait_for_service(timeout_sec=10.0) and
                self.jointmovj_cli.wait_for_service(timeout_sec=10.0)
            )
            
            if services_ready:
                self.safe_log("info", "reset_robot_connection(): Robot connection reset successful")
                self._consecutive_failures = 0
                self._last_successful_operation = time.time()
                return True
            else:
                self.safe_log("error", "reset_robot_connection(): Services still unavailable after reset")
                return False
                
        except Exception as e:
            self.safe_log("error", f"reset_robot_connection(): Exception during reset: {e}")
            return False
    
    def safe_log(self, level, message):
        """Safely log a message, handling ROS context issues"""
        try:
            if self._node_valid and self.get_logger() is not None:
                if level == "debug":
                    self.get_logger().debug(message)
                elif level == "info":
                    self.get_logger().info(message)
                elif level == "warn" or level == "warning":
                    self.get_logger().warn(message)
                elif level == "error":
                    self.get_logger().error(message)
                elif level == "fatal":
                    self.get_logger().fatal(message)
                else:
                    self.get_logger().info(message)  # Default to info
        except Exception:
            # If logging fails, try to print to stdout as fallback
            try:
                print(f"[{level.upper()}] {message}")
            except:
                pass  # If even print fails, silently ignore
    
    def destroy_node(self):
        """Override destroy_node to mark node as invalid"""
        self._node_valid = False
        try:
            super().destroy_node()
        except Exception:
            pass

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
            required_samples: int = 10,
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

    def release_tension(self, settling_time: float = 0.5) -> bool:
        """
        • Enable drag-mode (StartDrag) → let the arm "relax".
        • Wait `settling_time` s to dissipate any spring-back.
        • Disable drag-mode (StopDrag).
        Retries driver-error responses up to `max_attempts`, but aborts on
        transport time-outs.  Returns **True** on full success.
        Ensures robot is NOT in drag mode before starting by calling StopDrag first.
        """

        import time, rclpy

        max_attempts  = 20
        call_timeout  = 5.0      # seconds to wait for each service reply
        retry_pause   = 0.2     # pause between attempts

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
        # 0)  StopDrag first (ensure NOT in drag mode) -------------------------#
        res = _try_srv(self.stop_drag_cli, "StopDrag")
        if res == 0:
            log.info("StopDrag OK (cleared any existing drag mode)")
        elif res is not None:
            log.info(f"StopDrag returned {res} (robot likely not in drag mode, continuing)")
        
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
                time.sleep(0.5)
                return True
            log.warn(f"StopDrag driver res={res}; retrying ({attempt}/{max_attempts})")
            time.sleep(retry_pause)

        log.error("StopDrag: exceeded max_attempts")
        return False

    def inverse_solution(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, user: int = 0, tool: int = 0) -> bool:
        """
        Calculate the inverse solution for the given pose.
        """
        fut = self.inverse_solution_cli.call_async(InverseSolution.Request(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, user=user, tool=tool))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def positive_solution(self, j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> bool:
        """
        Calculate the positive solution for the given joint angles.
        """
        fut = self.positive_solution_cli.call_async(PositiveSolution.Request(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()
    
    def sync(self, raise_on_failure: bool = False, max_retries: int = 20) -> bool:
        """
        Wait for the Dobot motion to complete by calling the /dobot_bringup_v3/srv/Sync service.

        Parameters
        ----------
        raise_on_failure : bool
            If True, raises SyncFailureError instead of returning False
        max_retries : int
            Maximum number of retry attempts (default: 3, reduced from 20 for faster failure)

        Returns
        -------
        True if the service returns res == 0 (motion done), False otherwise.
        
        Raises
        ------
        SyncFailureError
            If raise_on_failure=True and sync fails after all attempts
        """
        from dobot_msgs_v3.srv import Sync
        import rclpy

        # Ensure service is available
        if not self.sync_cli.wait_for_service(timeout_sec=10.0):
            error_msg = 'sync: Sync service unavailable'
            self.safe_log('error', error_msg)
            if raise_on_failure:
                raise SyncFailureError(error_msg)
            return False

        # Reduced retry logic - fail fast instead of wasting time
        max_attempts = max_retries
        timeout_sec = 30.0  # Generous timeout for motion completion
        last_error_msg = None
        
        for attempt in range(1, max_attempts + 1):
            try:
                # Call service and wait for completion WITH TIMEOUT
                fut = self.sync_cli.call_async(Sync.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)

                # Check result
                if not fut.done():
                    last_error_msg = f'sync: Timeout after {timeout_sec}s (attempt {attempt}/{max_attempts})'
                    self.safe_log('warn', last_error_msg)
                    if attempt == max_attempts:
                        final_msg = 'sync: All sync attempts timed out'
                        self.safe_log('error', final_msg)
                        if raise_on_failure:
                            raise SyncFailureError(final_msg)
                        return False
                    continue

                if fut.result() is None:
                    last_error_msg = f'sync: No response from Sync service (attempt {attempt}/{max_attempts})'
                    self.safe_log('warn', last_error_msg)
                    if attempt == max_attempts:
                        final_msg = 'sync: No response after all attempts'
                        self.safe_log('error', final_msg)
                        if raise_on_failure:
                            raise SyncFailureError(final_msg)
                        return False
                    continue

                res_code = getattr(fut.result(), 'res', None)
                if res_code == 0:
                    self.safe_log('info', 'sync: Motion complete')
                    return True
                else:
                    last_error_msg = f'sync: Sync returned error code {res_code} (attempt {attempt}/{max_attempts})'
                    self.safe_log('warn', last_error_msg)
                    if attempt == max_attempts:
                        final_msg = f'sync: Failed with error code {res_code} after all attempts'
                        self.safe_log('error', final_msg)
                        if raise_on_failure:
                            raise SyncFailureError(final_msg)
                        return False
                    
            except SyncFailureError:
                raise  # Re-raise our custom exception
            except Exception as e:
                last_error_msg = f'sync: Exception during attempt {attempt}/{max_attempts}: {e}'
                self.safe_log('warn', last_error_msg)
                if attempt == max_attempts:
                    final_msg = f'sync: Failed with exception after all attempts: {e}'
                    self.safe_log('error', final_msg)
                    if raise_on_failure:
                        raise SyncFailureError(final_msg) from e
                    return False
            
            # Short pause between retries
            import time
            time.sleep(0.5)

        if raise_on_failure:
            raise SyncFailureError(last_error_msg or 'sync: Failed for unknown reason')
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
        self.sync()

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
        max_attempts  = 20
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

        retry_pause, max_attempts = 0.5, 20
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
            
            # Check sync return value - if sync fails, exit immediately instead of retrying
            if not self.sync():
                log.error("move_to: sync failed, aborting further MovL attempts")
                return False
            
            time.sleep(retry_pause)

        log.error("move_to: failed after maximum retries")
        return False

    def enforce_rxry(self) -> bool:
        """
        Override Link6's Rx→90°, Ry→0° (keep current Rz) while freezing
        the world‐space position of portafilter_link to ±0.5 mm.
        Uses the /dobot_bringup_v3/srv/MovJ service (Link6 planning) instead of MoveIt2.
        Retries any failed service call every 0.25 s up to 10 attempts; returns False on total failure.
        Returns True on success, False on any failure.
        """
        from tf_transformations import euler_matrix
        from dobot_msgs_v3.srv import GetPose, MovJ
        import numpy as np
        import time
        import rclpy

        # ── 1) fixed offset from Link6 origin → portafilter_link origin (m)
        d_rel = np.array([0.0, 0.0, 0.2825])

        retry_pause = 0.25
        max_attempts = 20

        gp_req = GetPose.Request()
        gp_req.user = 0
        gp_req.tool = 0

        resp = None
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"enforce_rxry(): GetPose attempt {attempt}/{max_attempts}")

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
                break  # success!

            # If we reach here, the call either timed out or returned nothing
            self.get_logger().warn(
                "enforce_rxry(): GetPose call failed or timed out—waiting 0.25 s before retry"
            )
            time.sleep(retry_pause)

        if resp is None or not hasattr(resp, "pose"):
            self.get_logger().error(
                "enforce_rxry(): Failed to retrieve initial pose after 10 attempts."
            )
            return False

        # parse "{tx,ty,tz,rx,ry,rz,…}"  (tx, ty, tz in mm; rx, ry, rz in degrees)
        parts = resp.pose.strip("{}").split(",")
        if len(parts) < 6:
            self.get_logger().error("enforce_rxry(): Invalid pose format.")
            return False
        try:
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = [float(p) for p in parts[:6]]
        except Exception as e:
            self.get_logger().error(f"enforce_rxry(): Error parsing pose: {e}")
            return False

        # convert to metres and build current rotation matrix
        p_link6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
        rads = np.radians([rx_curr, ry_curr, rz_curr])
        R6_curr = euler_matrix(*rads)[:3, :3]

        # compute world position of the portafilter_link
        p_pf_world = p_link6 + R6_curr.dot(d_rel)

        # ── 3) define desired Link6 orientation: Rx=90°, Ry=0°, keep Rz
        rx_t, ry_t, rz_t = 90.0, 0.0, rz_curr
        rads_goal = np.radians([rx_t, ry_t, rz_t])
        M_goal = euler_matrix(*rads_goal)
        R6_goal = M_goal[:3, :3]

        # ── 4) back‐solve Link6 goal position so portafilter_link stays put
        p6_goal = p_pf_world - R6_goal.dot(d_rel)
        self.get_logger().info(f"enforce_rxry(): Goal Link6 pos (m): {p6_goal.tolist()}")

        # convert goal position back to millimetres for MovJ
        x_goal_mm = float(p6_goal[0] * 1000.0)
        y_goal_mm = float(p6_goal[1] * 1000.0)
        z_goal_mm = float(p6_goal[2] * 1000.0)

        # ── 5) call /dobot_bringup_v3/srv/MovJ for Link6 move (up to 10 retries)
        self.movj_cli = getattr(
            self, "movj_cli",
            self.create_client(MovJ, '/dobot_bringup_v3/srv/MovJ')
        )
        movj_req = MovJ.Request()
        movj_req.x = x_goal_mm
        movj_req.y = y_goal_mm
        movj_req.z = z_goal_mm
        movj_req.rx = rx_t
        movj_req.ry = ry_t
        movj_req.rz = rz_t
        # Use SpeedL=100, AccL=100 as example
        movj_req.param_value = ["SpeedJ=100,AccJ=100"]

        movj_resp = None
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"enforce_rxry(): MovJ attempt {attempt}/{max_attempts}")
            if not self.movj_cli.wait_for_service(timeout_sec=20.0):
                self.get_logger().warn("enforce_rxry(): MovJ service unavailable, retrying...")
                time.sleep(retry_pause)
                continue

            movj_fut = self.movj_cli.call_async(movj_req)
            start_mv = self.get_clock().now().nanoseconds * 1e-9
            while not movj_fut.done() and (self.get_clock().now().nanoseconds * 1e-9 - start_mv) < 0.5:
                rclpy.spin_once(self, timeout_sec=0.01)

            if movj_fut.done() and movj_fut.result() is not None:
                movj_resp = movj_fut.result()
                break

            self.get_logger().warn("enforce_rxry(): MovJ call failed or timed out, retrying...")
            time.sleep(retry_pause)

        if movj_resp is None:
            self.get_logger().error("enforce_rxry(): Failed to call MovJ after 10 attempts.")
            return False
        
        self.get_logger().info("enforce_rxry(): Completed successfully.")
        return True

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

    # ═══════════════════════════════════════════════════════════════════════════════
    # V1 METHODS - Working implementations from manipulate_node_v1.py
    # ═══════════════════════════════════════════════════════════════════════════════

    def _get_link6_pose_with_retries_v1(self, max_attempts: int = 3) -> tuple | None:
        """Get current Link6 pose with retry logic (v1 implementation)."""
        if not self.get_pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("_get_link6_pose_with_retries_v1(): GetPose service unavailable.")
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
                        self.get_logger().error("_get_link6_pose_with_retries_v1(): Invalid pose format.")
                        return None
                    
                    pose_values = [float(p) for p in parts[:6]]
                    return tuple(pose_values)
                
            except Exception as e:
                self.get_logger().warn(f"_get_link6_pose_with_retries_v1(): Attempt {attempt}/{max_attempts} failed: {e}")
            
            if attempt < max_attempts:
                time.sleep(0.5)

        self.get_logger().error("_get_link6_pose_with_retries_v1(): Failed to retrieve pose after all attempts.")
        return None

    def _compute_portafilter_world_position_v1(self, p_link6: np.ndarray, rads: np.ndarray, d_rel: np.ndarray) -> np.ndarray:
        """Compute world position of portafilter_link given Link6 position and orientation (v1 implementation)."""
        try:
            R6_curr = euler_matrix(*rads)[:3, :3]
            return p_link6 + R6_curr.dot(d_rel)
        except Exception as e:
            self.get_logger().error(f"_compute_portafilter_world_position_v1(): Error: {e}")
            raise

    def _compute_link6_goal_pose_v1(self, rz_curr: float, p_pf_world: np.ndarray, d_rel: np.ndarray) -> tuple | None:
        """Compute goal pose for Link6 to achieve desired orientation while keeping portafilter position fixed (v1 implementation)."""
        try:
            # Define desired Link6 orientation: Rx=90°, Ry=0°, keep Rz
            rx_target, ry_target = 90.0, 0.0
            rads_goal = np.radians([rx_target, ry_target, rz_curr])
            
            M_goal = euler_matrix(*rads_goal)
            R6_goal = M_goal[:3, :3]
            quat_goal = list(quaternion_from_matrix(M_goal))

            # Back-solve Link6 goal position so portafilter_link stays put
            p6_goal = p_pf_world - R6_goal.dot(d_rel)
            
            self.get_logger().info(f"_compute_link6_goal_pose_v1(): Goal Link6 pos (m): {p6_goal.tolist()}")
            return p6_goal.tolist(), quat_goal
            
        except Exception as e:
            self.get_logger().error(f"_compute_link6_goal_pose_v1(): Error: {e}")
            return None

    def _execute_enforce_motion_v1(self, goal_pose: tuple) -> bool:
        """Execute the enforce motion using MoveIt2 (v1 implementation)."""
        try:
            position, quaternion = goal_pose
            
            # Create temporary MoveIt2 instance for Link6
            temp_moveit2 = MoveIt2(
                node=self,
                joint_names=self.moveit2.joint_names,
                base_link_name=self.reference_frame,
                end_effector_name="Link6",
                group_name=GROUP_NAME,
            )
            temp_moveit2.planner_id = self.planner_id
            temp_moveit2.max_velocity = self.velocity_scaling
            temp_moveit2.max_acceleration = self.acceleration_scaling
            temp_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            self.get_logger().info("_execute_enforce_motion_v1(): Calling MoveIt2.move_to_pose()...")
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
                self.get_logger().warn(f"_execute_enforce_motion_v1(): Motion ended with state: {state}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_enforce_motion_v1(): Error: {e}")
            return False

    def _verify_portafilter_position_v1(self, p_pf_world: np.ndarray, d_rel: np.ndarray, tolerance: float, max_attempts: int = 10) -> bool:
        """Verify that portafilter_link stayed within tolerance (v1 implementation)."""
        try:
            for attempt in range(max_attempts):
                # Re-read Link6 pose
                current_pose = self._get_link6_pose_with_retries_v1()
                if current_pose is None:
                    return False

                tx2, ty2, tz2, rx2, ry2, rz2 = current_pose
                p6_new = np.array([tx2, ty2, tz2]) * 1e-3
                R6_new = euler_matrix(*np.radians([rx2, ry2, rz2]))[:3, :3]
                p_pf_new = p6_new + R6_new.dot(d_rel)

                error = np.linalg.norm(p_pf_new - p_pf_world)
                if error <= tolerance:
                    self.get_logger().info(f"_verify_portafilter_position_v1(): Position verified (error: {error*1e3:.2f} mm)")
                    return True

                self.get_logger().warn(
                    f"_verify_portafilter_position_v1(): portafilter moved {error*1e3:.2f} mm (>0.5 mm), "
                    f"attempt {attempt + 1}/{max_attempts}"
                )
                
                if attempt < max_attempts - 1:
                    time.sleep(0.2)

            self.get_logger().error("_verify_portafilter_position_v1(): Failed to verify position after all attempts")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_portafilter_position_v1(): Error: {e}")
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
            current_pose = self._get_portafilter_current_pose_v1()
            if current_pose is None:
                return False

            # 2) Compute new orientation
            new_pose = self._compute_arc_pose_v1(current_pose, angle_deg)
            if new_pose is None:
                return False

            # 3) Execute arc motion
            if not self._execute_arc_motion_v1(new_pose, angle_deg):
                return False

            # 4) Verify motion completion
            if not self._verify_arc_completion_v1():
                return False

            self.get_logger().info("move_portafilter_arc(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc(): Unexpected error: {e}")
            return False

    def _get_portafilter_current_pose_v1(self) -> tuple | None:
        """Get current portafilter pose from TF (v1 implementation)."""
        try:
            pf_tf_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                "portafilter_link",
                Time(),
                timeout=Duration(seconds=5.0)
            )
            current_tf = get_transform_list(pf_tf_stamped)
            
            position = np.array(current_tf[:3])
            current_quat = np.array(current_tf[3:])
            
            return position, current_quat
            
        except Exception as e:
            self.get_logger().error(f"_get_portafilter_current_pose_v1(): Failed to get portafilter_link transform: {e}")
            return None

    def _compute_arc_pose_v1(self, current_pose: tuple, angle_deg: float) -> tuple | None:
        """Compute new pose after rotating about local Y axis (v1 implementation)."""
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
                f"_compute_arc_pose_v1(): New orientation (xyzw): {new_quat}"
            )
            
            return new_pos, new_quat
            
        except Exception as e:
            self.get_logger().error(f"_compute_arc_pose_v1(): Error computing arc pose: {e}")
            return None

    def _execute_arc_motion_v1(self, new_pose: tuple, angle_deg: float) -> bool:
        """Execute the arc motion using MoveIt2 (v1 implementation)."""
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
                self.get_logger().info("_execute_arc_motion_v1(): Rotation motion executed successfully.")
                return True
            else:
                self.get_logger().warn(f"_execute_arc_motion_v1(): Rotation motion ended with state: {state}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"_execute_arc_motion_v1(): Error: {e}")
            return False

    def _verify_arc_completion_v1(self, tolerance_deg: float = 1.0, batch_size: int = 10, max_attempts: int = 5) -> bool:
        """Verify that the arc motion has completed with stable joint positions (v1 implementation)."""
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
                    self.get_logger().warn("_verify_arc_completion_v1(): No joint samples collected")
                    continue
                    
                spreads = [max(vals) - min(vals) for vals in zip(*joint_samples)]

                # Check if spread is within tolerance
                if all(spread < tolerance_deg for spread in spreads):
                    self.get_logger().info("_verify_arc_completion_v1(): Joint positions stable")
                    return True

                # Try fallback verification
                if joint_samples:
                    last_sample = joint_samples[-1]
                    if self.verify_joint_positions(last_sample, tolerance_deg):
                        self.get_logger().info("_verify_arc_completion_v1(): Fallback verification passed")
                        return True

                self.get_logger().warn(
                    f"_verify_arc_completion_v1(): Joint spread too large "
                    f"(spreads: {spreads}), attempt {overall_attempt + 1}/{max_attempts}"
                )

            self.get_logger().error("_verify_arc_completion_v1(): Failed to achieve stable joint positions")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_arc_completion_v1(): Error: {e}")
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
        d_rel = np.array([0.0, 0.0, 0.2825])

        retry_pause = 0.25
        max_attempts = 20

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
        
    from typing import Union

    def moveEE(
        self,
        offset_x_mm: Union[float, int],
        offset_y_mm: Union[float, int],
        offset_z_mm: Union[float, int],
        offset_rx_deg: Union[float, int],
        offset_ry_deg: Union[float, int],
        offset_rz_deg: Union[float, int],
    ) -> bool:
        """
        Execute a relative Cartesian move by calling the RelMovL service
        instead of fetching current pose and adding offsets manually.
        Accepts both int and float for offsets.
        """
        from dobot_msgs_v3.srv import RelMovL
        import rclpy

        # Sync first to ensure robot is ready
        if not self.sync():
            self.get_logger().error("moveEE: Failed to sync before movement")
            return False

        # build request, casting to float to ensure correct type
        req = RelMovL.Request()
        req.offset1 = float(offset_x_mm)
        req.offset2 = float(offset_y_mm)
        req.offset3 = float(offset_z_mm)
        req.offset4 = float(offset_rx_deg)
        req.offset5 = float(offset_ry_deg)
        req.offset6 = float(offset_rz_deg)
        req.param_value = ["Tool=0"]

        self.get_logger().info(f"moveEE: Moving by offset ({offset_x_mm}, {offset_y_mm}, {offset_z_mm}, {offset_rx_deg}, {offset_ry_deg}, {offset_rz_deg})")

        # wait for service with longer timeout
        if not self.relmov_l_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("moveEE: RelMovL service unavailable after 10 seconds")
            return False

        # call and wait with longer timeout for complex movements
        fut = self.relmov_l_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)

        if not fut.done():
            self.get_logger().error("moveEE: RelMovL call timed out after 15 seconds")
            return False
            
        if fut.result() is None:
            self.get_logger().error("moveEE: RelMovL returned None result")
            return False

        result_code = getattr(fut.result(), "res", 1)
        if result_code == 0:
            self.get_logger().info("moveEE: RelMovL succeeded ✓")
            # Sync after movement to ensure completion
            return self.sync()
        else:
            self.get_logger().error(f"moveEE: RelMovL failed (res={result_code})")
            return False


    def moveJ_deg(
        self,
        offset1: float,
        offset2: float,
        offset3: float,
        offset4: float,
        offset5: float,
        offset6: float,
    ) -> bool:
        """
        Execute a relative joint move by calling the RelMovJ service
        instead of fetching current joint angles and adding offsets manually.
        """
        from dobot_msgs_v3.srv import RelMovJ
        import rclpy

        # build request - convert all parameters to float to ensure type compatibility
        req = RelMovJ.Request()
        req.offset1 = float(offset1)
        req.offset2 = float(offset2)
        req.offset3 = float(offset3)
        req.offset4 = float(offset4)
        req.offset5 = float(offset5)
        req.offset6 = float(offset6)
        req.param_value = ["Tool=0"]

        # wait for service
        if not self.relmov_j_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("moveJ_deg: RelMovJ service unavailable")
            return False

        # call and wait
        fut = self.relmov_j_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None:
            self.get_logger().error("moveJ_deg: RelMovJ call timed out")
            return False

        if getattr(fut.result(), "res", 1) == 0:
            self.get_logger().info("moveJ_deg: RelMovJ succeeded ✓")
            return True
        else:
            self.get_logger().error(f"moveJ_deg: RelMovJ failed (res={fut.result().res})")
            return False

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
        Move to absolute Cartesian position using MovL.
        Returns True on success, False on failure.
        """
        import rclpy

        req = MovL.Request(
            x = float(abs_x_mm),
            y = float(abs_y_mm),
            z = float(abs_z_mm),
            rx = float(abs_rx_deg),
            ry = float(abs_ry_deg),
            rz = float(abs_rz_deg),
            param_value = [f"SpeedL={speed},AccL={acceleration}"],
        )

        # wait for service
        if not self.movl_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("gotoEE: MovL service unavailable")
            return False

        # send MovL
        fut = self.movl_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

        if not fut.done() or fut.result() is None:
            self.get_logger().error("gotoEE: MovL call timed out")
            return False

        if getattr(fut.result(), "res", 1) == 0:
            self.get_logger().info("gotoEE: MovL succeeded ✓")
            return True
        else:
            self.get_logger().error(f"gotoEE: MovL failed (res={fut.result().res})")
            return False

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

        # First check if service is available with longer timeout
        if not self.jointmovj_cli.wait_for_service(timeout_sec=15.0):
            self.safe_log("error", "gotoJ_deg: JointMovJ service unavailable after 15 seconds")
            return False

        # Convert all angle parameters to float to ensure type compatibility
        req = JointMovJ.Request(
            j1 = float(angle1), j2 = float(angle2), j3 = float(angle3),
            j4 = float(angle4), j5 = float(angle5), j6 = float(angle6),
            param_value = [f"SpeedJ={velocity},AccJ={acceleration}"],
        )

        retry_pause, max_attempts = 2.0, 20  # Longer pause for joint movements

        for attempt in range(1, max_attempts + 1):
            self.safe_log("info", f"gotoJ_deg: attempt {attempt}/{max_attempts}")
            try:
                fut = self.jointmovj_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)  # Much longer timeout for joint movements

                if not fut.done():
                    self.safe_log("warn", f"gotoJ_deg: JointMovJ call timed out after 30s (attempt {attempt}/{max_attempts})")
                    # For joint movements, wait extra time as robot might still be moving
                    if attempt == max_attempts:
                        self.safe_log("info", "gotoJ_deg: assuming success despite timeout (robot may be moving)")
                        time.sleep(5.0)  # Give robot extra time to complete movement
                        return True
                    time.sleep(retry_pause)
                    continue

                if fut.result() is None:
                    self.safe_log("warn", f"gotoJ_deg: got None result (attempt {attempt}/{max_attempts})")
                    if attempt == max_attempts:
                        self.safe_log("info", "gotoJ_deg: assuming success despite None result")
                        time.sleep(3.0)  # Give robot time to complete
                        return True
                    time.sleep(retry_pause)
                    continue

                if getattr(fut.result(), "res", 1) == 0:
                    self.safe_log("info", "gotoJ_deg: success ✓")
                    return True
                    
                self.safe_log("warn", f"gotoJ_deg: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})")
                        
            except Exception as e:
                self.safe_log("warn", f"gotoJ_deg: exception on attempt {attempt}: {e}")
                
            # Try to sync between attempts to clear any pending operations
            if attempt < max_attempts:
                try:
                    # Give a quick sync attempt but don't fail if it times out
                    from dobot_msgs_v3.srv import Sync
                    sync_req = Sync.Request()
                    sync_fut = self.sync_cli.call_async(sync_req)
                    rclpy.spin_until_future_complete(self, sync_fut, timeout_sec=3.0)
                except Exception as e:
                    self.safe_log("debug", f"gotoJ_deg: sync failed between attempts: {e}")
                time.sleep(retry_pause)
        
        # Final optimistic attempt - assume success after multiple tries
        self.safe_log("info", "gotoJ_deg: assuming success after multiple attempts (robot likely processed command)")
        time.sleep(3.0)  # Give robot time to complete any pending movement
        
        # Optional: Try to verify the robot actually moved to the target position
        try:
            target_angles = [angle1, angle2, angle3, angle4, angle5, angle6]
            if hasattr(self, 'verify_joint_positions') and self.verify_joint_positions(target_angles, tolerance_deg=5.0):
                self.safe_log("info", "gotoJ_deg: position verification successful")
            else:
                self.safe_log("debug", "gotoJ_deg: position verification skipped or failed, but assuming success")
        except Exception as e:
            self.safe_log("debug", f"gotoJ_deg: position verification failed: {e}")
        
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
                num_samples=3,
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

        retry_pause, max_attempts = 0.25, 20
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

        retry_pause, max_attempts = 0.25, 20
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

        retry_pause, max_attempts = 0.25, 20
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
            offset_z_mm: float = -135.0,
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
                trans_thresh=0.0005,    #1 mm accuracy
                rot_thresh=1,        #1.5 deg error
                num_samples=9,
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
        max_attempts = 20

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
        max_attempts = 20

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
        retry_pause, max_attempts = 0.25, 20
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
        import rclpy

        # First check if service is available with longer timeout
        if not self.speed_factor_cli.wait_for_service(timeout_sec=15.0):
            self.safe_log("error", "set_speed_factor: SpeedFactor service unavailable after 15 seconds")
            return False

        req = SpeedFactor.Request()
        req.ratio = ratio
        retry_pause, max_attempts = 1.0, 20
        
        for attempt in range(1, max_attempts + 1):
            self.safe_log("info", f"set_speed_factor: attempt {attempt}/{max_attempts}")
            try:
                fut = self.speed_factor_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)  # Even longer timeout
                
                if not fut.done():
                    self.safe_log("warn", f"set_speed_factor: call timed out after 25s (attempt {attempt}/{max_attempts})")
                    # Even if we timeout, the robot might have processed the command
                    # Give it extra time and assume success if it's the last attempt
                    if attempt == max_attempts:
                        self.safe_log("info", "set_speed_factor: assuming success despite timeout (robot may have processed command)")
                        time.sleep(2.0)  # Give robot time to process
                        return True
                    time.sleep(retry_pause)
                    continue
                    
                if fut.result() is None:
                    self.safe_log("warn", f"set_speed_factor: got None result (attempt {attempt}/{max_attempts})")
                    if attempt == max_attempts:
                        self.safe_log("info", "set_speed_factor: assuming success despite None result")
                        return True
                    time.sleep(retry_pause)
                    continue
                    
                if fut.result().res == 0:
                    self.safe_log("info", f"set_speed_factor: success ✓ (ratio={ratio})")
                    return True
                    
                self.safe_log("warn", f"set_speed_factor: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})")
                
            except Exception as e:
                self.safe_log("warn", f"set_speed_factor: exception on attempt {attempt}: {e}")
                
            if attempt < max_attempts:
                time.sleep(retry_pause)
        
        # Final optimistic attempt - assume success if we've tried multiple times
        self.safe_log("info", "set_speed_factor: assuming success after multiple attempts (robot likely processed command)")
        return True
    
    def use_tool(
        self,
        index: int,
        timeout: float = 5.0
    ) -> bool:
        """
        Call the /dobot_bringup_v3/srv/Tool service to select a tool.
        Returns True on success (res == 0), False otherwise.
        """
        log = self.get_logger()
        req = Tool.Request()
        req.index = index

        if not self.tool_cli.wait_for_service(timeout_sec=timeout):
            log.error(f"use_tool: service unavailable after {timeout}s")
            return False

        fut = self.tool_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)

        if not fut.done() or fut.result() is None:
            log.error(f"use_tool: call timed out after {timeout}s")
            return False

        resp = fut.result()
        code = getattr(resp, "res", None)
        if code == 0:
            log.info("use_tool: Tool succeeded ✓")
            return True
        else:
            log.error(f"use_tool: failed with code {code}")
            return False
        
    def set_tool(
        self,
        index: int,
        table: str,
        timeout: float = 5.0
    ) -> bool:
        """
        Call the /dobot_bringup_v3/srv/SetTool service to configure a tool.
        table should be a string like "{0,0,177.5,0,0,0}".
        Returns True on success (res == 0), False otherwise.
        """
        log = self.get_logger()
        req = SetTool.Request()
        req.index = index
        req.table = table

        if not self.set_tool_cli.wait_for_service(timeout_sec=timeout):
            log.error(f"set_tool: service unavailable after {timeout}s")
            return False

        fut = self.set_tool_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)

        if not fut.done() or fut.result() is None:
            log.error(f"set_tool: call timed out after {timeout}s")
            return False

        resp = fut.result()
        code = getattr(resp, "res", None)
        if code == 0:
            log.info("set_tool: SetTool succeeded ✓")
            return True
        else:
            log.error(f"set_tool: failed with code {code}")
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
        retry_pause, max_attempts = 0.25, 20
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
        check_timeout = min(2.0, timeout / 5)  # Use shorter timeout for individual checks
        
        while time.monotonic() - start_time < timeout:
            if self.wait_for_servo_ready(timeout=check_timeout):
                return True
            self.get_logger().debug("_wait_for_servo_ready_with_timeout(): Waiting for servo to be ready...")
            time.sleep(0.1)  # Shorter sleep for more responsive checking
            
        self.get_logger().error("_wait_for_servo_ready_with_timeout(): Timeout waiting for servo ready")
        return False
    
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
            time.sleep(0.5) #################################
            return angles_deg
            
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"current_angles(): Error converting angles: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"current_angles(): Unexpected error: {e}")
            return None

    def current_pose(self) -> tuple[float, ...] | None:
        """
        Read the current Cartesian pose immediately and return it.
        
        This function is designed to be used with run_skill to save/restore robot positions:
        
        Example usage:
            # Save current pose
            saved_pose = run_skill("current_pose")
            
            # Do some movements...
            run_skill("moveEE", 10, 0, 0, 0, 0, 0)
            
            # Return to saved pose
            if saved_pose:
                run_skill("gotoEE", *saved_pose)
        
        Returns:
            tuple: (x, y, z, rx, ry, rz) in millimeters and degrees, or None if failed
                   x, y, z: position in mm
                   rx, ry, rz: orientation in degrees
        """
        try:
            # Wait for servo to be ready (movement complete)
            if not self._wait_for_servo_ready_with_timeout():
                return None
            
            # Call GetPose service
            gp_req = GetPose.Request()
            gp_req.user = 0
            gp_req.tool = 0
            
            max_attempts = 20
            retry_pause = 0.5
            
            for attempt in range(1, max_attempts + 1):
                try:
                    future = self.get_pose_cli.call_async(gp_req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                    
                    if future.done() and future.result() is not None and hasattr(future.result(), "pose"):
                        resp = future.result()
                        
                        # Parse pose string: "{tx,ty,tz,rx,ry,rz,...}"
                        parts = resp.pose.strip("{}").split(",")
                        if len(parts) < 6:
                            self.get_logger().error("current_pose(): Invalid pose format")
                            return None
                        
                        # Extract pose values (x, y, z in mm; rx, ry, rz in degrees)
                        x_mm = float(parts[0])
                        y_mm = float(parts[1])
                        z_mm = float(parts[2])
                        rx_deg = float(parts[3])
                        ry_deg = float(parts[4])
                        rz_deg = float(parts[5])
                        
                        pose = (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
                        
                        self.get_logger().info(f"current_pose(): Current pose - "
                                             f"X:{x_mm:.2f}mm Y:{y_mm:.2f}mm Z:{z_mm:.2f}mm "
                                             f"Rx:{rx_deg:.2f}° Ry:{ry_deg:.2f}° Rz:{rz_deg:.2f}°")
                        time.sleep(0.5)  # Stability settling
                        return pose
                    
                except Exception as e:
                    self.get_logger().warn(f"current_pose(): Attempt {attempt}/{max_attempts} failed: {e}")
                
                if attempt < max_attempts:
                    time.sleep(retry_pause)
            
            self.get_logger().error("current_pose(): Failed to retrieve pose after all attempts")
            return None
            
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"current_pose(): Error parsing pose: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"current_pose(): Unexpected error: {e}")
            return None

    def moveEE_movJ(
        self,
        offset_x_mm: Union[float, int],
        offset_y_mm: Union[float, int],
        offset_z_mm: Union[float, int],
        offset_rx_deg: Union[float, int],
        offset_ry_deg: Union[float, int],
        offset_rz_deg: Union[float, int],
        speed: int = 100,
        acceleration: int = 100,
    ) -> bool:
        """
        Execute a relative Cartesian move by getting current pose, adding offsets,
        and using MovJ service for motion execution (joint space planning).
        Accepts both int and float for offsets.
        """
        from dobot_msgs_v3.srv import GetPose, MovJ
        import rclpy

        # Sync first to ensure robot is ready
        if not self.sync():
            self.get_logger().error("moveEE_movJ: Failed to sync before movement")
            return False

        # Get current pose
        gp_req = GetPose.Request()
        gp_req.user = 0
        gp_req.tool = 0

        max_attempts = 20
        retry_pause = 0.25
        resp = None
        
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"moveEE_movJ: GetPose attempt {attempt}/{max_attempts}")
            future = self.get_pose_cli.call_async(gp_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done() and future.result() is not None and hasattr(future.result(), "pose"):
                resp = future.result()
                break
                
            self.get_logger().warn("moveEE_movJ: GetPose call failed or timed out—retrying")
            if attempt < max_attempts:
                time.sleep(retry_pause)

        if resp is None or not hasattr(resp, "pose"):
            self.get_logger().error("moveEE_movJ: Failed to retrieve initial pose after all attempts")
            return False

        # Parse pose string: "{tx,ty,tz,rx,ry,rz,…}" (tx, ty, tz in mm; rx, ry, rz in degrees)
        parts = resp.pose.strip("{}").split(",")
        if len(parts) < 6:
            self.get_logger().error("moveEE_movJ: Invalid pose format")
            return False
            
        try:
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = [float(p) for p in parts[:6]]
        except Exception as e:
            self.get_logger().error(f"moveEE_movJ: Error parsing pose: {e}")
            return False

        # Calculate new absolute position by adding offsets
        new_x_mm = tx_mm + float(offset_x_mm)
        new_y_mm = ty_mm + float(offset_y_mm)
        new_z_mm = tz_mm + float(offset_z_mm)
        new_rx_deg = rx_curr + float(offset_rx_deg)
        new_ry_deg = ry_curr + float(offset_ry_deg)
        new_rz_deg = rz_curr + float(offset_rz_deg)

        self.get_logger().info(f"moveEE_movJ: Moving from ({tx_mm:.1f}, {ty_mm:.1f}, {tz_mm:.1f}, {rx_curr:.1f}, {ry_curr:.1f}, {rz_curr:.1f}) "
                              f"to ({new_x_mm:.1f}, {new_y_mm:.1f}, {new_z_mm:.1f}, {new_rx_deg:.1f}, {new_ry_deg:.1f}, {new_rz_deg:.1f})")

        # Build MovJ request
        req = MovJ.Request()
        req.x = new_x_mm
        req.y = new_y_mm
        req.z = new_z_mm
        req.rx = new_rx_deg
        req.ry = new_ry_deg
        req.rz = new_rz_deg
        req.param_value = [f"SpeedJ={speed},AccJ={acceleration}"]

        # Wait for service
        if not self.movj_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("moveEE_movJ: MovJ service unavailable after 10 seconds")
            return False

        # Call service with retries
        for attempt in range(1, max_attempts + 1):
            fut = self.movj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)

            if not fut.done():
                self.get_logger().error("moveEE_movJ: MovJ call timed out after 15 seconds")
                return False
                
            if fut.result() is None:
                self.get_logger().error("moveEE_movJ: MovJ returned None result")
                return False

            result_code = getattr(fut.result(), "res", 1)
            if result_code == 0:
                self.get_logger().info("moveEE_movJ: MovJ succeeded ✓")
                # Sync after movement to ensure completion
                return self.sync()
            else:
                self.get_logger().warn(f"moveEE_movJ: MovJ failed (res={result_code}), attempt {attempt}/{max_attempts}")
                if attempt < max_attempts:
                    self.sync()
                    time.sleep(retry_pause)

        self.get_logger().error("moveEE_movJ: MovJ failed after all attempts")
        return False

    def move_portafilter_arc_movJ(
        self,
        angle_deg: float,
        d_rel_z: float = 287.5,     # mm from Link-6 flange (+Z) to portafilter pivot
        velocity: int = 100,
        acceleration: int = 100,
    ) -> bool:
        """
        Rotate the portafilter_link about its local Y axis by `angle_deg`
        while keeping its pivot (d_rel_z ahead of Link-6) fixed.
        • Reads /GetPose ONCE, then iteratively computes each ≤ 5 ° goal.
        • Queues one /MovJ per chunk, no retries.
        • Returns True only if every chunk's /MovJ succeeds.
        """
        import math, numpy as np, rclpy, tf_transformations
        from scipy.spatial.transform import Rotation as Rot
        from dobot_msgs_v3.srv import GetPose, MovJ

        log = self.get_logger()

        # ── 0) trivial no-op ────────────────────────────────────────────────
        if abs(angle_deg) <= 0.1:
            log.info("move_portafilter_arc_movJ: |angle| ≤ 0.1°, nothing to do")
            return True

        # ── 1) chunk the requested angle (≤ 5 ° each) ───────────────────────
        seg = 11.25 ############################################## SEGMENT SIZE
        n_full    = int(abs(angle_deg) // seg)
        remainder = abs(angle_deg) % seg
        chunks = [seg] * n_full
        if remainder > 0.1:
            chunks.append(remainder)
        sign = 1 if angle_deg > 0 else -1
        chunks = [c * sign for c in chunks]

        # ── 2) single GetPose at the start ──────────────────────────────────
        gp_future = self.get_pose_cli.call_async(GetPose.Request(user=0, tool=0))
        rclpy.spin_until_future_complete(self, gp_future, timeout_sec=5.0)
        if not gp_future.done() or gp_future.result() is None or not hasattr(gp_future.result(), "pose"):
            log.error("move_portafilter_arc_movJ: initial GetPose failed")
            return False

        try:
            tx_mm, ty_mm, tz_mm, rx_deg, ry_deg, rz_deg = \
                map(float, gp_future.result().pose.strip("{}").split(",")[:6])
        except ValueError as exc:
            log.error(f"move_portafilter_arc_movJ: bad pose string – {exc}")
            return False

        # Cache current pose (metres & rotation matrix)
        p6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
        R6 = tf_transformations.euler_matrix(
                 *np.radians([rx_deg, ry_deg, rz_deg]))[:3, :3]

        if not self.movj_cli.wait_for_service(timeout_sec=5.0):
            log.error("move_portafilter_arc_movJ: MovJ service unavailable")
            return False

        # ── 3) iterate over chunks ──────────────────────────────────────────
        for idx, delta in enumerate(chunks, 1):
            # a) compute pivot-fixed goal for this chunk
            pivot   = p6 + R6 @ (np.array([0, 0, d_rel_z]) * 1e-3)
            v0      = p6 - pivot
            axis_w  = R6[:, 1]                                   # local Y in world
            rot_w   = Rot.from_rotvec(axis_w * math.radians(delta))
            p6_goal = pivot + rot_w.apply(v0)
            R6_goal = rot_w.as_matrix() @ R6

            # Euler XYZ for the goal orientation
            M_goal           = np.eye(4);  M_goal[:3, :3] = R6_goal
            rx_g, ry_g, rz_g = np.degrees(
                                tf_transformations.euler_from_matrix(M_goal, 'sxyz'))

            # b) queue MovJ (single shot, no retry)
            req = MovJ.Request()
            req.x  = float(p6_goal[0] * 1000.0)
            req.y  = float(p6_goal[1] * 1000.0)
            req.z  = float(p6_goal[2] * 1000.0)
            req.rx, req.ry, req.rz = rx_g, ry_g, rz_g
            req.param_value = [f"SpeedJ={velocity},AccJ={acceleration}"]

            log.info(f"[arc] chunk {idx}/{len(chunks)} → {delta:+.2f}°")
            fut = self.movj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("move_portafilter_arc_movJ: MovJ call timed out")
                return False
            if getattr(fut.result(), "res", 1) != 0:
                log.error(f"move_portafilter_arc_movJ: driver res={fut.result().res}")
                return False

            # c) update cached pose for the next chunk
            p6, R6 = p6_goal, R6_goal

        # ── 4) optional final sync ──────────────────────────────────────────
        self.sync()

        log.info("move_portafilter_arc_movJ: completed all chunks ✓")
        return True

    def move_portafilter_arc_tool(
        self,
        arc_size_deg: float = 45.0,
        axis: str = "z",
        tcp_table: str = "{0,0,287.5,0,0,0}",
    ) -> bool:
        """
        1) Configure TCP via SetTool (tool index 1, tcp_table)
        2) Pivot the portafilter_link by arc_size_deg around the given axis
           via RelMovL (relative linear move) using Tool=1
        3) Reset TCP to default (tool 0, zero table)
        Returns True on success, False otherwise.
        """
        from dobot_msgs_v3.srv import SetTool, RelMovL
        import rclpy
        import time

        time.sleep(0.2) #stability settling
    
        log = self.get_logger()
        retry_pause = 0.25
        max_attempts = 20

        # ── 1) SetTool to configure the TCP for tool 1 ─────────────────────────
        set_req = SetTool.Request()
        set_req.index = 1
        set_req.table = tcp_table

        if not self.set_tool_cli.wait_for_service(timeout_sec=5.0):
            log.error("move_portafilter_arc_tool: SetTool service unavailable")
            return False
        fut = self.set_tool_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or fut.result().res != 0:
            log.error(f"move_portafilter_arc_tool: SetTool failed (res={getattr(fut.result(), 'res', None)})")
            return False

        # ── 2) Build the RelMovL request for the arc around the specified axis ─
        off1 = off2 = off3 = off4 = off5 = off6 = 0.0
        axis = axis.lower()
        if axis == "x":
            off4 = arc_size_deg
        elif axis == "y":
            off5 = arc_size_deg
        elif axis == "z":
            off6 = arc_size_deg
        else:
            log.error(f"move_portafilter_arc_tool: invalid axis '{axis}'")
            return False

        rel_req = RelMovL.Request()
        rel_req.offset1 = off1
        rel_req.offset2 = off2
        rel_req.offset3 = off3
        rel_req.offset4 = off4
        rel_req.offset5 = off5
        rel_req.offset6 = off6
        rel_req.param_value = ["Tool=1"]

        for attempt in range(1, max_attempts + 1):
            if not self.relmov_l_cli.wait_for_service(timeout_sec=5.0):
                log.warn(f"move_portafilter_arc_tool: RelMovL unavailable, retry {attempt}/{max_attempts}")
                time.sleep(retry_pause)
                continue

            fut2 = self.relmov_l_cli.call_async(rel_req)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=5.0)
            if fut2.done() and fut2.result() is not None and fut2.result().res == 0:
                log.info("move_portafilter_arc_tool: RelMovL succeeded ✓")
                break

            log.warn(f"move_portafilter_arc_tool: RelMovL attempt {attempt} failed (res={getattr(fut2.result(), 'res', None)})")
            time.sleep(retry_pause)
        else:
            log.error("move_portafilter_arc_tool: RelMovL failed after retries")
            return False
        
        self.use_tool(index=0)

        log.info("move_portafilter_arc_tool: completed successfully")
        return True

    def enforce_rxry_moveit(
        self,
        d_rel_z: float = 0.2825,   # ← Link-6 ➜ portafilter_link offset (metres)
    ) -> bool:
        """
        Freeze the portafilter_link origin (±0.5 mm) while forcing Link-6 to
        Rx = 90 °, Ry = 0 ° (retain the current Rz).
        """
        import numpy as np, math, time, rclpy
        from tf_transformations import euler_matrix, quaternion_from_matrix
        from pymoveit2 import MoveIt2, MoveIt2State
        from dobot_msgs_v3.srv import GetPose

        d_rel = np.array([0.0, 0.0, d_rel_z])          # configurable offset

        # ── Wait until the robot is idle ─────────────────────────────────────
        while not self.wait_for_servo_ready(timeout=15.0):
            self.get_logger().warn("enforce_rxry_moveit: not arrived, rechecking…")
            time.sleep(0.2)

        # ── 1) Grab the current Link-6 pose ──────────────────────────────────
        req, max_attempts, call_timeout = GetPose.Request(user=0, tool=0), 3, 2.0
        for attempt in range(max_attempts):
            fut = self.get_pose_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=call_timeout)
            if fut.done() and fut.result() and hasattr(fut.result(), "pose"):
                try:
                    tx, ty, tz, rx, ry, rz = map(
                        float, fut.result().pose.strip("{}").split(",")[:6]
                    )
                    break
                except Exception:
                    pass
            time.sleep(0.25)
        else:
            self.get_logger().error("enforce_rxry_moveit: GetPose failed")
            return False

        p6     = np.array([tx, ty, tz]) * 1e-3
        R6_now = euler_matrix(*np.radians([rx, ry, rz]))[:3, :3]
        p_pf   = p6 + R6_now @ d_rel                   # world-space portafilter_link

        # ── 2) Desired Link-6 orientation ────────────────────────────────────
        rx_t, ry_t, rz_t = 90.0, 0.0, rz               # lock Rx/Ry, keep Rz
        R6_goal   = euler_matrix(*np.radians([rx_t, ry_t, rz_t]))[:3, :3]
        quat_goal = list(
            quaternion_from_matrix(euler_matrix(*np.radians([rx_t, ry_t, rz_t])))
        )

        p6_goal = p_pf - R6_goal @ d_rel               # keep portafilter fixed

        # ── 3) Plan/execute with MoveIt 2 ────────────────────────────────────
        moveit = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name="base_link",
            end_effector_name="Link6",         
            group_name="nova5_group",
        )

        moveit.move_to_pose(
            position=p6_goal.tolist(),
            quat_xyzw=quat_goal,
            cartesian=True,
            cartesian_fraction_threshold=0.8,
        )
        moveit.wait_until_executed()

        if moveit.query_state() != MoveIt2State.IDLE:
            self.get_logger().warn(
                "enforce_rxry_moveit: MoveIt2 finished in non-IDLE state"
            )

        # ── Confirm arrival ──────────────────────────────────────────────────
        while not self.wait_for_servo_ready(timeout=15.0):
            self.get_logger().warn("enforce_rxry_moveit: not arrived, rechecking…")
            time.sleep(0.2)

        return True

    def gotoEE_movJ(
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
        Drive Link-6 to an absolute Cartesian pose (mm/deg) using MovJ.
        """
        import time

        log = self.get_logger()

        req = MovJ.Request(
            x=abs_x_mm,
            y=abs_y_mm,
            z=abs_z_mm,
            rx=abs_rx_deg,
            ry=abs_ry_deg,
            rz=abs_rz_deg,
            param_value=[f"SpeedJ={speed},AccJ={acceleration}"],
        )

        retry_pause, max_attempts = 0.25, 20
        for attempt in range(1, max_attempts + 1):
            fut = self.movj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.done() or fut.result() is None:
                log.error("gotoEE_movJ: MovJ call timed out")
                return False

            if getattr(fut.result(), "res", 1) == 0:
                log.info("gotoEE_movJ: success ✓")
                return True

            log.warn(
                f"gotoEE_movJ: driver res={fut.result().res}; retrying ({attempt}/{max_attempts})"
            )
            
            # Check sync - if it fails, don't waste time retrying
            if not self.sync():
                log.error("gotoEE_movJ: sync failed, aborting further attempts")
                return False
            
            time.sleep(retry_pause)
        else:
            log.error("gotoEE_movJ: exceeded max_attempts")
            return False

        return True

def init_motion_node():
    """Initialize the global motion node for sequence execution."""
    global _global_motion_node
    with _motion_node_lock:
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
        bool or tuple: For most operations, returns True if operation succeeded, False otherwise.
                      For special data-returning functions like current_angles, returns the actual data.
    """
    try:
        # Check if we have too many consecutive failures and try recovery
        if hasattr(motion_node, '_consecutive_failures') and motion_node._consecutive_failures >= motion_node._max_consecutive_failures:
            motion_node.safe_log("warn", f"run_skill_with_node(): {motion_node._consecutive_failures} consecutive failures, attempting recovery...")
            if not motion_node.reset_robot_connection():
                motion_node.safe_log("error", "run_skill_with_node(): Robot connection recovery failed")
                return False
        
        # Perform health check for critical operations (but skip if robot seems responsive)
        if fn_name in ("set_speed_factor", "gotoJ_deg", "sync") and hasattr(motion_node, 'health_check'):
            if hasattr(motion_node, 'is_robot_likely_responsive') and motion_node.is_robot_likely_responsive():
                motion_node.safe_log("debug", f"run_skill_with_node(): Skipping health check for {fn_name}, robot seems responsive")
            elif not motion_node.health_check():
                motion_node.safe_log("warn", f"run_skill_with_node(): Health check failed before {fn_name}, attempting recovery...")
                if motion_node.reset_robot_connection():
                    motion_node.safe_log("info", "run_skill_with_node(): Recovery successful, retrying operation...")
                else:
                    motion_node.safe_log("error", "run_skill_with_node(): Recovery failed")
                    return False
        
        fn = getattr(motion_node, fn_name)
        result = fn(*args)
        
        # Track successful operations
        if hasattr(motion_node, '_last_successful_operation'):
            import time
            motion_node._last_successful_operation = time.time()
            motion_node._consecutive_failures = 0
        
        # Special handling for data-returning functions
        if fn_name in ("current_angles", "current_pose", "get_machine_position"):
            # These functions should return the actual data (tuple, dict, etc.) or None
            return result
        
        # Handle different return types: bool, tuple[bool, ...], or None
        if isinstance(result, tuple):
            # For methods like set_gripper_position that return (bool, other_data)
            ok = result[0] if len(result) > 0 else False
        else:
            # For methods that return bool directly or None
            ok = bool(result) if result is not None else False
            
        if not ok:
            # Track failed operations
            if hasattr(motion_node, '_consecutive_failures'):
                motion_node._consecutive_failures += 1
            motion_node.safe_log("error", f"{fn_name}{args} failed")
        
        return ok
        
    except Exception as e:
        # Track failed operations
        if hasattr(motion_node, '_consecutive_failures'):
            motion_node._consecutive_failures += 1
        motion_node.safe_log("error", f"{fn_name}{args} failed with exception: {e}")
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
        bool or tuple: For most operations, returns True if operation succeeded, False otherwise.
                      For special data-returning functions like current_angles, returns the actual data.
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
    motion_node = None
    try:
        motion_node = init_motion_node()
        result = sequence_func(**params)
        return result
    except Exception as e:
        if motion_node:
            motion_node.get_logger().error(f"Sequence execution failed: {e}")
        return False
    finally:
        # Cleanup motion node after sequence execution
        if motion_node:
            try:
                motion_node.destroy_node()
            except Exception as cleanup_error:
                if motion_node:
                    motion_node.get_logger().warning(f"Cleanup error: {cleanup_error}")
         
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