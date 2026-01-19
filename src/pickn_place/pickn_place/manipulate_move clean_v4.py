# Python Imports
import sys
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ament_index_python.packages import get_package_share_directory

# ROS Imports
import rclpy   
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import tf_transformations

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from control_msgs.action import FollowJointTrajectory

import threading
from rclpy.executors import SingleThreadedExecutor

from tf_transformations import euler_matrix, quaternion_from_matrix
from pymoveit2 import MoveIt2, MoveIt2State

class robot_perception(Node):
    def __init__(self):
        super().__init__("robot_perception")
        self._percep_exec = SingleThreadedExecutor()
        self._percep_exec.add_node(self)
        threading.Thread(target=self._percep_exec.spin, daemon=True).start()

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
                    reference_frame, target_frame, rclpy.time.Time())
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
)

class robot_motion(Node):
    def __init__(self):
        super().__init__("robot_motion")          # keep the node alive for the whole run

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
            "SetTool":             self.set_tool_cli
        }

        missing = [name for name, cli in service_map.items()
                   if not cli.wait_for_service(timeout_sec=timeout)]
        if missing:
            self.get_logger().error(
                f"⚠️  services unavailable after {timeout}s: {', '.join(missing)}")
        else:
            self.get_logger().info("✅  all motion-control services are ready")

    def wait_for_servo_ready(self, timeout: float = 15.0) -> bool:
        """
        Poll /get_servo_status (Trigger) until status == 0 (READY) or timeout.
        """

        start = self.get_clock().now().nanoseconds * 1e-9
        while True:
            req = Trigger.Request()
            future = self.servo_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            res = future.result()
            if res is not None:
                try:
                    code = int(res.message)
                except:
                    code = None
                if code == 0:
                    return True
                self.get_logger().info(f"Servo busy (status={code}), waiting…")
            else:
                self.get_logger().warn("No response from /get_servo_status")

            if (self.get_clock().now().nanoseconds * 1e-9 - start) > timeout:
                self.get_logger().error("Timeout waiting for servo READY")
                return False
            time.sleep(0.1)

    def move_portafilter_arc_movJ(
        self,
        angle_deg: float,
        d_rel_z: float = 278.5,     # mm from Link-6 flange (+Z) to portafilter pivot
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
        seg = 20.0 ############################################## SEGMENT SIZE
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
    
    def enforce_rxry(self) -> bool:
        """
        Override Link6’s Rx→90°, Ry→0° (keep current Rz) while freezing
        the world‐space position of portafilter_link to ±0.5 mm.
        Uses the /dobot_bringup_v3/srv/MovJ service (Link6 planning) instead of MoveIt2.
        Retries any failed service call every 0.25 s up to 10 attempts; returns False on total failure.
        Returns True on success, False on any failure.
        """
        from tf_transformations import euler_matrix
        from dobot_msgs_v3.srv import GetPose, MovJ

        # ── 1) fixed offset from Link6 origin → portafilter_link origin (m)
        d_rel = np.array([0.0, 0.0, 0.2825])

        retry_pause = 0.25
        max_attempts = 10

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
    
    def move_portafilter_arc_tool(
        self,
        arc_size_deg: float,
        axis: str = "z",
        tcp_table: str = "{0,0,282.5,0,0,0}",
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
        max_attempts = 5

        # ── 1) SetTool to configure the TCP for tool 1 ─────────────────────────
        set_req = SetTool.Request()
        set_req.index = 1
        set_req.table = tcp_table

        if not self.set_tool_cli.wait_for_service(timeout_sec=5.0):
            log.error("move_portafilter_arc: SetTool service unavailable")
            return False
        fut = self.set_tool_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or fut.result().res != 0:
            log.error(f"move_portafilter_arc: SetTool failed (res={getattr(fut.result(), 'res', None)})")
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
            log.error(f"move_portafilter_arc: invalid axis '{axis}'")
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
                log.warn(f"move_portafilter_arc: RelMovL unavailable, retry {attempt}/{max_attempts}")
                time.sleep(retry_pause)
                continue

            fut2 = self.relmov_l_cli.call_async(rel_req)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=5.0)
            if fut2.done() and fut2.result() is not None and fut2.result().res == 0:
                log.info("move_portafilter_arc: RelMovL succeeded ✓")
                break

            log.warn(f"move_portafilter_arc: RelMovL attempt {attempt} failed (res={getattr(fut2.result(), 'res', None)})")
            time.sleep(retry_pause)
        else:
            log.error("move_portafilter_arc: RelMovL failed after retries")
            return False
        
        self.use_tool(index=0)

        log.info("move_portafilter_arc: completed successfully")
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
            self.get_logger().warn("move_to(): not arrived, rechecking…")
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
            self.get_logger().warn("move_to(): not arrived, rechecking…")
            time.sleep(0.2)

        return True


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
        • Enable drag-mode (StartDrag) → let the arm “relax”.
        • Wait `settling_time` s to dissipate any spring-back.
        • Disable drag-mode (StopDrag).
        Retries driver-error responses up to `max_attempts`, but aborts on
        transport time-outs.  Returns **True** on full success.
        """

        import time, rclpy

        max_attempts  = 3
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
                time.slee(0.5)
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

        if future.result().res == 0:
            log.info("sync: motion queue empty ✓")
            return True
        else:
            log.error(f"sync: queue not cleared (res={future.result().res})")
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
                trans_thresh=0.003,    #2 mm accuracy
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
            time.sleep(retry_pause)

        log.error("move_to: failed after maximum retries")
        return False
    
    def moveEE_movJ(
        self,
        offset_x_mm: float,
        offset_y_mm: float,
        offset_z_mm: float,
        offset_rx_deg: float,
        offset_ry_deg: float,
        offset_rz_deg: float,
        velocity: int = 100,
        acceleration: int = 100,
    ) -> bool:
        """
        Relative Cartesian move executed with *MovJ* (joint–space planning).
        """

        from dobot_msgs_v3.srv import GetPose, MovJ
        import rclpy, time

        log = self.get_logger()
        retry_pause, max_attempts = 0.25, 5

        # ---- 1) current pose ------------------------------------------------
        gp_req = GetPose.Request(user=0, tool=0)
        fut = self.get_pose_cli.call_async(gp_req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None or not hasattr(fut.result(), "pose"):
            log.error("moveEE_movJ: GetPose call failed")
            return False

        try:
            x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = \
                map(float, fut.result().pose.strip("{}").split(",")[:6])
        except Exception as exc:
            log.error(f"moveEE_movJ: bad pose string – {exc}")
            return False

        # ---- 2) apply offsets ----------------------------------------------
        x_goal = x_mm + offset_x_mm
        y_goal = y_mm + offset_y_mm
        z_goal = z_mm + offset_z_mm
        rx_goal = rx_deg + offset_rx_deg
        ry_goal = ry_deg + offset_ry_deg
        rz_goal = rz_deg + offset_rz_deg

        # ---- 3) build MovJ request -----------------------------------------
        req = MovJ.Request()
        req.x,  req.y,  req.z  = x_goal,  y_goal,  z_goal
        req.rx, req.ry, req.rz = rx_goal, ry_goal, rz_goal
        req.param_value = [f"SpeedJ={velocity},AccJ={acceleration}"]

        # ---- 4) single wait-for-service gate -------------------------------
        if not self.movj_cli.wait_for_service(timeout_sec=5.0):
            log.error("moveEE_movJ: MovJ service unavailable")
            return False

        # ---- 5) retry loop on driver-error ----------------------------------
        for attempt in range(1, max_attempts + 1):
            log.info(f"moveEE_movJ: MovJ attempt {attempt}/{max_attempts}")
            fut2 = self.movj_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=10.0)

            if not fut2.done() or fut2.result() is None:
                log.error("moveEE_movJ: MovJ call timed out")
                return False      # do *not* retry on transport failure

            if getattr(fut2.result(), "res", 1) == 0:
                log.info("moveEE_movJ: success ✓")
                return True

            log.warn(f"moveEE_movJ: driver res={fut2.result().res}; retrying…")
            time.sleep(retry_pause)

        log.error("moveEE_movJ: failed after maximum retries")
        return False

    def moveEE(
        self,
        offset_x_mm: float,
        offset_y_mm: float,
        offset_z_mm: float,
        offset_rx_deg: float,
        offset_ry_deg: float,
        offset_rz_deg: float,
    ) -> bool:
        """
        Execute a relative Cartesian move by calling the RelMovL service
        instead of fetching current pose and adding offsets manually.
        """
        from dobot_msgs_v3.srv import RelMovL
        import rclpy

        # build request
        req = RelMovL.Request()
        req.offset1 = offset_x_mm
        req.offset2 = offset_y_mm
        req.offset3 = offset_z_mm
        req.offset4 = offset_rx_deg
        req.offset5 = offset_ry_deg
        req.offset6 = offset_rz_deg
        req.param_value = ["Tool=0"]

        # wait for service
        if not self.relmov_l_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("moveEE: RelMovL service unavailable")
            return False

        # call and wait
        fut = self.relmov_l_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None:
            self.get_logger().error("moveEE: RelMovL call timed out")
            return False

        if getattr(fut.result(), "res", 1) == 0:
            self.get_logger().info("moveEE: RelMovL succeeded ✓")
            return True
        else:
            self.get_logger().error(f"moveEE: RelMovL failed (res={fut.result().res})")
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

        # build request
        req = RelMovJ.Request()
        req.offset1 = offset1
        req.offset2 = offset2
        req.offset3 = offset3
        req.offset4 = offset4
        req.offset5 = offset5
        req.offset6 = offset6
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
            time.sleep(retry_pause)
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
            time.sleep(retry_pause)
        else:
            log.error("gotoJ_deg: exceeded max_attempts")
            return False

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

        retry_pause, max_attempts = 0.25, 5
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
            time.sleep(retry_pause)
        else:
            log.error("gotoEE_movJ: exceeded max_attempts")
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
            offset_z_mm: float = -127.5,
    ) -> bool:
        """
        Go to the YAML-defined “approach_pose” of `target_tf`, then apply local
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
            time.sleep(retry_pause)

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
        Drive to <machine>/<point>.approach_pose (world frame) via MovL.    # run_skill("sync")
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
            time.sleep(retry_pause)

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
            time.sleep(retry_pause)

        log.error("mount_machine: exceeded max_attempts")
        return False
    
    def grab_tool(
            self,
            target_tf: str,
            speed: int = 100,
            acceleration: int = 100,
            offset_x_mm: float = -6.0,
            offset_y_mm: float = -5.0,
            offset_z_mm: float = -135,
    ) -> bool:
        """
        Drive Link6 to the pre-configured “grab” pose for *target_tf* and
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
                rot_thresh=1.8,        #1.5 deg error
                num_samples=10,
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
            time.sleep(retry_pause)

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
                time.sleep(retry_pause)
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
        offset1: tuple[float, float, float, float, float, float],
        offset2: tuple[float, float, float, float, float, float],
        param_value: list[str] | None = None,
    ) -> bool:
        """
        Execute an arc motion by specifying two offset poses relative to the current pose via Arc service.

        Returns True on success (res==0), False on timeout or driver error.
        """
        # 1) synchronize to ensure an up-to-date pose
        self.sync()

        # parse optional user/tool overrides from param_value
        user, tool = 0, 0
        pv = param_value or []
        for p in pv:
            if "=" not in p:
                continue
            key, val = p.split("=", 1)
            key_l = key.strip().lower()
            if key_l == "user":
                try:
                    user = int(val)
                except ValueError:
                    pass
            elif key_l == "tool":
                try:
                    tool = int(val)
                except ValueError:
                    pass

        # 2) fetch current pose via GetPose, with any overrides
        fut = self.get_pose_cli.call_async(GetPose.Request(user=user, tool=tool))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not hasattr(fut.result(), "pose"):
            self.get_logger().error("move_arc: GetPose call failed")
            return False

        # parse pose string into components
        try:
            vals = [float(v) for v in fut.result().pose.strip("{}").split(",")[:6]]
            cx, cy, cz, crx, cry, crz = vals
        except Exception as exc:
            self.get_logger().error(f"move_arc: bad pose string – {exc}")
            return False

        # log starting pose
        start_pose = {"x": cx, "y": cy, "z": cz, "rx": crx, "ry": cry, "rz": crz}
        self.get_logger().info(f"[move_arc] start_pose: {start_pose}")

        # 3) compute the two arc waypoints
        dx1, dy1, dz1, drx1, dry1, drz1 = offset1
        dx2, dy2, dz2, drx2, dry2, drz2 = offset2

        x1, y1, z1    = cx + dx1, cy + dy1, cz + dz1
        rx1, ry1, rz1 = crx + drx1, cry + dry1, crz + drz1
        x2, y2, z2    = cx + dx2, cy + dy2, cz + dz2
        rx2, ry2, rz2 = crx + drx2, cry + dry2, crz + drz2

        point1 = {"x": x1, "y": y1, "z": z1, "rx": rx1, "ry": ry1, "rz": rz1}
        point2 = {"x": x2, "y": y2, "z": z2, "rx": rx2, "ry": ry2, "rz": rz2}
        self.get_logger().info(f"[move_arc] point1: {point1}")
        self.get_logger().info(f"[move_arc] point2: {point2}")

        # stash datalog for inspection
        self.datalog_lastrun_move_arc = {
            "start_pose": start_pose,
            "point1":     point1,
            "point2":     point2,
        }

        # 4) build and send Arc request, passing through param_value
        req = Arc.Request()
        req.x1, req.y1, req.z1    = x1,  y1,  z1
        req.rx1, req.ry1, req.rz1 = rx1, ry1, rz1
        req.x2, req.y2, req.z2    = x2,  y2,  z2
        req.rx2, req.ry2, req.rz2 = rx2, ry2, rz2
        req.param_value           = pv

        # 5) retry logic
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut2 = self.arc_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=10.0)

            if not fut2.done() or fut2.result() is None:
                self.get_logger().error("move_arc: call timed out")
                return False

            if fut2.result().res == 0:
                self.get_logger().info(f"move_arc: success ✓")
                return True

            self.get_logger().warn(
                f"move_arc: driver res={fut2.result().res}; retrying ({attempt}/{max_attempts})"
            )
            time.sleep(retry_pause)

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

        Returns True on success, False on timeout or driver error.
        """
        self.sync()  # sync first for accurate get pose

        # parse optional user/tool overrides from param_value
        user, tool = 0, 0
        pv = param_value or []
        for p in pv:
            if "=" not in p:
                continue
            key, val = p.split("=", 1)
            key_l = key.strip().lower()
            if key_l == "user":
                try:
                    user = int(val)
                except ValueError:
                    pass
            elif key_l == "tool":
                try:
                    tool = int(val)
                except ValueError:
                    pass

        # 1) fetch current pose via GetPose service, using any overridden user/tool
        fut = self.get_pose_cli.call_async(GetPose.Request(user=user, tool=tool))
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

        # 3) build Circle3 request, passing along the same param_value list
        req = Circle3.Request()
        req.count = count
        req.x1, req.y1, req.z1    = x1,  y1,  z1
        req.rx1, req.ry1, req.rz1 = rx1, ry1, rz1
        req.x2, req.y2, req.z2    = x2,  y2,  z2
        req.rx2, req.ry2, req.rz2 = rx2, ry2, rz2
        req.param_value           = pv

        # 4) call service with retries
        retry_pause, max_attempts = 0.25, 5
        for attempt in range(1, max_attempts + 1):
            fut2 = self.circle3_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut2, timeout_sec=5.0)

            if not fut2.done() or fut2.result() is None:
                self.get_logger().error("move_circle: call timed out")
                return False

            if fut2.result().res == 0:
                self.get_logger().info(f"move_circle: success ✓ (count={count})")
                return True

            self.get_logger().warn(
                f"move_circle: driver res={fut2.result().res}; retrying ({attempt}/{max_attempts})"
            )
            time.sleep(retry_pause)

        self.get_logger().error("move_circle: failed after maximum retries")
        return False

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
        `table` should be a string like "{0,0,177.5,0,0,0}".
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


# --------------------------------------------------------------------------- #
#  main() – sequential execution
# --------------------------------------------------------------------------- #
def main(args=None):

    rclpy.init(args=args)
    motion_node = robot_motion()

    # helper: call fn and abort on failure ----------------------------------
    def run_skill(fn_name: str, *args):
        fn = getattr(motion_node, fn_name)
        ok = fn(*args)
        if not ok:
            motion_node.get_logger().error(f"{fn_name}{args} failed – aborting")
            motion_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    # ---------------------------------------------------------------------- #
    #  TEST SEQUENCE 
    # ---------------------------------------------------------------------- #
    # run_skill("set_speed_factor", 100)

    # run_skill("gotoEE_movJ", -420.0, 315.0, 210.0, 90.0, 0.0,-90.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("move_portafilter_arc_tool", -45.0,"z","{0,0,282.5,0,0,0}")
    # run_skill("moveEE", 0.0, 0.0,-100.0, 0.0, 0.0, 0.0)
    # run_skill("moveEE", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("move_portafilter_arc_tool", 45.0,"z","{0,0,282.5,0,0,0}")

    # run_skill("moveEE", 0.0, -230.0, 0.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("move_portafilter_arc_tool", -35.0,"z","{0,0,282.5,0,0,0}")
    # run_skill("sync")
    # run_skill("move_portafilter_arc_movJ", -10.0, 282.5)
    # run_skill("sync")
    # run_skill("moveEE_movJ", 0.0, 0.0,-100.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("moveEE_movJ", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("sync")
    # run_skill("move_portafilter_arc_movJ", 10.0, 282.5)
    # run_skill("move_portafilter_arc_tool", 35.0,"z","{0,0,282.5,0,0,0}")
    
    # run_skill("moveEE", 0.0,-230.0, 0.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("sync")
    # run_skill("move_portafilter_arc_movJ", -45.0, 282.5)
    # run_skill("moveEE", 0.0, 0.0,-100.0, 0.0, 0.0, 0.0)
    # run_skill("moveEE", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0)
    # run_skill("sync")
    # run_skill("enforce_rxry")
    # run_skill("sync")
    # run_skill("move_portafilter_arc_movJ", 45.0, 282.5)

    # run_skill("move_to", "single_portafilter", 0.28)
    # run_skill("sync")
    # run_skill("approach_tool", "single_portafilter", 100, 100)
    # run_skill("sync")
    # run_skill("grab_tool", "single_portafilter", 100, 10)
    # time.sleep(10)

    # # 0) acquire steam‑wand marker position ----------------------------------------
    # run_skill("set_speed_factor", 100)

    # run_skill("set_gripper_position", 255, 0, 255)
    # run_skill("gotoJ_deg", -37.507774,-9.732915,-120.128281,-72.867943,-82.007668,10.413922)  # steam‑wand home
    # for _ in range(4):
    #     run_skill("move_to", "left_steam_wand", 0.27)
    #     run_skill("sync")
    # run_skill("get_machine_position", "left_steam_wand", 3)

    # # align wand -------------------------------------------------------------
    # run_skill("mount_machine", "left_steam_wand", "fix_wand")
    # run_skill("sync")
    # run_skill("set_gripper_position", 255, 200, 255)
    # run_skill("gotoJ_deg", -38.211334,-67.804291,-32.895622,-81.891319,-76.224342,4.003191)
    # run_skill("sync")
    # run_skill("set_gripper_position", 255, 0, 255)

    # # re aquire position -----------------------------------------------------------
    # time.sleep(1.0)
    # run_skill("moveEE", -10.0, -5.0, 0.0, 0.0, 0.0, 0.0)
    # for _ in range(4):
    #     run_skill("move_to", "left_steam_wand", 0.28)
    #     run_skill("sync")
    # run_skill("get_machine_position", "left_steam_wand", 6)
    # run_skill("gotoJ_deg", -37.507774,-9.732915,-120.128281,-72.867943,-82.007668,10.413922)

    # # 1) pick up the pitcher -------------------------------------------------
    # run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)   # home
    # run_skill("sync")
    # run_skill("move_to", "milk_frother_2", 0.30)
    # run_skill("set_gripper_position", 255, 180, 255, False)
    # run_skill("sync")
    # run_skill("approach_tool", "milk_frother_2", 100, 60)
    # run_skill("sync")
    # run_skill("set_speed_factor", 60)
    # run_skill("grab_tool", "milk_frother_2", 50, 10)
    # run_skill("sync")
    # run_skill("set_gripper_position", 255, 255)
    # run_skill("set_speed_factor", 50)
    # run_skill("moveEE", -100.0, 0.0, 20.0, 0.0, 0.0, 0.0)  # up 
    # run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455,-84.523247, -67.974960, 13.926845)

    # # 2) steam‑wand mounting -------------------------------------------------
    # run_skill("set_DO", 1, 0)     
    # time.sleep(0.8)
    # run_skill("set_DO", 1, 0)
    # run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960,  13.926845) # steam‑wand home
    # run_skill("approach_machine", "left_steam_wand", "deep_froth")
    # run_skill("sync")
    # run_skill("set_speed_factor", 20)
    # run_skill("mount_machine",    "left_steam_wand", "deep_froth")
    # run_skill("sync")
    # run_skill("set_speed_factor", 50)
    # run_skill("set_DO", 1, 0);  
    # time.sleep(3) 
    # run_skill("set_speed_factor", 10)
    # run_skill("set_DO", 1, 0)
    # run_skill("approach_machine", "left_steam_wand", "deep_froth")
    # run_skill("sync")
    # run_skill("set_speed_factor", 60)
    # run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)

    # # pour milk ---------------------------------------------------------------
    # run_skill("gotoEE_movJ", -200.0, -300.0, 250.0, 90.0, 0.0, 0.0)
    # run_skill("sync")

    # # circle pour milk --------------------------------------------------------
    # run_skill("set_tool", 1, "{80,-30,150,0,0,0}")
    # run_skill("use_tool", 1)
    # run_skill("set_speed_factor", 15)  
    # run_skill("move_circle", 6,
    #     ( -30.0,  0.0, 0.0, 0.0, 0.0, 0.0),    # point1  offset1
    #     ( -15.0,-15.0, 0.0, 0.0, 0.0, 0.0),    # point2  offset2
    #     ["tool=1"])
    # run_skill("sync")
    # run_skill("use_tool", 0)
    # run_skill("set_speed_factor", 100)
    # run_skill("set_speed_factor", 80)
    # run_skill("moveEE", -30.0, 0.0, -50.0, 0.0, 30.0, 0.0)
    # run_skill("sync")
    # run_skill("use_tool", 1)
    # run_skill("set_speed_factor", 20)
    # run_skill("move_circle", 6,
    #     ( -20.0,  0.0, 0.0, 0.0, 80.0, 0.0),    # point1 offset1
    #     ( -10.0,-10.0, 0.0, 0.0, 80.0, 0.0),    # point2 offset2
    #     ["tool=1"])
    # run_skill("sync")
    # run_skill("set_speed_factor", 100)
    # run_skill("use_tool", 0)
    # run_skill("moveEE", 0.0, 0.0, -10.0, 0.0, 0.0, 0.0)

    # # arc pour milk -----------------------------------------------------------
    # run_skill("use_tool", 0)
    # run_skill("set_speed_factor", 100)
    # run_skill("moveEE", -34.0, 0.0, 0.0, 0.0, 60.0, 0.0)
    # run_skill("sync")
    # run_skill("set_speed_factor", 70)
    # run_skill("move_arc",                  # five-segment arc
    #     (10.0, 0.0, -10.0, 0.0, 60.0, 0.0),  # point1 offset1
    #     (20.0, 0.0, 30.0, 0.0, -40.0, 0.0),)  # point2 offset2
    # run_skill("moveEE", -15.0, 0.0, -20.0, 0.0, 50.0, 0.0)
    # run_skill("sync")
    # run_skill("move_arc",                  # five-segment arc
    #     (10.0, 0.0, -10.0, 0.0, 60.0, 0.0),  # point1 offset1
    #     (20.0, 0.0, 30.0, 0.0, -40.0, 0.0),)  # point2 offset2
    # run_skill("sync")
    # run_skill("set_speed_factor", 30)

    # run_skill("moveEE", -10.0, 0.0, -20.0, 0.0, 50.0, 0.0)
    # run_skill("sync")
    # run_skill("set_speed_factor", 70)
    # run_skill("move_arc",                 # five-segment arc
    #     (10.0, 0.0, -10.0, 0.0, 60.0, 0.0),  # point1 offset1
    #     (20.0, 0.0, 30.0, 0.0, -40.0, 0.0),)  # point2 offset2
    # run_skill("sync")
    # run_skill("set_speed_factor", 40)
    # run_skill("moveEE", 35.0, 0.0, -25.0, 0.0, 60.0, 0.0)
    # run_skill("sync")
    # run_skill("set_speed_factor", 100)
    # run_skill("moveEE", 0.0, 0.0, -10.0, 0.0, 0.0, 0.0)
    # time. sleep(2.0)
    # run_skill("gotoEE", -260.0, -300.0, 250.0, 90.0, -10.0, 0.0)

    # # 5) carry to milk dispenser ---------------------------------------------
    # run_skill("moveJ_deg", 60.0, 0.0, 0.0, 0.0, 0.0, 0.0)        # ccw joint‑1
    # run_skill("gotoJ_deg", 31.479187, -66.335266, -97.210228, -20.666107, -61.815777, 10.486634)   # pre‑drop
    # run_skill("gotoJ_deg", 26.482991,-80.889463,-56.757608,-46.402478,-66.829720,10.099471)   # drops
    # #693.898314,156.614374,-65.443329,93.985262,-8.538911,92.715915
    # run_skill("moveEE", 0.0, 0.0, -20.0, 0.0, 0.0, 0.0)          # down
    # run_skill("sync")
    # run_skill("set_gripper_position", 255, 180) 
    # time.sleep(0.6)
    # run_skill("moveEE", -150.0, -15.0, 20.0, 0.0, 0.0, 0.0)        # back + up
    # run_skill("sync")
    # run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845) # home
    # run_skill("set_gripper_position", 255, 0, 255, False)

    ##################################################################
    # run_skill("gotoJ_deg",  25.297836,-22.044975,-130.264145,-27.691168,-64.685471,-0.000148)
    # run_skill("moveEE", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    # run_skill("moveEE", 0.0, 0.0, 0.0, -2.0, 0.0, 0.0)
    # run_skill("move_portafilter_arc_tool", -45.0,"z","{0,0,282.5,0,0,0}")
    # run_skill("moveEE", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    #####################################################################

    #----------------------------------------------------------------------------------------------------------#

    motion_node.get_logger().info("Sequence completed successfully ✅")

    motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                                       
# move_portafilter_arc_movJ(angle_deg, d_rel_z=278.5, velocity=100, acceleration=100)
# enforce_rxry()
# move_portafilter_arc_tool(arc_size_deg, axis="z", tcp_table="{0,0,282.5,0,0,0}")
# enforce_rxry_moveit(d_rel_z=0.2825)
# get_machine_position(target_tf, required_samples=6, *, acq_timeout=10.0, debug=False)
# release_tension(settling_time=2.0)
# sync(timeout=20.0)
# set_gripper_position(speed=255, position=255, force=255, wait_finish=True)
# move_to(target_tf, distance, speed=100, acceleration=100, offset_x_mm=0.0, offset_y_mm=0.0, offset_z_mm=0.0, offset_rx_deg=0.0, offset_ry_deg=0.0, offset_rz_deg=0.0)
# moveEE_movJ(offset_x_mm, offset_y_mm, offset_z_mm, offset_rx_deg, offset_ry_deg, offset_rz_deg, velocity=100, acceleration=100)
# moveEE(offset_x_mm, offset_y_mm, offset_z_mm, offset_rx_deg, offset_ry_deg, offset_rz_deg)
# moveJ_deg(offset1, offset2, offset3, offset4, offset5, offset6)
# gotoEE(abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg, speed=100, acceleration=100)
# gotoJ_deg(angle1, angle2, angle3, angle4, angle5, angle6, velocity=100, acceleration=100)
# gotoEE_movJ(abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg, speed=100, acceleration=100)
# approach_tool(target_tf, speed=100, acceleration=100, *, offset_x_mm=0.0, offset_y_mm=0.0, offset_z_mm=-127.5)
# approach_machine(machine_name, point_name, *, speed=100, acceleration=100)
# mount_machine(machine_name, point_name, *, speed=100, acceleration=100)
# grab_tool(target_tf, speed=100, acceleration=100, offset_x_mm=-6.0, offset_y_mm=-5.0, offset_z_mm=-135)
# set_DO(index, status)
# toggle_drag_mode(max_attempts=5, retry_pause=0.25)
# move_arc(offset1, offset2, param_value=None)
# set_speed_factor(ratio)
# move_circle(count, offset1, offset2, param_value=None)
# use_tool(index, timeout=5.0)
# set_tool(index, table, timeout=5.0)