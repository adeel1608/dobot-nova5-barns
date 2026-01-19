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
from tf2_ros import Buffer, TransformListener
import tf_transformations

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from control_msgs.action import FollowJointTrajectory

import threading
from rclpy.executors import SingleThreadedExecutor

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
            time.sleep(retry_pause)

        log.error("move_to: failed after maximum retries")
        return False

    def enforce_rxry(self) -> bool:
        """
        Lock Link-6 orientation to Rx = 90°, Ry = 0° (keep Rz), while holding the
        portafilter_link origin fixed (±0.5 mm).  Uses GetPose → MovJ only.
        Returns True on success.
        """

        import numpy as np, math, time, rclpy
        from tf_transformations import euler_matrix, quaternion_from_matrix

        log           = self.get_logger()
        retry_pause   = 0.25
        max_attempts  = 6
        call_timeout  = 3.0
        d_rel         = np.array([0.0, 0.0, 0.276])       # portafilter offset (m)

        # ── 1) GET current Link-6 pose ------------------------------------------------
        gp_req = GetPose.Request(user=0, tool=0)

        for attempt in range(1, max_attempts + 1):
            fut = self.get_pose_cli.call_async(gp_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=call_timeout)

            if fut.done() and fut.result() is not None and hasattr(fut.result(), "pose"):
                try:
                    tx, ty, tz, rx, ry, rz = map(float, fut.result().pose.strip("{}").split(",")[:6])
                    break                           # ✓ got pose
                except Exception as exc:
                    log.error(f"enforce_rxry: bad pose string – {exc}")
                    return False

            log.warn(f"enforce_rxry: GetPose failed (attempt {attempt}/{max_attempts})")
            time.sleep(retry_pause)
        else:
            log.error("enforce_rxry: unable to fetch Link-6 pose")
            return False

        # ── 2) Compute goal position/orientation -------------------------------------
        p_link6   = np.array([tx, ty, tz]) * 1e-3        # mm → m
        R_curr    = euler_matrix(*np.radians([rx, ry, rz]))[:3, :3]
        p_pf      = p_link6 + R_curr @ d_rel             # world pos of portafilter_link

        rx_goal, ry_goal, rz_goal = 90.0, 0.0, rz        # lock Rx/Ry, keep Rz
        R_goal    = euler_matrix(*np.radians([rx_goal, ry_goal, rz_goal]))[:3, :3]
        p6_goal   = p_pf - R_goal @ d_rel                # back-solve Link-6 world pos

        # ── 3) Build & send MovJ ------------------------------------------------------
        movj_req = MovJ.Request(
            x  = float(p6_goal[0] * 1000.0),
            y  = float(p6_goal[1] * 1000.0),
            z  = float(p6_goal[2] * 1000.0),
            rx = rx_goal,
            ry = ry_goal,
            rz = rz_goal,
            param_value = ["SpeedJ=100,AccJ=100"],
        )

        for attempt in range(1, max_attempts + 1):
            fut = self.movj_cli.call_async(movj_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=call_timeout)

            if fut.done() and fut.result() is not None:
                if getattr(fut.result(), "res", 1) == 0:
                    log.info("enforce_rxry: MovJ accepted ✓")
                    break
                log.warn(f"enforce_rxry: driver res={fut.result().res} "
                        f"(attempt {attempt}/{max_attempts})")
            else:
                log.error("enforce_rxry: MovJ call timed-out")
                return False

            time.sleep(retry_pause)
        else:
            log.error("enforce_rxry: MovJ failed after retries")
            return False

        log.info("enforce_rxry: completed successfully")
        return True
    
    def move_portafilter_arc(self, angle_deg: float) -> bool:
        """
        Move the portafilter_link in an arc by angle_deg about its local Y axis,
        using pymoveit2. Retrieves the current Link6 pose via GetPose, computes
        portafilter_link pose by applying the fixed offset d_rel, then applies
        the same rotation and Cartesian move. Returns True on success, False on failure.
        """
        from pymoveit2 import MoveIt2, MoveIt2State
        import tf_transformations

        # Fixed offset from Link6 origin → portafilter_link origin (m)
        d_rel = np.array([0.0, 0.0, 0.276])

        # Cartesian parameters
        self.cartesian_fraction_threshold = 0.5
        self.cartesian_avoid_collisions = True
        self.reference_frame = "base_link"
        self.planner_id = "OMPL"
        self.cartesian = True

        retry_pause = 0.25
        max_attempts = 10

        gp_req = GetPose.Request()
        gp_req.user = 0
        gp_req.tool = 0

        while not self.wait_for_servo_ready(timeout=10.0):
            self.get_logger().warn("move_portafilter_arc: Motion not complete—retrying verification...")
            time.sleep(0.2)
            
        resp = None
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"move_portafilter_arc(): GetPose (Link6) attempt {attempt}/{max_attempts}")
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
                "move_portafilter_arc(): GetPose call failed or timed out—retrying in 0.25 s"
            )
            time.sleep(retry_pause)

        if resp is None or not hasattr(resp, "pose"):
            self.get_logger().error("move_portafilter_arc(): Failed to retrieve Link6 pose after 10 attempts.")
            return False

        # parse "{tx,ty,tz,rx,ry,rz,…}"  (tx, ty, tz in mm; rx, ry, rz in degrees)
        parts = resp.pose.strip("{}").split(",")
        if len(parts) < 6:
            self.get_logger().error("move_portafilter_arc(): Invalid pose format from GetPose.")
            return False
        try:
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = [float(p) for p in parts[:6]]
        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc(): Error parsing GetPose: {e}")
            return False

        # convert Link6 translation to metres
        p_link6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
        # build Link6 rotation matrix and quaternion from Euler (XYZ in degrees)
        rads = np.radians([rx_curr, ry_curr, rz_curr])
        M6 = tf_transformations.euler_matrix(*rads)
        R6_curr = M6[:3, :3]
        q_link6 = tf_transformations.quaternion_from_matrix(M6)

        # compute current portafilter_link pose: translation + same orientation
        p_pf = p_link6 + R6_curr.dot(d_rel)
        current_quat = q_link6

        # ── 2) Compute target orientation by rotating about portafilter local Y axis
        R_pf_curr = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]
        rotation_axis = R_pf_curr[:, 1]
        theta = math.radians(angle_deg)
        relative_quat = tf_transformations.quaternion_about_axis(theta, rotation_axis)
        new_quat = tf_transformations.quaternion_multiply(relative_quat, current_quat)
        new_pos = p_pf.tolist()

        self.get_logger().info(
            f"move_portafilter_arc: Rotating portafilter_link by {angle_deg}° about local Y axis."
        )

        # ── 3) Initialize MoveIt2 for portafilter_link
        pf_moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name=self.reference_frame,
            end_effector_name="portafilter_link",
            group_name="portafilter_center",
        )

        # ── 4) Execute Cartesian arc motion
        pf_moveit2.move_to_pose(
            position=new_pos,
            quat_xyzw=new_quat,
            cartesian=self.cartesian,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
        pf_moveit2.wait_until_executed()

        # ── 5) Verify via servo readiness
        while not self.wait_for_servo_ready(timeout=20.0):
            self.get_logger().warn("move_portafilter_arc: Motion not complete—retrying verification...")
            time.sleep(0.2)

        self.get_logger().info("move_portafilter_arc: Completed successfully.")
        return True

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
            time.sleep(retry_pause)

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
                time.sleep(retry_pause)
                continue

            movl_fut = self.movl_cli.call_async(movl_req)
            start_mv = self.get_clock().now().nanoseconds * 1e-9
            while not movl_fut.done() and (self.get_clock().now().nanoseconds * 1e-9 - start_mv) < 0.5:
                rclpy.spin_once(self, timeout_sec=0.01)

            if movl_fut.done() and movl_fut.result() is not None:
                movl_resp = movl_fut.result()
                break

            self.get_logger().warn("move_portafilter_arc(): MovL call failed or timed out, retrying...")
            time.sleep(retry_pause)

        if movl_resp is None:
            self.get_logger().error("move_portafilter_arc(): Failed to call MovL after 10 attempts.")
            return False
        time.sleep(0.5)

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
            time.sleep(retry_pause)
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
            time.sleep(retry_pause)
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
            offset_x_mm: float = 0.0,
            offset_y_mm: float = 0.0,
            offset_z_mm: float = -128,
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
        count: int,
        offset1: tuple[float, float, float, float, float, float],
        offset2: tuple[float, float, float, float, float, float],
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
            time.sleep(retry_pause)

        self.get_logger().error("move_circle: failed after maximum retries")
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
    # # 0) acquire steam‑wand marker position ---------------------------------
    run_skill("set_speed_factor", 100)
    run_skill("set_gripper_position", 255, 0, 255, False)
    run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960,  13.926845)  # steam‑wand home
    for _ in range(1):
        run_skill("sync")
        run_skill("move_to", "left_steam_wand", 0.27)
    run_skill("sync")
    run_skill("get_machine_position", "left_steam_wand", 6)
    run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960,  13.926845)

    # # 1) pick up the pitcher -------------------------------------------------
    run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)   # home
    run_skill("set_gripper_position", 255, 0, 255, False)
    run_skill("gotoJ_deg",  26.891018, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)
    run_skill("sync")
    run_skill("move_to", "milk_frother_2", 0.28)
    run_skill("set_gripper_position", 255, 180, 255, False)
    run_skill("sync")
    run_skill("approach_tool", "milk_frother_2", 100, 60)
    run_skill("sync")
    run_skill("grab_tool", "milk_frother_2", 50, 10)
    run_skill("sync")
    run_skill("set_gripper_position", 255, 255)
    run_skill("set_speed_factor", 40)
    run_skill("moveEE", 0.0, 0.0, 20.0, 0.0, 0.0, 0.0)            # up 
    run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455,-84.523247, -67.974960, 13.926845)

    # 2) steam‑wand mounting -------------------------------------------------
    run_skill("set_DO", 1, 1);     
    time.sleep(0.5);  
    run_skill("set_DO", 1, 0)
    run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960,  13.926845)   # steam‑wand home
    run_skill("approach_machine", "left_steam_wand", "deep_froth")
    run_skill("mount_machine",    "left_steam_wand", "deep_froth")
    run_skill("sync")
    run_skill("set_DO", 1, 1);  
    time.sleep(5); 
    run_skill("set_speed_factor", 20)
    run_skill("approach_machine", "left_steam_wand", "deep_froth"); 
    run_skill("set_DO", 1, 0)
    time.sleep(1); 
    run_skill("set_speed_factor", 60)
    run_skill("gotoJ_deg", -34.108982, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)

    # pour milk ------------------------------------------------
    run_skill("gotoEE", -232.533234, -323.002747, 254.201981, 92.561615, -11.636983, -6.082766)
    run_skill("sync")
    run_skill("set_speed_factor", 30)
    run_skill("move_circle", 5,
        ( -30.0,  0.0, 0.0, 0.0, 0.0, 0.0),     # point1  offset1
        ( -15.0,-15.0, 0.0, 0.0, 0.0, 0.0),)    # point2  offset2
    run_skill("sync")
    run_skill("set_speed_factor", 30)
    run_skill("move_circle", 3,
        ( -30.0,  0.0, 0.0, 0.0, 90.0, 0.0),    # point1  offset1
        ( -15.0,-15.0, 0.0, 0.0, 90.0, 0.0),)   # point2  offset2
    run_skill("sync")
    run_skill("set_speed_factor", 6)
    run_skill("moveEE", 50.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    run_skill("sync")
    run_skill("set_speed_factor", 15)
    run_skill("move_circle", 3,
        ( -30.0,  0.0, 0.0, 0.0, 25.0, 0.0),    # point1 offset1
        ( -15.0,-15.0, 0.0, 0.0, 25.0, 0.0),)   # point2 offset2
    run_skill("sync")
    run_skill("set_speed_factor", 100)
    run_skill("gotoEE", -232.533234, -323.002747, 254.201981, 92.561615, -11.636983, -6.082766)

    # 5) carry to milk dispenser --------------------------------------------
    run_skill("moveJ_deg", 60.0, 0.0, 0.0, 0.0, 0.0, 0.0)        # ccw joint‑1
    run_skill("gotoJ_deg", 31.479187, -66.335266, -97.210228, -20.666107, -61.815777, 10.486634)  # pre‑drop
    run_skill("gotoJ_deg", 26.482792, -77.795357, -58.903595, -47.351318, -66.826134, 10.101309)  # drop
    run_skill("moveEE", 0.0, 0.0, -40.0, 0.0, 0.0, 0.0)          # down
    run_skill("sync")
    run_skill("set_gripper_position", 255, 180)
    run_skill("moveEE", -150.0, -15.0, 20, 0.0, 0.0, 0.0)      # back + up
    run_skill("gotoJ_deg",  25.891018, -12.468539, -116.353455, -84.523247, -67.974960, 13.926845)  # home

    motion_node.get_logger().info("Sequence completed successfully ✅")

    motion_node.destroy_node()
    rclpy.shutdown()


    # run_skill("move_arc", 5,                  # five-segment arc
    #     (-30.0,  0.0,   0.0, 0.0, 0.0, 0.0),  # point1 offset1
    #     (-15.0, -15.0,  0.0, 0.0, 0.0, 0.0),  # point2 offset2


if __name__ == '__main__':
    main()
                                       
