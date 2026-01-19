#!/usr/bin/env python3

import os
import yaml
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory

def to_python(obj):
    """Recursively convert numpy types in obj to native Python types."""
    if isinstance(obj, dict):
        return {to_python(k): to_python(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [to_python(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj

# ──────────────────────────────────────────────────────────────────────────────
# Define explicit file paths for merging/saving offsets
SRC_OFFSET_FILEPATH = os.path.expanduser(
    "~/barns_ws/src/pickn_place/share/machine_offset_points.yaml"
)
INSTALL_OFFSET_FILEPATH = os.path.join(
    get_package_share_directory("pickn_place"),
    "machine_offset_points.yaml"
)
# ──────────────────────────────────────────────────────────────────────────────

class MachineMountTeach(Node):
    MARKER_SAMPLES  = 100
    MARKER_INTERVAL = 0.1
    OUTLIER_K       = 1.5

    def __init__(self):
        super().__init__("machine_mount_teach")

        # TF listener & broadcasters
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcaster        = tf2_ros.TransformBroadcaster(self)

        # Keep TF callbacks alive
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()

        self.get_logger().info("MachineMountTeach: initialized and waiting for TF frames...")

    def spin_for(self, seconds: float):
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def lookup(self, parent: str, child: str, timeout: float = 1.0):
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, Time(), Duration(seconds=timeout)
            )
        except Exception:
            return None

    def avg_quat(self, quaternions):
        M = np.zeros((4, 4))
        for q in quaternions:
            v = np.array(q).reshape(4,1)
            M += v @ v.T
        vals, vecs = np.linalg.eig(M / len(quaternions))
        q_avg = vecs[:, vals.argmax()]
        return (q_avg / np.linalg.norm(q_avg)).tolist()

    def average_marker(self, marker_frame: str):
        translations, quats = [], []
        while len(translations) < self.MARKER_SAMPLES:
            tfm = self.lookup("base_link", marker_frame, timeout=self.MARKER_INTERVAL * 1.5)
            if tfm:
                t = tfm.transform.translation
                r = tfm.transform.rotation
                translations.append([t.x, t.y, t.z])
                # store as [w,x,y,z] for consistency in avg_quat
                quats.append([r.w, r.x, r.y, r.z])
            self.spin_for(self.MARKER_INTERVAL)

        arr    = np.array(translations)
        median = np.median(arr, axis=0)
        dists  = np.linalg.norm(arr - median, axis=1)
        q1, q3 = np.percentile(dists, [25, 75])
        iqr    = q3 - q1
        valid  = [i for i, d in enumerate(dists)
                  if (q1 - self.OUTLIER_K * iqr) <= d <= (q3 + self.OUTLIER_K * iqr)]

        avg_t = np.mean(arr[valid], axis=0).tolist()
        avg_q = self.avg_quat([quats[i] for i in valid])
        return avg_t, avg_q

    def capture_tf(self, parent: str, child: str, timeout: float = 1.0):
        grabber = Node('tf_grabber')
        buf     = tf2_ros.Buffer()
        tf_l    = tf2_ros.TransformListener(buf, grabber, spin_thread=False)
        exe     = SingleThreadedExecutor()
        exe.add_node(grabber)

        deadline = grabber.get_clock().now() + Duration(seconds=timeout)
        result = None
        while grabber.get_clock().now() < deadline and result is None:
            exe.spin_once(timeout_sec=0.05)
            try:
                result = buf.lookup_transform(parent, child, Time(), Duration(seconds=0.0))
            except Exception:
                pass

        try:
            exe.remove_node(grabber)
        except Exception:
            pass
        grabber.destroy_node()

        if result is None:
            raise RuntimeError(f"TF {parent}->{child} not received in {timeout}s")
        return result

    def sample_offset(self, tm, rm, tcp_frame: str, debug_name: str):
        # 1) capture dynamic TCP pose in base_link
        tfm = self.capture_tf("base_link", tcp_frame, timeout=2.0)
        t, r = tfm.transform.translation, tfm.transform.rotation

        # 2) compute offset in machine frame using correct quaternion ordering
        M_btcp = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
        M_btcp[0:3, 3] = [t.x, t.y, t.z]
        M_bm = tft.quaternion_matrix([rm[1], rm[2], rm[3], rm[0]])
        M_bm[0:3, 3] = tm
        M_off = np.linalg.inv(M_bm).dot(M_btcp)

        t_off = M_off[0:3, 3].tolist()
        x, y, z, w = tft.quaternion_from_matrix(M_off)
        q_off = [w, x, y, z]

        # 3) broadcast debug frame relative to machine_saved
        dbg = TransformStamped()
        dbg.header.stamp    = self.get_clock().now().to_msg()
        dbg.header.frame_id = self.machine_saved_frame_id
        dbg.child_frame_id  = debug_name
        dbg.transform.translation.x = t_off[0]
        dbg.transform.translation.y = t_off[1]
        dbg.transform.translation.z = t_off[2]
        dbg.transform.rotation.w    = q_off[0]
        dbg.transform.rotation.x    = q_off[1]
        dbg.transform.rotation.y    = q_off[2]
        dbg.transform.rotation.z    = q_off[3]
        self.static_broadcaster.sendTransform(dbg)

        self.get_logger().info(f"{debug_name}: offset t={t_off}, q={q_off}")
        return t_off, q_off

    def prompt_for_frame(self, parent: str, prompt: str, retries: int = 3):
        while True:
            frame = input(prompt).strip()
            for attempt in range(1, retries + 1):
                if self.lookup(parent, frame):
                    return frame
                self.spin_for(self.MARKER_INTERVAL)
                self.get_logger().warn(
                    f"Lookup {parent}->{frame} failed (attempt {attempt}/{retries})"
                )
            self.get_logger().error(
                f"No TF {parent}->{frame} after {retries} tries—please enter again."
            )

    def prompt_and_sample(self, tm, rm, tcp_prompt: str, debug_name: str, retries: int = 3):
        """
        Returns a triple: (translation, rotation, tcp_frame_name).
        """
        while True:
            tcp_frame = input(tcp_prompt).strip()
            for attempt in range(1, retries + 1):
                try:
                    t_off, q_off = self.sample_offset(tm, rm, tcp_frame, debug_name)
                    return t_off, q_off, tcp_frame
                except RuntimeError as e:
                    self.get_logger().warn(
                        f"{debug_name}: {e} (attempt {attempt}/{retries})"
                    )
            self.get_logger().error(
                f"Could not sample {tcp_frame} after {retries} tries—please enter again."
            )

    def run(self):
        # warm up TF
        self.spin_for(3.0)

        # teach machine frame with retry
        machine = self.prompt_for_frame("base_link", "Enter machine TF name: ")
        self.get_logger().info(f"Machine entered: '{machine}'")

        # average marker once
        t_m, q_m = self.average_marker(machine)
        self.get_logger().info(f"Machine averaged: t={t_m}, q={q_m}")

        # store in memory and publish static
        machine_saved = f"{machine}_saved"
        self.machine_saved_frame_id    = machine_saved
        self.machine_saved_translation = t_m
        self.machine_saved_rotation    = q_m

        tfm = TransformStamped()
        tfm.header.stamp    = self.get_clock().now().to_msg()
        tfm.header.frame_id = "base_link"
        tfm.child_frame_id  = machine_saved
        tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z = t_m
        tfm.transform.rotation.w, tfm.transform.rotation.x = q_m[0], q_m[1]
        tfm.transform.rotation.y, tfm.transform.rotation.z = q_m[2], q_m[3]
        self.static_broadcaster.sendTransform(tfm)
        self.spin_for(0.2)

        # teach points with retry
        offsets = {machine: {}}
        while True:
            pt = input("Enter point name (blank=finish): ").strip()
            if not pt:
                break

            # now capture and keep the TCP frame name as well
            a_t, a_q, a_tcp = self.prompt_and_sample(
                self.machine_saved_translation,
                self.machine_saved_rotation,
                "  TCP frame for approach_pose: ",
                f"{pt}_approach_test"
            )
            m_t, m_q, m_tcp = self.prompt_and_sample(
                self.machine_saved_translation,
                self.machine_saved_rotation,
                "  TCP frame for mount_pose:    ",
                f"{pt}_mount_test"
            )

            offsets[machine][pt] = {
                "approach_pose": {
                    "TCP":         a_tcp,
                    "translation": dict(zip(("x","y","z"), a_t)),
                    "rotation":    dict(zip(("w","x","y","z"), a_q)),
                },
                "mount_pose": {
                    "TCP":         m_tcp,
                    "translation": dict(zip(("x","y","z"), m_t)),
                    "rotation":    dict(zip(("w","x","y","z"), m_q)),
                },
            }

        if not offsets[machine]:
            self.get_logger().warn("No points recorded.")
            return

        # persist to YAML with non-destructive merge
        def merge_write(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
            except FileNotFoundError:
                data = {}

            if not isinstance(data, dict):
                data = {}

            machine_name = machine
            existing = data.get(machine_name, {})
            if not isinstance(existing, dict):
                existing = {}

            new_points = offsets.get(machine_name, {})
            for pt_name, pt_data in new_points.items():
                existing[pt_name] = pt_data

            data[machine_name] = existing

            os.makedirs(os.path.dirname(path), exist_ok=True)
            data = to_python(data)
            with open(path, "w") as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(f"Offsets saved → {path}")

        # Write to both the source and install locations
        merge_write(SRC_OFFSET_FILEPATH)
        merge_write(INSTALL_OFFSET_FILEPATH)

        self.get_logger().info("Teaching complete.")

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = MachineMountTeach()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
