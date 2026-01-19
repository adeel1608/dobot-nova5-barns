#!/usr/bin/env python3
import os
import time
import yaml
import numpy as np
import math
import tf_transformations as tf_trans

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory

# ──────────────────────────────────────────────────────────────────────────────
# Explicit paths for saving the tool offsets in both src and install locations
SRC_TOOL_OFFSET_FILEPATH = os.path.expanduser(
    "~/barns_ws/src/pickn_place/share/tool_offset_points.yaml"
)
INSTALL_TOOL_OFFSET_FILEPATH = os.path.join(
    get_package_share_directory("pickn_place"),
    "tool_offset_points.yaml"
)
# ──────────────────────────────────────────────────────────────────────────────

# Offsets for approach_pose (if all zero, no change)
OFFSET_X = 0.0
OFFSET_Y = -0.04
OFFSET_Z = 0.025
OFFSET_ROLL_DEG = 0.0
OFFSET_PITCH_DEG = 0.0
OFFSET_YAW_DEG = -10.0

class ToolMountTeach(Node):
    def __init__(self):
        super().__init__('tool_mount_teach')
        self.get_logger().info("Mount the tool on the gripper and ensure the tool TF is available.")
        # Setup for TF lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Dynamic broadcaster for TFs that update with the tool
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # List for transforms to be dynamically published
        self.dynamic_transforms = []
        self.broadcast_timer = None

    def wait_for_frames(self, wait_time=3.0):
        """Spin for a given time to allow the TF buffer to populate."""
        self.get_logger().info(f"Waiting for TF frames for {wait_time} seconds...")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def verify_target_frames(self, required_frames):
        """Check if the required frames exist in the TF tree and return a list of missing frames."""
        tf_tree_str = self.tf_buffer.all_frames_as_string()
        missing = [frame for frame in required_frames if frame not in tf_tree_str]
        return missing

    def get_transform(self, target_frame, source_frame, timeout_sec=3.0):
        """
        Look up the transform from source_frame to target_frame (i.e., ^(target_frame)T_(source_frame)).
        Returns the transform if found; otherwise, returns None.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                Duration(seconds=timeout_sec)
            )
            return transform
        except Exception as e:
            self.get_logger().error(
                f"Transform from '{source_frame}' to '{target_frame}' not found within {timeout_sec} sec: {e}"
            )
            return None

    def average_quaternions(self, quaternions):
        """
        Average a list of quaternions (each as [w, x, y, z]) using an eigenvalue method.
        Returns the averaged quaternion as a list.
        """
        M = np.zeros((4, 4))
        for q in quaternions:
            q = np.array(q, dtype=np.float64).reshape(4, 1)
            M += q @ q.T
        M /= len(quaternions)
        eigenvalues, eigenvectors = np.linalg.eig(M)
        max_index = eigenvalues.argmax()
        avg = eigenvectors[:, max_index]
        avg = avg / np.linalg.norm(avg)
        return avg.tolist()

    def filter_translation_outliers(self, translations, k=1.5):
        """
        Filter out translation samples that are outliers using an IQR approach.
        Returns (filtered_translations, valid_indices).
        """
        translations_np = np.array(translations)
        median_translation = np.median(translations_np, axis=0)
        distances = np.linalg.norm(translations_np - median_translation, axis=1)
        Q1, Q3 = np.percentile(distances, [25, 75])
        IQR = Q3 - Q1
        lb, ub = max(0, Q1 - k * IQR), Q3 + k * IQR
        valid_indices = [i for i, d in enumerate(distances) if lb <= d <= ub]
        filtered_translations = [translations[i] for i in valid_indices]
        return filtered_translations, valid_indices

    def average_transform(self, target_frame, source_frame, sample_count=100, sample_interval=0.1):
        """
        Collect and average multiple samples of the transform.
        Returns (avg_translation, avg_quaternion) or (None, None) on failure.
        """
        translations, quaternions = [], []
        for i in range(sample_count):
            transform = self.get_transform(target_frame, source_frame, timeout_sec=sample_interval * 1.5)
            if transform:
                t = transform.transform.translation
                r = transform.transform.rotation
                translations.append([t.x, t.y, t.z])
                quaternions.append([r.w, r.x, r.y, r.z])
            else:
                self.get_logger().error(f"Failed to get sample {i+1} for {source_frame}->{target_frame}")
            time.sleep(sample_interval)

        if not translations:
            self.get_logger().error("No samples collected.")
            return None, None

        filt_t, idx = self.filter_translation_outliers(translations, k=1.5)
        filt_q = [quaternions[i] for i in idx]
        avg_t = [sum(x) / len(x) for x in zip(*filt_t)]
        avg_q = self.average_quaternions(filt_q)
        return avg_t, avg_q

    def setup_dynamic_broadcast(self, tool_name, approach_offset, grab_offset):
        transforms = []

        t_approach = TransformStamped()
        t_approach.header.frame_id = tool_name
        t_approach.child_frame_id = "approach_pose"
        t_approach.transform.translation.x = approach_offset["translation"]["x"]
        t_approach.transform.translation.y = approach_offset["translation"]["y"]
        t_approach.transform.translation.z = approach_offset["translation"]["z"]
        t_approach.transform.rotation.w = approach_offset["rotation"]["w"]
        t_approach.transform.rotation.x = approach_offset["rotation"]["x"]
        t_approach.transform.rotation.y = approach_offset["rotation"]["y"]
        t_approach.transform.rotation.z = approach_offset["rotation"]["z"]
        transforms.append(t_approach)

        t_grab = TransformStamped()
        t_grab.header.frame_id = tool_name
        t_grab.child_frame_id = "grab_pose"
        t_grab.transform.translation.x = grab_offset["translation"]["x"]
        t_grab.transform.translation.y = grab_offset["translation"]["y"]
        t_grab.transform.translation.z = grab_offset["translation"]["z"]
        t_grab.transform.rotation.w = grab_offset["rotation"]["w"]
        t_grab.transform.rotation.x = grab_offset["rotation"]["x"]
        t_grab.transform.rotation.y = grab_offset["rotation"]["y"]
        t_grab.transform.rotation.z = grab_offset["rotation"]["z"]
        transforms.append(t_grab)

        self.dynamic_transforms = transforms
        self.broadcast_timer = self.create_timer(0.1, self.publish_dynamic_transforms)
        self.get_logger().info("Dynamic TF broadcast timer started.")

    def publish_dynamic_transforms(self):
        current_time = self.get_clock().now().to_msg()
        for transform in self.dynamic_transforms:
            transform.header.stamp = current_time
        self.tf_broadcaster.sendTransform(self.dynamic_transforms)

    def run(self):
        # Warm up TF buffer
        self.wait_for_frames(wait_time=3.0)

        # Prompt for tool TF and verify
        tool_name = input("Enter tool tf name: ").strip()
        if not tool_name:
            self.get_logger().info("Please enter a valid tool tf name.")
            return

        tool_tf = self.get_transform(tool_name, "calibrated_camera_link", timeout_sec=1.0)
        if tool_tf is None:
            self.get_logger().error(f"Tool TF '{tool_name}' not found in the TF tree.")
            return

        missing = self.verify_target_frames(["Link6", "tool_link"])
        if missing:
            self.get_logger().error("Missing TF frames: " + ", ".join(missing))
            return

        # Sample approach and grab transforms
        self.get_logger().info(f"Sampling approach_pose (Link6 -> {tool_name})...")
        approach_translation, approach_quaternion = self.average_transform(tool_name, "Link6")
        self.get_logger().info(f"Sampling grab_pose (tool_link -> {tool_name})...")
        grab_translation, grab_quaternion     = self.average_transform(tool_name, "tool_link")

        if None in (approach_translation, approach_quaternion, grab_translation, grab_quaternion):
            self.get_logger().error("TF averaging failed for one or more transforms. Exiting.")
            return

        # Apply static offsets to approach_pose
        new_approach_translation = [
            approach_translation[0] + OFFSET_X,
            approach_translation[1] + OFFSET_Y,
            approach_translation[2] + OFFSET_Z,
        ]
        off_roll = math.radians(OFFSET_ROLL_DEG)
        off_pitch = math.radians(OFFSET_PITCH_DEG)
        off_yaw = math.radians(OFFSET_YAW_DEG)
        off_quat = tf_trans.quaternion_from_euler(off_roll, off_pitch, off_yaw)
        new_approach_quaternion = tf_trans.quaternion_multiply(approach_quaternion, off_quat)
        new_approach_quaternion = [float(v) for v in new_approach_quaternion]

        approach_offset = {
            "translation": dict(x=new_approach_translation[0],
                                y=new_approach_translation[1],
                                z=new_approach_translation[2]),
            "rotation":    dict(w=new_approach_quaternion[0],
                                x=new_approach_quaternion[1],
                                y=new_approach_quaternion[2],
                                z=new_approach_quaternion[3]),
        }
        grab_offset = {
            "translation": dict(x=grab_translation[0],
                                y=grab_translation[1],
                                z=grab_translation[2]),
            "rotation":    dict(w=grab_quaternion[0],
                                x=grab_quaternion[1],
                                y=grab_quaternion[2],
                                z=grab_quaternion[3]),
        }

        # ── Persist to both source & install locations ────────────────────────────
        def merge_write(path):
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            except FileNotFoundError:
                data = {}
            if not isinstance(data, dict):
                data = {}
            data[tool_name] = {"approach_pose": approach_offset, "grab_pose": grab_offset}
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            self.get_logger().info(f"Tool offset points saved → {path}")

        merge_write(SRC_TOOL_OFFSET_FILEPATH)
        merge_write(INSTALL_TOOL_OFFSET_FILEPATH)
        # ──────────────────────────────────────────────────────────────────────────

        # Start dynamic TF broadcast
        self.setup_dynamic_broadcast(tool_name, approach_offset, grab_offset)
        self.get_logger().info("Dynamic TF broadcast complete; spin to verify.")
        rclpy.spin(self)

    def shutdown(self):
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ToolMountTeach()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down, ignore

if __name__ == '__main__':
    main()
