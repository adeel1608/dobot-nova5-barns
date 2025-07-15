#!/usr/bin/env python3
import os
import time
import yaml
import numpy as np
import math
from threading import Thread

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import tf_transformations
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import TransformStamped
# For loading the YAML file from the package share folder.
from ament_index_python.packages import get_package_share_directory

# --- Helper Functions ---
def get_transform_list(tf_stamped):
    """Convert a TransformStamped message into a list: [tx, ty, tz, qx, qy, qz, qw]."""
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]

def compute_translation_rotation_diff(tf1, tf2):
    """Compute differences between two transforms (list format)."""
    t1 = np.array(tf1[:3])
    t2 = np.array(tf2[:3])
    trans_diff = np.linalg.norm(t2 - t1)
    r1 = Rot.from_quat(tf1[3:])
    r2 = Rot.from_quat(tf2[3:])
    rel_rot = r1.inv() * r2
    rot_deg = math.degrees(rel_rot.magnitude())
    return trans_diff, rot_deg

def compute_max_spread(samples):
    """Compute maximum translation and rotation differences among a list of transforms."""
    max_trans, max_rot = 0.0, 0.0
    n = len(samples)
    for i in range(n):
        for j in range(i+1, n):
            dt, dr = compute_translation_rotation_diff(samples[i], samples[j])
            max_trans = max(max_trans, dt)
            max_rot = max(max_rot, dr)
    return max_trans, max_rot

def average_quaternions(quaternions):
    """Average a list of quaternions using an eigenvalue method."""
    aligned = []
    for q in quaternions:
        if np.dot(q, quaternions[0]) < 0:
            aligned.append(-q)
        else:
            aligned.append(q)
    aligned = np.array(aligned)
    M = np.zeros((4, 4))
    for q in aligned:
        M += np.outer(q, q)
    M /= aligned.shape[0]
    eigenvalues, eigenvectors = np.linalg.eig(M)
    avg_q = eigenvectors[:, np.argmax(eigenvalues)]
    norm = np.linalg.norm(avg_q)
    if norm < 1e-8:
        return np.array([0., 0., 0., 1.])
    return avg_q / norm

def average_transforms(transforms):
    """Average a list of transforms given as [tx, ty, tz, qx, qy, qz, qw]."""
    translations = np.array([t[:3] for t in transforms])
    avg_translation = np.mean(translations, axis=0)
    quaternions = np.array([t[3:] for t in transforms])
    avg_quat = average_quaternions(quaternions)
    return list(avg_translation) + list(avg_quat)

def get_stable_transform(tf_buffer, reference_frame, target_frame, node,
                         samples=5, delay=0.1, threshold_trans=0.001, threshold_rot=0.5):
    """
    Samples multiple transforms between reference_frame and target_frame.
    Returns the averaged transform if the maximum spread is within thresholds;
    otherwise, returns None.
    """
    sample_list = []
    for i in range(samples):
        try:
            tf_stamped = tf_buffer.lookup_transform(reference_frame, target_frame,
                                                    rclpy.time.Time(), timeout=Duration(seconds=2.0))
        except Exception as e:
            node.get_logger().warn(f"Failed to get transform sample {i+1} for '{target_frame}': {e}")
            return None
        sample_list.append(get_transform_list(tf_stamped))
        time.sleep(delay)
    max_trans, max_rot = compute_max_spread(sample_list)
    if max_trans > threshold_trans or max_rot > threshold_rot:
        node.get_logger().warn(
            f"Transform '{target_frame}' unstable: max translation diff {max_trans:.6f} m, max rotation diff {max_rot:.6f} deg")
        return None
    return average_transforms(sample_list)

# --- Test Node ---
class TestOffsetNode(Node):
    def __init__(self):
        super().__init__("test_offset_node")
        # Reference frame is used only to verify that the target exists.
        self.reference_frame = "base_link"
        # The target TF is the tool frame as taught.
        self.target_tf = "double_portafilter"  # Modify this TF as needed.
        self.offset_file = os.path.join(get_package_share_directory("pickn_place"), "tool_offset_points.yaml")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        # Run the test after a short delay (allowing TF population).
        self.create_timer(3.0, self.compute_and_publish_poses)

    def compute_and_publish_poses(self):
        self.get_logger().info(f"Computing poses based on target TF '{self.target_tf}'...")
        # Use stable sampling to ensure the target TF exists.
        avg_target_tf = get_stable_transform(self.tf_buffer, self.reference_frame, self.target_tf, self)
        if avg_target_tf is None:
            self.get_logger().error(f"Could not obtain a stable transform for '{self.target_tf}'.")
            return

        # Load offset data.
        try:
            with open(self.offset_file, "r") as f:
                offsets_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load offset file: {e}")
            return

        if self.target_tf not in offsets_data:
            self.get_logger().error(f"No offset points found for '{self.target_tf}'.")
            return

        tf_offsets = offsets_data[self.target_tf]
        if "approach_pose" not in tf_offsets or "grab_pose" not in tf_offsets:
            self.get_logger().error(f"Offset file for '{self.target_tf}' is missing required entries.")
            return

        # Instead of applying the offset to the base frame, we now assume the taught offsets are
        # stored relative to the tool frame (target_tf). So, if you apply the approach offset,
        # you should get the original Link6 pose; if you apply the grab offset, you get the original tool_link pose.
        #
        # That means we publish the offsets as static transforms with parent = target_tf.
        # For clarity, we set the child frame IDs to "Link6" for the approach offset and "tool_link" for the grab offset.

        # --- Approach Offset (to recover Link6) ---
        approach_offset = tf_offsets["approach_pose"]
        approach_translation = [
            approach_offset["translation"]["x"],
            approach_offset["translation"]["y"],
            approach_offset["translation"]["z"]
        ]
        approach_rot = approach_offset["rotation"]
        # Teaching node saves quaternion as [w, x, y, z]; convert to [x, y, z, w]
        approach_quat = [approach_rot["x"], approach_rot["y"], approach_rot["z"], approach_rot["w"]]

        # --- Grab Offset (to recover tool_link) ---
        grab_offset = tf_offsets["grab_pose"]
        grab_translation = [
            grab_offset["translation"]["x"],
            grab_offset["translation"]["y"],
            grab_offset["translation"]["z"]
        ]
        grab_rot = grab_offset["rotation"]
        grab_quat = [grab_rot["x"], grab_rot["y"], grab_rot["z"], grab_rot["w"]]

        # Create TransformStamped messages with parent set to the target TF.
        approach_tf = TransformStamped()
        approach_tf.header.stamp = self.get_clock().now().to_msg()
        approach_tf.header.frame_id = self.target_tf
        # Child frame should be Link6, as taught.
        approach_tf.child_frame_id = "Link6"
        approach_tf.transform.translation.x = approach_translation[0]
        approach_tf.transform.translation.y = approach_translation[1]
        approach_tf.transform.translation.z = approach_translation[2]
        approach_tf.transform.rotation.x = approach_quat[0]
        approach_tf.transform.rotation.y = approach_quat[1]
        approach_tf.transform.rotation.z = approach_quat[2]
        approach_tf.transform.rotation.w = approach_quat[3]

        mount_tf = TransformStamped()
        mount_tf.header.stamp = self.get_clock().now().to_msg()
        mount_tf.header.frame_id = self.target_tf
        # Child frame should be tool_link, as taught.
        mount_tf.child_frame_id = "tool_link"
        mount_tf.transform.translation.x = grab_translation[0]
        mount_tf.transform.translation.y = grab_translation[1]
        mount_tf.transform.translation.z = grab_translation[2]
        mount_tf.transform.rotation.x = grab_quat[0]
        mount_tf.transform.rotation.y = grab_quat[1]
        mount_tf.transform.rotation.z = grab_quat[2]
        mount_tf.transform.rotation.w = grab_quat[3]

        self.static_broadcaster.sendTransform([approach_tf, mount_tf])
        self.get_logger().info(f"Published TFs: parent '{self.target_tf}' -> child 'Link6' (approach offset) and child 'tool_link' (grab offset).")

def main(args=None):
    rclpy.init(args=args)
    node = TestOffsetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
