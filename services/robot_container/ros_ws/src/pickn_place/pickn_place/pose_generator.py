#!/usr/bin/env python3
# pose_tf_publisher_node.py
#
# • Broadcasts dynamic TF   calibrated_camera_link → <marker>
#   only when a new timestamp appears in the memory file.
# • Publishes a PoseStamped with the same header/pose on
#     /marker_poses/<marker_name>

import os
import subprocess
import yaml
from time import time

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory

# ──────────────────────────────────────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────────────────────────────────────
PACKAGE_NAME = "pickn_place"
DEFAULT_POSE_DATA_FILENAME = "pose_data_memory.yaml"
TIMER_INTERVAL = 0.10          # seconds – publish rate
ARUCO_PERCEPTION_COMMAND = ["ros2", "run", PACKAGE_NAME, "aruco_perception"]
# ──────────────────────────────────────────────────────────────────────────────


class PoseTFPublisherNode(Node):
    def __init__(self):
        super().__init__("pose_tf_publisher_node")

        # Dynamic TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Path to YAML “memory” file
        pkg_share = get_package_share_directory(PACKAGE_NAME)
        self.pose_data_file = os.path.join(pkg_share, DEFAULT_POSE_DATA_FILENAME)

        # On-demand PoseStamped publishers
        self.pose_publishers = {}

        # Track last‐published timestamp per marker
        self.last_published = {}

        # Timer that calls publish_tf_and_pose() at TIMER_INTERVAL
        self.create_timer(TIMER_INTERVAL, self.publish_tf_and_pose)

    def _log_tf(self, msg: TransformStamped):
        self.get_logger().info(
            f"[DYNAMIC TF] {msg.child_frame_id} ← {msg.header.frame_id}  "
            f"t=({msg.transform.translation.x:.3f}, "
            f"{msg.transform.translation.y:.3f}, "
            f"{msg.transform.translation.z:.3f})  "
            f"q=({msg.transform.rotation.x:.3f}, "
            f"{msg.transform.rotation.y:.3f}, "
            f"{msg.transform.rotation.z:.3f}, "
            f"{msg.transform.rotation.w:.3f})"
        )

    def _ensure_pose_pub(self, marker):
        topic = f"/marker_poses/{marker}"
        if marker not in self.pose_publishers:
            self.pose_publishers[marker] = self.create_publisher(
                PoseStamped, topic, 10
            )
        return self.pose_publishers[marker]

    def publish_tf_and_pose(self):
        # Load saved poses from YAML
        try:
            with open(self.pose_data_file, "r") as f:
                pose_data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Cannot load pose data file: {e}")
            return

        # Unwrap top‐level “poses:” if present
        if "poses" in pose_data and isinstance(pose_data["poses"], dict):
            pose_data = pose_data["poses"]

        for marker, data in pose_data.items():
            stored_time = float(data.get("Time", 0.0))
            last = self.last_published.get(marker, 0.0)

            # Only publish if the timestamp is strictly newer
            if stored_time <= last:
                continue

            # Build and stamp the dynamic TF
            dyn_tf = TransformStamped()
            dyn_tf.header.frame_id = "calibrated_camera_link"
            dyn_tf.child_frame_id = marker
            secs = int(stored_time)
            nsecs = int((stored_time - secs) * 1e9)
            dyn_tf.header.stamp.sec = secs
            dyn_tf.header.stamp.nanosec = nsecs

            tr = data.get("translation", {})
            dyn_tf.transform.translation.x = tr.get("x", 0.0)
            dyn_tf.transform.translation.y = tr.get("y", 0.0)
            dyn_tf.transform.translation.z = tr.get("z", 0.0)

            rot = data.get("rotation", {})
            dyn_tf.transform.rotation.x = rot.get("x", 0.0)
            dyn_tf.transform.rotation.y = rot.get("y", 0.0)
            dyn_tf.transform.rotation.z = rot.get("z", 0.0)
            dyn_tf.transform.rotation.w = rot.get("w", 1.0)

            # Broadcast TF and log it
            self.tf_broadcaster.sendTransform(dyn_tf)
            self._log_tf(dyn_tf)

            # Remember this timestamp so we don’t republish it
            self.last_published[marker] = stored_time

            # Publish the same data as a PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = dyn_tf.header
            pose_msg.pose.position.x = dyn_tf.transform.translation.x
            pose_msg.pose.position.y = dyn_tf.transform.translation.y
            pose_msg.pose.position.z = dyn_tf.transform.translation.z
            pose_msg.pose.orientation = dyn_tf.transform.rotation
            self._ensure_pose_pub(marker).publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    # Launch the ArUco perception subprocess
    aruco_proc = subprocess.Popen(ARUCO_PERCEPTION_COMMAND)
    print(f"Started '{' '.join(ARUCO_PERCEPTION_COMMAND)}'")

    node = PoseTFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

        # Terminate ArUco process if still running
        if aruco_proc.poll() is None:
            aruco_proc.terminate()
            print(f"Terminated '{' '.join(ARUCO_PERCEPTION_COMMAND)}'")


if __name__ == "__main__":
    main()
