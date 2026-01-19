#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

def ransac_plane(points, threshold=0.015, iterations=100):
    """
    A simple RANSAC algorithm to segment the dominant plane in a point cloud.
    :param points: numpy array of shape (N,3)
    :param threshold: distance threshold for a point to be considered an inlier
    :param iterations: number of iterations for RANSAC
    :return: indices of inlier points and plane parameters (A, B, C, D)
    """
    best_inliers_count = 0
    best_plane = None
    best_inlier_idx = np.array([], dtype=int)
    n_points = points.shape[0]
    if n_points < 3:
        return best_inlier_idx, best_plane

    for _ in range(iterations):
        # Randomly sample 3 unique points
        sample_idx = np.random.choice(n_points, 3, replace=False)
        sample = points[sample_idx, :]
        p1, p2, p3 = sample

        # Compute plane normal using cross product
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_normal = np.linalg.norm(normal)
        if norm_normal == 0:
            continue
        normal = normal / norm_normal
        A, B, C = normal
        D = -np.dot(normal, p1)

        # Compute distances from all points to the plane
        distances = np.abs(np.dot(points, normal) + D)
        inlier_idx = np.where(distances < threshold)[0]
        inliers_count = inlier_idx.size

        # Keep track of the best plane found so far
        if inliers_count > best_inliers_count:
            best_inliers_count = inliers_count
            best_plane = (A, B, C, D)
            best_inlier_idx = inlier_idx

    return best_inlier_idx, best_plane

class PointcloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        # Subscriber for raw point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.listener_callback,
            10)
        # Publisher for filtered/downsampled point cloud (optional)
        self.publisher_filtered_ = self.create_publisher(
            PointCloud2,
            '/camera/depth/points_filtered',
            10)
        # Publishers for plane and object points
        self.publisher_plane_ = self.create_publisher(
            PointCloud2,
            '/camera/depth/plane_points',
            10)
        self.publisher_object_ = self.create_publisher(
            PointCloud2,
            '/camera/depth/object_points',
            10)

        # Parameters (adjust as needed)
        self.target_frame = 'camera_depth_optical_frame'
        self.min_distance = 0.4  # meters
        self.max_distance = 1.5  # meters
        self.downsample_rate = 5  # take every nth point

        # RANSAC parameters
        self.ransac_threshold = 0.015  # distance threshold (meters)
        self.ransac_iterations = 200  # number of iterations

    def listener_callback(self, msg):
        # Update frame_id for visualization
        msg.header.frame_id = self.target_frame

        # Determine the number of points and convert to a NumPy array.
        total_points = msg.width * msg.height
        # Each point has point_step bytes; here we assume first 3 float32 values are x, y, z.
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(total_points, int(msg.point_step / 4))
        xyz = points[:, :3]

        # Filter points based on the distance range.
        distances = np.linalg.norm(xyz, axis=1)
        mask = (distances >= self.min_distance) & (distances <= self.max_distance)
        filtered_xyz = xyz[mask]

        # Downsample the filtered points.
        downsampled_xyz = filtered_xyz[::self.downsample_rate]

        if downsampled_xyz.shape[0] < 3:
            self.get_logger().warn("Not enough points after downsampling for segmentation.")
            return

        # Optional: Publish the filtered point cloud (all points after downsampling)
        filtered_points_list = [tuple(p) for p in downsampled_xyz]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        filtered_msg = pc2.create_cloud(msg.header, fields, filtered_points_list)
        self.publisher_filtered_.publish(filtered_msg)

        # Perform plane segmentation using RANSAC.
        inlier_idx, plane_params = ransac_plane(downsampled_xyz, self.ransac_threshold, self.ransac_iterations)
        if plane_params is None or inlier_idx.size == 0:
            self.get_logger().warn("Plane segmentation did not find a valid plane.")
            return

        # Separate plane points and object points.
        plane_points = downsampled_xyz[inlier_idx]
        # Outlier indices: points that are not part of the plane.
        all_indices = np.arange(downsampled_xyz.shape[0])
        object_idx = np.setdiff1d(all_indices, inlier_idx)
        object_points = downsampled_xyz[object_idx]

        plane_points_list = [tuple(p) for p in plane_points]
        object_points_list = [tuple(p) for p in object_points]

        # Create PointCloud2 messages for plane and object points.
        plane_msg = pc2.create_cloud(msg.header, fields, plane_points_list)
        object_msg = pc2.create_cloud(msg.header, fields, object_points_list)

        # Publish the segmented point clouds.
        self.publisher_plane_.publish(plane_msg)
        self.publisher_object_.publish(object_msg)

        self.get_logger().info(f"Published plane cloud with {len(plane_points_list)} points and object cloud with {len(object_points_list)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudProcessor()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
