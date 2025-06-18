#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dobot_msgs_v3.srv import EnableRobot, ServoJ, GetPose

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        mane = os.getenv("DOBOT_TYPE", "dobot")
        
        # Idle Mode: Subscribe to joint states for updating robot position.
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.joint_callback,
            10
        )
        self.idle_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'{mane}_group_controller/follow_joint_trajectory'
        )
        
        # Action Mode: Trajectory command subscription and action server.
        self.trajectory_sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'{mane}_group_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # Service Clients.
        self.toggle_client = self.create_client(Trigger, '/toggle_TF')
        if not self.toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('toggle_TF service is not available, proceeding with motion command')
        
        self.servoj_client = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        while not self.servoj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dobot_bringup_v3/srv/ServoJ service...')
        
        self.getpose_client = self.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose')
        while not self.getpose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dobot_bringup_v3/srv/GetPose service...')
        
        self.enable_robot_client = self.create_client(EnableRobot, '/dobot_bringup_v3/srv/EnableRobot')
        while not self.enable_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dobot_bringup_v3/srv/EnableRobot service...')
        
        # Internal state and configuration.
        self.Trajectory = False  # Flag for action mode.
        self.adc = []            # Accumulator for trajectory points.
        self.max_points = 25     # Threshold for max trajectory points.
        
        self.step_sleep = 0.35    # Delay between servo commands.
        self.point_time = 0.35    # Duration for each trajectory point.
        self.pose_diff_threshold = 0.001  # Threshold for stationary check.
        self.last_idle_update = 0.5      # Timestamp of last idle update.
        self.settle_time = 0.05           # Settling time in seconds after a successful TF toggle

    # ----------------------------------------------------------------------
    # Idle Mode: Update robot position from joint states every 0.5 sec.
    # ----------------------------------------------------------------------
    def joint_callback(self, msg):
        if not self.Trajectory:
            current_time = time.time()
            if (current_time - self.last_idle_update) < 0.5:
                return  # Skip update if 0.5 sec has not passed.
            self.last_idle_update = current_time
            
            # Extract first six joint positions and round them.
            joint = msg.position[0:6]
            joint_values = [round(val, 3) for val in joint]
            
            # Prepare a single-point trajectory goal.
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = [
                "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
            ]
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start.sec = int(self.point_time)
            point.time_from_start.nanosec = int((self.point_time - int(self.point_time)) * 1e9)
            goal_msg.trajectory.points.append(point)
            
            # Send idle update if the action server is ready.
            if self.idle_action_client.server_is_ready():
                self.idle_action_client.send_goal_async(goal_msg)

    # ----------------------------------------------------------------------
    # Action Mode: Process incoming trajectory command.
    # ----------------------------------------------------------------------
    def trajectory_callback(self, msg):
        self.adc = []  # Clear previous trajectory points.
        if msg.trajectory and msg.trajectory[0].joint_trajectory.points:
            for point in msg.trajectory[0].joint_trajectory.points:
                # Convert each point's angles from radians to degrees.
                angles_deg = [round(180 * angle / 3.14159, 3) for angle in point.positions]
                self.adc.append(angles_deg)
                self.get_logger().info(f"Planning location: {angles_deg}")
            self.Trajectory = True
        # Suppress extra logging to reduce datalog spam.

    # ----------------------------------------------------------------------
    # Action Server Callback: Execute the trajectory command.
    # ----------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        if self.Trajectory:
            num_points = len(self.adc)
            self.get_logger().info(f"Received trajectory with {num_points} points.")
            
            # Check the threshold of trajectory points.
            if num_points > self.max_points:
                self.get_logger().warn(f"Trajectory exceeds threshold ({self.max_points} points). Terminating action.")
                goal_handle.succeed()
                result = FollowJointTrajectory.Result()
                result.error_code = 1
                self.Trajectory = False
                self.adc = []
                return result
            
            # --- Ensure Toggle_TF service has responded before moving ---
            self.get_logger().info("Sending toggle service request to disable data saving before motion.")
            toggle_result = self.call_toggle_service()  # Blocks until response received or timed out.
            if toggle_result:
                self.get_logger().info("Toggle service call succeeded. Waiting settling time before executing motion.")
                time.sleep(self.settle_time)
            else:
                self.get_logger().warn("Toggle_TF service did not return success; proceeding with motion command without extra settling delay.")
            
            # Execute each servo motion step.
            for angles in self.adc:
                self.ServoJ_C(*angles)
                time.sleep(self.step_sleep)
            
            # Wait for the robot to become stationary.
            self.get_logger().info("Checking if robot is stationary before re-enabling data saving.")
            # Loop indefinitely until stationarity is confirmed.
            while not self.wait_until_stationary():
                self.get_logger().warn("Robot is not stationary yet. Waiting...")
            self.get_logger().info("Robot is stationary.")
            
            # --- Re-enable data saving ---
            self.get_logger().info("Sending toggle service request to re-enable data saving.")
            toggle_result = self.call_toggle_service()
            if toggle_result:
                self.get_logger().info("Toggle service re-enable succeeded. Waiting settling time before returning to idle mode.")
                time.sleep(self.settle_time)
            else:
                self.get_logger().warn("Toggle_TF service re-enable call failed; proceeding anyway.")
            
            self.Trajectory = False
            self.adc = []
        else:
            # No trajectory data; do nothing.
            pass
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    # ----------------------------------------------------------------------
    # Helper: Call the toggle service to disable/enable data saving.
    # ----------------------------------------------------------------------
    def call_toggle_service(self):
        req = Trigger.Request()
        future = self.toggle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f"Toggle service succeeded: {response.message}")
                return True
            else:
                self.get_logger().warn(f"Toggle service returned failure: {response.message}")
                return False
        else:
            self.get_logger().warn("Toggle service call timed out.")
            return False

    # ----------------------------------------------------------------------
    # Helper: Wait until the robot is stationary (pose differences below threshold).
    # Logs the actual error value of the stationary check.
    # ----------------------------------------------------------------------
    def wait_until_stationary(self):
        previous_pose = self.get_current_pose()
        if previous_pose is None:
            return False
        while True:
            time.sleep(0.5)
            current_pose = self.get_current_pose()
            if current_pose is None:
                continue
            diff = sum(abs(a - b) for a, b in zip(previous_pose, current_pose))
            self.get_logger().info(f"Stationary check error (pose diff): {diff:.4f}")
            if diff < self.pose_diff_threshold:
                return True
            previous_pose = current_pose

    # ----------------------------------------------------------------------
    # Helper: Get the current robot pose by calling the GetPose service.
    # ----------------------------------------------------------------------
    def get_current_pose(self):
        req = GetPose.Request()
        req.user = 0
        req.tool = 0
        future = self.getpose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            response = future.result()
            pose_str = response.pose  # Expected format: "{x,y,z,roll,pitch,yaw}"
            try:
                pose_str = pose_str.strip('{}')
                parts = pose_str.split(',')
                pose = [float(x) for x in parts]
                return pose
            except Exception as e:
                self.get_logger().error(f"Error parsing pose: {e}")
                return None
        else:
            self.get_logger().error("GetPose service call timed out.")
            return None

    # ----------------------------------------------------------------------
    # Helper: Send a ServoJ command with the given joint angles.
    # ----------------------------------------------------------------------
    def ServoJ_C(self, j1, j2, j3, j4, j5, j6):
        req = ServoJ.Request()
        req.j1 = float(j1)
        req.j2 = float(j2)
        req.j3 = float(j3)
        req.j4 = float(j4)
        req.j5 = float(j5)
        req.j6 = float(j6)
        req.t = float(self.point_time)
        future = self.servoj_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            self.get_logger().info("ServoJ service response received.")
        else:
            self.get_logger().warn("ServoJ service call timed out.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
