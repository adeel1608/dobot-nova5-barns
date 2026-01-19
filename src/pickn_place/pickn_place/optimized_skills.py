#!/usr/bin/env python3
"""
Optimized Skills Implementation
Enhanced versions of the original robot skills with:
- Better error handling and recovery
- Performance optimizations
- Parallel processing capabilities
- Advanced caching and resource management
"""

import asyncio
import time
import math
import numpy as np
from typing import List, Optional, Dict, Tuple, Any
import yaml
import os
from dataclasses import dataclass
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

import rclpy
from rclpy.node import Node
import tf_transformations
from scipy.spatial.transform import Rotation as Rot
from pymoveit2 import MoveIt2, MoveIt2State

from dobot_msgs_v3.srv import *
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Import the base optimized controller
from .optimized_robot_controller import OptimizedRobotController, with_retry, with_timeout, RobotError

class OptimizedSkills:
    """Enhanced skill implementations with performance optimizations"""
    
    def __init__(self, controller: OptimizedRobotController):
        self.controller = controller
        self.node = controller.node
        self.config = controller.config
        self.tf_manager = controller.tf_manager
        self.service_manager = controller.service_manager
        self.motion_manager = controller.motion_manager
        
        # Caches for expensive computations
        self._pose_cache = {}
        self._trajectory_cache = {}
        self._validation_cache = {}
        
        # Thread pools for parallel operations
        self.motion_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="motion")
        self.io_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="io")
    
    # =====================================================================================
    # ENHANCED MOTION SKILLS
    # =====================================================================================
    
    @with_retry(max_attempts=3)
    @with_timeout(30.0)
    def move_to_optimized(self, target_tf: str, distance: float, 
                         offset_x_mm: float = 0.0, offset_y_mm: float = 0.0, offset_z_mm: float = 0.0,
                         offset_roll_deg: float = 0.0, offset_pitch_deg: float = 0.0, offset_yaw_deg: float = 0.0,
                         use_cache: bool = True, validate_arrival: bool = True) -> bool:
        """
        Enhanced move_to with caching, parallel validation, and advanced error recovery
        """
        cache_key = f"{target_tf}_{distance}_{offset_x_mm}_{offset_y_mm}_{offset_z_mm}_{offset_roll_deg}_{offset_pitch_deg}_{offset_yaw_deg}"
        
        start_time = time.time()
        self.node.get_logger().info(f"move_to_optimized: {target_tf}, distance={distance}m")
        
        try:
            # 1) Fast TF lookup with caching
            if use_cache and cache_key in self._pose_cache:
                stable_tf, cache_time = self._pose_cache[cache_key]
                if time.time() - cache_time < self.config.tf_cache_duration:
                    self.controller.metrics.cache_hits += 1
                else:
                    stable_tf = None
            else:
                stable_tf = None
                self.controller.metrics.cache_misses += 1
            
            if stable_tf is None:
                stable_tf = self.tf_manager.get_stable_transform(target_tf)
                if stable_tf is None:
                    raise RobotError(f"Failed to get stable TF for {target_tf}")
                
                if use_cache:
                    self._pose_cache[cache_key] = (stable_tf, time.time())
            
            # 2) Parallel goal computation and validation
            with ThreadPoolExecutor(max_workers=2) as executor:
                # Compute goal pose
                goal_future = executor.submit(self._compute_enhanced_goal_pose, 
                                            stable_tf, distance, offset_x_mm, offset_y_mm, offset_z_mm,
                                            offset_roll_deg, offset_pitch_deg, offset_yaw_deg)
                
                # Pre-validate motion safety
                safety_future = executor.submit(self._validate_motion_safety, stable_tf, distance) if validate_arrival else None
                
                goal_pos, goal_quat = goal_future.result()
                if safety_future and not safety_future.result():
                    raise RobotError("Motion safety validation failed")
            
            # 3) Execute motion with advanced monitoring
            success = self._execute_motion_with_monitoring(goal_pos, goal_quat)
            
            # 4) Parallel arrival verification
            if validate_arrival and success:
                verification_future = self.io_pool.submit(self._verify_arrival_parallel, goal_pos)
                success = verification_future.result(timeout=5.0)
            
            duration = time.time() - start_time
            self.controller.metrics.record_operation("move_to_optimized", duration)
            
            return success
            
        except Exception as e:
            self.node.get_logger().error(f"move_to_optimized failed: {e}")
            self.controller.metrics.record_error("move_to_optimized")
            return False
    
    def _compute_enhanced_goal_pose(self, stable_tf: List[float], distance: float,
                                  offset_x_mm: float, offset_y_mm: float, offset_z_mm: float,
                                  offset_roll_deg: float, offset_pitch_deg: float, offset_yaw_deg: float) -> Tuple[List[float], List[float]]:
        """Optimized goal pose computation with vectorized operations"""
        target_pos = np.array(stable_tf[:3])
        base_ref = np.array([0.0, 0.0, target_pos[2] + 0.075])
        
        # Vectorized approach vector computation
        approach_vec = target_pos - base_ref
        approach_norm = np.linalg.norm(approach_vec)
        
        if approach_norm < 1e-6:
            raise RobotError("Approach vector too small")
        
        unit_vec = approach_vec / approach_norm
        goal_pos = target_pos - distance * unit_vec
        
        # Apply offsets efficiently
        offsets_m = np.array([offset_x_mm, offset_y_mm, offset_z_mm]) / 1000.0
        goal_pos += offsets_m
        
        # Compute orientation using optimized rotation matrices
        goal_quat = self._compute_orientation_optimized(unit_vec, offset_roll_deg, offset_pitch_deg, offset_yaw_deg)
        
        return goal_pos.tolist(), goal_quat
    
    def _compute_orientation_optimized(self, unit_vec: np.ndarray, roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[float]:
        """Optimized orientation computation using scipy rotations"""
        # Use scipy for faster rotation operations
        z_axis = unit_vec
        up = np.array([0, 0, 1])
        
        if abs(np.dot(z_axis, up)) > 0.99:
            up = np.array([0, 1, 0])
        
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        R_base = np.column_stack((x_axis, y_axis, z_axis))
        
        # Apply rotation offsets if any
        if any(angle != 0.0 for angle in [roll_deg, pitch_deg, yaw_deg]):
            r_offset = Rot.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True)
            R_total = R_base @ r_offset.as_matrix()
        else:
            R_total = R_base
        
        # Convert to quaternion using scipy
        quat_scipy = Rot.from_matrix(R_total).as_quat()
        return quat_scipy.tolist()  # [x, y, z, w] format
    
    def _validate_motion_safety(self, stable_tf: List[float], distance: float) -> bool:
        """Pre-validate motion for safety constraints"""
        # Check workspace bounds
        target_pos = np.array(stable_tf[:3])
        goal_pos = target_pos - distance * np.array([0, 0, 1])  # Simplified check
        
        # Basic workspace validation
        if goal_pos[2] < 0.1:  # Too low
            return False
        if np.linalg.norm(goal_pos[:2]) > 0.8:  # Too far from base
            return False
        
        return True
    
    def _execute_motion_with_monitoring(self, goal_pos: List[float], goal_quat: List[float]) -> bool:
        """Execute motion with real-time monitoring"""
        moveit2 = self.motion_manager.get_moveit2()
        
        # Set up monitoring
        motion_monitor = MotionMonitor(self.node, self.tf_manager)
        monitor_thread = threading.Thread(target=motion_monitor.start_monitoring, daemon=True)
        monitor_thread.start()
        
        try:
            moveit2.move_to_pose(
                position=goal_pos,
                quat_xyzw=goal_quat,
                cartesian=self.config.cartesian,
                cartesian_max_step=self.config.cartesian_max_step,
                cartesian_fraction_threshold=self.config.cartesian_fraction_threshold,
            )
            
            moveit2.wait_until_executed()
            return moveit2.query_state() == MoveIt2State.IDLE
            
        finally:
            motion_monitor.stop_monitoring()
    
    def _verify_arrival_parallel(self, expected_pose: List[float]) -> bool:
        """Parallel arrival verification using multiple methods"""
        with ThreadPoolExecutor(max_workers=3) as executor:
            # Multiple verification methods
            servo_future = executor.submit(self._wait_for_servo_ready)
            pose_future = executor.submit(self._verify_pose_accuracy, expected_pose)
            stability_future = executor.submit(self._verify_motion_stability)
            
            # Wait for all verifications
            servo_ready = servo_future.result(timeout=10.0)
            pose_accurate = pose_future.result(timeout=5.0)
            motion_stable = stability_future.result(timeout=3.0)
            
            return servo_ready and pose_accurate and motion_stable
    
    def _wait_for_servo_ready(self, timeout: float = 15.0) -> bool:
        """Enhanced servo ready check with exponential backoff"""
        from std_srvs.srv import Trigger
        
        client = self.service_manager.get_client(Trigger, 'get_servo_status')
        start_time = time.time()
        check_interval = 0.1
        
        while time.time() - start_time < timeout:
            try:
                request = Trigger.Request()
                response = self.service_manager.call_service(Trigger, 'get_servo_status', request)
                
                if hasattr(response, 'message'):
                    try:
                        status = int(response.message)
                        if status == 0:  # READY
                            return True
                    except ValueError:
                        pass
                
                time.sleep(min(check_interval, timeout - (time.time() - start_time)))
                check_interval = min(check_interval * 1.1, 1.0)  # Exponential backoff
                
            except Exception as e:
                self.node.get_logger().warn(f"Servo status check failed: {e}")
                time.sleep(0.5)
        
        return False
    
    def _verify_pose_accuracy(self, expected_pose: List[float], tolerance_mm: float = 2.0) -> bool:
        """Verify pose accuracy using current robot state"""
        try:
            from dobot_msgs_v3.srv import GetPose
            
            request = GetPose.Request()
            request.user = 0
            request.tool = 0
            
            response = self.service_manager.call_service(GetPose, '/dobot_bringup_v3/srv/GetPose', request)
            
            if response and hasattr(response, 'pose'):
                pose_str = response.pose.strip('{}')
                parts = pose_str.split(',')
                if len(parts) >= 3:
                    current_pos_mm = np.array([float(parts[i]) for i in range(3)])
                    expected_pos_mm = np.array(expected_pose[:3]) * 1000.0
                    
                    error_mm = np.linalg.norm(current_pos_mm - expected_pos_mm)
                    return error_mm <= tolerance_mm
            
            return False
            
        except Exception as e:
            self.node.get_logger().error(f"Pose verification failed: {e}")
            return False
    
    def _verify_motion_stability(self, samples: int = 5, threshold_deg: float = 0.5) -> bool:
        """Verify robot has stopped moving by checking joint stability"""
        joint_samples = []
        
        for _ in range(samples):
            try:
                # Get current joint state
                js_msg = self._get_joint_state_blocking(timeout=1.0)
                if js_msg and len(js_msg.position) >= 6:
                    angles_deg = [math.degrees(j) for j in js_msg.position[:6]]
                    joint_samples.append(angles_deg)
                
                time.sleep(0.05)  # 50ms between samples
                
            except Exception:
                continue
        
        if len(joint_samples) < samples:
            return False
        
        # Check stability across all joints
        for joint_idx in range(6):
            joint_values = [sample[joint_idx] for sample in joint_samples]
            if max(joint_values) - min(joint_values) > threshold_deg:
                return False
        
        return True
    
    def _get_joint_state_blocking(self, timeout: float = 2.0) -> Optional[JointState]:
        """Get joint state with timeout"""
        msg_container = {"msg": None}
        
        def callback(msg):
            msg_container["msg"] = msg
        
        subscription = self.node.create_subscription(
            JointState, "/joint_states_robot", callback, 10
        )
        
        start_time = time.time()
        while msg_container["msg"] is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.node.destroy_subscription(subscription)
        return msg_container["msg"]
    
    # =====================================================================================
    # ENHANCED GRIPPER AND TOOL CONTROL
    # =====================================================================================
    
    @with_retry(max_attempts=3)
    def set_gripper_optimized(self, position: int, speed: int = 255, force: int = 255,
                            verify_position: bool = True, timeout: float = 5.0) -> Optional[int]:
        """
        Enhanced gripper control with position verification and feedback
        """
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))
        
        start_time = time.time()
        
        try:
            # Parallel execution of command and monitoring
            with ThreadPoolExecutor(max_workers=2) as executor:
                # Send gripper command
                command_future = executor.submit(self._send_gripper_command_async, position, speed, force)
                
                # Start position monitoring if verification requested
                if verify_position:
                    monitor_future = executor.submit(self._monitor_gripper_position, position, timeout)
                else:
                    monitor_future = None
                
                # Wait for command completion
                command_success = command_future.result(timeout=timeout)
                if not command_success:
                    return None
                
                # Wait for position verification
                if monitor_future:
                    final_position = monitor_future.result(timeout=timeout)
                    return final_position
                else:
                    # Just wait a bit for settling
                    time.sleep(0.2)
                    return self._read_gripper_position()
        
        except Exception as e:
            self.node.get_logger().error(f"Gripper control failed: {e}")
            return None
    
    def _send_gripper_command_async(self, position: int, speed: int, force: int) -> bool:
        """Send gripper command asynchronously"""
        try:
            from dobot_msgs_v3.srv import SetGripperPosition
            
            request = SetGripperPosition.Request()
            request.position = position
            request.speed = speed
            request.force = force
            
            response = self.service_manager.call_service(
                SetGripperPosition,
                '/dobot_bringup_v3/srv/SetGripperPosition',
                request
            )
            
            return getattr(response, 'res', 1) == 0
            
        except Exception as e:
            self.node.get_logger().error(f"Gripper command failed: {e}")
            return False
    
    def _monitor_gripper_position(self, target_position: int, timeout: float) -> Optional[int]:
        """Monitor gripper position until target is reached or timeout"""
        start_time = time.time()
        tolerance = 5  # Position tolerance
        stable_readings = 0
        required_stable_readings = 3
        
        while time.time() - start_time < timeout:
            current_position = self._read_gripper_position()
            
            if current_position is not None:
                error = abs(current_position - target_position)
                
                if error <= tolerance:
                    stable_readings += 1
                    if stable_readings >= required_stable_readings:
                        return current_position
                else:
                    stable_readings = 0
            
            time.sleep(0.1)
        
        return None
    
    def _read_gripper_position(self) -> Optional[int]:
        """Read current gripper position"""
        try:
            from dobot_msgs_v3.srv import GetGripperPosition
            
            request = GetGripperPosition.Request()
            request.index = 0
            
            response = self.service_manager.call_service(
                GetGripperPosition,
                '/dobot_bringup_v3/srv/GetGripperPosition',
                request
            )
            
            return getattr(response, 'position', None)
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to read gripper position: {e}")
            return None
    
    # =====================================================================================
    # ENHANCED JOINT CONTROL
    # =====================================================================================
    
    @with_retry(max_attempts=2)
    @with_timeout(30.0)
    def move_joints_optimized(self, joint_angles_deg: List[float], 
                            velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0,
                            relative: bool = True, validate_limits: bool = True) -> bool:
        """
        Enhanced joint movement with trajectory optimization and safety validation
        """
        try:
            # Input validation and clamping
            joint_angles_deg = [float(x) for x in joint_angles_deg[:6]]  # Ensure 6 joints
            velocity_scaling = max(0.1, min(1.0, float(velocity_scaling)))
            acceleration_scaling = max(0.1, min(1.0, float(acceleration_scaling)))
            
            # Get current joint state if relative movement
            if relative:
                current_joints = self._get_current_joint_angles()
                if current_joints is None:
                    raise RobotError("Failed to get current joint state")
                target_joints_deg = [c + r for c, r in zip(current_joints, joint_angles_deg)]
            else:
                target_joints_deg = joint_angles_deg
            
            # Validate joint limits
            if validate_limits and not self._validate_joint_limits(target_joints_deg):
                raise RobotError("Joint angles exceed limits")
            
            # Convert to radians
            target_joints_rad = [math.radians(angle) for angle in target_joints_deg]
            
            # Execute motion with temporary scaling
            with self._joint_scaling_context(velocity_scaling, acceleration_scaling):
                moveit2 = self.motion_manager.get_moveit2()
                
                # Plan and execute
                moveit2.move_to_configuration(target_joints_rad)
                moveit2.wait_until_executed()
                
                # Verify completion
                return self._verify_joint_arrival(target_joints_deg)
        
        except Exception as e:
            self.node.get_logger().error(f"Joint movement failed: {e}")
            return False
    
    def _get_current_joint_angles(self, timeout: float = 2.0) -> Optional[List[float]]:
        """Get current joint angles in degrees with stability check"""
        samples = []
        required_samples = 3
        
        for _ in range(required_samples):
            js = self._get_joint_state_blocking(timeout=timeout/required_samples)
            if js and len(js.position) >= 6:
                angles_deg = [math.degrees(j) for j in js.position[:6]]
                samples.append(angles_deg)
            time.sleep(0.05)
        
        if len(samples) < required_samples:
            return None
        
        # Check stability and return average
        for joint_idx in range(6):
            joint_values = [sample[joint_idx] for sample in samples]
            if max(joint_values) - min(joint_values) > 1.0:  # 1 degree tolerance
                self.node.get_logger().warn(f"Joint {joint_idx+1} unstable during reading")
        
        # Return average
        return [sum(samples[i][j] for i in range(len(samples))) / len(samples) for j in range(6)]
    
    def _validate_joint_limits(self, joints_deg: List[float]) -> bool:
        """Validate joint angles against limits"""
        joint_limits_deg = [
            (-355, 355),    # joint1
            (-180, 180),    # joint2  
            (-160, 0),      # joint3
            (-360, 360),    # joint4
            (-360, 360),    # joint5
            (-360, 360),    # joint6
        ]
        
        for idx, (angle, (min_deg, max_deg)) in enumerate(zip(joints_deg, joint_limits_deg)):
            if not (min_deg <= angle <= max_deg):
                self.node.get_logger().error(
                    f"Joint {idx+1} angle {angle:.1f}° outside limits [{min_deg}°, {max_deg}°]"
                )
                return False
        
        return True
    
    @contextmanager
    def _joint_scaling_context(self, velocity_scaling: float, acceleration_scaling: float):
        """Context manager for temporary joint scaling"""
        moveit2 = self.motion_manager.get_moveit2()
        orig_vel = moveit2.max_velocity
        orig_acc = moveit2.max_acceleration
        
        try:
            moveit2.max_velocity = velocity_scaling
            moveit2.max_acceleration = acceleration_scaling
            yield
        finally:
            moveit2.max_velocity = orig_vel
            moveit2.max_acceleration = orig_acc
    
    def _verify_joint_arrival(self, target_joints_deg: List[float], tolerance_deg: float = 1.0) -> bool:
        """Verify robot reached target joint configuration"""
        current_joints = self._get_current_joint_angles()
        if current_joints is None:
            return False
        
        for current, target in zip(current_joints, target_joints_deg):
            if abs(current - target) > tolerance_deg:
                return False
        
        return True

# =====================================================================================
# MOTION MONITORING
# =====================================================================================

class MotionMonitor:
    """Real-time motion monitoring for safety and performance"""
    
    def __init__(self, node: Node, tf_manager):
        self.node = node
        self.tf_manager = tf_manager
        self.monitoring = False
        self.safety_limits = {
            'max_velocity': 1.0,  # m/s
            'max_acceleration': 2.0,  # m/s²
            'workspace_bounds': [[-0.8, 0.8], [-0.8, 0.8], [0.0, 1.0]]  # x, y, z limits
        }
        self.previous_pose = None
        self.previous_time = None
    
    def start_monitoring(self):
        """Start motion monitoring in background"""
        self.monitoring = True
        self.previous_pose = None
        self.previous_time = None
        
        while self.monitoring:
            try:
                self._check_motion_safety()
                time.sleep(0.01)  # 100 Hz monitoring
            except Exception as e:
                self.node.get_logger().error(f"Motion monitoring error: {e}")
                break
    
    def stop_monitoring(self):
        """Stop motion monitoring"""
        self.monitoring = False
    
    def _check_motion_safety(self):
        """Check motion safety constraints"""
        current_pose = self.tf_manager.get_transform("Link6")
        current_time = time.time()
        
        if current_pose is None:
            return
        
        # Check workspace bounds
        if not self._check_workspace_bounds(current_pose[:3]):
            self.node.get_logger().warn("Robot approaching workspace boundary")
        
        # Check velocity if we have previous data
        if self.previous_pose is not None and self.previous_time is not None:
            dt = current_time - self.previous_time
            if dt > 0:
                velocity = np.linalg.norm(np.array(current_pose[:3]) - np.array(self.previous_pose[:3])) / dt
                if velocity > self.safety_limits['max_velocity']:
                    self.node.get_logger().warn(f"High velocity detected: {velocity:.3f} m/s")
        
        self.previous_pose = current_pose
        self.previous_time = current_time
    
    def _check_workspace_bounds(self, position: List[float]) -> bool:
        """Check if position is within workspace bounds"""
        x, y, z = position
        bounds = self.safety_limits['workspace_bounds']
        
        return (bounds[0][0] <= x <= bounds[0][1] and
                bounds[1][0] <= y <= bounds[1][1] and
                bounds[2][0] <= z <= bounds[2][1])

if __name__ == "__main__":
    # Example usage of optimized skills
    from .optimized_robot_controller import OptimizedRobotController
    
    try:
        controller = OptimizedRobotController()
        skills = OptimizedSkills(controller)
        
        # Example: Enhanced move operation
        success = skills.move_to_optimized("target_frame", 0.1, offset_z_mm=50)
        print(f"Move operation: {'Success' if success else 'Failed'}")
        
        # Example: Optimized gripper control
        position = skills.set_gripper_optimized(150, verify_position=True)
        print(f"Gripper position: {position}")
        
        # Example: Enhanced joint movement
        success = skills.move_joints_optimized([10, 0, 0, 0, 0, 0], relative=True)
        print(f"Joint movement: {'Success' if success else 'Failed'}")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown() 