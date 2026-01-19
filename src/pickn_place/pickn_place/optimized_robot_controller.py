#!/usr/bin/env python3
"""
Optimized Robot Controller - Enhanced version of the original manipulate_node
Key improvements:
- Modular architecture with specialized components
- Persistent node instance (eliminates node creation overhead)
- Advanced error handling and recovery
- Performance optimizations and caching
- Better resource management
- Comprehensive logging and monitoring
"""

from __future__ import annotations
import asyncio
import threading
import time
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple, Any, Union, Callable
from contextlib import asynccontextmanager, contextmanager
from concurrent.futures import ThreadPoolExecutor
import functools
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformListener
import tf_transformations
from scipy.spatial.transform import Rotation as Rot

from pymoveit2 import MoveIt2, MoveIt2State
from dobot_msgs_v3.srv import *
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# =====================================================================================
# CONFIGURATION MANAGEMENT
# =====================================================================================

@dataclass
class RobotConfig:
    """Centralized configuration management"""
    # Motion parameters
    reference_frame: str = "base_link"
    end_effector_name: str = "tool_link"
    group_name: str = "portafilter_center"
    planner_id: str = "OMPL"
    
    # Cartesian motion
    cartesian: bool = True
    cartesian_max_step: float = 0.001
    cartesian_fraction_threshold: float = 0.8
    cartesian_jump_threshold: float = 0.0
    cartesian_avoid_collisions: bool = True
    
    # Motion scaling
    velocity_scaling: float = 1.0
    acceleration_scaling: float = 1.0
    synchronous: bool = True
    
    # TF stability parameters
    num_consecutive_tf: int = 9
    translation_threshold: float = 0.002
    rotation_threshold: float = 2.1
    sample_delay: float = 0.1
    
    # Performance tuning
    max_workers: int = 4
    cache_size: int = 100
    tf_cache_duration: float = 0.1
    service_timeout: float = 5.0
    motion_timeout: float = 30.0

    @classmethod
    def from_yaml(cls, file_path: str) -> 'RobotConfig':
        """Load configuration from YAML file"""
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            return cls(**data)
        except Exception:
            return cls()  # Return default config

# =====================================================================================
# PERFORMANCE MONITORING
# =====================================================================================

@dataclass
class PerformanceMetrics:
    """Track performance metrics for optimization"""
    operation_times: Dict[str, List[float]] = field(default_factory=dict)
    error_counts: Dict[str, int] = field(default_factory=dict)
    cache_hits: int = 0
    cache_misses: int = 0
    
    def record_operation(self, operation: str, duration: float):
        if operation not in self.operation_times:
            self.operation_times[operation] = []
        self.operation_times[operation].append(duration)
        # Keep only last 100 measurements
        if len(self.operation_times[operation]) > 100:
            self.operation_times[operation].pop(0)
    
    def record_error(self, operation: str):
        self.error_counts[operation] = self.error_counts.get(operation, 0) + 1
    
    def get_average_time(self, operation: str) -> float:
        times = self.operation_times.get(operation, [])
        return sum(times) / len(times) if times else 0.0

# =====================================================================================
# ADVANCED ERROR HANDLING
# =====================================================================================

class RobotError(Exception):
    """Base exception for robot operations"""
    pass

class MotionError(RobotError):
    """Motion planning or execution error"""
    pass

class ServiceError(RobotError):
    """Service communication error"""
    pass

class TimeoutError(RobotError):
    """Operation timeout error"""
    pass

def with_retry(max_attempts: int = 3, delay: float = 0.1, backoff: float = 2.0):
    """Decorator for automatic retry with exponential backoff"""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None
            current_delay = delay
            
            for attempt in range(max_attempts):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_attempts - 1:
                        time.sleep(current_delay)
                        current_delay *= backoff
                    continue
            
            raise last_exception
        return wrapper
    return decorator

def with_timeout(timeout_seconds: float):
    """Decorator for operation timeout"""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            import signal
            
            def timeout_handler(signum, frame):
                raise TimeoutError(f"{func.__name__} timed out after {timeout_seconds}s")
            
            old_handler = signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(int(timeout_seconds))
            
            try:
                result = func(*args, **kwargs)
                signal.alarm(0)
                return result
            except TimeoutError:
                raise
            finally:
                signal.signal(signal.SIGALRM, old_handler)
        return wrapper
    return decorator

# =====================================================================================
# SPECIALIZED COMPONENTS
# =====================================================================================

class TFManager:
    """High-performance TF management with caching"""
    
    def __init__(self, node: Node, config: RobotConfig):
        self.node = node
        self.config = config
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.cache: Dict[str, Tuple[List[float], float]] = {}
        self.lock = threading.RLock()
    
    @with_retry(max_attempts=3)
    def get_transform(self, target_frame: str, source_frame: str = None) -> Optional[List[float]]:
        """Get transform with caching and retry logic"""
        source_frame = source_frame or self.config.reference_frame
        cache_key = f"{source_frame}->{target_frame}"
        
        with self.lock:
            # Check cache first
            if cache_key in self.cache:
                transform, timestamp = self.cache[cache_key]
                if time.time() - timestamp < self.config.tf_cache_duration:
                    return transform.copy()
            
            try:
                tf_obj = self.tf_buffer.lookup_transform(
                    source_frame, target_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                
                t = tf_obj.transform.translation
                r = tf_obj.transform.rotation
                transform = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
                
                # Cache the result
                self.cache[cache_key] = (transform, time.time())
                return transform
                
            except Exception as e:
                self.node.get_logger().error(f"TF lookup failed for {cache_key}: {e}")
                return None
    
    def get_stable_transform(self, target_frame: str) -> Optional[List[float]]:
        """Get stable transform using statistical analysis"""
        samples = []
        for _ in range(self.config.num_consecutive_tf):
            transform = self.get_transform(target_frame)
            if transform:
                samples.append(transform)
            time.sleep(self.config.sample_delay)
        
        if len(samples) < self.config.num_consecutive_tf:
            return None
        
        # Analyze stability
        translations = np.array([s[:3] for s in samples])
        rotations = np.array([s[3:] for s in samples])
        
        trans_std = np.std(translations, axis=0)
        if np.max(trans_std) > self.config.translation_threshold:
            return None
        
        # Return average transform
        avg_trans = np.mean(translations, axis=0)
        avg_quat = self._average_quaternions(rotations)
        
        return list(avg_trans) + list(avg_quat)
    
    def _average_quaternions(self, quaternions: np.ndarray) -> np.ndarray:
        """Compute average quaternion using proper methodology"""
        # Markley method for quaternion averaging
        M = np.zeros((4, 4))
        for q in quaternions:
            M += np.outer(q, q)
        M /= quaternions.shape[0]
        
        eigenvalues, eigenvectors = np.linalg.eig(M)
        max_index = np.argmax(eigenvalues)
        return eigenvectors[:, max_index]

class ServiceManager:
    """Centralized service management with connection pooling"""
    
    def __init__(self, node: Node, config: RobotConfig):
        self.node = node
        self.config = config
        self.clients: Dict[str, Any] = {}
        self.lock = threading.RLock()
    
    def get_client(self, service_type, service_name: str):
        """Get or create service client with caching"""
        with self.lock:
            if service_name not in self.clients:
                client = self.node.create_client(service_type, service_name)
                self.clients[service_name] = client
            return self.clients[service_name]
    
    @with_retry(max_attempts=3)
    @with_timeout(5.0)
    def call_service(self, service_type, service_name: str, request) -> Any:
        """Call service with retry and timeout"""
        client = self.get_client(service_type, service_name)
        
        if not client.wait_for_service(timeout_sec=self.config.service_timeout):
            raise ServiceError(f"Service {service_name} not available")
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.config.service_timeout)
        
        if not future.done() or future.result() is None:
            raise ServiceError(f"Service call {service_name} failed")
        
        return future.result()

class MotionManager:
    """Advanced motion planning and execution"""
    
    def __init__(self, node: Node, config: RobotConfig):
        self.node = node
        self.config = config
        self.moveit2_instances: Dict[str, MoveIt2] = {}
        self.lock = threading.RLock()
    
    def get_moveit2(self, end_effector: str = None) -> MoveIt2:
        """Get or create MoveIt2 instance"""
        end_effector = end_effector or self.config.end_effector_name
        
        with self.lock:
            if end_effector not in self.moveit2_instances:
                moveit2 = MoveIt2(
                    node=self.node,
                    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                    base_link_name=self.config.reference_frame,
                    end_effector_name=end_effector,
                    group_name=self.config.group_name,
                )
                self._configure_moveit2(moveit2)
                self.moveit2_instances[end_effector] = moveit2
            
            return self.moveit2_instances[end_effector]
    
    def _configure_moveit2(self, moveit2: MoveIt2):
        """Configure MoveIt2 instance with optimal settings"""
        moveit2.planner_id = self.config.planner_id
        moveit2.max_velocity = self.config.velocity_scaling
        moveit2.max_acceleration = self.config.acceleration_scaling
        moveit2.cartesian_jump_threshold = self.config.cartesian_jump_threshold
        moveit2.cartesian_avoid_collisions = self.config.cartesian_avoid_collisions
    
    @with_timeout(30.0)
    def execute_motion(self, target_pose: List[float], end_effector: str = None) -> bool:
        """Execute motion with advanced error handling"""
        moveit2 = self.get_moveit2(end_effector)
        
        try:
            position = target_pose[:3]
            quaternion = target_pose[3:] if len(target_pose) > 3 else [0, 0, 0, 1]
            
            moveit2.move_to_pose(
                position=position,
                quat_xyzw=quaternion,
                cartesian=self.config.cartesian,
                cartesian_max_step=self.config.cartesian_max_step,
                cartesian_fraction_threshold=self.config.cartesian_fraction_threshold,
            )
            
            moveit2.wait_until_executed()
            return moveit2.query_state() == MoveIt2State.IDLE
            
        except Exception as e:
            self.node.get_logger().error(f"Motion execution failed: {e}")
            return False

# =====================================================================================
# MAIN OPTIMIZED CONTROLLER
# =====================================================================================

class OptimizedRobotController:
    """Main robot controller with all optimizations"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern for persistent node"""
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
            
        self.config = RobotConfig()
        self.metrics = PerformanceMetrics()
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node("optimized_robot_controller")
        self.executor = MultiThreadedExecutor(num_threads=self.config.max_workers)
        self.executor.add_node(self.node)
        
        # Initialize specialized managers
        self.tf_manager = TFManager(self.node, self.config)
        self.service_manager = ServiceManager(self.node, self.config)
        self.motion_manager = MotionManager(self.node, self.config)
        
        # Start executor in background thread
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=False)
        self.executor_thread.start()
        
        self._initialized = True
        self.node.get_logger().info("OptimizedRobotController initialized")
    
    def cleanup(self):
        """Properly cleanup the controller and its threads"""
        try:
            if hasattr(self, 'executor'):
                self.executor.shutdown()
            if hasattr(self, 'executor_thread') and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=3.0)
            if hasattr(self, 'node'):
                self.node.destroy_node()
        except Exception as e:
            print(f"Warning: Error during OptimizedRobotController cleanup: {e}")
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()
    
    def performance_monitor(self, operation_name: str):
        """Decorator for performance monitoring"""
        def decorator(func):
            @functools.wraps(func)
            def wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    duration = time.time() - start_time
                    self.metrics.record_operation(operation_name, duration)
                    self.node.get_logger().debug(f"{operation_name} completed in {duration:.3f}s")
                    return result
                except Exception as e:
                    self.metrics.record_error(operation_name)
                    self.node.get_logger().error(f"{operation_name} failed: {e}")
                    raise
            return wrapper
        return decorator
    
    @performance_monitor("move_to_pose")
    def move_to_pose(self, position: List[float], orientation: List[float] = None, 
                    end_effector: str = None) -> bool:
        """Move to absolute pose with enhanced reliability"""
        orientation = orientation or [0, 0, 0, 1]
        target_pose = position + orientation
        return self.motion_manager.execute_motion(target_pose, end_effector)
    
    @performance_monitor("get_stable_transform")
    def get_stable_transform(self, target_frame: str) -> Optional[List[float]]:
        """Get stable transform with caching"""
        return self.tf_manager.get_stable_transform(target_frame)
    
    @performance_monitor("set_gripper")
    def set_gripper_position(self, position: int, speed: int = 255, force: int = 255) -> bool:
        """Set gripper position with retry logic"""
        try:
            from dobot_msgs_v3.srv import SetGripperPosition
            request = SetGripperPosition.Request()
            request.position = max(0, min(255, position))
            request.speed = max(0, min(255, speed))
            request.force = max(0, min(255, force))
            
            response = self.service_manager.call_service(
                SetGripperPosition, 
                '/dobot_bringup_v3/srv/SetGripperPosition',
                request
            )
            
            return getattr(response, 'res', 1) == 0
            
        except Exception as e:
            self.node.get_logger().error(f"Gripper control failed: {e}")
            return False
    
    def get_performance_report(self) -> Dict[str, Any]:
        """Get detailed performance metrics"""
        report = {
            "operation_averages": {
                op: self.metrics.get_average_time(op) 
                for op in self.metrics.operation_times.keys()
            },
            "error_counts": self.metrics.error_counts.copy(),
            "cache_hit_rate": (
                self.metrics.cache_hits / 
                (self.metrics.cache_hits + self.metrics.cache_misses)
                if (self.metrics.cache_hits + self.metrics.cache_misses) > 0 else 0
            ),
            "total_operations": sum(
                len(times) for times in self.metrics.operation_times.values()
            )
        }
        return report
    
    def shutdown(self):
        """Clean shutdown"""
        if hasattr(self, 'executor'):
            self.executor.shutdown()
        if hasattr(self, 'node'):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# =====================================================================================
# ENHANCED SKILL EXECUTION
# =====================================================================================

class SkillExecutor:
    """Advanced skill execution with parallel processing"""
    
    def __init__(self):
        self.controller = OptimizedRobotController()
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        
    def execute_skill(self, skill_name: str, *args, **kwargs) -> Any:
        """Execute single skill with monitoring"""
        if not hasattr(self.controller, skill_name):
            raise ValueError(f"Unknown skill: {skill_name}")
        
        method = getattr(self.controller, skill_name)
        return method(*args, **kwargs)
    
    def execute_parallel(self, skill_commands: List[Tuple[str, tuple, dict]]) -> List[Any]:
        """Execute multiple skills in parallel where safe"""
        futures = []
        for skill_name, args, kwargs in skill_commands:
            future = self.thread_pool.submit(self.execute_skill, skill_name, *args, **kwargs)
            futures.append(future)
        
        return [future.result() for future in futures]
    
    def execute_sequence(self, skill_sequence: List[Tuple[str, tuple, dict]], 
                        stop_on_error: bool = True) -> List[Any]:
        """Execute skills in sequence with error handling"""
        results = []
        for skill_name, args, kwargs in skill_sequence:
            try:
                result = self.execute_skill(skill_name, *args, **kwargs)
                results.append(result)
                if stop_on_error and result is False:
                    break
            except Exception as e:
                if stop_on_error:
                    raise
                results.append(None)
        
        return results

# =====================================================================================
# SIMPLIFIED API
# =====================================================================================

# Global skill executor instance
_skill_executor = None

def get_skill_executor() -> SkillExecutor:
    """Get global skill executor instance"""
    global _skill_executor
    if _skill_executor is None:
        _skill_executor = SkillExecutor()
    return _skill_executor

def run_skill(skill_name: str, *args, **kwargs) -> Any:
    """Optimized skill execution - no node creation overhead"""
    executor = get_skill_executor()
    return executor.execute_skill(skill_name, *args, **kwargs)

def run_skills_parallel(*skill_specs) -> List[Any]:
    """Execute multiple skills in parallel"""
    executor = get_skill_executor()
    commands = [(spec[0], spec[1:], {}) for spec in skill_specs]
    return executor.execute_parallel(commands)

def run_skills_sequence(*skill_specs, stop_on_error: bool = True) -> List[Any]:
    """Execute skills in sequence"""
    executor = get_skill_executor()
    commands = [(spec[0], spec[1:], {}) for spec in skill_specs]
    return executor.execute_sequence(commands, stop_on_error)

# Cleanup function
def cleanup():
    """Clean shutdown of all resources"""
    global _skill_executor
    if _skill_executor:
        _skill_executor.controller.shutdown()
        _skill_executor = None

if __name__ == "__main__":
    # Example usage
    try:
        # Single skill
        result = run_skill("move_to_pose", [0.3, 0.0, 0.2])
        
        # Parallel execution
        results = run_skills_parallel(
            ("get_stable_transform", "target_frame_1"),
            ("get_stable_transform", "target_frame_2")
        )
        
        # Performance report
        executor = get_skill_executor()
        report = executor.controller.get_performance_report()
        print(f"Performance report: {report}")
        
    finally:
        cleanup() 