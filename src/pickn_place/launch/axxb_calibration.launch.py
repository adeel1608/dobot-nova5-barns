#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    TimerAction, 
    IncludeLaunchDescription, 
    OpaqueFunction, 
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

import rclpy
from rclpy.node import Node as RCLNode
import time

# Global state tracking
class NodeStateTracker:
    """Track which nodes have successfully started"""
    camera_ready = False
    perception_ready = False
    
    @classmethod
    def reset(cls):
        cls.camera_ready = False
        cls.perception_ready = False

def check_node_ready(context, node_name, topic_name, timeout=10.0):
    """
    Check if a node is ready by verifying its topics are publishing.
    Returns True if ready, False otherwise.
    """
    try:
        rclpy.init()
        node = RCLNode(f'checker_{node_name}')
        
        start_time = time.time()
        ready = False
        
        while time.time() - start_time < timeout:
            topic_names_and_types = node.get_topic_names_and_types()
            topic_list = [name for name, _ in topic_names_and_types]
            
            if topic_name in topic_list:
                time.sleep(0.5)
                ready = True
                break
            
            time.sleep(0.2)
        
        node.destroy_node()
        rclpy.shutdown()
        return ready
    except Exception as e:
        print(f"Error checking {node_name}: {e}")
        return False

def wait_for_camera(context):
    """Wait for camera to be ready before launching perception"""
    print("⏳ Waiting for Orbbec camera to initialize...")
    
    # Check if color and depth topics are available
    color_ready = check_node_ready(context, 'camera_color', '/camera/color/image_raw', timeout=15.0)
    depth_ready = check_node_ready(context, 'camera_depth', '/camera/depth/image_raw', timeout=15.0)
    
    if color_ready and depth_ready:
        print("✅ Camera is ready!")
        NodeStateTracker.camera_ready = True
        return [create_calibration_perception_node()]
    else:
        print("❌ Camera failed to initialize properly!")
        return [EmitEvent(event=Shutdown(reason='Camera initialization failed'))]

def wait_for_perception(context):
    """Wait for perception node to be ready before launching calibration node"""
    if not NodeStateTracker.camera_ready:
        print("⚠️ Camera not ready, delaying perception...")
        return []
    
    print("⏳ Waiting for calibration perception node...")
    time.sleep(2.0)
    
    # Check if TF is being published
    ready = check_node_ready(context, 'perception', '/tf', timeout=10.0)
    
    if ready:
        print("✅ Perception node is ready!")
        NodeStateTracker.perception_ready = True
        return [create_calibration_node()]
    else:
        print("⚠️ Perception node startup issue (may be OK if markers not visible yet)")
        NodeStateTracker.perception_ready = True
        return [create_calibration_node()]

def create_calibration_perception_node():
    """Create the calibration perception node"""
    return Node(
        package='pickn_place',
        executable='calibration_perception',
        name='calibration_perception_node',
        output='screen',
        parameters=[{'initialized': True}],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=False,
        respawn_delay=2.0
    )

def create_calibration_node():
    """Create the calibration node"""
    return Node(
        package='pickn_place',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[{'initialized': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=False
    )

def prompt_calibration_move(context, *args, **kwargs):
    """Prompt user before starting calibration move sequence"""
    if not NodeStateTracker.perception_ready:
        print("⚠️ Perception not ready yet, please wait...")
        return []
    
    print("\n" + "="*60)
    print("         🎯 MANUAL DRAG-TEACH CALIBRATION READY")
    print("="*60)
    print("📋 PRE-FLIGHT CHECKLIST:")
    print("   ✓ Ensure all 4 ArUco markers (IDs 1,2,3,4) are visible")
    print("   ✓ Check visualization window for marker detection")
    print("   ✓ ROI box will show GREEN if distance is good")
    print("")
    print("🤖 HOW IT WORKS:")
    print("   1. Press ENTER → Robot enters drag mode (free movement)")
    print("   2. Physically move robot to a new pose")
    print("   3. Press ENTER → Robot locks & captures data")
    print("   4. Repeat (minimum: 10 samples, recommended: 51)")
    print("   5. Type 'done' when finished")
    print("")
    print("💡 TIPS:")
    print("   • Vary poses widely for best results")
    print("   • Distance ~85cm is ideal but not enforced")
    print("   • At least 10 samples required, 51 recommended")
    print("="*60)
    
    try:
        print("✋ Press ENTER when ready to start calibration sequence: ")
        sys.stdout.flush()
        sys.stdin.readline()
        print("\n🚀 Starting calibration sequence in new terminal...")
        
        # Launch in a separate gnome-terminal to get proper stdin
        return [ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'source /home/adeel/barns_ws/install/setup.bash && ' +
                 'ros2 run pickn_place calibration_move --ros-args -p initialized:=true --log-level info; ' +
                 'exec bash'],
            output='screen'
        )]
    except EOFError:
        print("\n⚠️ No input available (non-interactive environment)")
        return []

def on_calibration_move_exit(event, context):
    """
    Handle calibration_move node exit.
    If it exits with error, prompt user to retry.
    """
    if hasattr(event, 'returncode') and event.returncode != 0:
        print("\n" + "="*60)
        print("⚠️  CALIBRATION MOVE FAILED")
        print("="*60)
        print("Common issues:")
        print("  • Calibration markers not detected")
        print("  • Robot drag mode not working")
        print("  • Stability check too strict")
        print("\n💡 Fix the issue and press ENTER to retry...")
        print("="*60 + "\n")
        return [OpaqueFunction(function=prompt_calibration_move)]
    elif hasattr(event, 'returncode') and event.returncode == 0:
        print("\n" + "="*60)
        print("✅ CALIBRATION SEQUENCE COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("📄 Calibration data saved to:")
        print("   ~/barns_ws/src/pickn_place/share/axab_calibration.yaml")
        print("="*60 + "\n")
    return []

def generate_launch_description():
    # Get launch file paths
    try:
        orbbec_camera_launch_file = os.path.join(
            get_package_share_directory('orbbec_camera'),
            'launch',
            'gemini_330_series.launch.py'
        )
    except Exception as e:
        print(f"❌ Error: Could not find orbbec_camera package: {e}")
        sys.exit(1)
    
    # Reset state tracker
    NodeStateTracker.reset()
    
    # Orbbec Camera
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_launch_file),
        launch_arguments={
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',
            'log_level': 'warn'
        }.items()
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Informational messages
    ld.add_action(LogInfo(msg=""))
    ld.add_action(LogInfo(msg="="*70))
    ld.add_action(LogInfo(msg="🚀 AxxB Hand-Eye Calibration System"))
    ld.add_action(LogInfo(msg="="*70))
    ld.add_action(LogInfo(msg=""))
    ld.add_action(LogInfo(msg="⚠️  PREREQUISITE: Make sure robot stack is already running!"))
    ld.add_action(LogInfo(msg=""))
    ld.add_action(LogInfo(msg="   Required nodes:"))
    ld.add_action(LogInfo(msg="   ✓ Dobot bringup"))
    ld.add_action(LogInfo(msg="   ✓ MoveIt"))
    ld.add_action(LogInfo(msg="   ✓ Servo action server"))
    ld.add_action(LogInfo(msg="   ✓ Obstacle generator"))
    ld.add_action(LogInfo(msg=""))
    ld.add_action(LogInfo(msg="   If not running, use your startup shell script first!"))
    ld.add_action(LogInfo(msg="="*70))
    ld.add_action(LogInfo(msg=""))
    
    # Launch camera
    ld.add_action(LogInfo(msg="📷 Launching Orbbec Camera..."))
    ld.add_action(orbbec_camera_launch)
    
    # Wait for camera, then launch perception
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="👁️ Launching calibration perception..."),
            OpaqueFunction(function=wait_for_camera)
        ]
    ))
    
    # Wait for perception, then launch calibration node
    ld.add_action(TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="🔍 Launching calibration node..."),
            OpaqueFunction(function=wait_for_perception)
        ]
    ))
    
    # After calibration node is ready, prompt user to start move sequence
    ld.add_action(TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="🎯 Ready for calibration..."),
            OpaqueFunction(function=prompt_calibration_move)
        ]
    ))
    
    # Register event handler for calibration_move exit (for retry logic)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=None,
            on_exit=on_calibration_move_exit
        )
    ))
    
    return ld
