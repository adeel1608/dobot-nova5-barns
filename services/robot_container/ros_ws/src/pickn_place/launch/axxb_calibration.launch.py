#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def create_calibration_move_node():
    return Node(
        package='pickn_place',
        executable='calibration_move',
        name='calibration_move_node',
        output='screen',
        parameters=[{'initialized': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True  # Ensures interactive input works.
    )

def prompt_calibration_move(context, *args, **kwargs):
    print("************************************************")
    print("                * INSTRUCTIONS *")
    print("************************************************")
    print("Make sure all 4 calibration markers are visible.")
    print("Use MoveIt to align the robot camera.")
    print("Calibration board distance should be approx ~85cm.")
    print("ROI is green if distance is valid.")
    input("Press Enter to start: ")
    return [create_calibration_move_node()]

def on_calibration_move_exit(event, context):
    # If the calibration_move node exits with a nonzero code, schedule the prompt.
    if event.action.name == "calibration_move_node" and event.returncode != 0:
        return [OpaqueFunction(function=prompt_calibration_move)]
    return []

def generate_launch_description():
    # Set global environment variable to suppress logs (FATAL by default).
    set_env = SetEnvironmentVariable(name='RCUTILS_LOG_SEVERITY_THRESHOLD', value='FATAL')

    # Include the orbbec_camera launch file with enable_point_cloud true.
    orbbec_camera_launch_file = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'gemini_330_series.launch.py'
    )
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_launch_file),
        launch_arguments={'enable_point_cloud': 'true'}.items()
    )

    # Launch calibration_perception node (visualization) with logs suppressed.
    calibration_perception_node = Node(
        package='pickn_place',
        executable='calibration_perception',
        name='calibration_perception_node',
        output='screen',
        parameters=[{'initialized': True}],
        arguments=['--ros-args', '--log-level', 'fatal']
    )

    # Launch calibration_node (visualization) with logs suppressed.
    calibration_node = Node(
        package='pickn_place',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[{'initialized': True}],
        arguments=['--ros-args', '--log-level', 'fatal']
    )

    ld = LaunchDescription()
    ld.add_action(set_env)

    # Launch orbbec_camera node after 1 second.
    ld.add_action(TimerAction(
        period=5.0,
        actions=[orbbec_camera_launch]
    ))
    # Launch calibration_perception node after 2 seconds.
    ld.add_action(TimerAction(
        period=8.0,
        actions=[calibration_perception_node]
    ))
    # Launch calibration_node after 3 seconds.
    ld.add_action(TimerAction(
        period=10.0,
        actions=[calibration_node]
    ))
    # After a 4-second delay, prompt the user to launch calibration_move.
    ld.add_action(TimerAction(
        period=12.0,
        actions=[OpaqueFunction(function=prompt_calibration_move)]
    ))
    # Register an event handler that monitors calibration_move's exit.
    ld.add_action(RegisterEventHandler(
        OnProcessExit(on_exit=on_calibration_move_exit)
    ))
    return ld
