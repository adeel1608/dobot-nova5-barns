#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare mode argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='operational',
        description="Mode: 'operational' or 'calibration'"
    )
    mode = LaunchConfiguration('mode')

    # 2) Define the common steps
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dobot_bringup_v3'),
                'launch',
                'dobot_bringup_ros2.launch.py'
            )
        )
    )
    move_client = Node(
        package='servo_action',
        executable='action_move_client_reality',
        name='action_move_client_reality',
        output='screen'
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dobot_moveit'),
                'launch',
                'dobot_moveit.launch.py'
            )
        )
    )
    move_server = Node(
        package='servo_action',
        executable='action_move_server_reality',
        name='action_move_server_reality',
        output='screen'
    )

    # 3) Operational-specific steps (5-7)
    oper_cond = IfCondition(PythonExpression(["'", mode, "' == 'operational' "]))
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py'
            )
        ),
        launch_arguments={'enable_point_cloud': 'true'}.items(),
        condition=oper_cond
    )
    pose_gen = Node(
        package='pickn_place',
        executable='pose_generator',
        name='pose_generator',
        output='screen',
        condition=oper_cond
    )
    obstacle = Node(
        package='pickn_place',
        executable='obstacle',
        name='obstacle',
        output='screen',
        condition=oper_cond
    )

    # 4) Calibration-only step
    calib_cond = IfCondition(PythonExpression(["'", mode, "' == 'calibration' "]))
    calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pickn_place'),
                'launch',
                'axxb_calibration.launch.py'
            )
        ),
        condition=calib_cond
    )

    # 5) Assemble with 1.5s spacing between each
    return LaunchDescription([
        mode_arg,
        TimerAction(period=0.0, actions=[bringup]),
        TimerAction(period=1.5, actions=[move_client]),
        TimerAction(period=3.0, actions=[moveit]),
        TimerAction(period=4.5, actions=[move_server]),
        TimerAction(period=6.0, actions=[camera, calibration]),
        TimerAction(period=7.5, actions=[pose_gen]),
        TimerAction(period=9.0, actions=[obstacle]),
    ])

