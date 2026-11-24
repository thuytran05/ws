# Copyright (c) 2026 Airport Robot Escort Project
# Phase 11 Real Robot Hardware Escort Navigation Launch
# Chained Pipeline:
# Nav2 smoother -> /cmd_vel_smoothed -> Velocity Guard -> /tracking/cmd_vel_guarded
#   -> Velocity Watchdog -> /tracking/cmd_vel_watchdog -> Collision Monitor -> /cmd_vel -> Base Firmware

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_tracking = FindPackageShare('person_tracking')
    pkg_linorobot2_nav = FindPackageShare('linorobot2_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Default paths
    nav2_launch_path = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
    default_test_map_path = PathJoinSubstitution([pkg_linorobot2_nav, 'maps', 'turtlebot3_world.yaml'])
    guard_nav_params_path = PathJoinSubstitution([pkg_tracking, 'config', 'navigation_guard.yaml'])

    # Launch Configurations
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_watchdog = LaunchConfiguration('enable_watchdog')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true, wall clock if false'
        ),
        DeclareLaunchArgument(
            name='map',
            default_value=default_test_map_path,
            description='Full path to map yaml file (Required for physical airport/facility map; turtlebot3_world.yaml is used only for testing/simulation)'
        ),
        DeclareLaunchArgument(
            name='params_file',
            default_value=guard_nav_params_path,
            description='Full path to ROS 2 parameters file with Collision Monitor overlay'
        ),
        DeclareLaunchArgument(
            name='max_linear_x',
            default_value='0.30',
            description='Safe linear speed limit for physical hardware testing (m/s)'
        ),
        DeclareLaunchArgument(
            name='max_angular_z',
            default_value='0.20',
            description='Safe angular speed limit for physical hardware testing (rad/s)'
        ),
        DeclareLaunchArgument(
            name='enable_watchdog',
            default_value='true',
            description='Enable independent velocity watchdog node'
        ),

        # 1. Nav2 Stack with Guard Navigation Params Overlay (Collision Monitor input: /tracking/cmd_vel_watchdog)
        # Note on Initial Pose: Nav2 Jazzy bringup_launch.py does not take initial_pose_* launch arguments.
        # Initial pose is configured via params_file (amcl.ros__parameters.initial_pose) or at runtime via /initialpose topic.
        GroupAction(
            actions=[
                SetRemap(src='docking_server:cmd_vel', dst='/cmd_vel_docking'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'map': map_yaml,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': 'true',
                    }.items()
                ),
            ]
        ),

        # 2. Escort Tracking Manager Node (Session state & gesture lock)
        Node(
            package='person_tracking',
            executable='tracking_manager',
            name='escort_tracking_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 3. Escort Velocity Guard Node (Safe envelope: in: /cmd_vel_smoothed, out: /tracking/cmd_vel_guarded)
        Node(
            package='person_tracking',
            executable='velocity_guard',
            name='escort_velocity_guard',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cmd_vel_in_topic': '/cmd_vel_smoothed',
                'cmd_vel_out_topic': '/tracking/cmd_vel_guarded',
                'session_state_topic': '/tracking/session_state',
                'observation_topic': '/tracking/observation',
                'max_linear_x': max_linear_x,
                'max_angular_z': max_angular_z,
            }]
        ),

        # 4. Escort Velocity Watchdog Node (Independent Active Zeroing: in: /tracking/cmd_vel_guarded, out: /tracking/cmd_vel_watchdog)
        Node(
            package='person_tracking',
            executable='velocity_watchdog',
            name='escort_velocity_watchdog',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cmd_vel_in_topic': '/tracking/cmd_vel_guarded',
                'cmd_vel_out_topic': '/tracking/cmd_vel_watchdog',
                'timeout_sec': 0.15,
                'control_rate_hz': 20.0,
                'max_linear_x': max_linear_x,
                'max_angular_z': max_angular_z,
            }],
            condition=IfCondition(enable_watchdog)
        ),
    ])
