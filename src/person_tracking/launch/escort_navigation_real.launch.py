# Copyright (c) 2026 Airport Robot Escort Project
# Phase 11 Real Robot Hardware Escort Navigation Launch
# Chained Pipeline:
# Nav2 smoother -> /cmd_vel_smoothed -> Velocity Guard -> /tracking/cmd_vel_guarded
#   -> Velocity Watchdog -> /tracking/cmd_vel_watchdog -> Collision Monitor -> /cmd_vel -> Base Firmware

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def setup_escort_navigation(context, *args, **kwargs):
    pkg_tracking = FindPackageShare('person_tracking')
    pkg_linorobot2_nav = FindPackageShare('linorobot2_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    nav2_launch_path = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
    default_test_map_path = PathJoinSubstitution([pkg_linorobot2_nav, 'maps', 'turtlebot3_world.yaml'])
    guard_nav_params_path = PathJoinSubstitution([pkg_tracking, 'config', 'navigation_guard.yaml'])

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    is_sim = use_sim_time_str in ('true', '1')

    map_str = LaunchConfiguration('map').perform(context).strip()
    params_file = LaunchConfiguration('params_file')
    enable_watchdog = LaunchConfiguration('enable_watchdog')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')

    # Map Validation: Strict fail-closed policy
    if is_sim:
        if not map_str:
            map_str = default_test_map_path.perform(context)
        elif not os.path.isfile(map_str):
            raise FileNotFoundError(f"Simulation map file not found: '{map_str}'")
    else:
        # Real robot mode (use_sim_time=false)
        if not map_str:
            raise RuntimeError(
                "BLOCKED: 'map' argument is mandatory in real robot mode (use_sim_time=false). "
                "You must explicitly provide an actual facility map via map:=/path/to/airport_map.yaml. "
                "Simulation map turtlebot3_world is strictly prohibited on real hardware."
            )
        if "turtlebot3_world" in os.path.basename(map_str):
            raise RuntimeError(
                f"BLOCKED: Simulation map '{map_str}' cannot be used on physical robot (use_sim_time=false). "
                "Please provide an actual facility/airport map."
            )
        if not os.path.isfile(map_str):
            raise FileNotFoundError(f"Specified physical map file does not exist: '{map_str}'")

    actions = [
        # 1. Nav2 Stack with Guard Navigation Params Overlay (Collision Monitor input: /tracking/cmd_vel_watchdog)
        # Note on Initial Pose: Nav2 Jazzy bringup_launch.py does not take initial_pose_* launch arguments.
        # Initial pose is configured via params_file (amcl.ros__parameters.initial_pose) or at runtime via /initialpose topic.
        GroupAction(
            actions=[
                SetRemap(src='docking_server:cmd_vel', dst='/cmd_vel_docking'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'map': map_str,
                        'use_sim_time': 'true' if is_sim else 'false',
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
            parameters=[{'use_sim_time': is_sim}]
        ),

        # 3. Escort Velocity Guard Node (Safe envelope: in: /cmd_vel_smoothed, out: /tracking/cmd_vel_guarded)
        Node(
            package='person_tracking',
            executable='velocity_guard',
            name='escort_velocity_guard',
            output='screen',
            parameters=[{
                'use_sim_time': is_sim,
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
                'use_sim_time': is_sim,
                'cmd_vel_in_topic': '/tracking/cmd_vel_guarded',
                'cmd_vel_out_topic': '/tracking/cmd_vel_watchdog',
                'timeout_sec': 0.15,
                'control_rate_hz': 20.0,
                'max_linear_x': max_linear_x,
                'max_angular_z': max_angular_z,
            }],
            condition=IfCondition(enable_watchdog)
        ),
    ]

    return actions


def generate_launch_description():
    pkg_tracking = FindPackageShare('person_tracking')
    guard_nav_params_path = PathJoinSubstitution([pkg_tracking, 'config', 'navigation_guard.yaml'])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true, wall clock if false'
        ),
        DeclareLaunchArgument(
            name='map',
            default_value='',
            description='Full path to map yaml file (Mandatory when use_sim_time:=false; defaults to turtlebot3_world in simulation mode)'
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

        OpaqueFunction(function=setup_escort_navigation),
    ])
