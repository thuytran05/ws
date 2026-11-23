# Copyright (c) 2026 Airport Robot Escort Project
# Phase 8 Gazebo & Nav2 Integration Launch with Velocity Guard & Collision Monitor

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_tracking = FindPackageShare('person_tracking')
    pkg_linorobot2_gazebo = FindPackageShare('linorobot2_gazebo')
    pkg_linorobot2_desc = FindPackageShare('linorobot2_description')
    pkg_linorobot2_nav = FindPackageShare('linorobot2_navigation')
    pkg_linorobot2_base = FindPackageShare('linorobot2_base')
    pkg_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Paths
    gz_launch_path = PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
    desc_launch_path = PathJoinSubstitution([pkg_linorobot2_desc, 'launch', 'description.launch.py'])
    nav2_launch_path = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])

    ekf_config_path = PathJoinSubstitution([pkg_linorobot2_base, 'config', 'ekf.yaml'])
    default_map_path = PathJoinSubstitution([pkg_linorobot2_nav, 'maps', 'turtlebot3_world.yaml'])
    guard_nav_params_path = PathJoinSubstitution([pkg_tracking, 'config', 'navigation_guard.yaml'])

    urdf_path = PathJoinSubstitution([pkg_linorobot2_desc, 'urdf', 'robots', 'mecanum.urdf.xacro'])

    # Launch Configurations
    use_sim_time = LaunchConfiguration('sim')
    gui = LaunchConfiguration('gui')
    world_name = LaunchConfiguration('world_name')
    map_yaml = LaunchConfiguration('map')

    world_path = PathJoinSubstitution([pkg_linorobot2_gazebo, 'worlds', [world_name, '.sdf']])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='true',
            description='Enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='false',
            description='Enable Gazebo GUI (false for headless)'
        ),
        DeclareLaunchArgument(
            name='world_name',
            default_value='turtlebot3_world',
            description='Gazebo world name'
        ),
        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Full path to map file'
        ),
        DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.5',
            description='Initial robot X'
        ),
        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0',
            description='Initial robot Y'
        ),
        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='0.0',
            description='Initial robot Yaw'
        ),

        # 1. Gazebo Sim Server (Headless)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={'gz_args': [' -r -s ', world_path]}.items()
        ),

        # Gazebo Sim Client (optional GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            condition=IfCondition(gui),
            launch_arguments={'gz_args': [' -g']}.items()
        ),

        # 2. Spawn Robot Entity
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'linorobot2',
                '-x', LaunchConfiguration('initial_pose_x'),
                '-y', LaunchConfiguration('initial_pose_y'),
                '-z', '0.0',
                '-Y', LaunchConfiguration('initial_pose_yaw'),
            ]
        ),

        # 3. ROS-Gazebo Parameter Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ]
        ),

        # 4. Robot Description (State Publisher)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_joints': 'false',
                'urdf': urdf_path,
            }.items()
        ),

        # 5. EKF Node for /odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, ekf_config_path],
            remappings=[('odometry/filtered', '/odom')]
        ),

        # 6. Nav2 Stack with Guard Navigation Params Overlay
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': guard_nav_params_path,
                'autostart': 'true',
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items()
        ),

        # 7. Tracking Manager Node
        Node(
            package='person_tracking',
            executable='tracking_manager',
            name='escort_tracking_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 8. Velocity Guard Node
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
            }]
        ),
    ])
