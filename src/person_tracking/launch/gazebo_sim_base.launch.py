# Copyright (c) 2026 Airport Robot Escort Project
# Simulation Base Only Launch (No Nav2, No Guard, No Watchdog)
# Provides: Gazebo world, Robot entity spawn, ROS-GZ parameter bridge (/clock, /cmd_vel, /odom/unfiltered, /scan, /imu),
#           robot_state_publisher (TF), and EKF node (/odom).

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
    pkg_gz_sim = FindPackageShare('ros_gz_sim')

    # Paths
    gz_launch_path = PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
    desc_launch_path = PathJoinSubstitution([pkg_linorobot2_desc, 'launch', 'description.launch.py'])
    ekf_config_path = PathJoinSubstitution([pkg_tracking, 'config', 'ekf.yaml'])

    # Use tracking overlay URDF with clean MecanumDrive (no duplicate/spiking OdometryPublisher)
    urdf_path = PathJoinSubstitution([pkg_tracking, 'urdf', 'robots', 'mecanum.urdf.xacro'])

    # Launch Configurations
    use_sim_time = LaunchConfiguration('sim')
    gui = LaunchConfiguration('gui')
    world_name = LaunchConfiguration('world_name')

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
        # Gazebo spawn coordinates (Gazebo origin is center: cylinder at 0, 0)
        DeclareLaunchArgument(
            name='spawn_x',
            default_value='-0.50',
            description='Gazebo spawn position in X'
        ),
        DeclareLaunchArgument(
            name='spawn_y',
            default_value='-0.50',
            description='Gazebo spawn position in Y'
        ),
        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0',
            description='Gazebo spawn heading'
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
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', '0.0',
                '-Y', LaunchConfiguration('spawn_yaw'),
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
    ])
