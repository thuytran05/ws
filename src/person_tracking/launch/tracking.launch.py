"""Start person perception and selection without starting the follower."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('person_tracking'), 'config', 'person_tracking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'python_executable', default_value=sys.executable,
            description='Python interpreter with ROS, cv_bridge, torch and ultralytics.'),
        Node(
            package='person_tracking', executable='person_tracker',
            namespace='person_tracking', name='person_tracker',
            prefix=[LaunchConfiguration('python_executable')],
            parameters=[LaunchConfiguration('params_file')], output='screen',
        ),
    ])
