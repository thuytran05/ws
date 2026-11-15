"""Start person perception and following; velocity output defaults to /cmd_vel."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('person_tracking')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'person_tracking.yaml')),
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Velocity command topic; geometry_msgs/msg/Twist.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'tracking.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'python_executable': LaunchConfiguration('python_executable'),
            }.items(),
        ),
        Node(
            package='person_tracking', executable='person_follower',
            namespace='person_tracking', name='person_follower',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            output='screen',
        ),
    ])
