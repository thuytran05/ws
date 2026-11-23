"""Launch perception source node for offline video replay or synthetic fixtures.

DOES NOT start robot base controller, Nav2, or velocity follower.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('person_tracking')
    default_video = os.path.join(pkg_share, 'fixtures', 'synthetic_video.mp4')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source_mode',
            default_value='synthetic_fixture',
            description="Source mode: 'video_replay', 'synthetic_fixture', or 'live_astra'"
        ),
        DeclareLaunchArgument(
            'video_path',
            default_value=default_video,
            description='Path to local MP4/AVI video file for video_replay mode'
        ),
        DeclareLaunchArgument(
            'synthetic_case',
            default_value='person_valid_rgbd',
            description="Synthetic case: 'person_valid_rgbd', 'empty_scene', 'corrupted_depth'"
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='15.0',
            description='Target publishing frequency in Hz'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Whether to loop the video/synthetic stream'
        ),
        DeclareLaunchArgument(
            'retime_stamps',
            default_value='true',
            description='Whether to stamp frames with current ROS time'
        ),
        Node(
            package='person_tracking',
            executable='perception_source',
            name='perception_source_node',
            namespace='person_tracking',
            output='screen',
            parameters=[{
                'source_mode': LaunchConfiguration('source_mode'),
                'video_path': LaunchConfiguration('video_path'),
                'synthetic_case': LaunchConfiguration('synthetic_case'),
                'fps': LaunchConfiguration('fps'),
                'loop': LaunchConfiguration('loop'),
                'retime_stamps': LaunchConfiguration('retime_stamps'),
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'info_topic': '/camera/color/camera_info',
                'status_topic': '/tracking/source_status',
            }],
        ),
    ])
