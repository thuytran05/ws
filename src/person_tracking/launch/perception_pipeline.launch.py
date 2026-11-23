"""Launch full perception pipeline (Source -> Detector/Tracker) for Phase 3.

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
    default_model = '/home/ws/yolo26n-pose.pt'

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
            'model_path',
            default_value=default_model,
            description='Path to YOLO Pose model checkpoint'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description="Compute device: 'cuda:0' or 'cpu'"
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Whether to publish debug visualization image'
        ),
        # 1. Perception Source Node
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
                'loop': True,
                'retime_stamps': True,
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'info_topic': '/camera/color/camera_info',
                'status_topic': '/tracking/source_status',
            }],
        ),
        # 2. Multi-person Detector & Tracker Node
        Node(
            package='person_tracking',
            executable='person_detector',
            name='person_detector_node',
            namespace='person_tracking',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'info_topic': '/camera/color/camera_info',
                'status_topic': '/tracking/source_status',
                'observation_topic': '/tracking/observation',
                'debug_image_topic': '/tracking/debug_image',
            }],
        ),
    ])
