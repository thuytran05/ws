"""Launch full perception pipeline (Source -> Detector/Tracker).

Supports source_mode:
- 'synthetic_fixture'
- 'video_replay'
- 'live_astra'
- 'live_webcam'

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
            description="Source mode: 'video_replay', 'synthetic_fixture', 'live_astra', or 'live_webcam'"
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
            'camera_index',
            default_value='0',
            description='OpenCV webcam device index (/dev/videoN) for live_webcam mode'
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='Requested webcam frame width in pixels'
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='Requested webcam frame height in pixels'
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
        DeclareLaunchArgument(
            'image_qos_reliability',
            default_value='best_effort',
            description="Subscriber QoS for RGB/Depth: 'best_effort' or 'reliable'"
        ),
        DeclareLaunchArgument(
            'image_qos_depth',
            default_value='5',
            description='Subscriber QoS KEEP_LAST queue depth for RGB/Depth'
        ),
        DeclareLaunchArgument(
            'prioritize_latest_frame',
            default_value='true',
            description='Drop old queued frames when YOLO is busy'
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
                'camera_index': LaunchConfiguration('camera_index'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
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
                'session_state_topic': '/tracking/session_state',
                'debug_lock_state_topic': '/tracking/debug_lock_state',
                'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
                'image_qos_depth': LaunchConfiguration('image_qos_depth'),
                'info_qos_reliability': LaunchConfiguration('image_qos_reliability'),
                'info_qos_depth': LaunchConfiguration('image_qos_depth'),
                'prioritize_latest_frame': LaunchConfiguration('prioritize_latest_frame'),
            }],
        ),
    ])
