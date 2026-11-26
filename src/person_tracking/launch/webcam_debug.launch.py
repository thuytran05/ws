"""Integrated Webcam + Tracking Manager Debug Launch.

Starts ONLY:
1. perception_source (live_webcam, RGB_ONLY)
2. person_detector (YOLO Pose + ByteTrack + DebugVisualizer)
3. tracking_manager (PassengerLockManager FSM & single lock authority)
4. Optional rqt_image_view for /tracking/debug_image (when view_debug:=true)

STRICTLY DOES NOT start Nav2, Velocity Guard, Velocity Watchdog, Voice, or Robot Base.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = '/home/ws/yolo26n-pose.pt'

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='OpenCV webcam device index (/dev/videoN)'
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
            default_value='30.0',
            description='Webcam capture & publish frequency in Hz'
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
            description='Whether to publish /tracking/debug_image'
        ),
        DeclareLaunchArgument(
            'view_debug',
            default_value='false',
            description='If true, also launch rqt_image_view on /tracking/debug_image'
        ),
        # 1. Live Webcam Source Adapter
        Node(
            package='person_tracking',
            executable='perception_source',
            name='perception_source_node',
            namespace='person_tracking',
            output='screen',
            parameters=[{
                'source_mode': 'live_webcam',
                'camera_index': LaunchConfiguration('camera_index'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('fps'),
                'retime_stamps': True,
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'info_topic': '/camera/color/camera_info',
                'status_topic': '/tracking/source_status',
            }],
        ),
        # 2. Person Detector & Debug Visualizer
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
                'image_qos_reliability': 'reliable',
                'image_qos_depth': 5,
                'info_qos_reliability': 'reliable',
                'info_qos_depth': 5,
                'prioritize_latest_frame': True,
                'readiness_min_frames': 3,
            }],
        ),
        # 3. Tracking Manager (Single Lock Authority)
        Node(
            package='person_tracking',
            executable='tracking_manager',
            name='escort_tracking_manager',
            namespace='person_tracking',
            output='screen',
            parameters=[{
                'observation_topic': '/tracking/observation',
                'session_state_topic': '/tracking/session_state',
                'debug_lock_state_topic': '/tracking/debug_lock_state',
                'safety_request_topic': '/tracking/safety_request',
            }],
        ),
        # 4. Optional Debug Image Viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='debug_image_viewer',
            arguments=['/tracking/debug_image'],
            condition=IfCondition(LaunchConfiguration('view_debug')),
            output='screen',
        ),
    ])
