"""Launch Astra S RGB-D Perception + Tracking Manager for stationary bench & debug testing.

Assumes Astra S driver is launched separately (e.g. astra.launch.xml with
depth_registration:=true color_depth_synchronization:=true).

Configured with Astra S benchmarked QoS:
- RELIABLE, KEEP_LAST, VOLATILE, depth=5 for RGB, Depth, and CameraInfo.
- prioritize_latest_frame:=true (drops older unstarted frames if YOLO is slow).
- readiness_min_frames:=5 (waits for 5 stable synchronized frames before camera_ready=True).

STRICTLY DOES NOT start Nav2, Velocity Guard, Watchdog, Voice, or Robot Base.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = '/home/ws/yolo26n-pose.pt'

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=default_model, description='Path to YOLO Pose checkpoint'),
        DeclareLaunchArgument('device', default_value='cuda:0', description="Compute device: 'cuda:0' or 'cpu'"),
        DeclareLaunchArgument('publish_debug_image', default_value='true', description='Publish /tracking/debug_image'),
        DeclareLaunchArgument('view_debug', default_value='false', description='Launch rqt_image_view if true'),
        DeclareLaunchArgument('image_qos_reliability', default_value='reliable', description="RGB/Depth QoS reliability: 'reliable' or 'best_effort'"),
        DeclareLaunchArgument('image_qos_depth', default_value='5', description='RGB/Depth QoS KEEP_LAST queue depth'),
        DeclareLaunchArgument('info_qos_reliability', default_value='reliable', description="CameraInfo QoS reliability: 'reliable' or 'best_effort'"),
        DeclareLaunchArgument('info_qos_depth', default_value='5', description='CameraInfo QoS KEEP_LAST queue depth'),
        DeclareLaunchArgument('prioritize_latest_frame', default_value='true', description='Drop old queued frames when YOLO is busy'),
        DeclareLaunchArgument('readiness_min_frames', default_value='5', description='Minimum consecutive valid frames before camera_ready=True'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('info_topic', default_value='/camera/color/camera_info'),

        # 1. Person Detector & Tracker (Astra S RGB-D subscriber)
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
                'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
                'image_qos_depth': LaunchConfiguration('image_qos_depth'),
                'info_qos_reliability': LaunchConfiguration('info_qos_reliability'),
                'info_qos_depth': LaunchConfiguration('info_qos_depth'),
                'prioritize_latest_frame': LaunchConfiguration('prioritize_latest_frame'),
                'readiness_min_frames': LaunchConfiguration('readiness_min_frames'),
                'require_hardware_alignment_proof': True,
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'info_topic': LaunchConfiguration('info_topic'),
                'status_topic': '/tracking/source_status',
                'observation_topic': '/tracking/observation',
                'debug_image_topic': '/tracking/debug_image',
                'session_state_topic': '/tracking/session_state',
                'debug_lock_state_topic': '/tracking/debug_lock_state',
            }],
        ),
        # 2. Tracking Manager (Single Lock Authority)
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
        # 3. Optional Debug Image Viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='debug_image_viewer',
            arguments=['/tracking/debug_image'],
            condition=IfCondition(LaunchConfiguration('view_debug')),
            output='screen',
        ),
    ])
