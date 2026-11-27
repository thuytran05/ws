import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_voice = get_package_share_directory('airport_robot_voice')
    pkg_tracking = get_package_share_directory('person_tracking')

    places_sim = os.path.join(pkg_voice, 'config', 'places_sim.json')
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    tts_engine = LaunchConfiguration('tts_engine')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Enable Gazebo GUI'),
        DeclareLaunchArgument('launch_rviz', default_value='false', description='Enable Voice RViz2'),
        DeclareLaunchArgument('tts_engine', default_value='log', description='TTS engine (log for headless demo)'),

        # 1. Isolated Gazebo Base (world, robot spawn, ros_gz_bridge, EKF /odom)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tracking, 'launch', 'gazebo_sim_base.launch.py')
            ),
            launch_arguments={
                'sim': 'true',
                'gui': gui,
            }.items(),
        ),

        # 2. Escort Navigation + Guard -> Watchdog -> Collision Monitor -> /cmd_vel
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tracking, 'launch', 'escort_navigation_real.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),

        # 3. Real Voice System (intent_node + tts_node + nav_command_node with escort enabled)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_voice, 'launch', 'voice_system.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'launch_stt': 'false',
                'tts_engine': tts_engine,
                'enable_escort': 'true',
                'places_file': places_sim,
                'launch_rviz': launch_rviz,
            }.items(),
        ),
    ])
