import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('airport_robot_voice')
    cfg = os.path.join(pkg, 'config')

    default_intents = os.path.join(cfg, 'intents.json')
    default_places = os.path.join(cfg, 'places.json')

    # Launch Configurations
    mode = LaunchConfiguration('mode')                  # keyboard | mic | wav
    wav_file = LaunchConfiguration('wav_file')          # Path to wav file
    engine = LaunchConfiguration('tts_engine')          # log | piper | espeak
    tts_policy = LaunchConfiguration('tts_policy')      # flexible | strict
    use_sim_time = LaunchConfiguration('use_sim_time')  # true | false
    places_file = LaunchConfiguration('places_file')    # Path to places.json
    model = LaunchConfiguration('vosk_model')
    use_nav = LaunchConfiguration('use_nav')            # true | false
    launch_stt = LaunchConfiguration('launch_stt')      # true | false
    enable_escort = LaunchConfiguration('enable_escort')# true | false

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='keyboard',
                              description='Chế độ nhập liệu STT: keyboard, mic, hoặc wav'),
        DeclareLaunchArgument('wav_file', default_value='',
                              description='Đường dẫn file WAV khi dùng mode:=wav'),
        DeclareLaunchArgument('tts_engine', default_value='log',
                              description='Engine phát âm TTS: log (mặc định cho Docker), piper, espeak'),
        DeclareLaunchArgument('tts_policy', default_value='flexible',
                              description='Chính sách TTS: flexible (tiếp tục gửi goal nếu cảnh báo/timeout) hoặc strict'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Sử dụng simulation clock (/clock) cho môi trường Gazebo'),
        DeclareLaunchArgument('places_file', default_value=default_places,
                              description='Đường dẫn file cấu hình địa điểm places.json'),
        DeclareLaunchArgument('vosk_model', default_value='/opt/vosk/vosk-model-small-vn-0.4',
                              description='Đường dẫn thư mục model Vosk tiếng Việt offline'),
        DeclareLaunchArgument('use_nav', default_value='true',
                              description='Có khởi chạy node điều hướng nav_command_node hay không'),
        DeclareLaunchArgument('launch_stt', default_value='true',
                              description='Có khởi chạy stt_node trong launch này hay không (đặt false nếu muốn chạy stt_node riêng cho keyboard)'),
        DeclareLaunchArgument('enable_escort', default_value='false',
                              description='Bật chế độ tích hợp Escort Tracking (Phase 9)'),

        # STT Node (Chỉ chạy khi launch_stt == true)
        Node(
            package='airport_robot_voice',
            executable='stt_node',
            name='stt_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mode': mode,
                'wav_file': wav_file,
                'model_path': model,
                'wake_word': '',
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(launch_stt)
        ),

        # Intent Node (Lõi NLU nhận diện ý định)
        Node(
            package='airport_robot_voice',
            executable='intent_node',
            name='intent_node',
            output='screen',
            parameters=[{
                'intents_file': default_intents,
                'places_file': places_file,
                'use_sim_time': use_sim_time
            }]
        ),

        # TTS Node (Phát tiếng nói hoặc ghi log)
        Node(
            package='airport_robot_voice',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'engine': engine,
                'use_sim_time': use_sim_time
            }]
        ),

        # Nav Command Node (Chỉ chạy khi use_nav == true)
        Node(
            package='airport_robot_voice',
            executable='nav_command_node',
            name='nav_command_node',
            output='screen',
            parameters=[{
                'places_file': places_file,
                'tts_policy': tts_policy,
                'enable_escort': enable_escort,
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(use_nav)
        ),
    ])
