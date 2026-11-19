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

    intents = os.path.join(cfg, 'intents.json')
    places = os.path.join(cfg, 'places.json')

    mode = LaunchConfiguration('mode')                  # keyboard | mic | wav
    wav_file = LaunchConfiguration('wav_file')          # Path to wav file
    engine = LaunchConfiguration('tts_engine')          # log | piper | espeak | gtts
    model = LaunchConfiguration('vosk_model')
    use_nav = LaunchConfiguration('use_nav')            # true | false

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='keyboard',
                              description='Chế độ nhập liệu STT: keyboard, mic, hoặc wav'),
        DeclareLaunchArgument('wav_file', default_value='',
                              description='Đường dẫn file WAV khi dùng mode:=wav'),
        DeclareLaunchArgument('tts_engine', default_value='log',
                              description='Engine phát âm TTS: log (mặc định cho Docker), piper, espeak'),
        DeclareLaunchArgument('vosk_model', default_value='/opt/vosk/vosk-model-small-vn-0.4',
                              description='Đường dẫn model Vosk tiếng Việt'),
        DeclareLaunchArgument('use_nav', default_value='true',
                              description='Có khởi chạy node điều hướng nav_command_node hay không'),

        # STT Node
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
                'wake_word': ''
            }]
        ),

        # Intent Node (Bộ não NLU)
        Node(
            package='airport_robot_voice',
            executable='intent_node',
            name='intent_node',
            output='screen',
            parameters=[{
                'intents_file': intents,
                'places_file': places
            }]
        ),

        # TTS Node
        Node(
            package='airport_robot_voice',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'engine': engine
            }]
        ),

        # Nav Command Node (chỉ chạy khi use_nav == true)
        Node(
            package='airport_robot_voice',
            executable='nav_command_node',
            name='nav_command_node',
            output='screen',
            parameters=[{
                'places_file': places
            }],
            condition=IfCondition(use_nav)
        ),
    ])
