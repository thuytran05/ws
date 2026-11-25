import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('airport_robot_voice')
    cfg = os.path.join(pkg, 'config')

    default_intents = os.path.join(cfg, 'intents.json')
    default_places = os.path.join(cfg, 'places.json')
    default_rviz_config = os.path.join(pkg, 'rviz', 'voice_navigation.rviz')

    # Launch Configurations — STT
    mode = LaunchConfiguration('mode')
    stt_backend = LaunchConfiguration('stt_backend')
    wav_file = LaunchConfiguration('wav_file')
    vosk_model = LaunchConfiguration('vosk_model')
    device = LaunchConfiguration('device')
    sample_rate = LaunchConfiguration('sample_rate')
    wake_word = LaunchConfiguration('wake_word')

    # Launch Configurations — Faster-Whisper & Anti-Echo
    whisper_model = LaunchConfiguration('whisper_model')
    whisper_model_path = LaunchConfiguration('whisper_model_path')
    whisper_cache_dir = LaunchConfiguration('whisper_cache_dir')
    whisper_python = LaunchConfiguration('whisper_python')
    whisper_device = LaunchConfiguration('whisper_device')
    whisper_compute_type = LaunchConfiguration('whisper_compute_type')
    whisper_language = LaunchConfiguration('whisper_language')
    whisper_beam_size = LaunchConfiguration('whisper_beam_size')
    whisper_local_files_only = LaunchConfiguration('whisper_local_files_only')
    silence_duration_sec = LaunchConfiguration('silence_duration_sec')
    vad_rms_threshold = LaunchConfiguration('vad_rms_threshold')
    max_utterance_sec = LaunchConfiguration('max_utterance_sec')
    post_tts_cooldown_sec = LaunchConfiguration('post_tts_cooldown_sec')
    tts_echo_suppression_sec = LaunchConfiguration('tts_echo_suppression_sec')

    # Launch Configurations — TTS & Navigation
    tts_engine = LaunchConfiguration('tts_engine')
    tts_player = LaunchConfiguration('tts_player')
    piper_model = LaunchConfiguration('piper_model')
    tts_policy = LaunchConfiguration('tts_policy')
    use_sim_time = LaunchConfiguration('use_sim_time')
    places_file = LaunchConfiguration('places_file')
    use_nav = LaunchConfiguration('use_nav')
    launch_stt = LaunchConfiguration('launch_stt')
    enable_escort = LaunchConfiguration('enable_escort')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mic',
                              description='Chế độ nhập liệu STT: mic (mặc định), keyboard, hoặc wav'),
        DeclareLaunchArgument('stt_backend', default_value='faster_whisper',
                              description='Backend STT: faster_whisper (mặc định) hoặc vosk (dự phòng)'),
        DeclareLaunchArgument('wav_file', default_value='',
                              description='Đường dẫn file WAV khi dùng mode:=wav'),
        DeclareLaunchArgument('vosk_model', default_value='/opt/vosk/vosk-model-small-vn-0.4',
                              description='Đường dẫn thư mục model Vosk tiếng Việt offline (dự phòng)'),
        DeclareLaunchArgument('device', default_value='9',
                              description='Index thiết bị thu âm sounddevice (9 = ThinkPad hw:1,7; -1 = mặc định hệ thống)'),
        DeclareLaunchArgument('sample_rate', default_value='16000',
                              description='Tần số lấy mẫu thu âm (Hz), mặc định 16000'),
        DeclareLaunchArgument('wake_word', default_value='',
                              description='Từ khóa đánh thức (để trống = luôn nghe)'),

        DeclareLaunchArgument('whisper_model', default_value='small',
                              description='Tên model Faster-Whisper (small mặc định; hỗ trợ base, medium, large-v3...)'),
        DeclareLaunchArgument('whisper_model_path', default_value='',
                              description='Đường dẫn cục bộ tới thư mục model Faster-Whisper (ưu tiên nếu có)'),
        DeclareLaunchArgument('whisper_cache_dir', default_value='',
                              description='Thư mục cache HuggingFace/download_root cho Faster-Whisper'),
        DeclareLaunchArgument('whisper_python', default_value='',
                              description='Đường dẫn Python venv cho worker Faster-Whisper (trống = tự động dò ~/.venvs/airport-whisper-test)'),
        DeclareLaunchArgument('whisper_device', default_value='cpu',
                              description='Thiết bị suy luận Faster-Whisper: cpu hoặc cuda'),
        DeclareLaunchArgument('whisper_compute_type', default_value='int8',
                              description='Kiểu tính toán Faster-Whisper: int8, float16, float32'),
        DeclareLaunchArgument('whisper_language', default_value='vi',
                              description='Mã ngôn ngữ Faster-Whisper'),
        DeclareLaunchArgument('whisper_beam_size', default_value='5',
                              description='Kích thước beam search của Faster-Whisper'),
        DeclareLaunchArgument('whisper_local_files_only', default_value='true',
                              description='Chỉ dùng model đã tải offline (không truy cập mạng)'),
        DeclareLaunchArgument('silence_duration_sec', default_value='0.8',
                              description='Thời gian im lặng để chốt câu nói khi dùng mic Faster-Whisper (giây)'),
        DeclareLaunchArgument('vad_rms_threshold', default_value='0.005',
                              description='Ngưỡng RMS phân biệt tiếng nói và nhiễu nền (mảng float32)'),
        DeclareLaunchArgument('max_utterance_sec', default_value='12.0',
                              description='Độ dài tối đa một câu nói khi thu trực tiếp (giây)'),
        DeclareLaunchArgument('post_tts_cooldown_sec', default_value='0.7',
                              description='Thời gian trễ sau khi robot ngừng nói để xả hết đuôi loa/PulseAudio (giây)'),
        DeclareLaunchArgument('tts_echo_suppression_sec', default_value='3.5',
                              description='Cửa sổ lọc transcript lặp lại chính câu robot vừa phát (giây)'),

        DeclareLaunchArgument('tts_engine', default_value='piper',
                              description='Engine phát âm TTS: piper (mặc định), log (kiểm thử/Gazebo), espeak'),
        DeclareLaunchArgument('tts_player', default_value='paplay',
                              description='Trình phát âm thanh cho Piper: paplay (mặc định, dùng PULSE_SERVER), aplay, ffplay'),
        DeclareLaunchArgument('piper_model', default_value='/opt/piper/vi_VN-vais1000-medium.onnx',
                              description='Đường dẫn model ONNX cho Piper TTS'),
        DeclareLaunchArgument('tts_policy', default_value='flexible',
                              description='Chính sách TTS: flexible (tiếp tục gửi goal nếu cảnh báo/timeout) hoặc strict'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Sử dụng simulation clock (/clock) cho môi trường Gazebo'),
        DeclareLaunchArgument('places_file', default_value=default_places,
                              description='Đường dẫn file cấu hình địa điểm places.json'),
        DeclareLaunchArgument('use_nav', default_value='true',
                              description='Có khởi chạy node điều hướng nav_command_node hay không'),
        DeclareLaunchArgument('launch_stt', default_value='true',
                              description='Có khởi chạy stt_node trong launch này hay không (đặt false nếu chạy riêng cho keyboard)'),
        DeclareLaunchArgument('enable_escort', default_value='false',
                              description='Bật chế độ tích hợp Escort Tracking (Phase 9)'),
        DeclareLaunchArgument('launch_rviz', default_value='false',
                              description='Bật RViz với cấu hình riêng của voice (voice_navigation.rviz)'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Đường dẫn file cấu hình .rviz cho voice navigation'),

        # STT Node (Chỉ chạy khi launch_stt == true)
        Node(
            package='airport_robot_voice',
            executable='stt_node',
            name='stt_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mode': ParameterValue(mode, value_type=str),
                'stt_backend': ParameterValue(stt_backend, value_type=str),
                'wav_file': ParameterValue(wav_file, value_type=str),
                'model_path': ParameterValue(vosk_model, value_type=str),
                'device': ParameterValue(device, value_type=int),
                'sample_rate': ParameterValue(sample_rate, value_type=int),
                'wake_word': ParameterValue(wake_word, value_type=str),
                'whisper_model': ParameterValue(whisper_model, value_type=str),
                'whisper_model_path': ParameterValue(whisper_model_path, value_type=str),
                'whisper_cache_dir': ParameterValue(whisper_cache_dir, value_type=str),
                'whisper_python': ParameterValue(whisper_python, value_type=str),
                'whisper_device': ParameterValue(whisper_device, value_type=str),
                'whisper_compute_type': ParameterValue(whisper_compute_type, value_type=str),
                'whisper_language': ParameterValue(whisper_language, value_type=str),
                'whisper_beam_size': ParameterValue(whisper_beam_size, value_type=int),
                'whisper_local_files_only': ParameterValue(whisper_local_files_only, value_type=bool),
                'silence_duration_sec': ParameterValue(silence_duration_sec, value_type=float),
                'silence_rms_threshold': ParameterValue(vad_rms_threshold, value_type=float),
                'max_utterance_sec': ParameterValue(max_utterance_sec, value_type=float),
                'post_tts_cooldown_sec': ParameterValue(post_tts_cooldown_sec, value_type=float),
                'tts_echo_suppression_sec': ParameterValue(tts_echo_suppression_sec, value_type=float),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
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
                'intents_file': ParameterValue(default_intents, value_type=str),
                'places_file': ParameterValue(places_file, value_type=str),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }]
        ),

        # TTS Node (Phát tiếng nói hoặc ghi log)
        Node(
            package='airport_robot_voice',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'engine': ParameterValue(tts_engine, value_type=str),
                'player': ParameterValue(tts_player, value_type=str),
                'piper_model': ParameterValue(piper_model, value_type=str),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }]
        ),

        # Nav Command Node (Chỉ chạy khi use_nav == true)
        Node(
            package='airport_robot_voice',
            executable='nav_command_node',
            name='nav_command_node',
            output='screen',
            parameters=[{
                'places_file': ParameterValue(places_file, value_type=str),
                'tts_policy': ParameterValue(tts_policy, value_type=str),
                'enable_escort': ParameterValue(enable_escort, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=IfCondition(use_nav)
        ),

        # RViz2 riêng của Voice (Chỉ chạy khi launch_rviz == true)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(launch_rviz)
        ),
    ])
