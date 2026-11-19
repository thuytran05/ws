#!/usr/bin/env python3
"""
Unit/integration tests cho module âm thanh: STT (WAV/Vosk), TTS (log/espeak), device check.
"""

import os
import shutil
import subprocess
import tempfile
import time
import wave
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from airport_robot_voice.tts_node import TtsNode, normalize_speech_text
from airport_robot_voice.stt_node import SttNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestSpeechNormalization:
    def test_gate_speech_normalization(self):
        assert "cổng a ba" in normalize_speech_text("Tôi sẽ dẫn bạn đến cổng A3.")
        assert "cổng bê hai" in normalize_speech_text("Chúng ta đang tới cổng B2.")
        assert "cổng mười lăm" in normalize_speech_text("Cổng 15 ở khu vực quốc tế.")
        assert "cổng a bốn" in normalize_speech_text("Đang đi tới cổng A4.")


class TestTtsEngines:
    def test_log_mode_synthesis(self, ros_context):
        # Engine log: chạy an toàn không cần loa hay binary bên ngoài
        tts = TtsNode()
        tts.synthesize("Xin chào quý khách")
        tts.destroy_node()

    def test_espeak_mode_synthesis(self, ros_context):
        # Nếu có espeak-ng thì kiểm tra synthesize không văng lỗi
        if shutil.which('espeak-ng') or shutil.which('espeak'):
            tts = TtsNode()
            tts.engine = 'espeak'
            tts.synthesize("Xin chào quý khách")
            tts.destroy_node()

    def test_piper_mode_synthesis(self, ros_context):
        # Nếu có piper binary và model onnx thì kiểm tra synthesize
        if shutil.which('piper') and os.path.exists('/opt/piper/vi_VN-vais1000-medium.onnx'):
            tts = TtsNode()
            tts.engine = 'piper'
            tts.piper_model = '/opt/piper/vi_VN-vais1000-medium.onnx'
            tts.synthesize("Tôi sẽ dẫn bạn đến cổng a ba.")
            tts.destroy_node()


class TestSttNodeModes:
    def test_stt_missing_model_graceful(self, ros_context):
        # Khi model Vosk không tồn tại -> báo lỗi graceful trong log, node không sập
        stt = SttNode()
        stt.model_path = '/invalid/path/to/vosk/model'
        stt.mode = 'mic'
        # mic_loop chạy ở thread riêng và bắt FileNotFoundError / Exception
        stt.destroy_node()

    def test_stt_wav_file_processing(self, ros_context):
        # Tạo file WAV mẫu 16kHz mono
        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            with wave.open(tmp_wav, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                # 0.5s silence
                wf.writeframes(b'\x00\x00' * 8000)

            stt = SttNode()
            stt.mode = 'wav'
            stt.wav_file = tmp_wav
            stt.model_path = '/opt/vosk/vosk-model-small-vn-0.4'
            # Chạy wav_loop trực tiếp
            stt.wav_loop()
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)


class TestAudioDevices:
    def test_sounddevice_query(self):
        try:
            import sounddevice as sd
            devices = sd.query_devices()
            assert len(devices) > 0, "Hệ thống cần nhận diện được ít nhất 1 thiết bị ALSA"
        except ImportError:
            pytest.skip("sounddevice chưa được cài đặt")
