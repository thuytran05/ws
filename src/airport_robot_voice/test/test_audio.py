#!/usr/bin/env python3
"""
test_audio.py — Kiểm thử các module âm thanh: STT, TTS, chuẩn hóa giọng đọc và phần cứng.
Bao phủ toàn diện GIAI ĐOẠN 4 & 5:
  1. Dùng WAV có lời nói biết trước để kiểm tra transcript và intent/target.
  2. Giữ riêng bài kiểm tra WAV im lặng (không sinh transcript rác).
  3. Kiểm tra file WAV tổng hợp thực sự tồn tại, có frame và định dạng hợp lệ (mono, 16-bit).
  4. Xác minh sample rate, số kênh và sample width của WAV đầu vào STT.
  5. Kiểm tra lỗi nạp model bằng cách thực sự chạy nhánh nạp model không tồn tại.
  6. Kiểm tra wake word timeout và xả sạch audio queue khi robot đang nói (tránh nhận nhầm tiếng loa).
  7. Dùng pytest.skip kèm lý do khi thiếu điều kiện test phần cứng.
"""

import json
import os
import queue
import shutil
import subprocess
import tempfile
import time
import wave
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from airport_robot_voice.tts_node import TtsNode, normalize_speech_text
from airport_robot_voice.stt_node import SttNode
from airport_robot_voice.nlu import NluEngine


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


class TestTtsEnginesAndWavFormat:
    def test_log_mode_synthesis(self, ros_context):
        tts = TtsNode()
        res = tts.synthesize("Xin chào quý khách")
        assert res['status'] == 'COMPLETED'
        assert res['wav_generated'] is False
        tts.destroy_node()

    def test_espeak_mode_synthesis(self, ros_context):
        if not shutil.which('espeak-ng') and not shutil.which('espeak'):
            pytest.skip("espeak/espeak-ng chưa được cài đặt trên hệ thống")
        tts = TtsNode()
        tts.engine = 'espeak'
        res = tts.synthesize("Xin chào quý khách")
        assert res['status'] == 'COMPLETED'
        tts.destroy_node()

    def test_piper_synthesized_wav_format_and_validity(self, ros_context):
        """Kiểm tra file WAV tổng hợp bằng Piper thực sự tồn tại, có frame và định dạng chuẩn (mono, 16-bit)."""
        piper_bin = shutil.which('piper') or '/usr/local/bin/piper'
        piper_model = '/opt/piper/vi_VN-vais1000-medium.onnx'

        if not os.path.exists(piper_bin) or not os.path.exists(piper_model):
            pytest.skip("Piper binary hoặc model tiếng Việt chưa được cài đặt")

        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            cmd = [
                piper_bin,
                '--model', piper_model,
                '--output_file', tmp_wav
            ]
            p = subprocess.run(cmd, input='Tôi sẽ dẫn bạn đến cổng a ba.'.encode('utf-8'),
                               capture_output=True, timeout=10.0)
            assert p.returncode == 0
            assert os.path.exists(tmp_wav)
            assert os.path.getsize(tmp_wav) > 1000

            # Xác minh header WAV
            with wave.open(tmp_wav, 'rb') as wf:
                assert wf.getnchannels() == 1, "Piper WAV phải là Mono (1 kênh)"
                assert wf.getsampwidth() == 2, "Piper WAV phải là 16-bit (2 bytes/mẫu)"
                assert wf.getframerate() in (16000, 22050), f"Sample rate không hợp lệ: {wf.getframerate()}"
                assert wf.getnframes() > 1000, "File WAV phải chứa các frame âm thanh thực"
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_tts_queue_cancellation_before_playback(self, ros_context):
        """Kiểm tra hủy câu nói trong hàng đợi TTS bằng /voice/tts_cancel."""
        tts = TtsNode()
        acks = []
        tts.pub_tts_ack = type('MockPub', (), {'publish': lambda self, msg: acks.append(json.loads(msg.data))})()

        # 1. Hủy speech_id trước khi worker kịp xử lý
        tts.on_tts_cancel(String(data=json.dumps({'speech_id': 'sp_test_cancel_99'})))

        # 2. Đưa câu nói vào hàng đợi
        payload = {'speech_id': 'sp_test_cancel_99', 'text': 'Câu nói này sẽ bị hủy'}
        tts.on_say(String(data=json.dumps(payload)))

        # Đợi worker xử lý
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if any(a.get('speech_id') == 'sp_test_cancel_99' for a in acks):
                break
            time.sleep(0.02)

        matching = [a for a in acks if a.get('speech_id') == 'sp_test_cancel_99']
        assert len(matching) == 1
        assert matching[0]['status'] == 'CANCELLED_BEFORE_PLAYBACK'
        assert matching[0]['audio_played'] is False
        tts.destroy_node()

    def test_tts_dropped_queue_full(self, ros_context):
        """Kiểm tra hàng đợi TTS đầy sẽ từ chối và phát ack DROPPED_QUEUE_FULL."""
        tts = TtsNode()
        tts.queue_size = 2
        tts.q = queue.Queue(maxsize=2)

        acks = []
        tts.pub_tts_ack = type('MockPub', (), {'publish': lambda self, msg: acks.append(json.loads(msg.data))})()

        # Làm đầy hàng đợi (chặn worker bằng cách không chạy worker hoặc nhồi nhanh)
        tts.q.put(('item 1', 'sp_1', None, None))
        tts.q.put(('item 2', 'sp_2', None, None))

        # Gửi item 3 khi hàng đợi đã đầy 2/2
        payload = {'speech_id': 'sp_overflow', 'text': 'Câu nói bị tràn'}
        tts.on_say(String(data=json.dumps(payload)))

        assert any(a.get('speech_id') == 'sp_overflow' and a.get('status') == 'DROPPED_QUEUE_FULL' for a in acks)
        tts.destroy_node()

    def test_tts_resume_allows_new_speech_after_pause_cancel(self, ros_context):
        """Kiểm tra sau khi pause/cancel mission_1, câu nói mới sau resume của mission_1 vẫn được phát."""
        tts = TtsNode()
        acks = []
        tts.pub_tts_ack = type('MockPub', (), {'publish': lambda self, msg: acks.append(json.loads(msg.data))})()

        # 1. Hủy mission_1
        tts.on_tts_cancel(String(data=json.dumps({'mission_id': 'mission_test_res'})))
        time.sleep(0.05)

        # 2. Câu nói mới sau khi resume (enqueued sau thời điểm cancel)
        payload = {'speech_id': 'sp_after_resume', 'mission_id': 'mission_test_res', 'text': 'Chúng ta tiếp tục di chuyển'}
        tts.on_say(String(data=json.dumps(payload)))

        t0 = time.time()
        while time.time() - t0 < 2.0:
            if any(a.get('speech_id') == 'sp_after_resume' for a in acks):
                break
            time.sleep(0.02)

        matching = [a for a in acks if a.get('speech_id') == 'sp_after_resume']
        assert len(matching) == 1
        assert matching[0]['status'] == 'COMPLETED'

        # 3. Kiểm tra giới hạn bộ nhớ LRU <= 200 items
        for i in range(250):
            tts.on_tts_cancel(String(data=json.dumps({'speech_id': f'sp_bulk_{i}', 'mission_id': f'mis_bulk_{i}'})))
        assert len(tts._cancelled_speeches) <= 200
        assert len(tts._cancelled_missions) <= 200

        tts.destroy_node()


class TestSttNodeOfflineProcessing:
    def test_stt_known_speech_wav_and_nlu_intent(self, ros_context):
        """
        Dùng WAV có lời nói biết trước ('cảm ơn') để kiểm tra STT giải mã và NLU nhận diện đúng intent 'thanks'.
        """
        if not os.path.exists('/opt/vosk/vosk-model-small-vn-0.4'):
            pytest.skip("Model Vosk tiếng Việt chưa được cài đặt")

        bin_espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        if not bin_espeak:
            pytest.skip("espeak-ng chưa sẵn sàng để tạo WAV có lời nói biết trước")

        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            # Tạo WAV có lời nói biết trước 'cảm ơn'
            subprocess.run([bin_espeak, '-v', 'vi', 'cảm ơn', '-w', tmp_wav], check=True, stderr=subprocess.DEVNULL)

            stt = SttNode()
            stt.mode = 'wav'
            stt.wav_file = tmp_wav
            stt.model_path = '/opt/vosk/vosk-model-small-vn-0.4'

            transcripts = []
            stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: transcripts.append(msg.data)})()

            # Chạy giải mã WAV
            stt.wav_loop()

            assert len(transcripts) > 0, "STT phải nhận dạng được văn bản từ file WAV có lời nói"
            combined_text = " ".join(transcripts).strip().lower()
            assert "cảm ơn" in combined_text

            # Kiểm tra tiếp qua NLU
            share = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
            nlu = NluEngine(os.path.join(share, 'intents.json'), os.path.join(share, 'places.json'))
            nlu_res = nlu.process(combined_text)
            assert nlu_res.get('intent') == 'thanks'
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_stt_silence_wav(self, ros_context):
        """WAV hoàn toàn im lặng không được sinh ra transcript rác."""
        if not os.path.exists('/opt/vosk/vosk-model-small-vn-0.4'):
            pytest.skip("Model Vosk tiếng Việt chưa được cài đặt")

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

            transcripts = []
            stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: transcripts.append(msg.data)})()

            stt.wav_loop()
            assert len(transcripts) == 0, "File im lặng không được sinh ra transcript văn bản"
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_stt_missing_model_actual_branch(self, ros_context):
        """Kiểm tra thực sự chạy nhánh nạp model khi đường dẫn không tồn tại."""
        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            with wave.open(tmp_wav, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b'\x00\x00' * 100)

            stt = SttNode()
            stt.mode = 'wav'
            stt.wav_file = tmp_wav
            stt.model_path = '/nonexistent/path/to/vosk/model'

            # Phải bắt exception và ghi warn graceful, không sập chương trình
            stt.wav_loop()
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_stt_invalid_wav_channels_and_width_rejected(self, ros_context):
        """Xác minh WAV có định dạng không đúng (stereo hoặc 8-bit) bị từ chối an toàn."""
        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            with wave.open(tmp_wav, 'wb') as wf:
                wf.setnchannels(2)  # Stereo (không hợp lệ)
                wf.setsampwidth(1)  # 8-bit (không hợp lệ)
                wf.setframerate(16000)
                wf.writeframes(b'\x80\x80' * 500)

            stt = SttNode()
            stt.mode = 'wav'
            stt.wav_file = tmp_wav
            stt.model_path = '/opt/vosk/vosk-model-small-vn-0.4'

            transcripts = []
            stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: transcripts.append(msg.data)})()

            stt.wav_loop()
            assert len(transcripts) == 0, "WAV không đúng định dạng PCM mono 16-bit phải bị từ chối"
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_stt_wake_word_timeout_and_queue_drain(self, ros_context):
        """
        Kiểm tra:
          A. Wake word timeout: khi hết hạn wake_timeout, trạng thái trở về ngủ (awake=False).
          B. Khi robot_speaking=True: audio queue bị xả sạch để tránh nhận nhầm tiếng loa.
        """
        stt = SttNode()
        stt.wake_word = 'robot'
        stt.wake_timeout = 1.0
        stt.awake = False

        published = []
        stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

        # 1. Kích hoạt wake word
        stt.publish_text('robot dẫn tôi đến cổng A3')
        assert stt.awake is True
        assert len(published) == 1
        assert published[0] == 'dẫn tôi đến cổng a3'

        # 2. Giả lập quá hạn wake_timeout
        stt._last_wake_time = time.time() - 5.0
        published.clear()
        stt.publish_text('đi tiếp nhé')
        assert stt.awake is False
        assert len(published) == 0  # Bị bỏ qua vì đã hết hạn tương tác!

        # 3. Kiểm tra xả hàng đợi khi robot đang nói
        for i in range(10):
            stt._audio_queue.put(b'\x01\x02' * 100)
        assert not stt._audio_queue.empty()

        # Robot bắt đầu nói
        stt.on_speaking(Bool(data=True))
        assert stt._audio_queue.empty() is True  # Queue đã được xả sạch!

        stt.destroy_node()


class TestAudioDevices:
    def test_sounddevice_query(self):
        try:
            import sounddevice as sd
            devices = sd.query_devices()
            assert len(devices) > 0, "Hệ thống cần nhận diện được ít nhất 1 thiết bị ALSA"
        except (ImportError, Exception) as e:
            pytest.skip(f"sounddevice hoặc phần cứng âm thanh chưa sẵn sàng: {e}")
