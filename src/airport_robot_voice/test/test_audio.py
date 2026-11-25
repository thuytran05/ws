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

import struct
import numpy as np
from airport_robot_voice.tts_node import TtsNode, normalize_speech_text, build_player_command
from airport_robot_voice.faster_whisper_worker import (
    DEFAULT_WHISPER_VENV_PYTHON,
    UtteranceSegmenter,
    is_tts_self_echo,
    clean_whisper_segments,
    compute_rms,
    pcm16_bytes_to_float32,
    resample_float32,
)
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
    def test_tts_default_parameters_piper_and_paplay(self, ros_context):
        tts = TtsNode()
        assert tts.engine == "piper"
        assert tts.player == "paplay"
        tts.destroy_node()

    def test_log_mode_synthesis(self, ros_context):
        tts = TtsNode()
        tts.engine = "log"
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
        tts.engine = "log"
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
        tts.engine = "log"
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
        tts.engine = "log"
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
            stt.stt_backend = 'vosk'
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
            stt.stt_backend = 'vosk'
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
            stt.stt_backend = 'vosk'
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
            stt.stt_backend = 'vosk'
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


class TestTtsPlayerCommandAndPaplay:
    def test_build_player_command_arguments(self):
        """aplay giữ '-q', paplay KHÔNG truyền '-q', ffplay dùng tham số không giao diện."""
        assert build_player_command('aplay', '/tmp/a.wav') == ['aplay', '-q', '/tmp/a.wav']
        assert build_player_command('/usr/bin/aplay', '/tmp/a.wav') == ['/usr/bin/aplay', '-q', '/tmp/a.wav']
        assert build_player_command('paplay', '/tmp/a.wav') == ['paplay', '/tmp/a.wav']
        assert build_player_command('/usr/bin/paplay', '/tmp/a.wav') == ['/usr/bin/paplay', '/tmp/a.wav']
        assert '-q' not in build_player_command('paplay', '/tmp/a.wav')
        assert build_player_command('ffplay', '/tmp/a.wav') == [
            'ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', '/tmp/a.wav'
        ]

    def test_tts_piper_invokes_paplay_without_q_flag(self, ros_context, monkeypatch):
        """TtsNode khi engine=piper và player=paplay gọi paplay không kèm '-q' và giữ PULSE_SERVER từ env."""
        tts = TtsNode()
        tts.engine = 'piper'
        tts.player = 'paplay'
        tts.piper_bin = '/usr/local/bin/piper'
        tts.piper_model = '/opt/piper/vi_VN-vais1000-medium.onnx'

        recorded_cmds = []

        def fake_which(name):
            if name in ('piper', '/usr/local/bin/piper', 'paplay'):
                return f'/usr/bin/{os.path.basename(name)}'
            return None

        def fake_run(cmd, input=None, capture_output=False, timeout=None):
            recorded_cmds.append(cmd)
            # Khi piper synthesize tạo WAV hợp lệ >= 44 bytes
            if '--output_file' in cmd:
                out_path = cmd[cmd.index('--output_file') + 1]
                with wave.open(out_path, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(22050)
                    wf.writeframes(b'\x00\x00' * 1000)
            return subprocess.CompletedProcess(cmd, 0, b'', b'')

        monkeypatch.setattr('airport_robot_voice.tts_node.shutil.which', fake_which)
        monkeypatch.setattr('airport_robot_voice.tts_node.os.path.exists', lambda p: True)
        monkeypatch.setattr('airport_robot_voice.tts_node.subprocess.run', fake_run)

        res = tts.synthesize('Xin chào')
        assert res['status'] == 'COMPLETED'
        assert res['wav_generated'] is True
        assert res['audio_played'] is True

        play_cmds = [c for c in recorded_cmds if c and os.path.basename(c[0]) == 'paplay']
        assert len(play_cmds) == 1
        assert len(play_cmds[0]) == 2
        assert play_cmds[0][0] == 'paplay'
        assert play_cmds[0][1].endswith('.wav')
        assert '-q' not in play_cmds[0]

        # Kiểm tra nhánh paplay thất bại (ví dụ chưa kết nối PulseAudio)
        def fake_run_fail(cmd, input=None, capture_output=False, timeout=None):
            if '--output_file' in cmd:
                out_path = cmd[cmd.index('--output_file') + 1]
                with wave.open(out_path, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(22050)
                    wf.writeframes(b'\x00\x00' * 1000)
                return subprocess.CompletedProcess(cmd, 0, b'', b'')
            return subprocess.CompletedProcess(cmd, 1, b'', b'Connection refused')

        monkeypatch.setattr('airport_robot_voice.tts_node.subprocess.run', fake_run_fail)
        res_fail = tts.synthesize('Xin chào')
        assert res_fail['status'] == 'WARNING_PLAYBACK_EXIT_1'
        assert res_fail['wav_generated'] is True
        assert res_fail['audio_played'] is False

        tts.destroy_node()


class TestFasterWhisperHelpersAndSegmentation:
    def test_numpy_float32_conversion_and_resampling(self):
        """Kiểm tra chuẩn hóa PCM16 -> NumPy float32 [-1.0, 1.0) và resample về 16 kHz."""
        pcm = struct.pack('<4h', 0, 32767, -32768, 16384)
        arr = pcm16_bytes_to_float32(pcm)
        assert isinstance(arr, np.ndarray)
        assert arr.dtype == np.float32
        assert arr.ndim == 1
        assert arr.flags['C_CONTIGUOUS']
        assert arr[0] == pytest.approx(0.0)
        assert arr[1] == pytest.approx(32767.0 / 32768.0, abs=1e-6)
        assert arr[2] == pytest.approx(-1.0, abs=1e-6)
        assert arr[3] == pytest.approx(0.5, abs=1e-6)

        assert pcm16_bytes_to_float32(b'').size == 0
        with pytest.raises(ValueError):
            pcm16_bytes_to_float32(b'\x00\x01\x02')

        # Resample 22050 -> 16000 Hz giữ nguyên kiểu float32
        src = np.sin(np.linspace(0, 2 * np.pi * 440, 22050, endpoint=False)).astype(np.float32)
        dst = resample_float32(src, 22050, 16000)
        assert dst.dtype == np.float32
        assert dst.shape[0] == 16000
        assert compute_rms(dst) > 0.5

    def test_utterance_segmenter_silence_multiple_and_max_cap(self):
        """Kiểm tra im lặng không sinh câu, kết thúc theo khoảng lặng < 6s, tách nhiều câu và giới hạn độ dài."""
        sr = 16000
        chunk_samples = 1600  # 100ms
        quiet = np.zeros(chunk_samples, dtype=np.int16).tobytes()
        low_noise = (np.ones(chunk_samples, dtype=np.int16) * 30).tobytes()  # rms ~ 0.0009 < 0.005
        loud = (np.sin(np.linspace(0, 2 * np.pi * 300, chunk_samples)) * 4000).astype(np.int16).tobytes()

        seg = UtteranceSegmenter(
            sample_rate=sr,
            vad_rms_threshold=0.005,
            silence_duration_sec=0.5,
            pre_roll_sec=0.2,
            min_speech_sec=0.2,
            max_utterance_sec=1.5,
            trailing_keep_sec=0.2,
        )

        # 1. Im lặng hoàn toàn & nhiễu nhỏ trong 5s -> không bao giờ phát câu
        for _ in range(50):
            assert seg.feed(quiet) is None
            assert seg.feed(low_noise) is None
        assert seg._pre_roll_dur <= 0.25

        # 2. Nháy nhiễu quá ngắn (100ms < min_speech_sec=0.2s) -> bỏ qua sau khoảng lặng
        assert seg.feed(loud) is None
        emitted_after_click = [seg.feed(quiet) for _ in range(6)]
        assert all(x is None for x in emitted_after_click)

        # 3. Hai câu nói liên tiếp ngăn bởi khoảng lặng 0.5s -> phát đúng 2 câu tách biệt
        # Câu 1: 0.4s speech rồi im lặng -> kết thúc ngay tại 0.5s (không chờ 6s)
        for _ in range(4):
            assert seg.feed(loud) is None
        utt1 = None
        for _ in range(6):
            res = seg.feed(quiet)
            if res is not None:
                utt1 = res
                break
        assert utt1 is not None
        assert 0.5 * sr * 2 <= len(utt1) <= 1.0 * sr * 2

        # Câu 2 ngay sau câu 1
        for _ in range(3):
            assert seg.feed(loud) is None
        utt2 = None
        for _ in range(6):
            res = seg.feed(quiet)
            if res is not None:
                utt2 = res
                break
        assert utt2 is not None

        # 4. Giới hạn độ dài một câu (max_utterance_sec=1.5s) khi âm thanh kéo dài liên tục
        forced = None
        for _ in range(20):
            res = seg.feed(loud)
            if res is not None:
                forced = res
                break
        assert forced is not None
        assert len(forced) == int(1.5 * sr * 2)

    def test_clean_whisper_segments_filters_punctuation_only(self):
        """Lọc segment rỗng hoặc chỉ có dấu câu; giữ nguyên mã A3 hoặc A ba."""
        mk = lambda t: type('Seg', (), {'text': t})()
        assert clean_whisper_segments([mk(' ... '), mk('?')]) == ''
        assert clean_whisper_segments([mk('  Dẫn tôi đến cổng A3. ')]) == 'Dẫn tôi đến cổng A3.'
        assert clean_whisper_segments([mk(' Dẫn tôi '), mk(' đến cổng A ba ')]) == 'Dẫn tôi đến cổng A ba'


class TestFasterWhisperNodeIntegration:
    def test_faster_whisper_silence_wav_and_single_final_transcript(self, ros_context):
        """WAV im lặng không sinh transcript rác; câu có lời chỉ publish đúng 1 transcript cuối."""
        stt = SttNode()
        stt.stt_backend = 'faster_whisper'
        stt.mode = 'wav'

        received_arrays = []

        class MockRecognizer:
            load_time_sec = 0.01

            def __init__(self, reply_text):
                self.reply_text = reply_text

            def transcribe_float32(self, audio_f32):
                assert isinstance(audio_f32, np.ndarray)
                assert audio_f32.dtype == np.float32
                received_arrays.append(audio_f32)
                if np.max(np.abs(audio_f32)) < 1e-4:
                    return '', 0.0
                return self.reply_text, 0.02

            def close(self):
                pass

        published = []
        stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

        # 1. WAV im lặng tuyệt đối
        tmp_silence = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        tmp_speech = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            with wave.open(tmp_silence, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b'\x00\x00' * 8000)

            stt._whisper_recognizer = MockRecognizer('Hãy subscribe')  # Ngay cả khi model định bịa, im lặng vẫn bị chặn
            stt.wav_file = tmp_silence
            stt.wav_loop()
            assert len(published) == 0

            # 2. WAV có tín hiệu thật -> truyền đúng NumPy float32 và publish duy nhất 1 lần
            with wave.open(tmp_speech, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                samples = (np.sin(np.linspace(0, 2 * np.pi * 250, 8000)) * 5000).astype(np.int16)
                wf.writeframes(samples.tobytes())

            stt._whisper_recognizer = MockRecognizer('Dẫn tôi đến cổng A3.')
            stt.wav_file = tmp_speech
            stt.wav_loop()
            assert published == ['Dẫn tôi đến cổng A3.']
            assert len(received_arrays) == 2
            assert received_arrays[1].dtype == np.float32
            assert np.max(np.abs(received_arrays[1])) <= 1.0
        finally:
            stt.destroy_node()
            for p in (tmp_silence, tmp_speech):
                if os.path.exists(p):
                    os.remove(p)

    def test_faster_whisper_real_venv_worker_offline_wav_and_nlu(self, ros_context):
        """Kiểm tra worker Faster-Whisper thật trong venv chạy offline qua mảng NumPy float32 + NLU."""
        cache_dir = os.path.expanduser('~/.cache/huggingface/hub/models--Systran--faster-whisper-small')
        if not os.path.exists(DEFAULT_WHISPER_VENV_PYTHON) or not os.path.exists(cache_dir):
            pytest.skip('Chưa có venv ~/.venvs/airport-whisper-test hoặc model small offline cache')

        stt = SttNode()
        stt.stt_backend = 'faster_whisper'
        stt.mode = 'wav'
        stt.whisper_model = 'small'
        stt.whisper_local_files_only = True

        published = []
        stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

        tmp_silence = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            # 1. File im lặng tuyệt đối chạy qua worker thật phải trả 0 transcript
            with wave.open(tmp_silence, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b'\x00\x00' * 16000)

            stt.wav_file = tmp_silence
            stt.wav_loop()
            assert published == []

            # 2. Thử với file /tmp/robot-mic-time.wav nếu còn trên máy (cùng một worker đã nạp sẵn)
            if os.path.exists('/tmp/robot-mic-time.wav'):
                stt.wav_file = '/tmp/robot-mic-time.wav'
                stt.wav_loop()
                assert len(published) == 1
                share = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
                nlu = NluEngine(os.path.join(share, 'intents.json'), os.path.join(share, 'places.json'))
                r = nlu.process(published[0])
                assert r['intent'] == 'ask_time'
        finally:
            stt.destroy_node()
            if os.path.exists(tmp_silence):
                os.remove(tmp_silence)

    def test_nlu_accepts_faster_whisper_target_and_cancel_variants(self):
        """Xác minh các câu đích của Faster-Whisper (kể cả 'A ba', dấu câu) và lệnh hủy giữ nguyên ngữ nghĩa."""
        share = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
        nlu = NluEngine(os.path.join(share, 'intents.json'), os.path.join(share, 'places.json'))

        r_time = nlu.process('Mấy giờ rồi đấy?')
        assert r_time['intent'] == 'ask_time'
        assert r_time['action'] is None

        for phrase in ('Dẫn tôi đến cổng A3.', 'Dẫn tôi đến cổng A ba.', 'Dẫn tôi đến cổng A 3'):
            r_a3 = nlu.process(phrase)
            assert r_a3['intent'] == 'guide_me'
            assert r_a3['action'] == 'navigate'
            assert r_a3['target'] == 'gate_a3'

        r_toilet = nlu.process('Nhà vệ sinh ở đâu?')
        assert r_toilet['intent'] == 'ask_toilet'
        assert r_toilet['action'] == 'navigate'
        assert r_toilet['category'] == 'toilet'

        for cancel_phrase in ('Hủy chuyến', 'Hủy chuyến.', 'Huỷ chuyến'):
            r_cancel = nlu.process(cancel_phrase)
            assert r_cancel['intent'] == 'cancel_robot'
            assert r_cancel['action'] == 'cancel'

    def test_stt_faster_whisper_missing_dependency_or_model_graceful(self, ros_context):
        """Khi thiếu python venv hoặc thiếu thư mục model Faster-Whisper, SttNode ghi lỗi cụ thể và không crash."""
        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            with wave.open(tmp_wav, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b'\x01\x02' * 800)

            stt1 = SttNode()
            stt1.stt_backend = 'faster_whisper'
            stt1.mode = 'wav'
            stt1.wav_file = tmp_wav
            stt1.whisper_python = '/nonexistent/python_whisper'
            stt1.wav_loop()
            stt1.destroy_node()

            stt2 = SttNode()
            stt2.stt_backend = 'faster_whisper'
            stt2.mode = 'wav'
            stt2.wav_file = tmp_wav
            stt2.whisper_model_path = '/nonexistent/whisper_model_dir'
            stt2.wav_loop()
            stt2.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)


    def test_mic_capture_inference_thread_isolation_and_clean_shutdown(self, ros_context):
        """Tách luồng suy luận khỏi callback thu âm, nhiều câu liên tiếp mỗi câu 1 transcript, tắt sạch."""
        import threading

        stt = SttNode()
        stt.stt_backend = 'faster_whisper'
        stt.silence_duration_sec = 0.4
        stt.silence_rms_threshold = 0.005
        stt.min_speech_sec = 0.2
        stt.max_utterance_sec = 3.0

        replies = iter(['Dẫn tôi đến cổng A3.', 'Hủy chuyến.'])
        infer_threads = []

        class SlowMockRecognizer:
            load_time_sec = 0.01
            closed = False

            def transcribe_float32(self, audio_f32):
                infer_threads.append(threading.current_thread().ident)
                time.sleep(0.05)  # Mô phỏng suy luận không chặn luồng capture
                return next(replies, ''), 0.05

            def close(self):
                self.closed = True

        mock_rec = SlowMockRecognizer()
        stt._whisper_recognizer = mock_rec

        published = []
        stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

        infer_t = threading.Thread(target=stt._whisper_inference_loop, args=(mock_rec,), daemon=True)
        stt._worker_threads.append(infer_t)
        infer_t.start()

        segmenter = UtteranceSegmenter(
            sample_rate=16000,
            vad_rms_threshold=stt.silence_rms_threshold,
            silence_duration_sec=stt.silence_duration_sec,
            pre_roll_sec=0.2,
            min_speech_sec=stt.min_speech_sec,
            max_utterance_sec=stt.max_utterance_sec,
        )

        loud = (np.sin(np.linspace(0, 2 * np.pi * 300, 1600)) * 5000).astype(np.int16).tobytes()
        quiet = np.zeros(1600, dtype=np.int16).tobytes()

        capture_tid = threading.current_thread().ident
        utt_seq = 0

        # Giả lập thu 2 câu liên tiếp qua segmenter -> queue -> inference worker
        for _ in range(2):
            for _ in range(3):
                assert segmenter.feed(loud) is None
            utt = None
            for _ in range(5):
                res = segmenter.feed(quiet)
                if res is not None:
                    utt = res
                    break
            assert utt is not None
            utt_seq += 1
            stt._enqueue_utterance(utt_seq, utt, time.monotonic())

        t0 = time.time()
        while len(published) < 2 and time.time() - t0 < 2.0:
            time.sleep(0.02)

        assert published == ['Dẫn tôi đến cổng A3.', 'Hủy chuyến.']
        assert all(tid != capture_tid for tid in infer_threads), 'Suy luận phải chạy ở luồng riêng'

        stt.destroy_node()
        assert mock_rec.closed is True
        assert not infer_t.is_alive()


class TestStandardizedDefaultsAndAntiEcho:
    def test_stt_default_parameters_are_faster_whisper_thinkpad_profile(self, ros_context):
        """Xác minh cấu hình mặc định mới của stt_node: faster_whisper, small, cpu, int8, vi, beam=5, mic 9, 16kHz."""
        stt = SttNode()
        assert stt.stt_backend == 'faster_whisper'
        assert stt.whisper_model == 'small'
        assert stt.whisper_device == 'cpu'
        assert stt.whisper_compute_type == 'int8'
        assert stt.whisper_language == 'vi'
        assert stt.whisper_beam_size == 5
        assert stt.whisper_local_files_only is True
        assert stt.sample_rate == 16000
        assert stt.device == 9
        assert stt.mode == 'keyboard'
        stt.destroy_node()

    def test_faster_whisper_error_does_not_fallback_to_vosk(self, ros_context):
        """Khi Faster-Whisper lỗi (sai đường dẫn model), tuyệt đối không âm thầm fallback sang Vosk."""
        bin_espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
        try:
            if bin_espeak:
                subprocess.run([bin_espeak, '-v', 'vi', 'cảm ơn', '-w', tmp_wav], check=True, stderr=subprocess.DEVNULL)
            else:
                with wave.open(tmp_wav, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(16000)
                    samples = (np.sin(np.linspace(0, 2 * np.pi * 250, 8000)) * 5000).astype(np.int16)
                    wf.writeframes(samples.tobytes())

            stt = SttNode()
            assert stt.stt_backend == 'faster_whisper'
            stt.mode = 'wav'
            stt.model_path = '/opt/vosk/vosk-model-small-vn-0.4'  # Vosk model có sẵn nhưng không được fallback!
            stt.whisper_model_path = '/nonexistent/broken_whisper_model'
            stt.wav_file = tmp_wav

            published = []
            stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

            stt.wav_loop()
            assert published == [], 'Faster-Whisper lỗi tuyệt đối không được chuyển ngầm sang Vosk'
            stt.destroy_node()
        finally:
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def test_self_listening_cooldown_and_tts_echo_suppression(self, ros_context):
        """Xác minh cơ chế chống robot tự thu lại tiếng phát từ loa gây vòng lặp lặp lại."""
        import threading

        stt = SttNode()
        stt.stt_backend = 'faster_whisper'
        stt.post_tts_cooldown_sec = 0.4
        stt.tts_echo_suppression_sec = 3.5

        published = []
        stt.pub_text = type('MockPub', (), {'publish': lambda self, msg: published.append(msg.data)})()

        # 1. Khi /voice/speaking=True -> gating bật ngay, audio queue bị xả
        stt._audio_queue.put(b'\x01\x02' * 100)
        stt.on_tts_say(String(data=json.dumps({
            'text': 'Đã xác định yêu cầu tới cổng A3, nhưng hệ thống điều hướng chưa sẵn sàng.'
        })))
        stt.on_speaking(Bool(data=True))
        assert stt.is_audio_gated() is True
        assert stt._audio_queue.empty() is True

        # 2. Khi /voice/speaking=False -> vẫn giữ cooldown 0.4s để xả đuôi PulseAudio/loa
        stt.on_tts_ack(String(data=json.dumps({
            'text': 'Đã xác định yêu cầu tới cổng A3, nhưng hệ thống điều hướng chưa sẵn sàng.',
            'spoken_text': 'Đã xác định yêu cầu tới cổng a ba, nhưng hệ thống điều hướng chưa sẵn sàng.',
            'status': 'COMPLETED'
        })))
        stt.on_speaking(Bool(data=False))
        assert stt.is_audio_gated() is True
        time.sleep(0.45)
        assert stt.is_audio_gated() is False

        # 3. Nếu tiếng loa phản hồi lọt qua mic sau cooldown -> bộ lọc anti-echo phải chặn
        # Còn câu lệnh thật của người dùng ngay sau đó vẫn phải được nhận và publish
        stt.on_tts_say(String(data='Bây giờ là 12 giờ 45 phút.'))
        stt.on_speaking(Bool(data=True))
        stt.on_speaking(Bool(data=False))

        replies = iter([
            'nhưng hệ thống điều hướng chưa sẵn sàng',   # Đuôi echo từ câu 1 -> BỊ CHẶN
            'Bây giờ là 12 giờ 45 phút.',                # Echo nguyên câu 2 -> BỊ CHẶN
            'Dẫn tôi đến cổng A3.',                      # Lệnh thật của người dùng -> CHO QUA
            'Hủy chuyến.',                               # Lệnh hủy thật -> CHO QUA
        ])

        class EchoTestRecognizer:
            load_time_sec = 0.01

            def transcribe_float32(self, audio_f32):
                return next(replies, ''), 0.01

            def close(self):
                pass

        mock_rec = EchoTestRecognizer()
        stt._whisper_recognizer = mock_rec
        infer_t = threading.Thread(target=stt._whisper_inference_loop, args=(mock_rec,), daemon=True)
        stt._worker_threads.append(infer_t)
        infer_t.start()

        fake_pcm = (np.ones(1600, dtype=np.int16) * 3000).tobytes()
        for idx in range(1, 5):
            stt._enqueue_utterance(idx, fake_pcm, time.monotonic())

        t0 = time.time()
        while len(published) < 2 and time.time() - t0 < 2.0:
            time.sleep(0.02)

        assert published == ['Dẫn tôi đến cổng A3.', 'Hủy chuyến.']
        stt.destroy_node()

    def test_nlu_all_required_keywords_and_gate_variants(self):
        """Kiểm tra đủ bộ từ khóa: A3, A4, B2, nhà vệ sinh, dừng lại, đi tiếp, đổi đích, hủy chuyến."""
        share = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
        nlu = NluEngine(os.path.join(share, 'intents.json'), os.path.join(share, 'places.json'))

        cases = [
            ('Dẫn tôi đến cổng A3', 'navigate', 'gate_a3', 'gate'),
            ('Dẫn tôi đến cổng A ba.', 'navigate', 'gate_a3', 'gate'),
            ('Dẫn tôi đến cổng A4', 'navigate', 'gate_a4', 'gate'),
            ('Cho tôi tới cổng B2', 'navigate', 'gate_b2', 'gate'),
            ('Cho tôi tới cổng bê hai', 'navigate', 'gate_b2', 'gate'),
            ('Nhà vệ sinh ở đâu?', 'navigate', None, 'toilet'),
            ('Dừng lại', 'pause', None, None),
            ('Đi tiếp', 'resume', None, None),
            ('Đổi sang cổng A4', 'replace', 'gate_a4', 'gate'),
            ('Hủy chuyến', 'cancel', None, None),
        ]
        for text, exp_action, exp_target, exp_cat in cases:
            res = nlu.process(text)
            assert res['action'] == exp_action, f'Failed action for {text!r}: {res}'
            assert res['target'] == exp_target, f'Failed target for {text!r}: {res}'
            assert res['category'] == exp_cat, f'Failed category for {text!r}: {res}'
