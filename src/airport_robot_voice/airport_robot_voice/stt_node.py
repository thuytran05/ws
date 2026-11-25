#!/usr/bin/env python3
"""
stt_node.py — Nhận dạng tiếng nói thành văn bản tiếng Việt offline.
Hỗ trợ 3 chế độ nhập:
  1. keyboard : Nhập văn bản qua bàn phím / terminal stdin.
  2. wav      : Đọc trực tiếp từ file WAV (kiểm thử lặp lại).
  3. mic      : Thu âm trực tiếp từ micro qua ALSA/sounddevice.

Hỗ trợ 2 backend nhận diện:
  - faster_whisper : Backend STT mặc định (small/cpu/int8/vi, offline qua NumPy float32
                     và worker sống lâu trong venv hoặc import trực tiếp khi có faster_whisper).
  - vosk           : Offline nhẹ dự phòng (/opt/vosk/vosk-model-small-vn-0.4), dùng khi stt_backend:=vosk.

Publish:
  /voice/text     (std_msgs/String)
Subscribe:
  /voice/speaking (std_msgs/Bool) -> Tạm ngưng thu âm khi robot đang nói, xả sạch buffer.
"""

import json
import os
import queue
import threading
import time
import wave
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Bool

import collections

from airport_robot_voice.faster_whisper_worker import (
    UtteranceSegmenter,
    create_faster_whisper_recognizer,
    is_tts_self_echo,
    norm_text_for_echo,
    pcm16_bytes_to_float32,
    resample_float32,
)
from airport_robot_voice.tts_node import normalize_speech_text


def _format_input_devices(sd_mod) -> str:
    """Liệt kê ngắn gọn các thiết bị thu âm khả dụng để hỗ trợ chẩn đoán khi mở mic lỗi."""
    try:
        devs = sd_mod.query_devices()
        lines = []
        for idx, d in enumerate(devs):
            in_ch = d.get('max_input_channels', 0)
            if in_ch > 0:
                sr = int(d.get('default_samplerate', 0))
                lines.append(f'[{idx}] {d.get("name", "?")} (in={in_ch}, default_sr={sr})')
        return '; '.join(lines) if lines else 'Không có thiết bị thu âm nào'
    except Exception as exc:
        return f'Không truy vấn được danh sách thiết bị: {exc}'


class SttNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Tham số cốt lõi (chuẩn hóa Faster-Whisper + ThinkPad mic index 9 làm mặc định)
        self.declare_parameter('mode', 'keyboard')              # keyboard | mic | wav
        self.declare_parameter('stt_backend', 'faster_whisper') # faster_whisper | vosk
        self.declare_parameter('wav_file', '')                  # Đường dẫn file wav khi mode=wav
        self.declare_parameter('model_path', '/opt/vosk/vosk-model-small-vn-0.4')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device', 9)                     # Index mic mặc định ThinkPad hw:1,7 (-1 = mặc định hệ thống)
        self.declare_parameter('wake_word', '')             # Để trống = luôn nghe
        self.declare_parameter('wake_timeout_sec', 15.0)    # Timeout trở về trạng thái chờ
        self.declare_parameter('min_chars', 2)
        self.declare_parameter('audio_queue_size', 50)

        # Tham số cấu hình Faster-Whisper
        self.declare_parameter('whisper_model', 'small')
        self.declare_parameter('whisper_model_path', '')
        self.declare_parameter('whisper_cache_dir', '')
        self.declare_parameter('whisper_python', '')
        self.declare_parameter('whisper_device', 'cpu')
        self.declare_parameter('whisper_compute_type', 'int8')
        self.declare_parameter('whisper_language', 'vi')
        self.declare_parameter('whisper_beam_size', 5)
        self.declare_parameter('whisper_local_files_only', True)
        self.declare_parameter('whisper_vad_filter', True)
        self.declare_parameter('whisper_without_timestamps', True)
        self.declare_parameter('whisper_condition_on_previous_text', False)
        self.declare_parameter('whisper_cpu_threads', 0)
        self.declare_parameter('whisper_initial_prompt', '')
        self.declare_parameter('whisper_infer_timeout_sec', 30.0)

        # Tham số phân đoạn câu nói cho mic Faster-Whisper
        self.declare_parameter('mic_block_ms', 100)
        self.declare_parameter('silence_rms_threshold', 0.005)
        self.declare_parameter('silence_duration_sec', 0.8)
        self.declare_parameter('pre_roll_sec', 0.3)
        self.declare_parameter('min_speech_sec', 0.2)
        self.declare_parameter('max_utterance_sec', 12.0)
        self.declare_parameter('utterance_queue_size', 4)

        # Tham số chống tự thu tiếng loa (acoustic echo / self-listening suppression)
        self.declare_parameter('post_tts_cooldown_sec', 0.7)
        self.declare_parameter('tts_echo_suppression_sec', 3.5)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.mode = self.get_parameter('mode').value
        self.stt_backend = str(self.get_parameter('stt_backend').value).strip().lower()
        self.wav_file = self.get_parameter('wav_file').value
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.device = int(self.get_parameter('device').value)
        self.wake_word = self.get_parameter('wake_word').value.lower().strip()
        self.wake_timeout = float(self.get_parameter('wake_timeout_sec').value)
        self.min_chars = int(self.get_parameter('min_chars').value)
        self.audio_queue_size = int(self.get_parameter('audio_queue_size').value)

        self.whisper_model = str(self.get_parameter('whisper_model').value)
        self.whisper_model_path = str(self.get_parameter('whisper_model_path').value)
        self.whisper_cache_dir = str(self.get_parameter('whisper_cache_dir').value)
        self.whisper_python = str(self.get_parameter('whisper_python').value)
        self.whisper_device = str(self.get_parameter('whisper_device').value)
        self.whisper_compute_type = str(self.get_parameter('whisper_compute_type').value)
        self.whisper_language = str(self.get_parameter('whisper_language').value)
        self.whisper_beam_size = int(self.get_parameter('whisper_beam_size').value)
        self.whisper_local_files_only = bool(self.get_parameter('whisper_local_files_only').value)
        self.whisper_vad_filter = bool(self.get_parameter('whisper_vad_filter').value)
        self.whisper_without_timestamps = bool(self.get_parameter('whisper_without_timestamps').value)
        self.whisper_condition_on_previous_text = bool(self.get_parameter('whisper_condition_on_previous_text').value)
        self.whisper_cpu_threads = int(self.get_parameter('whisper_cpu_threads').value)
        self.whisper_initial_prompt = str(self.get_parameter('whisper_initial_prompt').value)
        self.whisper_infer_timeout_sec = float(self.get_parameter('whisper_infer_timeout_sec').value)

        self.mic_block_ms = int(self.get_parameter('mic_block_ms').value)
        self.silence_rms_threshold = float(self.get_parameter('silence_rms_threshold').value)
        self.silence_duration_sec = float(self.get_parameter('silence_duration_sec').value)
        self.pre_roll_sec = float(self.get_parameter('pre_roll_sec').value)
        self.min_speech_sec = float(self.get_parameter('min_speech_sec').value)
        self.max_utterance_sec = float(self.get_parameter('max_utterance_sec').value)
        self.utterance_queue_size = max(1, int(self.get_parameter('utterance_queue_size').value))
        self.post_tts_cooldown_sec = max(0.0, float(self.get_parameter('post_tts_cooldown_sec').value))
        self.tts_echo_suppression_sec = max(0.0, float(self.get_parameter('tts_echo_suppression_sec').value))

        # Publishers & Subscribers
        self.pub_text = self.create_publisher(String, '/voice/text', 10)
        self.create_subscription(Bool, '/voice/speaking', self.on_speaking, 10)
        self.create_subscription(String, '/voice/say', self.on_tts_say, 10)
        self.create_subscription(String, '/voice/tts_ack', self.on_tts_ack, 10)

        self.robot_speaking = False
        self._speaking_cooldown_until = 0.0
        self._recent_tts_speeches = collections.deque(maxlen=20)
        self._tts_echo_lock = threading.Lock()
        self._reset_segmenter_requested = False
        self.awake = (self.wake_word == '')
        self._last_wake_time = time.time()
        self._running = True
        self._audio_queue = queue.Queue(maxsize=self.audio_queue_size)
        self._utterance_queue = queue.Queue(maxsize=self.utterance_queue_size)
        self._whisper_recognizer = None
        self._worker_threads = []

        # Khởi chạy luồng theo mode
        if self.mode == 'keyboard':
            self.get_logger().info('STT chế độ KEYBOARD: Gõ câu nói rồi nhấn Enter.')
            t = threading.Thread(target=self.keyboard_loop, daemon=True)
            self._worker_threads.append(t)
            t.start()
        elif self.mode == 'wav':
            self.get_logger().info(f'STT chế độ WAV (backend={self.stt_backend}): Đọc file {self.wav_file}')
            t = threading.Thread(target=self.wav_loop, daemon=True)
            self._worker_threads.append(t)
            t.start()
        elif self.mode == 'mic':
            self.get_logger().info(
                f'STT chế độ MICRO (backend={self.stt_backend}, device={self.device}, rate={self.sample_rate}): đang khởi tạo...'
            )
            t = threading.Thread(target=self.mic_loop, daemon=True)
            self._worker_threads.append(t)
            t.start()
        else:
            self.get_logger().error(f'Mode không hợp lệ: {self.mode}. Dùng keyboard, mic hoặc wav.')

    def destroy_node(self):
        self._running = False
        try:
            self._utterance_queue.put_nowait(None)
        except Exception:
            pass
        rec = getattr(self, '_whisper_recognizer', None)
        if rec is not None:
            try:
                rec.close()
            except (Exception, KeyboardInterrupt):
                pass
            self._whisper_recognizer = None
        for t in self._worker_threads:
            if t.is_alive() and threading.current_thread() is not t:
                try:
                    t.join(timeout=1.0)
                except (Exception, KeyboardInterrupt):
                    pass
        try:
            super().destroy_node()
        except (Exception, KeyboardInterrupt):
            pass

    def _safe_log(self, level: str, msg: str):
        if not rclpy.ok():
            return
        try:
            logger = self.get_logger()
            if level == 'error':
                logger.error(msg)
            elif level == 'warn':
                logger.warn(msg)
            else:
                logger.info(msg)
        except Exception:
            pass

    def _drain_audio_queue(self):
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

    def _record_tts_text(self, text: str):
        now = time.monotonic()
        raw_norm = norm_text_for_echo(text)
        spoken_norm = norm_text_for_echo(normalize_speech_text(text))
        with self._tts_echo_lock:
            if raw_norm:
                self._recent_tts_speeches.append((raw_norm, now))
            if spoken_norm and spoken_norm != raw_norm:
                self._recent_tts_speeches.append((spoken_norm, now))

    def _refresh_recent_tts_timestamps(self):
        now = time.monotonic()
        with self._tts_echo_lock:
            if not self._recent_tts_speeches:
                return
            refreshed = [(txt, now) for txt, ts in self._recent_tts_speeches if (now - ts) <= 15.0]
            self._recent_tts_speeches.clear()
            self._recent_tts_speeches.extend(refreshed)

    def on_tts_say(self, msg: String):
        data_str = (msg.data or '').strip()
        if not data_str:
            return
        text = data_str
        if data_str.startswith('{') and data_str.endswith('}'):
            try:
                text = json.loads(data_str).get('text', data_str)
            except Exception:
                text = data_str
        self._record_tts_text(text)

    def on_tts_ack(self, msg: String):
        data_str = (msg.data or '').strip()
        if not data_str:
            return
        try:
            payload = json.loads(data_str)
            for key in ('text', 'spoken_text'):
                val = payload.get(key)
                if val:
                    self._record_tts_text(str(val))
        except Exception:
            pass

    def is_audio_gated(self, now_mono: float = None) -> bool:
        """Trả về True khi robot đang nói hoặc vẫn đang trong khoảng trễ xả đuôi loa sau TTS."""
        if self.robot_speaking:
            return True
        now = time.monotonic() if now_mono is None else now_mono
        return now < self._speaking_cooldown_until

    def is_echo_transcript(self, text: str) -> tuple:
        if self.tts_echo_suppression_sec <= 0.0:
            return False, ''
        now = time.monotonic()
        with self._tts_echo_lock:
            recent = list(self._recent_tts_speeches)
        return is_tts_self_echo(text, recent, now, self.tts_echo_suppression_sec)

    def on_speaking(self, msg: Bool):
        prev_speaking = self.robot_speaking
        self.robot_speaking = bool(msg.data)
        if self.robot_speaking:
            # Xả hết buffer audio đọng khi robot đang nói để tránh nhận nhầm tiếng loa
            self._drain_audio_queue()
            self._reset_segmenter_requested = True
        elif prev_speaking and not self.robot_speaking:
            # Khi robot vừa nói xong, áp dụng post_tts_cooldown_sec để bỏ đuôi buffer PulseAudio/loa
            self._speaking_cooldown_until = time.monotonic() + self.post_tts_cooldown_sec
            self._refresh_recent_tts_timestamps()
            self._drain_audio_queue()
            self._reset_segmenter_requested = True

    def publish_text(self, text: str):
        text = (text or '').strip()
        if len(text) < self.min_chars:
            return

        now = time.time()
        # Xử lý wake word
        if self.wake_word:
            if not self.awake:
                if self.wake_word in text.lower():
                    self.awake = True
                    self._last_wake_time = now
                    text = text.lower().replace(self.wake_word, '').strip()
                    self._safe_log('info', 'Wake word đã được kích hoạt.')
                    if not text:
                        self.pub_text.publish(String(data='__wake__'))
                        return
                else:
                    return
            else:
                if now - self._last_wake_time > self.wake_timeout:
                    self.awake = False
                    self._safe_log('info', 'Hết thời gian tương tác (wake timeout), trở về chờ wake word.')
                    return
                else:
                    self._last_wake_time = now

        self._safe_log('info', f'NGHE ĐƯỢC: "{text}"')
        self.pub_text.publish(String(data=text))

    def _get_or_create_whisper_recognizer(self):
        if self._whisper_recognizer is not None:
            return self._whisper_recognizer
        rec = create_faster_whisper_recognizer(
            whisper_python=self.whisper_python,
            model=self.whisper_model,
            model_path=self.whisper_model_path,
            cache_dir=self.whisper_cache_dir,
            device=self.whisper_device,
            compute_type=self.whisper_compute_type,
            language=self.whisper_language,
            beam_size=self.whisper_beam_size,
            local_files_only=self.whisper_local_files_only,
            vad_filter=self.whisper_vad_filter,
            without_timestamps=self.whisper_without_timestamps,
            condition_on_previous_text=self.whisper_condition_on_previous_text,
            cpu_threads=self.whisper_cpu_threads,
            initial_prompt=self.whisper_initial_prompt,
            infer_timeout_sec=self.whisper_infer_timeout_sec,
            logger=lambda m: self._safe_log('info', m),
        )
        self._whisper_recognizer = rec
        load_t = getattr(rec, 'load_time_sec', 0.0)
        self._safe_log(
            'info',
            f'Đã nạp Faster-Whisper (model={self.whisper_model_path or self.whisper_model}, '
            f'device={self.whisper_device}, compute_type={self.whisper_compute_type}, '
            f'lang={self.whisper_language}, beam={self.whisper_beam_size}, offline={self.whisper_local_files_only}) '
            f'trong {load_t:.2f}s.'
        )
        return rec

    def keyboard_loop(self):
        while self._running and rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break
            self.publish_text(line)

    def wav_loop(self):
        backend = (self.stt_backend or 'faster_whisper').strip().lower()
        if backend not in ('vosk', 'faster_whisper'):
            self._safe_log('error', f'stt_backend không hợp lệ: "{self.stt_backend}". Dùng vosk hoặc faster_whisper.')
            return

        if not self.wav_file or not os.path.exists(self.wav_file):
            self._safe_log('error', f'File WAV không tồn tại: {self.wav_file}')
            return

        try:
            wf = wave.open(self.wav_file, 'rb')
        except Exception as e:
            self._safe_log('error', f'Lỗi mở file WAV: {e}')
            return

        if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
            self._safe_log(
                'error',
                f'WAV phải là mono 16-bit PCM! (Kênh: {wf.getnchannels()}, Mẫu: {wf.getsampwidth()*8}-bit)'
            )
            wf.close()
            return

        wav_rate = wf.getframerate()

        if backend == 'faster_whisper':
            self._wav_loop_faster_whisper(wf, wav_rate)
        else:
            self._wav_loop_vosk(wf, wav_rate)

    def _wav_loop_vosk(self, wf, wav_rate: int):
        try:
            from vosk import Model, KaldiRecognizer, SetLogLevel
            SetLogLevel(-1)
            model = Model(self.model_path)
            rec = KaldiRecognizer(model, wav_rate)
        except Exception as e:
            self._safe_log(
                'warn',
                f'Không thể nạp Vosk cho file WAV ({e}). Đang phát văn bản mô phỏng nếu có.'
            )
            wf.close()
            return

        try:
            while self._running and rclpy.ok():
                data = wf.readframes(4000)
                if len(data) == 0:
                    break
                if rec.AcceptWaveform(data):
                    res = json.loads(rec.Result())
                    self.publish_text(res.get('text', ''))
            final_res = json.loads(rec.FinalResult())
            self.publish_text(final_res.get('text', ''))
            wf.close()
            self._safe_log('info', 'Đã xử lý xong file WAV (Vosk).')
        except Exception as e:
            self._safe_log('error', f'Lỗi đọc file WAV: {e}')
            try:
                wf.close()
            except Exception:
                pass

    def _wav_loop_faster_whisper(self, wf, wav_rate: int):
        try:
            pcm_bytes = wf.readframes(wf.getnframes())
        except Exception as exc:
            self._safe_log('error', f'Lỗi đọc dữ liệu WAV: {exc}')
            return
        finally:
            try:
                wf.close()
            except Exception:
                pass

        try:
            recognizer = self._get_or_create_whisper_recognizer()
        except Exception as exc:
            self._safe_log('error', f'Không thể khởi tạo Faster-Whisper cho WAV: {exc}')
            return

        try:
            t_endpoint = time.monotonic()
            audio_f32 = pcm16_bytes_to_float32(pcm_bytes)
            if wav_rate != 16000:
                audio_f32 = resample_float32(audio_f32, wav_rate, 16000)

            text, infer_sec = recognizer.transcribe_float32(audio_f32)
            elapsed = time.monotonic() - t_endpoint
            if text:
                self._safe_log(
                    'info',
                    f'[faster_whisper][wav] endpoint->publish={elapsed:.3f}s (infer={infer_sec:.3f}s): "{text}"'
                )
                self.publish_text(text)
            else:
                self._safe_log(
                    'info',
                    f'[faster_whisper][wav] File im lặng hoặc không có lời nói (elapsed={elapsed:.3f}s).'
                )
            self._safe_log('info', 'Đã xử lý xong file WAV (Faster-Whisper).')
        except Exception as exc:
            self._safe_log('error', f'Lỗi nhận diện WAV bằng Faster-Whisper: {exc}')

    def mic_loop(self):
        backend = (self.stt_backend or 'faster_whisper').strip().lower()
        if backend == 'faster_whisper':
            self._mic_loop_faster_whisper()
        elif backend == 'vosk':
            self._mic_loop_vosk()
        else:
            self._safe_log('error', f'stt_backend không hợp lệ: "{self.stt_backend}". Dùng vosk hoặc faster_whisper.')

    def _mic_loop_vosk(self):
        try:
            import sounddevice as sd
            from vosk import Model, KaldiRecognizer, SetLogLevel
        except ImportError as e:
            self._safe_log(
                'error',
                f'Thiếu thư viện âm thanh: {e}. '
                'Để kiểm thử offline không cần micro, dùng mode:=keyboard hoặc mode:=wav.'
            )
            return

        SetLogLevel(-1)

        try:
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f'Đường dẫn model không tồn tại: {self.model_path}')
            model = Model(self.model_path)
        except Exception as e:
            self._safe_log(
                'error',
                f'Không nạp được model Vosk tại {self.model_path}: {e}. '
                'Vui lòng tải model hoặc dùng mode:=keyboard.'
            )
            return

        rec = KaldiRecognizer(model, self.sample_rate)
        rec.SetWords(False)

        def audio_callback(indata, frames, time_info, status):
            if status:
                self._safe_log('warn', str(status))
            try:
                self._audio_queue.put_nowait(bytes(indata))
            except queue.Full:
                pass  # Bỏ qua chunk cũ khi hàng đợi đầy để tránh tràn bộ nhớ

        dev = None if self.device < 0 else self.device
        self._safe_log('info', 'Micro ALSA đã sẵn sàng (Vosk). Bắt đầu lắng nghe...')

        try:
            with sd.RawInputStream(samplerate=self.sample_rate, blocksize=4000,
                                   device=dev, dtype='int16',
                                   channels=1, callback=audio_callback):
                while self._running and rclpy.ok():
                    try:
                        data = self._audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if self.is_audio_gated():
                        rec.Reset()
                        continue

                    if rec.AcceptWaveform(data):
                        res = json.loads(rec.Result())
                        self.publish_text(res.get('text', ''))
        except Exception as e:
            devs = _format_input_devices(sd)
            self._safe_log(
                'error',
                f'Lỗi thiết bị âm thanh micro (device={self.device}, rate={self.sample_rate}): {e}. '
                f'Các thiết bị thu âm khả dụng: {devs}'
            )

    def _enqueue_utterance(self, utt_id: int, pcm_bytes: bytes, endpoint_time: float):
        item = (utt_id, pcm_bytes, endpoint_time)
        try:
            self._utterance_queue.put_nowait(item)
        except queue.Full:
            # Giới hạn bộ đệm: loại câu cũ nhất nếu suy luận chưa kịp tiêu thụ
            try:
                dropped = self._utterance_queue.get_nowait()
                if dropped is not None:
                    self._safe_log('warn', f'Hàng đợi câu thoại đầy ({self.utterance_queue_size}), bỏ qua câu cũ #{dropped[0]}.')
            except queue.Empty:
                pass
            try:
                self._utterance_queue.put_nowait(item)
            except queue.Full:
                pass

    def _whisper_inference_loop(self, recognizer):
        while self._running and rclpy.ok():
            try:
                item = self._utterance_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:
                break

            utt_id, pcm_bytes, endpoint_time = item
            if not self._running or not rclpy.ok():
                break

            try:
                audio_f32 = pcm16_bytes_to_float32(pcm_bytes)
                if self.sample_rate != 16000:
                    audio_f32 = resample_float32(audio_f32, self.sample_rate, 16000)

                text, infer_sec = recognizer.transcribe_float32(audio_f32)
                if not self._running or not rclpy.ok():
                    break

                text = (text or '').strip()
                elapsed = time.monotonic() - endpoint_time
                if text:
                    is_echo, echo_reason = self.is_echo_transcript(text)
                    if is_echo:
                        self._safe_log(
                            'info',
                            f'[anti-echo] Bỏ qua tiếng loa phản hồi ({echo_reason}, endpoint->drop={elapsed:.3f}s): "{text}"'
                        )
                        continue
                    self._safe_log(
                        'info',
                        f'[faster_whisper][mic] Câu #{utt_id} (endpoint->publish={elapsed:.3f}s, infer={infer_sec:.3f}s): "{text}"'
                    )
                    self.publish_text(text)
                else:
                    self._safe_log(
                        'info',
                        f'[faster_whisper][mic] Câu #{utt_id} không có lời nói rõ (endpoint->done={elapsed:.3f}s).'
                    )
            except Exception as exc:
                self._safe_log('error', f'Lỗi suy luận Faster-Whisper câu #{utt_id}: {exc}')

    def _mic_loop_faster_whisper(self):
        try:
            import sounddevice as sd
        except ImportError as e:
            self._safe_log(
                'error',
                f'Thiếu thư viện sounddevice: {e}. Không thể mở micro.'
            )
            return

        # Nạp model Faster-Whisper 1 lần trước khi mở luồng thu âm
        try:
            recognizer = self._get_or_create_whisper_recognizer()
        except Exception as e:
            self._safe_log('error', f'Không thể nạp Faster-Whisper trước khi mở micro: {e}')
            return

        segmenter = UtteranceSegmenter(
            sample_rate=self.sample_rate,
            vad_rms_threshold=self.silence_rms_threshold,
            silence_duration_sec=self.silence_duration_sec,
            pre_roll_sec=self.pre_roll_sec,
            min_speech_sec=self.min_speech_sec,
            max_utterance_sec=self.max_utterance_sec,
        )

        blocksize = max(160, int(round(self.sample_rate * max(20, self.mic_block_ms) / 1000.0)))
        self._drain_audio_queue()
        while not self._utterance_queue.empty():
            try:
                self._utterance_queue.get_nowait()
            except queue.Empty:
                break

        # Luồng suy luận tách rời hoàn toàn khỏi callback thu âm và luồng segment
        infer_thread = threading.Thread(
            target=self._whisper_inference_loop,
            args=(recognizer,),
            daemon=True,
        )
        self._worker_threads.append(infer_thread)
        infer_thread.start()

        def audio_callback(indata, frames, time_info, status):
            if status:
                self._safe_log('warn', str(status))
            if not self._running or self.is_audio_gated():
                return
            try:
                self._audio_queue.put_nowait(bytes(indata))
            except queue.Full:
                # Xả bớt 1 chunk cũ nhất để giữ dữ liệu mới trong phạm vi giới hạn hàng đợi
                try:
                    self._audio_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._audio_queue.put_nowait(bytes(indata))
                except queue.Full:
                    pass

        dev = None if self.device < 0 else self.device
        self._safe_log(
            'info',
            f'Đang mở micro ALSA (Faster-Whisper, device={self.device}, rate={self.sample_rate})...'
        )

        utt_seq = 0
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=blocksize,
                device=dev,
                dtype='int16',
                channels=1,
                callback=audio_callback,
            ):
                self._safe_log(
                    'info',
                    f'Micro ALSA đã sẵn sàng (Faster-Whisper, device={self.device}, rate={self.sample_rate}, '
                    f'silence={self.silence_duration_sec:.2f}s, rms_th={self.silence_rms_threshold:.4f}, '
                    f'max_utt={self.max_utterance_sec:.1f}s). Mời bạn nói...'
                )
                while self._running and rclpy.ok():
                    try:
                        data = self._audio_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    if self.is_audio_gated() or self._reset_segmenter_requested:
                        segmenter.reset()
                        self._reset_segmenter_requested = False
                        if self.is_audio_gated():
                            continue

                    utt_bytes = segmenter.feed(data)
                    if utt_bytes is not None:
                        endpoint_time = time.monotonic()
                        utt_seq += 1
                        self._enqueue_utterance(utt_seq, utt_bytes, endpoint_time)
        except Exception as e:
            devs = _format_input_devices(sd)
            self._safe_log(
                'error',
                f'Lỗi mở hoặc đọc micro (device={self.device}, sample_rate={self.sample_rate}): {e}. '
                f'Kiểm tra lại index micro trong danh sách khả dụng: {devs}'
            )
        finally:
            try:
                self._utterance_queue.put_nowait(None)
            except Exception:
                pass
            if infer_thread.is_alive():
                infer_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == '__main__':
    main()
