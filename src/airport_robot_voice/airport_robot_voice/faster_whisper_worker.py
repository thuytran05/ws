#!/usr/bin/env python3
"""
faster_whisper_worker.py — Lõi chuyển đổi NumPy float32, phân đoạn câu nói theo khoảng lặng
và worker Faster-Whisper offline cho stt_node.

Module này KHÔNG phụ thuộc rclpy, nên có thể chạy trực tiếp:
  - Trong chính Python của node ROS (nếu faster_whisper cài sẵn), HOẶC
  - Như một subprocess worker sống lâu chạy trong venv (mặc định ~/.venvs/airport-whisper-test)
    giao tiếp nhị phân qua stdin/stdout với stt_node.
"""

import argparse
import collections
import difflib
import importlib.util
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import unicodedata
from typing import Callable, Optional, Tuple

import numpy as np

DEFAULT_WHISPER_VENV_PYTHON = os.path.expanduser('~/.venvs/airport-whisper-test/bin/python')


def pcm16_bytes_to_float32(pcm_bytes: bytes) -> np.ndarray:
    """Chuyển buffer PCM 16-bit signed little-endian thành mảng NumPy float32 chuẩn hóa [-1.0, 1.0)."""
    if not pcm_bytes:
        return np.zeros(0, dtype=np.float32)
    if len(pcm_bytes) % 2 != 0:
        raise ValueError(f'Dữ liệu PCM16 phải có số byte chẵn, nhận được {len(pcm_bytes)} bytes')
    arr = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.float32) / 32768.0
    return np.ascontiguousarray(arr, dtype=np.float32)


def resample_float32(audio: np.ndarray, src_rate: int, dst_rate: int = 16000) -> np.ndarray:
    """Nội suy tuyến tính mảng float32 mono từ src_rate về dst_rate (16 kHz cho Whisper)."""
    if src_rate <= 0 or dst_rate <= 0:
        raise ValueError(f'Sample rate không hợp lệ: src_rate={src_rate}, dst_rate={dst_rate}')
    arr = np.asarray(audio, dtype=np.float32)
    if arr.size == 0 or src_rate == dst_rate:
        return np.ascontiguousarray(arr, dtype=np.float32)
    new_len = max(1, int(round(arr.shape[0] * float(dst_rate) / float(src_rate))))
    x_old = np.linspace(0.0, 1.0, num=arr.shape[0], endpoint=False)
    x_new = np.linspace(0.0, 1.0, num=new_len, endpoint=False)
    res = np.interp(x_new, x_old, arr).astype(np.float32)
    return np.ascontiguousarray(res, dtype=np.float32)


def compute_rms(audio: np.ndarray) -> float:
    """Tính RMS năng lượng tín hiệu trên mảng float32."""
    if audio.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(audio, dtype=np.float32))))


def is_digital_silence(audio: np.ndarray, min_peak: float = 1e-4) -> bool:
    """Kiểm tra âm thanh gần như bằng 0 tuyệt đối để bỏ qua trước khi chạy model."""
    if audio.size == 0:
        return True
    peak = float(np.max(np.abs(audio)))
    return peak < min_peak


def clean_whisper_segments(segments) -> str:
    """Ghép các segment Whisper thành một transcript cuối và lọc chuỗi chỉ có dấu câu/khoảng trắng."""
    parts = []
    for seg in segments:
        txt = (getattr(seg, 'text', '') or '').strip()
        if txt:
            parts.append(txt)
    joined = ' '.join(parts).strip()
    if not joined or all(not ch.isalnum() for ch in joined):
        return ''
    return joined


def norm_text_for_echo(text: str) -> str:
    """Chuẩn hóa văn bản (NFC, chữ thường, bỏ dấu câu) để so khớp tiếng loa phản hồi."""
    t = unicodedata.normalize('NFC', text or '').lower().strip()
    t = re.sub(r'[^\w\s]', ' ', t, flags=re.UNICODE)
    return re.sub(r'\s+', ' ', t).strip()


def is_tts_self_echo(
    candidate_text: str,
    recent_speeches: list,
    now_mono: float,
    echo_window_sec: float = 3.5,
) -> Tuple[bool, str]:
    """Kiểm tra transcript vừa nhận có phải tiếng TTS robot tự phát bị micro thu lại hay không.

    Chỉ áp dụng trong cửa sổ `echo_window_sec` quanh thời điểm robot nói xong.
    Không chặn các lệnh ngắn hợp lệ của người dùng như 'hủy chuyến', 'dừng lại', 'đi tiếp'.
    """
    cand_norm = norm_text_for_echo(candidate_text)
    if not cand_norm:
        return False, ''

    for spoken_norm, spoken_ts in recent_speeches:
        if not spoken_norm:
            continue
        if (now_mono - spoken_ts) > echo_window_sec:
            continue
        ratio = difflib.SequenceMatcher(None, cand_norm, spoken_norm).ratio()
        if ratio >= 0.80:
            return True, f'similarity={ratio:.2f}'
        if len(cand_norm) >= 15 and cand_norm in spoken_norm:
            return True, 'substring_in_tts'
    return False, ''


def resolve_whisper_model_source(model_name: str, model_path: str = '') -> Tuple[str, bool]:
    """Xác định nguồn model Faster-Whisper và cho biết đó có phải đường dẫn cục bộ hay không.

    Trả về (model_size_or_path, is_local_path).
    Nếu đường dẫn cục bộ không tồn tại sẽ ném FileNotFoundError với thông báo rõ ràng.
    """
    raw_path = (model_path or '').strip()
    raw_name = (model_name or '').strip() or 'small'

    if raw_path:
        expanded = os.path.expanduser(raw_path)
        if not os.path.exists(expanded):
            raise FileNotFoundError(
                f'Không tìm thấy thư mục/file model Faster-Whisper tại whisper_model_path={expanded}'
            )
        return expanded, True

    expanded_name = os.path.expanduser(raw_name)
    looks_local = (
        raw_name.startswith('/')
        or raw_name.startswith('./')
        or raw_name.startswith('../')
        or raw_name.startswith('~')
        or os.path.exists(expanded_name)
    )
    if looks_local:
        if not os.path.exists(expanded_name):
            raise FileNotFoundError(
                f'Không tìm thấy thư mục/file model Faster-Whisper tại whisper_model={expanded_name}'
            )
        return expanded_name, True

    return raw_name, False


class UtteranceSegmenter:
    """Phân đoạn câu nói theo năng lượng RMS và khoảng lặng cuối câu (endpointing).

    - IDLE: giữ buffer pre-roll giới hạn độ dài để không mất âm đầu câu.
    - SPEAKING: tích lũy chunk đến khi im lặng đủ `silence_duration_sec` hoặc chạm `max_utterance_sec`.
    - Chỉ trả về 1 mảng bytes hoàn chỉnh cho mỗi câu hợp lệ (>= min_speech_sec).
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        vad_rms_threshold: float = 0.005,
        silence_duration_sec: float = 0.8,
        pre_roll_sec: float = 0.3,
        min_speech_sec: float = 0.2,
        max_utterance_sec: float = 12.0,
        trailing_keep_sec: float = 0.3,
    ):
        self.sample_rate = max(1, int(sample_rate))
        self.vad_rms_threshold = max(0.0, float(vad_rms_threshold))
        self.silence_duration_sec = max(0.05, float(silence_duration_sec))
        self.pre_roll_sec = max(0.0, float(pre_roll_sec))
        self.min_speech_sec = max(0.02, float(min_speech_sec))
        self.max_utterance_sec = max(0.2, float(max_utterance_sec))
        self.trailing_keep_sec = max(0.0, float(trailing_keep_sec))

        self._pre_roll = collections.deque()
        self._pre_roll_dur = 0.0
        self._speaking = False
        self._chunks = []  # List[Tuple[bytes, bool, float]]
        self._last_speech_idx = -1
        self._total_dur = 0.0
        self._speech_dur = 0.0
        self._silence_dur = 0.0

    @property
    def is_speaking(self) -> bool:
        return self._speaking

    def reset(self):
        self._pre_roll.clear()
        self._pre_roll_dur = 0.0
        self._speaking = False
        self._chunks.clear()
        self._last_speech_idx = -1
        self._total_dur = 0.0
        self._speech_dur = 0.0
        self._silence_dur = 0.0

    def _trim_and_emit(self) -> Optional[bytes]:
        if self._speech_dur < self.min_speech_sec or not self._chunks:
            self.reset()
            return None

        # Giữ tới vị trí có tiếng nói cuối cùng + tối đa trailing_keep_sec khoảng lặng đuôi
        if self._last_speech_idx >= 0:
            end_idx = self._last_speech_idx + 1
            kept_tail = 0.0
            while end_idx < len(self._chunks) and kept_tail < self.trailing_keep_sec:
                kept_tail += self._chunks[end_idx][2]
                end_idx += 1
            pcm_out = b''.join(c[0] for c in self._chunks[:end_idx])
        else:
            pcm_out = b''.join(c[0] for c in self._chunks)

        self.reset()
        return pcm_out

    def feed(self, pcm_bytes: bytes) -> Optional[bytes]:
        """Nạp một chunk PCM16 mono. Trả về bytes cả câu khi phát hiện kết thúc câu, ngược lại None."""
        if not pcm_bytes:
            return None

        audio_f32 = pcm16_bytes_to_float32(pcm_bytes)
        if audio_f32.size == 0:
            return None

        chunk_dur = float(audio_f32.size) / float(self.sample_rate)
        rms = compute_rms(audio_f32)
        is_speech = rms >= self.vad_rms_threshold

        if not self._speaking:
            if is_speech:
                self._speaking = True
                for pre_bytes, pre_dur in self._pre_roll:
                    self._chunks.append((pre_bytes, False, pre_dur))
                    self._total_dur += pre_dur
                self._pre_roll.clear()
                self._pre_roll_dur = 0.0

                self._chunks.append((pcm_bytes, True, chunk_dur))
                self._last_speech_idx = len(self._chunks) - 1
                self._total_dur += chunk_dur
                self._speech_dur += chunk_dur
                self._silence_dur = 0.0

                if self._total_dur >= self.max_utterance_sec:
                    return self._trim_and_emit()
                return None

            self._pre_roll.append((pcm_bytes, chunk_dur))
            self._pre_roll_dur += chunk_dur
            while self._pre_roll and self._pre_roll_dur > self.pre_roll_sec:
                _, dropped_dur = self._pre_roll.popleft()
                self._pre_roll_dur -= dropped_dur
            return None

        # Đang trong trạng thái thu một câu (SPEAKING)
        self._chunks.append((pcm_bytes, is_speech, chunk_dur))
        self._total_dur += chunk_dur
        if is_speech:
            self._last_speech_idx = len(self._chunks) - 1
            self._speech_dur += chunk_dur
            self._silence_dur = 0.0
        else:
            self._silence_dur += chunk_dur

        if self._silence_dur >= self.silence_duration_sec or self._total_dur >= self.max_utterance_sec:
            return self._trim_and_emit()

        return None


class InProcessFasterWhisperRecognizer:
    """Backend Faster-Whisper chạy trực tiếp trong Python hiện tại (khi đã cài faster_whisper)."""

    def __init__(
        self,
        model: str = 'small',
        model_path: str = '',
        cache_dir: str = '',
        device: str = 'cpu',
        compute_type: str = 'int8',
        language: str = 'vi',
        beam_size: int = 5,
        local_files_only: bool = True,
        vad_filter: bool = True,
        without_timestamps: bool = True,
        condition_on_previous_text: bool = False,
        cpu_threads: int = 0,
        initial_prompt: str = '',
    ):
        if local_files_only:
            os.environ.setdefault('HF_HUB_OFFLINE', '1')
            os.environ.setdefault('HF_DATASETS_OFFLINE', '1')

        from faster_whisper import WhisperModel

        target, _ = resolve_whisper_model_source(model, model_path)
        download_root = os.path.expanduser(cache_dir.strip()) if (cache_dir or '').strip() else None

        self.language = (language or 'vi').strip() or None
        self.beam_size = max(1, int(beam_size))
        self.vad_filter = bool(vad_filter)
        self.without_timestamps = bool(without_timestamps)
        self.condition_on_previous_text = bool(condition_on_previous_text)
        self.initial_prompt = (initial_prompt or '').strip() or None

        t0 = time.monotonic()
        self.model = WhisperModel(
            target,
            device=device,
            compute_type=compute_type,
            cpu_threads=int(cpu_threads),
            download_root=download_root,
            local_files_only=bool(local_files_only),
        )
        self.load_time_sec = time.monotonic() - t0

    def transcribe_float32(self, audio_f32: np.ndarray) -> Tuple[str, float]:
        arr = np.ascontiguousarray(audio_f32, dtype=np.float32)
        if is_digital_silence(arr):
            return '', 0.0

        t0 = time.monotonic()
        segments, _info = self.model.transcribe(
            arr,
            language=self.language,
            beam_size=self.beam_size,
            vad_filter=self.vad_filter,
            without_timestamps=self.without_timestamps,
            condition_on_previous_text=self.condition_on_previous_text,
            initial_prompt=self.initial_prompt,
        )
        text = clean_whisper_segments(segments)
        return text, time.monotonic() - t0

    def close(self):
        pass


class SubprocessFasterWhisperClient:
    """Quản lý worker subprocess sống lâu chạy trong venv để giữ mô hình nạp sẵn qua nhiều câu."""

    def __init__(
        self,
        python_executable: str,
        worker_script: Optional[str] = None,
        model: str = 'small',
        model_path: str = '',
        cache_dir: str = '',
        device: str = 'cpu',
        compute_type: str = 'int8',
        language: str = 'vi',
        beam_size: int = 5,
        local_files_only: bool = True,
        vad_filter: bool = True,
        without_timestamps: bool = True,
        condition_on_previous_text: bool = False,
        cpu_threads: int = 0,
        initial_prompt: str = '',
        startup_timeout_sec: float = 90.0,
        infer_timeout_sec: float = 30.0,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self._python_exec = os.path.expanduser(python_executable)
        if not os.path.exists(self._python_exec):
            raise FileNotFoundError(
                f'Không tìm thấy Python của venv Faster-Whisper tại: {self._python_exec}'
            )

        self._worker_script = worker_script or os.path.abspath(__file__)
        if not os.path.exists(self._worker_script):
            raise FileNotFoundError(f'Không tìm thấy faster_whisper_worker.py tại: {self._worker_script}')

        # Xác thực đường dẫn model cục bộ sớm nếu có
        resolve_whisper_model_source(model, model_path)

        self.infer_timeout_sec = max(1.0, float(infer_timeout_sec))
        self._logger = logger
        self._io_lock = threading.Lock()
        self._req_seq = 0
        self._closed = False
        self._stderr_tail = collections.deque(maxlen=40)

        cmd = [
            self._python_exec,
            '-u',
            self._worker_script,
            '--model', str(model),
            '--device', str(device),
            '--compute-type', str(compute_type),
            '--language', str(language),
            '--beam-size', str(int(beam_size)),
            '--cpu-threads', str(int(cpu_threads)),
        ]
        if (model_path or '').strip():
            cmd.extend(['--model-path', os.path.expanduser(model_path.strip())])
        if (cache_dir or '').strip():
            cmd.extend(['--cache-dir', os.path.expanduser(cache_dir.strip())])
        if (initial_prompt or '').strip():
            cmd.extend(['--initial-prompt', initial_prompt.strip()])
        if local_files_only:
            cmd.append('--local-files-only')
        else:
            cmd.append('--no-local-files-only')
        if vad_filter:
            cmd.append('--vad-filter')
        else:
            cmd.append('--no-vad-filter')
        if without_timestamps:
            cmd.append('--without-timestamps')
        else:
            cmd.append('--with-timestamps')
        if condition_on_previous_text:
            cmd.append('--condition-on-previous-text')
        else:
            cmd.append('--no-condition-on-previous-text')

        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        env['PYTHONIOENCODING'] = 'utf-8'
        if local_files_only:
            env['HF_HUB_OFFLINE'] = '1'
            env['HF_DATASETS_OFFLINE'] = '1'

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self._stderr_thread.start()

        ready_msg = self._read_json_line(timeout=startup_timeout_sec)
        if not ready_msg:
            stderr_info = ' | '.join(self._stderr_tail)
            self.close()
            raise RuntimeError(
                f'Worker Faster-Whisper thoát trước khi sẵn sàng (exit={self._proc.returncode}). '
                f'Chi tiết: {stderr_info or "không có stderr"}'
            )
        if ready_msg.get('event') != 'ready':
            err = ready_msg.get('message') or ready_msg.get('error') or str(ready_msg)
            self.close()
            raise RuntimeError(f'Lỗi khởi tạo Faster-Whisper: {err}')

        self.load_time_sec = float(ready_msg.get('load_time_sec', 0.0))

    def _drain_stderr(self):
        try:
            assert self._proc.stderr is not None
            for raw_line in iter(self._proc.stderr.readline, b''):
                line = raw_line.decode('utf-8', errors='replace').rstrip()
                if line:
                    self._stderr_tail.append(line)
        except Exception:
            pass

    def _read_json_line(self, timeout: float) -> Optional[dict]:
        result_q: queue.Queue = queue.Queue(maxsize=1)

        def _reader():
            try:
                assert self._proc.stdout is not None
                line = self._proc.stdout.readline()
                result_q.put(('line', line))
            except Exception as exc:
                result_q.put(('err', exc))

        t = threading.Thread(target=_reader, daemon=True)
        t.start()
        try:
            kind, payload = result_q.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f'Worker Faster-Whisper quá thời gian phản hồi ({timeout:.1f}s)')

        if kind == 'err':
            raise RuntimeError(f'Lỗi đọc từ worker Faster-Whisper: {payload}')
        if not payload:
            return None
        return json.loads(payload.decode('utf-8'))

    def transcribe_float32(self, audio_f32: np.ndarray) -> Tuple[str, float]:
        if self._closed:
            raise RuntimeError('Worker Faster-Whisper đã đóng')

        arr = np.ascontiguousarray(audio_f32, dtype=np.float32)
        if is_digital_silence(arr):
            return '', 0.0

        raw_bytes = arr.tobytes()
        with self._io_lock:
            if self._closed or self._proc.poll() is not None:
                raise RuntimeError(f'Worker Faster-Whisper không còn chạy (exit={self._proc.poll()})')

            self._req_seq += 1
            req_id = self._req_seq
            header = json.dumps({
                'cmd': 'transcribe',
                'id': req_id,
                'num_bytes': len(raw_bytes),
                'dtype': 'float32',
                'sample_rate': 16000,
            }) + '\n'

            assert self._proc.stdin is not None
            self._proc.stdin.write(header.encode('utf-8'))
            self._proc.stdin.write(raw_bytes)
            self._proc.stdin.flush()

            resp = self._read_json_line(timeout=self.infer_timeout_sec)
            if not resp:
                stderr_info = ' | '.join(self._stderr_tail)
                raise RuntimeError(
                    f'Worker Faster-Whisper đóng bất ngờ (exit={self._proc.poll()}): {stderr_info}'
                )
            if resp.get('id') != req_id:
                raise RuntimeError(f'Sai lệch req_id từ worker: expected={req_id}, got={resp.get("id")}')
            if not resp.get('ok', False):
                raise RuntimeError(resp.get('error', 'Lỗi nhận diện không xác định từ worker'))
            return resp.get('text', ''), float(resp.get('inference_sec', 0.0))

    def close(self):
        if self._closed:
            return
        self._closed = True
        proc = getattr(self, '_proc', None)
        if proc is None:
            return
        try:
            if proc.poll() is None and proc.stdin:
                try:
                    proc.stdin.write(b'{"cmd": "shutdown"}\n')
                    proc.stdin.flush()
                except Exception:
                    pass
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            if proc.poll() is None:
                try:
                    proc.wait(timeout=1.0)
                except (subprocess.TimeoutExpired, KeyboardInterrupt):
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.0)
                    except (subprocess.TimeoutExpired, KeyboardInterrupt):
                        proc.kill()
                        try:
                            proc.wait(timeout=0.5)
                        except (Exception, KeyboardInterrupt):
                            pass
        except (Exception, KeyboardInterrupt):
            if proc.poll() is None:
                try:
                    proc.kill()
                except Exception:
                    pass


def create_faster_whisper_recognizer(
    whisper_python: str = '',
    model: str = 'small',
    model_path: str = '',
    cache_dir: str = '',
    device: str = 'cpu',
    compute_type: str = 'int8',
    language: str = 'vi',
    beam_size: int = 5,
    local_files_only: bool = True,
    vad_filter: bool = True,
    without_timestamps: bool = True,
    condition_on_previous_text: bool = False,
    cpu_threads: int = 0,
    initial_prompt: str = '',
    infer_timeout_sec: float = 30.0,
    logger: Optional[Callable[[str], None]] = None,
):
    """Tạo bộ nhận diện Faster-Whisper phù hợp với môi trường Python hiện tại.

    - Nếu whisper_python trống: kiểm tra import trong Python hiện tại trước; nếu không có
      thì tự động dùng venv worker tại ~/.venvs/airport-whisper-test/bin/python.
    - Nếu whisper_python được chỉ định rõ: dùng đúng Python đó qua long-lived worker.
    """
    explicit_py = (whisper_python or '').strip()
    in_proc_ok = importlib.util.find_spec('faster_whisper') is not None

    if not explicit_py:
        if in_proc_ok:
            return InProcessFasterWhisperRecognizer(
                model=model,
                model_path=model_path,
                cache_dir=cache_dir,
                device=device,
                compute_type=compute_type,
                language=language,
                beam_size=beam_size,
                local_files_only=local_files_only,
                vad_filter=vad_filter,
                without_timestamps=without_timestamps,
                condition_on_previous_text=condition_on_previous_text,
                cpu_threads=cpu_threads,
                initial_prompt=initial_prompt,
            )
        if os.path.exists(DEFAULT_WHISPER_VENV_PYTHON):
            explicit_py = DEFAULT_WHISPER_VENV_PYTHON
        else:
            raise RuntimeError(
                'Python hiện tại chưa cài faster_whisper và không tìm thấy venv mặc định '
                f'({DEFAULT_WHISPER_VENV_PYTHON}). Hãy cấu hình tham số whisper_python.'
            )

    return SubprocessFasterWhisperClient(
        python_executable=explicit_py,
        model=model,
        model_path=model_path,
        cache_dir=cache_dir,
        device=device,
        compute_type=compute_type,
        language=language,
        beam_size=beam_size,
        local_files_only=local_files_only,
        vad_filter=vad_filter,
        without_timestamps=without_timestamps,
        condition_on_previous_text=condition_on_previous_text,
        cpu_threads=cpu_threads,
        initial_prompt=initial_prompt,
        infer_timeout_sec=infer_timeout_sec,
        logger=logger,
    )


def _read_exact(stream, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            raise EOFError(f'EOF khi đang đọc {n} bytes (mới nhận {len(buf)} bytes)')
        buf.extend(chunk)
    return bytes(buf)


def _write_proto_json(proto_out, obj: dict):
    proto_out.write((json.dumps(obj, ensure_ascii=False) + '\n').encode('utf-8'))
    proto_out.flush()


def _worker_main():
    # Chặn SIGINT in traceback khi parent Ctrl+C; parent sẽ đóng stdin hoặc SIGTERM gọn
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    parser = argparse.ArgumentParser(description='Long-lived Faster-Whisper worker')
    parser.add_argument('--model', default='small')
    parser.add_argument('--model-path', default='')
    parser.add_argument('--cache-dir', default='')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--compute-type', default='int8')
    parser.add_argument('--language', default='vi')
    parser.add_argument('--beam-size', type=int, default=5)
    parser.add_argument('--cpu-threads', type=int, default=0)
    parser.add_argument('--initial-prompt', default='')
    parser.add_argument('--local-files-only', dest='local_files_only', action='store_true', default=True)
    parser.add_argument('--no-local-files-only', dest='local_files_only', action='store_false')
    parser.add_argument('--vad-filter', dest='vad_filter', action='store_true', default=True)
    parser.add_argument('--no-vad-filter', dest='vad_filter', action='store_false')
    parser.add_argument('--without-timestamps', dest='without_timestamps', action='store_true', default=True)
    parser.add_argument('--with-timestamps', dest='without_timestamps', action='store_false')
    parser.add_argument('--condition-on-previous-text', dest='condition_on_previous_text', action='store_true', default=False)
    parser.add_argument('--no-condition-on-previous-text', dest='condition_on_previous_text', action='store_false')
    args = parser.parse_args()

    # Tách kênh giao thức stdout nhị phân và đẩy mọi log thư viện sang stderr
    proto_in = sys.stdin.buffer
    proto_out = sys.stdout.buffer
    sys.stdout = sys.stderr

    try:
        engine = InProcessFasterWhisperRecognizer(
            model=args.model,
            model_path=args.model_path,
            cache_dir=args.cache_dir,
            device=args.device,
            compute_type=args.compute_type,
            language=args.language,
            beam_size=args.beam_size,
            local_files_only=args.local_files_only,
            vad_filter=args.vad_filter,
            without_timestamps=args.without_timestamps,
            condition_on_previous_text=args.condition_on_previous_text,
            cpu_threads=args.cpu_threads,
            initial_prompt=args.initial_prompt,
        )
    except Exception as exc:
        msg = str(exc)
        if 'LocalEntryNotFoundError' in type(exc).__name__ or 'local_files_only' in msg:
            cache_disp = args.cache_dir or '~/.cache/huggingface/hub'
            msg = (
                f'Model "{args.model}" chưa có sẵn trong cache offline ({cache_disp}). '
                f'Hãy tải model trước khi chạy offline hoặc truyền whisper_local_files_only:=false. Chi tiết: {exc}'
            )
        _write_proto_json(proto_out, {
            'event': 'error',
            'error_type': type(exc).__name__,
            'message': msg,
        })
        sys.exit(1)

    _write_proto_json(proto_out, {
        'event': 'ready',
        'load_time_sec': round(engine.load_time_sec, 4),
    })

    while True:
        try:
            raw_line = proto_in.readline()
        except Exception:
            break
        if not raw_line:
            break

        try:
            req = json.loads(raw_line.decode('utf-8'))
        except Exception as exc:
            _write_proto_json(proto_out, {'ok': False, 'error': f'JSON không hợp lệ: {exc}'})
            continue

        cmd = req.get('cmd')
        if cmd == 'shutdown':
            break
        if cmd != 'transcribe':
            _write_proto_json(proto_out, {
                'id': req.get('id'),
                'ok': False,
                'error': f'Lệnh worker không hỗ trợ: {cmd}',
            })
            continue

        req_id = req.get('id')
        num_bytes = int(req.get('num_bytes', 0))
        dtype_str = req.get('dtype', 'float32')
        sample_rate = int(req.get('sample_rate', 16000))

        try:
            payload = _read_exact(proto_in, num_bytes)
            if dtype_str == 'float32':
                audio = np.frombuffer(payload, dtype='<f4')
            elif dtype_str == 'int16':
                audio = pcm16_bytes_to_float32(payload)
            else:
                raise ValueError(f'dtype không hỗ trợ: {dtype_str}')

            if sample_rate != 16000:
                audio = resample_float32(audio, sample_rate, 16000)

            text, infer_sec = engine.transcribe_float32(audio)
            _write_proto_json(proto_out, {
                'id': req_id,
                'ok': True,
                'text': text,
                'inference_sec': round(infer_sec, 4),
            })
        except Exception as exc:
            _write_proto_json(proto_out, {
                'id': req_id,
                'ok': False,
                'error': f'{type(exc).__name__}: {exc}',
            })


if __name__ == '__main__':
    _worker_main()
