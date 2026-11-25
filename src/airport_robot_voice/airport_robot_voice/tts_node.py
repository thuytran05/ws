#!/usr/bin/env python3
"""
tts_node.py — Chuyển văn bản thành tiếng nói (TTS) tiếng Việt offline.

Các engine hỗ trợ:
  - log    : Chế độ chỉ ghi log (log-only), dùng khi test trong Docker không có loa.
  - piper  : Offline chất lượng cao với model ONNX tiếng Việt.
  - espeak : Offline siêu nhẹ với espeak-ng / espeak.
  - gtts   : Online (dự phòng khi có Internet).

Trình phát hỗ trợ cho piper:
  - aplay  : Phát ALSA trực tiếp (kèm cờ '-q').
  - paplay : Phát qua PulseAudio/PIPEWIRE (dùng PULSE_SERVER từ môi trường, không truyền '-q').
  - ffplay : Phát qua ffmpeg/ffplay.

Publish:
  /voice/speaking (std_msgs/Bool)         -> Báo STT tạm ngưng nghe
  /voice/tts_ack  (std_msgs/String [JSON]) -> Xác nhận đã hoàn tất đọc câu nói kèm status chi tiết
Subscribe:
  /voice/say      (std_msgs/String)        <- Nhận nội dung cần phát (plain text hoặc JSON)
"""

import collections
import json
import os
import queue
import re
import shutil
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Bool


def normalize_speech_text(text: str) -> str:
    """Chuẩn hóa cách phát âm các mã cổng cho giọng đọc tự nhiên.
    VD: 'cổng A3' -> 'cổng a ba', 'cổng B2' -> 'cổng bê hai', 'cổng 15' -> 'cổng mười lăm'.
    """
    t = text
    t = re.sub(r'\bcổng\s+A1\b', 'cổng a một', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+A2\b', 'cổng a hai', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+A3\b', 'cổng a ba', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+A4\b', 'cổng a bốn', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+B1\b', 'cổng bê một', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+B2\b', 'cổng bê hai', t, flags=re.IGNORECASE)
    t = re.sub(r'\bcổng\s+15\b', 'cổng mười lăm', t, flags=re.IGNORECASE)
    return t


def build_player_command(player: str, wav_path: str) -> list:
    """Tạo đối số dòng lệnh phù hợp cho từng trình phát âm thanh.
    - aplay  : truyền '-q' để giảm log ALSA.
    - paplay : truyền trực tiếp wav_path (paplay không nhận cờ '-q').
    - ffplay : truyền '-nodisp -autoexit -loglevel quiet'.
    """
    player_str = (player or '').strip()
    base = os.path.basename(player_str)
    if base == 'aplay':
        return [player_str, '-q', wav_path]
    if base == 'paplay':
        return [player_str, wav_path]
    if base == 'ffplay':
        return [player_str, '-nodisp', '-autoexit', '-loglevel', 'quiet', wav_path]
    return [player_str, wav_path]


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('engine', 'piper')             # piper | paplay/aplay | log | espeak | gtts
        self.declare_parameter('piper_bin', 'piper')
        self.declare_parameter('piper_model', '/opt/piper/vi_VN-vais1000-medium.onnx')
        self.declare_parameter('player', 'paplay')            # paplay | aplay | ffplay
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('normalize_codes', True)
        self.declare_parameter('queue_size', 30)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.engine = self.get_parameter('engine').value
        self.piper_bin = self.get_parameter('piper_bin').value
        self.piper_model = self.get_parameter('piper_model').value
        self.player = self.get_parameter('player').value
        self.speed = float(self.get_parameter('speed').value)
        self.normalize_codes = bool(self.get_parameter('normalize_codes').value)
        self.queue_size = int(self.get_parameter('queue_size').value)

        self.q = queue.Queue(maxsize=self.queue_size)
        self._running = True
        self._cancelled_speeches = collections.OrderedDict()  # speech_id -> cancel_time
        self._cancelled_missions = collections.OrderedDict()  # mission_id -> cancel_time

        self.pub_speaking = self.create_publisher(Bool, '/voice/speaking', 10)
        self.pub_tts_ack = self.create_publisher(String, '/voice/tts_ack', 10)
        self.create_subscription(String, '/voice/say', self.on_say, 10)
        self.create_subscription(String, '/voice/tts_cancel', self.on_tts_cancel, 10)

        self._worker_thread = threading.Thread(target=self.worker, daemon=True)
        self._worker_thread.start()
        self.get_logger().info(
            f'TTS sẵn sàng (engine={self.engine}, player={self.player}, queue_size={self.queue_size}).'
        )

    def destroy_node(self):
        self._running = False
        try:
            self.q.put_nowait(None)
        except Exception:
            pass
        if hasattr(self, '_worker_thread') and self._worker_thread.is_alive():
            try:
                self._worker_thread.join(timeout=1.5)
            except (Exception, KeyboardInterrupt):
                pass
        try:
            super().destroy_node()
        except (Exception, KeyboardInterrupt):
            pass

    def on_tts_cancel(self, msg: String):
        data_str = msg.data.strip()
        if not data_str:
            return
        now = time.time()
        try:
            data = json.loads(data_str)
            action = data.get('action')
            sp_id = data.get('speech_id')
            m_id = data.get('mission_id')
            if action == 'clear_mission_cancel' and m_id:
                self._cancelled_missions.pop(m_id, None)
            else:
                if sp_id:
                    self._cancelled_speeches[sp_id] = now
                if m_id:
                    self._cancelled_missions[m_id] = now
        except Exception:
            self._cancelled_speeches[data_str] = now
            self._cancelled_missions[data_str] = now

        while len(self._cancelled_speeches) > 200:
            self._cancelled_speeches.popitem(last=False)
        while len(self._cancelled_missions) > 200:
            self._cancelled_missions.popitem(last=False)
        self.get_logger().info(f"Đã nhận yêu cầu hủy câu nói TTS: {data_str}")

    def on_say(self, msg: String):
        data_str = msg.data.strip()
        if not data_str:
            return

        speech_id = None
        mission_id = None
        request_id = None
        text = data_str

        # Thử parse JSON nếu đầu vào có cấu trúc
        if data_str.startswith('{') and data_str.endswith('}'):
            try:
                payload = json.loads(data_str)
                text = payload.get('text', data_str)
                speech_id = payload.get('speech_id')
                mission_id = payload.get('mission_id')
                request_id = payload.get('request_id')
            except Exception:
                text = data_str

        if not speech_id:
            speech_id = f"sp_{int(time.time() * 1000)}"

        enqueued_at = time.time()
        try:
            self.q.put_nowait((text, speech_id, mission_id, request_id, enqueued_at))
        except queue.Full:
            self.get_logger().error(f'Hàng đợi TTS đầy ({self.queue_size}), từ chối câu nói mới: {text}')
            ack_payload = {
                'speech_id': speech_id,
                'mission_id': mission_id,
                'request_id': request_id,
                'text': text,
                'spoken_text': text,
                'status': 'DROPPED_QUEUE_FULL',
                'engine': self.engine,
                'wav_generated': False,
                'audio_played': False,
                'timestamp': time.time()
            }
            self.pub_tts_ack.publish(String(data=json.dumps(ack_payload, ensure_ascii=False)))

    def worker(self):
        while self._running and rclpy.ok():
            try:
                item = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:
                break

            raw_text, speech_id, mission_id, request_id, enqueued_at = item
            is_cancelled = False
            if speech_id in self._cancelled_speeches:
                is_cancelled = True
            elif mission_id and mission_id in self._cancelled_missions:
                cancel_time = self._cancelled_missions[mission_id]
                if enqueued_at <= cancel_time:
                    is_cancelled = True

            if is_cancelled:
                self.get_logger().info(f"Bỏ qua câu nói đã bị hủy trong hàng đợi: [{speech_id}] '{raw_text}'")
                ack_payload = {
                    'speech_id': speech_id,
                    'mission_id': mission_id,
                    'request_id': request_id,
                    'text': raw_text,
                    'spoken_text': raw_text,
                    'status': 'CANCELLED_BEFORE_PLAYBACK',
                    'engine': self.engine,
                    'wav_generated': False,
                    'audio_played': False,
                    'timestamp': time.time()
                }
                self.pub_tts_ack.publish(String(data=json.dumps(ack_payload, ensure_ascii=False)))
                continue

            spoken_text = normalize_speech_text(raw_text) if self.normalize_codes else raw_text

            self.pub_speaking.publish(Bool(data=True))
            self.get_logger().info(f'[{speech_id}] NÓI: "{spoken_text}"')

            res_info = {'status': 'COMPLETED', 'wav_generated': False, 'audio_played': False}
            try:
                res_info = self.synthesize(spoken_text)
            except Exception as e:
                self.get_logger().error(f'Lỗi ngoại lệ TTS: {e}')
                res_info['status'] = f'ERROR_{type(e).__name__}'
            finally:
                if rclpy.ok():
                    try:
                        self.pub_speaking.publish(Bool(data=False))
                        ack_payload = {
                            'speech_id': speech_id,
                            'mission_id': mission_id,
                            'request_id': request_id,
                            'text': raw_text,
                            'spoken_text': spoken_text,
                            'status': res_info.get('status', 'COMPLETED'),
                            'engine': self.engine,
                            'wav_generated': res_info.get('wav_generated', False),
                            'audio_played': res_info.get('audio_played', False),
                            'timestamp': time.time()
                        }
                        self.pub_tts_ack.publish(String(data=json.dumps(ack_payload, ensure_ascii=False)))
                    except Exception:
                        pass

    def synthesize(self, text: str) -> dict:
        if self.engine == 'log':
            # Chế độ ghi log: mô phỏng thời gian phát âm ngắn và hoàn tất ngay
            self.get_logger().info(f'[TTS LOG ONLY] "{text}"')
            time.sleep(0.05)
            return {'status': 'COMPLETED', 'wav_generated': False, 'audio_played': False}

        if self.engine == 'espeak':
            bin_name = 'espeak-ng' if shutil.which('espeak-ng') else 'espeak'
            if not shutil.which(bin_name):
                self.get_logger().warn('Không tìm thấy espeak/espeak-ng. Chuyển sang log-only fallback.')
                self.get_logger().info(f'[TTS LOG ONLY] "{text}"')
                return {'status': 'WARNING_ESPEAK_MISSING_FALLBACK_LOG', 'wav_generated': False, 'audio_played': False}

            cmd = [bin_name, '-v', 'vi', '-s', str(int(160 * self.speed)), text]
            p = subprocess.run(cmd, capture_output=True, timeout=10.0)
            if p.returncode == 0:
                return {'status': 'COMPLETED', 'wav_generated': False, 'audio_played': True}
            else:
                return {'status': f'ERROR_ESPEAK_EXIT_{p.returncode}', 'wav_generated': False, 'audio_played': False}

        if self.engine == 'piper':
            piper_exec = shutil.which(self.piper_bin) or self.piper_bin
            if not os.path.exists(piper_exec) and not shutil.which(piper_exec):
                self.get_logger().warn(f'Không tìm thấy piper binary ({piper_exec}). Fallback log-only.')
                return {'status': 'ERROR_PIPER_BINARY_MISSING', 'wav_generated': False, 'audio_played': False}

            if not os.path.exists(self.piper_model):
                self.get_logger().warn(f'Không tìm thấy model Piper ({self.piper_model}). Fallback log-only.')
                return {'status': 'ERROR_PIPER_MODEL_MISSING', 'wav_generated': False, 'audio_played': False}

            wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
            try:
                cmd = [
                    piper_exec,
                    '--model', self.piper_model,
                    '--length_scale', str(round(1.0 / max(self.speed, 0.1), 2)),
                    '--output_file', wav
                ]
                p = subprocess.run(cmd, input=text.encode('utf-8'), capture_output=True, timeout=10.0)
                if p.returncode != 0 or not os.path.exists(wav) or os.path.getsize(wav) < 44:
                    return {'status': f'ERROR_PIPER_SYNTHESIS_{p.returncode}', 'wav_generated': False, 'audio_played': False}

                # Tổng hợp WAV thành công
                wav_ok = True
                audio_ok = False

                player_resolved = shutil.which(self.player)
                if not player_resolved and os.path.isfile(self.player) and os.access(self.player, os.X_OK):
                    player_resolved = self.player

                if player_resolved:
                    play_cmd = build_player_command(self.player, wav)
                    play_p = subprocess.run(play_cmd, capture_output=True, timeout=15.0)
                    if play_p.returncode == 0:
                        audio_ok = True
                        status_str = 'COMPLETED'
                    else:
                        err_msg = (play_p.stderr or b'').decode('utf-8', errors='replace').strip()
                        pulse_env = os.environ.get('PULSE_SERVER', '(unset)')
                        self.get_logger().warn(
                            f'Lỗi phát âm qua {self.player} (exit={play_p.returncode}, PULSE_SERVER={pulse_env}): {err_msg}'
                        )
                        status_str = f'WARNING_PLAYBACK_EXIT_{play_p.returncode}'
                else:
                    self.get_logger().warn(f'Không tìm thấy trình phát âm thanh "{self.player}".')
                    status_str = 'WARNING_NO_PLAYER'

                return {'status': status_str, 'wav_generated': wav_ok, 'audio_played': audio_ok}
            finally:
                if os.path.exists(wav):
                    try:
                        os.remove(wav)
                    except OSError:
                        pass

        if self.engine == 'gtts':
            try:
                from gtts import gTTS
                mp3 = tempfile.NamedTemporaryFile(suffix='.mp3', delete=False).name
                gTTS(text=text, lang='vi').save(mp3)
                played = False
                if shutil.which('ffplay'):
                    p = subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', mp3],
                                       capture_output=True, timeout=15.0)
                    played = (p.returncode == 0)
                if os.path.exists(mp3):
                    os.remove(mp3)
                return {'status': 'COMPLETED', 'wav_generated': True, 'audio_played': played}
            except Exception as e:
                return {'status': f'ERROR_GTTS_{type(e).__name__}', 'wav_generated': False, 'audio_played': False}

        return {'status': f'UNKNOWN_ENGINE_{self.engine}', 'wav_generated': False, 'audio_played': False}


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
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
