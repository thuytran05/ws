#!/usr/bin/env python3
"""
tts_node.py — Chuyển văn bản thành tiếng nói (TTS) tiếng Việt offline.

Các engine hỗ trợ:
  - log    : Chế độ chỉ ghi log (log-only), dùng khi test trong Docker không có loa.
  - piper  : Offline chất lượng cao với model ONNX tiếng Việt.
  - espeak : Offline siêu nhẹ với espeak-ng / espeak.
  - gtts   : Online (cần Internet).

Publish:
  /voice/speaking (std_msgs/Bool)         -> Báo STT tạm ngưng nghe
  /voice/tts_ack  (std_msgs/String [JSON]) -> Xác nhận đã hoàn tất đọc câu nói
Subscribe:
  /voice/say      (std_msgs/String)        <- Nhận nội dung cần phát
"""

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


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('engine', 'log')               # log | piper | espeak | gtts
        self.declare_parameter('piper_bin', 'piper')
        self.declare_parameter('piper_model', '/opt/piper/vi_VN-vais1000-medium.onnx')
        self.declare_parameter('player', 'aplay')             # aplay | paplay | ffplay
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('normalize_codes', True)

        self.engine = self.get_parameter('engine').value
        self.piper_bin = self.get_parameter('piper_bin').value
        self.piper_model = self.get_parameter('piper_model').value
        self.player = self.get_parameter('player').value
        self.speed = float(self.get_parameter('speed').value)
        self.normalize_codes = bool(self.get_parameter('normalize_codes').value)

        self.q = queue.Queue()
        self.pub_speaking = self.create_publisher(Bool, '/voice/speaking', 10)
        self.pub_tts_ack = self.create_publisher(String, '/voice/tts_ack', 10)
        self.create_subscription(String, '/voice/say', self.on_say, 10)

        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'TTS sẵn sàng (engine={self.engine}).')

    def on_say(self, msg: String):
        text = msg.data.strip()
        if text:
            self.q.put(text)

    def worker(self):
        while rclpy.ok():
            raw_text = self.q.get()
            spoken_text = normalize_speech_text(raw_text) if self.normalize_codes else raw_text

            self.pub_speaking.publish(Bool(data=True))
            self.get_logger().info(f'NÓI: "{spoken_text}"')

            status = 'COMPLETED'
            try:
                self.synthesize(spoken_text)
            except Exception as e:
                self.get_logger().error(f'Lỗi TTS: {e}')
                status = f'ERROR_{type(e).__name__}'
            finally:
                self.pub_speaking.publish(Bool(data=False))
                ack_payload = {
                    'text': raw_text,
                    'spoken_text': spoken_text,
                    'status': status,
                    'timestamp': time.time()
                }
                self.pub_tts_ack.publish(String(data=json.dumps(ack_payload, ensure_ascii=False)))

    def synthesize(self, text: str):
        if self.engine == 'log':
            # Chế độ ghi log: mô phỏng thời gian nói ngắn và hoàn tất ngay
            self.get_logger().info(f'[TTS LOG ONLY] "{text}"')
            time.sleep(0.1)
            return

        if self.engine == 'espeak':
            bin_name = 'espeak-ng' if shutil.which('espeak-ng') else 'espeak'
            if not shutil.which(bin_name):
                self.get_logger().warn('Không tìm thấy espeak/espeak-ng. Chuyển sang log-only.')
                self.get_logger().info(f'[TTS LOG ONLY] "{text}"')
                return
            subprocess.run([bin_name, '-v', 'vi', '-s', str(int(160 * self.speed)), text],
                           check=False, timeout=10.0)
            return

        if self.engine == 'piper':
            if not shutil.which(self.piper_bin) or not os.path.exists(self.piper_model):
                self.get_logger().warn(
                    f'Không tìm thấy piper binary ({self.piper_bin}) hoặc model ({self.piper_model}). Chuyển sang log-only.')
                self.get_logger().info(f'[TTS LOG ONLY] "{text}"')
                return

            wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False).name
            try:
                subprocess.run(
                    [self.piper_bin, '--model', self.piper_model,
                     '--length_scale', str(round(1.0 / max(self.speed, 0.1), 2)),
                     '--output_file', wav],
                    input=text.encode('utf-8'), check=True, timeout=10.0)

                if shutil.which(self.player):
                    subprocess.run([self.player, '-q', wav], check=False, timeout=15.0)
                else:
                    self.get_logger().info(f'[TTS Playback LOG] Player {self.player} không có sẵn, đã tạo file {wav}')
            finally:
                if os.path.exists(wav):
                    os.remove(wav)
            return

        if self.engine == 'gtts':
            try:
                from gtts import gTTS
                mp3 = tempfile.NamedTemporaryFile(suffix='.mp3', delete=False).name
                gTTS(text=text, lang='vi').save(mp3)
                if shutil.which('ffplay'):
                    subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', mp3],
                                   check=False, timeout=15.0)
                os.remove(mp3)
            except Exception as e:
                self.get_logger().error(f'Lỗi gTTS: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
