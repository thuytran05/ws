#!/usr/bin/env python3
"""
stt_node.py — Nhận dạng tiếng nói thành văn bản tiếng Việt offline.
Hỗ trợ 3 chế độ:
  1. keyboard : Nhập văn bản qua bàn phím / terminal stdin.
  2. wav      : Đọc trực tiếp từ file WAV (kiểm thử lặp lại).
  3. mic      : Thu âm trực tiếp từ micro qua ALSA/sounddevice + Vosk.

Publish:
  /voice/text     (std_msgs/String)
Subscribe:
  /voice/speaking (std_msgs/Bool) -> Tạm ngưng thu âm khi robot đang nói, xả sạch buffer.
"""

import json
import os
import queue
import sys
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SttNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Khai báo tham số
        self.declare_parameter('mode', 'keyboard')          # keyboard | mic | wav
        self.declare_parameter('wav_file', '')              # Đường dẫn file wav khi mode=wav
        self.declare_parameter('model_path', '/opt/vosk/vosk-model-small-vn-0.4')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device', -1)                # Index thiết bị micro (-1 = mặc định)
        self.declare_parameter('wake_word', '')             # Để trống = luôn nghe
        self.declare_parameter('wake_timeout_sec', 15.0)    # Timeout trở về trạng thái chờ
        self.declare_parameter('min_chars', 2)
        self.declare_parameter('audio_queue_size', 50)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.mode = self.get_parameter('mode').value
        self.wav_file = self.get_parameter('wav_file').value
        self.model_path = self.get_parameter('model_path').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.device = int(self.get_parameter('device').value)
        self.wake_word = self.get_parameter('wake_word').value.lower().strip()
        self.wake_timeout = float(self.get_parameter('wake_timeout_sec').value)
        self.min_chars = int(self.get_parameter('min_chars').value)
        self.audio_queue_size = int(self.get_parameter('audio_queue_size').value)

        # Publishers & Subscribers
        self.pub_text = self.create_publisher(String, '/voice/text', 10)
        self.create_subscription(Bool, '/voice/speaking', self.on_speaking, 10)

        self.robot_speaking = False
        self.awake = (self.wake_word == '')
        self._last_wake_time = time.time()
        self._running = True
        self._audio_queue = queue.Queue(maxsize=self.audio_queue_size)

        # Khởi chạy luồng theo mode
        if self.mode == 'keyboard':
            self.get_logger().info('STT chế độ KEYBOARD: Gõ câu nói rồi nhấn Enter.')
            threading.Thread(target=self.keyboard_loop, daemon=True).start()
        elif self.mode == 'wav':
            self.get_logger().info(f'STT chế độ WAV: Đọc file {self.wav_file}')
            threading.Thread(target=self.wav_loop, daemon=True).start()
        elif self.mode == 'mic':
            self.get_logger().info('STT chế độ MICRO: Khởi động Vosk offline...')
            threading.Thread(target=self.mic_loop, daemon=True).start()
        else:
            self.get_logger().error(f'Mode không hợp lệ: {self.mode}. Dùng keyboard, mic hoặc wav.')

    def destroy_node(self):
        self._running = False
        super().destroy_node()

    def on_speaking(self, msg: Bool):
        self.robot_speaking = msg.data
        if self.robot_speaking:
            # Xả hết buffer audio đọng khi robot đang nói để tránh nhận nhầm tiếng robot
            while not self._audio_queue.empty():
                try:
                    self._audio_queue.get_nowait()
                except queue.Empty:
                    break

    def publish_text(self, text: str):
        text = text.strip()
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
                    self.get_logger().info('Wake word đã được kích hoạt.')
                    if not text:
                        self.pub_text.publish(String(data='__wake__'))
                        return
                else:
                    return
            else:
                # Kiểm tra timeout
                if now - self._last_wake_time > self.wake_timeout:
                    self.awake = False
                    self.get_logger().info('Hết thời gian tương tác (wake timeout), trở về chờ wake word.')
                    return
                else:
                    self._last_wake_time = now

        self.get_logger().info(f'NGHE ĐƯỢC: "{text}"')
        self.pub_text.publish(String(data=text))

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
        if not self.wav_file or not os.path.exists(self.wav_file):
            self.get_logger().error(f'File WAV không tồn tại: {self.wav_file}')
            return

        try:
            wf = wave.open(self.wav_file, 'rb')
        except Exception as e:
            self.get_logger().error(f'Lỗi mở file WAV: {e}')
            return

        if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
            self.get_logger().error(
                f'WAV phải là mono 16-bit PCM! (Kênh: {wf.getnchannels()}, Mẫu: {wf.getsampwidth()*8}-bit)')
            wf.close()
            return

        wav_rate = wf.getframerate()

        try:
            from vosk import Model, KaldiRecognizer, SetLogLevel
            SetLogLevel(-1)
            model = Model(self.model_path)
            rec = KaldiRecognizer(model, wav_rate)
        except Exception as e:
            self.get_logger().warn(
                f'Không thể nạp Vosk cho file WAV ({e}). Đang phát văn bản mô phỏng nếu có.')
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
            self.get_logger().info('Đã xử lý xong file WAV.')
        except Exception as e:
            self.get_logger().error(f'Lỗi đọc file WAV: {e}')
            try:
                wf.close()
            except Exception:
                pass
    def mic_loop(self):
        try:
            import sounddevice as sd
            from vosk import Model, KaldiRecognizer, SetLogLevel
        except ImportError as e:
            self.get_logger().error(
                f'Thiếu thư viện âm thanh: {e}. '
                'Để kiểm thử offline không cần micro, dùng mode:=keyboard hoặc mode:=wav.')
            return

        SetLogLevel(-1)

        try:
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f'Đường dẫn model không tồn tại: {self.model_path}')
            model = Model(self.model_path)
        except Exception as e:
            self.get_logger().error(
                f'Không nạp được model Vosk tại {self.model_path}: {e}. '
                'Vui lòng tải model hoặc dùng mode:=keyboard.')
            return

        rec = KaldiRecognizer(model, self.sample_rate)
        rec.SetWords(False)

        def audio_callback(indata, frames, time_info, status):
            if status:
                self.get_logger().warn(str(status))
            try:
                self._audio_queue.put_nowait(bytes(indata))
            except queue.Full:
                pass  # Bỏ qua chunk cũ khi hàng đợi đầy để tránh tràn bộ nhớ

        dev = None if self.device < 0 else self.device
        self.get_logger().info('Micro ALSA đã sẵn sàng. Bắt đầu lắng nghe...')

        try:
            with sd.RawInputStream(samplerate=self.sample_rate, blocksize=4000,
                                   device=dev, dtype='int16',
                                   channels=1, callback=audio_callback):
                while self._running and rclpy.ok():
                    try:
                        data = self._audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if self.robot_speaking:
                        rec.Reset()
                        continue

                    if rec.AcceptWaveform(data):
                        res = json.loads(rec.Result())
                        self.publish_text(res.get('text', ''))
        except Exception as e:
            self.get_logger().error(f'Lỗi thiết bị âm thanh micro: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
