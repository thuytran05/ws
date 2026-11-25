#!/usr/bin/env python3
"""
intent_node.py — "Bộ não" hiểu ý định người dùng bằng NLU tiếng Việt offline.
Không sử dụng LLM.

Subscribe:
  /voice/text        (std_msgs/String)         <- từ stt_node (keyboard, mic, wav)
  /voice/nav_state   (std_msgs/String, QoS TL) <- trạng thái từ nav_command_node
Publish:
  /voice/say         (std_msgs/String)         -> tts_node đọc
  /voice/intent      (std_msgs/String [JSON])  -> nav_command_node nhận lệnh

Hỗ trợ 2 chế độ:
- Voice độc lập: khi nav_command_node chưa chạy, vẫn nhận diện đầy đủ target_id,
  phát payload JSON và thông báo hệ thống điều hướng chưa sẵn sàng.
- Voice kết hợp navigation: khi nav_command_node đang chạy, chuyển giao việc
  thông báo bắt đầu/kết thúc cho nav_command_node để tránh trùng lặp câu nói.
"""

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from airport_robot_voice.nlu import NluEngine


class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')

        # Tìm thư mục config chuẩn qua ament_index_python
        try:
            from ament_index_python.packages import get_package_share_directory
            share = os.path.join(get_package_share_directory('airport_robot_voice'), 'config')
        except Exception:
            share = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')

        self.declare_parameter('intents_file', os.path.join(share, 'intents.json'))
        self.declare_parameter('places_file', os.path.join(share, 'places.json'))
        self.declare_parameter('nav_timeout_sec', 3.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        intents_path = self.get_parameter('intents_file').value
        places_path = self.get_parameter('places_file').value
        self.nav_timeout = float(self.get_parameter('nav_timeout_sec').value)

        self.engine = NluEngine(intents_path, places_path)
        self._seq = 0
        self._last_nav_heartbeat = 0.0
        self._last_nav_state = {}

        # QoS
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Topic publishers & subscribers
        self.pub_say = self.create_publisher(String, '/voice/say', 10)
        self.pub_intent = self.create_publisher(String, '/voice/intent', 10)

        self.create_subscription(String, '/voice/text', self.on_text, 10)
        self.create_subscription(String, '/voice/nav_state', self.on_nav_state, qos_transient)

        self.get_logger().info(
            f'Intent node sẵn sàng: {len(self.engine.cfg["intents"])} intents, '
            f'{len(self.engine.places)} places, {len(self.engine.categories)} categories.')

    @property
    def is_nav_online(self) -> bool:
        """Kiểm tra nav_command_node có đang hoạt động hay không."""
        return (
            self._last_nav_state.get('node_online', False)
            and (time.monotonic() - self._last_nav_heartbeat) < self.nav_timeout
        )

    def on_nav_state(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._last_nav_heartbeat = time.monotonic()
            self._last_nav_state = data
        except Exception as e:
            self.get_logger().warn(f'Lỗi giải mã /voice/nav_state: {e}')

    def on_text(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return

        if raw == '__wake__':
            self.pub_say.publish(String(data='Vâng, tôi nghe đây.'))
            return

        self._seq += 1
        req_id = f'req_{int(time.time()*1000)}_{self._seq}'

        r = self.engine.process(raw)
        if not r:
            return

        self.get_logger().info(
            f'[{req_id}] "{raw}" -> intent={r["intent"]} (score={r["score"]:.1f}) '
            f'action={r["action"]} target={r["target"]} cat={r["category"]}')

        # Đóng gói JSON payload theo schema quy chuẩn
        intent_payload = {
            'schema_version': '1.0',
            'request_id': req_id,
            'timestamp': time.time(),
            'raw': raw,
            'intent': r['intent'],
            'action': r['action'],
            'score': round(r['score'], 1),
            'target_id': r['target'],
            'category': r['category'],
            'selection_policy': r['selection_policy'],
            'slots': r['slots']
        }

        # Luôn publish /voice/intent cho các thành phần điều hướng / giám sát
        self.pub_intent.publish(String(data=json.dumps(intent_payload, ensure_ascii=False)))

        # Phân nhánh phát tiếng (say):
        # 1. Các hành động liên quan tới navigation (navigate, pause, resume, cancel, status)
        if r['action'] in ('navigate', 'replace', 'pause', 'resume', 'cancel', 'status'):
            if self.is_nav_online:
                # Nav node đang hoạt động: nhường quyền phát thông báo cho nav node
                self.get_logger().info(
                    f'Nav gateway đang online -> chuyển quyền phát ngôn cho nav_command_node.')
            else:
                # Chế độ voice độc lập: thông báo rõ trạng thái
                if r['action'] in ('navigate', 'replace'):
                    target_name = 'điểm đến'
                    if r['target'] and r['target'] in self.engine.places:
                        target_name = self.engine.places[r['target']].get('name', r['target'])
                    elif r['category'] and r['category'] in self.engine.categories:
                        target_name = self.engine.categories[r['category']].get('name', r['category'])

                    speech = f'Đã xác định yêu cầu tới {target_name}, nhưng hệ thống điều hướng chưa sẵn sàng.'
                    self.get_logger().info(f'[Voice độc lập] {speech}')
                    self.pub_say.publish(String(data=speech))
                elif r['action'] == 'status':
                    speech = 'Hệ thống điều hướng hiện chưa sẵn sàng.'
                    self.pub_say.publish(String(data=speech))
                else:
                    speech = 'Hiện không có chuyến đi nào đang hoạt động.'
                    self.pub_say.publish(String(data=speech))
        else:
            # 2. Các intent thông tin offline (greeting, thanks, capability, wifi, hours, help...)
            # hoặc các trường hợp từ chối (place_not_found, ambiguous_target, missing_destination, negated)
            speech = r['text']
            self.pub_say.publish(String(data=speech))


def main(args=None):
    rclpy.init(args=args)
    node = IntentNode()
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
