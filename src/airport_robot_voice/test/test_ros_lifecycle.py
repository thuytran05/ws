#!/usr/bin/env python3
"""
Integration test kiểm tra giao tiếp ROS 2 độc lập và vòng đời state machine.
Chạy cô lập bằng rclpy, không can thiệp vào Gazebo hay robot thực tế.
"""

import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool

from airport_robot_voice.intent_node import IntentNode
from airport_robot_voice.tts_node import TtsNode
from airport_robot_voice.nav_command_node import NavCommandNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_intent_node_standalone_mode(ros_context):
    """Kiểm thử IntentNode ở chế độ Voice độc lập (khi nav_command_node chưa chạy)."""
    intent_node = IntentNode()
    test_node = Node('test_listener_node')

    say_msgs = []
    intent_msgs = []

    test_node.create_subscription(String, '/voice/say', lambda m: say_msgs.append(m.data), 10)
    test_node.create_subscription(String, '/voice/intent', lambda m: intent_msgs.append(json.loads(m.data)), 10)
    pub_text = test_node.create_publisher(String, '/voice/text', 10)

    # Đợi 0.2s để kết nối discovery
    for _ in range(5):
        rclpy.spin_once(intent_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)

    # 1. Gửi câu hỏi Wi-Fi (nhánh thông tin offline)
    pub_text.publish(String(data="Wifi sân bay tên gì"))
    for _ in range(10):
        rclpy.spin_once(intent_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if say_msgs:
            break

    assert len(say_msgs) > 0
    assert "Airport-Free-WiFi" in say_msgs[-1]

    # 2. Gửi yêu cầu dẫn tới cổng A3 (nhánh dẫn đường khi Nav chưa chạy)
    # Xả hết tin nhắn đọng trong queue
    for _ in range(5):
        rclpy.spin_once(intent_node, timeout_sec=0.02)
        rclpy.spin_once(test_node, timeout_sec=0.02)
    say_msgs.clear()
    intent_msgs.clear()
    pub_text.publish(String(data="Dẫn tôi đến cổng A3"))
    for _ in range(10):
        rclpy.spin_once(intent_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if intent_msgs and say_msgs:
            break

    assert len(intent_msgs) == 1
    assert intent_msgs[0]['action'] == 'navigate'
    assert intent_msgs[0]['target_id'] == 'gate_a3'
    assert intent_msgs[0]['schema_version'] == '1.0'

    assert len(say_msgs) == 1
    assert "cổng A3" in say_msgs[0]
    assert "chưa sẵn sàng" in say_msgs[0]

    intent_node.destroy_node()
    test_node.destroy_node()


def test_tts_node_log_mode(ros_context):
    """Kiểm thử TTS Node ở chế độ log-only: đảm bảo speaking state và tts_ack hoạt động."""
    tts_node = TtsNode()
    test_node = Node('test_tts_listener')

    speaking_states = []
    ack_msgs = []

    test_node.create_subscription(Bool, '/voice/speaking', lambda m: speaking_states.append(m.data), 10)
    test_node.create_subscription(String, '/voice/tts_ack', lambda m: ack_msgs.append(json.loads(m.data)), 10)
    pub_say = test_node.create_publisher(String, '/voice/say', 10)

    # Gửi câu nói vào /voice/say
    pub_say.publish(String(data="Xin chào quý khách"))

    for _ in range(15):
        rclpy.spin_once(tts_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if ack_msgs:
            break

    assert len(ack_msgs) >= 1
    assert ack_msgs[0]['status'] == 'COMPLETED'
    assert True in speaking_states
    assert False in speaking_states

    tts_node.destroy_node()
    test_node.destroy_node()


def test_nav_command_node_state_publication_and_controls(ros_context):
    """Kiểm thử NavCommandNode: công bố /voice/nav_state và phản hồi các lệnh điều khiển khi rảnh."""
    nav_node = NavCommandNode()
    test_node = Node('test_nav_listener')

    qos_transient = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
    )

    nav_states = []
    say_msgs = []

    test_node.create_subscription(String, '/voice/nav_state', lambda m: nav_states.append(json.loads(m.data)), qos_transient)
    test_node.create_subscription(String, '/voice/say', lambda m: say_msgs.append(m.data), 10)
    pub_intent = test_node.create_publisher(String, '/voice/intent', 10)

    for _ in range(5):
        rclpy.spin_once(nav_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)

    assert len(nav_states) >= 1
    assert nav_states[-1]['node_online'] is True
    assert nav_states[-1]['mission_state'] == 'IDLE'

    # Gửi lệnh pause khi IDLE
    say_msgs.clear()
    pause_payload = {'action': 'pause', 'request_id': 'req_test_pause'}
    pub_intent.publish(String(data=json.dumps(pause_payload)))

    for _ in range(10):
        rclpy.spin_once(nav_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if say_msgs:
            break

    assert len(say_msgs) >= 1
    assert "không có chuyến đi nào" in say_msgs[0].lower()

    # Gửi lệnh cancel khi IDLE
    say_msgs.clear()
    cancel_payload = {'action': 'cancel', 'request_id': 'req_test_cancel'}
    pub_intent.publish(String(data=json.dumps(cancel_payload)))

    for _ in range(10):
        rclpy.spin_once(nav_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if say_msgs:
            break

    assert len(say_msgs) >= 1
    assert "không có chuyến đi nào" in say_msgs[0].lower()

    # Gửi lệnh status khi IDLE
    say_msgs.clear()
    status_payload = {'action': 'status', 'request_id': 'req_test_status'}
    pub_intent.publish(String(data=json.dumps(status_payload)))

    for _ in range(10):
        rclpy.spin_once(nav_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if say_msgs:
            break

    assert len(say_msgs) >= 1
    assert "chưa có chuyến đi nào" in say_msgs[0].lower() or "chờ" in say_msgs[0].lower()

    nav_node.destroy_node()
    test_node.destroy_node()
