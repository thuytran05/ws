import json
import os
import threading
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from airport_robot_voice.intent_node import IntentNode
from airport_robot_voice.nav_command_node import MissionState, NavCommandNode


@pytest.fixture(scope='module')
def ros_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin_until(executor, predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_ui_catalog_idempotency_busy_guard_and_nlu_correlation(ros_ctx):
    nav_node = NavCommandNode()
    intent_node = IntentNode()
    client = Node('ui_test_client')

    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    qos_rel = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)

    catalogs = []
    ui_statuses = []
    feedbacks = []

    client.create_subscription(String, '/voice/places_catalog', lambda m: catalogs.append(json.loads(m.data)), qos_tl)
    client.create_subscription(String, '/voice/ui_status', lambda m: ui_statuses.append(json.loads(m.data)), qos_tl)
    client.create_subscription(String, '/voice/feedback', lambda m: feedbacks.append(json.loads(m.data)), qos_rel)
    pub_req = client.create_publisher(String, '/voice/ui_request', qos_rel)

    ex = SingleThreadedExecutor()
    ex.add_node(nav_node)
    ex.add_node(intent_node)
    ex.add_node(client)

    try:
        assert _spin_until(ex, lambda: len(catalogs) >= 1 and len(ui_statuses) >= 1, timeout=3.0)
        cat = catalogs[-1]
        assert cat['read_only'] is True
        assert cat['count'] == len(nav_node.places)
        assert any(p['id'] == 'gate_a3' for p in cat['places'])

        # 1. Unknown NLU text via /voice/ui_request -> correlated REJECTED
        pub_req.publish(String(data=json.dumps({
            'client_request_id': 'c-bad-nlu-1',
            'type': 'text_nlu',
            'text': 'xin chào trời hôm nay đẹp quá',
        })))
        assert _spin_until(
            ex,
            lambda: any(f.get('client_request_id') == 'c-bad-nlu-1' and f.get('event') == 'REJECTED' for f in feedbacks),
            timeout=3.0,
        )

        # 2. Retry same client_request_id -> duplicate=True cached response
        prev_fb_len = len(feedbacks)
        pub_req.publish(String(data=json.dumps({
            'client_request_id': 'c-bad-nlu-1',
            'type': 'text_nlu',
            'text': 'xin chào trời hôm nay đẹp quá',
        })))
        assert _spin_until(
            ex,
            lambda: len(feedbacks) > prev_fb_len and feedbacks[-1].get('duplicate') is True,
            timeout=3.0,
        )

        # 3. Busy guard: when state != IDLE, navigate_id must NOT trigger replace
        nav_node.state = MissionState.NAVIGATING
        nav_node._current_target_key = 'gate_a3'
        pub_req.publish(String(data=json.dumps({
            'client_request_id': 'c-busy-1',
            'type': 'navigate_id',
            'destination_id': 'gate_a4',
        })))
        assert _spin_until(
            ex,
            lambda: any(
                f.get('client_request_id') == 'c-busy-1'
                and f.get('event') == 'REJECTED'
                and f.get('status') == 'BUSY_MISSION_ACTIVE'
                for f in feedbacks
            ),
            timeout=3.0,
        )
        assert nav_node._current_target_key == 'gate_a3'

        # 4. Escort mode rejects pause cleanly
        nav_node.enable_escort = True
        pub_req.publish(String(data=json.dumps({
            'client_request_id': 'c-pause-escort',
            'type': 'pause',
        })))
        assert _spin_until(
            ex,
            lambda: any(
                f.get('client_request_id') == 'c-pause-escort'
                and f.get('event') == 'REJECTED'
                and f.get('status') == 'PAUSE_UNSUPPORTED_IN_ESCORT'
                for f in feedbacks
            ),
            timeout=3.0,
        )
    finally:
        ex.remove_node(client)
        ex.remove_node(intent_node)
        ex.remove_node(nav_node)
        client.destroy_node()
        intent_node.destroy_node()
        nav_node.destroy_node()
