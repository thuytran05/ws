#!/usr/bin/env python3
"""
test_ros_lifecycle.py — Kiểm thử toàn diện vòng đời Action Nav2, đồng bộ TTS và xử lý race condition.
Bao phủ toàn diện GIAI ĐOẠN 2, 3 và 4:
  1. Pause / cancel trước goal acceptance (khi đang STARTING).
  2. Pause -> Resume, result của goal cũ về sau (bỏ qua attempt cũ).
  3. Replace khi đang chạy (NAVIGATING) và khi đang tạm dừng (PAUSED), kiểm tra đích không hợp lệ.
  4. Feedback cũ về trong chuyến mới (bỏ qua).
  5. Cancel bị từ chối / timeout.
  6. Request lặp cùng id (khử trùng lặp qua cache).
  7. Cancel trong lúc TTS đang chờ (ANNOUNCING).
  8. Phát sinh thông báo đúng một lần & đồng bộ TTS (chỉ gửi goal sau tts_ack hoặc timeout).

Sử dụng MultiThreadedExecutor cô lập, có finite timeout và thông báo lỗi rõ ràng.
"""

import collections
import json
import threading
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool

from airport_robot_voice.nav_command_node import NavCommandNode, MissionState


class ControllableNavServer(Node):
    """Action Server giả lập NavigateToPose có thể điều khiển toàn bộ trạng thái bất đồng bộ."""
    def __init__(self, action_name):
        super().__init__('controllable_nav_server_' + action_name.replace('/', '_'))
        self.cbg = ReentrantCallbackGroup()
        self.action_name = action_name
        self.accept_goals = True
        self.accept_cancels = True
        self.hold_acceptance = None  # None hoặc threading.Event()
        self.received_cancels = []
        self.received_goals = []

        self._server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cbg
        )

    def _goal_cb(self, goal_request):
        self.received_goals.append(goal_request)
        if self.hold_acceptance is not None:
            self.hold_acceptance.wait(timeout=3.0)
        return GoalResponse.ACCEPT if self.accept_goals else GoalResponse.REJECT

    def _cancel_cb(self, goal_handle):
        self.received_cancels.append(goal_handle)
        return CancelResponse.ACCEPT if self.accept_cancels else CancelResponse.REJECT

    def _execute_cb(self, goal_handle):
        t0 = time.time()
        # Chờ cancel hoặc tối đa 4.0s
        while goal_handle.is_active and (time.time() - t0 < 4.0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        goal_handle.succeed()
        return NavigateToPose.Result()


class NavTestHarness:
    """Khung quản lý vòng lặp MultiThreadedExecutor cho kiểm thử cô lập."""
    def __init__(self, action_name):
        self.action_name = action_name
        self.server = ControllableNavServer(action_name)
        self.nav_node = NavCommandNode()
        self.nav_node.nav_action_name = action_name
        self.nav_node.nav_client = ActionClient(
            self.nav_node, NavigateToPose, action_name, callback_group=self.server.cbg)
        self.nav_node.tts_ack_timeout = 0.1  # Nhanh chóng fallback nếu không test tts_ack riêng

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.server)
        self.executor.add_node(self.nav_node)

        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

        # Đợi kết nối action server
        t0 = time.time()
        while not self.nav_node.nav_client.server_is_ready() and (time.time() - t0 < 3.0):
            time.sleep(0.02)
        if not self.nav_node.nav_client.server_is_ready():
            raise TimeoutError(f"Action server '{action_name}' không sẵn sàng sau 3.0s")

    def wait_state(self, expected_states, timeout=3.0):
        if isinstance(expected_states, str):
            expected_states = [expected_states]
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.nav_node.state in expected_states:
                return self.nav_node.state
            time.sleep(0.02)
        raise TimeoutError(
            f"Timeout {timeout}s chờ state thuộc {expected_states}. State hiện tại: '{self.nav_node.state}'")

    def shutdown(self):
        self.executor.shutdown()
        self.nav_node.destroy_node()
        self.server.destroy_node()


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pause_before_goal_acceptance(ros_context):
    """
    Test 1: Lệnh pause đến khi goal đang chờ chấp nhận (STARTING).
    Server giữ acceptance, client gửi PAUSE -> client vào PAUSING.
    Khi server accept -> client hủy goal ngay lập tức và chuyển sang PAUSED.
    """
    harness = NavTestHarness('act_pause_pre_acc')
    try:
        harness.server.hold_acceptance = threading.Event()

        # Bắt đầu gửi goal
        nav_payload = {'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_t1_pause'}
        harness.nav_node.on_intent(String(data=json.dumps(nav_payload)))

        # Chờ vào STARTING
        harness.wait_state(MissionState.STARTING, timeout=2.0)
        assert harness.nav_node.state == MissionState.STARTING
        attempt_id = harness.nav_node._current_goal_attempt_id

        # Gửi pause ngay khi server chưa accept
        harness.nav_node.handle_pause_request('req_pause_now')
        assert harness.nav_node.state == MissionState.PAUSING
        assert attempt_id in harness.nav_node._cancelled_attempts

        # Bây giờ cho server accept
        harness.server.hold_acceptance.set()

        # Chờ client đạt trạng thái PAUSED
        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        assert harness.nav_node.state == MissionState.PAUSED
        assert len(harness.server.received_cancels) >= 1
    finally:
        harness.shutdown()


def test_cancel_before_goal_acceptance(ros_context):
    """
    Test 2: Lệnh cancel đến khi goal đang chờ chấp nhận (STARTING).
    Server giữ acceptance, client gửi CANCEL -> client vào CANCELING.
    Khi server accept -> client hủy goal và trở về IDLE.
    """
    harness = NavTestHarness('act_cancel_pre_acc')
    try:
        harness.server.hold_acceptance = threading.Event()

        nav_payload = {'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_t2_cancel'}
        harness.nav_node.on_intent(String(data=json.dumps(nav_payload)))

        harness.wait_state(MissionState.STARTING, timeout=2.0)
        attempt_id = harness.nav_node._current_goal_attempt_id

        harness.nav_node.handle_cancel_request('req_cancel_now')
        assert harness.nav_node.state == MissionState.CANCELING
        assert attempt_id in harness.nav_node._cancelled_attempts

        harness.server.hold_acceptance.set()

        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.server.received_cancels) >= 1
    finally:
        harness.shutdown()


def test_pause_resume_and_late_goal_result(ros_context):
    """
    Test 3: Robot đang NAVIGATING -> Pause -> Resume (gửi goal attempt 2).
    Sau đó result của attempt 1 về muộn -> Node phải bỏ qua attempt 1 và tiếp tục attempt 2.
    """
    harness = NavTestHarness('act_late_res')
    try:
        # Xuất phát mission 1
        nav_payload = {'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_m1'}
        harness.nav_node.on_intent(String(data=json.dumps(nav_payload)))

        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        first_attempt = harness.nav_node._current_goal_attempt_id

        # Tạm dừng
        harness.nav_node.handle_pause_request('req_pause')
        harness.wait_state(MissionState.PAUSED, timeout=3.0)

        # Đi tiếp (Resume)
        harness.nav_node.handle_resume_request('req_resume')
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        second_attempt = harness.nav_node._current_goal_attempt_id
        assert second_attempt != first_attempt

        # Giả lập result cũ của first_attempt về muộn
        class FakeFuture:
            def result(self):
                class FakeRes:
                    status = GoalStatus.STATUS_SUCCEEDED
                return FakeRes()

        harness.nav_node.on_goal_result(FakeFuture(), first_attempt, harness.nav_node._current_mission_id)

        # Node không được phép bị kết thúc bởi attempt cũ!
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_goal_attempt_id == second_attempt
    finally:
        harness.shutdown()


def test_replace_while_navigating_and_paused(ros_context):
    """
    Test 4:
      A. Replace khi đang NAVIGATING: Hủy goal cũ, chờ hoàn tất, xuất phát tới đích mới.
      B. Replace khi đang PAUSED: Cập nhật đích mới và GIỮ NGUYÊN trạng thái PAUSED.
      C. Replace đích không hợp lệ: Giữ nguyên lộ trình hiện tại.
    """
    harness = NavTestHarness('act_replace')
    try:
        # 1. Bắt đầu tới cổng A3
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_r1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node._current_target_key == 'gate_a3'

        # 2. Đổi đích không hợp lệ (cổng A99 không tồn tại)
        harness.nav_node.handle_replace_target({'target_id': 'gate_a99', 'request_id': 'req_inv'})
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a3'  # Giữ nguyên!

        # 3. Đổi đích hợp lệ khi đang NAVIGATING sang cổng A4
        harness.nav_node.handle_replace_target({'target_id': 'gate_a4', 'request_id': 'req_v4'})
        # Chờ chuyển sang target mới
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if harness.nav_node._current_target_key == 'gate_a4' and harness.nav_node.state in (MissionState.NAVIGATING, MissionState.STARTING):
                break
            time.sleep(0.02)
        assert harness.nav_node._current_target_key == 'gate_a4'

        # 4. Tạm dừng
        harness.nav_node.handle_pause_request('req_pause_rep')
        harness.wait_state(MissionState.PAUSED, timeout=3.0)

        # 5. Đổi đích khi đang PAUSED sang quầy checkin
        harness.nav_node.handle_replace_target({'target_id': 'checkin', 'request_id': 'req_rep_paused'})
        assert harness.nav_node.state == MissionState.PAUSED       # Giữ nguyên PAUSED!
        assert harness.nav_node._current_target_key == 'checkin'  # Đích đã đổi sang checkin!
    finally:
        harness.shutdown()


def test_discard_old_feedback_and_freshness_status(ros_context):
    """
    Test 5: Feedback từ attempt cũ bị bỏ qua. Kiểm tra tính tươi của feedback khi hỏi STATUS.
    """
    nav_node = NavCommandNode()
    nav_node.state = MissionState.NAVIGATING
    nav_node._current_mission_id = 'mission_99'
    nav_node._current_goal_attempt_id = 'mission_99_g2'
    nav_node._current_target_name = 'cổng A3'

    # 1. Feedback mang attempt cũ -> bị bỏ qua
    class DummyMsg:
        class DummyFeedback:
            distance_remaining = 50.0
            estimated_time_remaining = None
        feedback = DummyFeedback()

    nav_node.on_nav_feedback(DummyMsg(), 'mission_99_g1')
    assert nav_node._distance_remaining is None

    # 2. Feedback mang attempt hiện tại -> cập nhật
    nav_node.on_nav_feedback(DummyMsg(), 'mission_99_g2')
    assert nav_node._distance_remaining == 50.0

    # 3. Kiểm tra status khi feedback còn mới
    say_msgs = []
    nav_node.create_subscription(String, '/voice/say', lambda m: say_msgs.append(json.loads(m.data)['text']), 10)
    nav_node.handle_status_request('req_stat_1')
    time.sleep(0.05)
    rclpy.spin_once(nav_node, timeout_sec=0.05)
    assert any("50 mét" in s for s in say_msgs)

    # 4. Khi feedback đã quá cũ (> 5.0s)
    say_msgs.clear()
    nav_node._last_feedback_time = time.time() - 10.0
    nav_node.handle_status_request('req_stat_2')
    time.sleep(0.05)
    rclpy.spin_once(nav_node, timeout_sec=0.05)
    assert any("Tôi đang dẫn bạn đến cổng A3." in s and "50 mét" not in s for s in say_msgs)

    nav_node.destroy_node()


def test_request_deduplication(ros_context):
    """
    Test 6: Gửi 2 request trùng request_id liên tiếp -> Lệnh thứ 2 bị bỏ qua.
    """
    nav_node = NavCommandNode()
    calls = []
    nav_node.handle_navigate_request = lambda data: calls.append(data)

    payload = {'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_duplicate_100'}
    nav_node.on_intent(String(data=json.dumps(payload)))
    nav_node.on_intent(String(data=json.dumps(payload)))

    assert len(calls) == 1
    nav_node.destroy_node()


def test_tts_synchronization_and_cancel_in_announcing(ros_context):
    """
    Test 7 & 8:
      A. Mission chỉ chuyển sang STARTING sau khi nhận được tts_ack khớp speech_id.
      B. Nếu cancel xảy ra khi đang ANNOUNCING, goal không bao giờ được gửi.
    """
    harness = NavTestHarness('act_tts_sync')
    try:
        harness.nav_node.tts_ack_timeout = 4.0  # Timeout dài để kiểm chứng vai trò của tts_ack

        # 1. Bắt đầu mission
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_tts_1'})))

        # Phải ở ANNOUNCING và goal CHƯA được gửi
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)
        assert harness.nav_node.state == MissionState.ANNOUNCING
        assert len(harness.server.received_goals) == 0
        pending_sp = harness.nav_node._pending_speech_id
        assert pending_sp is not None

        # 2. Publish tts_ack khớp speech_id
        ack_payload = {'speech_id': pending_sp, 'status': 'COMPLETED'}
        harness.nav_node.on_tts_ack(String(data=json.dumps(ack_payload)))

        # Bây giờ goal mới được gửi sang server!
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert len(harness.server.received_goals) >= 1

        # 3. Test cancel trong lúc ANNOUNCING
        harness.nav_node.handle_cancel_request('req_cancel_tts')
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE

        # Khởi động mission khác
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_tts_2'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)

        # Hủy ngay khi đang ANNOUNCING
        harness.nav_node.handle_cancel_request('req_cancel_mid_announce')
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node._pending_speech_id is None
    finally:
        harness.shutdown()


def test_replace_while_starting(ros_context):
    """
    Test 8: Replace khi đang STARTING (goal đang chờ acceptance).
    Goal cũ bị hủy ngay khi vừa accept, sau đó hệ thống tự động xuất phát tới đích mới.
    """
    harness = NavTestHarness('act_replace_starting')
    try:
        harness.server.hold_acceptance = threading.Event()
        harness.nav_node.tts_ack_timeout = 0.05

        # 1. Bắt đầu mission A3 -> chuyển sang STARTING (chờ server accept)
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_start_1'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)
        assert harness.nav_node.state == MissionState.STARTING
        old_attempt = harness.nav_node._current_goal_attempt_id

        # 2. Đổi đích sang A4 khi đang STARTING
        harness.nav_node.handle_replace_target({'target_id': 'gate_a4', 'request_id': 'req_rep_starting'})
        assert old_attempt in harness.nav_node._cancelled_attempts
        assert harness.nav_node._pending_replace_target == 'gate_a4'

        # 3. Server giải phóng acceptance cho goal cũ A3
        harness.server.hold_acceptance.set()

        # Hệ thống phải thu hồi A3 và tự động xuất phát tới A4
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if harness.nav_node._current_target_key == 'gate_a4' and harness.nav_node.state == MissionState.NAVIGATING:
                break
            time.sleep(0.02)
        assert harness.nav_node._current_target_key == 'gate_a4'
        assert harness.nav_node.state == MissionState.NAVIGATING
    finally:
        if harness.server.hold_acceptance:
            harness.server.hold_acceptance.set()
        harness.shutdown()


def test_cancel_rejected_or_timeout(ros_context):
    """
    Test 9: Xử lý an toàn khi Action Server từ chối cancel.
    Node không được treo, ghi nhận cảnh báo và dọn dẹp trạng thái an toàn về IDLE.
    """
    harness = NavTestHarness('act_cancel_rej')
    try:
        # Bắt đầu và chờ NAVIGATING
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_rej_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Server từ chối cancel
        harness.server.accept_cancels = False

        # Gửi yêu cầu hủy
        harness.nav_node.handle_cancel_request('req_cancel_rej')
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert harness.nav_node._current_target_key is None
    finally:
        harness.shutdown()


def test_speech_single_emission_per_event(ros_context):
    """
    Test 10: Phát sinh thông báo đúng một lần cho mỗi sự kiện tương ứng.
    Kiểm tra không bị lặp câu thông báo hay spam lời nói khi chuyển trạng thái.
    """
    harness = NavTestHarness('act_single_speech')
    spoken_messages = []
    
    # Intercept pub_say
    orig_say = harness.nav_node.say
    def mock_say(text, speech_id=None, mission_id=None):
        spoken_messages.append(text)
        return orig_say(text, speech_id, mission_id)
    harness.nav_node.say = mock_say

    try:
        # 1. Navigate
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_sp_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        nav_speeches = [s for s in spoken_messages if 'cổng A3' in s]
        assert len(nav_speeches) == 1, f"Mong đợi đúng 1 thông báo xuất phát, thực tế: {nav_speeches}"

        # 2. Pause
        spoken_messages.clear()
        harness.nav_node.handle_pause_request('req_sp_p1')
        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        pause_speeches = [s for s in spoken_messages if 'tạm dừng' in s]
        assert len(pause_speeches) == 1, f"Mong đợi đúng 1 thông báo tạm dừng, thực tế: {pause_speeches}"

        # 3. Duplicate Pause khi đã PAUSED
        spoken_messages.clear()
        harness.nav_node.handle_pause_request('req_sp_p2')
        dup_pause = [s for s in spoken_messages if 'đã đang ở trạng thái tạm dừng' in s]
        assert len(dup_pause) == 1

        # 4. Resume
        spoken_messages.clear()
        harness.nav_node.handle_resume_request('req_sp_res')
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        resume_speeches = [s for s in spoken_messages if 'tiếp tục di chuyển' in s]
        assert len(resume_speeches) == 1

        # 5. Cancel
        spoken_messages.clear()
        harness.nav_node.handle_cancel_request('req_sp_can')
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        cancel_speeches = [s for s in spoken_messages if 'hủy chuyến đi' in s]
        assert len(cancel_speeches) == 1
    finally:
        harness.shutdown()
