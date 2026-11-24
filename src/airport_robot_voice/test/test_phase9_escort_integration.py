# Copyright (c) 2026 Airport Escort Robot Project
# Phase 9 Escort Tracking Integration Test Suite for airport_robot_voice

import json
import time
import uuid
import threading
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool

from airport_robot_voice.nav_command_node import NavCommandNode, MissionState


class ControllableNavServer(Node):
    """Mock Nav2 NavigateToPose Action Server với các cờ điều khiển."""
    def __init__(self, action_name):
        super().__init__(f'mock_nav2_server_{action_name.replace("/", "_")}')
        self.cbg = ReentrantCallbackGroup()
        self.action_name = action_name
        self.accept_goals = True
        self.accept_cancels = True
        self.hold_acceptance = None
        self.hold_cancel_response = None
        self.hold_terminal = None
        self.terminal_status = GoalStatus.STATUS_CANCELED

        self.received_cancels = []
        self.received_goals = []
        self.active_goal_handles = []

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
        if self.hold_cancel_response is not None:
            self.hold_cancel_response.wait(timeout=3.0)
        return CancelResponse.ACCEPT if self.accept_cancels else CancelResponse.REJECT

    def _execute_cb(self, goal_handle):
        self.active_goal_handles.append(goal_handle)
        t0 = time.time()
        while goal_handle.is_active and (time.time() - t0 < 5.0):
            if goal_handle.is_cancel_requested:
                if self.hold_terminal is not None:
                    self.hold_terminal.wait(timeout=3.0)
                if self.terminal_status == GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.succeed()
                elif self.terminal_status == GoalStatus.STATUS_ABORTED:
                    goal_handle.abort()
                else:
                    goal_handle.canceled()
                if goal_handle in self.active_goal_handles:
                    self.active_goal_handles.remove(goal_handle)
                return NavigateToPose.Result()
            if self.terminal_status == GoalStatus.STATUS_SUCCEEDED and self.hold_terminal is None:
                if time.time() - t0 >= 0.1:
                    goal_handle.succeed()
                    if goal_handle in self.active_goal_handles:
                        self.active_goal_handles.remove(goal_handle)
                    return NavigateToPose.Result()
            time.sleep(0.02)
        if goal_handle.is_active:
            if self.terminal_status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
            elif self.terminal_status == GoalStatus.STATUS_ABORTED:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            if goal_handle in self.active_goal_handles:
                self.active_goal_handles.remove(goal_handle)
        return NavigateToPose.Result()


class Phase9TestHarness:
    """Khung quản lý môi trường test Phase 9 tích hợp Voice và Tracking Topic Mock."""
    def __init__(self, action_name, enable_escort=True):
        self.action_name = action_name
        self.server = ControllableNavServer(action_name)
        self.nav_node = NavCommandNode()
        self.nav_node.enable_escort = enable_escort
        self.nav_node.nav_action_name = action_name
        self.nav_node.nav_client = ActionClient(
            self.nav_node, NavigateToPose, action_name, callback_group=self.server.cbg)

        # Topic theo dõi
        self.nav_statuses = []
        self.nav_states = []
        self.feedbacks = []
        self.speeches = []

        # Hook các hàm phát tin của nav_node để ghi nhận tức thì trong unit test
        orig_pub_state = self.nav_node.publish_state
        def mock_pub_state():
            orig_pub_state()
            rec = self.nav_node._attempts.get(self.nav_node._current_goal_attempt_id)
            self.nav_states.append({
                'mission_state': self.nav_node.state,
                'session_id': self.nav_node._current_session_id,
                'mission_id': self.nav_node._current_mission_id,
                'goal_attempt_id': self.nav_node._current_goal_attempt_id,
                'has_unresolved_goals': self.nav_node.has_unresolved_goals(),
            })
        self.nav_node.publish_state = mock_pub_state

        orig_pub_fb = self.nav_node.publish_feedback
        def mock_pub_fb(event, status, message=''):
            orig_pub_fb(event, status, message)
            self.feedbacks.append({'event': event, 'status': status, 'message': message})
        self.nav_node.publish_feedback = mock_pub_fb

        orig_say = self.nav_node.say
        def mock_say(text, speech_id=None, mission_id=None):
            sid = orig_say(text, speech_id, mission_id)
            self.speeches.append({'text': text, 'speech_id': sid, 'mission_id': mission_id})
            return sid
        self.nav_node.say = mock_say

        orig_pub_status = self.nav_node.publish_nav_status
        def mock_pub_status(status, nav_event, is_terminal, session_id=None, mission_id=None,
                            goal_attempt_id=None, nav2_uuid=None, destination_id=None,
                            request_id=None, message=''):
            orig_pub_status(status, nav_event, is_terminal, session_id, mission_id,
                            goal_attempt_id, nav2_uuid, destination_id, request_id, message)
            self.nav_statuses.append({
                'status': status,
                'nav_event': nav_event,
                'is_terminal': is_terminal,
                'session_id': session_id,
                'mission_id': mission_id,
                'goal_attempt_id': goal_attempt_id,
                'nav2_uuid': nav2_uuid,
                'destination_id': destination_id,
                'request_id': request_id,
                'message': message,
            })
        self.nav_node.publish_nav_status = mock_pub_status

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server)
        self.executor.add_node(self.nav_node)

        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def send_session_state(self, session_id="sess-001", locked=True, state="LOCKED_WAIT_DESTINATION",
                           obs_fresh=True, reason_code="HAND_RAISE_HELD_NEAREST", age_offset=0.0):
        now = self.nav_node.get_clock().now().to_msg()
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": now.sec, "nanosec": now.nanosec},
            "last_observation_stamp": {"sec": now.sec, "nanosec": now.nanosec},
            "obs_fresh": obs_fresh,
            "sequence_num": 1,
            "session_id": session_id,
            "state": state,
            "locked": locked,
            "person_track_id": 3,
            "person_distance_m": 1.45,
            "depth_proven": True,
            "confidence": 0.95,
            "reason_code": reason_code,
            "source_session_id": "src-01"
        }
        msg = String(data=json.dumps(payload))
        self.nav_node.on_session_state(msg)
        if age_offset > 0:
            self.nav_node._last_session_state_mono -= age_offset

    def send_safety_request(self, request_id="req-abort-1", session_id="sess-001", command="ABORT",
                            reason_code="VISIBILITY_TIMEOUT_EXCEEDED"):
        now = self.nav_node.get_clock().now().to_msg()
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": now.sec, "nanosec": now.nanosec},
            "request_id": request_id,
            "session_id": session_id,
            "command": command,
            "reason_code": reason_code,
            "message": f"Escort abort triggered: {reason_code}"
        }
        msg = String(data=json.dumps(payload))
        self.nav_node.on_safety_request(msg)

    def send_intent(self, action="navigate", target_id="gate_a3", req_id="req-nav-1"):
        payload = {
            "action": action,
            "target_id": target_id,
            "request_id": req_id
        }
        self.nav_node.on_intent(String(data=json.dumps(payload)))

    def send_tts_ack(self, speech_id):
        self.nav_node.on_tts_ack(String(data=json.dumps({
            "speech_id": speech_id,
            "status": "COMPLETED"
        })))

    def shutdown(self):
        try:
            self.executor.shutdown()
        except Exception:
            pass
        self.server.destroy_node()
        self.nav_node.destroy_node()


# ==============================================================================
# TEST CASES
# ==============================================================================

@pytest.fixture(autouse=True)
def setup_teardown_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_01_standalone_mode():
    """Khi escort tắt (enable_escort=False), Voice chạy standalone bình thường mà không cần session_state."""
    harness = Phase9TestHarness('test_act_p9_01', enable_escort=False)
    try:
        harness.send_intent(target_id='gate_a3', req_id='req-standalone-1')
        time.sleep(0.1)

        # Chuyển ANNOUNCING
        assert harness.nav_node.state == MissionState.ANNOUNCING
        pending_speech = harness.nav_node._pending_speech_id
        assert pending_speech is not None

        # TTS hoàn tất -> Gửi goal
        harness.send_tts_ack(pending_speech)
        time.sleep(0.3)

        assert len(harness.server.received_goals) == 1
        assert harness.nav_node.state == MissionState.NAVIGATING
        # Không phát nav_status khi escort tắt
        assert len(harness.nav_statuses) == 0
    finally:
        harness.shutdown()


def test_02_escort_rejection_when_unlocked():
    """Khi escort bật, từ chối dispatch nếu chưa khóa (locked=False)."""
    harness = Phase9TestHarness('test_act_p9_02', enable_escort=True)
    try:
        harness.send_session_state(session_id="sess-02", locked=False, state="IDLE")
        harness.send_intent(target_id='gate_a3', req_id='req-unlocked-1')
        time.sleep(0.1)

        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.server.received_goals) == 0
        fb_events = [f['event'] for f in harness.feedbacks]
        assert 'REJECTED' in fb_events
        assert any('NOT_LOCKED' in f['status'] for f in harness.feedbacks)
        assert any('Xin quý khách vui lòng giơ tay' in s['text'] for s in harness.speeches)
    finally:
        harness.shutdown()


def test_03_escort_rejection_when_stale_heartbeat():
    """Khi escort bật, từ chối dispatch nếu heartbeat session_state quá hạn > 1.0s."""
    harness = Phase9TestHarness('test_act_p9_03', enable_escort=True)
    try:
        harness.send_session_state(session_id="sess-03", locked=True, state="LOCKED_WAIT_DESTINATION", age_offset=1.5)
        harness.send_intent(target_id='gate_a3', req_id='req-stale-hb')
        time.sleep(0.1)

        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.server.received_goals) == 0
        assert any('HEARTBEAT_EXPIRED' in f['status'] for f in harness.feedbacks)
    finally:
        harness.shutdown()


def test_04_escort_rejection_when_observation_stale():
    """Khi escort bật, từ chối dispatch nếu obs_fresh=False."""
    harness = Phase9TestHarness('test_act_p9_04', enable_escort=True)
    try:
        harness.send_session_state(session_id="sess-04", locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=False)
        harness.send_intent(target_id='gate_a3', req_id='req-obs-stale')
        time.sleep(0.1)

        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.server.received_goals) == 0
        assert any('OBSERVATION_STALE' in f['status'] for f in harness.feedbacks)
    finally:
        harness.shutdown()


def test_05_escort_valid_lock_and_goal_accepted():
    """Khóa hợp lệ + Goal accepted -> phát NAVIGATION_STARTED với đủ 5 định danh."""
    harness = Phase9TestHarness('test_act_p9_05', enable_escort=True)
    try:
        sess_id = "sess-valid-005"
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)
        harness.send_intent(target_id='gate_a3', req_id='req-valid-1')
        time.sleep(0.1)

        assert harness.nav_node.state == MissionState.ANNOUNCING
        pending_speech = harness.nav_node._pending_speech_id
        assert pending_speech is not None

        # TTS hoàn tất -> send_nav_goal
        harness.send_tts_ack(pending_speech)
        time.sleep(0.3)

        assert len(harness.server.received_goals) == 1
        assert harness.nav_node.state == MissionState.NAVIGATING

        # Kiểm tra NAVIGATION_STARTED
        nav_started_events = [s for s in harness.nav_statuses if s['nav_event'] == 'NAVIGATION_STARTED']
        assert len(nav_started_events) == 1
        ev = nav_started_events[0]
        assert ev['status'] == 'NAVIGATING'
        assert ev['is_terminal'] is False
        assert ev['session_id'] == sess_id
        assert ev['mission_id'] is not None
        assert ev['goal_attempt_id'] is not None
        assert ev['nav2_uuid'] is not None
        assert ev['destination_id'] == 'gate_a3'
    finally:
        harness.shutdown()


def test_06_escort_lock_lost_while_awaiting_goal_response():
    """Mất khóa trong lúc chờ Nav2 chấp nhận goal -> hủy goal ngay, KHÔNG phát NAVIGATION_STARTED."""
    harness = Phase9TestHarness('test_act_p9_06', enable_escort=True)
    try:
        sess_id = "sess-drop-006"
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)

        # Giữ acceptance để mô phỏng độ trễ của Nav2 server
        harness.server.hold_acceptance = threading.Event()

        harness.send_intent(target_id='gate_a3', req_id='req-drop-1')
        time.sleep(0.1)
        pending_speech = harness.nav_node._pending_speech_id
        harness.send_tts_ack(pending_speech)
        time.sleep(0.1)

        # Lúc này goal đang được gửi tới server nhưng server chưa accept
        # Khách mất dấu hoặc bỏ đi: tracking gửi session_state locked=False
        harness.send_session_state(session_id=sess_id, locked=False, state="ABORTING", obs_fresh=False)

        # Server giải phóng acceptance
        harness.server.hold_acceptance.set()
        time.sleep(0.3)

        # Goal phải bị hủy ngay
        assert len(harness.server.received_cancels) >= 1
        # Tuyệt đối không phát NAVIGATION_STARTED
        nav_started = [s for s in harness.nav_statuses if s['nav_event'] == 'NAVIGATION_STARTED']
        assert len(nav_started) == 0
        assert harness.nav_node.state in (MissionState.CANCELING, MissionState.IDLE)
        assert any('SESSION_LOST_ON_ACCEPT' in f['status'] for f in harness.feedbacks)
    finally:
        harness.shutdown()


def test_07_escort_abort_wrong_session_ignored():
    """Nhận ABORT sai session_id -> Voice hoàn toàn bỏ qua."""
    harness = Phase9TestHarness('test_act_p9_07', enable_escort=True)
    try:
        sess_id = "sess-active-007"
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)
        harness.send_intent(target_id='gate_a3', req_id='req-nav-7')
        time.sleep(0.1)
        harness.send_tts_ack(harness.nav_node._pending_speech_id)
        time.sleep(0.3)

        assert harness.nav_node.state == MissionState.NAVIGATING

        # Gửi ABORT với session sai
        harness.send_safety_request(request_id="abort-wrong-1", session_id="sess-WRONG-999")
        time.sleep(0.1)

        # Vẫn tiếp tục NAVIGATING, không phát ACK hay Cancel
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert len(harness.server.received_cancels) == 0
        ack_events = [s for s in harness.nav_statuses if s['nav_event'] == 'CANCEL_ACKNOWLEDGED']
        assert len(ack_events) == 0
    finally:
        harness.shutdown()


def test_08_escort_abort_idempotency_and_ack():
    """Nhận ABORT đúng session: phát ACK CANCEL_ACKNOWLEDGED, hủy goal, retry cùng request_id là idempotent."""
    harness = Phase9TestHarness('test_act_p9_08', enable_escort=True)
    try:
        sess_id = "sess-active-008"
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)
        harness.send_intent(target_id='gate_a3', req_id='req-nav-8')
        time.sleep(0.1)
        harness.send_tts_ack(harness.nav_node._pending_speech_id)
        time.sleep(0.3)
        assert harness.nav_node.state == MissionState.NAVIGATING

        # Gửi ABORT lần 1
        req_abort_id = "abort-req-888"
        harness.send_safety_request(request_id=req_abort_id, session_id=sess_id)
        time.sleep(0.2)

        # Kiểm tra ACK lần 1
        acks = [s for s in harness.nav_statuses if s['nav_event'] == 'CANCEL_ACKNOWLEDGED']
        assert len(acks) == 1
        ack = acks[0]
        assert ack['status'] == 'CANCELING'
        assert ack['is_terminal'] is False
        assert ack['request_id'] == req_abort_id
        assert ack['session_id'] == sess_id
        assert ack['goal_attempt_id'] is not None
        assert ack['nav2_uuid'] is not None

        # Server nhận cancel request
        assert len(harness.server.received_cancels) == 1
        num_speeches_after_first_abort = len([s for s in harness.speeches if 'Đã mất dấu hành khách' in s['text']])
        assert num_speeches_after_first_abort == 1

        # Gửi retry ABORT lần 2 cùng request_id
        harness.send_safety_request(request_id=req_abort_id, session_id=sess_id)
        time.sleep(0.1)

        # Nhận lại ACK (tổng 2 ACKs)
        acks_retry = [s for s in harness.nav_statuses if s['nav_event'] == 'CANCEL_ACKNOWLEDGED']
        assert len(acks_retry) == 2
        # Nhưng KHÔNG gọi cancel lặp (server chỉ nhận 1 cancel)
        assert len(harness.server.received_cancels) == 1
        # Và KHÔNG phát TTS lặp
        num_speeches_after_retry = len([s for s in harness.speeches if 'Đã mất dấu hành khách' in s['text']])
        assert num_speeches_after_retry == 1
    finally:
        harness.shutdown()


def test_09_escort_ack_does_not_replace_terminal_and_terminal_completion():
    """ACK không thay thế terminal. Chỉ khi action result về mới phát GOAL_COMPLETED is_terminal=True."""
    harness = Phase9TestHarness('test_act_p9_09', enable_escort=True)
    try:
        sess_id = "sess-term-009"
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)
        harness.send_intent(target_id='gate_a3', req_id='req-nav-9')
        time.sleep(0.1)
        harness.send_tts_ack(harness.nav_node._pending_speech_id)
        time.sleep(0.3)

        # Giữ terminal của server để kiểm tra trạng thái khi mới chỉ có ACK
        harness.server.hold_terminal = threading.Event()
        harness.server.terminal_status = GoalStatus.STATUS_CANCELED

        req_abort_id = "abort-req-999"
        harness.send_safety_request(request_id=req_abort_id, session_id=sess_id)
        time.sleep(0.2)

        # Đã có ACK
        acks = [s for s in harness.nav_statuses if s['nav_event'] == 'CANCEL_ACKNOWLEDGED']
        assert len(acks) == 1
        # Nhưng chưa có terminal
        terms = [s for s in harness.nav_statuses if s['is_terminal'] is True]
        assert len(terms) == 0
        # Goal vẫn chưa hoàn thành
        assert harness.nav_node.has_unresolved_goals() is True

        # Server giải phóng terminal result
        harness.server.hold_terminal.set()
        time.sleep(0.4)

        # Đã có terminal GOAL_COMPLETED
        terms = [s for s in harness.nav_statuses if s['is_terminal'] is True]
        assert len(terms) == 1
        term = terms[0]
        assert term['nav_event'] == 'GOAL_COMPLETED'
        assert term['status'] == 'CANCELED'
        assert term['session_id'] == sess_id
        assert term['request_id'] == req_abort_id
        assert harness.nav_node.has_unresolved_goals() is False
        assert harness.nav_node.state == MissionState.IDLE
    finally:
        harness.shutdown()


def test_10_escort_goal_succeeded_terminal():
    """Khi goal tới đích bình thường, phát GOAL_COMPLETED với status=SUCCEEDED và is_terminal=True."""
    harness = Phase9TestHarness('test_act_p9_10', enable_escort=True)
    try:
        sess_id = "sess-succ-010"
        harness.server.terminal_status = GoalStatus.STATUS_SUCCEEDED
        harness.send_session_state(session_id=sess_id, locked=True, state="LOCKED_WAIT_DESTINATION", obs_fresh=True)
        harness.send_intent(target_id='gate_a3', req_id='req-nav-10')
        time.sleep(0.1)
        harness.send_tts_ack(harness.nav_node._pending_speech_id)
        time.sleep(0.4)

        # Chờ server hoàn tất thành công
        succ_events = [s for s in harness.nav_statuses if s['status'] == 'SUCCEEDED' and s['is_terminal'] is True]
        assert len(succ_events) == 1
        ev = succ_events[0]
        assert ev['nav_event'] == 'GOAL_COMPLETED'
        assert ev['session_id'] == sess_id
        assert ev['destination_id'] == 'gate_a3'
        assert harness.nav_node.state == MissionState.IDLE
        assert harness.nav_node.has_unresolved_goals() is False
    finally:
        harness.shutdown()
