"""
Bộ kiểm thử toàn diện vòng đời Action Nav2 (test_ros_lifecycle.py).
Kiểm thử chuyên sâu các trường hợp biên, race condition, trạng thái terminal,
từ chối hủy, trì hoãn terminal result, hủy khi PAUSING, replace khi STARTING,
và đồng bộ TTS ACK.
"""

import json
import threading
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from airport_robot_voice.nav_command_node import NavCommandNode, MissionState
from airport_robot_voice.intent_node import IntentNode


class ControllableNavServer(Node):
    """Action Server giả lập NavigateToPose có thể điều khiển toàn bộ trạng thái bất đồng bộ."""
    def __init__(self, action_name):
        super().__init__('controllable_nav_server_' + action_name.replace('/', '_'))
        self.cbg = ReentrantCallbackGroup()
        self.action_name = action_name
        self.accept_goals = True
        self.accept_cancels = True
        self.hold_acceptance = None         # threading.Event()
        self.hold_cancel_response = None    # threading.Event()
        self.hold_terminal = None           # threading.Event()
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
            time.sleep(0.02)
        if goal_handle.is_active:
            goal_handle.succeed()
            if goal_handle in self.active_goal_handles:
                self.active_goal_handles.remove(goal_handle)
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
        self.nav_node.tts_ack_timeout = 0.05
        self.planner_action_name = action_name + '_planner'
        self.planner_hold = None
        self._planner_server = ActionServer(
            self.server,
            ComputePathToPose,
            self.planner_action_name,
            execute_callback=self._execute_planner,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            callback_group=self.server.cbg
        )
        self.nav_node.planner_action_name = self.planner_action_name
        self.nav_node.planner_client = ActionClient(
            self.nav_node, ComputePathToPose, self.planner_action_name, callback_group=self.server.cbg
        )

        self.spoken_messages = []
        self.feedback_messages = []

        orig_say = self.nav_node.say
        def mock_say(text, speech_id=None, mission_id=None):
            self.spoken_messages.append(text)
            return orig_say(text, speech_id, mission_id)
        self.nav_node.say = mock_say

        orig_fb = self.nav_node.publish_feedback
        def mock_fb(event, status, message=''):
            self.feedback_messages.append({'event': event, 'status': status, 'message': message})
            return orig_fb(event, status, message)
        self.nav_node.publish_feedback = mock_fb

        self.intent_node = IntentNode()
        self.intent_node.nav_timeout = 10.0
        self.text_pub = self.server.create_publisher(String, '/voice/text', 10)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.server)
        self.executor.add_node(self.nav_node)
        self.executor.add_node(self.intent_node)

        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

        # Đợi kết nối action server
        t0 = time.time()
        while not (self.nav_node.nav_client.server_is_ready() and self.nav_node.planner_client.server_is_ready()) and (time.time() - t0 < 4.0):
            time.sleep(0.02)
        if not self.nav_node.nav_client.server_is_ready():
            raise TimeoutError(f"Nav Action server '{action_name}' không sẵn sàng sau 4.0s")
        if not self.nav_node.planner_client.server_is_ready():
            raise TimeoutError(f"Planner Action server '{self.planner_action_name}' không sẵn sàng sau 4.0s")

    def _execute_planner(self, goal_handle):
        if self.planner_hold is not None:
            self.planner_hold.wait(timeout=3.0)
        res = ComputePathToPose.Result()
        path = Path()
        path.header.frame_id = 'map'
        p1 = PoseStamped()
        p2 = PoseStamped()
        p2.pose.position.x = 5.0
        path.poses = [p1, p2]
        res.error_code = 0
        res.path = path
        goal_handle.succeed()
        return res

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
        if self.server.hold_acceptance:
            self.server.hold_acceptance.set()
        if self.server.hold_cancel_response:
            self.server.hold_cancel_response.set()
        if self.server.hold_terminal:
            self.server.hold_terminal.set()
        if self.planner_hold:
            self.planner_hold.set()
        time.sleep(0.05)
        if hasattr(self, '_planner_server') and self._planner_server:
            try:
                self._planner_server.destroy()
            except Exception:
                pass
        if hasattr(self.server, '_server') and self.server._server:
            try:
                self.server._server.destroy()
            except Exception:
                pass
        self.executor.shutdown()
        if hasattr(self, 'intent_node') and self.intent_node:
            try:
                self.intent_node.destroy_node()
            except Exception:
                pass
        self.nav_node.destroy_node()
        self.server.destroy_node()

    def send_voice_text(self, text: str):
        self.text_pub.publish(String(data=text))


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pause_before_goal_acceptance(ros_context):
    """
    Test 1: Lệnh tạm dừng (PAUSE) đến khi goal đang chờ server chấp nhận (STARTING).
    Xác nhận: Goal được hủy ngay khi vừa được accept và robot chuyển sang PAUSED.
    """
    harness = NavTestHarness('act_p_before_acc')
    try:
        harness.server.hold_acceptance = threading.Event()
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_pba_1'})))

        harness.wait_state(MissionState.STARTING, timeout=2.0)
        assert harness.nav_node.state == MissionState.STARTING

        # Gửi pause khi goal chưa accept
        harness.nav_node.handle_pause_request('req_pause_pre')
        assert harness.nav_node.state == MissionState.PAUSING

        # Server chấp nhận goal
        harness.server.hold_acceptance.set()

        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        assert harness.nav_node.state == MissionState.PAUSED
        assert len(harness.server.received_cancels) >= 1
    finally:
        harness.shutdown()


def test_cancel_before_goal_acceptance(ros_context):
    """
    Test 2: Lệnh hủy (CANCEL) đến khi goal đang chờ server chấp nhận (STARTING).
    Xác nhận: Goal được hủy ngay khi accept và robot dọn dẹp về IDLE.
    """
    harness = NavTestHarness('act_c_before_acc')
    try:
        harness.server.hold_acceptance = threading.Event()
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_cba_1'})))

        harness.wait_state(MissionState.STARTING, timeout=2.0)
        assert harness.nav_node.state == MissionState.STARTING

        harness.nav_node.handle_cancel_request('req_cancel_pre')
        assert harness.nav_node.state == MissionState.CANCELING

        harness.server.hold_acceptance.set()

        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert harness.nav_node._current_target_key is None
        assert len(harness.server.received_cancels) >= 1
    finally:
        harness.shutdown()


def test_cancel_rejected_leaves_goal_active_and_navigating(ros_context):
    """
    Test 3: Khi Server từ chối lệnh CANCEL (return_code != 0).
    Xác nhận theo trạng thái thực tế:
    - Robot KHÔNG được chuyển sang IDLE hay CANCELED.
    - Trạng thái quay trở lại NAVIGATING (vì robot vẫn đang chạy thực tế trên server).
    - Goal handle vẫn được giữ để tiếp tục theo dõi.
    - Có thông báo giọng nói giải thích từ chối.
    - Lệnh resume bị chặn.
    """
    harness = NavTestHarness('act_cancel_rejected')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_rej_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert len(harness.server.active_goal_handles) == 1

        # Cấu hình server từ chối hủy
        harness.server.accept_cancels = False

        # Gửi lệnh cancel
        harness.nav_node.handle_cancel_request('req_rej_cancel')

        # Đợi xử lý phản hồi từ chối
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if any('Không thể hủy chuyến đi lúc này' in s for s in harness.spoken_messages):
                break
            time.sleep(0.02)

        # Trạng thái phải là NAVIGATING (không được tự ý IDLE!)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._active_goal_handle is not None
        assert len(harness.server.active_goal_handles) == 1

        # Resume phải bị chặn vì không ở trạng thái PAUSED
        harness.nav_node.handle_resume_request('req_res_blocked')
        assert any('không trong trạng thái tạm dừng' in s for s in harness.spoken_messages)
    finally:
        harness.shutdown()


def test_pause_rejected_leaves_goal_active_and_navigating(ros_context):
    """
    Test 4: Khi Server từ chối lệnh PAUSE (return_code != 0).
    Xác nhận:
    - Robot không chuyển sang PAUSED.
    - Trạng thái duy trì NAVIGATING.
    - Thông báo lỗi phát ra đúng chuẩn.
    """
    harness = NavTestHarness('act_pause_rejected')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_prej_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        harness.server.accept_cancels = False
        harness.nav_node.handle_pause_request('req_rej_pause')

        t0 = time.time()
        while time.time() - t0 < 3.0:
            if any('Không thể tạm dừng lúc này' in s for s in harness.spoken_messages):
                break
            time.sleep(0.02)

        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._active_goal_handle is not None
    finally:
        harness.shutdown()


def test_cancel_accepted_waits_for_terminal_result_before_paused_or_idle(ros_context):
    """
    Test 5: Server chấp nhận cancel nhưng trì hoãn việc gửi terminal result.
    Xác nhận phân biệt 3 pha:
    - Khi cancel accepted, node ở PAUSING (chưa được sang PAUSED!).
    - Chỉ sau khi terminal result arrive, node mới sang PAUSED.
    """
    harness = NavTestHarness('act_term_delay')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_td_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Chặn kết thúc terminal trên server
        harness.server.hold_terminal = threading.Event()

        # Gửi pause
        harness.nav_node.handle_pause_request('req_td_pause')

        # Chờ cancel response được nhận (server chấp nhận cancel)
        t0 = time.time()
        while len(harness.server.received_cancels) == 0 and time.time() - t0 < 2.0:
            time.sleep(0.02)
        assert len(harness.server.received_cancels) >= 1

        # Node PHẢI đang ở PAUSING, KHÔNG ĐƯỢC ở PAUSED
        time.sleep(0.1)
        assert harness.nav_node.state == MissionState.PAUSING

        # Giải phóng terminal result
        harness.server.hold_terminal.set()

        # Bây giờ mới sang PAUSED
        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        assert harness.nav_node.state == MissionState.PAUSED
    finally:
        harness.shutdown()


def test_goal_finishes_succeeded_or_aborted_during_cancel(ros_context):
    """
    Test 6: Goal kết thúc SUCCEEDED trong lúc yêu cầu hủy đang xử lý
    (robot kịp tới đích trước khi dừng).
    Xác nhận: Node nhận diện đã tới đích, báo SUCCEEDED và về IDLE (không bị kẹt ở PAUSED).
    """
    harness = NavTestHarness('act_succ_during_cancel')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_sdc_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Cấu hình server hoàn tất với SUCCEEDED khi nhận cancel
        harness.server.terminal_status = GoalStatus.STATUS_SUCCEEDED

        harness.nav_node.handle_pause_request('req_sdc_pause')

        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert any('Chúng ta đã tới' in s for s in harness.spoken_messages)
    finally:
        harness.shutdown()


def test_cancel_received_while_pausing_transitions_to_canceling(ros_context):
    """
    Test 7: Nhận lệnh CANCEL trong lúc đang PAUSING.
    Xác nhận: State được nâng cấp sang CANCELING và sau khi goal dừng hẳn sẽ về IDLE.
    """
    harness = NavTestHarness('act_can_while_pause')
    try:
        harness.server.hold_terminal = threading.Event()
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_cwp_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Yêu cầu pause
        harness.nav_node.handle_pause_request('req_cwp_pause')
        assert harness.nav_node.state == MissionState.PAUSING

        # Yêu cầu cancel khi đang PAUSING
        harness.nav_node.handle_cancel_request('req_cwp_cancel')
        assert harness.nav_node.state == MissionState.CANCELING

        # Cho phép terminal result hoàn tất
        harness.server.hold_terminal.set()

        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert any('Tôi đã hủy chuyến đi hiện tại' in s for s in harness.spoken_messages)
    finally:
        harness.shutdown()


def test_pending_goal_rejected_after_pause_cancel_replace(ros_context):
    """
    Test 8: Goal đang pending acceptance trong STARTING, sau đó server từ chối goal (REJECTED).
    Xác nhận: Node xử lý sạch sẽ, đưa về IDLE không bị deadlock hay gửi double cancel.
    """
    harness = NavTestHarness('act_pending_reject')
    try:
        harness.server.hold_acceptance = threading.Event()
        harness.server.accept_goals = False  # Server sẽ reject

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_pr_1'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)

        # Gửi pause khi đang chờ acceptance
        harness.nav_node.handle_pause_request('req_pr_pause')

        # Server trả về rejection
        harness.server.hold_acceptance.set()

        harness.wait_state([MissionState.IDLE, MissionState.PAUSED], timeout=3.0)
        assert harness.nav_node.state in (MissionState.IDLE, MissionState.PAUSED)
        assert harness.nav_node._active_goal_handle is None
    finally:
        harness.shutdown()


def test_replace_waits_for_old_goal_terminal_result_before_sending_new_goal(ros_context):
    """
    Test 9: Replace khi đang NAVIGATING.
    Xác nhận: Goal mới (A4) CHỈ ĐƯỢC GỬI SANG SERVER sau khi goal cũ (A3) đã kết thúc terminal CANCELED.
    """
    harness = NavTestHarness('act_rep_wait_term')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_rw_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert len(harness.server.received_goals) == 1

        # Trì hoãn terminal result của goal 1
        harness.server.hold_terminal = threading.Event()

        # Gửi lệnh đổi đích sang A4
        harness.nav_node.handle_replace_target({'target_id': 'gate_a4', 'request_id': 'req_rw_rep'})

        # Trong khi goal 1 chưa kết thúc terminal, server CHƯA ĐƯỢC NHẬN goal 2
        time.sleep(0.15)
        assert len(harness.server.received_goals) == 1
        assert harness.nav_node.state == MissionState.PAUSING

        # Giải phóng terminal result cho goal 1
        harness.server.hold_terminal.set()

        # Goal 2 mới được gửi
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert len(harness.server.received_goals) == 2
        assert harness.nav_node._current_target_key == 'gate_a4'
    finally:
        harness.shutdown()


def test_pause_and_resume_during_planner_resolving(ros_context):
    """
    Test 10: Tạm dừng và đi tiếp khi đang trong tiến trình chọn đường (RESOLVING).
    Xác nhận: Category được bảo toàn, resume tiếp tục giải quyết category mà không dùng nhầm pose cũ.
    """
    harness = NavTestHarness('act_p_res_resolving')
    try:
        # Giữ execution planner để node duy trì ở RESOLVING
        harness.planner_hold = threading.Event()

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'category': 'toilet', 'request_id': 'req_prr_1'})))
        harness.wait_state(MissionState.RESOLVING, timeout=2.0)
        assert harness.nav_node.state == MissionState.RESOLVING

        # Gửi pause ngay khi đang RESOLVING
        harness.nav_node.handle_pause_request('req_pause_res')
        harness.wait_state(MissionState.PAUSED, timeout=2.0)
        assert harness.nav_node.state == MissionState.PAUSED
        assert harness.nav_node._paused_resolution is not None
        assert harness.nav_node._paused_resolution['category'] == 'toilet'
        assert harness.nav_node._current_target_pose is None

        # Thả hold planner
        harness.planner_hold.set()

        # Gửi resume -> tiếp tục resolve và bắt đầu mission
        harness.nav_node.handle_resume_request('req_resume_res')
        harness.wait_state([MissionState.NAVIGATING, MissionState.STARTING], timeout=3.0)
        assert harness.nav_node._current_target_key in ('toilet_1', 'toilet_2')
    finally:
        if harness.planner_hold:
            harness.planner_hold.set()
        harness.shutdown()


def test_discard_old_feedback_and_freshness_status(ros_context):
    """
    Test 11: Feedback của goal cũ và kiểm tra độ tươi (freshness).
    """
    harness = NavTestHarness('act_feedback_fresh')
    try:
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_fb_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Gửi feedback hợp lệ cho attempt hiện tại
        curr_att = harness.nav_node._current_goal_attempt_id
        fb_msg = type('FbMsg', (), {
            'distance_remaining': 15.2,
            'estimated_time_remaining': type('Eta', (), {'sec': 25})()
        })()
        harness.nav_node.on_nav_feedback(fb_msg, curr_att)

        assert harness.nav_node._distance_remaining == 15.2
        assert harness.nav_node._eta_seconds == 25

        # Báo cáo trạng thái tươi
        harness.nav_node.handle_status_request('req_stat_1')
        assert any('15 mét' in s for s in harness.spoken_messages)

        # Feedback của attempt cũ không được cập nhật
        old_fb_msg = type('FbMsg', (), {
            'distance_remaining': 99.9,
            'estimated_time_remaining': type('Eta', (), {'sec': 100})()
        })()
        harness.nav_node.on_nav_feedback(old_fb_msg, 'att_outdated_999')
        assert harness.nav_node._distance_remaining == 15.2  # Không đổi!
    finally:
        harness.shutdown()


def test_request_deduplication(ros_context):
    """
    Test 12: Khử trùng lặp request_id (LRU Cache 200 items).
    """
    harness = NavTestHarness('act_dedup')
    try:
        # Gửi request lần 1
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_dup_test'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert len(harness.server.received_goals) == 1

        # Gửi lại cùng request_id ngay sau đó
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_dup_test'})))
        time.sleep(0.1)

        # Server không nhận thêm goal nào, đích vẫn giữ nguyên A3
        assert len(harness.server.received_goals) == 1
        assert harness.nav_node._current_target_key == 'gate_a3'
    finally:
        harness.shutdown()


def test_tts_synchronization_and_cancel_in_announcing(ros_context):
    """
    Test 13: Đồng bộ TTS qua speech_id và huỷ trong lúc ANNOUNCING.
    """
    harness = NavTestHarness('act_tts_sync')
    try:
        harness.nav_node.tts_ack_timeout = 4.0

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_tts_1'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)
        assert len(harness.server.received_goals) == 0

        pending_sp = harness.nav_node._pending_speech_id
        assert pending_sp is not None

        # ACK hợp lệ
        ack_payload = {'speech_id': pending_sp, 'status': 'COMPLETED'}
        harness.nav_node.on_tts_ack(String(data=json.dumps(ack_payload)))

        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert len(harness.server.received_goals) >= 1
    finally:
        harness.shutdown()


def test_speech_single_emission_per_event(ros_context):
    """
    Test 14: Phát sinh thông báo đúng một lần cho mỗi sự kiện tương ứng.
    """
    harness = NavTestHarness('act_single_speech')
    try:
        # 1. Navigate
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_sp_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        nav_speeches = [s for s in harness.spoken_messages if 'cổng A3' in s]
        assert len(nav_speeches) == 1

        # 2. Pause
        harness.spoken_messages.clear()
        harness.nav_node.handle_pause_request('req_sp_p1')
        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        pause_speeches = [s for s in harness.spoken_messages if 'tạm dừng' in s]
        assert len(pause_speeches) == 1

        # Xác nhận goal gửi lần đầu có z == 0.0 và orientation hợp lệ
        assert len(harness.server.received_goals) == 1
        first_pose = harness.server.received_goals[0].pose.pose
        assert first_pose.position.z == pytest.approx(0.0, abs=1e-6)

        # 3. Resume
        harness.spoken_messages.clear()
        harness.nav_node.handle_resume_request('req_sp_res')
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        resume_speeches = [s for s in harness.spoken_messages if 'tiếp tục di chuyển' in s]
        assert len(resume_speeches) == 1
        assert len(harness.server.received_goals) == 2
        resumed_pose = harness.server.received_goals[1].pose.pose
        assert resumed_pose.position.x == pytest.approx(first_pose.position.x, abs=1e-6)
        assert resumed_pose.position.y == pytest.approx(first_pose.position.y, abs=1e-6)
        assert resumed_pose.position.z == pytest.approx(0.0, abs=1e-6)
        assert resumed_pose.orientation.x == pytest.approx(first_pose.orientation.x, abs=1e-6)
        assert resumed_pose.orientation.y == pytest.approx(first_pose.orientation.y, abs=1e-6)
        assert resumed_pose.orientation.z == pytest.approx(first_pose.orientation.z, abs=1e-6)
        assert resumed_pose.orientation.w == pytest.approx(first_pose.orientation.w, abs=1e-6)

        # 4. Cancel
        harness.spoken_messages.clear()
        harness.nav_node.handle_cancel_request('req_sp_can')
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        cancel_speeches = [s for s in harness.spoken_messages if 'hủy chuyến đi' in s]
        assert len(cancel_speeches) == 1
    finally:
        harness.shutdown()


def test_acceptance_timeout_and_late_acceptance_revoked(ros_context):
    """
    Test 15: Acceptance timeout xảy ra (STARTING -> IDLE), sau đó server chấp nhận muộn.
    Xác nhận: Goal được hủy ngay, node duy trì IDLE, không bị lọt sang NAVIGATING.
    """
    harness = NavTestHarness('act_acc_to_late')
    try:
        harness.nav_node.goal_acceptance_timeout_sec = 0.3
        harness.server.hold_acceptance = threading.Event()

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_acc_to_1'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)

        # Chờ timeout xảy ra và node về IDLE
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert 'att_' in str(harness.nav_node._revoked_attempts)

        # Server chấp nhận muộn
        harness.server.hold_acceptance.set()

        # Chờ 0.5s để callback acceptance muộn được xử lý
        t0 = time.time()
        while len(harness.server.received_cancels) == 0 and time.time() - t0 < 3.0:
            time.sleep(0.02)

        # Goal muộn phải bị hủy ngay lập tức
        assert len(harness.server.received_cancels) >= 1
        # Trạng thái local vẫn là IDLE, không bị chuyển sang NAVIGATING
        assert harness.nav_node.state == MissionState.IDLE
        assert harness.nav_node._active_goal_handle is None
    finally:
        harness.shutdown()


def test_acceptance_timeout_new_request_late_acceptance_discarded(ros_context):
    """
    Test 16: Attempt 1 bị timeout. Yêu cầu mới Attempt 2 được gửi.
    Theo quy tắc an toàn: Attempt 2 bị từ chối chừng nào Attempt 1 chưa terminal.
    Sau khi Attempt 1 giải phóng acceptance muộn, bị thu hồi và terminal CANCELED,
    Attempt 2 mới được chấp nhận và chuyển sang NAVIGATING.
    """
    harness = NavTestHarness('act_acc_to_new_req')
    try:
        harness.nav_node.goal_acceptance_timeout_sec = 0.3
        harness.server.hold_acceptance = threading.Event()

        # 1. Gửi request 1
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_to_req1'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.nav_node._unresolved_attempts) >= 1

        # 2. Gửi request 2 khi Attempt 1 chưa giải quyết xong -> Phải bị từ chối
        harness.nav_node.goal_acceptance_timeout_sec = 4.0
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_to_req2'})))
        assert any(fb.get('status') == 'UNRESOLVED_GOALS_PENDING' for fb in harness.feedback_messages)
        assert harness.nav_node.state == MissionState.IDLE

        # 3. Server nhả acceptance cho Attempt 1 -> node thu hồi và hủy an toàn
        harness.server.hold_acceptance.set()
        t0 = time.time()
        while len(harness.nav_node._unresolved_attempts) > 0 and time.time() - t0 < 3.0:
            time.sleep(0.02)
        assert len(harness.nav_node._unresolved_attempts) == 0

        # 4. Khi Attempt 1 đã terminal sạch sẽ, gửi request 2 thành công
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_to_req3'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a4'

        # Goal 1 đã bị hủy và Goal 2 đang active
        assert len(harness.server.received_cancels) >= 1
    finally:
        harness.shutdown()


def test_late_acceptance_cancel_rejected(ros_context):
    """
    Test 17: Goal cũ được chấp nhận muộn nhưng server từ chối hủy.
    Xác nhận: Node ghi nhận cảnh báo REVOKED_GOAL_CANCEL_REJECTED, local state không bị crash.
    """
    harness = NavTestHarness('act_late_acc_rej')
    try:
        harness.nav_node.goal_acceptance_timeout_sec = 0.3
        harness.server.hold_acceptance = threading.Event()
        harness.server.accept_cancels = False  # Từ chối hủy

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_late_rej'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)
        harness.wait_state(MissionState.IDLE, timeout=2.0)

        # Server nhả acceptance muộn
        harness.server.hold_acceptance.set()

        t0 = time.time()
        while not any(fb.get('status') == 'REVOKED_GOAL_CANCEL_REJECTED' for fb in harness.feedback_messages) and time.time() - t0 < 3.0:
            time.sleep(0.02)

        assert any(fb.get('status') == 'REVOKED_GOAL_CANCEL_REJECTED' for fb in harness.feedback_messages)
        assert harness.nav_node.state == MissionState.IDLE
    finally:
        harness.shutdown()


def test_pending_acceptance_never_completes(ros_context):
    """
    Test 18: Server không bao giờ phản hồi acceptance (treo).
    Xác nhận: Node tự động timeout hữu hạn, về IDLE an toàn, không bị kẹt mãi ở STARTING.
    """
    harness = NavTestHarness('act_never_acc')
    try:
        harness.nav_node.goal_acceptance_timeout_sec = 0.4
        harness.server.hold_acceptance = threading.Event()  # Giữ mãi không set

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_never_acc'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)

        # Chờ timeout kích hoạt
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert any(fb.get('status') == 'ACCEPTANCE_TIMEOUT' for fb in harness.feedback_messages)
    finally:
        harness.shutdown()


def test_timer_isolation_between_operations(ros_context):
    """
    Test 19: Kiểm tra cô lập timer giữa các attempt/operation.
    Callback cũ của attempt 1 không được dừng timer acceptance của attempt 2.
    """
    harness = NavTestHarness('act_timer_iso')
    try:
        # Giả lập timer của attempt 2 đang chạy
        harness.nav_node._start_acceptance_timer('att_2')
        assert harness.nav_node._acceptance_timer_attempt == 'att_2'
        assert harness.nav_node._acceptance_timeout_timer is not None

        # Gọi _stop_acceptance_timer với attempt_id cũ 'att_1'
        harness.nav_node._stop_acceptance_timer('att_1')
        # Timer của att_2 vẫn phải tồn tại!
        assert harness.nav_node._acceptance_timeout_timer is not None
        assert harness.nav_node._acceptance_timer_attempt == 'att_2'

        # Gọi đúng att_2 -> timer mới được dọn dẹp
        harness.nav_node._stop_acceptance_timer('att_2')
        assert harness.nav_node._acceptance_timeout_timer is None
    finally:
        harness.shutdown()


def test_cancel_terminal_timeout_and_late_terminal(ros_context):
    """
    Test 20: Server chấp nhận cancel nhưng trì hoãn terminal result.
    Xác nhận: Kích hoạt cancel_terminal_timeout, ghi nhận unconfirmed terminal, chặn resume/replace.
    Khi terminal result đến muộn: Dọn dẹp trạng thái và cho phép hoạt động lại.
    """
    harness = NavTestHarness('act_can_term_to')
    try:
        harness.nav_node.cancel_terminal_timeout_sec = 0.3
        harness.server.hold_terminal = threading.Event()

        # Xuất phát và chuyển sang NAVIGATING
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_ctt_1'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)

        # Gửi pause -> server chấp nhận cancel nhưng kẹt ở hold_terminal
        harness.nav_node.handle_pause_request('req_pause_term')
        harness.wait_state(MissionState.PAUSING, timeout=2.0)

        # Chờ terminal timeout kích hoạt
        t0 = time.time()
        while not harness.nav_node._unconfirmed_terminal and time.time() - t0 < 2.0:
            time.sleep(0.02)
        assert harness.nav_node._unconfirmed_terminal is True
        assert any(fb.get('status') == 'TIMEOUT' and fb.get('event') == 'CANCEL_TERMINAL_TIMEOUT' for fb in harness.feedback_messages)

        # Lệnh resume bị từ chối do chưa xác nhận dừng
        harness.nav_node.handle_resume_request('req_resume_blocked')
        assert any(fb.get('status') == 'UNCONFIRMED_TERMINAL_PENDING' for fb in harness.feedback_messages)

        # Server nhả terminal result
        harness.server.hold_terminal.set()
        harness.wait_state(MissionState.PAUSED, timeout=3.0)
        assert harness.nav_node._unconfirmed_terminal is False
        assert harness.nav_node.state == MissionState.PAUSED
    finally:
        harness.shutdown()


def test_tts_strict_policy_aborts_on_timeout_and_error(ros_context):
    """
    Test 21: Chính sách tts_policy = 'strict'.
    Khi timeout chờ ACK hoặc nhận ACK lỗi -> Hủy gửi goal, về IDLE, feedback FAILED.
    """
    harness = NavTestHarness('act_tts_strict')
    try:
        harness.nav_node.tts_policy = 'strict'
        harness.nav_node.tts_ack_timeout = 0.3

        # Gửi navigate nhưng không trả ACK
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_strict_to'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)

        # Chờ timeout strict xảy ra
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert any(fb.get('status') == 'TTS_TIMEOUT_STRICT_ABORT' for fb in harness.feedback_messages)
        # Không có goal Nav2 nào được gửi
        assert len(harness.server.received_goals) == 0
    finally:
        harness.shutdown()


def test_tts_flexible_policy_fallbacks_on_timeout(ros_context):
    """
    Test 22: Chính sách tts_policy = 'flexible'.
    Khi timeout chờ ACK -> Fallback gửi goal Nav2 kèm feedback WARNING.
    """
    harness = NavTestHarness('act_tts_flex')
    try:
        harness.nav_node.tts_policy = 'flexible'
        harness.nav_node.tts_ack_timeout = 0.3

        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_flex_to'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)

        # Sau timeout flexible, goal được gửi và chuyển sang NAVIGATING
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert any(fb.get('status') == 'TTS_TIMEOUT_FALLBACK' for fb in harness.feedback_messages)
        assert len(harness.server.received_goals) >= 1
    finally:
        harness.shutdown()



def test_end_to_end_voice_text_replace_while_navigating(ros_context):
    """
    Test 23: Luồng đổi đích từ đầu vào giọng nói '/voice/text' qua IntentNode -> NavCommandNode
    khi robot đang NAVIGATING với câu 'đổi sang cổng A4'.
    Xác nhận:
    - Câu thoại ban đầu 'dẫn tôi đến cổng A3' đưa robot sang NAVIGATING.
    - Câu thoại 'đổi sang cổng A4' kích hoạt cancel goal 1 (A3).
    - Sau khi goal 1 dừng hẳn (terminal CANCELED), goal 2 mới được gửi tới A4.
    - Robot chuyển sang NAVIGATING với target 'gate_a4'.
    """
    harness = NavTestHarness('act_e2e_rep_nav')
    try:
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a3'
        assert len(harness.server.received_goals) == 1

        # Gửi câu đổi đích qua /voice/text
        harness.send_voice_text('đổi sang cổng A4')

        # Đợi cho đến khi goal 1 được hủy và goal 2 (gate_a4) chuyển sang NAVIGATING
        t0 = time.time()
        while (harness.nav_node._current_target_key != 'gate_a4' or harness.nav_node.state != MissionState.NAVIGATING) and time.time() - t0 < 5.0:
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a4'
        assert len(harness.server.received_cancels) >= 1
        assert len(harness.server.received_goals) == 2
    finally:
        harness.shutdown()


def test_end_to_end_voice_text_navigate_phrase_while_navigating(ros_context):
    """
    Test 24: Khi đang NAVIGATING mà nhận câu 'dẫn tôi đến cổng A4' thông thường.
    Xác nhận:
    - handle_navigate_request() nhận diện robot đang chạy và tự động chuyển sang luồng replace.
    - KHÔNG gọi thẳng start_specific_mission() bỏ qua replace.
    - Thứ tự hủy goal cũ rồi mới gửi goal mới được bảo đảm an toàn.
    """
    harness = NavTestHarness('act_e2e_nav_in_nav')
    try:
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a3'

        # Chặn terminal result của goal 1 để xác nhận goal 2 chưa được gửi
        harness.server.hold_terminal = threading.Event()

        # Người dùng nói câu dẫn đường thông thường trong lúc robot đang chạy
        harness.send_voice_text('dẫn tôi đến cổng A4')

        # Node phải ở PAUSING chờ goal cũ dừng
        t0 = time.time()
        while harness.nav_node.state != MissionState.PAUSING and time.time() - t0 < 2.0:
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.PAUSING
        assert len(harness.server.received_goals) == 1

        # Giải phóng terminal cho goal 1
        harness.server.hold_terminal.set()

        # Goal 2 được gửi và robot sang NAVIGATING với gate_a4
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a4'
        assert len(harness.server.received_goals) == 2
    finally:
        harness.shutdown()


def test_end_to_end_voice_text_replace_while_paused(ros_context):
    """
    Test 25: Đổi đích khi robot đang ở trạng thái PAUSED.
    Xác nhận:
    - Đích được cập nhật sang gate_a4.
    - Trạng thái robot VẪN GIỮ NGUYÊN là PAUSED (không tự ý chạy tiếp!).
    - Chỉ sau khi có lệnh 'đi tiếp', goal mới tới gate_a4 mới được gửi đi.
    """
    harness = NavTestHarness('act_e2e_rep_paused')
    try:
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)

        # Gửi pause
        harness.send_voice_text('dừng lại')
        harness.wait_state(MissionState.PAUSED, timeout=4.0)
        assert harness.nav_node.state == MissionState.PAUSED

        # Gửi lệnh đổi đích khi đang PAUSED
        harness.send_voice_text('đổi sang cổng A4')

        # Đợi xử lý intent
        t0 = time.time()
        while harness.nav_node._current_target_key != 'gate_a4' and time.time() - t0 < 2.0:
            time.sleep(0.02)

        # Đích đã đổi sang gate_a4 nhưng state VẪN LÀ PAUSED
        assert harness.nav_node._current_target_key == 'gate_a4'
        assert harness.nav_node.state == MissionState.PAUSED

        # Lệnh 'đi tiếp'
        harness.send_voice_text('đi tiếp')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a4'
    finally:
        harness.shutdown()


def test_replace_with_invalid_target_preserves_current_mission(ros_context):
    """
    Test 26: Đổi đích sang mục tiêu không hợp lệ khi đang NAVIGATING.
    Xác nhận:
    - Node phản hồi từ chối rõ ràng (feedback REJECTED, INVALID_NEW_TARGET).
    - Bảo toàn nguyên vẹn nhiệm vụ hiện tại (tiếp tục NAVIGATING tới đích cũ).
    """
    harness = NavTestHarness('act_rep_invalid')
    try:
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node._current_target_key == 'gate_a3'

        # Đổi sang cổng A99 không tồn tại
        harness.send_voice_text('đổi sang cổng A99')

        t0 = time.time()
        while not any('A99' in s or 'không hợp lệ' in s for s in harness.spoken_messages) and time.time() - t0 < 2.0:
            time.sleep(0.02)

        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a3'
        assert len(harness.server.received_cancels) == 0
    finally:
        harness.shutdown()


def test_unresolved_attempts_blocks_new_mission_until_cleared(ros_context):
    """
    Test 27: Goal 1 bị acceptance timeout. Attempt vẫn lưu trong _unresolved_attempts.
    Yêu cầu nhiệm vụ mới bị chặn (UNRESOLVED_GOALS_PENDING) cho tới khi goal cũ dừng hẳn.
    """
    harness = NavTestHarness('act_unresolved_block')
    try:
        harness.nav_node.goal_acceptance_timeout_sec = 0.3
        harness.server.hold_acceptance = threading.Event()

        # Gửi goal 1
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_unres_1'})))
        harness.wait_state(MissionState.STARTING, timeout=2.0)
        harness.wait_state(MissionState.IDLE, timeout=2.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.nav_node._unresolved_attempts) >= 1

        # Gửi goal 2 khi goal 1 chưa giải quyết xong -> Phải bị chặn
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_unres_2'})))
        assert any(fb.get('status') == 'UNRESOLVED_GOALS_PENDING' for fb in harness.feedback_messages)
        assert harness.nav_node.state == MissionState.IDLE

        # Server giải phóng acceptance muộn cho goal 1 -> node tự hủy và dọn dẹp
        harness.server.hold_acceptance.set()

        t0 = time.time()
        while len(harness.nav_node._unresolved_attempts) > 0 and time.time() - t0 < 3.0:
            time.sleep(0.02)
        assert len(harness.nav_node._unresolved_attempts) == 0

        # Bây giờ goal 2 gửi được bình thường
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a4', 'request_id': 'req_unres_3'})))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a4'
    finally:
        harness.shutdown()


def test_late_result_does_not_clear_new_goal_unconfirmed_terminal(ros_context):
    """
    Test 28: Goal mới đang có _unconfirmed_terminal = True.
    Một callback kết quả muộn từ goal cũ trở về -> KHÔNG ĐƯỢC xóa _unconfirmed_terminal của goal mới.
    """
    harness = NavTestHarness('act_late_res_iso')
    try:
        # Giả lập goal 2 đang có unconfirmed terminal
        harness.nav_node._current_goal_attempt_id = 'att_new_2'
        harness.nav_node._current_mission_id = 'mis_new_2'
        harness.nav_node._unconfirmed_terminal = True

        # Giả lập mock future của goal cũ
        class MockFuture:
            def result(self):
                class MockRes:
                    status = GoalStatus.STATUS_CANCELED
                return MockRes()

        # Gọi on_goal_result với attempt cũ 'att_old_1'
        harness.nav_node.on_goal_result(MockFuture(), 'att_old_1', 'mis_old_1')

        # _unconfirmed_terminal của goal 2 PHẢI VẪN LÀ TRUE!
        assert harness.nav_node._unconfirmed_terminal is True
    finally:
        harness.shutdown()


def test_status_inquiry_with_unresolved_attempts(ros_context):
    """
    Test 29: Hỏi trạng thái (STATUS) khi node ở IDLE nhưng vẫn còn attempt chưa giải quyết.
    Xác nhận: Phản hồi rõ ràng UNRESOLVED_GOALS_PENDING cho người dùng.
    """
    harness = NavTestHarness('act_status_unresolved')
    try:
        harness.nav_node._unresolved_attempts.add('att_dummy_pending')
        harness.nav_node.handle_status_request('req_status_test')

        assert any(fb.get('status') == 'UNRESOLVED_GOALS_PENDING' for fb in harness.feedback_messages)
        assert any('chuyến đi trước' in s or 'chưa hoàn tất' in s for s in harness.spoken_messages)
    finally:
        harness.shutdown()


def test_tts_strict_and_actual_ack_statuses(ros_context):
    """
    Test 30: Kiểm tra chi tiết các trạng thái TTS ACK thực tế từ TtsNode.
    - WARNING_NO_PLAYER: strict hủy goal, flexible fallback gửi goal.
    - WARNING_PLAYBACK_1: strict hủy goal, flexible fallback gửi goal.
    - ERROR_PIPER_SYNTHESIS_1: strict hủy goal, flexible fallback gửi goal.
    - COMPLETED với engine 'log': strict chấp nhận và gửi goal.
    - CANCELLED_BEFORE_PLAYBACK: không để node mắc lại ANNOUNCING.
    """
    harness = NavTestHarness('act_tts_actual_acks')
    try:
        harness.nav_node.tts_policy = 'strict'
        harness.nav_node.tts_ack_timeout = 2.0

        # 1. WARNING_NO_PLAYER trong strict -> abort to IDLE
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_ack_1'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)
        sp_id = harness.nav_node._pending_speech_id

        # Giả lập TtsNode phát ACK WARNING_NO_PLAYER
        ack_msg = json.dumps({'speech_id': sp_id, 'mission_id': harness.nav_node._current_mission_id, 'status': 'WARNING_NO_PLAYER', 'engine': 'piper', 'audio_played': False})
        harness.nav_node.on_tts_ack(String(data=ack_msg))
        assert harness.nav_node.state == MissionState.IDLE
        assert any('TTS_WARNING_NO_PLAYER' in fb.get('status', '') for fb in harness.feedback_messages)

        # 2. CANCELLED_BEFORE_PLAYBACK -> thoát ANNOUNCING về IDLE
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_ack_2'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)
        sp_id2 = harness.nav_node._pending_speech_id
        ack_cancel = json.dumps({'speech_id': sp_id2, 'mission_id': harness.nav_node._current_mission_id, 'status': 'CANCELLED_BEFORE_PLAYBACK', 'engine': 'log', 'audio_played': False})
        harness.nav_node.on_tts_ack(String(data=ack_cancel))
        assert harness.nav_node.state == MissionState.IDLE

        # 3. COMPLETED với engine 'log' -> chấp nhận và sang STARTING / NAVIGATING
        harness.nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_ack_3'})))
        harness.wait_state(MissionState.ANNOUNCING, timeout=2.0)
        sp_id3 = harness.nav_node._pending_speech_id
        ack_log = json.dumps({'speech_id': sp_id3, 'mission_id': harness.nav_node._current_mission_id, 'status': 'COMPLETED', 'engine': 'log', 'audio_played': False})
        harness.nav_node.on_tts_ack(String(data=ack_log))
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
    finally:
        harness.shutdown()


def test_heartbeat_with_sim_time_and_no_clock(ros_context):
    """
    Test 31: Heartbeat khi use_sim_time=True nhưng /clock không phát (ví dụ Gazebo pause hoặc chưa chạy).
    Xác nhận:
    - nav_command_node sử dụng steady clock cho _heartbeat_timer nên timer vẫn phát đều đặn 1.0s.
    - intent_node đo thời gian bằng time.monotonic() nên nhận diện nav_command_node online (is_nav_online == True).
    - Phân luồng câu hỏi dẫn đường sang action 'navigate' chính xác thay vì fallback trả lời offline.
    """
    from rclpy.parameter import Parameter
    nav_node = NavCommandNode()
    nav_node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    intent_node = IntentNode()
    intent_node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    intent_node.nav_timeout = 3.0

    executor = MultiThreadedExecutor()
    executor.add_node(nav_node)
    executor.add_node(intent_node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        # Chờ heartbeat đầu tiên được IntentNode ghi nhận qua /voice/nav_state
        t0 = time.monotonic()
        while not intent_node.is_nav_online and (time.monotonic() - t0 < 3.0):
            time.sleep(0.05)

        assert intent_node.is_nav_online is True, "IntentNode phải nhận diện NavCommandNode online dù không có /clock"

        # Gửi transcript câu thoại và kiểm tra intent_node phân luồng
        intent_received = []
        sub_intent = nav_node.create_subscription(
            String, '/voice/intent', lambda m: intent_received.append(json.loads(m.data)), 10
        )
        pub_text = nav_node.create_publisher(String, '/voice/text', 10)
        time.sleep(0.1)

        pub_text.publish(String(data='dẫn tôi đến cổng A3'))
        t1 = time.monotonic()
        while not intent_received and (time.monotonic() - t1 < 2.0):
            time.sleep(0.05)

        assert len(intent_received) > 0, "Phải nhận được intent message"
        assert intent_received[0].get('action') == 'navigate'
        assert intent_received[0].get('target_id') == 'gate_a3'
    finally:
        executor.shutdown()
        nav_node.destroy_node()
        intent_node.destroy_node()


def test_cancel_during_replace_clears_pending_replace_and_cancels(ros_context):
    """
    Test 32: Khách yêu cầu đổi đích (A3 -> A4), sau đó nói 'hủy chuyến' trong lúc goal cũ đang hủy.
    Xác nhận:
    - Goal A3 bị hủy (CANCELED).
    - Pending replace 'gate_a4' bị xóa hoàn toàn, KHÔNG gửi goal A4 sang Nav2.
    - Máy trạng thái về IDLE an toàn.
    - Câu thông báo hủy 'Tôi đã hủy chuyến đi hiện tại.' chỉ phát đúng 1 lần, không thông báo kép hay đổi đích.
    """
    harness = NavTestHarness('act_cancel_during_rep')
    try:
        # 1. Bắt đầu đi tới A3
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING
        assert harness.nav_node._current_target_key == 'gate_a3'
        assert len(harness.server.received_goals) == 1

        # Chặn terminal result của goal 1
        harness.server.hold_terminal = threading.Event()

        # 2. Khách yêu cầu đổi sang A4
        harness.send_voice_text('đổi sang cổng A4')
        t0 = time.time()
        while harness.nav_node.state != MissionState.PAUSING and (time.time() - t0 < 2.0):
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.PAUSING
        assert harness.nav_node._pending_replace_target == 'gate_a4'

        # 3. Khách nói 'hủy chuyến' ngay khi goal 1 đang hủy
        harness.send_voice_text('hủy chuyến')
        t1 = time.time()
        while harness.nav_node.state != MissionState.CANCELING and (time.time() - t1 < 2.0):
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.CANCELING
        assert harness.nav_node._pending_replace_target is None, "Pending replace target phải bị xóa ngay"

        # 4. Cho phép goal 1 kết thúc terminal CANCELED
        harness.server.hold_terminal.set()

        # Đợi robot chuyển về IDLE
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE

        # Xác nhận KHÔNG gửi goal A4 sang server
        assert len(harness.server.received_goals) == 1, "Tuyệt đối không gửi goal thứ 2 khi đã hủy"
        assert harness.nav_node._pending_replace_target is None

        # Xác nhận câu thông báo hủy phát đúng 1 lần, không có thông báo đổi lộ trình A4
        cancel_speeches = [s for s in harness.spoken_messages if 'đã hủy chuyến đi' in s]
        assert len(cancel_speeches) == 1, f"Câu hủy chuyến phải phát đúng 1 lần, tìm thấy: {cancel_speeches}"
        assert not any('cổng A4' in s for s in harness.spoken_messages), "Không được phát thông báo tới A4"
    finally:
        harness.shutdown()


def test_cancel_during_category_replace_clears_pending_category(ros_context):
    """
    Test 33: Khách yêu cầu đổi sang nhóm (toilet), sau đó nói 'hủy chuyến' trong lúc goal cũ đang hủy.
    Xác nhận:
    - Pending replace category bị xóa.
    - Không gọi planner để chọn điểm trong nhóm.
    - Robot về IDLE và thông báo hủy đúng 1 lần.
    """
    harness = NavTestHarness('act_cancel_cat_rep')
    try:
        harness.send_voice_text('dẫn tôi đến cổng A3')
        harness.wait_state(MissionState.NAVIGATING, timeout=4.0)
        assert harness.nav_node.state == MissionState.NAVIGATING

        harness.server.hold_terminal = threading.Event()

        # Yêu cầu đổi sang nhà vệ sinh
        harness.send_voice_text('đổi sang nhà vệ sinh')
        t0 = time.time()
        while harness.nav_node.state != MissionState.PAUSING and (time.time() - t0 < 2.0):
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.PAUSING
        assert harness.nav_node._pending_replace_category == 'toilet'

        # Nói 'hủy chuyến'
        harness.send_voice_text('hủy chuyến')
        t1 = time.time()
        while harness.nav_node.state != MissionState.CANCELING and (time.time() - t1 < 2.0):
            time.sleep(0.02)
        assert harness.nav_node.state == MissionState.CANCELING
        assert harness.nav_node._pending_replace_category is None

        # Cho goal 1 kết thúc
        harness.server.hold_terminal.set()
        harness.wait_state(MissionState.IDLE, timeout=3.0)
        assert harness.nav_node.state == MissionState.IDLE
        assert len(harness.server.received_goals) == 1
    finally:
        harness.shutdown()


def test_delayed_cancel_terminal_result_statuses(ros_context):
    """
    Test 34: Kiểm tra toàn diện terminal result của goal thu hồi muộn và cô lập attempt:
    - Trong CANCELING:
      + CANCELED: Báo đã hủy, feedback CANCELED OK, giải phóng attempt khỏi _unresolved_attempts.
      + SUCCEEDED: Báo goal đã tới đích trước khi hủy có hiệu lực, feedback SUCCEEDED ARRIVED_BEFORE_CANCEL, giải phóng attempt.
      + ABORTED: Báo lỗi điều hướng, feedback FAILED ABORTED, giải phóng attempt.
      + Exception / UNKNOWN: Báo chưa xác nhận được trạng thái goal, GIỮ attempt trong _unresolved_attempts, đặt _unconfirmed_terminal=True, tiếp tục chặn gửi goal mới.
    - Khôi phục (Recovery): Lệnh hủy trong IDLE xóa sạch attempt chưa giải quyết và đưa hệ thống về trạng thái sẵn sàng.
    - Cô lập attempt cũ trong cùng mission: Callback của attempt cũ (att_old != att_current) không thay đổi mission, không đổi đích, không kích hoạt pending replace.
    """
    node = NavCommandNode()
    try:
        class MockFuture:
            def __init__(self, status=None, exc=None):
                self._status = status
                self._exc = exc
            def result(self):
                if self._exc:
                    raise self._exc
                class Res:
                    pass
                r = Res()
                r.status = self._status
                return r

        spoken = []
        node.say = lambda t, s=None, m=None: spoken.append(t)
        feedbacks = []
        node.publish_feedback = lambda ev, st, msg='': feedbacks.append({'event': ev, 'status': st, 'msg': msg})

        # 1. CANCELING -> CANCELED
        node._current_mission_id = 'mis_can_1'
        node._current_goal_attempt_id = 'att_can_1'
        node._unresolved_attempts = {'att_can_1'}
        node.state = MissionState.CANCELING

        spoken.clear()
        feedbacks.clear()
        node._on_delayed_cancel_terminal_result(MockFuture(status=GoalStatus.STATUS_CANCELED), 'att_can_1', 'mis_can_1')
        assert node.state == MissionState.IDLE
        assert 'att_can_1' not in node._unresolved_attempts
        assert any(fb['event'] == 'CANCELED' for fb in feedbacks)
        assert any('đã hủy chuyến đi' in s for s in spoken)

        # 2. CANCELING -> SUCCEEDED (đã tới đích trước khi hủy)
        node._current_mission_id = 'mis_can_2'
        node._current_goal_attempt_id = 'att_can_2'
        node._current_target_name = 'cổng A3'
        node._unresolved_attempts = {'att_can_2'}
        node.state = MissionState.CANCELING

        spoken.clear()
        feedbacks.clear()
        node._on_delayed_cancel_terminal_result(MockFuture(status=GoalStatus.STATUS_SUCCEEDED), 'att_can_2', 'mis_can_2')
        assert node.state == MissionState.IDLE
        assert 'att_can_2' not in node._unresolved_attempts
        assert any(fb['event'] == 'SUCCEEDED' and fb['status'] == 'ARRIVED_BEFORE_CANCEL' for fb in feedbacks)
        assert any('trước khi lệnh hủy có hiệu lực' in s for s in spoken)

        # 3. CANCELING -> ABORTED (lỗi điều hướng)
        node._current_mission_id = 'mis_can_3'
        node._current_goal_attempt_id = 'att_can_3'
        node._unresolved_attempts = {'att_can_3'}
        node.state = MissionState.CANCELING

        spoken.clear()
        feedbacks.clear()
        node._on_delayed_cancel_terminal_result(MockFuture(status=GoalStatus.STATUS_ABORTED), 'att_can_3', 'mis_can_3')
        assert node.state == MissionState.IDLE
        assert 'att_can_3' not in node._unresolved_attempts
        assert any(fb['event'] == 'FAILED' and fb['status'] == 'ABORTED' for fb in feedbacks)

        # 4. CANCELING -> Exception / STATUS_UNKNOWN (chưa xác nhận được trạng thái goal)
        node._current_mission_id = 'mis_can_4'
        node._current_goal_attempt_id = 'att_can_4'
        node._unresolved_attempts = {'att_can_4'}
        node.state = MissionState.CANCELING

        spoken.clear()
        feedbacks.clear()
        node._on_delayed_cancel_terminal_result(MockFuture(exc=RuntimeError('RPC lost')), 'att_can_4', 'mis_can_4')
        assert node.state == MissionState.IDLE
        # Phải GIỮ LẠI attempt trong _unresolved_attempts và _unconfirmed_terminal=True
        assert 'att_can_4' in node._unresolved_attempts
        assert node._unconfirmed_terminal is True
        assert any(fb['event'] == 'FAILED' and fb['status'] == 'CANCEL_UNCONFIRMED_TERMINAL' for fb in feedbacks)
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)

        # Thử gửi goal mới khi còn goal chưa xác nhận -> PHẢI BỊ CHẶN
        feedbacks.clear()
        node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_blocked'})))
        assert any(fb['status'] == 'UNRESOLVED_GOALS_PENDING' for fb in feedbacks)
        assert node.state == MissionState.IDLE

        # Khi gửi lệnh cancel trong IDLE khi còn attempt chưa xác minh: GIỮ GATE ĐÓNG, BÁO NGẮN, KHÔNG XÓA ATTEMPT
        feedbacks.clear()
        spoken.clear()
        node.handle_cancel_request('req_clear_recovery')
        assert len(node._unresolved_attempts) > 0
        assert node._unconfirmed_terminal is True
        assert any(fb['status'] == 'UNCONFIRMED_TERMINAL_PENDING' for fb in feedbacks)
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)

        # Giải phóng khi nhận terminal xác nhận thực sự
        node._on_delayed_cancel_terminal_result(MockFuture(status=GoalStatus.STATUS_CANCELED), 'att_can_4', 'mis_can_4')
        assert len(node._unresolved_attempts) == 0
        assert node._unconfirmed_terminal is False

        # 5. Callback của attempt cũ trong cùng mission không tác động attempt mới
        node._current_mission_id = 'mis_same'
        node._current_goal_attempt_id = 'att_current_new'
        node._current_target_key = 'gate_a3'
        node._pending_replace_target = 'gate_a4'
        node.state = MissionState.PAUSING
        node._unresolved_attempts = {'att_old_revoked', 'att_current_new'}

        spoken.clear()
        feedbacks.clear()
        # Callback của att_old_revoked về SUCCEEDED
        node._on_delayed_cancel_terminal_result(MockFuture(status=GoalStatus.STATUS_SUCCEEDED), 'att_old_revoked', 'mis_same')
        # att_old_revoked được giải phóng
        assert 'att_old_revoked' not in node._unresolved_attempts
        # Nhưng att_current_new vẫn còn
        assert 'att_current_new' in node._unresolved_attempts
        # Và trạng thái hiện tại KHÔNG bị thay đổi
        assert node.state == MissionState.PAUSING
        assert node._current_goal_attempt_id == 'att_current_new'
        assert node._pending_replace_target == 'gate_a4'
        assert node._current_target_key == 'gate_a3'
    finally:
        node.destroy_node()


def test_unconfirmed_attempts_multi_cancel_and_gate(ros_context):
    """
    Test 35: Kiểm tra UNKNOWN và exception, gửi lệnh hủy nhiều lần, chặn navigate/resume/replace:
    - Khi có attempt gặp UNKNOWN hoặc exception: gate đóng.
    - Gửi 'hủy chuyến' nhiều lần ở IDLE: luôn bị từ chối với thông báo ngắn, gate tiếp tục đóng.
    - Thử gửi goal mới, resume, replace: đều bị chặn, số attempt chưa xác minh không đổi.
    - Số goal thực sự gửi tới Nav2 không tăng.
    """
    node = NavCommandNode()
    try:
        class MockGoalHandle:
            def __init__(self, accepted=True):
                self.accepted = accepted
                self.cancel_called = 0
            def cancel_goal_async(self):
                self.cancel_called += 1
                class Fut:
                    def add_done_callback(self, cb):
                        pass
                return Fut()
            def get_result_async(self):
                class Fut:
                    def add_done_callback(self, cb):
                        pass
                return Fut()

        spoken = []
        node.say = lambda t, s=None, m=None: spoken.append(t)
        feedbacks = []
        node.publish_feedback = lambda ev, st, msg='': feedbacks.append({'event': ev, 'status': st, 'msg': msg})

        # Giả lập mock nav_client
        mock_goals_sent = []
        node.nav_client.send_goal_async = lambda g, feedback_callback=None: mock_goals_sent.append(g)

        # Tạo một attempt bị exception
        rec = node._get_or_create_attempt('att_unk_1', 'mis_unk_1')
        gh = MockGoalHandle(accepted=True)
        rec.goal_handle = gh
        node._mark_attempt_unconfirmed('att_unk_1', GoalStatus.STATUS_UNKNOWN)

        assert node.has_unresolved_goals() is True
        assert node._unconfirmed_terminal is True
        assert len(node._unresolved_attempts) == 1

        # Gửi 'hủy chuyến' lần 1
        spoken.clear()
        feedbacks.clear()
        node.handle_cancel_request('req_cancel_1')
        assert gh.cancel_called == 1
        assert node.has_unresolved_goals() is True
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)
        assert any(fb['status'] == 'UNCONFIRMED_TERMINAL_PENDING' for fb in feedbacks)

        # Gửi 'hủy chuyến' lần 2
        spoken.clear()
        feedbacks.clear()
        node.handle_cancel_request('req_cancel_2')
        assert gh.cancel_called == 2
        assert node.has_unresolved_goals() is True
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)

        # Thử gửi goal mới (navigate) -> bị chặn
        spoken.clear()
        feedbacks.clear()
        node.handle_navigate_request({'target_id': 'gate_a3', 'request_id': 'req_nav_blocked'})
        assert len(mock_goals_sent) == 0
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)
        assert any(fb['status'] == 'UNRESOLVED_GOALS_PENDING' for fb in feedbacks)

        # Thử resume -> bị chặn
        spoken.clear()
        feedbacks.clear()
        node.handle_resume_request('req_resume_blocked')
        assert len(mock_goals_sent) == 0
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)
        assert any(fb['status'] in ('UNCONFIRMED_TERMINAL_PENDING', 'UNRESOLVED_GOALS_PENDING') for fb in feedbacks)

        # Thử replace -> bị chặn
        spoken.clear()
        feedbacks.clear()
        node._pending_replace_target = 'gate_a4'
        node._execute_pending_replace()
        assert len(mock_goals_sent) == 0
        assert any('Chưa xác nhận được chuyến đi cũ đã kết thúc' in s for s in spoken)
        assert any(fb['status'] == 'UNRESOLVED_GOALS_PENDING' for fb in feedbacks)

        # Mock server không nhận thêm goal nào
        assert len(mock_goals_sent) == 0
        assert len(node._unresolved_attempts) == 1

        # Cuối cùng, terminal result CANCELED về
        node._confirm_attempt_terminal('att_unk_1', GoalStatus.STATUS_CANCELED)
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False
        assert len(node._unresolved_attempts) == 0
    finally:
        node.destroy_node()


def test_cancel_acceptance_rejection_and_multi_attempt_terminal(ros_context):
    """
    Test 36: Kiểm tra cancel accepted vs rejected, late acceptance, và multi-attempt terminal:
    - Cancel accepted (return_code=0) chỉ là trung gian, gate vẫn đóng cho tới terminal result.
    - Cancel rejected (return_code!=0): ghi nhận CANCEL_REJECTED, gate vẫn đóng.
    - Acceptance muộn: tự động gửi cancel, gate vẫn đóng.
    - Khi có 2 attempt cùng chưa xác minh: 1 cái terminal CANCELED nhưng cái kia chưa xong -> gate VẪN ĐÓNG.
      Chỉ mở gate khi TẤT CẢ attempt đều đã confirm.
    """
    node = NavCommandNode()
    try:
        spoken = []
        node.say = lambda t, s=None, m=None: spoken.append(t)
        feedbacks = []
        node.publish_feedback = lambda ev, st, msg='': feedbacks.append({'event': ev, 'status': st, 'msg': msg})

        class MockCancelFuture:
            def __init__(self, return_code=0, goals_canceling=1):
                self.return_code = return_code
                self.goals_canceling = [object()] if goals_canceling else []
            def result(self):
                return self

        class MockTermFuture:
            def __init__(self, status):
                self.status = status
            def result(self):
                return self

        # 1. Cancel accepted (return_code=0)
        rec1 = node._get_or_create_attempt('att_c1', 'mis_c1')
        node._on_delayed_cancel_confirmed(MockCancelFuture(return_code=0, goals_canceling=1), 'att_c1')
        assert rec1.state == 'CANCEL_ACCEPTED'
        assert rec1.is_terminal_confirmed is False
        assert node.has_unresolved_goals() is True

        # 2. Cancel rejected (return_code=1)
        rec2 = node._get_or_create_attempt('att_c2', 'mis_c2')
        node._on_delayed_cancel_confirmed(MockCancelFuture(return_code=1, goals_canceling=0), 'att_c2')
        assert rec2.state == 'CANCEL_REJECTED'
        assert rec2.is_terminal_confirmed is False
        assert node.has_unresolved_goals() is True

        # 3. Multi-attempt: att_c1 terminal CANCELED, nhưng att_c2 chưa terminal
        node._on_delayed_cancel_terminal_result(MockTermFuture(GoalStatus.STATUS_CANCELED), 'att_c1', 'mis_c1')
        assert rec1.is_terminal_confirmed is True
        assert rec2.is_terminal_confirmed is False
        # Gate VẪN ĐÓNG vì att_c2 chưa xác minh!
        assert node.has_unresolved_goals() is True

        # Thử gửi navigate -> vẫn bị chặn vì att_c2
        feedbacks.clear()
        node.handle_navigate_request({'target_id': 'gate_a3', 'request_id': 'req_blocked_by_c2'})
        assert any(fb['status'] == 'UNRESOLVED_GOALS_PENDING' for fb in feedbacks)

        # Bây giờ att_c2 cũng nhận terminal ABORTED
        node._on_delayed_cancel_terminal_result(MockTermFuture(GoalStatus.STATUS_ABORTED), 'att_c2', 'mis_c2')
        assert rec2.is_terminal_confirmed is True
        # TẤT CẢ attempt đều đã confirm -> gate mở!
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False

        # 4. Late acceptance: attempt timeout trước đó, sau đó server accept muộn
        rec3 = node._get_or_create_attempt('att_late', 'mis_late')
        node._revoked_attempts.add('att_late')
        cancel_called = []
        class MockLateGoalHandle:
            accepted = True
            def cancel_goal_async(self):
                cancel_called.append(True)
                class F:
                    def add_done_callback(self, cb):
                        pass
                return F()
            def get_result_async(self):
                class F:
                    def add_done_callback(self, cb):
                        pass
                return F()

        class MockRespFuture:
            def result(self):
                return MockLateGoalHandle()

        node.on_goal_response(MockRespFuture(), 'att_late', 'mis_late')
        assert len(cancel_called) == 1
        assert rec3.state == 'CANCEL_REQUESTED'
        assert rec3.is_terminal_confirmed is False
        assert node.has_unresolved_goals() is True

        # Khi terminal của late goal về
        node._on_delayed_cancel_terminal_result(MockTermFuture(GoalStatus.STATUS_CANCELED), 'att_late', 'mis_late')
        assert rec3.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
    finally:
        node.destroy_node()


def test_old_attempt_reject_callback_same_mission_does_not_affect_current_attempt(ros_context):
    """
    Test 37: Callback reject của attempt cũ cùng mission chỉ được cập nhật bản ghi của chính nó.
    Kiểm tra cả mission_id và attempt_id trước khi đổi trạng thái hoặc gọi pending replace.
    - att_1 (cũ) và att_2 (mới) cùng thuộc mission_1.
    - att_1 bị reject muộn -> chỉ cập nhật bản ghi att_1 thành REJECTED.
    - att_2 vẫn tiếp tục NAVIGATING, không bị đổi sang IDLE, không gọi pending replace nhầm.
    - Số attempt chưa xác minh giảm cho att_1, att_2 vẫn chưa xác minh cho tới khi hoàn tất.
    """
    node = NavCommandNode()
    try:
        spoken = []
        node.say = lambda t, s=None, m=None: spoken.append(t)
        feedbacks = []
        node.publish_feedback = lambda ev, st, msg='': feedbacks.append({'event': ev, 'status': st, 'msg': msg})

        # Thiết lập mission hiện tại có attempt att_2 đang NAVIGATING
        node._current_mission_id = 'mis_same'
        node._current_goal_attempt_id = 'att_current_2'
        node._current_target_key = 'gate_a3'
        node._current_target_name = 'cổng A3'
        node.state = MissionState.NAVIGATING

        # Tạo bản ghi cho attempt cũ att_1 và attempt hiện tại att_2
        rec1 = node._get_or_create_attempt('att_old_1', 'mis_same')
        rec2 = node._get_or_create_attempt('att_current_2', 'mis_same')

        assert node.has_unresolved_goals() is True
        assert len(node._unresolved_attempts) == 2

        # Đặt một pending replace giả lập
        node._pending_replace_target = 'gate_a4'
        replace_called = []
        node._execute_pending_replace = lambda: replace_called.append(True)

        class MockRejectedGoalHandle:
            accepted = False

        class MockRejectFuture:
            def result(self):
                return MockRejectedGoalHandle()

        # Callback reject của attempt cũ att_old_1 về
        node.on_goal_response(MockRejectFuture(), 'att_old_1', 'mis_same')

        # 1. Bản ghi att_old_1 được cập nhật chính xác
        assert rec1.state == 'REJECTED'
        assert rec1.is_terminal_confirmed is True
        assert 'att_old_1' not in node._unresolved_attempts

        # 2. Chuyến hiện tại KHÔNG bị tác động:
        assert node._current_goal_attempt_id == 'att_current_2'
        assert node.state == MissionState.NAVIGATING
        assert len(replace_called) == 0  # Không bị kích hoạt replace nhầm bởi att cũ
        assert not any('Tôi không thể di chuyển' in s for s in spoken)  # Không phát thông báo lỗi cho chuyến hiện tại
        assert not any(fb.get('status') == 'GOAL_REJECTED' for fb in feedbacks)

        # 3. Gate vẫn đóng vì att_current_2 vẫn đang chạy:
        assert node.has_unresolved_goals() is True
        assert len(node._unresolved_attempts) == 1
        assert 'att_current_2' in node._unresolved_attempts

        # 4. Khi att_current_2 kết thúc SUCCEEDED:
        class MockTermFuture:
            def __init__(self, status):
                self.status = status
            def result(self):
                return self

        node.on_goal_result(MockTermFuture(GoalStatus.STATUS_SUCCEEDED), 'att_current_2', 'mis_same')
        assert rec2.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False
        assert len(node._unresolved_attempts) == 0
    finally:
        node.destroy_node()


def test_confirmed_terminal_attempt_not_reverted_by_late_unknown_or_repeated_cancel(ros_context):
    """
    Test 38: Khi một attempt đã có terminal xác nhận, callback trả UNKNOWN hoặc exception về sau
    không được làm attempt trở lại chưa xác minh.
    Kiểm tra cả các callback lặp do người dùng gửi "hủy chuyến" nhiều lần.
    """
    node = NavCommandNode()
    try:
        spoken = []
        node.say = lambda t, s=None, m=None: spoken.append(t)
        feedbacks = []
        node.publish_feedback = lambda ev, st, msg='': feedbacks.append({'event': ev, 'status': st, 'msg': msg})

        class MockTermFuture:
            def __init__(self, status):
                self.status = status
            def result(self):
                return self

        class MockExceptionFuture:
            def result(self):
                raise RuntimeError("Connection broken to Nav2 action server")

        # 1. Tạo attempt và xác nhận terminal CANCELED
        rec = node._get_or_create_attempt('att_test_38', 'mis_38')
        node._confirm_attempt_terminal('att_test_38', GoalStatus.STATUS_CANCELED)

        assert rec.is_terminal_confirmed is True
        assert rec.terminal_status == GoalStatus.STATUS_CANCELED
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False

        # 2. Callback muộn/lặp trả về STATUS_UNKNOWN -> KHÔNG được làm attempt trở lại unconfirmed
        node._on_delayed_cancel_terminal_result(MockTermFuture(GoalStatus.STATUS_UNKNOWN), 'att_test_38', 'mis_38')
        assert rec.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False
        assert 'att_test_38' not in node._unresolved_attempts

        # 3. Callback muộn/lặp ném Exception -> KHÔNG được làm attempt trở lại unconfirmed
        node._on_delayed_cancel_terminal_result(MockExceptionFuture(), 'att_test_38', 'mis_38')
        assert rec.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False

        # 4. Callback on_goal_result với STATUS_UNKNOWN -> KHÔNG được làm attempt trở lại unconfirmed
        node.on_goal_result(MockTermFuture(GoalStatus.STATUS_UNKNOWN), 'att_test_38', 'mis_38')
        assert rec.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False

        # 5. Gọi _mark_attempt_unconfirmed trực tiếp -> bị chặn nếu đã terminal_confirmed
        node._mark_attempt_unconfirmed('att_test_38', GoalStatus.STATUS_UNKNOWN)
        assert rec.is_terminal_confirmed is True
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False

        # 6. Người dùng gửi "hủy chuyến" nhiều lần khi ở IDLE
        node.state = MissionState.IDLE
        spoken.clear()
        feedbacks.clear()
        mock_cancel_goals_count = 0

        class MockGoalHandle:
            def cancel_goal_async(self):
                nonlocal mock_cancel_goals_count
                mock_cancel_goals_count += 1
                class F:
                    def add_done_callback(self, cb):
                        pass
                return F()
            def get_result_async(self):
                class F:
                    def add_done_callback(self, cb):
                        pass
                return F()

        rec.goal_handle = MockGoalHandle()

        # Gửi hủy chuyến lần 1, lần 2, lần 3
        for i in range(3):
            node.handle_cancel_request(f'req_cancel_38_{i}')

        # Vì att_test_38 đã is_terminal_confirmed = True, hệ thống nhận diện không có chuyến nào cần hủy
        assert any('Hiện không có chuyến đi nào đang hoạt động' in s for s in spoken)
        assert mock_cancel_goals_count == 0  # Không spam gọi cancel lên goal đã kết thúc
        assert node.has_unresolved_goals() is False
        assert node._unconfirmed_terminal is False
    finally:
        node.destroy_node()
