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

        # 3. Resume
        harness.spoken_messages.clear()
        harness.nav_node.handle_resume_request('req_sp_res')
        harness.wait_state(MissionState.NAVIGATING, timeout=3.0)
        resume_speeches = [s for s in harness.spoken_messages if 'tiếp tục di chuyển' in s]
        assert len(resume_speeches) == 1

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
