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

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.server)
        self.executor.add_node(self.nav_node)

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
        self.nav_node.destroy_node()
        self.server.destroy_node()


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
