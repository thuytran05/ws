#!/usr/bin/env python3
"""
test_planner_resolution.py — Kiểm thử chọn đích bằng planner (ComputePathToPose).
Bao phủ toàn diện GIAI ĐOẠN 1 & 3:
  1. Điểm đầu (toilet_1) có đường dài hơn điểm thứ hai (toilet_2) -> Đích chọn toilet_2.
  2. Điểm đầu không thể tới (NO_VALID_PATH) -> Đích chọn toilet_2.
  3. Tất cả các điểm không thể tới -> Báo lỗi NO_VALID_PATH và về IDLE.
  4. Lệnh cancel trong khi đang chờ planner -> Hủy resolve và chuyển IDLE.
  5. Ứng viên đầu timeout, kết quả cũ về lúc đang tính ứng viên thứ hai -> Bỏ qua kết quả cũ, chọn ứng viên hợp lệ.
  6. Cancel rồi bắt đầu phiên mới, callback cũ về muộn -> Phiên mới không bị ảnh hưởng.
  7. Replace từ đích cụ thể sang nhóm category -> Chạy đúng logic chọn đường của category.
  8. Planner server chưa sẵn sàng -> Báo chưa thể chọn điểm tối ưu và fallback rõ ràng.
"""

import json
import math
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from airport_robot_voice.nav_command_node import NavCommandNode, MissionState


class MockPlannerAndNavServer(Node):
    """Mock Action Server cho ComputePathToPose và NavigateToPose."""
    def __init__(self, planner_action_name='mock_compute_path_to_pose', nav_action_name='mock_navigate_to_pose'):
        super().__init__('mock_planner_and_nav_server_' + planner_action_name.replace('/', '_'))
        self.planner_action_name = planner_action_name
        self.nav_action_name = nav_action_name

        self.path_lengths = {}      # target_x -> length
        self.unreachable_x = set()  # set of x coordinates that return error
        self.delayed_x = {}         # target_x -> delay_sec
        self.error_code_x = {}      # target_x -> error_code
        self.planner_received_goals = []
        self.nav_received_goals = []

        self._planner_server = ActionServer(
            self,
            ComputePathToPose,
            self.planner_action_name,
            execute_callback=self._execute_planner,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT
        )

        self._nav_server = ActionServer(
            self,
            NavigateToPose,
            self.nav_action_name,
            execute_callback=self._execute_nav,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT
        )

    def _execute_planner(self, goal_handle):
        req = goal_handle.request
        target_x = round(req.goal.pose.position.x, 2)
        self.planner_received_goals.append(target_x)

        if target_x in self.delayed_x:
            time.sleep(self.delayed_x[target_x])

        res = ComputePathToPose.Result()
        path = Path()
        path.header.frame_id = 'map'

        if target_x in self.unreachable_x or target_x in self.error_code_x:
            res.error_code = self.error_code_x.get(target_x, 208)
            res.path = path
            goal_handle.succeed()
            return res

        length = self.path_lengths.get(target_x, 10.0)
        p1 = PoseStamped()
        p1.pose.position.x = 0.0
        p1.pose.position.y = 0.0

        p2 = PoseStamped()
        p2.pose.position.x = float(length)
        p2.pose.position.y = 0.0

        path.poses = [p1, p2]
        res.error_code = 0
        res.path = path
        goal_handle.succeed()
        return res

    def _execute_nav(self, goal_handle):
        self.nav_received_goals.append(goal_handle.request)
        goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_both(node1, node2, count=5, timeout=0.03):
    for _ in range(count):
        rclpy.spin_once(node1, timeout_sec=timeout)
        rclpy.spin_once(node2, timeout_sec=timeout)


def test_planner_chooses_shorter_path_when_first_candidate_is_longer(ros_context):
    """
    Test 1: toilet_1 (ứng viên đầu tiên) có đường đi dài (25.0m),
    toilet_2 (ứng viên thứ hai) có đường đi ngắn hơn (6.0m).
    Xác nhận planner nhận cả 2 yêu cầu và đích cuối cùng được chọn là toilet_2.
    """
    mock_server = MockPlannerAndNavServer('test_p1_planner', 'test_p1_nav')
    nav_node = NavCommandNode()
    nav_node.planner_action_name = 'test_p1_planner'
    nav_node.nav_action_name = 'test_p1_nav'
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p1_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p1_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    mock_server.path_lengths[toilet_1_x] = 25.0
    mock_server.path_lengths[toilet_2_x] = 6.0

    spin_both(nav_node, mock_server, 10)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_shorter'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state in (MissionState.ANNOUNCING, MissionState.STARTING):
            break

    assert toilet_1_x in mock_server.planner_received_goals
    assert toilet_2_x in mock_server.planner_received_goals
    assert nav_node._current_target_key == 'toilet_2'

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_chooses_second_candidate_when_first_candidate_is_unreachable(ros_context):
    """
    Test 2: toilet_1 không thể tới (NO_VALID_PATH),
    toilet_2 có đường đi hợp lệ (12.0m).
    Xác nhận đích chọn toilet_2.
    """
    mock_server = MockPlannerAndNavServer('test_p2_planner', 'test_p2_nav')
    nav_node = NavCommandNode()
    nav_node.planner_action_name = 'test_p2_planner'
    nav_node.nav_action_name = 'test_p2_nav'
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p2_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p2_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    mock_server.unreachable_x.add(toilet_1_x)
    mock_server.path_lengths[toilet_2_x] = 12.0

    spin_both(nav_node, mock_server, 10)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_unreachable'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state in (MissionState.ANNOUNCING, MissionState.STARTING):
            break

    assert toilet_1_x in mock_server.planner_received_goals
    assert toilet_2_x in mock_server.planner_received_goals
    assert nav_node._current_target_key == 'toilet_2'

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_all_candidates_unreachable(ros_context):
    """
    Test 3: Tất cả ứng viên đều không thể tới.
    Xác nhận: Hệ thống báo FAILED NO_VALID_PATH và đưa trạng thái về IDLE.
    """
    mock_server = MockPlannerAndNavServer('test_p3_planner', 'test_p3_nav')
    nav_node = NavCommandNode()
    nav_node.planner_action_name = 'test_p3_planner'
    nav_node.nav_action_name = 'test_p3_nav'
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p3_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p3_nav')

    feedbacks = []
    orig_fb = nav_node.publish_feedback
    def mock_fb(event, status, message=''):
        feedbacks.append({'event': event, 'status': status, 'message': message})
        return orig_fb(event, status, message)
    nav_node.publish_feedback = mock_fb

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    mock_server.unreachable_x.add(toilet_1_x)
    mock_server.unreachable_x.add(toilet_2_x)

    spin_both(nav_node, mock_server, 10)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_all_unreachable'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state == MissionState.IDLE and len(feedbacks) > 0:
            break

    assert nav_node.state == MissionState.IDLE
    assert any(fb.get('status') == 'NO_VALID_PATH' for fb in feedbacks)

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_cancel_during_planner_resolving(ros_context):
    """
    Test 4: Khi đang ở trạng thái RESOLVING, người dùng phát lệnh hủy (cancel).
    Xác nhận tiến trình chọn đường bị hủy ngay lập tức và trạng thái về IDLE.
    """
    mock_server = MockPlannerAndNavServer('test_p4_planner', 'test_p4_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p4_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p4_nav')

    spin_both(nav_node, mock_server, 5)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_resolve_cancel'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    spin_both(nav_node, mock_server, 2)
    assert nav_node.state in (MissionState.RESOLVING, MissionState.IDLE, MissionState.ANNOUNCING)

    nav_node.handle_cancel_request('req_cancel_now')
    spin_both(nav_node, mock_server, 5)

    assert nav_node.state == MissionState.IDLE
    assert nav_node._resolving_active is False

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_candidate_timeout_and_late_result_discarded(ros_context):
    """
    Test 5: Ứng viên 1 (toilet_1) bị timeout (trì hoãn phản hồi).
    Ứng viên 2 (toilet_2) trả về kết quả hợp lệ ngay.
    Xác nhận: Hệ thống bỏ qua ứng viên 1, chọn toilet_2, và kết quả muộn của ứng viên 1 bị loại bỏ.
    """
    mock_server = MockPlannerAndNavServer('test_p5_planner', 'test_p5_nav')
    nav_node = NavCommandNode()
    nav_node.planner_timeout = 0.2  # Timeout ngắn để test nhanh
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p5_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p5_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    # Điểm 1 bị trễ 0.4s (vượt quá 0.2s timeout)
    mock_server.delayed_x[toilet_1_x] = 0.4
    mock_server.path_lengths[toilet_2_x] = 8.0

    spin_both(nav_node, mock_server, 5)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_timeout_cand'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    # Quay executor đủ thời gian để timeout điểm 1 và xử lý điểm 2
    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state in (MissionState.ANNOUNCING, MissionState.STARTING):
            break

    assert nav_node._current_target_key == 'toilet_2'

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_cancel_and_new_resolution_late_callback(ros_context):
    """
    Test 6: Cancel phiên chọn đường cũ, bắt đầu phiên chọn đường mới.
    Callback cũ về muộn không được ghi vào phiên mới và không làm sai lệch current_idx.
    """
    mock_server = MockPlannerAndNavServer('test_p6_planner', 'test_p6_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p6_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p6_nav')

    spin_both(nav_node, mock_server, 5)

    # Phiên 1
    nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'category': 'toilet', 'request_id': 'req_ses_1'})))
    spin_both(nav_node, mock_server, 2)
    old_res_id = nav_node._current_resolution_id

    # Hủy phiên 1
    nav_node.handle_cancel_request('req_cancel_ses1')
    spin_both(nav_node, mock_server, 3)
    assert nav_node.state == MissionState.IDLE

    # Phiên 2
    nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'category': 'toilet', 'request_id': 'req_ses_2'})))
    spin_both(nav_node, mock_server, 2)
    new_res_id = nav_node._current_resolution_id
    assert new_res_id != old_res_id

    # Giả lập callback muộn từ phiên 1
    nav_node._advance_planner_evaluation(old_res_id)
    # current_idx của phiên 2 không được tăng do callback của phiên 1
    assert nav_node._planner_eval['current_idx'] == 0

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_replace_from_specific_target_to_category(ros_context):
    """
    Test 7: Replace từ đích cụ thể (cổng A3) sang nhóm điểm (category: toilet).
    Xác nhận: Hệ thống chạy cùng logic chọn đường của planner để tìm toilet gần nhất.
    """
    mock_server = MockPlannerAndNavServer('test_p7_planner', 'test_p7_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p7_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p7_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 20.0
    mock_server.path_lengths[toilet_2_x] = 5.0

    spin_both(nav_node, mock_server, 5)

    # 1. Bắt đầu đích cụ thể gate_a3
    nav_node.on_intent(String(data=json.dumps({'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_p7_1'})))
    spin_both(nav_node, mock_server, 5)

    # 2. Đổi đích sang category toilet
    nav_node.handle_replace_target({'category': 'toilet', 'request_id': 'req_p7_rep'})

    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node._current_target_key == 'toilet_2':
            break

    # Phải chọn toilet_2 vì đường 5m ngắn hơn 20m
    assert nav_node._current_target_key == 'toilet_2'

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_server_unavailable_reports_unavailability(ros_context):
    """
    Test 8: Planner server vắng mặt / chưa sẵn sàng.
    Xác nhận: Hệ thống phát thông báo giải thích rõ chưa thể chọn điểm theo đường đi,
    không tự tuyên bố đã chọn điểm gần nhất, không tự ý gửi goal Nav2, về IDLE và phát feedback PLANNER_UNAVAILABLE.
    """
    # Dùng action name không có server nào lắng nghe
    nav_node = NavCommandNode()
    nav_node.planner_action_name = 'nonexistent_planner_action'
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'nonexistent_planner_action')

    spoken = []
    feedbacks = []
    sent_goals = []

    orig_say = nav_node.say
    def mock_say(text, speech_id=None, mission_id=None):
        spoken.append(text)
        return orig_say(text, speech_id, mission_id)
    nav_node.say = mock_say

    orig_fb = nav_node.publish_feedback
    def mock_fb(event, status, message=''):
        feedbacks.append({'event': event, 'status': status, 'message': message})
        return orig_fb(event, status, message)
    nav_node.publish_feedback = mock_fb

    orig_send = nav_node.send_nav_goal
    def mock_send():
        sent_goals.append(True)
        return orig_send()
    nav_node.send_nav_goal = mock_send

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_unavail'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(10):
        rclpy.spin_once(nav_node, timeout_sec=0.03)

    # 1. Thông báo rõ chưa sẵn sàng
    assert any('chưa sẵn sàng' in s for s in spoken)
    assert not any('tối ưu' in s or 'gần nhất' in s for s in spoken if 'chưa thể' not in s)

    # 2. Báo lỗi PLANNER_UNAVAILABLE
    assert any(fb.get('status') == 'PLANNER_UNAVAILABLE' for fb in feedbacks)

    # 3. Trạng thái về IDLE
    assert nav_node.state == MissionState.IDLE

    # 4. TUYỆT ĐỐI không gửi bất kỳ navigation goal nào
    assert len(sent_goals) == 0
    assert nav_node._current_goal_attempt_id is None

    nav_node.destroy_node()
