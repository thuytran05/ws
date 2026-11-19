#!/usr/bin/env python3
"""
test_planner_resolution.py — Kiểm thử chọn đích bằng planner (ComputePathToPose).
Bao phủ toàn diện GIAI ĐOẠN 1:
  1. Điểm đầu (toilet_1) có đường dài hơn điểm thứ hai (toilet_2) -> Đích chọn toilet_2.
  2. Điểm đầu không thể tới (NO_VALID_PATH) -> Đích chọn toilet_2.
  3. Tất cả các điểm không thể tới -> Báo lỗi NO_VALID_PATH và về IDLE.
  4. Lệnh pause / cancel trong khi đang chờ planner -> Hủy resolve và chuyển trạng thái đúng.
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
        super().__init__('mock_planner_and_nav_server')
        self.planner_action_name = planner_action_name
        self.nav_action_name = nav_action_name

        self.path_lengths = {}      # target_x -> length
        self.unreachable_x = set()  # set of x coordinates that return error
        self.planner_received_goals = []
        self.nav_received_goals = []

        self._planner_server = ActionServer(
            self,
            ComputePathToPose,
            self.planner_action_name,
            execute_callback=self._execute_planner,
            goal_callback=lambda req: GoalResponse.ACCEPT
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

        res = ComputePathToPose.Result()
        path = Path()
        path.header.frame_id = 'map'

        if target_x in self.unreachable_x:
            # Mô phỏng không tìm thấy đường đi (NO_VALID_PATH = 208)
            res.error_code = 208
            res.path = path
            goal_handle.succeed()
            return res

        # Tạo đường đi với chiều dài tương ứng
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
    # Re-initialize clients with mock action names
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p1_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p1_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    # Điểm 1 dài 25m, Điểm 2 chỉ 6m
    mock_server.path_lengths[toilet_1_x] = 25.0
    mock_server.path_lengths[toilet_2_x] = 6.0

    spin_both(nav_node, mock_server, 10)

    # Gửi lệnh tìm nhà vệ sinh (category: toilet)
    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_shorter'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    # Quay executor để xử lý resolving -> announcing
    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state in (MissionState.ANNOUNCING, MissionState.STARTING):
            break

    # Xác nhận server nhận được cả 2 tọa độ
    assert len(mock_server.planner_received_goals) == 2
    assert toilet_1_x in mock_server.planner_received_goals
    assert toilet_2_x in mock_server.planner_received_goals

    # Xác nhận ứng viên được chọn là toilet_2 (vì 6m < 25m)
    assert nav_node._current_target_key == 'toilet_2'
    assert 'toilet_2' in nav_node._current_target_name or 'nhà vệ sinh sảnh b' in nav_node._current_target_name.lower()

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_chooses_second_candidate_when_first_candidate_is_unreachable(ros_context):
    """
    Test 2: toilet_1 không thể tới (NO_VALID_PATH),
    toilet_2 có đường hợp lệ (8.0m).
    Xác nhận toilet_1 bị loại và toilet_2 được chọn.
    """
    mock_server = MockPlannerAndNavServer('test_p2_planner', 'test_p2_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p2_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p2_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    mock_server.unreachable_x.add(toilet_1_x)
    mock_server.path_lengths[toilet_2_x] = 8.0

    spin_both(nav_node, mock_server, 10)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_unreachable_first'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state in (MissionState.ANNOUNCING, MissionState.STARTING):
            break

    assert len(mock_server.planner_received_goals) == 2
    assert nav_node._current_target_key == 'toilet_2'

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_planner_all_candidates_unreachable(ros_context):
    """
    Test 3: Tất cả ứng viên trong nhóm đều không thể tới.
    Xác nhận hệ thống báo lỗi FAILED: NO_VALID_PATH và quay về IDLE.
    """
    mock_server = MockPlannerAndNavServer('test_p3_planner', 'test_p3_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p3_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p3_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)

    mock_server.unreachable_x.add(toilet_1_x)
    mock_server.unreachable_x.add(toilet_2_x)

    feedbacks = []
    nav_node.create_subscription(String, '/voice/feedback', lambda m: feedbacks.append(json.loads(m.data)), 10)

    spin_both(nav_node, mock_server, 10)

    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_all_unreachable'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    for _ in range(30):
        spin_both(nav_node, mock_server, 1, timeout=0.05)
        if nav_node.state == MissionState.IDLE and feedbacks:
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

    # Bắt đầu resolve
    nav_payload = {'action': 'navigate', 'category': 'toilet', 'request_id': 'req_test_resolve_cancel'}
    nav_node.on_intent(String(data=json.dumps(nav_payload)))

    # Quay 1 nhịp để vào RESOLVING
    spin_both(nav_node, mock_server, 2)
    assert nav_node.state in (MissionState.RESOLVING, MissionState.IDLE, MissionState.ANNOUNCING)

    # Gửi lệnh cancel
    nav_node.handle_cancel_request('req_cancel_now')
    spin_both(nav_node, mock_server, 5)

    assert nav_node.state == MissionState.IDLE
    assert nav_node._resolving_active is False

    nav_node.destroy_node()
    mock_server.destroy_node()
