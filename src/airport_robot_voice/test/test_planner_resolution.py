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
from visualization_msgs.msg import MarkerArray
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


def test_rviz_confirmed_plan_and_markers_match_winning_toilet_before_tts(ros_context):
    """
    Test 9 (Regression): Khi toilet_1 (ứng viên 1) ngắn hơn toilet_2 (ứng viên 2),
    planner_server có thể đã publish /plan sang toilet_2 ở lượt đánh giá cuối.
    Hệ thống PHẢI khóa toilet_1 trước khi phát TTS, đồng bộ lại /plan, /voice/confirmed_path,
    /voice/confirmed_goal, /waypoints và /voice/planner_debug_markers đúng về toilet_1
    ngay trong lúc ANNOUNCING, và gửi đúng pose toilet_1 tới NavigateToPose sau TTS ACK
    mà KHÔNG chọn lại lần thứ hai.
    """
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p9_planner', 'test_p9_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p9_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p9_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 4.2
    mock_server.path_lengths[toilet_2_x] = 9.8

    # Giả lập planner_server publish mỗi đường thử ra /plan như Nav2 thật
    orig_exec = mock_server._execute_planner
    plan_pub = mock_server.create_publisher(Path, '/plan', 10)
    def exec_with_plan_pub(gh):
        res = orig_exec(gh)
        if res.error_code == 0 and res.path.poses:
            # Gán tọa độ cuối khớp candidate để dễ kiểm chứng điểm đích trên /plan
            res.path.poses[-1].pose.position.x = float(gh.request.goal.pose.position.x)
            res.path.poses[-1].pose.position.y = float(gh.request.goal.pose.position.y)
            plan_pub.publish(res.path)
        return res
    mock_server._planner_server.register_execute_callback(exec_with_plan_pub)

    # Observer đọc các topic hiển thị và /voice/say
    obs = rclpy.create_node('test_p9_rviz_observer')
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    seen_plan = []
    seen_confirmed_goal = []
    seen_confirmed_path = []
    seen_waypoints = []
    seen_debug_markers = []
    seen_says = []

    obs.create_subscription(Path, '/plan', lambda m: seen_plan.append(m), 10)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: seen_confirmed_goal.append(m), qos_tl)
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: seen_confirmed_path.append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/waypoints', lambda m: seen_waypoints.append(m), 10)
    obs.create_subscription(MarkerArray, '/voice/planner_debug_markers', lambda m: seen_debug_markers.append(m), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: seen_says.append(json.loads(m.data)), 10)

    def spin_all(n=5, timeout=0.03):
        for _ in range(n):
            rclpy.spin_once(nav_node, timeout_sec=timeout)
            rclpy.spin_once(mock_server, timeout_sec=timeout)
            rclpy.spin_once(obs, timeout_sec=timeout)

    spin_all(8)

    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p9_toilet'
    })))

    for _ in range(40):
        spin_all(1, timeout=0.03)
        if nav_node.state == MissionState.ANNOUNCING:
            break

    # Quay thêm vài vòng để obs nhận đủ các bản tin /plan và Marker mới nhất qua DDS
    spin_all(8, timeout=0.03)

    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'toilet_1'
    assert len(mock_server.nav_received_goals) == 0  # Chưa gửi goal trước khi TTS xong

    # Trong lúc đang phát TTS (ANNOUNCING), RViz đã khóa đúng về toilet_1
    assert len(seen_plan) >= 1
    assert round(seen_plan[-1].poses[-1].pose.position.x, 2) == toilet_1_x
    assert len(seen_confirmed_goal) >= 1
    assert round(seen_confirmed_goal[-1].pose.position.x, 2) == toilet_1_x
    assert len(seen_confirmed_path) >= 1
    assert round(seen_confirmed_path[-1].poses[-1].pose.position.x, 2) == toilet_1_x
    assert len(seen_waypoints) >= 1
    assert round(seen_waypoints[-1].markers[0].pose.position.x, 2) == toilet_1_x
    assert len(seen_debug_markers) >= 1
    labels = [m.text for m in seen_debug_markers[-1].markers if m.text]
    assert any('[CHỌN] toilet_1' in t for t in labels)
    assert any('[THỬ] toilet_2' in t for t in labels)

    # Kết thúc TTS -> gửi đúng pose toilet_1 tới NavigateToPose, không chạy planner lần 2
    planner_calls_before_ack = len(mock_server.planner_received_goals)
    sid = seen_says[-1]['speech_id']
    mid = seen_says[-1]['mission_id']
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED',
        'engine': 'piper', 'audio_played': True
    })))

    for _ in range(30):
        spin_all(1, timeout=0.03)
        if len(mock_server.nav_received_goals) >= 1 and nav_node.state == MissionState.IDLE:
            break

    assert len(mock_server.planner_received_goals) == planner_calls_before_ack
    assert len(mock_server.nav_received_goals) == 1
    dispatched_goal = mock_server.nav_received_goals[0]
    assert round(dispatched_goal.pose.pose.position.x, 2) == toilet_1_x
    assert round(dispatched_goal.pose.pose.position.y, 2) == round(nav_node.places['toilet_1']['y'], 2)

    obs.destroy_node()
    nav_node.destroy_node()
    mock_server.destroy_node()


def test_chosen_toilet_becomes_unavailable_before_dispatch(ros_context):
    """
    Test 10 (Regression): Nếu điểm toilet đã chốt trở nên không khả dụng trước khi gửi goal
    (sau khi TTS hoàn thành), hệ thống phải báo lỗi rõ ràng, không gửi goal Nav2 và về IDLE.
    """
    mock_server = MockPlannerAndNavServer('test_p10_planner', 'test_p10_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p10_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p10_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 3.0
    mock_server.path_lengths[toilet_2_x] = 10.0

    feedbacks = []
    orig_fb = nav_node.publish_feedback
    def rec_fb(ev, st, msg=''):
        feedbacks.append({'event': ev, 'status': st, 'message': msg})
        return orig_fb(ev, st, msg)
    nav_node.publish_feedback = rec_fb

    speeches = []
    orig_say = nav_node.say
    def rec_say(text, speech_id=None, mission_id=None):
        sid = orig_say(text, speech_id, mission_id)
        speeches.append((text, sid, mission_id))
        return sid
    nav_node.say = rec_say

    spin_both(nav_node, mock_server, 8)
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p10_unavail'
    })))

    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state == MissionState.ANNOUNCING:
            break

    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'toilet_1'

    # Vô hiệu hóa toilet_1 trong lúc robot đang nói
    nav_node.places['toilet_1']['enabled'] = False
    _, sid, mid = speeches[-1]
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED',
        'engine': 'piper', 'audio_played': True
    })))

    spin_both(nav_node, mock_server, 10)
    assert nav_node.state == MissionState.IDLE
    assert len(mock_server.nav_received_goals) == 0
    assert any(fb.get('status') == 'TARGET_BECAME_UNAVAILABLE' for fb in feedbacks)
    assert any('không còn khả dụng' in t for t, _, _ in speeches)

    nav_node.destroy_node()
    mock_server.destroy_node()


def test_pause_resume_and_cancel_during_toilet_resolving_and_announcing(ros_context):
    """
    Test 11 (Regression): Tạm dừng & đi tiếp khi đang RESOLVING, và hủy chuyến khi đang ANNOUNCING
    cho yêu cầu nhà vệ sinh; kiểm tra handle_status_request không báo 'None'.
    """
    mock_server = MockPlannerAndNavServer('test_p11_planner', 'test_p11_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p11_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p11_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.delayed_x[toilet_1_x] = 0.25
    mock_server.path_lengths[toilet_1_x] = 3.5
    mock_server.path_lengths[toilet_2_x] = 9.0

    speeches = []
    orig_say = nav_node.say
    def rec_say(text, speech_id=None, mission_id=None):
        speeches.append((text, speech_id, mission_id))
        return orig_say(text, speech_id, mission_id)
    nav_node.say = rec_say

    spin_both(nav_node, mock_server, 8)
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p11_1'
    })))
    spin_both(nav_node, mock_server, 2)
    assert nav_node.state == MissionState.RESOLVING

    # 1. Tạm dừng trong lúc đang tìm đường RESOLVING
    nav_node.handle_pause_request('req_p11_pause')
    assert nav_node.state == MissionState.PAUSED
    assert nav_node._paused_resolution is not None
    nav_node.handle_status_request('req_p11_status')
    assert all('None' not in t for t, _, _ in speeches)

    # 2. Đi tiếp -> tìm lại đường và chốt toilet_1 -> ANNOUNCING
    mock_server.delayed_x.clear()
    nav_node.handle_resume_request('req_p11_resume')
    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'toilet_1'

    # 3. Hủy chuyến khi đang ANNOUNCING -> về IDLE sạch, không gửi NavigateToPose
    nav_node.handle_cancel_request('req_p11_cancel')
    spin_both(nav_node, mock_server, 8)
    assert nav_node.state == MissionState.IDLE
    assert len(mock_server.nav_received_goals) == 0

    nav_node.destroy_node()
    mock_server.destroy_node()
