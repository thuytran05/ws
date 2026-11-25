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
from visualization_msgs.msg import Marker, MarkerArray
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


def spin_nodes(*nodes, count=5, timeout=0.03):
    for _ in range(count):
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=timeout)


def ack_tts(nav_node, speech_id=None, mission_id=None):
    sid = speech_id if speech_id is not None else nav_node._pending_speech_id
    mid = mission_id if mission_id is not None else nav_node._current_mission_id
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED',
        'engine': 'piper', 'audio_played': True
    })))


def create_rviz_observer(name: str):
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    obs = rclpy.create_node(name)
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    data = {'cp': [], 'cm': [], 'cg': [], 'ns': [], 'dm': [], 'says': []}
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: data['cp'].append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/voice/confirmed_goal_marker', lambda m: data['cm'].append(m), qos_tl)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: data['cg'].append(m), qos_tl)
    obs.create_subscription(String, '/voice/nav_state', lambda m: data['ns'].append(json.loads(m.data)), qos_tl)
    obs.create_subscription(MarkerArray, '/voice/planner_debug_markers', lambda m: data['dm'].append(m), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: data['says'].append(json.loads(m.data)), 10)
    return obs, data


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
    Test 9 (Regression): Tách quyền sở hữu topic hiển thị RViz & không làm biến đổi pose gốc.
    - Nav2 sở hữu /plan (nav_command_node KHÔNG publish hay xóa /plan hay /waypoints).
    - /voice/confirmed_goal_marker, /voice/confirmed_path, /voice/confirmed_goal, và
      /voice/nav_state (confirmed_goal_valid=True, z=0.0) khóa đúng toilet_1 trước khi phát TTS.
    - Marker CYLINDER z==0.15, ARROW z==0.18, KHÔNG làm lệch self._current_target_pose.pose.position.z == 0.0.
    - NavigateToPose nhận đúng x, y, z == 0.0 và quaternion orientation của toilet_1 sau TTS ACK.
    """
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p9_planner', 'test_p9_nav')
    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p9_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p9_nav')

    toilet_1 = nav_node.places['toilet_1']
    toilet_2 = nav_node.places['toilet_2']
    toilet_1_x = round(toilet_1['x'], 2)
    toilet_2_x = round(toilet_2['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 4.2
    mock_server.path_lengths[toilet_2_x] = 9.8

    # Giả lập planner_server publish đường thử ra /plan như Nav2 thật
    orig_exec = mock_server._execute_planner
    plan_pub = mock_server.create_publisher(Path, '/plan', 10)
    def exec_with_plan_pub(gh):
        res = orig_exec(gh)
        if res.error_code == 0 and res.path.poses:
            res.path.poses[-1].pose.position.x = float(gh.request.goal.pose.position.x)
            res.path.poses[-1].pose.position.y = float(gh.request.goal.pose.position.y)
            plan_pub.publish(res.path)
        return res
    mock_server._planner_server.register_execute_callback(exec_with_plan_pub)

    # Observer đọc các topic hiển thị, /voice/nav_state và /voice/say
    obs = rclpy.create_node('test_p9_rviz_observer')
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    seen_plan = []
    seen_confirmed_goal = []
    seen_confirmed_path = []
    seen_confirmed_markers = []
    seen_waypoints = []
    seen_debug_markers = []
    seen_nav_states = []
    seen_says = []

    obs.create_subscription(Path, '/plan', lambda m: seen_plan.append(m), 10)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: seen_confirmed_goal.append(m), qos_tl)
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: seen_confirmed_path.append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/voice/confirmed_goal_marker', lambda m: seen_confirmed_markers.append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/waypoints', lambda m: seen_waypoints.append(m), 10)
    obs.create_subscription(MarkerArray, '/voice/planner_debug_markers', lambda m: seen_debug_markers.append(m), qos_tl)
    obs.create_subscription(String, '/voice/nav_state', lambda m: seen_nav_states.append(json.loads(m.data)), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: seen_says.append(json.loads(m.data)), 10)

    def spin_all(n=5, timeout=0.03):
        for _ in range(n):
            rclpy.spin_once(nav_node, timeout_sec=timeout)
            rclpy.spin_once(mock_server, timeout_sec=timeout)
            rclpy.spin_once(obs, timeout_sec=timeout)

    spin_all(8)

    # nav_command_node không tạo publisher trên /plan hay /waypoints
    assert nav_node.count_publishers('/plan') == 1  # Chỉ có plan_pub của mock_server
    assert nav_node.count_publishers('/waypoints') == 0

    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p9_toilet'
    })))

    for _ in range(40):
        spin_all(1, timeout=0.03)
        if nav_node.state == MissionState.ANNOUNCING:
            break

    spin_all(8, timeout=0.03)

    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'toilet_1'
    assert len(mock_server.nav_received_goals) == 0  # Chưa gửi goal trước khi TTS xong

    # /plan vẫn là của mock planner (toilet_2 ở lượt thử cuối), nav_command_node KHÔNG ghi đè /plan hay /waypoints
    assert len(seen_plan) >= 1
    assert round(seen_plan[-1].poses[-1].pose.position.x, 2) == toilet_2_x
    assert len(seen_waypoints) == 0

    # Các topic sở hữu bởi voice khóa chặt về toilet_1 trước TTS
    assert len(seen_confirmed_goal) >= 1
    cg = seen_confirmed_goal[-1]
    assert cg.header.frame_id == 'map'
    assert round(cg.pose.position.x, 2) == toilet_1_x
    assert round(cg.pose.position.y, 2) == round(toilet_1['y'], 2)
    assert cg.pose.position.z == pytest.approx(0.0, abs=1e-6)

    assert len(seen_confirmed_path) >= 1
    assert round(seen_confirmed_path[-1].poses[-1].pose.position.x, 2) == toilet_1_x

    # Marker CYLINDER z=0.15, ARROW z=0.18, nhưng pose gốc trong nav_node & confirmed_goal vẫn giữ z=0.0
    add_markers = [m for ma in seen_confirmed_markers for m in ma.markers if m.action == Marker.ADD]
    assert len(add_markers) >= 3
    pin = [m for m in seen_confirmed_markers[-1].markers if m.type == Marker.CYLINDER][0]
    arrow = [m for m in seen_confirmed_markers[-1].markers if m.type == Marker.ARROW][0]
    assert pin.pose.position.z == pytest.approx(0.15, abs=1e-6)
    assert arrow.pose.position.z == pytest.approx(0.18, abs=1e-6)
    assert nav_node._current_target_pose.pose.position.z == pytest.approx(0.0, abs=1e-6)

    # /voice/nav_state xác nhận confirmed_goal_valid=True với z=0.0
    latest_state = seen_nav_states[-1]
    assert latest_state['confirmed_goal_valid'] is True
    assert latest_state['current_target_id'] == 'toilet_1'
    assert latest_state['confirmed_goal_pose']['z'] == pytest.approx(0.0, abs=1e-6)

    assert len(seen_debug_markers) >= 1
    labels = [m.text for m in seen_debug_markers[-1].markers if m.text]
    assert any('[CHỌN] toilet_1' in t for t in labels)
    assert any('[THỬ] toilet_2' in t for t in labels)

    # Kết thúc TTS -> gửi đúng pose toilet_1 (z == 0.0 + quaternion gốc) tới NavigateToPose
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
    yaw = float(toilet_1.get('yaw', 0.0))
    expected_qz = math.sin(yaw / 2.0)
    expected_qw = math.cos(yaw / 2.0)
    assert dispatched_goal.pose.pose.position.x == pytest.approx(float(toilet_1['x']), abs=1e-5)
    assert dispatched_goal.pose.pose.position.y == pytest.approx(float(toilet_1['y']), abs=1e-5)
    assert dispatched_goal.pose.pose.position.z == pytest.approx(0.0, abs=1e-6)
    assert dispatched_goal.pose.pose.orientation.x == pytest.approx(0.0, abs=1e-6)
    assert dispatched_goal.pose.pose.orientation.y == pytest.approx(0.0, abs=1e-6)
    assert dispatched_goal.pose.pose.orientation.z == pytest.approx(expected_qz, abs=1e-6)
    assert dispatched_goal.pose.pose.orientation.w == pytest.approx(expected_qw, abs=1e-6)

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


def test_late_joining_subscriber_sees_cleared_visuals_and_invalid_state_after_terminal_idle(ros_context):
    """
    Test 12 (Regression): Sau khi hủy chuyến hoặc kết thúc hành trình (IDLE),
    mọi marker/path phải được xóa, /voice/confirmed_goal bị vô hiệu hóa (frame_id rỗng),
    /voice/nav_state báo confirmed_goal_valid=False, và một subscriber kết nối MUỘN
    sau khi về IDLE không bao giờ nhận nhầm mục tiêu cũ là đang hiệu lực.
    """
    import threading
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p12_planner', 'test_p12_nav')
    hold_nav = threading.Event()

    def hold_exec_nav(gh):
        mock_server.nav_received_goals.append(gh.request)
        # Chờ cancel hoặc release tối đa 3s
        for _ in range(60):
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            if hold_nav.is_set():
                break
            time.sleep(0.05)
        gh.succeed()
        return NavigateToPose.Result()

    mock_server._nav_server.register_execute_callback(hold_exec_nav)

    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p12_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p12_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 3.5
    mock_server.path_lengths[toilet_2_x] = 9.0

    # Gán /voice/confirmed_goal_path lúc đang chạy để chắc chắn lúc đầu có dữ liệu
    speeches = []
    orig_say = nav_node.say
    def rec_say(text, speech_id=None, mission_id=None):
        sid = orig_say(text, speech_id, mission_id)
        speeches.append((text, sid, mission_id))
        return sid
    nav_node.say = rec_say

    spin_both(nav_node, mock_server, 8)
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p12_1'
    })))

    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    assert nav_node.state == MissionState.ANNOUNCING

    _, sid, mid = speeches[-1]
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED',
        'engine': 'piper', 'audio_played': True
    })))
    for _ in range(40):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state == MissionState.NAVIGATING:
            break
    assert nav_node.state == MissionState.NAVIGATING

    # Hủy khi đang NAVIGATING -> về IDLE
    nav_node.handle_cancel_request('req_p12_cancel')
    for _ in range(50):
        spin_both(nav_node, mock_server, 1, timeout=0.03)
        if nav_node.state == MissionState.IDLE:
            break
    assert nav_node.state == MissionState.IDLE
    spin_both(nav_node, mock_server, 5, timeout=0.03)

    # Tạo subscriber kết nối MUỘN sau khi hệ thống đã về IDLE
    late_obs, late_data = create_rviz_observer('test_p12_late_subscriber')
    for _ in range(25):
        spin_nodes(nav_node, late_obs, count=1)
        if all(late_data[k] for k in ('ns', 'cg', 'cm', 'cp', 'dm')):
            break

    assert late_data['ns'][-1]['mission_state'] == MissionState.IDLE
    assert late_data['ns'][-1]['confirmed_goal_valid'] is False
    assert late_data['ns'][-1]['confirmed_goal_pose'] is None
    assert late_data['cg'][-1].header.frame_id == ''
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in late_data['cm'][-1].markers)
    assert len(late_data['cp'][-1].poses) == 0
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in late_data['dm'][-1].markers)

    late_obs.destroy_node()
    nav_node.destroy_node()
    mock_server.destroy_node()


def test_preview_path_cleared_when_navigating_starts_and_uniform_for_direct_target(ros_context):
    """
    Test 13 (Regression):
    - Với nhà vệ sinh: /voice/confirmed_path được xóa rỗng ngay khi NavigateToPose được chấp nhận (NAVIGATING)
      để không tồn tại đường preview xanh lá song song với /plan của Nav2.
    - Với điểm đến trực tiếp (gate_a3): /voice/confirmed_path luôn rỗng, marker giữ đúng gate_a3,
      và reset sạch sẽ khi SUCCEEDED.
    """
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p13_planner', 'test_p13_nav')
    release_nav = threading.Event()
    srv_exec = MultiThreadedExecutor(num_threads=4)
    srv_exec.add_node(mock_server)
    srv_stop = threading.Event()
    srv_thread = threading.Thread(target=lambda: [srv_exec.spin_once(timeout_sec=0.05) for _ in iter(lambda: not srv_stop.is_set(), False)], daemon=True)
    srv_thread.start()

    def hold_exec_nav(gh):
        mock_server.nav_received_goals.append(gh.request)
        for _ in range(100):
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            if release_nav.is_set():
                break
            time.sleep(0.02)
        gh.succeed()
        return NavigateToPose.Result()

    mock_server._nav_server.register_execute_callback(hold_exec_nav)

    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p13_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p13_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 3.2
    mock_server.path_lengths[toilet_2_x] = 8.4

    obs = rclpy.create_node('test_p13_obs')
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    seen_cp, seen_cm, seen_cg, seen_ns, seen_says = [], [], [], [], []
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: seen_cp.append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/voice/confirmed_goal_marker', lambda m: seen_cm.append(m), qos_tl)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: seen_cg.append(m), qos_tl)
    obs.create_subscription(String, '/voice/nav_state', lambda m: seen_ns.append(json.loads(m.data)), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: seen_says.append(json.loads(m.data)), 10)

    def spin_all(n=5, timeout=0.03):
        for _ in range(n):
            rclpy.spin_once(nav_node, timeout_sec=timeout)
            rclpy.spin_once(obs, timeout_sec=timeout)

    spin_all(8)

    # 1) Yêu cầu nhà vệ sinh
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p13_toilet'
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    spin_all(5)
    assert nav_node.state == MissionState.ANNOUNCING

    sid = nav_node._pending_speech_id
    mid = nav_node._current_mission_id
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED', 'engine': 'piper', 'audio_played': True
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.NAVIGATING:
            break
    spin_all(5)
    assert nav_node.state == MissionState.NAVIGATING

    # Khi đang NAVIGATING: /voice/confirmed_path đã được xóa rỗng, chỉ còn marker đích toilet_1
    assert len(seen_cp[-1].poses) == 0
    assert any(m.action == Marker.ADD for m in seen_cm[-1].markers)
    assert round(seen_cg[-1].pose.position.x, 2) == toilet_1_x

    # Kết thúc SUCCEEDED -> reset toàn bộ
    release_nav.set()
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.IDLE:
            break
    spin_all(5)
    assert nav_node.state == MissionState.IDLE
    assert seen_ns[-1]['confirmed_goal_valid'] is False
    assert seen_cg[-1].header.frame_id == ''
    assert len(seen_cp[-1].poses) == 0
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in seen_cm[-1].markers)

    # 2) Yêu cầu điểm đến trực tiếp gate_a3
    release_nav.clear()
    seen_cp.clear()
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'target_id': 'gate_a3', 'request_id': 'req_p13_a3'
    })))
    for _ in range(30):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    spin_all(5)
    assert nav_node.state == MissionState.ANNOUNCING
    assert all(len(p.poses) == 0 for p in seen_cp)
    assert round(seen_cg[-1].pose.position.x, 2) == round(nav_node.places['gate_a3']['x'], 2)

    sid = nav_node._pending_speech_id
    mid = nav_node._current_mission_id
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED', 'engine': 'piper', 'audio_played': True
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.NAVIGATING:
            break
    spin_all(5)
    assert nav_node.state == MissionState.NAVIGATING
    assert len(seen_cp[-1].poses) == 0

    release_nav.set()
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.IDLE:
            break
    spin_all(5)
    assert nav_node.state == MissionState.IDLE
    assert seen_ns[-1]['confirmed_goal_valid'] is False
    assert seen_cg[-1].header.frame_id == ''

    srv_stop.set()
    srv_thread.join(timeout=2.0)
    srv_exec.shutdown()
    obs.destroy_node()
    nav_node.destroy_node()
    mock_server.destroy_node()


def test_toilet_to_gate_a3_and_gate_a3_to_toilet_transitions(ros_context):
    """
    Test 14 (Regression): Chuyển đổi trực tiếp trong khi đang di chuyển:
    - Nhà vệ sinh -> cổng A3
    - Cổng A3 -> nhà vệ sinh
    Xác nhận mỗi thời điểm chỉ có một NavigateToPose goal đang hoạt động, preview path được dọn sạch.
    """
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p14_planner', 'test_p14_nav')
    active_goals = {'count': 0}
    srv_exec = MultiThreadedExecutor(num_threads=4)
    srv_exec.add_node(mock_server)
    srv_stop = threading.Event()
    srv_thread = threading.Thread(target=lambda: [srv_exec.spin_once(timeout_sec=0.05) for _ in iter(lambda: not srv_stop.is_set(), False)], daemon=True)
    srv_thread.start()

    def exec_nav_track(gh):
        mock_server.nav_received_goals.append(gh.request)
        active_goals['count'] += 1
        try:
            for _ in range(150):
                if gh.is_cancel_requested:
                    gh.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.02)
            gh.succeed()
            return NavigateToPose.Result()
        finally:
            active_goals['count'] -= 1

    mock_server._nav_server.register_execute_callback(exec_nav_track)

    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p14_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p14_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    gate_a3_x = round(nav_node.places['gate_a3']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 3.0
    mock_server.path_lengths[toilet_2_x] = 9.0

    obs = rclpy.create_node('test_p14_obs')
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    seen_cp, seen_cg, seen_says = [], [], []
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: seen_cp.append(m), qos_tl)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: seen_cg.append(m), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: seen_says.append(json.loads(m.data)), 10)

    def spin_all(n=5, timeout=0.03):
        for _ in range(n):
            rclpy.spin_once(nav_node, timeout_sec=timeout)
            rclpy.spin_once(obs, timeout_sec=timeout)

    def ack_latest_tts():
        spin_all(3)
        sid = nav_node._pending_speech_id
        mid = nav_node._current_mission_id
        nav_node.on_tts_ack(String(data=json.dumps({
            'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED', 'engine': 'piper', 'audio_played': True
        })))

    spin_all(8)

    # 1) Bắt đầu đi toilet_1
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p14_1'
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    ack_latest_tts()
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.NAVIGATING:
            break
    assert nav_node.state == MissionState.NAVIGATING
    assert active_goals['count'] == 1

    # 2) Đổi mục tiêu sang gate_a3 (Nhà vệ sinh -> A3)
    nav_node.on_intent(String(data=json.dumps({
        'action': 'replace', 'target_id': 'gate_a3', 'request_id': 'req_p14_2'
    })))
    for _ in range(50):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING and nav_node._current_target_key == 'gate_a3':
            break
    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'gate_a3'
    assert active_goals['count'] == 0
    ack_latest_tts()
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.NAVIGATING:
            break
    spin_all(4)
    assert nav_node.state == MissionState.NAVIGATING
    assert active_goals['count'] == 1
    assert len(seen_cp[-1].poses) == 0
    assert round(seen_cg[-1].pose.position.x, 2) == gate_a3_x

    # 3) Đổi mục tiêu sang nhà vệ sinh (A3 -> Nhà vệ sinh)
    nav_node.on_intent(String(data=json.dumps({
        'action': 'replace', 'category': 'toilet', 'request_id': 'req_p14_3'
    })))
    for _ in range(60):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING and nav_node._current_target_key == 'toilet_1':
            break
    assert nav_node.state == MissionState.ANNOUNCING
    assert nav_node._current_target_key == 'toilet_1'
    assert active_goals['count'] == 0
    ack_latest_tts()
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.NAVIGATING:
            break
    spin_all(4)
    assert nav_node.state == MissionState.NAVIGATING
    assert active_goals['count'] == 1
    assert len(seen_cp[-1].poses) == 0
    assert round(seen_cg[-1].pose.position.x, 2) == toilet_1_x

    nav_node.handle_cancel_request('req_p14_cancel')
    for _ in range(50):
        spin_all(1)
        if nav_node.state == MissionState.IDLE:
            break
    assert nav_node.state == MissionState.IDLE
    assert active_goals['count'] == 0

    srv_stop.set()
    srv_thread.join(timeout=2.0)
    srv_exec.shutdown()
    obs.destroy_node()
    nav_node.destroy_node()
    mock_server.destroy_node()


def test_aborted_terminal_and_no_valid_path_reset_visuals(ros_context):
    """
    Test 15 (Regression): Kiểm tra dọn sạch visualization khi:
    - Nhiệm vụ kết thúc với ABORTED.
    - Không tìm được đường đến tất cả nhà vệ sinh (NO_VALID_PATH).
    - Hủy trong khi đang RESOLVING.
    """
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    mock_server = MockPlannerAndNavServer('test_p15_planner', 'test_p15_nav')

    def abort_exec_nav(gh):
        mock_server.nav_received_goals.append(gh.request)
        gh.abort()
        return NavigateToPose.Result()

    mock_server._nav_server.register_execute_callback(abort_exec_nav)

    nav_node = NavCommandNode()
    nav_node.planner_client = rclpy.action.ActionClient(nav_node, ComputePathToPose, 'test_p15_planner')
    nav_node.nav_client = rclpy.action.ActionClient(nav_node, NavigateToPose, 'test_p15_nav')

    toilet_1_x = round(nav_node.places['toilet_1']['x'], 2)
    toilet_2_x = round(nav_node.places['toilet_2']['x'], 2)
    mock_server.path_lengths[toilet_1_x] = 4.0
    mock_server.path_lengths[toilet_2_x] = 9.0

    obs = rclpy.create_node('test_p15_obs')
    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    seen_cp, seen_cm, seen_cg, seen_ns, seen_says = [], [], [], [], []
    obs.create_subscription(Path, '/voice/confirmed_path', lambda m: seen_cp.append(m), qos_tl)
    obs.create_subscription(MarkerArray, '/voice/confirmed_goal_marker', lambda m: seen_cm.append(m), qos_tl)
    obs.create_subscription(PoseStamped, '/voice/confirmed_goal', lambda m: seen_cg.append(m), qos_tl)
    obs.create_subscription(String, '/voice/nav_state', lambda m: seen_ns.append(json.loads(m.data)), qos_tl)
    obs.create_subscription(String, '/voice/say', lambda m: seen_says.append(json.loads(m.data)), 10)

    def spin_all(n=5, timeout=0.03):
        for _ in range(n):
            rclpy.spin_once(nav_node, timeout_sec=timeout)
            rclpy.spin_once(mock_server, timeout_sec=timeout)
            rclpy.spin_once(obs, timeout_sec=timeout)

    spin_all(8)

    # 1) Nhiệm vụ kết thúc ABORTED
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p15_1'
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.ANNOUNCING:
            break
    spin_all(5)
    sid = nav_node._pending_speech_id
    mid = nav_node._current_mission_id
    nav_node.on_tts_ack(String(data=json.dumps({
        'speech_id': sid, 'mission_id': mid, 'status': 'COMPLETED', 'engine': 'piper', 'audio_played': True
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.IDLE and len(mock_server.nav_received_goals) >= 1:
            break
    spin_all(5)
    assert nav_node.state == MissionState.IDLE
    assert seen_ns[-1]['confirmed_goal_valid'] is False
    assert seen_cg[-1].header.frame_id == ''
    assert len(seen_cp[-1].poses) == 0
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in seen_cm[-1].markers)

    # 2) NO_VALID_PATH trên tất cả nhà vệ sinh -> vẫn sạch hoàn toàn
    mock_server.unreachable_x.update([toilet_1_x, toilet_2_x])
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p15_2'
    })))
    for _ in range(40):
        spin_all(1)
        if nav_node.state == MissionState.IDLE and len(seen_says) >= 2:
            break
    spin_all(5)
    assert nav_node.state == MissionState.IDLE
    assert seen_ns[-1]['confirmed_goal_valid'] is False
    assert seen_cg[-1].header.frame_id == ''
    assert len(seen_cp[-1].poses) == 0
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in seen_cm[-1].markers)

    # 3) Hủy trong lúc RESOLVING -> sạch hoàn toàn
    mock_server.unreachable_x.clear()
    mock_server.delayed_x[toilet_1_x] = 0.25
    nav_node.on_intent(String(data=json.dumps({
        'action': 'navigate', 'category': 'toilet', 'request_id': 'req_p15_3'
    })))
    spin_all(2)
    assert nav_node.state == MissionState.RESOLVING
    nav_node.handle_cancel_request('req_p15_cancel_res')
    spin_all(8)
    assert nav_node.state == MissionState.IDLE
    assert seen_ns[-1]['confirmed_goal_valid'] is False
    assert seen_cg[-1].header.frame_id == ''
    assert len(seen_cp[-1].poses) == 0
    assert all(m.action in (Marker.DELETE, Marker.DELETEALL) for m in seen_cm[-1].markers)

    obs.destroy_node()
    nav_node.destroy_node()
    mock_server.destroy_node()
