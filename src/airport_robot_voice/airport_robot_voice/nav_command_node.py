"""
Cổng điều phối nhiệm vụ Nav2 và quản lý trạng thái di chuyển (NavCommandNode).
Hoàn thiện kiến trúc máy trạng thái, đồng bộ vòng đời Action Nav2, chọn đích
bằng ComputePathToPose (Giai đoạn 1 & 2), và liên kết TTS (Giai đoạn 3).

Đặc tính chính:
- Tách biệt rõ ràng 3 pha hủy/dừng: Gửi yêu cầu -> Server chấp nhận -> Terminal result.
- Xử lý máy trạng thái an toàn khi server từ chối hủy (revert state, cảnh báo, chặn resume/replace).
- Phân lập phiên chọn đường planner (resolution_id, candidate_attempt_id) chống callback muộn.
- Hỗ trợ đổi đích (replace) sang category nhóm điểm với logic planner tương đương.
- Hỗ trợ pause/resume trong lúc đang RESOLVING với dữ liệu category được bảo toàn.
- Xác thực toạ độ hữu hạn (x, y, yaw) và đúng frame_id ('map').
- Tích hợp timeout cho acceptance, cancel response, planner candidates, tts_ack.
- Độc lập với voice: công bố trạng thái qua /voice/nav_state (QoS Transient Local).
"""

import collections
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory


class MissionState:
    IDLE = 'IDLE'
    RESOLVING = 'RESOLVING'
    ANNOUNCING = 'ANNOUNCING'
    STARTING = 'STARTING'
    NAVIGATING = 'NAVIGATING'
    PAUSING = 'PAUSING'
    PAUSED = 'PAUSED'
    CANCELING = 'CANCELING'


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def compute_path_length(path_msg) -> float:
    """Tính tổng khoảng cách euclidean dọc theo toàn bộ các pose trong Path."""
    if not path_msg or not hasattr(path_msg, 'poses') or len(path_msg.poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path_msg.poses)):
        p1 = path_msg.poses[i - 1].pose.position
        p2 = path_msg.poses[i].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        total += math.sqrt(dx * dx + dy * dy)
    return total


class NavCommandNode(Node):
    def __init__(self):
        super().__init__('nav_command_node')

        share = os.path.join(os.path.dirname(__file__), '..', 'config')
        try:
            share = os.path.join(get_package_share_directory('airport_robot_voice'), 'config')
        except Exception:
            pass

        self.declare_parameter('places_file', os.path.join(share, 'places.json'))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('nav_timeout_sec', 5.0)
        self.declare_parameter('planner_timeout_sec', 3.0)
        self.declare_parameter('tts_ack_timeout_sec', 4.0)
        self.declare_parameter('feedback_freshness_sec', 5.0)
        self.declare_parameter('cancel_timeout_sec', 3.0)
        self.declare_parameter('goal_acceptance_timeout_sec', 4.0)
        self.declare_parameter('tts_policy', 'flexible')  # 'strict' | 'flexible'

        places_path = self.get_parameter('places_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.planner_action_name = self.get_parameter('planner_action_name').value
        self.nav_timeout = float(self.get_parameter('nav_timeout_sec').value)
        self.planner_timeout = float(self.get_parameter('planner_timeout_sec').value)
        self.tts_ack_timeout = float(self.get_parameter('tts_ack_timeout_sec').value)
        self.feedback_freshness_sec = float(self.get_parameter('feedback_freshness_sec').value)
        self.cancel_timeout_sec = float(self.get_parameter('cancel_timeout_sec').value)
        self.goal_acceptance_timeout_sec = float(self.get_parameter('goal_acceptance_timeout_sec').value)
        self.tts_policy = str(self.get_parameter('tts_policy').value)

        with open(places_path, 'r', encoding='utf-8') as f:
            db = json.load(f)
            self.places = db.get('places', {})
            self.categories = db.get('categories', {})

        # Máy trạng thái & Danh tính
        self.state = MissionState.IDLE
        self._mission_counter = 0
        self._goal_attempt_counter = 0
        self._speech_counter = 0

        self._current_mission_id = None
        self._current_request_id = None
        self._current_goal_attempt_id = None
        self._current_target_key = None
        self._current_target_name = None
        self._current_target_pose = None

        self._active_goal_handle = None
        self._pending_goal_future = None
        self._cancelled_attempts = set()

        # Quản lý Resolving (Planner)
        self._resolving_active = False
        self._resolution_counter = 0
        self._current_resolution_id = None
        self._current_cand_attempt_id = None
        self._planner_eval = None
        self._planner_candidate_timer = None
        self._paused_resolution = None  # Lưu category khi pause trong lúc RESOLVING

        # Quản lý Announcing (TTS)
        self._pending_speech_id = None
        self._tts_timeout_timer = None

        # Quản lý Replace
        self._pending_replace_target = None
        self._pending_replace_category = None
        self._pending_replace_req_id = None

        # Quản lý Timers
        self._acceptance_timeout_timer = None
        self._cancel_response_timer = None

        # Feedback
        self._distance_remaining = None
        self._eta_seconds = None
        self._last_feedback_time = None

        # Cache chống trùng lặp request_id (LRU 200 items)
        self._processed_requests = collections.OrderedDict()

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)

        # Publishers & Subscribers
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_nav_state = self.create_publisher(String, '/voice/nav_state', qos_transient)
        self.pub_feedback = self.create_publisher(String, '/voice/feedback', 10)
        self.pub_say = self.create_publisher(String, '/voice/say', 10)
        self.pub_tts_cancel = self.create_publisher(String, '/voice/tts_cancel', 10)

        self.create_subscription(String, '/voice/intent', self.on_intent, 10)
        self.create_subscription(Bool, '/voice/speaking', self.on_speaking, 10)
        self.create_subscription(String, '/voice/tts_ack', self.on_tts_ack, 10)

        # Heartbeat timer (1.0s) để duy trì /voice/nav_state latched và fresh heartbeat cho intent_node
        self._heartbeat_timer = self.create_timer(1.0, self.publish_state)

        # Công bố trạng thái ban đầu
        self.publish_state()
        self.get_logger().info('NavCommandNode đã khởi động và sẵn sàng.')

    def destroy_node(self):
        if hasattr(self, '_heartbeat_timer') and self._heartbeat_timer:
            try:
                self._heartbeat_timer.cancel()
                self.destroy_timer(self._heartbeat_timer)
            except Exception:
                pass
        self._cancel_tts_timeout_timer()
        self._cancel_planner_candidate_timer()
        self._stop_acceptance_timer()
        self._stop_cancel_timer()
        super().destroy_node()

    # ------------------------------------------------------------------
    # Giao tiếp giọng nói và sự kiện
    # ------------------------------------------------------------------
    def say(self, text: str, speech_id: str = None, mission_id: str = None):
        """Phát thông điệp ra /voice/say. Hỗ trợ JSON payload kèm speech_id để theo dõi ack."""
        if not speech_id:
            self._speech_counter += 1
            speech_id = f"sp_{self._speech_counter}_{int(time.time() * 1000) % 100000}"

        payload = {
            'speech_id': speech_id,
            'text': text,
            'mission_id': mission_id or self._current_mission_id,
            'timestamp': time.time()
        }
        self.pub_say.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        return speech_id

    def cancel_tts(self, speech_id: str = None, mission_id: str = None):
        """Gửi thông báo vô hiệu hóa câu nói trong hàng đợi TTS khi có lệnh cancel/replace."""
        payload = {'speech_id': speech_id, 'mission_id': mission_id}
        self.pub_tts_cancel.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def on_speaking(self, msg: Bool):
        pass

    def on_tts_ack(self, msg: String):
        """Nhận phản hồi từ TTS Node. Kiểm tra speech_id, mission_id, và status trước khi xuất phát."""
        try:
            ack = json.loads(msg.data)
            ack_speech_id = ack.get('speech_id')
            ack_mission_id = ack.get('mission_id')
            status = ack.get('status', 'COMPLETED')
            audio_played = ack.get('audio_played', False)
        except Exception:
            return

        if self.state == MissionState.ANNOUNCING and ack_speech_id == self._pending_speech_id:
            # Kiểm tra mission_id nếu có
            if ack_mission_id and ack_mission_id != self._current_mission_id:
                self.get_logger().warn(f'Bỏ qua TTS ACK vì mission_id không khớp: {ack_mission_id} != {self._current_mission_id}')
                return

            if status == 'CANCELLED_BEFORE_PLAYBACK':
                self.get_logger().info(f'Thông báo [{ack_speech_id}] đã bị hủy trước khi phát.')
                return

            if status == 'DROPPED_QUEUE_FULL':
                self.get_logger().warn(f'Thông báo [{ack_speech_id}] bị DROPPED_QUEUE_FULL.')
                if self.tts_policy == 'strict':
                    self.say('Hệ thống âm thanh bận, không thể phát thông báo.')
                    self.state = MissionState.IDLE
                    self.publish_state()
                    self.publish_feedback('FAILED', 'TTS_DROPPED_QUEUE_FULL')
                    return

            if status.startswith('ERROR_'):
                self.get_logger().warn(f'Thông báo [{ack_speech_id}] gặp lỗi TTS: {status}')
                if self.tts_policy == 'strict':
                    self.state = MissionState.IDLE
                    self.publish_state()
                    self.publish_feedback('FAILED', f'TTS_{status}')
                    return

            self.get_logger().info(f'Nhận được TTS ACK hợp lệ cho [{ack_speech_id}] (status={status}). Tiến hành gửi goal Nav2.')
            self._cancel_tts_timeout_timer()
            self._pending_speech_id = None
            self.state = MissionState.STARTING
            self.publish_state()
            self.send_nav_goal()

    def _cancel_tts_timeout_timer(self):
        if self._tts_timeout_timer:
            try:
                self._tts_timeout_timer.cancel()
                self.destroy_timer(self._tts_timeout_timer)
            except Exception:
                pass
            self._tts_timeout_timer = None

    def _on_tts_ack_timeout(self, expected_speech_id: str):
        self._cancel_tts_timeout_timer()
        if self.state == MissionState.ANNOUNCING and self._pending_speech_id == expected_speech_id:
            self.get_logger().warn(f'Timeout ({self.tts_ack_timeout}s) chờ tts_ack cho [{expected_speech_id}]. Tiến hành gửi goal fallback.')
            self._pending_speech_id = None
            self.state = MissionState.STARTING
            self.publish_state()
            self.send_nav_goal()

    # ------------------------------------------------------------------
    # Timer cho Acceptance và Cancel Response
    # ------------------------------------------------------------------
    def _start_acceptance_timer(self, attempt_id: str):
        self._stop_acceptance_timer()
        self._acceptance_timeout_timer = self.create_timer(
            self.goal_acceptance_timeout_sec,
            lambda: self._on_acceptance_timeout(attempt_id)
        )

    def _stop_acceptance_timer(self):
        if hasattr(self, '_acceptance_timeout_timer') and self._acceptance_timeout_timer:
            try:
                self._acceptance_timeout_timer.cancel()
                self.destroy_timer(self._acceptance_timeout_timer)
            except Exception:
                pass
            self._acceptance_timeout_timer = None

    def _on_acceptance_timeout(self, attempt_id: str):
        self._stop_acceptance_timer()
        if self.state == MissionState.STARTING and attempt_id == self._current_goal_attempt_id:
            self.get_logger().error(f"[{attempt_id}] Timeout ({self.goal_acceptance_timeout_sec}s) chờ Nav2 chấp nhận goal.")
            speech = 'Hệ thống điều hướng không phản hồi yêu cầu di chuyển.'
            self.say(speech)
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('FAILED', 'ACCEPTANCE_TIMEOUT', speech)

    def _start_cancel_timer(self, attempt_id: str, purpose: str):
        self._stop_cancel_timer()
        self._cancel_response_timer = self.create_timer(
            self.cancel_timeout_sec,
            lambda: self._on_cancel_timeout(attempt_id, purpose)
        )

    def _stop_cancel_timer(self):
        if hasattr(self, '_cancel_response_timer') and self._cancel_response_timer:
            try:
                self._cancel_response_timer.cancel()
                self.destroy_timer(self._cancel_response_timer)
            except Exception:
                pass
            self._cancel_response_timer = None

    def _on_cancel_timeout(self, attempt_id: str, purpose: str):
        self._stop_cancel_timer()
        if attempt_id != self._current_goal_attempt_id:
            return
        self.get_logger().error(f"[{attempt_id}] Timeout ({self.cancel_timeout_sec}s) chờ phản hồi hủy goal (mục đích: {purpose}).")
        # Không tự ý coi như đã dừng an toàn! Robot có thể vẫn đang chạy!
        if self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self.state = MissionState.NAVIGATING
            speech = 'Không nhận được phản hồi hủy từ hệ thống điều hướng. Robot tiếp tục lộ trình.'
            self.say(speech)
            self.publish_feedback('CANCEL_TIMEOUT', 'TIMEOUT', speech)
            self.publish_state()

    # ------------------------------------------------------------------
    # Công bố trạng thái
    # ------------------------------------------------------------------
    def publish_state(self):
        payload = {
            'schema_version': '1.0',
            'node_online': True,
            'mission_state': self.state,
            'mission_id': self._current_mission_id,
            'goal_attempt_id': self._current_goal_attempt_id,
            'current_target_id': self._current_target_key,
            'current_target_name': self._current_target_name,
            'distance_remaining': self._distance_remaining,
            'eta_seconds': self._eta_seconds,
            'last_feedback_time': self._last_feedback_time,
            'timestamp': time.time()
        }
        self.pub_nav_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_feedback(self, event: str, status: str, message: str = ''):
        payload = {
            'schema_version': '1.0',
            'event': event,
            'status': status,
            'message': message,
            'mission_id': self._current_mission_id,
            'goal_attempt_id': self._current_goal_attempt_id,
            'target_id': self._current_target_key,
            'timestamp': time.time()
        }
        self.pub_feedback.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # Tiếp nhận intent
    # ------------------------------------------------------------------
    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Lỗi giải mã /voice/intent: {e}')
            return

        action = data.get('action')
        req_id = data.get('request_id', f'req_{int(time.time()*1000)}')

        # Deduplication request_id qua LRU Cache
        now = time.time()
        if req_id in self._processed_requests:
            if now - self._processed_requests[req_id] < 10.0:
                self.get_logger().info(f'Bỏ qua request trùng lặp: {req_id}')
                return
        self._processed_requests[req_id] = now
        while len(self._processed_requests) > 200:
            self._processed_requests.popitem(last=False)

        if action == 'navigate':
            self.handle_navigate_request(data)
        elif action == 'pause':
            self.handle_pause_request(req_id)
        elif action == 'resume':
            self.handle_resume_request(req_id)
        elif action == 'cancel':
            self.handle_cancel_request(req_id)
        elif action == 'replace':
            self.handle_replace_target(data)
        elif action == 'status':
            self.handle_status_request(req_id)

    # ------------------------------------------------------------------
    # GIAI ĐOẠN 1: Điều hướng & Chọn đích bằng Planner
    # ------------------------------------------------------------------
    def handle_navigate_request(self, data: dict):
        target_id = data.get('target_id')
        category = data.get('category')
        req_id = data.get('request_id')

        # Đang trong tiến trình chuyển tiếp (PAUSING hoặc CANCELING)
        if self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self.get_logger().warn('Nhận lệnh navigate khi đang PAUSING/CANCELING. Chuyển thành replace hoặc yêu cầu chờ.')
            if target_id or category:
                self.handle_replace_target(data)
            return

        # 1. Đích cụ thể
        if target_id:
            if target_id not in self.places or not self.places[target_id].get('enabled', True):
                name = self.places.get(target_id, {}).get('name', target_id)
                self.say(f'Địa điểm {name} hiện không khả dụng.')
                self.publish_feedback('REJECTED', 'DISABLED_OR_UNKNOWN_TARGET')
                return
            self.start_specific_mission(target_id, req_id)
            return

        # 2. Đích theo category (nhóm điểm)
        if category:
            if category not in self.categories:
                self.say('Không tìm thấy danh mục yêu cầu.')
                self.publish_feedback('REJECTED', 'UNKNOWN_CATEGORY')
                return
            self.resolve_category_and_start(category, req_id)
            return

        self.say('Không xác định được điểm đến hợp lệ.')
        self.publish_feedback('REJECTED', 'NO_TARGET_SPECIFIED')

    def get_valid_candidates(self, category: str):
        candidates = []
        member_keys = self.categories.get(category, {}).get('members')
        if not member_keys:
            member_keys = [k for k, p in self.places.items() if p.get('category') == category]
        for key in member_keys:
            if key not in self.places:
                continue
            p = self.places[key]
            if not p.get('enabled', True):
                continue
            if p.get('frame_id', self.frame_id) != self.frame_id:
                continue
            try:
                x = float(p.get('x'))
                y = float(p.get('y'))
                yaw = float(p.get('yaw', 0.0))
                if any(math.isnan(v) or math.isinf(v) for v in (x, y, yaw)):
                    continue
            except (ValueError, TypeError):
                continue
            candidates.append(key)
        return candidates

    def resolve_category_and_start(self, category: str, req_id: str):
        self._resolution_counter += 1
        res_id = f"res_{self._resolution_counter}_{int(time.time() * 1000) % 100000}"
        self._current_resolution_id = res_id
        self.state = MissionState.RESOLVING
        self._resolving_active = True
        self._paused_resolution = None
        self.publish_state()

        candidates = self.get_valid_candidates(category)
        if not candidates:
            self.say('Không tìm thấy địa điểm nào phù hợp trong danh mục.')
            self.state = MissionState.IDLE
            self._resolving_active = False
            self.publish_state()
            self.publish_feedback('FAILED', 'NO_CANDIDATES')
            return

        # Kiểm tra planner action server (với timeout ngắn)
        if not self.planner_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(f'Planner server chưa sẵn sàng. Fallback chọn ứng viên đầu tiên: {candidates[0]}')
            chosen_key = candidates[0]
            chosen_name = self.places[chosen_key].get('name', chosen_key)
            self._resolving_active = False
            speech = f'Bộ điều phối đường chưa sẵn sàng nên chưa thể tìm điểm tối ưu theo đường đi. Tôi sẽ tạm dẫn bạn đến {chosen_name}.'
            self.say(speech)
            self.publish_feedback('PLANNER_UNAVAILABLE_FALLBACK', 'WARNING_PLANNER_UNAVAILABLE', speech)
            self.start_specific_mission(chosen_key, req_id, custom_speech=speech)
            return

        self.get_logger().info(f'[{res_id}] Bắt đầu truy vấn planner cho {len(candidates)} ứng viên trong nhóm {category}...')
        self._planner_eval = {
            'resolution_id': res_id,
            'candidates': candidates,
            'current_idx': 0,
            'results': {},
            'req_id': req_id,
            'category': category
        }
        self._evaluate_next_planner_candidate()

    def _evaluate_next_planner_candidate(self):
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            return
        eval_data = self._planner_eval
        if not eval_data:
            return
        res_id = eval_data.get('resolution_id')
        if res_id != self._current_resolution_id:
            return

        idx = eval_data['current_idx']
        candidates = eval_data['candidates']

        if idx >= len(candidates):
            self._resolving_active = False
            results = eval_data['results']
            if results:
                best_key = min(results, key=results.get)
                best_length = results[best_key]
                self.get_logger().info(f"[{res_id}] Planner đã chọn {best_key} (khoảng cách {best_length:.2f}m) trong các ứng viên: {results}")
                self.start_specific_mission(best_key, eval_data['req_id'])
            else:
                self.get_logger().error(f"[{res_id}] Tất cả các ứng viên đều không tìm được đường đi khả dụng.")
                self.say('Hệ thống không tìm thấy đường đi tới các điểm yêu cầu.')
                self.state = MissionState.IDLE
                self.publish_state()
                self.publish_feedback('FAILED', 'NO_VALID_PATH')
            return

        cand_key = candidates[idx]
        cand_attempt_id = f"cand_{res_id}_{idx}"
        self._current_cand_attempt_id = cand_attempt_id
        place = self.places[cand_key]
        goal = ComputePathToPose.Goal()
        goal.goal = self.create_pose_stamped(place)
        goal.use_start = False
        goal.planner_id = ''

        self.get_logger().info(f"[{res_id}] Đang tính đường tới ứng viên [{idx + 1}/{len(candidates)}]: {cand_key}")
        self._cancel_planner_candidate_timer()
        self._planner_candidate_timer = self.create_timer(
            self.planner_timeout,
            lambda: self._on_planner_candidate_timeout(res_id, cand_attempt_id, cand_key)
        )
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut: self._on_planner_goal_response(fut, res_id, cand_attempt_id, cand_key)
        )

    def _cancel_planner_candidate_timer(self):
        if hasattr(self, '_planner_candidate_timer') and self._planner_candidate_timer:
            try:
                self._planner_candidate_timer.cancel()
                self.destroy_timer(self._planner_candidate_timer)
            except Exception:
                pass
            self._planner_candidate_timer = None

    def _on_planner_candidate_timeout(self, res_id: str, cand_attempt_id: str, cand_key: str):
        self._cancel_planner_candidate_timer()
        if (res_id != self._current_resolution_id or
            cand_attempt_id != getattr(self, '_current_cand_attempt_id', None) or
            not self._resolving_active or
            self.state != MissionState.RESOLVING):
            return
        self.get_logger().warn(f"[{res_id}] Timeout ({self.planner_timeout}s) tính đường cho ứng viên {cand_key}. Bỏ qua ứng viên này.")
        self._current_cand_attempt_id = None
        self._advance_planner_evaluation(res_id)

    def _on_planner_goal_response(self, future, res_id: str, cand_attempt_id: str, cand_key: str):
        if (res_id != self._current_resolution_id or
            cand_attempt_id != getattr(self, '_current_cand_attempt_id', None) or
            not self._resolving_active or
            self.state != MissionState.RESOLVING):
            try:
                gh = future.result()
                if gh and gh.accepted:
                    gh.cancel_goal_async()
            except Exception:
                pass
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[{res_id}] Lỗi gửi goal planner cho {cand_key}: {e}")
            self._advance_planner_evaluation(res_id)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"[{res_id}] Planner từ chối tính đường cho {cand_key}")
            self._advance_planner_evaluation(res_id)
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(
            lambda rfut: self._on_planner_goal_result(rfut, res_id, cand_attempt_id, cand_key)
        )

    def _on_planner_goal_result(self, future, res_id: str, cand_attempt_id: str, cand_key: str):
        if (res_id != self._current_resolution_id or
            cand_attempt_id != getattr(self, '_current_cand_attempt_id', None) or
            not self._resolving_active or
            self.state != MissionState.RESOLVING):
            self.get_logger().info(f"[{res_id}] Bỏ qua kết quả muộn của candidate attempt cũ: {cand_attempt_id} ({cand_key})")
            return
        self._cancel_planner_candidate_timer()
        self._current_cand_attempt_id = None

        try:
            wrapped_result = future.result()
            res = wrapped_result.result
            error_code = getattr(res, 'error_code', 0)
            path = getattr(res, 'path', None)
            poses = getattr(path, 'poses', []) if path else []

            if error_code == 0 and len(poses) >= 2:
                length = compute_path_length(path)
                self._planner_eval['results'][cand_key] = length
                self.get_logger().info(f"[{res_id}] Ứng viên {cand_key}: đường đi hợp lệ = {length:.2f}m")
            elif 100 <= error_code < 200:
                self.get_logger().warn(f"[{res_id}] Ứng viên {cand_key}: Lỗi TF (error_code={error_code})")
            else:
                self.get_logger().warn(f"[{res_id}] Ứng viên {cand_key}: Không tìm thấy đường đi (error_code={error_code}, poses={len(poses)})")
        except Exception as e:
            self.get_logger().error(f"[{res_id}] Lỗi đọc kết quả path cho {cand_key}: {e}")

        self._advance_planner_evaluation(res_id)

    def _advance_planner_evaluation(self, res_id: str):
        self._cancel_planner_candidate_timer()
        if (res_id != self._current_resolution_id or
            not self._resolving_active or
            self.state != MissionState.RESOLVING):
            return
        self._planner_eval['current_idx'] += 1
        self._evaluate_next_planner_candidate()

    # ------------------------------------------------------------------
    # GIAI ĐOẠN 2 & 3: Khởi động nhiệm vụ cụ thể & Đồng bộ TTS
    # ------------------------------------------------------------------
    def validate_target(self, target_key: str, category: str = None) -> str:
        """Xác minh điểm đến hợp lệ (tọa độ hữu hạn x, y, yaw, đúng map/frame)."""
        if target_key and target_key in self.places and self.places[target_key].get('enabled', True):
            p = self.places[target_key]
            if p.get('frame_id', self.frame_id) == self.frame_id:
                try:
                    x = float(p.get('x'))
                    y = float(p.get('y'))
                    yaw = float(p.get('yaw', 0.0))
                    if not any(math.isnan(v) or math.isinf(v) for v in (x, y, yaw)):
                        return target_key
                except (ValueError, TypeError):
                    pass
        return None

    def start_specific_mission(self, target_key: str, req_id: str, custom_speech: str = None):
        self._mission_counter += 1
        self._current_mission_id = f"mission_{self._mission_counter}_{int(time.time() * 1000) % 100000}"
        self._current_request_id = req_id
        self._current_target_key = target_key

        place = self.places[target_key]
        self._current_target_name = place.get('name', target_key)
        self._current_target_pose = self.create_pose_stamped(place)
        self._paused_resolution = None

        # Trạng thái ANNOUNCING: Phát câu nói và chờ tts_ack
        self.state = MissionState.ANNOUNCING
        self.publish_state()

        if custom_speech:
            speech = custom_speech
        else:
            speech = f'Tôi sẽ dẫn bạn đến {self._current_target_name}. Xin hãy đi theo tôi.'

        self._pending_speech_id = self.say(speech, mission_id=self._current_mission_id)
        self.publish_feedback('ANNOUNCING', 'OK', speech)

        # Timer timeout dự phòng nếu TTS vắng mặt hoặc ack bị trễ
        self._cancel_tts_timeout_timer()
        self._tts_timeout_timer = self.create_timer(
            self.tts_ack_timeout,
            lambda: self._on_tts_ack_timeout(self._pending_speech_id)
        )

    def create_pose_stamped(self, place: dict) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = place.get('frame_id', self.frame_id)
        p.pose.position.x = float(place.get('x', 0.0))
        p.pose.position.y = float(place.get('y', 0.0))
        p.pose.position.z = 0.0
        yaw = float(place.get('yaw', 0.0))
        qx, qy, qz, qw = yaw_to_quat(yaw)
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def send_nav_goal(self):
        if not self._current_target_pose:
            self.get_logger().error('Không có pose để gửi cho Nav2!')
            self.state = MissionState.IDLE
            self.publish_state()
            return

        self._goal_attempt_counter += 1
        attempt_id = f"att_{self._goal_attempt_counter}_{int(time.time() * 1000) % 100000}"
        self._current_goal_attempt_id = attempt_id
        mission_id = self._current_mission_id

        goal = NavigateToPose.Goal()
        goal.pose = self._current_target_pose

        self.get_logger().info(f'[{attempt_id}] Gửi goal tới Nav2 cho mission {mission_id} ({self._current_target_name})...')
        self._start_acceptance_timer(attempt_id)
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda f_msg: self.on_nav_feedback(f_msg.feedback, attempt_id)
        )
        future.add_done_callback(
            lambda fut: self.on_goal_response(fut, attempt_id, mission_id)
        )

    def on_nav_feedback(self, feedback_msg, attempt_id: str):
        # Bỏ qua feedback của attempt cũ
        if attempt_id != self._current_goal_attempt_id or self.state != MissionState.NAVIGATING:
            return

        self._distance_remaining = getattr(feedback_msg, 'distance_remaining', None)
        eta_obj = getattr(feedback_msg, 'estimated_time_remaining', None)
        if eta_obj:
            self._eta_seconds = getattr(eta_obj, 'sec', None)
        self._last_feedback_time = time.time()
        self.publish_state()

    def on_goal_response(self, future, attempt_id: str, mission_id: str):
        self._stop_acceptance_timer()
        self._pending_goal_future = None

        # 1. Nếu attempt này đã bị đánh dấu cancel trước khi kịp accept
        is_pre_cancelled = (attempt_id in self._cancelled_attempts)
        is_outdated_mission = (mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi nhận goal handle: {e}')
            if not is_outdated_mission:
                self.state = MissionState.IDLE
                self.publish_state()
            return

        # Goal bị từ chối bởi server
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{attempt_id}] Goal bị Nav2 từ chối.')
            if self._pending_replace_target or self._pending_replace_category:
                self._execute_pending_replace()
                return
            if is_outdated_mission or is_pre_cancelled:
                if self.state == MissionState.PAUSING:
                    self._transition_to_paused()
                elif self.state == MissionState.CANCELING:
                    self._transition_to_cancelled()
                return
            self.say('Tôi không thể di chuyển tới đó lúc này.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'GOAL_REJECTED')
            return

        # Goal được chấp nhận nhưng đã bị thu hồi / cancel trước đó
        if is_outdated_mission or is_pre_cancelled:
            self.get_logger().info(f'[{attempt_id}] Thu hồi goal cũ được chấp nhận muộn.')
            cancel_fut = goal_handle.cancel_goal_async()
            cancel_fut.add_done_callback(
                lambda cf: self._on_delayed_cancel_confirmed(cf, attempt_id)
            )
            # Đồng thời theo dõi terminal result của goal thu hồi
            goal_handle.get_result_async().add_done_callback(
                lambda rf: self._on_delayed_cancel_terminal_result(rf, attempt_id)
            )
            return

        # Goal được chấp nhận bình thường cho mission hiện tại
        self._active_goal_handle = goal_handle
        self.get_logger().info(f'[{attempt_id}] Goal được Nav2 chấp nhận. Bắt đầu di chuyển.')
        self.state = MissionState.NAVIGATING
        self.publish_state()
        self.publish_feedback('STARTED', 'GOAL_ACCEPTED')

        # Chờ kết quả điều hướng
        goal_handle.get_result_async().add_done_callback(
            lambda fut: self.on_goal_result(fut, attempt_id, mission_id)
        )

    def _on_delayed_cancel_confirmed(self, future, attempt_id: str):
        try:
            res = future.result()
            self.get_logger().info(f'[{attempt_id}] Phản hồi hủy goal thu hồi muộn (return_code={res.return_code}).')
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi hủy goal thu hồi muộn: {e}')

    def _on_delayed_cancel_terminal_result(self, future, attempt_id: str):
        self.get_logger().info(f'[{attempt_id}] Goal thu hồi muộn đã kết thúc terminal.')
        if self._pending_replace_target or self._pending_replace_category:
            self._execute_pending_replace()
        elif self.state == MissionState.PAUSING:
            self._transition_to_paused()
        elif self.state == MissionState.CANCELING:
            self._transition_to_cancelled()

    def on_goal_result(self, future, attempt_id: str, mission_id: str):
        self._stop_cancel_timer()
        # Bỏ qua nếu không phải attempt hiện tại
        if attempt_id != self._current_goal_attempt_id or mission_id != self._current_mission_id:
            self.get_logger().warn(f'Bỏ qua goal result của attempt cũ: {attempt_id}')
            return

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi lấy kết quả goal: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f'[{attempt_id}] Goal kết thúc terminal với status: {status}')
        self._active_goal_handle = None

        # 1. TH Hủy thành công (STATUS_CANCELED)
        if status == GoalStatus.STATUS_CANCELED:
            if self._pending_replace_target or self._pending_replace_category:
                self._execute_pending_replace()
                return
            elif self.state == MissionState.PAUSING:
                self._transition_to_paused()
                return
            elif self.state == MissionState.CANCELING:
                self._transition_to_cancelled()
                return

        # 2. TH Đã tới đích thành công trước khi kịp hủy hoặc khi đang chạy
        if status == GoalStatus.STATUS_SUCCEEDED:
            speech = f'Chúng ta đã tới {self._current_target_name}. Chúc bạn chuyến đi vui vẻ!'
            self.say(speech)
            self.publish_feedback('SUCCEEDED', 'OK', speech)
            self.state = MissionState.IDLE
            self._current_target_key = None
            self._current_target_name = None
            self._current_target_pose = None
            self._pending_replace_target = None
            self._pending_replace_category = None
            self.publish_state()
            return

        # 3. TH Gặp chướng ngại hoặc abort
        if status == GoalStatus.STATUS_ABORTED:
            speech = 'Tôi gặp trở ngại trên đường đi và không thể tiếp tục.'
            self.say(speech)
            self.publish_feedback('FAILED', 'ABORTED', speech)
            self.state = MissionState.IDLE
            self._current_target_key = None
            self._current_target_name = None
            self._current_target_pose = None
            self._pending_replace_target = None
            self._pending_replace_category = None
            self.publish_state()
            return

        # Các lỗi khác
        speech = 'Điều hướng không thành công.'
        self.say(speech)
        self.publish_feedback('FAILED', f'STATUS_{status}', speech)
        self.state = MissionState.IDLE
        self._pending_replace_target = None
        self._pending_replace_category = None
        self.publish_state()

    # ------------------------------------------------------------------
    # Tạm dừng (PAUSE) và Đi tiếp (RESUME)
    # ------------------------------------------------------------------
    def handle_pause_request(self, req_id: str):
        if self.state == MissionState.NAVIGATING:
            self.state = MissionState.PAUSING
            self.publish_state()
            self.get_logger().info('Đang gửi yêu cầu hủy goal để tạm dừng...')
            if self._active_goal_handle:
                attempt_id = self._current_goal_attempt_id
                self._start_cancel_timer(attempt_id, purpose='pause')
                fut = self._active_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self._on_pause_cancel_response(f, attempt_id))
            else:
                self._transition_to_paused()

        elif self.state == MissionState.STARTING:
            self.state = MissionState.PAUSING
            self.publish_state()
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            self.get_logger().info('Đã nhận lệnh tạm dừng khi đang STARTING. Sẽ hủy ngay khi goal được chấp nhận.')

        elif self.state == MissionState.ANNOUNCING:
            self._cancel_tts_timeout_timer()
            self.cancel_tts(speech_id=self._pending_speech_id, mission_id=self._current_mission_id)
            self._pending_speech_id = None
            self._transition_to_paused()

        elif self.state == MissionState.RESOLVING:
            self._resolving_active = False
            self._cancel_planner_candidate_timer()
            self._paused_resolution = {
                'category': self._planner_eval.get('category'),
                'req_id': self._planner_eval.get('req_id')
            } if self._planner_eval else None
            self._transition_to_paused(is_resolving_pause=True)

        elif self.state == MissionState.PAUSED:
            self.say('Robot đã đang ở trạng thái tạm dừng.')

        elif self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self.say('Yêu cầu dừng hoặc hủy đang được xử lý, vui lòng chờ.')

        else:
            self.say('Hiện không có chuyến đi nào để tạm dừng.')

    def _on_pause_cancel_response(self, future, attempt_id: str):
        self._stop_cancel_timer()
        if attempt_id != self._current_goal_attempt_id:
            return
        try:
            cancel_res = future.result()
            return_code = getattr(cancel_res, 'return_code', 0)
            goals_canceling = getattr(cancel_res, 'goals_canceling', [])
        except Exception as e:
            self.get_logger().error(f"[{attempt_id}] Lỗi phản hồi cancel: {e}")
            return_code = 1
            goals_canceling = []

        is_accepted = (return_code == 0 and len(goals_canceling) > 0)
        if not is_accepted:
            # Server từ chối hủy! Robot vẫn đang di chuyển!
            self.get_logger().warn(f"[{attempt_id}] Server từ chối hủy goal để tạm dừng (code={return_code}, canceling={len(goals_canceling)}). Robot tiếp tục di chuyển.")
            if self.state == MissionState.PAUSING:
                self.state = MissionState.NAVIGATING
                speech = 'Không thể tạm dừng lúc này do hệ thống điều hướng từ chối yêu cầu.'
                self.say(speech)
                self.publish_feedback('PAUSE_REJECTED', f'RETURN_CODE_{return_code}', speech)
                self.publish_state()
            return

        # Server chấp nhận cancel -> Chờ on_goal_result xác nhận STATUS_CANCELED trước khi chuyển PAUSED
        self.get_logger().info(f"[{attempt_id}] Server chấp nhận yêu cầu dừng. Đang chờ robot dừng hẳn...")

    def _transition_to_paused(self, is_resolving_pause: bool = False):
        self.state = MissionState.PAUSED
        self._active_goal_handle = None
        if is_resolving_pause:
            speech = 'Tôi đã tạm dừng việc tìm đường. Bạn bảo đi tiếp khi sẵn sàng nhé.'
        else:
            speech = 'Tôi đã tạm dừng. Bạn bảo đi tiếp khi sẵn sàng nhé.'
        self.say(speech)
        self.publish_feedback('PAUSED', 'OK', speech)
        self.publish_state()

    def handle_resume_request(self, req_id: str):
        # Resume CHỈ được phép khi PAUSED hoàn tất (không còn active goal handle)
        if self.state != MissionState.PAUSED or self._active_goal_handle is not None:
            self.say('Robot hiện không trong trạng thái tạm dừng hoàn tất.')
            return

        # TH1: Tạm dừng khi đang chọn đường bằng planner (RESOLVING)
        if self._paused_resolution:
            res_data = self._paused_resolution
            self._paused_resolution = None
            cat_name = self.categories.get(res_data['category'], {}).get('name', res_data['category'])
            speech = f'Chúng ta tiếp tục tìm đường tới {cat_name}.'
            self.say(speech)
            self.publish_feedback('RESUMED', 'OK', speech)
            self.resolve_category_and_start(res_data['category'], res_data['req_id'])
            return

        # TH2: Tạm dừng khi đang trên đường tới đích cụ thể
        if not self._current_target_pose:
            self.say('Không tìm thấy điểm đến để tiếp tục.')
            self.state = MissionState.IDLE
            self.publish_state()
            return

        speech = 'Chúng ta tiếp tục di chuyển.'
        self.say(speech)
        self.publish_feedback('RESUMED', 'OK', speech)
        self.state = MissionState.STARTING
        self.publish_state()
        self.send_nav_goal()

    # ------------------------------------------------------------------
    # Hủy nhiệm vụ (CANCEL)
    # ------------------------------------------------------------------
    def handle_cancel_request(self, req_id: str):
        if self.state == MissionState.NAVIGATING:
            self.state = MissionState.CANCELING
            self.publish_state()
            if self._active_goal_handle:
                attempt_id = self._current_goal_attempt_id
                self._start_cancel_timer(attempt_id, purpose='cancel')
                fut = self._active_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self._on_cancel_action_response(f, attempt_id))
            else:
                self._transition_to_cancelled()

        elif self.state == MissionState.PAUSING:
            # Nâng cấp từ PAUSING sang CANCELING
            self.state = MissionState.CANCELING
            self.publish_state()
            self.get_logger().info('Đã nhận lệnh hủy trong lúc đang PAUSING. Chuyển sang CANCELING và chờ kết thúc.')

        elif self.state == MissionState.STARTING:
            self.state = MissionState.CANCELING
            self.publish_state()
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            self.get_logger().info('Đã nhận lệnh hủy khi đang STARTING. Sẽ hủy ngay khi goal được chấp nhận.')

        elif self.state in (MissionState.ANNOUNCING, MissionState.RESOLVING, MissionState.PAUSED):
            self._cancel_tts_timeout_timer()
            self.cancel_tts(speech_id=self._pending_speech_id, mission_id=self._current_mission_id)
            self._pending_speech_id = None
            self._resolving_active = False
            self._cancel_planner_candidate_timer()
            self._paused_resolution = None
            self._transition_to_cancelled()

        elif self.state == MissionState.CANCELING:
            self.say('Yêu cầu hủy đang được thực hiện.')

        else:
            self.say('Hiện không có chuyến đi nào đang hoạt động.')

    def _on_cancel_action_response(self, future, attempt_id: str):
        self._stop_cancel_timer()
        if attempt_id != self._current_goal_attempt_id:
            return
        try:
            cancel_res = future.result()
            return_code = getattr(cancel_res, 'return_code', 0)
            goals_canceling = getattr(cancel_res, 'goals_canceling', [])
        except Exception as e:
            self.get_logger().error(f"[{attempt_id}] Lỗi phản hồi cancel: {e}")
            return_code = 1
            goals_canceling = []

        is_accepted = (return_code == 0 and len(goals_canceling) > 0)
        if not is_accepted:
            # Server từ chối hủy! Robot vẫn đang di chuyển!
            self.get_logger().warn(f"[{attempt_id}] Server từ chối hủy goal (code={return_code}, canceling={len(goals_canceling)}). Robot tiếp tục di chuyển.")
            if self.state == MissionState.CANCELING:
                self.state = MissionState.NAVIGATING
                speech = 'Không thể hủy chuyến đi lúc này do hệ thống điều hướng từ chối yêu cầu.'
                self.say(speech)
                self.publish_feedback('CANCEL_REJECTED', f'RETURN_CODE_{return_code}', speech)
                self.publish_state()
            return

        # Server chấp nhận cancel -> Chờ on_goal_result xác nhận STATUS_CANCELED trước khi chuyển CANCELLED/IDLE
        self.get_logger().info(f"[{attempt_id}] Server chấp nhận yêu cầu hủy. Đang chờ goal kết thúc terminal...")

    def _transition_to_cancelled(self):
        speech = 'Tôi đã hủy chuyến đi hiện tại.'
        self.say(speech)
        self.publish_feedback('CANCELED', 'OK', speech)

        # Dọn dẹp trạng thái về IDLE
        self.state = MissionState.IDLE
        self._current_mission_id = None
        self._current_goal_attempt_id = None
        self._current_target_key = None
        self._current_target_name = None
        self._current_target_pose = None
        self._active_goal_handle = None
        self._pending_replace_target = None
        self._pending_replace_category = None
        self._paused_resolution = None
        self.publish_state()

    # ------------------------------------------------------------------
    # Đổi đích (REPLACE)
    # ------------------------------------------------------------------
    def handle_replace_target(self, data: dict):
        new_target_id = data.get('target_id')
        new_cat = data.get('category')
        req_id = data.get('request_id')

        # 1. Xác minh điểm đến mới trước
        valid_key = self.validate_target(new_target_id)
        valid_cat = new_cat if (new_cat and new_cat in self.categories and self.get_valid_candidates(new_cat)) else None

        if not valid_key and not valid_cat:
            self.say('Điểm đến mới không hợp lệ. Tôi tiếp tục lộ trình hiện tại.')
            self.publish_feedback('REJECTED', 'INVALID_NEW_TARGET')
            return

        # 2. Nếu đang PAUSED: Cập nhật đích mới và GIỮ NGUYÊN trạng thái PAUSED
        if self.state == MissionState.PAUSED:
            if valid_key:
                new_place = self.places[valid_key]
                new_name = new_place.get('name', valid_key)
                self._current_target_key = valid_key
                self._current_target_name = new_name
                self._current_target_pose = self.create_pose_stamped(new_place)
                self._paused_resolution = None
                speech = f'Đã đổi đích sang {new_name}. Bạn bảo đi tiếp khi sẵn sàng nhé.'
            else:
                cat_name = self.categories[valid_cat].get('name', valid_cat)
                self._paused_resolution = {'category': valid_cat, 'req_id': req_id}
                self._current_target_key = None
                self._current_target_name = cat_name
                self._current_target_pose = None
                speech = f'Đã đổi đích sang {cat_name}. Bạn bảo đi tiếp khi sẵn sàng để tôi tìm đường nhé.'

            self.say(speech)
            self.publish_feedback('TARGET_REPLACED', 'OK', speech)
            self.publish_state()
            return

        # 3. Nếu đang ANNOUNCING hoặc RESOLVING: Hủy tiến trình cũ và đổi ngay
        if self.state in (MissionState.ANNOUNCING, MissionState.RESOLVING):
            self._cancel_tts_timeout_timer()
            self.cancel_tts(speech_id=self._pending_speech_id, mission_id=self._current_mission_id)
            self._pending_speech_id = None
            self._resolving_active = False
            self._cancel_planner_candidate_timer()
            if valid_cat:
                self.resolve_category_and_start(valid_cat, req_id)
            else:
                self.start_specific_mission(valid_key, req_id)
            return

        # 4. Nếu đang NAVIGATING hoặc STARTING hoặc PAUSING: Hủy goal cũ, chờ hoàn tất rồi mới gửi goal mới
        self._pending_replace_target = valid_key
        self._pending_replace_category = valid_cat
        self._pending_replace_req_id = req_id

        if self.state == MissionState.STARTING:
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            self.state = MissionState.PAUSING
            self.publish_state()
        elif self.state == MissionState.NAVIGATING:
            self.state = MissionState.PAUSING
            self.publish_state()
            if self._active_goal_handle:
                attempt_id = self._current_goal_attempt_id
                self._start_cancel_timer(attempt_id, purpose='replace')
                fut = self._active_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self._on_pause_cancel_response(f, attempt_id))
            else:
                self._execute_pending_replace()

    def _execute_pending_replace(self):
        target_key = self._pending_replace_target
        category = self._pending_replace_category
        req_id = self._pending_replace_req_id
        self._pending_replace_target = None
        self._pending_replace_category = None
        self._pending_replace_req_id = None

        if category:
            cat_name = self.categories.get(category, {}).get('name', category)
            speech = f'Đã đổi lộ trình, tôi sẽ tìm đường tới {cat_name} phù hợp nhất.'
            self.say(speech)
            self.resolve_category_and_start(category, req_id)
        elif target_key:
            place = self.places[target_key]
            name = place.get('name', target_key)
            speech = f'Đã đổi lộ trình, tôi sẽ dẫn bạn đến {name}.'
            self.say(speech)
            self.start_specific_mission(target_key, req_id)

    # ------------------------------------------------------------------
    # Hỏi trạng thái (STATUS)
    # ------------------------------------------------------------------
    def handle_status_request(self, req_id: str):
        now = time.time()
        is_fresh = (self._last_feedback_time is not None and (now - self._last_feedback_time) <= self.feedback_freshness_sec)

        if self.state == MissionState.NAVIGATING:
            if is_fresh and self._distance_remaining is not None and self._distance_remaining > 0:
                dist_str = f'{int(round(self._distance_remaining))} mét'
                speech = f'Tôi đang dẫn bạn đến {self._current_target_name}, còn khoảng {dist_str}.'
            else:
                speech = f'Tôi đang dẫn bạn đến {self._current_target_name}.'
        elif self.state == MissionState.PAUSED:
            speech = f'Tôi đang tạm dừng trên đường tới {self._current_target_name}.'
        elif self.state in (MissionState.ANNOUNCING, MissionState.STARTING, MissionState.RESOLVING):
            speech = f'Tôi đang chuẩn bị dẫn bạn tới {self._current_target_name}.'
        elif self.state in (MissionState.PAUSING, MissionState.CANCELING):
            speech = 'Hệ thống đang xử lý thay đổi lộ trình, vui lòng chờ.'
        else:
            speech = 'Tôi đang ở vị trí chờ, chưa có chuyến đi nào.'

        self.say(speech)
        self.publish_feedback('STATUS_REPORT', 'OK', speech)


def main(args=None):
    rclpy.init(args=args)
    node = NavCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
