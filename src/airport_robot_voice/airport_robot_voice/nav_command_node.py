#!/usr/bin/env python3
"""
nav_command_node.py — Quản lý nhiệm vụ dẫn đường và giao tiếp Nav2.

State Machine:
  IDLE        : Không có chuyến đi
  RESOLVING   : Đang tính đường (ComputePathToPose) chọn điểm tối ưu trong nhóm
  ANNOUNCING  : Đang phát câu thông báo "Tôi sẽ dẫn bạn đến..." và chờ tts_ack
  STARTING    : Đang gửi goal và chờ Nav2 chấp nhận
  NAVIGATING  : Đang di chuyển, nhận feedback
  PAUSING     : Đang xử lý tạm dừng (hủy goal Nav2, chờ xác nhận hủy)
  PAUSED      : Đang tạm dừng, sẵn sàng RESUME
  CANCELING   : Đang hủy chuyến hoàn toàn (chờ xác nhận hủy trước khi về IDLE)

Tính năng cốt lõi:
- Độc lập với voice: công bố trạng thái qua /voice/nav_state (QoS Transient Local).
- Vòng đời đầy đủ: PAUSE, RESUME, CANCEL, REPLACE (đổi đích), STATUS.
- Chống race condition & goal leak: tách bạch mission_id, request_id, goal_attempt_id.
- Hỗ trợ cancel khi goal đang chờ chấp nhận (pending goal future).
- Chọn điểm trong nhóm (category như toilet): tính chiều dài đường từ ComputePathToPose.
- Đồng bộ TTS: chỉ xuất phát sau khi nhận tts_ack của câu thông báo (kèm timeout fallback).
- Khử trùng lặp request_id bằng cache giới hạn.
- Kiểm tra độ tươi của feedback trước khi đọc khoảng cách/ETA.
"""

import collections
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus


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
    """Tính tổng chiều dài đoạn đường từ danh sách poses trong Path."""
    poses = getattr(path_msg, 'poses', [])
    if len(poses) < 2:
        return 0.0
    length = 0.0
    for i in range(len(poses) - 1):
        p1 = poses[i].pose.position
        p2 = poses[i + 1].pose.position
        length += math.hypot(p2.x - p1.x, p2.y - p1.y)
    return length


class NavCommandNode(Node):
    def __init__(self):
        super().__init__('nav_command_node')

        try:
            from ament_index_python.packages import get_package_share_directory
            share = os.path.join(get_package_share_directory('airport_robot_voice'), 'config')
        except Exception:
            share = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')

        self.declare_parameter('places_file', os.path.join(share, 'places.json'))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('nav_timeout_sec', 5.0)
        self.declare_parameter('planner_timeout_sec', 3.0)
        self.declare_parameter('tts_ack_timeout_sec', 4.0)
        self.declare_parameter('feedback_freshness_sec', 5.0)

        places_path = self.get_parameter('places_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.planner_action_name = self.get_parameter('planner_action_name').value
        self.nav_timeout = float(self.get_parameter('nav_timeout_sec').value)
        self.planner_timeout = float(self.get_parameter('planner_timeout_sec').value)
        self.tts_ack_timeout = float(self.get_parameter('tts_ack_timeout_sec').value)
        self.feedback_freshness_sec = float(self.get_parameter('feedback_freshness_sec').value)

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
        self._planner_eval = None
        self._planner_candidate_timer = None

        # Quản lý Announcing (TTS)
        self._pending_speech_id = None
        self._tts_timeout_timer = None

        # Quản lý Replace
        self._pending_replace_target = None
        self._pending_replace_req_id = None

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

    def on_speaking(self, msg: Bool):
        pass

    def on_tts_ack(self, msg: String):
        """Nhận phản hồi từ TTS Node. Nếu khớp speech_id đang chờ trong ANNOUNCING -> tiến hành gửi goal."""
        try:
            ack = json.loads(msg.data)
            ack_speech_id = ack.get('speech_id')
        except Exception:
            return

        if self.state == MissionState.ANNOUNCING and self._pending_speech_id and ack_speech_id == self._pending_speech_id:
            self.get_logger().info(f"Đã nhận tts_ack cho speech_id={ack_speech_id}. Tiến hành gửi Nav goal.")
            self._cancel_tts_timeout_timer()
            self._pending_speech_id = None
            self.state = MissionState.STARTING
            self.publish_state()
            self.send_nav_goal()

    def _cancel_tts_timeout_timer(self):
        if self._tts_timeout_timer:
            self._tts_timeout_timer.cancel()
            self.destroy_timer(self._tts_timeout_timer)
            self._tts_timeout_timer = None

    def _on_tts_ack_timeout(self, expected_speech_id: str):
        """Fallback: Nếu TTS node vắng mặt hoặc không phản hồi sau timeout, tự động tiến hành."""
        self._cancel_tts_timeout_timer()
        if self.state == MissionState.ANNOUNCING and self._pending_speech_id == expected_speech_id:
            self.get_logger().warn(f"TTS ack timeout ({self.tts_ack_timeout}s) cho speech_id={expected_speech_id}. Tự động gửi Nav goal.")
            self._pending_speech_id = None
            self.state = MissionState.STARTING
            self.publish_state()
            self.send_nav_goal()

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
            'mission_id': self._current_mission_id,
            'request_id': self._current_request_id,
            'event': event,
            'status': status,
            'message': message,
            'timestamp': time.time()
        }
        self.pub_feedback.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # Tiếp nhận Intent
    # ------------------------------------------------------------------
    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Lỗi giải mã JSON intent: {e}')
            return

        action = data.get('action')
        req_id = data.get('request_id')

        # Khử trùng lặp request_id (LRU cache)
        if req_id:
            if req_id in self._processed_requests:
                self.get_logger().warn(f'Bỏ qua request_id trùng lặp: {req_id}')
                return
            self._processed_requests[req_id] = time.time()
            if len(self._processed_requests) > 200:
                self._processed_requests.popitem(last=False)

        self._current_request_id = req_id

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
    # Xử lý lệnh dẫn đường (Navigate)
    # ------------------------------------------------------------------
    def handle_navigate_request(self, data: dict):
        req_id = data.get('request_id')
        target_id = data.get('target_id')
        category = data.get('category')

        # 1. Nếu đang có chuyến di chuyển hoặc đang tạm dừng -> Đây là yêu cầu đổi đích (Replace)
        if self.state in (MissionState.NAVIGATING, MissionState.STARTING, MissionState.ANNOUNCING,
                          MissionState.PAUSED, MissionState.RESOLVING):
            self.get_logger().info(f'Nhận yêu cầu đổi đích sang target={target_id}, category={category}')
            self.handle_replace_target(data)
            return

        # 2. Bắt đầu chuyến đi mới
        if category and not target_id:
            self.resolve_category_and_start(category, req_id)
        elif target_id:
            self.start_specific_mission(target_id, req_id)
        else:
            self.say('Tôi chưa xác định được điểm đến của bạn.')
            self.publish_feedback('REJECTED', 'MISSING_TARGET', 'Chưa có điểm đến.')

    # ------------------------------------------------------------------
    # GIAI ĐOẠN 1: Chọn đích bằng planner (ComputePathToPose)
    # ------------------------------------------------------------------
    def get_valid_candidates(self, category: str):
        candidates = []
        for key, p in self.places.items():
            if p.get('category') != category:
                continue
            if not p.get('enabled', True):
                continue
            p_frame = p.get('frame_id', self.frame_id)
            if p_frame != self.frame_id:
                self.get_logger().warn(f"Bỏ qua {key}: frame_id '{p_frame}' khác '{self.frame_id}'")
                continue
            try:
                x = float(p.get('x'))
                y = float(p.get('y'))
                if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                    continue
            except (ValueError, TypeError):
                continue
            candidates.append(key)
        return candidates

    def resolve_category_and_start(self, category: str, req_id: str):
        self.state = MissionState.RESOLVING
        self._resolving_active = True
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
            self._resolving_active = False
            self.start_specific_mission(chosen_key, req_id)
            return

        # Khởi tạo quá trình đánh giá bất đồng bộ qua ComputePathToPose
        self.get_logger().info(f'Bắt đầu truy vấn planner cho {len(candidates)} ứng viên trong nhóm {category}...')
        self._planner_eval = {
            'candidates': candidates,
            'current_idx': 0,
            'results': {},  # cand_key: path_length
            'req_id': req_id,
            'category': category
        }
        self._evaluate_next_planner_candidate()

    def _evaluate_next_planner_candidate(self):
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            self.get_logger().info('Tiến trình chọn đích bằng planner đã bị hủy.')
            return

        eval_data = self._planner_eval
        idx = eval_data['current_idx']
        candidates = eval_data['candidates']

        if idx >= len(candidates):
            # Đã đánh giá xong toàn bộ ứng viên
            self._resolving_active = False
            results = eval_data['results']
            if results:
                # Chọn ứng viên có chiều dài đường đi ngắn nhất
                best_key = min(results, key=results.get)
                best_length = results[best_key]
                self.get_logger().info(
                    f"Planner đã chọn {best_key} (khoảng cách {best_length:.2f}m) trong các ứng viên: {results}")
                self.start_specific_mission(best_key, eval_data['req_id'])
            else:
                self.get_logger().error('Tất cả các ứng viên đều không tìm được đường đi khả dụng.')
                self.say('Hệ thống không tìm thấy đường đi tới các điểm yêu cầu.')
                self.state = MissionState.IDLE
                self.publish_state()
                self.publish_feedback('FAILED', 'NO_VALID_PATH')
            return

        cand_key = candidates[idx]
        place = self.places[cand_key]
        goal = ComputePathToPose.Goal()
        goal.goal = self.create_pose_stamped(place)
        goal.use_start = False
        goal.planner_id = ''

        self.get_logger().info(f"Đang tính đường tới ứng viên [{idx + 1}/{len(candidates)}]: {cand_key}")
        self._cancel_planner_candidate_timer()
        self._planner_candidate_timer = self.create_timer(
            2.5, lambda: self._on_planner_candidate_timeout(cand_key))
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_planner_goal_response(fut, cand_key))

    def _on_planner_goal_response(self, future, cand_key: str):
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Lỗi gửi goal planner cho {cand_key}: {e}")
            self._advance_planner_evaluation()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"Planner từ chối tính đường cho {cand_key}")
            self._advance_planner_evaluation()
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(lambda rfut: self._on_planner_goal_result(rfut, cand_key))

    def _on_planner_goal_result(self, future, cand_key: str):
        self._cancel_planner_candidate_timer()
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            return

        try:
            wrapped_result = future.result()
            res = wrapped_result.result
            error_code = getattr(res, 'error_code', 0)
            path = getattr(res, 'path', None)
            poses = getattr(path, 'poses', []) if path else []

            if error_code == 0 and len(poses) >= 2:
                length = compute_path_length(path)
                self._planner_eval['results'][cand_key] = length
                self.get_logger().info(f"Ứng viên {cand_key}: đường đi hợp lệ = {length:.2f}m")
            else:
                self.get_logger().warn(
                    f"Ứng viên {cand_key}: không có đường hợp lệ (error_code={error_code}, poses={len(poses)})")
        except Exception as e:
            self.get_logger().error(f"Lỗi đọc kết quả path cho {cand_key}: {e}")

        self._advance_planner_evaluation()

    def _cancel_planner_candidate_timer(self):
        if hasattr(self, '_planner_candidate_timer') and self._planner_candidate_timer:
            try:
                self._planner_candidate_timer.cancel()
                self.destroy_timer(self._planner_candidate_timer)
            except Exception:
                pass
            self._planner_candidate_timer = None

    def _on_planner_candidate_timeout(self, cand_key: str):
        self._cancel_planner_candidate_timer()
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            return
        self.get_logger().warn(f"Timeout (2.5s) tính đường cho ứng viên {cand_key}. Bỏ qua ứng viên này.")
        self._advance_planner_evaluation()

    def _advance_planner_evaluation(self):
        self._cancel_planner_candidate_timer()
        if not self._resolving_active or self.state != MissionState.RESOLVING:
            return
        self._planner_eval['current_idx'] += 1
        self._evaluate_next_planner_candidate()

    # ------------------------------------------------------------------
    # GIAI ĐOẠN 2 & 3: Khởi động nhiệm vụ cụ thể & Đồng bộ TTS
    # ------------------------------------------------------------------
    def validate_target(self, target_key: str, category: str = None) -> str:
        """Xác minh điểm đến hợp lệ và trả về key, hoặc None nếu không hợp lệ."""
        if target_key and target_key in self.places and self.places[target_key].get('enabled', True):
            p = self.places[target_key]
            if p.get('frame_id', self.frame_id) == self.frame_id:
                return target_key
        if category and category in self.categories:
            cands = self.get_valid_candidates(category)
            if cands:
                return cands[0]
        return None

    def start_specific_mission(self, target_key: str, req_id: str):
        place = self.places.get(target_key)
        if not place or not place.get('enabled', True):
            self.say('Địa điểm này hiện không khả dụng để dẫn đường.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'PLACE_NOT_FOUND')
            return

        p_frame = place.get('frame_id', self.frame_id)
        if p_frame != self.frame_id:
            self.say('Địa điểm này nằm ngoài bản đồ hiện tại.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'INVALID_FRAME')
            return

        self._mission_counter += 1
        self._current_mission_id = f'mission_{self._mission_counter}'
        self._current_target_key = target_key
        self._current_target_name = place.get('name', target_key)
        self._current_target_pose = self.create_pose_stamped(place)
        self._distance_remaining = None
        self._eta_seconds = None
        self._last_feedback_time = None

        # Trạng thái thông báo (ANNOUNCING)
        self.state = MissionState.ANNOUNCING
        self.publish_state()

        announce_speech = f'Tôi sẽ dẫn bạn đến {self._current_target_name}.'
        self._speech_counter += 1
        speech_id = f"sp_{self._current_mission_id}_{self._speech_counter}"
        self._pending_speech_id = speech_id

        self.say(announce_speech, speech_id=speech_id, mission_id=self._current_mission_id)
        self.publish_feedback('ACCEPTED', 'OK', announce_speech)

        # Thiết lập timer timeout cho tts_ack
        self._cancel_tts_timeout_timer()
        self._tts_timeout_timer = self.create_timer(
            self.tts_ack_timeout,
            lambda: self._on_tts_ack_timeout(speech_id)
        )

    def create_pose_stamped(self, place: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = place.get('frame_id', self.frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(place.get('x', 0.0))
        pose.pose.position.y = float(place.get('y', 0.0))
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(float(place.get('yaw', 0.0)))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_nav_goal(self):
        mission_id = self._current_mission_id
        self._goal_attempt_counter += 1
        attempt_id = f"{mission_id}_g{self._goal_attempt_counter}"
        self._current_goal_attempt_id = attempt_id

        if not self.nav_client.wait_for_server(timeout_sec=self.nav_timeout):
            self.get_logger().error('Nav2 action server không sẵn sàng.')
            self.say('Hệ thống điều hướng chưa sẵn sàng.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('FAILED', 'SERVER_UNAVAILABLE')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._current_target_pose

        self.get_logger().info(f'[{attempt_id}] Gửi goal tới Nav2 ({self._current_target_name})')
        self._pending_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.on_nav_feedback(fb, attempt_id)
        )
        self._pending_goal_future.add_done_callback(
            lambda fut: self.on_goal_response(fut, attempt_id, mission_id)
        )

    # ------------------------------------------------------------------
    # Feedback & Result Action Lifecycle
    # ------------------------------------------------------------------
    def on_nav_feedback(self, feedback_msg, attempt_id: str):
        # Bỏ qua feedback của attempt cũ hoặc khi không còn ở NAVIGATING
        if attempt_id != self._current_goal_attempt_id:
            return
        if self.state != MissionState.NAVIGATING:
            return

        fb = feedback_msg.feedback
        self._distance_remaining = getattr(fb, 'distance_remaining', None)
        eta = getattr(fb, 'estimated_time_remaining', None)
        if eta is not None:
            self._eta_seconds = getattr(eta, 'sec', None)
        self._last_feedback_time = time.time()
        self.publish_state()

    def on_goal_response(self, future, attempt_id: str, mission_id: str):
        self._pending_goal_future = None

        # 1. Nếu attempt này đã bị đánh dấu cancel trước khi kịp accept (ví dụ pause/cancel/replace đến khi STARTING)
        is_pre_cancelled = (attempt_id in self._cancelled_attempts)

        # 2. Chống race condition: bỏ qua nếu mission đã thay đổi
        is_outdated_mission = (mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi nhận goal handle: {e}')
            if not is_outdated_mission:
                self.state = MissionState.IDLE
                self.publish_state()
            return

        # Goal bị từ chối
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{attempt_id}] Goal bị Nav2 từ chối.')
            if is_outdated_mission or is_pre_cancelled:
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
        self.get_logger().info(f'[{attempt_id}] Goal thu hồi muộn đã được xác nhận cancel.')
        if self._pending_replace_target:
            self._execute_pending_replace()
        elif self.state == MissionState.PAUSING and attempt_id in self._cancelled_attempts:
            self._transition_to_paused()
        elif self.state == MissionState.CANCELING and attempt_id in self._cancelled_attempts:
            self._transition_to_cancelled()

    def on_goal_result(self, future, attempt_id: str, mission_id: str):
        # Bỏ qua nếu không phải attempt hiện tại
        if attempt_id != self._current_goal_attempt_id or mission_id != self._current_mission_id:
            self.get_logger().warn(f'Bỏ qua goal result của attempt cũ: {attempt_id}')
            return

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi lấy kết quả goal: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f'[{attempt_id}] Goal kết thúc với status: {status}')
        self._active_goal_handle = None

        # Xử lý theo trạng thái hiện tại
        if self.state == MissionState.PAUSING:
            self._transition_to_paused()
            return

        if self.state == MissionState.CANCELING:
            self._transition_to_cancelled()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            speech = f'Chúng ta đã tới {self._current_target_name}. Chúc bạn chuyến đi vui vẻ!'
            self.say(speech)
            self.publish_feedback('SUCCEEDED', 'OK', speech)
            self.state = MissionState.IDLE
            self._current_target_key = None
            self._current_target_name = None
            self._current_target_pose = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{attempt_id}] Goal đã được hủy thành công.')
            # Nếu đang có replace pending
            if self._pending_replace_target:
                self._execute_pending_replace()
                return
        elif status == GoalStatus.STATUS_ABORTED:
            speech = 'Tôi gặp trở ngại trên đường đi và không thể tiếp tục.'
            self.say(speech)
            self.publish_feedback('FAILED', 'ABORTED', speech)
            self.state = MissionState.IDLE
        else:
            speech = 'Điều hướng không thành công.'
            self.say(speech)
            self.publish_feedback('FAILED', f'STATUS_{status}', speech)
            self.state = MissionState.IDLE

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
                fut = self._active_goal_handle.cancel_goal_async()
                attempt_id = self._current_goal_attempt_id
                fut.add_done_callback(lambda f: self._on_pause_cancel_response(f, attempt_id))
            else:
                self._transition_to_paused()

        elif self.state == MissionState.STARTING:
            # Goal đang chờ acceptance -> đánh dấu cancel
            self.state = MissionState.PAUSING
            self.publish_state()
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            self.get_logger().info('Đã nhận lệnh tạm dừng khi đang STARTING. Sẽ hủy ngay khi goal được chấp nhận.')

        elif self.state == MissionState.ANNOUNCING:
            self._cancel_tts_timeout_timer()
            self._pending_speech_id = None
            self._transition_to_paused()

        elif self.state == MissionState.RESOLVING:
            self._resolving_active = False
            self._transition_to_paused()

        elif self.state == MissionState.PAUSED:
            self.say('Robot đã đang ở trạng thái tạm dừng.')
        else:
            self.say('Hiện không có chuyến đi nào để tạm dừng.')

    def _on_pause_cancel_response(self, future, attempt_id: str):
        try:
            cancel_res = future.result()
            # Nếu server từ chối hủy (hiếm khi xảy ra)
            if cancel_res.return_code != 0:
                self.get_logger().warn(f"Server phản hồi cancel code: {cancel_res.return_code}")
        except Exception as e:
            self.get_logger().error(f"Lỗi phản hồi cancel: {e}")
        # Chuyển sang PAUSED khi đã gửi cancel
        if self.state == MissionState.PAUSING:
            self._transition_to_paused()

    def _transition_to_paused(self):
        self.state = MissionState.PAUSED
        self._active_goal_handle = None
        speech = 'Tôi đã tạm dừng. Bạn bảo đi tiếp khi sẵn sàng nhé.'
        self.say(speech)
        self.publish_feedback('PAUSED', 'OK', speech)
        self.publish_state()

    def handle_resume_request(self, req_id: str):
        # Resume CHỈ được phép khi PAUSED hoàn tất
        if self.state != MissionState.PAUSED:
            self.say('Robot hiện không trong trạng thái tạm dừng.')
            return

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
                fut = self._active_goal_handle.cancel_goal_async()
                attempt_id = self._current_goal_attempt_id
                fut.add_done_callback(lambda f: self._on_cancel_action_response(f, attempt_id))
            else:
                self._transition_to_cancelled()

        elif self.state == MissionState.STARTING:
            self.state = MissionState.CANCELING
            self.publish_state()
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            self.get_logger().info('Đã nhận lệnh hủy khi đang STARTING. Sẽ hủy ngay khi goal được chấp nhận.')

        elif self.state in (MissionState.ANNOUNCING, MissionState.RESOLVING, MissionState.PAUSED, MissionState.PAUSING):
            self._cancel_tts_timeout_timer()
            self._pending_speech_id = None
            self._resolving_active = False
            self._transition_to_cancelled()

        else:
            self.say('Hiện không có chuyến đi nào đang hoạt động.')

    def _on_cancel_action_response(self, future, attempt_id: str):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Lỗi phản hồi cancel: {e}")
        if self.state == MissionState.CANCELING:
            self._transition_to_cancelled()

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
        self.publish_state()

    # ------------------------------------------------------------------
    # Đổi đích (REPLACE)
    # ------------------------------------------------------------------
    def handle_replace_target(self, data: dict):
        new_target_id = data.get('target_id')
        new_cat = data.get('category')
        req_id = data.get('request_id')

        # 1. Xác minh điểm đến mới trước
        valid_key = self.validate_target(new_target_id, new_cat)
        if not valid_key:
            self.say('Điểm đến mới không hợp lệ. Tôi tiếp tục lộ trình hiện tại.')
            self.publish_feedback('REJECTED', 'INVALID_NEW_TARGET')
            return

        new_place = self.places[valid_key]
        new_name = new_place.get('name', valid_key)

        # 2. Nếu đang PAUSED: Cập nhật đích mới và GIỮ NGUYÊN trạng thái PAUSED
        if self.state == MissionState.PAUSED:
            self._current_target_key = valid_key
            self._current_target_name = new_name
            self._current_target_pose = self.create_pose_stamped(new_place)
            speech = f'Đã đổi đích sang {new_name}. Bạn bảo đi tiếp khi sẵn sàng nhé.'
            self.say(speech)
            self.publish_feedback('TARGET_REPLACED', 'OK', speech)
            self.publish_state()
            return

        # 3. Nếu đang ANNOUNCING hoặc RESOLVING: Hủy thông báo cũ và đổi ngay
        if self.state in (MissionState.ANNOUNCING, MissionState.RESOLVING):
            self._cancel_tts_timeout_timer()
            self._pending_speech_id = None
            self._resolving_active = False
            self.start_specific_mission(valid_key, req_id)
            return

        # 4. Nếu đang NAVIGATING hoặc STARTING: Hủy goal cũ, chờ hoàn tất rồi mới gửi goal mới
        self.get_logger().info(f'Đang hủy lộ trình cũ để đổi sang {new_name}...')
        self._pending_replace_target = valid_key
        self._pending_replace_req_id = req_id

        if self.state == MissionState.STARTING:
            if self._current_goal_attempt_id:
                self._cancelled_attempts.add(self._current_goal_attempt_id)
            # Khi goal cũ được accept/reject, _execute_pending_replace sẽ được gọi
            self.state = MissionState.PAUSING
            self.publish_state()
        elif self.state == MissionState.NAVIGATING:
            self.state = MissionState.PAUSING
            self.publish_state()
            if self._active_goal_handle:
                fut = self._active_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self._execute_pending_replace())
            else:
                self._execute_pending_replace()

    def _execute_pending_replace(self):
        if not self._pending_replace_target:
            return
        target_key = self._pending_replace_target
        req_id = self._pending_replace_req_id
        self._pending_replace_target = None
        self._pending_replace_req_id = None

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
