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
from rclpy.clock import Clock, ClockType
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


class GoalAttemptRecord:
    """Bản ghi theo dõi vòng đời từng attempt gửi goal tới Nav2."""
    def __init__(self, attempt_id: str, mission_id: str, target_id: str = None, target_name: str = None):
        self.attempt_id = attempt_id
        self.mission_id = mission_id
        self.target_id = target_id
        self.target_name = target_name
        self.created_at = time.time()
        self.goal_handle = None
        self.nav2_uuid = None
        self.state = 'SENDING'  # SENDING, ACCEPTED, REJECTED, CANCEL_REQUESTED, CANCEL_ACCEPTED, CANCEL_REJECTED, TERMINAL_CONFIRMED, UNCONFIRMED_TERMINAL
        self.is_terminal_confirmed = False
        self.terminal_status = None
        self.cancel_on_accept = False


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
        self.declare_parameter('cancel_terminal_timeout_sec', 4.0)
        self.declare_parameter('tts_policy', 'flexible')  # 'strict' | 'flexible'
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

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
        self.cancel_terminal_timeout_sec = float(self.get_parameter('cancel_terminal_timeout_sec').value)
        self.tts_policy = str(self.get_parameter('tts_policy').value)

        with open(places_path, 'r', encoding='utf-8') as f:
            db = json.load(f)
            self.places = db.get('places', {})
            self.categories = db.get('categories', {})

        # Máy trạng thái & Danh tính duy nhất theo phiên
        self.state = MissionState.IDLE
        self._session_start_ms = int(time.time() * 1000)
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
        self._revoked_attempts = set()
        self._unresolved_attempts = set()
        self._unconfirmed_terminal = False
        self._attempts = {}  # Dict[str, GoalAttemptRecord]

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

        # Quản lý Timers gắn định danh attempt / operation
        self._acceptance_timeout_timer = None
        self._acceptance_timer_attempt = None
        self._cancel_response_timer = None
        self._cancel_response_timer_attempt = None
        self._cancel_terminal_timer = None
        self._cancel_terminal_timer_attempt = None
        self._tts_timeout_timer = None
        self._tts_timer_speech_id = None

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
        self._heartbeat_timer = self.create_timer(1.0, self.publish_state, clock=self._steady_clock)

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

    def has_unresolved_goals(self) -> bool:
        """Kiểm tra xem còn attempt nào chưa được xác nhận terminal hay không."""
        return len(self._unresolved_attempts) > 0 or any(not att.is_terminal_confirmed for att in self._attempts.values())

    def _get_or_create_attempt(self, attempt_id: str, mission_id: str = None) -> GoalAttemptRecord:
        if attempt_id not in self._attempts:
            self._attempts[attempt_id] = GoalAttemptRecord(
                attempt_id=attempt_id,
                mission_id=mission_id or self._current_mission_id,
                target_id=self._current_target_key,
                target_name=self._current_target_name
            )
            self._unresolved_attempts.add(attempt_id)
        return self._attempts[attempt_id]

    def _confirm_attempt_terminal(self, attempt_id: str, status: int):
        rec = self._attempts.get(attempt_id)
        if rec:
            rec.is_terminal_confirmed = True
            rec.terminal_status = status
            if rec.state != 'REJECTED':
                rec.state = 'TERMINAL_CONFIRMED'
        self._unresolved_attempts.discard(attempt_id)
        if not self.has_unresolved_goals():
            self._unconfirmed_terminal = False

    def _mark_attempt_unconfirmed(self, attempt_id: str, status: int = GoalStatus.STATUS_UNKNOWN):
        rec = self._get_or_create_attempt(attempt_id)
        if rec.is_terminal_confirmed:
            self.get_logger().info(
                f'[{attempt_id}] Attempt đã có terminal xác nhận (status={rec.terminal_status}), '
                f'bỏ qua đánh dấu unconfirmed từ callback muộn/lặp.'
            )
            return
        rec.is_terminal_confirmed = False
        rec.terminal_status = status
        rec.state = 'UNCONFIRMED_TERMINAL'
        self._unresolved_attempts.add(attempt_id)
        self._unconfirmed_terminal = True

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
            engine = ack.get('engine', 'log')
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
                self._stop_tts_timeout_timer(ack_speech_id)
                self._pending_speech_id = None
                # Không được để node mắc lại ANNOUNCING sau khi timer đã dừng
                if self._pending_replace_target or self._pending_replace_category:
                    self._execute_pending_replace()
                elif self.state == MissionState.PAUSING:
                    self._transition_to_paused()
                elif self.state == MissionState.CANCELING:
                    self._transition_to_cancelled()
                else:
                    self.state = MissionState.IDLE
                    self.publish_state()
                return

            if status == 'DROPPED_QUEUE_FULL':
                self.get_logger().warn(f'Thông báo [{ack_speech_id}] bị DROPPED_QUEUE_FULL.')
                self._stop_tts_timeout_timer(ack_speech_id)
                self._pending_speech_id = None
                if self.tts_policy == 'strict':
                    self.say('Hệ thống âm thanh bận, không thể phát thông báo.')
                    self.state = MissionState.IDLE
                    self.publish_state()
                    self.publish_feedback('FAILED', 'TTS_DROPPED_QUEUE_FULL', 'Hàng đợi âm thanh đầy')
                    return
                else:
                    self.publish_feedback('WARNING', 'TTS_DROPPED_QUEUE_FULL_FALLBACK', 'Hàng đợi âm thanh đầy, tiếp tục gửi goal')
                    self.state = MissionState.STARTING
                    self.publish_state()
                    self.send_nav_goal()
                    return

            is_error = status.startswith('ERROR_')
            is_playback_warning = (status == 'WARNING_NO_PLAYER' or status.startswith('WARNING_PLAYBACK_'))
            is_unplayed = (engine != 'log' and not audio_played)

            if is_error or is_playback_warning or (self.tts_policy == 'strict' and is_unplayed):
                self.get_logger().warn(f'Thông báo [{ack_speech_id}] gặp sự cố phát âm: status={status}, played={audio_played}, engine={engine}')
                self._stop_tts_timeout_timer(ack_speech_id)
                self._pending_speech_id = None
                if self.tts_policy == 'strict':
                    self.state = MissionState.IDLE
                    self.publish_state()
                    self.publish_feedback('FAILED', f'TTS_{status}', f'Lỗi hoặc không phát được âm thanh trong chế độ strict (status={status})')
                    return
                else:
                    self.publish_feedback('WARNING', f'TTS_{status}_FALLBACK', f'Sự cố âm thanh ({status}), tiếp tục gửi goal')
                    self.state = MissionState.STARTING
                    self.publish_state()
                    self.send_nav_goal()
                    return

            self.get_logger().info(f'Nhận được TTS ACK hợp lệ cho [{ack_speech_id}] (status={status}, played={audio_played}). Tiến hành gửi goal Nav2.')
            self._stop_tts_timeout_timer(ack_speech_id)
            self._pending_speech_id = None
            self.state = MissionState.STARTING
            self.publish_state()
            self.send_nav_goal()

    def _start_tts_timeout_timer(self, expected_speech_id: str):
        self._stop_tts_timeout_timer()
        self._tts_timer_speech_id = expected_speech_id
        self._tts_timeout_timer = self.create_timer(
            self.tts_ack_timeout,
            lambda: self._on_tts_ack_timeout(expected_speech_id),
            clock=self._steady_clock
        )

    def _stop_tts_timeout_timer(self, speech_id: str = None):
        if speech_id is not None and self._tts_timer_speech_id != speech_id:
            return
        if hasattr(self, '_tts_timeout_timer') and self._tts_timeout_timer:
            try:
                self._tts_timeout_timer.cancel()
                self.destroy_timer(self._tts_timeout_timer)
            except Exception:
                pass
            self._tts_timeout_timer = None
            self._tts_timer_speech_id = None

    def _cancel_tts_timeout_timer(self):
        self._stop_tts_timeout_timer()

    def _on_tts_ack_timeout(self, expected_speech_id: str):
        self._stop_tts_timeout_timer(expected_speech_id)
        if self.state != MissionState.ANNOUNCING or self._pending_speech_id != expected_speech_id:
            return

        if self.tts_policy == 'strict':
            self.get_logger().error(f"Timeout ({self.tts_ack_timeout}s) chờ tts_ack cho [{expected_speech_id}] ở chế độ strict. Hủy gửi goal!")
            self._pending_speech_id = None
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('FAILED', 'TTS_TIMEOUT_STRICT_ABORT', 'Không nhận được xác nhận âm thanh trong chế độ strict')
            return

        self.get_logger().warn(f"Timeout ({self.tts_ack_timeout}s) chờ tts_ack cho [{expected_speech_id}] ở chế độ flexible. Tiếp tục gửi goal fallback.")
        self._pending_speech_id = None
        self.publish_feedback('WARNING', 'TTS_TIMEOUT_FALLBACK', 'Tiếp tục gửi goal dù TTS không phản hồi')
        self.state = MissionState.STARTING
        self.publish_state()
        self.send_nav_goal()

    # ------------------------------------------------------------------
    # Timer cho Acceptance, Cancel Response và Cancel Terminal
    # ------------------------------------------------------------------
    def _start_acceptance_timer(self, attempt_id: str):
        self._stop_acceptance_timer()
        self._acceptance_timer_attempt = attempt_id
        self._acceptance_timeout_timer = self.create_timer(
            self.goal_acceptance_timeout_sec,
            lambda: self._on_acceptance_timeout(attempt_id),
            clock=self._steady_clock
        )

    def _stop_acceptance_timer(self, attempt_id: str = None):
        if attempt_id is not None and self._acceptance_timer_attempt != attempt_id:
            return
        if hasattr(self, '_acceptance_timeout_timer') and self._acceptance_timeout_timer:
            try:
                self._acceptance_timeout_timer.cancel()
                self.destroy_timer(self._acceptance_timeout_timer)
            except Exception:
                pass
            self._acceptance_timeout_timer = None
            self._acceptance_timer_attempt = None

    def _on_acceptance_timeout(self, attempt_id: str):
        self._stop_acceptance_timer(attempt_id)
        # Đánh dấu attempt này bị thu hồi để nếu server accept muộn thì lập tức hủy
        self._revoked_attempts.add(attempt_id)
        self._cancelled_attempts.add(attempt_id)

        if self.state in (MissionState.STARTING, MissionState.CANCELING) and attempt_id == self._current_goal_attempt_id:
            self.get_logger().error(f"[{attempt_id}] Timeout ({self.goal_acceptance_timeout_sec}s) chờ Nav2 chấp nhận goal.")
            self._current_goal_attempt_id = None
            if self.state == MissionState.CANCELING:
                self._transition_to_cancelled()
            else:
                speech = 'Hệ thống điều hướng không phản hồi yêu cầu di chuyển.'
                self.say(speech)
                self.state = MissionState.IDLE
                self.publish_state()
                self.publish_feedback('FAILED', 'ACCEPTANCE_TIMEOUT', speech)

    def _start_cancel_timer(self, attempt_id: str, purpose: str):
        self._stop_cancel_timer()
        self._cancel_response_timer_attempt = attempt_id
        self._cancel_response_timer = self.create_timer(
            self.cancel_timeout_sec,
            lambda: self._on_cancel_timeout(attempt_id, purpose),
            clock=self._steady_clock
        )

    def _stop_cancel_timer(self, attempt_id: str = None):
        if attempt_id is not None and self._cancel_response_timer_attempt != attempt_id:
            return
        if hasattr(self, '_cancel_response_timer') and self._cancel_response_timer:
            try:
                self._cancel_response_timer.cancel()
                self.destroy_timer(self._cancel_response_timer)
            except Exception:
                pass
            self._cancel_response_timer = None
            self._cancel_response_timer_attempt = None

    def _on_cancel_timeout(self, attempt_id: str, purpose: str):
        self._stop_cancel_timer(attempt_id)
        if attempt_id != self._current_goal_attempt_id:
            return
        self.get_logger().error(f"[{attempt_id}] Timeout ({self.cancel_timeout_sec}s) chờ phản hồi hủy goal (mục đích: {purpose}).")
        if self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self.state = MissionState.NAVIGATING
            speech = 'Không nhận được phản hồi hủy từ hệ thống điều hướng. Robot tiếp tục lộ trình.'
            self.say(speech)
            self.publish_feedback('CANCEL_TIMEOUT', 'TIMEOUT', speech)
            self.publish_state()

    def _start_cancel_terminal_timer(self, attempt_id: str, purpose: str):
        self._stop_cancel_terminal_timer()
        self._cancel_terminal_timer_attempt = attempt_id
        self._cancel_terminal_timer = self.create_timer(
            self.cancel_terminal_timeout_sec,
            lambda: self._on_cancel_terminal_timeout(attempt_id, purpose),
            clock=self._steady_clock
        )

    def _stop_cancel_terminal_timer(self, attempt_id: str = None):
        if attempt_id is not None and self._cancel_terminal_timer_attempt != attempt_id:
            return
        if hasattr(self, '_cancel_terminal_timer') and self._cancel_terminal_timer:
            try:
                self._cancel_terminal_timer.cancel()
                self.destroy_timer(self._cancel_terminal_timer)
            except Exception:
                pass
            self._cancel_terminal_timer = None
            self._cancel_terminal_timer_attempt = None

    def _on_cancel_terminal_timeout(self, attempt_id: str, purpose: str):
        self._stop_cancel_terminal_timer(attempt_id)
        if attempt_id != self._current_goal_attempt_id:
            return
        self.get_logger().error(f"[{attempt_id}] Timeout ({self.cancel_terminal_timeout_sec}s) chờ xác nhận kết thúc terminal sau khi hủy đã được chấp nhận.")
        if self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self._unconfirmed_terminal = True
            speech = 'Hệ thống điều hướng chưa xác nhận dừng hoàn toàn. Vui lòng chờ trước khi tiếp tục.'
            self.say(speech)
            self.publish_feedback('CANCEL_TERMINAL_TIMEOUT', 'TIMEOUT', speech)
            self.publish_state()

    # ------------------------------------------------------------------
    # Công bố trạng thái
    # ------------------------------------------------------------------
    def publish_state(self):
        rec = self._attempts.get(self._current_goal_attempt_id)
        nav2_uuid = rec.nav2_uuid if rec else None
        payload = {
            'schema_version': '1.0',
            'node_online': True,
            'mission_state': self.state,
            'mission_id': self._current_mission_id,
            'goal_attempt_id': self._current_goal_attempt_id,
            'nav2_uuid': nav2_uuid,
            'current_target_id': self._current_target_key,
            'current_target_name': self._current_target_name,
            'distance_remaining': self._distance_remaining,
            'eta_seconds': self._eta_seconds,
            'last_feedback_time': self._last_feedback_time,
            'unresolved_attempts_count': len(self._unresolved_attempts),
            'has_unresolved_goals': self.has_unresolved_goals(),
            'timestamp': time.time()
        }
        self.pub_nav_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_feedback(self, event: str, status: str, message: str = ''):
        rec = self._attempts.get(self._current_goal_attempt_id)
        nav2_uuid = rec.nav2_uuid if rec else None
        payload = {
            'schema_version': '1.0',
            'event': event,
            'status': status,
            'message': message,
            'mission_id': self._current_mission_id,
            'goal_attempt_id': self._current_goal_attempt_id,
            'nav2_uuid': nav2_uuid,
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

        # 1. Nếu không ở IDLE (đang NAVIGATING, STARTING, PAUSED, ANNOUNCING, RESOLVING, PAUSING, CANCELING):
        # Chuyển yêu cầu đích mới qua luồng replace!
        if self.state != MissionState.IDLE:
            if self.state == MissionState.CANCELING:
                self.say('Hệ thống đang hủy chuyến đi hiện tại, vui lòng chờ.')
                self.publish_feedback('REJECTED', 'CANCELING_ACTIVE')
                return
            self.get_logger().info(f'Nhận lệnh điều hướng khi state={self.state}. Chuyển sang luồng replace_target.')
            self.handle_replace_target(data)
            return

        # 2. Guard khi ở IDLE mà còn goal chưa giải quyết từ trước
        if self.has_unresolved_goals() or self._unconfirmed_terminal:
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING', speech)
            return

        # 3. Đích cụ thể
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
        if self.has_unresolved_goals() or self._unconfirmed_terminal:
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING', speech)
            return

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
            self.get_logger().warn('Bộ điều phối đường (planner) chưa sẵn sàng. Không thể chọn điểm tối ưu cho danh mục.')
            self._resolving_active = False
            self.state = MissionState.IDLE
            self.publish_state()
            speech = 'Bộ điều phối đường chưa sẵn sàng nên hiện tại tôi chưa thể tìm đường đến các điểm trong nhóm yêu cầu.'
            self.say(speech)
            self.publish_feedback('FAILED', 'PLANNER_UNAVAILABLE', speech)
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
            lambda: self._on_planner_candidate_timeout(res_id, cand_attempt_id, cand_key),
            clock=self._steady_clock
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
        self._current_mission_id = f"mis_{self._session_start_ms}_{self._mission_counter}"
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

        # Khởi động timer cho speech này
        self._start_tts_timeout_timer(self._pending_speech_id)

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
        if self.has_unresolved_goals():
            self.get_logger().error('Không thể gửi goal Nav2: còn attempt chưa xác minh.')
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING', speech)
            self.state = MissionState.IDLE
            self.publish_state()
            return

        if not self._current_target_pose:
            self.get_logger().error('Không có pose để gửi cho Nav2!')
            self.state = MissionState.IDLE
            self.publish_state()
            return

        self._goal_attempt_counter += 1
        attempt_id = f"att_{self._session_start_ms}_{self._goal_attempt_counter}"
        self._current_goal_attempt_id = attempt_id
        mission_id = self._current_mission_id
        rec = GoalAttemptRecord(attempt_id, mission_id, self._current_target_key, self._current_target_name)
        self._attempts[attempt_id] = rec
        self._unresolved_attempts.add(attempt_id)

        goal = NavigateToPose.Goal()
        goal.pose = self._current_target_pose

        self.get_logger().info(f'[{attempt_id}] Gửi goal tới Nav2 cho mission {mission_id} ({self._current_target_name})...')
        self._start_acceptance_timer(attempt_id)
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda f_msg: self.on_nav_feedback(f_msg.feedback, attempt_id)
        )
        self._pending_goal_future = future
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
        # Chỉ dừng acceptance timer của chính attempt này
        self._stop_acceptance_timer(attempt_id)
        if self._pending_goal_future == future:
            self._pending_goal_future = None

        rec = self._get_or_create_attempt(attempt_id, mission_id)
        is_revoked = (attempt_id in self._revoked_attempts or attempt_id in self._cancelled_attempts or rec.cancel_on_accept)
        is_outdated_mission = (mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi nhận goal handle: {e}')
            self._mark_attempt_unconfirmed(attempt_id, GoalStatus.STATUS_UNKNOWN)
            if not is_outdated_mission and not is_revoked:
                self.state = MissionState.IDLE
                speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
                self.say(speech)
                self.publish_feedback('FAILED', 'GOAL_RESPONSE_EXCEPTION', speech)
                self.publish_state()
            return

        # Goal bị từ chối bởi server lúc gửi goal -> Đây là bằng chứng xác nhận request bị từ chối
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{attempt_id}] Goal bị Nav2 từ chối.')
            rec.goal_handle = goal_handle
            rec.state = 'REJECTED'
            self._confirm_attempt_terminal(attempt_id, GoalStatus.STATUS_ABORTED)

            # Callback reject của attempt cũ (cùng mission hoặc khác mission):
            # Chỉ được cập nhật bản ghi của chính attempt đó, không đổi trạng thái chuyến hiện tại hoặc gọi pending replace!
            if mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id:
                self.get_logger().info(
                    f'[{attempt_id}] Callback reject của attempt cũ '
                    f'(attempt={attempt_id}, current_att={self._current_goal_attempt_id}, '
                    f'mission={mission_id}, current_mis={self._current_mission_id}), không tác động chuyến hiện tại.'
                )
                return

            if self._pending_replace_target or self._pending_replace_category:
                self._execute_pending_replace()
                return
            elif self.state == MissionState.PAUSING:
                self._transition_to_paused()
                return
            elif self.state == MissionState.CANCELING:
                self._transition_to_cancelled()
                return

            self.say('Tôi không thể di chuyển tới đó lúc này.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'GOAL_REJECTED')
            return

        # Trích xuất UUID Nav2 nếu có
        rec.goal_handle = goal_handle
        rec.state = 'ACCEPTED'
        if hasattr(goal_handle, 'goal_id') and hasattr(goal_handle.goal_id, 'uuid'):
            try:
                rec.nav2_uuid = bytes(goal_handle.goal_id.uuid).hex()
            except Exception:
                pass

        # Goal được chấp nhận nhưng đã bị thu hồi / cancel / timeout trước đó
        if is_outdated_mission or is_revoked:
            self.get_logger().info(f'[{attempt_id}] Thu hồi goal cũ được chấp nhận muộn (uuid={rec.nav2_uuid}).')
            rec.state = 'CANCEL_REQUESTED'
            cancel_fut = goal_handle.cancel_goal_async()
            cancel_fut.add_done_callback(
                lambda cf: self._on_delayed_cancel_confirmed(cf, attempt_id, goal_handle)
            )
            # Đồng thời theo dõi terminal result của goal thu hồi
            goal_handle.get_result_async().add_done_callback(
                lambda rf: self._on_delayed_cancel_terminal_result(rf, attempt_id, mission_id)
            )
            return

        # Goal được chấp nhận bình thường cho mission hiện tại
        self._active_goal_handle = goal_handle
        self.get_logger().info(f'[{attempt_id}] Goal được Nav2 chấp nhận (uuid={rec.nav2_uuid}). Bắt đầu di chuyển.')
        self.state = MissionState.NAVIGATING
        self.publish_state()
        self.publish_feedback('STARTED', 'GOAL_ACCEPTED')

        # Chờ kết quả điều hướng
        goal_handle.get_result_async().add_done_callback(
            lambda fut: self.on_goal_result(fut, attempt_id, mission_id)
        )

    def _on_delayed_cancel_confirmed(self, future, attempt_id: str, goal_handle=None):
        rec = self._attempts.get(attempt_id)
        try:
            res = future.result()
            return_code = getattr(res, 'return_code', 0)
            goals_canceling = getattr(res, 'goals_canceling', [])
            is_accepted = (return_code == 0 and len(goals_canceling) > 0)
            if not is_accepted:
                if rec:
                    rec.state = 'CANCEL_REJECTED'
                self.get_logger().warn(f'[{attempt_id}] Server từ chối hủy goal thu hồi muộn (code={return_code}).')
                self.publish_feedback('WARNING', 'REVOKED_GOAL_CANCEL_REJECTED', f'Goal cũ {attempt_id} từ chối hủy')
            else:
                if rec:
                    rec.state = 'CANCEL_ACCEPTED'
                self.get_logger().info(f'[{attempt_id}] Server chấp nhận hủy goal thu hồi muộn (return_code={return_code}).')
        except Exception as e:
            if rec:
                rec.state = 'CANCEL_ERROR'
            self.get_logger().error(f'[{attempt_id}] Lỗi hủy goal thu hồi muộn: {e}')

    def _on_delayed_cancel_terminal_result(self, future, attempt_id: str, mission_id: str):
        rec = self._get_or_create_attempt(attempt_id, mission_id)
        if rec.is_terminal_confirmed:
            self.get_logger().info(
                f'[{attempt_id}] Attempt đã có terminal xác nhận trước đó (status={rec.terminal_status}), '
                f'bỏ qua callback terminal muộn/lặp.'
            )
            return
        try:
            res = future.result()
            status = getattr(res, 'status', GoalStatus.STATUS_UNKNOWN)
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi lấy terminal result cho goal thu hồi muộn: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f'[{attempt_id}] Goal thu hồi muộn kết thúc với status: {status}')

        # 1. Chỉ giải phóng attempt khi có bằng chứng goal đã kết thúc rõ ràng
        is_terminal_confirmed = (status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED))
        if is_terminal_confirmed:
            self._confirm_attempt_terminal(attempt_id, status)
        else:
            self.get_logger().warn(
                f'[{attempt_id}] Chưa xác minh được terminal result (status={status}). Giữ attempt trong unresolved_attempts.'
            )
            self._mark_attempt_unconfirmed(attempt_id, status)

        # 2. Xác minh mission_id VÀ attempt_id trước khi thay đổi chuyến hiện tại hoặc thực thi pending replace
        if mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id:
            self.get_logger().info(
                f'[{attempt_id}] Callback của attempt cũ (attempt={attempt_id}, current_att={self._current_goal_attempt_id}, '
                f'mission={mission_id}, current_mis={self._current_mission_id}), không tác động chuyến hiện tại.'
            )
            return

        # 3. Phân biệt rõ trong CANCELING
        if self.state == MissionState.CANCELING:
            self._pending_replace_target = None
            self._pending_replace_category = None
            if status == GoalStatus.STATUS_CANCELED:
                self._transition_to_cancelled()
                return
            elif status == GoalStatus.STATUS_SUCCEEDED:
                speech = f'Chúng ta đã tới {self._current_target_name} trước khi lệnh hủy có hiệu lực. Chúc bạn chuyến đi vui vẻ!'
                self.say(speech)
                self.publish_feedback('SUCCEEDED', 'ARRIVED_BEFORE_CANCEL', speech)
                self.state = MissionState.IDLE
                self._current_target_key = None
                self._current_target_name = None
                self._current_target_pose = None
                self.publish_state()
                return
            elif status == GoalStatus.STATUS_ABORTED:
                speech = 'Tôi gặp trở ngại trên đường đi và không thể tiếp tục.'
                self.say(speech)
                self.publish_feedback('FAILED', 'ABORTED', speech)
                self.state = MissionState.IDLE
                self._current_target_key = None
                self._current_target_name = None
                self._current_target_pose = None
                self.publish_state()
                return
            else:
                speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
                self.say(speech)
                self.publish_feedback('FAILED', 'CANCEL_UNCONFIRMED_TERMINAL', speech)
                self.state = MissionState.IDLE
                self.publish_state()
                return

        # 4. Khi KHÔNG ở CANCELING:
        # Nếu goal kết thúc SUCCEEDED trước khi lệnh hủy tới nơi
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{attempt_id}] Goal thu hồi muộn đã hoàn thành SUCCEEDED trước khi dừng.')
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

        # Nếu goal kết thúc ABORTED
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'[{attempt_id}] Goal thu hồi muộn bị ABORTED.')
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

        # Nếu goal kết thúc CANCELED
        if status == GoalStatus.STATUS_CANCELED:
            if self._pending_replace_target or self._pending_replace_category:
                self._execute_pending_replace()
                return
            elif self.state == MissionState.PAUSING:
                self._transition_to_paused()
                return
            else:
                self._transition_to_cancelled()
                return

        # Trạng thái UNKNOWN hoặc exception khi không ở CANCELING
        self.get_logger().warn(f'[{attempt_id}] Goal thu hồi muộn kết thúc với trạng thái không xác định: {status}')
        speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
        self.say(speech)
        self.publish_feedback('FAILED', f'UNCONFIRMED_STATUS_{status}', speech)
        self.state = MissionState.IDLE
        self._pending_replace_target = None
        self._pending_replace_category = None
        self.publish_state()

    def on_goal_result(self, future, attempt_id: str, mission_id: str):
        self._stop_cancel_timer(attempt_id)
        self._stop_cancel_terminal_timer(attempt_id)

        # Bỏ qua nếu không phải attempt hiện tại của mission hiện tại.
        # Callback attempt cũ chỉ cập nhật bản ghi của đúng attempt, không đổi trạng thái chuyến hiện tại!
        # Tuyệt đối không xóa _unconfirmed_terminal của mission mới khi callback cũ về muộn!
        if attempt_id != self._current_goal_attempt_id or mission_id != self._current_mission_id:
            self.get_logger().warn(f'Bỏ qua goal result của attempt cũ: {attempt_id}')
            rec = self._attempts.get(attempt_id)
            if rec and rec.is_terminal_confirmed:
                return
            try:
                status = future.result().status
            except Exception:
                status = GoalStatus.STATUS_UNKNOWN
            if rec:
                rec.terminal_status = status
                if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
                    rec.is_terminal_confirmed = True
                    rec.state = 'TERMINAL_CONFIRMED'
                    self._unresolved_attempts.discard(attempt_id)
                else:
                    self._mark_attempt_unconfirmed(attempt_id, status)
            return

        rec = self._get_or_create_attempt(attempt_id, mission_id)
        if rec.is_terminal_confirmed:
            self.get_logger().info(
                f'[{attempt_id}] Attempt đã có terminal xác nhận trước đó (status={rec.terminal_status}), '
                f'bỏ qua callback terminal muộn/lặp.'
            )
            return

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'[{attempt_id}] Lỗi lấy kết quả goal: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f'[{attempt_id}] Goal kết thúc terminal với status: {status}')

        is_terminal_confirmed = (status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED))
        if is_terminal_confirmed:
            self._confirm_attempt_terminal(attempt_id, status)
        else:
            self.get_logger().warn(
                f'[{attempt_id}] Chưa xác minh được terminal result (status={status}). Giữ attempt trong unresolved_attempts.'
            )
            self._mark_attempt_unconfirmed(attempt_id, status)

        self._active_goal_handle = None

        if not is_terminal_confirmed:
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('FAILED', 'CANCEL_UNCONFIRMED_TERMINAL', speech)
            self.state = MissionState.IDLE
            self._pending_replace_target = None
            self._pending_replace_category = None
            self.publish_state()
            return

        # Phân biệt rõ trong CANCELING
        if self.state == MissionState.CANCELING:
            self._pending_replace_target = None
            self._pending_replace_category = None
            if status == GoalStatus.STATUS_CANCELED:
                self._transition_to_cancelled()
                return
            elif status == GoalStatus.STATUS_SUCCEEDED:
                speech = f'Chúng ta đã tới {self._current_target_name} trước khi lệnh hủy có hiệu lực. Chúc bạn chuyến đi vui vẻ!'
                self.say(speech)
                self.publish_feedback('SUCCEEDED', 'ARRIVED_BEFORE_CANCEL', speech)
                self.state = MissionState.IDLE
                self._current_target_key = None
                self._current_target_name = None
                self._current_target_pose = None
                self.publish_state()
                return
            elif status == GoalStatus.STATUS_ABORTED:
                speech = 'Tôi gặp trở ngại trên đường đi và không thể tiếp tục.'
                self.say(speech)
                self.publish_feedback('FAILED', 'ABORTED', speech)
                self.state = MissionState.IDLE
                self._current_target_key = None
                self._current_target_name = None
                self._current_target_pose = None
                self.publish_state()
                return
            else:
                speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
                self.say(speech)
                self.publish_feedback('FAILED', 'CANCEL_UNCONFIRMED_TERMINAL', speech)
                self.state = MissionState.IDLE
                self.publish_state()
                return

        # 1. TH Hủy thành công (STATUS_CANCELED) khi không ở CANCELING
        if status == GoalStatus.STATUS_CANCELED:
            if self._pending_replace_target or self._pending_replace_category:
                self._execute_pending_replace()
                return
            elif self.state == MissionState.PAUSING:
                self._transition_to_paused()
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

        # Các lỗi khác hoặc UNKNOWN
        speech = 'Chưa xác nhận được trạng thái kết thúc của goal điều hướng.'
        self.say(speech)
        self.publish_feedback('FAILED', f'UNCONFIRMED_STATUS_{status}', speech)
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

        elif self.state == MissionState.PAUSING:
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
            elif self.state == MissionState.CANCELING:
                self.state = MissionState.NAVIGATING
                speech = 'Không thể hủy chuyến đi lúc này do hệ thống điều hướng từ chối yêu cầu.'
                self.say(speech)
                self.publish_feedback('CANCEL_REJECTED', f'RETURN_CODE_{return_code}', speech)
                self.publish_state()
            return

        # Server chấp nhận cancel -> Khởi động terminal timer và chờ on_goal_result
        self._stop_cancel_timer(attempt_id)
        purpose = 'cancel' if self.state == MissionState.CANCELING else 'pause'
        self._start_cancel_terminal_timer(attempt_id, purpose=purpose)
        self.get_logger().info(f"[{attempt_id}] Server chấp nhận yêu cầu dừng ({purpose}). Đang chờ robot dừng hẳn...")

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
        # 1. Resume CHỈ được phép khi không bị unconfirmed terminal
        if self._unconfirmed_terminal:
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNCONFIRMED_TERMINAL_PENDING', speech)
            return

        # 2. Phải ở trạng thái PAUSED hoàn tất
        if self.state != MissionState.PAUSED or self._active_goal_handle is not None:
            self.say('Robot hiện không trong trạng thái tạm dừng hoàn tất.')
            return

        # 3. Không còn unresolved attempts nào khác
        if self.has_unresolved_goals():
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING', speech)
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
        self._pending_replace_target = None
        self._pending_replace_category = None
        self._pending_replace_req_id = None
        self._paused_resolution = None

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
                rec = self._attempts.get(self._current_goal_attempt_id)
                if rec:
                    rec.cancel_on_accept = True
            self.get_logger().info('Đã nhận lệnh hủy khi đang STARTING. Sẽ hủy ngay khi goal được chấp nhận.')

        elif self.state in (MissionState.ANNOUNCING, MissionState.RESOLVING, MissionState.PAUSED):
            self._cancel_tts_timeout_timer()
            self.cancel_tts(speech_id=self._pending_speech_id, mission_id=self._current_mission_id)
            self._pending_speech_id = None
            self._resolving_active = False
            self._cancel_planner_candidate_timer()
            self._paused_resolution = None
            for att_id, rec in self._attempts.items():
                if not rec.is_terminal_confirmed:
                    if rec.goal_handle:
                        try:
                            cfut = rec.goal_handle.cancel_goal_async()
                            cfut.add_done_callback(lambda f, a=att_id, gh=rec.goal_handle: self._on_delayed_cancel_confirmed(f, a, gh))
                            rec.goal_handle.get_result_async().add_done_callback(
                                lambda rf, a=att_id, m=rec.mission_id: self._on_delayed_cancel_terminal_result(rf, a, m)
                            )
                        except Exception:
                            pass
                    else:
                        rec.cancel_on_accept = True
                        self._cancelled_attempts.add(att_id)
            self._transition_to_cancelled()

        elif self.state == MissionState.CANCELING:
            self.say('Yêu cầu hủy đang được thực hiện.')

        else:
            if self.has_unresolved_goals() or self._unconfirmed_terminal:
                self.get_logger().warn('Nhận lệnh hủy ở IDLE khi còn attempt chưa xác minh. Thử hủy lại các goal cũ nếu còn khả năng.')
                for att_id, rec in self._attempts.items():
                    if not rec.is_terminal_confirmed:
                        if rec.goal_handle is not None:
                            try:
                                cfut = rec.goal_handle.cancel_goal_async()
                                cfut.add_done_callback(lambda f, a=att_id, gh=rec.goal_handle: self._on_delayed_cancel_confirmed(f, a, gh))
                                rec.goal_handle.get_result_async().add_done_callback(
                                    lambda rf, a=att_id, m=rec.mission_id: self._on_delayed_cancel_terminal_result(rf, a, m)
                                )
                            except Exception as e:
                                self.get_logger().error(f'Lỗi khi gửi lại cancel cho attempt {att_id}: {e}')
                        else:
                            rec.cancel_on_accept = True
                            self._cancelled_attempts.add(att_id)

                speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
                self.say(speech)
                self.publish_feedback('REJECTED', 'UNCONFIRMED_TERMINAL_PENDING', speech)
                self.publish_state()
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

        # Server chấp nhận cancel -> Khởi động terminal timer và chờ on_goal_result
        self._stop_cancel_timer(attempt_id)
        self._start_cancel_terminal_timer(attempt_id, purpose='cancel')
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

        # 1. Xác minh điểm đến mới trước (phản hồi rõ khi không hợp lệ, bảo toàn nhiệm vụ hiện tại)
        valid_key = self.validate_target(new_target_id)
        valid_cat = new_cat if (new_cat and new_cat in self.categories and self.get_valid_candidates(new_cat)) else None

        if not valid_key and not valid_cat:
            self.say('Điểm đến mới không hợp lệ. Tôi tiếp tục lộ trình hiện tại.')
            self.publish_feedback('REJECTED', 'INVALID_NEW_TARGET')
            return

        if getattr(self, '_unconfirmed_terminal', False):
            self.say('Hệ thống điều hướng chưa xác nhận trạng thái dừng của chuyến đi trước. Vui lòng chờ trong giây lát.')
            self.publish_feedback('REJECTED', 'UNCONFIRMED_TERMINAL_PENDING')
            return

        # 2. Nếu đang ở IDLE:
        if self.state == MissionState.IDLE:
            if len(self._unresolved_attempts) > 0:
                self.say('Hệ thống điều hướng đang xử lý chuyến đi trước, vui lòng chờ.')
                self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING')
                return
            if valid_key:
                self.start_specific_mission(valid_key, req_id)
            else:
                self.resolve_category_and_start(valid_cat, req_id)
            return

        # 3. Nếu đang PAUSED: Cập nhật đích mới và GIỮ NGUYÊN trạng thái PAUSED
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
        elif self.state in (MissionState.PAUSING, MissionState.CANCELING):
            self.get_logger().info('Đã ghi nhận pending replace trong lúc PAUSING/CANCELING. Sẽ thực hiện sau khi goal dừng.')

    def _execute_pending_replace(self):
        target_key = self._pending_replace_target
        category = self._pending_replace_category
        req_id = self._pending_replace_req_id
        self._pending_replace_target = None
        self._pending_replace_category = None
        self._pending_replace_req_id = None

        if self.has_unresolved_goals() or self._unconfirmed_terminal:
            self.get_logger().warn('Không thể thực thi replace: còn attempt chưa xác minh.')
            speech = 'Chưa xác nhận được chuyến đi cũ đã kết thúc.'
            self.say(speech)
            self.publish_feedback('REJECTED', 'UNRESOLVED_GOALS_PENDING', speech)
            return

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
            if len(self._unresolved_attempts) > 0:
                speech = 'Tôi đang ở vị trí chờ, nhưng hệ thống đang xác nhận trạng thái dừng của chuyến đi trước.'
                self.say(speech)
                self.publish_feedback('STATUS_REPORT', 'UNRESOLVED_GOALS_PENDING', speech)
                return
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
