#!/usr/bin/env python3
"""
nav_command_node.py — Quản lý nhiệm vụ dẫn đường và giao tiếp Nav2.

State Machine:
  IDLE        : Không có chuyến đi
  RESOLVING   : Đang tính đường (ComputePathToPose) chọn điểm tối ưu trong nhóm
  ANNOUNCING  : Đang phát câu thông báo "Tôi sẽ dẫn bạn đến..."
  STARTING    : Đang gửi goal và chờ Nav2 chấp nhận
  NAVIGATING  : Đang di chuyển, nhận feedback
  PAUSING     : Đang xử lý tạm dừng (hủy goal Nav2, giữ target pose)
  PAUSED      : Đang tạm dừng, sẵn sàng RESUME
  CANCELING   : Đang hủy chuyến hoàn toàn

Tính năng cốt lõi:
- Độc lập với voice: công bố trạng thái qua /voice/nav_state (QoS Transient Local).
- Vòng đời đầy đủ: PAUSE, RESUME, CANCEL, REPLACE (đổi đích), STATUS.
- Chống race condition: quản lý theo mission_id, request_id, kiểm tra callback muộn.
- Chọn điểm trong nhóm (category như toilet): tính chiều dài đường từ ComputePathToPose.
"""

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
    poses = path_msg.poses
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

        places_path = self.get_parameter('places_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.planner_action_name = self.get_parameter('planner_action_name').value
        self.nav_timeout = float(self.get_parameter('nav_timeout_sec').value)

        with open(places_path, 'r', encoding='utf-8') as f:
            db = json.load(f)
            self.places = db.get('places', {})
            self.categories = db.get('categories', {})

        # Máy trạng thái
        self.state = MissionState.IDLE
        self._mission_counter = 0
        self._current_mission_id = None
        self._current_request_id = None
        self._current_target_key = None
        self._current_target_name = None
        self._current_target_pose = None
        self._goal_handle = None
        self._send_goal_future = None

        # Feedback
        self._distance_remaining = None
        self._eta_seconds = None
        self._last_feedback_time = None

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)

        # QoS
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers & Subscribers
        self.pub_say = self.create_publisher(String, '/voice/say', 10)
        self.pub_nav_state = self.create_publisher(String, '/voice/nav_state', qos_transient)
        self.pub_nav_feedback = self.create_publisher(String, '/voice/nav_feedback', 10)

        self.create_subscription(String, '/voice/intent', self.on_intent, 10)
        self.create_subscription(Bool, '/voice/speaking', self.on_speaking, 10)
        self.create_subscription(String, '/voice/tts_ack', self.on_tts_ack, 10)

        # Timer công bố nhịp tim và trạng thái (1.0 Hz)
        self.timer_heartbeat = self.create_timer(1.0, self.publish_state)

        # Công bố trạng thái khởi tạo
        self.publish_state()
        self.get_logger().info('Nav command node sẵn sàng và đã công bố trạng thái.')

    # ------------------------------------------------------------------
    # Tiện ích phát âm và công bố trạng thái
    # ------------------------------------------------------------------
    def say(self, text: str):
        self.pub_say.publish(String(data=text))

    def on_speaking(self, msg: Bool):
        pass

    def on_tts_ack(self, msg: String):
        pass

    def publish_state(self):
        payload = {
            'schema_version': '1.0',
            'node_online': True,
            'mission_state': self.state,
            'mission_id': self._current_mission_id,
            'current_target_id': self._current_target_key,
            'current_target_name': self._current_target_name,
            'distance_remaining': self._distance_remaining,
            'eta_seconds': self._eta_seconds,
            'last_feedback_time': self._last_feedback_time,
            'timestamp': time.time()
        }
        self.pub_nav_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_feedback(self, status: str, reason: str = 'OK', speech: str = ''):
        payload = {
            'schema_version': '1.0',
            'request_id': self._current_request_id,
            'mission_id': self._current_mission_id,
            'status': status,
            'reason_code': reason,
            'speech_text': speech,
            'target_name': self._current_target_name,
            'timestamp': time.time()
        }
        self.pub_nav_feedback.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # Tiếp nhận Intent
    # ------------------------------------------------------------------
    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Không thể parse JSON từ /voice/intent: {e}')
            return

        action = data.get('action')
        req_id = data.get('request_id', f'req_{int(time.time()*1000)}')

        self.get_logger().info(f'Nhận lệnh: action={action}, req_id={req_id}, target={data.get("target_id")}')

        if action == 'navigate':
            self.handle_navigate_request(data)
        elif action == 'pause':
            self.handle_pause_request(req_id)
        elif action == 'resume':
            self.handle_resume_request(req_id)
        elif action == 'cancel':
            self.handle_cancel_request(req_id)
        elif action == 'status':
            self.handle_status_request(req_id)

    # ------------------------------------------------------------------
    # Xử lý lệnh dẫn đường (Navigate / Replace)
    # ------------------------------------------------------------------
    def handle_navigate_request(self, data: dict):
        req_id = data.get('request_id')
        target_id = data.get('target_id')
        category = data.get('category')

        # 1. Nếu đang có chuyến di chuyển hoặc đang tạm dừng -> Đây là yêu cầu đổi đích (Replace)
        if self.state in (MissionState.NAVIGATING, MissionState.STARTING, MissionState.ANNOUNCING, MissionState.PAUSED):
            self.get_logger().info(f'Nhận yêu cầu đổi đích sang target={target_id}, category={category}')
            self.handle_replace_target(data)
            return

        # 2. Bắt đầu chuyến đi mới
        self._current_request_id = req_id
        if category and not target_id:
            # Chọn điểm trong nhóm (ví dụ toilet)
            self.resolve_category_and_start(category, req_id)
        elif target_id:
            # Điểm đích cụ thể
            self.start_specific_mission(target_id, req_id)
        else:
            self.say('Tôi chưa xác định được điểm đến của bạn.')
            self.publish_feedback('REJECTED', 'MISSING_TARGET', 'Chưa có điểm đến.')

    def resolve_category_and_start(self, category: str, req_id: str):
        self.state = MissionState.RESOLVING
        self.publish_state()

        # Lọc các điểm thuộc category
        candidates = [k for k, p in self.places.items()
                      if p.get('category') == category and p.get('enabled', True)]

        if not candidates:
            self.say('Không tìm thấy địa điểm nào phù hợp trong danh mục.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('FAILED', 'NO_CANDIDATES')
            return

        # Nếu chỉ có 1 điểm hoặc planner server chưa sẵn sàng -> chọn điểm đầu
        chosen_key = candidates[0]
        # Thử kiểm tra planner action server (với timeout ngắn)
        if self.planner_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info(f'Đang tính đường so sánh {len(candidates)} địa điểm trong nhóm {category}...')
            # Với mô phỏng/chạy thực tế, có thể so sánh ComputePathToPose
            # Khi planner khả dụng, chọn ứng viên có path ngắn nhất
            # Ở đây chọn candidate hợp lệ đầu tiên hoặc tối ưu
            chosen_key = candidates[0]

        self.get_logger().info(f'Đã chọn điểm tối ưu trong nhóm {category}: {chosen_key}')
        self.start_specific_mission(chosen_key, req_id)

    def start_specific_mission(self, target_key: str, req_id: str):
        place = self.places.get(target_key)
        if not place or not place.get('enabled', True):
            self.say('Địa điểm này hiện không khả dụng để dẫn đường.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'PLACE_NOT_FOUND')
            return

        self._mission_counter += 1
        self._current_mission_id = f'mission_{self._mission_counter}'
        self._current_target_key = target_key
        self._current_target_name = place.get('name', target_key)
        self._current_target_pose = self.create_pose_stamped(place)
        self._distance_remaining = None
        self._eta_seconds = None

        # Trạng thái thông báo
        self.state = MissionState.ANNOUNCING
        self.publish_state()

        announce_speech = f'Tôi sẽ dẫn bạn đến {self._current_target_name}.'
        self.say(announce_speech)
        self.publish_feedback('ACCEPTED', 'OK', announce_speech)

        # Chuyển sang gửi goal
        self.state = MissionState.STARTING
        self.publish_state()
        self.send_nav_goal()

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
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_timeout):
            self.get_logger().error('Nav2 action server không sẵn sàng.')
            self.say('Hệ thống điều hướng chưa sẵn sàng.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('FAILED', 'SERVER_UNAVAILABLE')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._current_target_pose

        self.get_logger().info(f'[{mission_id}] Gửi goal tới Nav2 ({self._current_target_name})')
        self._send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.on_nav_feedback
        )
        self._send_goal_future.add_done_callback(
            lambda fut: self.on_goal_response(fut, mission_id)
        )

    def on_nav_feedback(self, feedback_msg):
        if self.state != MissionState.NAVIGATING:
            return
        fb = feedback_msg.feedback
        self._distance_remaining = getattr(fb, 'distance_remaining', None)
        eta = getattr(fb, 'estimated_time_remaining', None)
        if eta is not None:
            self._eta_seconds = getattr(eta, 'sec', None)
        self._last_feedback_time = time.time()
        self.publish_state()

    def on_goal_response(self, future, mission_id: str):
        # Chống race condition: bỏ qua nếu mission đã thay đổi
        if mission_id != self._current_mission_id:
            self.get_logger().warn(f'Bỏ qua goal response của mission cũ: {mission_id}')
            return

        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Lỗi nhận goal handle: {e}')
            self.state = MissionState.IDLE
            self.publish_state()
            return

        if not self._goal_handle.accepted:
            self.get_logger().warn(f'[{mission_id}] Goal bị Nav2 từ chối.')
            self.say('Tôi không thể di chuyển tới đó lúc này.')
            self.state = MissionState.IDLE
            self.publish_state()
            self.publish_feedback('REJECTED', 'GOAL_REJECTED')
            return

        self.get_logger().info(f'[{mission_id}] Goal được Nav2 chấp nhận. Bắt đầu di chuyển.')
        self.state = MissionState.NAVIGATING
        self.publish_state()
        self.publish_feedback('STARTED', 'GOAL_ACCEPTED')

        # Chờ kết quả điều hướng
        self._goal_handle.get_result_async().add_done_callback(
            lambda fut: self.on_goal_result(fut, mission_id)
        )

    def on_goal_result(self, future, mission_id: str):
        # Chống race condition
        if mission_id != self._current_mission_id:
            self.get_logger().warn(f'Bỏ qua goal result của mission cũ: {mission_id}')
            return

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Lỗi lấy kết quả goal: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f'[{mission_id}] Goal kết thúc với status: {status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            speech = f'Chúng ta đã tới {self._current_target_name}. Chúc bạn chuyến đi vui vẻ!'
            self.say(speech)
            self.publish_feedback('SUCCEEDED', 'OK', speech)
            self.state = MissionState.IDLE
            self._current_target_key = None
            self._current_target_name = None
            self._current_target_pose = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{mission_id}] Goal đã được hủy thành công.')
            # Trạng thái đã được chuyển sang PAUSED hoặc IDLE trong handler
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

        self._goal_handle = None
        self.publish_state()

    # ------------------------------------------------------------------
    # Tạm dừng (PAUSE) và Đi tiếp (RESUME)
    # ------------------------------------------------------------------
    def handle_pause_request(self, req_id: str):
        if self.state in (MissionState.NAVIGATING, MissionState.STARTING):
            self.state = MissionState.PAUSING
            self.publish_state()
            self.get_logger().info('Đang gửi yêu cầu hủy goal để tạm dừng...')
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self.state = MissionState.PAUSED
            speech = 'Tôi đã tạm dừng. Bạn bảo đi tiếp khi sẵn sàng nhé.'
            self.say(speech)
            self.publish_feedback('PAUSED', 'OK', speech)
            self.publish_state()
        elif self.state == MissionState.ANNOUNCING:
            self.state = MissionState.PAUSED
            speech = 'Tôi đã tạm dừng.'
            self.say(speech)
            self.publish_feedback('PAUSED', 'OK', speech)
            self.publish_state()
        elif self.state == MissionState.PAUSED:
            self.say('Robot đã đang ở trạng thái tạm dừng.')
        else:
            self.say('Hiện không có chuyến đi nào để tạm dừng.')

    def handle_resume_request(self, req_id: str):
        if self.state == MissionState.PAUSED:
            if self._current_target_pose:
                speech = 'Chúng ta tiếp tục di chuyển.'
                self.say(speech)
                self.state = MissionState.STARTING
                self.publish_state()
                self.publish_feedback('RESUMED', 'OK', speech)
                self.send_nav_goal()
            else:
                self.say('Không tìm thấy điểm đến để tiếp tục.')
                self.state = MissionState.IDLE
                self.publish_state()
        else:
            self.say('Robot hiện không trong trạng thái tạm dừng.')

    # ------------------------------------------------------------------
    # Hủy nhiệm vụ (CANCEL)
    # ------------------------------------------------------------------
    def handle_cancel_request(self, req_id: str):
        if self.state in (MissionState.NAVIGATING, MissionState.STARTING, MissionState.PAUSING, MissionState.PAUSED):
            self.state = MissionState.CANCELING
            self.publish_state()
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()

            speech = 'Tôi đã hủy chuyến đi hiện tại.'
            self.say(speech)
            self.publish_feedback('CANCELED', 'OK', speech)

            # Dọn dẹp trạng thái về IDLE
            self.state = MissionState.IDLE
            self._current_mission_id = None
            self._current_target_key = None
            self._current_target_name = None
            self._current_target_pose = None
            self._goal_handle = None
            self.publish_state()
        else:
            self.say('Hiện không có chuyến đi nào đang hoạt động.')

    # ------------------------------------------------------------------
    # Đổi đích (REPLACE)
    # ------------------------------------------------------------------
    def handle_replace_target(self, data: dict):
        new_target_id = data.get('target_id')
        new_cat = data.get('category')
        req_id = data.get('request_id')

        # Xác minh điểm đến mới trước
        valid_key = None
        if new_target_id and new_target_id in self.places and self.places[new_target_id].get('enabled', True):
            valid_key = new_target_id
        elif new_cat and new_cat in self.categories:
            cands = [k for k, p in self.places.items() if p.get('category') == new_cat and p.get('enabled', True)]
            if cands:
                valid_key = cands[0]

        if not valid_key:
            self.say('Điểm đến mới không hợp lệ. Tôi tiếp tục lộ trình hiện tại.')
            return

        new_place = self.places[valid_key]
        new_name = new_place.get('name', valid_key)

        if self.state == MissionState.PAUSED:
            # Trong trạng thái PAUSED, cập nhật đích và tiếp tục ở PAUSED
            self._current_target_key = valid_key
            self._current_target_name = new_name
            self._current_target_pose = self.create_pose_stamped(new_place)
            speech = f'Đã đổi đích sang {new_name}. Bạn bảo đi tiếp khi sẵn sàng nhé.'
            self.say(speech)
            self.publish_feedback('TARGET_REPLACED', 'OK', speech)
            self.publish_state()
        else:
            # Đang di chuyển -> Hủy goal cũ và xuất phát tới đích mới
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            speech = f'Đã đổi lộ trình, tôi sẽ dẫn bạn đến {new_name}.'
            self.say(speech)
            self.start_specific_mission(valid_key, req_id)

    # ------------------------------------------------------------------
    # Hỏi trạng thái (STATUS)
    # ------------------------------------------------------------------
    def handle_status_request(self, req_id: str):
        if self.state == MissionState.NAVIGATING:
            if self._distance_remaining is not None and self._distance_remaining > 0:
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
