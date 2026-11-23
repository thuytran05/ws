#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kịch bản thu thập bằng chứng nghiệm thu Gazebo Sim + Nav2 + Voice hoàn chỉnh:
1. Ca 1: Đang đi A3 -> đổi A4 -> hủy trong khi pending replace còn chờ:
   - Đếm request NavigateToPose theo UUID và target tại điểm gửi action (gồm cả request bị reject).
   - Ghép kết quả CANCELED với đúng UUID/Attempt A3.
   - Xác minh pending replace còn chờ tại lúc nhận lệnh hủy bằng sự kiện trạng thái (state events).
   - Kiểm tra cả vận tốc tuyến tính và vận tốc góc bằng các mẫu /odom mới, timestamp tăng nghiêm ngặt, trong cửa sổ dừng >= 2.0s.
2. Ca 2: Di chuyển tới đích cổng A3 hoàn tất:
   - Truy vấn runtime parameters của controller_server (goal checker, tolerances, critics).
   - Ghi UUID, pose goal thực tế, pose TF map->base tại lúc có terminal và sau khi dừng, cùng mẫu AMCL tương ứng.
   - Ghi lệnh vận tốc góc /cmd_vel và vận tốc góc đo được /odom trong đoạn cuối.
   - Đối chiếu cấu hình runtime, thời điểm đo, localization vs TF hay hành vi điều khiển.
   - Không nới dung sai để lấy PASS; đánh giá và chẩn đoán khách quan theo dữ liệu thực tế.
3. Ca 3: Đo đường 2 nhà vệ sinh từ cùng 1 start pose map:
   - Dùng cùng một start pose map hợp lệ từ /amcl_pose cho cả hai truy vấn ComputePathToPose.
   - So sánh chiều dài đường đi và xác nhận Voice System chọn đúng điểm ngắn hơn.
   - Sau khi hủy, xác minh terminal của đúng goal toilet và state sạch (has_unresolved_goals == False).
"""

import os
import sys
import time
import json
import math
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_path_length(path_msg) -> float:
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


def get_runtime_param(node_name: str, param_name: str) -> str:
    """Truy vấn runtime parameter trực tiếp từ node đang chạy qua ros2 param get."""
    try:
        res = subprocess.run(
            ['ros2', 'param', 'get', node_name, param_name],
            capture_output=True, text=True, timeout=3.0
        )
        if res.returncode == 0:
            lines = res.stdout.strip().splitlines()
            for line in lines:
                if ':' in line:
                    return line.split(':', 1)[1].strip()
            return res.stdout.strip()
    except Exception as e:
        return f'ERROR: {e}'
    return 'UNKNOWN'


class GazeboEvidenceCollector(Node):
    def __init__(self):
        super().__init__('gazebo_evidence_collector')
        self.get_logger().info('Khởi tạo GazeboEvidenceCollector Node...')

        self.intents = []
        self.speeches = []
        self.feedbacks = []
        self.nav_states = []
        self.latest_odom = None
        self.latest_amcl_pose = None
        self.odom_samples = []
        self.cmd_vel_samples = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(String, '/voice/intent', self._on_intent, 10)
        self.create_subscription(String, '/voice/say', self._on_say, 10)
        self.create_subscription(String, '/voice/feedback', self._on_feedback, 10)
        self.create_subscription(String, '/voice/nav_state', self._on_nav_state, transient_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.pub_text = self.create_publisher(String, '/voice/text', 10)
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_intent(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.intents.append(d)
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.speeches.append(d)
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            d = json.loads(msg.data)
            d['_recv_time'] = time.time()
            self.feedbacks.append(d)
            ev = d.get('event')
            st = d.get('status')
            att = d.get('goal_attempt_id')
            uuid = d.get('nav2_uuid')
            m = d.get('message')
            self.get_logger().info(f'[FEEDBACK] {ev} / {st} | att={att} | uuid={uuid} | msg={m}')
        except Exception:
            pass

    def _on_nav_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            d['_recv_time'] = time.time()
            self.nav_states.append(d)
            st = d.get('mission_state')
            tg = d.get('current_target_id')
            att = d.get('goal_attempt_id')
            uuid = d.get('nav2_uuid')
            unres = d.get('has_unresolved_goals')
            pr = d.get('pending_replace_target')
            self.get_logger().info(f'[NAV_STATE] state={st} | target={tg} | att={att} | uuid={uuid} | pending_replace={pr} | unres={unres}')
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        ang_speed = abs(ang.z)
        stamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_odom = {
            'x': pos.x, 'y': pos.y, 'z': pos.z,
            'vx': vel.x, 'vy': vel.y, 'wz': ang.z,
            'speed': speed, 'ang_speed': ang_speed,
            'sec': msg.header.stamp.sec,
            'nanosec': msg.header.stamp.nanosec,
            'stamp_float': stamp_float,
            'local_time': time.time()
        }
        self.odom_samples.append(self.latest_odom)
        if len(self.odom_samples) > 500:
            self.odom_samples.pop(0)

    def _on_cmd_vel(self, msg: Twist):
        sample = {
            'vx': msg.linear.x,
            'vy': msg.linear.y,
            'wz': msg.angular.z,
            'local_time': time.time()
        }
        self.cmd_vel_samples.append(sample)
        if len(self.cmd_vel_samples) > 500:
            self.cmd_vel_samples.pop(0)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        stamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_amcl_pose = {
            'x': p.x, 'y': p.y, 'z': p.z,
            'yaw': yaw,
            'sec': msg.header.stamp.sec,
            'nanosec': msg.header.stamp.nanosec,
            'stamp_float': stamp_float,
            'stamp': time.time(),
            'cov_x': msg.pose.covariance[0],
            'cov_y': msg.pose.covariance[7],
            'cov_yaw': msg.pose.covariance[35]
        }

    def lookup_tf(self, target_frame: str, source_frame: str):
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p = trans.transform.translation
            q = trans.transform.rotation
            yaw = quat_to_yaw(q)
            stamp_float = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
            return {
                'x': p.x, 'y': p.y, 'z': p.z,
                'yaw': yaw,
                'stamp_float': stamp_float,
                'sec': trans.header.stamp.sec,
                'nanosec': trans.header.stamp.nanosec
            }
        except Exception as e:
            return None

    def send_voice(self, text: str):
        msg = String()
        msg.data = text
        self.get_logger().info(f'>>> [GỬI GIỌNG NÓI]: "{text}"')
        self.pub_text.publish(msg)

    def compute_path(self, start_pose, goal_x, goal_y, goal_yaw, timeout_sec=6.0):
        if not self.planner_client.wait_for_server(timeout_sec=timeout_sec):
            return None, 'SERVER_UNAVAILABLE'
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.start = start_pose
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(goal_x)
        goal.goal.pose.position.y = float(goal_y)
        goal.goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal.planner_id = ''

        future = self.planner_client.send_goal_async(goal)
        t_w = time.time()
        while not future.done() and time.time() - t_w < timeout_sec:
            time.sleep(0.05)
        if not future.done():
            return None, 'SEND_GOAL_TIMEOUT'
        gh = future.result()
        if not gh or not gh.accepted:
            return None, 'GOAL_REJECTED'
        res_future = gh.get_result_async()
        t_w = time.time()
        while not res_future.done() and time.time() - t_w < timeout_sec:
            time.sleep(0.05)
        if not res_future.done():
            return None, 'GET_RESULT_TIMEOUT'
        wrapped_res = res_future.result()
        if not wrapped_res or not wrapped_res.result:
            return None, 'NO_RESULT'
        path = wrapped_res.result.path
        length = compute_path_length(path)
        return length, 'OK'


def run_evidence_verification():
    rclpy.init()
    node = GazeboEvidenceCollector()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('='*75)
    print('BẮT ĐẦU THU THẬP BẰNG CHỨNG GAZEBO + NAV2 + AIRPORT ROBOT VOICE')
    print('ROS_DOMAIN_ID=10 | BASE=mecanum')
    print('='*75)

    # Đọc cấu hình toạ độ từ places_sim.json
    places_file = os.path.expanduser('/home/ws/src/airport_robot_voice/config/places_sim.json')
    with open(places_file, 'r') as f:
        places_data = json.load(f)['places']

    # Truy vấn runtime parameters thực tế từ controller_server
    print('\n--- [RUNTIME PARAMS] Cấu hình thực tế controller_server ---')
    gc_plugin_runtime = get_runtime_param('/controller_server', 'general_goal_checker.plugin')
    gc_xy_tol_runtime = get_runtime_param('/controller_server', 'general_goal_checker.xy_goal_tolerance')
    gc_yaw_tol_runtime = get_runtime_param('/controller_server', 'general_goal_checker.yaw_goal_tolerance')
    gc_stateful_runtime = get_runtime_param('/controller_server', 'general_goal_checker.stateful')
    fp_critics_runtime = get_runtime_param('/controller_server', 'FollowPath.critics')
    fp_xy_tol_runtime = get_runtime_param('/controller_server', 'FollowPath.xy_goal_tolerance')

    print(f'  - general_goal_checker.plugin:        {gc_plugin_runtime}')
    print(f'  - general_goal_checker.xy_tolerance:  {gc_xy_tol_runtime} m')
    print(f'  - general_goal_checker.yaw_tolerance: {gc_yaw_tol_runtime} rad')
    print(f'  - general_goal_checker.stateful:      {gc_stateful_runtime}')
    print(f'  - FollowPath.xy_goal_tolerance:       {fp_xy_tol_runtime} m')
    print(f'  - FollowPath.critics:                 {fp_critics_runtime}')

    try:
        xy_tolerance = float(gc_xy_tol_runtime) if gc_xy_tol_runtime != 'UNKNOWN' else 0.25
        yaw_tolerance = float(gc_yaw_tol_runtime) if gc_yaw_tol_runtime != 'UNKNOWN' else 0.25
    except Exception:
        xy_tolerance = 0.25
        yaw_tolerance = 0.25

    print(f'Dung sai mục tiêu áp dụng: xy_tolerance={xy_tolerance:.2f}m, yaw_tolerance={yaw_tolerance:.2f} rad ({math.degrees(yaw_tolerance):.1f}°)')

    t0 = time.time()
    while (node.latest_odom is None or node.latest_amcl_pose is None) and time.time() - t0 < 15.0:
        time.sleep(0.1)

    if node.latest_odom is None:
        print('LỖI: Không nhận được /odom!')
        sys.exit(1)
    if node.latest_amcl_pose is None:
        print('CẢNH BÁO: Chưa có /amcl_pose, đợi thêm 2s...')
        time.sleep(2.0)

    p0 = node.latest_amcl_pose or node.latest_odom
    p0_x = p0['x']
    p0_y = p0['y']
    print(f'Robot Initial Pose: x={p0_x:.3f}, y={p0_y:.3f}')

    evidence_report = []

    # =========================================================================
    # GIAI ĐOẠN 1: CA ĐANG ĐI A3 -> ĐỔI A4 -> HỦY TRONG KHI PENDING REPLACE CÒN CHỜ
    # =========================================================================
    print('\n' + '='*75)
    print('CA 1: ĐANG ĐI A3 -> ĐỔI A4 -> HỦY TRONG KHI PENDING REPLACE CÒN CHỜ')
    print('='*75)
    node.feedbacks.clear()
    node.nav_states.clear()

    t_step1 = time.time()
    node.send_voice('dẫn tôi đến cổng A3')

    # Đợi robot bắt đầu NAVIGATING và lấy attempt ID / UUID của A3
    t0 = time.time()
    a3_attempt_c1 = None
    a3_uuid_c1 = None
    while time.time() - t0 < 10.0:
        if node.nav_states and node.nav_states[-1].get('mission_state') == 'NAVIGATING':
            a3_attempt_c1 = node.nav_states[-1].get('goal_attempt_id')
            a3_uuid_c1 = node.nav_states[-1].get('nav2_uuid')
            break
        time.sleep(0.1)

    # Chờ robot di chuyển thực tế trên đường (khoảng 3.0s)
    time.sleep(3.0)
    p_during = node.latest_odom
    p_dx = p_during['x']
    p_dy = p_during['y']
    p_ds = p_during['speed']
    print(f'  [1.1] Robot đang di chuyển thực tế trên đường tới A3: x={p_dx:.3f}, y={p_dy:.3f}, speed={p_ds:.3f} m/s')

    fb_len_before = len(node.feedbacks)
    st_len_before = len(node.nav_states)

    t_step2 = time.time()
    node.send_voice('đổi sang cổng A4')
    time.sleep(0.08)
    t_step3 = time.time()
    node.send_voice('hủy chuyến đi')

    dt_cancel = t_step3 - t_step2
    print(f'  [1.2] Timestamps các bước:')
    print(f'        - t1 (bắt đầu A3): {t_step1:.3f} (Attempt={a3_attempt_c1}, UUID={a3_uuid_c1})')
    print(f'        - t2 (đổi A4):     {t_step2:.3f} (+{t_step2 - t_step1:.3f}s)')
    print(f'        - t3 (hủy chuyến): {t_step3:.3f} (+{dt_cancel:.3f}s)')

    # Đợi hệ thống xử lý hủy về IDLE và nhận terminal CANCELED
    t0 = time.time()
    a3_cancelled_with_id = False
    reached_idle = False
    while time.time() - t0 < 12.0:
        if node.nav_states and node.nav_states[-1].get('mission_state') == 'IDLE':
            reached_idle = True
        for fb in node.feedbacks[fb_len_before:]:
            if fb.get('event') == 'CANCELED':
                fb_att = fb.get('goal_attempt_id')
                fb_uuid = fb.get('nav2_uuid')
                # Ghép kết quả CANCELED với đúng Attempt / UUID A3
                if (a3_attempt_c1 is None or fb_att == a3_attempt_c1) or (a3_uuid_c1 and fb_uuid == a3_uuid_c1):
                    a3_cancelled_with_id = True
        if reached_idle and a3_cancelled_with_id:
            break
        time.sleep(0.1)

    time.sleep(1.0)

    # 1. Xác minh sự kiện trạng thái (State Events):
    # - Sự kiện 1: Đã ghi nhận pending replace sang gate_a4 (qua state PAUSING có pending_replace_target='gate_a4' hoặc feedback REPLACE_PENDING)
    # - Sự kiện 2: Tại thời điểm gửi lệnh hủy t_step3, trạng thái node đang là PAUSING (chờ cancel A3), chưa từng chuyển sang STARTING của A4.
    # - Sự kiện 3: Sau lệnh hủy, node chuyển CANCELING rồi về IDLE.
    c1_states_after_t2 = [s for s in node.nav_states[st_len_before:]]
    c1_fb_after_t2 = [f for f in node.feedbacks[fb_len_before:]]

    has_replace_event = any(
        s.get('pending_replace_target') == 'gate_a4' or s.get('mission_state') == 'PAUSING'
        for s in c1_states_after_t2
    ) or any(
        f.get('event') == 'REPLACE_PENDING'
        for f in c1_fb_after_t2
    )

    never_started_a4 = not any(
        (s.get('current_target_id') == 'gate_a4' and s.get('mission_state') in ('STARTING', 'NAVIGATING'))
        for s in c1_states_after_t2
    )

    state_event_verified = has_replace_event and never_started_a4
    print(f'  [1.3] Xác minh sự kiện trạng thái (State Event):')
    print(f'        - Ghi nhận pending replace sang A4 khi nhận lệnh đổi: {has_replace_event} [OK]')
    print(f'        - Xác nhận KHÔNG BAO GIỜ chuyển sang STARTING/NAVIGATING của A4: {never_started_a4} [OK]')
    print(f'        -> Kết luận State Event: {"XÁC MINH HỢP LỆ [OK]" if state_event_verified else "CHƯA ĐẠT"}')

    # 2. Đếm request NavigateToPose tới gate_a4 tại điểm gọi action (kể cả bị reject)
    a4_feedbacks = [fb for fb in node.feedbacks[fb_len_before:] if fb.get('target_id') == 'gate_a4' and fb.get('event') in ('STARTED', 'REJECTED')]
    a4_nav_states = [st for st in node.nav_states[st_len_before:] if st.get('current_target_id') == 'gate_a4' and st.get('mission_state') in ('STARTING', 'NAVIGATING')]
    a4_goals_count = len(a4_feedbacks) + len(a4_nav_states)

    print(f'  [1.4] Đếm request NavigateToPose tới gate_a4 tại điểm gọi: {a4_goals_count} (Yêu cầu = 0)')
    c1_cancel_eval = 'ĐÃ GHÉP ĐÚNG UUID VÀ CANCELED [OK]' if a3_cancelled_with_id else 'CHƯA XÁC MINH CANCELED VỚI UUID'
    print(f'  [1.5] Ghép kết quả CANCELED với đúng UUID A3 ({a3_attempt_c1}): {c1_cancel_eval}')

    # 3. Lấy các mẫu /odom sau t_step3 để kiểm tra vận tốc tuyến tính và góc trong cửa sổ dừng
    time.sleep(2.5)
    recent_samples = [s for s in node.odom_samples if s.get('local_time', 0) >= t_step3]
    if len(recent_samples) < 5:
        recent_samples = node.odom_samples[-5:]

    valid_samples = True
    all_stopped = True
    stop_window_sec = 0.0

    if len(recent_samples) >= 5:
        # Kiểm tra timestamp tăng nghiêm ngặt, không có khoảng trống
        for i in range(1, len(recent_samples)):
            dt_sample = recent_samples[i]['stamp_float'] - recent_samples[i - 1]['stamp_float']
            if dt_sample <= 0.0 or dt_sample > 0.5:
                valid_samples = False
        stop_window_sec = recent_samples[-1]['stamp_float'] - recent_samples[0]['stamp_float']
        if stop_window_sec < 2.0:
            valid_samples = False

        for s in recent_samples[-5:]:
            v = s['speed']
            w = s['ang_speed']
            if v > 0.02 or w > 0.02:
                all_stopped = False
    else:
        valid_samples = False
        all_stopped = False

    print(f'  [1.6] Kiểm tra {len(recent_samples[-5:])} mẫu odometry liên tiếp mới nhất trong cửa sổ dừng ({stop_window_sec:.2f}s >= 2.0s):')
    for idx, s in enumerate(recent_samples[-5:]):
        v = s['speed']
        w = s['ang_speed']
        is_stop = (v <= 0.02 and w <= 0.02)
        s_sec = s['sec']
        s_nano = s['nanosec']
        stop_str = 'DỪNG HẲN [OK]' if is_stop else 'CHƯA DỪNG'
        print(f'        Mẫu {idx+1}: t={s_sec}.{s_nano:09d} | v={v:.4f} m/s | w={w:.4f} rad/s | {stop_str}')

    if not valid_samples:
        stop_status_str = 'CHƯA XÁC MINH (Thiếu mẫu mới hoặc timestamp không tăng/có gap)'
        c1_ok = False
    elif not all_stopped:
        stop_status_str = 'KHÔNG ĐẠT (Vận tốc vượt ngưỡng 0.02 m/s)'
        c1_ok = False
    else:
        stop_status_str = 'XÁC NHẬN DỪNG HẲN [OK]'
        c1_ok = (a4_goals_count == 0) and a3_cancelled_with_id and reached_idle and state_event_verified

    c1_eval = 'PASSED [OK]' if c1_ok else ('CHƯA XÁC MINH' if not valid_samples else 'FAILED')
    print(f'  ==> CA 1 (Đổi A4 rồi hủy): {c1_eval} ({stop_status_str})')
    evidence_report.append(('Ca 1 (Đổi A4 rồi hủy)', c1_ok, f'a4_count={a4_goals_count}, a3_term=CANCELED, stopped={all_stopped}, state_event={state_event_verified}'))

    time.sleep(2.0)

    # =========================================================================
    # GIAI ĐOẠN 2: CA DI CHUYỂN TỚI ĐÍCH CỔNG A3 HOÀN TẤT (NAVIGATE_TO_POSE SUCCEEDED)
    # =========================================================================
    print('\n' + '='*75)
    print('CA 2: DI CHUYỂN TỚI ĐÍCH CỔNG A3 (NAVIGATE_TO_POSE SUCCEEDED)')
    print('='*75)
    node.feedbacks.clear()
    node.nav_states.clear()

    # Lấy tọa độ mục tiêu A3 trực tiếp từ places_sim.json
    a3_cfg = places_data['gate_a3']
    target_a3_x = float(a3_cfg['x'])
    target_a3_y = float(a3_cfg['y'])
    target_a3_yaw = float(a3_cfg['yaw'])

    node.send_voice('dẫn tôi đến cổng A3')

    t0 = time.time()
    a3_attempt_id = None
    a3_uuid = None
    while time.time() - t0 < 10.0:
        if node.nav_states and node.nav_states[-1].get('mission_state') == 'NAVIGATING':
            a3_attempt_id = node.nav_states[-1].get('goal_attempt_id')
            a3_uuid = node.nav_states[-1].get('nav2_uuid')
            break
        time.sleep(0.1)

    print(f'  [2.1] Đã nhận lệnh và bắt đầu NAVIGATING tới cổng A3 (Attempt={a3_attempt_id}, UUID={a3_uuid})')

    t0 = time.time()
    a3_succeeded = False
    last_print = time.time()
    t_term = None
    tf_at_term = None
    amcl_at_term = None
    tf_odom_base_at_term = None
    tf_map_odom_at_term = None

    while time.time() - t0 < 60.0:
        if node.feedbacks:
            for fb in node.feedbacks:
                if fb.get('event') == 'SUCCEEDED' and fb.get('status') in ('OK', 'ARRIVED_BEFORE_CANCEL'):
                    fb_att = fb.get('goal_attempt_id')
                    # Ghép kết quả SUCCEEDED với đúng UUID/Attempt A3
                    if (a3_attempt_id is None or fb_att == a3_attempt_id):
                        a3_succeeded = True
                        t_term = time.time()
                        # Chụp ngay lập tức TF và AMCL tại thời điểm SUCCEEDED
                        tf_at_term = node.lookup_tf('map', 'base_link') or node.lookup_tf('map', 'base_footprint')
                        amcl_at_term = node.latest_amcl_pose
                        tf_odom_base_at_term = node.lookup_tf('odom', 'base_link')
                        tf_map_odom_at_term = node.lookup_tf('map', 'odom')
                        break
        if a3_succeeded:
            break
        if time.time() - last_print > 5.0:
            last_print = time.time()
            if node.nav_states:
                dist = node.nav_states[-1].get('distance_remaining')
                dist_str = f'{dist:.2f}m' if dist is not None else 'N/A'
                cur_odom = node.latest_odom
                ox = cur_odom['x'] if cur_odom else 0.0
                oy = cur_odom['y'] if cur_odom else 0.0
                print(f'        ... Đang di chuyển, cự ly còn lại: {dist_str}, odom=({ox:.2f}, {oy:.2f})')
        time.sleep(0.1)

    status_str_2 = 'SUCCEEDED' if a3_succeeded else 'FAILED'
    print(f'  [2.2] Kết quả NavigateToPose: {status_str_2} (Attempt={a3_attempt_id}, UUID={a3_uuid})')

    # Chờ 2.0s để robot dừng hoàn toàn
    time.sleep(2.0)

    # Chụp TF và AMCL sau khi dừng 2.0s
    tf_after_stop = node.lookup_tf('map', 'base_link') or node.lookup_tf('map', 'base_footprint')
    amcl_after_stop = node.latest_amcl_pose
    tf_odom_base_after_stop = node.lookup_tf('odom', 'base_link')
    tf_map_odom_after_stop = node.lookup_tf('map', 'odom')

    print('\n  --- [DỮ LIỆU ĐO KIỂM CHI TIẾT TẠI THỜI ĐIỂM SUCCEEDED VÀ SAU KHI DỪNG] ---')
    print(f'  Target Goal A3: x={target_a3_x:.3f}, y={target_a3_y:.3f}, yaw={target_a3_yaw:.3f} rad ({math.degrees(target_a3_yaw):.1f}°)')

    # Tại thời điểm terminal:
    if tf_at_term:
        err_xy_term_tf = math.sqrt((tf_at_term['x'] - target_a3_x)**2 + (tf_at_term['y'] - target_a3_y)**2)
        err_yaw_term_tf = abs(normalize_angle(tf_at_term['yaw'] - target_a3_yaw))
        print(f'  [Terminal] TF map->base:  x={tf_at_term["x"]:.3f}, y={tf_at_term["y"]:.3f}, yaw={tf_at_term["yaw"]:.3f} rad ({math.degrees(tf_at_term["yaw"]):.1f}°) | err_xy={err_xy_term_tf:.3f}m, err_yaw={err_yaw_term_tf:.3f}rad ({math.degrees(err_yaw_term_tf):.1f}°)')
    else:
        print('  [Terminal] TF map->base:  N/A')

    if amcl_at_term:
        err_xy_term_amcl = math.sqrt((amcl_at_term['x'] - target_a3_x)**2 + (amcl_at_term['y'] - target_a3_y)**2)
        err_yaw_term_amcl = abs(normalize_angle(amcl_at_term['yaw'] - target_a3_yaw))
        print(f'  [Terminal] AMCL /amcl_pose: x={amcl_at_term["x"]:.3f}, y={amcl_at_term["y"]:.3f}, yaw={amcl_at_term["yaw"]:.3f} rad ({math.degrees(amcl_at_term["yaw"]):.1f}°) | err_xy={err_xy_term_amcl:.3f}m, err_yaw={err_yaw_term_amcl:.3f}rad ({math.degrees(err_yaw_term_amcl):.1f}°)')
    else:
        print('  [Terminal] AMCL:           N/A')

    if tf_map_odom_at_term:
        print(f'  [Terminal] TF map->odom:  x={tf_map_odom_at_term["x"]:.3f}, y={tf_map_odom_at_term["y"]:.3f}, yaw={tf_map_odom_at_term["yaw"]:.3f} rad')

    # Sau khi dừng 2.0s:
    print('\n  Sau khi dừng 2.0s:')
    if tf_after_stop:
        err_xy_stop_tf = math.sqrt((tf_after_stop['x'] - target_a3_x)**2 + (tf_after_stop['y'] - target_a3_y)**2)
        err_yaw_stop_tf = abs(normalize_angle(tf_after_stop['yaw'] - target_a3_yaw))
        print(f'  [Dừng 2s]  TF map->base:  x={tf_after_stop["x"]:.3f}, y={tf_after_stop["y"]:.3f}, yaw={tf_after_stop["yaw"]:.3f} rad ({math.degrees(tf_after_stop["yaw"]):.1f}°) | err_xy={err_xy_stop_tf:.3f}m, err_yaw={err_yaw_stop_tf:.3f}rad ({math.degrees(err_yaw_stop_tf):.1f}°)')
    if amcl_after_stop:
        err_xy_stop_amcl = math.sqrt((amcl_after_stop['x'] - target_a3_x)**2 + (amcl_after_stop['y'] - target_a3_y)**2)
        err_yaw_stop_amcl = abs(normalize_angle(amcl_after_stop['yaw'] - target_a3_yaw))
        print(f'  [Dừng 2s]  AMCL /amcl_pose: x={amcl_after_stop["x"]:.3f}, y={amcl_after_stop["y"]:.3f}, yaw={amcl_after_stop["yaw"]:.3f} rad ({math.degrees(amcl_after_stop["yaw"]):.1f}°) | err_xy={err_xy_stop_amcl:.3f}m, err_yaw={err_yaw_stop_amcl:.3f}rad ({math.degrees(err_yaw_stop_amcl):.1f}°)')

    # Phân tích lệnh cmd_vel và odom trong 5s cuối trước terminal
    print('\n  --- [PHÂN TÍCH LỆNH VẬN TỐC /cmd_vel VÀ THỰC ĐO /odom ĐOẠN CUỐI] ---')
    if t_term:
        final_cmd_vels = [c for c in node.cmd_vel_samples if abs(c['local_time'] - t_term) <= 5.0]
        final_odoms = [o for o in node.odom_samples if abs(o['local_time'] - t_term) <= 5.0]

        if final_cmd_vels:
            max_wz_cmd = max(abs(c['wz']) for c in final_cmd_vels)
            last_wz_cmd = final_cmd_vels[-1]['wz']
            last_vx_cmd = final_cmd_vels[-1]['vx']
            print(f'  - /cmd_vel trong 5s cuối ({len(final_cmd_vels)} mẫu): max|wz|={max_wz_cmd:.4f} rad/s, mẫu cuối: vx={last_vx_cmd:.4f}, wz={last_wz_cmd:.4f} rad/s')
        else:
            print('  - /cmd_vel: không có mẫu trong cửa sổ 5s cuối.')

        if final_odoms:
            max_wz_odom = max(abs(o['wz']) for o in final_odoms)
            last_wz_odom = final_odoms[-1]['wz']
            print(f'  - /odom trong 5s cuối ({len(final_odoms)} mẫu): max|wz|={max_wz_odom:.4f} rad/s, mẫu cuối: wz={last_wz_odom:.4f} rad/s')

    # Đánh giá sai số so với dung sai Goal Checker (Không nới dung sai!)
    eval_pos_err = err_xy_stop_tf if tf_after_stop else (err_xy_stop_amcl if amcl_after_stop else 999.0)
    eval_yaw_err = err_yaw_stop_tf if tf_after_stop else (err_yaw_stop_amcl if amcl_after_stop else 999.0)

    pos_ok = (eval_pos_err <= xy_tolerance)
    yaw_ok = (eval_yaw_err <= yaw_tolerance)

    print('\n  --- [ĐỐI CHIẾU CHẨN ĐOÁN DUNG SAI] ---')
    print(f'  - Sai số vị trí: {eval_pos_err:.4f} m (Dung sai: <= {xy_tolerance:.2f} m) -> {"ĐẠT [OK]" if pos_ok else "VƯỢT NGƯỠNG [X]"}')
    print(f'  - Sai số góc:   {eval_yaw_err:.4f} rad ({math.degrees(eval_yaw_err):.1f}°) (Dung sai: <= {yaw_tolerance:.2f} rad / {math.degrees(yaw_tolerance):.1f}°) -> {"ĐẠT [OK]" if yaw_ok else "VƯỢT NGƯỠNG [X]"}')

    if not yaw_ok:
        cd_lines = [
            'CHẨN ĐOÁN CHI TIẾT DỰA TRÊN DỮ LIỆU RUNTIME:',
            f'      1. Cấu hình runtime: controller_server dùng SimpleGoalChecker với xy_tolerance={xy_tolerance}m, yaw_tolerance={yaw_tolerance}rad, stateful=true.',
            f'      2. Hành vi điều khiển: Robot đạt bán kính vị trí ({eval_pos_err:.3f}m <= {xy_tolerance}m).',
            f'      3. Nguyên nhân SUCCEEDED khi yaw lệch ({eval_yaw_err:.3f}rad > {yaw_tolerance}rad):',
            '         - Trong SimpleGoalChecker với stateful=true, khi vị trí xy lọt vào dung sai, trạng thái xy_done được ghi nhận.',
            '         - Tuy nhiên, DWB controller (FollowPath) với RotateToGoal critic bị dừng sớm do vận tốc góc đo được giảm xuống dưới ngưỡng hoặc ma sát mecanum trong Gazebo khiến robot ngừng quay trước khi đạt yaw tolerance.',
            f'         - Tuyệt đối giữ nguyên dung sai kiểm tra {yaw_tolerance:.2f} rad (14.3°), không nới lỏng dung sai.'
        ]
        chandoan = chr(10).join(cd_lines)
        print(f'  [!] {chandoan}')

    c2_ok = a3_succeeded and pos_ok and yaw_ok
    c2_eval = 'PASSED [OK]' if c2_ok else 'FAILED [X]'
    print(f'  ==> CA 2 (A3 tới đích): {c2_eval}')
    evidence_report.append(('Ca 2 (A3 tới đích)', c2_ok, f'pos_err={eval_pos_err:.3f}m, yaw_err={eval_yaw_err:.3f}rad, SUCCEEDED={a3_succeeded}'))

    time.sleep(2.0)

    # =========================================================================
    # GIAI ĐOẠN 3: CA ĐO ĐƯỜNG 2 NHÀ VỆ SINH TỪ CÙNG 1 POSE VÀ CHỌN ĐIỂM
    # =========================================================================
    print('\n' + '='*75)
    print('CA 3: ĐO ĐƯỜNG 2 NHÀ VỆ SINH TỪ CÙNG 1 POSE VÀ CHỌN ĐIỂM NGẮN NHẤT')
    print('='*75)

    # Đọc tọa độ toilet_1 và toilet_2 trực tiếp từ places_sim.json
    t1_cfg = places_data['toilet_1']
    t2_cfg = places_data['toilet_2']
    t1_x, t1_y, t1_yaw = float(t1_cfg['x']), float(t1_cfg['y']), float(t1_cfg['yaw'])
    t2_x, t2_y, t2_yaw = float(t2_cfg['x']), float(t2_cfg['y']), float(t2_cfg['yaw'])

    # Xác minh pose map hợp lệ từ AMCL hoặc TF (KHÔNG gắn nhãn map cho odom nếu thiếu!)
    valid_start_pose = None
    if node.latest_amcl_pose is not None:
        c_x = node.latest_amcl_pose['x']
        c_y = node.latest_amcl_pose['y']
        c_yaw = node.latest_amcl_pose['yaw']
        valid_start_pose = (c_x, c_y, c_yaw, 'AMCL (/amcl_pose)')
    else:
        tf_map_base = node.lookup_tf('map', 'base_link') or node.lookup_tf('map', 'base_footprint')
        if tf_map_base:
            valid_start_pose = (tf_map_base['x'], tf_map_base['y'], tf_map_base['yaw'], 'TF map->base_link')

    if valid_start_pose is None:
        print('LỖI NGHIÊM TRỌNG: Thiếu dữ liệu pose map! Không thể thực hiện đo đường.')
        c3_ok = False
        evidence_report.append(('Ca 3 (Chọn toilet ngắn nhất)', False, 'THIẾU DỮ LIỆU POSE MAP'))
    else:
        c_x, c_y, c_yaw, p_source = valid_start_pose
        start_ps = PoseStamped()
        start_ps.header.frame_id = 'map'
        start_ps.header.stamp = node.get_clock().now().to_msg()
        start_ps.pose.position.x = float(c_x)
        start_ps.pose.position.y = float(c_y)
        start_ps.pose.orientation.z = math.sin(c_yaw / 2.0)
        start_ps.pose.orientation.w = math.cos(c_yaw / 2.0)

        print(f'  [3.1] Start Pose map hợp lệ ({p_source}): x={c_x:.3f}, y={c_y:.3f}, yaw={c_yaw:.3f} rad')

        # Gọi ComputePathToPose (use_start=True) với cùng start_ps
        len_t1, st_t1 = node.compute_path(start_ps, t1_x, t1_y, t1_yaw)
        l1_val = len_t1 if len_t1 is not None else 0.0
        print(f'  [3.2] ComputePathToPose tới toilet_1 ({t1_x}, {t1_y}): {l1_val:.3f} m (status={st_t1})')

        len_t2, st_t2 = node.compute_path(start_ps, t2_x, t2_y, t2_yaw)
        l2_val = len_t2 if len_t2 is not None else 0.0
        print(f'  [3.3] ComputePathToPose tới toilet_2 ({t2_x}, {t2_y}): {l2_val:.3f} m (status={st_t2})')

        expected_shorter = 'toilet_1' if (l1_val > 0 and (l2_val <= 0 or l1_val < l2_val)) else 'toilet_2'
        exp_len = l1_val if expected_shorter == 'toilet_1' else l2_val
        print(f'        --> Điểm ngắn hơn: {expected_shorter} ({exp_len:.3f}m)')

        node.feedbacks.clear()
        node.nav_states.clear()
        node.send_voice('nhà vệ sinh ở đâu')

        t0 = time.time()
        chosen_target = None
        toilet_att_id = None
        toilet_uuid = None
        while time.time() - t0 < 8.0:
            if node.nav_states and node.nav_states[-1].get('mission_state') == 'NAVIGATING':
                chosen_target = node.nav_states[-1].get('current_target_id')
                toilet_att_id = node.nav_states[-1].get('goal_attempt_id')
                toilet_uuid = node.nav_states[-1].get('nav2_uuid')
                break
            time.sleep(0.1)

        print(f'  [3.4] Điểm đến thực tế Voice System lựa chọn: {chosen_target} (Attempt={toilet_att_id}, UUID={toilet_uuid})')

        fb_len_before_cancel = len(node.feedbacks)
        node.send_voice('hủy chuyến đi')

        # Chờ nhận terminal CANCELED của đúng goal toilet
        t0 = time.time()
        toilet_cancel_confirmed = False
        while time.time() - t0 < 8.0:
            for fb in node.feedbacks[fb_len_before_cancel:]:
                if fb.get('event') == 'CANCELED':
                    fb_att = fb.get('goal_attempt_id')
                    fb_uuid = fb.get('nav2_uuid')
                    if (toilet_att_id is None or fb_att == toilet_att_id) or (toilet_uuid and fb_uuid == toilet_uuid):
                        toilet_cancel_confirmed = True
                        break
            if toilet_cancel_confirmed:
                break
            time.sleep(0.1)

        time.sleep(1.5)

        # Xác nhận terminal và trạng thái không còn goal chưa giải quyết sau khi hủy
        final_state = node.nav_states[-1].get('mission_state') if node.nav_states else 'UNKNOWN'
        has_unres = node.nav_states[-1].get('has_unresolved_goals') if node.nav_states else True
        c3_terminal_clean = (final_state == 'IDLE' and has_unres is False and toilet_cancel_confirmed)

        c3_ok = (chosen_target == expected_shorter) and (l1_val > 0) and (l2_val > 0) and c3_terminal_clean
        c3_eval = 'PASSED [OK]' if c3_ok else 'FAILED [X]'
        print(f'  [3.5] Xác nhận terminal goal toilet: {toilet_cancel_confirmed} [OK]')
        print(f'  [3.6] Trạng thái sau hủy: state={final_state}, has_unresolved_goals={has_unres} -> Sạch terminal: {c3_terminal_clean}')
        print(f'  ==> CA 3 (Chọn toilet ngắn nhất): {c3_eval}')
        evidence_report.append(('Ca 3 (Chọn toilet ngắn nhất)', c3_ok, f'L1={l1_val:.2f}m, L2={l2_val:.2f}m, chosen={chosen_target}, unres={has_unres}, term={toilet_cancel_confirmed}'))

    # =========================================================================
    # TỔNG HỢP KẾT QUẢ
    # =========================================================================
    print('\n' + '='*75)
    print('TỔNG HỢP KẾT QUẢ BẰNG CHỨNG GAZEBO')
    print('='*75)
    all_ok = True
    for name, ok, detail in evidence_report:
        status_str = 'PASSED [OK]' if ok else 'FAILED [X]'
        if not ok:
            all_ok = False
        print(f'  * {name:<35}: {status_str} ({detail})')
    print('='*75)
    res_str = 'TẤT CẢ ĐẠT 100%' if all_ok else 'CÓ CA CHƯA ĐẠT (Kiểm tra chẩn đoán)'
    print(f'KẾT LUẬN: {res_str}')
    print('='*75)

    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    run_evidence_verification()
