#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kịch bản nghiệm thu Smoke Test Phase 9: Tích hợp Voice Thật + Gazebo Escort + Nav2:
- Voice thật (nav_command_node với enable_escort:=true) là bên DUY NHẤT sở hữu Nav2 Action Client.
- Tracking Manager (escort_tracking_manager) là bên DUY NHẤT cấp session_id và quản lý khóa khách.
- Velocity Guard và Velocity Watchdog giám sát an toàn và ngắt vận tốc độc lập.

Bốn Ca Kiểm Chứng Nghiêm Ngặt:
1. Ca 1: Không dispatch trước khóa (chặn khi locked=False).
2. Ca 2: Phiên hợp lệ dẫn đường được (locked -> NAVIGATION_STARTED -> Navigating -> robot di chuyển vật lý).
3. Ca 3: Mất khách làm xe dừng và hủy đúng goal (stop -> ABORT -> ACK -> CANCELED -> clean IDLE -> dừng hẳn >= 2.0s).
4. Ca 4: Đến đích trả SUCCEEDED và Tracking về IDLE (SUCCEEDED -> ARRIVED -> clean IDLE).
"""

import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Phase9SmokeTestDriver(Node):
    """Driver kiểm thử và giám sát tích hợp Voice + Tracking trên đồ thị ROS 2 thật."""

    def __init__(self):
        super().__init__(
            "phase9_smoke_test_driver",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self.pub_obs = self.create_publisher(String, "/tracking/observation", reliable_qos)
        self.pub_intent = self.create_publisher(String, "/voice/intent", 10)
        self.pub_tts_ack = self.create_publisher(String, "/voice/tts_ack", 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_test_scan = self.create_publisher(LaserScan, "/tracking/scan_test", 10)

        # Subscribers
        self.sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.sub_guarded_cmd = self.create_subscription(Twist, "/tracking/cmd_vel_guarded", self._on_guarded_cmd, 10)
        self.sub_watchdog_cmd = self.create_subscription(Twist, "/tracking/cmd_vel_watchdog", self._on_watchdog_cmd, 10)

        self.sub_session_state = self.create_subscription(String, "/tracking/session_state", self._on_session_state, transient_qos)
        self.sub_safety_request = self.create_subscription(String, "/tracking/safety_request", self._on_safety_request, reliable_qos)
        self.sub_nav_status = self.create_subscription(String, "/voice/nav_status", self._on_nav_status, transient_qos)
        self.sub_nav_state = self.create_subscription(String, "/voice/nav_state", self._on_nav_state, transient_qos)
        self.sub_feedback = self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.sub_say = self.create_subscription(String, "/voice/say", self._on_say, 10)

        # History Buffers
        self.latest_clock: Optional[Clock] = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_history: List[Tuple[float, float, float, float, float]] = []  # (t, vx, wz, x, y)
        self.cmd_history: List[Tuple[float, float, float]] = []          # (t, vx, wz)
        self.guarded_history: List[Tuple[float, float, float]] = []

        self.session_states: List[dict] = []
        self.safety_requests: List[dict] = []
        self.nav_statuses: List[dict] = []
        self.nav_states: List[dict] = []
        self.feedbacks: List[dict] = []
        self.speeches: List[dict] = []

        # Background Scan Stream (20 Hz) for Collision Monitor
        self.active_scan_ranges = [10.0] * 361
        self.scan_timer = self.create_timer(0.05, self._publish_scan_loop)

        # Observation generation state (20 Hz)
        self._obs_timer_running = False
        self._obs_track_id = 1
        self._obs_dist_m = 1.6
        self._obs_hand_raised = False
        self._obs_near_edge = False
        self._last_obs_stamp: Tuple[int, int] = (0, 0)
        self._obs_timer = self.create_timer(0.05, self._obs_tick)

    def _on_clock(self, msg: Clock):
        self.latest_clock = msg

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        now = time.monotonic()
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_history.append((now, vx, wz, x, y))
        if len(self.odom_history) > 3000:
            self.odom_history.pop(0)

    def _on_cmd_vel(self, msg: Twist):
        now = time.monotonic()
        self.cmd_history.append((now, msg.linear.x, msg.angular.z))
        if len(self.cmd_history) > 1500:
            self.cmd_history.pop(0)

    def _on_guarded_cmd(self, msg: Twist):
        now = time.monotonic()
        self.guarded_history.append((now, msg.linear.x, msg.angular.z))
        if len(self.guarded_history) > 1500:
            self.guarded_history.pop(0)

    def _on_watchdog_cmd(self, msg: Twist):
        pass

    def _on_session_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.session_states.append(d)
        except Exception:
            pass

    def _on_safety_request(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.safety_requests.append(d)
        except Exception:
            pass

    def _on_nav_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.nav_statuses.append(d)
        except Exception:
            pass

    def _on_nav_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.nav_states.append(d)
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.feedbacks.append(d)
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.speeches.append(d)
            sid = d.get("speech_id")
            if sid:
                ack_msg = String(data=json.dumps({"speech_id": sid, "status": "COMPLETED"}))
                self.pub_tts_ack.publish(ack_msg)
        except Exception:
            pass

    def _publish_scan_loop(self):
        """Publish test scan at 20 Hz to satisfy Collision Monitor observation source."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = list(self.active_scan_ranges)
        self.pub_test_scan.publish(scan)

    def start_obs_stream(self, hand_raised=True, dist_m=1.6, near_edge=False):
        self._obs_hand_raised = hand_raised
        self._obs_dist_m = dist_m
        self._obs_near_edge = near_edge
        self._obs_timer_running = True

    def stop_obs_stream(self):
        self._obs_timer_running = False

    def _obs_tick(self):
        if not self._obs_timer_running:
            return

        now = self.get_clock().now()
        sec = now.to_msg().sec
        nanosec = now.to_msg().nanosec
        if (sec, nanosec) <= self._last_obs_stamp:
            sec, nanosec = self._last_obs_stamp
            nanosec += 33_333_333
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
        self._last_obs_stamp = (sec, nanosec)
        now_mono = time.monotonic()

        cand = {
            "track_id": self._obs_track_id,
            "is_confirmed": True,
            "is_ambiguous": False,
            "confidence": 0.95,
            "bbox_xyxy": [220.0, 100.0, 420.0, 400.0],
            "torso_depth_m": self._obs_dist_m,
            "depth_valid": True,
            "center_offset_x_norm": 0.0,
            "near_edge": self._obs_near_edge,
            "keypoints": [[320.0, 200.0, 0.9]] * 17,
        }
        cand["keypoints"][5] = [300.0, 180.0, 0.9]  # Left shoulder
        if self._obs_hand_raised:
            cand["keypoints"][9] = [300.0, 120.0, 0.95]  # Left wrist raised
        else:
            cand["keypoints"][9] = [300.0, 250.0, 0.95]  # Left wrist lowered

        obs = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_frame",
            "receive_monotonic_time": now_mono,
            "source_mode": "live_astra",
            "source_session_id": "src-gazebo-01",
            "data_quality": "VALID_RGBD",
            "processing_time_ms": 5.0,
            "device": "cuda:0",
            "active_track_ids": [self._obs_track_id],
            "lost_track_ids": [],
            "candidates": [cand],
        }
        self.pub_obs.publish(String(data=json.dumps(obs)))

    def publish_initial_pose(self, x=-1.0, y=-0.5, yaw=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        if self.latest_clock:
            msg.header.stamp = self.latest_clock.clock
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        for _ in range(5):
            self.pub_initial_pose.publish(msg)
            time.sleep(0.05)

    def send_voice_intent(self, action="navigate", target_id="gate_15", req_id="req-p9-1"):
        payload = {
            "action": action,
            "target_id": target_id,
            "request_id": req_id
        }
        t0 = time.time()
        while time.time() - t0 < 3.0 and self.pub_intent.get_subscription_count() == 0:
            time.sleep(0.1)
        for _ in range(2):
            self.pub_intent.publish(String(data=json.dumps(payload)))
            time.sleep(0.05)


def run_phase9_smoke_suite():
    evidence_dir = "/home/ws/src/.agent/airport_robot_voice/evidence"
    os.makedirs(evidence_dir, exist_ok=True)
    evidence_log = os.path.join(evidence_dir, "phase9_gazebo_escort_smoke.log")

    log_lines = []
    def log(msg):
        t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        line = f"[{t_str}] {msg}"
        print(line)
        log_lines.append(line)

    log("=" * 80)
    log("KHỞI ĐỘNG SMOKE TEST PHASE 9: VOICE THẬT + GAZEBO ESCORT + NAV2")
    log("=" * 80)

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = Phase9SmokeTestDriver()

    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    test_results = {}

    try:
        # Bước 0: Khởi tạo AMCL initialpose và chờ clock, odom
        log("[Bước 0] Đang chờ hệ thống Gazebo, Nav2, /clock và /odom sẵn sàng...")
        node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)

        ready = False
        for _ in range(40):
            if node.latest_clock and node.latest_clock.clock.sec > 0 and node.latest_odom is not None:
                ready = True
                break
            time.sleep(0.5)

        if not ready:
            log("[FAIL] Không nhận được /clock hoặc /odom sau 20s. Hủy test.")
            return

        log(f"[Bước 0] Hệ thống sẵn sàng! Clock sec={node.latest_clock.clock.sec}, odom x={node.latest_odom.pose.pose.position.x:.3f}")
        time.sleep(2.0)

        # ----------------------------------------------------------------------
        # CA 1: CHỨNG MINH KHÔNG DISPATCH TRƯỚC KHÓA (locked=False)
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 1: KIỂM CHỨNG CHẶN DISPATCH KHI CHƯA KHÓA HÀNH KHÁCH")
        log("-" * 80)
        fb_len_before = len(node.feedbacks)
        status_len_before = len(node.nav_statuses)
        node.send_voice_intent(action="navigate", target_id="gate_15", req_id="intent-c1-unlocked")

        time.sleep(2.0)

        c1_rejected = any(
            fb.get("event") == "REJECTED" and any(k in fb.get("status", "") for k in ("NOT_LOCKED", "NO_SESSION_ID", "ESCORT"))
            for fb in node.feedbacks[fb_len_before:]
        )
        c1_no_nav_started = not any(s.get("nav_event") == "NAVIGATION_STARTED" for s in node.nav_statuses[status_len_before:])
        c1_robot_still = all(abs(s[1]) < 0.02 and abs(s[2]) < 0.02 for s in node.odom_history[-30:])

        c1_pass = c1_rejected and c1_no_nav_started and c1_robot_still
        st_c1 = "PASSED [OK]" if c1_pass else "FAILED [X]"
        test_results["Ca 1 (Không dispatch trước khóa)"] = (c1_pass, f"rejected={c1_rejected}, no_nav_started={c1_no_nav_started}, robot_still={c1_robot_still}")
        log(f"Kết quả Ca 1: {st_c1} (rejected={c1_rejected}, no_nav_started={c1_no_nav_started}, robot_still={c1_robot_still})")

        # ----------------------------------------------------------------------
        # CA 2: PHIÊN HỢP LỆ DẪN ĐƯỜNG ĐƯỢC (locked -> NAVIGATION_STARTED -> xe chạy)
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 2: PHIÊN HỢP LỆ KHỞI HÀNH VÀ DẪN ĐƯỜNG THỰC TẾ TRONG GAZEBO")
        log("-" * 80)
        log("Bắt đầu phát observation giơ tay 20 Hz...")
        node.start_obs_stream(hand_raised=True, dist_m=1.6)

        locked_session_id = None
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if node.session_states and node.session_states[-1].get("locked") and node.session_states[-1].get("state") == "LOCKED_WAIT_DESTINATION":
                locked_session_id = node.session_states[-1].get("session_id")
                break
            time.sleep(0.1)

        log(f"Trạng thái Tracking: locked_session_id={locked_session_id}")
        assert locked_session_id is not None, "Tracking không khóa được hành khách!"

        node.start_obs_stream(hand_raised=False, dist_m=1.6)
        time.sleep(0.5)

        status_len_before = len(node.nav_statuses)
        odom_len_before = len(node.odom_history)
        log(f"Gửi lệnh dẫn đường tới cổng 15 cho session {locked_session_id}...")
        node.send_voice_intent(action="navigate", target_id="gate_15", req_id="intent-c2-start")

        nav_started_msg = None
        t0 = time.time()
        while time.time() - t0 < 8.0:
            for s in node.nav_statuses[status_len_before:]:
                if s.get("nav_event") == "NAVIGATION_STARTED":
                    nav_started_msg = s
                    break
            if nav_started_msg:
                break
            time.sleep(0.1)

        c2_nav_started = nav_started_msg is not None
        c2_ids_match = False
        if c2_nav_started:
            c2_ids_match = (
                nav_started_msg.get("session_id") == locked_session_id and
                bool(nav_started_msg.get("mission_id")) and
                bool(nav_started_msg.get("goal_attempt_id")) and
                bool(nav_started_msg.get("nav2_uuid")) and
                nav_started_msg.get("destination_id") == "gate_15"
            )
            log(f"Nhận NAVIGATION_STARTED: session={nav_started_msg.get('session_id')}, mission={nav_started_msg.get('mission_id')}, goal={nav_started_msg.get('goal_attempt_id')}, uuid={nav_started_msg.get('nav2_uuid')}")

        log("Đang theo dõi robot di chuyển vật lý trong Gazebo...")
        time.sleep(3.5)

        moving_samples = node.odom_history[odom_len_before:]
        max_vx_odom = max([s[1] for s in moving_samples]) if moving_samples else 0.0
        start_x, start_y = moving_samples[0][3], moving_samples[0][4] if moving_samples else (0, 0)
        curr_x, curr_y = moving_samples[-1][3], moving_samples[-1][4] if moving_samples else (0, 0)
        delta_dist = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)

        c2_moving = max_vx_odom > 0.05 and delta_dist > 0.08
        c2_pass = c2_nav_started and c2_ids_match and c2_moving
        st_c2 = "PASSED [OK]" if c2_pass else "FAILED [X]"
        test_results["Ca 2 (Phiên hợp lệ dẫn đường)"] = (c2_pass, f"nav_started={c2_nav_started}, ids_match={c2_ids_match}, max_vx={max_vx_odom:.3f}m/s, delta_d={delta_dist:.3f}m")
        log(f"Kết quả Ca 2: {st_c2} (max_vx={max_vx_odom:.3f} m/s, delta_d={delta_dist:.3f} m)")

        # ----------------------------------------------------------------------
        # CA 3: MẤT KHÁCH LÀM XE DỪNG VÀ HỦY ĐÚNG GOAL
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 3: MẤT KHÁCH -> XE DỪNG -> SAFETY ABORT -> ACK -> CANCELED -> IDLE")
        log("-" * 80)
        log("Mô phỏng mất khách: ngừng phát /tracking/observation...")
        node.stop_obs_stream()
        t_stop_obs = time.time()

        abort_req = None
        ack_status = None
        term_status = None

        while time.time() - t_stop_obs < 8.0:
            if not abort_req and node.safety_requests:
                for req in node.safety_requests:
                    if req.get("session_id") == locked_session_id and req.get("command") == "ABORT":
                        abort_req = req
                        break

            if abort_req and not ack_status:
                for st in node.nav_statuses:
                    if st.get("nav_event") == "CANCEL_ACKNOWLEDGED" and st.get("request_id") == abort_req.get("request_id"):
                        ack_status = st
                        break

            if abort_req and not term_status:
                for st in node.nav_statuses:
                    if st.get("is_terminal") and st.get("status") == "CANCELED" and st.get("session_id") == locked_session_id:
                        term_status = st
                        break

            if abort_req and ack_status and term_status:
                break
            time.sleep(0.1)

        c3_abort_received = abort_req is not None
        c3_ack_matched = ack_status is not None
        c3_term_matched = term_status is not None

        time.sleep(2.5)
        recent_odom = node.odom_history[-100:]
        all_stopped = all(abs(s[1]) < 0.02 and abs(s[2]) < 0.02 for s in recent_odom)
        tracking_idle = (node.session_states[-1].get("state") == "IDLE" and not node.session_states[-1].get("locked")) if node.session_states else False

        c3_pass = c3_abort_received and c3_ack_matched and c3_term_matched and all_stopped and tracking_idle
        st_c3 = "PASSED [OK]" if c3_pass else "FAILED [X]"
        test_results["Ca 3 (Mất khách xe dừng và hủy goal)"] = (c3_pass, f"abort={c3_abort_received}, ack={c3_ack_matched}, term={c3_term_matched}, stopped={all_stopped}, tracking_idle={tracking_idle}")
        log(f"Kết quả Ca 3: {st_c3} (abort={c3_abort_received}, ack={c3_ack_matched}, term={c3_term_matched}, stopped={all_stopped}, tracking_idle={tracking_idle})")

        # ----------------------------------------------------------------------
        # CA 4: ĐẾN ĐÍCH TRẢ SUCCEEDED VÀ TRACKING VỀ IDLE
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 4: ĐẾN ĐÍCH TRẢ SUCCEEDED VÀ TRACKING VỀ IDLE SẠCH SẼ")
        log("-" * 80)
        log("Bắt đầu phát observation giơ tay để khóa phiên mới (session 2)...")
        node.start_obs_stream(hand_raised=True, dist_m=1.6)

        session_2_id = None
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if node.session_states and node.session_states[-1].get("locked") and node.session_states[-1].get("state") == "LOCKED_WAIT_DESTINATION":
                if node.session_states[-1].get("session_id") != locked_session_id:
                    session_2_id = node.session_states[-1].get("session_id")
                    break
            time.sleep(0.1)

        log(f"Trạng thái Tracking Session 2: {session_2_id}")
        assert session_2_id is not None, "Tracking không khóa được session 2!"

        node.start_obs_stream(hand_raised=False, dist_m=1.6)
        time.sleep(0.5)

        status_len_before = len(node.nav_statuses)
        # Đến gate_15 (x=0.0, y=-0.6) là điểm đến thực tế nằm thẳng phía trước robot
        log(f"Gửi lệnh dẫn đường tới cổng 15 cho session 2 ({session_2_id})...")
        node.send_voice_intent(action="navigate", target_id="gate_15", req_id="intent-c4-arrive")

        succeeded_event = None
        t0 = time.time()
        last_diag = 0.0
        while time.time() - t0 < 40.0:
            now_t = time.time()
            if now_t - last_diag >= 2.0:
                last_diag = now_t
                curr_odom = node.latest_odom
                curr_vx = curr_odom.twist.twist.linear.x if curr_odom else 0.0
                curr_x = curr_odom.pose.pose.position.x if curr_odom else 0.0
                curr_y = curr_odom.pose.pose.position.y if curr_odom else 0.0
                log(f"  [Ca 4 Progress] t={now_t - t0:.1f}s | pos=({curr_x:.2f}, {curr_y:.2f}) | vx={curr_vx:.2f}")

            for s in node.nav_statuses[status_len_before:]:
                if s.get("is_terminal") and s.get("status") == "SUCCEEDED" and s.get("session_id") == session_2_id:
                    succeeded_event = s
                    break
            if succeeded_event:
                break
            time.sleep(0.2)

        c4_succeeded = succeeded_event is not None
        if c4_succeeded:
            log(f"Nhận được terminal SUCCEEDED cho session 2!")

        # Chờ Tracking Manager và Voice FSM chuyển về IDLE sạch sẽ
        t_wait_idle = time.time()
        c4_tracking_idle = False
        c4_voice_idle = False
        while time.time() - t_wait_idle < 6.0:
            c4_tracking_idle = bool(node.session_states and node.session_states[-1].get("state") == "IDLE" and not node.session_states[-1].get("locked"))
            c4_voice_idle = bool(node.nav_states and node.nav_states[-1].get("mission_state") == "IDLE" and not node.nav_states[-1].get("has_unresolved_goals"))
            if c4_tracking_idle and c4_voice_idle:
                break
            time.sleep(0.3)

        c4_pass = c4_succeeded and c4_tracking_idle and c4_voice_idle
        st_c4 = "PASSED [OK]" if c4_pass else "FAILED [X]"
        test_results["Ca 4 (Đến đích SUCCEEDED và IDLE)"] = (c4_pass, f"succeeded={c4_succeeded}, tracking_idle={c4_tracking_idle}, voice_idle={c4_voice_idle}")
        log(f"Kết quả Ca 4: {st_c4} (succeeded={c4_succeeded}, tracking_idle={c4_tracking_idle}, voice_idle={c4_voice_idle})")

    finally:
        node.stop_obs_stream()
        rclpy.shutdown()

    log("=" * 80)
    log("TỔNG HỢP KẾT QUẢ SMOKE TEST PHASE 9 THỰC TẾ TRÊN GAZEBO")
    log("=" * 80)
    all_passed = True
    for name, (ok, details) in test_results.items():
        st = "PASSED [OK]" if ok else "FAILED [X]"
        log(f"  * {name:<35}: {st} ({details})")
        if not ok:
            all_passed = False

    log("=" * 80)
    conclusion_str = "TẤT CẢ ĐẠT 100%" if all_passed else "CÓ CA THẤT BẠI"
    log(f"KẾT LUẬN: {conclusion_str}")
    log("=" * 80)

    with open(evidence_log, "w", encoding="utf-8") as f:
        f.write("\n".join(log_lines) + "\n")
    log(f"Đã lưu log chi tiết vào: {evidence_log}")

    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    run_phase9_smoke_suite()
