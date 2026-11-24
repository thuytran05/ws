#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 11 Pre-Check: Software Escort Handover Smoke Test in Gazebo Simulation.
Verifies the complete handover chain:
Case 1: Unlocked passenger -> Voice refuses goal dispatch (send_goal=0, robot still).
Case 2: Valid passenger lock -> Voice dispatches goal, NAVIGATION_STARTED emitted, robot moves in Gazebo.
Case 3: Stop observation -> Guard clamps cmd_vel to zero, safety_request abort emitted, Voice cancels Nav2 goal, terminal CANCELED received, robot halts.
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Phase11SmokeDriver(Node):
    def __init__(self):
        super().__init__(
            "phase11_smoke_driver",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.pub_obs = self.create_publisher(String, "/tracking/observation", reliable_qos)
        self.pub_intent = self.create_publisher(String, "/voice/intent", 10)
        self.pub_tts_ack = self.create_publisher(String, "/voice/tts_ack", 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_test_scan = self.create_publisher(LaserScan, "/tracking/scan_test", 10)

        # Subscribers
        self.sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.sub_cmd_smoothed = self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_cmd_smoothed, 20)
        self.sub_cmd_guarded = self.create_subscription(Twist, "/tracking/cmd_vel_guarded", self._on_cmd_guarded, 20)
        self.sub_cmd_watchdog = self.create_subscription(Twist, "/tracking/cmd_vel_watchdog", self._on_cmd_watchdog, 20)
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.sub_session = self.create_subscription(String, "/tracking/session_state", self._on_session, transient_qos)
        self.sub_safety = self.create_subscription(String, "/tracking/safety_request", self._on_safety, reliable_qos)
        self.sub_say = self.create_subscription(String, "/voice/say", self._on_say, 10)
        self.sub_feedback = self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.sub_nav_status = self.create_subscription(String, "/voice/nav_status", self._on_nav_status, transient_qos)

        self.latest_sim_time = 0.0
        self.latest_clock_msg: Optional[Clock] = None
        self.latest_session: Optional[dict] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd_smoothed: Optional[Twist] = None
        self.latest_cmd_guarded: Optional[Twist] = None
        self.latest_cmd_watchdog: Optional[Twist] = None
        self.latest_cmd_vel: Optional[Twist] = None

        self.feedbacks: List[dict] = []
        self.nav_statuses: List[dict] = []
        self.safety_requests: List[dict] = []
        self.odom_history: List[Tuple[float, float, float]] = []  # (sim_time, vx, vy)

        self.active_scan_ranges = [10.0] * 361
        self.scan_timer = self.create_timer(0.05, self._publish_scan_loop)

        self._obs_running = False
        self._obs_hand_raised = False
        self._obs_dist_m = 1.6
        self._obs_session = "src-smoke"
        self._last_obs_stamp = (0, 0)
        self.obs_timer = self.create_timer(0.05, self._obs_tick)

    def _on_clock(self, msg: Clock):
        self.latest_clock_msg = msg
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_cmd_smoothed(self, msg: Twist):
        self.latest_cmd_smoothed = msg

    def _on_cmd_guarded(self, msg: Twist):
        self.latest_cmd_guarded = msg

    def _on_cmd_watchdog(self, msg: Twist):
        self.latest_cmd_watchdog = msg

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.odom_history.append((self.latest_sim_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y))
        if len(self.odom_history) > 1000:
            self.odom_history.pop(0)

    def _on_session(self, msg: String):
        try:
            self.latest_session = json.loads(msg.data)
        except Exception:
            pass

    def _on_safety(self, msg: String):
        try:
            req = json.loads(msg.data)
            self.safety_requests.append(req)
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            payload = json.loads(msg.data)
            speech_id = payload.get("speech_id")
            mission_id = payload.get("mission_id")
            if speech_id:
                ack_msg = String()
                ack_msg.data = json.dumps({
                    "speech_id": speech_id,
                    "mission_id": mission_id,
                    "status": "COMPLETED",
                    "engine": "piper",
                    "audio_played": True
                })
                self.pub_tts_ack.publish(ack_msg)
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            fb = json.loads(msg.data)
            self.feedbacks.append(fb)
        except Exception:
            pass

    def _on_nav_status(self, msg: String):
        try:
            st = json.loads(msg.data)
            self.nav_statuses.append(st)
        except Exception:
            pass

    def _publish_scan_loop(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.12
        scan.range_max = 12.0
        scan.ranges = list(self.active_scan_ranges)
        self.pub_test_scan.publish(scan)

    def start_obs_stream(self, hand_raised=True, dist_m=1.6):
        self._obs_hand_raised = hand_raised
        self._obs_dist_m = dist_m
        self._obs_running = True

    def stop_obs_stream(self):
        self._obs_running = False

    def _obs_tick(self):
        if not self._obs_running:
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

        cand = {
            "track_id": 1,
            "is_confirmed": True,
            "is_ambiguous": False,
            "confidence": 0.95,
            "bbox_xyxy": [220.0, 100.0, 420.0, 400.0],
            "torso_depth_m": self._obs_dist_m,
            "depth_valid": True,
            "center_offset_x_norm": 0.0,
            "near_edge": False,
            "keypoints": [[320.0, 200.0, 0.9]] * 17,
        }
        cand["keypoints"][5] = [300.0, 180.0, 0.9]  # Left shoulder
        cand["keypoints"][6] = [340.0, 180.0, 0.9]  # Right shoulder
        if self._obs_hand_raised:
            cand["keypoints"][9] = [300.0, 120.0, 0.95]  # Left wrist raised (y=120 < 180)
        else:
            cand["keypoints"][9] = [300.0, 240.0, 0.95]  # Left wrist lowered
        cand["keypoints"][10] = [340.0, 240.0, 0.9]

        frame_data = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_link",
            "source_session_id": self._obs_session,
            "data_quality": "VALID_RGBD",
            "is_metric_valid": True,
            "candidates": [cand],
            "persons": [cand],
        }

        msg = String()
        msg.data = json.dumps(frame_data)
        self.pub_obs.publish(msg)

    def publish_initial_pose(self, x=-1.0, y=-0.5, yaw=0.0):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.068
        self.pub_initial_pose.publish(pose_msg)

    def send_voice_intent(self, action="navigate", target_id="gate_15", req_id="intent-1"):
        intent = String()
        intent.data = json.dumps({"action": action, "target_id": target_id, "req_id": req_id})
        self.pub_intent.publish(intent)


def run_phase11_smoke():
    log_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_11"
    os.makedirs(log_dir, exist_ok=True)
    summary_log = os.path.join(log_dir, "phase11_software_smoke.log")

    log_lines = []
    def log(msg):
        line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
        print(line)
        log_lines.append(line)

    log("=" * 80)
    log("PHASE 11 PRE-CHECK: SOFTWARE ESCORT HANDOVER SMOKE TEST (GAZEBO SIMULATION)")
    log("=" * 80)

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = Phase11SmokeDriver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    test_results = {}
    all_passed = True

    try:
        # Step 0: Wait for Gazebo clock, odom, amcl
        log("[Bước 0] Đang chờ Gazebo, Nav2, /clock và /odom sẵn sàng...")
        node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)

        ready = False
        for _ in range(40):
            if node.latest_clock_msg and node.latest_clock_msg.clock.sec > 0 and node.latest_odom is not None:
                ready = True
                break
            time.sleep(0.5)

        if not ready:
            log("[FAIL] Không nhận được /clock hoặc /odom sau 20s. Hủy test.")
            all_passed = False
            return

        log(f"[Bước 0] Sẵn sàng! Clock sec={node.latest_clock_msg.clock.sec}, odom pos=({node.latest_odom.pose.pose.position.x:.3f}, {node.latest_odom.pose.pose.position.y:.3f})")

        # Await DDS connection to Voice Node
        log("[Bước 0] Chờ DDS kết nối tới Voice node (/voice/intent)...")
        for _ in range(60):
            if node.pub_intent.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        log(f"[Bước 0] Đã kết nối /voice/intent: subs={node.pub_intent.get_subscription_count()}")
        time.sleep(1.0)

        # ----------------------------------------------------------------------
        # CASE 1: UNLOCKED PASSENGER -> VOICE REFUSES GOAL DISPATCH
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 1: CHƯA KHÓA HÀNH KHÁCH -> VOICE KHÔNG GỬI GOAL, XE ĐỨNG YÊN")
        log("-" * 80)
        fb_len_before = len(node.feedbacks)
        status_len_before = len(node.nav_statuses)
        node.send_voice_intent(action="navigate", target_id="gate_15", req_id="c1-unlocked-intent")

        c1_rejected = False
        t_c1 = time.time()
        while time.time() - t_c1 < 4.0:
            c1_rejected = any(fb.get("event") == "REJECTED" and any(k in fb.get("status", "") for k in ("NOT_LOCKED", "NO_SESSION_ID", "ESCORT")) for fb in node.feedbacks[fb_len_before:])
            if c1_rejected:
                break
            time.sleep(0.1)

        c1_dispatched = [fb for fb in node.feedbacks[fb_len_before:] if fb.get("event") == "NAV_GOAL_DISPATCHED"]
        c1_nav_started = [s for s in node.nav_statuses[status_len_before:] if s.get("nav_event") == "NAVIGATION_STARTED"]
        c1_robot_still = all(abs(s[1]) < 0.02 and abs(s[2]) < 0.02 for s in node.odom_history[-30:])

        c1_pass = (len(c1_dispatched) == 0) and (len(c1_nav_started) == 0) and c1_rejected and c1_robot_still
        test_results["Case 1 (Chặn dispatch trước khóa)"] = (c1_pass, f"dispatched={len(c1_dispatched)}, nav_started={len(c1_nav_started)}, rejected={c1_rejected}, robot_still={c1_robot_still}")
        log(f"Kết quả Ca 1: {'PASSED [OK]' if c1_pass else 'FAILED [X]'} (dispatched={len(c1_dispatched)}, rejected={c1_rejected}, robot_still={c1_robot_still})")
        if not c1_pass:
            all_passed = False

        # ----------------------------------------------------------------------
        # CASE 2: VALID LOCK -> VOICE SENDS GOAL -> NAVIGATION_STARTED -> XE CHẠY
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 2: KHÓA HỢP LỆ -> VOICE GỬI GOAL -> NAVIGATION_STARTED -> XE CHẠY")
        log("-" * 80)
        log("Bắt đầu phát observation giơ tay 20 Hz để khóa...")
        node.start_obs_stream(hand_raised=True, dist_m=1.6)

        locked_session_id = None
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("state") == "LOCKED_WAIT_DESTINATION":
                locked_session_id = node.latest_session.get("session_id")
                break
            time.sleep(0.1)

        log(f"  [Trạng thái khóa] locked_session_id={locked_session_id}")
        node.start_obs_stream(hand_raised=False, dist_m=1.6)

        c2_pass = False
        if locked_session_id:
            pos_start = (node.latest_odom.pose.pose.position.x, node.latest_odom.pose.pose.position.y)
            status_len_before_c2 = len(node.nav_statuses)
            fb_len_before_c2 = len(node.feedbacks)

            node.send_voice_intent(action="navigate", target_id="gate_15", req_id="c2-locked-intent")

            # Wait for goal dispatch and navigation started
            c2_started = []
            c2_dispatched = []
            t_wait_dispatch = time.time()
            while time.time() - t_wait_dispatch < 6.0:
                c2_started = [s for s in node.nav_statuses[status_len_before_c2:] if s.get("nav_event") == "NAVIGATION_STARTED"]
                c2_dispatched = [fb for fb in node.feedbacks[fb_len_before_c2:] if fb.get("event") == "NAV_GOAL_DISPATCHED"]
                if c2_dispatched and c2_started:
                    break
                time.sleep(0.05)

            # Wait for physical motion in Gazebo (up to 12.0s)
            log("Chờ Nav2 tính toán đường đi và robot di chuyển trong Gazebo...")
            t_motion_start = time.time()
            max_vx = 0.0
            dist_moved = 0.0
            while time.time() - t_motion_start < 12.0:
                if node.latest_odom:
                    vx = node.latest_odom.twist.twist.linear.x
                    max_vx = max(max_vx, vx)
                    dx = node.latest_odom.pose.pose.position.x - pos_start[0]
                    dy = node.latest_odom.pose.pose.position.y - pos_start[1]
                    dist_moved = math.hypot(dx, dy)
                    if dist_moved > 0.05 and max_vx > 0.05:
                        break
                time.sleep(0.05)

            has_5_ids = False
            if c2_started:
                st = c2_started[0]
                has_5_ids = all(bool(st.get(k)) for k in ("session_id", "mission_id", "goal_attempt_id", "nav2_uuid", "destination_id"))

            c2_pass = (len(c2_dispatched) >= 1) and (len(c2_started) >= 1) and has_5_ids and (dist_moved > 0.05) and (max_vx > 0.05)
            test_results["Case 2 (Khóa hợp lệ & Xe di chuyển)"] = (c2_pass, f"dispatched={len(c2_dispatched)}, has_5_ids={has_5_ids}, dist_moved={dist_moved:.3f}m, max_vx={max_vx:.3f}m/s")
            log(f"Kết quả Ca 2: {'PASSED [OK]' if c2_pass else 'FAILED [X]'} (dispatched={len(c2_dispatched)}, dist_moved={dist_moved:.3f}m, max_vx={max_vx:.3f}m/s)")
        else:
            test_results["Case 2 (Khóa hợp lệ & Xe di chuyển)"] = (False, "Timeout không khóa được hành khách")
            log("Kết quả Ca 2: FAILED [X] (Timeout khóa hành khách)")

        if not c2_pass:
            all_passed = False

        # Allow robot to cruise smoothly for 1.0s with observations continuous
        time.sleep(1.0)

        # ----------------------------------------------------------------------
        # CASE 3: OBSERVATION LOSS -> ACTIVE ZERO -> SAFETY ABORT -> NAV2 CANCEL
        # ----------------------------------------------------------------------
        log("-" * 80)
        log("CA 3: MẤT QUAN SÁT -> CMD_VEL ZERO -> SAFETY ABORT -> NAV2 CANCEL -> DỪNG HẲN")
        log("-" * 80)
        log("Mô phỏng mất khách: Ngừng phát /tracking/observation...")
        t_stop_obs_mono = time.monotonic()
        node.stop_obs_stream()

        # Measure Guard zero clamp and safety abort
        t_zero_mono = None
        t_abort_mono = None
        c3_safety_req = None

        t_wait = time.time()
        while time.time() - t_wait < 6.0:
            now_mono = time.monotonic()
            if t_zero_mono is None and node.latest_cmd_vel and abs(node.latest_cmd_vel.linear.x) < 0.001 and abs(node.latest_cmd_vel.angular.z) < 0.001:
                t_zero_mono = now_mono

            if node.safety_requests and t_abort_mono is None:
                t_abort_mono = now_mono
                c3_safety_req = node.safety_requests[-1]

            if t_zero_mono is not None and t_abort_mono is not None:
                break
            time.sleep(0.02)

        dt_zero = (t_zero_mono - t_stop_obs_mono) if t_zero_mono else 999.0
        dt_abort = (t_abort_mono - t_stop_obs_mono) if t_abort_mono else 999.0
        log(f"  [Mốc đo lường] dt_zero={dt_zero:.3f}s (<=0.65s), dt_abort={dt_abort:.3f}s (<=2.20s)")

        # Wait for Voice to ACK, Cancel Nav2, and emit terminal CANCELED
        t_cancel_wait = time.time()
        terminal_canceled = False
        matching_goal_attempt = False
        while time.time() - t_cancel_wait < 8.0:
            canceled_statuses = [
                s for s in node.nav_statuses
                if (s.get("is_terminal") is True and str(s.get("status")).upper() == "CANCELED")
                or str(s.get("terminal_outcome")).upper() == "CANCELED"
                or str(s.get("nav_event")).upper() in ("TERMINAL_CANCELED", "CANCELED")
            ]
            if canceled_statuses:
                terminal_canceled = True
                st = canceled_statuses[-1]
                matching_goal_attempt = (st.get("session_id") == locked_session_id)
                break
            time.sleep(0.1)

        # Verify physical halt on /odom
        time.sleep(1.0)
        c3_stopped = all(abs(s[1]) < 0.02 for s in node.odom_history[-20:])
        fsm_idle = (node.latest_session and node.latest_session.get("state") == "IDLE" and not node.latest_session.get("locked"))

        c3_pass = (dt_zero <= 0.65) and (dt_abort <= 2.20) and terminal_canceled and matching_goal_attempt and c3_stopped and fsm_idle
        test_results["Case 3 (Mất khách & Hủy goal Nav2)"] = (c3_pass, f"dt_zero={dt_zero:.3f}s, dt_abort={dt_abort:.3f}s, terminal_canceled={terminal_canceled}, matching_goal={matching_goal_attempt}, c3_stopped={c3_stopped}, fsm_idle={fsm_idle}")
        log(f"Kết quả Ca 3: {'PASSED [OK]' if c3_pass else 'FAILED [X]'} (dt_zero={dt_zero:.3f}s, dt_abort={dt_abort:.3f}s, terminal_canceled={terminal_canceled}, matching_goal={matching_goal_attempt})")

        if not c3_pass:
            all_passed = False

    finally:
        log("=" * 80)
        log("TỔNG KẾT SMOKE TEST PHASE 11:")
        for name, (passed, details) in test_results.items():
            status_str = "PASSED" if passed else "FAILED"
            log(f"  - {name}: {status_str} | {details}")
        log(f"KẾT LUẬN TOÀN BỘ: {'PASS (100% ĐẠT)' if all_passed else 'FAIL'}")
        log("=" * 80)

        with open(summary_log, "w", encoding="utf-8") as f:
            f.write("\n".join(log_lines) + "\n")
        print(f"Summary log written to: {summary_log}")

        try:
            executor.shutdown()
            th.join(timeout=1.0)
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    run_phase11_smoke()
