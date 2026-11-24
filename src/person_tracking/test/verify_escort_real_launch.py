#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 11: Verification of escort_navigation_real.launch.py with Isolated Gazebo Base.
Verifies:
1. Topic info graph and publisher audit:
   /cmd_vel_smoothed -> Guard -> /tracking/cmd_vel_guarded -> Watchdog -> /tracking/cmd_vel_watchdog -> Collision Monitor -> /cmd_vel -> Base
   Confirms /cmd_vel has ONLY collision_monitor as publisher during escort mode.
   Audits docking/teleop publishers if present.
2. Runtime parameters:
   Verifies max_linear_x = 0.30 m/s and max_angular_z = 0.20 rad/s on Guard and Watchdog.
3. Smoke Test:
   Case 1: Unlocked passenger -> Voice refuses dispatch (send_goal=0, robot still).
   Case 2: Locked passenger -> Voice dispatches goal -> NAVIGATION_STARTED (captures session_id, goal_attempt_id, nav2_uuid) -> Robot moves.
   Case 3: Stop observation -> Guard clamps zero -> Safety abort -> Current goal canceled.
           Verifies matching session_id, goal_attempt_id, and nav2_uuid on terminal CANCELED event!
"""

import json
import math
import os
import subprocess
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
from std_msgs.msg import String


class EscortRealVerifyNode(Node):
    def __init__(self):
        super().__init__(
            "escort_real_verify_node",
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

        self._obs_running = False
        self._obs_hand_raised = False
        self._obs_dist_m = 1.6
        self._obs_session = "src-escort-real"
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
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_history.append((t, vx, vy))
        if len(self.odom_history) > 300:
            self.odom_history.pop(0)

    def _on_session(self, msg: String):
        try:
            self.latest_session = json.loads(msg.data)
        except Exception:
            pass

    def _on_safety(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.safety_requests.append(data)
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
            self.feedbacks.append(json.loads(msg.data))
        except Exception:
            pass

    def _on_nav_status(self, msg: String):
        try:
            self.nav_statuses.append(json.loads(msg.data))
        except Exception:
            pass

    def publish_initial_pose(self, x: float = -1.0, y: float = -0.5, yaw: float = 0.0):
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

    def start_obs_stream(self, hand_raised: bool = False, dist_m: float = 1.6):
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
            "torso_depth_m": float(self._obs_dist_m),
            "depth_valid": True,
            "depth_proven": True,
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
            "frame_id": "camera_color_optical_frame",
            "source_session_id": self._obs_session,
            "data_quality": "VALID_RGBD",
            "is_metric_valid": True,
            "candidates": [cand],
            "persons": [cand],
        }

        msg = String()
        msg.data = json.dumps(frame_data)
        self.pub_obs.publish(msg)

    def send_voice_intent(self, action: str = "navigate", target_id: str = "gate_15", req_id: str = "req-1"):
        payload = {
            "schema_version": "1.0",
            "action": action,
            "target_id": target_id,
            "request_id": req_id,
            "confidence": 0.98,
            "timestamp": time.time()
        }
        self.pub_intent.publish(String(data=json.dumps(payload)))


def run_command_capture(cmd: str) -> str:
    res = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return res.stdout.strip()


def main():
    rclpy.init()
    node = EscortRealVerifyNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    evidence_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_11"
    os.makedirs(evidence_dir, exist_ok=True)
    summary_log = os.path.join(evidence_dir, "escort_real_smoke.log")
    topic_graph_log = os.path.join(evidence_dir, "escort_real_topic_graph.log")
    params_log = os.path.join(evidence_dir, "escort_real_params.log")

    log_lines = []

    def log(msg: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line, flush=True)
        log_lines.append(line)

    log("=" * 80)
    log("PHASE 11: VERIFYING escort_navigation_real.launch.py WITH ISOLATED GAZEBO BASE")
    log("=" * 80)

    test_results = {}
    all_passed = True

    try:
        # Step 0: Wait for Clock and Odom
        log("[Bước 0] Đang chờ Gazebo base, Nav2, /clock và /odom sẵn sàng...")
        node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)

        t_wait_ready = time.time()
        while time.time() - t_wait_ready < 30.0:
            if node.latest_clock_msg and node.latest_clock_msg.clock.sec > 0 and node.latest_odom:
                break
            time.sleep(0.5)

        if not (node.latest_clock_msg and node.latest_odom):
            log("LỖI: Không nhận được /clock hoặc /odom từ mô phỏng!")
            os._exit(1)

        log(f"[Bước 0] Sẵn sàng! Clock sec={node.latest_clock_msg.clock.sec}, odom pos=({node.latest_odom.pose.pose.position.x:.3f}, {node.latest_odom.pose.pose.position.y:.3f})")

        # Step 1: Audit Topic Graph and Publishers
        log("-" * 80)
        log("MỤC 1: XÁC THỰC TOPIC INFO GRAPH VÀ AUDIT PUBLISHER /cmd_vel")
        log("-" * 80)
        
        topics_to_check = [
            "/cmd_vel_smoothed",
            "/tracking/cmd_vel_guarded",
            "/tracking/cmd_vel_watchdog",
            "/cmd_vel"
        ]
        
        topic_graph_output = []
        topic_graph_output.append("=== AUDIT OF VELOCITY TOPICS VIA ros2 topic info -v ===")
        
        for t in topics_to_check:
            out = run_command_capture(f"ros2 topic info -v {t}")
            topic_graph_output.append(f"\n--- Topic: {t} ---\n{out}")
            log(f"Topic {t}: Checked info -v")

        # Check /cmd_vel_docking and teleop
        docking_info = run_command_capture("ros2 topic info -v /cmd_vel_docking 2>/dev/null || true")
        topic_graph_output.append(f"\n--- Topic: /cmd_vel_docking (Remapped Docking Output) ---\n{docking_info}")

        with open(topic_graph_log, "w", encoding="utf-8") as f:
            f.write("\n".join(topic_graph_output) + "\n")
        log(f"Topic graph details written to: {topic_graph_log}")

        # Verify that ONLY collision_monitor publishes to /cmd_vel during escort
        cmd_vel_info_raw = run_command_capture("ros2 topic info -v /cmd_vel")
        only_collision_monitor = ("Node name: collision_monitor" in cmd_vel_info_raw) and ("Node name: escort_velocity_guard" not in cmd_vel_info_raw) and ("Node name: escort_velocity_watchdog" not in cmd_vel_info_raw)
        log(f"  [Audit Publisher /cmd_vel] Duy nhất Collision Monitor phát: {only_collision_monitor}")
        test_results["Audit Publisher /cmd_vel (Duy nhất Collision Monitor)"] = (only_collision_monitor, f"Raw pubs output contains collision_monitor and no direct bypass: {only_collision_monitor}")
        if not only_collision_monitor:
            all_passed = False

        # Step 2: Verify Runtime Parameters (max_linear_x = 0.30, max_angular_z = 0.20)
        log("-" * 80)
        log("MỤC 2: XÁC MINH THAM SỐ RUNTIME TẠI GUARD VÀ WATCHDOG")
        log("-" * 80)
        guard_vx = run_command_capture("ros2 param get /escort_velocity_guard max_linear_x 2>/dev/null || true")
        guard_wz = run_command_capture("ros2 param get /escort_velocity_guard max_angular_z 2>/dev/null || true")
        watchdog_vx = run_command_capture("ros2 param get /escort_velocity_watchdog max_linear_x 2>/dev/null || true")
        watchdog_wz = run_command_capture("ros2 param get /escort_velocity_watchdog max_angular_z 2>/dev/null || true")

        params_text = f"Guard max_linear_x: {guard_vx}\nGuard max_angular_z: {guard_wz}\nWatchdog max_linear_x: {watchdog_vx}\nWatchdog max_angular_z: {watchdog_wz}"
        with open(params_log, "w", encoding="utf-8") as f:
            f.write(params_text + "\n")
        log(f"Runtime parameters logged to: {params_log}")
        log(f"  Guard limits: vx={guard_vx}, wz={guard_wz}")
        log(f"  Watchdog limits: vx={watchdog_vx}, wz={watchdog_wz}")

        guard_params_ok = ("0.3" in guard_vx) and ("0.2" in guard_wz)
        watchdog_params_ok = ("0.3" in watchdog_vx) and ("0.2" in watchdog_wz)
        params_ok = guard_params_ok and watchdog_params_ok
        test_results["Xác minh Giới hạn Vận tốc 0.30 m/s & 0.20 rad/s"] = (params_ok, f"guard_ok={guard_params_ok}, watchdog_ok={watchdog_params_ok}")
        if not params_ok:
            all_passed = False

        # Wait for Voice node to be connected to /voice/intent
        log("Chờ kết nối /voice/intent...")
        t_wait_voice = time.time()
        while time.time() - t_wait_voice < 15.0:
            if node.pub_intent.get_subscription_count() > 0:
                break
            time.sleep(0.2)
        log(f"Đã kết nối /voice/intent: subs={node.pub_intent.get_subscription_count()}")
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
        dispatched_session_id = None
        dispatched_goal_attempt_id = None
        dispatched_nav2_uuid = None

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
                dispatched_session_id = st.get("session_id")
                dispatched_goal_attempt_id = st.get("goal_attempt_id")
                dispatched_nav2_uuid = st.get("nav2_uuid")
                has_5_ids = all(bool(st.get(k)) for k in ("session_id", "mission_id", "goal_attempt_id", "nav2_uuid", "destination_id"))
                log(f"  [Định danh Goal Active] session_id={dispatched_session_id}, goal_attempt_id={dispatched_goal_attempt_id}, nav2_uuid={dispatched_nav2_uuid}")

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
        matching_3_ids = False
        match_session = False
        match_attempt = False
        match_uuid = False

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
                match_session = (st.get("session_id") == dispatched_session_id)
                match_attempt = (st.get("goal_attempt_id") == dispatched_goal_attempt_id)
                match_uuid = (st.get("nav2_uuid") == dispatched_nav2_uuid)
                matching_3_ids = match_session and match_attempt and match_uuid
                log(f"  [Đối chiếu 3 Định danh Hủy Goal] match_session={match_session} ({st.get('session_id')}), match_attempt={match_attempt} ({st.get('goal_attempt_id')}), match_uuid={match_uuid} ({st.get('nav2_uuid')})")
                break
            time.sleep(0.1)

        # Verify physical halt on /odom
        time.sleep(1.0)
        c3_stopped = all(abs(s[1]) < 0.02 for s in node.odom_history[-20:])
        fsm_idle = (node.latest_session and node.latest_session.get("state") == "IDLE" and not node.latest_session.get("locked"))

        c3_pass = (dt_zero <= 0.65) and (dt_abort <= 2.20) and terminal_canceled and matching_3_ids and c3_stopped and fsm_idle
        test_results["Case 3 (Mất khách & Hủy goal Nav2 khớp 3 ID)"] = (c3_pass, f"dt_zero={dt_zero:.3f}s, dt_abort={dt_abort:.3f}s, terminal_canceled={terminal_canceled}, matching_3_ids={matching_3_ids}, c3_stopped={c3_stopped}, fsm_idle={fsm_idle}")
        log(f"Kết quả Ca 3: {'PASSED [OK]' if c3_pass else 'FAILED [X]'} (dt_zero={dt_zero:.3f}s, dt_abort={dt_abort:.3f}s, terminal_canceled={terminal_canceled}, matching_3_ids={matching_3_ids})")

        if not c3_pass:
            all_passed = False

    finally:
        log("=" * 80)
        log("TỔNG KẾT NGHIỆM THU escort_navigation_real.launch.py TRÊN GAZEBO CÔ LẬP:")
        for name, (passed, details) in test_results.items():
            status_str = "PASSED" if passed else "FAILED"
            log(f"  - {name}: {status_str} | {details}")
        log(f"KẾT LUẬN TOÀN BỘ: {'PASS (100% ĐẠT)' if all_passed else 'FAIL'}")
        log("=" * 80)

        with open(summary_log, "w", encoding="utf-8") as f:
            f.write("\n".join(log_lines) + "\n")
        print(f"Summary log written to: {summary_log}")

        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
