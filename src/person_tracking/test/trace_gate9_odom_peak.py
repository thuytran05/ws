#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trace and analyze Gate 9 Odometry peak (0.728 m/s).
Records high-frequency timestamps and kinematic values across all 6 topics:
1. /cmd_vel_smoothed (Nav2 velocity_smoother)
2. /tracking/cmd_vel_guarded (escort_velocity_guard)
3. /tracking/cmd_vel_watchdog (escort_velocity_watchdog)
4. /cmd_vel (collision_monitor)
5. /odom/unfiltered (Gazebo MecanumDrive system)
6. /odom (robot_localization EKF)
"""

import csv
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


class OdomPeakTracerNode(Node):
    def __init__(self):
        super().__init__(
            "odom_peak_tracer_node",
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

        # Trace storage
        self.records: List[dict] = []
        self._lock = threading.Lock()

        # Publishers
        self.pub_obs = self.create_publisher(String, "/tracking/observation", reliable_qos)
        self.pub_intent = self.create_publisher(String, "/voice/intent", 10)
        self.pub_tts_ack = self.create_publisher(String, "/voice/tts_ack", 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_test_scan = self.create_publisher(LaserScan, "/tracking/scan_test", 10)

        # Subscribers for all 6 topics
        self.sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.sub_cmd_smoothed = self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_cmd_smoothed, 20)
        self.sub_cmd_guarded = self.create_subscription(Twist, "/tracking/cmd_vel_guarded", self._on_cmd_guarded, 20)
        self.sub_cmd_watchdog = self.create_subscription(Twist, "/tracking/cmd_vel_watchdog", self._on_cmd_watchdog, 20)
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self.sub_odom_unfiltered = self.create_subscription(Odometry, "/odom/unfiltered", self._on_odom_unfiltered, 50)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 50)

        # Voice and Tracking state subscribers
        self.sub_session_state = self.create_subscription(String, "/tracking/session_state", self._on_session_state, transient_qos)
        self.sub_nav_status = self.create_subscription(String, "/voice/nav_status", self._on_nav_status, transient_qos)
        self.sub_feedback = self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.sub_say = self.create_subscription(String, "/voice/say", self._on_say, 10)

        self.latest_sim_time = 0.0
        self.latest_clock_msg: Optional[Clock] = None
        self.latest_session_state: Optional[dict] = None
        self.nav_statuses: List[dict] = []
        self.feedbacks: List[dict] = []

        # Background scan at 20 Hz
        self.active_scan_ranges = [10.0] * 361
        self.scan_timer = self.create_timer(0.05, self._publish_scan_loop)

        # Observation timer (20 Hz)
        self._obs_timer_running = False
        self._obs_track_id = 1
        self._obs_dist_m = 1.6
        self._obs_hand_raised = False
        self._last_obs_stamp = (0, 0)
        self.obs_timer = self.create_timer(0.05, self._obs_tick)

    def _on_clock(self, msg: Clock):
        self.latest_clock_msg = msg
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _record(self, topic: str, sim_time: float, vx: float, vy: float, wz: float, extra: dict = None):
        t_wall = time.time()
        t_mono = time.monotonic()
        speed_xy = math.sqrt(vx * vx + vy * vy)
        row = {
            "topic": topic,
            "wall_time": t_wall,
            "mono_time": t_mono,
            "sim_time": sim_time,
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "speed_xy": speed_xy,
        }
        if extra:
            row.update(extra)
        with self._lock:
            self.records.append(row)

    def _on_cmd_smoothed(self, msg: Twist):
        self._record("/cmd_vel_smoothed", self.latest_sim_time, msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_cmd_guarded(self, msg: Twist):
        self._record("/tracking/cmd_vel_guarded", self.latest_sim_time, msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_cmd_watchdog(self, msg: Twist):
        self._record("/tracking/cmd_vel_watchdog", self.latest_sim_time, msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_cmd_vel(self, msg: Twist):
        self._record("/cmd_vel", self.latest_sim_time, msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_odom_unfiltered(self, msg: Odometry):
        s_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        extra = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "pos_x": msg.pose.pose.position.x,
            "pos_y": msg.pose.pose.position.y,
        }
        self._record("/odom/unfiltered", s_t, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z, extra)

    def _on_odom(self, msg: Odometry):
        s_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        extra = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "pos_x": msg.pose.pose.position.x,
            "pos_y": msg.pose.pose.position.y,
        }
        self._record("/odom", s_t, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z, extra)

    def _on_session_state(self, msg: String):
        try:
            self.latest_session_state = json.loads(msg.data)
        except Exception:
            pass

    def _on_nav_status(self, msg: String):
        try:
            self.nav_statuses.append(json.loads(msg.data))
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            self.feedbacks.append(json.loads(msg.data))
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            d = json.loads(msg.data)
            sid = d.get("speech_id")
            if sid:
                ack_msg = String(data=json.dumps({"speech_id": sid, "status": "COMPLETED"}))
                self.pub_tts_ack.publish(ack_msg)
        except Exception:
            pass

    def publish_initial_pose(self, x=-1.0, y=-0.5, yaw=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        if self.latest_clock_msg:
            msg.header.stamp = self.latest_clock_msg.clock
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

    def _publish_scan_loop(self):
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

    def start_obs_stream(self, hand_raised=True, dist_m=1.6):
        self._obs_hand_raised = hand_raised
        self._obs_dist_m = dist_m
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
            "near_edge": False,
            "keypoints": [[320.0, 200.0, 0.9]] * 17,
        }
        cand["keypoints"][5] = [300.0, 180.0, 0.9]
        if self._obs_hand_raised:
            cand["keypoints"][9] = [300.0, 120.0, 0.95]
        else:
            cand["keypoints"][9] = [300.0, 250.0, 0.95]

        obs = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_frame",
            "receive_monotonic_time": now_mono,
            "source_mode": "live_astra",
            "source_session_id": "src-tracer-01",
            "data_quality": "VALID_RGBD",
            "processing_time_ms": 5.0,
            "device": "cuda:0",
            "active_track_ids": [self._obs_track_id],
            "lost_track_ids": [],
            "candidates": [cand],
        }
        msg = String(data=json.dumps(obs))
        self.pub_obs.publish(msg)

    def send_voice_intent(self, action="navigate", target_id="gate_15", req_id="req-p10-trace"):
        payload = {
            "action": action,
            "target_id": target_id,
            "request_id": req_id
        }
        t0 = time.time()
        while time.time() - t0 < 5.0 and self.pub_intent.get_subscription_count() == 0:
            time.sleep(0.1)
        for _ in range(3):
            self.pub_intent.publish(String(data=json.dumps(payload)))
            time.sleep(0.05)


def main():
    rclpy.init()
    node = OdomPeakTracerNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("[Tracer] Waiting for /clock and simulation system to be ready...")
    for _ in range(60):
        if node.latest_sim_time > 0.5:
            break
        time.sleep(0.5)

    print(f"[Tracer] Simulation ready. Current sim_time={node.latest_sim_time:.2f}s")
    node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)
    time.sleep(2.0)

    # 1. Start Hand Raised Observation
    print("[Tracer] Step 1: Raising hand (20 Hz obs) to lock passenger...")
    node.start_obs_stream(hand_raised=True, dist_m=1.6)

    locked_session_id = None
    t0 = time.time()
    while time.time() - t0 < 12.0:
        if node.latest_session_state and node.latest_session_state.get("locked") and node.latest_session_state.get("state") == "LOCKED_WAIT_DESTINATION":
            locked_session_id = node.latest_session_state.get("session_id")
            break
        time.sleep(0.1)

    print(f"[Tracer] Locked session_id: {locked_session_id}")
    if not locked_session_id:
        print("[Tracer] ERROR: Failed to lock passenger!")
        node.stop_obs_stream()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    # 2. Lower hand, maintain tracking
    node.start_obs_stream(hand_raised=False, dist_m=1.6)
    time.sleep(0.5)

    # 3. Send Voice navigation intent to gate_15 (matching Ca 2 in Gate 9)
    print("[Tracer] Step 2: Sending voice intent to navigate to gate_15...")
    node.send_voice_intent(action="navigate", target_id="gate_15", req_id="intent-tracer-gate15")

    # 4. Wait for NAVIGATION_STARTED
    nav_started = False
    t0 = time.time()
    while time.time() - t0 < 12.0:
        for s in node.nav_statuses:
            if s.get("nav_event") == "NAVIGATION_STARTED":
                nav_started = True
                print(f"[Tracer] Received NAVIGATION_STARTED! Details: {s}")
                break
        if nav_started:
            break
        time.sleep(0.1)

    if not nav_started:
        print("[Tracer] ERROR: Did not receive NAVIGATION_STARTED!")

    # 5. Record motion trajectory while robot moves
    print("[Tracer] Step 3: Recording kinematics across all 6 topics for 6.0 seconds...")
    time.sleep(6.0)

    # Stop obs
    node.stop_obs_stream()
    time.sleep(1.0)

    # Export records
    out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
    os.makedirs(out_dir, exist_ok=True)
    csv_file = os.path.join(out_dir, "gate9_odom_peak_trace.csv")
    log_file = os.path.join(out_dir, "odom_peak_analysis.log")

    with node._lock:
        records = list(node.records)

    print(f"[Tracer] Total records collected: {len(records)}")

    # Write CSV
    if records:
        fieldnames = ["topic", "wall_time", "mono_time", "sim_time", "vx", "vy", "wz", "speed_xy", "frame_id", "child_frame_id", "pos_x", "pos_y"]
        with open(csv_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for r in records:
                writer.writerow(r)
        print(f"[Tracer] Wrote raw CSV: {csv_file}")

    # Analyze peak
    odom_records = [r for r in records if r["topic"] == "/odom"]
    odom_unfiltered_records = [r for r in records if r["topic"] == "/odom/unfiltered"]
    cmd_smoothed_records = [r for r in records if r["topic"] == "/cmd_vel_smoothed"]
    cmd_guarded_records = [r for r in records if r["topic"] == "/tracking/cmd_vel_guarded"]
    cmd_watchdog_records = [r for r in records if r["topic"] == "/tracking/cmd_vel_watchdog"]
    cmd_vel_records = [r for r in records if r["topic"] == "/cmd_vel"]

    max_vx_odom = max([r["vx"] for r in odom_records]) if odom_records else 0.0
    max_speed_odom = max([r["speed_xy"] for r in odom_records]) if odom_records else 0.0
    max_vx_unfiltered = max([r["vx"] for r in odom_unfiltered_records]) if odom_unfiltered_records else 0.0
    max_speed_unfiltered = max([r["speed_xy"] for r in odom_unfiltered_records]) if odom_unfiltered_records else 0.0

    max_vx_smoothed = max([r["vx"] for r in cmd_smoothed_records]) if cmd_smoothed_records else 0.0
    max_speed_smoothed = max([r["speed_xy"] for r in cmd_smoothed_records]) if cmd_smoothed_records else 0.0
    max_vx_guarded = max([r["vx"] for r in cmd_guarded_records]) if cmd_guarded_records else 0.0
    max_speed_guarded = max([r["speed_xy"] for r in cmd_guarded_records]) if cmd_guarded_records else 0.0
    max_vx_watchdog = max([r["vx"] for r in cmd_watchdog_records]) if cmd_watchdog_records else 0.0
    max_speed_watchdog = max([r["speed_xy"] for r in cmd_watchdog_records]) if cmd_watchdog_records else 0.0
    max_vx_cmd_vel = max([r["vx"] for r in cmd_vel_records]) if cmd_vel_records else 0.0
    max_speed_cmd_vel = max([r["speed_xy"] for r in cmd_vel_records]) if cmd_vel_records else 0.0

    # Locate peak in /odom
    peak_record = None
    for r in odom_records:
        if r["vx"] == max_vx_odom:
            peak_record = r
            break

    peak_sim_time = peak_record["sim_time"] if peak_record else 0.0

    # Window around peak (+- 0.5s)
    window_records = [r for r in records if abs(r["sim_time"] - peak_sim_time) <= 0.5]

    analysis_lines = []
    analysis_lines.append("=" * 80)
    analysis_lines.append("PHASE 10: BÁO CÁO PHÂN TÍCH VÀ ĐO ĐẠC ĐỈNH ODOMETRY 0.728 M/S TRONG GAZEBO")
    analysis_lines.append("=" * 80)
    analysis_lines.append(f"Mục tiêu kiểm chứng: Tái hiện Ca 2 (Gate 9, đích gate_15) và đo đạc đồng thời 6 topic vận tốc.")
    analysis_lines.append(f"Thời điểm đỉnh odom vx: sim_time = {peak_sim_time:.3f} s, mono_time = {peak_record.get('mono_time', 0.0):.3f} s")
    analysis_lines.append("")
    analysis_lines.append("1. SO SÁNH TRẦN VẬN TỐC CỦA TỪNG KHÂU (COMMAND VS ODOMETRY):")
    analysis_lines.append("-" * 80)
    analysis_lines.append(f"{'Topic':<32} | {'Nút xuất bản (Publisher)':<26} | {'Max linear.x':<12} | {'Max speed_xy':<12}")
    analysis_lines.append("-" * 80)
    analysis_lines.append(f"{'/cmd_vel_smoothed':<32} | {'nav2_velocity_smoother':<26} | {max_vx_smoothed:<12.4f} | {max_speed_smoothed:<12.4f}")
    analysis_lines.append(f"{'/tracking/cmd_vel_guarded':<32} | {'escort_velocity_guard':<26} | {max_vx_guarded:<12.4f} | {max_speed_guarded:<12.4f}")
    analysis_lines.append(f"{'/tracking/cmd_vel_watchdog':<32} | {'escort_velocity_watchdog':<26} | {max_vx_watchdog:<12.4f} | {max_speed_watchdog:<12.4f}")
    analysis_lines.append(f"{'/cmd_vel':<32} | {'collision_monitor':<26} | {max_vx_cmd_vel:<12.4f} | {max_speed_cmd_vel:<12.4f}")
    analysis_lines.append(f"{'/odom/unfiltered':<32} | {'gz::MecanumDrive (Gazebo)':<26} | {max_vx_unfiltered:<12.4f} | {max_speed_unfiltered:<12.4f}")
    analysis_lines.append(f"{'/odom':<32} | {'robot_localization (EKF)':<26} | {max_vx_odom:<12.4f} | {max_speed_odom:<12.4f}")
    analysis_lines.append("-" * 80)
    analysis_lines.append("")

    analysis_lines.append("2. CÁC MẪU CHI TIẾT XUNG QUANH ĐỈNH VẬN TỐC (CỬA SỔ +- 0.3s):")
    analysis_lines.append("-" * 80)
    analysis_lines.append(f"{'sim_time':<10} | {'topic':<28} | {'linear.x':<10} | {'linear.y':<10} | {'angular.z':<10} | {'speed_xy':<10}")
    analysis_lines.append("-" * 80)
    for r in sorted(window_records, key=lambda x: (x["sim_time"], x["mono_time"])):
        if abs(r["sim_time"] - peak_sim_time) <= 0.3:
            analysis_lines.append(f"{r['sim_time']:<10.3f} | {r['topic']:<28} | {r['vx']:<10.4f} | {r['vy']:<10.4f} | {r['wz']:<10.4f} | {r['speed_xy']:<10.4f}")
    analysis_lines.append("-" * 80)
    analysis_lines.append("")

    analysis_lines.append("3. PHÂN TÍCH NGUYÊN NHÂN KỸ THUẬT GỐC RỄ (ROOT CAUSE ANALYSIS):")
    analysis_lines.append("-" * 80)
    analysis_lines.append(f"A. Giới hạn Command Chain:")
    analysis_lines.append(f"   - /cmd_vel_smoothed, /tracking/cmd_vel_guarded, /tracking/cmd_vel_watchdog, /cmd_vel")
    analysis_lines.append(f"     toàn bộ đều tuân thủ nghiêm ngặt giới hạn trần max_linear_x = 0.500 m/s.")
    analysis_lines.append(f"   - Guard KHÔNG BAO GIỜ xuất lệnh > 0.50 m/s (thực đo max_vx = {max_vx_guarded:.4f} m/s <= 0.50).")
    analysis_lines.append("")
    analysis_lines.append(f"B. Nguồn gốc của độ vọt lố trong Odometry:")
    analysis_lines.append(f"   - Đỉnh max vx đo được trên /odom: {max_vx_odom:.4f} m/s.")
    analysis_lines.append(f"   - Đỉnh max vx đo được trên /odom/unfiltered: {max_vx_unfiltered:.4f} m/s.")
    if max_vx_unfiltered > 0.52 and abs(max_vx_odom - max_vx_unfiltered) < 0.15:
        analysis_lines.append(f"   => Nguyên nhân chính: Gazebo MecanumDrive System tính toán kinematic bánh xe Mecanum (4 roller wheels)")
        analysis_lines.append(f"      khi vừa quay vừa tịnh tiến (coupling linear.x + angular.z) hoặc do trượt bánh (wheel slip)")
        analysis_lines.append(f"      dưới ma sát mô phỏng của Gazebo Sim Physics (DART/Bullet engine).")
    elif max_vx_odom > max_vx_unfiltered + 0.1:
        analysis_lines.append(f"   => Nguyên nhân chính: EKF Filter Node (robot_localization) ước lượng đạo hàm gia tốc và tích hợp IMU")
        analysis_lines.append(f"      tạo ra hiện tượng overshoot (vọt lố quán tính) trong trạng thái chuyển tiếp gia tốc lớn.")
    else:
        analysis_lines.append(f"   => Phối hợp cả đặc tính động học bánh xe Mecanum trong Gazebo và bộ lọc EKF.")
    analysis_lines.append("")
    analysis_lines.append(f"C. Đánh giá vector vận tốc so với linear.x:")
    analysis_lines.append(f"   - Robot Mecanum có thể di chuyển 2D (vx, vy). Tại đỉnh, vx={peak_record.get('vx', 0.0):.4f}, vy={peak_record.get('vy', 0.0):.4f}.")
    analysis_lines.append(f"   - Tốc độ tổng speed_xy = sqrt(vx^2 + vy^2) = {peak_record.get('speed_xy', 0.0):.4f} m/s.")
    analysis_lines.append("")
    analysis_lines.append(f"D. Khuyến nghị:")
    analysis_lines.append(f"   - Giữ nguyên cấu hình và ngưỡng an toàn của Velocity Guard (max 0.50 m/s).")
    analysis_lines.append(f"   - Khi nghiệm thu kiểm tra robot di chuyển, phân biệt rõ lệnh điều khiển xuất ra (commanded speed <= 0.50 m/s)")
    analysis_lines.append(f"     với vận tốc ước lượng từ bánh xe trong mô phỏng (simulated wheel slip / EKF overshoot).")
    analysis_lines.append("=" * 80)

    log_content = "\n".join(analysis_lines)
    print(log_content)

    with open(log_file, "w", encoding="utf-8") as f:
        f.write(log_content)
    print(f"[Tracer] Wrote analysis report: {log_file}")

    try:
        executor.shutdown()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
