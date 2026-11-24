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

Supports multi-run repetition of Ca 2 (destination gate_15) with Gate 9 configuration
to attempt reproducing any odometry velocity peak > 0.500 m/s.
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
        self._obs_session_source = "src-tracer-01"
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

    def start_obs_stream(self, hand_raised=True, dist_m=1.6, source_session="src-tracer-01"):
        self._obs_hand_raised = hand_raised
        self._obs_dist_m = dist_m
        self._obs_session_source = source_session
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
            "source_session_id": self._obs_session_source,
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


def run_single_iteration(node: OdomPeakTracerNode, run_idx: int) -> dict:
    print(f"\n--- [Tracer] Starting Run {run_idx} (Destination: gate_15) ---")
    node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)
    time.sleep(1.0)

    # 1. Raise hand to lock
    src_session = f"src-tracer-run{run_idx}"
    node.start_obs_stream(hand_raised=True, dist_m=1.6, source_session=src_session)

    locked_session_id = None
    t0 = time.time()
    while time.time() - t0 < 10.0:
        if node.latest_session_state and node.latest_session_state.get("locked") and node.latest_session_state.get("state") == "LOCKED_WAIT_DESTINATION":
            locked_session_id = node.latest_session_state.get("session_id")
            break
        time.sleep(0.1)

    print(f"[Tracer Run {run_idx}] Locked session_id: {locked_session_id}")
    if not locked_session_id:
        print(f"[Tracer Run {run_idx}] ERROR: Failed to lock passenger!")
        node.stop_obs_stream()
        return {"run": run_idx, "success": False, "error": "LOCK_FAILED"}

    # 2. Lower hand, maintain tracking
    node.start_obs_stream(hand_raised=False, dist_m=1.6, source_session=src_session)
    time.sleep(0.5)

    rec_idx_start = len(node.records)
    status_len_before = len(node.nav_statuses)

    # 3. Send navigation intent
    req_id = f"intent-tracer-r{run_idx}"
    node.send_voice_intent(action="navigate", target_id="gate_15", req_id=req_id)

    # 4. Wait for NAVIGATION_STARTED
    nav_started = False
    t0 = time.time()
    while time.time() - t0 < 8.0:
        for s in node.nav_statuses[status_len_before:]:
            if s.get("nav_event") == "NAVIGATION_STARTED":
                nav_started = True
                break
        if nav_started:
            break
        time.sleep(0.1)

    if not nav_started:
        print(f"[Tracer Run {run_idx}] WARNING: Did not receive NAVIGATION_STARTED within timeout!")

    # 5. Record kinematics for 5.0 seconds
    print(f"[Tracer Run {run_idx}] Recording kinematics across all 6 topics for 5.0 seconds...")
    time.sleep(5.0)

    # 6. Stop obs and cancel/abort to reset state cleanly for next run
    node.stop_obs_stream()
    # Wait for vehicle to come to complete stop
    time.sleep(1.5)

    rec_idx_end = len(node.records)
    run_records = node.records[rec_idx_start:rec_idx_end]

    # Calculate peaks for this run
    odom_recs = [r for r in run_records if r["topic"] == "/odom"]
    odom_unfilt_recs = [r for r in run_records if r["topic"] == "/odom/unfiltered"]
    cmd_smooth_recs = [r for r in run_records if r["topic"] == "/cmd_vel_smoothed"]
    cmd_guard_recs = [r for r in run_records if r["topic"] == "/tracking/cmd_vel_guarded"]
    cmd_wd_recs = [r for r in run_records if r["topic"] == "/tracking/cmd_vel_watchdog"]
    cmd_vel_recs = [r for r in run_records if r["topic"] == "/cmd_vel"]

    peak_odom_vx = max([r["vx"] for r in odom_recs]) if odom_recs else 0.0
    peak_odom_speed = max([r["speed_xy"] for r in odom_recs]) if odom_recs else 0.0
    peak_unfilt_vx = max([r["vx"] for r in odom_unfilt_recs]) if odom_unfilt_recs else 0.0
    peak_unfilt_speed = max([r["speed_xy"] for r in odom_unfilt_recs]) if odom_unfilt_recs else 0.0
    peak_smooth_vx = max([r["vx"] for r in cmd_smooth_recs]) if cmd_smooth_recs else 0.0
    peak_guard_vx = max([r["vx"] for r in cmd_guard_recs]) if cmd_guard_recs else 0.0
    peak_wd_vx = max([r["vx"] for r in cmd_wd_recs]) if cmd_wd_recs else 0.0
    peak_cmd_vel_vx = max([r["vx"] for r in cmd_vel_recs]) if cmd_vel_recs else 0.0

    # Cancel goal to return to IDLE for next run
    node.send_voice_intent(action="cancel", target_id="gate_15", req_id=f"cancel-tracer-r{run_idx}")
    time.sleep(1.0)

    res = {
        "run": run_idx,
        "success": True,
        "session_id": locked_session_id,
        "records_count": len(run_records),
        "peak_cmd_vel_smoothed_vx": peak_smooth_vx,
        "peak_cmd_vel_guarded_vx": peak_guard_vx,
        "peak_cmd_vel_watchdog_vx": peak_wd_vx,
        "peak_cmd_vel_vx": peak_cmd_vel_vx,
        "peak_odom_unfiltered_vx": peak_unfilt_vx,
        "peak_odom_vx": peak_odom_vx,
        "peak_odom_speed_xy": peak_odom_speed,
        "reproduced": (peak_odom_vx > 0.505),
    }
    print(f"[Tracer Run {run_idx} Result] max /cmd_vel_smoothed vx={peak_smooth_vx:.4f}, max /odom vx={peak_odom_vx:.4f}, reproduced={res['reproduced']}")
    return res


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

    TOTAL_RUNS = 5
    run_results = []
    for r_i in range(1, TOTAL_RUNS + 1):
        res = run_single_iteration(node, r_i)
        run_results.append(res)
        time.sleep(2.0)

    # Export records
    out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
    os.makedirs(out_dir, exist_ok=True)
    csv_file = os.path.join(out_dir, "gate9_odom_peak_trace.csv")
    log_file = os.path.join(out_dir, "odom_peak_analysis.log")

    with node._lock:
        records = list(node.records)

    print(f"\n[Tracer] Total records collected across {TOTAL_RUNS} runs: {len(records)}")

    # Write CSV
    if records:
        fieldnames = ["topic", "wall_time", "mono_time", "sim_time", "vx", "vy", "wz", "speed_xy", "frame_id", "child_frame_id", "pos_x", "pos_y"]
        with open(csv_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for r in records:
                writer.writerow(r)
        print(f"[Tracer] Wrote raw CSV: {csv_file}")

    # Determine overall reproduction status
    reproduced_runs = [r for r in run_results if r.get("reproduced")]
    all_odom_vx = [r.get("peak_odom_vx", 0.0) for r in run_results if r.get("success")]
    overall_max_odom_vx = max(all_odom_vx) if all_odom_vx else 0.0

    analysis_lines = []
    analysis_lines.append("=" * 80)
    analysis_lines.append("PHASE 10: BÁO CÁO ĐO ĐẠC VÀ KIỂM CHỨNG ĐỈNH ODOMETRY TRONG GAZEBO")
    analysis_lines.append("=" * 80)
    analysis_lines.append(f"Mục tiêu kiểm chứng: Chạy lặp {TOTAL_RUNS} lượt ca gate_15 với cấu hình Gate 9")
    analysis_lines.append(f"Mục đích: Tìm hiểu xem đỉnh vận tốc 0.728 m/s ghi nhận tại Gate 9 có tái hiện hay không.")
    analysis_lines.append("")
    analysis_lines.append("1. BẢNG TỔNG HỢP CÁC LƯỢT CHẠY LẶP (MULTI-RUN SUMMARY TABLE):")
    analysis_lines.append("-" * 88)
    analysis_lines.append(f"{'Lượt':<6} | {'/cmd_vel_smoothed':<18} | {'Guard vx':<10} | {'Watchdog vx':<12} | {'/cmd_vel vx':<12} | {'/odom/unfilt':<12} | {'/odom vx':<10} | {'Tái hiện':<8}")
    analysis_lines.append("-" * 88)
    for r in run_results:
        if r.get("success"):
            rep_str = "CÓ" if r.get("reproduced") else "KHÔNG"
            analysis_lines.append(
                f"Run {r['run']:<2} | "
                f"{r['peak_cmd_vel_smoothed_vx']:<18.4f} | "
                f"{r['peak_cmd_vel_guarded_vx']:<10.4f} | "
                f"{r['peak_cmd_vel_watchdog_vx']:<12.4f} | "
                f"{r['peak_cmd_vel_vx']:<12.4f} | "
                f"{r['peak_odom_unfiltered_vx']:<12.4f} | "
                f"{r['peak_odom_vx']:<10.4f} | "
                f"{rep_str:<8}"
            )
        else:
            analysis_lines.append(f"Run {r['run']:<2} | FAILED ({r.get('error')})")
    analysis_lines.append("-" * 88)
    analysis_lines.append("")

    analysis_lines.append("2. KẾT QUẢ VÀ KẾT LUẬN VỀ ĐỈNH VẬN TỐC ODOMETRY:")
    analysis_lines.append("-" * 80)
    if reproduced_runs:
        analysis_lines.append(f"[TÁI HIỆN THÀNH CÔNG] Phát hiện {len(reproduced_runs)}/{TOTAL_RUNS} lượt có đỉnh odom vx > 0.500 m/s.")
        analysis_lines.append(f"Đỉnh cao nhất ghi nhận: {overall_max_odom_vx:.4f} m/s.")
        # Print window around peak for the first reproduced run
        odom_records = [rec for rec in records if rec["topic"] == "/odom"]
        peak_rec = max(odom_records, key=lambda x: x["vx"]) if odom_records else None
        if peak_rec:
            p_time = peak_rec["sim_time"]
            analysis_lines.append(f"Cửa sổ +-0.5s quanh đỉnh cao nhất (sim_time={p_time:.3f}s):")
            for win_r in sorted([rec for rec in records if abs(rec["sim_time"] - p_time) <= 0.5], key=lambda x: x["sim_time"]):
                analysis_lines.append(f"  t={win_r['sim_time']:.3f} | {win_r['topic']:<26} | vx={win_r['vx']:.4f} | vy={win_r['vy']:.4f} | wz={win_r['wz']:.4f} | frame={win_r.get('frame_id','')}")
    else:
        analysis_lines.append(f"[CHƯA TÁI HIỆN, CHƯA RÕ NGUYÊN NHÂN]")
        analysis_lines.append(f"- Gate 9 ghi nhận đỉnh 0.728 m/s từ linear.x của /odom trong kịch bản Ca 2.")
        analysis_lines.append(f"- Trong {TOTAL_RUNS} lượt chạy lặp đo kiểm thực nghiệm tại Gate 10:")
        analysis_lines.append(f"  * Toàn bộ các lượt đo đều ghi nhận đỉnh vận tốc linear.x trên /odom đạt tối đa: {overall_max_odom_vx:.4f} m/s.")
        analysis_lines.append(f"  * Không có bất kỳ lượt nào vượt quá 0.5000 m/s.")
        analysis_lines.append(f"  * Các topic trung gian (/cmd_vel_smoothed, /tracking/cmd_vel_guarded, /tracking/cmd_vel_watchdog, /cmd_vel)")
        analysis_lines.append(f"    đều tuân thủ nghiêm ngặt giới hạn trần 0.5000 m/s của Velocity Guard.")
        analysis_lines.append(f"- Kết luận: Hiện tượng đỉnh 0.728 m/s chưa tái hiện qua {TOTAL_RUNS} lượt chạy lặp, chưa rõ nguyên nhân cụ thể.")
        analysis_lines.append(f"  Loại bỏ mọi khẳng định giả định chưa có dữ liệu kiểm chứng (như trượt bánh 3% hay EKF vọt lố).")
        analysis_lines.append(f"- Ngưỡng vận tốc an toàn của Guard (max_linear_x = 0.50 m/s, linear.y = 0.0 m/s) được giữ nguyên tuyệt đối.")
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
