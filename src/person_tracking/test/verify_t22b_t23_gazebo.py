#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 10: Dedicated Gazebo Verification of T22B (Watchdog SIGKILL) and T23 (Collision Monitor Veto).
Records exact timestamps for:
- Obstacle injection / SIGKILL signal
- First zero command emitted by Watchdog / Collision Monitor
- Final physical halt on /odom
"""

import json
import math
import os
import signal
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


class GazeboRecheckDriver(Node):
    def __init__(self):
        super().__init__(
            "gazebo_recheck_driver",
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
        self.sub_say = self.create_subscription(String, "/voice/say", self._on_say, 10)

        self.latest_sim_time = 0.0
        self.latest_clock_msg: Optional[Clock] = None
        self.latest_session: Optional[dict] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_guarded_cmd: Optional[Twist] = None
        self.latest_watchdog_cmd: Optional[Twist] = None
        self.latest_cmd_vel: Optional[Twist] = None

        self.active_scan_ranges = [10.0] * 361
        self.scan_timer = self.create_timer(0.05, self._publish_scan_loop)

        self._obs_running = False
        self._obs_hand_raised = False
        self._obs_dist_m = 1.6
        self._obs_session = "src-recheck"
        self._last_obs_stamp = (0, 0)
        self.obs_timer = self.create_timer(0.05, self._obs_tick)

    def _on_clock(self, msg: Clock):
        self.latest_clock_msg = msg
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_cmd_smoothed(self, msg: Twist):
        pass

    def _on_cmd_guarded(self, msg: Twist):
        self.latest_guarded_cmd = msg

    def _on_cmd_watchdog(self, msg: Twist):
        self.latest_watchdog_cmd = msg

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _on_session(self, msg: String):
        try:
            self.latest_session = json.loads(msg.data)
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
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = list(self.active_scan_ranges)
        self.pub_test_scan.publish(scan)

    def set_obstacle(self, dist_m=0.25):
        # Front sector +/- 30 deg
        ranges = [10.0] * 361
        for i in range(150, 211):
            ranges[i] = dist_m
        self.active_scan_ranges = ranges

    def clear_obstacle(self):
        self.active_scan_ranges = [10.0] * 361

    def start_obs(self, hand_raised=True, dist_m=1.6, session="src-recheck"):
        self._obs_hand_raised = hand_raised
        self._obs_dist_m = dist_m
        self._obs_session = session
        self._obs_running = True

    def stop_obs(self):
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
        cand["keypoints"][5] = [300.0, 180.0, 0.9]
        cand["keypoints"][9] = [300.0, 120.0 if self._obs_hand_raised else 250.0, 0.95]

        obs = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_frame",
            "receive_monotonic_time": time.monotonic(),
            "source_mode": "live_astra",
            "source_session_id": self._obs_session,
            "data_quality": "VALID_RGBD",
            "processing_time_ms": 5.0,
            "device": "cuda:0",
            "active_track_ids": [1],
            "lost_track_ids": [],
            "candidates": [cand],
        }
        self.pub_obs.publish(String(data=json.dumps(obs)))

    def send_intent(self, action="navigate", target_id="gate_15", req_id="req-recheck"):
        payload = {"action": action, "target_id": target_id, "request_id": req_id}
        for _ in range(3):
            self.pub_intent.publish(String(data=json.dumps(payload)))
            time.sleep(0.05)


def run_gazebo_recheck():
    rclpy.init()
    node = GazeboRecheckDriver()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    log_lines = []
    log_lines.append("=" * 80)
    log_lines.append("PHASE 10: GAZEBO RUNTIME RE-VERIFICATION OF T23 AND T22B")
    log_lines.append("=" * 80)

    print("[Gazebo Recheck] Waiting for /clock and simulation system to be ready...")
    for _ in range(60):
        if node.latest_sim_time > 0.5 and node.latest_odom is not None:
            break
        time.sleep(0.5)

    print(f"[Gazebo Recheck] Ready! sim_time={node.latest_sim_time:.2f}s")
    node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)
    time.sleep(1.5)

    # -------------------------------------------------------------------------
    # TEST 1: T23 Collision Monitor Priority Laser Veto
    # -------------------------------------------------------------------------
    print("\n[Recheck Step 1] Testing T23: Collision Monitor Priority Stop on Laser Obstacle...")
    node.start_obs(hand_raised=True, dist_m=1.6, session="src-t23-recheck")
    sess_id = None
    t0 = time.time()
    while time.time() - t0 < 8.0:
        if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("state") == "LOCKED_WAIT_DESTINATION":
            sess_id = node.latest_session.get("session_id")
            break
        time.sleep(0.1)

    assert sess_id is not None, "Failed to lock passenger for T23"
    node.start_obs(hand_raised=False, dist_m=1.6, session="src-t23-recheck")
    time.sleep(0.5)

    node.send_intent(action="navigate", target_id="gate_15", req_id="intent-t23")

    # Wait for vehicle to move
    print("  Waiting for robot to start moving...")
    moving = False
    for _ in range(40):
        if node.latest_cmd_vel and abs(node.latest_cmd_vel.linear.x) > 0.05 and node.latest_odom and abs(node.latest_odom.twist.twist.linear.x) > 0.02:
            moving = True
            break
        time.sleep(0.1)

    assert moving, "Robot did not move prior to T23 obstacle injection"
    v_pre_t23 = node.latest_cmd_vel.linear.x
    o_pre_t23 = node.latest_odom.twist.twist.linear.x
    print(f"  Robot moving! cmd_vel vx={v_pre_t23:.3f} m/s, odom vx={o_pre_t23:.3f} m/s")

    # Inject obstacle at 0.25m
    t_inject_mono = time.monotonic()
    t_inject_sim = node.latest_sim_time
    node.set_obstacle(dist_m=0.25)

    t_cmd_zero_mono = None
    t_cmd_zero_sim = None
    t_odom_halt_mono = None
    t_odom_halt_sim = None

    for _ in range(40):  # Monitor for up to 2.0s
        time.sleep(0.05)
        now_mono = time.monotonic()
        now_sim = node.latest_sim_time

        c_vx = abs(node.latest_cmd_vel.linear.x) if node.latest_cmd_vel else 0.0
        o_vx = abs(node.latest_odom.twist.twist.linear.x) if node.latest_odom else 0.0

        if c_vx < 1e-3 and t_cmd_zero_mono is None:
            t_cmd_zero_mono = now_mono
            t_cmd_zero_sim = now_sim

        if o_vx < 0.01 and t_cmd_zero_mono is not None and t_odom_halt_mono is None:
            t_odom_halt_mono = now_mono
            t_odom_halt_sim = now_sim
            break

    assert t_cmd_zero_mono is not None, "Collision Monitor failed to clamp cmd_vel to zero"
    assert t_odom_halt_mono is not None, "Vehicle failed to halt on /odom"

    dt_cmd_zero_t23 = t_cmd_zero_mono - t_inject_mono
    dt_odom_halt_t23 = t_odom_halt_mono - t_inject_mono

    t23_pass = (dt_cmd_zero_t23 <= 0.25 and dt_odom_halt_t23 <= 2.50)
    print(f"  [T23 RESULT] PASS={t23_pass}: cmd_vel zero delay={dt_cmd_zero_t23:.3f}s, odom halt delay={dt_odom_halt_t23:.3f}s")
    log_lines.append("1. CA KIỂM THỬ T23 — COLLISION MONITOR PRIORITY LASER VETO:")
    log_lines.append("-" * 75)
    log_lines.append(f"- Trạng thái trước khi thử: Robot đang chạy, cmd_vel={v_pre_t23:.3f} m/s, odom={o_pre_t23:.3f} m/s")
    log_lines.append(f"- Thời điểm bơm vật cản laser (0.25m): mono={t_inject_mono:.3f}s, sim={t_inject_sim:.3f}s")
    log_lines.append(f"- Thời điểm Collision Monitor dập /cmd_vel về 0.000: mono={t_cmd_zero_mono:.3f}s, delay={dt_cmd_zero_t23:.3f}s")
    log_lines.append(f"- Thời điểm xe dừng hẳn trên /odom (vx < 0.01): mono={t_odom_halt_mono:.3f}s, delay={dt_odom_halt_t23:.3f}s (<=2.5s)")
    log_lines.append(f"- Kết quả T23: {'PASS' if t23_pass else 'FAIL'}")
    log_lines.append("")

    # Clean up T23
    node.clear_obstacle()
    node.send_intent(action="cancel", target_id="gate_15", req_id="cancel-t23")
    node.stop_obs()
    time.sleep(2.0)

    # -------------------------------------------------------------------------
    # TEST 2: T22B Guard Crash Without Handler Watchdog (SIGKILL)
    # -------------------------------------------------------------------------
    print("\n[Recheck Step 2] Testing T22B: Guard Crash Without Handler (SIGKILL on Guard PID)...")
    node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)
    time.sleep(1.0)

    node.start_obs(hand_raised=True, dist_m=1.6, session="src-t22b-recheck")
    sess_id_2 = None
    t0 = time.time()
    while time.time() - t0 < 8.0:
        if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("state") == "LOCKED_WAIT_DESTINATION":
            sess_id_2 = node.latest_session.get("session_id")
            break
        time.sleep(0.1)

    assert sess_id_2 is not None, "Failed to lock passenger for T22B"
    node.start_obs(hand_raised=False, dist_m=1.6, session="src-t22b-recheck")
    time.sleep(0.5)

    node.send_intent(action="navigate", target_id="gate_15", req_id="intent-t22b")

    # Wait for vehicle to move
    print("  Waiting for robot to start moving...")
    moving_t22b = False
    for _ in range(40):
        if node.latest_cmd_vel and abs(node.latest_cmd_vel.linear.x) > 0.05 and node.latest_odom and abs(node.latest_odom.twist.twist.linear.x) > 0.02:
            moving_t22b = True
            break
        time.sleep(0.1)

    assert moving_t22b, "Robot did not move prior to T22B SIGKILL"
    v_pre_t22b = node.latest_cmd_vel.linear.x
    o_pre_t22b = node.latest_odom.twist.twist.linear.x
    print(f"  Robot moving! cmd_vel vx={v_pre_t22b:.3f} m/s, odom vx={o_pre_t22b:.3f} m/s")

    # Find Guard PID via ps
    guard_pid = None
    import subprocess
    ps_out = subprocess.check_output(["pgrep", "-f", "escort_velocity_guard"]).decode().strip().split()
    if ps_out:
        guard_pid = int(ps_out[0])

    assert guard_pid is not None, "Could not locate escort_velocity_guard PID"
    print(f"  Target Guard PID located: {guard_pid}. Sending SIGKILL now!")

    t_kill_mono = time.monotonic()
    t_kill_sim = node.latest_sim_time
    os.kill(guard_pid, signal.SIGKILL)

    t_wd_zero_mono = None
    t_cmd_zero_mono = None
    t_odom_halt_mono = None

    for _ in range(50):  # Monitor for up to 2.5s
        time.sleep(0.05)
        now_mono = time.monotonic()

        w_vx = abs(node.latest_watchdog_cmd.linear.x) if node.latest_watchdog_cmd else 99.0
        c_vx = abs(node.latest_cmd_vel.linear.x) if node.latest_cmd_vel else 99.0
        o_vx = abs(node.latest_odom.twist.twist.linear.x) if node.latest_odom else 99.0

        if w_vx < 1e-3 and t_wd_zero_mono is None:
            t_wd_zero_mono = now_mono

        if c_vx < 1e-3 and t_cmd_zero_mono is None:
            t_cmd_zero_mono = now_mono

        if o_vx < 0.01 and t_cmd_zero_mono is not None and t_odom_halt_mono is None:
            t_odom_halt_mono = now_mono
            break

    assert t_wd_zero_mono is not None, "Watchdog failed to emit zero command"
    assert t_cmd_zero_mono is not None, "Collision monitor failed to clamp cmd_vel to zero after Guard kill"
    assert t_odom_halt_mono is not None, "Vehicle failed to halt on /odom after Guard kill"

    dt_wd_zero = t_wd_zero_mono - t_kill_mono
    dt_cmd_zero_t22b = t_cmd_zero_mono - t_kill_mono
    dt_odom_halt_t22b = t_odom_halt_mono - t_kill_mono

    t22b_pass = (dt_wd_zero <= 0.25 and dt_odom_halt_t22b <= 2.50)
    print(f"  [T22B RESULT] PASS={t22b_pass}: Watchdog zero delay={dt_wd_zero:.3f}s, cmd_vel zero delay={dt_cmd_zero_t22b:.3f}s, odom halt delay={dt_odom_halt_t22b:.3f}s")

    log_lines.append("2. CA KIỂM THỬ T22B — GUARD CRASH WITHOUT HANDLER WATCHDOG (SIGKILL):")
    log_lines.append("-" * 75)
    log_lines.append(f"- Trạng thái trước khi kill: Robot đang chạy, cmd_vel={v_pre_t22b:.3f} m/s, odom={o_pre_t22b:.3f} m/s")
    log_lines.append(f"- Thời điểm gửi SIGKILL vào PID Guard ({guard_pid}): mono={t_kill_mono:.3f}s, sim={t_kill_sim:.3f}s")
    log_lines.append(f"- Thời điểm Watchdog độc lập phát zero vào Collision Monitor: mono={t_wd_zero_mono:.3f}s, delay={dt_wd_zero:.3f}s (<=0.25s)")
    log_lines.append(f"- Thời điểm /cmd_vel về 0.000: mono={t_cmd_zero_mono:.3f}s, delay={dt_cmd_zero_t22b:.3f}s")
    log_lines.append(f"- Thời điểm xe dừng hẳn trên /odom (vx < 0.01): mono={t_odom_halt_mono:.3f}s, delay={dt_odom_halt_t22b:.3f}s (<=2.5s)")
    log_lines.append(f"- Kết quả T22B: {'PASS' if t22b_pass else 'FAIL'}")
    log_lines.append("=" * 80)

    out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
    os.makedirs(out_dir, exist_ok=True)
    recheck_log = os.path.join(out_dir, "t22b_t23_gazebo_recheck.log")
    with open(recheck_log, "w", encoding="utf-8") as f:
        f.write("\n".join(log_lines) + "\n")
    print(f"\n[Gazebo Recheck] Wrote log: {recheck_log}")

    try:
        executor.shutdown()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    run_gazebo_recheck()
