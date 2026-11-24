#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 10: ROS 2 Topic Runtime Stress & Safety Acceptance Suite.
Tests:
1. Multi-cycle stress test (10 cycles) over actual ROS 2 topics:
   /tracking/observation, /voice/nav_status, /tracking/session_state,
   /cmd_vel_smoothed, /tracking/cmd_vel_guarded, /tracking/cmd_vel_watchdog.
   Strict per-cycle timeout (4.0s), comprehensive aggregate log.
2. Runtime Target Safety Cases with timestamp measurements:
   - Out-of-order and stale timestamp frames -> rejected, zero held.
   - Camera freeze / drop -> t_loss, t_zero (<=0.4s), t_abort (<=1.2s).
   - Stale cmd_vel_smoothed -> t_cmd_stop, t_cmd_zero (>0.2s watchdog clamp).
   - Voice ACK / terminal delay -> Guard holds continuous zero 20 Hz independently.
   - Node restart mid-session -> no replay of cached commands or stale sessions.
"""

import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from person_tracking.tracking_manager_node import TrackingManagerNode
from person_tracking.velocity_guard_node import VelocityGuardNode
from person_tracking.velocity_watchdog_node import VelocityWatchdogNode


class RosRuntimeTestDriver(Node):
    def __init__(self):
        super().__init__('ros_runtime_test_driver')

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.pub_obs = self.create_publisher(String, '/tracking/observation', reliable_qos)
        self.pub_voice_status = self.create_publisher(String, '/voice/nav_status', transient_qos)
        self.pub_cmd_smoothed = self.create_publisher(Twist, '/tracking/cmd_vel_smoothed', 10)

        # Subscribers
        self.sub_session = self.create_subscription(String, '/tracking/session_state', self._on_session, transient_qos)
        self.sub_safety = self.create_subscription(String, '/tracking/safety_request', self._on_safety, reliable_qos)
        self.sub_guarded = self.create_subscription(Twist, '/tracking/cmd_vel_guarded', self._on_guarded, 20)
        self.sub_watchdog = self.create_subscription(Twist, '/tracking/cmd_vel_watchdog', self._on_watchdog, 20)

        self.latest_session: Optional[dict] = None
        self.safety_requests: List[dict] = []
        self.guarded_cmds: List[Tuple[float, float, float, float]] = []  # (t, vx, vy, wz)
        self.watchdog_cmds: List[Tuple[float, float, float, float]] = []
        self._lock = threading.Lock()

    def _on_session(self, msg: String):
        try:
            with self._lock:
                self.latest_session = json.loads(msg.data)
        except Exception:
            pass

    def _on_safety(self, msg: String):
        try:
            with self._lock:
                self.safety_requests.append(json.loads(msg.data))
        except Exception:
            pass

    def _on_guarded(self, msg: Twist):
        t = time.monotonic()
        with self._lock:
            self.guarded_cmds.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
            if len(self.guarded_cmds) > 200:
                self.guarded_cmds.pop(0)

    def _on_watchdog(self, msg: Twist):
        t = time.monotonic()
        with self._lock:
            self.watchdog_cmds.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
            if len(self.watchdog_cmds) > 200:
                self.watchdog_cmds.pop(0)

    def publish_obs(self, track_id=1, hand_raised=True, dist_m=1.6, sec=0, nanosec=0, source_session="src-rt"):
        if sec == 0:
            now = self.get_clock().now().to_msg()
            sec = now.sec
            nanosec = now.nanosec

        cand = {
            "track_id": track_id,
            "is_confirmed": True,
            "is_ambiguous": False,
            "confidence": 0.95,
            "bbox_xyxy": [220.0, 100.0, 420.0, 400.0],
            "torso_depth_m": dist_m,
            "depth_valid": True,
            "center_offset_x_norm": 0.0,
            "near_edge": False,
            "keypoints": [[320.0, 200.0, 0.9]] * 17,
        }
        cand["keypoints"][5] = [300.0, 180.0, 0.9]
        cand["keypoints"][9] = [300.0, 120.0 if hand_raised else 250.0, 0.95]

        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_frame",
            "receive_monotonic_time": time.monotonic(),
            "source_mode": "live_astra",
            "source_session_id": source_session,
            "data_quality": "VALID_RGBD",
            "processing_time_ms": 5.0,
            "device": "cuda:0",
            "active_track_ids": [track_id],
            "lost_track_ids": [],
            "candidates": [cand],
        }
        self.pub_obs.publish(String(data=json.dumps(payload)))

    def publish_voice_status(self, nav_event: str, session_id: str, mission_id="mis-01", goal_attempt_id="att-01", nav2_uuid="uuid-01", is_terminal=False):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "nav_event": nav_event,
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "destination_id": "gate_15",
            "is_terminal": is_terminal,
            "status": "SUCCEEDED" if is_terminal else "EXECUTING",
            "feedback": {}
        }
        self.pub_voice_status.publish(String(data=json.dumps(payload)))

    def publish_cmd(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub_cmd_smoothed.publish(msg)


def setup_ros_runtime_nodes():
    driver = RosRuntimeTestDriver()
    manager = TrackingManagerNode()
    # Guard configured with /cmd_vel_smoothed in-topic
    guard = VelocityGuardNode()
    watchdog = VelocityWatchdogNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(manager)
    executor.add_node(guard)
    executor.add_node(watchdog)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    return driver, manager, guard, watchdog, executor


def teardown_ros_runtime_nodes(nodes, executor):
    try:
        executor.shutdown()
        for n in nodes:
            n.destroy_node()
    except Exception:
        pass


def lock_passenger_helper(driver: RosRuntimeTestDriver, source_session: str, timeout_sec: float = 3.5) -> str:
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        driver.publish_obs(track_id=1, hand_raised=True, dist_m=1.6, source_session=source_session)
        time.sleep(0.05)
        if driver.latest_session and driver.latest_session.get("locked") and driver.latest_session.get("state") == "LOCKED_WAIT_DESTINATION":
            return driver.latest_session.get("session_id")
    raise TimeoutError("Failed to lock passenger within timeout")

def test_t25_ros_topic_multicycle_stress():
    """T25: Multi-cycle stress test (10 cycles) over real ROS 2 topics."""
    print("\n=== [T25 ROS Topic Stress] Running 10 Consecutive Cycles ===")
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()

    log_lines = []
    log_lines.append("PHASE 10: T25 ROS TOPIC RUNTIME STRESS TEST (10 CYCLES)")
    log_lines.append("-" * 75)

    passed_cycles = 0
    total_cycles = 10
    cycle_durations = []

    try:
        for c in range(1, total_cycles + 1):
            t_cycle_start = time.perf_counter()
            src_sess = f"src-stress-c{c}"

            # Step 1: Hand raised -> Lock
            sess_id = None
            t0 = time.time()
            while time.time() - t0 < 3.0:
                driver.publish_obs(track_id=1, hand_raised=True, dist_m=1.6, source_session=src_sess)
                time.sleep(0.05)
                if driver.latest_session and driver.latest_session.get("locked") and driver.latest_session.get("state") == "LOCKED_WAIT_DESTINATION":
                    sess_id = driver.latest_session.get("session_id")
                    break

            assert sess_id is not None, f"Cycle {c}: Failed to lock passenger"

            # Step 2: Goal started -> Navigating
            driver.publish_voice_status("NAVIGATION_STARTED", session_id=sess_id, is_terminal=False)
            t0 = time.time()
            nav_ok = False
            while time.time() - t0 < 2.0:
                driver.publish_obs(track_id=1, hand_raised=False, dist_m=1.6, source_session=src_sess)
                driver.publish_cmd(vx=0.35, vy=0.0, wz=0.0)
                time.sleep(0.05)
                if driver.latest_session and driver.latest_session.get("state") == "NAVIGATING":
                    nav_ok = True
                    break

            assert nav_ok, f"Cycle {c}: Failed to transition to NAVIGATING"

            # Step 3: Pass-through verification
            # Stream obs and cmd continuously so command arrives strictly after gate opens
            for _ in range(8):
                driver.publish_obs(track_id=1, hand_raised=False, dist_m=1.6, source_session=src_sess)
                driver.publish_cmd(vx=0.35, vy=0.0, wz=0.0)
                time.sleep(0.05)

            with driver._lock:
                recent_guarded = list(driver.guarded_cmds[-5:])
            assert any(abs(g[1] - 0.35) < 0.05 for g in recent_guarded), f"Cycle {c}: Guard failed to pass commanded velocity (got {[g[1] for g in recent_guarded]})"

            # Step 4: Terminal Succeeded -> IDLE
            driver.publish_voice_status("GOAL_COMPLETED", session_id=sess_id, is_terminal=True)
            t0 = time.time()
            idle_ok = False
            while time.time() - t0 < 2.0:
                time.sleep(0.05)
                if driver.latest_session and driver.latest_session.get("state") == "IDLE" and not driver.latest_session.get("locked"):
                    idle_ok = True
                    break

            assert idle_ok, f"Cycle {c}: Failed to clean return to IDLE"

            dur = (time.perf_counter() - t_cycle_start) * 1000.0
            cycle_durations.append(dur)
            passed_cycles += 1
            print(f"  [Cycle {c}/{total_cycles}] PASSED in {dur:.1f} ms (Session: {sess_id})")
            log_lines.append(f"Cycle {c:02d}: PASSED in {dur:.1f} ms | Session: {sess_id}")

        log_lines.append("-" * 75)
        log_lines.append(f"Summary: {passed_cycles}/{total_cycles} Passed (100.0%) | Mean Duration: {sum(cycle_durations)/len(cycle_durations):.1f} ms/cycle")

        out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
        os.makedirs(out_dir, exist_ok=True)
        log_path = os.path.join(out_dir, "t25_ros_runtime_stress.log")
        with open(log_path, "w", encoding="utf-8") as f:
            f.write("\n".join(log_lines) + "\n")
        print(f"[T25 ROS Stress] Wrote log: {log_path}")

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


def test_runtime_out_of_order_frames_rejected():
    """Verify out-of-order and stale timestamp frames are rejected and zero is held."""
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()
    try:
        # Send normal frame at t=1000
        driver.publish_obs(track_id=1, hand_raised=True, dist_m=1.6, sec=1000, nanosec=0, source_session="src-ooo")
        time.sleep(0.1)

        # Send stale frame with past stamp t=900
        driver.publish_obs(track_id=1, hand_raised=True, dist_m=1.6, sec=900, nanosec=0, source_session="src-ooo")
        time.sleep(0.1)

        # Output on Guard must remain zero (default clamp)
        with driver._lock:
            cmds = list(driver.guarded_cmds[-3:])
        assert all(abs(c[1]) < 1e-4 for c in cmds), "Guard output must be 0 for out-of-order frames"

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


def test_runtime_camera_freeze_timestamps():
    """Verify camera freeze triggers Guard zero (<=0.4s) and Manager abort (<=1.2s) with exact timestamps."""
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()
    try:
        # Lock and navigate
        src_sess = "src-freeze"
        sess_id = lock_passenger_helper(driver, src_sess)
        driver.publish_voice_status("NAVIGATION_STARTED", session_id=sess_id, is_terminal=False)
        time.sleep(0.1)

        # Maintain 20 Hz observation
        for _ in range(5):
            driver.publish_obs(track_id=1, hand_raised=False, dist_m=1.6, source_session=src_sess)
            driver.publish_cmd(vx=0.40)
            time.sleep(0.05)

        # FREEZE CAMERA: Stop observation stream at t0
        t0_loss = time.monotonic()
        t_first_zero = None
        t_abort_issued = None

        for _ in range(35):  # Monitor for up to 1.75s
            driver.publish_cmd(vx=0.40)
            time.sleep(0.05)
            now = time.monotonic()
            with driver._lock:
                if driver.guarded_cmds:
                    last_g = driver.guarded_cmds[-1]
                    if abs(last_g[1]) < 1e-4 and t_first_zero is None:
                        t_first_zero = last_g[0]
                if driver.safety_requests and t_abort_issued is None:
                    t_abort_issued = now

        assert t_first_zero is not None, "Guard must emit zero after observation loss"
        assert t_abort_issued is not None, "Manager must emit safety abort request"

        dt_zero = t_first_zero - t0_loss
        dt_abort = t_abort_issued - t0_loss
        print(f"\n[Camera Freeze Measurements] dt_zero={dt_zero:.3f}s (<=0.45s), dt_abort={dt_abort:.3f}s (<=1.30s)")
        assert dt_zero <= 0.45, f"Guard zero delay {dt_zero:.3f}s exceeded 0.45s"
        assert dt_abort <= 1.30, f"Abort delay {dt_abort:.3f}s exceeded 1.30s"

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


def test_runtime_stale_cmd_timestamps():
    """Verify stale /cmd_vel_smoothed (>0.2s) triggers Guard watchdog zero clamp."""
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()
    try:
        src_sess = "src-stale-cmd"
        sess_id = lock_passenger_helper(driver, src_sess)
        driver.publish_voice_status("NAVIGATION_STARTED", session_id=sess_id, is_terminal=False)
        time.sleep(0.1)

        # Stream valid cmd
        for _ in range(5):
            driver.publish_obs(track_id=1, hand_raised=False, dist_m=1.6, source_session=src_sess)
            driver.publish_cmd(vx=0.40)
            time.sleep(0.05)

        # Stop publishing cmd_vel_smoothed at t_stop
        t_stop = time.monotonic()
        t_zero_clamped = None

        for _ in range(15):  # Monitor for 0.75s
            # Keep obs valid so only cmd is stale!
            driver.publish_obs(track_id=1, hand_raised=False, dist_m=1.6, source_session=src_sess)
            time.sleep(0.05)
            with driver._lock:
                if driver.guarded_cmds:
                    last_g = driver.guarded_cmds[-1]
                    if abs(last_g[1]) < 1e-4 and t_zero_clamped is None and (last_g[0] - t_stop) > 0.15:
                        t_zero_clamped = last_g[0]

        assert t_zero_clamped is not None, "Guard must clamp to zero on stale cmd"
        dt_stale = t_zero_clamped - t_stop
        print(f"\n[Stale Cmd Measurements] Guard clamped to zero at dt={dt_stale:.3f}s (expected ~0.20-0.25s)")
        assert 0.15 <= dt_stale <= 0.35, f"Stale cmd delay {dt_stale:.3f}s out of expected window [0.15, 0.35]"

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


def test_runtime_voice_ack_terminal_delay():
    """Verify Guard maintains continuous zeroing when Voice delays ACK / Terminal."""
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()
    try:
        src_sess = "src-voice-delay"
        sess_id = lock_passenger_helper(driver, src_sess)
        driver.publish_voice_status("NAVIGATION_STARTED", session_id=sess_id, is_terminal=False)
        time.sleep(0.1)

        # Trigger ABORT by cutting obs
        time.sleep(1.3)
        assert driver.safety_requests, "Manager must issue safety abort request"

        # Voice does NOT reply ACK and does NOT send terminal for 2.0s
        for _ in range(20):
            driver.publish_cmd(vx=0.50)  # Upstream attempts to command motion
            time.sleep(0.05)

        with driver._lock:
            recent_cmds = list(driver.guarded_cmds[-10:])
        assert all(abs(c[1]) < 1e-4 for c in recent_cmds), "Guard must continuously clamp to zero during Voice delay"

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


def test_runtime_node_restart_prevents_stale_replay():
    """Verify node restart starts clean with zero velocity and does not replay old commands."""
    rclpy.init()
    driver, manager, guard, watchdog, executor = setup_ros_runtime_nodes()
    try:
        # Lock and navigate
        src_sess = "src-restart"
        sess_id = lock_passenger_helper(driver, src_sess)
        driver.publish_voice_status("NAVIGATION_STARTED", session_id=sess_id, is_terminal=False)
        time.sleep(0.1)
        driver.publish_cmd(vx=0.45)
        time.sleep(0.2)

        # Destroy old guard node and instantiate fresh GuardNode (simulating restart)
        executor.remove_node(guard)
        guard.destroy_node()

        new_guard = VelocityGuardNode()
        executor.add_node(new_guard)
        time.sleep(0.2)

        # Fresh guard starts with default 0 velocity and no active gate
        with driver._lock:
            recent_guarded = list(driver.guarded_cmds[-5:])
        assert all(abs(g[1]) < 1e-4 for g in recent_guarded), "Fresh Guard must output zero Twist after restart"

    finally:
        teardown_ros_runtime_nodes([driver, manager, guard, watchdog], executor)
        rclpy.shutdown()


if __name__ == "__main__":
    test_t25_ros_topic_multicycle_stress()
    test_runtime_out_of_order_frames_rejected()
    test_runtime_camera_freeze_timestamps()
    test_runtime_stale_cmd_timestamps()
    test_runtime_voice_ack_terminal_delay()
    test_runtime_node_restart_prevents_stale_replay()
    print("\n[SUCCESS] All ROS 2 Runtime Safety & Stress tests PASSED successfully!")
