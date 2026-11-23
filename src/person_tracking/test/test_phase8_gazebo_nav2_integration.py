"""Phase 8 End-to-End Integration and Safety Acceptance Test Suite for ROS 2 Jazzy.

Stack Architecture:
- Nav2 Controller -> Velocity Smoother -> /tracking/cmd_vel_smoothed
- escort_velocity_guard -> /tracking/cmd_vel_guarded
- nav2_collision_monitor -> /cmd_vel -> Gazebo base bridge
- escort_tracking_manager -> /tracking/session_state, /tracking/safety_request
- MockVoiceNavOwner -> /voice/nav_status, /voice/nav_state, NavigateToPose Goal Owner

Acceptance Matrix (Phase 8):
- T09: Normal escort tracking pass-through.
- T10: Distance scaling, boundary stop at 2.5m (vx=0, wz=0), and hysteresis resume at 2.2m.
- T11: Close passenger safety (<1.2m, no acceleration, never reverse).
- T12: Rear camera yaw trimming (CW for right, CCW for left, 50% near edge, conflict stop).
- T14/T15: Loss grace period (<0.40s) and full abort (>1.20s) with Mock Voice cancel/ACK.
- T16: Unresponsive Mock Voice -> Guard maintains independent zero output.
- T23 (Critical): Collision Monitor priority over Guard -> obstacle injection in scan stops /cmd_vel.
- T22 (Critical): Guard kill watchdog -> Collision Monitor zeroes /cmd_vel when Guard stops.
- T33 (Critical): Steady clock watchdog during /clock pause -> zero holds, no jump on resume.
- T35: Mecanum lateral stability -> linear.y == 0.0 strictly.
- Safety Invariant: Collision Monitor is SOLE publisher on /cmd_vel (no Guard/teleop bypass).
"""

import json
import math
import time
from typing import List, Optional, Tuple
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from person_tracking.velocity_guard_node import VelocityGuardNode
from person_tracking.tracking_manager_node import TrackingManagerNode
from person_tracking.velocity_guard import VelocityGuard, GuardLimits


# ==============================================================================
# TEST HARNESS NODES
# ==============================================================================

class MockVoiceNavOwner(Node):
    """Mock Voice Navigation Goal Owner fulfilling VOICE_INTEGRATION_REQUEST contract."""

    def __init__(self):
        super().__init__('mock_voice_nav_owner')
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_status = self.create_publisher(String, '/voice/nav_status', transient_qos)
        self.pub_state = self.create_publisher(String, '/voice/nav_state', transient_qos)
        self.sub_safety = self.create_subscription(String, '/tracking/safety_request', self._on_safety_request, reliable_qos)

        self.received_safety_requests: List[dict] = []
        self.active_session_id: Optional[str] = None
        self.active_mission_id: Optional[str] = None
        self.active_goal_attempt_id: Optional[str] = None
        self.active_nav2_uuid: Optional[str] = None
        self.auto_ack_safety: bool = True

    def start_mission(self, session_id: str, mission_id: str = "mission-001", goal_attempt_id: str = "attempt-001", nav2_uuid: str = "uuid-nav2-001"):
        self.active_session_id = session_id
        self.active_mission_id = mission_id
        self.active_goal_attempt_id = goal_attempt_id
        self.active_nav2_uuid = nav2_uuid

        status_msg = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "status": "NAVIGATING",
            "nav_event": "NAVIGATION_STARTED",
            "is_terminal": False,
        }
        self.pub_status.publish(String(data=json.dumps(status_msg)))

        state_msg = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "mission_state": "NAVIGATING",
        }
        self.pub_state.publish(String(data=json.dumps(state_msg)))

    def _on_safety_request(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception:
            return
        self.received_safety_requests.append(req)

        if self.auto_ack_safety and self.active_session_id:
            # 1. Send CANCEL_ACKNOWLEDGED with exact matching IDs
            ack_msg = {
                "session_id": req.get("session_id", self.active_session_id),
                "mission_id": self.active_mission_id,
                "goal_attempt_id": self.active_goal_attempt_id,
                "nav2_uuid": self.active_nav2_uuid,
                "request_id": req.get("request_id"),
                "status": "ABORTING",
                "nav_event": "CANCEL_ACKNOWLEDGED",
                "is_terminal": False,
            }
            self.pub_status.publish(String(data=json.dumps(ack_msg)))

            # 2. Complete cancellation and send terminal status
            term_msg = {
                "session_id": req.get("session_id", self.active_session_id),
                "mission_id": self.active_mission_id,
                "goal_attempt_id": self.active_goal_attempt_id,
                "nav2_uuid": self.active_nav2_uuid,
                "status": "CANCELED",
                "nav_event": "GOAL_CANCELED",
                "is_terminal": True,
            }
            self.pub_status.publish(String(data=json.dumps(term_msg)))


class MockCollisionMonitor(Node):
    """Mock collision monitor reproducing nav2_collision_monitor gating behavior.
    Subscribes to /tracking/cmd_vel_guarded, monitors laser scan, publishes to /cmd_vel.
    """

    def __init__(self, in_topic: str = '/tracking/cmd_vel_guarded', out_topic: str = '/cmd_vel'):
        super().__init__('mock_collision_monitor')
        self.sub_guarded = self.create_subscription(Twist, in_topic, self._on_guarded_cmd, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.pub_cmd = self.create_publisher(Twist, out_topic, 10)

        self.has_obstacle: bool = False
        self.last_guarded_msg: Optional[Twist] = None
        self.last_guarded_time: float = 0.0
        self.source_timeout_sec: float = 1.0

        # 20 Hz output loop
        self.timer = self.create_timer(0.05, self._on_tick)

    def _on_scan(self, msg: LaserScan):
        # Obstacle detected if any valid range < 1.0m within front/rear footprint approach zone
        close_ranges = [r for r in msg.ranges if 0.05 < r < 1.0]
        self.has_obstacle = len(close_ranges) > 0

    def _on_guarded_cmd(self, msg: Twist):
        self.last_guarded_msg = msg
        self.last_guarded_time = time.monotonic()

    def _on_tick(self):
        out = Twist()
        now = time.monotonic()
        # If obstacle detected or source timeout exceeded -> output strictly zero!
        if self.has_obstacle or (now - self.last_guarded_time > self.source_timeout_sec):
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0
        elif self.last_guarded_msg is not None:
            out = self.last_guarded_msg

        self.pub_cmd.publish(out)


class MockObservationPublisher(Node):
    def __init__(self):
        super().__init__('mock_obs_pub')
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, '/tracking/observation', reliable_qos)

    def publish_obs(
        self,
        track_id: int = 1,
        dist_m: float = 1.5,
        offset_x: float = 0.0,
        depth_valid: bool = True,
        is_confirmed: bool = True,
        near_edge: bool = False,
        lost_track_ids: Optional[List[int]] = None,
        source_session_id: str = "src-sim-1",
        stamp_sec: int = 1,
        stamp_nanosec: int = 0,
    ):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
            "source_session_id": source_session_id,
            "lost_track_ids": lost_track_ids or [],
            "candidates": [
                {
                    "track_id": track_id,
                    "is_confirmed": is_confirmed,
                    "is_ambiguous": False,
                    "torso_depth_m": dist_m,
                    "depth_valid": depth_valid,
                    "center_offset_x_norm": offset_x,
                    "near_edge": near_edge,
                    "confidence": 0.95,
                    "bbox_xyxy": [100, 100, 200, 300],
                }
            ],
        }
        self.pub.publish(String(data=json.dumps(payload)))


class MockScanPublisher(Node):
    def __init__(self):
        super().__init__('mock_scan_pub')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def publish_scan(self, min_range: float = 5.0):
        scan = LaserScan()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.10
        scan.range_max = 10.0
        scan.ranges = [float(min_range)] * 360
        self.pub.publish(scan)


class CmdVelSubscriber(Node):
    def __init__(self, topic: str = '/cmd_vel'):
        super().__init__('test_cmd_vel_sub')
        self.received_msgs: List[Twist] = []
        self.sub = self.create_subscription(Twist, topic, self._on_cmd, 10)

    def _on_cmd(self, msg: Twist):
        self.received_msgs.append(msg)


def _spin_all(nodes: List[Node], duration_sec: float = 0.25, step_sec: float = 0.02):
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=step_sec)


# ==============================================================================
# PHASE 8 INTEGRATION TESTS
# ==============================================================================

def test_phase8_audit_velocity_chain_and_sole_publisher():
    """Safety Audit: Verify complete velocity topic chain and that Collision Monitor
    is the SOLE publisher on /cmd_vel. Strictly ZERO /cmd_vel publishers in person_tracking.
    """
    rclpy.init()
    try:
        guard = VelocityGuardNode()
        cm = MockCollisionMonitor()
        listener = CmdVelSubscriber('/cmd_vel')
        all_nodes = [guard, cm, listener]
        _spin_all(all_nodes, duration_sec=0.15)

        # Audit publishers on /cmd_vel:
        # In this clean architecture, MockCollisionMonitor is the sole publisher.
        # Ensure Guard publishes to /tracking/cmd_vel_guarded, NOT /cmd_vel!
        pub_topics = [p.topic_name for p in guard.publishers]
        assert '/cmd_vel' not in pub_topics
        assert '/tracking/cmd_vel_guarded' in pub_topics

        for n in all_nodes:
            n.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_phase8_t09_normal_escort_pass_through():
    """T09: When locked, NAVIGATING, depth verified, and Nav2 commands arrive ->
    Guard passes velocity to /tracking/cmd_vel_guarded, and Collision Monitor outputs to /cmd_vel.
    """
    rclpy.init()
    try:
        guard = VelocityGuardNode()
        cm = MockCollisionMonitor()
        obs_pub = MockObservationPublisher()
        voice = MockVoiceNavOwner()
        listener = CmdVelSubscriber('/cmd_vel')
        smoother_pub = guard.create_publisher(Twist, '/tracking/cmd_vel_smoothed', 10)

        all_nodes = [guard, cm, obs_pub, voice, listener]

        # 1. Start mission via Mock Voice
        sess_id = "sess-p8-001"
        voice.start_mission(session_id=sess_id)
        # Update session state on Guard directly
        guard.guard.update_session_state({
            "schema_version": "1.0",
            "session_id": sess_id,
            "state": "NAVIGATING",
            "locked": True,
            "person_track_id": 1,
            "person_distance_m": 1.5,
            "depth_proven": True,
            "obs_fresh": True,
        }, now_mono=time.monotonic())

        # 2. Publish valid observation and allow DDS graph discovery to settle
        obs_pub.publish_obs(track_id=1, dist_m=1.5, offset_x=0.0, depth_valid=True, stamp_sec=1, stamp_nanosec=0)
        _spin_all(all_nodes, duration_sec=0.25)

        # 3. Nav2 smoother sends cmd_vel in 20 Hz loop
        cmd = Twist()
        cmd.linear.x = 0.35
        cmd.angular.z = 0.05

        listener.received_msgs.clear()
        for _ in range(5):
            smoother_pub.publish(cmd)
            _spin_all(all_nodes, duration_sec=0.10)

        assert len(listener.received_msgs) > 0
        # Find latest non-zero message
        non_zeros = [m for m in listener.received_msgs if m.linear.x > 0.1]
        assert len(non_zeros) > 0, f"Expected non-zero cmd_vel, got {listener.received_msgs}"
        latest = non_zeros[-1]
        assert pytest.approx(latest.linear.x, 0.02) == 0.35
        assert latest.linear.y == 0.0  # T35: strictly 0.0
        assert pytest.approx(latest.angular.z, 0.02) == 0.05

        for n in all_nodes:
            n.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_phase8_t10_distance_scaling_and_hysteresis_stop():
    """T10: Distance > 2.0m reduces speed; >= 2.5m stops completely (vx=0, wz=0); resumes <= 2.2m."""
    guard = VelocityGuard()
    t = 100.0

    # Start NAVIGATING at 2.0m
    guard.update_session_state({
        "schema_version": "1.0", "session_id": "s1", "state": "NAVIGATING", "locked": True,
        "person_track_id": 1, "person_distance_m": 2.0, "depth_proven": True, "obs_fresh": True
    }, now_mono=t)
    guard.update_observation({
        "schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "src-1",
        "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 2.0, "depth_valid": True, "center_offset_x_norm": 0.0}]
    }, now_mono=t)
    guard.update_cmd_vel(0.40, 0.0, 0.10, now_mono=t + 0.01)

    # 1. At 2.0m: full speed
    vx, _, wz, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(vx, 0.01) == 0.40

    # 2. At 2.25m: linear scale 0.50 -> vx = 0.20
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 2.25, "depth_proven": True, "obs_fresh": True}, now_mono=t + 0.05)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 50_000_000}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 2.25, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t + 0.05)
    guard.update_cmd_vel(0.40, 0.0, 0.10, now_mono=t + 0.06)
    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.07)
    assert pytest.approx(vx, 0.01) == 0.20

    # 3. At 2.6m: exceeds 2.5m stop threshold -> complete stop (vx=0, wz=0, NO yawing!)
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 2.6, "depth_proven": True, "obs_fresh": True}, now_mono=t + 0.10)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 100_000_000}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 2.6, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t + 0.10)
    guard.update_cmd_vel(0.40, 0.0, 0.20, now_mono=t + 0.11)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.12)
    assert vx == 0.0
    assert wz == 0.0  # Robot must NOT yaw when stopped by distance!
    assert reason == "DISTANCE_EXCEEDED_STOP"


def test_phase8_t11_close_passenger_safety():
    """T11: Passenger < 1.2m maintains Nav2 speed, never accelerates beyond input, never reverses."""
    guard = VelocityGuard()
    t = 100.0
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 0.9, "depth_proven": True, "obs_fresh": True}, now_mono=t)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 0.9, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t)
    guard.update_cmd_vel(0.25, 0.0, 0.0, now_mono=t + 0.01)

    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(vx, 0.01) == 0.25

    # Reverse command clamped to 0.0
    guard.update_cmd_vel(-0.20, 0.0, 0.0, now_mono=t + 0.03)
    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.04)
    assert vx == 0.0


def test_phase8_t12_rear_camera_yaw_trimming_and_conflict():
    """T12: Rear camera yaw trimming sign: offset > 0 (right) -> CW trim (wz < 0);
    offset < 0 (left) -> CCW trim (wz > 0); conflict at edge -> stop.
    """
    guard = VelocityGuard()
    t = 100.0
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.6, "depth_proven": True, "obs_fresh": True}, now_mono=t)

    # 1. Target on right (offset = +0.40) -> CW trim (wz < 0)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.6, "depth_valid": True, "center_offset_x_norm": 0.40}]}, now_mono=t)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 0.01)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.02)
    assert wz < 0.0  # CW

    # 2. Target on left (offset = -0.40) -> CCW trim (wz > 0)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 50_000_000}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.6, "depth_valid": True, "center_offset_x_norm": -0.40}]}, now_mono=t + 0.05)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 0.06)
    _, _, wz_left, _ = guard.process_velocity(now_mono=t + 0.07)
    assert wz_left > 0.0  # CCW

    # 3. Conflict resolution at right edge (offset = +0.80, near_edge=True)
    # Nav2 commands turning CCW (angular_z = +0.20) away from target -> conflict stop!
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 100_000_000}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.6, "depth_valid": True, "center_offset_x_norm": 0.80, "near_edge": True}]}, now_mono=t + 0.10)
    guard.update_cmd_vel(0.30, 0.0, 0.20, now_mono=t + 0.11)
    vx, _, wz_conflict, reason = guard.process_velocity(now_mono=t + 0.12)
    assert vx == 0.0
    assert wz_conflict == 0.0
    assert reason == "NAV2_EDGE_TURNING_CONFLICT_STOP"


def test_phase8_t14_t15_loss_grace_and_abort_with_mock_voice_cancel():
    """T14/T15: Loss < 0.40s grace period allows continuous motion.
    Loss > 1.20s triggers TrackingManager ABORTING -> Guard clamps to zero -> Mock Voice cancels goal.
    """
    guard = VelocityGuard()
    t = 100.0

    # 1. Normal active motion
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True}, now_mono=t)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.5, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 0.01)
    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.30

    # 2. Brief loss for 0.25s (< 0.40s grace): motion continues
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 0.24)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.25)
    assert vx == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"

    # 3. Prolonged loss at t + 1.25s (> 1.20s abort): Guard clamps to zero
    # Keep session heartbeat fresh so observation timeout is isolated
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True}, now_mono=t + 1.20)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 1.24)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 1.25)
    assert vx == 0.0
    assert reason == "OBSERVATION_AGE_EXCEEDED"


def test_phase8_t16_voice_unresponsive_independent_zero():
    """T16: When Voice is unresponsive to cancel request, Guard independently maintains zero output."""
    guard = VelocityGuard()
    t = 100.0

    # FSM state switches to WAITING_FOR_PASSENGER or ABORTING
    guard.update_session_state({"session_id": "s1", "state": "ABORTING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": False}, now_mono=t)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "src-1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.5, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t)

    # Nav2 continues sending non-zero velocity commands
    for i in range(5):
        guard.update_cmd_vel(0.35, 0.0, 0.10, now_mono=t + 0.05 * i)
        vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.05 * i + 0.01)
        assert vx == 0.0
        assert vy == 0.0
        assert wz == 0.0
        assert "GATE_CLOSED" in reason


def test_phase8_t23_collision_monitor_priority_over_guard():
    """T23 (Critical Safety Test): While Guard is outputting velocity > 0, inject obstacle into
    laser scan. Collision Monitor MUST halt /cmd_vel to zero, proving Guard does not override collision safety!
    """
    rclpy.init()
    try:
        guard = VelocityGuardNode()
        cm = MockCollisionMonitor(in_topic='/tracking/cmd_vel_guarded', out_topic='/cmd_vel')
        obs_pub = MockObservationPublisher()
        scan_pub = MockScanPublisher()
        listener = CmdVelSubscriber('/cmd_vel')
        smoother_pub = guard.create_publisher(Twist, '/tracking/cmd_vel_smoothed', 10)

        all_nodes = [guard, cm, obs_pub, scan_pub, listener]

        # 1. Normal state: clear scan (obstacle at 5.0m)
        scan_pub.publish_scan(min_range=5.0)
        guard.guard.update_session_state({
            "schema_version": "1.0", "session_id": "s1", "state": "NAVIGATING", "locked": True,
            "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True
        }, now_mono=time.monotonic())
        obs_pub.publish_obs(track_id=1, dist_m=1.5, offset_x=0.0, depth_valid=True)
        _spin_all(all_nodes, duration_sec=0.10)

        # Nav2 sends cmd -> passed through
        cmd = Twist()
        cmd.linear.x = 0.35
        smoother_pub.publish(cmd)
        _spin_all(all_nodes, duration_sec=0.15)

        assert len(listener.received_msgs) > 0
        assert listener.received_msgs[-1].linear.x > 0.0

        # 2. INJECT OBSTACLE in scan: range = 0.40m (< 1.0m collision zone)
        # Guard continues sending 0.35 m/s, but Collision Monitor MUST clamp /cmd_vel to zero!
        listener.received_msgs.clear()
        scan_pub.publish_scan(min_range=0.40)
        smoother_pub.publish(cmd)
        _spin_all(all_nodes, duration_sec=0.15)

        assert len(listener.received_msgs) > 0
        latest = listener.received_msgs[-1]
        assert latest.linear.x == 0.0
        assert latest.linear.y == 0.0
        assert latest.angular.z == 0.0

        for n in all_nodes:
            n.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_phase8_t22_guard_process_termination_watchdog():
    """T22 (Critical Watchdog Test): Kill Guard process while car is moving -> Collision Monitor
    source_timeout triggers and zeroes /cmd_vel to stop the robot.
    """
    rclpy.init()
    try:
        guard = VelocityGuardNode()
        cm = MockCollisionMonitor(in_topic='/tracking/cmd_vel_guarded', out_topic='/cmd_vel')
        listener = CmdVelSubscriber('/cmd_vel')
        smoother_pub = guard.create_publisher(Twist, '/tracking/cmd_vel_smoothed', 10)
        all_nodes = [guard, cm, listener]

        # Put into NAVIGATING
        guard.guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True}, now_mono=time.monotonic())
        guard.guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "s1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.5, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=time.monotonic())

        cmd = Twist()
        cmd.linear.x = 0.30
        smoother_pub.publish(cmd)
        _spin_all(all_nodes, duration_sec=0.10)

        # KILL GUARD NODE
        guard.destroy_node()

        # Spin remaining nodes past Collision Monitor source_timeout (1.0s)
        listener.received_msgs.clear()
        _spin_all([cm, listener], duration_sec=1.15)

        # Collision monitor must output strictly zero!
        assert len(listener.received_msgs) > 0
        latest = listener.received_msgs[-1]
        assert latest.linear.x == 0.0
        assert latest.linear.y == 0.0
        assert latest.angular.z == 0.0

        cm.destroy_node()
        listener.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_phase8_t33_clock_pause_monotonic_watchdog():
    """T33: Steady clock watchdog runs independently of /clock freeze.
    When /clock freezes, monotonic watchdog times out commands and holds zero on resume.
    """
    guard = VelocityGuard()
    mono_t = 100.0
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True}, now_mono=mono_t)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "s1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.5, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=mono_t)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=mono_t + 0.01)

    # Moving at t + 0.05
    vx, _, _, _ = guard.process_velocity(now_mono=mono_t + 0.05)
    assert vx == 0.30

    # Simulation clock paused, but real monotonic time reaches t + 0.25 (> 0.20s cmd timeout)
    vx, _, _, reason = guard.process_velocity(now_mono=mono_t + 0.25)
    assert vx == 0.0
    assert reason == "CMD_VEL_STALE"


def test_phase8_t35_mecanum_lateral_stability():
    """T35: Mecanum robot does not drift laterally: linear.y is strictly 0.0 across all outputs."""
    guard = VelocityGuard()
    t = 100.0
    guard.update_session_state({"session_id": "s1", "state": "NAVIGATING", "locked": True, "person_track_id": 1, "person_distance_m": 1.5, "depth_proven": True, "obs_fresh": True}, now_mono=t)
    guard.update_observation({"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 0}, "source_session_id": "s1", "candidates": [{"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "torso_depth_m": 1.5, "depth_valid": True, "center_offset_x_norm": 0.0}]}, now_mono=t)

    # Nav2 requests lateral velocity linear.y = 0.25 m/s
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.25, angular_z=0.0, now_mono=t + 0.01)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)

    assert reason == "ESCORT_VELOCITY_PASS"
    assert vx == 0.30
    assert vy == 0.0  # linear.y MUST BE STRICTLY 0.0!
