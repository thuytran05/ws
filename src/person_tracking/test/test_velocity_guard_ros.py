"""ROS 2 Integration and Safety Audit Tests for VelocityGuardNode (escort_velocity_guard).

Phase 7 Integration Matrix:
- Audit topic chain: /tracking/cmd_vel_smoothed -> escort_velocity_guard -> /tracking/cmd_vel_guarded.
- CRITICAL SAFETY INVARIANT: Strictly ZERO /cmd_vel publishers on escort_velocity_guard and person_tracking.
- Active Continuous Zeroing: Default and safety-triggered output is Twist(0,0,0) at 20 Hz.
- Pass-through: When NAVIGATING, locked, depth verified, and fresh cmd received -> passes velocity.
- T14 / T15 / Safety Clamp: When manager triggers ABORTING or WAITING, output drops to zero immediately.
- T16: Voice unresponsive -> Guard maintains zero independently of Voice latency.
- T22: Guard kill behavior -> downstream subscriber stops receiving, no residual self-motion.
- T33: Monotonic watchdog operates independently of /clock.
- T35: Lateral motion separation: linear.y = 0.0 strictly.
"""

import inspect
import json
import time
from typing import List, Optional
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from person_tracking.velocity_guard_node import VelocityGuardNode
from person_tracking import velocity_guard_node, passenger_lock_manager, tracking_manager_node


class MockNav2Smoother(Node):
    def __init__(self):
        super().__init__('mock_nav2_smoother')
        self.pub_cmd = self.create_publisher(Twist, '/tracking/cmd_vel_smoothed', 10)

    def send_cmd(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)


class MockTrackingPublisher(Node):
    def __init__(self):
        super().__init__('mock_tracking_publisher')
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_session = self.create_publisher(String, '/tracking/session_state', transient_qos)
        self.pub_obs = self.create_publisher(String, '/tracking/observation', reliable_qos)

    def publish_session(self, state: str, locked: bool, dist_m: float = 1.6, depth_proven: bool = True, obs_fresh: bool = True):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "last_observation_stamp": {"sec": 1, "nanosec": 0},
            "obs_fresh": obs_fresh,
            "sequence_num": 1,
            "session_id": "test-fsm-session",
            "state": state,
            "locked": locked,
            "person_track_id": 1,
            "person_distance_m": dist_m,
            "depth_proven": depth_proven,
            "confidence": 0.95,
            "reason_code": f"STATE_{state}",
        }
        self.pub_session.publish(String(data=json.dumps(payload)))

    def publish_observation(self, track_id: int = 1, dist_m: float = 1.6, depth_valid: bool = True, offset_x: float = 0.0):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "source_session_id": "src-1",
            "candidates": [
                {
                    "track_id": track_id,
                    "is_confirmed": True,
                    "is_ambiguous": False,
                    "torso_depth_m": dist_m,
                    "depth_valid": depth_valid,
                    "center_offset_x_norm": offset_x,
                    "near_edge": False,
                }
            ],
        }
        self.pub_obs.publish(String(data=json.dumps(payload)))


class GuardOutputSubscriber(Node):
    def __init__(self):
        super().__init__('guard_output_subscriber')
        self.received_msgs: List[Twist] = []
        self.sub = self.create_subscription(Twist, '/tracking/cmd_vel_guarded', self._on_msg, 10)

    def _on_msg(self, msg: Twist):
        self.received_msgs.append(msg)


def _spin_all(nodes: List[Node], duration_sec: float = 0.25, step_sec: float = 0.02):
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=step_sec)


# ==============================================================================
# TESTS
# ==============================================================================

def test_safety_audit_zero_cmd_vel_publishers():
    """CRITICAL SAFETY INVARIANT: escort_velocity_guard has STRICTLY ZERO /cmd_vel publishers."""
    rclpy.init()
    try:
        guard_node = VelocityGuardNode()

        # Check all published topics on guard_node
        pub_topics = [p.topic_name for p in guard_node.publishers]
        assert "/cmd_vel" not in pub_topics, f"VIOLATION: guard publishes directly to /cmd_vel: {pub_topics}"
        assert "/tracking/cmd_vel_guarded" in pub_topics

        # Source code AST inspection: verify no node in person_tracking creates a publisher for /cmd_vel
        modules = [velocity_guard_node, passenger_lock_manager, tracking_manager_node]
        for mod in modules:
            source_lines = inspect.getsource(mod).splitlines()
            for line in source_lines:
                stripped = line.strip()
                if stripped.startswith("#") or stripped.startswith('"""'):
                    continue
                if "create_publisher" in stripped and "cmd_vel" in stripped:
                    assert "/cmd_vel" not in stripped or "cmd_vel_guarded" in stripped or "cmd_vel_smoothed" in stripped, (
                        f"VIOLATION: create_publisher with /cmd_vel in {mod.__name__}: {line}"
                    )

        guard_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_guard_ros2_pass_through_and_active_zeroing():
    """Integration: Pass-through when NAVIGATING, immediate continuous zeroing on non-NAVIGATING."""
    rclpy.init()
    try:
        guard_node = VelocityGuardNode()
        smoother = MockNav2Smoother()
        tracker = MockTrackingPublisher()
        listener = GuardOutputSubscriber()

        all_nodes = [guard_node, smoother, tracker, listener]

        # 1. Initially UNLOCKED: Guard outputs continuous zeros
        _spin_all(all_nodes, duration_sec=0.15)
        assert len(listener.received_msgs) > 0
        latest = listener.received_msgs[-1]
        assert latest.linear.x == 0.0
        assert latest.linear.y == 0.0
        assert latest.angular.z == 0.0

        # 2. Transition to NAVIGATING with valid depth & observation
        tracker.publish_session(state="NAVIGATING", locked=True, dist_m=1.6, depth_proven=True)
        tracker.publish_observation(track_id=1, dist_m=1.6, depth_valid=True, offset_x=0.0)
        _spin_all(all_nodes, duration_sec=0.10)

        # 3. Nav2 sends cmd_vel_smoothed: linear.x = 0.35, linear.y = 0.15, angular.z = 0.05
        smoother.send_cmd(vx=0.35, vy=0.15, wz=0.05)
        _spin_all(all_nodes, duration_sec=0.15)

        assert len(listener.received_msgs) > 0
        latest = listener.received_msgs[-1]
        assert pytest.approx(latest.linear.x, 0.01) == 0.35
        # T35: linear.y = 0.0 strictly
        assert latest.linear.y == 0.0
        assert pytest.approx(latest.angular.z, 0.01) == 0.05

        # 4. Trigger ABORT (target lost > 1.20s) -> Immediate clamping to zero!
        listener.received_msgs.clear()
        tracker.publish_session(state="ABORTING", locked=True, dist_m=1.6)
        _spin_all(all_nodes, duration_sec=0.10)

        assert len(listener.received_msgs) > 0
        for msg in listener.received_msgs[-3:]:
            assert msg.linear.x == 0.0
            assert msg.linear.y == 0.0
            assert msg.angular.z == 0.0

        for n in all_nodes:
            n.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_t16_voice_unresponsive_independent_zero():
    """T16: When Voice does not ACK an abort request, Guard independently maintains continuous zero."""
    rclpy.init()
    try:
        guard_node = VelocityGuardNode()
        smoother = MockNav2Smoother()
        tracker = MockTrackingPublisher()
        listener = GuardOutputSubscriber()
        all_nodes = [guard_node, smoother, tracker, listener]

        # Put into NAVIGATING first
        tracker.publish_session(state="NAVIGATING", locked=True, dist_m=1.5, depth_proven=True)
        tracker.publish_observation(track_id=1, dist_m=1.5, depth_valid=True)
        smoother.send_cmd(vx=0.30)
        _spin_all(all_nodes, duration_sec=0.10)

        # Trigger WAITING_FOR_PASSENGER (e.g. lost 0.40s) while Nav2 continues sending commands
        tracker.publish_session(state="WAITING_FOR_PASSENGER", locked=True, dist_m=1.5, obs_fresh=False)
        listener.received_msgs.clear()

        # Nav2 keeps sending forward velocity, but Voice has not yet canceled Nav2
        for _ in range(5):
            smoother.send_cmd(vx=0.30)
            _spin_all(all_nodes, duration_sec=0.05)

        # Guard MUST maintain strictly zero velocity despite continuous Nav2 commands!
        assert len(listener.received_msgs) > 0
        for msg in listener.received_msgs:
            assert msg.linear.x == 0.0
            assert msg.linear.y == 0.0
            assert msg.angular.z == 0.0

        for n in all_nodes:
            n.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_t22_guard_kill_behavior():
    """T22: When escort_velocity_guard is killed, downstream subscribers stop receiving, no lingering outputs."""
    rclpy.init()
    try:
        guard_node = VelocityGuardNode()
        listener = GuardOutputSubscriber()

        # Spin briefly while running
        _spin_all([guard_node, listener], duration_sec=0.15)
        count_before_kill = len(listener.received_msgs)
        assert count_before_kill > 0

        # Kill guard node
        guard_node.destroy_node()

        # Spin listener alone
        listener.received_msgs.clear()
        _spin_all([listener], duration_sec=0.15)

        # Zero new messages received!
        assert len(listener.received_msgs) == 0

        listener.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
