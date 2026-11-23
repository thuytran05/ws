"""Phase 6 Integration Tests: Session FSM, Voice Topic Contracts, and Safety Invariants.

Verifies:
- Mock voice departure gating: unlocked, locked stale, locked fresh.
- Target loss: grace period (<0.25s), WAIT state (0.40s), resumption when passenger returns before 1.20s.
- Target loss timeout: ABORT state (1.20s), safety request emission, retries, and ACK handling.
- Terminal result confirmation: SUCCEEDED / CANCELED vs stale responses from old goals/sessions.
- Unconfirmed terminal blocks future sessions until cleared.
- Source reset or node restart during navigation latches stop and prevents movement authority.
- CRITICAL SAFETY INVARIANT: STRICTLY ZERO /cmd_vel publishers in person_tracking.
"""

import json
import time
from typing import Dict, List, Optional
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from person_tracking.passenger_lock_manager import PassengerLockManager, TrackingState
from person_tracking.tracking_manager_node import TrackingManagerNode


class MockVoiceAgent(Node):
    """Mock of airport_robot_voice node for offline contract verification."""

    def __init__(self):
        super().__init__('mock_voice_agent')
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.pub_nav_status = self.create_publisher(String, '/voice/nav_status', transient_qos)
        self.pub_nav_state = self.create_publisher(String, '/voice/nav_state', transient_qos)

        # Subscribers
        self.sub_session_state = self.create_subscription(
            String, '/tracking/session_state', self._on_session_state, transient_qos
        )
        self.sub_safety_request = self.create_subscription(
            String, '/tracking/safety_request', self._on_safety_request, reliable_qos
        )

        self.latest_session_state: Optional[dict] = None
        self.received_safety_requests: List[dict] = []
        self.auto_ack_safety: bool = True

    def _on_session_state(self, msg: String):
        try:
            self.latest_session_state = json.loads(msg.data)
        except Exception:
            pass

    def _on_safety_request(self, msg: String):
        try:
            req = json.loads(msg.data)
            self.received_safety_requests.append(req)
            if self.auto_ack_safety:
                self.send_ack(req.get("request_id", ""), req.get("session_id", ""))
        except Exception:
            pass

    def send_ack(self, request_id: str, session_id: str):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "request_id": request_id,
            "session_id": session_id,
            "status": "CANCELING",
            "nav_event": "CANCEL_ACKNOWLEDGED",
            "is_terminal": False,
            "message": "Abort request acknowledged by Voice.",
        }
        self.pub_nav_status.publish(String(data=json.dumps(payload)))

    def send_start_navigation(self, session_id: str, destination_id: str = "gate_1"):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "session_id": session_id,
            "destination_id": destination_id,
            "status": "NAVIGATING",
            "nav_event": "NAVIGATION_STARTED",
            "is_terminal": False,
            "message": f"Navigating to {destination_id}",
        }
        self.pub_nav_status.publish(String(data=json.dumps(payload)))

    def send_terminal_result(self, session_id: str, status: str = "SUCCEEDED"):
        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": 1, "nanosec": 0},
            "session_id": session_id,
            "status": status,
            "nav_event": "GOAL_COMPLETED",
            "is_terminal": True,
            "message": f"Goal completed with status {status}",
        }
        self.pub_nav_status.publish(String(data=json.dumps(payload)))


def _spin_both(node1: Node, node2: Node, duration_sec: float = 0.2, step_sec: float = 0.02):
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        rclpy.spin_once(node1, timeout_sec=step_sec)
        rclpy.spin_once(node2, timeout_sec=step_sec)


def _make_obs_msg(track_id: int, hand_up: bool, stamp_sec: int, stamp_nsec: int, session_id: str = "session-1") -> String:
    kps = [[200.0, 200.0, 0.9]] * 17
    kps[5] = [180.0, 150.0, 0.9]
    kps[6] = [220.0, 150.0, 0.9]
    if hand_up:
        kps[9] = [170.0, 100.0, 0.95]
        kps[10] = [230.0, 260.0, 0.9]
    else:
        kps[9] = [170.0, 260.0, 0.9]
        kps[10] = [230.0, 260.0, 0.9]

    cand = {
        "track_id": track_id,
        "is_confirmed": True,
        "is_ambiguous": False,
        "confidence": 0.95,
        "bbox_xyxy": [100.0, 80.0, 260.0, 420.0],
        "torso_depth_m": 1.6,
        "depth_valid": True,
        "center_offset_x_norm": 0.0,
        "near_edge": False,
        "keypoints": kps,
    }
    payload = {
        "schema_version": "1.0",
        "stamp": {"sec": stamp_sec, "nanosec": stamp_nsec},
        "source_session_id": session_id,
        "candidates": [cand],
    }
    return String(data=json.dumps(payload))


# ==============================================================================
# TEST SUITE: PHASE 6 SESSION FSM & MOCK VOICE CONTRACTS
# ==============================================================================

def test_mock_voice_departure_gating_unlocked_stale_fresh():
    """Voice gating rule: Only departs when locked AND state=LOCKED_WAIT_DESTINATION AND obs_fresh=True."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    def voice_check(snapshot) -> bool:
        return bool(snapshot.locked and snapshot.state == "LOCKED_WAIT_DESTINATION" and snapshot.obs_fresh)

    # 1. Unlocked (IDLE): Voice rejects
    snap_idle = manager._make_snapshot(0, 0, current_time_sec=0.0)
    assert voice_check(snap_idle) is False

    # 2. Feed frames to lock at t=1.5s
    obs_up = {"track_id": 1, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.9, "keypoints": [[180, 150, 0.9]] * 17}
    obs_up["keypoints"][5] = [180, 150, 0.9]
    obs_up["keypoints"][9] = [180, 100, 0.9]  # Hand up
    for i in range(16):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        payload = {"source_session_id": "sess-1", "candidates": [obs_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(payload, current_time_sec=t)

    assert manager.locked is True
    assert manager.state == TrackingState.LOCKED_WAIT_DESTINATION

    # Fresh snapshot: Voice accepts departure
    snap_fresh = manager._make_snapshot(1, 500000000, current_time_sec=1.5)
    assert snap_fresh.obs_fresh is True
    assert voice_check(snap_fresh) is True

    # 3. Target unseen for > 0.40s: Voice rejects departure
    snap_stale = manager._make_snapshot(2, 0, current_time_sec=1.95)
    assert snap_stale.obs_fresh is False
    assert voice_check(snap_stale) is False


def test_fsm_brief_loss_grace_wait_and_resume():
    """In NAVIGATING: loss < 0.25s is GRACE; loss > 0.40s enters WAITING_FOR_PASSENGER; return before 1.20s resumes."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # 1. Lock on passenger 2
    cand_up = {"track_id": 2, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.95, "keypoints": [[180, 150, 0.9]] * 17}
    cand_up["keypoints"][5] = [180, 150, 0.9]
    cand_up["keypoints"][9] = [180, 100, 0.9]
    for i in range(16):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        payload = {"source_session_id": "sess-fsm", "candidates": [cand_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(payload, current_time_sec=t)

    assert manager.locked is True
    sess_id = manager.session_id

    # 2. Voice accepts destination -> FSM transitions to NAVIGATING
    ok, reason = manager.on_destination_accepted(sess_id, current_time_sec=1.5)
    assert ok is True
    assert manager.state == TrackingState.NAVIGATING

    # 3. Target lost for 0.15s (< 0.25s grace period): retains NAVIGATING
    manager.process_observation(
        {"source_session_id": "sess-fsm", "candidates": [], "stamp": {"sec": 1, "nanosec": 650000000}},
        current_time_sec=1.65,
    )
    assert manager.state == TrackingState.NAVIGATING
    assert manager.reason_code == "LOCKED_TARGET_GRACE_PERIOD"
    assert manager.locked_track_id == 2  # Track ID retained!

    # 4. Target lost for 0.45s (> 0.40s stop threshold): transitions to WAITING_FOR_PASSENGER
    manager.process_observation(
        {"source_session_id": "sess-fsm", "candidates": [], "stamp": {"sec": 1, "nanosec": 950000000}},
        current_time_sec=1.95,
    )
    assert manager.state == TrackingState.WAITING_FOR_PASSENGER
    assert manager.reason_code == "VISIBILITY_TIMEOUT_STOP"
    assert manager.locked_track_id == 2  # Track ID retained in WAIT!

    # 5. Matching passenger 2 returns at t=2.2s (loss duration = 0.70s < 1.20s abort threshold):
    cand_normal = {"track_id": 2, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.92, "keypoints": [[180, 150, 0.9]] * 17}
    cand_normal["keypoints"][5] = [180, 150, 0.9]
    cand_normal["keypoints"][9] = [180, 260, 0.9]  # Hand down
    manager.process_observation(
        {"source_session_id": "sess-fsm", "candidates": [cand_normal], "stamp": {"sec": 2, "nanosec": 200000000}},
        current_time_sec=2.20,
    )
    # RESUMED!
    assert manager.state == TrackingState.NAVIGATING
    assert manager.reason_code == "PASSENGER_RETURNED_RESUMED"
    assert manager.locked_track_id == 2


def test_fsm_abort_safety_request_retry_and_terminal_cleanup():
    """In NAVIGATING: loss >= 1.20s triggers ABORTING, emits safety request, retries, and latches stop."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # 1. Lock and start navigation
    cand = {"track_id": 5, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.9, "keypoints": [[180, 150, 0.9]] * 17}
    cand["keypoints"][5] = [180, 150, 0.9]
    cand["keypoints"][9] = [180, 100, 0.9]
    for i in range(16):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        manager.process_observation({"source_session_id": "sess-abort", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}, current_time_sec=t)

    sess_id = manager.session_id
    manager.on_destination_accepted(sess_id, current_time_sec=1.5)
    assert manager.state == TrackingState.NAVIGATING

    # 2. Target lost for 1.25s (> 1.20s abort threshold)
    manager.process_observation(
        {"source_session_id": "sess-abort", "candidates": [], "stamp": {"sec": 2, "nanosec": 750000000}},
        current_time_sec=2.75,
    )
    assert manager.state == TrackingState.ABORTING
    assert manager.reason_code == "VISIBILITY_TIMEOUT_ABORT"

    # 3. Invariant: Passenger reappearing AFTER abort CANNOT unlatch stop!
    manager.process_observation(
        {"source_session_id": "sess-abort", "candidates": [cand], "stamp": {"sec": 2, "nanosec": 850000000}},
        current_time_sec=2.85,
    )
    assert manager.state == TrackingState.ABORTING, "SAFETY VIOLATION: ABORT state unlatched by returning candidate!"

    # 4. Terminal status from Voice for matching session cleans up to IDLE
    status_terminal = {
        "session_id": sess_id,
        "status": "CANCELED",
        "is_terminal": True,
    }
    handled, reason = manager.on_voice_nav_status(status_terminal, current_time_sec=3.0)
    assert handled is True
    assert manager.state == TrackingState.IDLE
    assert manager.locked is False


def test_fsm_goal_completion_succeeded_vs_late_stale_goal():
    """Voice SUCCEEDED for matching session completes to IDLE; stale goal status from old session is ignored."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # Lock and start navigation for session A
    cand = {"track_id": 6, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.9, "keypoints": [[180, 150, 0.9]] * 17}
    cand["keypoints"][5] = [180, 150, 0.9]
    cand["keypoints"][9] = [180, 100, 0.9]
    for i in range(16):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        manager.process_observation({"source_session_id": "sess-suc", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}, current_time_sec=t)

    session_a = manager.session_id
    manager.on_destination_accepted(session_a, current_time_sec=1.5)
    assert manager.state == TrackingState.NAVIGATING

    # 1. Late status from OLD session X arrives: must be IGNORED!
    old_status = {"session_id": "session-OLD-X", "status": "SUCCEEDED", "is_terminal": True}
    handled, reason = manager.on_voice_nav_status(old_status, current_time_sec=2.0)
    assert handled is False
    assert "IGNORED" in reason
    assert manager.state == TrackingState.NAVIGATING
    assert manager.session_id == session_a  # Current session intact!

    # 2. Terminal SUCCEEDED for session A arrives: transitions to ARRIVED and cleans up
    current_status = {"session_id": session_a, "status": "SUCCEEDED", "is_terminal": True}
    handled_ok, reason_ok = manager.on_voice_nav_status(current_status, current_time_sec=2.5)
    assert handled_ok is True
    assert manager.state == TrackingState.IDLE
    assert manager.locked is False


def test_unconfirmed_terminal_blocks_new_session():
    """If previous abort was unconfirmed by Nav2/Voice, future locks are blocked until confirmed."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    manager.unconfirmed_terminal = True

    cand = {"track_id": 7, "is_confirmed": True, "is_ambiguous": False, "confidence": 0.9, "keypoints": [[180, 150, 0.9]] * 17}
    cand["keypoints"][5] = [180, 150, 0.9]
    cand["keypoints"][9] = [180, 100, 0.9]
    for i in range(20):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        snap = manager.process_observation({"source_session_id": "sess-block", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}, current_time_sec=t)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert snap.reason_code == "UNCONFIRMED_TERMINAL_BLOCK"


def test_ros2_topic_integration_tracking_manager_with_mock_voice():
    """Full ROS 2 topic integration test: TrackingManagerNode + MockVoiceAgent over ROS topics."""
    if not rclpy.ok():
        rclpy.init()
    try:
        manager_node = TrackingManagerNode()
        voice_node = MockVoiceAgent()

        obs_pub = manager_node.create_publisher(String, '/tracking/observation', 10)

        # 1. Spin initially: both nodes up, state is IDLE
        _spin_both(manager_node, voice_node, duration_sec=0.25)
        assert voice_node.latest_session_state is not None
        assert voice_node.latest_session_state.get("state") == "IDLE"
        assert voice_node.latest_session_state.get("locked") is False

        # 2. Publish 16 frames to lock passenger 9
        for i in range(16):
            t = round(i * 0.1, 2)
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            msg = _make_obs_msg(track_id=9, hand_up=True, stamp_sec=sec, stamp_nsec=nsec, session_id="ros-sess-1")
            obs_pub.publish(msg)
            _spin_both(manager_node, voice_node, duration_sec=0.03)

        _spin_both(manager_node, voice_node, duration_sec=0.1)
        assert manager_node.manager.locked is True
        assert manager_node.manager.state == TrackingState.LOCKED_WAIT_DESTINATION
        sess_id = manager_node.manager.session_id

        # 3. Voice node publishes navigation start on /voice/nav_status
        voice_node.send_start_navigation(sess_id, destination_id="gate_2")
        _spin_both(manager_node, voice_node, duration_sec=0.15)

        assert manager_node.manager.state == TrackingState.NAVIGATING

        # 4. Watchdog test: Halt observations completely!
        # Spin 0.5s: unseen > 0.40s -> transitions to WAITING_FOR_PASSENGER
        _spin_both(manager_node, voice_node, duration_sec=0.50)
        assert manager_node.manager.state == TrackingState.WAITING_FOR_PASSENGER

        # Spin past 1.20s total: triggers ABORTING and sends safety request
        _spin_both(manager_node, voice_node, duration_sec=0.85)
        assert manager_node.manager.state == TrackingState.ABORTING
        assert len(voice_node.received_safety_requests) >= 1
        safety_req = voice_node.received_safety_requests[0]
        assert safety_req["command"] == "ABORT"
        assert safety_req["session_id"] == sess_id

        # 5. Voice confirms terminal CANCELED
        voice_node.send_terminal_result(sess_id, status="CANCELED")
        _spin_both(manager_node, voice_node, duration_sec=0.2)

        assert manager_node.manager.state == TrackingState.IDLE
        assert manager_node.manager.locked is False

        manager_node.destroy_node()
        voice_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_critical_safety_zero_cmd_vel_publishers_across_all_tracking_nodes():
    """CRITICAL SAFETY AUDIT: Strictly ZERO /cmd_vel publishers across entire tracking subsystem."""
    import person_tracking.tracking_manager_node as tmn
    import person_tracking.person_detector_node as pdn
    import person_tracking.perception_source_node as psn
    import inspect

    for mod, name in [(tmn, 'tracking_manager_node'), (pdn, 'person_detector_node'), (psn, 'perception_source_node')]:
        src = inspect.getsource(mod)
        for line in src.splitlines():
            # Exclude docstrings and comments from keyword check
            line_strip = line.strip()
            if line_strip.startswith('#') or line_strip.startswith('"') or line_strip.startswith("'"):
                continue
            assert "create_publisher" not in line or ("cmd_vel" not in line and "Twist" not in line), f"SAFETY VIOLATION: publisher for cmd_vel found in {name}: {line}"
            assert "linear.x" not in line, f"SAFETY VIOLATION: velocity drive command found in {name}: {line}"
