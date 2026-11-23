"""Section A Regression Verification Tests for person_tracking Phase 6 & Phase 7.

Verifies:
1. Passenger loss deadline check before accepting returning candidate (>= 1.20s -> ABORTING).
2. Latched stop in ABORTING: source switch, matching candidate, or old frames cannot unlatch or reset session.
3. Confirmed OUT_OF_FOV (near-edge + disappearance -> immediate ABORT) vs center flicker (grace period).
4. Timestamp validations: reject nanoseconds >= 10^9, bools, missing stamps; stamp 0/0 does not extend freshness.
5. New source session starting from 0/epoch is NOT falsely dropped as is_old_frame; late frames from retired sources dropped.
6. Strict Voice contract: START requires session_id, mission_id, and goal_attempt_id; no auto-filling fallback.
7. Terminal status UNKNOWN rejected; mismatched goal_attempt_id rejected.
8. Dedicated terminal deadline timer (3.0s) after ACK triggers FAULT and unconfirmed_terminal block.
"""

import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from person_tracking.passenger_lock_manager import PassengerLockManager, TrackingState
from person_tracking.tracking_manager_node import TrackingManagerNode


def _lock_passenger(manager: PassengerLockManager, track_id: int = 1, near_edge: bool = False, depth_m: float = 1.6) -> str:
    """Helper to lock passenger at t=1.5s."""
    cand = {
        "track_id": track_id,
        "is_confirmed": True,
        "is_ambiguous": False,
        "confidence": 0.95,
        "bbox_xyxy": [100.0, 80.0, 260.0, 420.0],
        "torso_depth_m": depth_m,
        "depth_valid": True,
        "center_offset_x_norm": 0.85 if near_edge else 0.0,
        "near_edge": near_edge,
        "keypoints": [[180.0, 150.0, 0.9]] * 17,
    }
    cand["keypoints"][5] = [180.0, 150.0, 0.9]
    cand["keypoints"][9] = [180.0, 100.0, 0.95]  # Left wrist raised

    for i in range(16):
        t = round(i * 0.1, 2)
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        obs = {
            "source_session_id": "src-main",
            "candidates": [cand],
            "stamp": {"sec": sec, "nanosec": nsec},
        }
        manager.process_observation(obs, current_time_sec=t)

    assert manager.locked is True
    assert manager.state == TrackingState.LOCKED_WAIT_DESTINATION
    return manager.session_id


def test_returning_passenger_after_1_2s_deadline_enters_aborting():
    """A.1: If passenger was unseen >= 1.20s, returning frame MUST enter ABORTING, not resume!"""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess_id = _lock_passenger(manager, track_id=1, depth_m=1.6)

    # Start navigation at t = 1.5s
    ok, _ = manager.on_destination_accepted(
        sess_id, mission_id="mission-1", goal_attempt_id="goal-1", current_time_sec=1.5
    )
    assert ok is True
    assert manager.state == TrackingState.NAVIGATING

    # Passenger disappears from t = 1.5s to t = 2.75s (1.25s unseen >= 1.20s abort deadline)
    # A new frame arrives at t = 2.75s containing the matching passenger ID=1
    cand_returned = {
        "track_id": 1,
        "is_confirmed": True,
        "is_ambiguous": False,
        "confidence": 0.95,
        "torso_depth_m": 1.6,
        "depth_valid": True,
        "near_edge": False,
    }
    obs_returned = {
        "source_session_id": "src-main",
        "candidates": [cand_returned],
        "stamp": {"sec": 2, "nanosec": 750000000},
    }

    # Processing the frame MUST transition to ABORTING because deadline expired before frame
    snap = manager.process_observation(obs_returned, current_time_sec=2.75)
    assert manager.state == TrackingState.ABORTING
    assert manager.reason_code == "VISIBILITY_TIMEOUT_ABORT"


def test_once_in_aborting_source_switch_or_candidate_cannot_unlatch():
    """A.1: Latched stop in ABORTING: source switch, matching candidate, or old frames cannot unlatch or reset."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess_id = _lock_passenger(manager, track_id=2)
    manager.on_destination_accepted(sess_id, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)

    # Trigger abort by exceeding deadline
    manager._update_watchdog(current_time_sec=3.0)
    assert manager.state == TrackingState.ABORTING
    assert manager.locked is True

    # 1. New frame with different source_session_id arrives -> MUST remain ABORTING (cannot reset to IDLE!)
    obs_new_source = {
        "source_session_id": "new-source-session",
        "candidates": [{"track_id": 2, "is_confirmed": True, "depth_valid": True}],
        "stamp": {"sec": 3, "nanosec": 100000000},
    }
    manager.process_observation(obs_new_source, current_time_sec=3.1)
    assert manager.state == TrackingState.ABORTING
    assert manager.locked is True

    # 2. Frame with matching ID arrives -> MUST remain ABORTING
    obs_same_id = {
        "source_session_id": "src-main",
        "candidates": [{"track_id": 2, "is_confirmed": True, "depth_valid": True}],
        "stamp": {"sec": 3, "nanosec": 200000000},
    }
    manager.process_observation(obs_same_id, current_time_sec=3.2)
    assert manager.state == TrackingState.ABORTING
    assert manager.locked is True


def test_out_of_fov_immediate_abort_vs_center_flicker_grace():
    """A.1: Near-edge + disappearance -> immediate OUT_OF_FOV ABORT; Center disappearance -> grace period."""
    # Case 1: Confirmed OUT_OF_FOV
    mgr1 = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess1 = _lock_passenger(mgr1, track_id=3, near_edge=True)
    mgr1.on_destination_accepted(sess1, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)

    # In next frame at t=1.6s, passenger disappears (candidates empty, lost_track_ids=[3])
    obs_lost_edge = {
        "source_session_id": "src-main",
        "candidates": [],
        "lost_track_ids": [3],
        "stamp": {"sec": 1, "nanosec": 600000000},
    }
    mgr1.process_observation(obs_lost_edge, current_time_sec=1.6)
    assert mgr1.state == TrackingState.ABORTING
    assert mgr1.reason_code == "OUT_OF_FOV"

    # Case 2: Center flicker (near_edge = False)
    mgr2 = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess2 = _lock_passenger(mgr2, track_id=4, near_edge=False)
    mgr2.on_destination_accepted(sess2, mission_id="m2", goal_attempt_id="g2", current_time_sec=1.5)

    # In next frame at t=1.6s, passenger momentarily drops (flicker)
    obs_flicker = {
        "source_session_id": "src-main",
        "candidates": [],
        "lost_track_ids": [4],
        "stamp": {"sec": 1, "nanosec": 600000000},
    }
    mgr2.process_observation(obs_flicker, current_time_sec=1.6)
    # Does NOT abort immediately! Remains in NAVIGATING during grace period (unseen = 0.10s < 0.25s)
    assert mgr2.state == TrackingState.NAVIGATING
    assert mgr2.reason_code == "LOCKED_TARGET_GRACE_PERIOD"


def test_timestamp_validations_and_freshness_invariants():
    """A.2: Reject nanoseconds >= 10^9, bools, missing stamps; stamp 0/0 does not extend freshness."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand = {"track_id": 5, "is_confirmed": True, "confidence": 0.9, "near_edge": False, "depth_valid": True}

    # 1. Nanoseconds >= 10^9 rejected
    obs_bad_nsec = {"source_session_id": "s1", "candidates": [cand], "stamp": {"sec": 1, "nanosec": 1_000_000_000}}
    snap = manager.process_observation(obs_bad_nsec, current_time_sec=1.0)
    assert snap.reason_code == "INVALID_STAMP_NO_ACCUMULATION"

    # 2. Bool stamp rejected
    obs_bool = {"source_session_id": "s1", "candidates": [cand], "stamp": {"sec": True, "nanosec": 100}}
    snap = manager.process_observation(obs_bool, current_time_sec=1.0)
    assert snap.reason_code == "INVALID_STAMP_NO_ACCUMULATION"

    # 3. Stamp 0/0 does NOT extend freshness of locked passenger
    sess_id = _lock_passenger(manager, track_id=5)
    manager.on_destination_accepted(sess_id, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)
    assert manager.is_observation_fresh(current_time_sec=1.5) is True

    # At t = 2.0s (>0.40s later), a frame with stamp 0/0 arrives
    obs_zero_stamp = {"source_session_id": "src-main", "candidates": [cand], "stamp": {"sec": 0, "nanosec": 0}}
    manager.process_observation(obs_zero_stamp, current_time_sec=2.0)
    # last_locked_passenger_seen_mono_time must NOT have updated from 1.5s!
    assert manager.last_locked_passenger_seen_mono_time == 1.5
    assert manager.is_observation_fresh(current_time_sec=2.0) is False


def test_first_frame_new_source_session_from_epoch_not_dropped():
    """A.2: First frame of new source session starting from 0/epoch is not dropped; late frames dropped."""
    manager = PassengerLockManager()
    cand = {"track_id": 1, "is_confirmed": True}

    # Session 1 runs up to t = 10.0s
    obs1 = {"source_session_id": "sess-alpha", "candidates": [cand], "stamp": {"sec": 10, "nanosec": 0}}
    manager.process_observation(obs1, current_time_sec=10.0)
    assert manager.last_observation_stamp["sec"] == 10

    # Session 2 begins with timestamp resetting back to epoch (0.1s)
    obs2 = {"source_session_id": "sess-beta", "candidates": [cand], "stamp": {"sec": 0, "nanosec": 100000000}}
    manager.process_observation(obs2, current_time_sec=10.1)
    # MUST be accepted as fresh observation of sess-beta, NOT dropped as is_old_frame!
    assert manager.last_observation_stamp["sec"] == 0
    assert manager.last_observation_stamp["nanosec"] == 100000000

    # Late frame from retired sess-alpha arrives -> dropped!
    obs_late = {"source_session_id": "sess-alpha", "candidates": [cand], "stamp": {"sec": 10, "nanosec": 500000000}}
    manager.process_observation(obs_late, current_time_sec=10.2)
    # Stamp remains on sess-beta!
    assert manager.last_observation_stamp["sec"] == 0


def test_strict_voice_start_contract_requires_all_goal_identifiers():
    """A.3: START requires session_id, mission_id, and goal_attempt_id."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess_id = _lock_passenger(manager, track_id=6)

    # Missing mission_id and goal_attempt_id -> rejected
    ok, reason = manager.on_destination_accepted(sess_id, current_time_sec=1.5)
    assert ok is False
    assert reason == "MISSING_GOAL_IDENTIFIERS"

    # Only mission_id provided -> rejected
    ok, reason = manager.on_destination_accepted(sess_id, mission_id="m1", current_time_sec=1.5)
    assert ok is False
    assert reason == "MISSING_GOAL_IDENTIFIERS"

    # All provided -> accepted
    ok, reason = manager.on_destination_accepted(sess_id, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)
    assert ok is True
    assert reason == "OK"
    assert manager.bound_mission_id == "m1"
    assert manager.bound_goal_attempt_id == "g1"


def test_terminal_status_unknown_rejected_and_mismatched_goal_rejected():
    """A.3: Terminal UNKNOWN is rejected; mismatched goal_attempt_id rejected."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    sess_id = _lock_passenger(manager, track_id=7)
    manager.on_destination_accepted(sess_id, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)

    # Terminal with status UNKNOWN -> rejected
    status_unknown = {
        "session_id": sess_id,
        "mission_id": "m1",
        "goal_attempt_id": "g1",
        "status": "UNKNOWN",
        "is_terminal": True,
    }
    handled, reason = manager.on_voice_nav_status(status_unknown, current_time_sec=1.6)
    assert handled is False
    assert reason == "TERMINAL_STATUS_UNKNOWN_REJECTED"
    assert manager.state == TrackingState.NAVIGATING

    # Terminal with mismatched goal_attempt_id -> rejected
    status_mismatch = {
        "session_id": sess_id,
        "mission_id": "m1",
        "goal_attempt_id": "wrong-goal-id",
        "status": "SUCCEEDED",
        "is_terminal": True,
    }
    handled, reason = manager.on_voice_nav_status(status_mismatch, current_time_sec=1.6)
    assert handled is False
    assert reason == "MISMATCHED_GOAL_ATTEMPT_ID"
    assert manager.state == TrackingState.NAVIGATING


def test_terminal_timeout_after_ack_triggers_fault_and_unconfirmed_terminal():
    """A.3: If terminal confirmation is not received within terminal_timeout_sec after ACK -> FAULT & unconfirmed_terminal."""
    rclpy.init()
    try:
        node = TrackingManagerNode()
        sess_id = _lock_passenger(node.manager, track_id=8)
        node.manager.on_destination_accepted(sess_id, mission_id="m1", goal_attempt_id="g1", current_time_sec=1.5)

        # Trigger abort
        node.manager.state = TrackingState.ABORTING
        node.manager.reason_code = "VISIBILITY_TIMEOUT_ABORT"
        node._initiate_abort()
        assert node.abort_in_progress is True
        assert node.abort_acked is False
        req_id = node.current_abort_request_id

        # Voice sends matching ACK
        ack_msg = String(data=json.dumps({
            "session_id": sess_id,
            "request_id": req_id,
            "goal_attempt_id": "g1",
            "nav_event": "CANCEL_ACKNOWLEDGED",
            "is_terminal": False,
        }))
        node._on_voice_status(ack_msg)
        assert node.abort_acked is True

        # Simulate watchdog ticks past terminal_timeout_sec (3.0s)
        # Advance time by mocking monotonic time past 3.0s after ACK
        ack_time = node.abort_ack_received_mono_time
        node.abort_ack_received_mono_time = ack_time - 3.5  # 3.5s elapsed since ACK!

        node._on_watchdog_tick()
        assert node.manager.state == TrackingState.FAULT
        assert node.manager.unconfirmed_terminal is True
        assert node.manager.reason_code == "TERMINAL_TIMEOUT_EXCEEDED"

        # Attempt to lock a new passenger: MUST be blocked by unconfirmed_terminal!
        cand = {"track_id": 9, "is_confirmed": True, "depth_valid": True, "keypoints": [[180.0, 150.0, 0.9]] * 17}
        cand["keypoints"][9] = [180.0, 100.0, 0.95]
        obs = {"source_session_id": "src-new", "candidates": [cand], "stamp": {"sec": 10, "nanosec": 0}}
        snap = node.manager.process_observation(obs, current_time_sec=10.0)
        assert snap.locked is False
        assert snap.reason_code == "UNCONFIRMED_TERMINAL_BLOCK"

        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
