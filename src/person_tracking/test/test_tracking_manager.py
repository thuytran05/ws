"""Unit and ROS 2 integration tests for Phase 4: Passenger Locking & Gesture Recognition.

Verifies:
- T02: Short gesture (< 1.5s) does NOT lock; lowering hand resets timer.
- T03: Valid gesture (>= 1.5s) locks; lowering hand retains lock.
- T04: Two candidates with valid depth -> nearest person selected.
- T05: Two candidates without depth and ambiguous sizes -> stays IDLE.
- Source reset / session switch invalidates lock and prevents ID hopping.
- Mock voice rejects departure when unlocked and accepts when locked.
- CRITICAL SAFETY CHECK: Zero /cmd_vel publishers on TrackingManagerNode.
"""

import json
import time
import numpy as np
from typing import Optional, List
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from person_tracking.passenger_lock_manager import PassengerLockManager, TrackingState
from person_tracking.tracking_manager_node import TrackingManagerNode


def _create_mock_candidate(
    track_id: int,
    hand_raised: bool = True,
    depth_m: Optional[float] = 1.5,
    depth_valid: bool = True,
    bbox_xyxy: Optional[List[float]] = None,
    is_ambiguous: bool = False,
) -> dict:
    if bbox_xyxy is None:
        bbox_xyxy = [100.0, 80.0, 260.0, 420.0]

    # Create 17 keypoints
    # Left shoulder: idx 5, Left wrist: idx 9
    # Wrist y < shoulder y = hand raised!
    kps = [[200.0, 200.0, 0.9]] * 17
    # Shoulders at y = 150
    kps[5] = [180.0, 150.0, 0.9]  # left shoulder
    kps[6] = [220.0, 150.0, 0.9]  # right shoulder

    if hand_raised:
        # Left wrist vertically higher than shoulder (y=100 < y=150)
        kps[9] = [170.0, 100.0, 0.95]
        # Right wrist down
        kps[10] = [230.0, 260.0, 0.9]
    else:
        # Both wrists down (y=260 > y=150)
        kps[9] = [170.0, 260.0, 0.9]
        kps[10] = [230.0, 260.0, 0.9]

    return {
        "track_id": track_id,
        "is_confirmed": True,
        "is_ambiguous": is_ambiguous,
        "confidence": 0.92,
        "bbox_xyxy": bbox_xyxy,
        "torso_depth_m": depth_m,
        "depth_valid": depth_valid,
        "center_offset_x_norm": 0.0,
        "near_edge": False,
        "keypoints": kps,
    }


def test_t02_gesture_hold_under_threshold_no_lock():
    """T02: Hand raised for < 1.5s does not lock; lowering hand resets timer."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # Frame 1: Candidate 1 raises hand at t = 0.0s
    cand = _create_mock_candidate(track_id=1, hand_raised=True)
    obs1 = {"source_session_id": "session-1", "candidates": [cand], "stamp": {"sec": 0, "nanosec": 0}}
    snap1 = manager.process_observation(obs1, current_time_sec=0.0)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert 1 in manager.gesture_raise_start

    # Frame 2: Candidate 1 still raising hand at t = 1.0s (< 1.5s)
    snap2 = manager.process_observation(obs1, current_time_sec=1.0)
    assert manager.locked is False
    assert manager.state == TrackingState.IDLE

    # Frame 3: Candidate 1 lowers hand at t = 1.2s -> timer resets
    cand_down = _create_mock_candidate(track_id=1, hand_raised=False)
    obs3 = {"source_session_id": "session-1", "candidates": [cand_down], "stamp": {"sec": 1, "nanosec": 200000000}}
    snap3 = manager.process_observation(obs3, current_time_sec=1.2)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert 1 not in manager.gesture_raise_start  # Timer cleanly reset!


def test_t03_valid_gesture_locks_and_hand_drop_retains_lock():
    """T03: Hand raised for >= 1.5s locks; lowering hand retains locked session."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    cand_up = _create_mock_candidate(track_id=2, hand_raised=True, depth_m=1.65, depth_valid=True)
    obs = {"source_session_id": "session-1", "candidates": [cand_up], "stamp": {"sec": 0, "nanosec": 0}}

    # Start gesture at t = 0.0s
    manager.process_observation(obs, current_time_sec=0.0)
    assert manager.locked is False

    # Continue gesture past threshold at t = 1.55s
    snap = manager.process_observation(obs, current_time_sec=1.55)
    assert manager.locked is True
    assert manager.state == TrackingState.LOCKED_WAIT_DESTINATION
    assert manager.locked_track_id == 2
    assert len(manager.session_id) > 0
    assert manager.locked_distance_m == 1.65
    assert manager.locked_depth_proven is True
    assert manager.reason_code == "HAND_RAISE_HELD_SINGLE"

    # INVARIANT: Lowering hand after locking DOES NOT lose lock!
    cand_down = _create_mock_candidate(track_id=2, hand_raised=False, depth_m=1.65, depth_valid=True)
    obs_down = {"source_session_id": "session-1", "candidates": [cand_down], "stamp": {"sec": 2, "nanosec": 0}}
    snap_after_drop = manager.process_observation(obs_down, current_time_sec=2.0)

    assert manager.locked is True
    assert manager.state == TrackingState.LOCKED_WAIT_DESTINATION
    assert manager.locked_track_id == 2


def test_t04_two_candidates_nearest_depth_selection():
    """T04: Two candidates raise hand; nearest person (smaller depth) is selected."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # Candidate 1: 1.4m away (nearer)
    c1 = _create_mock_candidate(track_id=10, hand_raised=True, depth_m=1.40, depth_valid=True)
    # Candidate 2: 2.8m away (farther)
    c2 = _create_mock_candidate(track_id=20, hand_raised=True, depth_m=2.80, depth_valid=True)

    obs = {"source_session_id": "session-1", "candidates": [c1, c2], "stamp": {"sec": 0, "nanosec": 0}}

    # Start gesture at t = 0.0s
    manager.process_observation(obs, current_time_sec=0.0)
    assert manager.locked is False

    # After 1.6s of holding: both eligible
    snap = manager.process_observation(obs, current_time_sec=1.6)
    assert manager.locked is True
    assert manager.locked_track_id == 10  # Nearest candidate (1.4m) selected!
    assert manager.locked_distance_m == 1.40
    assert manager.reason_code == "HAND_RAISE_HELD_NEAREST"


def test_t05_ambiguous_gesture_no_depth_stays_idle():
    """T05: Two candidates raise hand without depth and identical size -> refuses tentative lock."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # Identical bounding box size (160x340), depth_valid = False
    c1 = _create_mock_candidate(track_id=1, hand_raised=True, depth_valid=False, bbox_xyxy=[50.0, 80.0, 210.0, 420.0])
    c2 = _create_mock_candidate(track_id=2, hand_raised=True, depth_valid=False, bbox_xyxy=[300.0, 80.0, 460.0, 420.0])

    obs = {"source_session_id": "session-1", "candidates": [c1, c2], "stamp": {"sec": 0, "nanosec": 0}}

    manager.process_observation(obs, current_time_sec=0.0)
    snap = manager.process_observation(obs, current_time_sec=1.6)

    # Must stay in IDLE and report AMBIGUOUS_GESTURE!
    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert manager.reason_code == "AMBIGUOUS_GESTURE"


def test_source_reset_invalidates_session_no_id_hopping():
    """Verify source reset (new source_session_id) resets lock to IDLE and prevents ID hopping."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.0)

    # Lock Candidate 5 on session A
    c_up = _create_mock_candidate(track_id=5, hand_raised=True)
    obs_a = {"source_session_id": "session-A", "candidates": [c_up], "stamp": {"sec": 0, "nanosec": 0}}
    manager.process_observation(obs_a, current_time_sec=0.0)
    snap_a = manager.process_observation(obs_a, current_time_sec=1.2)
    assert manager.locked is True
    assert manager.locked_track_id == 5

    # New source session B arrives (e.g. video replay looped or camera switched)
    # Even if session B has a person with track_id=5, lock MUST NOT HOP!
    obs_b = {"source_session_id": "session-B", "candidates": [c_up], "stamp": {"sec": 10, "nanosec": 0}}
    snap_b = manager.process_observation(obs_b, current_time_sec=10.0)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert manager.reason_code == "SOURCE_RESET_DISCONNECT"


def test_mock_voice_departure_rejection_contract():
    """Verify Voice gate contract: destination requests rejected when unlocked, accepted when locked."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.0)

    def mock_voice_check_departure(manager_snapshot) -> bool:
        """Mock Voice destination gating logic matching INTERFACES.md Section 2.1."""
        if not manager_snapshot.locked or manager_snapshot.state != "LOCKED_WAIT_DESTINATION":
            return False  # Rejects departure
        return True  # Accepts destination and sends goal to Nav2

    # 1. When IDLE (unlocked): voice rejects
    snap_idle = manager._make_snapshot(0, 0)
    assert mock_voice_check_departure(snap_idle) is False

    # 2. When locked: voice accepts
    c_up = _create_mock_candidate(track_id=7, hand_raised=True)
    obs = {"source_session_id": "session-1", "candidates": [c_up], "stamp": {"sec": 0, "nanosec": 0}}
    manager.process_observation(obs, current_time_sec=0.0)
    snap_locked = manager.process_observation(obs, current_time_sec=1.2)
    assert snap_locked.locked is True
    assert mock_voice_check_departure(snap_locked) is True


def test_tracking_manager_node_smoke():
    """ROS 2 node integration test for TrackingManagerNode."""
    if not rclpy.ok():
        rclpy.init()
    try:
        manager_node = TrackingManagerNode()

        # Critical Safety Audit: Zero /cmd_vel publishers!
        pubs = manager_node.get_publisher_names_and_types_by_node(
            manager_node.get_name(), manager_node.get_namespace()
        )
        pub_topics = [t for t, _ in pubs]
        assert "/cmd_vel" not in pub_topics, "SAFETY VIOLATION: /cmd_vel found in TrackingManagerNode!"
        assert "cmd_vel" not in pub_topics

        # Check publisher for /tracking/session_state
        assert any("/tracking/session_state" in t for t in pub_topics)

        # Spin briefly to verify heartbeat timer
        rclpy.spin_once(manager_node, timeout_sec=0.25)
        assert manager_node.last_snapshot is None or manager_node.last_snapshot.state == "IDLE"

        manager_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
