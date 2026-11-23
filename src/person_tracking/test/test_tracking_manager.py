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

    # Feed frames with ascending timestamps up to 1.0s at 10 Hz (step 0.1s)
    cand = _create_mock_candidate(track_id=1, hand_raised=True)
    for i in range(11):  # t = 0.0 to 1.0s
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-1", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs, current_time_sec=t_sec)
        assert manager.locked is False
        assert manager.state == TrackingState.IDLE

    assert 1 in manager.gesture_raise_start

    # Frame at t = 1.1s with hand lowered -> timer resets
    cand_down = _create_mock_candidate(track_id=1, hand_raised=False)
    obs_down = {"source_session_id": "session-1", "candidates": [cand_down], "stamp": {"sec": 1, "nanosec": 100000000}}
    snap_down = manager.process_observation(obs_down, current_time_sec=1.1)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert 1 not in manager.gesture_raise_start  # Timer cleanly reset!


def test_t03_valid_gesture_locks_and_hand_drop_retains_lock():
    """T03: Hand raised for >= 1.5s with sequential advancing frames locks; lowering hand retains lock."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    cand_up = _create_mock_candidate(track_id=2, hand_raised=True, depth_m=1.65, depth_valid=True)

    # Feed frames with advancing timestamps from t = 0.0 to 1.6s at 10 Hz (step 0.1s <= 0.25s threshold)
    for i in range(17):  # 17 steps: t = 0.0 to 1.6s
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-1", "candidates": [cand_up], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs, current_time_sec=t_sec)
        if t_sec < 1.5:
            assert manager.locked is False
        else:
            assert manager.locked is True

    assert manager.locked is True
    assert manager.state == TrackingState.LOCKED_WAIT_DESTINATION
    assert manager.locked_track_id == 2
    assert len(manager.session_id) > 0
    assert manager.locked_distance_m == 1.65
    assert manager.locked_depth_proven is True
    assert manager.reason_code in ["HAND_RAISE_HELD_SINGLE", "LOCKED_TRACKING_ACTIVE"]

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

    # Feed frames advancing from t = 0.0 to 1.6s
    for i in range(17):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-1", "candidates": [c1, c2], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs, current_time_sec=t_sec)

    assert manager.locked is True
    assert manager.locked_track_id == 10  # Nearest candidate (1.4m) selected!
    assert manager.locked_distance_m == 1.40
    assert manager.reason_code in ["HAND_RAISE_HELD_NEAREST", "LOCKED_TRACKING_ACTIVE"]


def test_t05_ambiguous_gesture_no_depth_stays_idle():
    """T05: Two candidates raise hand without depth and identical size -> refuses tentative lock."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    # Identical bounding box size (160x340), depth_valid = False
    c1 = _create_mock_candidate(track_id=1, hand_raised=True, depth_valid=False, bbox_xyxy=[50.0, 80.0, 210.0, 420.0])
    c2 = _create_mock_candidate(track_id=2, hand_raised=True, depth_valid=False, bbox_xyxy=[300.0, 80.0, 460.0, 420.0])

    for i in range(17):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-1", "candidates": [c1, c2], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs, current_time_sec=t_sec)

    assert manager.locked is False
    assert manager.state == TrackingState.IDLE
    assert manager.reason_code == "AMBIGUOUS_GESTURE"


def test_duplicate_stamp_1_6s_apart_does_not_lock():
    """Regression test: Same observation stamp sent twice 1.6s apart MUST NOT accumulate hold time."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    cand = _create_mock_candidate(track_id=1, hand_raised=True)
    obs = {"source_session_id": "session-1", "candidates": [cand], "stamp": {"sec": 10, "nanosec": 0}}

    # Call 1: at t_mono = 0.0s with stamp = 10.0s
    snap1 = manager.process_observation(obs, current_time_sec=0.0)
    assert manager.locked is False

    # Call 2: 1.6s later (t_mono = 1.6s) but with IDENTICAL stamp = 10.0s
    snap2 = manager.process_observation(obs, current_time_sec=1.6)
    assert manager.locked is False, "REGRESSION: Duplicate observation stamp caused false lock!"
    assert manager.state == TrackingState.IDLE


def test_track_id_across_sources_does_not_accumulate():
    """Regression test: Same track_id appearing in source A then source B MUST NOT carry over gesture timer."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)

    cand = _create_mock_candidate(track_id=1, hand_raised=True)

    # In Source A: hand raised for 0.8s (t = 0.0 to 0.8s at 10 Hz)
    for i in range(9):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs_a = {"source_session_id": "source-A", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(obs_a, current_time_sec=t_sec)
        assert manager.locked is False

    # Switch to Source B: track_id 1 appears and raises hand for 0.8s (t = 1.0 to 1.8s)
    for i in range(9):
        t_sec = round(1.0 + i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs_b = {"source_session_id": "source-B", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs_b, current_time_sec=t_sec)
        # Even though total time since t=0 is 1.8s, in source B it's only 0.8s -> MUST NOT LOCK!
        assert manager.locked is False, "REGRESSION: Track ID accumulated hold duration across source sessions!"

    # Now continue in Source B until hold time in source B reaches 1.6s (t = 1.8 to 2.6s)
    for i in range(9):
        t_sec = round(1.9 + i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs_b = {"source_session_id": "source-B", "candidates": [cand], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs_b, current_time_sec=t_sec)

    assert manager.locked is True
    assert manager.locked_source_session_id == "source-B"


def test_frame_gap_resets_gesture_timer():
    """Test that observation gap exceeding max interval (0.25s) resets gesture timer."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand = _create_mock_candidate(track_id=1, hand_raised=True)

    # Frame 1: t = 0.0s
    obs1 = {"source_session_id": "session-1", "candidates": [cand], "stamp": {"sec": 0, "nanosec": 0}}
    manager.process_observation(obs1, current_time_sec=0.0)

    # Frame 2: t = 0.5s (gap of 0.5s > 0.25s)
    obs2 = {"source_session_id": "session-1", "candidates": [cand], "stamp": {"sec": 0, "nanosec": 500000000}}
    manager.process_observation(obs2, current_time_sec=0.5)

    # Timer should have reset on gap: accumulated hold time is 0.0
    cand_key = ("session-1", 1)
    assert manager.gesture_state[cand_key].accumulated_hold_sec == 0.0
    assert manager.locked is False


def test_heartbeat_observation_stamp_distinction_and_staleness():
    """Verify /tracking/session_state separates heartbeat stamp from last observation stamp."""
    if not rclpy.ok():
        rclpy.init()
    try:
        node = TrackingManagerNode()
        cand = _create_mock_candidate(track_id=3, hand_raised=True)

        # Feed observation with optical stamp (sec=100, nanosec=200)
        obs_payload = {
            "source_session_id": "sess-hb",
            "candidates": [cand],
            "stamp": {"sec": 100, "nanosec": 200}
        }
        msg = String(data=json.dumps(obs_payload))
        node._on_observation(msg)

        assert node.last_snapshot is not None
        assert node.last_snapshot.last_observation_stamp == {"sec": 100, "nanosec": 200}
        assert node.last_snapshot.obs_fresh is True

        # Heartbeat check
        node._on_heartbeat()
        hb_payload = node.manager.to_json_dict(node.last_snapshot)
        assert hb_payload["last_observation_stamp"] == {"sec": 100, "nanosec": 200}
        assert "obs_fresh" in hb_payload

        # Simulate staleness (> 0.4s)
        node.manager.last_observation_mono_time = time.monotonic() - 0.5
        node._on_heartbeat()
        assert node.last_snapshot.obs_fresh is False

        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_source_reset_invalidates_session_no_id_hopping():
    """Verify source reset (new source_session_id) resets lock to IDLE and prevents ID hopping."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.0)

    # Lock Candidate 5 on session A by advancing frames for 1.2s
    c_up = _create_mock_candidate(track_id=5, hand_raised=True)
    for i in range(13):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs_a = {"source_session_id": "session-A", "candidates": [c_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(obs_a, current_time_sec=t_sec)
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
    for i in range(13):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-1", "candidates": [c_up], "stamp": {"sec": sec, "nanosec": nsec}}
        snap_locked = manager.process_observation(obs, current_time_sec=t_sec)
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


def test_repeated_zero_stamps_over_1_5s_never_locks():
    """Section A.1: Multiple frames repeating stamp 0/0 over 2.0s MUST NOT accumulate time or lock."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand_up = _create_mock_candidate(track_id=1, hand_raised=True)

    # Send 25 frames, all with stamp (0, 0) spaced 0.1s apart in wall-clock time
    for i in range(25):
        t_sec = round(i * 0.1, 2)
        obs = {
            "source_session_id": "session-zero",
            "candidates": [cand_up],
            "stamp": {"sec": 0, "nanosec": 0},
        }
        snap = manager.process_observation(obs, current_time_sec=t_sec)
        assert manager.locked is False, f"SAFETY VIOLATION: Locked on repeating stamp 0/0 at step {i}!"
        assert manager.state == TrackingState.IDLE


def test_missing_or_invalid_stamps_never_accumulate():
    """Section A.1: Missing, negative, or unparseable stamps are rejected without accumulating."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand_up = _create_mock_candidate(track_id=1, hand_raised=True)

    # Frame with missing stamp
    obs_no_stamp = {"source_session_id": "session-bad", "candidates": [cand_up]}
    snap1 = manager.process_observation(obs_no_stamp, current_time_sec=1.0)
    assert manager.locked is False

    # Frame with negative stamp
    obs_neg_stamp = {"source_session_id": "session-bad", "candidates": [cand_up], "stamp": {"sec": -1, "nanosec": 0}}
    snap2 = manager.process_observation(obs_neg_stamp, current_time_sec=2.0)
    assert manager.locked is False


def test_locked_passenger_continuous_presence_past_0_4s_remains_fresh():
    """Section A.1: Passenger present continuously past 0.4s remains fresh."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand_up = _create_mock_candidate(track_id=3, hand_raised=True, depth_m=1.8, depth_valid=True)

    # Feed frames to lock at t=1.5s
    for i in range(16):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-fresh", "candidates": [cand_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(obs, current_time_sec=t_sec)

    assert manager.locked is True
    assert manager.locked_track_id == 3

    # Continue feeding frames up to t=2.5s (1.0s after lock)
    for i in range(16, 26):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        cand_track = _create_mock_candidate(track_id=3, hand_raised=False, depth_m=1.8, depth_valid=True)
        obs = {"source_session_id": "session-fresh", "candidates": [cand_track], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs, current_time_sec=t_sec)
        assert snap.locked is True
        assert snap.obs_fresh is True
        assert snap.reason_code == "LOCKED_TRACKING_ACTIVE"


def test_empty_frames_cause_passenger_freshness_timeout_despite_advancing_stamps():
    """Section A.1: Empty frames (or different person) cause passenger freshness to expire after 0.4s despite advancing stamps."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand_up = _create_mock_candidate(track_id=4, hand_raised=True, depth_m=1.8, depth_valid=True)

    # Feed frames to lock at t=1.5s
    for i in range(16):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-empty-test", "candidates": [cand_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(obs, current_time_sec=t_sec)

    assert manager.locked is True

    # Now feed frames with advancing stamps, but candidates is EMPTY
    # At t=1.6s to 1.8s (unseen for <= 0.30s): grace / temporarily unseen
    for i in range(16, 19):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs_empty = {"source_session_id": "session-empty-test", "candidates": [], "stamp": {"sec": sec, "nanosec": nsec}}
        snap = manager.process_observation(obs_empty, current_time_sec=t_sec)

    # At t=2.0s (unseen for 0.5s > 0.40s): passenger is no longer fresh!
    t_sec = 2.0
    sec = 2
    nsec = 0
    obs_empty = {"source_session_id": "session-empty-test", "candidates": [], "stamp": {"sec": sec, "nanosec": nsec}}
    snap_stale = manager.process_observation(obs_empty, current_time_sec=t_sec)

    assert snap_stale.locked is True
    assert snap_stale.obs_fresh is False
    assert snap_stale.reason_code == "PASSENGER_TEMPORARILY_UNSEEN"


def test_publisher_freeze_and_heartbeat_staleness():
    """Section A.1: Stream halting (>0.4s) causes staleness; heartbeat alone DOES NOT revive freshness."""
    manager = PassengerLockManager(gesture_hold_time_sec=1.5)
    cand_up = _create_mock_candidate(track_id=5, hand_raised=True, depth_m=1.8, depth_valid=True)

    # Lock at t=1.5s
    for i in range(16):
        t_sec = round(i * 0.1, 2)
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        obs = {"source_session_id": "session-freeze", "candidates": [cand_up], "stamp": {"sec": sec, "nanosec": nsec}}
        manager.process_observation(obs, current_time_sec=t_sec)

    assert manager.locked is True
    assert manager.is_observation_fresh(1.5) is True

    # Publisher dies: 0.5s pass with no observations
    freeze_time = 2.05
    assert manager.is_observation_fresh(freeze_time) is False

    # Heartbeat snapshot at t=2.05s
    snap_hb = manager._make_snapshot(stamp_sec=2, stamp_nanosec=50000000, current_time_sec=freeze_time)
    assert snap_hb.obs_fresh is False
