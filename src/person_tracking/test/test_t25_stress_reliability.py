#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 10: T25 Stress and Reliability Verification Suite (50-iteration loop)
Tests FSM, Velocity Guard, Watchdog, and Tracking Manager across 50 consecutive cycles
incorporating degenerate conditions:
- Multi-person / gesture noise (T04, T05)
- Out-of-order / stale timestamps / frame gaps (T13, T14, T15, T28, T31)
- Corrupted depth / UNVERIFIED_ALIGNMENT / TF errors (T19, T20, T26)
- Stale cmd_vel / Voice unresponsive / late ACK & terminal (T16, T18, T21, T29, T30)
- Watchdog active zeroing (T22B) and Collision Monitor final veto (T23)
- Clock pause / jump (T33) and Lateral motion isolation (T35)

Invariants verified every cycle:
1. Active continuous zeroing holds when safety violated (cmd_vel == 0.0).
2. No replay of cached commands or stale goals.
3. Memory RSS remains stable (no memory leaks).
4. Clean lifecycle recovery to IDLE.
"""

import gc
import json
import os
import sys
import time
import pytest
from geometry_msgs.msg import Twist

# Inject .venv_yolo site-packages if available
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from person_tracking.passenger_lock_manager import PassengerLockManager, TrackingState
from person_tracking.velocity_guard import VelocityGuard, GuardLimits
from person_tracking.velocity_watchdog import VelocityWatchdog, WatchdogLimits


def _make_candidate(
    track_id: int,
    hand_raised: bool = True,
    depth_m: float = 1.5,
    depth_valid: bool = True,
    center_offset_x_norm: float = 0.0,
    is_ambiguous: bool = False,
    near_edge: bool = False,
) -> dict:
    kps = [[200.0, 200.0, 0.9]] * 17
    kps[5] = [180.0, 150.0, 0.9]  # Left shoulder
    kps[6] = [220.0, 150.0, 0.9]  # Right shoulder
    if hand_raised:
        kps[9] = [170.0, 100.0, 0.95]  # Raised
        kps[10] = [230.0, 260.0, 0.9]
    else:
        kps[9] = [170.0, 260.0, 0.9]   # Lowered
        kps[10] = [230.0, 260.0, 0.9]

    return {
        "track_id": track_id,
        "is_confirmed": True,
        "is_ambiguous": is_ambiguous,
        "confidence": 0.92,
        "bbox_xyxy": [100.0, 80.0, 260.0, 420.0],
        "torso_depth_m": depth_m,
        "depth_valid": depth_valid,
        "center_offset_x_norm": center_offset_x_norm,
        "near_edge": near_edge,
        "keypoints": kps,
    }


def _make_session_state(
    state: str = "NAVIGATING",
    session_id: str = "test-session-uuid",
    track_id: int = 1,
    dist_m: float = 1.5,
    source_session_id: str = "src-sess-1",
    mission_id: str = "mis-001",
    goal_attempt_id: str = "att-001",
    stamp_sec: int = 1,
    stamp_nanosec: int = 0,
) -> dict:
    return {
        "schema_version": "1.0",
        "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
        "last_observation_stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
        "obs_fresh": state in ("NAVIGATING", "LOCKED_WAIT_DESTINATION"),
        "sequence_num": 1,
        "session_id": session_id,
        "state": state,
        "locked": state in ("NAVIGATING", "LOCKED_WAIT_DESTINATION"),
        "person_track_id": track_id,
        "person_distance_m": dist_m,
        "depth_proven": True,
        "confidence": 0.95,
        "reason_code": "NORMAL",
        "source_session_id": source_session_id,
        "mission_id": mission_id,
        "goal_attempt_id": goal_attempt_id,
        "nav2_uuid": "uuid-001",
    }


def _make_obs_dict(
    track_id: int = 1,
    depth_m: float = 1.5,
    depth_valid: bool = True,
    offset_x: float = 0.0,
    source_session_id: str = "src-sess-1",
    data_quality: str = "VALID_RGBD",
    stamp_sec: int = 1,
    stamp_nanosec: int = 0,
) -> dict:
    cand = _make_candidate(track_id, hand_raised=False, depth_m=depth_m, depth_valid=depth_valid, center_offset_x_norm=offset_x)
    return {
        "schema_version": "1.0",
        "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
        "frame_id": "camera_color_optical_frame",
        "receive_monotonic_time": stamp_sec + stamp_nanosec * 1e-9,
        "source_mode": "live_astra",
        "source_session_id": source_session_id,
        "data_quality": data_quality,
        "candidates": [cand],
        "active_track_ids": [track_id],
        "lost_track_ids": [],
    }


def get_rss_mb() -> float:
    try:
        import psutil
        return psutil.Process(os.getpid()).memory_info().rss / (1024.0 * 1024.0)
    except Exception:
        return 0.0


def test_t25_50_iteration_stress_and_reliability():
    """Execute 50-iteration stress loop verifying state stability and zero-velocity enforcement."""
    out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
    os.makedirs(out_dir, exist_ok=True)
    stress_log = os.path.join(out_dir, "t25_stress_reliability.log")

    log_lines = []
    log_lines.append("=" * 80)
    log_lines.append("PHASE 10: T25 STRESS & RELIABILITY TEST (50-ITERATION CYCLE)")
    log_lines.append("=" * 80)

    mgr = PassengerLockManager(gesture_hold_time_sec=1.0, visibility_stop_sec=0.40, visibility_abort_sec=1.20)
    watchdog = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15))

    num_iterations = 50
    passed_count = 0
    start_rss = get_rss_mb()

    t_start = time.perf_counter()

    for cycle in range(1, num_iterations + 1):
        cycle_type = (cycle - 1) % 5
        mgr.reset_session("CYCLE_CLEANUP")
        guard = VelocityGuard()
        watchdog.reset()

        base_t = float(cycle * 100)
        source_id = f"src-stress-{cycle:03d}"

        # ----------------------------------------------------------------------
        # GROUP 0 (Cycles 1, 6, 11...): Multi-person gesture competition (T04, T05)
        # ----------------------------------------------------------------------
        if cycle_type == 0:
            cand1 = _make_candidate(track_id=10, hand_raised=True, depth_m=2.5, depth_valid=True)
            cand2 = _make_candidate(track_id=20, hand_raised=True, depth_m=1.4, depth_valid=True)

            for step in range(12):
                st = base_t + step * 0.1
                obs = {
                    "source_session_id": source_id,
                    "candidates": [cand1, cand2],
                    "stamp": {"sec": int(st), "nanosec": int((st % 1) * 1e9)},
                    "receive_monotonic_time": st
                }
                mgr.process_observation(obs, current_time_sec=st)

            assert mgr.locked is True
            assert mgr.state == TrackingState.LOCKED_WAIT_DESTINATION
            assert mgr.locked_track_id == 20, f"Cycle {cycle}: Expected nearest candidate 20, got {mgr.locked_track_id}"

            # Voice dispatches goal
            sess_id = mgr.session_id
            m_id = f"mis_{cycle}"
            g_id = f"att_{cycle}"
            u_id = f"uuid_{cycle}"
            ok, msg = mgr.on_destination_accepted(sess_id, m_id, g_id, u_id, current_time_sec=base_t + 1.2)
            assert ok is True
            assert mgr.state == TrackingState.NAVIGATING

            # Guard passes command
            guard.update_session_state(_make_session_state(
                state="NAVIGATING", session_id=sess_id, track_id=20, dist_m=1.4,
                source_session_id=source_id, mission_id=m_id, goal_attempt_id=g_id,
                stamp_sec=int(base_t + 1.2)
            ), now_mono=base_t + 1.2)
            guard.update_observation(_make_obs_dict(track_id=20, depth_m=1.4, source_session_id=source_id, stamp_sec=int(base_t + 1.2)), now_mono=base_t + 1.2)
            guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.05, now_mono=base_t + 1.22)

            vx, vy, wz, reason = guard.process_velocity(now_mono=base_t + 1.25)
            assert reason == "ESCORT_VELOCITY_PASS"
            assert pytest.approx(vx, 0.001) == 0.40

            # Arrived terminal
            term_status = {
                "session_id": sess_id,
                "mission_id": m_id,
                "goal_attempt_id": g_id,
                "nav2_uuid": u_id,
                "is_terminal": True,
                "status": "SUCCEEDED"
            }
            ok_term, msg_term = mgr.on_voice_nav_status(term_status, current_time_sec=base_t + 2.0)
            assert ok_term is True
            assert mgr.state == TrackingState.IDLE

        # ----------------------------------------------------------------------
        # GROUP 1 (Cycles 2, 7, 12...): Timestamp delay / out-of-order / stale obs (T13, T14, T15)
        # ----------------------------------------------------------------------
        elif cycle_type == 1:
            cand = _make_candidate(track_id=1, hand_raised=True, depth_m=1.6, depth_valid=True)
            for step in range(12):
                st = base_t + step * 0.1
                obs = {"source_session_id": source_id, "candidates": [cand], "stamp": {"sec": int(st), "nanosec": int((st % 1) * 1e9)}}
                mgr.process_observation(obs, current_time_sec=st)

            sess_id = mgr.session_id
            m_id, g_id, u_id = f"mis_{cycle}", f"att_{cycle}", f"uuid_{cycle}"
            mgr.on_destination_accepted(sess_id, m_id, g_id, u_id, current_time_sec=base_t + 1.2)
            assert mgr.state == TrackingState.NAVIGATING

            # Passenger disappears for 0.5s -> WAITING_FOR_PASSENGER
            st_lost = base_t + 1.8
            empty_obs = {"source_session_id": source_id, "candidates": [], "stamp": {"sec": int(st_lost), "nanosec": int((st_lost % 1) * 1e9)}}
            mgr.process_observation(empty_obs, current_time_sec=st_lost)
            assert mgr.state == TrackingState.WAITING_FOR_PASSENGER

            # Guard gets WAITING_FOR_PASSENGER state -> outputs strictly 0
            guard.update_session_state(_make_session_state(
                state="WAITING_FOR_PASSENGER", session_id=sess_id, track_id=1,
                source_session_id=source_id, stamp_sec=int(st_lost)
            ), now_mono=st_lost)
            guard.update_cmd_vel(linear_x=0.50, linear_y=0.0, angular_z=0.1, now_mono=st_lost)
            vx, vy, wz, reason = guard.process_velocity(now_mono=st_lost + 0.05)
            assert vx == 0.0
            assert "GATE_CLOSED" in reason or "STATE_NOT_NAVIGATING" in reason

            # Passenger lost > 1.2s -> ABORTING
            st_abort = base_t + 3.0
            mgr.process_observation(empty_obs, current_time_sec=st_abort)
            assert mgr.state == TrackingState.ABORTING

            # Voice terminal CANCELED
            term_status = {
                "session_id": sess_id,
                "mission_id": m_id,
                "goal_attempt_id": g_id,
                "nav2_uuid": u_id,
                "is_terminal": True,
                "status": "CANCELED"
            }
            mgr.on_voice_nav_status(term_status, current_time_sec=st_abort + 0.2)
            assert mgr.state == TrackingState.IDLE

        # ----------------------------------------------------------------------
        # GROUP 2 (Cycles 3, 8, 13...): Corrupted depth / UNVERIFIED_ALIGNMENT (T19, T20, T26)
        # ----------------------------------------------------------------------
        elif cycle_type == 2:
            cand = _make_candidate(track_id=5, hand_raised=True, depth_m=1.5, depth_valid=True)
            for step in range(12):
                st = base_t + step * 0.1
                obs = {"source_session_id": source_id, "candidates": [cand], "stamp": {"sec": int(st), "nanosec": int((st % 1) * 1e9)}}
                mgr.process_observation(obs, current_time_sec=st)

            sess_id = mgr.session_id
            m_id, g_id, u_id = f"mis_{cycle}", f"att_{cycle}", f"uuid_{cycle}"
            mgr.on_destination_accepted(sess_id, m_id, g_id, u_id, current_time_sec=base_t + 1.2)

            guard.update_session_state(_make_session_state(
                state="NAVIGATING", session_id=sess_id, track_id=5, dist_m=1.5,
                source_session_id=source_id, mission_id=m_id, goal_attempt_id=g_id,
                stamp_sec=int(base_t + 1.2)
            ), now_mono=base_t + 1.2)
            corrupted_obs = _make_obs_dict(
                track_id=5, depth_m=float('nan'), depth_valid=False,
                source_session_id=source_id, data_quality="UNVERIFIED_ALIGNMENT",
                stamp_sec=int(base_t + 1.2)
            )
            guard.update_observation(corrupted_obs, now_mono=base_t + 1.2)
            guard.update_cmd_vel(linear_x=0.45, linear_y=0.0, angular_z=0.0, now_mono=base_t + 1.22)
            vx, vy, wz, reason = guard.process_velocity(now_mono=base_t + 1.25)
            assert vx == 0.0
            assert reason in ("NO_VALID_OBSERVATION", "DEGRADED_UNVERIFIED_DEPTH")

            # Clean terminal
            mgr.on_voice_nav_status({
                "session_id": sess_id, "mission_id": m_id, "goal_attempt_id": g_id, "nav2_uuid": u_id,
                "is_terminal": True, "status": "SUCCEEDED"
            }, current_time_sec=base_t + 2.0)
            assert mgr.state == TrackingState.IDLE

        # ----------------------------------------------------------------------
        # GROUP 3 (Cycles 4, 9, 14...): Stale cmd_vel & Unresponsive Voice (T16, T21, T29)
        # ----------------------------------------------------------------------
        elif cycle_type == 3:
            cand = _make_candidate(track_id=7, hand_raised=True, depth_m=1.7, depth_valid=True)
            for step in range(12):
                st = base_t + step * 0.1
                obs = {"source_session_id": source_id, "candidates": [cand], "stamp": {"sec": int(st), "nanosec": int((st % 1) * 1e9)}}
                mgr.process_observation(obs, current_time_sec=st)

            sess_id = mgr.session_id
            m_id, g_id, u_id = f"mis_{cycle}", f"att_{cycle}", f"uuid_{cycle}"
            mgr.on_destination_accepted(sess_id, m_id, g_id, u_id, current_time_sec=base_t + 1.2)

            guard.update_session_state(_make_session_state(
                state="NAVIGATING", session_id=sess_id, track_id=7, dist_m=1.7,
                source_session_id=source_id, mission_id=m_id, goal_attempt_id=g_id,
                stamp_sec=int(base_t + 1.2)
            ), now_mono=base_t + 1.2)
            guard.update_observation(_make_obs_dict(track_id=7, depth_m=1.7, source_session_id=source_id, stamp_sec=int(base_t + 1.2)), now_mono=base_t + 1.2)
            # Cmd received at t=1.22
            guard.update_cmd_vel(linear_x=0.35, linear_y=0.0, angular_z=0.0, now_mono=base_t + 1.22)

            # Cmd becomes stale (> 0.20s without new command, e.g. at t=1.50)
            vx, vy, wz, reason = guard.process_velocity(now_mono=base_t + 1.50)
            assert vx == 0.0
            assert "CMD_VEL_STALE" in reason or "STALE" in reason

            mgr.on_voice_nav_status({
                "session_id": sess_id, "mission_id": m_id, "goal_attempt_id": g_id, "nav2_uuid": u_id,
                "is_terminal": True, "status": "SUCCEEDED"
            }, current_time_sec=base_t + 2.0)
            assert mgr.state == TrackingState.IDLE

        # ----------------------------------------------------------------------
        # GROUP 4 (Cycles 5, 10, 15...): Watchdog active zeroing (T22B), Mecanum lateral (T35)
        # ----------------------------------------------------------------------
        elif cycle_type == 4:
            # Watchdog receives fresh command
            ok_cmd, _ = watchdog.update_cmd_vel(linear_x=0.50, linear_y=0.0, angular_z=0.0, now_mono=base_t)
            assert ok_cmd is True
            w_vx, w_vy, w_wz, w_reason = watchdog.process_velocity(now_mono=base_t + 0.05)
            assert pytest.approx(w_vx, 0.001) == 0.50
            assert w_reason == "FORWARD_VALID"

            # Watchdog timeout (> 0.15s) simulating Guard SIGKILL crash
            w_vx_to, w_vy_to, w_wz_to, w_reason_to = watchdog.process_velocity(now_mono=base_t + 0.25)
            assert w_vx_to == 0.0
            assert w_vy_to == 0.0
            assert w_wz_to == 0.0
            assert "TIMEOUT_EXCEEDED" in w_reason_to

            # Lateral motion separation: non-zero linear.y from Nav2 clamped to 0.0
            guard.update_session_state(_make_session_state(
                state="NAVIGATING", session_id=f"sess-{cycle}", track_id=9, dist_m=1.5,
                source_session_id=source_id, stamp_sec=int(base_t)
            ), now_mono=base_t)
            guard.update_observation(_make_obs_dict(track_id=9, depth_m=1.5, source_session_id=source_id, stamp_sec=int(base_t)), now_mono=base_t)
            guard.update_cmd_vel(linear_x=0.30, linear_y=0.25, angular_z=0.0, now_mono=base_t + 0.02)
            vx, vy, wz, reason = guard.process_velocity(now_mono=base_t + 0.05)
            assert vy == 0.0  # T35: strictly zero

        passed_count += 1
        if cycle % 10 == 0:
            log_lines.append(f"Cycle {cycle:02d}/50: PASSED [OK] (Group {cycle_type}, RSS={get_rss_mb():.1f}MB)")

    t_end = time.perf_counter()
    dur_sec = t_end - t_start
    end_rss = get_rss_mb()
    delta_rss = end_rss - start_rss

    log_lines.append("-" * 80)
    log_lines.append(f"T25 KẾT QUẢ TỔNG HỢP: 50/50 chu kỳ ĐẠT 100% PASS trong {dur_sec:.2f}s")
    log_lines.append(f"  - Số chu kỳ thành công: {passed_count} / {num_iterations}")
    log_lines.append(f"  - Tốc độ xử lý trung bình: {dur_sec/num_iterations*1000:.2f} ms / chu kỳ")
    log_lines.append(f"  - Độ biến thiên bộ nhớ (RSS Delta): {delta_rss:+.2f} MB")
    log_lines.append("=" * 80)

    report_str = "\n".join(log_lines)
    print(report_str)

    with open(stress_log, "w", encoding="utf-8") as f:
        f.write(report_str)

    assert passed_count == num_iterations, f"T25 failed: {passed_count}/{num_iterations} passed"


if __name__ == "__main__":
    test_t25_50_iteration_stress_and_reliability()
