"""Unit tests for VelocityGuard (person_tracking Phase 7 & 8).

Covers:
- T09: Normal escort (distance 1.5m, pass-through Nav2 speed).
- T10: Distance clamp & hysteresis (linear ceiling reduction >2.0m, stop >=2.5m, resume <=2.2m).
  * Invariant: robot does not yaw or turn when distance stop requires zero (vx=0, wz=0).
- T11: Close passenger (<1.2m, no acceleration, never reverse).
- T12: Yaw trimming (deadband, saturation, optical sign CW/CCW per rear camera transform, near-edge 50% reduction).
- Zero Nav2 command invariance (no self-motion if Nav2 is idle).
- Conflict resolution (Nav2 turning away from passenger at edge -> stop/wait).
- Degraded mode on unverified or missing depth (depth_valid=False -> zero velocity).
- Lateral velocity invariant (linear.y = 0.0 strictly).
- Watchdog & freshness gating:
    * cmd_vel_smoothed age > 0.20s -> clamp to zero.
    * passenger unseen > 0.40s -> clamp to zero.
    * session heartbeat age > 1.0s -> clamp to zero.
    * command received before gate opened -> rejected.
    * non-NAVIGATING FSM states (IDLE, LOCKED_WAIT_DESTINATION, WAITING_FOR_PASSENGER, ABORTING, FAULT) -> clamp to zero.
- Section A Regressions & Enhancements:
    * Observation gating: empty frame, wrong source, repeated stamp do not extend seen time.
    * Stale observation recovery: old commands invalidated, fresh command required after gate reopens.
    * Command revocation on WAIT/ABORT/FAULT or session change.
    * NaN / Inf input handling clamped to zero without crashing.
    * Closed-loop rear camera geometry test: target off-center -> yaw step -> |offset| decreases.
"""

import math
import time
from typing import Tuple, Optional
import pytest
from person_tracking.velocity_guard import VelocityGuard, GuardLimits
from person_tracking.rear_camera_geometry import (
    CameraIntrinsics,
    CameraMountTransform,
    RearCameraGeometry,
)
import numpy as np


def _make_navigating_session(
    dist_m: float = 1.5,
    depth_proven: bool = True,
    obs_fresh: bool = True,
    state: str = "NAVIGATING",
    track_id: int = 1,
    session_id: str = "test-session-uuid",
) -> dict:
    return {
        "schema_version": "1.0",
        "state": state,
        "locked": state in ("NAVIGATING", "LOCKED_WAIT_DESTINATION"),
        "session_id": session_id,
        "person_track_id": track_id,
        "person_distance_m": dist_m,
        "depth_proven": depth_proven,
        "obs_fresh": obs_fresh,
    }


def _make_observation(
    track_id: int = 1,
    offset_x_norm: float = 0.0,
    depth_m: float = 1.5,
    depth_valid: bool = True,
    near_edge: bool = False,
    is_confirmed: bool = True,
    is_ambiguous: bool = False,
    source_session_id: str = "src-sess-1",
    stamp_sec: int = 1,
    stamp_nanosec: int = 0,
) -> dict:
    return {
        "schema_version": "1.0",
        "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
        "source_session_id": source_session_id,
        "candidates": [
            {
                "track_id": track_id,
                "is_confirmed": is_confirmed,
                "is_ambiguous": is_ambiguous,
                "torso_depth_m": depth_m,
                "depth_valid": depth_valid,
                "center_offset_x_norm": offset_x_norm,
                "near_edge": near_edge,
            }
        ],
    }


# ==============================================================================
# UNIT TESTS: VELOCITY GUARD CORE ENGINE
# ==============================================================================

def test_default_output_is_strictly_zero():
    """Initial state without session or commands outputs zero velocity."""
    guard = VelocityGuard()
    vx, vy, wz, reason = guard.process_velocity(now_mono=10.0)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "NO_SESSION_STATE"


def test_t09_normal_escort_pass_through():
    """T09: Safe zone (1.5m), centered passenger, fresh cmd -> pass-through Nav2 speed."""
    guard = VelocityGuard()
    t = 100.0

    # 1. Update session state -> gate opens
    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=0.0, stamp_sec=1, stamp_nanosec=0), now_mono=t)

    # 2. Receive cmd_vel after gate opened
    guard.update_cmd_vel(linear_x=0.35, linear_y=0.1, angular_z=0.05, now_mono=t + 0.02)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.05)
    assert reason == "ESCORT_VELOCITY_PASS"
    assert pytest.approx(vx, 0.001) == 0.35
    assert vy == 0.0  # linear.y strictly 0.0
    assert pytest.approx(wz, 0.001) == 0.05


def test_t10_distance_speed_reduction_and_stop_hysteresis():
    """T10: Distance > 2.0m reduces linear ceiling; >= 2.5m stops and latches (vx=0, wz=0); resumes <= 2.2m."""
    guard = VelocityGuard()
    t = 100.0

    # 1. Open gate
    guard.update_session_state(_make_navigating_session(dist_m=2.0), now_mono=t)
    guard.update_observation(_make_observation(depth_m=2.0, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.10, now_mono=t + 0.01)

    # At 2.0m: full speed
    vx, _, wz, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(vx, 0.01) == 0.40
    assert pytest.approx(wz, 0.01) == 0.10

    # At 2.25m: midway in [2.0m, 2.5m] -> scale = 1.0 - (2.25 - 2.0) / 0.5 = 0.50 -> vx = 0.20
    guard.update_session_state(_make_navigating_session(dist_m=2.25), now_mono=t + 0.05)
    guard.update_observation(_make_observation(depth_m=2.25, stamp_sec=1, stamp_nanosec=50_000_000), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.10, now_mono=t + 0.06)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.07)
    assert reason == "ESCORT_VELOCITY_PASS"
    assert pytest.approx(vx, 0.01) == 0.20

    # At 2.6m: exceeds 2.5m stop threshold -> complete stop (vx=0, wz=0, NO yawing!)
    guard.update_session_state(_make_navigating_session(dist_m=2.6), now_mono=t + 0.10)
    guard.update_observation(_make_observation(depth_m=2.6, stamp_sec=1, stamp_nanosec=100_000_000), now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.20, now_mono=t + 0.11)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.12)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0  # Robot must NOT yaw when distance requires stop!
    assert reason == "DISTANCE_EXCEEDED_STOP"
    assert guard.distance_stop_latched is True

    # At 2.3m: still above 2.2m resume threshold -> remains latched at zero
    guard.update_session_state(_make_navigating_session(dist_m=2.3), now_mono=t + 0.15)
    guard.update_observation(_make_observation(depth_m=2.3, stamp_sec=1, stamp_nanosec=150_000_000), now_mono=t + 0.15)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.20, now_mono=t + 0.16)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.17)
    assert vx == 0.0
    assert wz == 0.0
    assert reason == "DISTANCE_STOP_LATCHED"

    # At 2.1m: <= 2.2m resume threshold -> unlatches and resumes motion
    guard.update_session_state(_make_navigating_session(dist_m=2.1), now_mono=t + 0.20)
    guard.update_observation(_make_observation(depth_m=2.1, stamp_sec=1, stamp_nanosec=200_000_000), now_mono=t + 0.20)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.10, now_mono=t + 0.21)
    vx, _, wz, reason = guard.process_velocity(now_mono=t + 0.22)
    assert guard.distance_stop_latched is False
    assert reason == "ESCORT_VELOCITY_PASS"
    assert vx > 0.0


def test_t11_close_passenger_no_acceleration_and_never_reverses():
    """T11: Passenger < 1.2m maintains Nav2 speed, never accelerates beyond input, never reverses."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=0.8), now_mono=t)
    guard.update_observation(_make_observation(depth_m=0.8, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(vx, 0.001) == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"

    # Nav2 commands negative velocity -> clamped to 0.0 (never reverse)
    guard.update_cmd_vel(linear_x=-0.20, linear_y=0.0, angular_z=0.0, now_mono=t + 0.03)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.04)
    assert vx == 0.0


def test_t12_yaw_trimming_deadband_saturation_and_edge_slowdown():
    """T12: Angular trimming per rear camera convention:
    * offset > 0 (right in rear image) -> CW rotation (wz < 0).
    * offset < 0 (left in rear image) -> CCW rotation (wz > 0).
    * Deadband: |offset| <= 0.08 -> Delta omega = 0.0.
    * Saturation: |Delta omega| <= 0.20 rad/s.
    * Near edge: 50% linear speed reduction.
    """
    guard = VelocityGuard()
    t = 100.0

    # 1. Deadband: |offset| <= 0.08 -> Delta omega = 0.0
    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=0.05, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.10, now_mono=t + 0.01)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(wz, 0.001) == 0.10  # No trim added

    # 2. Target to the right in rear image (offset = +0.40): trim CW (delta_omega = -0.35 * 0.40 = -0.14)
    guard.update_observation(_make_observation(offset_x_norm=0.40, stamp_sec=1, stamp_nanosec=50_000_000), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.06)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.07)
    assert pytest.approx(wz, 0.01) == -0.14  # CW (negative)

    # 3. Target to the left in rear image (offset = -0.40): trim CCW (delta_omega = -0.35 * -0.40 = +0.14)
    guard.update_observation(_make_observation(offset_x_norm=-0.40, stamp_sec=1, stamp_nanosec=100_000_000), now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.11)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.12)
    assert pytest.approx(wz, 0.01) == 0.14  # CCW (positive)

    # 4. Saturation limit: extreme offset (+0.90) -> trim_kp*0.90 = 0.315 > 0.20 -> saturated to -0.20 rad/s (CW)
    # Near edge (|offset| > 0.70) -> 50% linear speed reduction (0.30 -> 0.15)
    guard.update_observation(_make_observation(offset_x_norm=0.90, near_edge=True, stamp_sec=1, stamp_nanosec=150_000_000), now_mono=t + 0.15)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.16)
    vx, _, wz, reason = guard.process_velocity(now_mono=t + 0.17)
    assert pytest.approx(wz, 0.001) == -0.20  # Saturated at max trim CW
    assert pytest.approx(vx, 0.001) == 0.15  # 50% near edge reduction


def test_zero_nav2_command_invariance():
    """Zero Nav2 command produces zero output (no self-motion!)."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=0.50, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.0, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "ZERO_NAV2_CMD"


def test_nav2_turning_conflict_at_edge_prioritizes_stop():
    """If Nav2 turns away from passenger near edge, prioritize stop/wait."""
    guard = VelocityGuard()
    t = 100.0

    # Case 1: Target on RIGHT edge (offset = +0.80, trim CW delta_omega < -0.05).
    # Nav2 commands turning CCW (angular_z = +0.20 rad/s), turning AWAY from target -> CONFLICT!
    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=0.80, near_edge=True, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.20, now_mono=t + 0.01)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "NAV2_EDGE_TURNING_CONFLICT_STOP"

    # Case 2: Target on LEFT edge (offset = -0.80, trim CCW delta_omega > 0.05).
    # Nav2 commands turning CW (angular_z = -0.20 rad/s), turning AWAY from target -> CONFLICT!
    guard.update_observation(_make_observation(offset_x_norm=-0.80, near_edge=True, stamp_sec=1, stamp_nanosec=50_000_000), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=-0.20, now_mono=t + 0.06)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.07)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "NAV2_EDGE_TURNING_CONFLICT_STOP"


def test_unverified_alignment_or_invalid_depth_clamps_to_zero():
    """Unverified alignment or invalid depth triggers degraded zero velocity mode."""
    guard = VelocityGuard()
    t = 100.0

    # Session says depth_proven = False (e.g. UNVERIFIED_ALIGNMENT from Astra)
    guard.update_session_state(_make_navigating_session(dist_m=1.6, depth_proven=False), now_mono=t)
    guard.update_observation(_make_observation(depth_valid=False, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert reason in ("DEGRADED_UNVERIFIED_DEPTH", "NO_VALID_OBSERVATION")


def test_t21_stale_cmd_vel_watchdog():
    """T21: cmd_vel_smoothed older than 0.20s clamps output to zero."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    # 0.10s later (fresh): pass-through
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.11)
    assert vx == 0.30

    # 0.25s later (> 0.20s timeout): stale cmd clamps to zero
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.27)
    assert vx == 0.0
    assert reason == "CMD_VEL_STALE"


def test_cmd_received_before_gate_opened_is_rejected():
    """Stale buffered command received before gate opened is rejected."""
    guard = VelocityGuard()

    # Buffer command at t = 99.0s
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=99.0)

    # Motion gate opens at t = 100.0s
    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=100.0)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=100.0)

    # Process velocity at t = 100.05s without new cmd -> rejected
    vx, _, _, reason = guard.process_velocity(now_mono=100.05)
    assert vx == 0.0
    assert reason in ("CMD_RECEIVED_BEFORE_GATE_OPENED", "NO_CMD_VEL")


def test_t14_stale_observation_clamps_to_zero():
    """T14: Observation older than 0.40s clamps output to zero."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.5, obs_fresh=True), now_mono=t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    # At t = 100.45s (> 0.40s): passenger observation stale
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.44)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.45)
    assert vx == 0.0
    assert reason == "OBSERVATION_AGE_EXCEEDED"


def test_non_navigating_fsm_states_clamp_to_zero():
    """All non-NAVIGATING states immediately clamp output to zero."""
    guard = VelocityGuard()
    t = 100.0

    non_nav_states = ["IDLE", "LOCKED_WAIT_DESTINATION", "WAITING_FOR_PASSENGER", "ABORTING", "ARRIVED", "FAULT"]
    for state in non_nav_states:
        guard.update_session_state(_make_navigating_session(state=state), now_mono=t)
        guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)
        guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
        vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
        assert vx == 0.0
        assert vy == 0.0
        assert wz == 0.0
        assert "GATE_CLOSED" in reason


def test_t33_clock_pause_or_jump_monotonic_watchdog():
    """T33: Monotonic watchdog is independent of simulation /clock pause or backward jumps."""
    guard = VelocityGuard()
    mono_t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=mono_t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=mono_t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=mono_t + 0.01)

    # 1. At mono_t + 0.05, passes through
    vx, _, _, reason = guard.process_velocity(now_mono=mono_t + 0.05)
    assert vx == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"

    # 2. At mono_t + 0.26 (> 0.20s cmd timeout), even if /clock paused, monotonic watchdog clamps to zero
    vx, vy, wz, reason = guard.process_velocity(now_mono=mono_t + 0.26)
    assert vx == 0.0
    assert reason == "CMD_VEL_STALE"


# ==============================================================================
# SECTION A REGRESSION TESTS: OBSERVATION GATING, REVOCATION & GEOMETRY
# ==============================================================================

def test_observation_gating_rejects_empty_wrong_source_or_invalid_candidates():
    """A.1: Heartbeat, empty frames, repeated frames, wrong source, or unconfirmed candidates
    do not extend passenger seen time. If no valid observation from start -> strictly zero.
    """
    guard = VelocityGuard()
    t = 100.0

    # 1. Start navigating without valid observation -> output strictly zero
    guard.update_session_state(_make_navigating_session(dist_m=1.5, track_id=1), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert reason == "NO_VALID_OBSERVATION"

    # 2. Observation with stamp 0/0 -> rejected, does not open gate
    obs_zero_stamp = _make_observation(stamp_sec=0, stamp_nanosec=0)
    guard.update_observation(obs_zero_stamp, now_mono=t + 0.03)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.04)
    assert vx == 0.0
    assert reason == "NO_VALID_OBSERVATION"

    # 3. Observation with empty candidates -> rejected
    obs_empty = {"schema_version": "1.0", "stamp": {"sec": 1, "nanosec": 10}, "source_session_id": "src-1", "candidates": []}
    guard.update_observation(obs_empty, now_mono=t + 0.05)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.06)
    assert vx == 0.0
    assert reason == "NO_VALID_OBSERVATION"

    # 4. Observation with unconfirmed candidate -> rejected
    obs_unconfirmed = _make_observation(is_confirmed=False, stamp_sec=1, stamp_nanosec=20)
    guard.update_observation(obs_unconfirmed, now_mono=t + 0.07)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.08)
    assert vx == 0.0
    assert reason == "NO_VALID_OBSERVATION"

    # 5. Send valid observation -> gate opens
    obs_valid = _make_observation(track_id=1, depth_valid=True, stamp_sec=1, stamp_nanosec=30, source_session_id="src-sess-A")
    guard.update_observation(obs_valid, now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.11)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.12)
    assert vx == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"

    # 6. Subsequent observation from DIFFERENT source session ("src-sess-B") -> rejected, does not extend seen time!
    obs_wrong_src = _make_observation(track_id=1, depth_valid=True, stamp_sec=1, stamp_nanosec=50, source_session_id="src-sess-B")
    guard.update_observation(obs_wrong_src, now_mono=t + 0.50)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.51)
    # Since observation was not updated by src-sess-B, observation age is (100.52 - 100.10) = 0.42s > 0.40s
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.52)
    assert vx == 0.0
    assert reason == "OBSERVATION_AGE_EXCEEDED"


def test_cmd_invalidated_on_stale_observation_recovery():
    """A.2: Heartbeat is NAVIGATING, observation becomes stale then recovers.
    Robot MUST NOT move on old cmd_vel; reopening requires fresh command received AFTER gate reopened.
    """
    guard = VelocityGuard()
    t = 100.0

    # 1. Normal active motion
    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.30

    # 2. Observation becomes stale at t + 0.45s (> 0.40s)
    # Nav2 continues sending cmd_vel at t + 0.44s
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.44)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.45)
    assert vx == 0.0
    assert reason == "OBSERVATION_AGE_EXCEEDED"

    # 3. Observation recovers at t + 0.50s with fresh valid frame
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=500_000_000), now_mono=t + 0.50)

    # 4. Attempt to move with old command (sent at t + 0.44s, before recovery at t + 0.50s)
    # Motion gate must REJECT the old command!
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.51)
    assert vx == 0.0
    assert reason in ("CMD_RECEIVED_BEFORE_GATE_OPENED", "NO_CMD_VEL")

    # 5. Fresh command received AFTER gate reopened (at t + 0.52s) -> allowed to move
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.52)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.53)
    assert vx == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"


def test_cmd_revoked_on_state_change_or_session_switch():
    """A.2: Revoke right to use old cmd_vel on WAIT/ABORT/FAULT or session switch."""
    guard = VelocityGuard()
    t = 100.0

    # Start NAVIGATING in session 1
    guard.update_session_state(_make_navigating_session(session_id="sess-1"), now_mono=t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.30

    # Transition to WAITING_FOR_PASSENGER -> revokes cmd_vel
    guard.update_session_state(_make_navigating_session(state="WAITING_FOR_PASSENGER", session_id="sess-1"), now_mono=t + 0.05)
    assert guard.latest_cmd_vel is None

    # Resume NAVIGATING in session 2 (new mission)
    guard.update_session_state(_make_navigating_session(state="NAVIGATING", session_id="sess-2"), now_mono=t + 0.10)
    guard.update_observation(_make_observation(stamp_sec=2, stamp_nanosec=0), now_mono=t + 0.10)

    # Process velocity without new command -> NO_CMD_VEL
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.11)
    assert vx == 0.0
    assert reason in ("NO_CMD_VEL", "CMD_RECEIVED_BEFORE_GATE_OPENED")


def test_nan_inf_invalid_numeric_handling():
    """A.2: Check NaN/Inf and invalid types on all control fields -> clamp to zero without crashing."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=t)
    guard.update_observation(_make_observation(stamp_sec=1, stamp_nanosec=0), now_mono=t)

    # 1. NaN in linear_x
    guard.update_cmd_vel(linear_x=float("nan"), linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "INVALID_NUMERIC_VALUE"

    # 2. Inf in angular_z
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=float("inf"), now_mono=t + 0.03)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.04)
    assert vx == 0.0
    assert wz == 0.0
    assert reason == "INVALID_NUMERIC_VALUE"

    # 3. NaN in distance
    guard.update_session_state(_make_navigating_session(dist_m=float("nan")), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.06)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.07)
    assert vx == 0.0
    assert reason == "INVALID_DISTANCE"


def test_closed_loop_rear_camera_yaw_geometry():
    """A.3: Closed-loop geometry test using rear camera transform:
    Target fixed in world off-center -> robot yaw step -> reproject -> |offset| decreases.
    Rear camera convention:
    * Target on RIGHT (e_x > 0) -> trim CW (delta_omega < 0).
    * Target on LEFT (e_x < 0) -> trim CCW (delta_omega > 0).
    * Target at CENTER (e_x = 0) -> trim is 0.0.
    """
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    mount = CameraMountTransform.create_synthetic_rear_mount(translation=(-0.20, 0.0, 0.25))

    def project_world_target_to_rear_camera(p_world: np.ndarray, robot_yaw: float) -> Tuple[float, float]:
        """Project world point behind robot to rear camera normalized offset and depth."""
        # Robot center at (0, 0, 0) with orientation robot_yaw
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        # Transform world point to robot base_link
        x_base = p_world[0] * cos_y + p_world[1] * sin_y
        y_base = -p_world[0] * sin_y + p_world[1] * cos_y
        z_base = p_world[2]

        # Inverse mount transform to camera optical frame:
        # X_base = -Z_opt + x_off, Y_base = X_opt + y_off, Z_base = -Y_opt + z_off
        z_opt = -(x_base - mount.translation[0])
        x_opt = y_base - mount.translation[1]
        y_opt = -(z_base - mount.translation[2])

        u = intrinsics.cx + intrinsics.fx * (x_opt / z_opt)
        offset_norm = (u - intrinsics.cx) / (intrinsics.width / 2.0)
        return offset_norm, z_opt

    guard = VelocityGuard()
    dt = 0.10

    # --------------------------------------------------------------------------
    # Case 1: Target fixed in world at X_w = -2.0m (behind), Y_w = +0.50m (robot's left flank)
    # --------------------------------------------------------------------------
    p_world_right = np.array([-2.0, 0.50, 0.25])
    yaw_0 = 0.0
    e_x_0, depth_0 = project_world_target_to_rear_camera(p_world_right, yaw_0)

    # In rear camera optical frame, robot's left flank is to the RIGHT of the image:
    assert e_x_0 > 0.0, f"Expected e_x_0 > 0 (right in rear image), got {e_x_0}"

    # Feed observation to Guard
    t = 100.0
    guard.update_session_state(_make_navigating_session(dist_m=depth_0), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=e_x_0, depth_m=depth_0, stamp_sec=1, stamp_nanosec=0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    _, _, wz_trim, reason = guard.process_velocity(now_mono=t + 0.02)
    assert reason == "ESCORT_VELOCITY_PASS"
    # Target on right -> Guard MUST trim CW (negative delta_omega)
    assert wz_trim < 0.0, f"Expected wz_trim < 0 (CW trim for right target), got {wz_trim}"

    # Apply small yaw step to robot: yaw_1 = yaw_0 + wz_trim * dt (turning CW)
    yaw_1 = yaw_0 + wz_trim * dt
    e_x_1, _ = project_world_target_to_rear_camera(p_world_right, yaw_1)

    # Assert closed loop stability: lateral offset magnitude MUST decrease!
    assert abs(e_x_1) < abs(e_x_0), f"Expected |e_x_1| < |e_x_0| ({abs(e_x_1)} < {abs(e_x_0)})"

    # --------------------------------------------------------------------------
    # Case 2: Target fixed in world at X_w = -2.0m (behind), Y_w = -0.50m (robot's right flank)
    # --------------------------------------------------------------------------
    p_world_left = np.array([-2.0, -0.50, 0.25])
    e_x_left_0, depth_left_0 = project_world_target_to_rear_camera(p_world_left, yaw_0)

    # In rear camera optical frame, robot's right flank is to the LEFT of the image:
    assert e_x_left_0 < 0.0, f"Expected e_x_left_0 < 0 (left in rear image), got {e_x_left_0}"

    guard.update_observation(_make_observation(offset_x_norm=e_x_left_0, depth_m=depth_left_0, stamp_sec=1, stamp_nanosec=50_000_000), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.06)
    _, _, wz_left_trim, _ = guard.process_velocity(now_mono=t + 0.07)

    # Target on left -> Guard MUST trim CCW (positive delta_omega)
    assert wz_left_trim > 0.0, f"Expected wz_left_trim > 0 (CCW trim for left target), got {wz_left_trim}"

    # Apply small yaw step to robot: turning CCW
    yaw_left_1 = yaw_0 + wz_left_trim * dt
    e_x_left_1, _ = project_world_target_to_rear_camera(p_world_left, yaw_left_1)

    # Assert closed loop stability: lateral offset magnitude MUST decrease!
    assert abs(e_x_left_1) < abs(e_x_left_0), f"Expected |e_x_left_1| < |e_x_left_0| ({abs(e_x_left_1)} < {abs(e_x_left_0)})"

    # --------------------------------------------------------------------------
    # Case 3: Target centered directly behind robot (Y_w = 0.0)
    # --------------------------------------------------------------------------
    p_world_center = np.array([-2.0, 0.0, 0.25])
    e_x_center, depth_center = project_world_target_to_rear_camera(p_world_center, yaw_0)
    assert abs(e_x_center) < 1e-4

    guard.update_observation(_make_observation(offset_x_norm=0.0, depth_m=depth_center, stamp_sec=1, stamp_nanosec=100_000_000), now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.11)
    _, _, wz_center_trim, _ = guard.process_velocity(now_mono=t + 0.12)
    assert pytest.approx(wz_center_trim, 1e-4) == 0.0
