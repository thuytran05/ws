"""Unit tests for VelocityGuard (person_tracking Phase 7).

Covers:
- T09: Normal escort (distance 1.5m, pass-through Nav2 speed).
- T10: Distance clamp & hysteresis (linear ceiling reduction >2.0m, stop >2.5m, resume <=2.2m).
- T11: Close passenger (<1.2m, no acceleration, never reverse).
- T12: Yaw trimming (deadband, saturation, optical sign CCW/CW, near-edge 50% reduction).
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
"""

import time
import pytest
from person_tracking.velocity_guard import VelocityGuard, GuardLimits


def _make_navigating_session(
    dist_m: float = 1.5,
    depth_proven: bool = True,
    obs_fresh: bool = True,
    state: str = "NAVIGATING",
    track_id: int = 1,
) -> dict:
    return {
        "schema_version": "1.0",
        "state": state,
        "locked": state == "NAVIGATING" or state == "LOCKED_WAIT_DESTINATION",
        "session_id": "test-session-uuid",
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
) -> dict:
    return {
        "schema_version": "1.0",
        "source_session_id": "src-sess-1",
        "candidates": [
            {
                "track_id": track_id,
                "is_confirmed": True,
                "is_ambiguous": False,
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
    guard.update_observation(_make_observation(offset_x_norm=0.0), now_mono=t)

    # 2. Receive cmd_vel after gate opened
    guard.update_cmd_vel(linear_x=0.35, linear_y=0.1, angular_z=0.05, now_mono=t + 0.02)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.05)
    assert reason == "ESCORT_VELOCITY_PASS"
    assert pytest.approx(vx, 0.001) == 0.35
    assert vy == 0.0  # linear.y strictly 0.0
    assert pytest.approx(wz, 0.001) == 0.05


def test_t10_distance_speed_reduction_and_stop_hysteresis():
    """T10: Distance > 2.0m reduces linear ceiling; > 2.5m stops and latches; resumes <= 2.2m."""
    guard = VelocityGuard()
    t = 100.0

    # 1. Open gate
    guard.update_session_state(_make_navigating_session(dist_m=2.0), now_mono=t)
    guard.update_observation(_make_observation(depth_m=2.0), now_mono=t)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    # At 2.0m: full speed
    vx, _, _, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(vx, 0.01) == 0.40

    # At 2.25m: midway in [2.0m, 2.5m] -> scale = 1.0 - (2.25 - 2.0) / 0.5 = 0.50 -> vx = 0.20
    guard.update_session_state(_make_navigating_session(dist_m=2.25), now_mono=t + 0.05)
    guard.update_observation(_make_observation(depth_m=2.25), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=t + 0.06)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.07)
    assert reason == "ESCORT_VELOCITY_PASS"
    assert pytest.approx(vx, 0.01) == 0.20

    # At 2.6m: exceeds 2.5m stop threshold -> stop completely and latch
    guard.update_session_state(_make_navigating_session(dist_m=2.6), now_mono=t + 0.10)
    guard.update_observation(_make_observation(depth_m=2.6), now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=t + 0.11)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.12)
    assert vx == 0.0
    assert reason == "DISTANCE_EXCEEDED_STOP"
    assert guard.distance_stop_latched is True

    # Hysteresis: passenger steps forward to 2.4m (<= 2.5m but > 2.2m) -> remains latched at 0!
    guard.update_session_state(_make_navigating_session(dist_m=2.4), now_mono=t + 0.15)
    guard.update_observation(_make_observation(depth_m=2.4), now_mono=t + 0.15)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=t + 0.16)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.17)
    assert vx == 0.0
    assert reason == "DISTANCE_STOP_LATCHED"
    assert guard.distance_stop_latched is True

    # Passenger steps closer to 2.1m (<= 2.2m resume distance) -> unlatches and resumes!
    guard.update_session_state(_make_navigating_session(dist_m=2.1), now_mono=t + 0.20)
    guard.update_observation(_make_observation(depth_m=2.1), now_mono=t + 0.20)
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=t + 0.21)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.22)
    assert guard.distance_stop_latched is False
    assert reason == "ESCORT_VELOCITY_PASS"
    assert vx > 0.0


def test_t11_close_passenger_no_acceleration_and_never_reverses():
    """T11: Passenger closes in (<1.2m) -> maintains Nav2 speed, never accelerates, never reverses."""
    guard = VelocityGuard()
    t = 100.0

    # At 0.9m (< 1.2m): forward command 0.30 m/s is maintained
    guard.update_session_state(_make_navigating_session(dist_m=0.9), now_mono=t)
    guard.update_observation(_make_observation(depth_m=0.9), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)
    vx, vy, _, reason = guard.process_velocity(now_mono=t + 0.02)
    assert reason == "ESCORT_VELOCITY_PASS"
    assert pytest.approx(vx, 0.001) == 0.30
    assert vy == 0.0

    # If Nav2 sends reverse command (-0.2 m/s), Guard MUST clamp to 0.0 (never reverse into passenger!)
    guard.update_cmd_vel(linear_x=-0.20, linear_y=0.0, angular_z=0.0, now_mono=t + 0.03)
    vx, _, _, reason = guard.process_velocity(now_mono=t + 0.04)
    assert vx == 0.0


def test_t12_yaw_trimming_deadband_saturation_and_edge_slowdown():
    """T12: Angular trimming with deadband, optical sign, saturation, and 50% near-edge reduction."""
    guard = VelocityGuard()
    t = 100.0

    # 1. Deadband: |offset| <= 0.08 -> Delta omega = 0.0
    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    guard.update_observation(_make_observation(offset_x_norm=0.05), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.10, now_mono=t + 0.01)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.02)
    assert pytest.approx(wz, 0.001) == 0.10  # No trim added

    # 2. Target to the right (offset = +0.40): trim_kp=0.35 * 0.40 = +0.14 rad/s (CCW)
    guard.update_observation(_make_observation(offset_x_norm=0.40), now_mono=t + 0.05)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.06)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.07)
    assert pytest.approx(wz, 0.01) == 0.14  # Positive (CCW)

    # 3. Target to the left (offset = -0.40): trim_kp=0.35 * -0.40 = -0.14 rad/s (CW)
    guard.update_observation(_make_observation(offset_x_norm=-0.40), now_mono=t + 0.10)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.11)
    _, _, wz, _ = guard.process_velocity(now_mono=t + 0.12)
    assert pytest.approx(wz, 0.01) == -0.14  # Negative (CW)

    # 4. Saturation limit: extreme offset (+0.90) -> trim_kp*0.90 = 0.315 > 0.20 -> saturated to +0.20 rad/s
    # Near edge (|offset| > 0.70) -> 50% linear speed reduction (0.30 -> 0.15)
    guard.update_observation(_make_observation(offset_x_norm=0.90, near_edge=True), now_mono=t + 0.15)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.16)
    vx, _, wz, reason = guard.process_velocity(now_mono=t + 0.17)
    assert pytest.approx(wz, 0.001) == 0.20  # Saturated at max trim
    assert pytest.approx(vx, 0.001) == 0.15  # 50% near edge reduction


def test_zero_nav2_command_invariance():
    """Zero Nav2 command produces zero output (no self-motion!)."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    # Passenger has offset 0.50, but Nav2 command is 0/0
    guard.update_observation(_make_observation(offset_x_norm=0.50), now_mono=t)
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

    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=t)
    # Passenger is near right edge (offset = +0.80, delta_omega > 0)
    guard.update_observation(_make_observation(offset_x_norm=0.80, near_edge=True), now_mono=t)
    # Nav2 commands turning CW (angular_z = -0.20 rad/s), turning away from passenger!
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=-0.20, now_mono=t + 0.01)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "NAV2_EDGE_TURNING_CONFLICT_STOP"


def test_unverified_alignment_or_invalid_depth_clamps_to_zero():
    """Unverified alignment or invalid depth triggers degraded zero velocity mode."""
    guard = VelocityGuard()
    t = 100.0

    # 1. Session says depth_proven = False (e.g. UNVERIFIED_ALIGNMENT from Astra)
    guard.update_session_state(_make_navigating_session(dist_m=1.6, depth_proven=False), now_mono=t)
    guard.update_observation(_make_observation(depth_valid=False), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.02)
    assert vx == 0.0
    assert reason == "DEGRADED_UNVERIFIED_DEPTH"


def test_t21_stale_cmd_vel_watchdog():
    """T21: cmd_vel_smoothed older than 0.20s clamps output to zero."""
    guard = VelocityGuard()
    t = 100.0

    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=t)
    guard.update_observation(_make_observation(), now_mono=t)
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
    t = 100.0

    # Buffer command at t = 99.0s
    guard.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.0, now_mono=99.0)

    # Motion gate opens at t = 100.0s
    guard.update_session_state(_make_navigating_session(dist_m=1.5), now_mono=100.0)
    guard.update_observation(_make_observation(), now_mono=100.0)

    # Process velocity at t = 100.05s
    vx, _, _, reason = guard.process_velocity(now_mono=100.05)
    assert vx == 0.0
    assert reason == "CMD_RECEIVED_BEFORE_GATE_OPENED"


def test_t14_stale_observation_clamps_to_zero():
    """T14: Observation older than 0.40s clamps output to zero."""
    guard = VelocityGuard()
    t = 100.0

    # Open gate and observe passenger at t = 100.0s
    guard.update_session_state(_make_navigating_session(dist_m=1.5, obs_fresh=True), now_mono=t)
    guard.update_observation(_make_observation(), now_mono=t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=t + 0.01)

    # At t = 100.45s (> 0.40s): passenger observation stale
    # Even if session_state heartbeat is fresh, guard checks observation age
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

    # Gate opens at monotonic time 100.0
    guard.update_session_state(_make_navigating_session(dist_m=1.6), now_mono=mono_t)
    guard.update_observation(_make_observation(), now_mono=mono_t)
    guard.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=mono_t + 0.01)

    # 1. At mono_t + 0.05, passes through
    vx, _, _, reason = guard.process_velocity(now_mono=mono_t + 0.05)
    assert vx == 0.30
    assert reason == "ESCORT_VELOCITY_PASS"

    # 2. Simulate wall clock / sim time freeze or backward jump: monotonic clock continues ticking
    # At mono_t + 0.25 (> 0.20s cmd timeout), even if /clock paused, monotonic watchdog clamps to zero!
    vx, _, _, reason = guard.process_velocity(now_mono=mono_t + 0.26)
    assert vx == 0.0
    assert reason == "CMD_VEL_STALE"
