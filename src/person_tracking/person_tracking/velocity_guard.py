"""Velocity Guard module for escort robot in person_tracking Phase 7.

Safety Principles & Specifications:
1. Topic Chain:
   Nav2 cmd_vel_smoothed -> escort_velocity_guard -> cmd_vel_guarded -> Collision Monitor -> cmd_vel -> base.
   Strictly ZERO /cmd_vel publishers in person_tracking.
   No twist_mux in the chain.

2. Active Continuous Zeroing:
   Default output is strictly all-zero Twist at 20 Hz.
   Zero output triggered immediately on ANY violation or non-NAVIGATING state
   (IDLE, LOCKED_WAIT_DESTINATION, WAITING_FOR_PASSENGER, ABORTING, ARRIVED, FAULT).

3. Motion Gating Conditions (ALL must pass for non-zero output):
   - FSM state is NAVIGATING and locked == True.
   - Heartbeat from /tracking/session_state is fresh (<= 1.0s).
   - Passenger observation is fresh (<= 0.40s) and obs_fresh == True.
   - Verified depth only: depth_proven == True and depth_valid == True.
     If UNVERIFIED_ALIGNMENT, missing, or invalid depth -> degraded zero velocity.
   - Fresh cmd_vel_smoothed received (<= 0.20s).
   - cmd_vel_smoothed was received strictly AFTER the motion gate opened (no stale buffered commands).

4. Distance-Based Speed Limiting:
   - Safe zone [1.2m, 2.0m]: pass-through Nav2 speed (v_x = v_x_in).
   - Range (2.0m, 2.5m]: linear ceiling reduction: scale = (1.0 - (d - 2.0) / 0.5).
     v_x = min(v_x_in, v_x_in * scale).
   - Beyond 2.5m: stop completely (v_x = 0), latch distance stop.
   - Hysteresis: while latched, remain stopped until passenger returns to <= 2.2m.
   - Below 1.2m: safe close range, maintain Nav2 speed, never accelerate, never reverse (v_x >= 0).
   - Keep linear.y = 0.0 strictly.

5. Angular Trimming (Delta omega):
   - Optical convention (ADR-10):
     * center_offset_x_norm > 0 (target on right in rear image) -> CCW rotation (omega > 0).
     * center_offset_x_norm < 0 (target on left in rear image) -> CW rotation (omega < 0).
   - Deadband: |offset| <= 0.08 -> Delta omega = 0.0.
   - Saturated trimming: |Delta omega| <= 0.20 rad/s.
   - Near-edge speed reduction: |offset| > 0.70 -> reduce linear speed by 50%.
   - Zero Nav2 Command Invariance: if Nav2 cmd is 0 (v_x == 0 and omega == 0) -> output 0.
   - Conflict Resolution: if Nav2 turn opposes trimming at the edge, prioritize stop/wait.
"""

from dataclasses import dataclass
import time
from typing import Optional, Tuple


@dataclass
class GuardLimits:
    safe_dist_min: float = 1.2
    safe_dist_max: float = 2.0
    stop_dist: float = 2.5
    resume_dist: float = 2.2
    max_linear_x: float = 0.50
    max_angular_z: float = 0.40
    trim_angular_max: float = 0.20
    deadband_offset: float = 0.08
    trim_kp: float = 0.35
    near_edge_threshold: float = 0.70
    near_edge_linear_scale: float = 0.50
    cmd_stale_timeout_sec: float = 0.20
    passenger_timeout_sec: float = 0.40
    heartbeat_timeout_sec: float = 1.0


class VelocityGuard:
    """Core mathematical engine and safety policy for velocity modulation and gating."""

    def __init__(self, limits: Optional[GuardLimits] = None):
        self.limits = limits or GuardLimits()

        # Motion gate timing
        self.motion_gate_opened_mono_time: float = 0.0
        self.is_gate_open: bool = False

        # Freshness timestamps (monotonic)
        self.last_cmd_vel_mono_time: float = 0.0
        self.last_session_state_mono_time: float = 0.0
        self.last_observation_mono_time: float = 0.0

        # Latest inputs
        self.latest_session_state: Optional[dict] = None
        self.latest_observation: Optional[dict] = None
        self.latest_cmd_vel: Optional[dict] = None  # {linear_x, linear_y, angular_z}

        # Distance stop hysteresis latch
        self.distance_stop_latched: bool = False

        # Diagnostics
        self.last_reason_code: str = "INITIAL_ZERO"

    def update_session_state(self, session_payload: dict, now_mono: Optional[float] = None):
        if now_mono is None:
            now_mono = time.monotonic()
        prev_state = self.latest_session_state.get("state") if self.latest_session_state else None
        curr_state = session_payload.get("state")

        self.latest_session_state = session_payload
        self.last_session_state_mono_time = now_mono

        # Track motion gate opening event
        if curr_state == "NAVIGATING" and session_payload.get("locked", False):
            if prev_state != "NAVIGATING":
                self.motion_gate_opened_mono_time = now_mono
                self.is_gate_open = True
        else:
            self.is_gate_open = False
            self.distance_stop_latched = False

    def update_observation(self, obs_payload: dict, now_mono: Optional[float] = None):
        if now_mono is None:
            now_mono = time.monotonic()
        self.latest_observation = obs_payload
        self.last_observation_mono_time = now_mono

    def update_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float, now_mono: Optional[float] = None):
        if now_mono is None:
            now_mono = time.monotonic()
        self.latest_cmd_vel = {
            "linear_x": float(linear_x),
            "linear_y": float(linear_y),
            "angular_z": float(angular_z),
        }
        self.last_cmd_vel_mono_time = now_mono

    def evaluate_motion_gate(self, now_mono: Optional[float] = None) -> Tuple[bool, str]:
        if now_mono is None:
            now_mono = time.monotonic()

        if self.latest_session_state is None:
            return False, "NO_SESSION_STATE"

        # 1. Heartbeat check (<= 1.0s)
        heartbeat_age = now_mono - self.last_session_state_mono_time
        if heartbeat_age > self.limits.heartbeat_timeout_sec:
            return False, "HEARTBEAT_STALE"

        # 2. State & Lock check (must be NAVIGATING and locked == True)
        state = self.latest_session_state.get("state")
        locked = self.latest_session_state.get("locked", False)
        if not locked or state != "NAVIGATING":
            return False, f"GATE_CLOSED_{state}"

        # 3. Observation freshness (obs_fresh == True and age <= 0.40s)
        obs_fresh = self.latest_session_state.get("obs_fresh", False)
        if not obs_fresh:
            return False, "OBSERVATION_STALE"

        if self.last_observation_mono_time > 0.0:
            obs_age = now_mono - self.last_observation_mono_time
            if obs_age > self.limits.passenger_timeout_sec:
                return False, "OBSERVATION_AGE_EXCEEDED"

        # 4. Depth validity check (verified depth required!)
        depth_proven = self.latest_session_state.get("depth_proven", False)
        if not depth_proven:
            return False, "DEGRADED_UNVERIFIED_DEPTH"

        # If observation is available, also ensure locked candidate has depth_valid == True
        if self.latest_observation is not None:
            locked_id = self.latest_session_state.get("person_track_id")
            cands = self.latest_observation.get("candidates", [])
            for c in cands:
                if c.get("track_id") == locked_id:
                    if not c.get("depth_valid", False):
                        return False, "DEGRADED_UNVERIFIED_DEPTH"
                    break

        # 5. Nav2 cmd_vel existence and arrival ordering
        if self.latest_cmd_vel is None:
            return False, "NO_CMD_VEL"

        if self.last_cmd_vel_mono_time < self.motion_gate_opened_mono_time:
            return False, "CMD_RECEIVED_BEFORE_GATE_OPENED"

        # 6. Nav2 cmd_vel freshness (<= 0.20s)
        cmd_age = now_mono - self.last_cmd_vel_mono_time
        if cmd_age > self.limits.cmd_stale_timeout_sec:
            return False, "CMD_VEL_STALE"

        return True, "MOTION_GATE_OPEN"

    def process_velocity(self, now_mono: Optional[float] = None) -> Tuple[float, float, float, str]:
        if now_mono is None:
            now_mono = time.monotonic()

        gate_open, reason = self.evaluate_motion_gate(now_mono)
        if not gate_open:
            self.last_reason_code = reason
            return 0.0, 0.0, 0.0, reason

        linear_x_in = self.latest_cmd_vel["linear_x"]
        linear_y_in = self.latest_cmd_vel["linear_y"]
        angular_z_in = self.latest_cmd_vel["angular_z"]

        # Invariant: Zero Nav2 command -> Zero output (no self-motion!)
        if abs(linear_x_in) < 1e-4 and abs(angular_z_in) < 1e-4:
            self.last_reason_code = "ZERO_NAV2_CMD"
            return 0.0, 0.0, 0.0, "ZERO_NAV2_CMD"

        # Distance calculation
        dist = self.latest_session_state.get("person_distance_m")
        if dist is None or dist <= 0.0:
            self.last_reason_code = "INVALID_DISTANCE"
            return 0.0, 0.0, 0.0, "INVALID_DISTANCE"

        # Distance Stop Hysteresis
        if self.distance_stop_latched:
            if dist <= self.limits.resume_dist:
                self.distance_stop_latched = False
            else:
                self.last_reason_code = "DISTANCE_STOP_LATCHED"
                return 0.0, 0.0, 0.0, "DISTANCE_STOP_LATCHED"

        if dist > self.limits.stop_dist:
            self.distance_stop_latched = True
            self.last_reason_code = "DISTANCE_EXCEEDED_STOP"
            return 0.0, 0.0, 0.0, "DISTANCE_EXCEEDED_STOP"

        # Linear speed modulation based on distance
        if dist > self.limits.safe_dist_max:
            # Linear scaling from 1.0 down to 0.0 between 2.0m and 2.5m
            scale = max(0.0, 1.0 - (dist - self.limits.safe_dist_max) / (self.limits.stop_dist - self.limits.safe_dist_max))
            v_x_allowed = min(linear_x_in, linear_x_in * scale)
        elif dist < self.limits.safe_dist_min:
            # Below 1.2m: safe close range, maintain Nav2 speed, do not accelerate, never reverse
            v_x_allowed = max(0.0, linear_x_in)
        else:
            # Safe zone [1.2m, 2.0m]: pass-through Nav2 speed
            v_x_allowed = max(0.0, linear_x_in)

        # Angular trimming calculation
        offset = 0.0
        near_edge = False
        if self.latest_observation is not None:
            locked_id = self.latest_session_state.get("person_track_id")
            for c in self.latest_observation.get("candidates", []):
                if c.get("track_id") == locked_id:
                    offset = float(c.get("center_offset_x_norm", 0.0))
                    near_edge = bool(c.get("near_edge", False)) or abs(offset) > self.limits.near_edge_threshold
                    break

        if abs(offset) <= self.limits.deadband_offset:
            delta_omega = 0.0
        else:
            # offset > 0 (right in rear image) -> CCW (omega > 0)
            # offset < 0 (left in rear image) -> CW (omega < 0)
            delta_omega = self.limits.trim_kp * offset
            delta_omega = max(-self.limits.trim_angular_max, min(self.limits.trim_angular_max, delta_omega))

        # Near edge handling
        if near_edge:
            # 50% speed reduction when near edge
            v_x_allowed *= self.limits.near_edge_linear_scale

            # Conflict resolution: If Nav2 turn opposes keeping passenger in FOV at edge, prioritize stop/wait
            if (delta_omega > 0.05 and angular_z_in < -0.05) or (delta_omega < -0.05 and angular_z_in > 0.05):
                self.last_reason_code = "NAV2_EDGE_TURNING_CONFLICT_STOP"
                return 0.0, 0.0, 0.0, "NAV2_EDGE_TURNING_CONFLICT_STOP"

        # Combine angular velocity
        angular_z_out = angular_z_in + delta_omega
        angular_z_out = max(-self.limits.max_angular_z, min(self.limits.max_angular_z, angular_z_out))

        # Clamp linear velocity
        linear_x_out = max(0.0, min(self.limits.max_linear_x, v_x_allowed))

        # Invariant: linear.y = 0.0 strictly
        linear_y_out = 0.0

        self.last_reason_code = "ESCORT_VELOCITY_PASS"
        return linear_x_out, linear_y_out, angular_z_out, "ESCORT_VELOCITY_PASS"
