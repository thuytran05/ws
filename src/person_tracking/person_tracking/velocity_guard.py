"""Velocity Guard module for escort robot in person_tracking Phase 7 & 8.

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
   - Valid observation from bound session, source, and track has been received.
   - Passenger observation is fresh (<= 0.40s) and obs_fresh == True.
   - Verified depth only: depth_proven == True and depth_valid == True.
     If UNVERIFIED_ALIGNMENT, missing, or invalid depth -> degraded zero velocity.
   - Fresh cmd_vel_smoothed received (<= 0.20s).
   - cmd_vel_smoothed was received strictly AFTER the motion gate opened (no stale buffered commands).
   - Old cmd_vel revoked on any state transition, stale observation, or session change.

4. Distance-Based Speed Limiting:
   - Safe zone [1.2m, 2.0m]: pass-through Nav2 speed (v_x = v_x_in).
   - Range (2.0m, 2.5m]: linear ceiling reduction: scale = (1.0 - (d - 2.0) / 0.5).
     v_x = min(v_x_in, v_x_in * scale).
   - Boundary at d >= 2.5m: stop completely (v_x = 0, omega = 0), latch distance stop.
     Robot must NOT yaw or turn when distance requires stop.
   - Hysteresis: while latched, remain stopped until passenger returns to <= 2.2m.
   - Below 1.2m: safe close range, maintain Nav2 speed, never accelerate, never reverse (v_x >= 0).
   - Keep linear.y = 0.0 strictly.

5. Angular Trimming (Delta omega) & Conflict Resolution:
   - Rear camera coordinate transform (faces backward):
     * center_offset_x_norm > 0 (target on right in rear image) -> CW rotation (delta_omega < 0).
     * center_offset_x_norm < 0 (target on left in rear image) -> CCW rotation (delta_omega > 0).
   - Deadband: |offset| <= 0.08 -> Delta omega = 0.0.
   - Saturated trimming: |Delta omega| <= 0.20 rad/s.
   - Near-edge speed reduction: |offset| > 0.70 -> reduce linear speed by 50%.
   - Zero Nav2 Command Invariance: if Nav2 cmd is 0 (v_x == 0 and omega == 0) -> output 0.
   - Conflict Resolution: if Nav2 turn opposes trimming at the edge, prioritize stop/wait.
     * offset > 0.70 (trim CW, delta_omega < -0.05) and Nav2 omega_in > 0.05 (CCW) -> stop/wait.
     * offset < -0.70 (trim CCW, delta_omega > 0.05) and Nav2 omega_in < -0.05 (CW) -> stop/wait.
"""

from dataclasses import dataclass
import math
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
        self.last_passenger_seen_mono_time: float = 0.0

        # Session, source and goal binding
        self.current_session_id: str = ""
        self.bound_track_id: Optional[int] = None
        self.bound_source_session_id: str = ""
        self.bound_mission_id: Optional[str] = None
        self.bound_goal_attempt_id: Optional[str] = None
        self.bound_nav2_uuid: Optional[str] = None
        self.last_heartbeat_stamp: Tuple[int, int] = (-1, -1)
        self.last_obs_stamp: Tuple[int, int] = (-1, -1)
        self.last_valid_observation: Optional[dict] = None
        self.valid_obs_count: int = 0

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

        # Reject out-of-order or stale heartbeat after restart / delay
        stamp = session_payload.get("stamp", {})
        hb_sec = stamp.get("sec", -1)
        hb_nanosec = stamp.get("nanosec", -1)
        if isinstance(hb_sec, int) and isinstance(hb_nanosec, int) and hb_sec >= 0:
            if (hb_sec, hb_nanosec) < self.last_heartbeat_stamp:
                # Stale heartbeat from before restart or network delay: reject!
                return
            self.last_heartbeat_stamp = (hb_sec, hb_nanosec)

        new_sess_id = session_payload.get("session_id", "")
        # Session changed: reset session-bound state and revoke old commands
        if new_sess_id != self.current_session_id:
            self.current_session_id = new_sess_id
            self.bound_track_id = session_payload.get("person_track_id")
            self.bound_source_session_id = session_payload.get("source_session_id", "")
            self.bound_mission_id = session_payload.get("mission_id")
            self.bound_goal_attempt_id = session_payload.get("goal_attempt_id")
            self.bound_nav2_uuid = session_payload.get("nav2_uuid")
            self.last_obs_stamp = (-1, -1)
            self.last_valid_observation = None
            self.valid_obs_count = 0
            self.last_observation_mono_time = 0.0
            self.last_passenger_seen_mono_time = 0.0
            self.latest_cmd_vel = None
            self.motion_gate_opened_mono_time = 0.0
            self.is_gate_open = False
            self.distance_stop_latched = False
        else:
            self.bound_track_id = session_payload.get("person_track_id")
            src_sess = session_payload.get("source_session_id", "")
            if src_sess:
                self.bound_source_session_id = src_sess

            new_goal_attempt = session_payload.get("goal_attempt_id")
            new_mission_id = session_payload.get("mission_id")
            new_nav2_uuid = session_payload.get("nav2_uuid")

            # Detect goal change even within the same session
            goal_changed = (
                self.bound_goal_attempt_id is not None
                and new_goal_attempt is not None
                and new_goal_attempt != self.bound_goal_attempt_id
            ) or (
                self.bound_mission_id is not None
                and new_mission_id is not None
                and new_mission_id != self.bound_mission_id
            )

            if goal_changed:
                # Cancel/clear cached cmd_vel when goal changes, even within same session!
                self.latest_cmd_vel = None
                self.motion_gate_opened_mono_time = now_mono

            self.bound_mission_id = new_mission_id
            self.bound_goal_attempt_id = new_goal_attempt
            self.bound_nav2_uuid = new_nav2_uuid

        prev_state = self.latest_session_state.get("state") if self.latest_session_state else None
        curr_state = session_payload.get("state")
        locked = session_payload.get("locked", False)

        self.latest_session_state = session_payload
        self.last_session_state_mono_time = now_mono

        # Track motion gate opening event
        if curr_state == "NAVIGATING" and locked:
            # Gate opens if observation is already valid and fresh
            has_valid_obs = (
                self.valid_obs_count > 0
                and self.last_valid_observation is not None
                and (now_mono - self.last_passenger_seen_mono_time <= self.limits.passenger_timeout_sec)
            )
            if has_valid_obs:
                if not self.is_gate_open or prev_state != "NAVIGATING":
                    self.motion_gate_opened_mono_time = now_mono
                    self.is_gate_open = True
                    self.latest_cmd_vel = None
            else:
                self.is_gate_open = False
                self.motion_gate_opened_mono_time = 0.0
        else:
            self.is_gate_open = False
            self.latest_cmd_vel = None  # Revoke old cmd_vel on WAIT/ABORT/FAULT/etc.
            self.motion_gate_opened_mono_time = 0.0
            self.distance_stop_latched = False

    def update_observation(self, obs_payload: dict, now_mono: Optional[float] = None):
        """Validate observation stamp/sequence, session/track binding, and observation quality.
        
        Heartbeat, empty frames, repeated frames, wrong source frames, or invalid/unconfirmed
        candidates MUST NOT extend passenger seen time!
        """
        if now_mono is None:
            now_mono = time.monotonic()

        if not isinstance(obs_payload, dict):
            return

        # 1. Validate stamp
        stamp = obs_payload.get("stamp")
        if not isinstance(stamp, dict):
            return
        sec = stamp.get("sec")
        nanosec = stamp.get("nanosec")
        if not isinstance(sec, int) or not isinstance(nanosec, int):
            return
        if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
            return
        if sec == 0 and nanosec == 0:
            return  # Stamp 0/0 is invalid

        # 2. Validate source session and sequence monotonicity
        src_sess = str(obs_payload.get("source_session_id", ""))
        if not src_sess:
            return
        if self.bound_source_session_id and src_sess != self.bound_source_session_id:
            # Different source session: do not extend passenger seen time
            return
        if (sec, nanosec) <= self.last_obs_stamp:
            # Duplicate or backwards frame: do not extend passenger seen time
            return

        # 3. Match candidate for bound locked track ID
        track_id = self.bound_track_id
        if track_id is None and self.latest_session_state:
            track_id = self.latest_session_state.get("person_track_id")
        if track_id is None:
            return

        candidates = obs_payload.get("candidates", [])
        if not isinstance(candidates, list) or len(candidates) == 0:
            return

        matched = None
        for c in candidates:
            if c.get("track_id") == track_id:
                matched = c
                break

        if matched is None:
            return

        # 4. Validate candidate quality & depth
        if not matched.get("is_confirmed", False):
            return
        if matched.get("is_ambiguous", False):
            return
        if not matched.get("depth_valid", False):
            return

        depth = matched.get("torso_depth_m")
        offset = matched.get("center_offset_x_norm")
        if depth is None or not isinstance(depth, (int, float)) or not math.isfinite(depth) or depth <= 0.0:
            return
        if offset is None or not isinstance(offset, (int, float)) or not math.isfinite(offset):
            return

        # All validation passed: update valid observation state
        if not self.bound_source_session_id:
            self.bound_source_session_id = src_sess

        self.last_obs_stamp = (sec, nanosec)
        self.last_passenger_seen_mono_time = now_mono
        self.last_observation_mono_time = now_mono
        self.last_valid_observation = matched
        self.latest_observation = obs_payload
        self.valid_obs_count += 1

        # If session is NAVIGATING and locked, but gate was closed (waiting for obs or recovering):
        # Open gate now!
        if (
            self.latest_session_state
            and self.latest_session_state.get("state") == "NAVIGATING"
            and self.latest_session_state.get("locked", False)
        ):
            if not self.is_gate_open:
                self.is_gate_open = True
                self.motion_gate_opened_mono_time = now_mono
                self.latest_cmd_vel = None  # Clear commands buffered while gate was closed

    def update_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float, now_mono: Optional[float] = None):
        if now_mono is None:
            now_mono = time.monotonic()
        try:
            lx = float(linear_x)
            ly = float(linear_y)
            az = float(angular_z)
        except (TypeError, ValueError):
            lx, ly, az = float("nan"), float("nan"), float("nan")

        self.latest_cmd_vel = {
            "linear_x": lx,
            "linear_y": ly,
            "angular_z": az,
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
            self.is_gate_open = False
            return False, "HEARTBEAT_STALE"

        # 2. State & Lock check (must be NAVIGATING and locked == True)
        state = self.latest_session_state.get("state")
        locked = self.latest_session_state.get("locked", False)
        if not locked or state != "NAVIGATING":
            self.is_gate_open = False
            return False, f"GATE_CLOSED_{state}"

        

        # 3. Observation validation: must have at least one valid observation from start
        if self.valid_obs_count == 0 or self.last_valid_observation is None or self.last_passenger_seen_mono_time <= 0.0:
            self.is_gate_open = False
            return False, "NO_VALID_OBSERVATION"

        # 4. Observation freshness (<= 0.40s)
        obs_fresh = self.latest_session_state.get("obs_fresh", False)
        obs_age = now_mono - self.last_passenger_seen_mono_time
        if obs_age > self.limits.passenger_timeout_sec or not obs_fresh:
            # Stale observation closes the gate!
            self.is_gate_open = False
            return False, "OBSERVATION_AGE_EXCEEDED" if obs_age > self.limits.passenger_timeout_sec else "OBSERVATION_STALE"

        # Stale recovery handling: if gate was closed (e.g. while stale), reopening requires new command
        if not self.is_gate_open:
            self.is_gate_open = True
            self.motion_gate_opened_mono_time = now_mono
            self.latest_cmd_vel = None

        # 5. Depth validity check (verified depth required!)
        depth_proven = self.latest_session_state.get("depth_proven", False)
        if not depth_proven:
            return False, "DEGRADED_UNVERIFIED_DEPTH"

        if not self.last_valid_observation.get("depth_valid", False):
            return False, "DEGRADED_UNVERIFIED_DEPTH"

        # 6. Nav2 cmd_vel existence and arrival ordering
        if self.latest_cmd_vel is None:
            return False, "NO_CMD_VEL"

        if self.last_cmd_vel_mono_time < self.motion_gate_opened_mono_time:
            return False, "CMD_RECEIVED_BEFORE_GATE_OPENED"

        # 7. Nav2 cmd_vel freshness (<= 0.20s)
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

        linear_x_in = self.latest_cmd_vel.get("linear_x")
        linear_y_in = self.latest_cmd_vel.get("linear_y")
        angular_z_in = self.latest_cmd_vel.get("angular_z")

        # Sanity check: NaN/Inf on velocity inputs -> clamp to zero
        for val in (linear_x_in, linear_y_in, angular_z_in):
            if val is None or not isinstance(val, (int, float)) or not math.isfinite(val):
                self.last_reason_code = "INVALID_NUMERIC_VALUE"
                return 0.0, 0.0, 0.0, "INVALID_NUMERIC_VALUE"

        # Invariant: Zero Nav2 command -> Zero output (no self-motion!)
        if abs(linear_x_in) < 1e-4 and abs(angular_z_in) < 1e-4:
            self.last_reason_code = "ZERO_NAV2_CMD"
            return 0.0, 0.0, 0.0, "ZERO_NAV2_CMD"

        # Distance calculation
        dist = self.latest_session_state.get("person_distance_m")
        if dist is None or not isinstance(dist, (int, float)) or not math.isfinite(dist) or dist <= 0.0:
            self.last_reason_code = "INVALID_DISTANCE"
            return 0.0, 0.0, 0.0, "INVALID_DISTANCE"

        # Boundary condition at d >= 2.5m: robot must NOT yaw/turn when distance requires stop
        if self.distance_stop_latched:
            if dist <= self.limits.resume_dist:
                self.distance_stop_latched = False
            else:
                self.last_reason_code = "DISTANCE_STOP_LATCHED"
                return 0.0, 0.0, 0.0, "DISTANCE_STOP_LATCHED"

        if dist >= self.limits.stop_dist:
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

        # Angular trimming calculation using last valid observation (persists across 0.40s grace)
        offset = float(self.last_valid_observation.get("center_offset_x_norm", 0.0))
        if not math.isfinite(offset):
            self.last_reason_code = "INVALID_NUMERIC_VALUE"
            return 0.0, 0.0, 0.0, "INVALID_NUMERIC_VALUE"

        near_edge = bool(self.last_valid_observation.get("near_edge", False)) or abs(offset) > self.limits.near_edge_threshold

        if abs(offset) <= self.limits.deadband_offset:
            delta_omega = 0.0
        else:
            # Rear camera coordinate transform (faces backward):
            # offset > 0 (right in rear image) -> CW rotation (delta_omega < 0)
            # offset < 0 (left in rear image) -> CCW rotation (delta_omega > 0)
            delta_omega = - self.limits.trim_kp * offset
            delta_omega = max(-self.limits.trim_angular_max, min(self.limits.trim_angular_max, delta_omega))

        # Near edge handling & conflict resolution
        if near_edge:
            # 50% speed reduction when near edge
            v_x_allowed *= self.limits.near_edge_linear_scale

            # Conflict resolution:
            # If target is on right (trim CW, delta_omega < -0.05), but Nav2 turns CCW (angular_z_in > 0.05) -> oppose
            # If target is on left (trim CCW, delta_omega > 0.05), but Nav2 turns CW (angular_z_in < -0.05) -> oppose
            if (delta_omega < -0.05 and angular_z_in > 0.05) or (delta_omega > 0.05 and angular_z_in < -0.05):
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
