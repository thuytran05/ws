"""Passenger lock and gesture management module for person_tracking Phase 4 & Phase 6.

Implements ADR-01 (Single Lock Authority):
- Ingests /tracking/observation frames.
- Analyzes 17 body keypoints for one-hand raise gesture.
- Requires gesture hold duration >= gesture_hold_time_sec (default 1.5s).
- Rejects missing, zero (0/0), duplicate, backwards, or gapped timestamps (>0.25s).
- Disambiguates multiple candidates using valid torso depth (or proxy area if unambiguous).
- Transitions FSM through states:
    IDLE -> LOCKED_WAIT_DESTINATION -> NAVIGATING -> WAITING_FOR_PASSENGER -> ABORTING / ARRIVED / FAULT.
- Decouples raw camera frame arrival time from locked passenger observation time.
- Enforces safety thresholds: grace 0.25s, WAIT/stop 0.40s, ABORT 1.20s.
- Latching stop in ABORTING: cannot be resumed by new detections or matching IDs.
- Terminal result confirmation from Voice required before resetting to IDLE.
"""

from dataclasses import dataclass
from enum import Enum
import time
from typing import Dict, List, Optional, Tuple
import uuid


class TrackingState(str, Enum):
    IDLE = "IDLE"
    LOCKED_WAIT_DESTINATION = "LOCKED_WAIT_DESTINATION"
    NAVIGATING = "NAVIGATING"
    WAITING_FOR_PASSENGER = "WAITING_FOR_PASSENGER"
    ABORTING = "ABORTING"
    ARRIVED = "ARRIVED"
    FAULT = "FAULT"


@dataclass
class GestureTrackState:
    last_stamp_total: float  # stamp_sec + stamp_nanosec * 1e-9
    accumulated_hold_sec: float = 0.0


@dataclass
class SessionStateSnapshot:
    schema_version: str
    stamp_sec: int
    stamp_nanosec: int
    last_observation_stamp: Dict[str, int]
    obs_fresh: bool
    sequence_num: int
    session_id: str
    state: str
    locked: bool
    person_track_id: Optional[int]
    person_distance_m: Optional[float]
    depth_proven: bool
    confidence: float
    reason_code: str


class PassengerLockManager:
    """Core logic for hand raise detection, passenger selection, and session FSM lifecycle."""

    def __init__(
        self,
        gesture_hold_time_sec: float = 1.5,
        min_keypoint_conf: float = 0.35,
        max_ambiguity_ratio: float = 0.20,
        grace_period_sec: float = 0.25,
        visibility_stop_sec: float = 0.40,
        visibility_abort_sec: float = 1.20,
    ):
        self.gesture_hold_time_sec = gesture_hold_time_sec
        self.min_keypoint_conf = min_keypoint_conf
        self.max_ambiguity_ratio = max_ambiguity_ratio
        self.grace_period_sec = grace_period_sec
        self.visibility_stop_sec = visibility_stop_sec
        self.visibility_abort_sec = visibility_abort_sec

        # FSM State
        self.state = TrackingState.IDLE
        self.locked = False
        self.session_id: str = ""
        self.locked_track_id: Optional[int] = None
        self.locked_source_session_id: str = ""
        self.locked_distance_m: Optional[float] = None
        self.locked_depth_proven: bool = False
        self.locked_confidence: float = 0.0
        self.reason_code: str = "NO_LOCK"
        self.sequence_num: int = 0
        self.unconfirmed_terminal: bool = False

        # Gesture tracking: (source_session_id, track_id) -> GestureTrackState
        self.gesture_state: Dict[Tuple[str, int], GestureTrackState] = {}
        self.gesture_raise_start: Dict[int, float] = {}

        # Timing decoupled:
        # 1) last_observation_mono_time: when ANY valid observation stream frame arrived
        self.last_observation_stamp: Dict[str, int] = {"sec": 0, "nanosec": 0}
        self.last_observation_mono_time: float = 0.0

        # 2) last_locked_passenger_seen_mono_time: when the LOCKED passenger was specifically seen
        self.last_locked_passenger_stamp: Dict[str, int] = {"sec": 0, "nanosec": 0}
        self.last_locked_passenger_seen_mono_time: float = 0.0

    def reset_session(self, reason: str = "SESSION_RESET"):
        """Reset lock and return to IDLE state."""
        self.state = TrackingState.IDLE
        self.locked = False
        self.session_id = ""
        self.locked_track_id = None
        self.locked_source_session_id = ""
        self.locked_distance_m = None
        self.locked_depth_proven = False
        self.locked_confidence = 0.0
        self.reason_code = reason
        self.last_locked_passenger_seen_mono_time = 0.0
        self.last_locked_passenger_stamp = {"sec": 0, "nanosec": 0}
        self.gesture_state.clear()
        self.gesture_raise_start.clear()

    def is_hand_raised(self, keypoints: List[List[float]]) -> bool:
        """Check if left or right wrist is vertically higher than corresponding shoulder."""
        if not keypoints or len(keypoints) < 11:
            return False

        # Left side: shoulder=5, wrist=9
        ls = keypoints[5]
        lw = keypoints[9]
        left_raised = (
            len(ls) >= 3 and len(lw) >= 3 and
            ls[2] >= self.min_keypoint_conf and lw[2] >= self.min_keypoint_conf and
            lw[1] < ls[1]  # In pixel coordinates, lower y is vertically higher
        )

        # Right side: shoulder=6, wrist=10
        rs = keypoints[6]
        rw = keypoints[10]
        right_raised = (
            len(rs) >= 3 and len(rw) >= 3 and
            rs[2] >= self.min_keypoint_conf and rw[2] >= self.min_keypoint_conf and
            rw[1] < rs[1]
        )

        return left_raised or right_raised

    def is_observation_fresh(self, current_time_sec: float) -> bool:
        """Check if observation is fresh (< 0.40s).
        
        If locked, both observation stream and locked passenger must be fresh!
        """
        if self.last_observation_mono_time <= 0.0:
            return False
        stream_age = current_time_sec - self.last_observation_mono_time
        if stream_age > self.visibility_stop_sec:
            return False
        if self.locked:
            if self.last_locked_passenger_seen_mono_time <= 0.0:
                return False
            passenger_age = current_time_sec - self.last_locked_passenger_seen_mono_time
            if passenger_age > self.visibility_stop_sec:
                return False
        return True

    def _update_watchdog(self, current_time_sec: float):
        """Evaluate timeouts and trigger FSM transitions according to project thresholds."""
        if not self.locked:
            return

        # Check raw stream staleness (camera freeze or publisher killed)
        if self.last_observation_mono_time > 0.0:
            stream_age = current_time_sec - self.last_observation_mono_time
            if stream_age >= self.visibility_abort_sec and self.state in (TrackingState.NAVIGATING, TrackingState.WAITING_FOR_PASSENGER):
                self.state = TrackingState.ABORTING
                self.reason_code = "OBSERVATION_STALE_ABORT"
                return
            elif stream_age >= self.visibility_stop_sec and self.state == TrackingState.NAVIGATING:
                self.state = TrackingState.WAITING_FOR_PASSENGER
                self.reason_code = "OBSERVATION_STALE_WAIT"
                return

        # Check locked passenger unseen duration
        if self.last_locked_passenger_seen_mono_time > 0.0:
            unseen_sec = current_time_sec - self.last_locked_passenger_seen_mono_time
        else:
            unseen_sec = 999.0

        if self.state == TrackingState.NAVIGATING:
            if unseen_sec >= self.visibility_abort_sec:
                self.state = TrackingState.ABORTING
                self.reason_code = "VISIBILITY_TIMEOUT_ABORT"
            elif unseen_sec >= self.visibility_stop_sec:
                self.state = TrackingState.WAITING_FOR_PASSENGER
                self.reason_code = "VISIBILITY_TIMEOUT_STOP"
            elif unseen_sec >= self.grace_period_sec:
                self.reason_code = "LOCKED_TARGET_TEMPORARILY_UNSEEN"
            elif unseen_sec > 0.05:
                self.reason_code = "LOCKED_TARGET_GRACE_PERIOD"

        elif self.state == TrackingState.WAITING_FOR_PASSENGER:
            if unseen_sec >= self.visibility_abort_sec:
                self.state = TrackingState.ABORTING
                self.reason_code = "VISIBILITY_TIMEOUT_ABORT"
            else:
                self.reason_code = "VISIBILITY_TIMEOUT_STOP"

        elif self.state == TrackingState.LOCKED_WAIT_DESTINATION:
            if unseen_sec >= self.visibility_stop_sec:
                self.reason_code = "PASSENGER_TEMPORARILY_UNSEEN"

    def process_observation(self, obs_payload: dict, current_time_sec: Optional[float] = None) -> SessionStateSnapshot:
        """Process an incoming /tracking/observation payload and update session state."""
        if current_time_sec is None:
            current_time_sec = time.monotonic()

        self.sequence_num += 1
        stamp_sec = obs_payload.get("stamp", {}).get("sec", 0)
        stamp_nanosec = obs_payload.get("stamp", {}).get("nanosec", 0)
        source_session_id = obs_payload.get("source_session_id", "")
        candidates = obs_payload.get("candidates", [])

        stamp_dict = obs_payload.get("stamp")
        has_valid_stamp = (
            isinstance(stamp_dict, dict) and
            isinstance(stamp_dict.get("sec"), int) and
            isinstance(stamp_dict.get("nanosec"), int) and
            stamp_dict["sec"] >= 0 and
            stamp_dict["nanosec"] >= 0
        )
        if has_valid_stamp:
            stamp_sec = stamp_dict["sec"]
            stamp_nanosec = stamp_dict["nanosec"]
            t_obs = stamp_sec + stamp_nanosec * 1e-9
        else:
            stamp_sec = 0
            stamp_nanosec = 0
            t_obs = -1.0  # Invalid stamp marker

        # Check old or duplicate observation
        last_t = self.last_observation_stamp["sec"] + self.last_observation_stamp["nanosec"] * 1e-9
        is_old_frame = (has_valid_stamp and last_t > 0.0 and t_obs <= last_t)

        if has_valid_stamp and not is_old_frame:
            self.last_observation_stamp = {"sec": stamp_sec, "nanosec": stamp_nanosec}
            self.last_observation_mono_time = current_time_sec

        # -------------------------------------------------------------
        # 1. ALREADY LOCKED
        # -------------------------------------------------------------
        if self.locked:
            # Source session switch or reset during active lock
            if self.locked_source_session_id and source_session_id != self.locked_source_session_id:
                if self.state in (TrackingState.NAVIGATING, TrackingState.WAITING_FOR_PASSENGER):
                    self.state = TrackingState.ABORTING
                    self.reason_code = "SOURCE_RESET_DISCONNECT"
                    return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)
                else:
                    self.reset_session(reason="SOURCE_RESET_DISCONNECT")
                    return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

            # If already in ABORTING: latched! Reappearance of passenger cannot unlatch
            if self.state == TrackingState.ABORTING:
                return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

            # Search for locked candidate in fresh frame
            matched_cand = None
            if not is_old_frame:
                for cand in candidates:
                    if cand.get("track_id") == self.locked_track_id:
                        if not cand.get("is_ambiguous", False):
                            matched_cand = cand
                        break

            if matched_cand is not None:
                # Update observation of locked passenger
                self.last_locked_passenger_seen_mono_time = current_time_sec
                self.last_locked_passenger_stamp = {"sec": stamp_sec, "nanosec": stamp_nanosec}
                self.locked_confidence = matched_cand.get("confidence", 0.0)
                if matched_cand.get("depth_valid", False):
                    self.locked_distance_m = matched_cand.get("torso_depth_m")
                    self.locked_depth_proven = True
                else:
                    self.locked_distance_m = None
                    self.locked_depth_proven = False

                if self.state == TrackingState.WAITING_FOR_PASSENGER:
                    # Resumed: matching passenger returned before visibility_abort_sec
                    self.state = TrackingState.NAVIGATING
                    self.reason_code = "PASSENGER_RETURNED_RESUMED"
                elif self.state == TrackingState.NAVIGATING:
                    self.reason_code = "LOCKED_TRACKING_ACTIVE"
                elif self.state == TrackingState.LOCKED_WAIT_DESTINATION:
                    self.reason_code = "LOCKED_TRACKING_ACTIVE"
            else:
                # Candidate not seen, ambiguous, or old frame:
                # self.last_locked_passenger_seen_mono_time is NOT updated!
                pass

            self._update_watchdog(current_time_sec)
            return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

        # -------------------------------------------------------------
        # 2. NOT LOCKED (Evaluating Gesture for Lock)
        # -------------------------------------------------------------
        # If unconfirmed terminal from previous mission: block new locks!
        if self.unconfirmed_terminal:
            self.reason_code = "UNCONFIRMED_TERMINAL_BLOCK"
            return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

        # Invariant: Observation stamp MUST be valid (has_valid_stamp == True).
        # Multiple frames repeating stamp 0/0 or missing stamps CANNOT accumulate time!
        if not has_valid_stamp:
            self.reason_code = "INVALID_STAMP_NO_ACCUMULATION"
            return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

        active_keys = set()
        eligible_candidates = []

        for cand in candidates:
            track_id = cand.get("track_id")
            if track_id is None or not cand.get("is_confirmed", False):
                continue

            cand_key = (source_session_id, track_id)
            active_keys.add(cand_key)
            kps = cand.get("keypoints", [])
            hand_up = self.is_hand_raised(kps)

            if hand_up:
                if cand_key not in self.gesture_state:
                    self.gesture_state[cand_key] = GestureTrackState(
                        last_stamp_total=t_obs,
                        accumulated_hold_sec=0.0,
                    )
                    self.gesture_raise_start[track_id] = t_obs
                else:
                    g_state = self.gesture_state[cand_key]
                    delta_t = t_obs - g_state.last_stamp_total

                    if delta_t <= 0.0:
                        # Duplicate or backwards stamp: do NOT accumulate hold time
                        pass
                    elif delta_t > 0.25:
                        # Gap exceeded max interval (0.25s): reset gesture timer for this track!
                        self.gesture_state[cand_key] = GestureTrackState(
                            last_stamp_total=t_obs,
                            accumulated_hold_sec=0.0,
                        )
                        self.gesture_raise_start[track_id] = t_obs
                    else:
                        g_state.accumulated_hold_sec += delta_t
                        g_state.last_stamp_total = t_obs

                g_state = self.gesture_state[cand_key]
                if g_state.accumulated_hold_sec >= self.gesture_hold_time_sec:
                    if not cand.get("is_ambiguous", False):
                        eligible_candidates.append((cand, g_state.accumulated_hold_sec))
            else:
                # Lowered hand resets the hold timer
                self.gesture_state.pop(cand_key, None)
                self.gesture_raise_start.pop(track_id, None)

        # Clean up gesture timers for track IDs / sessions no longer active
        for k in list(self.gesture_state.keys()):
            if k not in active_keys:
                self.gesture_state.pop(k, None)
                self.gesture_raise_start.pop(k[1], None)

        # Disambiguate and select passenger
        if len(eligible_candidates) == 1:
            best_cand, _ = eligible_candidates[0]
            self._lock_candidate(
                best_cand,
                source_session_id,
                current_time_sec,
                stamp_sec,
                stamp_nanosec,
                reason="HAND_RAISE_HELD_SINGLE",
            )
        elif len(eligible_candidates) > 1:
            best_cand = self._select_nearest_candidate(eligible_candidates)
            if best_cand is not None:
                self._lock_candidate(
                    best_cand,
                    source_session_id,
                    current_time_sec,
                    stamp_sec,
                    stamp_nanosec,
                    reason="HAND_RAISE_HELD_NEAREST",
                )
            else:
                self.reason_code = "AMBIGUOUS_GESTURE"

        return self._make_snapshot(stamp_sec, stamp_nanosec, current_time_sec)

    def _select_nearest_candidate(self, eligible_candidates: List[Tuple[dict, float]]) -> Optional[dict]:
        """Disambiguate multiple eligible candidates based on depth or bbox proxy."""
        all_have_valid_depth = all(
            c[0].get("depth_valid", False) and c[0].get("torso_depth_m") is not None
            for c in eligible_candidates
        )
        if all_have_valid_depth:
            sorted_by_depth = sorted(eligible_candidates, key=lambda c: c[0]["torso_depth_m"])
            return sorted_by_depth[0][0]

        areas = []
        for cand, _ in eligible_candidates:
            bbox = cand.get("bbox_xyxy", [0, 0, 0, 0])
            area = max(0.0, bbox[2] - bbox[0]) * max(0.0, bbox[3] - bbox[1])
            areas.append((cand, area))

        areas.sort(key=lambda x: x[1], reverse=True)
        largest_cand, largest_area = areas[0]
        second_cand, second_area = areas[1]

        if largest_area > 0 and (largest_area - second_area) / largest_area > 0.30:
            return largest_cand

        return None

    def _lock_candidate(
        self,
        cand: dict,
        source_session_id: str,
        current_time_sec: float,
        stamp_sec: int,
        stamp_nanosec: int,
        reason: str,
    ):
        """Lock on the qualified candidate and transition FSM to LOCKED_WAIT_DESTINATION."""
        self.state = TrackingState.LOCKED_WAIT_DESTINATION
        self.locked = True
        self.session_id = str(uuid.uuid4())
        self.locked_track_id = cand.get("track_id")
        self.locked_source_session_id = source_session_id
        self.locked_confidence = cand.get("confidence", 0.0)
        self.last_locked_passenger_seen_mono_time = current_time_sec
        self.last_locked_passenger_stamp = {"sec": stamp_sec, "nanosec": stamp_nanosec}

        if cand.get("depth_valid", False):
            self.locked_distance_m = cand.get("torso_depth_m")
            self.locked_depth_proven = True
        else:
            self.locked_distance_m = None
            self.locked_depth_proven = False

        self.reason_code = reason

    def on_destination_accepted(self, session_id: str, current_time_sec: Optional[float] = None) -> Tuple[bool, str]:
        """Handle event: Voice accepts destination for the current session."""
        if current_time_sec is None:
            current_time_sec = time.monotonic()

        if self.state != TrackingState.LOCKED_WAIT_DESTINATION:
            return False, f"INVALID_STATE_{self.state.value}"
        if not self.session_id or self.session_id != session_id:
            return False, "SESSION_ID_MISMATCH"
        if self.unconfirmed_terminal:
            return False, "UNCONFIRMED_TERMINAL_BLOCK"
        if not self.is_observation_fresh(current_time_sec):
            return False, "OBSERVATION_NOT_FRESH"

        self.state = TrackingState.NAVIGATING
        self.reason_code = "DESTINATION_ACCEPTED_START"
        return True, "OK"

    def on_voice_nav_status(self, status_payload: dict, current_time_sec: Optional[float] = None) -> Tuple[bool, str]:
        """Handle status updates from /voice/nav_status."""
        if current_time_sec is None:
            current_time_sec = time.monotonic()

        session_id = status_payload.get("session_id", "")
        if not session_id or session_id != self.session_id:
            # Stale or mismatched session status: completely ignore!
            return False, "IGNORED_STALE_SESSION"

        is_terminal = bool(status_payload.get("is_terminal", False))
        status = str(status_payload.get("status", "")).upper()
        nav_event = str(status_payload.get("nav_event", "")).upper()

        if is_terminal:
            if status == "SUCCEEDED":
                self.state = TrackingState.ARRIVED
                self.reason_code = "GOAL_SUCCEEDED"
                self.unconfirmed_terminal = False
                self.reset_session(reason="GOAL_SUCCEEDED_CLEANUP")
                return True, "TERMINAL_SUCCEEDED"
            elif status in ("CANCELED", "ABORTED"):
                self.unconfirmed_terminal = False
                self.reset_session(reason=f"GOAL_{status}_CLEANUP")
                return True, f"TERMINAL_{status}"
            else:
                self.unconfirmed_terminal = False
                self.reset_session(reason="TERMINAL_UNKNOWN_CLEANUP")
                return True, "TERMINAL_OTHER"

        return True, f"STATUS_UPDATE_{nav_event or status}"

    def _make_snapshot(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        current_time_sec: Optional[float] = None,
    ) -> SessionStateSnapshot:
        if current_time_sec is None:
            current_time_sec = time.monotonic()

        fresh = self.is_observation_fresh(current_time_sec)
        reason = self.reason_code

        if not fresh and self.locked:
            if reason in ("LOCKED_TRACKING_ACTIVE", "HAND_RAISE_HELD_SINGLE", "HAND_RAISE_HELD_NEAREST"):
                reason = "PASSENGER_TEMPORARILY_UNSEEN"

        return SessionStateSnapshot(
            schema_version="1.0",
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            last_observation_stamp=dict(self.last_observation_stamp),
            obs_fresh=fresh,
            sequence_num=self.sequence_num,
            session_id=self.session_id,
            state=self.state.value,
            locked=self.locked,
            person_track_id=self.locked_track_id,
            person_distance_m=self.locked_distance_m,
            depth_proven=self.locked_depth_proven,
            confidence=self.locked_confidence,
            reason_code=reason,
        )

    def to_json_dict(self, snapshot: SessionStateSnapshot) -> dict:
        """Serialize snapshot to dictionary matching INTERFACES.md Section 2.1 schema."""
        return {
            "schema_version": snapshot.schema_version,
            "stamp": {"sec": snapshot.stamp_sec, "nanosec": snapshot.stamp_nanosec},
            "last_observation_stamp": snapshot.last_observation_stamp,
            "obs_fresh": snapshot.obs_fresh,
            "sequence_num": snapshot.sequence_num,
            "session_id": snapshot.session_id,
            "state": snapshot.state,
            "locked": snapshot.locked,
            "person_track_id": snapshot.person_track_id,
            "person_distance_m": snapshot.person_distance_m,
            "depth_proven": snapshot.depth_proven,
            "confidence": snapshot.confidence,
            "reason_code": snapshot.reason_code,
        }
