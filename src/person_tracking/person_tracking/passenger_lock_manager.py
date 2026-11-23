"""Passenger lock and gesture management module for person_tracking Phase 4.

Implements ADR-01 (Single Lock Authority):
- Ingests /tracking/observation frames.
- Analyzes 17 body keypoints for one-hand raise gesture.
- Requires gesture hold duration >= gesture_hold_time_sec (default 1.5s).
- Disambiguates multiple candidates using valid torso depth (or proxy area if unambiguous).
- Transitions FSM from IDLE to LOCKED_WAIT_DESTINATION.
- Ensures dropping hand does NOT break session.
- Isolates track IDs by source_session_id to prevent lock hopping across loops.
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
    """Core logic for hand raise detection, passenger selection, and session locking."""

    def __init__(
        self,
        gesture_hold_time_sec: float = 1.5,
        min_keypoint_conf: float = 0.35,
        max_ambiguity_ratio: float = 0.20,
    ):
        self.gesture_hold_time_sec = gesture_hold_time_sec
        self.min_keypoint_conf = min_keypoint_conf
        self.max_ambiguity_ratio = max_ambiguity_ratio

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

        # Gesture tracking: (source_session_id, track_id) -> GestureTrackState
        self.gesture_state: Dict[Tuple[str, int], GestureTrackState] = {}
        self.gesture_raise_start: Dict[int, float] = {}
        self.last_observation_stamp: Dict[str, int] = {"sec": 0, "nanosec": 0}
        self.last_observation_mono_time: float = 0.0

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

    def process_observation(self, obs_payload: dict, current_time_sec: Optional[float] = None) -> SessionStateSnapshot:
        """Process an incoming /tracking/observation payload and update session state."""
        if current_time_sec is None:
            current_time_sec = time.monotonic()

        self.sequence_num += 1
        stamp_sec = obs_payload.get("stamp", {}).get("sec", 0)
        stamp_nanosec = obs_payload.get("stamp", {}).get("nanosec", 0)
        source_session_id = obs_payload.get("source_session_id", "")
        candidates = obs_payload.get("candidates", [])

        # Invariant 1: Source session switch or reset invalidates existing session lock
        if self.locked:
            if self.locked_source_session_id and source_session_id != self.locked_source_session_id:
                # Source session changed (e.g. video loop or source reset)
                self.reset_session(reason="SOURCE_RESET_DISCONNECT")
                return self._make_snapshot(stamp_sec, stamp_nanosec)

            # Invariant 2: Once locked, dropping hand DOES NOT lose lock
            # Find the locked person candidate
            matched_cand = None
            for cand in candidates:
                if cand.get("track_id") == self.locked_track_id:
                    matched_cand = cand
                    break

            if matched_cand is not None:
                # Update distance and confidence from observation
                self.locked_confidence = matched_cand.get("confidence", 0.0)
                if matched_cand.get("depth_valid", False):
                    self.locked_distance_m = matched_cand.get("torso_depth_m")
                    self.locked_depth_proven = True
                else:
                    self.locked_depth_proven = False
                self.reason_code = "LOCKED_TRACKING_ACTIVE"
            else:
                self.reason_code = "LOCKED_TARGET_TEMPORARILY_UNSEEN"

            return self._make_snapshot(stamp_sec, stamp_nanosec)

        # Record observation arrival
        self.last_observation_stamp = {"sec": stamp_sec, "nanosec": stamp_nanosec}
        self.last_observation_mono_time = current_time_sec

        t_obs = stamp_sec + stamp_nanosec * 1e-9
        if t_obs == 0.0 and current_time_sec is not None:
            t_obs = current_time_sec

        # If not locked: evaluate candidates for hand raise gesture
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
                        # Duplicate or out-of-order stamp: do NOT accumulate time
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
            self._lock_candidate(best_cand, source_session_id, reason="HAND_RAISE_HELD_SINGLE")
        elif len(eligible_candidates) > 1:
            # Multiple candidates raised hand: select nearest
            best_cand = self._select_nearest_candidate(eligible_candidates)
            if best_cand is not None:
                self._lock_candidate(best_cand, source_session_id, reason="HAND_RAISE_HELD_NEAREST")
            else:
                self.reason_code = "AMBIGUOUS_GESTURE"

        return self._make_snapshot(stamp_sec, stamp_nanosec)

    def _select_nearest_candidate(self, eligible_candidates: List[Tuple[dict, float]]) -> Optional[dict]:
        """Disambiguate multiple eligible candidates based on depth or bbox proxy."""
        # 1. Depth check: if candidates have valid depth, pick minimum distance
        all_have_valid_depth = all(
            c[0].get("depth_valid", False) and c[0].get("torso_depth_m") is not None
            for c in eligible_candidates
        )
        if all_have_valid_depth:
            # Sort by torso_depth_m ascending
            sorted_by_depth = sorted(eligible_candidates, key=lambda c: c[0]["torso_depth_m"])
            return sorted_by_depth[0][0]

        # 2. If depth is invalid/unavailable: compare bbox area proxy
        areas = []
        for cand, _ in eligible_candidates:
            bbox = cand.get("bbox_xyxy", [0, 0, 0, 0])
            area = max(0.0, bbox[2] - bbox[0]) * max(0.0, bbox[3] - bbox[1])
            areas.append((cand, area))

        areas.sort(key=lambda x: x[1], reverse=True)
        largest_cand, largest_area = areas[0]
        second_cand, second_area = areas[1]

        # If largest is significantly bigger (> 30% larger than second), consider it clearly nearer
        if largest_area > 0 and (largest_area - second_area) / largest_area > 0.30:
            return largest_cand

        # Otherwise ambiguous: refuse to lock tentatively!
        return None

    def _lock_candidate(self, cand: dict, source_session_id: str, reason: str):
        """Lock on the qualified candidate and transition FSM to LOCKED_WAIT_DESTINATION."""
        self.state = TrackingState.LOCKED_WAIT_DESTINATION
        self.locked = True
        self.session_id = str(uuid.uuid4())
        self.locked_track_id = cand.get("track_id")
        self.locked_source_session_id = source_session_id
        self.locked_confidence = cand.get("confidence", 0.0)

        if cand.get("depth_valid", False):
            self.locked_distance_m = cand.get("torso_depth_m")
            self.locked_depth_proven = True
        else:
            self.locked_distance_m = None
            self.locked_depth_proven = False

        self.reason_code = reason

    def _make_snapshot(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        obs_fresh: bool = True,
    ) -> SessionStateSnapshot:
        return SessionStateSnapshot(
            schema_version="1.0",
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            last_observation_stamp=dict(self.last_observation_stamp),
            obs_fresh=obs_fresh,
            sequence_num=self.sequence_num,
            session_id=self.session_id,
            state=self.state.value,
            locked=self.locked,
            person_track_id=self.locked_track_id,
            person_distance_m=self.locked_distance_m,
            depth_proven=self.locked_depth_proven,
            confidence=self.locked_confidence,
            reason_code=self.reason_code,
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
