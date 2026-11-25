"""Unit tests for COCO 17 skeleton rendering and FSM/session-aware debug visualization."""

import numpy as np
import pytest

from person_tracking.debug_visualization import (
    COCO_SKELETON_EDGES,
    COLOR_ALERT,
    COLOR_HAND_RAISE,
    COLOR_LOCKED,
    COLOR_NEAR_EDGE,
    COLOR_NORMAL,
    DebugOverlayContext,
    DebugVisualizer,
    is_valid_keypoint,
)
from person_tracking.multi_person_tracker import MultiPersonObservation, PersonDetection


def _make_full_17_keypoints(cx: float = 320.0, cy: float = 240.0, scale: float = 1.0):
    """Build 17 valid COCO keypoints centered around (cx, cy)."""
    offsets = [
        (0, -90),    # 0 nose
        (-8, -96),   # 1 left_eye
        (8, -96),    # 2 right_eye
        (-16, -92),  # 3 left_ear
        (16, -92),   # 4 right_ear
        (-30, -55),  # 5 left_shoulder
        (30, -55),   # 6 right_shoulder
        (-45, -15),  # 7 left_elbow
        (45, -15),   # 8 right_elbow
        (-50, 20),   # 9 left_wrist
        (50, 20),    # 10 right_wrist
        (-22, 15),   # 11 left_hip
        (22, 15),    # 12 right_hip
        (-24, 70),   # 13 left_knee
        (24, 70),    # 14 right_knee
        (-26, 125),  # 15 left_ankle
        (26, 125),   # 16 right_ankle
    ]
    return [[cx + dx * scale, cy + dy * scale, 0.92] for dx, dy in offsets]


def _make_obs(candidates, source_mode="live_webcam", data_quality="RGB_ONLY", source_session_id="sess-1"):
    return MultiPersonObservation(
        stamp_sec=1700000000,
        stamp_nanosec=500000000,
        receive_monotonic_time=100.0,
        frame_id="camera_color_optical_frame",
        source_mode=source_mode,
        source_session_id=source_session_id,
        data_quality=data_quality,
        candidates=candidates,
        active_track_ids=[c.track_id for c in candidates if c.track_id is not None],
        lost_track_ids=[],
        processing_time_ms=20.0,
        device="cuda:0",
    )


def _has_color_in_region(img: np.ndarray, box, target_bgr, tol: int = 25) -> bool:
    x1, y1, x2, y2 = [max(0, int(v)) for v in box]
    patch = img[y1:y2, x1:x2]
    if patch.size == 0:
        return False
    diff = np.abs(patch.astype(np.int16) - np.array(target_bgr, dtype=np.int16))
    return bool(np.any(np.all(diff <= tol, axis=-1)))


def test_skeleton_full_17_keypoints_draws_edges_and_does_not_mutate_inputs():
    viz = DebugVisualizer(display_kp_conf=0.40)
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    orig_copy = blank.copy()

    kps = _make_full_17_keypoints(320, 240)
    cand = PersonDetection(
        bbox_xyxy=[250, 120, 390, 380],
        confidence=0.95,
        keypoints=kps,
        track_id=3,
        is_confirmed=True,
        depth_valid=False,
        center_offset_x_norm=0.0,
    )
    obs = _make_obs([cand])
    out = viz.render(blank, obs, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))

    # Input image & observation must not be mutated
    assert np.array_equal(blank, orig_copy)
    assert cand.keypoints[0][0] == 320.0

    # Check midpoints of skeleton edges (e.g., shoulder-to-elbow and hip-to-knee) were drawn
    p5 = np.array(kps[5][:2])
    p7 = np.array(kps[7][:2])
    mid_arm = ((p5 + p7) / 2.0).astype(int)
    assert np.any(out[mid_arm[1] - 2:mid_arm[1] + 3, mid_arm[0] - 2:mid_arm[0] + 3] > 0)

    p11 = np.array(kps[11][:2])
    p13 = np.array(kps[13][:2])
    mid_leg = ((p11 + p13) / 2.0).astype(int)
    assert np.any(out[mid_leg[1] - 2:mid_leg[1] + 3, mid_leg[0] - 2:mid_leg[0] + 3] > 0)


def test_skeleton_low_confidence_and_zero_origin_not_drawn():
    viz = DebugVisualizer(display_kp_conf=0.40)
    assert not is_valid_keypoint([0.0, 0.0, 0.99], 640, 480, min_conf=0.40)
    assert not is_valid_keypoint([300.0, 200.0, 0.20], 640, 480, min_conf=0.40)

    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    kps = _make_full_17_keypoints(320, 240)
    # Set left wrist (9) to (0, 0) with high conf, and right wrist (10) to low conf
    kps[9] = [0.0, 0.0, 0.99]
    kps[10] = [370.0, 260.0, 0.15]

    cand = PersonDetection(
        bbox_xyxy=[250, 120, 390, 380],
        confidence=0.90,
        keypoints=kps,
        track_id=1,
        is_confirmed=True,
    )
    obs = _make_obs([cand])
    out = viz.render(blank, obs, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))

    # Top-left corner (around 100, 90) along line from (0,0) to left_elbow (275, 225) must NOT have skeleton line
    # Check point (137, 112) which is midpoint between (0,0) and (275,225) and outside bbox/banner
    assert np.all(out[112, 137] < 40)


def test_skeleton_upper_body_only_works_without_legs():
    viz = DebugVisualizer(display_kp_conf=0.40)
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    kps = _make_full_17_keypoints(320, 220)
    # Remove legs (11..16)
    for idx in range(11, 17):
        kps[idx] = [0.0, 0.0, 0.0]

    cand = PersonDetection(
        bbox_xyxy=[250, 100, 390, 280],
        confidence=0.91,
        keypoints=kps,
        track_id=2,
        is_confirmed=True,
    )
    obs = _make_obs([cand])
    out = viz.render(blank, obs, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))

    # Shoulder edge (5-6) should still be drawn
    mid_sh = ((np.array(kps[5][:2]) + np.array(kps[6][:2])) / 2.0).astype(int)
    assert np.any(out[mid_sh[1] - 2:mid_sh[1] + 3, mid_sh[0] - 2:mid_sh[0] + 3] > 0)


def test_two_persons_no_cross_skeleton_lines():
    viz = DebugVisualizer(display_kp_conf=0.40)
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    kps_left = _make_full_17_keypoints(140, 240)
    kps_right = _make_full_17_keypoints(500, 240)

    c1 = PersonDetection(bbox_xyxy=[80, 120, 200, 380], confidence=0.9, keypoints=kps_left, track_id=1, is_confirmed=True)
    c2 = PersonDetection(bbox_xyxy=[440, 120, 560, 380], confidence=0.9, keypoints=kps_right, track_id=2, is_confirmed=True)
    obs = _make_obs([c1, c2])
    out = viz.render(blank, obs, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))

    # Center strip between x=240..400, y=150..350 (away from center line x=320) must remain empty
    strip = out[150:350, 250:300]
    assert np.all(strip == 0)


def test_fsm_color_states_hand_raise_lock_reset_stale_and_session_isolation():
    viz = DebugVisualizer()
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    kps = _make_full_17_keypoints(320, 240)
    cand = PersonDetection(
        bbox_xyxy=[250, 120, 390, 380],
        confidence=0.94,
        keypoints=kps,
        track_id=3,
        is_confirmed=True,
        center_offset_x_norm=0.0,
    )
    obs = _make_obs([cand], source_session_id="sess-A")

    # 1. IDLE -> Normal steel blue, not green or yellow
    img_idle = viz.render(blank, obs, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))
    assert _has_color_in_region(img_idle, (248, 118, 255, 200), COLOR_NORMAL)
    assert not _has_color_in_region(img_idle, (248, 118, 255, 200), COLOR_LOCKED)

    # 2. HAND RAISING -> Yellow with progress
    ctx_hand = DebugOverlayContext(
        fsm_available=True,
        fsm_state="IDLE",
        gesture_hold_time_sec=1.5,
        gesture_progress={("sess-A", 3): 0.8},
    )
    img_hand = viz.render(blank, obs, ctx_hand)
    assert _has_color_in_region(img_hand, (248, 118, 255, 200), COLOR_HAND_RAISE)

    # 3. LOCKED -> Green (even after hand lowered / gesture_progress empty)
    ctx_locked = DebugOverlayContext(
        fsm_available=True,
        fsm_state="LOCKED_WAIT_DESTINATION",
        locked=True,
        locked_track_id=3,
        locked_source_session_id="sess-A",
        obs_fresh=True,
    )
    img_locked = viz.render(blank, obs, ctx_locked)
    assert _has_color_in_region(img_locked, (248, 118, 255, 200), COLOR_LOCKED)

    # 4. Same track_id=3 but DIFFERENT source_session_id -> MUST NOT be green!
    obs_other_sess = _make_obs([cand], source_session_id="sess-B")
    img_other = viz.render(blank, obs_other_sess, ctx_locked)
    assert not _has_color_in_region(img_other, (248, 118, 255, 200), COLOR_LOCKED)
    assert _has_color_in_region(img_other, (248, 118, 255, 200), COLOR_NORMAL)

    # 5. Stale heartbeat -> MUST NOT be green!
    ctx_stale = DebugOverlayContext(
        fsm_available=True,
        fsm_stale=True,
        fsm_state="LOCKED_WAIT_DESTINATION",
        locked=True,
        locked_track_id=3,
        locked_source_session_id="sess-A",
    )
    img_stale = viz.render(blank, obs, ctx_stale)
    assert not _has_color_in_region(img_stale, (248, 118, 255, 200), COLOR_LOCKED)

    # 6. Near edge -> Orange
    cand_edge = PersonDetection(
        bbox_xyxy=[10, 120, 140, 380],
        confidence=0.90,
        keypoints=_make_full_17_keypoints(75, 240),
        track_id=3,
        is_confirmed=True,
        near_edge=True,
        center_offset_x_norm=-0.76,
    )
    obs_edge = _make_obs([cand_edge], source_session_id="sess-A")
    img_edge = viz.render(blank, obs_edge, ctx_locked)
    assert _has_color_in_region(img_edge, (8, 118, 16, 200), COLOR_NEAR_EDGE)

    # 7. Ambiguous -> Red
    cand_amb = PersonDetection(
        bbox_xyxy=[250, 120, 390, 380],
        confidence=0.90,
        keypoints=kps,
        track_id=4,
        is_confirmed=True,
        is_ambiguous=True,
    )
    obs_amb = _make_obs([cand_amb], source_session_id="sess-A")
    img_amb = viz.render(blank, obs_amb, DebugOverlayContext(fsm_available=True, fsm_state="IDLE"))
    assert _has_color_in_region(img_amb, (248, 118, 255, 200), COLOR_ALERT)
