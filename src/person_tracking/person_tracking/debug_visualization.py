"""Debug visualization module for person_tracking.

Renders:
- COCO 17-keypoint skeleton & valid joint markers (without mutating input image/observation).
- Per-person bounding box, ID, confidence, distance (strictly N/A in RGB_ONLY/UNVERIFIED),
  normalized center offset, and near_edge / ambiguous tags.
- Color coding aligned with Tracking Manager session authority:
  * Steel Blue / Gray: normal detected person (unlocked)
  * Yellow: candidate actively raising hand with real PassengerLockManager hold progress bar
  * Green: passenger confirmed LOCKED by Tracking Manager (matching source_session_id & track_id)
  * Orange: near_edge / warning state
  * Red: ambiguous observation or ABORTING / FAULT state
- Translucent lateral control zones from VelocityGuard & MultiPersonTracker thresholds.
- Compact system status banner from Tracking Manager FSM & source status.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# COCO 17-keypoint names by index
COCO_KEYPOINT_NAMES: Tuple[str, ...] = (
    "nose",            # 0
    "left_eye",        # 1
    "right_eye",       # 2
    "left_ear",        # 3
    "right_ear",       # 4
    "left_shoulder",   # 5
    "right_shoulder",  # 6
    "left_elbow",      # 7
    "right_elbow",     # 8
    "left_wrist",      # 9
    "right_wrist",     # 10
    "left_hip",        # 11
    "right_hip",       # 12
    "left_knee",       # 13
    "right_knee",      # 14
    "left_ankle",      # 15
    "right_ankle",     # 16
)

# Standard COCO 17 skeleton edges grouped by body region
COCO_SKELETON_EDGES: Tuple[Tuple[int, int], ...] = (
    # Head & face
    (0, 1), (0, 2), (1, 3), (2, 4), (1, 2),
    # Shoulders
    (5, 6),
    # Left arm (shoulder -> elbow -> wrist)
    (5, 7), (7, 9),
    # Right arm (shoulder -> elbow -> wrist)
    (6, 8), (8, 10),
    # Torso (shoulders -> hips)
    (5, 11), (6, 12),
    # Hips
    (11, 12),
    # Left leg (hip -> knee -> ankle)
    (11, 13), (13, 15),
    # Right leg (hip -> knee -> ankle)
    (12, 14), (14, 16),
)

# BGR Color palette
COLOR_NORMAL = (210, 150, 50)        # Steel Blue
COLOR_UNCONFIRMED = (160, 160, 160)  # Gray
COLOR_HAND_RAISE = (0, 220, 255)     # Yellow
COLOR_LOCKED = (0, 210, 0)           # Green
COLOR_NEAR_EDGE = (0, 140, 255)      # Orange
COLOR_ALERT = (0, 0, 255)            # Red
COLOR_SKELETON_FACE = (255, 220, 100)
COLOR_SKELETON_TORSO = (255, 180, 60)
COLOR_SKELETON_LIMB = (100, 240, 255)


@dataclass
class DebugOverlayContext:
    """External state passed from TrackingManager / PersonDetectorNode to debug visualizer."""
    fsm_state: str = "UNAVAILABLE"
    fsm_available: bool = False
    fsm_stale: bool = False
    locked: bool = False
    locked_track_id: Optional[int] = None
    locked_source_session_id: str = ""
    reason_code: str = ""
    obs_fresh: Optional[bool] = None
    gesture_hold_time_sec: float = 1.5
    # Maps (source_session_id, track_id) -> accumulated_hold_sec
    gesture_progress: Dict[Tuple[str, int], float] = field(default_factory=dict)
    # Control / perception thresholds for zone overlay
    edge_margin_ratio: float = 0.08
    deadband_offset: float = 0.08
    near_edge_offset_threshold: float = 0.70
    fps_override: Optional[float] = None


def is_valid_keypoint(
    kp: List[float],
    width: int,
    height: int,
    min_conf: float = 0.40,
) -> bool:
    """Return True if keypoint [x, y, conf] is finite, non-(0,0), within frame, and >= min_conf."""
    if not kp or len(kp) < 3:
        return False
    kx, ky, kconf = kp[0], kp[1], kp[2]
    if not (math.isfinite(kx) and math.isfinite(ky) and math.isfinite(kconf)):
        return False
    if kconf < min_conf:
        return False
    # Reject uninitialized / invalid origin keypoints at (0, 0)
    if abs(kx) < 1e-3 and abs(ky) < 1e-3:
        return False
    if kx < -1.0 or ky < -1.0 or kx > width + 1.0 or ky > height + 1.0:
        return False
    return True


def _edge_color(i: int, j: int, person_color: Tuple[int, int, int]) -> Tuple[int, int, int]:
    """Select subtle region-aware color for skeleton limb while preserving readability."""
    if i <= 4 and j <= 4:
        return COLOR_SKELETON_FACE
    if (i, j) in ((5, 6), (5, 11), (6, 12), (11, 12)):
        return person_color
    return COLOR_SKELETON_LIMB


def draw_coco_skeleton(
    canvas: np.ndarray,
    keypoints: List[List[float]],
    person_color: Tuple[int, int, int] = COLOR_NORMAL,
    min_conf: float = 0.40,
) -> None:
    """Draw valid COCO 17 keypoints and skeleton edges on canvas in-place."""
    if not keypoints:
        return
    h, w = canvas.shape[:2]
    num_kps = len(keypoints)

    valid_pts: Dict[int, Tuple[int, int]] = {}
    for idx in range(min(17, num_kps)):
        kp = keypoints[idx]
        if is_valid_keypoint(kp, w, h, min_conf=min_conf):
            px = max(0, min(w - 1, int(round(kp[0]))))
            py = max(0, min(h - 1, int(round(kp[1]))))
            valid_pts[idx] = (px, py)

    # Draw skeleton edges first (under joints)
    for i, j in COCO_SKELETON_EDGES:
        if i in valid_pts and j in valid_pts:
            ecolor = _edge_color(i, j, person_color)
            cv2.line(canvas, valid_pts[i], valid_pts[j], ecolor, 2, cv2.LINE_AA)

    # Draw joints on top
    for idx, pt in valid_pts.items():
        cv2.circle(canvas, pt, 4, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, pt, 3, person_color, -1, cv2.LINE_AA)


def _draw_text_with_bg(
    canvas: np.ndarray,
    text: str,
    org: Tuple[int, int],
    font_scale: float = 0.45,
    text_color: Tuple[int, int, int] = (255, 255, 255),
    bg_color: Tuple[int, int, int] = (20, 20, 20),
    thickness: int = 1,
    pad: int = 3,
) -> Tuple[int, int]:
    """Draw crisp text with a dark contrasting background, clamped inside image bounds."""
    h, w = canvas.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    x, y = org
    # Clamp horizontally and vertically
    x = max(pad, min(max(pad, w - tw - pad - 1), x))
    y = max(th + pad, min(h - baseline - pad - 1, y))

    cv2.rectangle(
        canvas,
        (x - pad, y - th - pad),
        (x + tw + pad, y + baseline + pad),
        bg_color,
        -1,
    )
    cv2.putText(canvas, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)
    return tw + 2 * pad, th + baseline + 2 * pad


def _draw_lateral_zones(canvas: np.ndarray, ctx: DebugOverlayContext) -> None:
    """Draw translucent lateral control zones (deadband, yaw trim, near_edge slowdown, bbox margin)."""
    h, w = canvas.shape[:2]
    overlay_layer = canvas.copy()

    cx = w // 2
    deadband_px = int(ctx.deadband_offset * (w / 2.0))
    near_edge_off_px = int(ctx.near_edge_offset_threshold * (w / 2.0))
    slow_left_x = max(0, cx - near_edge_off_px)
    slow_right_x = min(w - 1, cx + near_edge_off_px)

    # Bottom indicator strip (compact 14px height) so center view stays uncluttered
    strip_y1 = max(0, h - 14)
    strip_y2 = h - 1

    # Yaw trim left & right zones (subtle blue/teal tint)
    cv2.rectangle(overlay_layer, (slow_left_x, strip_y1), (cx - deadband_px, strip_y2), (160, 120, 30), -1)
    cv2.rectangle(overlay_layer, (cx + deadband_px, strip_y1), (slow_right_x, strip_y2), (160, 120, 30), -1)

    # Deadband zone (subtle green/cyan tint)
    cv2.rectangle(overlay_layer, (cx - deadband_px, strip_y1), (cx + deadband_px, strip_y2), (60, 180, 60), -1)

    # VelocityGuard offset > 0.70 slowdown zones (translucent orange tint on left/right strips)
    cv2.rectangle(overlay_layer, (0, 0), (slow_left_x, h - 1), (0, 110, 220), -1)
    cv2.rectangle(overlay_layer, (slow_right_x, 0), (w - 1, h - 1), (0, 110, 220), -1)

    cv2.addWeighted(overlay_layer, 0.14, canvas, 0.86, 0.0, dst=canvas)

    # Center axis & deadband boundary lines
    cv2.line(canvas, (cx, 0), (cx, h - 1), (180, 220, 180), 1, cv2.LINE_AA)
    cv2.line(canvas, (cx - deadband_px, strip_y1 - 6), (cx - deadband_px, strip_y2), (80, 210, 80), 1, cv2.LINE_AA)
    cv2.line(canvas, (cx + deadband_px, strip_y1 - 6), (cx + deadband_px, strip_y2), (80, 210, 80), 1, cv2.LINE_AA)

    # VelocityGuard offset > 0.70 vertical markers
    cv2.line(canvas, (slow_left_x, 0), (slow_left_x, h - 1), (0, 140, 255), 1, cv2.LINE_AA)
    cv2.line(canvas, (slow_right_x, 0), (slow_right_x, h - 1), (0, 140, 255), 1, cv2.LINE_AA)

    # Perception bbox near_edge margin box (8% outer margin)
    mx = int(ctx.edge_margin_ratio * w)
    my = int(ctx.edge_margin_ratio * h)
    cv2.rectangle(canvas, (mx, my), (w - mx, h - my), (110, 110, 110), 1, cv2.LINE_AA)


def _draw_status_banner(
    canvas: np.ndarray,
    observation,
    ctx: DebugOverlayContext,
) -> None:
    """Draw compact translucent system status banner at top-left and legend at bottom-left."""
    h, w = canvas.shape[:2]

    fps_val = ctx.fps_override if ctx.fps_override is not None else (
        1000.0 / max(1.0, float(getattr(observation, "processing_time_ms", 33.3)))
    )
    source_str = str(getattr(observation, "source_mode", "unknown")).upper()
    device_str = str(getattr(observation, "device", "cpu")).upper()
    if ":" in device_str:
        device_str = device_str.split(":")[0]
    persons_count = len(getattr(observation, "candidates", []))
    data_quality = str(getattr(observation, "data_quality", "UNKNOWN"))

    if data_quality == "VALID_RGBD":
        depth_str = "VALID RGB-D"
    elif data_quality == "RGB_ONLY":
        depth_str = "RGB ONLY"
    elif data_quality == "UNVERIFIED_ALIGNMENT":
        depth_str = "UNVERIFIED D2C"
    else:
        depth_str = data_quality

    if not ctx.fsm_available:
        state_str = "UNAVAILABLE"
        state_color = (180, 180, 180)
        locked_id_str = "None"
        obs_fresh_str = "N/A"
    elif ctx.fsm_stale:
        state_str = "STALE"
        state_color = COLOR_NEAR_EDGE
        locked_id_str = "None (stale)"
        obs_fresh_str = "STALE"
    else:
        state_str = ctx.fsm_state
        if state_str in ("ABORTING", "FAULT"):
            state_color = COLOR_ALERT
        elif state_str == "WAITING_FOR_PASSENGER":
            state_color = COLOR_NEAR_EDGE
        elif state_str in ("LOCKED_WAIT_DESTINATION", "NAVIGATING", "ARRIVED"):
            state_color = COLOR_LOCKED
        else:
            state_color = (230, 230, 230)
        locked_id_str = str(ctx.locked_track_id) if (ctx.locked and ctx.locked_track_id is not None) else "None"
        obs_fresh_str = "FRESH" if ctx.obs_fresh else ("STALE" if ctx.obs_fresh is False else "N/A")

    lines = [
        ("PERSON TRACKING", (255, 255, 255), 0.45, 2),
        (f"Source: {source_str}", (210, 210, 210), 0.38, 1),
        (f"Device: {device_str} | FPS: {fps_val:.1f}", (210, 210, 210), 0.38, 1),
        (f"Persons: {persons_count}", (210, 210, 210), 0.38, 1),
        (f"State: {state_str}", state_color, 0.38, 1),
        (f"Locked ID: {locked_id_str}", state_color if (ctx.locked and not ctx.fsm_stale) else (190, 190, 190), 0.38, 1),
        (f"Observation: {obs_fresh_str}", (120, 240, 120) if obs_fresh_str == "FRESH" else (180, 180, 180), 0.38, 1),
        (f"Depth: {depth_str}", (120, 240, 120) if depth_str == "VALID RGB-D" else (160, 210, 255), 0.38, 1),
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    max_tw = 0
    total_th = 6
    line_heights = []
    for txt, _, fs, thk in lines:
        (tw, th), _ = cv2.getTextSize(txt, font, fs, thk)
        max_tw = max(max_tw, tw)
        line_heights.append(th + 5)
        total_th += th + 5

    box_w = min(w - 12, max_tw + 14)
    box_h = min(h - 12, total_th + 6)
    x0, y0 = 6, 6

    panel = canvas.copy()
    cv2.rectangle(panel, (x0, y0), (x0 + box_w, y0 + box_h), (18, 18, 18), -1)
    cv2.addWeighted(panel, 0.65, canvas, 0.35, 0.0, dst=canvas)
    cv2.rectangle(canvas, (x0, y0), (x0 + box_w, y0 + box_h), state_color if state_str in ("ABORTING", "FAULT") else (90, 90, 90), 1)

    cur_y = y0 + 16
    for (txt, col, fs, thk), lh in zip(lines, line_heights):
        if cur_y > y0 + box_h - 4:
            break
        cv2.putText(canvas, txt, (x0 + 7, cur_y), font, fs, col, thk, cv2.LINE_AA)
        cur_y += lh

    # Compact color legend at bottom-left if image height permits
    if h >= 360:
        legend_items = [
            ("Normal", COLOR_NORMAL),
            ("Hand", COLOR_HAND_RAISE),
            ("Locked", COLOR_LOCKED),
            ("Edge", COLOR_NEAR_EDGE),
            ("Alert", COLOR_ALERT),
        ]
        lx = 8
        ly = h - 22
        for lbl, col in legend_items:
            if lx + 65 > w - 8:
                break
            cv2.rectangle(canvas, (lx, ly - 8), (lx + 9, ly + 1), col, -1)
            cv2.putText(canvas, lbl, (lx + 12, ly), font, 0.34, (235, 235, 235), 1, cv2.LINE_AA)
            lx += 58

    return x0 + box_w, y0 + box_h


class DebugVisualizer:
    """Stateless renderer producing annotated debug images without mutating inputs."""

    def __init__(self, display_kp_conf: float = 0.40):
        self.display_kp_conf = display_kp_conf

    def render(
        self,
        image: np.ndarray,
        observation,
        context: Optional[DebugOverlayContext] = None,
    ) -> np.ndarray:
        """Render full debug overlay onto a copy of image."""
        canvas = image.copy()
        h, w = canvas.shape[:2]
        ctx = context or DebugOverlayContext()

        # 1. Draw lateral control & safety zones
        _draw_lateral_zones(canvas, ctx)

        # 2. Draw system status banner at top-left first and record its bounds
        banner_x1, banner_y1 = _draw_status_banner(canvas, observation, ctx)

        obs_source_session = str(getattr(observation, "source_session_id", "") or "")
        data_quality = str(getattr(observation, "data_quality", "UNKNOWN"))
        source_mode = str(getattr(observation, "source_mode", ""))
        is_rgb_only = (
            data_quality == "RGB_ONLY"
            or source_mode in ("live_webcam", "video_replay")
        )

        # 3. Draw each detected person
        candidates = getattr(observation, "candidates", None)
        if candidates is None:
            candidates = getattr(observation, "persons", [])

        for person in candidates:
            bbox = getattr(person, "bbox_xyxy", None) or getattr(person, "bbox", [])
            if len(bbox) < 4:
                continue
            x1 = max(0, min(w - 1, int(round(bbox[0]))))
            y1 = max(0, min(h - 1, int(round(bbox[1]))))
            x2 = max(0, min(w - 1, int(round(bbox[2]))))
            y2 = max(0, min(h - 1, int(round(bbox[3]))))

            track_id = getattr(person, "track_id", None)
            is_confirmed = bool(getattr(person, "is_confirmed", False))
            is_ambiguous = bool(getattr(person, "is_ambiguous", False))
            near_edge = bool(getattr(person, "near_edge", False))
            conf = float(getattr(person, "confidence", 0.0))
            offset_x = float(getattr(person, "center_offset_x_norm", 0.0))

            # Determine if this candidate is the authoritatively locked passenger
            session_matches = (
                not ctx.locked_source_session_id
                or obs_source_session == ctx.locked_source_session_id
            )
            is_locked_person = (
                ctx.fsm_available
                and not ctx.fsm_stale
                and ctx.locked
                and ctx.fsm_state in ("LOCKED_WAIT_DESTINATION", "NAVIGATING", "WAITING_FOR_PASSENGER", "ARRIVED")
                and track_id is not None
                and ctx.locked_track_id is not None
                and track_id == ctx.locked_track_id
                and session_matches
            )

            # Check gesture hold progress from PassengerLockManager
            gesture_hold_sec: Optional[float] = None
            if (
                ctx.fsm_available
                and not ctx.fsm_stale
                and not ctx.locked
                and track_id is not None
                and (obs_source_session, track_id) in ctx.gesture_progress
            ):
                gesture_hold_sec = ctx.gesture_progress[(obs_source_session, track_id)]

            # Color assignment per Section D.2 priority
            tags: List[str] = []
            if is_ambiguous:
                color = COLOR_ALERT
                tags.append("AMBIGUOUS")
            elif near_edge or (is_locked_person and ctx.fsm_state == "WAITING_FOR_PASSENGER"):
                color = COLOR_NEAR_EDGE
                if near_edge:
                    tags.append("NEAR_EDGE")
                if is_locked_person:
                    tags.append("LOCKED")
            elif is_locked_person:
                color = COLOR_LOCKED
                tags.append("LOCKED")
            elif gesture_hold_sec is not None:
                color = COLOR_HAND_RAISE
            elif is_confirmed:
                color = COLOR_NORMAL
            else:
                color = COLOR_UNCONFIRMED

            # Draw bounding box
            thickness = 3 if is_locked_person else 2
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

            # Draw COCO 17 skeleton + valid keypoints
            draw_coco_skeleton(
                canvas,
                getattr(person, "keypoints", []),
                person_color=color,
                min_conf=self.display_kp_conf,
            )

            # Build per-person info lines (Section D.1)
            id_str = str(track_id) if track_id is not None else "?"
            line1 = f"ID: {id_str} | Conf: {conf:.2f}"
            if tags:
                line1 += " [" + ",".join(tags) + "]"

            depth_valid = bool(getattr(person, "depth_valid", False))
            torso_depth = getattr(person, "torso_depth_m", None)
            if depth_valid and torso_depth is not None and math.isfinite(torso_depth):
                line2 = f"Distance: {torso_depth:.2f} m"
            elif is_rgb_only:
                line2 = "Distance: N/A (RGB ONLY)"
            else:
                line2 = "Distance: N/A (UNVERIFIED)"

            line3 = f"Offset: {offset_x:+.2f}"

            # Place info block cleanly above bbox, avoiding top-left banner overlap
            label_y = y1 - 44 if y1 >= 52 else min(h - 52, y1 + 16)
            if x1 < banner_x1 + 4 and label_y < banner_y1 + 12:
                if y2 + 52 < h:
                    label_y = y2 + 14
                else:
                    label_y = min(h - 52, banner_y1 + 14)
            _draw_text_with_bg(canvas, line1, (x1, label_y), font_scale=0.42, text_color=color)
            _draw_text_with_bg(canvas, line2, (x1, label_y + 16), font_scale=0.40, text_color=(240, 240, 240))
            _draw_text_with_bg(canvas, line3, (x1, label_y + 32), font_scale=0.40, text_color=(210, 210, 210))

            # Draw HAND RAISED progress bar if active in PassengerLockManager (Section D.3)
            if gesture_hold_sec is not None:
                target_sec = max(0.1, ctx.gesture_hold_time_sec)
                ratio = max(0.0, min(1.0, gesture_hold_sec / target_sec))
                bar_x1 = x1
                bar_w = max(90, min(160, x2 - x1))
                bar_x1 = max(4, min(w - bar_w - 4, bar_x1))
                bar_y = min(h - 24, y2 + 16 if y2 + 28 < h else y2 - 22)

                _draw_text_with_bg(
                    canvas,
                    f"HAND RAISED {gesture_hold_sec:.1f}/{target_sec:.1f}s",
                    (bar_x1, bar_y),
                    font_scale=0.40,
                    text_color=COLOR_HAND_RAISE,
                )
                pb_y1 = min(h - 10, bar_y + 5)
                pb_y2 = pb_y1 + 7
                cv2.rectangle(canvas, (bar_x1, pb_y1), (bar_x1 + bar_w, pb_y2), (40, 40, 40), -1)
                fill_w = int(bar_w * ratio)
                if fill_w > 0:
                    cv2.rectangle(canvas, (bar_x1, pb_y1), (bar_x1 + fill_w, pb_y2), COLOR_HAND_RAISE, -1)
                cv2.rectangle(canvas, (bar_x1, pb_y1), (bar_x1 + bar_w, pb_y2), (220, 220, 220), 1)

        return canvas
