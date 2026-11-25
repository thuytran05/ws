#!/usr/bin/env python3
"""Generate Phase 11 webcam/debug visualization evidence images and run before/after overlay benchmark."""

import os
import sys
import time
import resource

import cv2
import numpy as np

_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

import psutil
import torch

from person_tracking.debug_visualization import DebugOverlayContext, DebugVisualizer
from person_tracking.multi_person_tracker import (
    MultiPersonObservation,
    MultiPersonTracker,
    PersonDetection,
)
from person_tracking.source_adapters import DataQuality, ObservationFrame, SourceMode


EVIDENCE_DIR = "/home/ws/src/.agent/person_tracking/evidence/phase_11_webcam_viz"


def _make_keypoints(cx: float, cy: float, hand_raised: bool = False):
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
    if hand_raised:
        offsets[8] = (48, -85)
        offsets[10] = (42, -130)  # Right wrist above nose
    return [[cx + dx, cy + dy, 0.94] for dx, dy in offsets]


def _make_background(w: int = 640, h: int = 480) -> np.ndarray:
    img = np.full((h, w, 3), 42, dtype=np.uint8)
    # Subtle grid for clear visual framing
    for x in range(0, w, 40):
        cv2.line(img, (x, 0), (x, h), (52, 52, 52), 1)
    for y in range(0, h, 40):
        cv2.line(img, (0, y), (w, y), (52, 52, 52), 1)
    return img


def generate_evidence_images():
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    viz = DebugVisualizer()
    bg = _make_background(640, 480)

    def make_obs(candidates, source_mode="live_webcam", quality="RGB_ONLY", proc_ms=18.5):
        return MultiPersonObservation(
            stamp_sec=1732540000,
            stamp_nanosec=100000000,
            receive_monotonic_time=100.0,
            frame_id="camera_color_optical_frame",
            source_mode=source_mode,
            source_session_id="demo-session-01",
            data_quality=quality,
            candidates=candidates,
            active_track_ids=[c.track_id for c in candidates if c.track_id is not None],
            lost_track_ids=[],
            processing_time_ms=proc_ms,
            device="cuda:0",
        )

    # 1. IDLE
    p1 = PersonDetection([245, 130, 395, 390], 0.93, _make_keypoints(320, 245), track_id=3, is_confirmed=True, center_offset_x_norm=0.0)
    p2 = PersonDetection([460, 145, 585, 395], 0.88, _make_keypoints(522, 255), track_id=5, is_confirmed=True, center_offset_x_norm=0.63)
    img_idle = viz.render(bg, make_obs([p1, p2]), DebugOverlayContext(fsm_available=True, fsm_state="IDLE", obs_fresh=True, fps_override=29.2))
    cv2.imwrite(os.path.join(EVIDENCE_DIR, "debug_idle.png"), img_idle)

    # 2. HAND RAISING
    p_hand = PersonDetection([245, 100, 395, 390], 0.95, _make_keypoints(320, 245, hand_raised=True), track_id=3, is_confirmed=True, center_offset_x_norm=0.0)
    ctx_hand = DebugOverlayContext(
        fsm_available=True, fsm_state="IDLE", obs_fresh=True,
        gesture_hold_time_sec=1.5, gesture_progress={("demo-session-01", 3): 0.9}, fps_override=28.8,
    )
    img_hand = viz.render(bg, make_obs([p_hand]), ctx_hand)
    cv2.imwrite(os.path.join(EVIDENCE_DIR, "debug_hand_raising.png"), img_hand)

    # 3. LOCKED
    p_locked = PersonDetection([245, 130, 395, 390], 0.94, _make_keypoints(320, 245), track_id=3, is_confirmed=True, center_offset_x_norm=-0.04)
    ctx_locked = DebugOverlayContext(
        fsm_available=True, fsm_state="LOCKED_WAIT_DESTINATION", locked=True,
        locked_track_id=3, locked_source_session_id="demo-session-01", obs_fresh=True, fps_override=29.0,
    )
    img_locked = viz.render(bg, make_obs([p_locked]), ctx_locked)
    cv2.imwrite(os.path.join(EVIDENCE_DIR, "debug_locked.png"), img_locked)

    # 4. NEAR EDGE
    p_edge = PersonDetection([18, 130, 155, 390], 0.91, _make_keypoints(86, 245), track_id=3, is_confirmed=True, near_edge=True, center_offset_x_norm=-0.73)
    ctx_edge = DebugOverlayContext(
        fsm_available=True, fsm_state="NAVIGATING", locked=True,
        locked_track_id=3, locked_source_session_id="demo-session-01", obs_fresh=True, fps_override=28.4,
    )
    img_edge = viz.render(bg, make_obs([p_edge]), ctx_edge)
    cv2.imwrite(os.path.join(EVIDENCE_DIR, "debug_near_edge.png"), img_edge)

    # 5. RGB ONLY (Two persons, unconfirmed + confirmed, strict N/A (RGB ONLY))
    p_rgb = PersonDetection([220, 130, 370, 390], 0.94, _make_keypoints(295, 245), track_id=1, is_confirmed=True, depth_valid=False, center_offset_x_norm=-0.08)
    img_rgb = viz.render(bg, make_obs([p_rgb]), DebugOverlayContext(fsm_available=True, fsm_state="IDLE", obs_fresh=True, fps_override=29.5))
    cv2.imwrite(os.path.join(EVIDENCE_DIR, "debug_rgb_only.png"), img_rgb)
    print(f"Generated 5 evidence PNGs in {EVIDENCE_DIR}")


def run_benchmark():
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    log_path = os.path.join(EVIDENCE_DIR, "benchmark_before_after_overlay.log")

    # Prepare synthetic 640x480 frame with a realistic-ish figure or fixture
    rgb = _make_background(640, 480)
    obs_frame = ObservationFrame(
        rgb=rgb,
        depth=None,
        depth_raw=None,
        camera_info=None,
        stamp_sec=1732540000,
        stamp_nanosec=0,
        receive_monotonic=time.monotonic(),
        frame_id="camera_color_optical_frame",
        source_mode=SourceMode.LIVE_WEBCAM.value,
        data_quality=DataQuality.RGB_ONLY.value,
    )

    proc = psutil.Process(os.getpid())
    devices = ["cuda:0"] if torch.cuda.is_available() else []
    devices.append("cpu")

    lines = [
        "PHASE 11 WEBCAM & DEBUG VISUALIZATION BENCHMARK (BEFORE vs AFTER OVERLAY)",
        f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "Scope: Synthetic 640x480 frames; separates YOLO+ByteTrack inference from DebugVisualizer overlay.",
        "Note: Offline benchmark does not represent real physical webcam USB/driver FPS.",
        "=" * 80,
    ]

    sample_cand = PersonDetection([245, 130, 395, 390], 0.94, _make_keypoints(320, 245, hand_raised=True), track_id=3, is_confirmed=True)
    viz = DebugVisualizer()
    ctx = DebugOverlayContext(
        fsm_available=True, fsm_state="IDLE", obs_fresh=True,
        gesture_hold_time_sec=1.5, gesture_progress={("bench", 3): 0.8},
    )

    for dev in devices:
        tracker = MultiPersonTracker(model_path="/home/ws/yolo26n-pose.pt", device=dev)
        # Warmup
        for _ in range(5):
            tracker.process_frame(obs_frame)

        n_iters = 50
        inf_times_ms = []
        ovl_times_ms = []
        tot_times_ms = []

        proc.cpu_percent(interval=None)
        for idx in range(n_iters):
            t0 = time.perf_counter()
            obs = tracker.process_frame(obs_frame)
            t1 = time.perf_counter()
            # Ensure realistic overlay work by rendering at least 1 person if scene is empty
            if not obs.candidates:
                obs.candidates = [sample_cand]
                obs.source_session_id = "bench"
            _ = viz.render(rgb, obs, ctx)
            t2 = time.perf_counter()

            inf_ms = (t1 - t0) * 1000.0
            ovl_ms = (t2 - t1) * 1000.0
            tot_ms = (t2 - t0) * 1000.0
            inf_times_ms.append(inf_ms)
            ovl_times_ms.append(ovl_ms)
            tot_times_ms.append(tot_ms)

        cpu_pct = proc.cpu_percent(interval=None)
        ram_mb = proc.memory_info().rss / (1024.0 * 1024.0)
        vram_mb = (torch.cuda.memory_allocated() / (1024.0 * 1024.0)) if dev.startswith("cuda") else 0.0

        inf_p50, inf_p95 = np.percentile(inf_times_ms, [50, 95])
        ovl_p50, ovl_p95 = np.percentile(ovl_times_ms, [50, 95])
        tot_p50, tot_p95 = np.percentile(tot_times_ms, [50, 95])

        fps_before = 1000.0 / float(np.mean(inf_times_ms))
        fps_after = 1000.0 / float(np.mean(tot_times_ms))

        block = [
            f"Device: {dev.upper()} ({n_iters} frames, 640x480)",
            f"  [BEFORE Overlay - Inference Only]  FPS: {fps_before:.1f} | P50: {inf_p50:.2f} ms | P95: {inf_p95:.2f} ms",
            f"  [Overlay Render Overhead Only]     P50: {ovl_p50:.2f} ms | P95: {ovl_p95:.2f} ms",
            f"  [AFTER Overlay - End-to-End]       FPS: {fps_after:.1f} | P50: {tot_p50:.2f} ms | P95: {tot_p95:.2f} ms",
            f"  Process CPU: {cpu_pct:.1f}% | RAM RSS: {ram_mb:.1f} MB | CUDA Allocated VRAM: {vram_mb:.1f} MB",
            "-" * 80,
        ]
        lines.extend(block)

    report = "\n".join(lines) + "\n"
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(report)
    print(report)


if __name__ == "__main__":
    generate_evidence_images()
    run_benchmark()
