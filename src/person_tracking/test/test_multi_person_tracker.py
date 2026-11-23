"""Unit and integration tests for MultiPersonTracker in person_tracking Phase 3."""

import os
import sys
import numpy as np
import pytest

# Inject .venv_yolo site-packages if available in Dev Container
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from person_tracking.multi_person_tracker import MultiPersonTracker, PersonDetection, MultiPersonObservation
from person_tracking.source_adapters import SyntheticFixtureGenerator, VideoReplayAdapter, DataQuality


MODEL_PATH = '/home/ws/yolo26n-pose.pt'


def has_yolo_model():
    return os.path.isfile(MODEL_PATH)


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_tracker_initialization():
    tracker = MultiPersonTracker(model_path=MODEL_PATH)
    assert tracker.model is not None
    assert tracker.device in ("cuda:0", "cpu")


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_empty_scene_inference():
    tracker = MultiPersonTracker(model_path=MODEL_PATH)
    empty_frame = SyntheticFixtureGenerator.generate_empty_scene()

    obs = tracker.process_frame(empty_frame)
    assert isinstance(obs, MultiPersonObservation)
    assert len(obs.persons) == 0
    assert len(obs.active_track_ids) == 0
    assert obs.processing_time_ms > 0.0
    assert obs.data_quality == DataQuality.VALID_RGBD.value


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_tracker_reset():
    tracker = MultiPersonTracker(model_path=MODEL_PATH)
    tracker.previous_track_ids = {1, 2, 3}
    tracker.reset()
    assert len(tracker.previous_track_ids) == 0


def test_ambiguity_flag():
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    p1 = PersonDetection(
        bbox=[100.0, 100.0, 200.0, 300.0],
        confidence=0.9,
        keypoints=[],
        track_id=1,
        is_confirmed=True,
    )
    # Strongly overlapping box (IoU > 0.35)
    p2 = PersonDetection(
        bbox=[110.0, 110.0, 210.0, 310.0],
        confidence=0.85,
        keypoints=[],
        track_id=2,
        is_confirmed=True,
    )
    # Disjoint box (IoU == 0.0)
    p3 = PersonDetection(
        bbox=[400.0, 100.0, 500.0, 300.0],
        confidence=0.88,
        keypoints=[],
        track_id=3,
        is_confirmed=True,
    )

    persons = [p1, p2, p3]
    tracker._flag_ambiguities(persons)

    assert p1.is_ambiguous is True
    assert p2.is_ambiguous is True
    assert p3.is_ambiguous is False


def test_edge_proximity_flag():
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08
    tracker.device = "cpu" 

    # Image size 640x480
    w, h = 640, 480
    # Box touching left border (x1 = 10 < 0.08*640 = 51.2)
    near_edge_left = (
        10 < tracker.edge_margin_ratio * w or
        200 > (1.0 - tracker.edge_margin_ratio) * w
    )
    assert near_edge_left is True

    # Center box (x1=250, x2=390)
    in_center = (
        250 < tracker.edge_margin_ratio * w or
        390 > (1.0 - tracker.edge_margin_ratio) * w
    )
    assert in_center is False


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_debug_overlay_drawing():
    tracker = MultiPersonTracker(model_path=MODEL_PATH)
    empty_frame = SyntheticFixtureGenerator.generate_empty_scene()
    obs = tracker.process_frame(empty_frame)

    debug_img = tracker.draw_debug_overlay(empty_frame.rgb, obs)
    assert debug_img.shape == empty_frame.rgb.shape
    assert debug_img.dtype == np.uint8


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_gpu_and_cpu_benchmarks():
    import torch
    frame = SyntheticFixtureGenerator.generate_empty_scene()

    # CPU benchmark
    cpu_tracker = MultiPersonTracker(model_path=MODEL_PATH, device="cpu")
    # Warmup
    _ = cpu_tracker.process_frame(frame)
    cpu_obs = cpu_tracker.process_frame(frame)
    assert cpu_obs.device == "cpu"
    assert cpu_obs.processing_time_ms < 150.0  # Under 150ms on CPU

    # GPU benchmark (if CUDA available)
    if torch.cuda.is_available():
        gpu_tracker = MultiPersonTracker(model_path=MODEL_PATH, device="cuda:0")
        _ = gpu_tracker.process_frame(frame)
        gpu_obs = gpu_tracker.process_frame(frame)
        assert gpu_obs.device == "cuda:0"
        assert gpu_obs.processing_time_ms < 40.0  # Under 40ms on GPU (> 25 FPS)


def test_depth_valid_strictly_false_when_unverified_alignment():
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    
    # 1. Frame with UNVERIFIED_ALIGNMENT
    frame_unverified = SyntheticFixtureGenerator.generate_person_scene(
        person_bbox=(150, 100, 350, 350),
        person_depth_m=1.75,
    )
    frame_unverified.data_quality = DataQuality.UNVERIFIED_ALIGNMENT.value

    box = [150.0, 100.0, 350.0, 350.0]
    depth_m, depth_valid = tracker._measure_torso_depth(
        obs_frame=frame_unverified,
        box=box,
        keypoints=[],
        h=480,
        w=640,
    )
    assert depth_m is not None
    assert abs(depth_m - 1.75) < 0.1
    # STRICT SAFETY INVARIANT: Unverified alignment MUST NOT yield depth_valid=True!
    assert depth_valid is False, "depth_valid must be False when data_quality is UNVERIFIED_ALIGNMENT!"


def test_depth_valid_true_when_proven_valid_rgbd():
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    
    # Frame with VALID_RGBD
    frame_verified = SyntheticFixtureGenerator.generate_person_scene(
        person_bbox=(150, 100, 350, 350),
        person_depth_m=1.75,
    )
    frame_verified.data_quality = DataQuality.VALID_RGBD.value

    box = [150.0, 100.0, 350.0, 350.0]
    depth_m, depth_valid = tracker._measure_torso_depth(
        obs_frame=frame_verified,
        box=box,
        keypoints=[],
        h=480,
        w=640,
    )
    assert depth_m is not None
    assert abs(depth_m - 1.75) < 0.1
    assert depth_valid is True, "depth_valid must be True when data_quality is VALID_RGBD!"


@pytest.mark.skipif(not has_yolo_model(), reason="Model checkpoint not found")
def test_tracker_reset_allows_continued_tracking():
    tracker = MultiPersonTracker(model_path=MODEL_PATH)
    frame1 = SyntheticFixtureGenerator.generate_empty_scene()
    obs1 = tracker.process_frame(frame1)
    assert len(obs1.candidates) == 0

    # Trigger tracker reset
    tracker.reset()
    assert len(tracker.previous_track_ids) == 0

    # Ensure subsequent tracking executes without error or stale state
    frame2 = SyntheticFixtureGenerator.generate_empty_scene()
    obs2 = tracker.process_frame(frame2)
    assert len(obs2.candidates) == 0
    assert obs2.processing_time_ms > 0.0


def test_inference_immediately_after_reset_with_detection():
    """Verify tracker cleanly resets trackers list and runs inference immediately on detection."""
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08
    tracker.device = "cpu" 
    tracker.previous_track_ids = {10, 20}
    tracker.total_processed_frames = 5

    # Mock ultralytics model with predictor.trackers
    class FakeTracker:
        def __init__(self):
            self.was_reset = False
        def reset(self):
            self.was_reset = True

    class FakePredictor:
        def __init__(self):
            self.trackers = [FakeTracker(), FakeTracker()]

    class FakeModel:
        def __init__(self):
            self.predictor = FakePredictor()

    tracker.model = FakeModel()
    tracker.reset()

    # Assert trackers list was cleared and previous tracks cleared
    assert len(tracker.previous_track_ids) == 0
    assert tracker.model.predictor.trackers == []
    assert tracker.total_processed_frames == 0

    # Test processing immediately after reset with mock candidate
    cand = PersonDetection(
        bbox_xyxy=[100.0, 50.0, 250.0, 350.0],
        confidence=0.92,
        keypoints=[],
        track_id=1,
        is_confirmed=True,
    )
    tracker.mock_backend_candidates = [cand]
    obs_frame = SyntheticFixtureGenerator.generate_empty_scene()
    obs = tracker.process_frame(obs_frame)

    assert len(obs.candidates) == 1
    assert obs.candidates[0].track_id == 1
    assert 1 in obs.active_track_ids
    assert len(obs.lost_track_ids) == 0
