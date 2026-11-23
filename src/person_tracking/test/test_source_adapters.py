"""Unit tests for person_tracking source adapters and data quality verification."""

import math
import os
import time
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from person_tracking.source_adapters import (
    SourceMode,
    DataQuality,
    ObservationFrame,
    compute_mad,
    create_simulated_camera_info,
    VideoReplayAdapter,
    SyntheticFixtureGenerator,
    LiveAstraAdapter,
)


def test_observation_frame_contract():
    frame = ObservationFrame(
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth=np.full((480, 640), 2.0, dtype=np.float32),
        depth_raw=np.full((480, 640), 2000, dtype=np.uint16),
        camera_info=CameraInfo(),
        stamp_sec=100,
        stamp_nanosec=500000000,
        receive_monotonic=time.monotonic(),
        frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value,
        data_quality=DataQuality.VALID_RGBD.value,
    )
    assert frame.has_depth is True
    assert math.isclose(frame.stamp_total_seconds, 100.5, rel_tol=1e-6)

    # RGB-only frame should report has_depth False
    rgb_only = ObservationFrame(
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth=None,
        depth_raw=None,
        camera_info=None,
        stamp_sec=100,
        stamp_nanosec=0,
        receive_monotonic=time.monotonic(),
        frame_id="camera_color_optical_frame",
        source_mode=SourceMode.VIDEO_REPLAY.value,
        data_quality=DataQuality.RGB_ONLY.value,
    )
    assert rgb_only.has_depth is False


def test_compute_mad():
    data = np.array([1.5, 1.52, 1.48, 1.51, 1.49, 10.0])  # Contains outlier 10.0
    med = float(np.median(data))
    mad = compute_mad(data, med)
    assert mad < 0.05  # Robust against outlier


def test_create_simulated_camera_info():
    info = create_simulated_camera_info(640, 480, hfov_deg=60.0)
    assert info.width == 640
    assert info.height == 480
    assert info.distortion_model == "simulated_pinhole"
    assert math.isclose(info.k[2], 320.0)  # cx
    assert math.isclose(info.k[5], 240.0)  # cy
    assert info.k[0] > 0  # fx


def test_synthetic_fixture_generator_empty():
    frame = SyntheticFixtureGenerator.generate_empty_scene()
    assert frame.rgb.shape == (480, 640, 3)
    assert frame.depth.shape == (480, 640)
    assert np.allclose(frame.depth, 3.0)
    assert frame.data_quality == DataQuality.VALID_RGBD.value


def test_synthetic_fixture_generator_person():
    frame = SyntheticFixtureGenerator.generate_person_scene(
        person_bbox=(200, 100, 400, 400),
        person_depth_m=1.5,
        bg_depth_m=3.5,
    )
    assert frame.has_depth is True
    assert math.isclose(float(frame.depth[200, 300]), 1.5, rel_tol=1e-3)
    assert math.isclose(float(frame.depth[50, 50]), 3.5, rel_tol=1e-3)


def test_synthetic_fixture_generator_corrupted_depth():
    frame = SyntheticFixtureGenerator.generate_corrupted_depth_scene(corruption_type="nan_and_zeros")
    assert frame.data_quality == DataQuality.INVALID_DEPTH.value
    # Center contains NaN
    assert np.isnan(frame.depth[200, 300])


def test_video_replay_adapter(tmp_path):
    # Test reading the synthetic_video.mp4 generated in fixtures
    video_path = os.path.join(os.path.dirname(__file__), "..", "fixtures", "synthetic_video.mp4")
    if not os.path.isfile(video_path):
        pytest.skip("synthetic_video.mp4 not generated yet.")

    adapter = VideoReplayAdapter(video_path=video_path, fps=15.0, loop=False, retime_stamps=True)
    assert adapter.width == 640
    assert adapter.height == 480
    assert adapter.total_frames == 30

    frame1 = adapter.read_frame()
    assert frame1 is not None
    assert frame1.rgb.shape == (480, 640, 3)
    # RGB-only guarantee: depth is strictly None, never synthesized!
    assert frame1.depth is None
    assert frame1.data_quality == DataQuality.RGB_ONLY.value

    # Read until EOF
    frames_read = 1
    while True:
        f = adapter.read_frame()
        if f is None:
            break
        frames_read += 1
    assert frames_read == 30
    assert adapter.eof is True
    adapter.close()


def test_live_astra_adapter_valid():
    adapter = LiveAstraAdapter(max_capture_age_sec=0.40, require_hardware_alignment_proof=True)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_16u = np.full((480, 640), 1500, dtype=np.uint16)  # 1500 mm = 1.5m
    info = create_simulated_camera_info(640, 480)

    now = 1000.0
    frame = adapter.process_messages(
        rgb_img=rgb,
        rgb_header_stamp=(1000, 0),
        rgb_frame_id="camera_color_optical_frame",
        depth_img=depth_16u,
        depth_header_stamp=(1000, 0),
        depth_frame_id="camera_color_optical_frame",
        depth_encoding="16UC1",
        camera_info=info,
        current_time_sec=now,
    )
    assert frame.has_depth is True
    assert math.isclose(float(frame.depth[240, 320]), 1.5, rel_tol=1e-3)
    # Flagged as UNVERIFIED_ALIGNMENT because hardware proof is required
    assert frame.data_quality == DataQuality.UNVERIFIED_ALIGNMENT.value


def test_live_astra_adapter_stale():
    adapter = LiveAstraAdapter(max_capture_age_sec=0.40)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    info = create_simulated_camera_info(640, 480)

    # Frame is 1.0s old (exceeds 0.40s limit)
    frame = adapter.process_messages(
        rgb_img=rgb,
        rgb_header_stamp=(999, 0),
        rgb_frame_id="camera_color_optical_frame",
        depth_img=None,
        depth_header_stamp=None,
        depth_frame_id=None,
        depth_encoding=None,
        camera_info=info,
        current_time_sec=1000.0,
    )
    assert frame.data_quality == DataQuality.STALE.value


def test_live_astra_adapter_dimension_mismatch():
    adapter = LiveAstraAdapter()
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_mismatched = np.zeros((240, 320), dtype=np.uint16)
    info = create_simulated_camera_info(640, 480)

    frame = adapter.process_messages(
        rgb_img=rgb,
        rgb_header_stamp=(1000, 0),
        rgb_frame_id="camera_color_optical_frame",
        depth_img=depth_mismatched,
        depth_header_stamp=(1000, 0),
        depth_frame_id="camera_color_optical_frame",
        depth_encoding="16UC1",
        camera_info=info,
        current_time_sec=1000.0,
    )
    assert frame.data_quality == DataQuality.DIMENSION_MISMATCH.value


def test_live_astra_adapter_frame_id_mismatch():
    adapter = LiveAstraAdapter()
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.zeros((480, 640), dtype=np.uint16)
    info = create_simulated_camera_info(640, 480)

    frame = adapter.process_messages(
        rgb_img=rgb,
        rgb_header_stamp=(1000, 0),
        rgb_frame_id="camera_color_optical_frame",
        depth_img=depth,
        depth_header_stamp=(1000, 0),
        depth_frame_id="camera_depth_optical_frame",  # Different frame!
        depth_encoding="16UC1",
        camera_info=info,
        current_time_sec=1000.0,
    )
    assert frame.data_quality == DataQuality.FRAME_ID_MISMATCH.value
