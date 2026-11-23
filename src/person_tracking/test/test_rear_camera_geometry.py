#!/usr/bin/env python3
"""Comprehensive unit and contract tests for Phase 5: RGB-D Metrology & Rear Camera Geometry.

Verifies:
1. RGB-D Quality & Torso Measurement:
   - Torso depth validity requires DataQuality.VALID_RGBD.
   - Torso depth with UNVERIFIED_ALIGNMENT has depth_valid = False (not usable for nearest selection).
   - Valid pixel ratio in torso ROI must be >= 30%.
   - MAD dispersion filter rejects noisy depth (> 0.25m).
   - Sensor range checks: depth < 0.4m or > 5.0m rejected.
   - Zero, NaN, noise/outliers, background clutter, stamp mismatch rejected.
2. Rear Camera Geometry & Coordinate Transforms (ADR-10):
   - Normalized pixel offset (center_offset_x_norm) decoupled from optical bearing (bearing_optical_rad).
   - Target left in image (u < cx) -> optical bearing < 0 -> robot left side / base Y < 0.
   - Target center in image (u = cx) -> optical bearing = 0 -> directly behind robot.
   - Target right in image (u > cx) -> optical bearing > 0 -> robot right side / base Y > 0.
   - Missing CameraInfo -> bearing_optical_rad is None, position is None.
   - Missing TF -> position_base is None, bearing_base_rad is None.
   - Frame ID mismatch -> is_metric_valid is False.
   - Invalid intrinsics (fx <= 0, NaN) -> is_metric_valid is False.
3. CRITICAL SAFETY AUDIT: Strictly ZERO velocity commands or /cmd_vel publishers.
"""

import math
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from person_tracking.rear_camera_geometry import (
    CameraIntrinsics,
    CameraMountTransform,
    RearCameraGeometry,
    TargetGeometryResult,
)
from person_tracking.source_adapters import (
    DataQuality,
    ObservationFrame,
    SourceMode,
    create_simulated_camera_info,
)
from person_tracking.multi_person_tracker import MultiPersonTracker, PersonDetection


def _create_mock_camera_info(width=640, height=480, fx=525.0, fy=525.0, cx=320.0, cy=240.0) -> CameraInfo:
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.header.frame_id = "camera_color_optical_frame"
    return info


# ==============================================================================
# SECTION 1: REAR CAMERA GEOMETRY & ADR-10 CONVENTIONS
# ==============================================================================

def test_camera_intrinsics_extraction_and_validation():
    """Verify CameraIntrinsics correctly parses valid CameraInfo and rejects invalid values."""
    # Valid info
    valid_info = _create_mock_camera_info()
    intr = CameraIntrinsics.from_camera_info(valid_info)
    assert intr is not None
    assert intr.fx == 525.0
    assert intr.fy == 525.0
    assert intr.cx == 320.0
    assert intr.cy == 240.0
    assert intr.width == 640
    assert intr.height == 480

    # None info
    assert CameraIntrinsics.from_camera_info(None) is None

    # Invalid fx <= 0
    bad_fx_info = _create_mock_camera_info(fx=0.0)
    assert CameraIntrinsics.from_camera_info(bad_fx_info) is None

    # NaN in intrinsics
    nan_info = _create_mock_camera_info(cx=float('nan'))
    assert CameraIntrinsics.from_camera_info(nan_info) is None

    # Truncated K matrix
    bad_k_info = CameraInfo()
    bad_k_info.k = [525.0, 0.0, 320.0]  # len < 9
    assert CameraIntrinsics.from_camera_info(bad_k_info) is None


def test_rear_mount_transform_orthonormality():
    """Verify ADR-10 mounting transform rotation matrix is orthonormal with det(R) = +1."""
    mount = CameraMountTransform.create_synthetic_rear_mount(translation=(-0.20, 0.0, 0.25))
    r = mount.rotation_matrix

    # Check determinant is +1 (proper rotation, no reflection)
    det = np.linalg.det(r)
    assert math.isclose(det, 1.0, rel_tol=1e-6)

    # Check orthonormality R * R^T = I
    ident = r @ r.T
    assert np.allclose(ident, np.eye(3), atol=1e-6)

    # Test transformation of forward-looking optical ray [0, 0, 1] (out of rear lens)
    p_opt_forward = np.array([0.0, 0.0, 1.0])
    p_base = mount.transform_point(p_opt_forward)
    # Optical Z=+1 must point towards -X in base_link (behind robot): -1.0 + (-0.20) = -1.20
    assert math.isclose(p_base[0], -1.20, abs_tol=1e-5)
    assert math.isclose(p_base[1], 0.0, abs_tol=1e-5)
    assert math.isclose(p_base[2], 0.25, abs_tol=1e-5)


def test_adr10_target_left_center_right_geometry():
    """Verify signed angles and coordinates for targets on image left, center, right per ADR-10."""
    geom_engine = RearCameraGeometry()
    intrinsics = CameraIntrinsics.from_camera_info(_create_mock_camera_info(fx=500.0, cx=320.0, cy=240.0))
    mount_tf = CameraMountTransform.create_synthetic_rear_mount(translation=(-0.20, 0.0, 0.25))
    target_depth = 2.0  # 2 meters

    # --- 1. TARGET IN CENTER (u = 320) ---
    res_center = geom_engine.compute_geometry(
        u=320.0, v=240.0,
        torso_depth_m=target_depth, depth_valid=True,
        intrinsics=intrinsics, mount_tf=mount_tf,
    )
    assert res_center.is_metric_valid is True
    assert math.isclose(res_center.center_offset_x_norm, 0.0, abs_tol=1e-5)
    assert math.isclose(res_center.optical_bearing_rad, 0.0, abs_tol=1e-5)
    # Optical position: [0.0, 0.0, 2.0]
    assert np.allclose(res_center.position_optical, [0.0, 0.0, 2.0], atol=1e-5)
    # Base position: X = -2.2m (behind), Y = 0.0m (center), Z = 0.25m
    assert math.isclose(res_center.position_base[0], -2.20, abs_tol=1e-5)
    assert math.isclose(res_center.position_base[1], 0.0, abs_tol=1e-5)
    # Base bearing for point directly behind robot: atan2(0, -2.2) = +/- pi
    assert math.isclose(abs(res_center.base_bearing_rad), math.pi, abs_tol=1e-5)

    # --- 2. TARGET ON RIGHT OF IMAGE (u = 420 > cx = 320) ---
    # Per ADR-10: Target on image right corresponds to robot's LEFT side (Y_base > 0)
    res_right = geom_engine.compute_geometry(
        u=420.0, v=240.0,
        torso_depth_m=target_depth, depth_valid=True,
        intrinsics=intrinsics, mount_tf=mount_tf,
    )
    assert res_right.is_metric_valid is True
    assert res_right.center_offset_x_norm > 0.0  # Normalized offset > 0
    assert res_right.optical_bearing_rad > 0.0   # Optical bearing > 0
    # Optical X = (420 - 320) * 2.0 / 500 = +0.40m
    assert math.isclose(res_right.position_optical[0], 0.40, abs_tol=1e-5)
    # In base_link: Y_base = X_opt = +0.40m (Robot's LEFT side!)
    assert math.isclose(res_right.position_base[1], 0.40, abs_tol=1e-5)
    # Base bearing: in second quadrant (pi/2 < bearing < pi)
    assert math.pi / 2.0 < res_right.base_bearing_rad < math.pi

    # --- 3. TARGET ON LEFT OF IMAGE (u = 220 < cx = 320) ---
    # Per ADR-10: Target on image left corresponds to robot's RIGHT side (Y_base < 0)
    res_left = geom_engine.compute_geometry(
        u=220.0, v=240.0,
        torso_depth_m=target_depth, depth_valid=True,
        intrinsics=intrinsics, mount_tf=mount_tf,
    )
    assert res_left.is_metric_valid is True
    assert res_left.center_offset_x_norm < 0.0  # Normalized offset < 0
    assert res_left.optical_bearing_rad < 0.0   # Optical bearing < 0
    # Optical X = (220 - 320) * 2.0 / 500 = -0.40m
    assert math.isclose(res_left.position_optical[0], -0.40, abs_tol=1e-5)
    # In base_link: Y_base = X_opt = -0.40m (Robot's RIGHT side!)
    assert math.isclose(res_left.position_base[1], -0.40, abs_tol=1e-5)
    # Base bearing: in third quadrant (-pi < bearing < -pi/2)
    assert -math.pi < res_left.base_bearing_rad < -math.pi / 2.0


def test_geometry_failure_modes_missing_tf_and_intrinsics():
    """Verify geometry engine cleanly handles missing TF, invalid intrinsics, and frame mismatch."""
    geom_engine = RearCameraGeometry(expected_frame_id="camera_color_optical_frame")
    valid_intr = CameraIntrinsics.from_camera_info(_create_mock_camera_info())
    valid_mount = CameraMountTransform.create_synthetic_rear_mount()

    # Case 1: Missing CameraInfo
    res_no_info = geom_engine.compute_geometry(
        u=350.0, v=240.0, torso_depth_m=1.5, depth_valid=True,
        intrinsics=None, mount_tf=valid_mount,
    )
    assert res_no_info.center_offset_x_norm != 0.0  # Pixel offset still computed!
    assert res_no_info.optical_bearing_rad is None
    assert res_no_info.position_base is None
    assert res_no_info.is_metric_valid is False
    assert "INVALID_OR_MISSING_CAMERA_INFO" in res_no_info.error_reason

    # Case 2: Missing TF
    res_no_tf = geom_engine.compute_geometry(
        u=350.0, v=240.0, torso_depth_m=1.5, depth_valid=True,
        intrinsics=valid_intr, mount_tf=None,
    )
    assert res_no_tf.optical_bearing_rad is not None  # Optical bearing computed
    assert res_no_tf.position_base is None           # Base position requires TF!
    assert res_no_tf.base_bearing_rad is None
    assert res_no_tf.is_metric_valid is False
    assert "MISSING_TF" in res_no_tf.error_reason

    # Case 3: Frame ID mismatch
    res_frame_mismatch = geom_engine.compute_geometry(
        u=350.0, v=240.0, torso_depth_m=1.5, depth_valid=True,
        intrinsics=valid_intr, mount_tf=valid_mount,
        frame_id="some_other_optical_frame",
    )
    assert res_frame_mismatch.is_metric_valid is False
    assert "FRAME_ID_MISMATCH" in res_frame_mismatch.error_reason

    # Case 4: Unverified alignment / invalid depth
    res_unverified = geom_engine.compute_geometry(
        u=350.0, v=240.0, torso_depth_m=1.5, depth_valid=False,
        intrinsics=valid_intr, mount_tf=valid_mount,
    )
    assert res_unverified.optical_bearing_rad is not None
    assert res_unverified.position_base is None
    assert res_unverified.is_metric_valid is False
    assert "UNVERIFIED_OR_INVALID_DEPTH" in res_unverified.error_reason


# ==============================================================================
# SECTION 2: DEPTH QUALITY & TORSO MEASUREMENT FILTERS
# ==============================================================================

def test_torso_depth_valid_strictly_false_when_unverified_alignment():
    """Verify depth_valid is strictly False when data_quality == UNVERIFIED_ALIGNMENT."""
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08

    # Create dummy RGB (480x640) and Depth image with perfect 1.8m depth
    rgb_dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_valid_data = np.full((480, 640), 1.80, dtype=np.float32)

    obs_frame = ObservationFrame(
        rgb=rgb_dummy,
        depth=depth_valid_data,
        depth_raw=None,
        camera_info=None,
        stamp_sec=10,
        stamp_nanosec=0,
        receive_monotonic=10.0,
        frame_id="camera_color_optical_frame",
        source_mode=SourceMode.LIVE_ASTRA.value,
        data_quality=DataQuality.UNVERIFIED_ALIGNMENT.value,  # Alignment unverified!
    )

    box = [200.0, 100.0, 400.0, 400.0]
    depth_m, depth_valid = tracker._measure_torso_depth(obs_frame, box, [], 480, 640)

    # Raw measurement is computed for logging, but depth_valid MUST BE FALSE
    assert depth_m is not None
    assert math.isclose(depth_m, 1.80, abs_tol=1e-3)
    assert depth_valid is False, "CRITICAL ERROR: depth_valid must be False when UNVERIFIED_ALIGNMENT"


def test_torso_depth_valid_true_when_valid_rgbd():
    """Verify depth_valid is True when data_quality == VALID_RGBD and depth is clean."""
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08

    rgb_dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_valid_data = np.full((480, 640), 2.10, dtype=np.float32)

    obs_frame = ObservationFrame(
        rgb=rgb_dummy,
        depth=depth_valid_data,
        depth_raw=None,
        camera_info=None,
        stamp_sec=10,
        stamp_nanosec=0,
        receive_monotonic=10.0,
        frame_id="camera_color_optical_frame",
        source_mode=SourceMode.LIVE_ASTRA.value,
        data_quality=DataQuality.VALID_RGBD.value,
    )

    box = [200.0, 100.0, 400.0, 400.0]
    depth_m, depth_valid = tracker._measure_torso_depth(obs_frame, box, [], 480, 640)

    assert depth_m is not None
    assert math.isclose(depth_m, 2.10, abs_tol=1e-3)
    assert depth_valid is True


def test_torso_depth_rejects_zero_nan_and_out_of_range():
    """Verify zero, NaN, and sensor range extremes (< 0.4m, > 5.0m) are rejected."""
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08
    rgb_dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    box = [200.0, 100.0, 400.0, 400.0]

    # 1. All Zeros
    d_zero = np.zeros((480, 640), dtype=np.float32)
    obs_zero = ObservationFrame(
        rgb=rgb_dummy, depth=d_zero, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_z, valid_z = tracker._measure_torso_depth(obs_zero, box, [], 480, 640)
    assert val_z is None
    assert valid_z is False

    # 2. All NaNs
    d_nan = np.full((480, 640), np.nan, dtype=np.float32)
    obs_nan = ObservationFrame(
        rgb=rgb_dummy, depth=d_nan, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_n, valid_n = tracker._measure_torso_depth(obs_nan, box, [], 480, 640)
    assert val_n is None
    assert valid_n is False

    # 3. Too close (< 0.4m)
    d_close = np.full((480, 640), 0.35, dtype=np.float32)
    obs_close = ObservationFrame(
        rgb=rgb_dummy, depth=d_close, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_c, valid_c = tracker._measure_torso_depth(obs_close, box, [], 480, 640)
    assert val_c is None
    assert valid_c is False

    # 4. Too far (> 5.0m)
    d_far = np.full((480, 640), 5.50, dtype=np.float32)
    obs_far = ObservationFrame(
        rgb=rgb_dummy, depth=d_far, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_f, valid_f = tracker._measure_torso_depth(obs_far, box, [], 480, 640)
    assert val_f is None
    assert valid_f is False


def test_torso_depth_rejects_noisy_mad_dispersion_and_background_clutter():
    """Verify MAD filter rejects noisy depth (> 0.25m) and background clutter with valid ratio < 30%."""
    tracker = MultiPersonTracker.__new__(MultiPersonTracker)
    tracker.edge_margin_ratio = 0.08
    rgb_dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    box = [200.0, 100.0, 400.0, 400.0]

    # 1. High MAD dispersion (noise/edge jitter > 0.25m)
    # Mix depths between 1.0m and 2.5m randomly in torso ROI
    rng = np.random.default_rng(42)
    noisy_depth = rng.uniform(1.0, 2.5, size=(480, 640)).astype(np.float32)
    obs_noisy = ObservationFrame(
        rgb=rgb_dummy, depth=noisy_depth, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_m, valid_m = tracker._measure_torso_depth(obs_noisy, box, [], 480, 640)
    assert val_m is None, "MAD filter failed to reject noisy dispersion (> 0.25m)"
    assert valid_m is False

    # 2. Background clutter: 80% invalid/NaN pixels (valid ratio = 20% < 30% threshold)
    clutter_depth = np.full((480, 640), np.nan, dtype=np.float32)
    # Torso ROI is inner 40%: x in [260, 340], y in [160, 280] (80x120 = 9600 pixels)
    # Set only 15% to valid 1.5m
    clutter_depth[160:175, 260:340] = 1.50
    obs_clutter = ObservationFrame(
        rgb=rgb_dummy, depth=clutter_depth, depth_raw=None, camera_info=None,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    val_cl, valid_cl = tracker._measure_torso_depth(obs_clutter, box, [], 480, 640)
    assert val_cl is None, "Failed to reject background clutter with valid depth ratio < 30%"
    assert valid_cl is False


# ==============================================================================
# SECTION 3: CRITICAL SAFETY AUDIT
# ==============================================================================

def test_critical_safety_zero_motion_output_in_geometry_module():
    """Verify rear_camera_geometry module contains NO velocity generation or publishers."""
    import person_tracking.rear_camera_geometry as rcg
    import inspect

    source_code = inspect.getsource(rcg)
    assert "cmd_vel" not in source_code
    assert "Twist" not in source_code
    assert "geometry_msgs" not in source_code
    assert "create_publisher" not in source_code
    assert "linear.x" not in source_code
    assert "angular.z" not in source_code


def test_rear_mount_transform_frame_id_mismatch():
    """Section A.3: Transform parent/child frame mismatch returns geometry_valid=False with descriptive error."""
    geom_engine = RearCameraGeometry(expected_frame_id="camera_color_optical_frame")
    intr = CameraIntrinsics.from_camera_info(_create_mock_camera_info())

    # 1. Wrong parent frame (e.g. odom instead of base_link)
    bad_parent_tf = CameraMountTransform(
        translation=np.array([-0.2, 0, 0.25]),
        rotation_matrix=np.eye(3),
        parent_frame="odom",
        child_frame="camera_color_optical_frame",
    )
    res1 = geom_engine.compute_geometry(
        u=320, v=240, torso_depth_m=1.5, depth_valid=True,
        intrinsics=intr, mount_tf=bad_parent_tf,
    )
    assert res1.geometry_valid is False
    assert res1.position_base is None
    assert res1.base_bearing_rad is None
    assert res1.geometry_source == "OPTICAL_ONLY"
    assert "TF_FRAME_MISMATCH" in res1.error_reason
    assert "expected parent 'base_link'" in res1.error_reason

    # 2. Wrong child frame (e.g. camera_link instead of camera_color_optical_frame)
    bad_child_tf = CameraMountTransform(
        translation=np.array([-0.2, 0, 0.25]),
        rotation_matrix=np.eye(3),
        parent_frame="base_link",
        child_frame="camera_link",
    )
    res2 = geom_engine.compute_geometry(
        u=320, v=240, torso_depth_m=1.5, depth_valid=True,
        intrinsics=intr, mount_tf=bad_child_tf,
    )
    assert res2.geometry_valid is False
    assert res2.position_base is None
    assert res2.base_bearing_rad is None
    assert res2.geometry_source == "OPTICAL_ONLY"
    assert "TF_FRAME_MISMATCH" in res2.error_reason
    assert "camera_link" in res2.error_reason


def test_tracker_observation_geometry_opt_in_and_unverified_depth():
    """Section A.3: MultiPersonTracker runtime defaults to no synthetic TF (position_base=None); explicit opt-in enables projection."""
    from person_tracking.multi_person_tracker import MultiPersonTracker, PersonDetection
    from person_tracking.source_adapters import ObservationFrame, SourceMode, DataQuality

    # Mock candidate
    mock_cand = PersonDetection(
        bbox_xyxy=[100.0, 80.0, 260.0, 420.0],
        confidence=0.92,
        keypoints=[[200.0, 200.0, 0.9]] * 17,
        track_id=1,
        is_confirmed=True,
        is_ambiguous=False,
    )
    cam_info = _create_mock_camera_info()
    rgb_dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_valid = np.full((480, 640), 1.5, dtype=np.float32)

    # 1. Runtime default: use_synthetic_mount=False
    tracker_no_tf = MultiPersonTracker(use_synthetic_mount=False)
    tracker_no_tf.mock_backend_candidates = [mock_cand]
    obs_frame_no_tf = ObservationFrame(
        rgb=rgb_dummy, depth=depth_valid, depth_raw=None, camera_info=cam_info,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.VALID_RGBD.value,
    )
    obs_out1 = tracker_no_tf.process_frame(obs_frame_no_tf)
    c1 = obs_out1.candidates[0]
    assert c1.position_base is None, "SAFETY VIOLATION: position_base generated without mount TF!"
    assert c1.bearing_base_rad is None
    assert c1.geometry_valid is False
    assert c1.geometry_source == "OPTICAL_ONLY"
    assert c1.bearing_optical_rad is not None  # Optical bearing is valid

    # 2. Test/Offline opt-in: use_synthetic_mount=True
    tracker_syn_tf = MultiPersonTracker(use_synthetic_mount=True)
    tracker_syn_tf.mock_backend_candidates = [mock_cand]
    obs_out2 = tracker_syn_tf.process_frame(obs_frame_no_tf)
    c2 = obs_out2.candidates[0]
    assert c2.geometry_valid is True
    assert c2.position_base is not None
    assert c2.bearing_base_rad is not None
    assert c2.geometry_source == "SYNTHETIC_TEST"

    # 3. UNVERIFIED_ALIGNMENT with synthetic mount enabled: depth_valid MUST be False, geometry_valid MUST be False
    obs_frame_unverified = ObservationFrame(
        rgb=rgb_dummy, depth=depth_valid, depth_raw=None, camera_info=cam_info,
        stamp_sec=1, stamp_nanosec=0, receive_monotonic=1.0, frame_id="camera_color_optical_frame",
        source_mode=SourceMode.SYNTHETIC_FIXTURE.value, data_quality=DataQuality.UNVERIFIED_ALIGNMENT.value,
    )
    obs_out3 = tracker_syn_tf.process_frame(obs_frame_unverified)
    c3 = obs_out3.candidates[0]
    assert c3.depth_valid is False, "SAFETY VIOLATION: depth_valid must be False when UNVERIFIED_ALIGNMENT!"
    assert c3.geometry_valid is False, "SAFETY VIOLATION: geometry_valid must be False when depth is unverified!"
    assert c3.position_base is None
