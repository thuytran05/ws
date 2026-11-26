"""Unit & ROS runtime tests for Astra S QoS sync, latest-frame priority, and camera_ready gating."""

import json
import time

import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from person_tracking.passenger_lock_manager import PassengerLockManager, TrackingState
from person_tracking.person_detector_node import PersonDetectorNode
from person_tracking.source_adapters import DataQuality, ObservationFrame, SourceMode
from person_tracking.velocity_guard import VelocityGuard


def test_build_stream_qos_reliable_keep_last_volatile_depth5():
    qos = PersonDetectorNode._build_stream_qos("reliable", 5)
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 5

    qos_be = PersonDetectorNode._build_stream_qos("best_effort", 1)
    assert qos_be.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_be.depth == 1


def test_camera_readiness_gate_blocks_lock_and_velocity_guard():
    # 1. PassengerLockManager must NOT accumulate gesture hold when camera_ready=False
    mgr = PassengerLockManager(gesture_hold_time_sec=1.5)
    obs_not_ready = {
        "stamp": {"sec": 10, "nanosec": 0},
        "source_session_id": "src-astra",
        "camera_ready": False,
        "camera_readiness_reason": "CAMERA_WARMING_UP_2_5",
        "candidates": [{
            "track_id": 1,
            "is_confirmed": True,
            "is_ambiguous": False,
            "confidence": 0.95,
            "keypoints": [[320, 200, 0.9]] + [[0, 0, 0]] * 8 + [[300, 100, 0.9]] + [[0, 0, 0]] * 7,
        }],
    }
    snap = mgr.process_observation(obs_not_ready, current_time_sec=10.0)
    assert snap.locked is False
    assert snap.state == TrackingState.IDLE.value
    assert snap.reason_code == "CAMERA_WARMING_UP_2_5"
    assert len(mgr.gesture_state) == 0

    # 2. VelocityGuard must clamp to zero when camera_ready=False
    guard = VelocityGuard()
    t = 100.0
    guard.update_session_state({
        "stamp": {"sec": 10, "nanosec": 0},
        "state": "NAVIGATING",
        "locked": True,
        "person_track_id": 1,
        "source_session_id": "src-astra",
        "obs_fresh": True,
        "session_id": "s1",
    }, now_mono=t)
    guard.update_observation({
        "stamp": {"sec": 10, "nanosec": 50_000_000},
        "source_session_id": "src-astra",
        "camera_ready": False,
        "camera_readiness_reason": "ASTRA_STREAM_INCOMPLETE_RGB_ONLY",
        "candidates": [{
            "track_id": 1,
            "is_confirmed": True,
            "is_ambiguous": False,
            "depth_valid": True,
            "torso_depth_m": 1.5,
            "center_offset_x_norm": 0.0,
        }],
    }, now_mono=t + 0.05)
    guard.update_cmd_vel(0.30, 0.0, 0.0, now_mono=t + 0.06)
    vx, vy, wz, reason = guard.process_velocity(now_mono=t + 0.07)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert "CAMERA" in reason or "NO_VALID" in reason


def test_camera_readiness_warmup_and_degradation_logic():
    node = PersonDetectorNode.__new__(PersonDetectorNode)
    node.readiness_min_frames = 5
    node._consecutive_ready_frames = 0

    def make_obs_frame(dq, sm=SourceMode.LIVE_ASTRA.value):
        return ObservationFrame(
            rgb=np.zeros((480, 640, 3), dtype=np.uint8),
            depth=None,
            depth_raw=None,
            camera_info=None,
            stamp_sec=1,
            stamp_nanosec=0,
            receive_monotonic=1.0,
            frame_id="camera_color_optical_frame",
            source_mode=sm,
            data_quality=dq,
        )

    # First 4 valid frames -> warming up (False)
    for idx in range(1, 5):
        ready, reason = node._evaluate_camera_readiness(make_obs_frame(DataQuality.VALID_RGBD.value))
        assert ready is False
        assert f"CAMERA_WARMING_UP_{idx}_5" == reason

    # 5th frame -> READY
    ready, reason = node._evaluate_camera_readiness(make_obs_frame(DataQuality.VALID_RGBD.value))
    assert ready is True
    assert reason == "CAMERA_READY"

    # Frame ID mismatch -> immediately resets readiness
    ready, reason = node._evaluate_camera_readiness(make_obs_frame(DataQuality.FRAME_ID_MISMATCH.value))
    assert ready is False
    assert reason == "CAMERA_DEGRADED_FRAME_ID_MISMATCH"
    assert node._consecutive_ready_frames == 0


def test_prioritize_latest_frame_drops_old_frames_when_yolo_slow():
    """Verify prioritize_latest_frame=True skips intermediate frames when inference is slow."""
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rclpy.init()
    node = None
    try:
        node = PersonDetectorNode()
        # Enable async latest-frame worker dynamically for test
        import threading
        node.prioritize_latest_frame = True
        node._infer_stop_event.clear()
        node._infer_thread = threading.Thread(target=node._infer_worker_loop, daemon=True)
        node._infer_thread.start()

        processed_stamps = []

        def slow_process_frame(obs_frame):
            time.sleep(0.06)
            processed_stamps.append(obs_frame.stamp_nanosec)
            from person_tracking.multi_person_tracker import MultiPersonObservation
            return MultiPersonObservation(
                stamp_sec=obs_frame.stamp_sec,
                stamp_nanosec=obs_frame.stamp_nanosec,
                receive_monotonic_time=obs_frame.receive_monotonic,
                frame_id=obs_frame.frame_id,
                source_mode=obs_frame.source_mode,
                data_quality=obs_frame.data_quality,
            )

        node.tracker.process_frame = slow_process_frame
        bridge = CvBridge()
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        now_ns = node.get_clock().now().nanoseconds

        # Push 6 frames rapidly (<10ms total) while each inference takes 60ms
        for i in range(6):
            msg = bridge.cv2_to_imgmsg(dummy, encoding='bgr8')
            t_ns = now_ns + i * 1_000_000
            msg.header.stamp.sec = int(t_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(t_ns % 1_000_000_000)
            msg.header.frame_id = "camera_color_optical_frame"
            node._process_rgb_message(msg, matched_status=None)
            time.sleep(0.002)

        time.sleep(0.25)
        assert node._dropped_old_infer_frames >= 3
        assert len(processed_stamps) < 6
        # Last processed frame must be the newest frame (i=5)
        expected_last_ns = int((now_ns + 5 * 1_000_000) % 1_000_000_000)
        assert processed_stamps[-1] == expected_last_ns
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
