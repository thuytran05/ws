import numpy as np
"""Integration smoke test for person_detector_node in person_tracking Phase 3."""

import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from person_tracking.perception_source_node import PerceptionSourceNode
from person_tracking.person_detector_node import PersonDetectorNode


class PerceptionObserverSubscriber(Node):
    def __init__(self):
        super().__init__('perception_observer_subscriber')
        self.received_obs = []
        self.received_debug_img = []

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, '/tracking/observation', self._on_obs, reliable_qos
        )
        self.create_subscription(
            Image, '/tracking/debug_image', self._on_img, qos_profile_sensor_data
        )

    def _on_obs(self, msg: String):
        self.received_obs.append(msg)

    def _on_img(self, msg: Image):
        self.received_debug_img.append(msg)


def test_person_detector_node_smoke():
    if not rclpy.ok():
        rclpy.init()
    try:
        source_node = PerceptionSourceNode()
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()

        # Spin nodes for ~2.0 seconds
        start_time = time.time()
        timeout = 6.0
        while time.time() - start_time < timeout:
            rclpy.spin_once(source_node, timeout_sec=0.05)
            rclpy.spin_once(detector_node, timeout_sec=0.05)
            rclpy.spin_once(test_sub, timeout_sec=0.05)
            if len(test_sub.received_obs) >= 5 and len(test_sub.received_debug_img) >= 5:
                break

        # Verification 1: Received perception messages
        assert len(test_sub.received_obs) >= 3, f"Expected >= 3 obs, got {len(test_sub.received_obs)}"
        assert len(test_sub.received_debug_img) >= 3, f"Expected >= 3 debug images, got {len(test_sub.received_debug_img)}"

        # Verification 2: Payload schema
        first_payload = json.loads(test_sub.received_obs[0].data)
        assert first_payload["schema_version"] == "1.0"
        assert "stamp" in first_payload
        assert "persons" in first_payload
        assert "active_track_ids" in first_payload
        assert "lost_track_ids" in first_payload
        assert "processing_time_ms" in first_payload
        assert first_payload["processing_time_ms"] > 0.0

        # Verification 3: Debug image
        first_img = test_sub.received_debug_img[0]
        assert first_img.width == 640
        assert first_img.height == 480
        assert first_img.encoding == "bgr8"

        # Verification 4: CRITICAL SAFETY CHECK - Zero /cmd_vel publishers!
        det_pubs = detector_node.get_publisher_names_and_types_by_node(
            detector_node.get_name(), detector_node.get_namespace()
        )
        src_pubs = source_node.get_publisher_names_and_types_by_node(
            source_node.get_name(), source_node.get_namespace()
        )
        all_topics = [t for t, _ in det_pubs] + [t for t, _ in src_pubs]
        assert "/cmd_vel" not in all_topics, "SAFETY VIOLATION: /cmd_vel found in publisher list!"
        assert "cmd_vel" not in all_topics

        source_node.destroy_node()
        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_observation_schema_types():
    """Verify JSON schema v1.0 data types and fields on /tracking/observation."""
    if not rclpy.ok():
        rclpy.init()
    try:
        source_node = PerceptionSourceNode()
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()

        start_time = time.time()
        timeout = 6.0
        while time.time() - start_time < timeout:
            rclpy.spin_once(source_node, timeout_sec=0.05)
            rclpy.spin_once(detector_node, timeout_sec=0.05)
            rclpy.spin_once(test_sub, timeout_sec=0.05)
            if len(test_sub.received_obs) >= 1:
                break

        assert len(test_sub.received_obs) >= 1, "Failed to receive observation payload"
        payload = json.loads(test_sub.received_obs[0].data)

        # 1. Top-level mandatory schema types
        assert payload["schema_version"] == "1.0"
        assert isinstance(payload["stamp"], dict)
        assert isinstance(payload["stamp"]["sec"], int)
        assert isinstance(payload["stamp"]["nanosec"], int)
        assert isinstance(payload["frame_id"], str)
        assert isinstance(payload["receive_monotonic_time"], (int, float))
        assert payload["receive_monotonic_time"] > 0
        assert payload["source_mode"] in ["live_astra", "video_replay", "bag_replay", "synthetic_fixture"]
        assert isinstance(payload["source_session_id"], str) and len(payload["source_session_id"]) > 0
        assert payload["data_quality"] in [
            "VALID_RGBD", "RGB_ONLY", "UNVERIFIED_ALIGNMENT", "DEGRADED_NO_INFO",
            "STALE", "DIMENSION_MISMATCH", "FRAME_ID_MISMATCH", "INVALID_DEPTH"
        ]
        assert isinstance(payload["processing_time_ms"], (int, float))
        assert payload["processing_time_ms"] >= 0.0
        assert isinstance(payload["device"], str)
        assert isinstance(payload["active_track_ids"], list)
        assert isinstance(payload["lost_track_ids"], list)
        assert isinstance(payload["candidates"], list)

        # 2. Aliases for backward compatibility
        assert "persons" in payload
        assert "receive_monotonic" in payload
        assert payload["persons"] == payload["candidates"]
        assert payload["receive_monotonic"] == payload["receive_monotonic_time"]

        # 3. If candidates exist, verify individual candidate schema types
        for cand in payload["candidates"]:
            assert cand["track_id"] is None or isinstance(cand["track_id"], int)
            assert isinstance(cand["is_confirmed"], bool)
            assert isinstance(cand["is_ambiguous"], bool)
            assert isinstance(cand["confidence"], float)
            assert 0.0 <= cand["confidence"] <= 1.0
            assert isinstance(cand["bbox_xyxy"], list)
            assert len(cand["bbox_xyxy"]) == 4
            for coord in cand["bbox_xyxy"]:
                assert isinstance(coord, (int, float))
            assert cand["torso_depth_m"] is None or isinstance(cand["torso_depth_m"], (int, float))
            assert isinstance(cand["depth_valid"], bool)
            assert isinstance(cand["center_offset_x_norm"], (int, float))
            assert -1.0 <= cand["center_offset_x_norm"] <= 1.0
            assert isinstance(cand["near_edge"], bool)
            assert isinstance(cand["keypoints"], list)
            assert "bbox" in cand

        source_node.destroy_node()
        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_detector_pipeline_reset_on_source_loop():
    """Verify detector node resets ByteTrack and buffers on loop / session switch."""
    if not rclpy.ok():
        rclpy.init()
    try:
        detector_node = PersonDetectorNode()
        status_pub = detector_node.create_publisher(
            String, '/tracking/source_status', 10
        )

        # 1. Initial status with session-1
        initial_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-1",
            "loop_count": 0,
            "reset_event": False,
            "frame_count": 10,
            "fps": 15.0,
            "data_quality": "VALID_RGBD",
            "eof": False
        }
        msg = String(data=json.dumps(initial_status))
        status_pub.publish(msg)
        rclpy.spin_once(detector_node, timeout_sec=0.1)

        assert detector_node.current_session_id == "session-1"
        assert detector_node.last_loop_count == 0

        # Simulate tracker having a previous track ID
        detector_node.tracker.previous_track_ids = {1, 2}

        # 2. Source loop event: loop_count increments & reset_event is True
        loop_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-1",
            "loop_count": 1,
            "reset_event": True,
            "frame_count": 0,
            "fps": 15.0,
            "data_quality": "VALID_RGBD",
            "eof": False
        }
        status_pub.publish(String(data=json.dumps(loop_status)))
        rclpy.spin_once(detector_node, timeout_sec=0.1)

        assert detector_node.last_loop_count == 1
        # Tracker previous_track_ids should be cleared by _reset_pipeline()
        assert len(detector_node.tracker.previous_track_ids) == 0

        # 3. Source session switch: new source_session_id
        detector_node.tracker.previous_track_ids = {3}
        switch_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-2",
            "loop_count": 0,
            "reset_event": False,
            "frame_count": 0,
            "fps": 15.0,
            "data_quality": "VALID_RGBD",
            "eof": False
        }
        status_pub.publish(String(data=json.dumps(switch_status)))
        rclpy.spin_once(detector_node, timeout_sec=0.1)

        assert detector_node.current_session_id == "session-2"
        assert len(detector_node.tracker.previous_track_ids) == 0

        detector_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_two_loop_replay_boundary_reset_and_out_of_order_status():
    """Verify reset occurs before first frame of new loop, only once per boundary, tolerating out-of-order status."""
    if not rclpy.ok():
        rclpy.init()
    try:
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()
        status_pub = detector_node.create_publisher(String, '/tracking/source_status', 10)
        rgb_pub = detector_node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)

        bridge = detector_node.bridge
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)

        # --- LOOP 0 ---
        status_0 = {
            "source_mode": "video_replay",
            "source_session_id": "session-loop-0",
            "loop_count": 0,
            "reset_event": False,
            "frame_count": 0,
            "fps": 15.0,
            "data_quality": "RGB_ONLY",
            "eof": False,
            "stamp": {"sec": 100, "nanosec": 0}
        }
        status_pub.publish(String(data=json.dumps(status_0)))
        rclpy.spin_once(detector_node, timeout_sec=0.05)

        img_msg_0 = bridge.cv2_to_imgmsg(dummy_img, encoding="bgr8")
        img_msg_0.header.stamp.sec = 100
        img_msg_0.header.stamp.nanosec = 0
        img_msg_0.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(img_msg_0)
        rclpy.spin_once(detector_node, timeout_sec=0.05)
        rclpy.spin_once(test_sub, timeout_sec=0.05)

        assert detector_node.current_session_id == "session-loop-0"
        assert detector_node.last_loop_count == 0

        # Simulate active tracks established in loop 0
        detector_node.tracker.previous_track_ids = {101, 102}

        # --- BOUNDARY 0 -> 1 (Status arrives AFTER Image: Out-of-order test) ---
        # 1. Publish Image 1 (first frame of loop 1) BEFORE status arrives
        img_msg_1 = bridge.cv2_to_imgmsg(dummy_img, encoding="bgr8")
        img_msg_1.header.stamp.sec = 101
        img_msg_1.header.stamp.nanosec = 0
        img_msg_1.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(img_msg_1)
        for _ in range(5):
            rclpy.spin_once(detector_node, timeout_sec=0.01)
        # Image is queued in pending_rgb_frames waiting for matching status
        assert len(detector_node.pending_rgb_frames) == 1

        # 2. Now publish Status 1 with reset_event=True and new session
        status_1 = {
            "source_mode": "video_replay",
            "source_session_id": "session-loop-1",
            "loop_count": 1,
            "reset_event": True,
            "frame_count": 0,
            "fps": 15.0,
            "data_quality": "RGB_ONLY",
            "eof": False,
            "stamp": {"sec": 101, "nanosec": 0}
        }
        status_pub.publish(String(data=json.dumps(status_1)))
        # Spin to trigger _on_status which drains pending_rgb_frames and resets BEFORE processing frame 1
        rclpy.spin_once(detector_node, timeout_sec=0.05)
        rclpy.spin_once(test_sub, timeout_sec=0.05)

        assert detector_node.last_loop_count == 1
        assert detector_node.current_session_id == "session-loop-1"
        assert detector_node.last_reset_boundary == ("session-loop-1", 1)
        # Verify tracker was reset (previous tracks cleared)
        assert len(detector_node.tracker.previous_track_ids) == 0

        # --- BOUNDARY 1 -> 2 (Status arrives BEFORE Image, with duplicate status) ---
        status_2 = {
            "source_mode": "video_replay",
            "source_session_id": "session-loop-2",
            "loop_count": 2,
            "reset_event": True,
            "frame_count": 0,
            "fps": 15.0,
            "data_quality": "RGB_ONLY",
            "eof": False,
            "stamp": {"sec": 102, "nanosec": 0}
        }
        status_pub.publish(String(data=json.dumps(status_2)))
        for _ in range(5):
            rclpy.spin_once(detector_node, timeout_sec=0.02)

        assert detector_node.last_loop_count == 2
        assert detector_node.current_session_id == "session-loop-2"
        assert detector_node.last_reset_boundary == ("session-loop-2", 2)

        # Publish duplicate status_2; verify it does not trigger duplicate reset
        status_pub.publish(String(data=json.dumps(status_2)))
        rclpy.spin_once(detector_node, timeout_sec=0.05)
        assert detector_node.last_reset_boundary == ("session-loop-2", 2)

        # Publish Image 2
        img_msg_2 = bridge.cv2_to_imgmsg(dummy_img, encoding="bgr8")
        img_msg_2.header.stamp.sec = 102
        img_msg_2.header.stamp.nanosec = 0
        img_msg_2.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(img_msg_2)
        rclpy.spin_once(detector_node, timeout_sec=0.05)
        rclpy.spin_once(test_sub, timeout_sec=0.05)

        assert detector_node.current_session_id == "session-loop-2"

        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_observation_payload_with_non_empty_candidates():
    """Verify /tracking/observation schema with non-empty candidates and depth_valid=False when UNVERIFIED_ALIGNMENT."""
    if not rclpy.ok():
        rclpy.init()
    try:
        from person_tracking.multi_person_tracker import PersonDetection
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()
        status_pub = detector_node.create_publisher(String, '/tracking/source_status', 10)
        rgb_pub = detector_node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        depth_pub = detector_node.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)

        # Configure tracker mock candidate
        mock_cand = PersonDetection(
            bbox_xyxy=[150.0, 80.0, 320.0, 420.0],
            confidence=0.94,
            keypoints=[[200.0, 100.0, 0.95]] * 17,
            track_id=1,
            is_confirmed=True,
            is_ambiguous=False,
        )
        detector_node.tracker.mock_backend_candidates = [mock_cand]

        now_msg = detector_node.get_clock().now().to_msg()
        curr_sec = now_msg.sec
        curr_nsec = now_msg.nanosec

        # Publish status with UNVERIFIED_ALIGNMENT
        status_msg = {
            "source_mode": "video_replay",
            "source_session_id": "session-test-candidates",
            "loop_count": 0,
            "reset_event": False,
            "frame_count": 1,
            "fps": 15.0,
            "data_quality": "UNVERIFIED_ALIGNMENT",
            "eof": False,
            "stamp": {"sec": curr_sec, "nanosec": curr_nsec}
        }
        status_pub.publish(String(data=json.dumps(status_msg)))
        rclpy.spin_once(detector_node, timeout_sec=0.05)

        # Publish depth image (raw uint16 millimeters, e.g. 1500mm = 1.5m)
        depth_arr = np.full((480, 640), 1500, dtype=np.uint16)
        depth_msg = detector_node.bridge.cv2_to_imgmsg(depth_arr, encoding="16UC1")
        depth_msg.header.stamp.sec = curr_sec
        depth_msg.header.stamp.nanosec = curr_nsec
        depth_msg.header.frame_id = "camera_color_optical_frame"
        depth_pub.publish(depth_msg)
        rclpy.spin_once(detector_node, timeout_sec=0.05)

        # Publish RGB image
        rgb_arr = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb_msg = detector_node.bridge.cv2_to_imgmsg(rgb_arr, encoding="bgr8")
        rgb_msg.header.stamp.sec = curr_sec
        rgb_msg.header.stamp.nanosec = curr_nsec
        rgb_msg.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(rgb_msg)

        # Spin to receive on test_sub
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(detector_node, timeout_sec=0.05)
            rclpy.spin_once(test_sub, timeout_sec=0.05)
            if len(test_sub.received_obs) >= 1:
                break

        assert len(test_sub.received_obs) >= 1, "Failed to receive observation payload"
        payload = json.loads(test_sub.received_obs[-1].data)

        # 1. Non-empty candidates check
        assert len(payload["candidates"]) == 1
        cand = payload["candidates"][0]

        # 2. Strict Depth Validity: depth_valid MUST BE FALSE when UNVERIFIED_ALIGNMENT
        assert payload["data_quality"] == "UNVERIFIED_ALIGNMENT"
        assert cand["depth_valid"] is False, "CRITICAL ERROR: depth_valid must be False when UNVERIFIED_ALIGNMENT"
        assert cand["torso_depth_m"] is not None  # Raw measurement available for logging

        # 3. Candidate fields and aliases
        assert cand["track_id"] == 1
        assert cand["is_confirmed"] is True
        assert cand["confidence"] == 0.94
        assert cand["bbox_xyxy"] == [150.0, 80.0, 320.0, 420.0]
        assert cand["bbox"] == cand["bbox_xyxy"]
        assert payload["persons"] == payload["candidates"]
        assert payload["source_session_id"] == "session-test-candidates"

        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_status_timeout_drops_frame_no_fallback_latest_status():
    """Verify frame is dropped when status times out (> 0.04s) and NOT misassigned via latest_status."""
    if not rclpy.ok():
        rclpy.init()
    try:
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()
        status_pub = detector_node.create_publisher(String, '/tracking/source_status', 10)
        rgb_pub = detector_node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)

        # 1. Establish status mode with initial status
        init_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-initial",
            "loop_count": 0,
            "reset_event": False,
            "stamp": {"sec": 1, "nanosec": 0}
        }
        status_pub.publish(String(data=json.dumps(init_status)))
        for _ in range(5):
            rclpy.spin_once(detector_node, timeout_sec=0.01)

        assert detector_node.has_seen_status is True
        assert detector_node.current_session_id == "session-initial"

        # 2. Publish RGB frame with stamp (100, 0) WITHOUT status
        rgb_arr = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb_msg = detector_node.bridge.cv2_to_imgmsg(rgb_arr, encoding="bgr8")
        rgb_msg.header.stamp.sec = 100
        rgb_msg.header.stamp.nanosec = 0
        rgb_msg.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(rgb_msg)

        # 3. Spin past timeout (0.50s > 0.30s timeout) so drain timer drops it
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < 0.50:
            rclpy.spin_once(detector_node, timeout_sec=0.02)
            rclpy.spin_once(test_sub, timeout_sec=0.02)
        detector_node._drain_pending_frames()

        # Frame MUST have been dropped from pending_rgb_frames and NOT published!
        assert len(detector_node.pending_rgb_frames) == 0
        assert len(test_sub.received_obs) == 0, "ERROR: Frame was published despite status timeout!"

        # 4. Now send a late status with stamp (100, 0)
        late_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-late",
            "loop_count": 0,
            "reset_event": False,
            "stamp": {"sec": 100, "nanosec": 0}
        }
        status_pub.publish(String(data=json.dumps(late_status)))
        for _ in range(5):
            rclpy.spin_once(detector_node, timeout_sec=0.01)
            rclpy.spin_once(test_sub, timeout_sec=0.01)

        # Still no observation because frame was already dropped at timeout
        assert len(test_sub.received_obs) == 0

        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_stale_status_from_old_loop_is_ignored():
    """Verify status arriving late from an older loop or past session is ignored and does not reset tracker."""
    if not rclpy.ok():
        rclpy.init()
    try:
        detector_node = PersonDetectorNode()
        detector_node.has_seen_status = True
        detector_node.last_loop_count = 5
        detector_node.current_session_id = "session-current"
        detector_node.past_session_ids = {"session-old-1", "session-old-2"}

        # 1. Send stale status from past session
        stale_session_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-old-1",
            "loop_count": 10,
            "reset_event": True,
            "stamp": {"sec": 50, "nanosec": 0}
        }
        detector_node._on_status(String(data=json.dumps(stale_session_status)))
        assert detector_node.current_session_id == "session-current"
        assert detector_node.last_loop_count == 5

        # 2. Send stale status from older loop of current session
        stale_loop_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-current",
            "loop_count": 3,
            "reset_event": True,
            "stamp": {"sec": 51, "nanosec": 0}
        }
        detector_node._on_status(String(data=json.dumps(stale_loop_status)))
        assert detector_node.current_session_id == "session-current"
        assert detector_node.last_loop_count == 5

        detector_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_first_frame_of_new_loop_with_detection_runs_cleanly():
    """Verify first frame of a new loop resets pipeline BEFORE inference and publishes observation."""
    if not rclpy.ok():
        rclpy.init()
    try:
        from person_tracking.multi_person_tracker import PersonDetection
        detector_node = PersonDetectorNode()
        test_sub = PerceptionObserverSubscriber()
        status_pub = detector_node.create_publisher(String, '/tracking/source_status', 10)
        rgb_pub = detector_node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)

        mock_cand = PersonDetection(
            bbox_xyxy=[100.0, 50.0, 250.0, 400.0],
            confidence=0.91,
            keypoints=[[150.0, 100.0, 0.9]] * 17,
            track_id=1,
            is_confirmed=True,
        )
        detector_node.tracker.mock_backend_candidates = [mock_cand]

        now_msg = detector_node.get_clock().now().to_msg()
        curr_sec = now_msg.sec
        curr_nsec = now_msg.nanosec

        new_loop_status = {
            "source_mode": "video_replay",
            "source_session_id": "session-new-loop-7",
            "loop_count": 7,
            "reset_event": True,
            "stamp": {"sec": curr_sec, "nanosec": curr_nsec}
        }
        status_pub.publish(String(data=json.dumps(new_loop_status)))
        rclpy.spin_once(detector_node, timeout_sec=0.05)

        rgb_arr = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb_msg = detector_node.bridge.cv2_to_imgmsg(rgb_arr, encoding="bgr8")
        rgb_msg.header.stamp.sec = curr_sec
        rgb_msg.header.stamp.nanosec = curr_nsec
        rgb_msg.header.frame_id = "camera_color_optical_frame"
        rgb_pub.publish(rgb_msg)

        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(detector_node, timeout_sec=0.02)
            rclpy.spin_once(test_sub, timeout_sec=0.02)
            if len(test_sub.received_obs) >= 1:
                break

        assert len(test_sub.received_obs) >= 1, "Failed to receive observation for frame 0 of new loop"
        payload = json.loads(test_sub.received_obs[-1].data)

        assert payload["source_session_id"] == "session-new-loop-7"
        assert len(payload["candidates"]) == 1
        assert payload["candidates"][0]["track_id"] == 1
        assert detector_node.last_loop_count == 7

        detector_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
