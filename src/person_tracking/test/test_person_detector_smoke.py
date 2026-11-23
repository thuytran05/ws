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
