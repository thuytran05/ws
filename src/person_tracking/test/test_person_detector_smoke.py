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
