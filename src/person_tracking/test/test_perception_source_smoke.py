"""ROS 2 smoke test for perception_source_node in person_tracking."""

import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from person_tracking.perception_source_node import PerceptionSourceNode


class PerceptionSourceTestSubscriber(Node):
    def __init__(self):
        super().__init__('perception_source_test_sub')
        self.received_rgb = []
        self.received_depth = []
        self.received_info = []
        self.received_status = []

        self.sub_rgb = self.create_subscription(
            Image, '/camera/color/image_raw', self._on_rgb, qos_profile_sensor_data
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, qos_profile_sensor_data
        )
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, qos_profile_sensor_data
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_status = self.create_subscription(
            String, '/tracking/source_status', self._on_status, reliable_qos
        )

    def _on_rgb(self, msg: Image):
        self.received_rgb.append(msg)

    def _on_depth(self, msg: Image):
        self.received_depth.append(msg)

    def _on_info(self, msg: CameraInfo):
        self.received_info.append(msg)

    def _on_status(self, msg: String):
        self.received_status.append(msg)


def test_perception_source_node_smoke():
    rclpy.init()
    try:
        source_node = PerceptionSourceNode()
        test_sub = PerceptionSourceTestSubscriber()

        # Spin both nodes for ~1.0 second (expect ~15 frames at 15 FPS)
        start_time = time.time()
        timeout = 5.0
        while time.time() - start_time < timeout:
            rclpy.spin_once(source_node, timeout_sec=0.05)
            rclpy.spin_once(test_sub, timeout_sec=0.05)
            if len(test_sub.received_rgb) >= 5 and len(test_sub.received_status) >= 5:
                break

        # Verification 1: Received messages
        assert len(test_sub.received_rgb) >= 5, f"Expected >= 5 RGB frames, got {len(test_sub.received_rgb)}"
        assert len(test_sub.received_depth) >= 5, f"Expected >= 5 Depth frames, got {len(test_sub.received_depth)}"
        assert len(test_sub.received_info) >= 5, f"Expected >= 5 CameraInfo msgs, got {len(test_sub.received_info)}"
        assert len(test_sub.received_status) >= 5, f"Expected >= 5 Status msgs, got {len(test_sub.received_status)}"

        # Verification 2: Message contents
        first_rgb = test_sub.received_rgb[0]
        assert first_rgb.width == 640
        assert first_rgb.height == 480
        assert first_rgb.encoding == "bgr8"
        assert first_rgb.header.frame_id == "camera_color_optical_frame"

        first_depth = test_sub.received_depth[0]
        assert first_depth.width == 640
        assert first_depth.height == 480
        assert first_depth.encoding == "16UC1"
        assert first_depth.header.frame_id == "camera_color_optical_frame"

        first_info = test_sub.received_info[0]
        assert first_info.width == 640
        assert first_info.height == 480
        assert first_info.k[0] > 0  # fx > 0

        first_status = json.loads(test_sub.received_status[0].data)
        assert first_status["source_mode"] == "synthetic_fixture"
        assert first_status["data_quality"] == "VALID_RGBD"
        assert first_status["eof"] is False

        # Verification 3: Safety Audit - NO cmd_vel publisher!
        # Query all topics published by source_node
        node_publishers = source_node.get_publisher_names_and_types_by_node(
            source_node.get_name(), source_node.get_namespace()
        )
        published_topic_names = [topic for topic, _ in node_publishers]
        assert "/cmd_vel" not in published_topic_names, "CRITICAL SAFETY VIOLATION: Source node must not publish to /cmd_vel!"
        assert "cmd_vel" not in published_topic_names

        source_node.destroy_node()
        test_sub.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
