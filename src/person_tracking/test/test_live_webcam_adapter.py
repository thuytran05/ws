"""Unit & ROS runtime tests for LiveWebcamAdapter and live_webcam source mode."""

import json
import time

import cv2
import numpy as np
import pytest

from person_tracking.source_adapters import DataQuality, LiveWebcamAdapter, SourceMode


class MockVideoCapture:
    """Deterministic mock OpenCV VideoCapture for automated webcam tests."""

    def __init__(self, index: int, open_ok: bool = True, fail_after: int = -1, width: int = 640, height: int = 480):
        self.index = index
        self._opened = open_ok
        self.fail_after = fail_after
        self.width = width
        self.height = height
        self.reads = 0
        self.released = False
        self.props = {
            cv2.CAP_PROP_FRAME_WIDTH: float(width),
            cv2.CAP_PROP_FRAME_HEIGHT: float(height),
            cv2.CAP_PROP_FPS: 30.0,
        }

    def isOpened(self) -> bool:
        return self._opened and not self.released

    def set(self, prop_id: int, value: float) -> bool:
        self.props[prop_id] = value
        return True

    def get(self, prop_id: int) -> float:
        return self.props.get(prop_id, 0.0)

    def read(self):
        if not self.isOpened():
            return False, None
        if self.fail_after >= 0 and self.reads >= self.fail_after:
            return False, None
        self.reads += 1
        img = np.full((int(self.props[cv2.CAP_PROP_FRAME_HEIGHT]), int(self.props[cv2.CAP_PROP_FRAME_WIDTH]), 3), 120, dtype=np.uint8)
        return True, img

    def release(self):
        self.released = True
        self._opened = False


def test_live_webcam_adapter_open_read_rgb_only_and_close():
    mock_cap = MockVideoCapture(0, open_ok=True, width=640, height=480)
    adapter = LiveWebcamAdapter(
        camera_index=0,
        width=640,
        height=480,
        fps=30.0,
        cap_factory=lambda idx: mock_cap,
    )

    frame = adapter.read_frame(current_ros_time=(1700000100, 250000000))
    assert frame is not None
    assert frame.rgb.shape == (480, 640, 3)
    assert frame.depth is None
    assert frame.depth_raw is None
    assert frame.camera_info is None
    assert not frame.has_depth
    assert frame.source_mode == SourceMode.LIVE_WEBCAM.value
    assert frame.data_quality == DataQuality.RGB_ONLY.value
    assert frame.stamp_sec == 1700000100
    assert frame.stamp_nanosec == 250000000

    adapter.close()
    assert mock_cap.released is True
    assert adapter.read_frame() is None


def test_live_webcam_adapter_open_failure_raises_cleanly():
    mock_cap = MockVideoCapture(9, open_ok=False)
    with pytest.raises(RuntimeError):
        LiveWebcamAdapter(camera_index=9, cap_factory=lambda idx: mock_cap)
    assert mock_cap.released is True


def test_live_webcam_adapter_read_failure_does_not_crash():
    mock_cap = MockVideoCapture(0, open_ok=True, fail_after=2)
    adapter = LiveWebcamAdapter(camera_index=0, cap_factory=lambda idx: mock_cap)

    assert adapter.read_frame((1700000100, 1000)) is not None
    assert adapter.read_frame((1700000100, 2000)) is not None
    # Simulated camera disconnect / read failure
    failed = adapter.read_frame((1700000100, 3000))
    assert failed is None
    assert adapter.consecutive_failures == 1
    assert adapter.total_failures == 1
    adapter.close()


def test_perception_source_node_live_webcam_failure_no_crash_no_cmd_vel(monkeypatch):
    """Verify PerceptionSourceNode handles webcam open/read failure safely without cmd_vel or fake depth."""
    import rclpy
    from person_tracking import perception_source_node as psn_mod

    # Patch LiveWebcamAdapter inside perception_source_node module
    mock_cap = MockVideoCapture(0, open_ok=True, fail_after=1)
    monkeypatch.setattr(
        psn_mod,
        "LiveWebcamAdapter",
        lambda **kwargs: LiveWebcamAdapter(cap_factory=lambda idx: mock_cap, **kwargs),
    )

    rclpy.init()
    try:
        node = psn_mod.PerceptionSourceNode()
        # Override to live_webcam & re-init cleanly
        node.source_mode = SourceMode.LIVE_WEBCAM.value
        node.webcam_adapter = LiveWebcamAdapter(cap_factory=lambda idx: mock_cap)

        # Should not publish /cmd_vel
        pub_topics = [p.topic_name for p in node.publishers]
        assert "/cmd_vel" not in pub_topics

        # Step 1: valid read
        node._publish_step()
        assert node.frame_count == 1
        # Step 2: read failure -> no crash, frame_count unchanged
        node._publish_step()
        assert node.frame_count == 1

        node.destroy_node()
        assert mock_cap.released is True
    finally:
        if rclpy.ok():
            rclpy.shutdown()
