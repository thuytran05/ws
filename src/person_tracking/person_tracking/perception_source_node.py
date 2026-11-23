import uuid
#!/usr/bin/env python3
"""ROS 2 node that publishes normalized camera streams for offline testing.

Supports:
- Video replay from file (MP4/AVI).
- Synthetic fixture playback (empty scene, person scene, corrupted depth).
- Live Astra monitoring/validation.

DOES NOT publish velocity commands or interact with robot actuators.
"""

import json
import os
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from person_tracking.source_adapters import (
    SourceMode,
    DataQuality,
    ObservationFrame,
    VideoReplayAdapter,
    SyntheticFixtureGenerator,
    create_simulated_camera_info,
)


class PerceptionSourceNode(Node):
    """Feeds perception inputs from video files, synthetic fixtures, or live streams."""

    def __init__(self):
        super().__init__('perception_source_node')

        # Declare parameters
        self.declare_parameter('source_mode', SourceMode.SYNTHETIC_FIXTURE.value)
        self.declare_parameter('video_path', '')
        self.declare_parameter('synthetic_case', 'person_valid_rgbd')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('retime_stamps', True)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('status_topic', '/tracking/source_status')

        self.source_mode = str(self.get_parameter('source_mode').value)
        self.video_path = str(self.get_parameter('video_path').value)
        self.synthetic_case = str(self.get_parameter('synthetic_case').value)
        self.fps = float(self.get_parameter('fps').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.retime_stamps = bool(self.get_parameter('retime_stamps').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.is_eof = False
        self.video_adapter = None
        self.source_session_id = str(uuid.uuid4())
        self.loop_count = 0

        # Setup publishers
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.rgb_pub = self.create_publisher(Image, str(self.get_parameter('rgb_topic').value), sensor_qos)
        self.depth_pub = self.create_publisher(Image, str(self.get_parameter('depth_topic').value), sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, str(self.get_parameter('info_topic').value), sensor_qos)
        self.status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), reliable_qos)

        # Initialize source
        self._init_source()

        # Timer loop for publishing
        period = 1.0 / max(1.0, self.fps)
        self.timer = self.create_timer(period, self._publish_step)
        self.get_logger().info(
            f"PerceptionSourceNode initialized in mode '{self.source_mode}' at {self.fps} FPS."
        )

    def _init_source(self):
        if self.source_mode == SourceMode.VIDEO_REPLAY.value:
            if not self.video_path or not os.path.isfile(self.video_path):
                self.get_logger().error(f"Invalid video_path: '{self.video_path}'. Falling back to synthetic.")
                self.source_mode = SourceMode.SYNTHETIC_FIXTURE.value
            else:
                try:
                    self.video_adapter = VideoReplayAdapter(
                        video_path=self.video_path,
                        fps=self.fps,
                        loop=self.loop,
                        retime_stamps=self.retime_stamps,
                        frame_id=self.frame_id,
                    )
                    self.get_logger().info(f"VideoReplayAdapter loaded: {self.video_path} ({self.video_adapter.total_frames} frames)")
                except Exception as exc:
                    self.get_logger().error(f"Failed to load video: {exc}. Falling back to synthetic.")
                    self.source_mode = SourceMode.SYNTHETIC_FIXTURE.value

    def _publish_step(self):
        if self.is_eof:
            return

        now_ros = self.get_clock().now()
        now_msg = now_ros.to_msg()
        current_time_tuple = (now_msg.sec, now_msg.nanosec)

        obs_frame = None
        reset_event = False

        if self.source_mode == SourceMode.VIDEO_REPLAY.value and self.video_adapter is not None:
            obs_frame = self.video_adapter.read_frame(current_ros_time=current_time_tuple)
            if obs_frame is None and self.video_adapter.eof:
                self.is_eof = True
                self._publish_status(eof=True, quality="EOF", reason="Video replay reached EOF.")
                self.get_logger().info("Video replay reached EOF.")
                return

            if getattr(self.video_adapter, 'just_looped', False):
                self.loop_count = self.video_adapter.loop_count
                self.video_adapter.just_looped = False
                reset_event = True
                self.source_session_id = str(uuid.uuid4())
                self.get_logger().info(
                    f"Video replay looped (loop_count={self.loop_count}, new session_id={self.source_session_id})."
                )

        elif self.source_mode == SourceMode.SYNTHETIC_FIXTURE.value:
            obs_frame = self._generate_synthetic_frame(current_time_tuple)

        elif self.source_mode == SourceMode.LIVE_ASTRA.value:
            # In live mode without hardware connected, status indicates pending
            self._publish_status(eof=False, quality=DataQuality.UNVERIFIED_ALIGNMENT.value,
                                 reason="Live mode configured; waiting for Astra camera device.")
            return

        if obs_frame is None:
            return

        if obs_frame is not None:
            obs_frame.source_session_id = self.source_session_id
        self.frame_count += 1

        # 1. Publish Source Status FIRST (so subscribers receive boundary metadata before image)
        self._publish_status(
            eof=False,
            quality=obs_frame.data_quality,
            reason=obs_frame.error_reason,
            stamp_sec=obs_frame.stamp_sec,
            stamp_nanosec=obs_frame.stamp_nanosec,
            reset_event=reset_event,
        )

        # 2. Publish RGB Image
        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(obs_frame.rgb, encoding='bgr8')
            rgb_msg.header.stamp.sec = obs_frame.stamp_sec
            rgb_msg.header.stamp.nanosec = obs_frame.stamp_nanosec
            rgb_msg.header.frame_id = obs_frame.frame_id
            self.rgb_pub.publish(rgb_msg)
        except Exception as exc:
            self.get_logger().error(f"Error publishing RGB image: {exc}")

        # 3. Publish Depth Image (if present)
        if obs_frame.depth_raw is not None:
            try:
                depth_msg = self.bridge.cv2_to_imgmsg(obs_frame.depth_raw, encoding='16UC1')
                depth_msg.header.stamp.sec = obs_frame.stamp_sec
                depth_msg.header.stamp.nanosec = obs_frame.stamp_nanosec
                depth_msg.header.frame_id = obs_frame.frame_id
                self.depth_pub.publish(depth_msg)
            except Exception as exc:
                self.get_logger().error(f"Error publishing depth image: {exc}")

        # 4. Publish CameraInfo
        if obs_frame.camera_info is not None:
            obs_frame.camera_info.header.stamp.sec = obs_frame.stamp_sec
            obs_frame.camera_info.header.stamp.nanosec = obs_frame.stamp_nanosec
            obs_frame.camera_info.header.frame_id = obs_frame.frame_id
            self.info_pub.publish(obs_frame.camera_info)

    def _generate_synthetic_frame(self, stamp: tuple) -> ObservationFrame:
        if self.synthetic_case == 'empty_scene':
            return SyntheticFixtureGenerator.generate_empty_scene(
                frame_id=self.frame_id, stamp=stamp
            )
        elif self.synthetic_case == 'corrupted_depth':
            return SyntheticFixtureGenerator.generate_corrupted_depth_scene(
                corruption_type='nan_and_zeros', frame_id=self.frame_id, stamp=stamp
            )
        else:  # default 'person_valid_rgbd'
            # Slightly move the person horizontally to simulate dynamic motion
            offset = int(40 * np.sin(self.frame_count * 0.1))
            bbox = (220 + offset, 80, 420 + offset, 420)
            return SyntheticFixtureGenerator.generate_person_scene(
                person_bbox=bbox, frame_id=self.frame_id, stamp=stamp
            )

    def _publish_status(
        self,
        eof: bool,
        quality: str,
        reason: str,
        stamp_sec: int = 0,
        stamp_nanosec: int = 0,
        reset_event: bool = False,
    ):
        status_payload = {
            "source_mode": self.source_mode,
            "source_session_id": self.source_session_id,
            "loop_count": self.loop_count,
            "reset_event": reset_event,
            "frame_count": self.frame_count,
            "fps": self.fps,
            "eof": eof,
            "data_quality": quality,
            "reason": reason,
            "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec},
            "timestamp": time.time(),
        }
        msg = String(data=json.dumps(status_payload))
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.video_adapter is not None:
            self.video_adapter.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
