#!/usr/bin/env python3
"""ROS 2 node that runs multi-person pose estimation and tracking.

Phase 3 Perception Node:
- Subscribes to RGB, Depth, CameraInfo topics.
- Synchronizes frames via TopicSynchronizer.
- Runs MultiPersonTracker (YOLO Pose + ByteTrack).
- Publishes /tracking/observation (JSON string) and /tracking/debug_image (Image).
- Listens to /tracking/source_status to reset tracker on loop/source changes.
- STRICTLY ZERO motion output: Does NOT publish /cmd_vel or call Nav2 actions.
"""

import json
import os
import sys
import time
import uuid

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Inject .venv_yolo site-packages if available in Dev Container
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from person_tracking.multi_person_tracker import MultiPersonTracker, MultiPersonObservation
from person_tracking.source_adapters import LiveAstraAdapter, TopicSynchronizer


class PersonDetectorNode(Node):
    """Perception node for multi-person pose extraction and tracking."""

    def __init__(self):
        super().__init__('person_detector_node')

        # Declare parameters
        self.declare_parameter('model_path', '/home/ws/yolo26n-pose.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('sync_slop_sec', 0.06)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('status_topic', '/tracking/source_status')
        self.declare_parameter('observation_topic', '/tracking/observation')
        self.declare_parameter('debug_image_topic', '/tracking/debug_image')

        model_path = str(self.get_parameter('model_path').value)
        device = str(self.get_parameter('device').value)
        conf_thresh = float(self.get_parameter('conf_threshold').value)
        iou_thresh = float(self.get_parameter('iou_threshold').value)
        sync_slop = float(self.get_parameter('sync_slop_sec').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize Adapter and Synchronizer
        self.adapter = LiveAstraAdapter(sync_slop_sec=sync_slop, require_hardware_alignment_proof=True)
        self.synchronizer = TopicSynchronizer(adapter=self.adapter)

        # Initialize MultiPersonTracker
        try:
            self.tracker = MultiPersonTracker(
                model_path=model_path,
                device=device,
                conf_threshold=conf_thresh,
                iou_threshold=iou_thresh,
            )
            self.get_logger().info(
                f"MultiPersonTracker loaded model '{model_path}' on device '{self.tracker.device}'."
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize MultiPersonTracker: {exc}")
            raise

        # Setup publishers
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        obs_topic_name = str(self.get_parameter('observation_topic').value)
        self.obs_pub = self.create_publisher(String, obs_topic_name, reliable_qos)

        if self.publish_debug_image:
            debug_topic_name = str(self.get_parameter('debug_image_topic').value)
            self.debug_pub = self.create_publisher(Image, debug_topic_name, sensor_qos)
        else:
            self.debug_pub = None

        # Setup subscribers
        rgb_topic = str(self.get_parameter('rgb_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('info_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)

        self.sub_rgb = self.create_subscription(Image, rgb_topic, self._on_rgb, sensor_qos)
        self.sub_depth = self.create_subscription(Image, depth_topic, self._on_depth, sensor_qos)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self._on_info, sensor_qos)
        self.sub_status = self.create_subscription(String, status_topic, self._on_status, reliable_qos)

        self.last_frame_count = 0
        self.processed_frames = 0
        self.current_session_id = str(uuid.uuid4())
        self.last_loop_count = 0
        self.has_seen_status = False
        self.status_by_stamp = {}
        self.latest_status = None
        self.last_reset_boundary = None
        self.last_frame_timestamp = -1.0
        self.pending_rgb_frames = []
        self.get_logger().info("PersonDetectorNode initialized and listening for perception streams.")

    def _on_status(self, msg: String):
        try:
            status_data = json.loads(msg.data)
            self.has_seen_status = True
            self.latest_status = status_data

            stamp_dict = status_data.get('stamp', {})
            sec = stamp_dict.get('sec', 0)
            nanosec = stamp_dict.get('nanosec', 0)
            self.status_by_stamp[(sec, nanosec)] = status_data

            if len(self.status_by_stamp) > 100:
                oldest_key = next(iter(self.status_by_stamp))
                self.status_by_stamp.pop(oldest_key, None)

            session_id = status_data.get('source_session_id', '')
            loop_count = status_data.get('loop_count', 0)
            reset_event = status_data.get('reset_event', False)
            boundary_key = (session_id, loop_count)

            # If status explicitly indicates a boundary reset and has not been executed yet
            if reset_event or loop_count > self.last_loop_count or (self.current_session_id and session_id != self.current_session_id):
                if boundary_key != self.last_reset_boundary:
                    self.get_logger().info(
                        f"Resetting perception pipeline on status boundary (session: {session_id}, loop: {loop_count})."
                    )
                    self._reset_pipeline()
                    self.last_reset_boundary = boundary_key
                    self.current_session_id = session_id
                    self.last_loop_count = loop_count

            self._drain_pending_frames()
        except Exception as exc:
            self.get_logger().warn(f"Error parsing source status: {exc}")

    def _reset_pipeline(self):
        # Reset tracker, depth buffers, and clock history cleanly
        self.tracker.reset()
        self.synchronizer.reset()
        self.last_frame_timestamp = -1.0

    def _on_info(self, msg: CameraInfo):
        self.synchronizer.set_camera_info(msg)

    def _on_depth(self, msg: Image):
        try:
            depth_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.synchronizer.add_depth(
                depth_img=depth_np,
                stamp=stamp,
                frame_id=msg.header.frame_id,
                encoding=msg.encoding,
            )
        except Exception as exc:
            self.get_logger().warn(f"Error decoding depth message: {exc}")

    def _on_rgb(self, msg: Image):
        frame_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        # If we have seen status before, but status for this frame hasn't arrived yet, buffer briefly
        if self.has_seen_status and frame_stamp not in self.status_by_stamp:
            now_mono = time.monotonic()
            self.pending_rgb_frames.append((now_mono, msg))
            self._drain_pending_frames()
            return

        self._process_rgb_message(msg)

    def _drain_pending_frames(self):
        now_mono = time.monotonic()
        remaining = []
        for queue_time, frame_msg in self.pending_rgb_frames:
            stamp = (frame_msg.header.stamp.sec, frame_msg.header.stamp.nanosec)
            # If status arrived OR timeout expired (> 0.04s)
            if stamp in self.status_by_stamp or (now_mono - queue_time) > 0.04:
                self._process_rgb_message(frame_msg)
            else:
                remaining.append((queue_time, frame_msg))
        self.pending_rgb_frames = remaining

    def _process_rgb_message(self, msg: Image):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        status_data = self.status_by_stamp.pop(stamp, None) or self.latest_status

        if status_data:
            session_id = status_data.get('source_session_id', '')
            loop_count = status_data.get('loop_count', 0)
            reset_event = status_data.get('reset_event', False)
            boundary_key = (session_id, loop_count)

            # Check if this frame is at a reset boundary and has NOT been reset yet
            if reset_event or loop_count > self.last_loop_count or (self.current_session_id and session_id != self.current_session_id):
                if boundary_key != self.last_reset_boundary:
                    self.get_logger().info(
                        f"Resetting perception pipeline BEFORE frame inference (session: {session_id}, loop: {loop_count})."
                    )
                    self._reset_pipeline()
                    self.last_reset_boundary = boundary_key
                    self.current_session_id = session_id
                    self.last_loop_count = loop_count
            elif session_id:
                self.current_session_id = session_id
        else:
            # Fallback for live camera / bag without status:
            t_curr = stamp[0] + stamp[1] * 1e-9
            if self.last_frame_timestamp > 0 and t_curr < self.last_frame_timestamp - 0.5:
                self.get_logger().info(f"Detected timestamp jump ({t_curr:.3f} < {self.last_frame_timestamp:.3f}). Resetting pipeline.")
                self._reset_pipeline()
                self.current_session_id = str(uuid.uuid4())
            self.last_frame_timestamp = t_curr

        try:
            rgb_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Error decoding RGB message: {exc}")
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Synchronize and normalize observation frame
        obs_frame = self.synchronizer.process_rgb(
            rgb_img=rgb_np,
            stamp=stamp,
            frame_id=msg.header.frame_id,
            current_time_sec=now_sec,
        )
        if self.current_session_id:
            obs_frame.source_session_id = self.current_session_id

        if obs_frame.data_quality == 'STALE':
            self.get_logger().warn(f"Dropped stale frame: {obs_frame.error_reason}")
            return

        # Run multi-person detection and tracking
        try:
            multi_obs = self.tracker.process_frame(obs_frame)
            self.processed_frames += 1
        except Exception as exc:
            self.get_logger().error(f"Error running tracker inference: {exc}")
            return

        # Publish observation payload
        self._publish_observation(multi_obs)

        # Publish debug overlay if enabled
        if self.publish_debug_image and self.debug_pub is not None:
            self._publish_debug_image(rgb_np, multi_obs, msg.header)

    def _publish_observation(self, obs: MultiPersonObservation):
        candidates_list = [
            {
                "track_id": p.track_id,
                "is_confirmed": p.is_confirmed,
                "is_ambiguous": p.is_ambiguous,
                "confidence": p.confidence,
                "bbox_xyxy": p.bbox_xyxy,
                "torso_depth_m": p.torso_depth_m,
                "depth_valid": p.depth_valid,
                "center_offset_x_norm": p.center_offset_x_norm,
                "near_edge": p.near_edge,
                "keypoints": p.keypoints,
                # Backward-compatibility alias
                "bbox": p.bbox_xyxy,
            }
            for p in obs.candidates
        ]

        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": obs.stamp_sec, "nanosec": obs.stamp_nanosec},
            "frame_id": obs.frame_id,
            "receive_monotonic_time": obs.receive_monotonic_time,
            "source_mode": obs.source_mode,
            "source_session_id": obs.source_session_id,
            "data_quality": obs.data_quality,
            "processing_time_ms": obs.processing_time_ms,
            "device": obs.device,
            "active_track_ids": obs.active_track_ids,
            "lost_track_ids": obs.lost_track_ids,
            "candidates": candidates_list,
            # Backward-compatibility aliases
            "receive_monotonic": obs.receive_monotonic_time,
            "persons": candidates_list,
        }
        msg = String(data=json.dumps(payload))
        self.obs_pub.publish(msg)

    def _publish_debug_image(self, bgr_img: np.ndarray, obs: MultiPersonObservation, header):
        try:
            debug_img = self.tracker.draw_debug_overlay(bgr_img, obs)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f"Error publishing debug image: {exc}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PersonDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Unhandled exception in PersonDetectorNode: {exc}", file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
