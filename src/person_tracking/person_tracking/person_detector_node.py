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

from collections import deque
import json
import os
import sys
import threading
import time
import uuid
from typing import Optional

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Inject .venv_yolo site-packages if available in Dev Container
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from person_tracking.debug_visualization import DebugOverlayContext
from person_tracking.multi_person_tracker import MultiPersonTracker, MultiPersonObservation
from person_tracking.source_adapters import DataQuality, LiveAstraAdapter, SourceMode, TopicSynchronizer


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
        self.declare_parameter('use_synthetic_mount', False)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('status_topic', '/tracking/source_status')
        self.declare_parameter('observation_topic', '/tracking/observation')
        self.declare_parameter('debug_image_topic', '/tracking/debug_image')
        self.declare_parameter('session_state_topic', '/tracking/session_state')
        self.declare_parameter('debug_lock_state_topic', '/tracking/debug_lock_state')
        self.declare_parameter('session_freshness_ttl_sec', 1.0)
        self.declare_parameter('image_qos_reliability', 'best_effort')
        self.declare_parameter('image_qos_depth', 5)
        self.declare_parameter('info_qos_reliability', 'best_effort')
        self.declare_parameter('info_qos_depth', 5)
        self.declare_parameter('prioritize_latest_frame', False)
        self.declare_parameter('readiness_min_frames', 1)
        self.declare_parameter('require_hardware_alignment_proof', True)

        model_path = str(self.get_parameter('model_path').value)
        device = str(self.get_parameter('device').value)
        conf_thresh = float(self.get_parameter('conf_threshold').value)
        iou_thresh = float(self.get_parameter('iou_threshold').value)
        sync_slop = float(self.get_parameter('sync_slop_sec').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        use_synthetic_mount = bool(self.get_parameter('use_synthetic_mount').value)
        self.prioritize_latest_frame = bool(self.get_parameter('prioritize_latest_frame').value)
        self.readiness_min_frames = max(1, int(self.get_parameter('readiness_min_frames').value))
        require_align_proof = bool(self.get_parameter('require_hardware_alignment_proof').value)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize Adapter and Synchronizer
        self.adapter = LiveAstraAdapter(
            sync_slop_sec=sync_slop,
            require_hardware_alignment_proof=require_align_proof,
        )
        self.synchronizer = TopicSynchronizer(adapter=self.adapter)

        # Initialize MultiPersonTracker
        try:
            self.tracker = MultiPersonTracker(
                model_path=model_path,
                device=device,
                conf_threshold=conf_thresh,
                iou_threshold=iou_thresh,
                use_synthetic_mount=use_synthetic_mount,
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
            debug_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=2,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.debug_pub = self.create_publisher(Image, debug_topic_name, debug_qos)
        else:
            self.debug_pub = None

        # Setup subscribers
        rgb_topic = str(self.get_parameter('rgb_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('info_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        session_topic = str(self.get_parameter('session_state_topic').value)
        debug_lock_topic = str(self.get_parameter('debug_lock_state_topic').value)
        self.session_freshness_ttl_sec = float(self.get_parameter('session_freshness_ttl_sec').value)
        img_rel = str(self.get_parameter('image_qos_reliability').value).strip().lower()
        img_depth = max(1, int(self.get_parameter('image_qos_depth').value))
        info_rel = str(self.get_parameter('info_qos_reliability').value).strip().lower()
        info_depth = max(1, int(self.get_parameter('info_qos_depth').value))

        img_sub_qos = self._build_stream_qos(img_rel, img_depth)
        info_sub_qos = self._build_stream_qos(info_rel, info_depth)

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub_rgb = self.create_subscription(Image, rgb_topic, self._on_rgb, img_sub_qos)
        self.sub_depth = self.create_subscription(Image, depth_topic, self._on_depth, img_sub_qos)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self._on_info, info_sub_qos)
        self.sub_status = self.create_subscription(String, status_topic, self._on_status, reliable_qos)
        self.sub_session_state = self.create_subscription(
            String, session_topic, self._on_session_state, transient_qos
        )
        self.sub_debug_lock = self.create_subscription(
            String, debug_lock_topic, self._on_debug_lock_state, reliable_qos
        )

        self.last_frame_count = 0
        self.processed_frames = 0
        self.current_session_id = str(uuid.uuid4())
        self.last_loop_count = 0
        self.has_seen_status = False
        self.past_session_ids = set()
        self.status_by_stamp = {}
        self.latest_status = None
        self.last_reset_boundary = None
        self.last_frame_timestamp = -1.0
        self.pending_rgb_frames = []
        self.max_pending_age_sec = 0.30

        # Read-only telemetry for debug visualization (never alters tracking/lock decisions)
        self.latest_session_state: Optional[dict] = None
        self.last_session_state_mono_time: float = 0.0
        self.latest_debug_lock_state: Optional[dict] = None
        self.last_debug_lock_mono_time: float = 0.0
        self._cam_fps_from_status: Optional[float] = None
        self._rgb_times = deque(maxlen=30)
        self._obs_times = deque(maxlen=30)
        self._dbg_times = deque(maxlen=30)

        # Camera readiness state
        self._consecutive_ready_frames = 0
        self._last_camera_ready = False
        self._last_readiness_reason = "CAMERA_WARMING_UP"

        # Optional 1-slot latest-frame inference worker (never accumulates old frames when YOLO is slow)
        self._infer_lock = threading.Lock()
        self._tracker_exec_lock = threading.Lock()
        self._infer_event = threading.Event()
        self._infer_stop_event = threading.Event()
        self._infer_slot = None
        self._dropped_old_infer_frames = 0
        self._infer_thread: Optional[threading.Thread] = None
        if self.prioritize_latest_frame:
            self._infer_thread = threading.Thread(
                target=self._infer_worker_loop,
                name="latest_frame_infer_worker",
                daemon=True,
            )
            self._infer_thread.start()

        # Async single-slot debug worker (decouples debug rendering/publish from observation path)
        self._debug_lock = threading.Lock()
        self._debug_event = threading.Event()
        self._debug_stop_event = threading.Event()
        self._debug_slot = None
        self._debug_dropped_frames = 0
        self._debug_thread: Optional[threading.Thread] = None
        if self.publish_debug_image and self.debug_pub is not None:
            self._debug_thread = threading.Thread(
                target=self._debug_worker_loop,
                name="debug_viz_worker",
                daemon=True,
            )
            self._debug_thread.start()

        # Periodic drain timer (10 Hz / 100ms) to ensure pending frames expire even if input stops
        self.drain_timer = self.create_timer(0.10, self._drain_pending_frames)
        self.get_logger().info("PersonDetectorNode initialized and listening for perception streams.")

    def _on_status(self, msg: String):
        try:
            status_data = json.loads(msg.data)
            session_id = status_data.get('source_session_id', '')
            loop_count = status_data.get('loop_count', 0)
            reset_event = status_data.get('reset_event', False)

            if self.has_seen_status:
                # Drop stale status from past expired sessions
                if session_id in self.past_session_ids:
                    self.get_logger().debug(
                        f"Dropped stale status from past session: {session_id}"
                    )
                    return
                # Drop stale status from older loops of current session
                if session_id == self.current_session_id and loop_count < self.last_loop_count:
                    self.get_logger().debug(
                        f"Dropped stale status: loop {loop_count} < {self.last_loop_count}"
                    )
                    return
                if session_id != self.current_session_id:
                    if self.current_session_id:
                        self.past_session_ids.add(self.current_session_id)
            else:
                self.has_seen_status = True
                self.current_session_id = session_id
                self.last_loop_count = loop_count

            self.latest_status = status_data
            if 'camera_fps' in status_data and status_data['camera_fps']:
                try:
                    cfps = float(status_data['camera_fps'])
                    if cfps > 0:
                        self._cam_fps_from_status = cfps
                except Exception:
                    pass

            stamp_dict = status_data.get('stamp', {})
            sec = stamp_dict.get('sec', 0)
            nanosec = stamp_dict.get('nanosec', 0)
            self.status_by_stamp[(sec, nanosec)] = status_data

            if len(self.status_by_stamp) > 100:
                oldest_key = next(iter(self.status_by_stamp))
                self.status_by_stamp.pop(oldest_key, None)

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
            elif session_id:
                self.current_session_id = session_id

            self._drain_pending_frames()
        except Exception as exc:
            self.get_logger().warn(f"Error parsing source status: {exc}")

    @staticmethod
    def _build_stream_qos(reliability: str, depth: int) -> QoSProfile:
        rel_policy = (
            ReliabilityPolicy.RELIABLE
            if str(reliability).strip().lower() == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(depth)),
            reliability=rel_policy,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _reset_pipeline(self):
        # Reset tracker, depth buffers, readiness, and clock history cleanly
        with self._tracker_exec_lock:
            self.tracker.reset()
            self.synchronizer.reset()
            self.last_frame_timestamp = -1.0
            self._consecutive_ready_frames = 0
            self._last_camera_ready = False
            self._last_readiness_reason = "PIPELINE_RESET_WARMING_UP"
        with self._infer_lock:
            self._infer_slot = None
            self._infer_event.clear()

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
        if self.has_seen_status:
            if frame_stamp not in self.status_by_stamp:
                # Buffer briefly in bounded queue waiting for matching status
                now_mono = time.monotonic()
                if len(self.pending_rgb_frames) >= 50:
                    self.pending_rgb_frames.pop(0)
                    self.get_logger().warn("Pending queue full (>=50); dropped oldest pending RGB frame.")
                self.pending_rgb_frames.append((now_mono, msg))
                self._drain_pending_frames()
                return
            else:
                matched_status = self.status_by_stamp.pop(frame_stamp)
                self._process_rgb_message(msg, matched_status=matched_status)
                return

        self._process_rgb_message(msg, matched_status=None)

    def _drain_pending_frames(self):
        now_mono = time.monotonic()
        remaining = []
        for queue_time, frame_msg in self.pending_rgb_frames:
            stamp = (frame_msg.header.stamp.sec, frame_msg.header.stamp.nanosec)
            if (now_mono - queue_time) > self.max_pending_age_sec:
                # Timeout expired: drop frame regardless of whether status matches now!
                self.status_by_stamp.pop(stamp, None)
                self.get_logger().warn(
                    f"Dropped frame with stamp {stamp}: status timeout in replay mode (waited > {self.max_pending_age_sec}s without status match)."
                )
            elif stamp in self.status_by_stamp:
                matched_status = self.status_by_stamp.pop(stamp)
                self._process_rgb_message(frame_msg, matched_status=matched_status)
            else:
                remaining.append((queue_time, frame_msg))
        self.pending_rgb_frames = remaining

    def _process_rgb_message(self, msg: Image, matched_status: Optional[dict] = None):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        status_data = matched_status

        if self.has_seen_status and status_data is None:
            # Replay mode requiring metadata: cannot process frame without matched status
            self.get_logger().warn(f"Dropped frame with stamp {stamp}: missing metadata in status mode.")
            return

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

        if self.prioritize_latest_frame:
            with self._infer_lock:
                if self._infer_slot is not None:
                    self._dropped_old_infer_frames += 1
                self._infer_slot = (msg, status_data, self.current_session_id)
                self._infer_event.set()
            return

        self._execute_rgb_inference(msg, status_data, self.current_session_id)

    def _infer_worker_loop(self):
        """Dedicated worker processing only the latest queued frame so YOLO never accumulates backlog."""
        while not self._infer_stop_event.is_set():
            if not self._infer_event.wait(timeout=0.2):
                continue
            with self._infer_lock:
                slot = self._infer_slot
                self._infer_slot = None
                self._infer_event.clear()
            if slot is None:
                continue
            msg, status_data, session_id = slot
            self._execute_rgb_inference(msg, status_data, session_id)

    def _evaluate_camera_readiness(self, obs_frame) -> tuple[bool, str]:
        """Verify camera stream & depth registration readiness before allowing motion/locking."""
        dq = obs_frame.data_quality
        sm = obs_frame.source_mode

        if dq in (
            DataQuality.STALE.value,
            DataQuality.DIMENSION_MISMATCH.value,
            DataQuality.FRAME_ID_MISMATCH.value,
            DataQuality.INVALID_DEPTH.value,
        ):
            self._consecutive_ready_frames = 0
            return False, f"CAMERA_DEGRADED_{dq}"

        if sm == SourceMode.LIVE_ASTRA.value:
            if dq in (DataQuality.RGB_ONLY.value, DataQuality.DEGRADED_NO_INFO.value):
                self._consecutive_ready_frames = 0
                return False, f"ASTRA_STREAM_INCOMPLETE_{dq}"

        self._consecutive_ready_frames += 1
        if self._consecutive_ready_frames < self.readiness_min_frames:
            return False, f"CAMERA_WARMING_UP_{self._consecutive_ready_frames}_{self.readiness_min_frames}"

        if dq == DataQuality.UNVERIFIED_ALIGNMENT.value:
            return True, "CAMERA_READY_UNVERIFIED_ALIGNMENT"
        return True, "CAMERA_READY"

    def _execute_rgb_inference(self, msg: Image, status_data: Optional[dict], session_id: str):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        try:
            rgb_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Error decoding RGB message: {exc}")
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        with self._tracker_exec_lock:
            # Synchronize and normalize observation frame
            obs_frame = self.synchronizer.process_rgb(
                rgb_img=rgb_np,
                stamp=stamp,
                frame_id=msg.header.frame_id,
                current_time_sec=now_sec,
            )
            if session_id:
                obs_frame.source_session_id = session_id

            if status_data and obs_frame.data_quality != DataQuality.STALE.value:
                reported_mode = str(status_data.get('source_mode', ''))
                if reported_mode:
                    obs_frame.source_mode = reported_mode
                if reported_mode == SourceMode.LIVE_WEBCAM.value and obs_frame.depth is None:
                    obs_frame.data_quality = DataQuality.RGB_ONLY.value
                    obs_frame.error_reason = "Live webcam RGB-only mode; depth and calibrated CameraInfo absent."

            if obs_frame.data_quality == 'STALE':
                self._consecutive_ready_frames = 0
                self._last_camera_ready = False
                self._last_readiness_reason = "CAMERA_FRAME_STALE"
                self.get_logger().warn(f"Dropped stale frame: {obs_frame.error_reason}")
                return

            camera_ready, readiness_reason = self._evaluate_camera_readiness(obs_frame)
            self._last_camera_ready = camera_ready
            self._last_readiness_reason = readiness_reason

            self._rgb_times.append(time.monotonic())

            # Run multi-person detection and tracking
            try:
                multi_obs = self.tracker.process_frame(obs_frame)
                self.processed_frames += 1
            except Exception as exc:
                self.get_logger().error(f"Error running tracker inference: {exc}")
                return

        # Publish observation payload immediately (critical path)
        self._publish_observation(multi_obs, camera_ready=camera_ready, readiness_reason=readiness_reason)
        self._obs_times.append(time.monotonic())

        # Enqueue into 1-slot debug buffer (non-blocking)
        if self.publish_debug_image and self.debug_pub is not None:
            self._enqueue_debug_image(rgb_np, multi_obs, msg.header)

    def _publish_observation(self, obs: MultiPersonObservation, camera_ready: bool = True, readiness_reason: str = "CAMERA_READY"):
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
                "near_horizontal_edge": getattr(p, 'near_horizontal_edge', p.near_edge),
                "bbox_clipped": getattr(p, 'bbox_clipped', False),
                "keypoints": p.keypoints,
                "bearing_optical_rad": getattr(p, 'bearing_optical_rad', None),
                "bearing_base_rad": getattr(p, 'bearing_base_rad', None),
                "position_base": getattr(p, 'position_base', None),
                "geometry_valid": getattr(p, 'geometry_valid', False),
                "geometry_source": getattr(p, 'geometry_source', 'OPTICAL_ONLY'),
                "geometry_error": getattr(p, 'geometry_error', ''),
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
            "camera_ready": camera_ready,
            "camera_readiness_reason": readiness_reason,
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

    def _on_session_state(self, msg: String):
        """Cache latest TrackingManager session_state for debug overlay (read-only)."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.latest_session_state = data
                self.last_session_state_mono_time = time.monotonic()
        except Exception:
            pass

    def _on_debug_lock_state(self, msg: String):
        """Cache latest TrackingManager gesture hold telemetry for debug overlay (read-only)."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.latest_debug_lock_state = data
                self.last_debug_lock_mono_time = time.monotonic()
        except Exception:
            pass

    @staticmethod
    def _compute_rolling_fps(times_deque: deque) -> Optional[float]:
        if len(times_deque) < 2:
            return None
        dt = times_deque[-1] - times_deque[0]
        if dt <= 0.05:
            return None
        return (len(times_deque) - 1) / dt

    def _build_debug_context(self, inference_ms: float = 0.0) -> DebugOverlayContext:
        now_mono = time.monotonic()
        cam_fps = self._cam_fps_from_status or self._compute_rolling_fps(self._rgb_times)
        obs_fps = self._compute_rolling_fps(self._obs_times)
        dbg_fps = self._compute_rolling_fps(self._dbg_times)

        ctx = DebugOverlayContext(
            edge_margin_ratio=self.tracker.edge_margin_ratio,
            camera_fps=cam_fps,
            observation_fps=obs_fps,
            debug_fps=dbg_fps,
            inference_ms=inference_ms,
        )

        # Prefer debug_lock_state if fresh, else fall back to session_state
        sess_fresh = (
            self.latest_session_state is not None
            and (now_mono - self.last_session_state_mono_time) <= self.session_freshness_ttl_sec
        )
        dbg_fresh = (
            self.latest_debug_lock_state is not None
            and (now_mono - self.last_debug_lock_mono_time) <= self.session_freshness_ttl_sec
        )

        if self.latest_session_state is None and self.latest_debug_lock_state is None:
            ctx.fsm_available = False
            ctx.fsm_stale = False
            return ctx

        ctx.fsm_available = True
        if not sess_fresh and not dbg_fresh:
            ctx.fsm_stale = True
            return ctx

        state_src = self.latest_debug_lock_state if dbg_fresh else self.latest_session_state
        ctx.fsm_stale = False
        ctx.fsm_state = str(state_src.get("state", "IDLE"))
        ctx.locked = bool(state_src.get("locked", False))
        ctx.locked_track_id = state_src.get("locked_track_id") if dbg_fresh else state_src.get("person_track_id")
        ctx.locked_source_session_id = str(
            state_src.get("locked_source_session_id", state_src.get("source_session_id", "")) or ""
        )
        ctx.reason_code = str(state_src.get("reason_code", ""))
        ctx.obs_fresh = state_src.get("obs_fresh")

        if dbg_fresh and self.latest_debug_lock_state is not None:
            ctx.gesture_hold_time_sec = float(self.latest_debug_lock_state.get("gesture_hold_time_sec", 1.5))
            for item in self.latest_debug_lock_state.get("gesture_candidates", []):
                tid = item.get("track_id")
                sid = str(item.get("source_session_id", "") or "")
                hold = float(item.get("accumulated_hold_sec", 0.0))
                if tid is not None:
                    ctx.gesture_progress[(sid, int(tid))] = hold

        return ctx

    def _enqueue_debug_image(self, bgr_img: np.ndarray, obs: MultiPersonObservation, header):
        """Place latest frame + matching observation into 1-slot buffer (never blocks _on_rgb)."""
        ctx = self._build_debug_context(inference_ms=obs.processing_time_ms)
        with self._debug_lock:
            if self._debug_slot is not None:
                self._debug_dropped_frames += 1
            self._debug_slot = (bgr_img, obs, header, ctx)
            self._debug_event.set()

    def _debug_worker_loop(self):
        """Dedicated background worker rendering & publishing latest debug image."""
        while not self._debug_stop_event.is_set():
            if not self._debug_event.wait(timeout=0.2):
                continue
            with self._debug_lock:
                slot = self._debug_slot
                self._debug_slot = None
                self._debug_event.clear()
            if slot is None:
                continue
            bgr_img, obs, header, ctx = slot
            self._publish_debug_image(bgr_img, obs, header, ctx=ctx)

    def _publish_debug_image(self, bgr_img: np.ndarray, obs: MultiPersonObservation, header, ctx: Optional[DebugOverlayContext] = None):
        try:
            if ctx is None:
                ctx = self._build_debug_context(inference_ms=obs.processing_time_ms)
            debug_img = self.tracker.draw_debug_overlay(bgr_img, obs, context=ctx)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
            self._dbg_times.append(time.monotonic())
        except Exception as exc:
            self.get_logger().warn(f"Error publishing debug image: {exc}")

    def destroy_node(self):
        self._infer_stop_event.set()
        self._infer_event.set()
        if self._infer_thread is not None and self._infer_thread.is_alive():
            self._infer_thread.join(timeout=2.0)
        self._debug_stop_event.set()
        self._debug_event.set()
        if self._debug_thread is not None and self._debug_thread.is_alive():
            self._debug_thread.join(timeout=2.0)
        super().destroy_node()


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
