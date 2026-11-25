"""Source adapters and data normalization for person tracking perception inputs.

Supports:
- Live Astra camera topics (RGB, Depth, CameraInfo).
- Local RGB video file replay (with synthetic/pinhole CameraInfo).
- Synthetic deterministic fixtures (for testing error conditions and degraded inputs).
- Bag replay topic normalization and clock decoupling.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


class SourceMode(str, Enum):
    LIVE_ASTRA = "live_astra"
    LIVE_WEBCAM = "live_webcam"
    VIDEO_REPLAY = "video_replay"
    BAG_REPLAY = "bag_replay"
    SYNTHETIC_FIXTURE = "synthetic_fixture"


class DataQuality(str, Enum):
    VALID_RGBD = "VALID_RGBD"
    RGB_ONLY = "RGB_ONLY"
    UNVERIFIED_ALIGNMENT = "UNVERIFIED_ALIGNMENT"
    DEGRADED_NO_INFO = "DEGRADED_NO_INFO"
    STALE = "STALE"
    DIMENSION_MISMATCH = "DIMENSION_MISMATCH"
    FRAME_ID_MISMATCH = "FRAME_ID_MISMATCH"
    INVALID_DEPTH = "INVALID_DEPTH"


@dataclass
class ObservationFrame:
    """Standardized observation frame passed from input adapters to perception."""
    rgb: np.ndarray  # HxWx3 uint8 BGR
    depth: Optional[np.ndarray]  # HxW float32 depth in meters (or None if RGB-only)
    depth_raw: Optional[np.ndarray]  # HxW uint16 depth in millimeters (or None)
    camera_info: Optional[CameraInfo]
    stamp_sec: int
    stamp_nanosec: int
    receive_monotonic: float
    frame_id: str
    source_mode: str
    data_quality: str
    error_reason: str = ""
    source_session_id: str = ""
    loop_count: int = 0

    @property
    def has_depth(self) -> bool:
        return self.depth is not None and self.data_quality in (
            DataQuality.VALID_RGBD,
            DataQuality.UNVERIFIED_ALIGNMENT,
        )

    @property
    def stamp_total_seconds(self) -> float:
        return self.stamp_sec + self.stamp_nanosec * 1e-9


def compute_mad(data: np.ndarray, median_val: float) -> float:
    """Compute Median Absolute Deviation (MAD): median(|data - median_val|)."""
    if data.size == 0:
        return float("inf")
    return float(np.median(np.abs(data - median_val)))


def create_simulated_camera_info(
    width: int,
    height: int,
    hfov_deg: float = 60.0,
    frame_id: str = "camera_color_optical_frame",
) -> CameraInfo:
    """Generate a CameraInfo message with an idealized pinhole model."""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    info.distortion_model = "simulated_pinhole"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    hfov_rad = math.radians(hfov_deg)
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0

    # Intrinsic camera matrix K [3x3 row-major]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

    # Rectification matrix R [3x3 identity]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # Projection matrix P [3x4 row-major]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    return info


class VideoReplayAdapter:
    """Reads a local video file (MP4/AVI) and outputs standardized frames."""

    def __init__(
        self,
        video_path: str,
        fps: Optional[float] = None,
        loop: bool = True,
        retime_stamps: bool = True,
        frame_id: str = "camera_color_optical_frame",
    ):
        self.video_path = video_path
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise FileNotFoundError(f"Cannot open video file: {video_path}")

        native_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if (fps and fps > 0) else (native_fps if native_fps > 0 else 15.0)
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.loop = loop
        self.retime_stamps = retime_stamps
        self.frame_id = frame_id

        self.current_frame_idx = 0
        self.eof = False
        self.loop_count = 0
        self.just_looped = False
        self.camera_info = create_simulated_camera_info(self.width, self.height, frame_id=self.frame_id)

    def read_frame(self, current_ros_time: Optional[Tuple[int, int]] = None) -> Optional[ObservationFrame]:
        """Read the next frame from the video source."""
        if not self.cap.isOpened():
            return None

        ret, bgr = self.cap.read()
        if not ret:
            if self.loop and self.total_frames > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.current_frame_idx = 0
                self.loop_count += 1
                self.just_looped = True
                ret, bgr = self.cap.read()
                if not ret:
                    self.eof = True
                    return None
            else:
                self.eof = True
                return None

        self.current_frame_idx += 1
        now_mono = time.monotonic()

        if self.retime_stamps and current_ros_time is not None:
            stamp_sec, stamp_nanosec = current_ros_time
        elif self.retime_stamps:
            now_wall = time.time()
            stamp_sec = int(now_wall)
            stamp_nanosec = int((now_wall - stamp_sec) * 1e9)
        else:
            # Use elapsed video time
            elapsed = self.current_frame_idx / self.fps
            stamp_sec = int(elapsed)
            stamp_nanosec = int((elapsed - stamp_sec) * 1e9)

        # Update CameraInfo timestamp
        if self.camera_info:
            self.camera_info.header.stamp.sec = stamp_sec
            self.camera_info.header.stamp.nanosec = stamp_nanosec

        return ObservationFrame(
            rgb=bgr,
            depth=None,  # RGB-only; never synthesizes fake metric depth!
            depth_raw=None,
            camera_info=self.camera_info,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            receive_monotonic=now_mono,
            frame_id=self.frame_id,
            source_mode=SourceMode.VIDEO_REPLAY.value,
            data_quality=DataQuality.RGB_ONLY.value,
            error_reason="RGB-only video replay; depth sensor absent.",
        )

    def close(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()


class LiveWebcamAdapter:
    """Reads live RGB frames from a local webcam via OpenCV VideoCapture (strictly RGB_ONLY)."""

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        frame_id: str = "camera_color_optical_frame",
        cap_factory=None,
    ):
        self.camera_index = int(camera_index)
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.fps = float(fps) if (fps and fps > 0) else 30.0
        self.frame_id = frame_id
        self._cap_factory = cap_factory or cv2.VideoCapture

        self.cap = self._cap_factory(self.camera_index)
        if self.cap is None or not self.cap.isOpened():
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
            raise RuntimeError(f"Cannot open webcam device camera_index={self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.requested_width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.requested_height))
        self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.width = actual_w if actual_w > 0 else self.requested_width
        self.height = actual_h if actual_h > 0 else self.requested_height

        self.frame_count = 0
        self.consecutive_failures = 0
        self.total_failures = 0
        self.last_error = ""

    def read_frame(self, current_ros_time: Optional[Tuple[int, int]] = None) -> Optional[ObservationFrame]:
        """Read next RGB frame from webcam. Never synthesizes depth or calibrated CameraInfo."""
        if self.cap is None or not self.cap.isOpened():
            self.consecutive_failures += 1
            self.total_failures += 1
            self.last_error = "Webcam device is not open."
            return None

        ret, bgr = self.cap.read()
        if not ret or bgr is None or bgr.size == 0:
            self.consecutive_failures += 1
            self.total_failures += 1
            self.last_error = "Failed to read frame from webcam."
            return None

        self.consecutive_failures = 0
        self.last_error = ""
        self.frame_count += 1
        now_mono = time.monotonic()

        if current_ros_time is not None:
            stamp_sec, stamp_nanosec = current_ros_time
        else:
            now_wall = time.time()
            stamp_sec = int(now_wall)
            stamp_nanosec = int((now_wall - stamp_sec) * 1e9)

        return ObservationFrame(
            rgb=bgr,
            depth=None,
            depth_raw=None,
            camera_info=None,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            receive_monotonic=now_mono,
            frame_id=self.frame_id,
            source_mode=SourceMode.LIVE_WEBCAM.value,
            data_quality=DataQuality.RGB_ONLY.value,
            error_reason="Live webcam RGB-only mode; depth and calibrated CameraInfo absent.",
        )

    def close(self):
        """Release webcam VideoCapture cleanly."""
        if self.cap is not None:
            try:
                if self.cap.isOpened():
                    self.cap.release()
            except Exception:
                pass
            self.cap = None


class SyntheticFixtureGenerator:
    """Generates deterministic synthetic RGB-D frames for testing."""

    @staticmethod
    def generate_empty_scene(
        width: int = 640,
        height: int = 480,
        depth_m: float = 3.0,
        frame_id: str = "camera_color_optical_frame",
        stamp: Tuple[int, int] = (1700000000, 0),
    ) -> ObservationFrame:
        """Empty scene: flat gray background, constant depth plane."""
        rgb = np.full((height, width, 3), 128, dtype=np.uint8)
        depth = np.full((height, width), depth_m, dtype=np.float32)
        depth_raw = (depth * 1000.0).astype(np.uint16)
        info = create_simulated_camera_info(width, height, frame_id=frame_id)
        info.header.stamp.sec, info.header.stamp.nanosec = stamp

        return ObservationFrame(
            rgb=rgb,
            depth=depth,
            depth_raw=depth_raw,
            camera_info=info,
            stamp_sec=stamp[0],
            stamp_nanosec=stamp[1],
            receive_monotonic=time.monotonic(),
            frame_id=frame_id,
            source_mode=SourceMode.SYNTHETIC_FIXTURE.value,
            data_quality=DataQuality.VALID_RGBD.value,
            error_reason="",
        )

    @staticmethod
    def generate_person_scene(
        width: int = 640,
        height: int = 480,
        person_bbox: Tuple[int, int, int, int] = (220, 80, 420, 420),
        person_depth_m: float = 1.5,
        bg_depth_m: float = 3.5,
        frame_id: str = "camera_color_optical_frame",
        stamp: Tuple[int, int] = (1700000000, 0),
    ) -> ObservationFrame:
        """Synthetic scene containing a rectangular target person at specified depth."""
        x1, y1, x2, y2 = person_bbox
        rgb = np.full((height, width, 3), 180, dtype=np.uint8)
        # Person colored patch
        rgb[y1:y2, x1:x2] = (50, 120, 220)

        depth = np.full((height, width), bg_depth_m, dtype=np.float32)
        depth[y1:y2, x1:x2] = person_depth_m
        depth_raw = (depth * 1000.0).astype(np.uint16)

        info = create_simulated_camera_info(width, height, frame_id=frame_id)
        info.header.stamp.sec, info.header.stamp.nanosec = stamp

        return ObservationFrame(
            rgb=rgb,
            depth=depth,
            depth_raw=depth_raw,
            camera_info=info,
            stamp_sec=stamp[0],
            stamp_nanosec=stamp[1],
            receive_monotonic=time.monotonic(),
            frame_id=frame_id,
            source_mode=SourceMode.SYNTHETIC_FIXTURE.value,
            data_quality=DataQuality.VALID_RGBD.value,
            error_reason="",
        )

    @staticmethod
    def generate_multi_person_scene(
        width: int = 640,
        height: int = 480,
        persons: Optional[List[Dict[str, Any]]] = None,
        bg_depth_m: float = 3.5,
        frame_id: str = "camera_color_optical_frame",
        stamp: Tuple[int, int] = (1700000000, 0),
        data_quality: str = DataQuality.VALID_RGBD.value,
        source_session_id: str = "",
    ) -> ObservationFrame:
        """Generate a synthetic frame containing multiple persons at specified boxes and depths."""
        if persons is None:
            persons = [
                {"bbox": (100, 80, 260, 420), "depth_m": 1.6, "color": (50, 120, 220)},
                {"bbox": (380, 80, 540, 420), "depth_m": 2.2, "color": (200, 100, 60)},
            ]

        rgb = np.full((height, width, 3), 180, dtype=np.uint8)
        depth = np.full((height, width), bg_depth_m, dtype=np.float32)

        for p in persons:
            x1, y1, x2, y2 = p["bbox"]
            color = p.get("color", (50, 120, 220))
            d_val = p.get("depth_m", 1.8)
            rgb[y1:y2, x1:x2] = color
            depth[y1:y2, x1:x2] = d_val

        depth_raw = (depth * 1000.0).astype(np.uint16)
        info = create_simulated_camera_info(width, height, frame_id=frame_id)
        info.header.stamp.sec, info.header.stamp.nanosec = stamp

        return ObservationFrame(
            rgb=rgb,
            depth=depth,
            depth_raw=depth_raw,
            camera_info=info,
            stamp_sec=stamp[0],
            stamp_nanosec=stamp[1],
            receive_monotonic=time.monotonic(),
            frame_id=frame_id,
            source_mode=SourceMode.SYNTHETIC_FIXTURE.value,
            data_quality=data_quality,
            error_reason="",
            source_session_id=source_session_id,
        )

    @staticmethod
    def generate_corrupted_depth_scene(
        width: int = 640,
        height: int = 480,
        corruption_type: str = "nan_and_zeros",
        frame_id: str = "camera_color_optical_frame",
        stamp: Tuple[int, int] = (1700000000, 0),
    ) -> ObservationFrame:
        """Scene with deliberate depth defects (NaN, 0.0, out of range, noise)."""
        rgb = np.full((height, width, 3), 100, dtype=np.uint8)
        depth = np.full((height, width), 2.0, dtype=np.float32)

        if corruption_type == "nan_and_zeros":
            # Set entire central region to NaN and zeros
            depth[100:300, 150:450] = np.nan
            depth[300:400, 150:450] = 0.0
            quality = DataQuality.INVALID_DEPTH.value
            reason = "Central depth region contains NaN and zero values."
        elif corruption_type == "out_of_range":
            depth[:, :] = 10.0  # Beyond Astra 5.0m max
            quality = DataQuality.INVALID_DEPTH.value
            reason = "Depth values exceed 5.0m maximum operational range."
        elif corruption_type == "mad_noise_outlier":
            # High noise dispersion to trigger MAD filter rejection
            noise = np.random.uniform(-1.5, 1.5, size=(height, width)).astype(np.float32)
            depth = np.clip(depth + noise, 0.4, 5.0)
            quality = DataQuality.INVALID_DEPTH.value
            reason = "High depth dispersion exceeds MAD threshold."
        else:
            quality = DataQuality.VALID_RGBD.value
            reason = ""

        depth_raw = np.nan_to_num(depth * 1000.0, nan=0.0).astype(np.uint16)
        info = create_simulated_camera_info(width, height, frame_id=frame_id)

        return ObservationFrame(
            rgb=rgb,
            depth=depth,
            depth_raw=depth_raw,
            camera_info=info,
            stamp_sec=stamp[0],
            stamp_nanosec=stamp[1],
            receive_monotonic=time.monotonic(),
            frame_id=frame_id,
            source_mode=SourceMode.SYNTHETIC_FIXTURE.value,
            data_quality=quality,
            error_reason=reason,
        )


class LiveAstraAdapter:
    """Validates and processes incoming ROS messages from live camera or bag topics."""

    def __init__(
        self,
        max_capture_age_sec: float = 0.40,
        sync_slop_sec: float = 0.06,
        require_hardware_alignment_proof: bool = True,
    ):
        self.max_capture_age_sec = max_capture_age_sec
        self.sync_slop_sec = sync_slop_sec
        self.require_hardware_alignment_proof = require_hardware_alignment_proof
        self.last_rgb_stamp: float = -1.0

    def reset(self):
        """Reset tracker clock tracking (e.g. on new bag or source switch)."""
        self.last_rgb_stamp = -1.0

    def process_messages(
        self,
        rgb_img: np.ndarray,
        rgb_header_stamp: Tuple[int, int],
        rgb_frame_id: str,
        depth_img: Optional[np.ndarray],
        depth_header_stamp: Optional[Tuple[int, int]],
        depth_frame_id: Optional[str],
        depth_encoding: Optional[str],
        camera_info: Optional[CameraInfo],
        current_time_sec: float,
    ) -> ObservationFrame:
        """Process incoming synchronized data into an ObservationFrame.
        
        Guarantees that RGB readiness is decoupled from depth validity:
        If depth is absent, has excessive stamp difference, dimension/frame_id mismatch,
        or invalid encoding, valid RGB is STILL retained and passed to downstream pose/track.
        """
        receive_mono = time.monotonic()
        stamp_sec, stamp_nanosec = rgb_header_stamp
        img_time = stamp_sec + stamp_nanosec * 1e-9
        age = current_time_sec - img_time

        # 1. Stale & Out-of-order check on RGB
        if self.last_rgb_stamp > 0.0 and img_time < self.last_rgb_stamp:
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=DataQuality.STALE.value,
                error_reason=f"Out-of-order frame: stamp {img_time:.4f}s < last stamp {self.last_rgb_stamp:.4f}s.",
            )

        if age > self.max_capture_age_sec or age < -0.05:
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=DataQuality.STALE.value,
                error_reason=f"Image age ({age:.3f}s) outside window [-0.05, {self.max_capture_age_sec}s].",
            )

        self.last_rgb_stamp = img_time

        # 2. CameraInfo validation
        info_missing = (
            camera_info is None or
            len(camera_info.k) < 9 or
            camera_info.k[0] <= 0
        )

        # 3. Depth check (Decoupled: missing or invalid depth degrades to RGB-only)
        if depth_img is None:
            quality = DataQuality.DEGRADED_NO_INFO.value if info_missing else DataQuality.RGB_ONLY.value
            reason = "CameraInfo missing & depth absent; operating in RGB-only mode." if info_missing else "Depth stream absent; operating in RGB-only mode."
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=quality,
                error_reason=reason,
            )

        # 4. Check Depth timestamp synchronization (sync_slop_sec)
        if depth_header_stamp is not None:
            depth_time = depth_header_stamp[0] + depth_header_stamp[1] * 1e-9
            slop = abs(img_time - depth_time)
            if slop > self.sync_slop_sec:
                # Stamp slop exceeded: drop depth, keep valid RGB!
                quality = DataQuality.DEGRADED_NO_INFO.value if info_missing else DataQuality.RGB_ONLY.value
                return ObservationFrame(
                    rgb=rgb_img,
                    depth=None,
                    depth_raw=None,
                    camera_info=camera_info,
                    stamp_sec=stamp_sec,
                    stamp_nanosec=stamp_nanosec,
                    receive_monotonic=receive_mono,
                    frame_id=rgb_frame_id,
                    source_mode=SourceMode.LIVE_ASTRA.value,
                    data_quality=quality,
                    error_reason=f"Depth stamp slop ({slop:.4f}s) exceeds limit ({self.sync_slop_sec:.4f}s); degraded to RGB-only.",
                )

        # 5. Dimension check
        if (rgb_img.shape[0], rgb_img.shape[1]) != (depth_img.shape[0], depth_img.shape[1]):
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=DataQuality.DIMENSION_MISMATCH.value,
                error_reason=f"RGB shape {rgb_img.shape[:2]} != Depth shape {depth_img.shape[:2]}; degraded to RGB-only.",
            )

        # 6. Frame ID check
        if depth_frame_id != rgb_frame_id:
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=DataQuality.FRAME_ID_MISMATCH.value,
                error_reason=f"RGB frame '{rgb_frame_id}' != Depth frame '{depth_frame_id}'; degraded to RGB-only.",
            )

        # 7. Unpack depth to meters
        depth_raw = None
        if depth_encoding == "16UC1":
            depth_raw = depth_img.copy()
            depth_m = depth_img.astype(np.float32) * 0.001
        elif depth_encoding == "32FC1":
            depth_m = depth_img.astype(np.float32)
            depth_raw = (depth_m * 1000.0).astype(np.uint16)
        else:
            return ObservationFrame(
                rgb=rgb_img,
                depth=None,
                depth_raw=None,
                camera_info=camera_info,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                receive_monotonic=receive_mono,
                frame_id=rgb_frame_id,
                source_mode=SourceMode.LIVE_ASTRA.value,
                data_quality=DataQuality.INVALID_DEPTH.value,
                error_reason=f"Unsupported depth encoding: {depth_encoding}; degraded to RGB-only.",
            )

        # 8. Alignment verification policy (D03 / Phase 0 Audit Addendum)
        if self.require_hardware_alignment_proof:
            quality = DataQuality.UNVERIFIED_ALIGNMENT.value
            reason = "Depth is present and frame IDs match, but hardware registration is pending field verification."
        elif info_missing:
            quality = DataQuality.DEGRADED_NO_INFO.value
            reason = "CameraInfo missing or contains invalid intrinsic focal length."
        else:
            quality = DataQuality.VALID_RGBD.value
            reason = ""

        return ObservationFrame(
            rgb=rgb_img,
            depth=depth_m,
            depth_raw=depth_raw,
            camera_info=camera_info,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            receive_monotonic=receive_mono,
            frame_id=rgb_frame_id,
            source_mode=SourceMode.LIVE_ASTRA.value,
            data_quality=quality,
            error_reason=reason,
        )


class TopicSynchronizer:
    """Buffers Depth and CameraInfo messages and syncs with incoming RGB frames."""

    def __init__(self, adapter: LiveAstraAdapter, max_depth_buffer: int = 30):
        self.adapter = adapter
        self.depth_buffer = []  # List of tuples: (t_depth, stamp_tuple, depth_img, frame_id, encoding)
        self.max_depth_buffer = max_depth_buffer
        self.last_camera_info: Optional[CameraInfo] = None

    def reset(self):
        """Clear internal depth buffer, cached CameraInfo, and reset adapter clock."""
        self.depth_buffer.clear()
        self.last_camera_info = None
        self.adapter.reset()

    def add_depth(
        self,
        depth_img: np.ndarray,
        stamp: Tuple[int, int],
        frame_id: str,
        encoding: str = "16UC1",
    ):
        t_depth = stamp[0] + stamp[1] * 1e-9
        self.depth_buffer.append((t_depth, stamp, depth_img, frame_id, encoding))
        if len(self.depth_buffer) > self.max_depth_buffer:
            self.depth_buffer.pop(0)

    def set_camera_info(self, info: CameraInfo):
        self.last_camera_info = info

    def process_rgb(
        self,
        rgb_img: np.ndarray,
        stamp: Tuple[int, int],
        frame_id: str,
        current_time_sec: float,
    ) -> ObservationFrame:
        """Match RGB frame with nearest buffered depth and produce ObservationFrame."""
        t_rgb = stamp[0] + stamp[1] * 1e-9

        best_depth = None
        best_stamp = None
        best_fid = None
        best_enc = None
        best_diff = float("inf")

        for t_d, d_stamp, d_img, d_fid, d_enc in self.depth_buffer:
            diff = abs(t_rgb - t_d)
            if diff < best_diff:
                best_diff = diff
                best_depth = d_img
                best_stamp = d_stamp
                best_fid = d_fid
                best_enc = d_enc

        # Clean old depth frames older than t_rgb - 1.0s
        self.depth_buffer = [d for d in self.depth_buffer if d[0] >= t_rgb - 1.0]

        # Only pass depth if difference is within sync_slop_sec
        if best_diff > self.adapter.sync_slop_sec:
            best_depth = None
            best_stamp = None
            best_fid = None
            best_enc = None

        return self.adapter.process_messages(
            rgb_img=rgb_img,
            rgb_header_stamp=stamp,
            rgb_frame_id=frame_id,
            depth_img=best_depth,
            depth_header_stamp=best_stamp,
            depth_frame_id=best_fid,
            depth_encoding=best_enc,
            camera_info=self.last_camera_info,
            current_time_sec=current_time_sec,
        )
