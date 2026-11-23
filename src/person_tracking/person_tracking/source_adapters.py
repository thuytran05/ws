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
from typing import Optional, Tuple, Dict, Any

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image


class SourceMode(str, Enum):
    LIVE_ASTRA = "live_astra"
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
        """Process incoming synchronized data into an ObservationFrame."""
        receive_mono = time.monotonic()
        stamp_sec, stamp_nanosec = rgb_header_stamp
        img_time = stamp_sec + stamp_nanosec * 1e-9
        age = current_time_sec - img_time

        # 1. Stale check
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

        # 2. CameraInfo validation
        if camera_info is None or len(camera_info.k) < 9 or camera_info.k[0] <= 0:
            quality = DataQuality.DEGRADED_NO_INFO.value
            reason = "CameraInfo missing or contains invalid intrinsic focal length."
        else:
            quality = DataQuality.VALID_RGBD.value
            reason = ""

        # 3. Depth check (if depth is absent or RGB-only stream)
        if depth_img is None:
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
                data_quality=DataQuality.RGB_ONLY.value,
                error_reason="Depth stream absent; operating in RGB-only mode.",
            )

        # 4. Dimension check
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
                error_reason=f"RGB shape {rgb_img.shape[:2]} != Depth shape {depth_img.shape[:2]}.",
            )

        # 5. Frame ID check
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
                error_reason=f"RGB frame '{rgb_frame_id}' != Depth frame '{depth_frame_id}'.",
            )

        # 6. Unpack depth to meters
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
                error_reason=f"Unsupported depth encoding: {depth_encoding}.",
            )

        # 7. Alignment verification policy (D03 / Phase 0 Audit Addendum)
        # Matching frame_id does not prove hardware registration succeeded!
        if self.require_hardware_alignment_proof and quality == DataQuality.VALID_RGBD.value:
            quality = DataQuality.UNVERIFIED_ALIGNMENT.value
            reason = "Depth is present and frame IDs match, but hardware registration is pending field verification."

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
