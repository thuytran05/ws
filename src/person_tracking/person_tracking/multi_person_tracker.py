"""Multi-person pose estimation and tracking using YOLO Pose + ByteTrack.

Phase 3 Perception Module:
- Ingests ObservationFrame from Phase 2 source adapters.
- Detects multiple persons and extracts 17 body keypoints.
- Tracks individuals using ByteTrack and outputs track IDs.
- Measures torso depth using MAD filter when depth data is valid.
- Flags tracking ambiguity, edge proximity, and lost tracks.
- Strictly perception-only: NO gesture locking, NO nearest selection, NO velocity control.
"""

import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Inject .venv_yolo site-packages if available in Dev Container
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

try:
    import torch
    from ultralytics import YOLO
except ImportError as exc:
    YOLO = None
    torch = None

from person_tracking.source_adapters import ObservationFrame, compute_mad, DataQuality
from person_tracking.rear_camera_geometry import (
    RearCameraGeometry,
    CameraIntrinsics,
    CameraMountTransform,
    TargetGeometryResult,
)


@dataclass
class PersonDetection:
    """Normalized representation of a single detected person candidate."""
    bbox_xyxy: List[float] = field(default_factory=list)  # [x1, y1, x2, y2] pixel coordinates
    confidence: float = 0.0
    keypoints: List[List[float]] = field(default_factory=list)  # 17 keypoints: [x, y, conf]
    track_id: Optional[int] = None  # None or -1 if unconfirmed
    is_confirmed: bool = False  # True if tracker assigned persistent positive ID
    is_ambiguous: bool = False  # True if overlapping bboxes or identity ambiguity
    torso_depth_m: Optional[float] = None
    depth_valid: bool = False  # True ONLY if alignment proven (VALID_RGBD) and depth valid
    center_offset_x_norm: float = 0.0  # -1.0 (far left) to 1.0 (far right)
    near_edge: bool = False  # True if bbox touches outer margin
    bearing_optical_rad: Optional[float] = None
    bearing_base_rad: Optional[float] = None
    position_base: Optional[List[float]] = None
    geometry_valid: bool = False
    geometry_source: str = "OPTICAL_ONLY"
    geometry_error: str = ""
    bbox: Optional[List[float]] = None  # Backward-compatibility alias

    def __post_init__(self):
        if not self.bbox_xyxy and self.bbox is not None:
            self.bbox_xyxy = self.bbox
        elif self.bbox is None and self.bbox_xyxy:
            self.bbox = self.bbox_xyxy


@dataclass
class MultiPersonObservation:
    """Perception output for a single video/sensor frame matching INTERFACES.md."""
    stamp_sec: int
    stamp_nanosec: int
    receive_monotonic_time: float
    frame_id: str
    source_mode: str
    data_quality: str
    source_session_id: str = ""
    candidates: List[PersonDetection] = field(default_factory=list)
    active_track_ids: List[int] = field(default_factory=list)
    lost_track_ids: List[int] = field(default_factory=list)
    processing_time_ms: float = 0.0
    device: str = "cpu"

    @property
    def persons(self) -> List[PersonDetection]:
        """Backward-compatibility alias for candidates."""
        return self.candidates

    @property
    def receive_monotonic(self) -> float:
        """Backward-compatibility alias for receive_monotonic_time."""
        return self.receive_monotonic_time


class MultiPersonTracker:
    """YOLO Pose + ByteTrack pipeline for multi-person perception."""

    def __init__(
        self,
        model_path: str = "/home/ws/yolo26n-pose.pt",
        device: Optional[str] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        tracker_config: str = "bytetrack.yaml",
        edge_margin_ratio: float = 0.08,
        use_synthetic_mount: bool = False,
    ):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.tracker_config = tracker_config
        self.edge_margin_ratio = edge_margin_ratio

        if torch is None or YOLO is None:
            raise RuntimeError(
                "Ultralytics and PyTorch are required for MultiPersonTracker. "
                "Ensure container has torch and ultralytics installed in .venv_yolo."
            )

        # Device selection: prefer cuda:0 if available and not explicitly overridden
        if device is None:
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"YOLO model checkpoint not found at: {self.model_path}"
            )

        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
        except Exception as exc:
            raise RuntimeError(f"Failed to load YOLO model from '{self.model_path}': {exc}")

        self.previous_track_ids = set()
        self.total_processed_frames = 0
        self.mock_backend_candidates: Optional[List[PersonDetection]] = None
        self.use_synthetic_mount = use_synthetic_mount
        self.geometry = RearCameraGeometry()
        self.mount_tf: Optional[CameraMountTransform] = (
            CameraMountTransform.create_synthetic_rear_mount() if use_synthetic_mount else None
        )

    def reset(self):
        """Reset tracker state on source switch, video loop, or stream reset."""
        self.previous_track_ids.clear()
        if hasattr(self.model, 'predictor') and self.model.predictor is not None:
            if hasattr(self.model.predictor, 'trackers'):
                for t in self.model.predictor.trackers:
                    if hasattr(t, 'reset'):
                        try:
                            t.reset()
                        except Exception:
                            pass
                self.model.predictor.trackers = []
        self.total_processed_frames = 0

    def process_frame(self, obs_frame: ObservationFrame) -> MultiPersonObservation:
        """Run pose inference and tracking on an ObservationFrame."""
        t_start = time.perf_counter()

        rgb = obs_frame.rgb
        h, w = rgb.shape[:2]

        detected_persons: List[PersonDetection] = []
        current_track_ids = set()

        if self.mock_backend_candidates is not None:
            # Deterministic / mock inference path for reproducible contract testing
            for cand in self.mock_backend_candidates:
                box = cand.bbox_xyxy
                x1, y1, x2, y2 = box
                kp_list = cand.keypoints
                box_cx = (x1 + x2) / 2.0
                center_offset_norm = float(np.clip((box_cx - (w / 2.0)) / (w / 2.0), -1.0, 1.0))
                near_edge = (
                    x1 < self.edge_margin_ratio * w or
                    x2 > (1.0 - self.edge_margin_ratio) * w or
                    y1 < self.edge_margin_ratio * h or
                    y2 > (1.0 - self.edge_margin_ratio) * h
                )
                torso_depth_m, depth_valid = self._measure_torso_depth(
                    obs_frame, box, kp_list, h, w
                )
                is_conf = cand.is_confirmed or (cand.track_id is not None and cand.track_id > 0)
                if is_conf and cand.track_id:
                    current_track_ids.add(cand.track_id)
                eff_depth = torso_depth_m if cand.torso_depth_m is None else cand.torso_depth_m
                intrinsics = CameraIntrinsics.from_camera_info(obs_frame.camera_info)
                geom_engine = getattr(self, 'geometry', None) or RearCameraGeometry()
                mount_tf = getattr(self, 'mount_tf', None)
                geom = geom_engine.compute_geometry(
                    u=box_cx,
                    v=(y1 + y2) / 2.0,
                    torso_depth_m=eff_depth,
                    depth_valid=depth_valid,
                    intrinsics=intrinsics,
                    mount_tf=mount_tf,
                    frame_id=obs_frame.frame_id,
                    image_width=w,
                )
                person = PersonDetection(
                    bbox_xyxy=box,
                    confidence=cand.confidence,
                    keypoints=kp_list,
                    track_id=cand.track_id,
                    is_confirmed=is_conf,
                    is_ambiguous=cand.is_ambiguous,
                    torso_depth_m=eff_depth,
                    depth_valid=depth_valid,
                    center_offset_x_norm=center_offset_norm,
                    near_edge=near_edge,
                    bearing_optical_rad=geom.optical_bearing_rad,
                    bearing_base_rad=geom.base_bearing_rad,
                    position_base=geom.position_base,
                    geometry_valid=geom.geometry_valid,
                    geometry_source=geom.geometry_source,
                    geometry_error=geom.error_reason,
                )
                detected_persons.append(person)
        else:
            # Run ByteTrack with Ultralytics
            results = self.model.track(
                source=rgb,
                persist=True,
                verbose=False,
                device=self.device,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                tracker=self.tracker_config,
            )

            if results and len(results) > 0:
                res = results[0]
                boxes = res.boxes
                keypoints_data = res.keypoints

                if boxes is not None and len(boxes) > 0:
                    xyxy_arr = boxes.xyxy.cpu().numpy()
                    conf_arr = boxes.conf.cpu().numpy()
                    ids_arr = boxes.id.cpu().numpy() if boxes.id is not None else None

                    kp_data = keypoints_data.data.cpu().numpy() if keypoints_data is not None else None

                    for i in range(len(boxes)):
                        box = xyxy_arr[i].tolist()
                        conf = float(conf_arr[i])
                        x1, y1, x2, y2 = box

                        # Track ID
                        track_id = int(ids_arr[i]) if (ids_arr is not None and ids_arr[i] is not None) else None
                        is_confirmed = track_id is not None and track_id > 0
                        if is_confirmed:
                            current_track_ids.add(track_id)

                        # Keypoints
                        kp_list = []
                        if kp_data is not None and i < len(kp_data):
                            kp_list = kp_data[i].tolist()  # 17x3

                        # Center offset normalized to [-1.0, 1.0]
                        box_cx = (x1 + x2) / 2.0
                        center_offset_norm = float(np.clip((box_cx - (w / 2.0)) / (w / 2.0), -1.0, 1.0))

                        # Edge proximity check
                        near_edge = (
                            x1 < self.edge_margin_ratio * w or
                            x2 > (1.0 - self.edge_margin_ratio) * w or
                            y1 < self.edge_margin_ratio * h or
                            y2 > (1.0 - self.edge_margin_ratio) * h
                        )

                        # Measure torso depth if depth image is available
                        torso_depth_m, depth_valid = self._measure_torso_depth(
                            obs_frame, box, kp_list, h, w
                        )

                        intrinsics = CameraIntrinsics.from_camera_info(obs_frame.camera_info)
                        geom_engine = getattr(self, 'geometry', None) or RearCameraGeometry()
                        mount_tf = getattr(self, 'mount_tf', None)
                        geom = geom_engine.compute_geometry(
                            u=box_cx,
                            v=(y1 + y2) / 2.0,
                            torso_depth_m=torso_depth_m,
                            depth_valid=depth_valid,
                            intrinsics=intrinsics,
                            mount_tf=mount_tf,
                            frame_id=obs_frame.frame_id,
                            image_width=w,
                        )
                        person = PersonDetection(
                            bbox_xyxy=box,
                            confidence=conf,
                            keypoints=kp_list,
                            track_id=track_id,
                            is_confirmed=is_confirmed,
                            is_ambiguous=False,
                            torso_depth_m=torso_depth_m,
                            depth_valid=depth_valid,
                            center_offset_x_norm=center_offset_norm,
                            near_edge=near_edge,
                            bearing_optical_rad=geom.optical_bearing_rad,
                            bearing_base_rad=geom.base_bearing_rad,
                            position_base=geom.position_base,
                            geometry_valid=geom.geometry_valid,
                            geometry_source=geom.geometry_source,
                            geometry_error=geom.error_reason,
                        )
                        detected_persons.append(person)

        # Detect spatial ambiguity (overlapping bboxes with IoU > 0.35)
        self._flag_ambiguities(detected_persons)

        # Detect lost tracks (tracks active previously but missing now)
        lost_track_ids = list(self.previous_track_ids - current_track_ids)
        active_track_ids = list(current_track_ids)
        self.previous_track_ids = current_track_ids

        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        self.total_processed_frames += 1

        return MultiPersonObservation(
            stamp_sec=obs_frame.stamp_sec,
            stamp_nanosec=obs_frame.stamp_nanosec,
            receive_monotonic_time=obs_frame.receive_monotonic,
            frame_id=obs_frame.frame_id,
            source_mode=obs_frame.source_mode,
            source_session_id=getattr(obs_frame, 'source_session_id', ''),
            data_quality=obs_frame.data_quality,
            candidates=detected_persons,
            active_track_ids=active_track_ids,
            lost_track_ids=lost_track_ids,
            processing_time_ms=t_elapsed,
            device=self.device,
        )

    def _measure_torso_depth(
        self,
        obs_frame: ObservationFrame,
        box: List[float],
        keypoints: List[List[float]],
        h: int,
        w: int,
    ) -> Tuple[Optional[float], bool]:
        """Extract depth from torso ROI using Median Absolute Deviation (MAD) filtering.
        
        Depth reliability constraint:
        - If obs_frame.data_quality == VALID_RGBD: depth_valid is True when MAD converges.
        - If obs_frame.data_quality == UNVERIFIED_ALIGNMENT (or unverified/missing):
          torso_depth_m may contain raw measured depth for logging, but depth_valid
          MUST BE FALSE to prevent downstream nearest passenger selection from relying on it.
        """
        if not obs_frame.has_depth or obs_frame.depth is None:
            return None, False

        alignment_proven = (obs_frame.data_quality == DataQuality.VALID_RGBD.value)
        depth_img = obs_frame.depth
        x1, y1, x2, y2 = [int(v) for v in box]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))

        if x2 <= x1 or y2 <= y1:
            return None, False

        # Define torso ROI: inner 40% of bounding box
        roi_x1 = int(x1 + 0.3 * (x2 - x1))
        roi_x2 = int(x1 + 0.7 * (x2 - x1))
        roi_y1 = int(y1 + 0.2 * (y2 - y1))
        roi_y2 = int(y1 + 0.6 * (y2 - y1))

        roi_depth = depth_img[roi_y1:roi_y2, roi_x1:roi_x2]
        total_pixels = roi_depth.size
        if total_pixels == 0:
            return None, False

        # Filter valid depth: non-NaN, non-zero, within Astra sensor range [0.4m, 5.0m]
        valid_mask = np.isfinite(roi_depth) & (roi_depth >= 0.4) & (roi_depth <= 5.0)
        valid_pixels = roi_depth[valid_mask]

        # Valid depth pixel ratio in torso ROI must be >= 30%
        valid_ratio = valid_pixels.size / float(total_pixels)
        if valid_ratio < 0.30 or valid_pixels.size < 15:
            return None, False

        median_d = float(np.median(valid_pixels))
        mad = compute_mad(valid_pixels, median_d)

        # Reject if dispersion MAD exceeds 0.25m (indicates noisy edge or reflection)
        if mad > 0.25:
            return None, False

        # Only valid for nearest passenger selection if alignment is field-proven!
        depth_valid = alignment_proven
        return median_d, depth_valid

    def _flag_ambiguities(self, persons: List[PersonDetection]):
        """Check pairwise bbox IoU to detect crossing / occlusion ambiguities."""
        n = len(persons)
        for i in range(n):
            for j in range(i + 1, n):
                iou = self._compute_iou(persons[i].bbox, persons[j].bbox)
                if iou > 0.35:
                    persons[i].is_ambiguous = True
                    persons[j].is_ambiguous = True

    @staticmethod
    def _compute_iou(boxA: List[float], boxB: List[float]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        interArea = max(0.0, xB - xA) * max(0.0, yB - yA)
        boxAArea = max(0.0, boxA[2] - boxA[0]) * max(0.0, boxA[3] - boxA[1])
        boxBArea = max(0.0, boxB[2] - boxB[0]) * max(0.0, boxB[3] - boxB[1])
        denom = boxAArea + boxBArea - interArea
        return interArea / denom if denom > 0 else 0.0

    def draw_debug_overlay(
        self,
        image: np.ndarray,
        observation: MultiPersonObservation,
    ) -> np.ndarray:
        """Render bounding boxes, keypoints, IDs, depth, and status onto image."""
        overlay = image.copy()
        h, w = overlay.shape[:2]

        # Draw edge safety margin lines
        margin_x = int(self.edge_margin_ratio * w)
        margin_y = int(self.edge_margin_ratio * h)
        cv2.rectangle(
            overlay,
            (margin_x, margin_y),
            (w - margin_x, h - margin_y),
            (100, 100, 100),
            1,
            cv2.LINE_AA,
        )

        for person in observation.persons:
            x1, y1, x2, y2 = [int(v) for v in person.bbox]

            # Box color: Green for confirmed, Yellow for unconfirmed, Orange for ambiguous
            if person.is_ambiguous:
                color = (0, 165, 255)  # Orange
                status_str = "AMBIGUOUS"
            elif person.is_confirmed:
                color = (0, 255, 0)  # Green
                status_str = f"ID: {person.track_id}"
            else:
                color = (0, 255, 255)  # Yellow
                status_str = "UNCONFIRMED"

            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)

            # Keypoints rendering
            for kp in person.keypoints:
                kx, ky, kconf = kp
                if kconf > 0.4:
                    cv2.circle(overlay, (int(kx), int(ky)), 3, (255, 0, 0), -1)

            # Label text
            d_str = f"{person.torso_depth_m:.2f}m" if person.depth_valid else "no-depth"
            label = f"{status_str} | {d_str} | {person.confidence:.2f}"
            cv2.putText(
                overlay,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        # Header Info Banner
        fps = 1000.0 / max(1.0, observation.processing_time_ms)
        header = (
            f"[{observation.source_mode.upper()}] Dev: {observation.device.upper()} | "
            f"FPS: {fps:.1f} ({observation.processing_time_ms:.1f}ms) | "
            f"Tracks: {len(observation.active_track_ids)} active, {len(observation.lost_track_ids)} lost"
        )
        cv2.putText(
            overlay,
            header,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return overlay
