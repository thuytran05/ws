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
from typing import List, Optional, Tuple

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
from person_tracking.debug_visualization import DebugOverlayContext, DebugVisualizer
from person_tracking.rear_camera_geometry import (
    RearCameraGeometry,
    CameraIntrinsics,
    CameraMountTransform,
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
    near_edge: bool = False  # True if passenger has horizontal edge risk (OUT_OF_FOV risk)
    near_horizontal_edge: bool = False  # Explicit alias for horizontal edge risk
    bbox_clipped: bool = False  # True if bbox touches outer 8% margin on X or Y
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
        self.visualizer = DebugVisualizer()

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
                near_edge, near_horiz, bbox_clipped = self._evaluate_edge_flags(
                    x1, y1, x2, y2, w, h, center_offset_norm
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
                    near_horizontal_edge=near_horiz,
                    bbox_clipped=bbox_clipped,
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

                        # Edge proximity check (horizontal risk decoupled from bbox clipping)
                        near_edge, near_horiz, bbox_clipped = self._evaluate_edge_flags(
                            x1, y1, x2, y2, w, h, center_offset_norm
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
                            near_horizontal_edge=near_horiz,
                            bbox_clipped=bbox_clipped,
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

    def _evaluate_edge_flags(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        w: int,
        h: int,
        center_offset_norm: float,
        horizontal_offset_threshold: float = 0.70,
    ) -> Tuple[bool, bool, bool]:
        """Separate bbox clipping from horizontal edge risk.

        Returns (near_edge, near_horizontal_edge, bbox_clipped).
        - bbox_clipped: bbox touches outer edge_margin_ratio (8%) on X or Y.
        - near_horizontal_edge / near_edge: passenger is laterally near left/right edge
          (|offset| > 0.70, or shifted >= 0.35 toward the touched horizontal margin).
          Never triggers solely from top/bottom clipping or a wide centered bbox.
        """
        mx = self.edge_margin_ratio * w
        my = self.edge_margin_ratio * h
        bbox_clipped = (x1 < mx or x2 > (w - mx) or y1 < my or y2 > (h - my))
        near_horizontal_edge = (
            abs(center_offset_norm) > horizontal_offset_threshold
            or (center_offset_norm < -0.35 and x1 < mx)
            or (center_offset_norm > 0.35 and x2 > (w - mx))
        )
        return near_horizontal_edge, near_horizontal_edge, bbox_clipped

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
        context: Optional[DebugOverlayContext] = None,
    ) -> np.ndarray:
        """Render bounding boxes, COCO 17 skeleton, IDs, depth, zones, and status banner."""
        if context is None:
            context = DebugOverlayContext(edge_margin_ratio=self.edge_margin_ratio)
        return self.visualizer.render(image, observation, context=context)
