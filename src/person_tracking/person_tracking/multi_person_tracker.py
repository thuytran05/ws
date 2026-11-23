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

from person_tracking.source_adapters import ObservationFrame, compute_mad


@dataclass
class PersonDetection:
    """Normalized representation of a single detected person."""
    bbox: List[float]  # [x1, y1, x2, y2] pixel coordinates
    confidence: float
    keypoints: List[List[float]]  # 17 keypoints: [x, y, conf]
    track_id: Optional[int] = None  # None or -1 if unconfirmed
    is_confirmed: bool = False  # True if tracker assigned persistent positive ID
    is_ambiguous: bool = False  # True if overlapping bboxes or identity ambiguity
    torso_depth_m: Optional[float] = None
    depth_valid: bool = False
    center_offset_x_norm: float = 0.0  # -1.0 (far left) to 1.0 (far right)
    near_edge: bool = False  # True if bbox touches outer margin


@dataclass
class MultiPersonObservation:
    """Perception output for a single video/sensor frame."""
    stamp_sec: int
    stamp_nanosec: int
    receive_monotonic: float
    frame_id: str
    source_mode: str
    data_quality: str
    persons: List[PersonDetection] = field(default_factory=list)
    active_track_ids: List[int] = field(default_factory=list)
    lost_track_ids: List[int] = field(default_factory=list)
    processing_time_ms: float = 0.0
    device: str = "cpu"


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

    def reset(self):
        """Reset tracker state on source switch, video loop, or stream reset."""
        self.previous_track_ids.clear()
        # Re-initialize tracker predictor in Ultralytics to clear ByteTrack tracks
        if hasattr(self.model, 'predictor') and self.model.predictor is not None:
            if hasattr(self.model.predictor, 'trackers'):
                self.model.predictor.trackers = []
        self.total_processed_frames = 0

    def process_frame(self, obs_frame: ObservationFrame) -> MultiPersonObservation:
        """Run pose inference and tracking on an ObservationFrame."""
        t_start = time.perf_counter()

        rgb = obs_frame.rgb
        h, w = rgb.shape[:2]

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

        detected_persons: List[PersonDetection] = []
        current_track_ids = set()

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

                    person = PersonDetection(
                        bbox=box,
                        confidence=conf,
                        keypoints=kp_list,
                        track_id=track_id,
                        is_confirmed=is_confirmed,
                        is_ambiguous=False,
                        torso_depth_m=torso_depth_m,
                        depth_valid=depth_valid,
                        center_offset_x_norm=center_offset_norm,
                        near_edge=near_edge,
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
            receive_monotonic=obs_frame.receive_monotonic,
            frame_id=obs_frame.frame_id,
            source_mode=obs_frame.source_mode,
            data_quality=obs_frame.data_quality,
            persons=detected_persons,
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
        """Extract depth from torso ROI using Median Absolute Deviation (MAD) filtering."""
        if not obs_frame.has_depth or obs_frame.depth is None:
            return None, False

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
        # Filter valid depth: non-NaN, non-zero, within Astra sensor range [0.4m, 5.0m]
        valid_mask = np.isfinite(roi_depth) & (roi_depth >= 0.4) & (roi_depth <= 5.0)
        valid_pixels = roi_depth[valid_mask]

        if valid_pixels.size < 15:
            return None, False

        median_d = float(np.median(valid_pixels))
        mad = compute_mad(valid_pixels, median_d)

        # Reject if dispersion MAD exceeds 0.25m (indicates noisy edge or reflection)
        if mad > 0.25:
            return None, False

        return median_d, True

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
