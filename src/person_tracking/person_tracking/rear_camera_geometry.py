#!/usr/bin/env python3
"""Rear Camera Geometry & Spatial Projection for Person Tracking Phase 5.

Implements ADR-10 (Rear Camera Coordinate Conventions):
- Decouples normalized pixel offset (center_offset_x_norm) from optical bearing (bearing_optical_rad).
- Projects target to 3D optical coordinates using valid intrinsics and depth.
- Transforms 3D position to base_link coordinates using mounting transform.
- Calculates robot base bearing (bearing_base_rad).
- Evaluates target on image left, center, right with signed angle verification.
- Robust against missing TF, frame ID mismatch, and invalid intrinsics.
- STRICTLY ZERO motion output: NO velocity commands, NO velocity command publishers.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple
import numpy as np
from sensor_msgs.msg import CameraInfo


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsic parameters."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, info: Optional[CameraInfo]) -> Optional['CameraIntrinsics']:
        """Extract and validate intrinsics from sensor_msgs/CameraInfo."""
        if info is None:
            return None
        if len(info.k) < 9:
            return None
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        w = int(info.width)
        h = int(info.height)

        if fx <= 0.0 or fy <= 0.0 or w <= 0 or h <= 0:
            return None
        if not np.isfinite([fx, fy, cx, cy]).all():
            return None

        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h)


@dataclass
class CameraMountTransform:
    """Rigid transform from camera optical frame to robot base_link frame."""
    translation: np.ndarray  # [x, y, z] in meters
    rotation_matrix: np.ndarray  # 3x3 orthonormal rotation matrix
    parent_frame: str = "base_link"
    child_frame: str = "camera_color_optical_frame"

    def transform_point(self, p_opt: np.ndarray) -> np.ndarray:
        """Transform 3D point from optical frame to base_link frame: P_base = R * P_opt + T."""
        return self.rotation_matrix @ p_opt + self.translation

    @classmethod
    def create_synthetic_rear_mount(
        cls,
        translation: Tuple[float, float, float] = (-0.20, 0.0, 0.25),
        child_frame: str = "camera_color_optical_frame",
    ) -> 'CameraMountTransform':
        """Create rear camera mounting transform per ADR-10.
        
        Mounting: Camera faces backward (yaw = pi relative to base_link).
        Optical frame: +X right, +Y down, +Z out of lens.
        base_link: +X forward, +Y left, +Z up.
        
        Transformation:
          X_base = -Z_opt + x_offset
          Y_base =  X_opt + y_offset
          Z_base = -Y_opt + z_offset
          
        Rotation matrix R (from optical to base):
          [  0,  0, -1 ]
          [  1,  0,  0 ]
          [  0, -1,  0 ]
        det(R) = +1, orthonormal.
        """
        r_mat = np.array([
            [0.0,  0.0, -1.0],
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
        ], dtype=np.float64)

        t_vec = np.array(translation, dtype=np.float64)
        return cls(translation=t_vec, rotation_matrix=r_mat, child_frame=child_frame)


@dataclass
class TargetGeometryResult:
    """Geometric projection output for a detected target candidate."""
    center_offset_x_norm: float  # [-1.0, 1.0]
    optical_bearing_rad: Optional[float] = None  # Signed angle in optical frame
    base_bearing_rad: Optional[float] = None  # Signed angle in base_link frame [-pi, pi]
    position_optical: Optional[List[float]] = None  # [X_opt, Y_opt, Z_opt] meters
    position_base: Optional[List[float]] = None  # [X_base, Y_base, Z_base] meters
    is_metric_valid: bool = False
    error_reason: str = ""


class RearCameraGeometry:
    """Geometric projection engine for rear-facing camera."""

    def __init__(self, expected_frame_id: str = "camera_color_optical_frame"):
        self.expected_frame_id = expected_frame_id

    @staticmethod
    def compute_center_offset_norm(u: float, width: int) -> float:
        """Compute normalized pixel center offset in [-1.0, 1.0].
        
        -1.0: far left of image
         0.0: horizontal center
        +1.0: far right of image
        """
        if width <= 0:
            return 0.0
        center_x = width / 2.0
        return float(np.clip((u - center_x) / center_x, -1.0, 1.0))

    @staticmethod
    def compute_optical_bearing(u: float, intrinsics: Optional[CameraIntrinsics]) -> Optional[float]:
        """Compute optical bearing angle: theta_opt = arctan2(u - cx, fx)."""
        if intrinsics is None or intrinsics.fx <= 0:
            return None
        return float(math.atan2(u - intrinsics.cx, intrinsics.fx))

    def compute_geometry(
        self,
        u: float,
        v: float,
        torso_depth_m: Optional[float],
        depth_valid: bool,
        intrinsics: Optional[CameraIntrinsics],
        mount_tf: Optional[CameraMountTransform],
        frame_id: str = "camera_color_optical_frame",
        image_width: int = 640,
    ) -> TargetGeometryResult:
        """Perform full geometric projection from 2D pixel to 3D base_link frame."""
        # 1. 2D Normalized Pixel Offset (always computable)
        w = intrinsics.width if intrinsics is not None else image_width
        center_offset_x = self.compute_center_offset_norm(u, w)

        # 2. Optical Bearing (requires valid intrinsics)
        optical_bearing = self.compute_optical_bearing(u, intrinsics)
        if intrinsics is None:
            return TargetGeometryResult(
                center_offset_x_norm=center_offset_x,
                optical_bearing_rad=None,
                is_metric_valid=False,
                error_reason="INVALID_OR_MISSING_CAMERA_INFO",
            )

        # 3. Frame ID check
        if frame_id != self.expected_frame_id:
            return TargetGeometryResult(
                center_offset_x_norm=center_offset_x,
                optical_bearing_rad=optical_bearing,
                is_metric_valid=False,
                error_reason=f"FRAME_ID_MISMATCH: expected '{self.expected_frame_id}', got '{frame_id}'",
            )

        # 4. TF check
        if mount_tf is None:
            return TargetGeometryResult(
                center_offset_x_norm=center_offset_x,
                optical_bearing_rad=optical_bearing,
                is_metric_valid=False,
                error_reason="MISSING_TF_TRANSFORM",
            )

        # 5. Depth validity check
        if not depth_valid or torso_depth_m is None or torso_depth_m <= 0.0 or not np.isfinite(torso_depth_m):
            return TargetGeometryResult(
                center_offset_x_norm=center_offset_x,
                optical_bearing_rad=optical_bearing,
                is_metric_valid=False,
                error_reason="UNVERIFIED_OR_INVALID_DEPTH",
            )

        # 6. 3D Optical projection
        z_opt = float(torso_depth_m)
        x_opt = float((u - intrinsics.cx) * z_opt / intrinsics.fx)
        y_opt = float((v - intrinsics.cy) * z_opt / intrinsics.fy)
        p_opt = np.array([x_opt, y_opt, z_opt], dtype=np.float64)

        # 7. Transform to base_link
        p_base = mount_tf.transform_point(p_opt)
        x_base = float(p_base[0])
        y_base = float(p_base[1])
        z_base = float(p_base[2])

        # 8. Base Bearing
        base_bearing = float(math.atan2(y_base, x_base))

        return TargetGeometryResult(
            center_offset_x_norm=center_offset_x,
            optical_bearing_rad=optical_bearing,
            base_bearing_rad=base_bearing,
            position_optical=[x_opt, y_opt, z_opt],
            position_base=[x_base, y_base, z_base],
            is_metric_valid=True,
            error_reason="",
        )
