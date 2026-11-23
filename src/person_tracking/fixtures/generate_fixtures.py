#!/usr/bin/env python3
"""Generate deterministic test fixtures and synthetic video for person tracking."""

import json
import os
import cv2
import numpy as np


def generate_all_fixtures(output_dir: str):
    os.makedirs(output_dir, exist_ok=True)
    metadata = {
        "version": "1.0",
        "description": "Deterministic test fixtures for person_tracking Phase 2",
        "note": "All data generated synthetically via algorithms; contains NO real Astra or human recordings.",
        "fixtures": {}
    }

    # 1. Empty Scene (T01 adapter check)
    empty_rgb = np.full((480, 640, 3), 120, dtype=np.uint8)
    # Add subtle grid lines for visual reference
    for y in range(0, 480, 40):
        cv2.line(empty_rgb, (0, y), (640, y), (140, 140, 140), 1)
    for x in range(0, 640, 40):
        cv2.line(empty_rgb, (x, 0), (x, 480), (140, 140, 140), 1)
    empty_depth_m = np.full((480, 640), 3.0, dtype=np.float32)
    empty_depth_raw = (empty_depth_m * 1000.0).astype(np.uint16)

    empty_path = os.path.join(output_dir, "synthetic_empty.npz")
    np.savez_compressed(
        empty_path,
        rgb=empty_rgb,
        depth_m=empty_depth_m,
        depth_raw=empty_depth_raw
    )
    metadata["fixtures"]["synthetic_empty.npz"] = {
        "type": "synthetic_npz",
        "resolution": [640, 480],
        "content": "Empty scene with uniform background at 3.0m depth",
        "expected_quality": "VALID_RGBD",
        "test_case": "T01_adapter"
    }

    # 2. Single Person Scene
    person_rgb = empty_rgb.copy()
    # Draw torso silhouette at (220, 100) to (420, 420)
    cv2.rectangle(person_rgb, (220, 100), (420, 420), (50, 120, 220), -1)
    # Draw head circle
    cv2.circle(person_rgb, (320, 70), 30, (50, 120, 220), -1)

    person_depth_m = np.full((480, 640), 3.5, dtype=np.float32)
    person_depth_m[100:420, 220:420] = 1.50
    # Head depth
    cv2.circle(person_depth_m, (320, 70), 30, 1.50, -1)
    person_depth_raw = (person_depth_m * 1000.0).astype(np.uint16)

    person_path = os.path.join(output_dir, "synthetic_person_1.npz")
    np.savez_compressed(
        person_path,
        rgb=person_rgb,
        depth_m=person_depth_m,
        depth_raw=person_depth_raw
    )
    metadata["fixtures"]["synthetic_person_1.npz"] = {
        "type": "synthetic_npz",
        "resolution": [640, 480],
        "content": "Synthetic person silhouette at 1.5m depth with 3.5m background",
        "expected_quality": "VALID_RGBD",
        "test_case": "T09_normal_escort"
    }

    # 3. Corrupted Depth Scene (NaN, 0.0, outlier noise)
    corrupted_rgb = person_rgb.copy()
    corrupted_depth_m = person_depth_m.copy()
    # Inject holes and zeros
    corrupted_depth_m[120:250, 240:400] = np.nan
    corrupted_depth_m[250:350, 240:400] = 0.0
    corrupted_depth_raw = np.nan_to_num(corrupted_depth_m * 1000.0, nan=0.0).astype(np.uint16)

    corrupted_path = os.path.join(output_dir, "synthetic_corrupted_depth.npz")
    np.savez_compressed(
        corrupted_path,
        rgb=corrupted_rgb,
        depth_m=corrupted_depth_m,
        depth_raw=corrupted_depth_raw
    )
    metadata["fixtures"]["synthetic_corrupted_depth.npz"] = {
        "type": "synthetic_npz",
        "resolution": [640, 480],
        "content": "Person scene with extensive central NaN and 0.0 depth dropouts",
        "expected_quality": "INVALID_DEPTH",
        "test_case": "T19_invalid_depth"
    }

    # 4. Deterministic MP4 Video Replay Fixture
    video_path = os.path.join(output_dir, "synthetic_video.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 15.0
    num_frames = 30
    writer = cv2.VideoWriter(video_path, fourcc, fps, (640, 480))

    for f_idx in range(num_frames):
        frame = np.full((480, 640, 3), 150, dtype=np.uint8)
        # Moving box horizontally from x=150 to x=350
        x_center = int(150 + (350 - 150) * (f_idx / float(num_frames - 1)))
        y_top = 100
        cv2.rectangle(frame, (x_center - 80, y_top), (x_center + 80, y_top + 280), (60, 140, 240), -1)
        cv2.circle(frame, (x_center, y_top - 30), 25, (60, 140, 240), -1)
        # Frame counter text
        cv2.putText(frame, f"Frame {f_idx+1}/{num_frames}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        writer.write(frame)
    writer.release()

    metadata["fixtures"]["synthetic_video.mp4"] = {
        "type": "mp4_video",
        "resolution": [640, 480],
        "fps": 15.0,
        "frame_count": 30,
        "duration_sec": 2.0,
        "content": "Deterministic 30-frame video of moving silhouette for video replay testing",
        "expected_quality": "RGB_ONLY",
        "test_case": "video_replay_adapter"
    }

    metadata_path = os.path.join(output_dir, "metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"Successfully generated all fixtures in {output_dir}")


if __name__ == "__main__":
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    generate_all_fixtures(out_dir)
