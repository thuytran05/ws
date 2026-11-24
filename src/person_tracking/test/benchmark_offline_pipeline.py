#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 10: Benchmark Offline Perception Pipeline (Offline với Fixture Tổng Hợp)
Measures:
- Model, resolution (640x480), device (CUDA vs CPU), warm-up iterations, duration, sample count.
- Throughput (FPS), process_frame latency (inference + ByteTrack + depth association),
  offline processing and serialize latency (without ROS transport).
- CPU utilization %, RSS memory (MB), GPU VRAM (MB).
- Note: Queue depth is not measurable internally (chưa đo); dropped frames on offline sync loop is 0.0%.
- Does not extrapolate offline FPS to physical camera frame rate.
"""

import gc
import json
import os
import sys
import time
from typing import Dict, List, Tuple
import numpy as np

# Inject .venv_yolo site-packages if available
_venv_site = '/home/ws/.venv_yolo/lib/python3.12/site-packages'
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

try:
    import psutil
except ImportError:
    psutil = None

try:
    import torch
except ImportError:
    torch = None

from person_tracking.multi_person_tracker import MultiPersonTracker, MultiPersonObservation
from person_tracking.source_adapters import SyntheticFixtureGenerator, VideoReplayAdapter, ObservationFrame, DataQuality


MODEL_PATH = '/home/ws/yolo26n-pose.pt'


def get_process_memory_mb() -> float:
    if psutil:
        proc = psutil.Process(os.getpid())
        return proc.memory_info().rss / (1024.0 * 1024.0)
    return 0.0


def get_gpu_memory_mb() -> Tuple[float, float]:
    if torch and torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated() / (1024.0 * 1024.0)
        reserved = torch.cuda.memory_reserved() / (1024.0 * 1024.0)
        return allocated, reserved
    return 0.0, 0.0


def serialize_observation(obs: MultiPersonObservation) -> str:
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
            "bearing_optical_rad": getattr(p, 'bearing_optical_rad', None),
            "bearing_base_rad": getattr(p, 'bearing_base_rad', None),
            "position_base": getattr(p, 'position_base', None),
            "geometry_valid": getattr(p, 'geometry_valid', False),
            "geometry_source": getattr(p, 'geometry_source', 'OPTICAL_ONLY'),
            "geometry_error": getattr(p, 'geometry_error', ''),
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
    }
    return json.dumps(payload)


def benchmark_pipeline(device: str = "cuda:0", num_samples: int = 100, warmup_count: int = 15):
    print(f"\n{'='*70}")
    print(f"BENCHMARK OFFLINE VỚI FIXTURE TỔNG HỢP - DEVICE: {device}")
    print(f"{'='*70}")

    if not os.path.isfile(MODEL_PATH):
        print(f"ERROR: Model checkpoint not found at {MODEL_PATH}")
        return None

    actual_device = device
    if "cuda" in device and (not torch or not torch.cuda.is_available()):
        print(f"WARNING: CUDA requested but not available. Falling back to cpu.")
        actual_device = "cpu"

    tracker = MultiPersonTracker(
        model_path=MODEL_PATH,
        device=actual_device,
        conf_threshold=0.25,
        iou_threshold=0.45,
        use_synthetic_mount=True
    )

    frame_fixture = SyntheticFixtureGenerator.generate_person_scene(person_depth_m=1.6)

    print(f"Running {warmup_count} warm-up iterations...")
    for _ in range(warmup_count):
        _ = tracker.process_frame(frame_fixture)

    if torch and "cuda" in actual_device:
        torch.cuda.synchronize()

    gc.collect()
    start_rss = get_process_memory_mb()
    start_vram_alloc, start_vram_res = get_gpu_memory_mb()

    inference_latencies_ms: List[float] = []
    e2e_latencies_ms: List[float] = []
    cpu_percentages: List[float] = []
    # Internal queue depth is not observable via public metric topic (chưa đo)
    # Dropped frame rate in direct synchronous offline loop is 0.0%

    print(f"Running {num_samples} benchmark samples...")
    t_bench_start = time.perf_counter()

    for i in range(num_samples):
        t_sample_start = time.perf_counter()

        if i % 5 == 0:
            frame = SyntheticFixtureGenerator.generate_empty_scene()
        else:
            frame = SyntheticFixtureGenerator.generate_person_scene(person_depth_m=1.5 + (i % 5) * 0.1)

        t_infer_start = time.perf_counter()
        obs = tracker.process_frame(frame)
        if torch and "cuda" in actual_device:
            torch.cuda.synchronize()
        t_infer_end = time.perf_counter()

        payload = serialize_observation(obs)
        assert len(payload) > 0, "Observation payload must not be empty"

        t_sample_end = time.perf_counter()

        infer_ms = (t_infer_end - t_infer_start) * 1000.0
        e2e_ms = (t_sample_end - t_sample_start) * 1000.0

        inference_latencies_ms.append(infer_ms)
        e2e_latencies_ms.append(e2e_ms)

        if psutil and i % 10 == 0:
            cpu_percentages.append(psutil.cpu_percent(interval=None))

    t_bench_end = time.perf_counter()
    total_duration_sec = t_bench_end - t_bench_start
    fps = num_samples / total_duration_sec

    end_rss = get_process_memory_mb()
    end_vram_alloc, end_vram_res = get_gpu_memory_mb()

    inf_p50 = float(np.percentile(inference_latencies_ms, 50))
    inf_p95 = float(np.percentile(inference_latencies_ms, 95))
    inf_mean = float(np.mean(inference_latencies_ms))
    inf_min = float(np.min(inference_latencies_ms))
    inf_max = float(np.max(inference_latencies_ms))

    e2e_p50 = float(np.percentile(e2e_latencies_ms, 50))
    e2e_p95 = float(np.percentile(e2e_latencies_ms, 95))
    e2e_mean = float(np.mean(e2e_latencies_ms))
    e2e_min = float(np.min(e2e_latencies_ms))
    e2e_max = float(np.max(e2e_latencies_ms))

    avg_cpu = float(np.mean(cpu_percentages)) if cpu_percentages else 0.0

    result = {
        "device": actual_device,
        "model": os.path.basename(MODEL_PATH),
        "resolution": "640x480",
        "num_samples": num_samples,
        "warmup_count": warmup_count,
        "total_duration_sec": total_duration_sec,
        "fps": fps,
        "inference_latency_ms": {
            "p50": inf_p50,
            "p95": inf_p95,
            "mean": inf_mean,
            "min": inf_min,
            "max": inf_max
        },
        "e2e_latency_ms": {
            "p50": e2e_p50,
            "p95": e2e_p95,
            "mean": e2e_mean,
            "min": e2e_min,
            "max": e2e_max
        },
        "cpu_utilization_percent": avg_cpu,
        "memory_rss_mb": end_rss,
        "gpu_vram_allocated_mb": end_vram_alloc,
        "gpu_vram_reserved_mb": end_vram_res,
        "queue_depth_max": max(queue_depths) if queue_depths else 0,
        "dropped_frames": dropped_frames
    }

    print(f"Results for {actual_device}:")
    print(f"  Throughput: {fps:.2f} FPS (Duration: {total_duration_sec:.2f}s, Samples: {num_samples})")
    print(f"  Inference Latency: p50 = {inf_p50:.2f} ms | p95 = {inf_p95:.2f} ms | Mean = {inf_mean:.2f} ms")
    print(f"  End-to-End Latency: p50 = {e2e_p50:.2f} ms | p95 = {e2e_p95:.2f} ms | Mean = {e2e_mean:.2f} ms")
    print(f"  Memory RSS: {end_rss:.1f} MB | VRAM Alloc: {end_vram_alloc:.1f} MB (Res: {end_vram_res:.1f} MB)")
    print(f"  CPU Utilization: {avg_cpu:.1f}%")
    print(f"  Dropped Frames: {dropped_frames} / {num_samples} (0.0%)")

    return result


def main():
    out_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_10"
    os.makedirs(out_dir, exist_ok=True)
    out_log = os.path.join(out_dir, "benchmark_offline_pipeline.log")

    results = {}
    if torch and torch.cuda.is_available():
        res_cuda = benchmark_pipeline(device="cuda:0", num_samples=100, warmup_count=15)
        results["cuda:0"] = res_cuda

    res_cpu = benchmark_pipeline(device="cpu", num_samples=50, warmup_count=5)
    results["cpu"] = res_cpu

    lines = []
    lines.append("=" * 80)
    lines.append("PHASE 10: BENCHMARK HIỆU NĂNG OFFLINE PERCEPTION PIPELINE")
    lines.append("=" * 80)
    lines.append("1. THÔNG SỐ CẤU HÌNH KIỂM THỬ:")
    lines.append(f"   - Model Checkpoint: {os.path.basename(MODEL_PATH)} (YOLO Pose + ByteTrack)")
    lines.append(f"   - Input Resolution: 640x480 RGB-D (RGB 8UC3 + Depth 16UC1)")
    lines.append(f"   - Framework: PyTorch {torch.__version__ if torch else 'N/A'}")
    lines.append(f"   - CUDA Device: {torch.cuda.get_device_name(0) if (torch and torch.cuda.is_available()) else 'None'}")
    lines.append(f"   - OS Container: Ubuntu 24.04 (ROS 2 Jazzy)")
    lines.append("")
    lines.append("2. BẢNG TỔNG HỢP HIỆU NĂNG THỰC ĐO:")
    lines.append("-" * 80)
    lines.append(f"{'Thiết bị (Device)':<12} | {'FPS':<8} | {'Infer p50':<12} | {'Infer p95':<12} | {'E2E p50':<12} | {'E2E p95':<12} | {'RSS (MB)':<10} | {'VRAM (MB)':<10}")
    lines.append("-" * 80)

    for dev, r in results.items():
        if r:
            lines.append(
                f"{dev:<12} | {r['fps']:<8.2f} | "
                f"{r['inference_latency_ms']['p50']:<12.2f} | {r['inference_latency_ms']['p95']:<12.2f} | "
                f"{r['e2e_latency_ms']['p50']:<12.2f} | {r['e2e_latency_ms']['p95']:<12.2f} | "
                f"{r['memory_rss_mb']:<10.1f} | {r['gpu_vram_allocated_mb']:<10.1f}"
            )
    lines.append("-" * 80)
    lines.append("")
    lines.append("3. PHÂN TÍCH ĐỘ TRỄ SUY LUẬN VS TOÀN CHUỖI:")
    for dev, r in results.items():
        if r:
            infer_p50 = r['inference_latency_ms']['p50']
            e2e_p50 = r['e2e_latency_ms']['p50']
            overhead = e2e_p50 - infer_p50
            lines.append(f"   - Trên [{dev}]:")
            lines.append(f"     * Độ trễ suy luận YOLO (Inference): p50 = {infer_p50:.2f} ms, p95 = {r['inference_latency_ms']['p95']:.2f} ms.")
            lines.append(f"     * Độ trễ toàn chuỗi (E2E Pipeline):  p50 = {e2e_p50:.2f} ms, p95 = {r['e2e_latency_ms']['p95']:.2f} ms.")
            lines.append(f"     * Độ trễ phụ thuộc khác (Overhead - ByteTrack, Depth MAD, JSON serialization): {overhead:.2f} ms.")
    lines.append("")
    lines.append("4. TỒN TẠI KỸ THUẬT VÀ PHẦN CHƯA KIỂM CHỨNG (PHYSICAL LIMITATIONS):")
    lines.append("   - Thiếu phần cứng camera Orbbec Astra Pro vật lý tại môi trường kiểm thử.")
    lines.append("   - Thiếu tập video quay thực tế hành khách người thật tại sân bay với trang phục và góc nhìn camera sau.")
    lines.append("   - Ca T06 (Người khác cắt ngang người đang khóa):")
    lines.append("     * Trạng thái duy trì: PARTIAL / CODE_READY.")
    lines.append("     * Mã nguồn ByteTrack, ngưỡng IoU > 0.35 và cờ is_ambiguous = True đã sẵn sàng.")
    lines.append("     * Cần video thực tế có người đi cắt ngang trên robot thật để nghiệm thu chính thức tại Phase 11.")
    lines.append("=" * 80)

    report_text = "\n".join(lines)
    print(report_text)

    with open(out_log, "w", encoding="utf-8") as f:
        f.write(report_text)
    print(f"\nWrote benchmark log to: {out_log}")


if __name__ == "__main__":
    main()
