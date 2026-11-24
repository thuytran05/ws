#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 11 Hardware Profiler: Profile PersonDetectorNode Process CPU, RSS, VRAM, and Topic FPS.
Measures:
1. Exact process PID CPU % over a sustained window (30-60s)
2. RAM RSS (MB)
3. GPU VRAM (MB)
4. DDS topic observation rate (FPS) and latency
"""

import argparse
import json
import os
import subprocess
import sys
import time
from typing import Optional, List

try:
    import psutil
except ImportError:
    psutil = None

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def find_detector_pid() -> Optional[int]:
    """Find PID of running person_detector node."""
    if psutil is not None:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if 'person_detector' in cmdline and 'profile_detector_pid' not in cmdline:
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    # Fallback to pgrep
    try:
        out = subprocess.check_output(["pgrep", "-f", "person_detector"], text=True).strip().split()
        my_pid = os.getpid()
        for p in out:
            pid = int(p)
            if pid != my_pid:
                return pid
    except Exception:
        pass
    return None


def get_vram_for_pid(pid: int) -> float:
    """Query nvidia-smi for VRAM used by specific PID in MB."""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory", "--format=csv,noheader,nounits"],
            text=True, stderr=subprocess.DEVNULL
        ).strip().splitlines()
        for line in out:
            parts = [p.strip() for p in line.split(',')]
            if len(parts) >= 2 and int(parts[0]) == pid:
                return float(parts[1])
    except Exception:
        pass
    return 0.0


class ObservationListener(Node):
    def __init__(self):
        super().__init__('observation_profiler_listener')
        self.msg_count = 0
        self.latencies = []
        self.sub = self.create_subscription(String, '/tracking/observation', self._on_obs, 20)

    def _on_obs(self, msg: String):
        self.msg_count += 1
        t_recv = time.time()
        try:
            data = json.loads(msg.data)
            stamp = data.get('header', {}).get('stamp', {})
            sec = stamp.get('sec', 0)
            nanosec = stamp.get('nanosec', 0)
            t_pub = sec + nanosec * 1e-9
            if t_pub > 0:
                self.latencies.append((t_recv - t_pub) * 1000.0)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Profile PersonDetectorNode CPU, RSS, VRAM, and FPS over a sustained window")
    parser.add_argument("--pid", type=int, default=None, help="Target process PID (auto-detected if omitted)")
    parser.add_argument("--duration", type=float, default=30.0, help="Profiling window duration in seconds (default: 30.0s)")
    parser.add_argument("--interval", type=float, default=0.5, help="Sampling interval in seconds (default: 0.5s)")
    parser.add_argument("--domain", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)

    target_pid = args.pid or find_detector_pid()
    if target_pid is None:
        print("ERROR: Could not find running 'person_detector' node process! Please specify --pid <PID>.")
        sys.exit(1)

    print(f"Target PID identified: {target_pid}")
    if psutil is not None:
        proc = psutil.Process(target_pid)
        print(f"Target process name: {proc.name()}, cmdline: {' '.join(proc.cmdline()[:3])}...")
    else:
        print("WARNING: 'psutil' not installed; using standard /proc and ps fallback.")

    rclpy.init()
    listener = ObservationListener()

    cpu_samples: List[float] = []
    rss_samples: List[float] = []
    vram_samples: List[float] = []

    print(f"=== Profiling PID {target_pid} and /tracking/observation for {args.duration}s (interval {args.interval}s)... ===")

    t0 = time.time()
    next_sample = t0 + args.interval

    # Warmup cpu measurement
    if psutil is not None:
        try:
            proc.cpu_percent()
        except Exception:
            pass

    try:
        while time.time() - t0 < args.duration:
            rclpy.spin_once(listener, timeout_sec=0.05)

            now = time.time()
            if now >= next_sample:
                next_sample = now + args.interval
                if psutil is not None:
                    try:
                        cpu_pct = proc.cpu_percent()
                        mem_info = proc.memory_info()
                        rss_mb = mem_info.rss / (1024.0 * 1024.0)
                        cpu_samples.append(cpu_pct)
                        rss_samples.append(rss_mb)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        print(f"WARNING: PID {target_pid} terminated during profiling!")
                        break
                else:
                    # Fallback via ps
                    try:
                        ps_out = subprocess.check_output(["ps", "-p", str(target_pid), "-o", "%cpu,rss", "--no-headers"], text=True).strip().split()
                        if len(ps_out) >= 2:
                            cpu_samples.append(float(ps_out[0]))
                            rss_samples.append(float(ps_out[1]) / 1024.0)
                    except Exception:
                        break

                vram_mb = get_vram_for_pid(target_pid)
                vram_samples.append(vram_mb)

    except KeyboardInterrupt:
        pass

    duration = time.time() - t0
    effective_fps = listener.msg_count / duration if duration > 0 else 0.0

    print("\n================================================================================")
    print("PHASE 11: SUSTAINED DETECTOR PROFILING REPORT")
    print("================================================================================")
    print(f"Target PID:                  {target_pid}")
    print(f"Profiling Duration:          {duration:.2f} s ({len(cpu_samples)} resource samples)")
    print(f"Observation Frames Received: {listener.msg_count}")
    print(f"Effective Throughput:        {effective_fps:.2f} FPS")

    if cpu_samples:
        mean_cpu = sum(cpu_samples) / len(cpu_samples)
        max_cpu = max(cpu_samples)
        min_cpu = min(cpu_samples)
        cpu_samples_sorted = sorted(cpu_samples)
        p95_cpu = cpu_samples_sorted[int(len(cpu_samples) * 0.95)]
        print(f"CPU Usage (%):               mean={mean_cpu:.1f}%, min={min_cpu:.1f}%, max={max_cpu:.1f}%, p95={p95_cpu:.1f}%")
    else:
        print("CPU Usage (%):               No samples collected.")

    if rss_samples:
        mean_rss = sum(rss_samples) / len(rss_samples)
        max_rss = max(rss_samples)
        print(f"RAM RSS Memory:              mean={mean_rss:.1f} MB, max={max_rss:.1f} MB (delta={rss_samples[-1] - rss_samples[0]:+.2f} MB)")
    else:
        print("RAM RSS Memory:              No samples collected.")

    if vram_samples and any(v > 0 for v in vram_samples):
        mean_vram = sum(vram_samples) / len(vram_samples)
        max_vram = max(vram_samples)
        print(f"GPU VRAM Memory:             mean={mean_vram:.1f} MB, max={max_vram:.1f} MB")
    else:
        print("GPU VRAM Memory:             Not reported by nvidia-smi (CPU mode or shared VRAM)")

    if listener.latencies:
        mean_lat = sum(listener.latencies) / len(listener.latencies)
        lat_sorted = sorted(listener.latencies)
        p50_lat = lat_sorted[int(len(lat_sorted) * 0.50)]
        p95_lat = lat_sorted[int(len(lat_sorted) * 0.95)]
        print(f"One-Way Transport Latency:   p50={p50_lat:.2f} ms, p95={p95_lat:.2f} ms, mean={mean_lat:.2f} ms")
    else:
        print("One-Way Transport Latency:   No paired timestamps.")

    print("--------------------------------------------------------------------------------")
    print("PROFILING COMPLETE.")


if __name__ == '__main__':
    main()
