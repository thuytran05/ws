#!/usr/bin/env python3
"""Measure steady-state ROS 2 pipeline FPS, inter-arrival gaps, frame drops, and PID CPU/RAM."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PipelineProfiler(Node):
    def __init__(self, warmup_sec: float = 5.0, measure_sec: float = 30.0, subscribe_debug: bool = True, img_qos_rel: str = "best_effort"):
        super().__init__("pipeline_profiler")
        self.t_start = time.monotonic()
        self.warmup_sec = warmup_sec
        self.measure_sec = measure_sec

        self.rgb_times = []
        self.rgb_stamps = set()
        self.obs_times = []
        self.obs_stamps = set()
        self.obs_proc_ms = []
        self.dbg_times = []
        self.dbg_stamps = set()
        self.status_times = []

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        img_qos = (
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=2,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            if img_qos_rel == "reliable"
            else qos_profile_sensor_data
        )
        self.create_subscription(Image, "/camera/color/image_raw", self._on_rgb, img_qos)
        self.create_subscription(String, "/tracking/observation", self._on_obs, reliable_qos)
        self.create_subscription(String, "/tracking/source_status", self._on_status, reliable_qos)
        if subscribe_debug:
            self.create_subscription(Image, "/tracking/debug_image", self._on_dbg, img_qos)

    def _in_window(self, now_mono: float) -> bool:
        dt = now_mono - self.t_start
        return self.warmup_sec <= dt <= (self.warmup_sec + self.measure_sec)

    def _on_rgb(self, msg: Image):
        now = time.monotonic()
        if self._in_window(now):
            self.rgb_times.append(now)
            self.rgb_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))

    def _on_obs(self, msg: String):
        now = time.monotonic()
        if self._in_window(now):
            self.obs_times.append(now)
            data = json.loads(msg.data)
            st = data.get("stamp", {})
            self.obs_stamps.add((st.get("sec", 0), st.get("nanosec", 0)))
            self.obs_proc_ms.append(float(data.get("processing_time_ms", 0.0)))

    def _on_dbg(self, msg: Image):
        now = time.monotonic()
        if self._in_window(now):
            self.dbg_times.append(now)
            self.dbg_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))

    def _on_status(self, msg: String):
        now = time.monotonic()
        if self._in_window(now):
            self.status_times.append(now)


def _stats(times, label):
    if len(times) < 2:
        return f"{label}: count={len(times)} (insufficient)"
    span = times[-1] - times[0]
    fps = (len(times) - 1) / span if span > 0 else 0.0
    gaps = np.diff(times) * 1000.0
    p50, p95, p99 = np.percentile(gaps, [50, 95, 99])
    mx = float(np.max(gaps))
    over_250 = int(np.sum(gaps > 250.0))
    return (f"{label}: count={len(times)}, FPS={fps:.2f}, "
            f"gap_ms(P50={p50:.1f}, P95={p95:.1f}, P99={p99:.1f}, max={mx:.1f}), gaps>250ms={over_250}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--publish-debug", default="true")
    parser.add_argument("--subscribe-debug", default="true")
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--img-qos", default="best_effort")
    parser.add_argument("--label", default="BASELINE")
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = "43"
    proc = subprocess.Popen(
        [
            "ros2", "launch", "person_tracking", "webcam_debug.launch.py",
            "camera_index:=0", "fps:=30.0", "device:=cuda:0",
            f"publish_debug_image:={args.publish_debug}", "view_debug:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    node = PipelineProfiler(
        warmup_sec=args.warmup,
        measure_sec=args.duration,
        subscribe_debug=(args.subscribe_debug.lower() == "true"),
        img_qos_rel=args.img_qos,
    )

    pid_samples = {}
    try:
        total_wait = args.warmup + args.duration + 0.5
        last_pid_poll = 0.0
        while (time.monotonic() - node.t_start) < total_wait:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if (now - node.t_start) >= args.warmup and (now - last_pid_poll) >= 2.0:
                last_pid_poll = now
                for p in psutil.process_iter(["pid", "cmdline"]):
                    try:
                        cmd = " ".join(p.info["cmdline"] or [])
                        for key in ("perception_source", "person_detector", "tracking_manager"):
                            if key in cmd and "measure_pipeline" not in cmd:
                                if p.pid not in pid_samples:
                                    pid_samples[p.pid] = {"name": key, "proc": p, "cpus": [], "rss": []}
                                    p.cpu_percent(interval=None)
                                else:
                                    pid_samples[p.pid]["cpus"].append(p.cpu_percent(interval=None))
                                    pid_samples[p.pid]["rss"].append(p.memory_info().rss / (1024.0 * 1024.0))
                    except Exception:
                        pass
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"=== {args.label} (publish_debug={args.publish_debug}, sub_debug={args.subscribe_debug}, window={args.duration}s) ===")
    print(_stats(node.status_times, "source_status"))
    print(_stats(node.rgb_times, "rgb_raw      "))
    print(_stats(node.obs_times, "observation  "))
    print(_stats(node.dbg_times, "debug_image  "))
    if node.obs_proc_ms:
        print(f"obs_proc_ms: P50={np.percentile(node.obs_proc_ms, 50):.2f}, P95={np.percentile(node.obs_proc_ms, 95):.2f}, max={max(node.obs_proc_ms):.2f}")
    rgb_n = len(node.rgb_stamps)
    obs_matched = len(node.rgb_stamps & node.obs_stamps)
    dbg_matched = len(node.rgb_stamps & node.dbg_stamps)
    print(f"stamp_match: rgb={rgb_n}, obs={obs_matched}/{rgb_n} ({(obs_matched/max(1,rgb_n))*100:.1f}%), dbg={dbg_matched}/{rgb_n} ({(dbg_matched/max(1,rgb_n))*100:.1f}%)")
    for pid, info in pid_samples.items():
        if info["cpus"]:
            print(f"PID {pid} ({info['name']}): mean_CPU={np.mean(info['cpus']):.1f}%, max_CPU={np.max(info['cpus']):.1f}%, RAM={np.mean(info['rss']):.1f}MB")


if __name__ == "__main__":
    main()
