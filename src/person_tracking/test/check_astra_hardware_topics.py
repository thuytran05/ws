#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 11 Hardware Helper: Check Astra Camera Topics, Encodings, QoS and Synchronicity.

Verifies:
1. /camera/color/image_raw (rgb8/bgr8, rate >= 20.0 Hz, expected 20-30 Hz)
2. /camera/depth/image_raw (16UC1/32FC1, rate >= 20.0 Hz, expected 20-30 Hz)
3. /camera/color/camera_info (valid intrinsics fx, fy, cx, cy > 0)
4. Synchronicity between RGB and Depth timestamps (delta <= 0.060s / 60ms)
5. Capture age (age <= 0.400s / 400ms, not future, valid clock domain)

Note:
PASS of this probe confirms stream reception and timestamp synchronicity
within the probe measurement window. It does NOT replace physical 3D D2C
alignment verification on RViz2 or ground-truth tape marker tests.
"""

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class AstraTopicChecker(Node):
    def __init__(
        self,
        check_duration_sec: float = 5.0,
        min_rate: float = 20.0,
        max_slop_sec: float = 0.060,
        max_age_sec: float = 0.400,
        rgb_topic: str = '/camera/color/image_raw',
        depth_topic: str = '/camera/depth/image_raw',
        info_topic: str = '/camera/color/camera_info',
    ):
        super().__init__('astra_topic_checker')
        self.check_duration_sec = check_duration_sec
        self.min_rate = min_rate
        self.max_slop_sec = max_slop_sec
        self.max_age_sec = max_age_sec

        # Raw frame records
        self.rgb_records: List[Dict] = []
        self.depth_records: List[Dict] = []
        self.info_count = 0
        self.camera_intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.camera_info_resolution: Optional[Tuple[int, int]] = None

        # Compatible QoS: BEST_EFFORT subscriber accepts both BEST_EFFORT and RELIABLE publishers
        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_info = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_rgb = self.create_subscription(Image, rgb_topic, self._on_rgb, qos_img)
        self.sub_depth = self.create_subscription(Image, depth_topic, self._on_depth, qos_img)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self._on_info, qos_info)

    def _eval_stamp_and_age(self, stamp_sec: int, stamp_nanosec: int, recv_wall: float) -> Tuple[float, bool, float, str]:
        """
        Evaluate timestamp validity and frame age.
        Returns: (t_stamp, is_age_valid, age_sec, reason)
        """
        if stamp_sec == 0 and stamp_nanosec == 0:
            return 0.0, False, 999.0, "Timestamp uninitialized (sec=0, nanosec=0)"

        t_stamp = stamp_sec + stamp_nanosec * 1e-9

        # Reference clock evaluation
        now_msg = self.get_clock().now().to_msg()
        t_ros = now_msg.sec + now_msg.nanosec * 1e-9

        # Use ROS clock if valid epoch or synchronized, else wall clock
        if abs(t_ros - t_stamp) < 86400.0 and t_ros > 100.0:
            t_ref = t_ros
            clock_name = "ROS clock"
        elif abs(recv_wall - t_stamp) < 86400.0:
            t_ref = recv_wall
            clock_name = "Wall clock"
        else:
            return (
                t_stamp,
                False,
                999.0,
                f"Clock domain mismatch (stamp={t_stamp:.3f}s vs ros_clock={t_ros:.3f}s, wall={recv_wall:.3f}s)",
            )

        age = t_ref - t_stamp
        if age < -0.050:
            return (
                t_stamp,
                False,
                age,
                f"Stamp is in the future by {-age*1000.0:.1f}ms (> 50ms clock skew against {clock_name})",
            )
        if age > self.max_age_sec:
            return (
                t_stamp,
                False,
                age,
                f"Frame age ({age*1000.0:.1f}ms) exceeds max limit ({self.max_age_sec*1000.0:.1f}ms against {clock_name})",
            )

        return t_stamp, True, age, f"OK ({clock_name})"

    def _on_rgb(self, msg: Image):
        recv_mono = time.monotonic()
        recv_wall = time.time()
        t_stamp, age_valid, age, reason = self._eval_stamp_and_age(
            msg.header.stamp.sec, msg.header.stamp.nanosec, recv_wall
        )
        self.rgb_records.append({
            'stamp': (msg.header.stamp.sec, msg.header.stamp.nanosec),
            'time': t_stamp,
            'recv_mono': recv_mono,
            'recv_wall': recv_wall,
            'encoding': msg.encoding,
            'shape': (msg.width, msg.height),
            'age': age,
            'age_valid': age_valid,
            'age_reason': reason,
        })

    def _on_depth(self, msg: Image):
        recv_mono = time.monotonic()
        recv_wall = time.time()
        t_stamp, age_valid, age, reason = self._eval_stamp_and_age(
            msg.header.stamp.sec, msg.header.stamp.nanosec, recv_wall
        )
        self.depth_records.append({
            'stamp': (msg.header.stamp.sec, msg.header.stamp.nanosec),
            'time': t_stamp,
            'recv_mono': recv_mono,
            'recv_wall': recv_wall,
            'encoding': msg.encoding,
            'shape': (msg.width, msg.height),
            'age': age,
            'age_valid': age_valid,
            'age_reason': reason,
        })

    def _on_info(self, msg: CameraInfo):
        self.info_count += 1
        if self.camera_intrinsics is None and len(msg.k) == 9:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            if fx > 0 and fy > 0 and cx > 0 and cy > 0:
                self.camera_intrinsics = (fx, fy, cx, cy)
                self.camera_info_resolution = (msg.width, msg.height)


def match_timestamps_1to1(
    rgb_records: List[Dict],
    depth_records: List[Dict],
    max_slop_sec: float = 0.060,
) -> Tuple[int, int, int, List[float], List[float]]:
    """
    Perform 1-to-1 nearest timestamp matching consistent with pipeline slop criteria.
    Uses globally sorted delta matching to prioritize closest timestamp pairings
    and prevent off-by-one cascading misassociations.
    Returns: (valid_pairs_count, unpaired_rgb_count, unpaired_depth_count, matched_deltas, exceeded_deltas)
    """
    if not rgb_records or not depth_records:
        return 0, len(rgb_records), len(depth_records), [], []

    valid_rgb = [r for r in rgb_records if r['time'] > 0]
    valid_depth = [d for d in depth_records if d['time'] > 0]

    # Generate all candidate pairs within 4 * max_slop_sec
    candidates = []
    for r_idx, r in enumerate(valid_rgb):
        t_r = r['time']
        for d_idx, d in enumerate(valid_depth):
            diff = abs(t_r - d['time'])
            if diff <= max_slop_sec * 4.0:
                candidates.append((diff, r_idx, d_idx))

    # Sort candidates by smallest difference first (greedy global minimum)
    candidates.sort(key=lambda x: x[0])

    matched_deltas: List[float] = []
    exceeded_deltas: List[float] = []
    used_rgb = set()
    used_depth = set()

    for diff, r_idx, d_idx in candidates:
        if r_idx in used_rgb or d_idx in used_depth:
            continue
        if diff <= max_slop_sec:
            used_rgb.add(r_idx)
            used_depth.add(d_idx)
            matched_deltas.append(diff)
        else:
            used_rgb.add(r_idx)
            used_depth.add(d_idx)
            exceeded_deltas.append(diff)

    # For any remaining unused RGB frames, find their closest depth frame for diagnostics
    for r_idx, r in enumerate(valid_rgb):
        if r_idx not in used_rgb:
            best_diff = min([abs(r['time'] - d['time']) for d in valid_depth]) if valid_depth else float('inf')
            if best_diff != float('inf') and best_diff > max_slop_sec:
                exceeded_deltas.append(best_diff)

    valid_pairs = len(matched_deltas)
    unpaired_rgb = len(valid_rgb) - valid_pairs
    unpaired_depth = len(valid_depth) - valid_pairs

    return valid_pairs, unpaired_rgb, unpaired_depth, matched_deltas, exceeded_deltas


def compute_stream_rate(records: List[Dict], total_listen_sec: float) -> float:
    """Compute stream rate in Hz using first-to-last message span or total listening duration."""
    if len(records) < 2:
        return float(len(records)) / total_listen_sec if total_listen_sec > 0 else 0.0
    t_span = records[-1]['recv_mono'] - records[0]['recv_mono']
    if t_span >= 0.5:
        return float(len(records) - 1) / t_span
    return float(len(records)) / total_listen_sec if total_listen_sec > 0 else 0.0


def run_audit(node: AstraTopicChecker, elapsed_sec: float) -> Tuple[bool, List[str]]:
    """Analyze collected records and evaluate Pass/Fail."""
    reasons = []

    rgb_count = len(node.rgb_records)
    depth_count = len(node.depth_records)
    info_count = node.info_count

    rgb_rate = compute_stream_rate(node.rgb_records, elapsed_sec)
    depth_rate = compute_stream_rate(node.depth_records, elapsed_sec)

    print("\n" + "=" * 80)
    print("PHASE 11: ASTRA CAMERA HARDWARE TOPIC AUDIT REPORT")
    print("=" * 80)
    print(f"Monitoring duration: {elapsed_sec:.2f} s")
    print(f"Thresholds: min_rate={node.min_rate:.1f} Hz (expected 20-30 Hz) | max_slop={node.max_slop_sec*1000.0:.1f} ms | max_age={node.max_age_sec*1000.0:.1f} ms")
    print("-" * 80)

    # 1. Color Topic
    rgb_last_enc = node.rgb_records[-1]['encoding'] if rgb_count > 0 else "NONE"
    rgb_last_shape = node.rgb_records[-1]['shape'] if rgb_count > 0 else (0, 0)
    rgb_rate_ok = (rgb_count >= 5 and rgb_rate >= node.min_rate)
    rgb_enc_ok = rgb_last_enc in ("rgb8", "bgr8", "rgba8", "bgra8")

    rgb_ages = [r['age'] for r in node.rgb_records if r['age_valid']]
    rgb_age_failures = [r for r in node.rgb_records if not r['age_valid']]
    rgb_age_ok = (len(rgb_age_failures) == 0 and len(rgb_ages) > 0)

    print(f"1. Color Topic  (/camera/color/image_raw):")
    print(f"   - Received: {rgb_count} msgs | Rate: {rgb_rate:.1f} Hz (Threshold: >= {node.min_rate:.1f} Hz) -> [{'PASS' if rgb_rate_ok else 'FAIL'}]")
    print(f"   - Encoding: '{rgb_last_enc}' | Resolution: {rgb_last_shape} -> [{'PASS' if rgb_enc_ok else 'FAIL'}]")
    if rgb_ages:
        print(f"   - Age check: min={min(rgb_ages)*1000:.1f} ms, max={max(rgb_ages)*1000:.1f} ms, mean={(sum(rgb_ages)/len(rgb_ages))*1000:.1f} ms (Limit: <= {node.max_age_sec*1000:.1f} ms) -> [{'PASS' if rgb_age_ok else 'FAIL'}]")
    else:
        print(f"   - Age check: NO VALID TIMESTAMPS -> [FAIL]")
    if rgb_age_failures:
        print(f"     * Sample age issue: {rgb_age_failures[0]['age_reason']}")

    # 2. Depth Topic
    depth_last_enc = node.depth_records[-1]['encoding'] if depth_count > 0 else "NONE"
    depth_last_shape = node.depth_records[-1]['shape'] if depth_count > 0 else (0, 0)
    depth_rate_ok = (depth_count >= 5 and depth_rate >= node.min_rate)
    depth_enc_ok = depth_last_enc in ("16UC1", "32FC1")

    depth_ages = [d['age'] for d in node.depth_records if d['age_valid']]
    depth_age_failures = [d for d in node.depth_records if not d['age_valid']]
    depth_age_ok = (len(depth_age_failures) == 0 and len(depth_ages) > 0)

    print(f"2. Depth Topic  (/camera/depth/image_raw):")
    print(f"   - Received: {depth_count} msgs | Rate: {depth_rate:.1f} Hz (Threshold: >= {node.min_rate:.1f} Hz) -> [{'PASS' if depth_rate_ok else 'FAIL'}]")
    print(f"   - Encoding: '{depth_last_enc}' | Resolution: {depth_last_shape} -> [{'PASS' if depth_enc_ok else 'FAIL'}]")
    if depth_ages:
        print(f"   - Age check: min={min(depth_ages)*1000:.1f} ms, max={max(depth_ages)*1000:.1f} ms, mean={(sum(depth_ages)/len(depth_ages))*1000:.1f} ms (Limit: <= {node.max_age_sec*1000:.1f} ms) -> [{'PASS' if depth_age_ok else 'FAIL'}]")
    else:
        print(f"   - Age check: NO VALID TIMESTAMPS -> [FAIL]")
    if depth_age_failures:
        print(f"     * Sample age issue: {depth_age_failures[0]['age_reason']}")

    # 3. Camera Info Topic
    info_ok = (info_count >= 1 and node.camera_intrinsics is not None)
    print(f"3. Info Topic   (/camera/color/camera_info):")
    print(f"   - Received: {info_count} msgs | Intrinsics (fx,fy,cx,cy): {node.camera_intrinsics} -> [{'PASS' if info_ok else 'FAIL'}]")

    # 4. Synchronicity & Pairing
    valid_pairs, unpaired_rgb, unpaired_depth, matched_deltas, exceeded_deltas = match_timestamps_1to1(
        node.rgb_records, node.depth_records, max_slop_sec=node.max_slop_sec
    )

    total_eval = max(rgb_count, depth_count)
    pair_ratio = (float(valid_pairs) / float(total_eval)) if total_eval > 0 else 0.0
    sync_ok = (valid_pairs > 0 and pair_ratio >= 0.85)

    print(f"4. Timestamp Pairing & Synchronicity (|t_rgb - t_depth| <= {node.max_slop_sec*1000:.1f} ms):")
    print(f"   - Total evaluated frames: {rgb_count} RGB, {depth_count} Depth")
    print(f"   - Valid matched pairs:   {valid_pairs} ({pair_ratio*100:.1f}%) -> [{'PASS' if sync_ok else 'FAIL'}]")
    print(f"   - Unpaired frames:        {unpaired_rgb} RGB, {unpaired_depth} Depth")
    if matched_deltas:
        mean_delta = (sum(matched_deltas) / len(matched_deltas)) * 1000.0
        max_delta = max(matched_deltas) * 1000.0
        min_delta = min(matched_deltas) * 1000.0
        print(f"   - Matched delta stats:    min={min_delta:.2f} ms, mean={mean_delta:.2f} ms, max={max_delta:.2f} ms")
    else:
        print(f"   - Matched delta stats:    NO VALID PAIRS FORMED!")
    if exceeded_deltas:
        print(f"   - Exceeded delta sample:  min={min(exceeded_deltas)*1000:.1f} ms, max={max(exceeded_deltas)*1000:.1f} ms")

    # Evaluation
    if not rgb_rate_ok:
        reasons.append(f"RGB rate too low ({rgb_rate:.1f} Hz < {node.min_rate:.1f} Hz, msgs={rgb_count})")
    if not rgb_enc_ok:
        reasons.append(f"Invalid RGB encoding '{rgb_last_enc}' (expected rgb8/bgr8)")
    if not rgb_age_ok:
        reasons.append(f"RGB age check failed ({len(rgb_age_failures)} invalid msgs; reason: {rgb_age_failures[0]['age_reason'] if rgb_age_failures else 'no stamps'})")

    if not depth_rate_ok:
        reasons.append(f"Depth rate too low ({depth_rate:.1f} Hz < {node.min_rate:.1f} Hz, msgs={depth_count})")
    if not depth_enc_ok:
        reasons.append(f"Invalid Depth encoding '{depth_last_enc}' (expected 16UC1/32FC1)")
    if not depth_age_ok:
        reasons.append(f"Depth age check failed ({len(depth_age_failures)} invalid msgs; reason: {depth_age_failures[0]['age_reason'] if depth_age_failures else 'no stamps'})")

    if not info_ok:
        reasons.append("No valid camera intrinsics received on /camera/color/camera_info")

    if not sync_ok:
        reasons.append(f"Insufficient paired frames ({valid_pairs}/{total_eval} = {pair_ratio*100:.1f}% < 85%) or slop > {node.max_slop_sec*1000:.1f}ms")

    passed = (len(reasons) == 0)

    print("-" * 80)
    if passed:
        print("STATUS: PASS")
        print("Note: PASS of this probe confirms stream reception and timestamp synchronicity.")
        print("      Physical 3D D2C alignment and floor distance metrics remain UNVERIFIED until physical check.")
    else:
        print(f"STATUS: FAIL - Issues found ({len(reasons)}):")
        for idx, r in enumerate(reasons, 1):
            print(f"  {idx}. {r}")
    print("=" * 80)

    return passed, reasons


def main():
    parser = argparse.ArgumentParser(description="Check Astra Camera ROS Topics and Alignment")
    parser.add_argument("--duration", type=float, default=5.0, help="Monitoring duration in seconds (default: 5.0)")
    parser.add_argument("--min-rate", type=float, default=20.0, help="Minimum acceptable rate in Hz for PASS (default: 20.0)")
    parser.add_argument("--max-slop", type=float, default=0.060, help="Max acceptable timestamp difference in seconds (default: 0.060)")
    parser.add_argument("--max-age", type=float, default=0.400, help="Max acceptable frame age in seconds (default: 0.400)")
    parser.add_argument("--rgb-topic", type=str, default="/camera/color/image_raw", help="RGB image topic")
    parser.add_argument("--depth-topic", type=str, default="/camera/depth/image_raw", help="Depth image topic")
    parser.add_argument("--info-topic", type=str, default="/camera/color/camera_info", help="CameraInfo topic")
    args = parser.parse_args()

    rclpy.init()
    node = AstraTopicChecker(
        check_duration_sec=args.duration,
        min_rate=args.min_rate,
        max_slop_sec=args.max_slop,
        max_age_sec=args.max_age,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        info_topic=args.info_topic,
    )

    print(f"=== Checking Astra Camera topics on current ROS domain for {args.duration}s... ===")

    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - t0
    passed, reasons = run_audit(node, elapsed)

    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass

    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
