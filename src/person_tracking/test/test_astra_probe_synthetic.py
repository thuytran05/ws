#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Suite: Synthetic Data Validation for check_astra_hardware_topics.py
Runs isolated on ROS_DOMAIN_ID=43.
Validates 6 distinct synthetic stream scenarios:
1. PASS case: 28 Hz RGB & Depth, paired within 1.5ms, fresh age < 50ms, valid info -> PASS (rc=0).
2. Low rate case: 10 Hz RGB & Depth (< 20.0 Hz threshold) -> FAIL (rc=1).
3. Stale frame case: 28 Hz, frame age = 700ms (> 400ms threshold) -> FAIL (rc=1).
4. Timestamp slop exceeded: 28 Hz, depth timestamp offset by +100ms (> 60ms threshold) -> FAIL (rc=1).
5. Out-of-order arrival: Depth published first (ahead of RGB), matching stamps -> PASS (rc=0).
6. Zero timestamp case: stamp.sec=0, stamp.nanosec=0 -> FAIL (rc=1).
"""

import os
import subprocess
import sys
import time

MOCK_PUB_CODE = r'''
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

mode = sys.argv[1] if len(sys.argv) > 1 else "pass"
rate_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 28.0
duration_sec = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0

rclpy.init()
node = Node("mock_astra_publisher")
pub_rgb = node.create_publisher(Image, "/camera/color/image_raw", 20)
pub_depth = node.create_publisher(Image, "/camera/depth/image_raw", 20)
pub_info = node.create_publisher(CameraInfo, "/camera/color/camera_info", 20)

period = 1.0 / rate_hz
t_end = time.time() + duration_sec
dummy_rgb = bytes(640 * 480 * 3)
dummy_depth = bytes(640 * 480 * 2)

next_tick = time.time()

try:
    while time.time() < t_end:
        now_wall = time.time()
        t_sec = int(now_wall)
        t_nanosec = int((now_wall - t_sec) * 1e9)

        info_msg = CameraInfo()
        info_msg.header.stamp.sec = t_sec
        info_msg.header.stamp.nanosec = t_nanosec
        info_msg.header.frame_id = "camera_color_optical_frame"
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]

        rgb_msg = Image()
        rgb_msg.header.frame_id = "camera_color_optical_frame"
        rgb_msg.width = 640
        rgb_msg.height = 480
        rgb_msg.encoding = "rgb8"
        rgb_msg.data = dummy_rgb

        depth_msg = Image()
        depth_msg.header.frame_id = "camera_color_optical_frame"
        depth_msg.width = 640
        depth_msg.height = 480
        depth_msg.encoding = "16UC1"
        depth_msg.data = dummy_depth

        if mode == "pass":
            rgb_msg.header.stamp.sec = t_sec
            rgb_msg.header.stamp.nanosec = t_nanosec
            # Depth offset by 1.5ms (well within 60ms)
            d_time = now_wall + 0.0015
            depth_msg.header.stamp.sec = int(d_time)
            depth_msg.header.stamp.nanosec = int((d_time - int(d_time)) * 1e9)
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)
            pub_depth.publish(depth_msg)

        elif mode == "low_rate":
            rgb_msg.header.stamp.sec = t_sec
            rgb_msg.header.stamp.nanosec = t_nanosec
            depth_msg.header.stamp.sec = t_sec
            depth_msg.header.stamp.nanosec = t_nanosec
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)
            pub_depth.publish(depth_msg)

        elif mode == "stale_frame":
            # Timestamp 700ms in the past (> 400ms)
            stale_time = now_wall - 0.700
            rgb_msg.header.stamp.sec = int(stale_time)
            rgb_msg.header.stamp.nanosec = int((stale_time - int(stale_time)) * 1e9)
            depth_msg.header.stamp.sec = int(stale_time)
            depth_msg.header.stamp.nanosec = int((stale_time - int(stale_time)) * 1e9)
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)
            pub_depth.publish(depth_msg)

        elif mode == "timestamp_slop":
            rgb_msg.header.stamp.sec = t_sec
            rgb_msg.header.stamp.nanosec = t_nanosec
            # Depth offset by +100ms (exceeds 60ms)
            slop_time = now_wall + 0.100
            depth_msg.header.stamp.sec = int(slop_time)
            depth_msg.header.stamp.nanosec = int((slop_time - int(slop_time)) * 1e9)
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)
            pub_depth.publish(depth_msg)

        elif mode == "out_of_order":
            # Invert arrival sequence: Depth is published first, followed by Info and RGB
            # Both share matching capture epochs (within 1.5ms slop), demonstrating that
            # buffer-based 1-to-1 matching succeeds regardless of arrival sequence.
            d_time = now_wall + 0.0015
            depth_msg.header.stamp.sec = int(d_time)
            depth_msg.header.stamp.nanosec = int((d_time - int(d_time)) * 1e9)
            pub_depth.publish(depth_msg)

            rgb_msg.header.stamp.sec = t_sec
            rgb_msg.header.stamp.nanosec = t_nanosec
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)

        elif mode == "zero_stamp":
            rgb_msg.header.stamp.sec = 0
            rgb_msg.header.stamp.nanosec = 0
            depth_msg.header.stamp.sec = 0
            depth_msg.header.stamp.nanosec = 0
            pub_info.publish(info_msg)
            pub_rgb.publish(rgb_msg)
            pub_depth.publish(depth_msg)

        next_tick += period
        sleep_dur = next_tick - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        else:
            next_tick = time.time()
except Exception:
    pass
finally:
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
'''


def run_test_case(test_name: str, mode: str, rate_hz: float, expected_exit_code: int, probe_script: str, log_file) -> bool:
    print(f"\n>>> Running Test Case: {test_name} (mode={mode}, rate={rate_hz}Hz, expected_rc={expected_exit_code})...")
    log_file.write(f"\n================================================================================\n")
    log_file.write(f"TEST CASE: {test_name} [mode={mode}, rate={rate_hz}Hz, expected_exit_code={expected_exit_code}]\n")
    log_file.write(f"================================================================================\n")

    # Cooldown between test cases for complete DDS endpoint teardown
    time.sleep(1.2)

    # Start mock publisher in background
    pub_proc = subprocess.Popen(
        [sys.executable, "-c", MOCK_PUB_CODE, mode, str(rate_hz), "5.0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.5)  # Wait for publisher discovery

    # Run probe
    probe_proc = subprocess.run(
        [sys.executable, probe_script, "--duration", "2.5", "--min-rate", "20.0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    pub_proc.terminate()
    try:
        pub_proc.wait(timeout=1.0)
    except Exception:
        pub_proc.kill()

    log_file.write("--- PROBE STDOUT ---\n")
    log_file.write(probe_proc.stdout)
    log_file.write("--- PROBE STDERR ---\n")
    log_file.write(probe_proc.stderr)
    log_file.write(f"Probe Exit Code: {probe_proc.returncode} (Expected: {expected_exit_code})\n")

    passed = (probe_proc.returncode == expected_exit_code)
    status_str = "PASS [OK]" if passed else f"FAIL [X] (rc={probe_proc.returncode} != {expected_exit_code})"
    print(f"    Result: {status_str}")
    log_file.write(f"Test Result: {status_str}\n")
    return passed


def main():
    os.environ["ROS_DOMAIN_ID"] = "43"
    probe_script = "/home/ws/src/person_tracking/test/check_astra_hardware_topics.py"
    evidence_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_11"
    os.makedirs(evidence_dir, exist_ok=True)
    log_path = os.path.join(evidence_dir, "astra_probe_synthetic_tests.log")

    cases = [
        ("Case 1: Standard Valid Stream (28 Hz, Delta=1.5ms, Fresh Age)", "pass", 28.0, 0),
        ("Case 2: Low Rate Stream (10 Hz < 20.0 Hz threshold)", "low_rate", 10.0, 1),
        ("Case 3: Stale Frame Stream (Age = 700ms > 400ms threshold)", "stale_frame", 28.0, 1),
        ("Case 4: Timestamp Slop Exceeded (Delta = 100ms > 60ms threshold)", "timestamp_slop", 28.0, 1),
        ("Case 5: Out-Of-Order Frame Arrival (Depth ahead of RGB, buffer matched)", "out_of_order", 28.0, 0),
        ("Case 6: Uninitialized Zero Timestamp (sec=0, nanosec=0)", "zero_stamp", 28.0, 1),
    ]

    all_ok = True
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("PHASE 11: ASTRA PROBE SYNTHETIC TEST SUITE\n")
        f.write(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"ROS_DOMAIN_ID: 43\n")
        f.write("Scope: Synthetic data validation of check_astra_hardware_topics.py probe only.\n")
        f.write("No Gazebo physics, Nav2, or /cmd_vel involved.\n")

        for name, mode, rate, exp_rc in cases:
            ok = run_test_case(name, mode, rate, exp_rc, probe_script, f)
            if not ok:
                all_ok = False

        summary_line = f"\nOVERALL RESULT: {'ALL 6/6 TESTS PASSED (100% OK)' if all_ok else 'SOME TESTS FAILED'}\n"
        print(summary_line)
        f.write(summary_line)

    print(f"Log written to: {log_path}")
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
