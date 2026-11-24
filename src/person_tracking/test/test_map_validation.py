#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Focused Test Suite: Map Validation in escort_navigation_real.launch.py
Verifies:
1. Case A: use_sim_time=false and no map provided -> BLOCKED (Missing mandatory map).
2. Case B: use_sim_time=false and non-existent map path -> BLOCKED (File not found).
3. Case C: use_sim_time=false and simulation map (turtlebot3_world) provided -> BLOCKED (Prohibited simulation map).
4. Case D: use_sim_time=true and default map -> Uses turtlebot3_world fallback validly.
5. Case E: use_sim_time=true and non-existent map -> BLOCKED (Simulation map file not found).
"""

import os
import subprocess
import sys
import tempfile
import time


def run_launch_check(args_list, timeout_sec=5.0):
    cmd = [
        "ros2", "launch", "person_tracking", "escort_navigation_real.launch.py"
    ] + args_list
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = "44"
    
    import signal
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        preexec_fn=os.setsid
    )
    output = ""
    try:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if p.poll() is not None:
                break
            time.sleep(0.1)
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                p.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
        output, _ = p.communicate()
    except Exception as exc:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
        output, _ = p.communicate()
    return p.returncode, output


def main():
    print("=" * 80)
    print("TEST SUITE: MAP VALIDATION IN escort_navigation_real.launch.py")
    print("=" * 80)
    
    results = {}
    all_ok = True

    # 1. Case A: use_sim_time:=false without map
    print("[1] Case A: use_sim_time:=false without map argument...")
    rc, out = run_launch_check(["use_sim_time:=false"])
    passed = (rc != 0) and ("'map' argument is mandatory" in out or "map argument is mandatory" in out)
    results["Case A (Missing map in real mode)"] = (passed, f"rc={rc}, error_caught={'map\' argument is mandatory' in out}")
    print(f"    {'PASS' if passed else 'FAIL'}: rc={rc}, output snippet: {out[:200]}...")
    if not passed:
        all_ok = False

    # 2. Case B: use_sim_time:=false with non-existent map path
    print("[2] Case B: use_sim_time:=false with non-existent map path...")
    rc, out = run_launch_check(["use_sim_time:=false", "map:=/invalid/path/to/missing_airport_map.yaml"])
    passed = (rc != 0) and ("Specified physical map file does not exist" in out)
    results["Case B (Non-existent map in real mode)"] = (passed, f"rc={rc}, error_caught={'does not exist' in out}")
    print(f"    {'PASS' if passed else 'FAIL'}: rc={rc}")
    if not passed:
        all_ok = False

    # 3. Case C: use_sim_time:=false with turtlebot3_world simulation map
    print("[3] Case C: use_sim_time:=false with turtlebot3_world simulation map...")
    tb_map = "/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml"
    rc, out = run_launch_check(["use_sim_time:=false", f"map:={tb_map}"])
    passed = (rc != 0) and ("cannot be used on physical robot" in out)
    results["Case C (Simulation map rejected in real mode)"] = (passed, f"rc={rc}, error_caught={'cannot be used on physical robot' in out}")
    print(f"    {'PASS' if passed else 'FAIL'}: rc={rc}")
    if not passed:
        all_ok = False

    # 4. Case D: use_sim_time:=true with default map (should NOT fail with map error)
    print("[4] Case D: use_sim_time:=true with default map...")
    rc, out = run_launch_check(["use_sim_time:=true"], timeout_sec=4.0)
    # When use_sim_time:=true, it launches Nav2 with turtlebot3_world. It won't fail with BLOCKED map error
    passed = ("BLOCKED:" not in out) and ("Specified physical map file does not exist" not in out)
    results["Case D (Simulation mode accepts default turtlebot map)"] = (passed, f"no_map_error={passed}")
    print(f"    {'PASS' if passed else 'FAIL'}: map validation passed in simulation mode")
    if not passed:
        all_ok = False

    # 5. Case E: use_sim_time:=true with non-existent map
    print("[5] Case E: use_sim_time:=true with non-existent map...")
    rc, out = run_launch_check(["use_sim_time:=true", "map:=/nonexistent/sim_map.yaml"])
    passed = (rc != 0) and ("Simulation map file not found" in out)
    results["Case E (Non-existent map rejected in simulation mode)"] = (passed, f"rc={rc}")
    print(f"    {'PASS' if passed else 'FAIL'}: rc={rc}")
    if not passed:
        all_ok = False

    # 6. Case F: use_sim_time:=false with valid temporary facility map
    print("[6] Case F: use_sim_time:=false with valid temporary facility map...")
    with tempfile.NamedTemporaryFile(suffix="_airport_map.yaml", delete=False) as tf:
        tf.write(b"image: test.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        temp_map_path = tf.name

    try:
        rc, out = run_launch_check(["use_sim_time:=false", f"map:={temp_map_path}"], timeout_sec=4.0)
        passed = ("BLOCKED:" not in out) and ("Specified physical map file does not exist" not in out)
        results["Case F (Valid facility map accepted in real mode)"] = (passed, f"no_map_error={passed}")
        print(f"    {'PASS' if passed else 'FAIL'}: map accepted cleanly in real mode")
        if not passed:
            all_ok = False
    finally:
        if os.path.exists(temp_map_path):
            os.remove(temp_map_path)

    print("=" * 80)
    print("SUMMARY OF MAP VALIDATION TESTS:")
    for k, (p, info) in results.items():
        print(f"  - {k}: {'PASS' if p else 'FAIL'} ({info})")
    print(f"OVERALL RESULT: {'PASS (100% OK)' if all_ok else 'FAIL'}")
    print("=" * 80)

    # Save evidence log
    evidence_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_11"
    os.makedirs(evidence_dir, exist_ok=True)
    log_path = os.path.join(evidence_dir, "map_validation_tests.log")
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("MAP VALIDATION TEST SUITE SUMMARY\n")
        for k, (p, info) in results.items():
            f.write(f"- {k}: {'PASS' if p else 'FAIL'} ({info})\n")
        f.write(f"OVERALL: {'PASS' if all_ok else 'FAIL'}\n")
    print(f"Evidence log written to: {log_path}")

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
