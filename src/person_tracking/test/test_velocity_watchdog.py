# Copyright (c) 2026 Airport Robot Escort Project
# Unit and ROS 2 Integration Tests for Velocity Watchdog

import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from person_tracking.velocity_watchdog import VelocityWatchdog, WatchdogLimits
from person_tracking.velocity_watchdog_node import VelocityWatchdogNode


def test_watchdog_default_is_strictly_zero():
    """1. Default output is strictly zero Twist before any valid input is received."""
    wd = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15))
    vx, vy, wz, reason = wd.process_velocity(now_mono=100.0)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert reason == "INITIAL_ZERO_NO_INPUT"
    assert wd.is_active_zeroing is True


def test_watchdog_forward_fresh_valid_cmd():
    """2. Valid commands within timeout are passed through cleanly."""
    wd = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15, max_linear_x=0.50, max_angular_z=0.40))
    ok, reason = wd.update_cmd_vel(linear_x=0.35, linear_y=0.0, angular_z=0.15, now_mono=10.0)
    assert ok is True
    assert reason == "OK"

    # Evaluate at 10.05s (age = 50ms, well within 150ms timeout)
    vx, vy, wz, reason = wd.process_velocity(now_mono=10.05)
    assert vx == 0.35
    assert vy == 0.0
    assert wz == 0.15
    assert reason == "FORWARD_VALID"
    assert wd.is_active_zeroing is False


def test_watchdog_active_zero_on_timeout():
    """3. When time since last command exceeds timeout_sec (0.15s), watchdog clamps to zero actively."""
    wd = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15))
    wd.update_cmd_vel(linear_x=0.40, linear_y=0.0, angular_z=0.20, now_mono=20.0)

    # At 20.10s (age = 0.10s <= 0.15s) -> Forward valid
    vx, vy, wz, reason = wd.process_velocity(now_mono=20.10)
    assert vx == 0.40

    # At 20.16s (age = 0.16s > 0.15s) -> Active zeroing triggered!
    vx, vy, wz, reason = wd.process_velocity(now_mono=20.16)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
    assert "TIMEOUT_EXCEEDED_ACTIVE_ZERO" in reason
    assert wd.is_active_zeroing is True
    assert wd.total_timeouts_triggered == 1


def test_watchdog_purges_cache_on_timeout_no_replay():
    """4. When timeout expires, cached command is purged to prevent replay upon reconnection."""
    wd = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15))
    wd.update_cmd_vel(linear_x=0.45, linear_y=0.0, angular_z=-0.10, now_mono=30.0)

    # Trigger timeout at 30.20s
    vx, vy, wz, reason = wd.process_velocity(now_mono=30.20)
    assert vx == 0.0

    # Next evaluation at 30.30s without new input: remains zero
    vx, vy, wz, reason = wd.process_velocity(now_mono=30.30)
    assert vx == 0.0
    assert vx == 0.0
    assert wd.has_valid_command is False

    # Guard restarts and reconnects at 35.0s with new command
    wd.update_cmd_vel(linear_x=0.20, linear_y=0.0, angular_z=0.0, now_mono=35.0)
    vx, vy, wz, reason = wd.process_velocity(now_mono=35.02)
    assert vx == 0.20
    assert reason == "FORWARD_VALID"


def test_watchdog_rejects_nan_and_inf():
    """5. Watchdog rejects NaN and Inf inputs, immediately zeroing output."""
    wd = VelocityWatchdog(limits=WatchdogLimits(timeout_sec=0.15))
    wd.update_cmd_vel(linear_x=0.30, linear_y=0.0, angular_z=0.0, now_mono=40.0)

    # Send NaN
    ok, reason = wd.update_cmd_vel(linear_x=float('nan'), linear_y=0.0, angular_z=0.0, now_mono=40.05)
    assert ok is False
    assert reason == "INVALID_NUMERIC_NAN_OR_INF"

    vx, vy, wz, reason = wd.process_velocity(now_mono=40.06)
    assert vx == 0.0
    assert wd.is_active_zeroing is True

    # Send Inf
    ok, reason = wd.update_cmd_vel(linear_x=float('inf'), linear_y=0.0, angular_z=0.0, now_mono=40.10)
    assert ok is False
    vx, vy, wz, reason = wd.process_velocity(now_mono=40.11)
    assert vx == 0.0


def test_watchdog_node_ros2_smoke():
    """6. ROS 2 node smoke test: verify active zeroing and pass-through over topics."""
    rclpy.init()
    try:
        node = VelocityWatchdogNode()

        # Before any input: check output on tick
        node._on_control_tick()
        assert node.watchdog.is_active_zeroing is True

        # Send valid command
        cmd_in = Twist()
        cmd_in.linear.x = 0.25
        cmd_in.angular.z = -0.10
        node._on_cmd_vel_in(cmd_in)
        assert node.watchdog.has_valid_command is True

        node._on_control_tick()
        assert node.watchdog.is_active_zeroing is False
        assert node.watchdog.last_linear_x == 0.25

        node.destroy_node()
    finally:
        rclpy.shutdown()
