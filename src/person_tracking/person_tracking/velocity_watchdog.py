# Copyright (c) 2026 Airport Robot Escort Project
# Phase 8 Velocity Watchdog - Independent Active Zero Watchdog

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple


@dataclass
class WatchdogLimits:
    """Parameters and safety limits for the Velocity Watchdog."""
    timeout_sec: float = 0.15          # 150 ms timeout: 3 x 50 ms (20 Hz Guard nominal period) + jitter allowance
    max_linear_x: float = 0.50         # Max forward linear velocity (m/s)
    max_angular_z: float = 0.40        # Max angular velocity (rad/s)


class VelocityWatchdog:
    """Independent monotonic time velocity watchdog.

    Safety Principles:
    1. Default output is strictly zero Twist before any valid input is received.
    2. Uses monotonic steady clock (time.monotonic()) to prevent bypass from /clock jumps or frozen clocks.
    3. Detects loss of Guard publisher within timeout_sec (0.15s) and actively publishes zero Twists at 20 Hz.
    4. Purges cached commands immediately upon timeout expiry, preventing stale command replay when Guard reconnects.
    5. Validates finite numeric values (rejects NaN / Inf).
    """

    def __init__(self, limits: Optional[WatchdogLimits] = None):
        self.limits = limits or WatchdogLimits()
        self.last_cmd_time_mono: float = 0.0
        self.last_linear_x: float = 0.0
        self.last_linear_y: float = 0.0
        self.last_angular_z: float = 0.0
        self.has_valid_command: bool = False
        self.is_active_zeroing: bool = True
        self.total_timeouts_triggered: int = 0
        self.total_messages_received: int = 0

    def update_cmd_vel(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        now_mono: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """Process incoming velocity command from upstream Velocity Guard."""
        if now_mono is None:
            now_mono = time.monotonic()

        # Reject non-finite numbers (NaN, Inf)
        if not (math.isfinite(linear_x) and math.isfinite(linear_y) and math.isfinite(angular_z)):
            self.has_valid_command = False
            self.last_linear_x = 0.0
            self.last_linear_y = 0.0
            self.last_angular_z = 0.0
            self.is_active_zeroing = True
            return False, "INVALID_NUMERIC_NAN_OR_INF"

        # Clamp to physical limits if exceeded
        clamped_x = max(0.0, min(float(linear_x), self.limits.max_linear_x))
        clamped_y = 0.0  # Strict invariant: tracking chain forces linear.y = 0.0
        clamped_wz = max(-self.limits.max_angular_z, min(float(angular_z), self.limits.max_angular_z))

        self.last_linear_x = clamped_x
        self.last_linear_y = clamped_y
        self.last_angular_z = clamped_wz
        self.last_cmd_time_mono = now_mono
        self.has_valid_command = True
        self.is_active_zeroing = False
        self.total_messages_received += 1
        return True, "OK"

    def process_velocity(self, now_mono: Optional[float] = None) -> Tuple[float, float, float, str]:
        """Evaluate safety watchdog and return commanded (vx, vy, wz, reason)."""
        if now_mono is None:
            now_mono = time.monotonic()

        if not self.has_valid_command or self.last_cmd_time_mono == 0.0:
            self.is_active_zeroing = True
            return 0.0, 0.0, 0.0, "INITIAL_ZERO_NO_INPUT"

        age = now_mono - self.last_cmd_time_mono
        if age > self.limits.timeout_sec:
            # Timeout exceeded: publisher died or stalled!
            # Actively zero and purge cached command to prevent replay upon reconnection
            self.has_valid_command = False
            self.last_linear_x = 0.0
            self.last_linear_y = 0.0
            self.last_angular_z = 0.0
            self.is_active_zeroing = True
            self.total_timeouts_triggered += 1
            return 0.0, 0.0, 0.0, f"TIMEOUT_EXCEEDED_ACTIVE_ZERO_{age:.3f}S"

        # Fresh and valid command
        self.is_active_zeroing = False
        return self.last_linear_x, self.last_linear_y, self.last_angular_z, "FORWARD_VALID"

    def reset(self):
        """Reset watchdog state completely."""
        self.last_cmd_time_mono = 0.0
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        self.has_valid_command = False
        self.is_active_zeroing = True
