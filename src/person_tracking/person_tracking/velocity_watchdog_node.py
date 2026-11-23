#!/usr/bin/env python3
# Copyright (c) 2026 Airport Robot Escort Project
# ROS 2 Node for Velocity Watchdog - Independent Process Active Zeroing

import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist

from person_tracking.velocity_watchdog import VelocityWatchdog, WatchdogLimits


class VelocityWatchdogNode(Node):
    """ROS 2 Node executing VelocityWatchdog policy between Guard and Collision Monitor."""

    def __init__(
        self,
        cmd_vel_in_topic: str = None,
        cmd_vel_out_topic: str = None,
        timeout_sec: float = None,
        control_rate_hz: float = None,
        **kwargs,
    ):
        super().__init__('escort_velocity_watchdog', **kwargs)

        # Parameters
        self.declare_parameter('cmd_vel_in_topic', cmd_vel_in_topic or '/tracking/cmd_vel_guarded')
        self.declare_parameter('cmd_vel_out_topic', cmd_vel_out_topic or '/tracking/cmd_vel_watchdog')
        self.declare_parameter('timeout_sec', timeout_sec if timeout_sec is not None else 0.15)
        self.declare_parameter('max_linear_x', 0.50)
        self.declare_parameter('max_angular_z', 0.40)
        self.declare_parameter('control_rate_hz', control_rate_hz if control_rate_hz is not None else 20.0)

        cmd_in_topic = str(self.get_parameter('cmd_vel_in_topic').value)
        cmd_out_topic = str(self.get_parameter('cmd_vel_out_topic').value)
        timeout_sec = float(self.get_parameter('timeout_sec').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        limits = WatchdogLimits(
            timeout_sec=timeout_sec,
            max_linear_x=float(self.get_parameter('max_linear_x').value),
            max_angular_z=float(self.get_parameter('max_angular_z').value),
        )
        self.watchdog = VelocityWatchdog(limits=limits)

        # QoS
        qos_depth = 10

        # Subscription to upstream Guard
        self.sub_cmd_in = self.create_subscription(Twist, cmd_in_topic, self._on_cmd_vel_in, qos_depth)

        # Publisher to downstream Collision Monitor
        self.pub_cmd_out = self.create_publisher(Twist, cmd_out_topic, qos_depth)

        # 20 Hz Monotonic Timer for Active Zeroing
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 0.05
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(timer_period, self._on_control_tick, clock=self.steady_clock)

        self.get_logger().info(
            f"VelocityWatchdogNode initialized (in: {cmd_in_topic}, out: {cmd_out_topic}, timeout: {timeout_sec:.3f}s, rate: {rate_hz} Hz)."
        )

    def _on_cmd_vel_in(self, msg: Twist):
        now_mono = time.monotonic()
        ok, reason = self.watchdog.update_cmd_vel(
            linear_x=msg.linear.x,
            linear_y=msg.linear.y,
            angular_z=msg.angular.z,
            now_mono=now_mono,
        )
        if not ok:
            self.get_logger().warn(f"Watchdog received invalid cmd_vel: {reason}")

    def _on_control_tick(self):
        """20 Hz control loop executing timeout evaluation and active zeroing."""
        now_mono = time.monotonic()
        vx, vy, wz, reason = self.watchdog.process_velocity(now_mono=now_mono)

        out_msg = Twist()
        out_msg.linear.x = float(vx)
        out_msg.linear.y = float(vy)
        out_msg.linear.z = 0.0
        out_msg.angular.x = 0.0
        out_msg.angular.y = 0.0
        out_msg.angular.z = float(wz)

        self.pub_cmd_out.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityWatchdogNode()

        def _sig_handler(sig, frame):
            if node is not None:
                try:
                    zero_msg = Twist()
                    for _ in range(5):
                        node.pub_cmd_out.publish(zero_msg)
                        time.sleep(0.01)
                except Exception:
                    pass
            sys.exit(0)

        signal.signal(signal.SIGTERM, _sig_handler)
        signal.signal(signal.SIGINT, _sig_handler)

        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:
        print(f"Unhandled exception in VelocityWatchdogNode: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                zero_msg = Twist()
                for _ in range(5):
                    node.pub_cmd_out.publish(zero_msg)
                    time.sleep(0.01)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
