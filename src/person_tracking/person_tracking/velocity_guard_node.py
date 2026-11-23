#!/usr/bin/env python3
"""ROS 2 Node for escort robot velocity guard in person_tracking Phase 7.

Safety Principles & Architecture:
- Subscribes to:
    1. /tracking/cmd_vel_smoothed (geometry_msgs/msg/Twist) from Nav2 velocity smoother.
    2. /tracking/session_state (std_msgs/msg/String, TRANSIENT_LOCAL) from tracking manager.
    3. /tracking/observation (std_msgs/msg/String, RELIABLE) from perception pipeline.
- Publishes to:
    /tracking/cmd_vel_guarded (geometry_msgs/msg/Twist) at 20 Hz (and on command updates).
- Downstream Chain:
    cmd_vel_smoothed -> escort_velocity_guard -> cmd_vel_guarded -> Collision Monitor -> cmd_vel -> base.
- Invariants:
    * Strictly ZERO /cmd_vel publishers in person_tracking.
    * Active continuous zeroing at 20 Hz when gate is closed or safety violated.
    * Monotonic time-based watchdog (immune to /clock manipulation).
    * Never causes self-motion if Nav2 command is zero.
    * linear.y = 0.0 strictly.
"""

import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from person_tracking.velocity_guard import VelocityGuard, GuardLimits


class VelocityGuardNode(Node):
    """ROS 2 Node executing VelocityGuard safety policy."""

    def __init__(self):
        super().__init__('escort_velocity_guard')

        # Declare parameters
        self.declare_parameter('cmd_vel_in_topic', '/tracking/cmd_vel_smoothed')
        self.declare_parameter('cmd_vel_out_topic', '/tracking/cmd_vel_guarded')
        self.declare_parameter('session_state_topic', '/tracking/session_state')
        self.declare_parameter('observation_topic', '/tracking/observation')
        self.declare_parameter('safe_dist_min', 1.2)
        self.declare_parameter('safe_dist_max', 2.0)
        self.declare_parameter('stop_dist', 2.5)
        self.declare_parameter('resume_dist', 2.2)
        self.declare_parameter('max_linear_x', 0.50)
        self.declare_parameter('max_angular_z', 0.40)
        self.declare_parameter('trim_angular_max', 0.20)
        self.declare_parameter('deadband_offset', 0.08)
        self.declare_parameter('trim_kp', 0.35)
        self.declare_parameter('near_edge_threshold', 0.70)
        self.declare_parameter('near_edge_linear_scale', 0.50)
        self.declare_parameter('cmd_stale_timeout_sec', 0.20)
        self.declare_parameter('passenger_timeout_sec', 0.40)
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)

        # Read parameters
        cmd_in_topic = str(self.get_parameter('cmd_vel_in_topic').value)
        cmd_out_topic = str(self.get_parameter('cmd_vel_out_topic').value)
        session_topic = str(self.get_parameter('session_state_topic').value)
        obs_topic = str(self.get_parameter('observation_topic').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        limits = GuardLimits(
            safe_dist_min=float(self.get_parameter('safe_dist_min').value),
            safe_dist_max=float(self.get_parameter('safe_dist_max').value),
            stop_dist=float(self.get_parameter('stop_dist').value),
            resume_dist=float(self.get_parameter('resume_dist').value),
            max_linear_x=float(self.get_parameter('max_linear_x').value),
            max_angular_z=float(self.get_parameter('max_angular_z').value),
            trim_angular_max=float(self.get_parameter('trim_angular_max').value),
            deadband_offset=float(self.get_parameter('deadband_offset').value),
            trim_kp=float(self.get_parameter('trim_kp').value),
            near_edge_threshold=float(self.get_parameter('near_edge_threshold').value),
            near_edge_linear_scale=float(self.get_parameter('near_edge_linear_scale').value),
            cmd_stale_timeout_sec=float(self.get_parameter('cmd_stale_timeout_sec').value),
            passenger_timeout_sec=float(self.get_parameter('passenger_timeout_sec').value),
            heartbeat_timeout_sec=float(self.get_parameter('heartbeat_timeout_sec').value),
        )

        self.guard = VelocityGuard(limits=limits)

        # QoS Profiles
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        default_qos = 10

        # Subscriptions
        self.sub_cmd_in = self.create_subscription(Twist, cmd_in_topic, self._on_cmd_vel_in, 1)
        self.sub_session = self.create_subscription(String, session_topic, self._on_session_state, transient_qos)
        self.sub_obs = self.create_subscription(String, obs_topic, self._on_observation, reliable_qos)

        # Publisher to guarded velocity
        self.pub_cmd_guarded = self.create_publisher(Twist, cmd_out_topic, default_qos)

        # 20 Hz Monotonic Timer for active zeroing / watchdog
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 0.05
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(timer_period, self._on_control_tick, clock=self.steady_clock)

        self.get_logger().info(
            f"VelocityGuardNode initialized (in: {cmd_in_topic}, out: {cmd_out_topic}, rate: {rate_hz} Hz)."
        )

    def _on_cmd_vel_in(self, msg: Twist):
        now_mono = time.monotonic()
        self.guard.update_cmd_vel(
            linear_x=msg.linear.x,
            linear_y=msg.linear.y,
            angular_z=msg.angular.z,
            now_mono=now_mono,
        )

    def _on_session_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid session state JSON: {exc}")
            return
        now_mono = time.monotonic()
        self.guard.update_session_state(payload, now_mono=now_mono)

    def _on_observation(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid observation JSON: {exc}")
            return
        now_mono = time.monotonic()
        self.guard.update_observation(payload, now_mono=now_mono)

    def _on_control_tick(self):
        """20 Hz control loop executing safety evaluation and publishing cmd_vel_guarded."""
        now_mono = time.monotonic()
        vx, vy, wz, reason = self.guard.process_velocity(now_mono=now_mono)

        out_msg = Twist()
        out_msg.linear.x = float(vx)
        out_msg.linear.y = float(vy)
        out_msg.linear.z = 0.0
        out_msg.angular.x = 0.0
        out_msg.angular.y = 0.0
        out_msg.angular.z = float(wz)

        self.pub_cmd_guarded.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityGuardNode()

        def _sig_handler(sig, frame):
            if node is not None:
                try:
                    zero_msg = Twist()
                    for _ in range(5):
                        node.pub_cmd_guarded.publish(zero_msg)
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
        print(f"Unhandled exception in VelocityGuardNode: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                zero_msg = Twist()
                for _ in range(5):
                    node.pub_cmd_guarded.publish(zero_msg)
                    time.sleep(0.01)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
