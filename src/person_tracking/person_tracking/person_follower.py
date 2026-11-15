#!/usr/bin/env python3
"""Follow a selected person using a rear-facing camera; publish cmd_vel."""
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from person_tracking.control import commands, slew


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower', namespace='person_tracking')
        defaults = dict(desired_distance=1.5, distance_deadband=0.10,
                        angle_deadband=0.05, kp_distance=0.20, kp_yaw=0.45,
                        max_speed=0.10, max_yaw=0.20,
                        max_accel=0.10, max_yaw_accel=0.30,
                        timeout=0.40, max_capture_age=0.40,
                        expected_frame='camera_color_optical_frame')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.target = None
        self.last_receive = None
        self.last_stamp = -1
        self.reset_stamp = self.get_clock().now().nanoseconds
        self.active = False
        self.blocked = False
        self.vx = self.wz = 0.0
        self.tick_time = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(PointStamped, 'target_position', self.on_target, qos)
        self.create_subscription(Int32, 'selected_id', self.on_id, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Publishing cmd_vel; rear camera; reset after target loss or timeout')

    def p(self, name):
        return self.get_parameter(name).value

    def on_id(self, msg):
        if msg.data == -1:
            self.target = None
            self.active = self.blocked = False
            self.last_stamp = -1
            self.reset_stamp = self.get_clock().now().nanoseconds
            self.stop()

    def stop(self):
        # Fault stops bypass slew limits.
        self.vx = self.wz = 0.0
        self.pub.publish(Twist())

    def fault(self, reason):
        if self.active and not self.blocked:
            self.get_logger().warn(reason + ' -- call /person_tracking/reset')
            self.blocked = True
        self.target = None
        self.stop()

    def on_target(self, msg):
        if self.blocked:
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.reset_stamp or stamp <= self.last_stamp:
            return
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if msg.header.frame_id != self.p('expected_frame') or not (-0.05 <= age <= self.p('max_capture_age')):
            self.fault('Invalid frame or stale target')
            return
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        if not all(math.isfinite(v) for v in (x, y, z)) or not 0.4 <= z <= 5.0:
            self.fault('Target/depth invalid or target lost')
            return
        self.active = True
        self.target = (x, z)
        self.last_stamp = stamp
        self.last_receive = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(0.10, now - self.tick_time)
        self.tick_time = now
        capture_age = (self.get_clock().now().nanoseconds - self.last_stamp) / 1e9
        if self.blocked or self.target is None:
            self.stop()
            return
        if now - self.last_receive > self.p('timeout') or capture_age > self.p('max_capture_age') or capture_age < -0.05:
            self.fault('Target stream timeout')
            return
        vx, wz = commands(*self.target, desired=self.p('desired_distance'),
                          distance_deadband=self.p('distance_deadband'),
                          angle_deadband=self.p('angle_deadband'),
                          kp_distance=self.p('kp_distance'), kp_yaw=self.p('kp_yaw'),
                          max_speed=self.p('max_speed'), max_yaw=self.p('max_yaw'))
        self.vx = slew(self.vx, vx, self.p('max_accel'), dt)
        self.wz = slew(self.wz, wz, self.p('max_yaw_accel'), dt)
        msg = Twist()
        msg.linear.x, msg.angular.z = self.vx, self.wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
