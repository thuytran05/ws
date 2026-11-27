#!/usr/bin/env python3
"""Interactive / CLI fake passenger observation controller for hardware-free Gazebo escort demo."""

import json
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FakePassengerController(Node):
    def __init__(self):
        super().__init__(
            'fake_passenger_controller',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ],
        )
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_obs = self.create_publisher(String, '/tracking/observation', reliable_qos)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.latest_session = None
        self.latest_ui_status = None
        self.create_subscription(String, '/tracking/session_state', self._on_session, transient_qos)
        self.create_subscription(String, '/voice/ui_status', self._on_ui_status, transient_qos)

        self.active = False
        self.hand_raised = False
        self.dist_m = 1.6
        self.offset_x_norm = 0.0
        self.near_edge = False
        self._last_stamp = (0, 0)
        self._hand_timer = None
        self.create_timer(0.05, self._tick)

    def _on_session(self, msg: String):
        try:
            self.latest_session = json.loads(msg.data)
        except Exception:
            pass

    def _on_ui_status(self, msg: String):
        try:
            self.latest_ui_status = json.loads(msg.data)
        except Exception:
            pass

    def publish_initial_pose(self, x: float = -1.0, y: float = -0.5, yaw: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.pub_initial_pose.publish(msg)

    def raise_hand(self, duration_sec: float = 1.6):
        self.active = True
        self.hand_raised = True
        if self._hand_timer is not None:
            self._hand_timer.cancel()
        self._hand_timer = threading.Timer(duration_sec, self._lower_hand)
        self._hand_timer.daemon = True
        self._hand_timer.start()

    def _lower_hand(self):
        self.hand_raised = False

    def _tick(self):
        if not self.active:
            return
        now = self.get_clock().now().to_msg()
        sec, nanosec = now.sec, now.nanosec
        if (sec, nanosec) <= self._last_stamp:
            sec, nanosec = self._last_stamp
            nanosec += 50_000_000
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
        self._last_stamp = (sec, nanosec)

        cx = 320.0 + self.offset_x_norm * 320.0
        cand = {
            'track_id': 1,
            'is_confirmed': True,
            'is_ambiguous': False,
            'confidence': 0.95,
            'bbox_xyxy': [cx - 100.0, 100.0, cx + 100.0, 400.0],
            'torso_depth_m': float(self.dist_m),
            'depth_valid': True,
            'depth_proven': True,
            'center_offset_x_norm': float(self.offset_x_norm),
            'near_edge': bool(self.near_edge),
            'keypoints': [[cx, 200.0, 0.9]] * 17,
        }
        cand['keypoints'][5] = [cx - 20.0, 180.0, 0.9]
        cand['keypoints'][6] = [cx + 20.0, 180.0, 0.9]
        cand['keypoints'][9] = [cx - 20.0, 120.0 if self.hand_raised else 240.0, 0.95]
        cand['keypoints'][10] = [cx + 20.0, 240.0, 0.9]

        payload = {
            'schema_version': '1.0',
            'stamp': {'sec': sec, 'nanosec': nanosec},
            'frame_id': 'camera_color_optical_frame',
            'source_session_id': 'fake-passenger-cli',
            'data_quality': 'VALID_RGBD',
            'is_metric_valid': True,
            'candidates': [cand],
            'persons': [cand],
        }
        self.pub_obs.publish(String(data=json.dumps(payload)))


HELP = """Commands:
  1 / spawn  : Stand center (1.6m, hand down)
  2 / hand   : Raise hand 1.6s to lock passenger
  3 / left   : Shift left (offset -0.35)
  4 / right  : Shift right (offset +0.35)
  5 / lag    : Lag behind (dist=2.7m)
  6 / lose   : Disappear (stop observation)
  7 / reset  : Center & hand down (1.6m)
  p / pose   : Publish AMCL /initialpose (-1.0, -0.5, 0.0)
  s / status : Print /tracking/session_state & /voice/ui_status
  q / quit   : Exit
"""


def main(args=None):
    rclpy.init(args=args)
    node = FakePassengerController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(HELP, flush=True)
    try:
        while rclpy.ok():
            cmd = input('passenger> ').strip().lower()
            if cmd in ('1', 'spawn'):
                node.active = True
                node.hand_raised = False
                node.dist_m = 1.6
                node.offset_x_norm = 0.0
                node.near_edge = False
                print('-> Passenger spawned at center (1.6m)', flush=True)
            elif cmd in ('2', 'hand'):
                node.raise_hand(1.6)
                print('-> Raising hand for 1.6s...', flush=True)
            elif cmd in ('3', 'left'):
                node.active = True
                node.offset_x_norm = -0.35
                node.near_edge = False
                print('-> Shifted left (offset=-0.35)', flush=True)
            elif cmd in ('4', 'right'):
                node.active = True
                node.offset_x_norm = 0.35
                node.near_edge = False
                print('-> Shifted right (offset=+0.35)', flush=True)
            elif cmd in ('5', 'lag'):
                node.active = True
                node.dist_m = 2.7
                print('-> Lagging behind (dist=2.7m)', flush=True)
            elif cmd in ('6', 'lose'):
                node.active = False
                node.hand_raised = False
                print('-> Passenger disappeared (observation stopped)', flush=True)
            elif cmd in ('7', 'reset'):
                node.active = True
                node.hand_raised = False
                node.dist_m = 1.6
                node.offset_x_norm = 0.0
                node.near_edge = False
                print('-> Reset to center (1.6m)', flush=True)
            elif cmd in ('p', 'pose'):
                node.publish_initial_pose()
                print('-> Published /initialpose (-1.0, -0.5, 0.0)', flush=True)
            elif cmd in ('s', 'status'):
                print('session_state:', json.dumps(node.latest_session, ensure_ascii=False), flush=True)
                print('ui_status:', json.dumps(node.latest_ui_status, ensure_ascii=False), flush=True)
            elif cmd in ('h', 'help'):
                print(HELP, flush=True)
            elif cmd in ('q', 'quit', 'exit'):
                break
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
