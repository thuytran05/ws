#!/usr/bin/env python3
"""ROS 2 Node that manages passenger locking and session lifecycle for person_tracking.

Phase 4 Tracking Manager Node (escort_tracking_manager):
- Subscribes to /tracking/observation (Schema v1.0).
- Runs PassengerLockManager to evaluate 17 body keypoints for one-hand raise gesture.
- Selects nearest qualified passenger and locks session (IDLE -> LOCKED_WAIT_DESTINATION).
- Publishes /tracking/session_state (TRANSIENT_LOCAL, 5 Hz heartbeat + event-driven).
- Strictly perception/session only: ZERO /cmd_vel publishers!
"""

import json
import sys
import time
from typing import Optional, List
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from person_tracking.passenger_lock_manager import PassengerLockManager, SessionStateSnapshot


class TrackingManagerNode(Node):
    """ROS 2 Node managing passenger lock authority and session state."""

    def __init__(self):
        super().__init__('escort_tracking_manager')

        self.declare_parameter('observation_topic', '/tracking/observation')
        self.declare_parameter('session_state_topic', '/tracking/session_state')
        self.declare_parameter('nav_status_topic', '/voice/nav_status')
        self.declare_parameter('gesture_hold_time_sec', 1.5)
        self.declare_parameter('min_keypoint_conf', 0.35)

        obs_topic = str(self.get_parameter('observation_topic').value)
        session_topic = str(self.get_parameter('session_state_topic').value)
        nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        hold_time = float(self.get_parameter('gesture_hold_time_sec').value)
        min_kp_conf = float(self.get_parameter('min_keypoint_conf').value)

        self.manager = PassengerLockManager(
            gesture_hold_time_sec=hold_time,
            min_keypoint_conf=min_kp_conf,
        )

        # QoS for /tracking/session_state: TRANSIENT_LOCAL, RELIABLE
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(String, session_topic, transient_qos)

        # Subscribers
        reliable_sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.obs_sub = self.create_subscription(String, obs_topic, self._on_observation, reliable_sub_qos)
        self.voice_sub = self.create_subscription(String, nav_status_topic, self._on_voice_status, transient_qos)

        self.last_published_state: Optional[str] = None
        self.last_snapshot: Optional[SessionStateSnapshot] = None

        # 5 Hz heartbeat timer for /tracking/session_state
        self.heartbeat_timer = self.create_timer(0.20, self._on_heartbeat)
        self.get_logger().info(f"TrackingManagerNode initialized (gesture threshold: {hold_time}s).")

    def _on_observation(self, msg: String):
        try:
            obs_payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid observation JSON: {exc}")
            return

        current_time = time.monotonic()
        prev_state = self.manager.state.value
        prev_locked = self.manager.locked

        snapshot = self.manager.process_observation(obs_payload, current_time_sec=current_time)
        self.last_snapshot = snapshot

        # Event-driven publish if state or lock changed
        if snapshot.state != prev_state or snapshot.locked != prev_locked:
            self.get_logger().info(
                f"Session state transition: {prev_state} -> {snapshot.state} (locked={snapshot.locked}, person_track_id={snapshot.person_track_id})"
            )
            self._publish_session_state(snapshot)

    def _on_voice_status(self, msg: String):
        # Reserved for Phase 6 goal completion handling
        pass

    def _on_heartbeat(self):
        now = self.get_clock().now().to_msg()
        now_mono = time.monotonic()
        obs_age = now_mono - self.manager.last_observation_mono_time
        obs_fresh = (self.manager.last_observation_mono_time > 0 and obs_age <= 0.40)

        if self.last_snapshot is not None:
            self.last_snapshot.stamp_sec = now.sec
            self.last_snapshot.stamp_nanosec = now.nanosec
            self.last_snapshot.obs_fresh = obs_fresh
            if not obs_fresh and self.last_snapshot.locked:
                self.last_snapshot.reason_code = "OBSERVATION_STALE"
            self._publish_session_state(self.last_snapshot)
        else:
            # Publish initial IDLE snapshot
            snapshot = self.manager._make_snapshot(now.sec, now.nanosec, obs_fresh=False)
            self._publish_session_state(snapshot)

    def _publish_session_state(self, snapshot: SessionStateSnapshot):
        now = self.get_clock().now().to_msg()
        # Update heartbeat timestamp to current ROS time
        snapshot.stamp_sec = now.sec
        snapshot.stamp_nanosec = now.nanosec
        payload = self.manager.to_json_dict(snapshot)
        msg = String(data=json.dumps(payload))
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrackingManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Unhandled exception in TrackingManagerNode: {exc}", file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
