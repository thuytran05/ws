#!/usr/bin/env python3
"""ROS 2 Node that manages passenger locking and session lifecycle for person_tracking.

Phase 4 & Phase 6 Tracking Manager Node (escort_tracking_manager):
- Subscribes to /tracking/observation (Schema v1.0).
- Runs PassengerLockManager to evaluate 17 body keypoints for one-hand raise gesture.
- Selects nearest qualified passenger and locks session (IDLE -> LOCKED_WAIT_DESTINATION).
- Manages navigation lifecycle:
    LOCKED_WAIT_DESTINATION -> NAVIGATING -> WAITING_FOR_PASSENGER -> ABORTING / ARRIVED / FAULT.
- Subscribes to /voice/nav_status and /voice/nav_state for mission synchronization.
- Publishes /tracking/session_state (TRANSIENT_LOCAL, 5 Hz heartbeat + event-driven).
- Publishes /tracking/safety_request (RELIABLE) on visibility/stream abort with retries.
- Strictly perception/session only: ZERO /cmd_vel publishers!
"""

import json
import sys
import time
from typing import Optional, List
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from person_tracking.passenger_lock_manager import (
    PassengerLockManager,
    SessionStateSnapshot,
    TrackingState,
)


class TrackingManagerNode(Node):
    """ROS 2 Node managing passenger lock authority and session state."""

    def __init__(self):
        super().__init__('escort_tracking_manager')

        self.declare_parameter('observation_topic', '/tracking/observation')
        self.declare_parameter('session_state_topic', '/tracking/session_state')
        self.declare_parameter('safety_request_topic', '/tracking/safety_request')
        self.declare_parameter('nav_status_topic', '/voice/nav_status')
        self.declare_parameter('nav_state_topic', '/voice/nav_state')
        self.declare_parameter('gesture_hold_time_sec', 1.5)
        self.declare_parameter('min_keypoint_conf', 0.35)
        self.declare_parameter('grace_period_sec', 0.25)
        self.declare_parameter('visibility_stop_sec', 0.40)
        self.declare_parameter('visibility_abort_sec', 1.20)
        self.declare_parameter('safety_retry_interval_sec', 0.20)
        self.declare_parameter('max_safety_retries', 3)
        self.declare_parameter('terminal_timeout_sec', 3.0)

        obs_topic = str(self.get_parameter('observation_topic').value)
        session_topic = str(self.get_parameter('session_state_topic').value)
        safety_topic = str(self.get_parameter('safety_request_topic').value)
        nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        nav_state_topic = str(self.get_parameter('nav_state_topic').value)
        hold_time = float(self.get_parameter('gesture_hold_time_sec').value)
        min_kp_conf = float(self.get_parameter('min_keypoint_conf').value)
        grace_sec = float(self.get_parameter('grace_period_sec').value)
        vis_stop_sec = float(self.get_parameter('visibility_stop_sec').value)
        vis_abort_sec = float(self.get_parameter('visibility_abort_sec').value)
        self.safety_retry_interval = float(self.get_parameter('safety_retry_interval_sec').value)
        self.max_safety_retries = int(self.get_parameter('max_safety_retries').value)
        self.terminal_timeout_sec = float(self.get_parameter('terminal_timeout_sec').value)

        self.manager = PassengerLockManager(
            gesture_hold_time_sec=hold_time,
            min_keypoint_conf=min_kp_conf,
            grace_period_sec=grace_sec,
            visibility_stop_sec=vis_stop_sec,
            visibility_abort_sec=vis_abort_sec,
        )

        # QoS Profiles
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.state_pub = self.create_publisher(String, session_topic, transient_qos)
        self.safety_pub = self.create_publisher(String, safety_topic, reliable_qos)

        # Subscribers
        self.obs_sub = self.create_subscription(String, obs_topic, self._on_observation, reliable_qos)
        self.voice_status_sub = self.create_subscription(String, nav_status_topic, self._on_voice_status, transient_qos)
        self.voice_state_sub = self.create_subscription(String, nav_state_topic, self._on_voice_nav_state, transient_qos)

        self.last_snapshot: Optional[SessionStateSnapshot] = None

        # Abort sequence state
        self.abort_in_progress = False
        self.abort_acked = False
        self.abort_ack_received_mono_time = 0.0
        self.current_abort_request_id: Optional[str] = None
        self.current_abort_session_id: Optional[str] = None
        self.abort_retries_sent = 0
        self.last_abort_send_mono_time = 0.0

        # Timers:
        # 1) Heartbeat timer (5 Hz / 0.20s)
        self.heartbeat_timer = self.create_timer(0.20, self._on_heartbeat)

        # 2) Watchdog timer (10 Hz / 0.10s)
        self.watchdog_timer = self.create_timer(0.10, self._on_watchdog_tick)

        self.get_logger().info(
            f"TrackingManagerNode initialized (gesture: {hold_time}s, stop: {vis_stop_sec}s, abort: {vis_abort_sec}s)."
        )

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

        # If observation triggered ABORTING and abort sequence is not yet started
        if snapshot.state == TrackingState.ABORTING.value and not self.abort_in_progress:
            self._initiate_abort()

        # Event-driven publish if state or lock changed
        if snapshot.state != prev_state or snapshot.locked != prev_locked:
            self.get_logger().info(
                f"Session state transition: {prev_state} -> {snapshot.state} (locked={snapshot.locked}, person_track_id={snapshot.person_track_id})"
            )
            self._publish_session_state(snapshot)

    def _on_voice_status(self, msg: String):
        try:
            status_data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid /voice/nav_status JSON: {exc}")
            return

        now_mono = time.monotonic()
        req_id = status_data.get("request_id")
        nav_event = str(status_data.get("nav_event", "")).upper()
        status_val = str(status_data.get("status", "")).upper()
        session_id = status_data.get("session_id", "")
        is_terminal = bool(status_data.get("is_terminal", False))

        # 1. Check if ACK for active abort request
        if self.abort_in_progress:
            if (
                nav_event == "CANCEL_ACKNOWLEDGED"
                and session_id == self.current_abort_session_id
                and req_id == self.current_abort_request_id
            ):
                if self.manager.bound_goal_attempt_id and status_data.get("goal_attempt_id"):
                    if status_data.get("goal_attempt_id") != self.manager.bound_goal_attempt_id:
                        self.get_logger().warn("ACK goal_attempt_id mismatch! Ignored.")
                        return
                self.get_logger().info(f"Received matching ACK for abort request: {self.current_abort_request_id}")
                self.abort_acked = True
                self.abort_ack_received_mono_time = now_mono

        # 2. Check if destination acceptance event
        if (nav_event == "NAVIGATION_STARTED" or status_val == "NAVIGATING") and not is_terminal:
            mission_id = status_data.get("mission_id")
            goal_attempt_id = status_data.get("goal_attempt_id")
            nav2_uuid = status_data.get("nav2_uuid")
            ok, reason = self.manager.on_destination_accepted(
                session_id,
                mission_id=mission_id,
                goal_attempt_id=goal_attempt_id,
                nav2_uuid=nav2_uuid,
                current_time_sec=now_mono,
            )
            if ok:
                self.get_logger().info(f"Destination accepted for session {session_id} -> NAVIGATING")
                now = self.get_clock().now().to_msg()
                snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
                self.last_snapshot = snapshot
                self._publish_session_state(snapshot)
            else:
                self.get_logger().warn(f"Destination acceptance rejected: {reason}")
            return

        # 3. Check for terminal event or session reset
        prev_state = self.manager.state.value
        handled, reason = self.manager.on_voice_nav_status(status_data, current_time_sec=now_mono)
        if handled:
            self.get_logger().info(f"Voice nav_status handled: {reason} ({prev_state} -> {self.manager.state.value})")
            if is_terminal and self.abort_in_progress:
                self.abort_in_progress = False
                self.abort_acked = False
            now = self.get_clock().now().to_msg()
            snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
            self.last_snapshot = snapshot
            self._publish_session_state(snapshot)

    def _on_voice_nav_state(self, msg: String):
        try:
            state_data = json.loads(msg.data)
        except Exception:
            return

        m_state = str(state_data.get("mission_state", "")).upper()
        if m_state == "NAVIGATING" and self.manager.state == TrackingState.LOCKED_WAIT_DESTINATION:
            now_mono = time.monotonic()
            sess_id = state_data.get("session_id")
            mission_id = state_data.get("mission_id")
            goal_attempt_id = state_data.get("goal_attempt_id")
            nav2_uuid = state_data.get("nav2_uuid")
            # Strict contract: session_id, mission_id, goal_attempt_id are mandatory
            if sess_id and sess_id == self.manager.session_id and mission_id and goal_attempt_id:
                ok, reason = self.manager.on_destination_accepted(
                    sess_id,
                    mission_id=mission_id,
                    goal_attempt_id=goal_attempt_id,
                    nav2_uuid=nav2_uuid,
                    current_time_sec=now_mono,
                )
                if ok:
                    self.get_logger().info(f"Nav state NAVIGATING accepted for session {sess_id}")
                    now = self.get_clock().now().to_msg()
                    snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
                    self.last_snapshot = snapshot
                    self._publish_session_state(snapshot)

    def _on_watchdog_tick(self):
        """Watchdog tick at 10 Hz ensures timeouts trigger even if observations halt completely."""
        now_mono = time.monotonic()
        prev_state = self.manager.state.value

        self.manager._update_watchdog(now_mono)

        if self.manager.state.value != prev_state:
            self.get_logger().warn(
                f"Watchdog triggered transition: {prev_state} -> {self.manager.state.value} ({self.manager.reason_code})"
            )
            if self.manager.state == TrackingState.ABORTING and not self.abort_in_progress:
                self._initiate_abort()

            now = self.get_clock().now().to_msg()
            snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
            self.last_snapshot = snapshot
            self._publish_session_state(snapshot)

        # Handle safety retries during abort sequence
        if self.abort_in_progress:
            if not self.abort_acked:
                if now_mono - self.last_abort_send_mono_time >= self.safety_retry_interval:
                    if self.abort_retries_sent < self.max_safety_retries:
                        self.abort_retries_sent += 1
                        self.last_abort_send_mono_time = now_mono
                        self.get_logger().warn(
                            f"Retrying safety request {self.current_abort_request_id} (retry {self.abort_retries_sent}/{self.max_safety_retries})..."
                        )
                        # Retain exact same current_abort_request_id across retries!
                        self._send_safety_request(self.current_abort_request_id, self.current_abort_session_id)
                    else:
                        self.get_logger().error(
                            f"Safety request {self.current_abort_request_id} timed out after {self.max_safety_retries} retries without ACK! Marking UNCONFIRMED_TERMINAL and FAULT."
                        )
                        self.manager.unconfirmed_terminal = True
                        self.manager.state = TrackingState.FAULT
                        self.manager.locked = False
                        self.manager.reason_code = "ACK_TIMEOUT_FAULT"
                        self.abort_in_progress = False
                        now = self.get_clock().now().to_msg()
                        snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
                        self.last_snapshot = snapshot
                        self._publish_session_state(snapshot)
            else:
                # ACK received: evaluate terminal deadline
                if now_mono - self.abort_ack_received_mono_time >= self.terminal_timeout_sec:
                    self.get_logger().error(
                        f"Terminal confirmation timed out ({self.terminal_timeout_sec}s) after ACK! Marking UNCONFIRMED_TERMINAL and FAULT."
                    )
                    self.manager.unconfirmed_terminal = True
                    self.manager.state = TrackingState.FAULT
                    self.manager.locked = False
                    self.manager.reason_code = "TERMINAL_TIMEOUT_EXCEEDED"
                    self.abort_in_progress = False
                    now = self.get_clock().now().to_msg()
                    snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
                    self.last_snapshot = snapshot
                    self._publish_session_state(snapshot)

    def _initiate_abort(self):
        self.abort_in_progress = True
        self.abort_acked = False
        self.current_abort_request_id = str(uuid.uuid4())
        self.current_abort_session_id = self.manager.session_id
        self.abort_retries_sent = 0
        self.last_abort_send_mono_time = time.monotonic()
        self.get_logger().warn(
            f"Initiating escort abort for session {self.current_abort_session_id} (request_id={self.current_abort_request_id})."
        )
        self._send_safety_request(self.current_abort_request_id, self.current_abort_session_id)

    def _send_safety_request(self, request_id: str, session_id: str):
        now = self.get_clock().now().to_msg()
        reason = self.manager.reason_code
        if "VISIBILITY" in reason:
            code = "VISIBILITY_TIMEOUT_EXCEEDED"
        elif "OBSERVATION" in reason:
            code = "CAMERA_FREEZE"
        elif "RESET" in reason:
            code = "EMERGENCY_STOP"
        else:
            code = "TARGET_LOST_PROLONGED"

        payload = {
            "schema_version": "1.0",
            "stamp": {"sec": now.sec, "nanosec": now.nanosec},
            "request_id": request_id,
            "session_id": session_id,
            "command": "ABORT",
            "reason_code": code,
            "message": f"Escort abort triggered: {reason}",
        }
        msg = String(data=json.dumps(payload))
        self.safety_pub.publish(msg)

    def _on_heartbeat(self):
        now = self.get_clock().now().to_msg()
        now_mono = time.monotonic()

        snapshot = self.manager._make_snapshot(now.sec, now.nanosec, now_mono)
        self.last_snapshot = snapshot
        self._publish_session_state(snapshot)

    def _publish_session_state(self, snapshot: SessionStateSnapshot):
        now = self.get_clock().now().to_msg()
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
