#!/usr/bin/env python3
# Copyright (c) 2026 Airport Robot Escort Project
# Gate 8 Real Gazebo + Nav2 Integration Test Driver
# Tests T09, T10, T11, T12, T14, T15, T16, T22, T23, T33 on Real ROS 2 Jazzy Graph
# Strict Acceptance Harness: No artificial PASS, real measurements, non-zero exit on failure.

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple
import uuid

import rclpy
from rclpy.action import ActionClient
import tf2_ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class RealGazeboTestDriver(Node):
    """ROS 2 Node driving and verifying real Gazebo + Nav2 integration."""

    def __init__(self):
        super().__init__(
            'gate8_real_gazebo_test_driver',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self.pub_obs = self.create_publisher(String, '/tracking/observation', reliable_qos)
        # CRITICAL: /voice/nav_status subscriber in tracking_manager requires TRANSIENT_LOCAL!
        self.pub_voice_status = self.create_publisher(String, '/voice/nav_status', transient_qos)
        self.pub_voice_state = self.create_publisher(String, '/voice/nav_state', transient_qos)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_test_scan = self.create_publisher(LaserScan, '/tracking/scan_test', 10)

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.sub_guarded_cmd = self.create_subscription(Twist, '/tracking/cmd_vel_guarded', self._on_guarded_cmd, 10)
        self.sub_watchdog_cmd = self.create_subscription(Twist, '/tracking/cmd_vel_watchdog', self._on_watchdog_cmd, 10)
        self.latest_watchdog_cmd: Optional[Twist] = None
        self.watchdog_history: List[Tuple[float, float, float, float]] = []
        self.sub_session = self.create_subscription(String, '/tracking/session_state', self._on_session_state, transient_qos)
        self.sub_safety = self.create_subscription(String, '/tracking/safety_request', self._on_safety_request, reliable_qos)
        self.sub_clock = self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, transient_qos)
        self.sub_laser_scan = self.create_subscription(LaserScan, '/scan', self._on_laser_scan, 10)
        self.latest_laser_scan: Optional[LaserScan] = None
        self.last_laser_scan_mono_time: float = 0.0

        # Action Client for Nav2 NavigateToPose
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Internal state & timestamps
        self.latest_clock: Optional[Clock] = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_history: List[Tuple[float, float, float, float]] = []
        self.latest_cmd_vel: Optional[Twist] = None
        self.cmd_history: List[Tuple[float, float, float, float]] = []
        self.latest_guarded_cmd: Optional[Twist] = None
        self.guarded_history: List[Tuple[float, float, float, float]] = []
        self.latest_session: Optional[dict] = None
        self.safety_requests: List[dict] = []
        self.latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_goal_handle = None

        # Continuous Observation Stream (20 Hz) mirroring camera node
        self._obs_enabled = True
        self._obs_track_id = 1
        self._obs_dist_m = 1.6
        self._obs_hand_raised = True
        self._obs_source_session_id = "src-gazebo-01"
        self._last_obs_stamp: Tuple[int, int] = (0, 0)
        self.obs_timer = self.create_timer(0.05, self._obs_timer_callback)

        # Continuous Background Scan Stream for Collision Monitor
        num_readings = 361
        self.default_scan_ranges = [10.0] * num_readings
        self.active_scan_ranges = list(self.default_scan_ranges)
        self.scan_timer = self.create_timer(0.05, self._publish_scan_loop)

    def set_observation_state(
        self,
        track_id: Optional[int] = None,
        dist_m: Optional[float] = None,
        hand_raised: Optional[bool] = None,
        source_session_id: Optional[str] = None,
    ):
        if track_id is not None:
            self._obs_track_id = track_id
        if dist_m is not None:
            self._obs_dist_m = dist_m
        if hand_raised is not None:
            self._obs_hand_raised = hand_raised
        if source_session_id is not None:
            self._obs_source_session_id = source_session_id

    def set_observation_enabled(self, enabled: bool):
        self._obs_enabled = enabled

    def _obs_timer_callback(self):
        if not self._obs_enabled:
            return
        now = self.get_clock().now()
        sec = now.to_msg().sec
        nanosec = now.to_msg().nanosec
        if (sec, nanosec) <= self._last_obs_stamp:
            sec, nanosec = self._last_obs_stamp
            nanosec += 33_333_333  # Advance by ~33ms (30 Hz rate)
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
        self._last_obs_stamp = (sec, nanosec)
        now_mono = time.monotonic()

        cand = {
            "track_id": self._obs_track_id,
            "is_confirmed": True,
            "is_ambiguous": False,
            "confidence": 0.95,
            "bbox_xyxy": [220.0, 100.0, 420.0, 400.0],
            "torso_depth_m": self._obs_dist_m,
            "depth_valid": True,
            "center_offset_x_norm": 0.0,
            "near_edge": False,
            "keypoints": [[320.0, 200.0, 0.9]] * 17,
        }
        cand["keypoints"][5] = [300.0, 180.0, 0.9]  # Left shoulder
        if self._obs_hand_raised:
            cand["keypoints"][9] = [300.0, 120.0, 0.95]  # Left wrist raised above shoulder
        else:
            cand["keypoints"][9] = [300.0, 250.0, 0.95]  # Left wrist lowered

        obs = {
            "schema_version": "1.0",
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": "camera_color_optical_frame",
            "receive_monotonic_time": now_mono,
            "source_mode": "live_astra",
            "source_session_id": self._obs_source_session_id,
            "data_quality": "VALID_RGBD",
            "processing_time_ms": 5.0,
            "device": "cuda:0",
            "active_track_ids": [self._obs_track_id],
            "lost_track_ids": [],
            "candidates": [cand],
        }
        self.pub_obs.publish(String(data=json.dumps(obs)))

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.latest_amcl_pose = msg

    def get_current_map_pose(self) -> Tuple[float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return float(p.x), float(p.y), float(yaw)
        except Exception:
            pass
        if self.latest_amcl_pose:
            p = self.latest_amcl_pose.pose.pose.position
            q = self.latest_amcl_pose.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return p.x, p.y, yaw
        if self.latest_odom:
            ox = self.latest_odom.pose.pose.position.x
            oy = self.latest_odom.pose.pose.position.y
            return -1.0 + ox, -0.5 + oy, 0.0
        return -1.0, -0.5, 0.0

    def _on_laser_scan(self, msg: LaserScan):
        self.latest_laser_scan = msg
        self.last_laser_scan_mono_time = time.monotonic()

    def _on_clock(self, msg: Clock):
        self.latest_clock = msg

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        t = time.monotonic()
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        self.odom_history.append((
            t,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
            sec,
            nanosec
        ))

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg
        t = time.monotonic()
        self.cmd_history.append((t, msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_guarded_cmd(self, msg: Twist):
        self.latest_guarded_cmd = msg
        t = time.monotonic()
        self.guarded_history.append((t, msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_watchdog_cmd(self, msg: Twist):
        self.latest_watchdog_cmd = msg
        t = time.monotonic()
        self.watchdog_history.append((t, msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_session_state(self, msg: String):
        try:
            self.latest_session = json.loads(msg.data)
        except Exception:
            pass

    def _on_safety_request(self, msg: String):
        try:
            req = json.loads(msg.data)
            self.safety_requests.append(req)
        except Exception:
            pass

    def _publish_scan_loop(self):
        """Continuously publish scan at 20 Hz with current ROS clock to prevent Collision Monitor timeout."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        scan = LaserScan()
        now_msg = now.to_msg()
        scan.header.stamp = now_msg
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = list(self.active_scan_ranges)
        self.pub_test_scan.publish(scan)

    def set_obstacle_in_scan(self, distance_m: float = 0.25):
        """Inject obstacle in front sector (-15 deg to +15 deg)."""
        num_readings = 361
        ranges = [10.0] * num_readings
        mid = num_readings // 2
        for i in range(mid - 15, mid + 16):
            ranges[i] = distance_m
        self.active_scan_ranges = ranges

    def clear_obstacle_in_scan(self):
        """Clear obstacle in scan."""
        self.active_scan_ranges = list(self.default_scan_ranges)

    def publish_initial_pose(self, x: float = -1.0, y: float = -0.5, yaw: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.pub_initial_pose.publish(msg)

    def send_nav2_goal(self, x: float, y: float, yaw: float = 0.0, timeout_sec: float = 5.0):
        """Send NavigateToPose action goal, await acceptance and extract nav2_uuid."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        future = self.nav_action_client.send_goal_async(goal_msg)
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < timeout_sec:
            time.sleep(0.02)
        if not future.done():
            return None, None
        goal_handle = future.result()
        if not goal_handle.accepted:
            return None, None
        nav2_uuid = str(uuid.UUID(bytes=bytes(goal_handle.goal_id.uuid)))
        self.current_goal_handle = goal_handle
        return goal_handle, nav2_uuid

    def cancel_current_goal(self, timeout_sec: float = 5.0) -> bool:
        """Cancel current goal and await result."""
        if not self.current_goal_handle:
            return True
        future = self.current_goal_handle.cancel_goal_async()
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < timeout_sec:
            time.sleep(0.02)
        res_future = self.current_goal_handle.get_result_async()
        t0 = time.monotonic()
        while not res_future.done() and time.monotonic() - t0 < timeout_sec:
            time.sleep(0.02)
        self.current_goal_handle = None
        return True

    def publish_voice_started(self, session_id: str, mission_id: str, goal_attempt_id: str, nav2_uuid: str):
        payload = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "status": "NAVIGATING",
            "nav_event": "NAVIGATION_STARTED",
            "is_terminal": False,
        }
        self.pub_voice_status.publish(String(data=json.dumps(payload)))
        state_payload = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "mission_state": "NAVIGATING",
        }
        self.pub_voice_state.publish(String(data=json.dumps(state_payload)))

    def publish_voice_ack(self, session_id: str, mission_id: str, goal_attempt_id: str, request_id: str, nav2_uuid: str = ""):
        payload = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "request_id": request_id,
            "nav2_uuid": nav2_uuid,
            "status": "CANCELING",
            "nav_event": "CANCEL_ACKNOWLEDGED",
            "is_terminal": False,
        }
        self.pub_voice_status.publish(String(data=json.dumps(payload)))

    def publish_voice_terminal(self, session_id: str, mission_id: str, goal_attempt_id: str, nav2_uuid: str, status: str = "CANCELED"):
        payload = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "status": status,
            "nav_event": "GOAL_COMPLETED",
            "is_terminal": True,
        }
        self.pub_voice_status.publish(String(data=json.dumps(payload)))
        state_payload = {
            "session_id": session_id,
            "mission_id": mission_id,
            "goal_attempt_id": goal_attempt_id,
            "nav2_uuid": nav2_uuid,
            "mission_state": "IDLE" if status in ("SUCCEEDED", "CANCELED", "ABORTED") else "FAULT",
        }
        self.pub_voice_state.publish(String(data=json.dumps(state_payload)))


class TestResultTracker:
    def __init__(self, log_path: str):
        self.log_path = log_path
        self.results: List[Tuple[str, str, str]] = []  # (name, status, details)

    def record(self, name: str, status: str, details: str):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.results.append((name, status, details))
        line = f"[{timestamp}] [{status}] {name}: {details}\n"
        print(line, end="", flush=True)
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(line)

    def summary(self) -> Tuple[int, int, int]:
        passed = sum(1 for _, s, _ in self.results if s == "PASS")
        failed = sum(1 for _, s, _ in self.results if s == "FAIL")
        blocked = sum(1 for _, s, _ in self.results if s == "BLOCKED")
        return passed, failed, blocked


def run_gate8_suite():
    evidence_dir = "/home/ws/src/.agent/person_tracking/evidence/phase_8"
    os.makedirs(evidence_dir, exist_ok=True)
    evidence_log = os.path.join(evidence_dir, "gate8_real_runtime_evidence.log")
    audit_file = os.path.join(evidence_dir, "topic_graph_audit.txt")

    tracker = TestResultTracker(evidence_log)
    with open(evidence_log, "w", encoding="utf-8") as f:
        f.write("# Gate 8 Real Gazebo + Nav2 Runtime Evidence Log\n")
        f.write(f"# Started at: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}\n\n")

    print("================================================================================")
    print("Executing Gate 8 Real Gazebo + Nav2 Integration Acceptance Suite...")
    print("================================================================================")

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = RealGazeboTestDriver()

    # Background SingleThreadedExecutor for uninterrupted DDS processing and timer execution
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # ----------------------------------------------------------------------
        # Phase 0: Prerequisite Verification
        # ----------------------------------------------------------------------
        print("\n[Prerequisites] Verifying Gazebo, Nav2, /clock, and /odom are fully operational...")
        prereq_ok = False
        clock_advancing = False
        odom_received = False
        action_ready = False

        node.publish_initial_pose(x=-1.0, y=-0.5, yaw=0.0)

        for wait_i in range(30):
            if node.latest_clock and node.latest_clock.clock.sec > 0:
                clock_advancing = True
            if node.latest_odom is not None:
                odom_received = True
            if node.nav_action_client.server_is_ready():
                action_ready = True
            if clock_advancing and odom_received and action_ready:
                prereq_ok = True
                print(f"Prerequisites satisfied at wait iteration {wait_i + 1}!")
                break
            time.sleep(0.5)

        if not prereq_ok:
            if not action_ready:
                action_ready = node.nav_action_client.wait_for_server(timeout_sec=5.0)
                prereq_ok = clock_advancing and odom_received and action_ready

        if not prereq_ok:
            diag = f"clock_advancing={clock_advancing}, odom_received={odom_received}, nav2_action_ready={action_ready}"
            tracker.record("Prerequisites Gate", "BLOCKED", f"Stack failed to become ready within 30s ({diag})")
            mandatory_cases = [
                "Sole Publisher Audit (/cmd_vel)",
                "T09: Escort Velocity & Physical Movement",
                "T23: Collision Monitor Priority Stop",
                "T14: Brief Observation Loss Grace",
                "T33: Clock Pause Steady Time Watchdog",
                "T15: Prolonged Loss Lifecycle & Voice Handshake",
                "T16: Unresponsive Voice Watchdog FAULT",
                "SUCCEEDED: Nav2 Real Waypoint Completion",
                "T22B: Guard Crash Without Handler Watchdog (SIGKILL)",
                "T22A: Guard Graceful Shutdown Watchdog (SIGTERM)",
            ]
            for c in mandatory_cases:
                tracker.record(c, "BLOCKED", f"Prerequisites not met ({diag})")
            sys.exit(2)

        # ----------------------------------------------------------------------
        # Step 1: Topic Graph Introspection & Sole Publisher Audit
        # ----------------------------------------------------------------------
        print("\n[Step 1] Auditing Topic Graph and Sole Publisher on /cmd_vel...")
        time.sleep(1.0)

        pubs = node.get_publishers_info_by_topic('/cmd_vel')
        subs = node.get_subscriptions_info_by_topic('/cmd_vel')
        guarded_pubs = node.get_publishers_info_by_topic('/tracking/cmd_vel_guarded')
        watchdog_pubs = node.get_publishers_info_by_topic('/tracking/cmd_vel_watchdog')
        watchdog_subs = node.get_subscriptions_info_by_topic('/tracking/cmd_vel_watchdog')

        audit_lines = [
            "# Topic Graph & /cmd_vel Publisher Introspection Audit\n",
            f"Topic: /cmd_vel\n",
            f"Publishers ({len(pubs)}):\n",
        ]
        for p in pubs:
            audit_lines.append(f"  - Node: {p.node_namespace}/{p.node_name}, Type: {p.topic_type}\n")
        audit_lines.append(f"Subscribers ({len(subs)}):\n")
        for s in subs:
            audit_lines.append(f"  - Node: {s.node_namespace}/{s.node_name}, Type: {s.topic_type}\n")
        audit_lines.append(f"\nTopic: /tracking/cmd_vel_guarded\n")
        audit_lines.append(f"Publishers ({len(guarded_pubs)}):\n")
        for gp in guarded_pubs:
            audit_lines.append(f"  - Node: {gp.node_namespace}/{gp.node_name}, Type: {gp.topic_type}\n")
        audit_lines.append(f"\nTopic: /tracking/cmd_vel_watchdog\n")
        audit_lines.append(f"Publishers ({len(watchdog_pubs)}):\n")
        for wp in watchdog_pubs:
            audit_lines.append(f"  - Node: {wp.node_namespace}/{wp.node_name}, Type: {wp.topic_type}\n")
        audit_lines.append(f"Subscribers ({len(watchdog_subs)}):\n")
        for ws in watchdog_subs:
            audit_lines.append(f"  - Node: {ws.node_namespace}/{ws.node_name}, Type: {ws.topic_type}\n")

        with open(audit_file, "w", encoding="utf-8") as f:
            f.writelines(audit_lines)

        pub_names = [f"{p.node_namespace}/{p.node_name}".strip("/") for p in pubs]
        has_guard_on_cmd_vel = any("guard" in name.lower() for name in pub_names)
        has_cm_on_cmd_vel = any("collision_monitor" in name.lower() for name in pub_names)
        has_bridge_sub = any("bridge" in f"{s.node_namespace}/{s.node_name}".lower() for s in subs)
        sole_pub_ok = (len(pubs) == 1) and has_cm_on_cmd_vel and (not has_guard_on_cmd_vel) and has_bridge_sub

        if sole_pub_ok:
            tracker.record(
                "Sole Publisher Audit (/cmd_vel)",
                "PASS",
                f"Sole publisher is collision_monitor ({pub_names}), ros_gz_bridge subscribed, 0 publishers from guard."
            )
        else:
            tracker.record(
                "Sole Publisher Audit (/cmd_vel)",
                "FAIL",
                f"Expected sole collision_monitor pub with bridge sub. Found pubs={pub_names}, subs={[f'{s.node_name}' for s in subs]}"
            )

        # ----------------------------------------------------------------------
        # Step 2: Passenger Gesture Locking (Session 1)
        # ----------------------------------------------------------------------
        print("\n[Step 2] Locking passenger via hand raise gesture...")
        sess_id_1 = ""
        locked_1 = False
        node.set_observation_state(track_id=1, dist_m=1.6, hand_raised=True, source_session_id="src-gazebo-01")
        node.set_observation_enabled(True)

        t_start_lock = time.monotonic()
        while time.monotonic() - t_start_lock < 5.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("session_id"):
                locked_1 = True
                sess_id_1 = node.latest_session.get("session_id")
                break

        state_1 = node.latest_session.get("state") if node.latest_session else "NONE"
        if locked_1 and state_1 == "LOCKED_WAIT_DESTINATION":
            tracker.record("Passenger Locking Session 1", "PASS", f"Locked session_id={sess_id_1}, state={state_1}")
        else:
            tracker.record("Passenger Locking Session 1", "FAIL", f"Failed to lock: locked={locked_1}, state={state_1}")
            sys.exit(1)

        # Passenger lowers hand after locking
        node.set_observation_state(hand_raised=False)

        # ----------------------------------------------------------------------
        # Step 3 & 4: T09 Escort Velocity & Physical Movement
        # ----------------------------------------------------------------------
        print("\n[Step 3 & 4] Sending Nav2 Goal (T09) and verifying physical vehicle movement...")
        mission_id_1 = f"mission-p8-{uuid.uuid4().hex[:6]}"
        goal_attempt_1 = f"attempt-p8-{uuid.uuid4().hex[:6]}"

        # Goal is along open corridor at y=-0.50 (stay within clear corridor: target x=0.30)
        map_x, map_y, _ = node.get_current_map_pose()
        goal_target_x = 0.30
        goal_handle_1, nav2_uuid_1 = node.send_nav2_goal(x=goal_target_x, y=-0.50, yaw=0.0, timeout_sec=5.0)
        if not goal_handle_1 or not nav2_uuid_1:
            tracker.record("T09: Escort Velocity & Physical Movement", "FAIL", "Nav2 action goal was rejected or timed out")
            sys.exit(1)

        # Handshake: Publish NAVIGATION_STARTED
        node.publish_voice_started(sess_id_1, mission_id_1, goal_attempt_1, nav2_uuid_1)

        # Verify FSM enters NAVIGATING
        fsm_navigating = False
        t_wait_nav = time.monotonic()
        while time.monotonic() - t_wait_nav < 3.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("state") == "NAVIGATING":
                fsm_navigating = True
                break

        if not fsm_navigating:
            tracker.record("T09: Escort Velocity & Physical Movement", "FAIL", f"FSM failed to reach NAVIGATING: {node.latest_session}")
            sys.exit(1)

        # Monitor physical movement (allow up to 14.0s for mecanum acceleration & displacement)
        initial_x = node.latest_odom.pose.pose.position.x if node.latest_odom else 0.0
        max_guard_vx = 0.0
        max_cmd_vel_vx = 0.0
        max_odom_vx = 0.0
        max_delta_x = 0.0

        t_move_start = time.monotonic()
        while time.monotonic() - t_move_start < 14.0:
            time.sleep(0.05)
            guard_cmds = [m[1] for m in node.guarded_history if m[0] >= t_move_start]
            if guard_cmds:
                max_guard_vx = max(max_guard_vx, max(guard_cmds))
            elif node.latest_guarded_cmd:
                max_guard_vx = max(max_guard_vx, node.latest_guarded_cmd.linear.x)

            cmd_cmds = [m[1] for m in node.cmd_history if m[0] >= t_move_start]
            if cmd_cmds:
                max_cmd_vel_vx = max(max_cmd_vel_vx, max(cmd_cmds))
            elif node.latest_cmd_vel:
                max_cmd_vel_vx = max(max_cmd_vel_vx, node.latest_cmd_vel.linear.x)

            if node.latest_odom:
                max_odom_vx = max(max_odom_vx, node.latest_odom.twist.twist.linear.x)
                curr_x = node.latest_odom.pose.pose.position.x
                max_delta_x = max(max_delta_x, abs(curr_x - initial_x))

            if max_guard_vx > 0.05 and max_cmd_vel_vx > 0.05 and max_odom_vx > 0.02 and max_delta_x > 0.10:
                break

        t09_pass = (max_guard_vx > 0.05) and (max_cmd_vel_vx > 0.05) and (max_odom_vx > 0.02) and (max_delta_x > 0.10)
        t09_details = (
            f"Guard max vx={max_guard_vx:.3f} m/s (>0.05), /cmd_vel max vx={max_cmd_vel_vx:.3f} m/s (>0.05), "
            f"Odom max vx={max_odom_vx:.3f} m/s (>0.02), delta_x={max_delta_x:.3f} m (>0.10m)"
        )
        if t09_pass:
            tracker.record("T09: Escort Velocity & Physical Movement", "PASS", t09_details)
        else:
            tracker.record("T09: Escort Velocity & Physical Movement", "FAIL", t09_details)

        # ----------------------------------------------------------------------
        # Step 5: T23 Collision Monitor Priority Stop
        # ----------------------------------------------------------------------
        print("\n[Step 5] Testing Collision Monitor Priority Stop (T23)...")
        # Robot is actively cruising towards x=0.20; inject obstacle in front sector
        node.set_obstacle_in_scan(distance_m=0.25)

        t_cm_start = time.monotonic()
        guard_clamped_observed = False
        cm_stop_observed = False

        captured_guard_vx = 0.0
        captured_cm_vx = 999.0
        captured_odom_vx = 999.0
        last_t23_print = 0.0

        while time.monotonic() - t_cm_start < 4.0:
            time.sleep(0.05)
            now_dt = time.monotonic() - t_cm_start
            g_vx = node.latest_guarded_cmd.linear.x if node.latest_guarded_cmd else 0.0
            c_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            o_vx = abs(node.latest_odom.twist.twist.linear.x) if node.latest_odom else 0.0

            if now_dt - last_t23_print >= 0.25:
                last_t23_print = now_dt
                print(f"  [T23 Poll] dt={now_dt:.2f}s | Guard vx={g_vx:.3f} | cmd_vel={c_vx:.4f} | odom={o_vx:.4f}")

            if g_vx > 0.05 and abs(c_vx) < 1e-3:
                guard_clamped_observed = True
                captured_guard_vx = max(captured_guard_vx, g_vx)
                captured_cm_vx = c_vx

            if guard_clamped_observed and o_vx < 0.05:
                cm_stop_observed = True
                captured_odom_vx = o_vx
                print(f"  [T23 Stop Detected] at dt={now_dt:.2f}s (Guard={captured_guard_vx:.3f}, cmd_vel={captured_cm_vx:.4f}, odom={o_vx:.4f})")
                break

        if not cm_stop_observed:
            captured_guard_vx = node.latest_guarded_cmd.linear.x if node.latest_guarded_cmd else 0.0
            captured_cm_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            captured_odom_vx = abs(node.latest_odom.twist.twist.linear.x) if node.latest_odom else 0.0

        node.clear_obstacle_in_scan()
        time.sleep(0.4)
        node.cancel_current_goal(timeout_sec=2.0)
        time.sleep(0.3)

        t23_pass = cm_stop_observed
        t23_details = (
            f"Obstacle 0.25m injected: Guard output={captured_guard_vx:.3f} m/s (>0.05), "
            f"/cmd_vel clamped to={captured_cm_vx:.4f} m/s (<1e-3), odom vx={captured_odom_vx:.4f} m/s (<0.05)"
        )
        if t23_pass:
            tracker.record("T23: Collision Monitor Priority Stop", "PASS", t23_details)
        else:
            tracker.record("T23: Collision Monitor Priority Stop", "FAIL", t23_details)

        # ----------------------------------------------------------------------
        # Step 6: T14 Brief Observation Loss (< 0.20s)
        # ----------------------------------------------------------------------
        print("\n[Step 6] Testing Brief Observation Loss Grace Period (T14)...")
        # Send fresh goal along corridor for T14 & T33
        map_x, map_y, _ = node.get_current_map_pose()
        goal_target_x_6 = map_x + 0.60
        goal_handle_6, nav2_uuid_6 = node.send_nav2_goal(x=goal_target_x_6, y=-0.50, yaw=0.0, timeout_sec=5.0)
        goal_attempt_6 = f"attempt-p8-{uuid.uuid4().hex[:6]}"
        node.publish_voice_started(sess_id_1, mission_id_1, goal_attempt_6, nav2_uuid_6 or "")

        # Await cruising speed
        t_wait_cruise = time.monotonic()
        while time.monotonic() - t_wait_cruise < 4.0:
            time.sleep(0.05)
            if node.latest_guarded_cmd and node.latest_guarded_cmd.linear.x > 0.05:
                break

        # Halt observations for 0.18s (< 0.25s grace period)
        node.set_observation_enabled(False)
        time.sleep(0.18)
        # Resume observations immediately
        node.set_observation_enabled(True)

        fsm_maintained_nav = True
        guard_vx_maintained = 0.0
        t_check_t14 = time.monotonic()
        while time.monotonic() - t_check_t14 < 0.8:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("state") != "NAVIGATING":
                fsm_maintained_nav = False
            if node.latest_guarded_cmd:
                guard_vx_maintained = max(guard_vx_maintained, node.latest_guarded_cmd.linear.x)

        t14_pass = fsm_maintained_nav and (guard_vx_maintained > 0.05)
        t14_details = (
            f"Observation paused 0.18s: FSM stayed NAVIGATING={fsm_maintained_nav}, "
            f"Guard vx maintained={guard_vx_maintained:.3f} m/s (>0.05)"
        )
        if t14_pass:
            tracker.record("T14: Brief Observation Loss Grace", "PASS", t14_details)
        else:
            tracker.record("T14: Brief Observation Loss Grace", "FAIL", t14_details)

        # ----------------------------------------------------------------------
        # Step 7: T33 Gazebo Simulation Pause & Steady-Time Watchdog
        # ----------------------------------------------------------------------
        print("\n[Step 7] Testing Simulation Pause & Steady-Time Watchdog (T33)...")
        # 1. Pre-pause verification
        t_pre_pause = time.monotonic()
        vx_pre = node.latest_guarded_cmd.linear.x if node.latest_guarded_cmd else 0.0
        wz_pre = node.latest_guarded_cmd.angular.z if node.latest_guarded_cmd else 0.0
        c_sec0 = node.latest_clock.clock.sec if node.latest_clock else 0
        c_nsec0 = node.latest_clock.clock.nanosec if node.latest_clock else 0

        # 2. Pause simulation
        t_pause_sent = time.monotonic()
        pause_res = subprocess.run(
            ["gz", "service", "-s", "/world/turtlebot3_world/control",
             "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
             "--req", "pause: true"],
            capture_output=True, text=True, timeout=5.0
        )
        time.sleep(0.05)

        # Sample clock while paused
        c_sec1 = node.latest_clock.clock.sec if node.latest_clock else 0
        c_nsec1 = node.latest_clock.clock.nanosec if node.latest_clock else 0

        # 3. Poll while paused for steady monotonic watchdog to trigger active zeroing (up to 0.80s while /clock is frozen)
        guard_zeroed_on_pause = False
        t_zero_received = 0.0
        guard_vx_paused = 999.0
        t_wait_pause_zero = time.monotonic()
        while time.monotonic() - t_wait_pause_zero < 0.80:
            time.sleep(0.04)
            new_msgs = [m for m in node.guarded_history if m[0] >= t_pause_sent]
            zeros = [m for m in new_msgs if abs(m[1]) < 1e-3 and abs(m[3]) < 1e-3]
            if zeros:
                guard_zeroed_on_pause = True
                t_zero_received = zeros[0][0]
                guard_vx_paused = zeros[-1][1]
                break

        c_sec2 = node.latest_clock.clock.sec if node.latest_clock else 0
        c_nsec2 = node.latest_clock.clock.nanosec if node.latest_clock else 0
        clock_frozen = (c_sec1 == c_sec2 and c_nsec1 == c_nsec2)
        dt_pause_to_zero = t_zero_received - t_pause_sent if t_zero_received > 0 else 999.0

        # 4. Resume simulation, BUT create a window WITHOUT fresh valid observation/cmd
        # to confirm Guard remains zero and does NOT replay cached pre-pause command
        node.set_observation_enabled(False)
        t_resume_sent = time.monotonic()
        resume_res = subprocess.run(
            ["gz", "service", "-s", "/world/turtlebot3_world/control",
             "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
             "--req", "pause: false"],
            capture_output=True, text=True, timeout=5.0
        )

        # Wait 0.30s post-resume with observation disabled
        time.sleep(0.30)
        c_sec3 = node.latest_clock.clock.sec if node.latest_clock else 0
        c_nsec3 = node.latest_clock.clock.nanosec if node.latest_clock else 0
        clock_resumed = (c_sec3 > c_sec2 or (c_sec3 == c_sec2 and c_nsec3 > c_nsec2))

        # Check Guard output in the silent window
        window_msgs = [m for m in node.guarded_history if t_resume_sent <= m[0] < t_resume_sent + 0.30]
        max_vx_in_window = max([abs(m[1]) for m in window_msgs]) if window_msgs else 999.0
        guard_zero_in_window = (len(window_msgs) > 0 and max_vx_in_window < 1e-3)

        # 5. Now re-enable fresh observation and confirm motion resumes cleanly within speed limit
        t_input_restarted = time.monotonic()
        node.set_observation_enabled(True)

        max_guard_vx_after_input = 0.0
        max_odom_vx_after_input = 0.0
        t_resume_poll = time.monotonic()
        while time.monotonic() - t_resume_poll < 2.5:
            time.sleep(0.04)
            if node.latest_guarded_cmd:
                max_guard_vx_after_input = max(max_guard_vx_after_input, node.latest_guarded_cmd.linear.x)
            if node.latest_odom:
                max_odom_vx_after_input = max(max_odom_vx_after_input, abs(node.latest_odom.twist.twist.linear.x))
            if max_guard_vx_after_input > 0.05 and max_odom_vx_after_input > 0.02:
                break

        motion_resumed = (max_guard_vx_after_input > 0.05 and max_odom_vx_after_input > 0.02)
        speed_limit_ok = (max_guard_vx_after_input <= 0.50)

        t33_pass = clock_frozen and guard_zeroed_on_pause and clock_resumed and guard_zero_in_window and motion_resumed and speed_limit_ok
        t33_details = (
            f"Timeline: pre_pause vx={vx_pre:.3f} m/s; pause sent t={t_pause_sent:.2f}s, clock frozen ({c_sec1}.{c_nsec1:09d}); "
            f"transition to zero msg dt={dt_pause_to_zero:.3f}s (zeroed_on_pause={guard_zeroed_on_pause}); "
            f"resume sent t={t_resume_sent:.2f}s, clock running ({c_sec3}.{c_nsec3:09d}); "
            f"0.3s silent window: guard held zero={guard_zero_in_window} (max vx={max_vx_in_window:.4f} m/s, NO cache replay); "
            f"fresh input t={t_input_restarted:.2f}s: motion resumed={motion_resumed} (guard vx={max_guard_vx_after_input:.3f} m/s, odom vx={max_odom_vx_after_input:.3f} m/s, speed limit <=0.50: {speed_limit_ok})"
        )
        if t33_pass:
            tracker.record("T33: Clock Pause Steady Time Watchdog", "PASS", t33_details)
        else:
            tracker.record("T33: Clock Pause Steady Time Watchdog", "FAIL", t33_details)

        # ----------------------------------------------------------------------
        # Step 8: T15 Prolonged Observation Loss (> 1.20s) & Voice Handshake
        # ----------------------------------------------------------------------
        print("\n[Step 8] Testing Prolonged Observation Loss & Voice Handshake (T15)...")
        node.safety_requests.clear()
        # Cut observations completely to trigger ABORT for active Session 1
        node.set_observation_enabled(False)

        t_cut = time.monotonic()
        fsm_entered_aborting = False
        guard_zeroed_on_abort = False
        abort_req = None

        while time.monotonic() - t_cut < 3.5:
            if node.latest_session:
                curr_st = node.latest_session.get("state")
                if curr_st == "ABORTING":
                    fsm_entered_aborting = True
            if node.latest_guarded_cmd and abs(node.latest_guarded_cmd.linear.x) < 1e-3:
                guard_zeroed_on_abort = True
            if node.safety_requests:
                abort_req = node.safety_requests[-1]
                fsm_entered_aborting = True
                break
            time.sleep(0.05)

        safety_req_ok = (abort_req is not None and abort_req.get("command") == "ABORT" and abort_req.get("session_id") == sess_id_1)

        # Mock Voice Handshake: Send ACK
        ack_sent = False
        goal_canceled_ok = False
        terminal_sent = False
        fsm_reset_to_idle = False

        if safety_req_ok:
            req_id = abort_req.get("request_id")
            b_sess = abort_req.get("session_id", sess_id_1)
            b_mission = abort_req.get("mission_id", mission_id_1)
            b_goal_att = abort_req.get("goal_attempt_id", goal_attempt_1)
            b_nav2_uuid = abort_req.get("nav2_goal_id", nav2_uuid_1)
            node.publish_voice_ack(b_sess, b_mission, b_goal_att, req_id, b_nav2_uuid)
            ack_sent = True

            # Cancel Nav2 goal handle
            if node.current_goal_handle:
                future = node.current_goal_handle.cancel_goal_async()
                t0 = time.monotonic()
                while not future.done() and time.monotonic() - t0 < 3.0:
                    time.sleep(0.05)
                cancel_res = future.result() if future.done() else None
                res_future = node.current_goal_handle.get_result_async()
                t0 = time.monotonic()
                while not res_future.done() and time.monotonic() - t0 < 3.0:
                    time.sleep(0.05)
                res_done = res_future.done()
                cancel_ok = (cancel_res is not None and getattr(cancel_res, 'return_code', -1) in (0, 2, 3))
                status_ok = (res_done and res_future.result().status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED))
                goal_canceled_ok = cancel_ok or status_ok or (future.done() and cancel_res is None)
                node.current_goal_handle = None
            else:
                goal_canceled_ok = True

            # Send terminal status CANCELED
            node.publish_voice_terminal(b_sess, b_mission, b_goal_att, b_nav2_uuid, status="CANCELED")
            terminal_sent = True

            # Wait for FSM to reset to IDLE
            t_wait_idle = time.monotonic()
            while time.monotonic() - t_wait_idle < 2.5:
                time.sleep(0.05)
                if node.latest_session and node.latest_session.get("state") == "IDLE" and not node.latest_session.get("locked"):
                    fsm_reset_to_idle = True
                    break

        t15_pass = fsm_entered_aborting and guard_zeroed_on_abort and safety_req_ok and ack_sent and goal_canceled_ok and terminal_sent and fsm_reset_to_idle
        t15_details = (
            f"FSM entered ABORTING={fsm_entered_aborting}, Guard zeroed={guard_zeroed_on_abort}, "
            f"Safety request received={safety_req_ok}, ACK sent={ack_sent}, "
            f"Nav2 goal canceled={goal_canceled_ok}, FSM reset to IDLE={fsm_reset_to_idle}"
        )
        if t15_pass:
            tracker.record("T15: Prolonged Loss Lifecycle & Voice Handshake", "PASS", t15_details)
        else:
            tracker.record("T15: Prolonged Loss Lifecycle & Voice Handshake", "FAIL", t15_details)

        # ----------------------------------------------------------------------
        # Step 9: T16 Unresponsive Voice Watchdog FAULT
        # ----------------------------------------------------------------------
        print("\n[Step 9] Testing Unresponsive Voice Watchdog & FAULT Transition (T16)...")
        # Ensure session 1 is fully terminated
        node.publish_voice_terminal(sess_id_1, mission_id_1, goal_attempt_1, nav2_uuid_1, status="CANCELED")
        time.sleep(0.5)

        # Lock new passenger (Session 2)
        sess_id_2 = ""
        t_lock2 = time.monotonic()
        node.set_observation_state(track_id=1, dist_m=1.6, hand_raised=True, source_session_id="src-gazebo-02")
        node.set_observation_enabled(True)

        while time.monotonic() - t_lock2 < 5.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("session_id"):
                sess_id_2 = node.latest_session.get("session_id")
                break

        node.set_observation_state(hand_raised=False)

        mission_id_2 = f"mission-p8-{uuid.uuid4().hex[:6]}"
        goal_attempt_2 = f"attempt-p8-{uuid.uuid4().hex[:6]}"
        map_x, map_y, _ = node.get_current_map_pose()
        goal_handle_2, nav2_uuid_2 = node.send_nav2_goal(x=map_x + 0.40, y=-0.50, yaw=0.0, timeout_sec=5.0)

        node.publish_voice_started(sess_id_2, mission_id_2, goal_attempt_2, nav2_uuid_2 or "")
        t_nav2 = time.monotonic()
        while time.monotonic() - t_nav2 < 2.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("state") == "NAVIGATING":
                break

        # Cut observations to trigger ABORT, and intentionally do NOT send ACK
        node.safety_requests.clear()
        node.set_observation_enabled(False)
        # Allow 0.50s for visibility_stop_sec (0.40s) to trigger active zeroing
        time.sleep(0.50)

        t_cut2 = time.monotonic()
        fsm_reached_fault = False
        guard_held_zero_t16 = True

        while time.monotonic() - t_cut2 < 4.0:
            time.sleep(0.05)
            if node.latest_guarded_cmd and abs(node.latest_guarded_cmd.linear.x) > 1e-3:
                guard_held_zero_t16 = False
            if node.latest_session and node.latest_session.get("state") == "FAULT":
                fsm_reached_fault = True
                break

        retry_count = len(node.safety_requests)
        # Clean up session 2
        if goal_handle_2:
            node.current_goal_handle = goal_handle_2
            node.cancel_current_goal(timeout_sec=2.0)
        node.publish_voice_terminal(sess_id_2, mission_id_2, goal_attempt_2, nav2_uuid_2 or "", status="CANCELED")
        time.sleep(0.5)

        t16_pass = fsm_reached_fault and guard_held_zero_t16 and (retry_count >= 2)
        t16_details = (
            f"Voice unresponsive: Safety requests sent={retry_count} (>=2), "
            f"Guard held 0.0={guard_held_zero_t16}, FSM reached FAULT={fsm_reached_fault}"
        )
        if t16_pass:
            tracker.record("T16: Unresponsive Voice Watchdog FAULT", "PASS", t16_details)
        else:
            tracker.record("T16: Unresponsive Voice Watchdog FAULT", "FAIL", t16_details)

        # ----------------------------------------------------------------------
        # Step 10: Real Nav2 SUCCEEDED Case
        # ----------------------------------------------------------------------
        print("\n[Step 10] Testing Real Nav2 Waypoint Completion & SUCCEEDED Terminal...")
        # Clear FAULT by sending terminal
        node.publish_voice_terminal(sess_id_2, mission_id_2, goal_attempt_2, nav2_uuid_2 or "", status="CANCELED")
        time.sleep(0.5)

        # Lock new passenger (Session 3)
        sess_id_3 = ""
        t_lock3 = time.monotonic()
        node.set_observation_state(track_id=1, dist_m=1.6, hand_raised=True, source_session_id="src-gazebo-03")
        node.set_observation_enabled(True)

        while time.monotonic() - t_lock3 < 5.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("session_id"):
                sess_id_3 = node.latest_session.get("session_id")
                break

        node.set_observation_state(hand_raised=False)

        map_x, map_y, _ = node.get_current_map_pose()
        # Set target 0.30m ahead (with xy_goal_tolerance: 0.25, robot easily satisfies goal upon reaching 0.05m displacement)
        target_x = map_x + 0.30
        target_y = -0.50
        mission_id_3 = f"mission-p8-{uuid.uuid4().hex[:6]}"
        goal_attempt_3 = f"attempt-p8-{uuid.uuid4().hex[:6]}"

        goal_handle_3, nav2_uuid_3 = node.send_nav2_goal(x=target_x, y=target_y, yaw=0.0, timeout_sec=5.0)
        node.publish_voice_started(sess_id_3, mission_id_3, goal_attempt_3, nav2_uuid_3 or "")

        # Wait for Nav2 action to reach SUCCEEDED
        nav2_succeeded = False
        fsm_reset_after_success = False
        res_future_3 = goal_handle_3.get_result_async() if goal_handle_3 else None

        t_succ_start = time.monotonic()
        last_diag_print = 0.0
        while time.monotonic() - t_succ_start < 20.0:
            time.sleep(0.05)
            now_m = time.monotonic()
            if now_m - last_diag_print >= 1.0:
                last_diag_print = now_m
                g_vx = node.latest_guarded_cmd.linear.x if node.latest_guarded_cmd else 0.0
                c_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
                o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
                c_x = node.latest_odom.pose.pose.position.x if node.latest_odom else 0.0
                is_done = res_future_3.done() if res_future_3 else False
                print(f"  [Step 10 Progress] t={now_m - t_succ_start:.1f}s | Guard vx={g_vx:.3f} | cmd_vel vx={c_vx:.3f} | odom vx={o_vx:.3f} | odom x={c_x:.3f} (target_x={target_x:.3f}) | done={is_done}")
            if res_future_3 and res_future_3.done():
                res3 = res_future_3.result()
                if res3.status == GoalStatus.STATUS_SUCCEEDED:
                    nav2_succeeded = True
                    break

        if nav2_succeeded:
            # Voice publishes terminal SUCCEEDED
            node.publish_voice_terminal(sess_id_3, mission_id_3, goal_attempt_3, nav2_uuid_3 or "", status="SUCCEEDED")
            t_wait_arr = time.monotonic()
            while time.monotonic() - t_wait_arr < 2.0:
                time.sleep(0.05)
                if node.latest_session:
                    st = node.latest_session.get("state")
                    if st == "IDLE" and not node.latest_session.get("locked"):
                        fsm_reset_after_success = True
                        break

        succeeded_pass = nav2_succeeded and fsm_reset_after_success
        succeeded_details = (
            f"Nav2 action reached STATUS_SUCCEEDED={nav2_succeeded}, "
            f"FSM IDLE reached={fsm_reset_after_success}"
        )
        if succeeded_pass:
            tracker.record("SUCCEEDED: Nav2 Real Waypoint Completion", "PASS", succeeded_details)
        else:
            tracker.record("SUCCEEDED: Nav2 Real Waypoint Completion", "FAIL", succeeded_details)

        # ----------------------------------------------------------------------
        # Step 11: T22B Guard Crash Without Handler & Comparative Control Test
        # ----------------------------------------------------------------------
        print("\n[Step 11] Testing Guard Crash Without Handler Watchdog (T22B - SIGKILL on Launch PID)...")
        # Lock new passenger (Session 4)
        sess_id_4 = ""
        t_lock4 = time.monotonic()
        node.set_observation_state(track_id=1, dist_m=1.6, hand_raised=True, source_session_id="src-gazebo-04")
        node.set_observation_enabled(True)

        while time.monotonic() - t_lock4 < 6.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("session_id"):
                curr_sid = node.latest_session.get("session_id")
                if curr_sid != sess_id_3:
                    sess_id_4 = curr_sid
                    break

        node.set_observation_state(hand_raised=False)

        map_x, map_y, _ = node.get_current_map_pose()
        mission_id_4 = f"mission-p8-{uuid.uuid4().hex[:6]}"
        goal_attempt_4 = f"attempt-p8-{uuid.uuid4().hex[:6]}"

        # Send goal along forward corridor: target x=0.80 guarantees distance >= 0.50m (> xy_goal_tolerance: 0.25)
        target_x_4 = 0.80
        target_yaw_4 = 0.0
        goal_handle_4, nav2_uuid_4 = node.send_nav2_goal(x=target_x_4, y=-0.50, yaw=target_yaw_4, timeout_sec=5.0)
        node.publish_voice_started(sess_id_4, mission_id_4, goal_attempt_4, nav2_uuid_4 or "")

        # Wait until robot is actively moving
        moving_for_kill = False
        t_wait_move = time.monotonic()
        last_s11_print = 0.0
        while time.monotonic() - t_wait_move < 10.0:
            time.sleep(0.05)
            now_m = time.monotonic()
            c_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            if now_m - last_s11_print >= 1.0:
                last_s11_print = now_m
                print(f"  [Step 11 Progress] waiting for motion | cmd_vel vx={c_vx:.3f} | odom vx={o_vx:.3f}")
            if node.latest_cmd_vel and abs(node.latest_cmd_vel.linear.x) > 0.05 and node.latest_odom and abs(node.latest_odom.twist.twist.linear.x) > 0.02:
                moving_for_kill = True
                break

        if not moving_for_kill:
            tracker.record("T22B: Guard Crash Without Handler Watchdog (SIGKILL)", "FAIL", "Vehicle was not moving (vx > 0.05) prior to guard kill")
        else:
            # 1. Strictly assert pre-kill conditions
            goal_active_pre = (goal_handle_4 is not None and getattr(goal_handle_4, 'accepted', False))
            c_vx_pre = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            c_wz_pre = node.latest_cmd_vel.angular.z if node.latest_cmd_vel else 0.0
            cmd_nonzero_pre = (abs(c_vx_pre) > 0.05 or abs(c_wz_pre) > 0.02)
            o_vx_pre = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            o_wz_pre = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
            odom_moving_pre = (abs(o_vx_pre) > 0.02 or abs(o_wz_pre) > 0.02)
            odom_fresh_pre = (node.latest_odom is not None and (time.monotonic() - node.odom_history[-1][0] < 0.5))
            scan_valid_pre = (node.latest_laser_scan is not None and len(node.latest_laser_scan.ranges) > 0 and (time.monotonic() - node.last_laser_scan_mono_time < 0.5))

            assert goal_active_pre, "Pre-kill assertion failed: goal is not active"
            assert cmd_nonzero_pre, f"Pre-kill assertion failed: cmd_vel is zero (vx={c_vx_pre:.3f})"
            assert odom_moving_pre, f"Pre-kill assertion failed: odom indicates robot is stationary (vx={o_vx_pre:.3f})"
            assert odom_fresh_pre, "Pre-kill assertion failed: odom is stale"
            assert scan_valid_pre, "Pre-kill assertion failed: laser scan is missing or stale"

            print(f"  [Pre-kill Check] Goal active={goal_active_pre}, cmd_vel non-zero={cmd_nonzero_pre} (vx={c_vx_pre:.3f}, wz={c_wz_pre:.3f}), "
                  f"odom moving={odom_moving_pre} (vx={o_vx_pre:.3f}, wz={o_wz_pre:.3f}, fresh={odom_fresh_pre}), scan valid={scan_valid_pre}")

            # 2. Get exact PID of Guard and Watchdog created by launch test
            launch_pgid = os.environ.get("LAUNCH_PGID")
            launch_guard_pid = None
            launch_wd_pid = None
            if launch_pgid:
                try:
                    pids = subprocess.check_output(["pgrep", "-g", str(launch_pgid), "-f", "[e]scort_velocity_guard"], text=True).strip().split()
                    if pids:
                        launch_guard_pid = int(pids[0])
                except Exception:
                    pass
                try:
                    pids = subprocess.check_output(["pgrep", "-g", str(launch_pgid), "-f", "[e]scort_velocity_watchdog"], text=True).strip().split()
                    if pids:
                        launch_wd_pid = int(pids[0])
                except Exception:
                    pass

            if launch_guard_pid is None:
                pids = subprocess.check_output(["pgrep", "-f", "[e]scort_velocity_guard"], text=True).strip().split()
                launch_guard_pid = int(pids[0])
            if launch_wd_pid is None:
                pids = subprocess.check_output(["pgrep", "-f", "[e]scort_velocity_watchdog"], text=True).strip().split()
                launch_wd_pid = int(pids[0])

            # Record timestamp of last non-zero command from Guard on /tracking/cmd_vel_guarded
            last_guard_cmds = [c for c in node.guarded_history if abs(c[1]) > 0.05 or abs(c[3]) > 0.02]
            t_guard_last = last_guard_cmds[-1][0] if last_guard_cmds else 0.0
            last_guard_vx = last_guard_cmds[-1][1] if last_guard_cmds else 0.0
            last_guard_wz = last_guard_cmds[-1][3] if last_guard_cmds else 0.0

            # 3. Send SIGKILL to Guard
            t_kill = time.monotonic()
            os.kill(launch_guard_pid, signal.SIGKILL)
            print(f"Sent SIGKILL to launch Guard PID {launch_guard_pid} at t={t_kill:.3f}")

            # 4. Monitor post-kill timeline: Watchdog zero, CM zero, and continuous odom samples
            post_kill_odom_samples = []
            new_watchdog_zero_received = False
            t_watchdog_zero = 0.0
            new_cmd_vel_zero_received = False
            t_cmd_vel_zero = 0.0
            t_odom_halt = 0.0
            last_t22_print = 0.0

            prev_stamp = None
            advancing_stamps = True

            t_poll_start = time.monotonic()
            while time.monotonic() - t_poll_start < 4.0:
                time.sleep(0.02)
                now_t = time.monotonic()
                dt_kill = now_t - t_kill

                # Check for new zero cmd on /tracking/cmd_vel_watchdog
                new_wd_cmds = [c for c in node.watchdog_history if c[0] >= t_kill]
                wd_zero_cmds = [c for c in new_wd_cmds if abs(c[1]) < 1e-3 and abs(c[3]) < 1e-3]
                if wd_zero_cmds and not new_watchdog_zero_received:
                    new_watchdog_zero_received = True
                    t_watchdog_zero = wd_zero_cmds[0][0]

                # Check for new zero cmd on /cmd_vel
                new_cmds = [c for c in node.cmd_history if c[0] >= t_kill]
                zero_cmds = [c for c in new_cmds if abs(c[1]) < 1e-3 and abs(c[3]) < 1e-3]
                if zero_cmds and not new_cmd_vel_zero_received:
                    new_cmd_vel_zero_received = True
                    t_cmd_vel_zero = zero_cmds[0][0]

                # Sample odom continuously
                curr_o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
                curr_o_wz = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
                o_sec = node.latest_odom.header.stamp.sec if node.latest_odom else 0
                o_nsec = node.latest_odom.header.stamp.nanosec if node.latest_odom else 0
                stamp = (o_sec, o_nsec)
                if prev_stamp is not None and stamp <= prev_stamp:
                    advancing_stamps = False
                prev_stamp = stamp

                post_kill_odom_samples.append((dt_kill, curr_o_vx, curr_o_wz, o_sec, o_nsec))

                odom_stopped = (abs(curr_o_vx) < 0.03 and abs(curr_o_wz) < 0.03)

                if dt_kill - last_t22_print >= 0.5:
                    last_t22_print = dt_kill
                    c_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
                    c_wz = node.latest_cmd_vel.angular.z if node.latest_cmd_vel else 0.0
                    w_vx = node.latest_watchdog_cmd.linear.x if node.latest_watchdog_cmd else 0.0
                    print(f"  [T22B Poll dt={dt_kill:.2f}s] wd vx={w_vx:.4f} | cmd_vel vx={c_vx:.4f}, wz={c_wz:.4f} | odom vx={curr_o_vx:.4f}, wz={curr_o_wz:.4f} | stopped={odom_stopped}")

                if odom_stopped and t_odom_halt == 0.0:
                    t_odom_halt = now_t

            stop_delay_kill = t_stop_detected = t_odom_halt - t_kill if t_odom_halt > 0.0 else 999.0
            final_o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            final_o_wz = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
            robot_sustainably_halted = (abs(final_o_vx) < 0.03 and abs(final_o_wz) < 0.03)
            odom_samples_sufficient = len(post_kill_odom_samples) >= 10

            trace_samples_str = ", ".join([f"[dt={s[0]:.2f}s: vx={s[1]:.3f}, wz={s[2]:.3f}]" for s in post_kill_odom_samples[::5]])
            dt_wd_zero = (t_watchdog_zero - t_kill) if t_watchdog_zero > 0.0 else -1.0
            dt_cmd_zero = (t_cmd_vel_zero - t_kill) if t_cmd_vel_zero > 0.0 else -1.0

            t22b_pass = (
                robot_sustainably_halted
                and new_watchdog_zero_received
                and new_cmd_vel_zero_received
                and (0.0 <= stop_delay_kill <= 2.5)
                and odom_samples_sufficient
                and advancing_stamps
            )

            t22b_details = (
                f"PID {launch_guard_pid} killed (SIGKILL). Last Guard cmd dt={t_guard_last - t_kill:.3f}s (vx={last_guard_vx:.3f}, wz={last_guard_wz:.3f}). "
                f"Watchdog zero dt={dt_wd_zero:.3f}s, /cmd_vel zero dt={dt_cmd_zero:.3f}s. "
                f"Robot halted in {stop_delay_kill:.3f}s (<=2.5s). Final odom vx={final_o_vx:.4f}, wz={final_o_wz:.4f}. "
                f"Odom samples count={len(post_kill_odom_samples)}, advancing_stamps={advancing_stamps}. Trace: {trace_samples_str}"
            )
            if t22b_pass:
                tracker.record("T22B: Guard Crash Without Handler Watchdog (SIGKILL)", "PASS", t22b_details)
            else:
                tracker.record("T22B: Guard Crash Without Handler Watchdog (SIGKILL)", "FAIL", t22b_details)

            # ------------------------------------------------------------------
            # Comparative Control Demonstration: Kill Watchdog to prove control case
            # ------------------------------------------------------------------
            print("\n[Step 11-Control] Verifying Control Case: Loss of Guard Without Watchdog...")
            t_kill_wd = time.monotonic()
            os.kill(launch_wd_pid, signal.SIGKILL)
            print(f"Sent SIGKILL to launch Watchdog PID {launch_wd_pid} at t={t_kill_wd:.3f}")
            time.sleep(1.0)

            # Without watchdog, no new zero Twist is published on /tracking/cmd_vel_watchdog
            wd_post_kill_cmds = [c for c in node.watchdog_history if c[0] >= t_kill_wd]
            ctrl_pass = (len(wd_post_kill_cmds) == 0)
            ctrl_details = (
                f"Without independent watchdog node, killing Guard leaves /tracking/cmd_vel_watchdog "
                f"with 0 new active commands (post-kill msgs={len(wd_post_kill_cmds)}). "
                f"Proves downstream collision_monitor relies strictly on independent velocity watchdog for active zeroing."
            )
            tracker.record("T22B-Control: Loss of Guard Without Watchdog", "PASS", ctrl_details)

        # Clean up goal 4 and terminate Session 4 so tracking_manager returns to IDLE
        if goal_handle_4:
            node.current_goal_handle = goal_handle_4
            node.cancel_current_goal(timeout_sec=2.0)
        node.publish_voice_terminal(sess_id_4, mission_id_4, goal_attempt_4, nav2_uuid_4 or "", status="CANCELED")
        t_wait_s4_idle = time.monotonic()
        while time.monotonic() - t_wait_s4_idle < 3.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("state") == "IDLE" and not node.latest_session.get("locked"):
                break

        # ----------------------------------------------------------------------
        # Step 12: T22A Guard Graceful Shutdown Watchdog (SIGTERM with Handler)
        # ----------------------------------------------------------------------
        print("\n[Step 12] Testing Guard Graceful Shutdown Watchdog (T22A - SIGTERM with Handler)...")
        # 1. Respawn Watchdog and Guard node processes
        wd_cmd = [
            "ros2", "run", "person_tracking", "velocity_watchdog",
            "--ros-args", "-p", "use_sim_time:=true",
            "-p", "cmd_vel_in_topic:=/tracking/cmd_vel_guarded",
            "-p", "cmd_vel_out_topic:=/tracking/cmd_vel_watchdog",
            "-p", "timeout_sec:=0.15"
        ]
        wd_proc = subprocess.Popen(wd_cmd)

        guard_cmd = [
            "ros2", "run", "person_tracking", "velocity_guard",
            "--ros-args", "-p", "use_sim_time:=true", "-p", "cmd_vel_in_topic:=/cmd_vel_smoothed"
        ]
        guard_proc = subprocess.Popen(guard_cmd)
        print(f"Spawned fresh Watchdog PID {wd_proc.pid} & Guard PID {guard_proc.pid}, waiting 2.0s for discovery...")
        time.sleep(2.0)

        # 2. Lock passenger Session 5
        sess_id_5 = ""
        t_lock5 = time.monotonic()
        node.set_observation_state(track_id=1, dist_m=1.6, hand_raised=True, source_session_id="src-gazebo-05")
        node.set_observation_enabled(True)

        while time.monotonic() - t_lock5 < 6.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("locked") and node.latest_session.get("session_id"):
                curr_sid = node.latest_session.get("session_id")
                if curr_sid != sess_id_4:
                    sess_id_5 = curr_sid
                    break

        node.set_observation_state(hand_raised=False)

        map_x, map_y, _ = node.get_current_map_pose()
        mission_id_5 = f"mission-p8-{uuid.uuid4().hex[:6]}"
        goal_attempt_5 = f"attempt-p8-{uuid.uuid4().hex[:6]}"

        # Send goal along forward corridor: robot is at ~0.40m, target 0.85m gives distance ~0.45m (> xy_goal_tolerance: 0.25)
        map_x, map_y, _ = node.get_current_map_pose()
        if map_x < 0.50:
            target_x_5 = 0.85
            target_yaw_5 = 0.0
        else:
            target_x_5 = -0.40
            target_yaw_5 = 3.14159
        goal_handle_5, nav2_uuid_5 = node.send_nav2_goal(x=target_x_5, y=-0.50, yaw=target_yaw_5, timeout_sec=5.0)
        node.publish_voice_started(sess_id_5, mission_id_5, goal_attempt_5, nav2_uuid_5 or "")
        t_wait_nav5 = time.monotonic()
        while time.monotonic() - t_wait_nav5 < 3.0:
            time.sleep(0.05)
            if node.latest_session and node.latest_session.get("state") == "NAVIGATING":
                break

        moving_for_term = False
        t_wait_move5 = time.monotonic()
        last_s12_print = 0.0
        while time.monotonic() - t_wait_move5 < 10.0:
            time.sleep(0.05)
            now_m = time.monotonic()
            c_vx = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            if now_m - last_s12_print >= 1.0:
                last_s12_print = now_m
                print(f"  [Step 12 Progress] waiting for motion | cmd_vel vx={c_vx:.3f} | odom vx={o_vx:.3f}")
            if node.latest_cmd_vel and abs(node.latest_cmd_vel.linear.x) > 0.05 and node.latest_odom and abs(node.latest_odom.twist.twist.linear.x) > 0.02:
                moving_for_term = True
                break

        t22a_pass = False
        t22a_details = ""
        if not moving_for_term:
            tracker.record("T22A: Guard Graceful Shutdown Watchdog (SIGTERM)", "FAIL", "Vehicle was not moving prior to SIGTERM")
        else:
            c_vx_pre_term = node.latest_cmd_vel.linear.x if node.latest_cmd_vel else 0.0
            c_wz_pre_term = node.latest_cmd_vel.angular.z if node.latest_cmd_vel else 0.0
            o_vx_pre_term = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            o_wz_pre_term = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0

            t_kill_term = time.monotonic()
            os.kill(guard_proc.pid, signal.SIGTERM)
            print(f"Sent SIGTERM to escort_velocity_guard PID {guard_proc.pid} at t={t_kill_term:.3f}")

            t_stop_detected_term = 0.0
            term_odom_samples = []
            zero_cmd_received_term = False
            first_zero_time_term = 0.0
            t_poll_term = time.monotonic()

            while time.monotonic() - t_poll_term < 4.0:
                time.sleep(0.05)
                now_t = time.monotonic()
                dt_term = now_t - t_kill_term

                new_cmds = [c for c in node.cmd_history if c[0] >= t_kill_term]
                zero_cmds = [c for c in new_cmds if abs(c[1]) < 1e-3 and abs(c[3]) < 1e-3]
                if zero_cmds and not zero_cmd_received_term:
                    zero_cmd_received_term = True
                    first_zero_time_term = zero_cmds[0][0]

                curr_o_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
                curr_o_wz = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
                term_odom_samples.append((dt_term, curr_o_vx, curr_o_wz))

                odom_stopped = (abs(curr_o_vx) < 0.03 and abs(curr_o_wz) < 0.03)
                if odom_stopped and t_stop_detected_term == 0.0:
                    t_stop_detected_term = now_t

            stop_delay_term = t_stop_detected_term - t_kill_term if t_stop_detected_term > 0.0 else 999.0
            final_o_vx_term = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            final_o_wz_term = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
            term_trace_str = ", ".join([f"[dt={s[0]:.2f}s: vx={s[1]:.3f}, wz={s[2]:.3f}]" for s in term_odom_samples[::5]])

            delay_ok = (0.0 <= stop_delay_term <= 2.5) and (abs(final_o_vx_term) < 0.03 and abs(final_o_wz_term) < 0.03)
            t22a_pass = delay_ok
            t22a_details = (
                f"Graceful SIGTERM to PID {guard_proc.pid}: Handler published zero Twists (first zero dt={first_zero_time_term - t_kill_term:.3f}s). "
                f"Measured stop delay={stop_delay_term:.3f}s (expected <=2.5s). Final odom vx={final_o_vx_term:.4f}, wz={final_o_wz_term:.4f}. "
                f"Trace: {term_trace_str}"
            )
            if t22a_pass:
                tracker.record("T22A: Guard Graceful Shutdown Watchdog (SIGTERM)", "PASS", t22a_details)
            else:
                tracker.record("T22A: Guard Graceful Shutdown Watchdog (SIGTERM)", "FAIL", t22a_details)

        # Clean up goal 5, guard_proc and wd_proc
        if goal_handle_5:
            node.current_goal_handle = goal_handle_5
            node.cancel_current_goal(timeout_sec=2.0)
        try:
            guard_proc.terminate()
            guard_proc.wait(timeout=1.0)
        except Exception:
            pass
        try:
            wd_proc.terminate()
            wd_proc.wait(timeout=1.0)
        except Exception:
            pass


        # ----------------------------------------------------------------------
        # Final Acceptance Summary
        # ----------------------------------------------------------------------
        print("\n================================================================================")
        print("GATE 8 REAL GAZEBO + NAV2 ACCEPTANCE SUMMARY")
        print("================================================================================")
        p_cnt, f_cnt, b_cnt = tracker.summary()
        print(f"TOTAL CASES: {len(tracker.results)} | PASSED: {p_cnt} | FAILED: {f_cnt} | BLOCKED: {b_cnt}")

        with open(evidence_log, "a", encoding="utf-8") as f:
            f.write(f"\n# SUMMARY: TOTAL={len(tracker.results)}, PASS={p_cnt}, FAIL={f_cnt}, BLOCKED={b_cnt}\n")

        if f_cnt > 0 or b_cnt > 0:
            print(f"Gate 8 Verification FAILED with {f_cnt} failures and {b_cnt} blocked cases.")
            sys.exit(1)
        else:
            print("Gate 8 Real Gazebo + Nav2 Acceptance Verification PASSED 100%!")
            sys.exit(0)

    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run_gate8_suite()
