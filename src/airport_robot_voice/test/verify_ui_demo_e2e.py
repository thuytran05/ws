#!/usr/bin/env python3
"""Hardware-free Gazebo E2E acceptance test for thin UI client contract (ROS_DOMAIN_ID=44)."""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class FakeWorldObserver(Node):
    def __init__(self):
        super().__init__(
            'e2e_world_observer',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ],
        )
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_obs = self.create_publisher(String, '/tracking/observation', qos_rel)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.clock_sec = 0.0
        self.latest_odom = None
        self.latest_session = None
        self.latest_guarded = None
        self.nav_statuses = []
        self.safety_requests = []
        self.feedbacks = []
        self.latest_ui_status = None
        self.latest_catalog = None

        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(Twist, '/tracking/cmd_vel_guarded', self._on_guarded, 20)
        self.create_subscription(String, '/tracking/session_state', self._on_session, qos_tl)
        self.create_subscription(String, '/tracking/safety_request', self._on_safety, qos_rel)
        self.create_subscription(String, '/voice/nav_status', self._on_nav_status, qos_tl)
        self.create_subscription(String, '/voice/feedback', self._on_feedback, qos_rel)
        self.create_subscription(String, '/voice/ui_status', self._on_ui_status, qos_tl)
        self.create_subscription(String, '/voice/places_catalog', self._on_catalog, qos_tl)

        self.obs_active = False
        self.hand_raised = False
        self.dist_m = 1.6
        self._last_stamp = (0, 0)
        self.create_timer(0.05, self._obs_tick)

    def _on_clock(self, msg: Clock):
        self.clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _on_guarded(self, msg: Twist):
        self.latest_guarded = msg

    def _on_session(self, msg: String):
        self.latest_session = json.loads(msg.data)

    def _on_safety(self, msg: String):
        self.safety_requests.append(json.loads(msg.data))

    def _on_nav_status(self, msg: String):
        self.nav_statuses.append(json.loads(msg.data))

    def _on_feedback(self, msg: String):
        self.feedbacks.append(json.loads(msg.data))

    def _on_ui_status(self, msg: String):
        self.latest_ui_status = json.loads(msg.data)

    def _on_catalog(self, msg: String):
        self.latest_catalog = json.loads(msg.data)

    def odom_speed(self) -> float:
        if not self.latest_odom:
            return 0.0
        t = self.latest_odom.twist.twist
        return math.hypot(t.linear.x, t.linear.y)

    def publish_initial_pose(self, x=-1.0, y=-0.5, yaw=0.0):
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.068
        self.pub_pose.publish(m)

    def _obs_tick(self):
        if not self.obs_active:
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

        cand = {
            'track_id': 1,
            'is_confirmed': True,
            'is_ambiguous': False,
            'confidence': 0.95,
            'bbox_xyxy': [220.0, 100.0, 420.0, 400.0],
            'torso_depth_m': float(self.dist_m),
            'depth_valid': True,
            'depth_proven': True,
            'center_offset_x_norm': 0.0,
            'near_edge': False,
            'keypoints': [[320.0, 200.0, 0.9]] * 17,
        }
        cand['keypoints'][5] = [300.0, 180.0, 0.9]
        cand['keypoints'][6] = [340.0, 180.0, 0.9]
        cand['keypoints'][9] = [300.0, 120.0 if self.hand_raised else 240.0, 0.95]
        cand['keypoints'][10] = [340.0, 240.0, 0.9]
        self.pub_obs.publish(String(data=json.dumps({
            'schema_version': '1.0',
            'stamp': {'sec': sec, 'nanosec': nanosec},
            'frame_id': 'camera_color_optical_frame',
            'source_session_id': 'e2e-fake-obs',
            'data_quality': 'VALID_RGBD',
            'is_metric_valid': True,
            'candidates': [cand],
            'persons': [cand],
        })))


class ThinUiClient(Node):
    def __init__(self, name='thin_ui_client'):
        super().__init__(name)
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_req = self.create_publisher(String, '/voice/ui_request', qos_rel)
        self.ui_status = None
        self.catalog = None
        self.feedbacks = []
        self.create_subscription(String, '/voice/ui_status', lambda m: setattr(self, 'ui_status', json.loads(m.data)), qos_tl)
        self.create_subscription(String, '/voice/places_catalog', lambda m: setattr(self, 'catalog', json.loads(m.data)), qos_tl)
        self.create_subscription(String, '/voice/feedback', lambda m: self.feedbacks.append(json.loads(m.data)), qos_rel)

    def send(self, payload: dict):
        self.pub_req.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def wait_until(pred, timeout=20.0, step=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(step)
    return False


def lock_passenger(obs: FakeWorldObserver):
    obs.obs_active = True
    obs.dist_m = 1.6
    obs.hand_raised = True
    ok = wait_until(
        lambda: obs.latest_session
        and obs.latest_session.get('locked') is True
        and obs.latest_session.get('state') == 'LOCKED_WAIT_DESTINATION',
        timeout=10.0,
    )
    obs.hand_raised = False
    assert ok, f'Failed to lock passenger: {obs.latest_session}'


def main():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '44'

    launch_proc = subprocess.Popen(
        ['ros2', 'launch', 'airport_robot_voice', 'ui_demo_gazebo.launch.py'],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    pgid = os.getpgid(launch_proc.pid)

    rclpy.init()
    obs = FakeWorldObserver()
    ui = ThinUiClient('e2e_ui_client_1')
    executor = SingleThreadedExecutor()
    executor.add_node(obs)
    executor.add_node(ui)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print('[E2E 0] Waiting for Gazebo, Nav2, Voice catalog & session_state...', flush=True)
        assert wait_until(
            lambda: obs.clock_sec > 1.0 and obs.latest_odom is not None and ui.catalog is not None and obs.latest_session is not None,
            timeout=45.0,
        ), 'Timeout waiting for simulation + Voice + Tracking startup'
        for _ in range(3):
            obs.publish_initial_pose(-1.0, -0.5, 0.0)
            time.sleep(0.5)
        time.sleep(3.0)

        # 1. Read-only catalog
        assert ui.catalog['read_only'] is True
        assert any(p['id'] == 'gate_15' and p['enabled'] for p in ui.catalog['places'])
        print('[E2E 1] PASS: Read-only catalog received with gate_15', flush=True)

        # 2. Unlocked -> rejected with client_request_id
        ui.send({'client_request_id': 'e2e-unlocked-1', 'type': 'navigate_id', 'destination_id': 'gate_15'})
        assert wait_until(
            lambda: any(f.get('client_request_id') == 'e2e-unlocked-1' and f.get('event') == 'REJECTED' for f in ui.feedbacks),
            timeout=8.0,
        ), f'Unlocked request not rejected: {ui.feedbacks}'
        assert obs.odom_speed() < 0.02
        print('[E2E 2] PASS: Unlocked navigate_id rejected with client_request_id', flush=True)

        # 3. Lock passenger -> select gate_15 -> moves + retry same client_request_id -> no 2nd goal
        lock_passenger(obs)
        prev_nav_count = len(obs.nav_statuses)
        ui.send({'client_request_id': 'e2e-nav-1', 'type': 'navigate_id', 'destination_id': 'gate_15'})
        assert wait_until(
            lambda: any(s.get('nav_event') == 'NAVIGATION_STARTED' for s in obs.nav_statuses[prev_nav_count:]),
            timeout=15.0,
        ), f'Goal did not start: {obs.nav_statuses}'
        started = [s for s in obs.nav_statuses[prev_nav_count:] if s.get('nav_event') == 'NAVIGATION_STARTED'][0]
        assert wait_until(lambda: obs.odom_speed() > 0.04, timeout=15.0), 'Robot did not move in Gazebo'

        # Retry same client_request_id
        ui.send({'client_request_id': 'e2e-nav-1', 'type': 'navigate_id', 'destination_id': 'gate_15'})
        assert wait_until(
            lambda: any(f.get('client_request_id') == 'e2e-nav-1' and f.get('duplicate') is True for f in ui.feedbacks),
            timeout=5.0,
        ), 'Duplicate retry did not return cached response'
        started_count = sum(1 for s in obs.nav_statuses[prev_nav_count:] if s.get('nav_event') == 'NAVIGATION_STARTED')
        assert started_count == 1, f'Expected 1 goal start, got {started_count}'
        print('[E2E 3] PASS: Locked navigate_id moved robot & retry did not send 2nd goal', flush=True)

        # 4. Passenger lost -> Guard zeros & Voice cancels exact goal & /odom stops
        obs.obs_active = False
        assert wait_until(
            lambda: any(
                s.get('is_terminal') is True
                and s.get('status') == 'CANCELED'
                and s.get('goal_attempt_id') == started['goal_attempt_id']
                and s.get('nav2_uuid') == started['nav2_uuid']
                for s in obs.nav_statuses[prev_nav_count:]
            ),
            timeout=15.0,
        ), f'Exact goal cancel not observed: {obs.nav_statuses[prev_nav_count:]}'
        assert wait_until(
            lambda: obs.latest_guarded is not None
            and abs(obs.latest_guarded.linear.x) < 1e-4
            and obs.odom_speed() < 0.02,
            timeout=10.0,
        ), f'Robot failed to stop: guarded={obs.latest_guarded}, speed={obs.odom_speed()}'
        assert wait_until(
            lambda: ui.ui_status and ui.ui_status.get('mission_state') == 'IDLE'
            and (ui.ui_status.get('last_terminal_mission') or {}).get('client_request_id') == 'e2e-nav-1',
            timeout=5.0,
        ), f'last_terminal_mission missing: {ui.ui_status}'
        print('[E2E 4] PASS: Passenger lost -> Guard zeroed, exact goal CANCELED, /odom stopped', flush=True)

        # 5. Real Vietnamese NLU ("Dẫn tôi tới cổng 15") + client exit safety
        lock_passenger(obs)
        prev_nav_count = len(obs.nav_statuses)
        ui.send({'client_request_id': 'e2e-nlu-1', 'type': 'text_nlu', 'text': 'Dẫn tôi tới cổng 15'})
        assert wait_until(
            lambda: any(s.get('nav_event') == 'NAVIGATION_STARTED' for s in obs.nav_statuses[prev_nav_count:]),
            timeout=15.0,
        ), f'NLU goal did not start: {obs.nav_statuses[prev_nav_count:]}'
        nlu_started = [s for s in obs.nav_statuses[prev_nav_count:] if s.get('nav_event') == 'NAVIGATION_STARTED'][0]
        assert wait_until(lambda: obs.odom_speed() > 0.04, timeout=15.0), 'Robot did not move after NLU command'

        # Disconnect/exit UI client completely!
        executor.remove_node(ui)
        ui.destroy_node()
        ui = None
        print('[E2E 5a] UI client exited while robot is navigating...', flush=True)

        obs.obs_active = False
        assert wait_until(
            lambda: any(
                s.get('is_terminal') is True
                and s.get('status') == 'CANCELED'
                and s.get('goal_attempt_id') == nlu_started['goal_attempt_id']
                for s in obs.nav_statuses[prev_nav_count:]
            ) and obs.odom_speed() < 0.02,
            timeout=15.0,
        ), 'Safety abort failed after UI client exited'
        print('[E2E 5] PASS: Real NLU + safety abort works with zero UI clients connected', flush=True)

        # 6. Reconnect late-join + complete trip to SUCCEEDED
        ui2 = ThinUiClient('e2e_ui_reconnect')
        executor.add_node(ui2)
        assert wait_until(
            lambda: ui2.ui_status is not None
            and (ui2.ui_status.get('last_terminal_mission') or {}).get('client_request_id') == 'e2e-nlu-1',
            timeout=8.0,
        ), f'Reconnected UI did not see last_terminal_mission: {ui2.ui_status}'

        lock_passenger(obs)
        prev_nav_count = len(obs.nav_statuses)
        ui2.send({'client_request_id': 'e2e-succ-1', 'type': 'navigate_id', 'destination_id': 'gate_15'})
        assert wait_until(
            lambda: any(
                s.get('is_terminal') is True and s.get('status') == 'SUCCEEDED'
                for s in obs.nav_statuses[prev_nav_count:]
            ),
            timeout=45.0,
        ), f'Trip did not reach SUCCEEDED: {obs.nav_statuses[prev_nav_count:]}'

        ui3 = ThinUiClient('e2e_ui_late_after_success')
        executor.add_node(ui3)
        assert wait_until(
            lambda: ui3.ui_status is not None
            and (ui3.ui_status.get('last_terminal_mission') or {}).get('status') == 'SUCCEEDED'
            and (ui3.ui_status.get('last_terminal_mission') or {}).get('client_request_id') == 'e2e-succ-1',
            timeout=8.0,
        ), f'Late-join UI did not see SUCCEEDED last_terminal_mission: {ui3.ui_status}'
        print('[E2E 6] PASS: Reconnect saw last_terminal_mission & trip reached SUCCEEDED', flush=True)
        print('ALL_E2E_UI_DEMO_TESTS_PASSED', flush=True)
    finally:
        executor.shutdown()
        if ui is not None:
            ui.destroy_node()
        obs.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(pgid, signal.SIGTERM)
            launch_proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass


if __name__ == '__main__':
    main()
