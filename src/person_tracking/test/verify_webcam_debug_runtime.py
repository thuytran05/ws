#!/usr/bin/env python3
"""Bounded 8-second ROS 2 runtime verification of webcam_debug.launch.py on ROS_DOMAIN_ID=43."""

import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class WebcamRuntimeVerifier(Node):
    def __init__(self):
        super().__init__("webcam_runtime_verifier")
        self.rgb_count = 0
        self.debug_img_count = 0
        self.obs_count = 0
        self.session_count = 0
        self.debug_lock_count = 0
        self.last_obs = None
        self.last_session = None
        self.last_debug_shape = None

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, "/camera/color/image_raw", self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/tracking/debug_image", self._on_debug, qos_profile_sensor_data)
        self.create_subscription(String, "/tracking/observation", self._on_obs, reliable_qos)
        self.create_subscription(String, "/tracking/session_state", self._on_sess, transient_qos)
        self.create_subscription(String, "/tracking/debug_lock_state", self._on_dbg_lock, reliable_qos)

    def _on_rgb(self, msg: Image):
        self.rgb_count += 1

    def _on_debug(self, msg: Image):
        self.debug_img_count += 1
        self.last_debug_shape = (msg.width, msg.height, msg.encoding)

    def _on_obs(self, msg: String):
        self.obs_count += 1
        self.last_obs = json.loads(msg.data)

    def _on_sess(self, msg: String):
        self.session_count += 1
        self.last_session = json.loads(msg.data)

    def _on_dbg_lock(self, msg: String):
        self.debug_lock_count += 1


def main():
    os.environ["ROS_DOMAIN_ID"] = "43"
    proc = subprocess.Popen(
        [
            "ros2", "launch", "person_tracking", "webcam_debug.launch.py",
            "camera_index:=0", "fps:=30.0", "device:=cuda:0", "view_debug:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    node = WebcamRuntimeVerifier()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.rgb_count >= 10
                and node.debug_img_count >= 5
                and node.obs_count >= 5
                and node.session_count >= 2
                and node.debug_lock_count >= 2
            ):
                break
        cmd_vel_pubs = node.count_publishers("/cmd_vel")
    finally:
        # Cleanly terminate only the launch process group we spawned
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"rgb_count={node.rgb_count}, debug_img_count={node.debug_img_count}, obs_count={node.obs_count}, "
          f"session_count={node.session_count}, debug_lock_count={node.debug_lock_count}, cmd_vel_pubs={cmd_vel_pubs}")
    print(f"last_debug_shape={node.last_debug_shape}")
    if node.last_obs:
        print(f"obs.source_mode={node.last_obs.get('source_mode')}, obs.data_quality={node.last_obs.get('data_quality')}")
    if node.last_session:
        print(f"session.state={node.last_session.get('state')}, session.locked={node.last_session.get('locked')}")

    ok = (
        node.rgb_count >= 5
        and node.debug_img_count >= 3
        and node.obs_count >= 3
        and node.session_count >= 1
        and node.debug_lock_count >= 1
        and cmd_vel_pubs == 0
        and node.last_debug_shape == (640, 480, "bgr8")
        and node.last_obs is not None
        and node.last_obs.get("source_mode") == "live_webcam"
        and node.last_obs.get("data_quality") == "RGB_ONLY"
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
