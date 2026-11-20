#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kịch bản nghiệm thu tích hợp Gazebo + Nav2 + Airport Robot Voice.
Thực hiện tự động chuỗi tương tác tiếng Việt và kiểm tra chuyển động thực tế trong Gazebo.
"""

import os
import sys
import time
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class GazeboVoiceVerifier(Node):
    def __init__(self):
        super().__init__("gazebo_voice_verifier")
        self.get_logger().info("Khởi tạo GazeboVoiceVerifier Node...")

        self.intents = []
        self.speeches = []
        self.feedbacks = []
        self.nav_states = []
        self.latest_odom = None
        self.latest_state = None
        self.odom_history = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(String, "/voice/intent", self._on_intent, 10)
        self.create_subscription(String, "/voice/say", self._on_say, 10)
        self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.create_subscription(String, "/voice/nav_state", self._on_nav_state, transient_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.pub_text = self.create_publisher(String, "/voice/text", 10)

    def _on_intent(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.intents.append(d)
            intent = d.get("intent")
            action = d.get("action")
            target = d.get("target_id")
            self.get_logger().info(f"[INTENT] {intent} | action={action} | target={target}")
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.speeches.append(d)
            text = d.get("text")
            self.get_logger().info(f"[SAY] \"{text}\"")
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.feedbacks.append(d)
            event = d.get("event")
            status = d.get("status")
            message = d.get("message")
            self.get_logger().info(f"[FEEDBACK] {event} / {status}: {message}")
        except Exception:
            pass

    def _on_nav_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.latest_state = d
            self.nav_states.append(d)
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.latest_odom = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "vx": vel.x, "vy": vel.y,
            "timestamp": time.time()
        }
        self.odom_history.append(self.latest_odom)
        if len(self.odom_history) > 500:
            self.odom_history.pop(0)

    def send_voice_text(self, text: str):
        msg = String()
        msg.data = text
        self.get_logger().info(f">>> [GỬI GIỌNG NÓI / BÀN PHÍM]: \"{text}\"")
        self.pub_text.publish(msg)

    def wait_for_state(self, target_state: str, timeout_sec: float = 10.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state and self.latest_state.get("mission_state") == target_state:
                return True
        return False

    def wait_for_movement(self, min_distance: float = 0.05, timeout_sec: float = 15.0) -> bool:
        start = time.time()
        rclpy.spin_once(self, timeout_sec=0.1)
        if not self.latest_odom:
            return False
        start_x = self.latest_odom["x"]
        start_y = self.latest_odom["y"]
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_odom:
                dx = self.latest_odom["x"] - start_x
                dy = self.latest_odom["y"] - start_y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist >= min_distance:
                    curr_x = self.latest_odom["x"]
                    curr_y = self.latest_odom["y"]
                    self.get_logger().info(f"[ODOM] Đã di chuyển {dist:.3f}m từ ({start_x:.2f}, {start_y:.2f}) -> ({curr_x:.2f}, {curr_y:.2f})")
                    return True
        return False

    def wait_for_speed(self, max_speed: float = 0.02, timeout_sec: float = 5.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_odom:
                vx = self.latest_odom["vx"]
                vy = self.latest_odom["vy"]
                speed = math.sqrt(vx*vx + vy*vy)
                if speed <= max_speed:
                    return True
        return False

def spin_for(node, duration_sec: float):
    start = time.time()
    while time.time() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

def run_verification():
    print("="*70)
    print("BẮT ĐẦU NGHIỆM THU TÍCH HỢP GAZEBO + NAV2 + AIRPORT ROBOT VOICE")
    print("="*70)

    rclpy.init()
    verifier = GazeboVoiceVerifier()

    results = {}

    try:
        verifier.get_logger().info("Chờ các topic /odom và /voice/nav_state sẵn sàng...")
        for _ in range(60):
            rclpy.spin_once(verifier, timeout_sec=0.2)
            if verifier.latest_odom and verifier.latest_state:
                break
        
        if not verifier.latest_odom:
            verifier.get_logger().warn("Chưa nhận được /odom. Tiếp tục thử nghiệm...")
        if not verifier.latest_state:
            verifier.get_logger().warn("Chưa nhận được /voice/nav_state. Tiếp tục thử nghiệm...")

        # ---------------------------------------------------------------------
        # CA 1: Đi tới đích (Cổng A3)
        # ---------------------------------------------------------------------
        print("\n" + "-"*60)
        print("CA 1: DI CHUYỂN TỚI ĐÍCH (CỔNG A3)")
        print("-"*60)
        verifier.send_voice_text("dẫn tôi đến cổng A3")
        spin_for(verifier, 1.5)

        has_nav_intent = any(i.get("intent") == "guide_me" and i.get("target_id") == "gate_a3" for i in verifier.intents)
        ok_intent = "ĐẠT" if has_nav_intent else "KHÔNG ĐẠT"
        print(f"  [1.1] Kiểm tra Intent: guide_me, target=gate_a3 -> {ok_intent}")

        is_navigating = verifier.wait_for_state("NAVIGATING", timeout_sec=10.0)
        ok_nav = "ĐẠT" if is_navigating else "KHÔNG ĐẠT"
        print(f"  [1.2] Kiểm tra Nav State: NAVIGATING -> {ok_nav}")
        if verifier.latest_state:
            m_id = verifier.latest_state.get("mission_id")
            att_id = verifier.latest_state.get("goal_attempt_id")
            print(f"        Mission ID: {m_id}, Attempt ID: {att_id}")

        moved = verifier.wait_for_movement(min_distance=0.08, timeout_sec=15.0)
        ok_moved = "ĐẠT" if moved else "KHÔNG ĐẠT"
        print(f"  [1.3] Kiểm tra Odometry chuyển động trong Gazebo (delta >= 0.08m) -> {ok_moved}")
        results["Ca 1: Đi tới đích"] = (has_nav_intent and is_navigating and moved)

        # ---------------------------------------------------------------------
        # CA 2: Tạm dừng và Đi tiếp (Pause & Resume)
        # ---------------------------------------------------------------------
        print("\n" + "-"*60)
        print("CA 2: TẠM DỪNG VÀ ĐI TIẾP (PAUSE & RESUME)")
        print("-"*60)
        verifier.send_voice_text("dừng lại")
        is_paused = verifier.wait_for_state("PAUSED", timeout_sec=8.0)
        stopped = verifier.wait_for_speed(max_speed=0.03, timeout_sec=5.0)
        ok_paused = "ĐẠT" if is_paused else "KHÔNG ĐẠT"
        ok_stopped = "ĐẠT" if stopped else "KHÔNG ĐẠT"
        print(f"  [2.1] Kiểm tra Lệnh dừng: Nav State PAUSED -> {ok_paused}")
        print(f"  [2.2] Kiểm tra Dừng vật lý (speed <= 0.03 m/s) -> {ok_stopped}")

        verifier.send_voice_text("đi tiếp")
        is_resumed = verifier.wait_for_state("NAVIGATING", timeout_sec=8.0)
        resumed_moving = verifier.wait_for_movement(min_distance=0.05, timeout_sec=12.0)
        ok_resumed = "ĐẠT" if is_resumed else "KHÔNG ĐẠT"
        ok_res_mov = "ĐẠT" if resumed_moving else "KHÔNG ĐẠT"
        print(f"  [2.3] Kiểm tra Lệnh đi tiếp: Nav State NAVIGATING -> {ok_resumed}")
        print(f"  [2.4] Kiểm tra Tiếp tục chuyển động sau resume -> {ok_res_mov}")
        results["Ca 2: Tạm dừng & Đi tiếp"] = (is_paused and stopped and is_resumed and resumed_moving)

        # ---------------------------------------------------------------------
        # CA 3: Đổi đích khi đang di chuyển (Replace destination)
        # ---------------------------------------------------------------------
        print("\n" + "-"*60)
        print("CA 3: ĐỔI ĐÍCH KHI ĐANG DI CHUYỂN (REPLACE -> CỔNG A4)")
        print("-"*60)
        prev_attempt_id = verifier.latest_state.get("goal_attempt_id") if verifier.latest_state else ""
        verifier.send_voice_text("đổi sang cổng A4")
        spin_for(verifier, 2.0)

        replace_ok = False
        start_wait = time.time()
        while time.time() - start_wait < 12.0:
            rclpy.spin_once(verifier, timeout_sec=0.2)
            if verifier.latest_state:
                curr_target = verifier.latest_state.get("current_target_id")
                curr_att = verifier.latest_state.get("goal_attempt_id")
                state = verifier.latest_state.get("mission_state")
                if curr_target == "gate_a4" and state == "NAVIGATING" and curr_att != prev_attempt_id:
                    replace_ok = True
                    break

        ok_rep = "ĐẠT" if replace_ok else "KHÔNG ĐẠT"
        print(f"  [3.1] Kiểm tra Đổi đích sang gate_a4: target mới & attempt mới -> {ok_rep}")
        if verifier.latest_state:
            t_name = verifier.latest_state.get("current_target_name")
            a_id = verifier.latest_state.get("goal_attempt_id")
            print(f"        Đích hiện tại: {t_name}, Attempt: {a_id}")
        
        replace_moving = verifier.wait_for_movement(min_distance=0.05, timeout_sec=12.0)
        ok_rep_mov = "ĐẠT" if replace_moving else "KHÔNG ĐẠT"
        print(f"  [3.2] Kiểm tra Robot tiếp tục di chuyển hướng về đích mới -> {ok_rep_mov}")
        results["Ca 3: Đổi đích khi đang di chuyển"] = (replace_ok and replace_moving)

        # ---------------------------------------------------------------------
        # CA 4: Hủy khi đang đổi đích (Cancel during replace)
        # ---------------------------------------------------------------------
        print("\n" + "-"*60)
        print("CA 4: HỦY KHI ĐANG ĐỔI ĐÍCH (CANCEL DURING REPLACE)")
        print("-"*60)
        verifier.send_voice_text("đổi sang quầy check in")
        time.sleep(0.1)
        verifier.send_voice_text("hủy chuyến đi")
        
        is_cancelled = verifier.wait_for_state("IDLE", timeout_sec=10.0)
        spin_for(verifier, 2.0)
        is_idle_stable = (verifier.latest_state.get("mission_state") == "IDLE" if verifier.latest_state else False)
        target_cleared = (verifier.latest_state.get("current_target_id") is None if verifier.latest_state else True)
        
        ok_can = "ĐẠT" if is_cancelled else "KHÔNG ĐẠT"
        ok_clr = "ĐẠT" if target_cleared else "KHÔNG ĐẠT"
        print(f"  [4.1] Kiểm tra Hủy thành công: Nav State về IDLE -> {ok_can}")
        print(f"  [4.2] Kiểm tra Đích thay thế bị loại bỏ, không gửi nhầm goal mới -> {ok_clr}")
        results["Ca 4: Hủy khi đang đổi đích"] = (is_cancelled and is_idle_stable and target_cleared)

        # ---------------------------------------------------------------------
        # CA 5: Chọn nhà vệ sinh tối ưu bằng Planner (Category Toilet)
        # ---------------------------------------------------------------------
        print("\n" + "-"*60)
        print("CA 5: CHỌN ĐÍCH TỐI ƯU TRONG NHÓM QUA PLANNER (TOILET)")
        print("-"*60)
        verifier.send_voice_text("nhà vệ sinh ở đâu")
        spin_for(verifier, 3.0)
        
        is_nav_toilet = verifier.wait_for_state("NAVIGATING", timeout_sec=10.0)
        chosen_target = verifier.latest_state.get("current_target_id") if verifier.latest_state else ""
        t_name = verifier.latest_state.get("current_target_name") if verifier.latest_state else ""
        ok_toi = "ĐẠT" if is_nav_toilet else "KHÔNG ĐẠT"
        print(f"  [5.1] Kiểm tra Planner chọn toilet và kích hoạt NAVIGATING -> {ok_toi}")
        print(f"        Toilet được chọn tối ưu: {chosen_target} ({t_name})")
        
        verifier.send_voice_text("hủy chuyến")
        verifier.wait_for_state("IDLE", timeout_sec=6.0)
        results["Ca 5: Chọn đích tối ưu qua Planner"] = (is_nav_toilet and chosen_target in ("toilet_1", "toilet_2"))

    finally:
        verifier.destroy_node()
        rclpy.shutdown()

    # -------------------------------------------------------------------------
    # TỔNG KẾT BẰNG CHỨNG
    # -------------------------------------------------------------------------
    print("\n" + "="*70)
    print("TỔNG HỢP KẾT QUẢ NGHIỆM THU GAZEBO + NAV2 + VOICE")
    print("="*70)
    all_passed = True
    for test_name, status in results.items():
        res_str = "PASSED" if status else "FAILED"
        if not status:
            all_passed = False
        print(f"  * {test_name:<42}: {res_str}")
    print("="*70)
    concl = "TẤT CẢ CÁC CA ĐẠT 100%" if all_passed else "CÓ CA CHƯA ĐẠT"
    print(f"KẾT LUẬN CHUNG: {concl}")
    print("="*70)
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(run_verification())
