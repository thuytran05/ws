#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kiểm thử chuyên sâu 2 ca nghiệm thu quan trọng theo yêu cầu:
1. Đang đi A3 -> đổi sang A4 -> hủy chuyến trong lúc xử lý đổi đích:
   Xác nhận không có goal A4 được gửi và robot mô phỏng đã dừng hẳn.
2. Từ cùng một pose ban đầu, lấy đường Nav2 tới 2 nhà vệ sinh,
   ghi chiều dài từng đường và xác nhận hệ thống chọn đúng điểm.
"""

import os
import sys
import time
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

def compute_path_length(path_msg) -> float:
    poses = getattr(path_msg, "poses", [])
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        p1 = poses[i - 1].pose.position
        p2 = poses[i].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total

class SpecificCasesVerifier(Node):
    def __init__(self):
        super().__init__("specific_cases_verifier")
        self.get_logger().info("Khởi tạo SpecificCasesVerifier Node...")

        self.intents = []
        self.speeches = []
        self.feedbacks = []
        self.nav_states = []
        self.latest_odom = None
        self.latest_state = None
        self.all_sent_nav_goals = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(String, "/voice/intent", self._on_intent, 10)
        self.create_subscription(String, "/voice/say", self._on_say, 10)
        self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.create_subscription(String, "/voice/nav_state", self._on_nav_state, transient_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.pub_text = self.create_publisher(String, "/voice/text", 10)
        self._path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def _on_intent(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.intents.append(d)
            it = d.get("intent")
            ac = d.get("action")
            tg = d.get("target_id")
            self.get_logger().info(f"[INTENT] {it} | action={ac} | target={tg}")
        except Exception:
            pass

    def _on_say(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.speeches.append(d)
            txt = d.get("text")
            self.get_logger().info(f"[SAY] \"{txt}\"")
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.feedbacks.append(d)
            ev = d.get("event")
            st = d.get("status")
            msg_t = d.get("message")
            self.get_logger().info(f"[FEEDBACK] {ev} / {st}: {msg_t}")
        except Exception:
            pass

    def _on_nav_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.latest_state = d
            self.nav_states.append(d)
            t_id = d.get("current_target_id")
            att_id = d.get("goal_attempt_id")
            st = d.get("mission_state")
            if st in ("STARTING", "NAVIGATING") and att_id:
                if not any(g.get("attempt_id") == att_id for g in self.all_sent_nav_goals):
                    self.all_sent_nav_goals.append({
                        "attempt_id": att_id,
                        "target_id": t_id,
                        "mission_id": d.get("mission_id"),
                        "timestamp": time.time()
                    })
                    self.get_logger().info(f"[GOAL RECORDED] Attempt={att_id} -> Target={t_id} (State={st})")
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.latest_odom = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "vx": vel.x, "vy": vel.y,
            "speed": math.sqrt(vel.x*vel.x + vel.y*vel.y),
            "timestamp": time.time()
        }

    def send_voice_text(self, text: str):
        msg = String()
        msg.data = text
        self.get_logger().info(f">>> [GỬI GIỌNG NÓI]: \"{text}\"")
        self.pub_text.publish(msg)

    def wait_for_movement(self, min_dist: float = 0.05, timeout_sec: float = 15.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_odom:
                break
        if not self.latest_odom:
            return False
        x0, y0 = self.latest_odom["x"], self.latest_odom["y"]
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_odom:
                dx = self.latest_odom["x"] - x0
                dy = self.latest_odom["y"] - y0
                dist = math.sqrt(dx*dx + dy*dy)
                if dist >= min_dist:
                    cur_x = self.latest_odom["x"]
                    cur_y = self.latest_odom["y"]
                    self.get_logger().info(f"[ODOM] Đã di chuyển {dist:.3f}m từ ({x0:.2f}, {y0:.2f}) -> ({cur_x:.2f}, {cur_y:.2f})")
                    return True
        return False

    def wait_for_stop(self, max_speed: float = 0.02, stable_sec: float = 2.0, timeout_sec: float = 8.0) -> bool:
        start = time.time()
        stopped_start = None
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_odom:
                if self.latest_odom["speed"] <= max_speed:
                    if stopped_start is None:
                        stopped_start = time.time()
                    elif time.time() - stopped_start >= stable_sec:
                        return True
                else:
                    stopped_start = None
        return False

    def compute_direct_path_length(self, target_x: float, target_y: float) -> float:
        """Gọi trực tiếp action server ComputePathToPose để đo chiều dài đường đi."""
        if not self._path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server compute_path_to_pose không sẵn sàng!")
            return -1.0

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = target_x
        goal.goal.pose.position.y = target_y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        send_future = self._path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"Goal ComputePathToPose tới ({target_x}, {target_y}) bị từ chối!")
            return -1.0

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=6.0)
        res_wrap = res_future.result()
        if not res_wrap or not res_wrap.result:
            return -1.0

        path = res_wrap.result.path
        length = compute_path_length(path)
        return length

def spin_for(node, sec: float):
    start = time.time()
    while time.time() - start < sec:
        rclpy.spin_once(node, timeout_sec=0.1)

def run():
    print("="*75)
    print("CHẠY XÁC MINH CHUYÊN SÂU 2 CA NGHIỆM THU TRÊN GAZEBO THỰC TẾ")
    print("="*75)

    rclpy.init()
    node = SpecificCasesVerifier()

    try:
        node.get_logger().info("Chờ topic /odom và /voice/nav_state sẵn sàng...")
        for _ in range(60):
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_odom and node.latest_state:
                break

        init_x = node.latest_odom["x"] if node.latest_odom else 0.0
        init_y = node.latest_odom["y"] if node.latest_odom else 0.0
        init_spd = node.latest_odom["speed"] if node.latest_odom else 0.0
        print(f"Robot pose ban đầu trên Gazebo: x={init_x:.3f}, y={init_y:.3f}, speed={init_spd:.3f} m/s")

        # =====================================================================
        # CA 1: ĐANG ĐI A3 -> ĐỔI SANG A4 -> HỦY CHUYẾN TRONG LÚC XỬ LÝ ĐỔI ĐÍCH
        # =====================================================================
        print("\n" + "-"*75)
        print("CA 1: ĐANG ĐI A3 -> ĐỔI SANG A4 -> HỦY CHUYẾN TRONG LÚC XỬ LÝ ĐỔI ĐÍCH")
        print("-" * 75)

        # 1. Gửi lệnh dẫn đường tới cổng A3
        node.send_voice_text("dẫn tôi đến cổng A3")
        spin_for(node, 1.5)

        # Chờ robot NAVIGATING và bắt đầu di chuyển
        is_navigating = False
        start_wait = time.time()
        while time.time() - start_wait < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_state and node.latest_state.get("mission_state") == "NAVIGATING":
                is_navigating = True
                break

        ok_nav = "ĐẠT" if is_navigating else "KHÔNG ĐẠT"
        print(f"  [1.1] Robot nhận goal A3 và bắt đầu NAVIGATING: {ok_nav}")
        if node.latest_state:
            m_id = node.latest_state.get("mission_id")
            att_id = node.latest_state.get("goal_attempt_id")
            print(f"        Mission A3: {m_id}, Attempt A3: {att_id}")

        # Chờ robot di chuyển thực tế ít nhất 0.05m
        moved_a3 = node.wait_for_movement(min_dist=0.05, timeout_sec=12.0)
        ok_mov = "ĐẠT" if moved_a3 else "KHÔNG ĐẠT"
        print(f"  [1.2] Robot di chuyển thực tế trong Gazebo hướng tới A3: {ok_mov}")

        # 2. Khi robot ĐANG DI CHUYỂN, gửi lệnh đổi sang cổng A4
        print("  >>> Phát lệnh đổi sang cổng A4 khi robot đang di chuyển...")
        node.send_voice_text("đổi sang cổng A4")
        time.sleep(0.08)

        # 3. NGAY LẬP TỨC trong lúc hệ thống đang cancel A3 và chuẩn bị A4: gửi lệnh HỦY CHUYẾN
        print("  >>> Phát lệnh HỦY CHUYẾN ngay trong lúc đang xử lý đổi đích...")
        node.send_voice_text("hủy chuyến đi")

        # 4. Chờ hệ thống dọn dẹp và về IDLE
        start_idle_wait = time.time()
        reached_idle = False
        while time.time() - start_idle_wait < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_state and node.latest_state.get("mission_state") == "IDLE":
                reached_idle = True
                break

        spin_for(node, 2.5)  # Quan sát thêm 2.5s để chắc chắn không có goal A4 phát sinh

        # Kiểm tra xác nhận KHÔNG CÓ GOAL A4 ĐƯỢC GỬI SANG NAV2
        goals_for_a4 = [g for g in node.all_sent_nav_goals if g.get("target_id") == "gate_a4"]
        no_a4_goal_sent = (len(goals_for_a4) == 0)

        # Kiểm tra ROBOT MÔ PHỎNG ĐÃ DỪNG HẲN
        robot_stopped = node.wait_for_stop(max_speed=0.02, stable_sec=2.0, timeout_sec=6.0)
        final_speed = node.latest_odom["speed"] if node.latest_odom else 999.0

        ok_idle = "ĐẠT" if reached_idle else "KHÔNG ĐẠT"
        ok_no_a4 = "ĐẠT" if no_a4_goal_sent else "KHÔNG ĐẠT"
        ok_stop = "ĐẠT" if robot_stopped else "KHÔNG ĐẠT"
        print(f"  [1.3] Máy trạng thái về IDLE an toàn: {ok_idle}")
        print(f"  [1.4] XÁC NHẬN KHÔNG CÓ GOAL A4 NÀO ĐƯỢC GỬI SANG NAV2: {ok_no_a4} (Số goal A4 = {len(goals_for_a4)})")
        print(f"  [1.5] XÁC NHẬN ROBOT MÔ PHỎNG ĐÃ DỪNG HẲN TRONG GAZEBO: {ok_stop} (Vận tốc hiện tại = {final_speed:.4f} m/s)")

        ca1_passed = (is_navigating and moved_a3 and reached_idle and no_a4_goal_sent and robot_stopped)

        # =====================================================================
        # CA 2: TỪ CÙNG MỘT POSE, LẤY ĐƯỜNG TỚI 2 NHÀ VỆ SINH, GHI CHIỀU DÀI & CHỌN ĐÚNG ĐIỂM
        # =====================================================================
        print("\n" + "-"*75)
        print("CA 2: LẤY ĐƯỜNG TỚI 2 NHÀ VỆ SINH TỪ CÙNG POSE, SO SÁNH CHIỀU DÀI VÀ CHỌN ĐIỂM")
        print("-" * 75)

        t1_x, t1_y = -1.0, 1.8
        t2_x, t2_y =  0.2, 1.8

        current_x = node.latest_odom["x"] if node.latest_odom else 0.0
        current_y = node.latest_odom["y"] if node.latest_odom else 0.0
        print(f"  Pose bắt đầu đo đường: x={current_x:.3f}, y={current_y:.3f}")

        len_t1 = node.compute_direct_path_length(t1_x, t1_y)
        len_t2 = node.compute_direct_path_length(t2_x, t2_y)

        print(f"  [2.1] Chiều dài đường Nav2 tới toilet_1 (-1.0, 1.8): {len_t1:.3f} mét")
        print(f"  [2.2] Chiều dài đường Nav2 tới toilet_2 ( 0.2, 1.8): {len_t2:.3f} mét")

        expected_shorter = "toilet_1" if (len_t1 > 0 and len_t1 < len_t2) else "toilet_2"
        expected_len = min(len_t1, len_t2)
        print(f"  --> Điểm có đường ngắn hơn trên bản đồ Nav2 là: {expected_shorter} ({expected_len:.3f}m)")

        # Gửi câu hỏi tiếng Việt vào Voice System: "nhà vệ sinh ở đâu"
        print("  >>> Phát câu nói: \"nhà vệ sinh ở đâu\"...")
        node.send_voice_text("nhà vệ sinh ở đâu")
        spin_for(node, 3.5)

        # Chờ node voice chọn điểm và chuyển NAVIGATING
        toilet_navigating = False
        chosen_toilet = None
        start_wait_toi = time.time()
        while time.time() - start_wait_toi < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_state and node.latest_state.get("mission_state") == "NAVIGATING":
                toilet_navigating = True
                chosen_toilet = node.latest_state.get("current_target_id")
                break

        ok_t_nav = "ĐẠT" if toilet_navigating else "KHÔNG ĐẠT"
        t_name = node.latest_state.get("current_target_name") if node.latest_state else ""
        ok_t_match = "ĐẠT" if chosen_toilet == expected_shorter else "KHÔNG ĐẠT"
        print(f"  [2.3] Hệ thống Voice kích hoạt NAVIGATING: {ok_t_nav}")
        print(f"  [2.4] Điểm đến được Voice System lựa chọn: {chosen_toilet} ({t_name})")
        print(f"  [2.5] XÁC NHẬN HỆ THỐNG ĐÃ CHỌN ĐÚNG ĐIỂM NGẮN NHẤT ({expected_shorter}): {ok_t_match}")

        # Dọn dẹp: Hủy chuyến về IDLE
        node.send_voice_text("hủy chuyến")
        spin_for(node, 2.0)

        ca2_passed = (len_t1 > 0 and len_t2 > 0 and toilet_navigating and chosen_toilet == expected_shorter)

        # =====================================================================
        # TỔNG KẾT
        # =====================================================================
        print("\n" + "="*75)
        print("KẾT QUẢ XÁC MINH 2 CA CHUYÊN SÂU TRÊN GAZEBO")
        print("="*75)
        res_ca1 = "PASSED" if ca1_passed else "FAILED"
        res_ca2 = "PASSED" if ca2_passed else "FAILED"
        print(f"  * Ca 1: Đi A3 -> Đổi A4 -> Hủy khi đang đổi (Không gửi A4 & Robot dừng): {res_ca1}")
        print(f"  * Ca 2: Chiều dài đường 2 toilet ({len_t1:.2f}m vs {len_t2:.2f}m) & Chọn đúng điểm: {res_ca2}")
        print("="*75)
        all_ok = ca1_passed and ca2_passed
        concl = "TẤT CẢ ĐẠT 100%" if all_ok else "CHƯA ĐẠT"
        print(f"KẾT LUẬN: {concl}")
        print("="*75)
        return 0 if all_ok else 1

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(run())
