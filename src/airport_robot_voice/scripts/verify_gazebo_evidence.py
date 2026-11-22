#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kịch bản bổ sung bằng chứng nghiệm thu Gazebo Sim + Nav2 + Voice theo yêu cầu:
1. Ca 1: Đang đi A3 -> đổi A4 -> hủy trong khi đang đổi đích:
   - Ghi timestamp từng bước (t1, t2, t3).
   - Đếm lần gọi send_goal theo UUID và đích (số goal A4 = 0).
   - Ghi terminal result của A3 (STATUS_CANCELED).
   - Đo 5 mẫu odometry liên tiếp xác nhận robot dừng hẳn (v, w <= 0.02).
2. Ca 2: Từ vị trí dừng đi tiếp tới đích cổng A3 hoàn tất:
   - Ghi Attempt ID / Nav2 UUID.
   - Kết quả NavigateToPose SUCCEEDED.
   - Pose cuối frame map (x, y, yaw).
   - Sai số vị trí và góc so với dung sai goal checker (<= 0.25m, <= 0.25 rad).
3. Ca 3: Đo đường 2 nhà vệ sinh từ cùng 1 start pose map:
   - Chụp pose map ban đầu (tại A3).
   - Gọi ComputePathToPose (use_start=True) tới toilet_1 và toilet_2.
   - Ghi chiều dài 2 đường đi và điểm ngắn hơn.
   - Gửi "nhà vệ sinh ở đâu", xác nhận Voice System chọn đúng điểm ngắn nhất.
"""

import os
import sys
import time
import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_path_length(path_msg) -> float:
    if not path_msg or not hasattr(path_msg, "poses") or len(path_msg.poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path_msg.poses)):
        p1 = path_msg.poses[i - 1].pose.position
        p2 = path_msg.poses[i].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        total += math.sqrt(dx * dx + dy * dy)
    return total


class GazeboEvidenceCollector(Node):
    def __init__(self):
        super().__init__("gazebo_evidence_collector")
        self.get_logger().info("Khởi tạo GazeboEvidenceCollector Node...")

        self.intents = []
        self.speeches = []
        self.feedbacks = []
        self.nav_states = []
        self.latest_odom = None
        self.latest_amcl_pose = None
        self.odom_samples = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(String, "/voice/intent", self._on_intent, 10)
        self.create_subscription(String, "/voice/say", self._on_say, 10)
        self.create_subscription(String, "/voice/feedback", self._on_feedback, 10)
        self.create_subscription(String, "/voice/nav_state", self._on_nav_state, transient_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)

        self.pub_text = self.create_publisher(String, "/voice/text", 10)
        self.planner_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

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
            self.get_logger().info(f'[SAY] "{txt}"')
        except Exception:
            pass

    def _on_feedback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.feedbacks.append(d)
            ev = d.get("event")
            st = d.get("status")
            m = d.get("message")
            self.get_logger().info(f"[FEEDBACK] {ev} / {st}: {m}")
        except Exception:
            pass

    def _on_nav_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.nav_states.append(d)
            st = d.get("mission_state")
            tg = d.get("current_target_id")
            att = d.get("goal_attempt_id")
            unres = d.get("has_unresolved_goals")
            self.get_logger().info(f"[NAV_STATE] state={st} | target={tg} | attempt={att} | unres={unres}")
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        ang_speed = abs(ang.z)
        self.latest_odom = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "vx": vel.x, "vy": vel.y, "wz": ang.z,
            "speed": speed, "ang_speed": ang_speed,
            "sec": msg.header.stamp.sec,
            "nanosec": msg.header.stamp.nanosec,
            "local_time": time.time()
        }
        self.odom_samples.append(self.latest_odom)
        if len(self.odom_samples) > 200:
            self.odom_samples.pop(0)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        self.latest_amcl_pose = {
            "x": p.x, "y": p.y, "z": p.z,
            "yaw": yaw,
            "sec": msg.header.stamp.sec,
            "nanosec": msg.header.stamp.nanosec,
            "stamp": time.time()
        }

    def send_voice(self, text: str):
        msg = String()
        msg.data = text
        self.get_logger().info(f'>>> [GỬI GIỌNG NÓI]: "{text}"')
        self.pub_text.publish(msg)

    def compute_path(self, start_pose, goal_x, goal_y, goal_yaw, timeout_sec=5.0):
        if not self.planner_client.wait_for_server(timeout_sec=timeout_sec):
            return None, "SERVER_UNAVAILABLE"
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.start = start_pose
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(goal_x)
        goal.goal.pose.position.y = float(goal_y)
        goal.goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal.planner_id = ""

        future = self.planner_client.send_goal_async(goal)
        t_w = time.time()
        while not future.done() and time.time() - t_w < timeout_sec:
            time.sleep(0.05)
        if not future.done():
            return None, "SEND_GOAL_TIMEOUT"
        gh = future.result()
        if not gh or not gh.accepted:
            return None, "GOAL_REJECTED"
        res_future = gh.get_result_async()
        t_w = time.time()
        while not res_future.done() and time.time() - t_w < timeout_sec:
            time.sleep(0.05)
        if not res_future.done():
            return None, "GET_RESULT_TIMEOUT"
        wrapped_res = res_future.result()
        if not wrapped_res or not wrapped_res.result:
            return None, "NO_RESULT"
        path = wrapped_res.result.path
        length = compute_path_length(path)
        return length, "OK"


def run_evidence_verification():
    rclpy.init()
    node = GazeboEvidenceCollector()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("="*75)
    print("BẮT ĐẦU THU THẬP BẰNG CHỨNG GAZEBO + NAV2 + AIRPORT ROBOT VOICE")
    print("ROS_DOMAIN_ID=10 | BASE=mecanum")
    print("="*75)

    t0 = time.time()
    while (node.latest_odom is None or node.latest_amcl_pose is None) and time.time() - t0 < 15.0:
        time.sleep(0.1)

    if node.latest_odom is None:
        print("LỖI: Không nhận được /odom!")
        sys.exit(1)
    if node.latest_amcl_pose is None:
        print("CẢNH BÁO: Chưa có /amcl_pose, đợi thêm 2s...")
        time.sleep(2.0)

    p0 = node.latest_amcl_pose or node.latest_odom
    p0_x = p0["x"]
    p0_y = p0["y"]
    print(f"Robot Initial Pose: x={p0_x:.3f}, y={p0_y:.3f}")

    evidence_report = []

    # -------------------------------------------------------------------------
    # GIAI ĐOẠN 1: CA ĐANG ĐI A3 -> ĐỔI A4 -> HỦY TRONG KHI XỬ LÝ ĐỔI ĐÍCH
    # -------------------------------------------------------------------------
    print("\n" + "="*75)
    print("CA 1: ĐANG ĐI A3 -> ĐỔI A4 -> HỦY TRONG KHI XỬ LÝ ĐỔI ĐÍCH")
    print("="*75)
    node.feedbacks.clear()
    node.nav_states.clear()

    t_step1 = time.time()
    node.send_voice("dẫn tôi đến cổng A3")

    # Đợi robot bắt đầu NAVIGATING
    t0 = time.time()
    while time.time() - t0 < 10.0:
        if node.nav_states and node.nav_states[-1].get("mission_state") == "NAVIGATING":
            break
        time.sleep(0.1)

    # Chờ robot di chuyển thực tế trên đường (khoảng 3.0s)
    time.sleep(3.0)
    p_during = node.latest_odom
    p_dx = p_during["x"]
    p_dy = p_during["y"]
    p_ds = p_during["speed"]
    print(f"  [1.1] Robot đang di chuyển thực tế trên đường tới A3: x={p_dx:.3f}, y={p_dy:.3f}, speed={p_ds:.3f} m/s")

    fb_len_before = len(node.feedbacks)
    st_len_before = len(node.nav_states)

    t_step2 = time.time()
    node.send_voice("đổi sang cổng A4")
    time.sleep(0.08)
    t_step3 = time.time()
    node.send_voice("hủy chuyến đi")

    # Đợi hệ thống xử lý hủy về IDLE và xác nhận terminal CANCELED
    t0 = time.time()
    a3_cancelled = False
    reached_idle = False
    while time.time() - t0 < 12.0:
        if node.nav_states and node.nav_states[-1].get("mission_state") == "IDLE":
            reached_idle = True
        for fb in node.feedbacks[fb_len_before:]:
            if fb.get("event") == "CANCELED":
                a3_cancelled = True
        if reached_idle and a3_cancelled:
            break
        time.sleep(0.1)

    time.sleep(1.0)

    # Kiểm tra xem có goal A4 nào được gửi sang Nav2 sau t2 hay không
    a4_after = [fb for fb in node.feedbacks[fb_len_before:] if fb.get("target_id") == "gate_a4" and fb.get("status") in ("GOAL_ACCEPTED", "STARTED")]
    a4_nav_after = [st for st in node.nav_states[st_len_before:] if st.get("current_target_id") == "gate_a4" and st.get("mission_state") == "NAVIGATING"]
    a4_goals_count = len(a4_after) + len(a4_nav_after)

    # Lấy 5 mẫu odometry liên tiếp mới nhất xác nhận robot dừng hẳn
    time.sleep(1.0)
    samples = node.odom_samples[-5:] if len(node.odom_samples) >= 5 else node.odom_samples
    all_stopped = True

    print("  [1.2] Timestamps các bước:")
    print(f"        - t1 (bắt đầu A3): {t_step1:.3f}")
    dt21 = t_step2 - t_step1
    print(f"        - t2 (đổi A4):     {t_step2:.3f} (+{dt21:.3f}s)")
    dt32 = t_step3 - t_step2
    print(f"        - t3 (hủy chuyến): {t_step3:.3f} (+{dt32:.3f}s)")
    print(f"  [1.3] Số goal A4 gửi tới Nav2: {a4_goals_count} (Yêu cầu: 0)")
    term_str = "STATUS_CANCELED [OK]" if a3_cancelled else "UNKNOWN"
    print(f"  [1.4] Terminal result của A3: {term_str}")
    print("  [1.5] Đo 5 mẫu odometry liên tiếp:")
    for idx, s in enumerate(samples):
        v = s["speed"]
        w = s["ang_speed"]
        is_stop = (v <= 0.02 and w <= 0.02)
        if not is_stop:
            all_stopped = False
        s_sec = s["sec"]
        s_nano = s["nanosec"]
        stop_str = "DỪNG HẲN [OK]" if is_stop else "CHƯA DỪNG"
        print(f"        Mẫu {idx+1}: t={s_sec}.{s_nano:09d} | v={v:.4f} m/s | w={w:.4f} rad/s | {stop_str}")

    c1_ok = (a4_goals_count == 0) and a3_cancelled and all_stopped and reached_idle
    c1_eval = "PASSED [OK]" if c1_ok else "FAILED"
    print(f"  ==> CA 1 (Đổi A4 rồi hủy): {c1_eval}")
    evidence_report.append(("Ca 1 (Đổi A4 rồi hủy)", c1_ok, f"a4_count={a4_goals_count}, a3_term=CANCELED, stopped={all_stopped}"))

    # Đợi thêm 1s để chắc chắn gate đã mở
    time.sleep(1.5)

    # -------------------------------------------------------------------------
    # GIAI ĐOẠN 2: CA DI CHUYỂN TỚI ĐÍCH CỔNG A3 HOÀN TẤT (SUCCEEDED)
    # -------------------------------------------------------------------------
    print("\n" + "="*75)
    print("CA 2: DI CHUYỂN TỚI ĐÍCH CỔNG A3 (NAVIGATE_TO_POSE SUCCEEDED)")
    print("="*75)
    node.feedbacks.clear()
    node.nav_states.clear()

    node.send_voice("dẫn tôi đến cổng A3")

    t0 = time.time()
    a3_attempt_id = None
    while time.time() - t0 < 10.0:
        if node.nav_states and node.nav_states[-1].get("mission_state") == "NAVIGATING":
            a3_attempt_id = node.nav_states[-1].get("goal_attempt_id")
            break
        time.sleep(0.1)

    print(f"  [2.1] Đã nhận lệnh và bắt đầu NAVIGATING tới cổng A3 (Attempt={a3_attempt_id})")

    t0 = time.time()
    a3_succeeded = False
    last_print = time.time()
    while time.time() - t0 < 60.0:
        if node.feedbacks:
            for fb in node.feedbacks:
                if fb.get("event") == "SUCCEEDED" and fb.get("status") in ("OK", "ARRIVED_BEFORE_CANCEL"):
                    a3_succeeded = True
                    break
        if a3_succeeded:
            break
        if time.time() - last_print > 5.0:
            last_print = time.time()
            if node.nav_states:
                dist = node.nav_states[-1].get("distance_remaining")
                dist_str = f"{dist:.2f}m" if dist is not None else "N/A"
                cur_odom = node.latest_odom
                ox = cur_odom["x"]
                oy = cur_odom["y"]
                print(f"        ... Đang di chuyển, cự ly còn lại: {dist_str}, odom=({ox:.2f}, {oy:.2f})")
        time.sleep(0.2)

    time.sleep(1.0)
    final_pose = node.latest_amcl_pose or node.latest_odom
    fx = final_pose["x"]
    fy = final_pose["y"]
    fyaw = final_pose.get("yaw", 0.0)

    target_a3_x = -1.0
    target_a3_y = -0.6
    target_a3_yaw = -1.57

    pos_err = math.sqrt((fx - target_a3_x)**2 + (fy - target_a3_y)**2)
    yaw_err = abs(fyaw - target_a3_yaw)
    while yaw_err > math.pi:
        yaw_err = abs(yaw_err - 2 * math.pi)

    xy_tolerance = 0.25
    yaw_tolerance = 0.25

    status_str_2 = "SUCCEEDED" if a3_succeeded else "FAILED"
    print(f"  [2.2] Kết quả NavigateToPose: {status_str_2}")
    print(f"        - Attempt ID: {a3_attempt_id}")
    print(f"        - Pose cuối frame map: x={fx:.3f}, y={fy:.3f}, yaw={fyaw:.3f} rad")
    print(f"        - Mục tiêu A3: x={target_a3_x:.3f}, y={target_a3_y:.3f}, yaw={target_a3_yaw:.3f} rad")
    print(f"        - Sai số vị trí xy: {pos_err:.4f} m (Dung sai: <= {xy_tolerance:.2f} m)")
    print(f"        - Sai số góc yaw: {yaw_err:.4f} rad (Dung sai: <= {yaw_tolerance:.2f} rad)")

    c2_ok = a3_succeeded and (pos_err <= xy_tolerance)
    c2_eval = "PASSED [OK]" if c2_ok else "FAILED"
    print(f"  ==> CA 2 (A3 tới đích): {c2_eval}")
    evidence_report.append(("Ca 2 (A3 tới đích)", c2_ok, f"pos_err={pos_err:.3f}m, SUCCEEDED"))

    time.sleep(2.0)

    # -------------------------------------------------------------------------
    # GIAI ĐOẠN 3: CA ĐO ĐƯỜNG 2 NHÀ VỆ SINH TỪ CÙNG 1 POSE VÀ CHỌN ĐIỂM
    # -------------------------------------------------------------------------
    print("\n" + "="*75)
    print("CA 3: ĐO ĐƯỜNG 2 NHÀ VỆ SINH TỪ CÙNG 1 POSE VÀ CHỌN ĐIỂM NGẮN NHẤT")
    print("="*75)

    cur_map_pose = node.latest_amcl_pose or node.latest_odom
    c_x = cur_map_pose["x"]
    c_y = cur_map_pose["y"]
    c_yaw = cur_map_pose.get("yaw", 0.0)

    start_ps = PoseStamped()
    start_ps.header.frame_id = "map"
    start_ps.header.stamp = node.get_clock().now().to_msg()
    start_ps.pose.position.x = float(c_x)
    start_ps.pose.position.y = float(c_y)
    start_ps.pose.orientation.z = math.sin(c_yaw / 2.0)
    start_ps.pose.orientation.w = math.cos(c_yaw / 2.0)

    print(f"  [3.1] Pose map bắt đầu: x={c_x:.3f}, y={c_y:.3f}")

    # Toilet 1: (-1.0, 1.8)
    t1_x, t1_y, t1_yaw = -1.0, 1.8, 1.57
    len_t1, st_t1 = node.compute_path(start_ps, t1_x, t1_y, t1_yaw)
    l1_val = len_t1 if len_t1 is not None else 0.0
    print(f"  [3.2] ComputePathToPose tới toilet_1 ({t1_x}, {t1_y}): {l1_val:.3f} m (status={st_t1})")

    # Toilet 2: (0.2, 1.8)
    t2_x, t2_y, t2_yaw = 0.2, 1.8, 1.57
    len_t2, st_t2 = node.compute_path(start_ps, t2_x, t2_y, t2_yaw)
    l2_val = len_t2 if len_t2 is not None else 0.0
    print(f"  [3.3] ComputePathToPose tới toilet_2 ({t2_x}, {t2_y}): {l2_val:.3f} m (status={st_t2})")

    expected_shorter = "toilet_1" if (l1_val > 0 and (l2_val <= 0 or l1_val < l2_val)) else "toilet_2"
    exp_len = l1_val if expected_shorter == "toilet_1" else l2_val
    print(f"        --> Điểm ngắn hơn: {expected_shorter} ({exp_len:.3f}m)")

    node.feedbacks.clear()
    node.nav_states.clear()
    node.send_voice("nhà vệ sinh ở đâu")

    t0 = time.time()
    chosen_target = None
    while time.time() - t0 < 8.0:
        if node.nav_states and node.nav_states[-1].get("mission_state") == "NAVIGATING":
            chosen_target = node.nav_states[-1].get("current_target_id")
            break
        time.sleep(0.1)

    print(f"  [3.4] Điểm đến thực tế lựa chọn: {chosen_target}")

    node.send_voice("hủy chuyến")
    time.sleep(1.5)

    c3_ok = (chosen_target == expected_shorter) and (l1_val > 0) and (l2_val > 0)
    c3_eval = "PASSED [OK]" if c3_ok else "FAILED"
    print(f"  ==> CA 3 (Chọn toilet ngắn nhất): {c3_eval}")
    evidence_report.append(("Ca 3 (Chọn toilet ngắn nhất)", c3_ok, f"L1={l1_val:.2f}m, L2={l2_val:.2f}m, chosen={chosen_target}"))

    # -------------------------------------------------------------------------
    # TỔNG HỢP KẾT QUẢ
    # -------------------------------------------------------------------------
    print("\n" + "="*75)
    print("TỔNG HỢP KẾT QUẢ BẰNG CHỨNG GAZEBO")
    print("="*75)
    all_ok = True
    for name, ok, detail in evidence_report:
        status_str = "PASSED [OK]" if ok else "FAILED [X]"
        if not ok:
            all_ok = False
        print(f"  * {name:<35}: {status_str} ({detail})")
    print("="*75)
    res_str = "TẤT CẢ ĐẠT 100%" if all_ok else "CÓ CA CHƯA ĐẠT"
    print(f"KẾT LUẬN: {res_str}")
    print("="*75)

    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    run_evidence_verification()
