# PHASE 8 EVIDENCE SUMMARY — END-TO-END INTEGRATION & ACCEPTANCE
**Môi trường:** ROS 2 Jazzy (Dev Container)  
**Thời gian:** 23/11/2026  
**Baseline Git:** `6ed2287989b6e43fad60a4f8d03f55412ad500b8`  
**Build Status:** Clean isolated build (`build_tracking`, `install_tracking`, `log_tracking`)  
**Test Suite:** 111/111 PASS (100% pass rate)

---

## 1. Kết Quả Nghiệm Thu Section A (Sửa Lỗi Guard / FSM)

| Mục | Nội Dung Yêu Cầu | Kết Quả Thực Hiện | Trạng Thái |
| :--- | :--- | :--- | :--- |
| **A.1** | Observation Gating | `update_observation` xác thực strict stamp/sequence, session binding, `is_confirmed`, `is_ambiguous`, `depth_valid`. Frame rỗng, lặp stamp, khác source không gia hạn seen time. Không có observation hợp lệ -> xuất strictly zero. | **PASS** |
| **A.2** | Command Revocation & Boundary Stop | Hủy quyền dùng `cmd_vel` cũ khi WAIT/ABORT/FAULT, stale obs, hoặc đổi session. Khi obs hồi phục từ stale, gate mở lại yêu cầu lệnh mới nhận sau thời điểm mở. Điều kiện biên $d \ge 2.5$ m: xe không được yaw/quay khi khoảng cách yêu cầu dừng ($v_x = 0, \omega = 0$). Kiểm tra NaN/Inf trên mọi trường điều khiển -> kẹp 0 an toàn. | **PASS** |
| **A.3** | Closed-loop Yaw Trimming Sign | Kiểm chứng qua `test_closed_loop_rear_camera_yaw_geometry`: Khách lệch PHẢI ảnh ($e_x > 0$) -> trim CW ($\Delta \omega < 0$). Khách lệch TRÁI ảnh ($e_x < 0$) -> trim CCW ($\Delta \omega > 0$). Triệt tiêu offset ở cả 2 phía. Đảo conflict resolution tại mép: Nav2 quay ngược hướng trim -> stop/wait. | **PASS** |
| **A.4** | Steady Clock & ACK/Terminal ID Binding | Sử dụng `Clock(clock_type=ClockType.STEADY_TIME)` cho Guard timer (20 Hz) và FSM heartbeat/watchdog (10 Hz), độc lập hoàn toàn với việc pause `/clock`. Phân biệt confirmed `OUT_OF_FOV` (ở mép + có trong `lost_track_ids` -> ABORT ngay) với suspected loss ở mép (chưa có trong `lost_track_ids` -> WAITING dừng sớm). Bỏ qua ACK/terminal sai ID. Khóa `unconfirmed_terminal` khi timeout sau ACK. | **PASS** |

---

## 2. Kết Quả Nghiệm Thu Section B (Phase 8 End-to-End Integration Matrix)

| Test ID | Nội Dung Kịch Bản | Kết Quả Quan Sát | Trạng Thái |
| :--- | :--- | :--- | :--- |
| **Audit** | Velocity Topic Chain & Sole Publisher | `cmd_vel_nav` -> `cmd_vel_smoothed` -> `/tracking/cmd_vel_guarded` -> `collision_monitor` -> `/cmd_vel`. Xác nhận Collision Monitor là publisher DUY NHẤT trên `/cmd_vel`. Strictly ZERO `/cmd_vel` publishers trong package `person_tracking`. | **PASS** |
| **T09** | Normal Escort Pass-Through | Observation hợp lệ, NAVIGATING, depth verified -> Guard cho phép vận tốc đi qua `/tracking/cmd_vel_guarded` đến `/cmd_vel` ($v_x = 0.35$ m/s, $\omega = 0.05$ rad/s). | **PASS** |
| **T10** | Distance Scaling & Hysteresis Stop | Khoảng cách $> 2.0$ m -> giảm tốc tuyến tính. Khoảng cách $\ge 2.5$ m -> dừng hoàn toàn ($v_x = 0, \omega = 0$), latch dừng. Khi khách quay lại $\le 2.2$ m -> unlatch, xe tiếp tục chạy. | **PASS** |
| **T11** | Close Passenger Safety | Khách ở cự ly gần $< 1.2$ m -> duy trì tốc độ Nav2, không vượt tốc vào khách, không phát lệnh lùi ($v_x \ge 0$). | **PASS** |
| **T12** | Rear Camera Trimming & Conflict | Lệch phải -> trim CW; lệch trái -> trim CCW; lệch mép $> 0.70$ -> giảm 50% tốc độ tiến. Khi Nav2 rẽ ngược hướng giữ khách -> dừng xe (`NAV2_EDGE_TURNING_CONFLICT_STOP`). | **PASS** |
| **T14 / T15** | Loss Grace & Full Abort with Mock Voice | Mất khách ngắn $< 0.40$ s -> grace period duy trì vận tốc theo observation cuối. Mất khách $> 1.20$ s -> chuyển `ABORTING`, Guard lập tức kẹp 0, Mock Voice nhận `safety_request`, ACK và hủy goal Nav2. | **PASS** |
| **T16** | Unresponsive Voice Safety | Mock Voice không phản hồi / mất kết nối -> Guard duy trì vận tốc 0 độc lập, không cho xe trôi theo lệnh Nav2 cũ. | **PASS** |
| **T23** | Collision Monitor Priority over Guard | Trong lúc Guard đang phát $v_x = 0.35$ m/s, bơm chướng ngại vật cự ly $0.40$ m vào laser scan `/scan`. Collision Monitor lập tức ngắt `/cmd_vel` về 0.0 m/s, chứng minh Guard không ghi đè an toàn va chạm. | **PASS** |
| **T22** | Guard Process Termination Watchdog | Kill process `escort_velocity_guard` khi đang chạy -> Collision Monitor `source_timeout` (1.0 s) kích hoạt và xuất 0.0 m/s trên `/cmd_vel`, xe dừng hẳn. | **PASS** |
| **T33** | Clock Pause Monotonic Watchdog | Khi `/clock` mô phỏng bị tạm dừng, watchdog chạy trên `ClockType.STEADY_TIME` vẫn tính timeout theo thời gian thực và kẹp vận tốc về 0. Khi resume không bị giật theo lệnh cũ. | **PASS** |
| **T35** | Mecanum Lateral Stability | Xuyên suốt toàn bộ các trạng thái và kiểm thử, $v_y$ (`linear.y`) luôn luôn bằng $0.0$ strictly. | **PASS** |

---

## 3. Danh Mục File & Bằng Chứng Tạo Mới

1. `src/person_tracking/config/navigation_guard.yaml`: Cấu hình overlay Nav2 chèn Guard vào trước Collision Monitor.
2. `src/person_tracking/launch/gazebo_escort_integration.launch.py`: Kịch bản launch Gazebo headless + Nav2 + Tracking Manager + Velocity Guard.
3. `src/person_tracking/test/test_phase8_gazebo_nav2_integration.py`: Bộ kiểm thử tích hợp 11 ca end-to-end cho Phase 8.
4. `.agent/person_tracking/evidence/phase_8/colcon_build.log`: Log build sạch isolated overlay.
5. `.agent/person_tracking/evidence/phase_8/pytest_results.txt`: Log chi tiết 111 test cases pass.
6. `.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt`: Biên bản audit topic graph và chuỗi điều khiển.
