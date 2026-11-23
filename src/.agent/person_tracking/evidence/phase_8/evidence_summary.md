# PHASE 8 EVIDENCE SUMMARY — REAL GAZEBO + NAV2 RUNTIME INTEGRATION & CONTRACT LOCK
**Môi trường:** ROS 2 Jazzy (Dev Container), ROS_DOMAIN_ID=43 (Cô lập)  
**Thời gian cập nhật:** 23/11/2026 — Gate 8 Acceptance & Phase 9 Handover  
**Baseline Git:** `53eed7e10795cab5dc8d62f654e1a65f73d5cad7`  
**Build Status:** Clean isolated build (`build_tracking`, `install_tracking`, `log_tracking`)  
**Test Suite Tổng:** 117/117 PASS (100% pass rate)

---

## 1. Kết Quả Nghiệm Thu Section A (Kiểm Tra & Chuẩn Hóa Hợp Đồng Phía Tracking)

| Mục | Nội Dung Yêu Cầu | Kết Quả Thực Hiện | Trạng Thái |
| :--- | :--- | :--- | :--- |
| **A.1** | Session State Identifiers & Command Revocation | Bổ sung `source_session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid` vào `/tracking/session_state`. Guard đối chiếu khớp phiên/nguồn/track và hủy `cmd_vel` đã cache khi đổi goal (kể cả cùng phiên). Thêm test frame cùng track ID từ nguồn khác -> 0; goal đổi khi NAVIGATING -> xóa cache cmd; heartbeat cũ sau restart -> từ chối. | **PASS** |
| **A.2** | Đồng Bộ Schema với Mã Nguồn | `START` bắt buộc liên kết với đúng phiên đã khóa (`is_terminal: false`). `ACK` là non-terminal, khớp `session_id`/`goal_attempt_id`/`request_id`. `ACK` trùng không kéo dài hạn chót terminal (3.0s). `Terminal` chỉ dọn session khi khớp goal đang liên kết. Restart khi còn goal cũ: gán `unconfirmed_terminal = true`, chặn khóa khách mới cho đến khi có terminal. | **PASS** |
| **A.3** | Khớp & Chuẩn Hóa Tài Liệu Giao Diện | Khớp schema, ví dụ payload, QoS giữa `INTERFACES.md` và `VOICE_INTEGRATION_REQUEST.md`. Quy định rõ `request_id` sinh khi vào `ABORTING`, giữ nguyên qua retry (tối đa 3 lần sau mỗi 200 ms). Sửa quy tắc chỉnh yaw camera sau ADR-10: Khách lệch PHẢI ảnh ($e_x > 0$) -> quay theo chiều kim đồng hồ (CW, $\Delta \omega < 0$); Khách lệch TRÁI ảnh ($e_x < 0$) -> quay ngược chiều kim đồng hồ (CCW, $\Delta \omega > 0$). | **PASS** |

---

## 2. Kết Quả Nghiệm Thu Section B (Bằng Chứng Gate 8 trên ROS 2 Jazzy + Gazebo Thật)

Chạy thực nghiệm trên `gazebo_escort_integration.launch.py` với Gazebo headless (`gz sim 8.15.0`) + Nav2 stack thật (`dwb_core`, `costmap_2d`, `bt_navigator`) + `nav2_collision_monitor` + `escort_velocity_guard` + `escort_tracking_manager` trên `ROS_DOMAIN_ID=43`:

| Test ID | Nội Dung Kịch Bản | Kết Quả Quan Sát Thực Tế | Trạng Thái |
| :--- | :--- | :--- | :--- |
| **Audit** | Topic Chain & Sole Publisher | Chuỗi: Nav2 Controller -> `/cmd_vel_smoothed` -> Guard -> `/tracking/cmd_vel_guarded` -> Collision Monitor -> `/cmd_vel` -> Gazebo bridge (`ros_gz_bridge` chỉ subscribe `]`). Xác nhận Collision Monitor là publisher DUY NHẤT trên `/cmd_vel`. Strictly ZERO `/cmd_vel` publishers trong package `person_tracking`. | **PASS** |
| **T09** | Normal Escort Pass-Through | Giơ tay khóa khách thành công (`session_id`), Voice gửi `NAVIGATION_STARTED`, gửi Goal thật sang Nav2 `NavigateToPose`. Nav2 Action Goal Accepted! Guard pass vận tốc và xe tiến hành dẫn đường an toàn. | **PASS** |
| **T23** | Real Collision Monitor Priority Stop | Khi robot đang di chuyển, bơm vật cản cự ly $0.30$ m vào topic test cô lập `/tracking/scan_test` (nằm trong `PolygonStop` circle bán kính $0.45$ m). Collision Monitor lập tức ép `/cmd_vel` về 0.0 m/s dù Guard đang phát lệnh di chuyển, `/odom` xác nhận xe dừng. | **PASS** |
| **T22** | Guard Process Termination Watchdog | Kill process `escort_velocity_guard` khi xe đang chạy. Collision Monitor kích hoạt cơ chế an toàn cấp dưới `source_timeout` (1.0 s) ngắt toàn bộ vận tốc trên `/cmd_vel` về 0.0 m/s, robot dừng hẳn bền vững. | **PASS** |
| **T33** | Clock Pause Monotonic Watchdog | Watchdog của Guard chạy trên `ClockType.STEADY_TIME`. Khi `/clock` mô phỏng bị đóng băng/pause, watchdog đơn điệu vẫn tính tuổi bản tin (0.20s/1.0s) và kẹp 0. Khi resume `/clock`, xe không bị giật bởi lệnh cũ. | **PASS** |
| **T14/T15/T16** | Mất Quan Sát & Bắt Tay Mock Voice | Ngắt luồng quan sát: Mất $< 0.40$ s (grace period) -> xe tiếp tục; Mất $> 1.20$ s -> TrackingManager vào `ABORTING`, Guard kẹp 0, phát `/tracking/safety_request` (ABORT). Mock Voice nhận yêu cầu, gửi ACK `CANCEL_ACKNOWLEDGED`, gọi `cancel_goal_async` trên Nav2. Trong lúc có ACK nhưng chưa có terminal: xe duy trì dừng 0 và session vẫn khóa. Khi Nav2 hoàn tất hủy, Voice gửi Terminal `CANCELED`, session dọn dẹp sạch về `IDLE`. | **PASS** |

---

## 3. Danh Mục File Bằng Chứng

1. `.agent/person_tracking/evidence/phase_8/gate8_real_runtime_evidence.log`: Log vết thực thi chi tiết 8 bước tích hợp thực tế.
2. `.agent/person_tracking/evidence/phase_8/gazebo_nav2_runtime.log`: Toàn bộ stdout/stderr của launch stack Gazebo + Nav2 + Tracking.
3. `.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt`: Biên bản audit topic graph, QoS, publishers và subscribers.
4. `src/person_tracking/test/run_gate8_real_gazebo_nav2.py`: Driver kiểm thử tự động hóa trên ROS graph thật.
5. `src/person_tracking/test/execute_gate8_verification.sh`: Script điều phối launch, wait lifecycle, và thực thi kiểm thử.
