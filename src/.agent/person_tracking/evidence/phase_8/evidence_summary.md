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

| Test ID | Kịch Bản & Trọng Tâm Đo Đạc | Kết Quả Đo Thực Tế Từ ROS Graph | Trạng Thái |
| :--- | :--- | :--- | :--- |
| **Audit** | Topic Chain & Sole Publisher | Sole publisher trên `/cmd_vel` là `collision_monitor` (1 pub), `ros_gz_bridge` subscribed, 0 publisher từ Guard. `/tracking/cmd_vel_guarded` có sole pub là `escort_velocity_guard`. | **PASS** |
| **Locking** | Khóa khách cử chỉ giơ tay | Nhận diện khách, tích lũy cử chỉ giơ tay -> `locked=True`, `session_id=f24274d0-17c0-43cd-b84f-f3748b8ddc35`, trạng thái `LOCKED_WAIT_DESTINATION`. | **PASS** |
| **T09** | Escort Velocity & Physical Movement | Voice phát `NAVIGATION_STARTED`, gửi Nav2 Goal. Đo thực tế: `Guard max vx=0.500 m/s (>0.05)`, `/cmd_vel max vx=0.500 m/s (>0.05)`, `Odom max vx=0.122 m/s (>0.02)`, `delta_x=0.100 m (>0.10m)`. | **PASS** |
| **T23** | Real Collision Monitor Priority Stop | Bơm vật cản 0.25 m vào `/tracking/scan_test` trong vùng `PolygonStop` (bán kính 0.30 m). Đo thực tế: `Guard output=0.500 m/s (>0.05)`, `/cmd_vel clamped to=0.0000 m/s (<1e-3)`, `odom vx=0.0041 m/s (<0.05)`. | **PASS** |
| **T14** | Brief Observation Loss Grace | Ngắt quan sát 0.18 s (< 0.25 s grace). Đo thực tế: FSM duy trì `NAVIGATING=True`, `Guard vx maintained=0.500 m/s (>0.05)`. | **PASS** |
| **T33** | Clock Pause Monotonic Watchdog | Pause world control Gazebo. Đo thực tế: `Clock frozen=True`, `Guard during pause=0.0000 m/s (<1e-3)`. Khi resume: `Clock resumed=True`, `Max vx after resume=0.500 m/s (<=0.50)`, không giật lệnh cũ. | **PASS** |
| **T15** | Prolonged Loss Lifecycle & Voice Handshake | Ngắt quan sát > 1.20 s. Đo thực tế: FSM vào `ABORTING=True`, `Guard zeroed=True`, `safety_request` gửi sang Voice. Voice gửi `ACK`, gọi `cancel_goal_async` (`Nav2 goal canceled=True`), gửi Terminal `CANCELED`, FSM dọn về `IDLE=True`. | **PASS** |
| **T16** | Unresponsive Voice Watchdog FAULT | Voice không phản hồi khi ngắt quan sát. Đo thực tế: `Safety requests sent=4 (>=2)`, Guard duy trì kẹp `0.0 m/s=True`, FSM chuyển sang `FAULT=True`. | **PASS** |
| **SUCCEEDED** | Nav2 Real Waypoint Completion | Xe di chuyển đến đích thực tế. Đo thực tế: `Nav2 action reached STATUS_SUCCEEDED=True`, Voice gửi terminal SUCCEEDED, `FSM IDLE reached=True`. | **PASS** |
| **T22** | Guard Termination Base Watchdog | Gửi `SIGTERM` tới process PID của `escort_velocity_guard` khi xe đang chạy. Đo thực tế: `Measured stop delay=0.401s (<=2.5s)`. Collision Monitor `source_timeout` (1.0s) cắt hoàn toàn `/cmd_vel` về 0, xe dừng hẳn. | **PASS** |

**Tổng kết:** TOTAL=10 | PASSED=10 | FAILED=0 | BLOCKED=0 | EXIT CODE=0

## 3. Danh Mục File Bằng Chứng

1. `.agent/person_tracking/evidence/phase_8/gate8_real_runtime_evidence.log`: Log vết thực thi chi tiết 8 bước tích hợp thực tế.
2. `.agent/person_tracking/evidence/phase_8/gazebo_nav2_runtime.log`: Toàn bộ stdout/stderr của launch stack Gazebo + Nav2 + Tracking.
3. `.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt`: Biên bản audit topic graph, QoS, publishers và subscribers.
4. `src/person_tracking/test/run_gate8_real_gazebo_nav2.py`: Driver kiểm thử tự động hóa trên ROS graph thật.
5. `src/person_tracking/test/execute_gate8_verification.sh`: Script điều phối launch, wait lifecycle, và thực thi kiểm thử.
