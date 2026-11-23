# PHASE 8 EVIDENCE SUMMARY — REAL GAZEBO + NAV2 RUNTIME INTEGRATION & CONTRACT LOCK
**Môi trường:** ROS 2 Jazzy (Dev Container), ROS_DOMAIN_ID=43 (Cô lập)  
**Thời gian cập nhật:** 23/11/2026 — Gate 8 Acceptance & Phase 9 Handover  
**Baseline Git:** `eb95457a4652c846ffd8d790921598443db44910` (Working tree modified: velocity watchdog & gate 8 suite)  
**Build Status:** Clean isolated build (`build_tracking`, `install_tracking`, `log_tracking`)  
**Test Suite Tổng:** 124/124 PASS (100% pass rate)  
**Gate 8 Gazebo Suite:** 12/12 PASS (100% pass rate)

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
| **T33** | Clock Pause Steady Time Watchdog | Pause Gazebo world control: `/clock` đóng băng (14.530s), Guard chủ động chuyển sang lệnh zero MỚI trong `dt=0.559s` (`zeroed_on_pause=True`). Khi resume: cửa sổ 0.3s không có observation/cmd mới xác nhận Guard duy trì kẹp zero (`max vx=0.0000 m/s`, không giật lệnh đệm trước pause). Cấp lại observation: xe tiếp tục chuyển động (`guard vx=0.500 m/s`, `odom vx=0.533 m/s`), tuân thủ giới hạn tốc độ `max vx <= 0.50 m/s`. | **PASS** |
| **T15** | Prolonged Loss Lifecycle & Voice Handshake | Ngắt quan sát > 1.20 s. Đo thực tế: FSM vào `ABORTING=True`, `Guard zeroed=True`, `safety_request` gửi sang Voice. Voice gửi `ACK`, gọi `cancel_goal_async` (`Nav2 goal canceled=True`), gửi Terminal `CANCELED`, FSM dọn về `IDLE=True`. | **PASS** |
| **T16** | Unresponsive Voice Watchdog FAULT | Voice không phản hồi khi ngắt quan sát. Đo thực tế: `Safety requests sent=4 (>=2)`, Guard duy trì kẹp `0.0 m/s=True`, FSM chuyển sang `FAULT=True`. | **PASS** |
| **SUCCEEDED** | Nav2 Real Waypoint Completion | Xe di chuyển đến đích thực tế. Đo thực tế: `Nav2 action reached STATUS_SUCCEEDED=True`, Voice gửi terminal SUCCEEDED, `FSM IDLE reached=True`. | **PASS** |
| **T22B** | Guard Crash Without Handler Watchdog (SIGKILL) | Gửi `SIGKILL` vào đúng PID Guard của launch test khi xe đang dẫn khách (goal active, cmd_vel khác 0, odom moving, scan valid). Node độc lập `escort_velocity_watchdog` phát hiện mất lệnh (timeout 0.15s) và chủ động xuất 0 liên tục ở 20 Hz vào Collision Monitor. Đo thực tế: Watchdog zero dt=0.179s, /cmd_vel zero dt=0.211s. Robot dừng hẳn trong `0.687s (<=2.5s)`. Odom cuối: vx=-0.0000 m/s, wz=0.0000 rad/s (198 mẫu advancing stamps). | **PASS** |
| **T22B-Control** | Loss of Guard Without Watchdog (Control Case) | Đo đối chứng: Khi tắt Watchdog, việc kill Guard để lại `/tracking/cmd_vel_watchdog` với đúng 0 bản tin mới, chứng minh downstream Collision Monitor phụ thuộc tuyệt đối vào node Watchdog độc lập để dập 0 chủ động. | **PASS** |
| **T22A** | Guard Graceful Shutdown Watchdog (SIGTERM) | Gửi `SIGTERM` tới PID `escort_velocity_guard` khi xe đang chạy. Signal handler bắt tín hiệu và chủ động phát zero Twist trước khi thoát (first zero dt=1.159s). Đo thực tế: `Measured stop delay=2.206s (<=2.5s)`. Odom cuối: vx=-0.0000 m/s, wz=0.0000 rad/s. | **PASS** |

**Tổng kết Gate 8:** TOTAL=12 | PASSED=12 | FAILED=0 | BLOCKED=0 | Trạng thái: **PASS** (100% Pass Rate)

## 3. Danh Mục File Bằng Chứng

1. `.agent/person_tracking/evidence/phase_8/gate8_real_runtime_evidence.log`: Log vết thực thi chi tiết 8 bước tích hợp thực tế.
2. `.agent/person_tracking/evidence/phase_8/gazebo_nav2_runtime.log`: Toàn bộ stdout/stderr của launch stack Gazebo + Nav2 + Tracking.
3. `.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt`: Biên bản audit topic graph, QoS, publishers và subscribers.
4. `src/person_tracking/test/run_gate8_real_gazebo_nav2.py`: Driver kiểm thử tự động hóa trên ROS graph thật.
5. `src/person_tracking/test/execute_gate8_verification.sh`: Script điều phối launch, wait lifecycle, và thực thi kiểm thử.
