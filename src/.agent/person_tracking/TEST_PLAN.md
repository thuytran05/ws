# PERSON TRACKING — KẾ HOẠCH KIỂM THỬ TOÀN DIỆN (COMPREHENSIVE TEST PLAN)

**Dự án:** Robot dẫn đường hành khách sân bay (Airport Escort Robot)  
**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.3 — 24/11/2026 (Chuẩn bị Nghiệm thu Phase 11)  
**Mục tiêu:** Ánh xạ toàn bộ Quyết định D01–D12, kiểm thử T01–T25 từ `WORKFLOW.md` và 10 ca kiểm thử an toàn mở rộng (T26–T35), phân loại rõ cấp độ Unit/Logic, ROS Runtime và Gazebo Integration.

---

## 1. Ma trận Kiểm thử Tổng thể (Test Matrix T01–T35)

| Mã test | Tên ca kiểm thử | Phase kiểm thử | Cấp độ kiểm thử | Nguồn dữ liệu | Tiêu chí Pass / Yêu cầu nghiệm thu | Bằng chứng & Log | Trạng thái hiện tại |
|---|---|---|---|---|---|---|---|
| **T01A** | Khung hình không có người (Adapter & Source Check) | Phase 2 | Unit / Logic | Video / Synthetic | Khởi tạo ObservationFrame rỗng chuẩn; data_quality VALID_RGBD; tuyệt đối không phát /cmd_vel; chuẩn bị bàn giao cho Phase 3. | `phase_2/launch_smoke_test.log` | **PASS** |
| **T01B** | Khung hình không có người (FSM & Guard Logic) | Phase 6/7 | ROS Runtime | Video / Synthetic | FSM giữ `IDLE`; xuất vận tốc 0 liên tục (20 Hz); từ chối nhận điểm đến khi không có người. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T02** | 1 người giơ tay < 1.5s | Phase 4 | Unit / Logic | Video / Replay | Không khóa ID; FSM giữ `IDLE`; hạ tay bộ đếm reset về 0. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T03** | 1 người giơ tay $\ge$ 1.5s | Phase 4 | Unit / Logic | Video / Replay | Khóa đúng ID; cấp `session_id`; hạ tay vẫn giữ khóa; robot đứng yên. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T04** | 2 người cùng giơ tay, khác depth | Phase 4 | Unit / Logic | Synthetic / Bag | Khóa người có depth nhỏ nhất (gần camera nhất); bỏ qua người ở xa. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T05** | 2 người giơ tay, thiếu depth, kích thước gần bằng nhau | Phase 4 | Unit / Logic | Synthetic / Video | Không khóa tùy tiện; báo `AMBIGUOUS_GESTURE`; giữ `IDLE` chờ xác định rõ. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T06** | Người khác cắt ngang người đang khóa | Phase 3/11 | Unit / Logic | Video Replay / Synthetic | ByteTrack gán track ID riêng; phát hiện mơ hồ (is_ambiguous = True) khi IoU > 0.35. Code sẵn sàng, chờ video người thật từ user ở Phase 11 để nghiệm thu hình ảnh. | `phase_3/multi_person_crossing_trace.log` | **PARTIAL / CODE_READY** |
| **T07** | Voice gửi đích khi chưa khóa | Phase 6 | ROS Runtime | Mock Voice | Bị chặn ở giao diện; Voice nhận cờ `locked == false` và từ chối. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T08** | Voice gửi đích sau khi đã khóa | Phase 6 | ROS Runtime | Mock Voice | Hợp lệ; Nav2 goal được chấp nhận; FSM chuyển sang `NAVIGATING`. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T09** | Khách ở giữa ảnh, cự ly chuẩn (1.5m) | Phase 7/8 | Gazebo Integration | Fake Obs + Nav2 | Cho phép Nav2 đi đúng tốc độ tính toán (trong giới hạn an toàn max vx <= 0.50 m/s). | `phase_7/t09_normal_escort.log`, `phase_8/evidence_summary.md` | **PASS** |
| **T10** | Khách tụt lại xa (> 2.0m) | Phase 7 | Unit / Logic | Fake Obs + Nav2 | Giảm dần trần tốc độ `linear.x`; nếu > 2.5m thì dừng chờ; có hysteresis; không tăng tốc vượt Nav2. | `phase_7/t10_distance_clamp.log` | **PASS** |
| **T11** | Khách tiến lại gần robot (< 1.2m) | Phase 7 | Unit / Logic | Fake Obs + Nav2 | Giữ nguyên tốc độ Nav2; tuyệt đối không tự lùi xe hoặc tăng tốc dãn cách. | `phase_7/t11_close_passenger.log` | **PASS** |
| **T12** | Khách lệch sang mép ảnh | Phase 7 | Unit / Logic | Fake Obs + Nav2 | Bổ sung `angular.z` nhẹ có giới hạn; dấu quay đúng (CCW/CW); bão hòa 0.20 rad/s; giảm tốc mép 50%. | `phase_7/t12_yaw_trimming.log` | **PASS** |
| **T13** | Mất frame ngắn (< 0.25s) | Phase 6 | ROS Runtime | Fake Obs Drop | Duy trì ID; FSM giữ `NAVIGATING`; không đổi phiên; không dừng giật cục. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T14** | Mất ảnh quá `visibility_stop_sec` (0.4s) | Phase 7/10 | ROS Runtime | Fake Obs Cut | Guard ép vận tốc về 0 ngay lập tức (thực đo 0.382s <= 0.45s); FSM chuyển `WAITING_FOR_PASSENGER`. | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T15** | Mất ảnh quá `visibility_abort_sec` (1.2s) | Phase 6/10 | ROS Runtime | Fake Obs Cut | Gửi yêu cầu `ABORT` sang Voice (thực đo 1.208s <= 1.30s); chốt STOP vĩnh viễn; phiên không tự hồi sinh. | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T16** | Voice không phản hồi lệnh cancel / ACK delay | Phase 7/10 | ROS Runtime | Mock Fault | Guard duy trì vận tốc 0 độc lập liên tục 20 Hz khi thiếu ACK/terminal từ Voice; xe không tự ý di chuyển. | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T17** | Voice báo `SUCCEEDED` đúng phiên | Phase 6/8 | Gazebo Integration | Mock Voice / Real Nav2 | Chuyển sang `ARRIVED`, xóa `selected_id`, xóa phiên, đưa Guard về 0, về `IDLE`. | `phase_8/evidence_summary.md` | **PASS** |
| **T18** | Nhận `SUCCEEDED` của phiên cũ đến muộn / Re-route | Phase 6/8 | ROS Runtime | Mock Voice Out-of-order | Bỏ qua bản tin do sai `session_id`; không làm ảnh hưởng phiên đang chạy; cho phép đổi đích sạch sẽ. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T19** | Depth chứa NaN / 0.0 / ngoài dải | Phase 5 | Unit / Logic | Synthetic Fixture | Bộ lọc MAD loại bỏ điểm lỗi; đánh dấu `depth_valid = false`; không dùng số metric giả. | `phase_10/pytest_results.txt` | **PASS** |
| **T20** | Mâu thuẫn CameraInfo / TF | Phase 5 | Unit / Logic | Injected TF Error | Phát hiện thiếu TF, frame ID mismatch, hoặc CameraInfo không hợp lệ; hạ cờ `is_metric_valid = false`; chỉ xuất 2D pixel offset. | `phase_5/phase_5_offline_verified.log` | **PASS** |
| **T21** | Lệnh `cmd_vel_smoothed` quá cũ (> 0.2s) | Phase 7/10 | ROS Runtime | Nav2 Drop | Watchdog phát hiện cmd stale sau 0.177s; Guard xuất vận tốc 0; không giữ hay phát lại lệnh cũ. | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T22A** | Guard Graceful Shutdown Watchdog | Phase 8 | Gazebo Integration | SIGTERM Kill | Signal handler bắt SIGTERM, chủ động xuất 5 bản tin zero Twist trước khi thoát. Robot dừng hẳn trong <=2.5s. | `phase_8/evidence_summary.md` | **PASS** |
| **T22B** | Guard Crash Without Handler Watchdog | Phase 8/10 | Gazebo Integration | SIGKILL Kill | Node độc lập `escort_velocity_watchdog` phát hiện mất lệnh Guard (timeout 0.15s), phát zero ở 20 Hz vào Collision Monitor (delay 0.200s). /cmd_vel zero delay 0.200s. Robot dừng hẳn trên /odom trong 0.851s (<=2.5s). | `phase_10/t22b_t23_gazebo_recheck.log` | **PASS** |
| **T23** | Collision Monitor nằm sau Guard | Phase 8/10 | Gazebo Integration | Integration Launch | Vật cản laser 0.25m xuất hiện -> Collision Monitor dập /cmd_vel về 0 trong 0.050s, robot dừng hẳn trên /odom trong 0.301s (<=2.5s). | `phase_10/t22b_t23_gazebo_recheck.log` | **PASS** |
| **T24** | Chạy kiểm thử trên Gazebo hiện có | Phase 8/9 | Gazebo Integration | Gazebo + Fake Perception | Robot dừng/quay/lách đúng quy tắc khi nhận tín hiệu quan sát giả lập; không sửa URDF. | `phase_8/evidence_summary.md`, `phase_9/evidence` | **PASS** |
| **T25** | Kiểm thử Độ bền & Tin cậy Hệ thống | Phase 10 | Unit & ROS Runtime | Pytest / Real Nodes | Phân biệt rõ: 50 chu kỳ stress logic Python (+0.00 MB leak) và 10 chu kỳ stress ROS topic runtime trên node thật (trung bình 2072.7 ms/chu kỳ). Cả hai đạt 100% PASS. | `phase_10/t25_stress_reliability.log`, `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T26** | Thiếu depth hoặc alignment chưa kiểm chứng | Phase 5 | Unit / Logic | RGB-only Video | Khi UNVERIFIED_ALIGNMENT: ép `depth_valid = false`, không xuất số đo metric tin cậy để chọn nearest, cấm thay 0 cho depth thiếu. | `phase_5/phase_5_offline_verified.log` | **PASS** |
| **T27** | ByteTrack giữ ID nhưng vị trí không gian mơ hồ | Phase 3 | Unit / Logic | Synthetic Crossing | Cờ is_ambiguous = True bật khi IoU > 0.35 và tự giải phóng khi tách rời; truyền payload chuẩn sang FSM & Guard. | `phase_3/multi_person_crossing_trace.log` | **PASS** |
| **T28** | Bản tin TRANSIENT_LOCAL cũ khi node mới bật | Phase 6 | ROS Runtime | Node Restart | So sánh thời gian và sequence; bỏ qua bản tin cũ đã hết hạn (`freshness_ttl`). | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T29** | Voice ACK cancel nhưng Goal chưa terminal | Phase 6/10 | ROS Runtime | Mock Voice In-flight | Guard vẫn giữ dừng 0; FSM chưa đóng phiên cho tới khi nhận xác nhận terminal. | `phase_6/phase_6_fsm_mock_voice.log`, `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T30** | Khóa khách mới khi Goal cũ còn unresolved | Phase 6 | ROS Runtime | Mock Nav Conflict | Manager từ chối giơ tay khóa mới; giữ FSM chờ Goal cũ dọn dẹp xong hoàn toàn. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T31** | Khách xuất hiện lại trước abort vs sau abort | Phase 6 | ROS Runtime | Fake Obs Toggle | Trước 1.2s: Hồi phục `NAVIGATING`. Sau 1.2s: Chốt hủy, ảnh về vẫn giữ dừng, bắt buộc khóa mới. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T32** | Từng node (Guard/Manager/Voice) bị restart giữa phiên | Phase 8/10 | ROS Runtime | Node Restart | Đã kiểm tra riêng cho `VelocityGuardNode`: Khởi động lại sạch sẽ từ zero; motion_gate đóng; không có hành vi tự chạy lại hoặc replay lệnh cache. | `phase_8/evidence_summary.md`, `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| **T33** | Mô phỏng bị pause hoặc `/clock` bị nhảy | Phase 8 | Gazebo Integration | Clock Pause Gazebo | Khi /clock đứng yên, Guard chủ động chuyển sang lệnh zero MỚI. Cửa sổ 0.3s post-resume không có obs/cmd mới duy trì 0. Cấp lại input xe chạy tiếp tuân thủ max vx <= 0.50 m/s. | `phase_8/evidence_summary.md` | **PASS** |
| **T34** | Guard crash và nguồn phát lạ xuất hiện | Phase 7/8 | ROS Runtime | Multi-publisher Test | Collision Monitor và Base watchdog can thiệp; xác minh cơ chế cô lập publisher. | `phase_8/topic_graph_audit.txt` | **PASS** |
| **T35** | Nav2 `linear.y` pass-through vs Tracking lateral correction | Phase 7/10 | ROS Runtime | Mecanum Motion Test | Guard duy trì linear.y = 0.0 tuyệt đối, ngăn ngừa can thiệp chuyển động ngang tự phát. | `phase_7/t35_lateral_motion_separation.log`, `phase_10/gate9_odom_peak_trace.csv` | **PASS** |

---

## 2. Quy trình Thực thi Kiểm thử Tự động hóa

1. **Khung kiểm thử:** Sử dụng `pytest` kết hợp `launch_testing` trong ROS 2 Jazzy.
2. **Quy định Timeout:** Mọi bài test đều bắt buộc phải khai báo tham số `timeout` rõ ràng (từ 4.0s đến 30.0s). Bất kỳ test nào vượt quá timeout sẽ tự động fail, không được phép treo luồng thực thi.
3. **Phân lập môi trường test:**
   - Đặt biến môi trường `ROS_DOMAIN_ID` riêng biệt khi chạy test (`ROS_DOMAIN_ID=43` cho ROS topic benchmark và runtime tests; `ROS_DOMAIN_ID=44` cho Gazebo integration tests).
   - Ghép nối hệ thống chính (Voice + Tracking + Nav2 + Robot thật) sử dụng `ROS_DOMAIN_ID=0`.
   - Sử dụng topic namespace riêng biệt khi chạy song song.
4. **Lưu trữ bằng chứng (Evidence Logging):**
   - Toàn bộ kết quả test của từng Phase $N$ bắt buộc phải được ghi lại thành file log thô trong `.agent/person_tracking/evidence/phase_N/`.

---

## 3. Ma trận 26 Ca Kiểm thử Trọng tâm Gate 10 (Theo Cấp độ Kiểm chứng)

Danh sách 26 ca kiểm thử trọng tâm phân loại rõ ràng:

### Nhóm 1: Unit / Logic Tests (Thuật toán, Cử chỉ, Hình học và Lọc nhiễu) — 10 Ca
1. **T01A:** Khởi tạo `ObservationFrame` rỗng chuẩn, `data_quality = VALID_RGBD`, không phát `cmd_vel`.
2. **T02:** Cử chỉ giơ tay $< 1.5\text{ s} \to$ không khóa ID, FSM giữ `IDLE`, hạ tay reset bộ đếm.
3. **T03:** Cử chỉ giơ tay $\ge 1.5\text{ s} \to$ khóa đúng ID, cấp `session_id`, hạ tay vẫn giữ khóa.
4. **T04:** Hai người cùng giơ tay $\to$ ưu tiên khóa người có depth gần nhất ($1.4\text{ m}$ vs $2.5\text{ m}$).
5. **T05:** Hai người giơ tay thiếu depth, kích thước tương đương $\to$ báo `AMBIGUOUS_GESTURE`, không khóa tùy tiện.
6. **T06 (PARTIAL / CODE_READY):** Người cắt ngang $\to$ ByteTrack gán track riêng, phát hiện mơ hồ (`is_ambiguous = True`) khi $\text{IoU} > 0.35$.
7. **T10:** Khách tụt lại xa ($> 2.0\text{ m}$ và $> 2.5\text{ m}$) $\to$ giảm dần trần $v_x$, dừng chờ có hysteresis.
8. **T11:** Khách tiến lại gần ($< 1.2\text{ m}$) $\to$ giữ nguyên tốc độ Nav2, cấm tự lùi xe.
9. **T12:** Khách lệch sang mép ảnh $\to$ bù $\omega_z$ nhẹ hướng tâm, bão hòa $0.20\text{ rad/s}$, giảm trần mép 50%.
10. **T19 & T20:** Lọc nhiễu Depth bằng trung vị + MAD, xử lý NaN/0.0/mất TF chuẩn xác.

### Nhóm 2: ROS Runtime Tests (Đồ thị Node, Topic DDS, FSM & Watchdog) — 10 Ca
11. **T01B:** Khung hình không người trên FSM & Guard $\to$ FSM giữ `IDLE`, kẹp zero liên tục 20 Hz.
12. **T07 & T08:** Voice gửi đích khi chưa khóa (từ chối) vs sau khi đã khóa (chấp nhận, `NAVIGATING`).
13. **T13:** Mất frame ngắn ($< 0.25\text{ s}$) $\to$ duy trì track ID và FSM `NAVIGATING`, không dừng giật cục.
14. **T14:** Mất ảnh quá $0.4\text{ s}$ $\to$ Guard dập zero trong $0.382\text{ s} \le 0.45\text{ s}$, FSM sang `WAITING_FOR_PASSENGER`.
15. **T15:** Mất ảnh quá $1.2\text{ s}$ $\to$ Manager phát `safety_request` abort tại $1.208\text{ s} \le 1.30\text{ s}$, dừng vĩnh viễn phiên.
16. **T16:** Voice trễ phản hồi ACK $\to$ Guard duy trì độc lập chuỗi dập zero 20 Hz, không bị trôi xe khi thiếu ACK/terminal.
17. **T18:** Đổi đích giữa đường (Re-route) hoặc nhận tin trễ $\to$ đối chiếu đúng 5 định danh, reset motion gate sạch sẽ.
18. **T21:** Lệnh `cmd_vel_smoothed` quá cũ ($> 0.2\text{ s}$) $\to$ Guard kẹp zero tại $0.177\text{ s}$, không phát lại cache.
19. **T25 (ROS Topic Runtime):** Stress test 10 chu kỳ qua topic ROS trên node thật $\to$ 10/10 PASS, trung bình 2072.7 ms/chu kỳ.
20. **T35:** Phân lập chuyển động ngang Mecanum $\to$ Guard kẹp $v_y = 0.0000\text{ m/s}$ tuyệt đối.

### Nhóm 3: Gazebo Integration Tests (Mô phỏng Đầy đủ Vật lý & Nav2 Thật) — 6 Ca
21. **T09:** Dẫn khách cự ly chuẩn $1.5\text{ m}$ trên Gazebo $\to$ Nav2 chạy trơn tru trong giới hạn $v_x \le 0.50\text{ m/s}$.
22. **T17:** Hoàn thành lộ trình tới đích $\to$ Nav2 báo `SUCCEEDED`, Voice gửi terminal, Tracking về `IDLE` sạch sẽ.
23. **T22A:** Guard nhận `SIGTERM` $\to$ graceful shutdown xuất 5 bản tin zero, robot dừng trong $2.206\text{ s} \le 2.5\text{ s}$.
24. **T22B:** Guard bị `SIGKILL` crash đột ngột $\to$ Watchdog độc lập phát zero sau $0.200\text{ s}$, xe dừng trên `/odom` trong $0.851\text{ s} \le 2.5\text{ s}$.
25. **T23:** Chướng ngại vật laser $0.25\text{ m}$ xuất hiện $\to$ Collision Monitor dập `/cmd_vel` sau $0.050\text{ s}$, xe dừng trong $0.301\text{ s} \le 2.5\text{ s}$.
26. **T33:** Mô phỏng `/clock` bị pause hoặc nhảy thời gian $\to$ Guard phát zero mới tức thời, không replay lệnh cache cũ khi resume.

---

## 4. Kế hoạch Nghiệm thu Phần cứng Thực tế Phase 11 (Hardware Acceptance)

- **Trạng thái hiện tại:** **SOFTWARE_READY / HARDWARE_PENDING** (Ca T06 duy trì **PARTIAL / CODE_READY**).
- **Tài liệu hướng dẫn vận hành chi tiết:** Xem tại [.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md](file:///home/thuytran/ws/src/.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md).
- **Bộ công cụ chẩn đoán & nghiệm thu đã sẵn sàng:**
  1. Bộ kiểm thử smoke test thông luồng phần mềm Gazebo: `src/person_tracking/test/verify_phase11_software_smoke.py` và runner `src/person_tracking/test/execute_phase11_software_smoke.sh` (Đã chạy đạt 3/3 PASS, exit code 0).
  2. Kiểm tra camera Astra tại chỗ: `src/person_tracking/test/check_astra_hardware_topics.py`.
  3. Đo tải CPU, RSS, VRAM, FPS theo đúng PID thật: `src/person_tracking/test/profile_detector_pid.py`.
- **Kết quả Pre-Check Phần mềm Smoke Test (Gazebo Simulation, 24/11/2026):**
  * Ca 1 (Chặn dispatch trước khóa): **PASS** (`dispatched=0, nav_started=0, rejected=True, robot_still=True`).
  * Ca 2 (Khóa hợp lệ & Xe di chuyển): **PASS** (`dispatched=1, has_5_ids=True, dist_moved=0.054m, max_vx=0.076m/s`).
  * Ca 3 (Mất khách & Hủy goal Nav2): **PASS** (`dt_zero=0.442s, dt_abort=1.225s, terminal_canceled=True, matching_goal=True, c3_stopped=True, fsm_idle=True`).
  * Log bằng chứng: [.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log).
- **Phân định Trách nhiệm Nghiệm thu:**
  * **Phần mềm (Software Readiness):** 100% ĐẠT yêu cầu bàn giao thông luồng, quản lý phiên và ngắt an toàn.
  * **Phần cứng (Hardware Acceptance Pending):** Bắt buộc người dùng có mặt tại hiện trường để cắm thiết bị, kiểm tra stream Astra, dán mốc đo cự ly, xác nhận watchdog đế xe, chạy video thật ca T06 và đo vận tốc vật lý độc lập.
- **Điều kiện Gate 11:** Chỉ chuyển sang **DONE / HARDWARE_VALIDATED** khi có đầy đủ log nghiệm thu đo đạc thật trên camera Orbbec Astra Pro và robot Mecanum vật lý theo bộ tiêu chí C-01 đến C-10.
