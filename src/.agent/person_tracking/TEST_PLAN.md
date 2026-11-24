# PERSON TRACKING — KẾ HOẠCH KIỂM THỬ TOÀN DIỆN (COMPREHENSIVE TEST PLAN)

**Dự án:** Robot dẫn đường hành khách sân bay (Airport Escort Robot)  
**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.0 — 23/11/2026 (Phase 1)  
**Mục tiêu:** Ánh xạ toàn bộ Quyết định D01–D12, kiểm thử T01–T25 từ `WORKFLOW.md` và 10 ca kiểm thử an toàn mở rộng (T26–T35).

---

## 1. Ma trận Kiểm thử Tổng thể (Test Matrix T01–T35)

| Mã test | Tên ca kiểm thử | Phase kiểm thử | Nguồn dữ liệu | Tiêu chí Pass / Yêu cầu nghiệm thu | Bằng chứng & Log | Trạng thái hiện tại |
|---|---|---|---|---|---|---|
| **T01A** | Khung hình không có người (Adapter & Source Check) | Phase 2 | Video / Synthetic | Khởi tạo ObservationFrame rỗng chuẩn; data_quality VALID_RGBD; tuyệt đối không phát /cmd_vel; chuẩn bị bàn giao cho Phase 3. | `phase_2/launch_smoke_test.log` | **PASS** |
| **T01B** | Khung hình không có người (FSM & Guard Logic) | Phase 6/7 | Video / Synthetic | FSM giữ `IDLE`; xuất vận tốc 0 liên tục (20 Hz); từ chối nhận điểm đến khi không có người. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS (FSM side)** |
| **T02** | 1 người giơ tay < 1.5s | Phase 4 | Video / Replay | Không khóa ID; FSM giữ `IDLE`; hạ tay bộ đếm reset về 0. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T03** | 1 người giơ tay $\ge$ 1.5s | Phase 4 | Video / Replay | Khóa đúng ID; cấp `session_id`; hạ tay vẫn giữ khóa; robot đứng yên. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T04** | 2 người cùng giơ tay, khác depth | Phase 4 | Synthetic / Bag | Khóa người có depth nhỏ nhất (gần camera nhất); bỏ qua người ở xa. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T05** | 2 người giơ tay, thiếu depth, kích thước gần bằng nhau | Phase 4 | Synthetic / Video | Không khóa tùy tiện; báo `AMBIGUOUS_GESTURE`; giữ `IDLE` chờ xác định rõ. | `phase_4/gesture_locking_verified.log` | **PASS** |
| **T06** | Người khác cắt ngang người đang khóa | Phase 3 | Video Replay / Synthetic | ByteTrack gán track ID riêng; phát hiện mơ hồ (is_ambiguous = True) khi IoU > 0.35. Code sẵn sàng, chờ video người thật từ user để nghiệm thu hình ảnh. | `phase_3/multi_person_crossing_trace.log` | **PARTIAL / CODE_READY** |
| **T07** | Voice gửi đích khi chưa khóa | Phase 6 | Mock Voice | Bị chặn ở giao diện; Voice nhận cờ `locked == false` và từ chối. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T08** | Voice gửi đích sau khi đã khóa | Phase 6 | Mock Voice | Hợp lệ; Nav2 goal được chấp nhận; FSM chuyển sang `NAVIGATING`. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T09** | Khách ở giữa ảnh, cự ly chuẩn (1.5m) | Phase 7 | Fake Obs + Nav2 | Cho phép Nav2 đi đúng tốc độ tính toán (trong giới hạn an toàn). | `phase_7/t09_normal_escort.log` | **PASS** |
| **T10** | Khách tụt lại xa (> 2.0m) | Phase 7 | Fake Obs + Nav2 | Giảm dần trần tốc độ `linear.x`; nếu > 2.5m thì dừng chờ; có hysteresis; không tăng tốc vượt Nav2. | `phase_7/t10_distance_clamp.log` | **PASS** |
| **T11** | Khách tiến lại gần robot (< 1.2m) | Phase 7 | Fake Obs + Nav2 | Giữ nguyên tốc độ Nav2; tuyệt đối không tự lùi xe hoặc tăng tốc dãn cách. | `phase_7/t11_close_passenger.log` | **PASS** |
| **T12** | Khách lệch sang mép ảnh | Phase 7 | Fake Obs + Nav2 | Bổ sung `angular.z` nhẹ có giới hạn; dấu quay đúng (CCW/CW); bão hòa 0.20 rad/s; giảm tốc mép 50%. | `phase_7/t12_yaw_trimming.log` | **PASS** |
| **T13** | Mất frame ngắn (< 0.25s) | Phase 6 | Fake Obs Drop | Duy trì ID; FSM giữ `NAVIGATING`; không đổi phiên; không dừng giật cục. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T14** | Mất ảnh quá `visibility_stop_sec` (0.4s) | Phase 7 | Fake Obs Cut | Guard ép vận tốc về 0 ngay lập tức; FSM chuyển `WAITING_FOR_PASSENGER`. | `phase_7/t14_visibility_stop.log` | **PASS** |
| **T15** | Mất ảnh quá `visibility_abort_sec` (1.2s) | Phase 6 | Fake Obs Cut | Gửi yêu cầu `ABORT` sang Voice; chốt STOP vĩnh viễn; phiên không tự hồi sinh. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T16** | Voice không phản hồi lệnh cancel | Phase 7 | Mock Fault | Guard vẫn duy trì vận tốc 0 độc lập liên tục; không bị ảnh hưởng bởi độ trễ Voice. | `phase_7/t16_voice_unresponsive.log` | **PASS** |
| **T17** | Voice báo `SUCCEEDED` đúng phiên | Phase 6 | Mock Voice | Chuyển sang `ARRIVED`, xóa `selected_id`, xóa phiên, đưa Guard về 0, về `IDLE`. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T18** | Nhận `SUCCEEDED` của phiên cũ đến muộn | Phase 6 | Mock Voice Out-of-order | Bỏ qua bản tin do sai `session_id`; không làm ảnh hưởng phiên đang chạy. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T19** | Depth chứa NaN / 0.0 / ngoài dải | Phase 5 | Synthetic Fixture | Bộ lọc MAD loại bỏ điểm lỗi; đánh dấu `depth_valid = false`; không dùng số metric giả. | `phase_10/pytest_results.txt` | **PASS** |
| **T20** | Mâu thuẫn CameraInfo / TF | Phase 5 | Injected TF Error | Phát hiện thiếu TF, frame ID mismatch, hoặc CameraInfo không hợp lệ; hạ cờ `is_metric_valid = false`; chỉ xuất 2D pixel offset, không tính position base. | `phase_5/phase_5_offline_verified.log` | **PASS** |
| **T21** | Lệnh `cmd_vel_smoothed` quá cũ (> 0.2s) | Phase 7 | Nav2 Drop | Watchdog phát hiện cmd stale; Guard xuất vận tốc 0; không giữ lệnh cũ. | `phase_7/t21_stale_cmd_watchdog.log` | **PASS** |
| **T22A** | Guard Graceful Shutdown Watchdog | Phase 8 | SIGTERM Kill | Signal handler bắt SIGTERM, chủ động xuất 5 bản tin zero Twist trước khi thoát. Robot dừng hẳn trong <=2.5s (đo thực tế: stop delay 2.206s, final odom vx=0.0). | `phase_8/evidence_summary.md` | **PASS** |
| **T22B** | Guard Crash Without Handler Watchdog | Phase 8 | SIGKILL Kill | Node độc lập `escort_velocity_watchdog` phát hiện mất lệnh từ Guard (timeout 0.15s), chủ động xuất 0 liên tục ở 20 Hz vào Collision Monitor. Gửi SIGKILL vào PID Guard khi xe đang chạy: Watchdog zero dt=0.179s, /cmd_vel zero dt=0.211s. Robot dừng hẳn trong 0.687s (<=2.5s). Final odom vx=-0.0000, wz=0.0000. Đo đối chứng (Control): khi watchdog chết, không có lệnh mới nào được phát, khẳng định tính độc lập. | `phase_8/evidence_summary.md` | **PASS** |
| **T23** | Collision Monitor nằm sau Guard | Phase 8 | Integration Launch | Vật cản laser xuất hiện -> Collision Monitor cắt lệnh cuối dù Guard có cho phép đi. | `phase_8/evidence_summary.md` | **PASS** |
| **T24** | Chạy kiểm thử trên Gazebo hiện có | Phase 8 | Gazebo + Fake Perception | Robot dừng/quay/lách đúng quy tắc khi nhận tín hiệu quan sát giả lập; không sửa URDF. | `phase_8/evidence_summary.md` | **PASS** |
| **T25** | ROS 2 automated tests lặp 50 vòng | Phase 10 | Pytest / Launch Test | 100% pass; không có hiện tượng test bị treo (timeout rõ ràng); tài nguyên ổn định. | `phase_10/t25_stress_reliability.log` | **PASS** |
| **T26** | Thiếu depth hoặc alignment chưa kiểm chứng | Phase 5 | RGB-only Video | Khi UNVERIFIED_ALIGNMENT: ép `depth_valid = false`, không xuất số đo metric tin cậy để chọn nearest, cấm thay 0 cho depth thiếu. | `phase_5/phase_5_offline_verified.log` | **PASS** |
| **T27** | ByteTrack giữ ID nhưng vị trí không gian mơ hồ | Phase 3 | Synthetic Crossing | Cờ is_ambiguous = True bật khi IoU > 0.35 và tự giải phóng khi tách rời; truyền payload chuẩn sang FSM & Guard. | `phase_3/multi_person_crossing_trace.log` | **PASS** |
| **T28** | Bản tin TRANSIENT_LOCAL cũ khi node mới bật | Phase 6 | Node Restart | So sánh thời gian và sequence; bỏ qua bản tin cũ đã hết hạn (`freshness_ttl`). | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T29** | Voice ACK cancel nhưng Goal chưa terminal | Phase 6 | Mock Voice In-flight | Guard vẫn giữ dừng 0; FSM chưa đóng phiên cho tới khi nhận xác nhận terminal. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T30** | Khóa khách mới khi Goal cũ còn unresolved | Phase 6 | Mock Nav Conflict | Manager từ chối giơ tay khóa mới; giữ FSM chờ Goal cũ dọn dẹp xong hoàn toàn. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T31** | Khách xuất hiện lại trước abort vs sau abort | Phase 6 | Fake Obs Toggle | Trước 1.2s: Hồi phục `NAVIGATING`. Sau 1.2s: Chốt hủy, ảnh về vẫn giữ dừng, bắt buộc khóa mới. | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| **T32** | Từng node (Guard/Manager/Voice) bị restart giữa phiên | Phase 8 | Chaos Test | Nhận diện mất heartbeat; Guard đưa về 0; không có hành vi tự chạy lại nguy hiểm. | `phase_8/evidence_summary.md` | **PASS** |
| **T33** | Mô phỏng bị pause hoặc `/clock` bị nhảy | Phase 8 | Clock Pause Gazebo | Khi /clock đứng yên, Guard chủ động chuyển sang lệnh zero MỚI. Cửa sổ 0.3s post-resume không có obs/cmd mới duy trì 0 (không replay cache). Cấp lại input xe chạy tiếp tuân thủ max vx <= 0.50 m/s. | `phase_8/evidence_summary.md` | **PASS** |
| **T34** | Guard crash và nguồn phát lạ xuất hiện | Phase 7 | Multi-publisher Test | Collision Monitor và Base watchdog can thiệp; xác minh cơ chế cô lập publisher. | `phase_8/topic_graph_audit.txt` | **PASS** |
| **T35** | Nav2 `linear.y` pass-through vs Tracking lateral correction | Phase 7 | Mecanum Motion Test | Guard duy trì linear.y = 0.0 tuyệt đối, ngăn ngừa can thiệp chuyển động ngang tự phát. | `phase_7/t35_lateral_motion_separation.log` | **PASS** |

---

## 2. Quy trình Thực thi Kiểm thử Tự động hóa

1. **Khung kiểm thử:** Sử dụng `pytest` kết hợp `launch_testing` trong ROS 2.
2. **Quy định Timeout:** Mọi bài test đều bắt buộc phải khai báo tham số `timeout` rõ ràng (thường từ 5.0s đến 30.0s). Bất kỳ test nào vượt quá timeout sẽ tự động fail, không được phép treo luồng thực thi.
3. **Phân lập môi trường test:**
   - Đặt biến môi trường `ROS_DOMAIN_ID` riêng biệt khi chạy test (ví dụ `ROS_DOMAIN_ID=42`) để không gây nhiễu sang các node đang chạy của agent voice hoặc mô phỏng chung.
   - Sử dụng namespace cô lập (ví dụ `/test_tracking/...`).
4. **Lưu trữ bằng chứng (Evidence Logging):**
   - Toàn bộ kết quả test của từng Phase $N$ bắt buộc phải được ghi lại thành file log thô trong `.agent/person_tracking/evidence/phase_N/`.
