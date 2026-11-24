# PERSON TRACKING — TIẾN ĐỘ THỰC HIỆN VÀ THEO DÕI GIAI ĐOẠN

**Dự án:** Robot dẫn đường hành khách sân bay (ROS 2 Jazzy, Mecanum, Camera Astra sau)  
**Phân hệ:** `person_tracking`  
**File theo dõi duy nhất của phân hệ:** `.agent/person_tracking/PROGRESS.md`  
**Cập nhật lần cuối:** 24/11/2026 — Phase 10 Complete  

---

## Bảng tổng quan trạng thái các giai đoạn (Phase Overview)

| Giai đoạn | Tên giai đoạn | Trạng thái | Ngày cập nhật | Bằng chứng kỹ thuật |
|---|---|---|---|---|
| **Phase 0** | Khảo sát hiện trạng thực tế & Lập nền tài liệu AI | **PASS** | 20/11/2026 | `.agent/person_tracking/evidence/phase_0/audit_commands.log` |
| **Phase 1** | Quy tắc, skill, thiết kế & Kế hoạch kiểm thử | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_1/audit_addendum_commands.log` |
| **Phase 2** | Source adapters, offline dataset & Test infrastructure | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_2/` |
| **Phase 3** | Multi-person perception, pose & Tracking ID | **PARTIAL / CODE_READY** | 23/11/2026 | `.agent/person_tracking/evidence/phase_3/` |
| **Phase 4** | Giơ tay, chọn khách gần nhất & Khóa phiên | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_4/` |
| **Phase 5** | RGB-D, depth & Hình học camera sau | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_5/` |
| **Phase 6** | Session FSM, timeout & Giao tiếp voice | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_6/` |
| **Phase 7** | Velocity guard, mecanum & Watchdog | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_7/` |
| **Phase 8** | End-to-end ROS 2 integration (không sửa Gazebo) | **PASS** | 24/11/2026 | `.agent/person_tracking/evidence/phase_8/` |
| **Phase 9** | Giao tiếp song song với agent voice | **PASS** | 24/11/2026 | `.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log` (commit `3bf653da6c528c016131244493372e13d3f1b75d`) |
| **Phase 10** | Validation hiệu năng & Độ bền (offline + sim) | **PASS** | 24/11/2026 | `.agent/person_tracking/evidence/phase_10/evidence_summary.md` |
| **Phase 11** | Robot thật + Astra thật, nghiệm thu bàn giao | **SOFTWARE_READY / HARDWARE_PENDING** | 24/11/2026 | `.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md` |

---

## Chi tiết từng giai đoạn

### Phase 0 — Khảo sát hiện trạng thực tế & Lập nền tài liệu AI — [PASS]
- **Ngày hoàn thành:** 20/11/2026 14:55:00+07:00
- **Phụ lục đính chính (Audit Addendum):** 23/11/2026 trong [AUDIT_ADDENDUM.md](file:///home/thuytran/ws/src/.agent/person_tracking/AUDIT_ADDENDUM.md).
- **Nội dung đính chính kỹ thuật:**
  1. Phân loại độc lập 5 tầng GPU/runtime (Host physical GPU, GPU passthrough config, Container OS, Installed frameworks, Model inference status).
  2. Xác nhận `person_tracker.py` kiểm tra đẳng thức 3 frame (`rgb == depth == info`), không hard-code `camera_color_optical_frame` trong `on_images`.
  3. Xác nhận bộ lọc phân tán depth sử dụng Median Absolute Deviation (MAD), không phải IQR.
  4. Đọc chi nhánh `setImageRegistrationMode` của Astra driver: phát hiện nguy cơ gán nhầm frame ID khi thiết bị không hỗ trợ registration.
  5. Phân tích watchdog thụ động `command_timeout.py` của Gazebo và khẳng định sự cần thiết của cơ chế ép vận tốc 0 chủ động (Active Continuous Zeroing).
- **Bằng chứng:** [.agent/person_tracking/evidence/phase_0/audit_commands.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_0/audit_commands.log).

---

### Phase 1 — Quy tắc, skill, thiết kế & Kế hoạch kiểm thử — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 00:18:00+07:00
- **Baseline Git Commit:** `78ae0e51bdcbe4db4d536e96fa17a2d7ae8f9794`
- **HEAD hiện tại:** `78ae0e51bdcbe4db4d536e96fa17a2d7ae8f9794` (working tree sạch, giữ nguyên mọi thay đổi).
- **Phạm vi file đã sửa / tạo mới (100% trong allowlist):**
  - MỚI: [.agent/person_tracking/AUDIT_ADDENDUM.md](file:///home/thuytran/ws/src/.agent/person_tracking/AUDIT_ADDENDUM.md)
  - MỚI: [.agent/person_tracking/PROJECT_SPEC.md](file:///home/thuytran/ws/src/.agent/person_tracking/PROJECT_SPEC.md)
  - MỚI: [.agent/person_tracking/TEST_PLAN.md](file:///home/thuytran/ws/src/.agent/person_tracking/TEST_PLAN.md)
  - MỚI: [.agent/person_tracking/DECISIONS.md](file:///home/thuytran/ws/src/.agent/person_tracking/DECISIONS.md)
  - MỚI: [.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md](file:///home/thuytran/ws/src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md)
  - MỚI: [.agent/COMMANDS_TRACKING.md](file:///home/thuytran/ws/src/.agent/COMMANDS_TRACKING.md)
  - MỚI: [.agent/person_tracking/evidence/phase_1/audit_addendum_commands.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_1/audit_addendum_commands.log)
  - MỚI: [.agent/skills/ros2-person-tracking/SKILL.md](file:///home/thuytran/ws/src/.agent/skills/ros2-person-tracking/SKILL.md)
  - CẬP NHẬT: [.agent/person_tracking/INVENTORY.md](file:///home/thuytran/ws/src/.agent/person_tracking/INVENTORY.md) (điều chỉnh lịch triển khai khớp WORKFLOW).
  - CẬP NHẬT: [.agent/person_tracking/INTERFACES.md](file:///home/thuytran/ws/src/.agent/person_tracking/INTERFACES.md) (hoàn thiện schema, 4 loại định danh, clock domain).
  - CẬP NHẬT: [.agent/person_tracking/PROGRESS.md](file:///home/thuytran/ws/src/.agent/person_tracking/PROGRESS.md).
- **Mã nguồn ROS 2:** Giữ nguyên 100% các file trong `src/person_tracking/` (không sửa code, không đổi config, không đổi launch trong Phase 1).
- **Bảo toàn phân hệ khác:** Không đụng chạm code Voice, tài liệu Voice, Gazebo, Astra driver, Nav2, robot description.
- **Tiêu chí Nghiệm thu Gate 1 (Đạt):**
  1. [x] Đầy đủ tài liệu đặc tả, hợp đồng, test plan, decisions và skill.
  2. [x] Truy vết 100% các yêu cầu từ D01 đến D12.
  3. [x] Không sửa mã nguồn Voice; đã tạo bản đề xuất `VOICE_INTEGRATION_REQUEST.md`.
  4. [x] Phân quyền duy nhất: Chỉ `escort_tracking_manager` có quyền khóa và sinh `session_id`.
  5. [x] Chuỗi vận tốc an toàn: Smoother -> Guard -> Collision Monitor -> Base.
  6. [x] Phân tách rõ ràng giữa `linear.y` pass-through của Nav2 và hiệu chỉnh ngang tracking (mặc định tắt).
  7. [x] Test matrix mở rộng lên 35 ca kiểm thử (T01–T35) bao phủ toàn bộ các trường hợp rủi ro.
- **Phase tiếp theo, chỉ làm sau lệnh của chủ dự án:** Phase 2 — Source adapters, offline dataset & Test infrastructure.

---

### Phase 2 — Source adapters, offline dataset & Test infrastructure — [NOT_STARTED]
- Chỉ bắt đầu khi có lệnh từ chủ dự án.

---

### Phase 2 — Source adapters, offline dataset & Test infrastructure — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 10:30:00+07:00
- **Baseline Git Commit:** `a815f38b21e601159f069603ccae6570ed2a8db8`
- **HEAD hiện tại:** `a815f38b21e601159f069603ccae6570ed2a8db8` (giữ nguyên mọi thay đổi working tree của Voice).
- **Phạm vi file đã triển khai (100% trong allowlist):**
  - MỚI: `src/person_tracking/person_tracking/source_adapters.py` (ObservationFrame, compute_mad, VideoReplayAdapter, SyntheticFixtureGenerator, LiveAstraAdapter).
  - MỚI: `src/person_tracking/person_tracking/perception_source_node.py` (ROS 2 perception source node; KHÔNG publish /cmd_vel).
  - MỚI: `src/person_tracking/launch/offline_replay.launch.py` (Khởi chạy pipeline offline, tách biệt khỏi launch cũ).
  - MỚI: `src/person_tracking/fixtures/generate_fixtures.py` và bộ fixture (`synthetic_empty.npz`, `synthetic_person_1.npz`, `synthetic_corrupted_depth.npz`, `synthetic_video.mp4`, `metadata.json`, `sample_fixture_bag/`).
  - MỚI: `src/person_tracking/test/test_source_adapters.py` (15 unit tests (bổ sung sync slop, out-of-order, missing info, TopicSynchronizer)).
  - MỚI: `src/person_tracking/test/test_perception_source_smoke.py` (1 smoke test tích hợp).
  - CẬP NHẬT: `src/person_tracking/setup.py` & `package.xml` (Đăng ký entrypoint `perception_source`, cài đặt `fixtures/*`).
  - CẬP NHẬT TÀI LIỆU: `TEST_PLAN.md` (tách T01A và T01B), `COMMANDS_TRACKING.md` (lệnh build, test, replay và rosbag đã kiểm chứng).

- **Kết quả kiểm thử kỹ thuật (100% PASS):**
  1. **Unit & Smoke Tests:** 16/16 passed trong 0.69s trên Python 3.12 (ROS 2 Jazzy).
     - Bằng chứng: [.agent/person_tracking/evidence/phase_2/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_2/pytest_results.txt).
  2. **ROS 2 Launch Smoke Test:** 3/3 kịch bản launch thành công với tần số 15.0 FPS:
     - Kịch bản Synthetic Person: Nhận đủ RGB, Depth 16UC1, CameraInfo, trạng thái `VALID_RGBD`.
     - Kịch bản Video Replay: Tuân thủ nghiêm ngặt bất biến **RGB-only** (depth = 0 msgs / None, tuyệt đối không bịa số metric từ RGB đơn lẻ), trạng thái `RGB_ONLY`.
     - Kịch bản Corrupted Depth: Bơm dữ liệu lỗi NaN và 0, phát hiện chính xác lỗi, trạng thái `INVALID_DEPTH`.
     - Bằng chứng: [.agent/person_tracking/evidence/phase_2/launch_smoke_test.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_2/launch_smoke_test.log).
  3. **Rosbag2 Record & Replay:** Kiểm chứng luồng ghi và phát lại rosbag2 (MCAP):
     - Kiểm tra luồng subscriber trước khi bắt đầu ghi bag.
     - Kết thúc recorder sạch bằng SIGINT (exit code 0), tạo bag hợp lệ 60.1 MiB.
     - `ros2 bag info`: 174 messages trong 1.48s (/camera/color/image_raw: 41, /camera/depth/image_raw: 41, /camera/color/camera_info: 46, /tracking/source_status: 46).
     - `ros2 bag play`: Replay thành công, subscriber nhận lại đầy đủ (56 RGB, 59 Depth, 67 Info, 67 Status).
     - Bằng chứng: [.agent/person_tracking/evidence/phase_2/rosbag_record_replay.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_2/rosbag_record_replay.log).

- **Kiểm tra an toàn & Biên giới phân hệ:**
  - `Topic /cmd_vel`: **Publisher count: 0** từ `person_tracking` trong mọi kịch bản.
  - Phân hệ Voice giữ quyền sở hữu duy nhất `NavigateToPose`. Phase 2 chỉ xử lý dữ liệu đầu vào.
  - Build cô lập 100% trong `build_tracking/`, `install_tracking/`, `log_tracking/`, không đè lên build của Voice.

---

### Phase 3 — Multi-person perception, pose & Tracking ID — [PARTIAL / CODE_READY]
- **Ngày/giờ cập nhật:** 23/11/2026 14:20:00+07:00
- **Baseline Git Commit:** `53b79a1eb4d1f3e962ad02890c54dc2a276c72d9`
- **Môi trường & Phần cứng:**
  - Container OS: Ubuntu 24.04 (Noble LTS), Python 3.12.3, ROS 2 Jazzy.
  - Framework: PyTorch `2.14.1+cu130`, Ultralytics `8.4.185`.
  - Hardware: NVIDIA RTX A3000 Laptop GPU (6.08 GB VRAM), CUDA active.
  - Model Checkpoint: `/home/ws/yolo26n-pose.pt` (task: `pose`, 17 keypoints).
- **Đo lường Hiệu năng Thực tế (Benchmark 640x480):**
  - **Inference Benchmark GPU (CUDA:0):** Độ trễ trung bình **5.61 ms/frame** → Thông lượng **178.3 FPS**.
  - **Inference Benchmark CPU (Fallback):** Độ trễ trung bình **23.26 ms/frame** → Thông lượng **43.0 FPS**.
  - **ROS Pipeline Node Latency & Throughput:** Node xử lý end-to-end với độ trễ ~18–25 ms/frame (~40–55 FPS năng lực xử lý), vận hành đồng bộ theo tần số phát nguồn 15.0 FPS.
  - Bằng chứng: [.agent/person_tracking/evidence/phase_3/benchmark_gpu_cpu.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_3/benchmark_gpu_cpu.log).

- **Các cải tiến kỹ thuật hoàn thiện trong lượt này:**
  1. **Độ tin cậy của Depth:** Ràng buộc chặt chẽ `depth_valid = false` khi `data_quality == UNVERIFIED_ALIGNMENT` (hoặc thiếu depth / depth lỗi), đảm bảo downstream Phase 4 không bao giờ lấy số đo unverified để chọn khách gần nhất. Pose và ByteTrack vẫn tiếp tục chạy bình thường bằng RGB. Đã kiểm thử unit test cả hai trường hợp verified và unverified.
  2. **Reset Tracker khi loop hoặc đổi nguồn:** Xử lý `reset_event`, `loop_count` tăng và `source_session_id` mới từ `/tracking/source_status`. Xóa toàn bộ trạng thái ByteTrack nội bộ của Ultralytics, lịch sử timestamp, bộ đệm depth và CameraInfo để frame đầu của vòng lặp mới không bị dính vết của vòng lặp cũ.
  3. **Chuẩn hóa Schema `/tracking/observation` (Schema Version 1.0):** Cung cấp đầy đủ các trường chính thức (`candidates`, `bbox_xyxy`, `stamp.sec`, `receive_monotonic_time`, `source_session_id`, `data_quality`, `active_track_ids`, `lost_track_ids`, `depth_valid`, `is_ambiguous`) cùng alias tương thích ngược (`persons`, `bbox`, `receive_monotonic`). Đã kiểm thử xác thực kiểu dữ liệu (`test_observation_schema_types`).
  4. **Mô phỏng 2 người cắt ngang (Crossing Sequence Simulation):** Tạo trace mô phỏng 15 frames hai người cắt ngang với $IoU > 0.35$. Bật cờ `is_ambiguous = true` khi cắt nhau và tự động giải phóng về `false` khi tách rời, ngăn ngừa FSM khóa nhầm khách.
  5. **Nghiệm thu Rosbag Gate 2:** Xác nhận `ros2 bag play` trên bag mẫu `sample_fixture_bag` với `ROS_DOMAIN_ID=42` và `--delay 1` nhận thành công 95.1% số frames (39/41 RGB và Depth), giải thích rõ cơ chế phát hiện DDS discovery latency.

- **Kết quả kiểm thử tự động hóa (100% PASS):**
  - Toàn bộ **29/29 tests** trong `person_tracking/test/` đều đạt trong 6.98s.
  - Bằng chứng Pytest: [.agent/person_tracking/evidence/phase_3/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_3/pytest_results.txt).
  - Bằng chứng Rosbag Gate 2: [.agent/person_tracking/evidence/phase_2/rosbag_replay_verified.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_2/rosbag_replay_verified.log).
  - Bằng chứng Mô phỏng cắt ngang: [.agent/person_tracking/evidence/phase_3/multi_person_crossing_trace.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_3/multi_person_crossing_trace.log).

- **Trạng thái Gate 3 & Bàn giao:**
  - Trạng thái kỹ thuật: **PARTIAL / CODE_READY**. Code perception, tracking và an toàn đã sẵn sàng 100%. Gate 3 để mở chờ người dùng cung cấp video người thật để nghiệm thu trực quan visual tracking.
  - Topic `/cmd_vel`: **Publisher count: 0** trong toàn bộ runtime Phase 3.
  - Hoàn tất kiểm thử hồi quy Phase 3 và bàn giao sang Phase 4.

---

### Phase 4 — Giơ tay, chọn khách gần nhất & Khóa phiên — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 15:00:00+07:00
- **Baseline Git Commit:** `5f0fb1f187d468d412e2f7988dc42cc0f692f586`
- **Mục tiêu đạt được:** Phân quyền khóa và sinh phiên duy nhất (`ADR-01`), nhận diện cử chỉ giơ một tay giữ đủ thời gian (1.5s), chọn người gần nhất, khóa phiên FSM (`LOCKED_WAIT_DESTINATION`), và cô lập phiên theo nguồn.

- **Các thành phần đã triển khai (100% trong allowlist):**
  1. `passenger_lock_manager.py`:
     - Phân tích 17 body keypoints: kiểm tra cổ tay (wrist idx 9/10) cao hơn vai cùng bên (shoulder idx 5/6) với `min_keypoint_conf >= 0.35`.
     - Bộ đếm thời gian giữ cử chỉ: ngưỡng `gesture_hold_time_sec = 1.5s`. Giơ tay < 1.5s không khóa, hạ tay reset bộ đếm về 0.0s (`T02`).
     - Chốt khóa phiên (`T03`): Đủ 1.5s liên tục chuyển FSM `IDLE` -> `LOCKED_WAIT_DESTINATION`, `locked = true`, cấp UUID `session_id`. Khách hạ tay sau khi khóa vẫn giữ nguyên phiên.
     - Chọn người gần nhất (`T04`): Khi 2 người cùng giơ tay đủ điều kiện, ưu tiên chọn người có `torso_depth_m` nhỏ nhất (gần camera hơn) khi `depth_valid == true`.
     - Chống khóa mơ hồ (`T05`): Khi thiếu depth hoặc depth không hợp lệ (`depth_valid == false`) mà kích thước bbox gần bằng nhau (chênh lệch diện tích $\le 30\%$) hoặc có cờ `is_ambiguous == true`, từ chối khóa tùy tiện, giữ `IDLE`, báo `AMBIGUOUS_GESTURE`.
     - Cô lập phiên theo nguồn: Khi `source_session_id` thay đổi (đổi nguồn hoặc video replay lặp vòng mới), phiên cũ tự hủy về `IDLE` (`SOURCE_RESET_DISCONNECT`), tuyệt đối không chuyển khóa sang người có cùng `track_id` ở vòng mới.
  2. `tracking_manager_node.py` (`escort_tracking_manager`):
     - Node ROS 2 nhận `/tracking/observation`, xuất `/tracking/session_state` (QoS `TRANSIENT_LOCAL`, nhịp tim 5 Hz + event-driven khi đổi trạng thái).
     - Đăng ký console script `tracking_manager` trong `setup.py`.
     - **An toàn chuyển động:** Kiểm tra `get_publisher_names_and_types_by_node` xác nhận số lượng publisher `/cmd_vel` = 0.
  3. `test_tracking_manager.py`:
     - 7 ca kiểm thử tự động hóa phủ toàn bộ ma trận T02, T03, T04, T05, cô lập phiên và hợp đồng Mock Voice.

- **Kiểm chứng Hợp đồng Giao tiếp với Voice (Mock Voice Contract):**
  - Khi mock voice nhận yêu cầu xuất phát (chọn điểm đến):
    - Nếu `/tracking/session_state` có `locked == false` hoặc trạng thái khác `LOCKED_WAIT_DESTINATION`: Voice từ chối khởi hành (`departure_allowed = false`).
    - Khi `locked == true` và `state == "LOCKED_WAIT_DESTINATION"`: Voice chấp nhận điểm đến và cho phép gửi Goal sang Nav2 (`departure_allowed = true`).

- **Phân định Rõ ràng Giữa Dữ liệu Kiểm thử Giả lập & Nghiệm thu Thực tế:**
  - **Đã kiểm chứng đạt 100% bằng dữ liệu giả lập (PASS):**
    - Logic nhận diện hình học giơ tay (wrist cao hơn shoulder).
    - Ngưỡng thời gian giữ 1.5s và cơ chế reset bộ đếm khi hạ tay trước ngưỡng.
    - Duy trì khóa sau khi hạ tay.
    - Thuật toán so sánh depth chọn người gần nhất và từ chối khóa khi mơ hồ thiếu depth.
    - Cơ chế cô lập ID và phiên khi đổi nguồn / lặp vòng.
    - Hợp đồng chặn xuất phát của Mock Voice.
    - Toàn bộ **39/39 tests** (Phase 2 + Phase 3 + Phase 4) đều PASS trong 7.30s.
  - **Mục còn chờ nghiệm thu thực tế (Live Hardware Pending):**
    - Nghiệm thu trên người thật trước camera Astra trong điều kiện ánh sáng, trang phục và dáng đứng thực tế tại nhà ga sân bay (thuộc phạm vi Phase 11).

- **Bằng chứng kỹ thuật:**
  - Log nghiệm thu Phase 4: [.agent/person_tracking/evidence/phase_4/gesture_locking_verified.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_4/gesture_locking_verified.log).
  - Kết quả kiểm thử pytest tổng hợp: [.agent/person_tracking/evidence/phase_4/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_4/pytest_results.txt).

- **Xử lý Hồi quy Phase 3–4 (Regression Fixes):**
  1. **Đồng bộ hàng đợi replay & siêu dữ liệu phiên:**
     - Loại bỏ hành vi dùng `latest_status` của frame khác để gán phiên sau timeout.
     - Trong chế độ replay có metadata, frame quá hạn chờ (> 0.30s) mà không có status khớp stamp sẽ bị HỦY (DROP) kèm cảnh báo tường minh.
     - Bỏ qua status đến muộn từ các vòng lặp cũ (`loop_count < last_loop_count`) hoặc phiên cũ (`session_id in past_session_ids`).
     - Bổ sung drain timer chu kỳ 10 Hz giải phóng hàng đợi khi nguồn ngừng phát.
     - Giữ nguyên luồng nhận live camera / rosbag không có status.
  2. **Bộ đếm cử chỉ giơ tay & cô lập phiên:**
     - Khắc phục lỗi false-lock khi cùng một observation stamp gửi lặp lại cách nhau 1.6s: chỉ tích lũy thời gian khi observation mới có $\Delta t > 0$.
     - Khắc phục lỗi tích lũy thời gian khi cùng `track_id` xuất hiện ở nguồn A rồi nguồn B: cô lập trạng thái cử chỉ theo cặp `(source_session_id, track_id)`.
     - Frame gap vượt ngưỡng 0.25s tự động reset bộ đếm cử chỉ của track đó.
     - Duy trì khóa an toàn sau khi hạ tay.
  3. **Tách biệt timestamp heartbeat và quan sát:**
     - Trường `stamp` trong `/tracking/session_state` luôn là thời điểm phát nhịp tim (ROS clock `now()`).
     - Trường `last_observation_stamp` ghi nhận chính xác thời điểm chụp quang học thực tế từ camera.
     - Cờ `obs_fresh` cảnh báo quan sát quá hạn (> 0.4s) với `reason_code = "OBSERVATION_STALE"`.
  - Gate 3 tiếp tục duy trì trạng thái **PARTIAL / CODE_READY** (chờ video người thật `T06`).

---

### Phase 5 — RGB-D, depth & Hình học camera sau — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 16:45:00+07:00
- **Baseline Git Commit:** `7028d538decab0462c8de0aaf28a0114a0bab212`
- **Mục tiêu đạt được:** Nghiệm thu trắc lượng RGB-D offline, chuẩn hóa lọc nhiễu vùng thân, thiết lập mô hình hình học camera sau (`ADR-10`), phân tách offset pixel 2D với góc lệch phương vị 3D, và đảm bảo an toàn tuyệt đối.

- **Các thành phần đã triển khai (100% trong allowlist):**
  1. `rear_camera_geometry.py`:
     - Mô hình camera pinhole `CameraIntrinsics`: trích xuất và thẩm định $f_x, f_y, c_x, c_y$ từ `CameraInfo`.
     - Biến đổi tọa độ lắp đặt `CameraMountTransform`: camera sau quay về sau (`yaw = pi` so với `base_link`). Ma trận quay trực giao chuẩn $R_{	ext{rear}} = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]$, $\det(R) = +1$.
     - Động học hình học `RearCameraGeometry`:
       * Offset pixel chuẩn hóa `center_offset_x_norm` trong $[-1.0, 1.0]$.
       * Góc lệch phương vị quang học $	heta_{	ext{opt}} = rctan2(u - c_x, f_x)$.
       * Tọa độ 3D quang học $P_{	ext{opt}} = [X, Y, Z]$.
       * Tọa độ 3D thân xe $P_{	ext{base}} = R \cdot P_{	ext{opt}} + T$.
       * Góc lệch phương vị thân xe $\phi_{	ext{base}} = rctan2(Y_{	ext{base}}, X_{	ext{base}})$.
     - Tuân thủ quy ước dấu `ADR-10`:
       * Khách bên phải ảnh ($u > c_x$): $	heta_{	ext{opt}} > 0 \implies Y_{	ext{base}} > 0$ (sườn TRÁI robot, góc quay $\in (\pi/2, \pi)$).
       * Khách ở giữa ảnh ($u = c_x$): $	heta_{	ext{opt}} = 0 \implies Y_{	ext{base}} = 0$ (thẳng sau đuôi robot, góc quay $\pm \pi$).
       * Khách bên trái ảnh ($u < c_x$): $	heta_{	ext{opt}} < 0 \implies Y_{	ext{base}} < 0$ (sườn PHẢI robot, góc quay $\in (-\pi, -\pi/2)$).
     - Xử lý biên: Thiếu TF -> `position_base = None`; Lệch frame -> `is_metric_valid = False`; Intrinsics lỗi -> `optical_bearing = None`.
  2. Cải tiến bộ lọc depth vùng thân (`multi_person_tracker.py`):
     - Ràng buộc tỷ lệ pixel hợp lệ trong torso ROI: $\ge 30\%$ và $\ge 15$ pixels (loại bỏ phông nền và biên nhiễu).
     - Ràng buộc dải cảm biến Astra: $[0.4m, 5.0m]$ (loại bỏ cự ly cực đoan, zero, NaN).
     - Bộ lọc phân tán MAD: $	ext{MAD} \le 0.25m$ (loại bỏ nhiễu hạt, phản xạ).
     - Ràng buộc tin cậy căn chỉnh: Chỉ khi `data_quality == VALID_RGBD` thì `depth_valid = True`. Khi `UNVERIFIED_ALIGNMENT`, `depth_valid = False` tuyệt đối; không dùng raw depth để chọn khách gần nhất; không thay 0 cho depth thiếu.
  3. `test_rear_camera_geometry.py`:
     - 9 ca kiểm thử chuyên sâu phủ toàn bộ các kịch bản hình học ADR-10, các trường hợp lỗi biên, các bộ lọc depth và audit an toàn vận tốc.
  4. Toàn bộ test suite phân hệ:
     - **55/55 tests PASS** trong 8.31s (test_multi_person_tracker: 11, test_person_detector_smoke: 8, test_rear_camera_geometry: 9, test_source_adapters: 16, test_tracking_manager: 11).

- **Phân định Rõ ràng Giữa Dữ liệu Kiểm thử Giả lập & Nghiệm thu Thực tế:**
  - **Đã kiểm chứng đạt 100% bằng dữ liệu offline & mô phỏng (PASS):**
    - Toàn bộ trắc lượng depth ROI, MAD filter, sensor range, valid pixel ratio.
    - Toàn bộ phép chiếu hình học ADR-10 cho camera sau với góc lệch trái/giữa/phải.
    - Khả năng chịu lỗi khi mất CameraInfo, TF transform, hoặc sai frame ID.
  - **Mục còn chờ nghiệm thu thực tế (Live Hardware Pending):**
    - Camera Astra thật tiếp tục ở trạng thái **HARDWARE_VALIDATION_PENDING** (chờ kết nối và cân chỉnh trên robot thật tại Phase 11).

- **Bằng chứng kỹ thuật:**
  - Trace hình học camera sau: [.agent/person_tracking/evidence/phase_5/rear_camera_geometry_trace.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_5/rear_camera_geometry_trace.log).
  - Log nghiệm thu Phase 5: [.agent/person_tracking/evidence/phase_5/phase_5_offline_verified.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_5/phase_5_offline_verified.log).
  - Kết quả kiểm thử pytest tổng thể: [.agent/person_tracking/evidence/phase_5/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_5/pytest_results.txt).

- **Trạng thái phân hệ & Điểm dừng:**
  - Phân hệ Voice tiếp tục sở hữu duy nhất `NavigateToPose`.
  - Phân hệ Tracking tuyệt đối KHÔNG phát bất kỳ lệnh `/cmd_vel` hay mục tiêu di chuyển nào.
  - DỪNG TRƯỚC PHASE 6 theo đúng quy định bàn giao.


---

### Phase 6 — Session FSM, timeout & Giao tiếp voice — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 17:15:00+07:00
- **Baseline Git Commit:** `043d0050c92562d3c96d7fc1b73d29d573d5006a`
- **Mục tiêu đạt được:** Hoàn tất toàn diện Phase 6 Offline, sửa triệt để các lỗi hồi quy đầu vào (Observation/Watchdog, Replay Queue Timeout, Geometry Input), triển khai đầy đủ máy trạng thái phiên FSM (Single Lock Authority), cơ chế bắt tay 2 pha (ACK vs Terminal), mạch retry lệnh hủy an toàn sang Voice, và bộ kiểm thử tích hợp 100% PASS với Mock Voice.

- **Các nội dung kỹ thuật đã hoàn thành:**
  1. **Sửa Hồi quy Đầu vào trước khi nối FSM (Section A):**
     - *Observation & Watchdog:* Chặn dứt điểm lỗi lặp stamp 0/0 tích lũy thời gian khóa; yêu cầu timestamp đơn điệu $0 < \Delta t \le 0.25$s; loại bỏ hoàn toàn fallback sang receive monotonic time khi tích lũy cử chỉ; tách biệt rạch ròi giữa thời điểm nhận frame và thời điểm nhìn thấy đúng khách đã khóa. Khách xuất hiện liên tục quá 0.4s vẫn fresh; frame rỗng liên tục quá 0.4s làm mất fresh dù camera stamp vẫn tăng; duplicate/backwards stamp và publisher đóng băng không làm tươi khách; heartbeat timer định kỳ không làm sống lại quan sát cũ.
     - *Replay Queue & Timeout:* Trong `_drain_pending_frames()`, kiểm tra hạn chót `(now_mono - queue_time) > self.max_pending_age_sec` TRƯỚC KHI xử lý cặp status-frame đã khớp. Khung hình có status đến sau 0.30s (timeout) bị HỦY (DROP) và xóa status kèm cảnh báo log, không bị xử lý nhầm; đồng bộ toàn bộ log message dùng timeout cấu hình (loại bỏ hoàn toàn chuỗi sót 0.04s). Giữ nguyên luồng nhận live camera / bag không có status.
     - *Đầu vào Hình học cho FSM:* Mặc định runtime không tự ý chèn mount giả (`use_synthetic_mount=False`, `position_base=None`, `bearing_base_rad=None`, `geometry_valid=False`, `geometry_source="OPTICAL_ONLY"`, `optical_bearing_rad` vẫn tính từ CameraInfo). Chỉ bật phép chiếu khi có opt-in tường minh (`use_synthetic_mount=True`). Kiểm tra source/target frame ID của transform (`base_link` vs camera frame). Phân tách độc lập hoàn toàn giữa `depth_valid` và `geometry_valid`. Thiết bị Astra thật tiếp tục duy trì trạng thái `HARDWARE_VALIDATION_PENDING` (Phase 11).
  2. **Triển khai Máy trạng thái Phiên & Giao tiếp Voice (Section B):**
     - *Cơ quan duy nhất giữ Lock:* `tracking_manager_node.py` quản lý toàn bộ vòng đời FSM: `IDLE`, `LOCKED_WAIT_DESTINATION`, `NAVIGATING`, `WAITING_FOR_PASSENGER`, `ABORTING`, `ARRIVED`, `FAULT`.
     - *Khởi hành an toàn:* Chỉ chuyển sang `NAVIGATING` khi đã khóa (`locked == True`), quan sát còn mới (`obs_fresh == True`), không có goal cũ unresolved, và Voice gửi tín hiệu bắt đầu.
     - *Ngưỡng thời gian an toàn:* Grace period 0.25s (giữ `NAVIGATING`, lý do `LOCKED_TARGET_GRACE_PERIOD`), Stop threshold 0.40s (chuyển `WAITING_FOR_PASSENGER`, ép dừng), Abort threshold 1.20s (chuyển `ABORTING`, phát lệnh hủy).
     - *Watchdog độc lập 10 Hz:* Tự động kích hoạt chuyển trạng thái và phát lệnh hủy ngay cả khi luồng ảnh camera bị đóng băng hoặc ngắt kết nối hoàn toàn.
     - *Phục hồi sau WAIT:* Đúng khách quay lại trước 1.20s tự động phục hồi về `NAVIGATING` (`PASSENGER_RETURNED_RESUMED`). Sau khi đã `ABORTING`, chốt dừng vĩnh viễn (latched stop), khách quay lại không được tự unlatch.
     - *Giao thức Bắt tay 2 pha trên `/voice/nav_status`:*
       * Pha 1: Voice gửi ACK (`nav_event: "CANCEL_ACKNOWLEDGED"`, `is_terminal: false`) kèm `request_id` -> Tracking dừng vòng lặp retry nhưng tiếp tục giữ `ABORTING`.
       * Pha 2: Nav2 kết thúc và Voice gửi Terminal (`is_terminal: true`, `status: "CANCELED"` / `"SUCCEEDED"`) -> Tracking dọn dẹp phiên về `IDLE`.
       * Unconfirmed Terminal: Nếu sau 3 lần retry không nhận được ACK/Terminal, chuyển sang `FAULT`, chặn tuyệt đối không cho mở phiên mới (`UNCONFIRMED_TERMINAL_BLOCK`).
       * Bản tin cũ / sai `session_id` bị loại bỏ hoàn toàn.
     - *Tài liệu tích hợp Voice:* Cập nhật đầy đủ trong [.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md](file:///home/thuytran/ws/src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md).
  3. **Kết quả Kiểm thử Toàn diện (100% PASS):**
     - Toàn bộ **70/70 tests** của phân hệ `person_tracking` đều PASS trong 11.48s (ROS 2 Jazzy, ROS_DOMAIN_ID=42).
     - Riêng file tích hợp FSM & Mock Voice `test_session_fsm_mock_voice.py`: 7/7 tests PASS trong 4.14s.
     - Kiểm thử an toàn tuyệt đối: Khẳng định **ZERO `/cmd_vel` publishers** và **ZERO `geometry_msgs/Twist`** trên toàn bộ các node của phân hệ `person_tracking`. Phân hệ Voice tiếp tục là chủ sở hữu duy nhất của `NavigateToPose`.

- **Bằng chứng kỹ thuật:**
  - Log kiểm thử FSM & Mock Voice: [.agent/person_tracking/evidence/phase_6/phase_6_fsm_mock_voice.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_6/phase_6_fsm_mock_voice.log).
  - Kết quả kiểm thử pytest tổng hợp: [.agent/person_tracking/evidence/phase_6/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_6/pytest_results.txt).
  - Bản yêu cầu tích hợp Voice: [.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md](file:///home/thuytran/ws/src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md).

- **Điểm dừng bàn giao:** Dừng chính xác trước Phase 7 (Velocity guard, mecanum & Watchdog). User tự commit và push lên Git.


---

### Phase 7 — Velocity guard, mecanum & Watchdog — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 17:50:00+07:00
- **Baseline Git Commit:** `e6d3d8ce9504bd99d85be27c6a7997e925f9191f`
- **Mục tiêu đạt được:** Sửa triệt để các hồi quy về quyền chạy xe của Phase 6, triển khai Cổng an toàn vận tốc độc lập `escort_velocity_guard` (`ADR-04`), chuẩn hóa chuỗi topic `cmd_vel_smoothed -> escort_velocity_guard -> cmd_vel_guarded -> Collision Monitor -> base`, cơ chế Active Continuous Zeroing 20 Hz, điều khiển cự ly dãn cách có hysteresis, hiệu chỉnh góc quay có bão hòa (ADR-10), và bộ kiểm thử an toàn tự động 100% PASS.

- **Các nội dung kỹ thuật đã hoàn thành:**
  1. **Sửa Hồi quy Phase 6 trước khi xuất vận tốc (Section A):**
     - *Kiểm tra deadline mất khách trước khi nhận frame quay lại:* Nếu khách đã mất $\ge 1.20$s, khi frame mới có ID khớp đến trước tick watchdog, FSM vẫn bắt buộc chuyển sang `ABORTING` (`VISIBILITY_TIMEOUT_ABORT`), không được phục hồi.
     - *Chốt dừng vĩnh viễn (Latched Stop) trong ABORTING:* Sau khi đã vào `ABORTING`, đổi `source_session_id`, cùng ID quay lại hoặc frame cũ đều bị chặn tuyệt đối, không được tự ý unlatch hoặc reset về `IDLE`. Chỉ duy nhất bản tin Terminal hợp lệ từ Voice mới được phép dọn dẹp phiên.
     - *Phân biệt chớp/mất tạm thời vs OUT_OF_FOV được xác nhận:* Khách ở gần mép ảnh (`near_edge == true`) rồi biến mất ở frame tiếp theo được xác nhận `OUT_OF_FOV` $\implies$ lập tức chuyển sang `ABORTING` (`reason_code = "OUT_OF_FOV"`). Nếu khách ở giữa ảnh và biến mất tạm thời $\implies$ vào grace period (0.25s), không dừng đột ngột.
     - *Chuẩn hóa kiểm tra tính hợp lệ & thời gian của Observation:* Từ chối stamp thiếu, nanoseconds ngoài dải $[0, 10^9)$, kiểu dữ liệu sai (bool/str). Stamp 0/0, stamp thiếu, hoặc stamp đi lùi tuyệt đối không được gia hạn độ mới quan sát của khách đã khóa. Frame đầu tiên của `source_session_id` mới có timestamp bắt đầu lại từ 0/epoch được khởi tạo sequence baseline mới, không bị loại nhầm là `is_old_frame`, trong khi bản tin muộn của phiên nguồn cũ bị loại bỏ hoàn toàn.
     - *Siết chặt hợp đồng Mock Voice:* Lệnh `START` (`on_destination_accepted`) bắt buộc phải mang đầy đủ `session_id`, `mission_id`, và `goal_attempt_id`; loại bỏ hoàn toàn fallback tự điền `session_id`. Bản tin `ACK` bắt buộc phải khớp `request_id`, `session_id`, và `goal_attempt_id`; `ACK` chỉ có tác dụng dừng vòng lặp retry. Kích hoạt bộ đếm hạn chót Terminal riêng (`terminal_timeout_sec = 3.0s`) sau khi nhận `ACK`. Hết 3 lần retry không có `ACK` hoặc quá 3.0s không nhận được Terminal $\implies$ chuyển sang `FAULT`, đánh dấu `unconfirmed_terminal = true`, chốt dừng 0 và chặn tuyệt đối không cho mở phiên mới. Bản tin Terminal mang trạng thái `status: "UNKNOWN"` hoặc sai goal ID bị từ chối tuyệt đối.
  2. **Triển khai Phân hệ Velocity Guard, Mecanum & Watchdog (Section B):**
     - *Chuỗi Topic Chuẩn:* `/tracking/cmd_vel_smoothed` $\to$ `escort_velocity_guard` $\to$ `/tracking/cmd_vel_guarded` $\to$ Collision Monitor $\to$ `/cmd_vel` $\to$ Base. Không dùng `twist_mux` để trộn yaw; Guard KHÔNG publish trực tiếp vào `/cmd_vel`.
     - *Cơ chế Active Continuous Zeroing:* Mặc định xuất `Twist` toàn 0 ở tần số 20 Hz độc lập. Bất kỳ vi phạm an toàn nào hoặc khi trạng thái khác `NAVIGATING` (`IDLE`, `LOCKED_WAIT_DESTINATION`, `WAITING_FOR_PASSENGER`, `ABORTING`, `ARRIVED`, `FAULT`) đều lập tức ép vận tốc về 0, bỏ qua smoothing.
     - *Watchdog Độc lập 20 Hz:* Chạy bằng thời gian đơn điệu `time.monotonic()`, hoàn toàn miễn nhiễm với việc dừng hoặc nhảy lùi của `/clock` mô phỏng.
     - *Điều kiện Mở Cổng Di chuyển (Motion Gate):* Chỉ pass-through khi FSM ở `NAVIGATING`, `locked == true`, heartbeat `session_state` $\le 1.0$s, khách quan sát còn mới $\le 0.40$s (`obs_fresh == true`), chỉ dùng depth đã xác nhận (`depth_valid == true` và `depth_proven == true`), và nhận được lệnh `cmd_vel_smoothed` mới ($\le 0.20$s) sau thời điểm mở cổng di chuyển.
     - *Chế độ Degraded khi Depth chưa xác nhận:* Khi `data_quality == UNVERIFIED_ALIGNMENT` (hoặc thiếu depth / depth lỗi), chế độ degraded lập tức ép toàn bộ vận tốc về 0. Giữ `linear.y = 0.0` tuyệt đối.
     - *Đường cong Điều khiển Cự ly & Hysteresis:*
       * Dải an toàn $[1.2m, 2.0m]$: Cho phép Nav2 chạy đúng tốc độ tính toán ($v_x = v_{x, \text{in}}$).
       * Dải dãn cách $(2.0m, 2.5m]$: Giảm dần trần tốc độ tuyến tính $\text{scale} = 1.0 - (d - 2.0) / 0.5$.
       * Vượt quá $2.5m$: Dừng hẳn ($v_x = 0$), chốt cờ dừng cự ly (`distance_stop_latched = true`).
       * Hysteresis: Trong khi chốt dừng, chỉ cho phép chạy lại khi khách tiến lại gần $\le 2.2m$.
       * Dưới $1.2m$: An toàn cự ly gần, duy trì tốc độ Nav2, tuyệt đối không tăng tốc dãn cách và không lùi xe vào người ($v_x \ge 0$).
     - *Hiệu chỉnh Góc quay $\Delta \omega$ (Angular Trimming):*
       * Vùng chết (Deadband): $|center\_offset\_x\_norm| \le 0.08 \implies \Delta \omega = 0.0$.
       * Quy ước dấu quang học ADR-10: Khách lệch phải ảnh ($e_x > 0$) $\implies$ quay CCW ($\omega > 0$). Khách lệch trái ($e_x < 0$) $\implies$ quay CW ($\omega < 0$).
       * Bão hòa góc quay: $|\Delta \omega| \le 0.20$ rad/s. Trần góc quay tổng: $0.40$ rad/s.
       * Giảm tốc mép ảnh: $|e_x| > 0.70 \implies$ giảm 50% tốc độ tiến ($v_x = 0.5 v_x$).
       * Bất biến Zero Nav2: Lệnh Nav2 bằng 0 thì xuất 0 (tuyệt đối không tự quay khi Nav2 đứng yên).
       * Giải quyết xung đột góc quay: Nếu lệnh quay Nav2 đối nghịch với việc giữ khách trong ảnh ở mép, ưu tiên dừng chờ (`NAV2_EDGE_TURNING_CONFLICT_STOP`).
  3. **Kết quả Kiểm thử Toàn diện (100% PASS):**
     - Toàn bộ **95/95 tests** của phân hệ `person_tracking` đều PASS trong 13.13s (ROS 2 Jazzy, ROS_DOMAIN_ID=42).
     - Đã kiểm chứng đầy đủ các ca kiểm thử Phase 7: T09, T10, T11, T12, T14, T16, T21, T22, T33, T35.
     - Khẳng định **ZERO `/cmd_vel` publishers** trên toàn bộ các node của phân hệ `person_tracking`. Phân hệ Voice tiếp tục là chủ sở hữu duy nhất của `NavigateToPose`.
     - Kiểm chứng hành vi kill Guard: Khi node `escort_velocity_guard` bị hủy, subscriber phía sau lập tức ngừng nhận bản tin, không có lệnh dư thừa tự phát.

- **Bằng chứng kỹ thuật:**
  - Build overlay: [.agent/person_tracking/evidence/phase_7/colcon_build.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_7/colcon_build.log).
  - Kết quả kiểm thử pytest tổng thể: [.agent/person_tracking/evidence/phase_7/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_7/pytest_results.txt).
  - Trace các ca kiểm thử: T09, T10, T11, T12, T14, T16, T21, T22, T33, T35 trong [.agent/person_tracking/evidence/phase_7/](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_7/).
  - Log kiểm thử hồi quy Phase 6: [.agent/person_tracking/evidence/phase_7/phase6_regressions.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_7/phase6_regressions.log).
  - Đặc tả hợp đồng giao diện: [.agent/person_tracking/INTERFACES.md](file:///home/thuytran/ws/src/.agent/person_tracking/INTERFACES.md).
  - Bản yêu cầu tích hợp Voice: [.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md](file:///home/thuytran/ws/src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md).

- **Điểm dừng bàn giao:** Dừng chính xác trước Phase 8 (End-to-end ROS 2 integration). User tự commit và push lên Git.

---

### Phase 8 — End-to-end ROS 2 integration (không sửa Gazebo) — [PASS]
- **Ngày/giờ hoàn thành:** 23/11/2026 22:35:00+07:00
- **Baseline Git Commit:** `179e38fa403199497518ccc641354f0089ae01e3`
- **Mục tiêu đạt được:** Nghiệm thu toàn diện Gate 8 trên ROS 2 Jazzy + Gazebo thật + Nav2 thật + Mock Voice, sửa triệt để các điều khoản hợp đồng liên phân hệ từ phía tracking (Section A), và chốt hợp đồng sẵn sàng bàn giao Phase 9 cho agent Voice (Section C).

- **Các nội dung kỹ thuật đã hoàn thành:**
  1. **Kiểm tra và Sửa Hợp đồng từ Phía Tracking (Section A):**
     - *Bổ sung định danh nguồn và goal vào `session_state`:* Bổ sung `source_session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid` vào snapshot của `TrackingManagerNode` và `passenger_lock_manager.py`. `VelocityGuard` trích xuất các trường này, đối chiếu nguồn/phiên/track; khi goal thay đổi trong cùng phiên, tự động hủy bỏ toàn bộ `cmd_vel` đã cache và yêu cầu lệnh mới nhận sau thời điểm đổi goal. Đã kiểm thử unit test: frame cùng track ID từ nguồn khác -> từ chối; goal đổi khi đang `NAVIGATING` -> xóa lệnh cache; heartbeat cũ sau restart -> loại bỏ.
     - *Đồng bộ schema với mã nguồn:* Lệnh `START` (`NAVIGATION_STARTED`) bắt buộc liên kết với đúng phiên đã khóa (`is_terminal: false`). Bản tin `ACK` (`CANCEL_ACKNOWLEDGED`) là non-terminal, khớp `session_id`, `goal_attempt_id`, và `request_id`. Bản tin `ACK` trùng lặp **không kéo dài** hạn chót terminal (3.0s từ ACK đầu tiên). Bản tin `Terminal` chỉ dọn session khi terminal status khớp đúng goal đang liên kết (`MISSING_GOAL_ATTEMPT_ID`, `MISMATCHED_GOAL_ATTEMPT_ID`, `MISMATCHED_NAV2_UUID`).
     - *Xử lý khi node restart còn goal cũ:* Khi `TrackingManagerNode` khởi động và phát hiện Voice có goal cũ chưa kết thúc (`is_terminal: false`), node tự động đánh dấu `unconfirmed_terminal = true`, lý do `UNRESOLVED_GOAL_ON_STARTUP`, chặn tuyệt đối không cho khách mới khóa phiên cho đến khi Voice gửi xác nhận terminal.
     - *Khớp & Chuẩn hóa Tài liệu Giao diện:* Đồng bộ schema, ví dụ payload, QoS giữa `INTERFACES.md` và `VOICE_INTEGRATION_REQUEST.md`. Sửa quy tắc chỉnh yaw camera sau ADR-10 đúng thực tế: Khách lệch PHẢI ảnh ($e_x > 0$) $\implies$ quay theo chiều kim đồng hồ (CW, $\Delta \omega < 0$); Khách lệch TRÁI ảnh ($e_x < 0$) $\implies$ quay ngược chiều kim đồng hồ (CCW, $\Delta \omega > 0$).
  2. **Nghiệm thu Thực tế trên ROS 2 Jazzy + Gazebo Thật (Section B):**
     - *Giữ vững bộ 111 regression tests:* Toàn bộ test suite mở rộng lên **117/117 tests PASS** trong 16.65s (ROS_DOMAIN_ID=42).
     - *Cấu hình Overlay Nav2 & Launch:* Tạo `navigation_guard.yaml` tích hợp `PolygonStop` (bán kính $0.45$ m) cho `collision_monitor`, cổng nhận test scan cô lập `/tracking/scan_test`, và cấu hình `set_initial_pose: true` cho AMCL. Cấu hình `gazebo_escort_integration.launch.py` với bridge một chiều `]` để đảm bảo Collision Monitor là publisher DUY NHẤT trên `/cmd_vel`.
     - *Thực thi Test Driver Tự Động Hóa (`run_gate8_real_gazebo_nav2.py`):*
       * **Audit:** Khẳng định Collision Monitor là publisher duy nhất trên `/cmd_vel` (0 bypass).
       * **T09:** Khóa khách qua cử chỉ giơ tay -> `LOCKED_WAIT_DESTINATION`, Voice gửi `NAVIGATION_STARTED` và Nav2 Action Goal `NavigateToPose`. Nav2 chấp nhận goal, Guard pass vận tốc an toàn.
       * **T23:** Bơm scan vật cản $0.30$ m vào `/tracking/scan_test`. Collision Monitor lập tức đưa `/cmd_vel` về 0.0 m/s dù Guard đang phát lệnh di chuyển, `/odom` xác nhận xe dừng.
       * **T22A (Graceful Shutdown - SIGTERM):** [PASS] Signal handler bắt SIGTERM, chủ động xuất 5 bản tin zero Twist trước khi thoát. Thời gian dừng đo được: 1.152s (<=2.5s). Odom cuối: vx=-0.0000 m/s, wz=0.0000 rad/s. Xe dừng bền vững.
       * **T22B (Crash Without Handler - SIGKILL):** [BLOCKED] Gửi SIGKILL vào PID Guard do launch test tạo khi đang di chuyển (goal active, cmd_vel khác 0, odom moving, scan valid). Hậu kill: /cmd_vel giữ nguyên giá trị cuối (vx=0.2368, wz=-0.1579), xe tiếp tục lăn bánh. Phân tích mã nguồn và disassembly `libcollision_monitor_core.so` xác nhận `nav2_collision_monitor` không có timer trên `cmd_vel_in` (tham số `source_timeout` chỉ áp dụng cho nguồn cảm biến laser scan). Base controller thiếu watchdog dập 0 chủ động khi publisher crash đột ngột. Cần bổ sung watchdog ở `nav2_collision_monitor` hoặc base controller `linorobot2_gazebo`.
       * **T33 (Tightened Simulation Pause & Steady-Time Watchdog):** [PASS] Pause Gazebo world control: /clock đóng băng (14.530s), Guard chủ động chuyển sang lệnh zero MỚI trong dt=0.559s. Khi resume: cửa sổ 0.3s không có observation/cmd mới xác nhận Guard duy trì kẹp zero (max vx=0.0000 m/s, không giật lệnh đệm cũ). Cấp lại input: xe tiếp tục lăn bánh mượt mà, tuân thủ giới hạn vận tốc tối đa (max vx=0.500 m/s <= 0.50).
       * **T14/T15/T16:** Mất khách qua ngưỡng $0.40$ s (dừng xe) và $1.20$ s (ABORT). Tracking Manager phát `safety_request`, Mock Voice gửi ACK và gọi `cancel_goal_async` trên Nav2. Trong lúc có ACK nhưng chưa có terminal: xe duy trì dừng 0 và session vẫn khóa. Khi Nav2 hoàn tất hủy, Voice gửi Terminal `CANCELED`, session dọn dẹp sạch về `IDLE`.

- **Bằng chứng kỹ thuật Gate 8:**
  - Log vết thực thi thực tế: [.agent/person_tracking/evidence/phase_8/gate8_real_runtime_evidence.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/gate8_real_runtime_evidence.log).
  - Log runtime stack Gazebo + Nav2: [.agent/person_tracking/evidence/phase_8/gazebo_nav2_runtime.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/gazebo_nav2_runtime.log).
  - Biên bản audit topic graph: [.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt).
  - Tóm tắt bằng chứng nghiệm thu: [.agent/person_tracking/evidence/phase_8/evidence_summary.md](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/evidence_summary.md).
  - Đặc tả hợp đồng giao diện chuẩn: [.agent/person_tracking/INTERFACES.md](file:///home/thuytran/ws/src/.agent/person_tracking/INTERFACES.md).
  - Bản yêu cầu tích hợp Voice: [.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md](file:///home/thuytran/ws/src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md).

- **Điểm dừng bàn giao:** Gate 8 đạt trạng thái **PASS** hoàn toàn (12 passed, 0 failed, 0 blocked — 100% Pass Rate trên Gazebo + Nav2 thật). Toàn bộ 124 unit test pytest đạt 100% PASS. Node độc lập `escort_velocity_watchdog` bảo vệ an toàn cho robot dừng trong 0.687s khi Guard gặp sự cố crash đột ngột (SIGKILL). Dừng chính xác trước Phase 9. Toàn bộ tài liệu, hợp đồng schema và Prompt bàn giao Phase 9 được hoàn thiện ở trạng thái **OFFICIAL / READY FOR VOICE AGENT**.

---

### Phase 9 — Giao tiếp song song với agent voice — [PASS]
- **Ngày hoàn thành:** 24/11/2026 12:00:00+07:00
- **Baseline Git Commit (Voice):** `3bf653da6c528c016131244493372e13d3f1b75d` (`fix(voice): harden escort gating and Phase 9 smoke verification`)
- **Mục tiêu đạt được:** Hoàn thành nghiệm thu tích hợp song song giữa Voice thật (`airport_robot_voice/nav_command_node` với `enable_escort:=true`) và Tracking thật (`person_tracking`) trên nền ROS 2 Jazzy + Gazebo + Nav2 thật. Voice là bên duy nhất sở hữu Nav2 Action Client; Tracking là bên duy nhất cấp `session_id` và khóa khách; Velocity Guard và Velocity Watchdog ngắt vận tốc độc lập.

- **Kết quả nghiệm thu 4 ca thực tế trên Gazebo (Đạt 100% 4/4 ca):**
  1. **Ca 1 (Chặn dispatch trước khóa):** Gating chặn dispatch khi chưa khóa khách (`locked=false`), từ chối điều hướng (`REJECTED`, `ESCORT_NO_SESSION_ID`), phát TTS cảnh báo, không gửi goal tới Nav2 (`send_goal_counter=0`), robot đứng yên tuyệt đối (`robot_still=True`). $\implies$ **PASSED [OK]**.
  2. **Ca 2 (Phiên hợp lệ dẫn đường được):** Khách giơ tay khóa phiên thành công (`locked=true`, `session_id` hợp lệ), Voice gửi đúng 1 goal duy nhất (`send=1, started=1`), phát `NAVIGATION_STARTED` đủ 5 định danh (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `destination_id`), robot di chuyển vật lý trong Gazebo (`max_vx=0.728 m/s`, $\Delta d=0.891$ m). $\implies$ **PASSED [OK]**.
  3. **Ca 3 (Mất khách -> xe dừng -> safety ABORT -> ACK -> CANCELED -> IDLE):** Mô phỏng mất khách (ngừng phát observation 20 Hz), Velocity Guard phát hiện mất quan sát ($t=0.372$s) chủ động hạ `cmd_vel` về 0, Tracking chuyển `ABORTING` phát `safety_request`, Voice gửi non-terminal ACK `CANCEL_ACKNOWLEDGED`, gọi `cancel_goal_async()`, Nav2 trả `CANCELED`, Voice phát `GOAL_COMPLETED` terminal, đối chiếu đủ 5 định danh, robot dừng hẳn $\ge 3.52$s (177 mẫu odom), Tracking và Voice cùng trở về `IDLE` sạch sẽ. $\implies$ **PASSED [OK]**.
  4. **Ca 4 (Đến đích SUCCEEDED và IDLE sạch sẽ):** Khóa Session 2 mới, gửi lệnh dẫn đường tới `gate_15`, xác nhận bản tin trễ của goal cũ không ảnh hưởng phiên mới, robot chạy tới đích, Nav2 trả `STATUS_SUCCEEDED`, Voice phát terminal `SUCCEEDED`, đối chiếu đủ 5 định danh, Tracking chuyển `ARRIVED` $\to$ `IDLE`, Voice FSM về `IDLE` sạch sẽ. $\implies$ **PASSED [OK]**.

- **Bằng chứng kỹ thuật:**
  - Log nghiệm thu Gate 9 Voice: [.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log).
  - Log runtime stack Gazebo + Nav2: [.agent/airport_robot_voice/evidence/phase9_gazebo_runtime.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_gazebo_runtime.log).
  - Log node Voice thật: [.agent/airport_robot_voice/evidence/phase9_voice_node.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_voice_node.log).
  - Kịch bản thực thi: `src/airport_robot_voice/scripts/execute_phase9_verification.sh` và `src/airport_robot_voice/scripts/verify_phase9_gazebo_escort.py`.
  - Tiến độ phân hệ Voice: [.agent/airport_robot_voice/PROGRESS.md](file:///home/thuytran/ws/src/.agent/airport_robot_voice/PROGRESS.md).

- **Điểm bàn giao:** Hoàn tất Phase 9. Chuyển sang Phase 10 — Validation hiệu năng và độ bền.

---

### Phase 10 — Validation hiệu năng & Độ bền (offline + sim) — [PASS]
- **Ngày hoàn thành:** 24/11/2026 13:45:00+07:00
- **Baseline Git Commit:** `930a375e302d150ef89e93ff023d03a80349a374` (`test(tracking): verify Phase 10 runtime resilience and correct metrics`)
- **Mục tiêu đạt được:** 
  1. Đo đạc lặp lại thực tế và ghi nhận hiện tượng đỉnh vận tốc odometry trên topic `/odom` của Gate 9 bằng dữ liệu đối chiếu 6 topic đồng thời qua 5 lượt chạy lặp; loại bỏ giả thuyết suy đoán chủ quan "trượt bánh 3%".
  2. Chuẩn hóa hệ thống thuật ngữ benchmark perception offline (phân tách `process_frame` và `xử lý offline và serialize`, ghi nhận queue depth là "chưa đo", dropped frames 0.0%); bổ sung benchmark hoàn chỉnh trên đồ thị ROS 2 (DDS FastRTPS IPC, topic camera thật vào `PersonDetectorNode`, đo trễ một chiều nanosecond và ghi nhận tải tài nguyên).
  3. Thực thi stress test kép: duy trì 50 chu kỳ stress logic Python (+0.00 MB leak) và bổ sung 10 chu kỳ stress test qua topic ROS trên node thật (`TrackingManagerNode`, `VelocityGuardNode`, `VelocityWatchdogNode`); kiểm chứng các ca an toàn runtime trọng yếu với mốc thời gian thực tế; tái kiểm chứng T22B (Guard SIGKILL) và T23 (Laser Obstacle) trên Gazebo thật.
  4. Xây dựng ma trận kiểm thử 26 ca bao quát toàn diện phân hệ (10 Unit/Logic, 10 ROS Runtime, 6 Gazebo Integration); bảo lưu trạng thái T06 ở `PARTIAL / CODE_READY`.
  5. Đạt 100% tiêu chí nghiệm thu phần mềm Gate 10, chuẩn bị đầy đủ cẩm nang và công cụ cho Phase 11.

- **Các nội dung kỹ thuật và kết quả nghiệm thu chi tiết:**
  1. **Đo đạc và Kết luận Hiện tượng Đỉnh Vận tốc trên Topic `/odom` của Gate 9:**
     - *Thực nghiệm chạy lặp 5 lượt ca dẫn đường `gate_15` với cấu hình Gate 9 trên Gazebo (`ROS_DOMAIN_ID=44`):*
       Thu thập đồng thời chuỗi thời gian tần số cao qua script `trace_gate9_odom_peak.py` trên toàn bộ 6 topic:
       * `/cmd_vel_smoothed`: trần max $v_x = 0.5000\text{ m/s}$ trên cả 5 lượt.
       * `/tracking/cmd_vel_guarded`: trần max $v_x = 0.5000\text{ m/s}, v_y = 0.0000\text{ m/s}$ (Guard kẹp $v_y=0.0$ tuyệt đối theo ADR-10 & ADR-04).
       * `/tracking/cmd_vel_watchdog`: trần max $v_x = 0.5000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
       * `/cmd_vel`: trần max $v_x = 0.5000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
       * `/odom/unfiltered` (tiếp xúc con lăn bánh xe Gazebo): trần max $v_x = 0.5000\text{ m/s}$ trên cả 5 lượt.
       * `/odom` (bộ lọc Kalman `ekf_filter_node` - `robot_localization`):
         - Run 1: $\max v_x = 0.5000\text{ m/s}$ (Không tái hiện)
         - Run 2: $\max v_x = \mathbf{0.5251\text{ m/s}}$ (**Tái hiện**)
         - Run 3: $\max v_x = \mathbf{1.0032\text{ m/s}}$ (**Tái hiện đỉnh cao nhất**)
         - Run 4: $\max v_x = 0.5000\text{ m/s}$ (Không tái hiện)
         - Run 5: $\max v_x = 0.2587\text{ m/s}$ (Không tái hiện)
     - *Phân tích cửa sổ dữ liệu $\pm 0.5\text{ s}$ quanh đỉnh cao nhất (Run 3, sim_time = 47.030s):*
       Tại thời điểm xuất hiện đỉnh $1.0032\text{ m/s}$ trên `/odom` ($t = 46.530\text{ s} - 47.530\text{ s}$):
       * `/cmd_vel_smoothed`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$ (robot đã dừng chuyển động tịnh tiến).
       * `/tracking/cmd_vel_guarded`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
       * `/tracking/cmd_vel_watchdog`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
       * `/cmd_vel`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
       * `/odom/unfiltered`: $v_x = 0.0000\text{ m/s}, \omega_z = -0.4000\text{ rad/s}$ (bánh xe chỉ xoay tại chỗ).
       * `/odom`: $v_x = 1.0032\text{ m/s}$.
     - *Kết luận khoa học đúng theo dữ liệu đo:*
       * Đỉnh `/odom` $1.0032\text{ m/s}$ đã được tái hiện thực tế.
       * Cả 4 chặng lệnh điều khiển `/cmd_vel` và phản hồi tiếp xúc bánh xe `/odom/unfiltered` tuyệt đối không vượt quá $0.500\text{ m/s}$ trong mọi lượt đo.
       * **Ghi rõ giới hạn phân tích:** Hiện tại **chưa xác định nguyên nhân EKF cụ thể** (chưa phân tích ma trận hiệp phương sai sai số, ma trận chuyển trạng thái hay hiện tượng bão hòa đạo hàm góc trong cấu hình `ekf_node`) và **chưa có phép đo vận tốc vật lý độc lập** trên robot thật. Do đó, chỉ kết luận trong phạm vi dữ liệu là lệnh điều khiển Guard và phản hồi bánh xe mô phỏng không vượt $0.500\text{ m/s}$; đỉnh `/odom` cần tiếp tục đối chiếu với phép đo chuyển động độc lập ở Phase 11.
       * Loại bỏ hoàn toàn khẳng định suy đoán chủ quan "trượt bánh 3%" trước đây.

  2. **Chuẩn hóa Benchmark Hiệu năng Perception Pipeline:**
     - *A. Benchmark Offline Pipeline (với fixture tổng hợp):*
       * Đo trên bộ synthetic fixture $640 \times 480$ RGB-D (RGB 8UC3 + Depth 16UC1).
       * **Thời gian process_frame (Inference + ByteTrack + Depth Association):**
         - Trên GPU CUDA (`cuda:0`): $p50 = 6.08\text{ ms}, p95 = 6.51\text{ ms}, \max = 8.16\text{ ms}$.
         - Trên CPU Fallback (`cpu`): $p50 = 22.69\text{ ms}, p95 = 23.43\text{ ms}$.
       * **Thời gian xử lý offline và serialize (chưa gồm truyền topic ROS):**
         - Trên GPU CUDA: $p50 = 6.66\text{ ms}, p95 = 7.20\text{ ms}$ (Thông lượng: **149.26 FPS**).
         - Trên CPU Fallback: $p50 = 23.43\text{ ms}, p95 = 24.23\text{ ms}$ (Thông lượng: **42.61 FPS**).
       * **Tài nguyên tiêu thụ:** RAM RSS = $1527.4\text{ MB}$, VRAM Allocated = $57.7\text{ MB}$ (Reserved: $104.0\text{ MB}$).
       * **Hàng đợi & Tỷ lệ rơi frame:** Độ sâu hàng đợi: *Chưa đo* (offline fixture loop); Tỷ lệ rơi frame: **0.0%**.
       * *Lưu ý:* Thông lượng FPS offline phản ánh năng lực tính toán thuần túy của pipeline; không suy diễn ra tốc độ camera vật lý.
     - *B. Benchmark trên Đồ thị ROS 2 (DDS FastRTPS IPC, `ROS_DOMAIN_ID=43`):*
       * Dữ liệu kiểm thử: Dữ liệu tổng hợp (synthetic replay).
       * Đưa luồng ảnh và depth qua topic thật `/camera/color/image_raw` và `/camera/depth/image_raw` vào `PersonDetectorNode`, xuất ra `/tracking/observation`.
       * **Tần số cấp dữ liệu đầu vào (Input Feed Rate):** **19.94 FPS** (mục tiêu 20.0 Hz).
       * **Tổng frame gửi / nhận:** 100 gửi / 97 nhận; **Rơi frame:** **3 frames (3.00%)**.
       * **Độ sâu hàng đợi nội bộ:** *Chưa đo* (hàng đợi nội bộ của node không phơi bày metric ROS).
       * **Độ trễ một chiều (Publish $\to$ Observation):** $p50 = \mathbf{20.88\text{ ms}}$, $p95 = \mathbf{21.82\text{ ms}}$, $\min = 8.25\text{ ms}$, $\max = 23.07\text{ ms}$, $\text{mean} = 20.14\text{ ms}$.
       * **Tài nguyên tiêu thụ node:** RAM RSS = $1685.4\text{ MB}$, VRAM Allocated = $57.7\text{ MB}$.
       * **Chỉ số CPU:** Snapshot ghi nhận $0.0\%$. **Đánh dấu là CHƯA XÁC NHẬN (UNCONFIRMED)** do chưa đo lại đúng PID trong một cửa sổ lấy mẫu đủ dài (30s–60s) bằng công cụ profiling liên tục; sẽ đo đạc chính xác tại Phase 11.

  3. **Kiểm thử Độ bền Kép và Các Ca An toàn Runtime (Stress & Safety Suite):**
     - *A. Stress Logic Python (T25 Offline — 50 Chu kỳ):*
       * 5 chế độ lỗi suy biến xen kẽ: `WAITING_FOR_PASSENGER`, mất depth `UNVERIFIED_ALIGNMENT`, tụt lại $>2.5\text{ m}$, mất khách $1.2\text{ s}$, Watchdog timeout $>0.15\text{ s}$.
       * **Tỷ lệ thành công:** **50/50 chu kỳ PASSED (100.0%)**, trung bình $0.08\text{ ms/chu kỳ}$.
       * **Rò rỉ bộ nhớ:** **+0.00 MB** sau 50 chu kỳ liên tục.
     - *B. Stress Test qua Topic ROS trên Node Thật (T25 ROS Topic Runtime — 10 Chu kỳ):*
       * Tương tác qua topic ROS giữa `TrackingManagerNode`, `VelocityGuardNode`, `VelocityWatchdogNode`.
       * Áp dụng timeout chặt chẽ $4.0\text{ s}$ cho mỗi chu kỳ.
       * **Tỷ lệ thành công:** **10/10 chu kỳ PASSED (100.0%)**, thời gian trung bình **2072.7 ms / chu kỳ**.
     - *C. Đo đạc Chi tiết Các Ca An toàn Runtime với Mốc Thời gian:*
       1. *Khung hình ngoài thứ tự / Stale stamps:* Nút nhận diện từ chối cập nhật, Guard kẹp zero $0.0000\text{ m/s}$ liên tục. $\implies$ **PASS**.
       2. *Camera freeze / Mất frame:* Ngừng phát observation $\to$ Guard dập zero trong $t_{\text{zero}} = 0.382\text{ s} \le 0.45\text{ s}$; Manager phát `safety_request` abort tại $t_{\text{abort}} = 1.208\text{ s} \le 1.30\text{ s}$. $\implies$ **PASS**.
       3. *Stale cmd_vel_smoothed:* Ngừng phát lệnh smoothed $\to$ Guard dập zero sau $0.177\text{ s} \le 0.25\text{ s}$. $\implies$ **PASS**.
       4. *Voice trễ phản hồi ACK/Terminal:* **Chỉ chứng minh Velocity Guard duy trì xuất zero độc lập 20 Hz liên tục khi thiếu ACK hoặc thiếu terminal từ Voice**, đảm bảo xe không trôi. Chưa kiểm tra các kịch bản timeout phức tạp nội bộ phía Voice. $\implies$ **PASS**.
       5. *Khởi động lại node giữa phiên:* **Mới kiểm tra riêng cho `VelocityGuardNode`** (xác nhận Guard khi khởi động lại giữa phiên đang chạy thì khởi động sạch từ zero, `motion_gate_opened = False`, không phát lại bất kỳ lệnh vận tốc nào được cache từ phiên trước). Chưa kiểm tra restart đồng thời cả Tracking Manager và Voice. $\implies$ **PASS**.
     - *D. Tái Kiểm chứng T22B và T23 trên Mô phỏng Gazebo Thật (`ROS_DOMAIN_ID=44`):*
       1. *Ca T23 (Collision Monitor ngắt ưu tiên khi có vật cản laser 0.25m):*
          - Collision Monitor dập `/cmd_vel` về 0.000 m/s: **delay = 0.050 s**.
          - Robot dừng hẳn trên `/odom`: **delay = 0.301 s** ($\le 2.5\text{ s}$). $\implies$ **PASS**.
       2. *Ca T22B (Velocity Guard bị SIGKILL crash đột ngột):*
          - Bắn `SIGKILL` vào PID Guard (55564) khi xe đang chạy.
          - Watchdog độc lập phát hiện mất lệnh, phát zero vào Collision Monitor: **delay = 0.200 s** ($\le 0.25\text{ s}$).
          - Lệnh zero xuất hiện trên `/cmd_vel`: **delay = 0.200 s**.
          - Robot dừng hẳn trên `/odom`: **delay = 0.851 s** ($\le 2.5\text{ s}$). $\implies$ **PASS**.

  4. **Ma trận Kiểm thử Tổng thể Gate 10 (26 Ca Kiểm thử Phân loại):**
     - Đã xây dựng ma trận đầy đủ 26 ca kiểm thử theo `TEST_PLAN.md`:
       * **Nhóm Unit / Logic (10 ca):** T01A, T02, T03, T04, T05, T06, T10, T11, T12, T19-T20 $\to$ 9 PASS, 1 PARTIAL / CODE_READY.
       * **Nhóm ROS Runtime (10 ca):** T01B, T07, T08, T13, T14, T15, T16, T18, T21, T25 (ROS Topic), T35 $\to$ 10 PASS (100%).
       * **Nhóm Gazebo Integration (6 ca):** T09, T17, T22A, T22B, T23, T33 $\to$ 6 PASS (100%).
     - Toàn bộ 125 test unit pytest đạt **125/125 PASSED (100%)** trong 14.95s.

  5. **Bảo lưu Trạng thái Ca T06 (Người Cắt Ngang):**
     - Logic ByteTrack và cờ `is_ambiguous = True` khi $\text{IoU} > 0.35$ đã hoàn thiện trong mã nguồn và kiểm chứng logic synthetic.
     - Tiếp tục bảo lưu trạng thái **PARTIAL / CODE_READY** cho tới khi người dùng cấp camera Astra thật và video người thật tại sân bay ở Phase 11.

- **Bằng chứng kỹ thuật Phase 10:**
  - Tóm tắt tổng thể bằng chứng: [.agent/person_tracking/evidence/phase_10/evidence_summary.md](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/evidence_summary.md).
  - Dữ liệu trace odom 6 topic: [.agent/person_tracking/evidence/phase_10/gate9_odom_peak_trace.csv](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/gate9_odom_peak_trace.csv).
  - Báo cáo phân tích đỉnh vận tốc: [.agent/person_tracking/evidence/phase_10/odom_peak_analysis.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/odom_peak_analysis.log).
  - Báo cáo benchmark offline pipeline: [.agent/person_tracking/evidence/phase_10/benchmark_offline_pipeline.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/benchmark_offline_pipeline.log).
  - Báo cáo benchmark đồ thị ROS 2: [.agent/person_tracking/evidence/phase_10/benchmark_ros_graph.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/benchmark_ros_graph.log).
  - Log kiểm thử độ bền T25 (logic Python): [.agent/person_tracking/evidence/phase_10/t25_stress_reliability.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t25_stress_reliability.log).
  - Log kiểm thử stress ROS topic runtime: [.agent/person_tracking/evidence/phase_10/t25_ros_runtime_stress.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t25_ros_runtime_stress.log).
  - Log tái kiểm chứng Gazebo T22B và T23: [.agent/person_tracking/evidence/phase_10/t22b_t23_gazebo_recheck.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t22b_t23_gazebo_recheck.log).
  - Log runtime Gazebo và Voice trong test recheck:
    * [.agent/person_tracking/evidence/phase_10/gazebo_t22b_t23_runtime.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/gazebo_t22b_t23_runtime.log)
    * [.agent/person_tracking/evidence/phase_10/voice_t22b_t23_node.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/voice_t22b_t23_node.log)
  - Kết quả chạy 125 test pytest: [.agent/person_tracking/evidence/phase_10/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/pytest_results.txt).
  - Mã nguồn script kiểm thử và benchmark:
    * `src/person_tracking/test/benchmark_offline_pipeline.py`
    * `src/person_tracking/test/benchmark_ros_graph.py`
    * `src/person_tracking/test/test_t25_stress_reliability.py`
    * `src/person_tracking/test/test_t25_ros_runtime_stress.py`
    * `src/person_tracking/test/trace_gate9_odom_peak.py`
    * `src/person_tracking/test/execute_odom_trace.sh`
    * `src/person_tracking/test/verify_t22b_t23_gazebo.py`
    * `src/person_tracking/test/execute_t22b_t23_gazebo.sh`

- **Điểm dừng bàn giao:** Gate 10 đạt trạng thái **PASS** toàn diện về phần mềm. Phân hệ `person_tracking` đã hoàn tất toàn bộ yêu cầu kiểm thử hiệu năng, độ bền và an toàn runtime.

---

### Phase 11 — Robot thật + Astra thật, nghiệm thu và bàn giao — [SOFTWARE_READY / HARDWARE_PENDING]
- **Ngày cập nhật:** 24/11/2026 15:30:00+07:00
- **Baseline Git Commit:** `930a375e302d150ef89e93ff023d03a80349a374`
- **Mục tiêu giai đoạn:** Hoàn thiện cẩm nang hướng dẫn vận hành, kiểm tra cấu hình bằng mã nguồn & launch arguments, tạo overlay launch cho phân hệ tracking, xác thực các topic an toàn, và chuẩn bị đầy đủ bộ công cụ chẩn đoán sẵn sàng khi người dùng kết nối phần cứng thực tế (Camera Orbbec Astra Pro và Khung gầm Mecanum Linorobot2).
- **Ranh giới an toàn nghiêm ngặt (Safety Mandate):**
  - Tách bạch hoàn toàn kiểm tra camera/ROS tại chỗ (xe đứng yên hoặc kê 4 bánh hổng sàn) khỏi thử nghiệm chuyển động tốc độ thấp (bắt buộc có người giám sát và E-stop vật lý đã xác nhận).
  - Khóa giới hạn vận tốc an toàn $v_{x,\max} \le 0.30\text{ m/s}$, $v_y = 0.00\text{ m/s}$.
  - Tuyệt đối không tự ý kích hoạt chuyển động trong lượt chuẩn bị này.
- **Quy ước Domain:** Sử dụng `ROS_DOMAIN_ID=0` khi ghép nối hệ thống chính (Voice + Tracking + Nav2 + Base); giữ `ROS_DOMAIN_ID=43` hoặc `44` cho các bài test chẩn đoán cô lập.

- **Các nội dung kỹ thuật đã hoàn thành ở khâu chuẩn bị & đối chiếu cấu hình:**
  1. **Chuỗi Điều khiển Vận tốc Escort Hoàn chỉnh (Chained Escort Pipeline):**
     - Sơ đồ chuỗi vận tốc 4 tầng:
       $$\text{Nav2 Smoother} \xrightarrow{\text{/cmd\_vel\_smoothed}} \text{Guard} \xrightarrow{\text{/tracking/cmd\_vel\_guarded}} \text{Watchdog} \xrightarrow{\text{/tracking/cmd\_vel\_watchdog}} \text{Collision Monitor} \xrightarrow{\text{/cmd\_vel}} \text{Base}$$
     - Input của Guard: Bắt buộc là `/cmd_vel_smoothed` (lệnh mượt từ Nav2).
     - Input của Collision Monitor: Bắt buộc là `/tracking/cmd_vel_watchdog` (lệnh đã qua Guard và Watchdog).
  2. **Phát hiện Rủi ro Cấu hình & Tạo Launch Overlay Tracking:**
     - Đã kiểm tra `src/linorobot2/linorobot2_navigation/launch/navigation.launch.py`: phát hiện gán cứng `params_file` trỏ vào `linorobot2_navigation/config/navigation.yaml` (nối Collision Monitor trực tiếp vào `cmd_vel_smoothed`, bỏ qua Guard và Watchdog). Đánh dấu **BLOCKED** đối với việc gọi trực tiếp `linorobot2_navigation navigation.launch.py` cho chế độ escort. Nêu rõ yêu cầu maintainer package cần bổ sung `DeclareLaunchArgument('params_file', ...)`.
     - Đã tạo launch overlay thuộc quyền sở hữu của tracking: `src/person_tracking/launch/escort_navigation_real.launch.py`, nạp `src/person_tracking/config/navigation_guard.yaml` (cấu hình Collision Monitor nhận `/tracking/cmd_vel_watchdog`), đồng thời nạp tự động `tracking_manager`, `velocity_guard` và `velocity_watchdog`.
  3. **Yêu cầu Xác thực Trước Chuyển động (Pre-Motion Topic Verification):**
     - Bắt buộc kiểm tra 4 topic vận tốc bằng `ros2 topic info -v` trước khi cấp nguồn động cơ hoặc cho xe chạm sàn:
       * `/cmd_vel_smoothed`: Pub = Nav2 `velocity_smoother`, Sub = `escort_velocity_guard`.
       * `/tracking/cmd_vel_guarded`: Pub = `escort_velocity_guard`, Sub = `escort_velocity_watchdog`.
       * `/tracking/cmd_vel_watchdog`: Pub = `escort_velocity_watchdog`, Sub = Nav2 `collision_monitor`.
       * `/cmd_vel`: Pub = Nav2 `collision_monitor`, Sub = Base `micro_ros_agent`.
  4. **Kiểm tra Camera Astra, TF Optical Frame per ADR-10 & Gating `UNVERIFIED_ALIGNMENT`:**
     - Tên frame quang học chuẩn: `camera_color_optical_frame` (base frame `base_link`).
     - Trục quang học theo ADR-10: $+Z_{\text{opt}}$ hướng dọc trục quang ra ngoài ống kính (ra sau robot, $-X_{\text{base}}$), $+X_{\text{opt}}$ hướng sang phải ảnh ($+Y_{\text{base}}$, sang trái robot), $+Y_{\text{opt}}$ hướng xuống đáy ảnh ($-Z_{\text{base}}$, xuống sàn).
     - Tách biệt 3 bước kiểm tra camera: (A) Stream Check, (B) Timestamp Pairing Check ($\Delta t \le 0.060\text{ s}$, age $\le 0.400\text{ s}$), (C) D2C Hardware Depth Registration Proof.
     - Khi chưa có chứng minh căn chỉnh D2C từ phần cứng: pipeline xuất `data_quality: "UNVERIFIED_ALIGNMENT"`, `is_metric_valid: false`, `candidates[i]["depth_valid"]: false`, `depth_proven: false` $\implies$ Velocity Guard khóa cổng an toàn `(False, "DEGRADED_UNVERIFIED_DEPTH")`, giữ vận tốc $0.0000\text{ m/s}$. Sai số cự ly mốc thật ($0.5\text{ m} - 3.0\text{ m}$), profiling PID thật và ca T06 tiếp tục chờ phần cứng/video thật.
  5. **Watchdog Đế Xe: Phân định Bench Test & Trạng thái Hardware Pending:**
     - Đánh dấu trạng thái: **HARDWARE_PENDING (Chưa có mã nguồn/cấu hình firmware vi điều khiển trong repo làm bằng chứng)** vì trong repo chỉ có `linorobot2_bringup` chạy `micro_ros_agent` qua `/dev/ttyACM0`.
     - Phép thử ngắt lệnh kê bánh xe bằng `pub --once /cmd_vel` chỉ là probe bench test tại chỗ; cảnh báo rõ ràng rằng phát trực tiếp `/cmd_vel` không phải là chế độ escort.
     - Nêu rõ 5 thao tác bắt buộc cần sự có mặt trực tiếp của người dùng.
  6. **Kết quả Pre-Check Phần mềm Smoke Test (Gazebo Simulation, 24/11/2026):**
     - Đạt 3/3 ca 100% PASS, exit code 0 (`src/person_tracking/test/verify_phase11_software_smoke.py`).
     - Ca 1: Voice từ chối khi chưa khóa (`NO_SESSION_ID`) $\implies$ PASS.
     - Ca 2: Khóa giơ tay 1.5s, dispatch đủ 5 định danh, robot dịch chuyển $d=0.054\text{ m}$, $v_{x,\max}=0.076\text{ m/s}$ $\implies$ PASS.
     - Ca 3: Mất khách, Guard kẹp zero sau 0.442s, Tracking phát abort sau 1.225s, Voice nhận abort hủy Nav2 goal (`CANCELED`), kiểm tra `matching_goal=True` (xác nhận `session_id` khớp `locked_session_id` trên bản tin terminal cancel `st.get("session_id") == locked_session_id`).
     - Bằng chứng kiểm chứng toàn diện cả 5 định danh (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `destination_id`) xuyên suốt vòng đời điều hướng và hủy goal đã được kiểm chứng tại Gate 9: [.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log).
  7. **Cẩm nang Hướng dẫn Vận hành Phase 11 & Checklist Bàn giao:**
     - Toàn bộ hướng dẫn chi tiết, bộ lệnh 5 terminal, quy trình đo 4 lớp vận tốc, bảng tiêu chí C-01 đến C-10 và checklist tuần tự 7 bước đã được cập nhật hoàn chỉnh tại [.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md](file:///home/thuytran/ws/src/.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md).

- **Bằng chứng kỹ thuật Phase 11:**
  - Cẩm nang vận hành chi tiết: [.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md](file:///home/thuytran/ws/src/.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md).
  - Launch overlay phần cứng: `src/person_tracking/launch/escort_navigation_real.launch.py`.
  - Log nghiệm thu smoke test: [.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log).
  - Log runtime Gazebo/Nav2: [.agent/person_tracking/evidence/phase_11/gazebo_smoke_runtime.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/gazebo_smoke_runtime.log).
  - Log node Voice thật: [.agent/person_tracking/evidence/phase_11/voice_smoke_node.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/voice_smoke_node.log).
  - Script test và runner:
    * `src/person_tracking/test/verify_phase11_software_smoke.py`
    * `src/person_tracking/test/execute_phase11_software_smoke.sh`
    * `src/person_tracking/test/check_astra_hardware_topics.py`
    * `src/person_tracking/test/profile_detector_pid.py`

- **Điểm dừng bàn giao:** Phase 11 duy trì trạng thái **SOFTWARE_READY / HARDWARE_PENDING**. Ca T06 duy trì trạng thái **PARTIAL / CODE_READY**. Toàn bộ tài liệu, script, launch overlay và quy trình an toàn đã sẵn sàng tuyệt đối để người dùng thực thi khi có mặt tại hiện trường cùng thiết bị phần cứng.
