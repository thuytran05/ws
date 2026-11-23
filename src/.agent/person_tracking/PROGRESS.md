# PERSON TRACKING — TIẾN ĐỘ THỰC HIỆN VÀ THEO DÕI GIAI ĐOẠN

**Dự án:** Robot dẫn đường hành khách sân bay (ROS 2 Jazzy, Mecanum, Camera Astra sau)  
**Phân hệ:** `person_tracking`  
**File theo dõi duy nhất của phân hệ:** `.agent/person_tracking/PROGRESS.md`  
**Cập nhật lần cuối:** 23/11/2026 — Phase 1 Complete  

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
| **Phase 8** | End-to-end ROS 2 integration (không sửa Gazebo) | **PASS** | 23/11/2026 | `.agent/person_tracking/evidence/phase_8/` |
| **Phase 9** | Giao tiếp song song với agent voice | **NOT_STARTED** | — | — |
| **Phase 10** | Validation hiệu năng & Độ bền (offline + sim) | **NOT_STARTED** | — | — |
| **Phase 11** | Robot thật + Astra thật, nghiệm thu bàn giao | **NOT_STARTED** | — | — |

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
- **Ngày/giờ hoàn thành:** 23/11/2026 18:30:00+07:00
- **Baseline Git Commit:** `6ed2287989b6e43fad60a4f8d03f55412ad500b8`
- **Mục tiêu đạt được:** Sửa triệt để toàn bộ 4 lỗi Guard/FSM (Section A), thiết lập kịch bản launch tái lập được (Section B) trên ROS 2 Jazzy tích hợp Gazebo headless + Nav2 thật + Mock Voice + Velocity Guard + Collision Monitor, kiểm chứng chuỗi topic an toàn tuyệt đối không bypass `/cmd_vel`, và nghiệm thu 100% ma trận kiểm thử 11 ca tự động hóa.

- **Các nội dung kỹ thuật đã hoàn thành:**
  1. **Sửa Lỗi Guard / FSM (Section A):**
     - *A.1 Observation Gating:* `VelocityGuard.update_observation()` xác thực strict stamp/sequence, session binding, `is_confirmed`, `is_ambiguous`, `depth_valid`. Frame rỗng, lặp stamp, khác source tuyệt đối không gia hạn seen time. Nếu không có observation hợp lệ từ đầu $\implies$ bắt buộc xuất zero.
     - *A.2 Command Revocation & Boundary Stop:* Hủy quyền dùng `cmd_vel` cũ khi WAIT/ABORT/FAULT, stale obs, hoặc đổi session. Khi observation hồi phục từ stale, gate mở lại yêu cầu lệnh mới nhận sau thời điểm mở (`CMD_RECEIVED_BEFORE_GATE_OPENED`). Điều kiện biên $d \ge 2.5$ m: xe không được yaw/quay khi khoảng cách yêu cầu dừng ($v_x = 0, \omega = 0$). Kiểm tra NaN/Inf trên mọi trường điều khiển -> kẹp 0 an toàn.
     - *A.3 Closed-loop Yaw Trimming Sign:* Kiểm chứng qua `test_closed_loop_rear_camera_yaw_geometry` dùng mô hình hình học camera sau thật (`RearCameraGeometry`): Khách lệch PHẢI ảnh ($e_x > 0$) $\implies$ trim CW ($\Delta \omega < 0$). Khách lệch TRÁI ảnh ($e_x < 0$) $\implies$ trim CCW ($\Delta \omega > 0$). Triệt tiêu offset ở cả 2 phía. Đảo conflict resolution tại mép: Nav2 quay ngược hướng trim -> stop/wait.
     - *A.4 Steady Clock & ACK/Terminal ID Binding:* Sử dụng `Clock(clock_type=ClockType.STEADY_TIME)` cho Guard timer (20 Hz) và FSM heartbeat/watchdog (10 Hz), độc lập hoàn toàn với việc pause `/clock`. Phân biệt confirmed `OUT_OF_FOV` (ở mép + có trong `lost_track_ids` -> ABORT ngay) với suspected loss ở mép (chưa có trong `lost_track_ids` -> WAITING dừng sớm). Bỏ qua ACK/terminal sai ID. Khóa `unconfirmed_terminal` khi timeout sau ACK.

  2. **Triển khai Tích hợp Phase 8 & Ma trận Kiểm thử (Section B):**
     - *Cấu hình Overlay Nav2:* Tạo `src/person_tracking/config/navigation_guard.yaml` tái sử dụng toàn bộ thông số Nav2 của Linorobot2 nhưng cấu hình `collision_monitor` nhận đầu vào từ `/tracking/cmd_vel_guarded` và xuất sang `/cmd_vel`. Không chỉnh sửa file nguồn của Linorobot2.
     - *Kịch bản Launch Tái lập được:* Tạo `src/person_tracking/launch/gazebo_escort_integration.launch.py` khởi chạy Gazebo headless + Nav2 + Tracking Manager + Velocity Guard với `ROS_DOMAIN_ID` cô lập.
     - *Audit Chuỗi Điều khiển:* Xác nhận Collision Monitor là publisher DUY NHẤT trên topic `/cmd_vel`. Không có bất kỳ bypass nào từ Nav2 controller, Velocity Smoother, Velocity Guard, hay teleop.
     - *Bộ Kiểm thử Nghiệm thu Phase 8 (`test_phase8_gazebo_nav2_integration.py`):*
       * `test_phase8_audit_velocity_chain_and_sole_publisher`: PASS.
       * `test_phase8_t09_normal_escort_pass_through`: PASS ($v_x = 0.35$ m/s, $\omega = 0.05$ rad/s).
       * `test_phase8_t10_distance_scaling_and_hysteresis_stop`: PASS ($>2.0$m giảm tốc, $\ge 2.5$m dừng hẳn cả $v_x$ và $\omega$, $\le 2.2$m tiếp tục).
       * `test_phase8_t11_close_passenger_safety`: PASS ($<1.2$m duy trì tốc độ, không vượt tốc, không lùi xe).
       * `test_phase8_t12_rear_camera_yaw_trimming_and_conflict`: PASS (trim CW/CCW đúng dấu, giảm 50% ở mép, conflict stop khi Nav2 rẽ ngược).
       * `test_phase8_t14_t15_loss_grace_and_abort_with_mock_voice_cancel`: PASS (mất $<0.40$s tiếp tục, mất $>1.20$s abort dừng xe + Mock Voice cancel).
       * `test_phase8_t16_voice_unresponsive_independent_zero`: PASS (Mock Voice mất kết nối -> Guard giữ 0 độc lập).
       * `test_phase8_t23_collision_monitor_priority_over_guard`: PASS (bơm vật cản laser scan $0.40$m -> Collision Monitor ngắt `/cmd_vel` về 0 ngay khi Guard đang phát 0.35 m/s).
       * `test_phase8_t22_guard_process_termination_watchdog`: PASS (kill Guard -> Collision Monitor timeout ngắt `/cmd_vel`).
       * `test_phase8_t33_clock_pause_monotonic_watchdog`: PASS (dừng `/clock` mô phỏng -> steady clock timeout kẹp 0).
       * `test_phase8_t35_mecanum_lateral_stability`: PASS ($v_y = 0.0$ strictly).

- **Kết quả Kiểm thử Toàn diện (100% PASS):**
  - Toàn bộ **111/111 tests** của phân hệ `person_tracking` đều PASS trong 16.25s (ROS 2 Jazzy, ROS_DOMAIN_ID=42).
  - Không có bất kỳ regression nào trong toàn bộ test suite.

- **Bằng chứng kỹ thuật:**
  - Build overlay: [.agent/person_tracking/evidence/phase_8/colcon_build.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/colcon_build.log).
  - Kết quả kiểm thử pytest tổng thể: [.agent/person_tracking/evidence/phase_8/pytest_results.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/pytest_results.txt).
  - Biên bản audit topic graph: [.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/topic_graph_audit.txt).
  - Tóm tắt bằng chứng nghiệm thu: [.agent/person_tracking/evidence/phase_8/evidence_summary.md](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_8/evidence_summary.md).

- **Điểm dừng bàn giao:** Dừng chính xác trước Phase 9 (Giao tiếp song song với agent voice). User tự commit và push lên Git.
