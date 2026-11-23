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
| **Phase 3** | Multi-person perception, pose & Tracking ID | **NOT_STARTED** | — | — |
| **Phase 4** | Giơ tay, chọn khách gần nhất & Khóa phiên | **NOT_STARTED** | — | — |
| **Phase 5** | RGB-D, depth & Hình học camera sau | **NOT_STARTED** | — | — |
| **Phase 6** | Session FSM, timeout & Giao tiếp voice | **NOT_STARTED** | — | — |
| **Phase 7** | Velocity guard, mecanum & Watchdog | **NOT_STARTED** | — | — |
| **Phase 8** | End-to-end ROS 2 integration (không sửa Gazebo) | **NOT_STARTED** | — | — |
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
  - MỚI: `src/person_tracking/test/test_source_adapters.py` (11 unit tests).
  - MỚI: `src/person_tracking/test/test_perception_source_smoke.py` (1 smoke test tích hợp).
  - CẬP NHẬT: `src/person_tracking/setup.py` & `package.xml` (Đăng ký entrypoint `perception_source`, cài đặt `fixtures/*`).
  - CẬP NHẬT TÀI LIỆU: `TEST_PLAN.md` (tách T01A và T01B), `COMMANDS_TRACKING.md` (lệnh build, test, replay và rosbag đã kiểm chứng).

- **Kết quả kiểm thử kỹ thuật (100% PASS):**
  1. **Unit & Smoke Tests:** 12/12 passed trong 0.69s trên Python 3.12 (ROS 2 Jazzy).
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
