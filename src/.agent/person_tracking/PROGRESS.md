# PERSON TRACKING — TIẾN ĐỘ THỰC HIỆN VÀ THEO DÕI GIAI ĐOẠN

**Dự án:** Robot dẫn đường hành khách sân bay (ROS 2 Jazzy, Mecanum, Camera Astra sau)  
**Phân hệ:** `person_tracking`  
**File theo dõi duy nhất của phân hệ:** `.agent/person_tracking/PROGRESS.md`  
**Khởi tạo:** 20/11/2026  

---

## Bảng tổng quan trạng thái các giai đoạn (Phase Overview)

| Giai đoạn | Tên giai đoạn | Trạng thái | Ngày cập nhật | Bằng chứng |
|---|---|---|---|---|
| **Phase 0** | Khảo sát hiện trạng thực tế & Lập nền tài liệu AI | **PASS** | 20/11/2026 | `.agent/person_tracking/evidence/phase_0/audit_commands.log` |
| **Phase 1** | Quy tắc, skill, thiết kế & Kế hoạch kiểm thử | **NOT_STARTED** | — | Chờ lệnh kích hoạt |
| **Phase 2** | Source adapters, offline dataset & Test infrastructure | **NOT_STARTED** | — | — |
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
- **Ngày/giờ:** 20/11/2026 14:55:00+07:00
- **Phạm vi file đã sửa / tạo mới (chỉ trong allowlist):**
  - MỚI: `.agent/person_tracking/AUDIT.md`
  - MỚI: `.agent/person_tracking/INVENTORY.md`
  - MỚI: `.agent/person_tracking/RISKS.md`
  - MỚI: `.agent/person_tracking/INTERFACES.md`
  - MỚI: `.agent/person_tracking/PROGRESS.md`
  - MỚI: `.agent/person_tracking/evidence/phase_0/audit_commands.log`
  - NGUYÊN VẸN: Toàn bộ 12 file trong `person_tracking/` được giữ nguyên (chưa xóa, chưa refactor code).
  - READ-ONLY: Các file voice, rules, Gazebo, Astra và Nav2 được giữ nguyên 100%.
- **Bằng chứng:** `.agent/person_tracking/evidence/phase_0/audit_commands.log`
- **Kiểm thử đã thực hiện:**
  - 14 lệnh khảo sát thành công: kiểm tra git status, canonical paths, devcontainer mount, build directory, nvidia-smi GPU, model weights, package tracking, mã nguồn Astra, cấu hình Nav2 và mã nguồn Voice.
  - Phân tách rõ các lệnh bị hạn chế trong sandbox (`docker ps`, `which ros2/colcon`, `ls /opt`).
- **Không kiểm được (Hardware & Runtime blockers):**
  - Không kiểm được live stream camera Astra thật (thiết bị không có tại chỗ).
  - Không chạy lệnh ROS trực tiếp trên host do môi trường host là Ubuntu 22.04 (Humble) và sandbox cô lập; môi trường thực thi chuẩn là VS Code Dev Container (ROS 2 Jazzy, Ubuntu 24.04).
- **Vấn đề / Rủi ro phát hiện:**
  - Astra driver mặc định tắt depth registration (`depth_registration: false`), dẫn đến lệch pixel RGB và Depth nếu không cấu hình lại.
  - Gazebo URDF mô phỏng camera hướng ra phía trước (`rpy="0 0 0"`), trong khi robot thật gắn camera hướng ra sau. Tuân thủ Rule A.4: tuyệt đối không sửa Gazebo, xử lý hình học chuẩn camera sau qua offline pipeline.
  - Vị trí Velocity Guard: Phải nằm trước Collision Monitor (`Nav2 -> Smoother -> Guard -> Collision Monitor -> Base`).
  - Colcon root thực sự là `/home/thuytran/ws` (trong container là `/home/ws`), thư mục `build/` và `install/` tại `src/` là do lệnh chạy nhầm trước đó của voice agent.
- **Giao diện cần agent voice thực hiện:** Đã lập dự thảo `INTERFACES.md`, chờ gửi sang agent voice sau khi chủ dự án duyệt.
- **Chốt Phase 0:** Đạt Gate 0 (Đã kiểm kê chi tiết, phân loại nguồn sự thật, lập đầy đủ 5 file tài liệu AI và evidence log, không sửa code ngoài allowlist).
- **Phase tiếp theo, chỉ làm sau lệnh của chủ dự án:** Phase 1 — Quy tắc, skill, thiết kế & Kế hoạch kiểm thử.

---

### Phase 1 — Quy tắc, skill, thiết kế & Kế hoạch kiểm thử — [NOT_STARTED]
- Chỉ bắt đầu khi có lệnh từ chủ dự án.

### Phase 2 — Source adapters, offline dataset & Test infrastructure — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 1 hoàn thành và được duyệt.

### Phase 3 — Multi-person perception, pose & Tracking ID — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 2 hoàn thành và được duyệt.

### Phase 4 — Giơ tay, chọn khách gần nhất & Khóa phiên — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 3 hoàn thành và được duyệt.

### Phase 5 — RGB-D, depth & Hình học camera sau — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 4 hoàn thành và được duyệt.

### Phase 6 — Session FSM, timeout & Giao tiếp voice — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 5 hoàn thành và được duyệt.

### Phase 7 — Velocity guard, mecanum & Watchdog — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 6 hoàn thành và được duyệt.

### Phase 8 — End-to-end ROS 2 integration (không sửa Gazebo) — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 7 hoàn thành và được duyệt.

### Phase 9 — Giao tiếp song song với agent voice — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 8 hoàn thành và được duyệt.

### Phase 10 — Validation hiệu năng & Độ bền (offline + sim) — [NOT_STARTED]
- Chỉ bắt đầu khi Phase 9 hoàn thành và được duyệt.

### Phase 11 — Robot thật + Astra thật, nghiệm thu bàn giao — [NOT_STARTED]
- Chỉ thực hiện khi có camera Astra thật và robot thật tại hiện trường.
