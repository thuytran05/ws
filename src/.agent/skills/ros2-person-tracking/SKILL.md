---
name: ros2-person-tracking
description: Workflow and technical guidelines for developing, testing, and integrating the person_tracking subsystem for the airport escort robot in ROS 2 Jazzy.
---

# ROS 2 Jazzy Person Tracking Subsystem Skill

Tài liệu này là cẩm nang hướng dẫn vận hành chuyên biệt cho bất kỳ kỹ sư hoặc AI Agent nào tiếp nhận phát triển phân hệ `person_tracking` trong dự án robot dẫn đường sân bay.

---

## 1. Ranh giới Thao tác và Allowlist Bắt buộc

### 1.1. Phạm vi Ghi (Write Allowlist)
Chỉ được phép tạo, chỉnh sửa hoặc xóa file bên trong các thư mục sau:
1. `src/person_tracking/**` (Mã nguồn ROS 2, launch, config, test của tracking).
2. `src/.agent/person_tracking/**` (Tài liệu đặc tả, kế hoạch kiểm thử, quyết định kiến trúc, log evidence).
3. `src/.agent/skills/ros2-person-tracking/**` (Skill AI của phân hệ tracking).
4. Các thư mục build/log cô lập riêng nếu có: `build_tracking/`, `install_tracking/`.

### 1.2. Vùng Cấm Tuyệt đối (Strictly READ-ONLY)
- `airport_robot_voice/**`, `.agent/airport_robot_voice/**`, `.agent/skills/ros2-voice-nav/**`.
- `linorobot2_gazebo/**`, `linorobot2_description/**` (Tuyệt đối không sửa URDF/xacro, camera mô phỏng hay Gazebo).
- `ros2_astra_camera/**` (Không sửa mã nguồn hay launch file của Astra driver).
- `linorobot2_navigation/**`, firmware, micro-ROS.
- `.agent/rules/workspace.md`, `.agent/COMMANDS.md`, `.agent/COMMANDS_VOICE.md`.
- Tuyệt đối không commit hoặc push Git (người dùng tự thực hiện sau khi review).

---

## 2. Quy trình Thực hiện Trước Mỗi Lượt Làm việc (Pre-turn Routine)

Mỗi khi bắt đầu một phiên làm việc mới:
1. **Kiểm tra Git:** Chạy `git status --short`. Nếu thấy file của agent voice đang sửa đổi, giữ nguyên 100%, không rollback.
2. **Đọc tài liệu cơ sở:**
   - [WORKFLOW.md](file:///home/thuytran/ws/src/.agent/person_tracking/WORKFLOW.md): Nguồn sự thật số 2 chứa D01–D12, 12 Phase và tiêu chí Gate.
   - [PROJECT_SPEC.md](file:///home/thuytran/ws/src/.agent/person_tracking/PROJECT_SPEC.md): Đặc tả kiến trúc phân hệ.
   - [INTERFACES.md](file:///home/thuytran/ws/src/.agent/person_tracking/INTERFACES.md): Hợp đồng giao tiếp giữa Tracking, Voice và Nav2.
   - [PROGRESS.md](file:///home/thuytran/ws/src/.agent/person_tracking/PROGRESS.md): Kiểm tra xem đang ở Phase nào, phase trước đã đạt Gate chưa.
3. **Tuân thủ đúng Phase được giao:** Chỉ làm phase được người dùng yêu cầu, không âm thầm nhảy phase.

---

## 3. Môi trường Thực thi và Build Cô lập (Build Isolation)

- **Môi trường chuẩn:** Thao tác trực tiếp qua terminal của VS Code Dev Container (Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 tại `/home/ws`). Không chạy lệnh ROS trên host Ubuntu 22.04 (Humble).
- **Quy chuẩn Colcon Build cô lập:**
  Tuyệt đối không chạy `colcon build` trần ở gốc workspace vì có nguy cơ ghi đè các artifact của phân hệ khác. Luôn chỉ định base riêng:
  ```bash
  cd /home/ws
  source /opt/ros/jazzy/setup.bash
  colcon build \
    --packages-select person_tracking \
    --build-base build_tracking \
    --install-base install_tracking \
    --symlink-install
  source /home/ws/install_tracking/setup.bash
  ```

---

## 4. Các Nguyên tắc An toàn Cốt lõi cần Ghi nhớ

1. **Voice là Chủ sở hữu Duy nhất của `NavigateToPose`:**
   - Tracking không bao giờ tạo `ActionClient<NavigateToPose>`, không gửi goal, không gọi cancel trực tiếp.
   - Tracking chỉ phát `/tracking/session_state` (báo `locked` và `session_id`) và `/tracking/safety_request` (yêu cầu `ABORT/STOP`).
2. **Cổng An toàn Vận tốc độc lập (`escort_velocity_guard`):**
   - Nằm trong chuỗi: `Nav2 Smoother -> Escort Velocity Guard -> Collision Monitor -> Base`.
   - Khi mất khách hoặc lỗi dữ liệu: Lập tức thực hiện **Active Continuous Zeroing** (phát liên tục vận tốc 0 ở 20 Hz), không chờ Voice phản hồi.
3. **Phân quyền Khóa Duy nhất:**
   - Chỉ duy nhất `escort_tracking_manager` có quyền khóa khách và sinh `session_id`. `passenger_perception` chỉ xuất quan sát thô.
4. **Không phục hồi tự động sau Abort:**
   - Một khi phiên đã bị hủy (`ABORT`), kể cả khi camera thấy lại người thì robot vẫn phải giữ nguyên vận tốc 0 cho đến khi có một lượt khóa mới với `session_id` mới.

---

## 5. Lưu trữ Bằng chứng và Báo cáo Gate (Evidence & Reporting)

- Với mỗi bài test thực thi trong Phase $N$, lưu file log thô vào `.agent/person_tracking/evidence/phase_N/`.
- Cập nhật [PROGRESS.md](file:///home/thuytran/ws/src/.agent/person_tracking/PROGRESS.md) với đầy đủ thông tin: Trạng thái, ngày giờ, số test pass/fail, bằng chứng, rủi ro và blocker phần cứng.
- Dừng lại và báo cáo theo mẫu chuẩn của dự án; chờ người dùng duyệt Gate trước khi bước sang phase tiếp theo.
