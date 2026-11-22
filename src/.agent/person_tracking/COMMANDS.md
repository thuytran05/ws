# PERSON TRACKING — SỔ TAY CÂU LỆNH VẬN HÀNH VÀ KHẢO SÁT (COMMANDS REFERENCE)

**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.0 — 23/11/2026 (Phase 1)  
**Mục tiêu:** Hướng dẫn thực thi lệnh an toàn cho phân hệ Tracking, phân định rõ giữa lệnh đã kiểm chứng thực tế và lệnh dự kiến cho các phase sau.

> [!IMPORTANT]
> - Đây là sổ tay lệnh riêng của phân hệ Tracking, đặt tại `.agent/person_tracking/COMMANDS.md`.
> - **Tuyệt đối không sửa** file `.agent/COMMANDS.md` chung hoặc `.agent/COMMANDS_VOICE.md` của phân hệ Voice.
> - Mọi thao tác build và chạy ROS 2 bắt buộc phải thực hiện trong terminal của **VS Code Dev Container (ROS 2 Jazzy)**. Không chạy ROS trên host Ubuntu 22.04 (Humble).

---

## 1. Các Câu lệnh Khảo sát Đã Xác minh (Verified in Phase 0 & 1)

Các câu lệnh sau đã được thực thi và kiểm chứng an toàn trong quá trình audit:

### 1.1. Kiểm tra Môi trường và Git
```bash
# Kiểm tra đường dẫn thư mục hiện tại và commit HEAD
pwd
git rev-parse HEAD
git status --short

# Kiểm tra GPU phần cứng trên máy
nvidia-smi

# Kiểm tra cấu hình Dev Container
cat /home/thuytran/ws/.devcontainer/devcontainer.json
```

### 1.2. Khảo sát Mã nguồn và Cấu hình (Read-only)
```bash
# Kiểm tra danh sách file package person_tracking
find src/person_tracking -type f | sort

# Khảo sát các topic và tham số của Astra Driver
grep -rn 'stream_name_' src/ros2_astra_camera/astra_camera/src
grep -rn 'setImageRegistrationMode' src/ros2_astra_camera/astra_camera/src

# Khảo sát chuỗi vận tốc Nav2 và Collision Monitor
grep -rn 'collision_monitor' src/linorobot2/linorobot2_navigation/config/navigation.yaml

# Khảo sát giao diện và ActionClient của phân hệ Voice
grep -rn 'ActionClient' src/airport_robot_voice/airport_robot_voice/nav_command_node.py
```

---

## 2. Quy trình Build Cô lập (Isolated Build Workflow)

Để bảo đảm không làm ghi đè hoặc xung đột thư mục `build/`, `install/`, `log/` dùng chung với phân hệ Voice:

```bash
# Mở terminal bên trong Dev Container (/home/ws)
cd /home/ws

# Source môi trường ROS 2 Jazzy nền tảng
source /opt/ros/jazzy/setup.bash

# [DỰ KIẾN TỪ PHASE 2] Build cô lập riêng cho package person_tracking
colcon build \
  --packages-select person_tracking \
  --build-base build_tracking \
  --install-base install_tracking \
  --symlink-install

# Source overlay riêng của tracking sau khi build
source /home/ws/install_tracking/setup.bash
```

---

## 3. Các Câu lệnh Dự kiến cho Các Phase Sau [PLANNED / NOT YET EXECUTED]

> [!WARNING]
> Toàn bộ các lệnh dưới đây là kế hoạch kỹ thuật cho các phase triển khai tiếp theo. **Chưa được thực thi trong Phase 1**.

### 3.1. Phase 2: Khởi chạy Perception Offline
```bash
# [PLANNED - Phase 2] Chạy node perception với video fixture
ros2 launch person_tracking tracking.launch.py source_mode:=video_replay video_path:=/path/to/test.mp4

# [PLANNED - Phase 2] Chạy kiểm thử pytest cho bộ đọc dữ liệu
pytest src/person_tracking/test/test_source_adapters.py -v
```

### 3.2. Phase 4: Kiểm thử Nhận diện Cử chỉ và Khóa Người Gần nhất
```bash
# [PLANNED - Phase 4] Chạy bài kiểm thử tự động nearest person selection
pytest src/person_tracking/test/test_nearest_selection.py -v --timeout=30
```

### 3.3. Phase 6: Kiểm thử State Machine và Mock Voice
```bash
# [PLANNED - Phase 6] Khởi chạy node FSM Manager kết hợp Mock Voice Owner
ros2 run person_tracking escort_tracking_manager --ros-args --params-file src/person_tracking/config/person_tracking.yaml &
ros2 run person_tracking mock_voice_owner &

# [PLANNED - Phase 6] Kiểm tra chuyển trạng thái qua CLI
ros2 topic echo /tracking/session_state
```

### 3.4. Phase 7: Kiểm thử Velocity Guard
```bash
# [PLANNED - Phase 7] Chạy test an toàn ngắt vận tốc tức thì khi mất khách
pytest src/person_tracking/test/test_velocity_guard_safety.py -v --timeout=20
```

### 3.5. Phase 8: Khởi chạy Tích hợp Toàn hệ thống
```bash
# [PLANNED - Phase 8] Khởi chạy toàn bộ stack tracking kết hợp Gazebo mô phỏng
ros2 launch person_tracking escort.launch.py use_sim_time:=true
```
