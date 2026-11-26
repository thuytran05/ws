# PERSON TRACKING — SỔ TAY CÂU LỆNH VẬN HÀNH VÀ KHẢO SÁT (COMMANDS REFERENCE)

**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.0 — 23/11/2026 (Phase 1)  
**Mục tiêu:** Hướng dẫn thực thi lệnh an toàn cho phân hệ Tracking, phân định rõ giữa lệnh đã kiểm chứng thực tế và lệnh dự kiến cho các phase sau.

> [!IMPORTANT]
> - Đây là sổ tay lệnh riêng của phân hệ Tracking, đặt tại `.agent/COMMANDS_TRACKING.md (cùng cấp với .agent/COMMANDS_VOICE.md)`.
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

# [ĐÃ KIỂM CHỨNG TẠI PHASE 2] Build cô lập riêng cho package person_tracking
colcon build \
  --packages-select person_tracking \
  --build-base build_tracking \
  --install-base install_tracking \
  --symlink-install

# Source overlay riêng của tracking sau khi build
source /home/ws/install_tracking/setup.bash
```

---

## 3. Quy trình Kiểm thử và Chạy Offline Replay Đã Kiểm chứng (Verified in Phase 2)

### 3.1. Chạy Bộ Kiểm thử Unit và Smoke Test (Pytest)
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
source /home/ws/install_tracking/setup.bash
export ROS_DOMAIN_ID=42

# Chạy toàn bộ 16 tests của adapter, đồng bộ stamp và source node (100% PASS)
pytest src/person_tracking/test/ -v
```

### 3.2. Chạy Perception Offline Replay (ROS 2 Launch)
```bash
# 1. Chế độ Synthetic Fixture (người có RGB-D hợp lệ, 15 FPS)
ros2 launch person_tracking offline_replay.launch.py \
  source_mode:=synthetic_fixture \
  synthetic_case:=person_valid_rgbd \
  fps:=15.0

# 2. Chế độ Video Replay (chỉ đọc RGB từ MP4, bảo đảm strict RGB-only, depth None)
ros2 launch person_tracking offline_replay.launch.py \
  source_mode:=video_replay \
  fps:=15.0 \
  loop:=true

# 3. Chế độ Synthetic Corrupted Depth (bơm NaN/0 để kiểm tra bộ lọc an toàn)
ros2 launch person_tracking offline_replay.launch.py \
  source_mode:=synthetic_fixture \
  synthetic_case:=corrupted_depth \
  fps:=15.0
```

### 3.3. Quy trình Ghi và Replay Rosbag2 An toàn
```bash
# Đặt ROS_DOMAIN_ID đồng nhất để tránh nhiễu
export ROS_DOMAIN_ID=42

# Chạy perception source
ros2 run person_tracking perception_source --ros-args -p source_mode:=synthetic_fixture &

# Ghi dữ liệu mẫu (kết thúc đúng cách bằng SIGINT / Ctrl+C, KHÔNG dùng cờ -d để tự dừng)
ros2 bag record -o src/person_tracking/fixtures/sample_fixture_bag \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/color/camera_info \
  /tracking/source_status

# Kiểm tra thông tin bag (metadata và message count)
ros2 bag info src/person_tracking/fixtures/sample_fixture_bag

# Phát lại rosbag2 đơn lẻ (lưu ý DDS discovery mất ~200ms đầu đối với QoS Best Effort)
ros2 bag play src/person_tracking/fixtures/sample_fixture_bag --rate 1.0
```

---

## 4. Các Câu lệnh Dự kiến cho Các Phase Tiếp Theo [PLANNED / NOT YET EXECUTED]

### 4.1. Phase 3: Multi-person Pose & Tracking ID
```bash
# Chạy node multi-person perception với debug image
ros2 run person_tracking person_detector_tracker --ros-args -p device:=cuda:0

# Kiểm tra output detection topic
ros2 topic echo /tracking/detected_persons
```

### 4.2. Phase 4: Nhận diện Cử chỉ và Khóa Người Gần nhất
```bash
# Chạy kiểm thử tự động chọn người gần nhất
pytest src/person_tracking/test/test_nearest_selection.py -v --timeout=30
```

### 4.3. Phase 6: State Machine và Giao thức Voice
```bash
ros2 run person_tracking tracking_manager &
ros2 topic echo /tracking/session_state
```

### 4.4. Phase 7: Velocity Guard & Watchdog
```bash
pytest src/person_tracking/test/test_velocity_guard_safety.py -v --timeout=20
```

### 4.5. Phase 8: Khởi chạy Tích hợp Toàn hệ thống
```bash
ros2 launch person_tracking escort.launch.py use_sim_time:=true
```

---

## 5. Kiểm Thử Trực Quan Bằng Webcam Laptop & Debug Visualization (`live_webcam`)

> [!NOTE]
> Chế độ `live_webcam` hoạt động ở chế độ **RGB_ONLY** (`depth_valid=false`, `depth_proven=false`, không tạo depth giả và không phát `/cmd_vel`). Chỉ dùng để kiểm thử nhận diện người, skeleton COCO 17 điểm, giơ tay 1.5s, khóa ID và FSM trực quan — **không thay thế** cho nghiệm thu camera Astra RGB-D hoặc robot thật tại Gate 11.

### 5.1. Source Workspace ROS 2 Jazzy (Bắt buộc cho mọi Terminal mới)
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
source /home/ws/install_tracking/setup.bash
export ROS_DOMAIN_ID=43
```

### 5.2. Chạy Webcam Chỉ Pipeline Nhận Diện (GPU `cuda:0`)
```bash
ros2 launch person_tracking perception_pipeline.launch.py \
  source_mode:=live_webcam \
  camera_index:=0 \
  camera_width:=640 \
  camera_height:=480 \
  fps:=30.0 \
  device:=cuda:0 \
  publish_debug_image:=true \
  image_qos_reliability:=reliable
```

### 5.3. Chạy Webcam Chỉ Pipeline Nhận Diện (CPU)
```bash
ros2 launch person_tracking perception_pipeline.launch.py \
  source_mode:=live_webcam \
  camera_index:=0 \
  camera_width:=640 \
  camera_height:=480 \
  fps:=30.0 \
  device:=cpu \
  publish_debug_image:=true \
  image_qos_reliability:=reliable
```

### 5.4. Chạy Tích Hợp Webcam + Detector + Tracking Manager (1 Lệnh)
Khởi chạy đồng thời Webcam Source, Person Detector (YOLO Pose + ByteTrack + DebugVisualizer) và Tracking Manager FSM (không chạy Nav2/Guard/Watchdog/Voice/Base):
```bash
ros2 launch person_tracking webcam_debug.launch.py \
  camera_index:=0 \
  camera_width:=640 \
  camera_height:=480 \
  fps:=30.0 \
  device:=cuda:0 \
  publish_debug_image:=true \
  view_debug:=false
```
*(Truyền `view_debug:=true` nếu muốn tự động mở luôn `rqt_image_view`.)*

### 5.5. Xem Hình Ảnh `/tracking/debug_image` Bằng `rqt_image_view`
```bash
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /tracking/debug_image
```

### 5.6. Phương Án Thay Thế Bằng RViz2 (Nếu Không Có `rqt_image_view`)
```bash
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
rviz2
```
Trong giao diện RViz2: chọn **Add** $\to$ **By topic** $\to$ `/tracking/debug_image` $\to$ **Image** (hoặc **Add** $\to$ **Image** rồi đặt Topic = `/tracking/debug_image`, Reliability Policy = `Best Effort`).

### 5.7. Theo Dõi `/tracking/session_state` (Định Dạng JSON Bằng Python, Không Cần `jq`)
```bash
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
ros2 topic echo /tracking/session_state --field data | python3 -c "
import sys, json
for line in sys.stdin:
    line = line.strip()
    if line:
        try:
            print(json.dumps(json.loads(line), indent=2, ensure_ascii=False), flush=True)
        except Exception:
            print(line, flush=True)
"
```

### 5.8. Đo FPS Đầu Vào và Đầu Ra
```bash
# Terminal A: Đo FPS ảnh gốc từ webcam
export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/color/image_raw

# Terminal B: Đo FPS ảnh debug overlay
export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash
ros2 topic hz /tracking/debug_image

# Terminal C: Đo FPS bản tin observation
export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash
ros2 topic hz /tracking/observation
```

### 5.9. Kiểm Tra Node Graph và Xác Nhận Không Có Publisher `/cmd_vel`
```bash
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic info -v /tracking/debug_image
ros2 topic info -v /tracking/session_state
ros2 topic info -v /cmd_vel
```
*(Kỳ vọng: `/cmd_vel` có `Publisher count: 0`.)*

### 5.10. Dừng Hệ Thống Sạch Sẽ
- Tại terminal đang chạy `ros2 launch`, nhấn **`Ctrl+C`** (SIGINT) một lần và chờ các node đóng `VideoCapture` và thoát hoàn toàn.
- Kiểm tra không còn process treo của phiên kiểm thử:
  ```bash
  ps aux | grep -E "perception_source|person_detector|tracking_manager" | grep -v grep
  ```

---

## 6. Kiểm Thử Camera Astra S Thực Tế Kết Hợp Person Tracking (Không Chạy Robot)

> [!IMPORTANT]
> Các lệnh dưới đây chỉ khởi chạy Astra S driver + `person_detector` + `escort_tracking_manager` (**không** chạy Nav2, Velocity Guard, Watchdog hay robot base; `/cmd_vel` luôn có 0 publisher). Sử dụng cấu hình QoS đã benchmark trên Astra S: **`RELIABLE`, `KEEP_LAST`, `VOLATILE`, `Depth 5`**.

### 6.1. Terminal 1 — Khởi Chạy Astra S Driver (Bật D2C & Đồng Bộ Thời Gian)
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43   # Dùng 43 khi test cô lập; dùng 0 khi ghép hệ thống chính
ros2 launch astra_camera astra.launch.xml \
  depth_registration:=true \
  color_depth_synchronization:=true
```

### 6.2. Terminal 2 — Kiểm Tra Stream, QoS & Depth Registration Bằng Probe
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
python3 /home/ws/src/person_tracking/test/check_astra_hardware_topics.py \
  --duration 5.0 \
  --min-rate 20.0 \
  --qos-reliability reliable \
  --qos-depth 5
```

### 6.3. Terminal 2 — Khởi Chạy Person Tracking + Tracking Manager Với Astra S
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
source /home/ws/install_tracking/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch person_tracking astra_tracking_debug.launch.py \
  device:=cuda:0 \
  publish_debug_image:=true \
  image_qos_reliability:=reliable \
  image_qos_depth:=5 \
  info_qos_reliability:=reliable \
  info_qos_depth:=5 \
  prioritize_latest_frame:=true \
  readiness_min_frames:=5 \
  view_debug:=false
```

### 6.4. Terminal 3 — Xem Ảnh Debug & Kiểm Tra `camera_ready` / `/cmd_vel`
```bash
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash

# Xem ảnh debug skeleton & depth
ros2 run rqt_image_view rqt_image_view /tracking/debug_image

# Kiểm tra camera_ready và data_quality trong /tracking/observation (Python, không cần jq)
ros2 topic echo /tracking/observation --field data | python3 -c "
import sys, json
for line in sys.stdin:
    line = line.strip()
    if line:
        d = json.loads(line)
        print(f'ready={d.get(\"camera_ready\")} ({d.get(\"camera_readiness_reason\")}) | quality={d.get(\"data_quality\")} | candidates={len(d.get(\"candidates\", []))}', flush=True)
"

# Xác nhận không có publisher /cmd_vel
ros2 topic info -v /cmd_vel
```
