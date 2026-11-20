# PERSON TRACKING — BÁO CÁO KHẢO SÁT HIỆN TRẠNG KỸ THUẬT (PHASE 0 AUDIT)

**Dự án:** Robot dẫn đường hành khách sân bay (ROS 2 Jazzy, Mecanum, Camera Astra sau)  
**Phân hệ:** `person_tracking`  
**Kỹ sư thực hiện:** Senior ROS 2 Jazzy Robotics / Computer Vision / Safety Integration Engineer  
**Ngày audit:** 20/11/2026 — Phase 0  
**Tài liệu tham chiếu:** `WORKFLOW.md` (Source of truth 2), `PROJECT_SPEC.md` của Voice, Code base hiện có trong workspace `ws`.

---

## 1. Kiểm tra Phạm vi, Ranh giới An toàn và Workspace

### 1.1. Đường dẫn Canonical và Mount Docker
- **Đường dẫn thư mục làm việc hiện tại:** `/home/thuytran/ws/src` (lệnh `pwd`, `readlink -f .`).
- **Gốc Git repository:** `/home/thuytran/ws` (lệnh `git rev-parse --show-toplevel`).
- **Cấu hình VS Code Dev Container (`.devcontainer/devcontainer.json`):**
  - Host directory: `${localWorkspaceFolder}` (`/home/thuytran/ws`).
  - Container workspace mount: `/home/ws` (loại mount: `bind`).
  - Container source directory: `/home/ws/src`.
  - Quyền hạn: `privileged: true`, `--net=host`, `--pid=host`, `--ipc=host`, `--gpus all`.
  - Môi trường: `ROS_DOMAIN_ID=0`, `NVIDIA_VISIBLE_DEVICES=all`.
  - Base Docker image: `ros:jazzy` (Ubuntu 24.04 Noble LTS, Python 3.12).
- **Phát hiện quan trọng về Colcon Root:**
  - Thư mục `/home/thuytran/ws/build` chứa toàn bộ 15 package của dự án (`linorobot2`, `astra_camera`, `airport_robot_voice`, `person_tracking`, v.v.). Đây chính là **colcon workspace root thực sự**.
  - Thư mục `/home/thuytran/ws/src/build` chỉ chứa duy nhất `airport_robot_voice` (được build vào lúc 12:56 ngày 20/11 do agent voice chạy nhầm lệnh `colcon build` bên trong thư mục `src`).
  - **Quy tắc an toàn:** Tuyệt đối không xóa hay dọn dẹp các thư mục `build/`, `install/`, `log/` dùng chung. Khi build phân hệ tracking, chỉ dùng `--packages-select person_tracking` và thực thi từ gốc workspace `/home/ws` trong container.

### 1.2. Trạng thái Git và Bảo toàn công việc của Agent Voice
- Lệnh `git status --short` xác nhận nhánh `master`:
  ```text
   M .agent/COMMANDS_VOICE.md
   D .agent/airport_robot_voice/COMMANDS.md
   M .agent/airport_robot_voice/PROGRESS.md
   M airport_robot_voice/airport_robot_voice/nav_command_node.py
   M airport_robot_voice/test/test_ros_lifecycle.py
  ?? .agent/person_tracking/
  ```
- **Xác minh:** Toàn bộ các thay đổi chưa commit thuộc về agent song song `airport_robot_voice`. Tuân thủ nghiêm ngặt Quy tắc A.7 và A.3: giữ nguyên 100%, không rollback, không `git add .`, không chạm vào các file của voice.

---

## 2. Khảo sát Mã nguồn Package `person_tracking`

### 2.1. Cấu trúc và Metadata
- Package `person_tracking` được tổ chức theo chuẩn `ament_python` v0.1.0 (`package.xml`, `setup.py`, `setup.cfg`).
- Có 2 executables đăng ký trong `setup.py`: `person_tracker` và `person_follower`.
- Có 2 launch files: `tracking.launch.py` (chạy tracker đơn lẻ) và `following.launch.py` (chạy tracker + follower).
- File trọng số mô hình YOLO: `/home/thuytran/ws/yolo26n-pose.pt` (kích thước 7.87 MB, PyTorch zip archive).

### 2.2. Đánh giá kỹ thuật chi tiết từng module

#### A. Node `person_tracker.py` (Nhận diện & Đo khoảng cách)
- **Cơ chế hoạt động:**
  - Lắng nghe đồng thời RGB (`/camera/color/image_raw`), Depth (`/camera/depth/image_raw`) qua `message_filters.ApproximateTimeSynchronizer` (slop = 0.06s).
  - Nhận nội tại camera `/camera/color/camera_info`.
  - Chạy YOLO Pose (`yolo26n-pose.pt`, ByteTrack tracker) để trích xuất bounding box và 17 keypoints chuẩn COCO.
  - Nhận diện giơ tay: cổ tay (wrist 9 hoặc 10) cao hơn mũi (nose 0) kèm biên độ đệm `max(10.0, height * 0.035)`, giữ liên tục trong `gesture_seconds` (2.0s).
  - Trích xuất depth thân người: lấy patch hình chữ nhật vùng torso giữa 4 điểm vai (5, 6) và hông (11, 12), thu hẹp 25% mỗi phía để tránh viền nền; đọc depth (hỗ trợ `16UC1` nhân 0.001 hoặc `32FC1`), lọc giá trị hữu hạn trong khoảng [0.4m, 5.0m], tính trung vị (`median`) và lọc độ phân tán IQR.
  - Xuất dữ liệu: `target_position` (`geometry_msgs/PointStamped`), `selected_id` (`std_msgs/Int32`), `target_pixel`, `target_visible`, và ảnh overlay `image_annotated`.
- **Hạn chế kỹ thuật và Khiếm khuyết nghiêm trọng:**
  1. **Không chọn người gần nhất khi nhiều người cùng giơ tay:** Vòng lặp duyệt danh sách bounding box chọn ngay người đầu tiên đạt ngưỡng 2.0s, vi phạm trực tiếp Quyết định D07.
  2. **Không có Session FSM:** Không có `session_id`, không quản lý trạng thái chuyển tiếp (`LOCKED_WAIT_DESTINATION`, `NAVIGATING`, `WAITING_FOR_PASSENGER`, v.v.).
  3. **Ràng buộc cứng về Frame ID:** Ép buộc `rgb_msg.header.frame_id == depth_msg.header.frame_id == info.header.frame_id == 'camera_color_optical_frame'`. Điều này chỉ chạy được nếu camera Astra bật phần cứng depth registration; nếu driver phát hành depth trong `camera_depth_optical_frame` thì node sẽ văng lỗi ngay lập tức.
  4. **Chốt lỗi cứng (Latch loss):** Khi mất mục tiêu, node chốt trạng thái `lost = True` và bắt buộc phải gọi service `/person_tracking/reset` thì mới cho phép nhận diện lại, không có cơ chế `grace period` cho các frame bị drop ngắn.

#### B. Node `person_follower.py` & `control.py` (Điều khiển bám theo người cũ)
- **Cơ chế hoạt động:** Lắng nghe `target_position`, tính khoảng cách sai số so với cự ly mong muốn `desired_distance = 1.5m` và góc lệch `angle = atan2(x, z)`. Tính vận tốc tịnh tiến `vx` và vận tốc góc `wz`, áp dụng bộ lọc gia tốc `slew()`, rồi xuất trực tiếp ra `/cmd_vel` với chu kỳ 20 Hz (0.05s).
- **Đánh giá xung đột:**
  - **SAI HOÀN TOÀN VỚI MỤC TIÊU NGHIỆP VỤ:** Phân hệ cũ được viết theo mô hình *robot chạy theo sau đuôi người* (khi người đi xa thì robot tăng tốc chạy theo; khi người lại gần thì robot lùi lại).
  - Trong dự án hiện tại: **Robot dẫn đường đi phía trước, hành khách đi phía sau; Nav2 bám lộ trình bản đồ sân bay.** Tracking không được tự ý lái robot và không được tăng tốc khi khách lại gần.
  - Node này phải được **THAY THẾ HOÀN TOÀN** bằng `escort_velocity_guard` (chỉ nhận lệnh Nav2, clamp tốc độ khi khách tụt xa, dừng khi mất khách, chỉnh nhẹ yaw để giữ khách trong tầm nhìn).

---

## 3. Khảo sát Phân hệ Camera Astra (`ros2_astra_camera`)

### 3.1. Các thông số tĩnh từ Mã nguồn (`ob_camera_node.cpp`)
- **Package:** `ros2_astra_camera/astra_camera` và `astra_camera_msgs`.
- **Node executable:** `astra_camera_node`.
- **Topics chuẩn được tạo:**
  - Ảnh màu: `/${camera_name}/color/image_raw` (`sensor_msgs/msg/Image`) — dòng 383, 386.
  - CameraInfo màu: `/${camera_name}/color/camera_info` (`sensor_msgs/msg/CameraInfo`) — dòng 389, 392.
  - Ảnh độ sâu: `/${camera_name}/depth/image_raw` (`sensor_msgs/msg/Image`) — dòng 383, 386.
  - CameraInfo độ sâu: `/${camera_name}/depth/camera_info` — dòng 389, 392.
  - Ảnh hồng ngoại: `/${camera_name}/ir/image_raw` — dòng 383, 386.
  - PointCloud2: `/camera/depth/color/points` remap sang `/camera/depth_registered/points`.
  - Extrinsics: `extrinsic/depth_to_color` (`astra_camera_msgs/msg/Extrinsics`).
- **Encodings:**
  - Depth: `sensor_msgs::image_encodings::TYPE_16UC1` (đơn vị: milimét, uint16) — dòng 288.
  - Color: `sensor_msgs::image_encodings::RGB8` (hoặc BGR8 trên UVC) — dòng 294.
  - IR: `sensor_msgs::image_encodings::MONO8` — dòng 300.
- **Frame IDs:**
  - Color optical: `camera_color_optical_frame` — dòng 312.
  - Depth optical: `camera_depth_optical_frame` — dòng 312.
  - Aligned depth optical: Khi `depth_registration_ == true`, frame ID của depth image được gán bằng `depth_aligned_frame_id_[stream_index]`, tức là chính `camera_color_optical_frame` (dòng 337, 529, 551).
- **Phát hiện cấu hình then chốt:**
  - File `astra.launch.xml` dòng 5: `<arg name="depth_registration" default="false"/>`.
  - **Kết luận:** Nếu chạy camera thật bằng launch mặc định mà không truyền tham số `depth_registration:=true`, depth sẽ ở frame quang học riêng và không căn chỉnh pixel với RGB!

### 3.2. Khảo sát Frame và URDF Robot (`linorobot2`)
- File `linorobot2_description/urdf/mecanum_properties.urdf.xacro` dòng 27:
  ```xml
  <xacro:property name="depth_sensor_pose">
    <origin xyz="0.2375 0 0.06" rpy="0 0 0"/>
  </xacro:property>
  ```
- File `linorobot2_description/urdf/sensors/depth_sensor.urdf.xacro`:
  - `camera_link` gắn vào `base_link` tại vị trí `depth_sensor_pose`.
  - Hướng quay: `rpy="0 0 0"` -> Hướng ra **phía trước** robot!
- **Sự khác biệt thực tế (Discrepancy):**
  - Robot thật có camera Astra gắn cố định **hướng ra phía sau** (gần với `rpy="0 0 ${pi}"`).
  - Tuân thủ Quy tắc A.4: **Tuyệt đối không sửa URDF/xacro hay launch của Gazebo.**
  - Giải pháp thiết kế: Toàn bộ module perception và guard xử lý hình học dựa trên quy ước quang học độc lập với camera sau; khi test với Gazebo chỉ dùng chuyển động robot cơ sở và giả lập quan sát (`fake observation injection`).

---

## 4. Khảo sát Phân hệ Voice & Nav2 (READ-ONLY Audit)

### 4.1. Phân hệ Voice (`airport_robot_voice`)
- Mã nguồn kiểm tra: `airport_robot_voice/airport_robot_voice/nav_command_node.py` (1,584 dòng).
- **Action Clients sở hữu bởi Voice:**
  - `self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)` (dòng 167)
  - `self.planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)` (dòng 168)
- **Publishers:**
  - `/voice/nav_state` (`std_msgs/msg/String`, JSON format, QoS Transient Local Reliable) — dòng 176, 480.
  - `/voice/feedback` (`std_msgs/msg/String`, JSON format) — dòng 177, 482.
  - `/voice/say` (`std_msgs/msg/String`) — dòng 178.
- **Nội dung bản tin `/voice/nav_state` (dòng 465–479):**
  - `schema_version`: "1.0"
  - `node_online`: bool
  - `mission_state`: string (IDLE, NAVIGATING, PAUSED, CANCELING, ARRIVED, v.v.)
  - `mission_id`: UUID
  - `goal_attempt_id`: UUID
  - `current_target_id`: string (ví dụ "gate_1")
  - `distance_remaining`, `eta_seconds`, `timestamp`.
- **Nhận định tích hợp:**
  - Hiện tại, `nav_command_node.py` **hoàn toàn chưa có cơ chế kiểm tra trạng thái khóa khách hàng**. Khi nhận câu lệnh "dẫn tôi tới cổng 1", node lập tức gửi goal sang Nav2 kể cả khi chưa có người phía sau!
  - Cần chuyển bản hợp đồng `INTERFACES.md` cho agent voice ở Phase 9 để họ bổ sung cổng chặn: chỉ gửi Goal khi nhận được trạng thái `LOCKED` từ tracking.

### 4.2. Chuỗi Vận tốc Nav2 (`linorobot2_navigation`)
- Cấu hình kiểm tra: `linorobot2_navigation/config/navigation.yaml` (370 dòng).
- **Chuỗi vận tốc hiện tại:**
  1. `controller_server` (DWB Local Planner, tần số 20 Hz) tính toán quỹ đạo và xuất vận tốc.
  2. `velocity_smoother` (tần số 20 Hz, dòng 277–290):
     - `max_velocity: [0.5, 0.5, 2.5]` (đã mở vận tốc trục Y cho mecanum).
     - Xuất ra topic: `cmd_vel_smoothed`.
  3. `collision_monitor` (dòng 291–319):
     - `cmd_vel_in_topic: "cmd_vel_smoothed"`
     - `cmd_vel_out_topic: "cmd_vel"`
     - Giám sát vùng đa giác `FootprintApproach` qua laser scan `/scan` (thời gian trước va chạm 1.2s).
  4. Lệnh `/cmd_vel` được chuyển tới mô phỏng Gazebo hoặc base microcontroller của robot.
- **Kiểu dữ liệu:** Toàn bộ chuỗi vận tốc sử dụng `geometry_msgs/msg/Twist` (không dùng `TwistStamped`).
- **Vị trí tích hợp Guard tối ưu:**
  - Chuỗi an toàn chuẩn: `Nav2 Smoother -> Escort Velocity Guard -> Collision Monitor -> Base`.
  - Guard sẽ đọc `cmd_vel_smoothed`, áp dụng quy tắc an toàn tracking, xuất ra `cmd_vel_guarded`. `collision_monitor` sẽ nhận `cmd_vel_guarded` và đưa ra quyết định an toàn va chạm cuối cùng tới `/cmd_vel`.

---

## 5. Khảo sát Phần cứng GPU và Môi trường Runtime

- **Lệnh thực thi:** `nvidia-smi`
  - GPU: **NVIDIA RTX A3000 Laptop GPU** (Bus ID: 00000000:01:00.0).
  - Driver Version: `580.178.04`.
  - CUDA Version hỗ trợ tối đa của Driver: `13.0`.
  - VRAM: `6144 MiB` (~6 GB).
- **Môi trường Host vs Container:**
  - Host: Ubuntu 22.04 LTS (Jammy), Python 3.10.12. Chưa cài đặt PyTorch global.
  - Venv `/home/thuytran/ws/.venv_yolo`: Được tạo bằng Python 3.10 của host với `--system-site-packages`. Chưa có torch/ultralytics trong danh sách `pip list`.
  - Container đích: ROS 2 Jazzy (Ubuntu 24.04 Noble LTS, Python 3.12). File `devcontainer.json` đã cấu hình `--gpus all`, `NVIDIA_VISIBLE_DEVICES=all`.
  - File cache `__pycache__/*.cpython-312.pyc` trong `person_tracking` chứng minh code đã từng được chạy thành công trên Python 3.12 trong container.

---

## 6. Sơ đồ Luồng Dữ liệu: Hiện tại vs Đề xuất

### 6.1. Sơ đồ Luồng Hiện tại (Legacy Follower — Xung đột)
```text
[Astra RGB-D] ---> person_tracker ---> target_position (PointStamped)
                                              |
                                              v
[Nav2 Stack]  ---> cmd_vel (xung đột) <--- person_follower (tự xuất cmd_vel)
```

### 6.2. Sơ đồ Luồng Đề xuất (Escort Robot Architecture — Tuân thủ D01–D12)
```text
[Astra Camera / Replay / Fixture]
             |
             v
   passenger_perception (YOLO Pose + ByteTrack + Torso Depth + Nearest Selection)
             |
    tracking/observation
             v
   escort_tracking_manager (FSM: IDLE -> LOCKED -> NAVIGATING -> ARRIVED/ABORT)
         |               ^
         |               | /voice/nav_status (goal result / state)
         |               |
         v               +--- /tracking/session_state & safety_request
   [airport_robot_voice / nav_command_node] (SỞ HỮU DUY NHẤT NavigateToPose)
         |
         v
   [Nav2 Action: NavigateToPose]
         |
         v
   [Nav2 Controller & Velocity Smoother]
         |
    cmd_vel_smoothed
         v
   escort_velocity_guard (Giảm tốc khi khách xa, ép vận tốc 0 khi mất khách)
         |
    cmd_vel_guarded
         v
   [Collision Monitor] (Bảo vệ va chạm bằng Lidar vòng ngoài)
         |
       cmd_vel
         v
   [Mecanum Base / Motor Driver]
```

---

## 7. Bảng Phân loại Mức độ Chứng minh (Proof Classification)

| Nội dung | Mức độ kiểm chứng | Bằng chứng kỹ thuật |
|---|---|---|
| Đường dẫn workspace thật `/home/thuytran/ws` | **ĐÃ CHỨNG MINH (PROVEN)** | Lệnh `readlink -f .`, `git rev-parse --show-toplevel`. |
| Cấu trúc Dev Container Jazzy & GPU passthrough | **ĐÃ CHỨNG MINH (PROVEN)** | File `devcontainer.json`, `Dockerfile`, `nvidia-smi`. |
| File trọng số YOLO `yolo26n-pose.pt` | **ĐÃ CHỨNG MINH (PROVEN)** | Đọc header binary: PyTorch zip archive 7.87 MB. |
| Topics, encodings, frames của Astra Driver | **SUY DIỄN MÃ NGUỒN (STATIC_SOURCE)** | Đọc trực tiếp `ob_camera_node.cpp`, `astra.launch.xml`. |
| Quyền sở hữu Nav2 thuộc về Voice | **SUY DIỄN MÃ NGUỒN (STATIC_SOURCE)** | Đọc `airport_robot_voice/nav_command_node.py` (ActionClient). |
| Chuỗi vận tốc `cmd_vel_smoothed` & `collision_monitor` | **SUY DIỄN MÃ NGUỒN (STATIC_SOURCE)** | Đọc `linorobot2_navigation/config/navigation.yaml`. |
| Kiểu dữ liệu vận tốc `geometry_msgs/Twist` | **SUY DIỄN MÃ NGUỒN (STATIC_SOURCE)** | Đọc `navigation.yaml`, `gazebo.launch.py`. |
| Luồng live stream camera Astra phần cứng | **CHƯA KIỂM CHỨNG (UNVERIFIED_HW)** | Camera không có tại chỗ; bắt buộc dùng video/fake/synthetic test. |
| Hiệu năng FPS / Latency thực tế của YOLO trên GPU | **CHƯA KIỂM CHỨNG (UNVERIFIED_HW)** | Sẽ benchmark ở Phase 2 và Phase 10. |
| Hành vi động học dạt ngang Mecanum (`linear.y`) | **CHƯA KIỂM CHỨNG (UNVERIFIED_HW)** | Giữ `linear.y = 0` cho đến khi kiểm chứng an toàn ở Phase 7. |

---

## 8. Kết luận Gate 0

- Toàn bộ hiện trạng hệ thống đã được audit rõ ràng, minh bạch, có dẫn chứng dòng code và file cụ thể.
- Đã xác định toàn bộ các rủi ro kỹ thuật và xung đột cấu trúc.
- Không có bất kỳ dòng code nào bị thay đổi ngoài allowlist. Không có file nào bị xóa.
- Các tài liệu chuẩn của phân hệ đã được thiết lập đầy đủ trong `.agent/person_tracking/`.
- **Đủ điều kiện nghiệm thu Phase 0 (Gate 0: PASS).** Sẵn sàng chuyển sang Phase 1 khi có lệnh của chủ dự án.
