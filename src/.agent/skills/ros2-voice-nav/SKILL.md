---
name: ros2-voice-nav
description: Workflow for developing and testing ROS 2 Jazzy voice assistant and Nav2 action lifecycle in Docker.
---

# ROS 2 Jazzy Voice & Nav2 Development Workflow

## 1. Môi trường thực thi (VS Code Dev Container)
- Môi trường: ROS 2 Jazzy (Ubuntu 24.04 Noble) chạy trực tiếp trong Dev Container.
- Thư mục làm việc: `/home/ws` (workspace), `/home/ws/src` (source code).
- Thao tác: Chạy trực tiếp trong terminal của VS Code (không cần dùng `docker exec`).
- Lệnh build:
  ```bash
  source /opt/ros/jazzy/setup.bash
  cd /home/ws
  colcon build --packages-select airport_robot_voice --symlink-install
  source /home/ws/install/setup.bash
  ```
- Chạy node:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
  ros2 run airport_robot_voice <node_name>
  ```

## 2. Vòng đời Nav2 và Action
- `ComputePathToPose`: tính đường đi cho các ứng viên cùng nhóm (ví dụ toilet). Lựa chọn đích có path length ngắn nhất.
- `NavigateToPose`: gửi goal, theo dõi feedback, nhận kết quả.
- Điều khiển: PAUSE (hủy goal hiện tại, lưu target pose, chuyển trạng thái PAUSED), RESUME (gửi lại goal tới target pose đang lưu), CANCEL (hủy goal, dọn trạng thái về IDLE), REPLACE (xác minh đích mới, hủy goal cũ, gửi goal mới).

## 3. Kiểm thử âm thanh & Mock
- Hỗ trợ STT: keyboard input qua stdin, WAV file qua test harness / Vosk recognizer, micro ALSA khi có thiết bị.
- Hỗ trợ TTS: Piper (onnx), eSpeak-ng, hoặc log-only (`--ros-args -p engine:=log`).
- Kiểm thử cô lập bằng pytest và ROS 2 test fixtures trước khi thử nghiệm với Gazebo / Nav2 thật.
