---
trigger: always_on
description: Ràng buộc dự án airport_robot_voice trên ROS 2 Jazzy
---

# Ràng buộc dự án airport_robot_voice

1. **Môi trường thực thi**:
   - Thư mục gốc dự án: `src` thuộc ROS workspace (`/home/ws/src` trong container ROS 2 Jazzy).
   - Thao tác trực tiếp qua Terminal của VS Code Dev Container (không dùng `docker exec`).
   - Mọi thao tác build (`colcon build`), chạy node (`ros2 run`), launch (`ros2 launch`) và kiểm thử thực thi trực tiếp trong môi trường container ROS 2 Jazzy.
   - Host là Ubuntu 22.04 với ROS 2 Humble. Không chạy lệnh ROS trực tiếp trên host.

2. **Quản lý tài liệu và AI artifacts**:
   - Tất cả tài liệu phục vụ AI phải nằm trong `.agent/`.
   - File đặc tả gốc: `.agent/airport_robot_voice/PROJECT_SPEC.md`.
   - File theo dõi duy nhất: `.agent/airport_robot_voice/PROGRESS.md`.
   - Khi bắt đầu phiên làm việc mới, đọc rule này, `PROJECT_SPEC.md` và `PROGRESS.md` trước khi sửa code.

3. **Cấu trúc package ROS**:
   - Toàn bộ code Python, cấu hình JSON, launch, test nằm đúng vị trí trong `airport_robot_voice/`.
   - Không di chuyển cấu trúc package nếu không có lý do kỹ thuật bắt buộc.
   - Bảo toàn các thay đổi người dùng và Git working tree.

4. **Yêu cầu chức năng cốt lõi**:
   - Hỗ trợ 2 chế độ vận hành: Voice độc lập (khi Nav2 chưa chạy) và Voice kết hợp Navigation.
   - NLU offline thuần Python, xử lý tiếng Việt có/không dấu, Unicode tổ hợp, phủ định, alias trọn token, kiểm tra tồn tại điểm đến trước khi điều hướng.
   - Quản lý vòng đời Nav2 đầy đủ: PAUSE, RESUME, CANCEL, REPLACE, STATUS; chọn điểm trong nhóm (restroom) qua độ dài đường đi `ComputePathToPose`.
   - Audio offline: Vosk cho STT, Piper / eSpeak cho TTS, hỗ trợ keyboard, file WAV và micro/loa thật khi có thiết bị. Chế độ log-only khi thiếu thiết bị.
