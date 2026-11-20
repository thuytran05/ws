# PERSON TRACKING — INVENTORY VÀ ĐÁNH GIÁ FILE HIỆN CÓ

**Phân hệ:** `person_tracking`  
**Ngày audit:** 20/11/2026 — Phase 0  
**Trạng thái quyết định:** Đề xuất kỹ thuật (chưa xóa, chưa refactor code trong Phase 0 per Rule G0.3 và Gate 0).

---

## 1. Bảng kiểm kê toàn bộ file trong package `person_tracking`

| Đường dẫn tệp | Loại tệp | Vai trò hiện tại | Quyết định đề xuất | Lý do & Phân tích kỹ thuật | Hướng di chuyển / Thay thế |
|---|---|---|---|---|---|
| `package.xml` | ROS 2 Metadata | Khai báo package `ament_python` v0.1.0, các dependencies `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `message_filters`. | **REFACTOR** | Cấu trúc cơ bản chuẩn `ament_python`. Cần bổ sung các message/service dependency mới nếu dùng interface chuẩn (`std_msgs`, `std_srvs`, custom msgs), cập nhật description và maintainer. | Giữ nguyên format, cập nhật dependency và node entrypoints ở Phase 1. |
| `setup.py` | Python Build Config | Đăng ký executables (`person_tracker`, `person_follower`), data files (launch, config, resource). | **REFACTOR** | Cần cập nhật danh sách `console_scripts` để phản ánh kiến trúc mới: tách `passenger_perception`, `escort_tracking_manager`, `escort_velocity_guard` (hoặc mock nodes cho test). Bỏ executable `person_follower`. | Giữ lại khung `setup.py`, đăng ký entry points mới ở Phase 1/Phase 6/Phase 7. |
| `setup.cfg` | Python Package Config | Cấu hình cài đặt scripts vào `lib/person_tracking`. | **KEEP** | Chuẩn của `ament_python`, không cần thay đổi. | Giữ nguyên. |
| `README.md` | Tài liệu package | Mô tả cách chạy `person_tracker` và `person_follower`, topics cũ, venv `.venv_yolo`. | **REFACTOR** | Tài liệu phản ánh kiến trúc cũ (robot bám theo người với `person_follower` xuất `/cmd_vel`). Cần viết lại phản ánh kiến trúc robot dẫn đường (robot đi trước, khách đi sau, guard vận tốc, Nav2 ownership bởi voice). | Viết lại ở Phase 11 khi hoàn thiện bàn giao. |
| `resource/person_tracking` | Ament index marker | Marker định danh package cho ROS 2 resource index. | **KEEP** | Bắt buộc cho `ament_python`. | Giữ nguyên. |
| `config/person_tracking.yaml` | ROS 2 Params | Chứa tham số cho `/person_tracking/person_tracker` và `/person_tracking/person_follower`. | **REFACTOR** | Các tham số hiện tại (`image_topic`, `depth_topic`, `conf`, `gesture_seconds`) tái sử dụng được một phần. Cần bổ sung tham số FSM (`raise_hand_hold_sec`, `visibility_stop_sec`, `visibility_abort_sec`), tham số guard (`max_speed_limit`, `yaw_trim_limit`), và xóa params của `person_follower`. | Refactor ở Phase 1 theo spec FSM và Velocity Guard. |
| `launch/tracking.launch.py` | ROS 2 Launch | Khởi chạy node `person_tracker` với tham số cấu hình. | **REFACTOR** | Launch logic sạch, hỗ trợ `params_file` và `python_executable`. Cần refactor để khởi chạy bộ node perception mới (hoặc cả stack perception + manager + guard khi cần). | Refactor ở Phase 2 / Phase 8. |
| `launch/following.launch.py` | ROS 2 Launch | Khởi chạy đồng thời `person_tracker` và `person_follower`. | **REPLACE** | Kiến trúc "following" trực tiếp đi ngược lại thiết kế hệ thống (robot dẫn đường, Nav2 điều khiển chính, tracking chỉ làm guard). | Thay thế bằng `escort.launch.py` hoặc `tracking_system.launch.py` trong Phase 8. |
| `person_tracking/__init__.py` | Python Module Init | Package init rỗng. | **KEEP** | Chuẩn Python. | Giữ nguyên. |
| `person_tracking/control.py` | Python Utility | Hàm `commands(x, z, ...)` tính vận tốc theo sau và `slew(...)` giới hạn gia tốc. | **REFACTOR** | Hàm `slew()` rất hữu ích cho bộ lọc vận tốc (rate limiter) của `escort_velocity_guard`. Nhưng hàm `commands()` được thiết kế để robot bám đuôi người (tăng tốc đuổi theo), không phù hợp với robot dẫn đường. | Giữ lại `slew()`, thay `commands()` bằng các hàm `guard_clamp_velocity()`, `trim_angular_yaw()` ở Phase 7. |
| `person_tracking/person_tracker.py` | ROS 2 Node (Python) | Node nhận diện YOLO Pose + ByteTrack, phát hiện giơ tay (2s), đo depth thân người từ `16UC1` patch. | **REFACTOR** | **Phần có giá trị lớn:** pipeline inference YOLO + ByteTrack, logic trích xuất depth vùng torso giữa vai và hông, bộ lọc `16UC1` sang mét, sync qua `ApproximateTimeSynchronizer`.<br>**Phần thiếu/lỗi:**<br>1. Không chọn người gần nhất khi nhiều người giơ tay (chọn người đầu tiên trong vòng lặp đạt 2s).<br>2. Không có `session_id`, không có FSM states.<br>3. Hard-code kiểm tra `rgb_msg.frame_id == depth_msg.frame_id == camera_color_optical_frame` (chỉ chạy khi depth registration bật).<br>4. Không tính góc lệch / bearing / confidence cho guard. | Tái cấu trúc thành module `passenger_perception.py` (hoặc giữ tên node nhưng refactor hoàn chỉnh logic chọn người gần nhất và xuất `tracking/observation`) ở Phase 3–5. |
| `person_tracking/person_follower.py` | ROS 2 Node (Python) | Node tính sai số khoảng cách/góc và publish trực tiếp `/cmd_vel` điều khiển robot bám theo người. | **REPLACE** | **Xung đột kiến trúc nghiêm trọng:** Module này tự sinh lệnh di chuyển cạnh tranh với Nav2. Trong dự án sân bay, Nav2 mới là người dẫn đường, voice quản lý goal, tracking chỉ được phép can thiệp bằng cách giảm tốc/dừng/sửa yaw nhẹ thông qua Velocity Guard. Không được publish thẳng `/cmd_vel` ghi đè Nav2. | Thay thế hoàn toàn bằng `escort_velocity_guard.py` ở Phase 7. Giữ file cũ ở Phase 0 để tham khảo cách xử lý timestamp và timeout. |
| `person_tracking/yolo_pose_node.py` | ROS 2 Node (Python) | Bản thử nghiệm 2D RGB ban đầu (nhận diện giơ tay qua RGB không có depth). | **KEEP (Tạm thời)** | Không được đăng ký trong `setup.py`. Code sạch về logic nhận diện giơ tay 2D (`hand_above_head`), hữu ích làm fallback khi depth hỏng hoặc cho unit test offline RGB-only. | Giữ làm tài liệu tham khảo cho Phase 2 (offline dataset/test fixture), sau đó xem xét dọn dẹp ở Phase 10 nếu không còn dùng. |

---

## 2. File bên ngoài package `person_tracking` nhưng liên quan trực tiếp

| Đường dẫn tệp | Loại tệp | Hiện trạng | Đánh giá / Lưu ý |
|---|---|---|---|
| `/home/thuytran/ws/yolo26n-pose.pt` | Model Weights | Tệp checkpoint PyTorch nặng ~7.87 MB tại root `ws`. | **KEEP**: Trọng số mô hình pose đã được chuẩn bị sẵn, cần đường dẫn tuyệt đối hoặc tham số cấu hình rõ ràng để node tìm thấy ở mọi thư mục làm việc. |
| `/home/thuytran/ws/.venv_yolo/` | Virtualenv | Môi trường Python 3.10 trên host. | **AUDIT NOTE**: Chứa thư viện host. Trong khi môi trường container đích là ROS 2 Jazzy (Python 3.12). Cần xác minh PyTorch trong container ở Phase 1. |
| `linorobot2/linorobot2_navigation/config/navigation.yaml` | Nav2 Config | READ-ONLY. Khai báo `velocity_smoother` và `collision_monitor`. | **PROPOSAL ONLY**: `collision_monitor` hiện nhận input từ `cmd_vel_smoothed`. Để chèn Guard an toàn trước Monitor, cần đề xuất remap hoặc cấu hình ở Phase 7/8, không tự ý sửa file này. |
| `airport_robot_voice/airport_robot_voice/nav_command_node.py` | Voice Node | READ-ONLY. Quản lý `NavigateToPose`. | **PROPOSAL ONLY**: Cần gửi giao diện `INTERFACES.md` sang agent voice để họ thêm điều kiện kiểm tra `LOCKED` và xử lý yêu cầu hủy từ tracking. Tuyệt đối không can thiệp mã nguồn voice. |

---

## 3. Kết luận và Kế hoạch xử lý file Phase 1

1. **Không xóa bất kỳ file nào trong Phase 0.** Toàn bộ 12 file của `person_tracking` được giữ nguyên vẹn để bảo toàn trạng thái hiện tại.
2. **Kế hoạch Phase 1:** Thiết lập đặc tả kỹ thuật, đóng băng giao diện `INTERFACES.md`, chuẩn bị khung node mới (Perception, FSM Manager, Velocity Guard) mà không xóa mã cũ trước khi có bản thay thế hoạt động được.
