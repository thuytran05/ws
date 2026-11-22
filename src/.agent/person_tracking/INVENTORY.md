# PERSON TRACKING — INVENTORY VÀ ĐÁNH GIÁ FILE HIỆN CÓ

**Phân hệ:** `person_tracking`  
**Ngày cập nhật:** 23/11/2026 — Phase 1 (Điều chỉnh lịch triển khai khớp WORKFLOW.md)  
**Trạng thái quyết định:** Đã đồng bộ với tiến độ 12 giai đoạn của dự án. Không thay đổi mã nguồn trong Phase 1.

---

## 1. Bảng kiểm kê toàn bộ file trong package `person_tracking`

| Đường dẫn tệp | Loại tệp | Vai trò hiện tại | Quyết định | Lý do & Phân tích kỹ thuật | Phase thực hiện theo WORKFLOW |
|---|---|---|---|---|---|
| `package.xml` | ROS 2 Metadata | Khai báo package `ament_python` v0.1.0, các dependencies `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `message_filters`. | **REFACTOR** | Cấu trúc chuẩn `ament_python`. Cần bổ sung dependencies (`std_msgs`, `std_srvs`, nav msgs) khi hoàn thiện các node FSM và Guard. | **Phase 2 & Phase 8** (Khi hoàn thiện các node mới và đóng gói launch E2E). |
| `setup.py` | Python Build Config | Đăng ký executables (`person_tracker`, `person_follower`), data files (launch, config, resource). | **REFACTOR** | Cần cập nhật `console_scripts` cho kiến trúc 3 node: `passenger_perception`, `escort_tracking_manager`, `escort_velocity_guard`. Bỏ executable `person_follower`. | **Phase 2** (Perception), **Phase 6** (Manager), **Phase 7** (Guard). |
| `setup.cfg` | Python Package Config | Cấu hình cài đặt scripts vào `lib/person_tracking`. | **KEEP** | Chuẩn của `ament_python`, không thay đổi. | Duy trì nguyên vẹn. |
| `README.md` | Tài liệu package | Mô tả cách chạy `person_tracker` và `person_follower` cũ, topics cũ, venv `.venv_yolo`. | **REFACTOR** | Cần viết lại phản ánh kiến trúc robot dẫn đường sân bay (robot đi trước, khách đi sau, guard vận tốc, Nav2 ownership bởi voice). | **Phase 11** (Nghiệm thu và bàn giao hệ thống). |
| `resource/person_tracking` | Ament index marker | Marker định danh package cho ROS 2 resource index. | **KEEP** | Bắt buộc cho `ament_python`. | Duy trì nguyên vẹn. |
| `config/person_tracking.yaml` | ROS 2 Params | Chứa tham số cho `/person_tracking/person_tracker` và `/person_tracking/person_follower`. | **REFACTOR** | Cần cập nhật tham số perception (`image_topic`, `depth_topic`, `conf`, `gesture_seconds`), bổ sung tham số FSM (`raise_hand_hold_sec`, `visibility_stop_sec`, `visibility_abort_sec`), tham số guard (`max_speed_limit`, `yaw_trim_limit`), và xóa params của `person_follower`. | **Phase 2** (Perception params), **Phase 6** (FSM params), **Phase 7** (Guard params). |
| `launch/tracking.launch.py` | ROS 2 Launch | Khởi chạy node `person_tracker` với tham số cấu hình. | **REFACTOR** | Cần hỗ trợ khởi chạy perception độc lập với các chế độ: live Astra, bag replay, hoặc synthetic video replay. | **Phase 2** (Offline perception testing). |
| `launch/following.launch.py` | ROS 2 Launch | Khởi chạy đồng thời `person_tracker` và `person_follower`. | **REPLACE** | Kiến trúc "following" trực tiếp xung đột với Nav2. Sẽ được thay thế bằng `escort.launch.py` khởi chạy trọn bộ escort stack có kèm guard và manager. | **Phase 8** (End-to-end integration launch). |
| `person_tracking/__init__.py` | Python Module Init | Package init rỗng. | **KEEP** | Chuẩn Python module. | Duy trì nguyên vẹn. |
| `person_tracking/control.py` | Python Utility | Hàm `commands(x, z, ...)` tính vận tốc theo sau và `slew(...)` giới hạn gia tốc. | **REFACTOR** | Hàm `slew()` hữu ích cho bộ lọc rate limiter của Guard. Hàm `commands()` bám đuôi người sẽ được thay thế bằng các hàm clamp vận tốc và yaw trimming an toàn. | **Phase 7** (Velocity guard implementation). |
| `person_tracking/person_tracker.py` | ROS 2 Node (Python) | Node nhận diện YOLO Pose + ByteTrack, phát hiện giơ tay (2s), đo depth thân người từ `16UC1` patch. | **REFACTOR** | Tái sử dụng pipeline trích xuất keypoint và đo depth torso bằng MAD filter; nâng cấp thuật toán chọn người gần nhất (D07) khi nhiều người giơ tay, xuất `tracking/observation`. | **Phase 3–5** (Multi-person, Hand raise, Depth geometry). |
| `person_tracking/person_follower.py` | ROS 2 Node (Python) | Node tính sai số khoảng cách/góc và publish trực tiếp `/cmd_vel` điều khiển robot bám theo người. | **REPLACE** | Thay thế hoàn toàn bằng `escort_velocity_guard.py` (chỉ nhận lệnh Nav2, can thiệp clamp/stop/yaw-trim). Giữ file cũ để tham khảo xử lý timestamp. | **Phase 7** (Velocity guard implementation). |
| `person_tracking/yolo_pose_node.py` | ROS 2 Node (Python) | Bản thử nghiệm 2D RGB ban đầu (nhận diện giơ tay qua RGB không có depth). | **KEEP (Tạm thời)** | Không đăng ký trong `setup.py`. Hữu ích làm tham chiếu logic nhận diện giơ tay 2D cho offline fixtures khi thiếu depth. | Tham khảo trong **Phase 2**, xem xét dọn dẹp ở **Phase 10**. |

---

## 2. File bên ngoài package `person_tracking` nhưng liên quan trực tiếp

| Đường dẫn tệp | Loại tệp | Hiện trạng | Đánh giá & Ranh giới trách nhiệm |
|---|---|---|---|
| `/home/thuytran/ws/yolo26n-pose.pt` | Model Weights | Tệp checkpoint PyTorch 7.87 MB tại root `ws`. | **KEEP**: Trọng số mô hình pose ứng viên cho Phase 3. Đường dẫn cấu hình sẽ trỏ về đây. |
| `/home/thuytran/ws/.venv_yolo/` | Virtualenv | Môi trường Python 3.12 trong container, chứa `torch 2.14.1` và `ultralytics 8.4.185`. | **READ-ONLY**: Được sử dụng bên trong container Dev Container Jazzy khi chạy node ở Phase 2+. |
| `linorobot2/linorobot2_navigation/config/navigation.yaml` | Nav2 Config | READ-ONLY. Khai báo `velocity_smoother` và `collision_monitor`. | **READ-ONLY**: Guard sẽ được kết nối trước Collision Monitor qua launch remap trong package tracking ở Phase 8, không sửa file gốc. |
| `airport_robot_voice/airport_robot_voice/nav_command_node.py` | Voice Node | READ-ONLY. Quản lý `NavigateToPose`. | **READ-ONLY**: Phối hợp qua tài liệu đề xuất `VOICE_INTEGRATION_REQUEST.md` để agent voice tự cập nhật ở Phase 9. |

---

## 3. Quy tắc Thực thi Phase 1

1. **Tuyệt đối không sửa file trong `src/person_tracking/` trong Phase 1.** Phase 1 chỉ tập trung hoàn thiện bộ tài liệu đặc tả, hợp đồng giao diện, kế hoạch kiểm thử, quyết định kiến trúc và skill AI trong `.agent/`.
2. **Các chỉnh sửa mã nguồn và cấu hình ROS sẽ được tiến hành tuần tự theo đúng Gate từ Phase 2 đến Phase 8.**
