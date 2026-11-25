# person_tracking

ROS 2 Jazzy package cho hệ thống nhận diện, bám đuổi và giữ an toàn hành khách (Escort Mode) sử dụng YOLO Pose, ByteTrack, đo depth RGB-D đã đăng ký và chuỗi an toàn đa tầng (`velocity_guard` $\rightarrow$ `velocity_watchdog` $\rightarrow$ `collision_monitor`).

## Các node chính (Phase 3–11)

| Executable | Vai trò |
|---|---|
| `perception_source` | Đọc video replay, synthetic fixture hoặc giám sát nguồn ảnh Astra |
| `person_detector` | Suy luận YOLO Pose + ByteTrack + đo depth vùng thân & hình học camera sau |
| `tracking_manager` | Máy trạng thái phiên Escort, khóa người giơ tay & giao tiếp `/tracking/session_state` |
| `velocity_guard` | Kiểm soát cổng vận tốc 7 bước + kẹp động học, xuất `/tracking/cmd_vel_guarded` ở 20 Hz |
| `velocity_watchdog` | Giám sát độc lập (steady clock 20 Hz), xuất `/tracking/cmd_vel_watchdog` tới `collision_monitor` |

## Launch files

- `launch/perception_pipeline.launch.py` — chạy độc lập pipeline nhận diện (`perception_source` + `person_detector`).
- `launch/offline_replay.launch.py` — phát lại video/fixture offline.
- `launch/gazebo_sim_base.launch.py` — mô phỏng Gazebo nền (không bật node tracking/guard).
- `launch/gazebo_escort_integration.launch.py` — tích hợp đầy đủ Gazebo + Nav2 + Escort Tracking (`navigation_guard_test.yaml`).
- `launch/escort_navigation_real.launch.py` — khởi chạy cho robot thật (bắt buộc truyền `map:=...`, chặn map mô phỏng, dùng `navigation_guard.yaml` và giới hạn mặc định `0.30 m/s`, `0.20 rad/s`).

## Build & kiểm thử

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ws
colcon build --symlink-install --packages-select person_tracking
source /home/ws/install/setup.bash
python3 -m pytest src/person_tracking/test/ -v
```
