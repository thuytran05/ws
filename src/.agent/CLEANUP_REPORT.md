# BÁO CÁO SAFE CODE CLEANUP — LOẠI BỎ CODE DƯ THỪA ĐÃ XÁC MINH

- **Baseline commit**: `03debba72d769f457ffd80374cc278546b775833`
- **Phạm vi thực hiện**: `src/airport_robot_voice/`, `src/person_tracking/`, và tài liệu liên quan trực tiếp trong `src/.agent/`.
- **Thành phần giữ nguyên tuyệt đối**: MPPI DiffDrive (`navigation.yaml`), `navigation_guard*.yaml`, RViz (`voice_navigation.rviz`), chuỗi an toàn `velocity_guard*` $\rightarrow$ `velocity_watchdog*` $\rightarrow$ `collision_monitor`, STT/NLU/TTS, cơ chế `GoalAttemptRecord`/late-callback.

---

## 1. Bảng tổng hợp các phát hiện đã xử lý

| ID | Vấn đề | File | Hành động | Bằng chứng | SLOC giảm | Kết quả test |
|---|---|---|---|---|---|---|
| `A-01` | Launch file trùng 100% với `voice_system.launch.py` | `airport_robot_voice/launch/voice_assistant.launch.py` | Chuyển thành wrapper `IncludeLaunchDescription` | `diff -u` trả về 0 dòng trước cleanup; `--show-args` khớp 100% sau cleanup (`SHOW_ARGS_IDENTICAL`) | **-175** | `voice_assistant.launch.py` chạy Gazebo MPPI PASS 4/4 |
| `A-02` | Prototype Phase 1/2 không còn dùng (`person_follower` pub thẳng `/cmd_vel`) | `person_tracker.py`, `person_follower.py`, `yolo_pose_node.py`, `control.py`, `tracking.launch.py`, `following.launch.py`, `person_tracking.yaml`, `setup.py`, `package.xml`, `README.md`, `COMMANDS_TRACKING.md` | Xóa code/launch/config lỗi thời; gỡ `console_scripts` & `std_srvs`/`message_filters`; cập nhật `README.md` & `COMMANDS_TRACKING.md` | Grep toàn workspace: 0 tham chiếu từ Phase 3–11 & test; `colcon build` & `ros2 pkg executables` chỉ còn 5 node chuẩn | **-485** prod, **-55** launch, **-29** cfg | `person_tracking` 131/131 pytest PASS; Phase 9 Gazebo Escort PASS 4/4 |
| `A-03` | Import không sử dụng & thứ tự shebang | `nlu.py`, `multi_person_tracker.py`, `perception_source_node.py`, `person_detector_node.py`, `rear_camera_geometry.py`, `source_adapters.py`, `tracking_manager_node.py` | Xóa 9 unused imports; đưa `import uuid` xuống sau shebang ở `perception_source_node.py` | Kiểm tra AST + đọc file xác nhận 0 tham chiếu | **-10** | 129/129 Voice + 131/131 Tracking pytest PASS |
| `A-05` | Biến `pkg_linorobot2_base` không sử dụng | `person_tracking/launch/gazebo_escort_integration.launch.py:19` | Xóa biến thừa | Đọc toàn file launch xác nhận không dùng | **-1** | `gazebo_escort_integration.launch.py --show-args` & runtime PASS |
| `A-06` + `B-01` | Lặp khối reset target + visuals và gán dư `self._confirmed_path = None` | `airport_robot_voice/airport_robot_voice/nav_command_node.py` | Thêm `_clear_target_and_visuals()` và thay thế các khối lặp giống hệt (giữ nguyên nhánh `PAUSED`, `REPLACE`, `:1910/:1925`) | `_clear_destination_visuals()` đã gọi `_clear_preview_path()` gán `_confirmed_path = None` | **-59** | 129/129 Voice pytest + Gazebo MPPI PASS |
| `B-03` | Lặp observer/spin/TTS ACK boilerplate trong test | `airport_robot_voice/test/test_planner_resolution.py` | Thêm helper cục bộ `spin_nodes`, `ack_tts`, `create_rviz_observer` và áp dụng cho Test 12 | Giữ nguyên 100% số test, timeout và assertion | **+6** *(chuẩn bị helper dùng chung)* | `test_planner_resolution.py` 15/15 PASS |
| `B-05` | Script verification cũ đã bị thay thế hoàn toàn | `airport_robot_voice/scripts/{run_gazebo_voice_verification.sh,verify_gazebo_voice.py,run_specific_cases.sh,verify_specific_cases.py}`, `person_tracking/test/{execute_phase11_software_smoke.sh,verify_phase11_software_smoke.py}` | Xóa script phase cũ không còn tham chiếu | Đã bị thay thế bởi `run_evidence_verification.sh`, `verify_gazebo_evidence.py`, `verify_escort_real_launch.py`; 0 tham chiếu từ doc/test | **-1,225** *(758 Voice + 467 Tracking)* | Bộ test chính & Phase 9 Gazebo Escort PASS |

---

## 2. Các phát hiện giữ nguyên và lý do

- **`A-04` (`SyntheticFixtureGenerator.generate_multi_person_scene`)**: Giữ nguyên vì là helper tạo synthetic fixture đa người có giá trị cho kiểm thử mở rộng sau này.
- **`B-02` (`on_goal_result` & `_on_delayed_cancel_terminal_result`)**: Giữ tách riêng theo nguyên tắc an toàn do liên quan trực tiếp đến race condition, `mission_id`, `attempt_id` và late callbacks (chỉ dọn các khối reset trùng lặp bên trong qua `B-01`).
- **`B-04` (`person_tracking/test/` cross-file helpers)**: Giữ nguyên để tránh refactor lớn trên nhiều file test đang ổn định.
- **`B-06` (`benchmark_offline_pipeline.py`, `profile_detector_pid.py`)**: Giữ nguyên ngoài phạm vi cleanup.
- **`C-01`, `C-02` & Nhóm `D`** (`nav_command_node.py` architecture, `navigation_guard*.yaml`, `velocity_guard*`, `velocity_watchdog*`, `passenger_lock_manager.py`, `use_sim_time`, `navigation.yaml` MPPI DiffDrive, `MPPI Local Trajectory` RViz): Giữ nguyên hoàn toàn.
- **`verify_t22b_t23_gazebo.py` & `trace_gate9_odom_peak.py`**: Giữ nguyên vì vẫn được tham chiếu trong `PROGRESS.md` và `evidence_summary.md` cho bằng chứng Phase 10.

---

## 3. Thống kê SLOC trước và sau cleanup

*(Sửa sai số thống kê trong báo cáo audit trước: tổng test và verification SLOC ban đầu của hai package là **14,959**, không phải 12,959.)*

| Hạng mục | Trước (SLOC) | Sau (SLOC) | Chênh lệch |
|---|---:|---:|---:|
| **Production SLOC (`airport_robot_voice`)** | 4,439 | 4,379 | **-60** |
| **Production SLOC (`person_tracking`)** | 3,628 | 3,143 | **-485** |
| **Tổng Production SLOC** | **8,067** | **7,522** | **-545** |
| **Test + Scripts SLOC (`airport_robot_voice`)** | 6,124 | 5,372 | **-752** |
| **Test + Support SLOC (`person_tracking`)** | 8,835 | 8,368 | **-467** |
| **Tổng Test + Verification SLOC** | **14,959** | **13,740** | **-1,219** |
| **Launch SLOC (2 package)** | 1,003 | 773 | **-230** |
| **Config SLOC (2 package)** | 2,823 | 2,794 | **-29** |
| **Tổng Launch + Config SLOC** | **3,826** | **3,567** | **-259** |

- **Tổng thay đổi Git (trước khi thêm báo cáo này)**: `27 files changed` (không tính log runtime tự ghi đè đã khôi phục), giảm ròng **~2,023 SLOC** trên production, launch, config, test và verification scripts.

---

## 4. Kết quả Build, Kiểm thử tự động & Gazebo

1. **`airport_robot_voice`**:
   - `colcon build --packages-select airport_robot_voice --symlink-install`: **Thành công (1.17s)**.
   - `diff -u` `--show-args` giữa `voice_system.launch.py` và `voice_assistant.launch.py`: **`SHOW_ARGS_IDENTICAL`**.
   - `scripts/test_nlu.py`: **61/61 đạt**.
   - `ROS_DOMAIN_ID=42 pytest src/airport_robot_voice/test/ -v`: **129 passed in 69.30s**.
2. **`person_tracking`**:
   - Clean build `build_tracking/person_tracking` + `install_tracking/person_tracking`: **Thành công (1.12s)**.
   - `ros2 pkg executables person_tracking`: chỉ còn `perception_source`, `person_detector`, `tracking_manager`, `velocity_guard`, `velocity_watchdog` (0 executable phát `/cmd_vel` trực tiếp).
   - `escort_navigation_real.launch.py --show-args` & `gazebo_escort_integration.launch.py --show-args`: **Hợp lệ**.
   - `ROS_DOMAIN_ID=42 pytest src/person_tracking/test/test_*.py -v`: **131 passed in 50.23s** (bao gồm audit zero `/cmd_vel` publishers và chuỗi Guard $ightarrow$ Watchdog $ightarrow$ Collision Monitor).
3. **Gazebo (`ROS_DOMAIN_ID=44`, MPPI DiffDrive không đổi)**:
   - Voice + `voice_assistant.launch.py` (wrapper) trên Gazebo + MPPI: `PASS_FIXED_GATE_A3`, `PASS_PAUSE_RESUME`, `PASS_REPLACE_AND_NEAREST_TOILET`, `PASS_CANCEL_AND_RESET`.
   - `execute_phase9_verification.sh` (Gazebo Escort + `tracking_manager` + `velocity_guard` + `velocity_watchdog` + `collision_monitor` + `nav_command_node`): **4/4 Ca PASSED 100%** (Ca 1 chặn khi chưa khóa, Ca 2 di chuyển vật lý `delta_d=0.848 m` với `max_vx=0.500 m/s`, Ca 3 mất khách dừng `0.419s` + cửa sổ odometry `2.98s`, Ca 4 đến đích `SUCCEEDED -> IDLE`).

---

## 5. Giới hạn kiểm thử còn tồn tại
- **Camera Astra thật**: **HARDWARE NOT TESTED** (không có phần cứng Astra kết nối trong môi trường container).
- **Ca di chuyển dài thuần MPPI trong `turtlebot3_world` ngoài hành lang hẹp**: Trong thử nghiệm độc lập ở `(-1.0, 0.22)` và `(-1.0, 1.42)` trên `turtlebot3_world`, `collision_monitor` kích hoạt chế độ tiếp cận chậm (`1.2s away from collision`) do tường/trụ gần và tải CPU headless (`5–6.6 Hz` vs `20 Hz`). Chuyển động thật từ ngoài goal tolerance đến đích `SUCCEEDED` đã được kiểm chứng đầy đủ qua `execute_phase9_verification.sh` (`delta_d=0.848 m`, `SUCCEEDED -> IDLE`).

---

## 6. Hướng dẫn Rollback
- Nếu cần hoàn nguyên toàn bộ cleanup:
  - `git revert <commit_tracking> <commit_voice>` (hoặc `git reset --hard 03debba72d769f457ffd80374cc278546b775833` nếu muốn quay về đúng baseline trước audit/cleanup).
- Nếu chỉ cần khôi phục riêng một file prototype Phase 1/2 của `person_tracking`:
  - `git checkout 03debba72d769f457ffd80374cc278546b775833 -- src/person_tracking/person_tracking/person_follower.py` (tương tự cho các file đã xóa).
