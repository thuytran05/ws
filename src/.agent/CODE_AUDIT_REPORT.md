# BÁO CÁO AUDIT MÃ NGUỒN (READ-ONLY) — DEAD CODE, LOGIC TRÙNG LẶP & OVERENGINEERING

- **Commit HEAD đối chiếu**: `03debba72d769f457ffd80374cc278546b775833` (`fix lỗi nếu có 2 điểm đến thì phải so sánh khoảng cách`)
- **Trạng thái Git working tree trước & sau audit**: Sạch (chỉ phát sinh file báo cáo duy nhất này trong `.agent/`). Không chỉnh sửa bất kỳ production code, test, launch, config hay script nào.
- **Phạm vi audit**:
  - `src/airport_robot_voice/` (production, launch, config, rviz, scripts, test)
  - `src/person_tracking/` (production, launch, config, fixtures, test)
  - `src/linorobot2/linorobot2_navigation/config/` (chỉ đọc đối chiếu MPPI DiffDrive vs `navigation_guard*.yaml` & `voice_navigation.rviz`)
  - `src/.agent/` (tài liệu và log kiểm thử)

---

## 1. Thống kê mã nguồn & Lịch sử Git

### 1.1. Phân loại dòng vật lý (Physical Lines) và SLOC thực tế
*(SLOC Python được đo bằng `tokenize`, loại trừ dòng trắng, comment `#` và docstring độc lập; file non-Python loại dòng trắng và comment `#`/`//`.)*

| Package / Thư mục | Phân loại | Số file | Physical lines | SLOC thực tế |
|---|---|---:|---:|---:|
| **`airport_robot_voice`** | **Production code** | 7 | **5,322** | **4,439** |
| | Unit test (`test_*.py`) | 4 | 2,448 | 1,896 |
| | Integration test (`test_ros_lifecycle.py`, `test_phase9_escort_integration.py`) | 2 | 2,481 | 1,884 |
| | Test scripts (`scripts/`) | 9 | 2,954 | 2,344 |
| | Launch (`launch/`) | 2 | 422 | 378 |
| | Configuration (`config/`, `rviz/`) | 4 | 1,957 | 1,957 |
| | Documentation (`README.md`) | 1 | 126 | 82 |
| | Build/Meta (`setup.py`, `setup.cfg`, `package.xml`, `resource`) | 4 | 62 | 57 |
| | Auto-generated (`__pycache__`, `.pytest_cache`) | 33 | 33,034 | 32,373 |
| **`person_tracking`** | **Production code** | 16 | **4,564** | **3,628** |
| | Unit test (`test_*.py`) | 15 | 5,692 | 4,194 |
| | Integration / Verification scripts (`test/run_gate8*`, `verify_*`, `test_phase8*`, `test_t25_ros*`) | 6 | 3,342 | 2,626 |
| | Test & Diagnostic support (`test/benchmark_*`, `profile_*`, `check_*`, `trace_*`, `test_astra_*`, `test_map_*`) | 9 | 2,543 | 2,015 |
| | Test fixtures (`fixtures/*.json`, `generate_fixtures.py`) | 3 | 340 | 310 |
| | Launch (`launch/`) | 7 | 745 | 625 |
| | Configuration (`config/`, `urdf/`) | 6 | 948 | 866 |
| | Documentation (`README.md`) | 1 | 195 | 142 |
| | Build/Meta (`setup.py`, `setup.cfg`, `package.xml`, `resource`, binary fixtures `.npz`/`.mp4`/`.mcap`) | 9 | — *(chứa binary `.mcap` 61.6 MB)* | — |
| | Auto-generated (`__pycache__`, `.pytest_cache`) | 50 | 28,935 | 28,173 |
| **`.agent`** | Documentation (`*.md`) | 19 | 5,485 | 4,068 |
| | Test logs (`evidence/**`) | 78 | 20,461 | 19,985 |

- **Tổng Production code toàn dự án**: **9,886 physical lines / 8,067 SLOC** (`airport_robot_voice`: 5,322 phys / 4,439 SLOC; `person_tracking`: 4,564 phys / 3,628 SLOC).
- **Tổng Test + Verification + Diagnostic scripts**: **16,809 physical lines / 14,959 SLOC** (`airport_robot_voice`: 7,883 phys / 6,124 SLOC; `person_tracking`: 11,577 phys / 8,835 SLOC — chưa tính binary).

### 1.2. Các file Production code lớn nhất (ưu tiên audit)
1. `airport_robot_voice/airport_robot_voice/nav_command_node.py` — **2,645 phys / 2,242 SLOC** (chiếm **50.5%** toàn bộ production SLOC của Voice; có 12 hàm $\ge 60$ dòng, lớn nhất `on_goal_result` 174 dòng, `__init__` 164 dòng, `_on_delayed_cancel_terminal_result` 145 dòng, `on_goal_response` 132 dòng).
2. `airport_robot_voice/airport_robot_voice/faster_whisper_worker.py` — **780 phys / 666 SLOC** (`_worker_main` 120 dòng, `__init__` 107 dòng).
3. `airport_robot_voice/airport_robot_voice/stt_node.py` — **755 phys / 647 SLOC** (`__init__` 124 dòng, `_mic_loop_faster_whisper` 114 dòng).
4. `person_tracking/person_tracking/passenger_lock_manager.py` — **646 phys / 510 SLOC** (`process_observation` 232 dòng).
5. `person_tracking/person_tracking/source_adapters.py` — **639 phys / 536 SLOC** (`process_messages` 181 dòng).
6. `airport_robot_voice/airport_robot_voice/nlu.py` — **580 phys / 447 SLOC** (`process` 225 dòng).
7. `person_tracking/person_tracking/multi_person_tracker.py` — **500 phys / 408 SLOC** (`process_frame` 178 dòng).
8. `person_tracking/person_tracking/velocity_guard.py` — **471 phys / 304 SLOC** (`process_velocity` 96 dòng, `update_observation` 93 dòng, `update_session_state` 90 dòng).
9. `person_tracking/person_tracking/tracking_manager_node.py` — **430 phys / 352 SLOC** (`_on_voice_status` 117 dòng).

### 1.3. Lịch sử Git & điểm nóng biến động (30 commit gần nhất)
- **Commit tăng dung lượng lớn nhất**: `930a375` (`+13,825 -2,725`, chủ yếu do log `.csv` và verification scripts Phase 10); `23b319b` (`+4,210 -341`); `29a4578` (`+4,189 -5`); `53b79a1` (`+3,715 -1,991`); `a03ec13` (`+3,535 -844`).
- **File code biến động nhiều nhất**: `nav_command_node.py` (`+1,223 -162` qua 9 commit), `run_gate8_real_gazebo_nav2.py` (`+1,937 -471` qua 4 commit), `verify_gazebo_evidence.py` (`+1,340 -486` qua 4 commit), `test_velocity_guard.py` (`+777 -71` qua 3 commit), `test_planner_resolution.py` (`+807 -21` qua 3 commit).
- **Chức năng sửa nhiều lần**: (1) Vòng đời cancel/replace/late callback và RViz topic ownership trong `nav_command_node.py`; (2) Verification scripts & CSV/log evidence của Phase 8–11.

---

## 2. Chi tiết phát hiện theo phân loại (A / B / C / D)

### Nhóm A — Dead code hoặc dư thừa đã được chứng minh

- **ID: `A-01`** | **Package**: `airport_robot_voice` | **File**: `launch/voice_assistant.launch.py:1-211` vs `launch/voice_system.launch.py:1-211`
  - **Mô tả**: Hai file launch giống hệt nhau **100% từng byte** (`diff -u` trả về 0 dòng khác biệt).
  - **Bằng chứng**: Chạy `diff -u airport_robot_voice/launch/voice_system.launch.py airport_robot_voice/launch/voice_assistant.launch.py` $\rightarrow$ rỗng.
  - **Phân tích hành vi**: Khi sửa một launch file mà quên file còn lại sẽ gây lệch cấu hình (từng phải vá đồng thời ở các commit trước).
  - **Đề xuất**: Giữ `voice_system.launch.py` làm file chính; biến `voice_assistant.launch.py` thành wrapper `IncludeLaunchDescription` (~10 dòng) để giữ tương thích tên gọi cũ.
  - **SLOC có thể giảm**: ~180 SLOC | **Chắc chắn**: Rất cao | **Rủi ro**: Rất thấp | **Test**: Launch check `voice_assistant.launch.py` và `voice_system.launch.py`.

- **ID: `A-02`** | **Package**: `person_tracking` | **File**: `person_tracking/person_tracker.py:1-243`, `person_follower.py:1-121`, `yolo_pose_node.py:1-159`, `control.py:1-22`, `launch/tracking.launch.py:1-26`, `launch/following.launch.py:1-36`, `config/person_tracking.yaml:1-30`
  - **Mô tả**: Cụm prototype Phase 1/2 (`PersonTracker` dùng `ApproximateTimeSynchronizer` + `Trigger`, `PersonFollower` publish trực tiếp `/cmd_vel` tại `:34`, `PersonPoseNode` không có entry point trong `setup.py`, `control.py` chỉ phục vụ `person_follower.py`).
  - **Bằng chứng**: Grep toàn repo xác nhận **0** tham chiếu từ bất kỳ node Phase 3–11, launch Escort (`gazebo_escort_integration.launch.py`, `escort_navigation_real.launch.py`, `perception_pipeline.launch.py`), hoặc file `test_*.py` nào. `person_follower.py:34` phát thẳng `/cmd_vel` (vi phạm kiến trúc an toàn Watchdog/Collision Monitor hiện tại).
  - **Đề xuất**: Di chuyển vào thư mục `legacy/` hoặc gỡ khỏi `setup.py` `entry_points` (`:35-36`) và `package.xml` (`std_srvs` `:14`, `message_filters` `:16`) sau khi xác nhận không còn kịch bản Phase 1/2 cần chạy lại.
  - **SLOC có thể giảm**: ~536 SLOC production + 62 lines launch/config | **Chắc chắn**: Cao | **Rủi ro**: Thấp (cần xác nhận không dùng lại prototype Phase 1/2) | **Test**: `colcon build` + toàn bộ `pytest person_tracking/test/`.

- **ID: `A-03`** | **Package**: `person_tracking` & `airport_robot_voice` | **File**:
  - `airport_robot_voice/airport_robot_voice/nlu.py:19` (`import os`)
  - `person_tracking/person_tracking/multi_person_tracker.py:16,38` (`Dict`, `TargetGeometryResult`)
  - `person_tracking/person_tracking/perception_source_node.py:31` (`create_simulated_camera_info`; lưu ý `:1` `import uuid` nằm trước shebang `:2`)
  - `person_tracking/person_tracking/person_detector_node.py:18` (`Dict`, `List`, `Tuple`)
  - `person_tracking/person_tracking/rear_camera_geometry.py:14` (`field`)
  - `person_tracking/person_tracking/source_adapters.py:18` (`Image`)
  - `person_tracking/person_tracking/tracking_manager_node.py:19` (`List`)
  - **Bằng chứng**: Xác minh bằng `ast.walk` + đọc file trực tiếp: tất cả các symbol trên không xuất hiện ở bất kỳ đâu khác trong file tương ứng.
  - **Đề xuất**: Xóa các import không sử dụng và đưa `import uuid` xuống sau shebang.
  - **SLOC có thể giảm**: ~10 SLOC | **Chắc chắn**: Rất cao | **Rủi ro**: Không | **Test**: `pytest` toàn bộ hai package.

- **ID: `A-04`** | **Package**: `person_tracking` | **File**: `person_tracking/source_adapters.py:267-311` (`SyntheticFixtureGenerator.generate_multi_person_scene`)
  - **Mô tả**: Phương thức tĩnh tạo frame giả lập nhiều người không có bất kỳ caller nào.
  - **Bằng chứng**: Grep toàn `/home/ws/src` $\rightarrow$ chỉ xuất hiện đúng 1 lần tại định nghĩa `:267`.
  - **Đề xuất**: Xóa nếu không có kế hoạch viết test synthetic đa người dùng hàm này.
  - **SLOC có thể giảm**: ~40 SLOC | **Chắc chắn**: Cao | **Rủi ro**: Thấp | **Test**: `pytest person_tracking/test/test_source_adapters.py`.

- **ID: `A-05`** | **Package**: `person_tracking` | **File**: `launch/gazebo_escort_integration.launch.py:19`
  - **Mô tả**: Khai báo `pkg_linorobot2_base = FindPackageShare('linorobot2_base')` nhưng không sử dụng biến này trong launch description.
  - **Bằng chứng**: Đọc file launch xác nhận không có tham chiếu tiếp theo tới `pkg_linorobot2_base`.
  - **SLOC có thể giảm**: 1 SLOC | **Chắc chắn**: Rất cao | **Rủi ro**: Không.

- **ID: `A-06`** | **Package**: `airport_robot_voice` | **File**: `airport_robot_voice/nav_command_node.py` (12 vị trí: `:581, :606, :661, :1568, :1594, :1710, :1880, :1892, :2058, :2070, :2100, :2116, :2444, :2493, :2504`)
  - **Mô tả**: Gán `self._confirmed_path = None` lặp lại ngay bên cạnh `self._clear_destination_visuals(clear_debug=True)` dù `_clear_destination_visuals()` (`:447`) đã gọi `self._clear_preview_path()` (`:426`) để gán `self._confirmed_path = None` và phát `Path(poses=[])`.
  - **Phân tích hành vi**: Khi `_clear_destination_visuals()` chạy trước hay sau thì `_confirmed_path` đều thành `None`. Ngoại lệ duy nhất cần giữ nguyên thứ tự là trong `handle_replace_target()` khi `PAUSED` với `valid_key` (`:2493-2496`): `_clear_destination_visuals()` chạy trước `_publish_confirmed_destination_visuals()`.
  - **SLOC có thể giảm**: ~12 SLOC | **Chắc chắn**: Rất cao | **Rủi ro**: Không | **Test**: `test_planner_resolution.py`, `test_ros_lifecycle.py`.

---

### Nhóm B — Logic có thể đơn giản hóa

- **ID: `B-01`** | **Package**: `airport_robot_voice` | **File**: `airport_robot_voice/nav_command_node.py` (14 nhánh kết thúc/lỗi tại `:577-581, 602-606, 657-661, 1281-1284, 1295-1298, 1347-1350, 1563-1568, 1590-1594, 1706-1710, 1875-1880, 1887-1892, 2053-2058, 2065-2070, 2095-2100, 2111-2116, 2437-2444`)
  - **Mô tả**: Lặp lại cùng cụm reset mục tiêu + dọn RViz (`_clear_destination_visuals`, `state = IDLE`, `_current_target_key/name/pose = None`, xóa `_pending_replace_*`). Đáng chú ý tại `:1910-1915` và `:1925-1930` (`_on_delayed_cancel_terminal_result` nhánh không ở `CANCELING`) lại **thiếu** `_clear_destination_visuals(clear_debug=True)` do copy-paste lệch phiên bản.
  - **Đề xuất**: Tạo 1 helper `_reset_active_target(clear_debug=True, clear_replace=False)` và dùng thống nhất ở các nhánh về `IDLE` (vẫn giữ nguyên các điều kiện guard trước khi gọi).
  - **SLOC có thể giảm**: ~55–70 SLOC | **Chắc chắn**: Cao | **Rủi ro**: Thấp–Trung bình | **Test**: Toàn bộ `test_ros_lifecycle.py`, `test_planner_resolution.py`, `test_phase9_escort_integration.py`.

- **ID: `B-02`** | **Package**: `airport_robot_voice` | **File**: `airport_robot_voice/nav_command_node.py:1810-1955` (`_on_delayed_cancel_terminal_result`) vs `:1956-2130` (`on_goal_result`)
  - **Mô tả**: Hai hàm dài 145 và 174 dòng chia sẻ ~100 dòng logic giống nhau: kiểm tra `is_terminal_confirmed`, map `status_str` + gọi `publish_nav_status`, rẽ nhánh `self.state == CANCELING` (`CANCELED`/`SUCCEEDED`/`ABORTED`/`UNKNOWN`), và rẽ nhánh `CANCELED`/`SUCCEEDED`/`ABORTED` khi không ở `CANCELING`.
  - **Đề xuất**: Tách helper chung `_handle_terminal_status_common(...)` sau bước kiểm tra ownership (`mission_id`/`attempt_id`) và `_confirm_attempt_terminal()`, đồng thời vá luôn thiếu sót `_clear_destination_visuals` ở `:1910/:1925`.
  - **SLOC có thể giảm**: ~80–95 SLOC | **Chắc chắn**: Cao | **Rủi ro**: Trung bình (liên quan late callback & race condition — cần giữ chặt thứ tự kiểm tra `rec.is_terminal_confirmed`) | **Test**: `test_ros_lifecycle.py` (38 tests) + `test_phase9_escort_integration.py` (13 tests).

- **ID: `B-03`** | **Package**: `airport_robot_voice` | **File**: `airport_robot_voice/test/test_planner_resolution.py` (đặc biệt Tests 9, 12, 13, 14, 15)
  - **Mô tả**: Lặp `QoSProfile(depth=1, TRANSIENT_LOCAL, RELIABLE)` (5 lần), tạo `obs` + 4–5 subscriptions giống nhau, hàm `spin_all` (5 lần), `on_tts_ack` JSON payload (7 lần), và boilerplate `MultiThreadedExecutor` (Tests 13–14).
  - **Đề xuất**: Gom vào 2 helper cục bộ trong `test_planner_resolution.py` (`_create_rviz_observer`, `_ack_tts`, `_ThreadedMockServer` context manager) — giữ nguyên 100% số lượng test và assertion.
  - **SLOC có thể giảm**: ~160–220 SLOC test | **Chắc chắn**: Rất cao | **Rủi ro**: Thấp | **Test**: `pytest airport_robot_voice/test/test_planner_resolution.py`.

- **ID: `B-04`** | **Package**: `person_tracking` | **File**: `person_tracking/test/*.py` (9+ file)
  - **Mô tả**: Lặp lại mock candidate 17-keypoints & `/tracking/observation` builder tại 8 file (`test_tracking_manager.py`, `test_phase6_regressions_verified.py`, `test_session_fsm_mock_voice.py`, `test_velocity_guard.py`, `test_velocity_guard_ros.py`, `test_phase8_gazebo_nav2_integration.py`, `test_t25_stress_reliability.py`, `test_t25_ros_runtime_stress.py`); lặp `_spin_all`/`_spin_both` tại 3 file; lặp zero-`/cmd_vel` graph audit tại 7 file; `test_rear_camera_geometry.py:44-50` tự viết `_create_mock_camera_info` dù đã import `create_simulated_camera_info` tại `:39`.
  - **Đề xuất**: Dùng chung `conftest.py` hoặc `test_helpers.py` trong `person_tracking/test/` cho mock observation/session/spin/cmd_vel audit.
  - **SLOC có thể giảm**: ~350–450 SLOC test | **Chắc chắn**: Cao | **Rủi ro**: Thấp | **Test**: Toàn bộ `pytest person_tracking/test/`.

- **ID: `B-05`** | **Package**: `person_tracking` & `airport_robot_voice` | **File**:
  - `person_tracking/test/execute_odom_trace.sh`, `execute_phase11_software_smoke.sh`, `execute_t22b_t23_gazebo.sh` (dòng 1–85 giống hệt nhau)
  - `person_tracking/test/verify_phase11_software_smoke.py` (493 dòng) bị `verify_escort_real_launch.py` (584 dòng) bao phủ gần như toàn bộ; `verify_t22b_t23_gazebo.py` (435 dòng) trùng lặp T22B/T23 với `run_gate8_real_gazebo_nav2.py`.
  - `airport_robot_voice/scripts/run_gazebo_voice_verification.sh` + `verify_gazebo_voice.py` (Phase 5/6) và `run_specific_cases.sh` + `verify_specific_cases.py` (Phase 6) bị `run_evidence_verification.sh` + `verify_gazebo_evidence.py` (Phase 7/8) bao phủ phần lớn.
  - **Đề xuất**: Chỉ giữ các script đại diện chuẩn hiện hành hoặc lưu trữ các script phase cũ vào thư mục `scripts/archive/` nếu không còn chạy định kỳ.
  - **SLOC có thể giảm**: ~1,200–1,600 SLOC script kiểm thử (không ảnh hưởng pytest) | **Chắc chắn**: Cao | **Rủi ro**: Thấp.

- **ID: `B-06`** | **Package**: `person_tracking` | **File**: `test/benchmark_offline_pipeline.py:138-139,219-220` & `test/profile_detector_pid.py:82`
  - **Mô tả**: Bug trong 2 script benchmark/profile: `benchmark_offline_pipeline.py` comment bỏ `queue_depths`/`dropped_frames` tại `:138-139` nhưng vẫn đọc tại `:219-220` (`NameError`); `profile_detector_pid.py:82` đọc `data.get('header', {}).get('stamp', {})` trong khi `person_detector_node.py:347` phát `"stamp"` ở top-level nên `latencies` luôn rỗng.
  - **Đề xuất**: Sửa hoặc đánh dấu deprecated nếu không còn sử dụng.

---

### Nhóm C — Vấn đề kiến trúc phức tạp

- **ID: `C-01`** | **Package**: `airport_robot_voice` | **File**: `airport_robot_voice/nav_command_node.py` (2,645 phys / 2,242 SLOC)
  - **Mô tả**: `NavCommandNode` đang gánh đồng thời 6 trách nhiệm lớn trong một file duy nhất: (1) Máy trạng thái mission & TTS synchronization; (2) Sequential `ComputePathToPose` candidate resolution; (3) Multi-attempt `GoalAttemptRecord` & late-callback/revocation lifecycle; (4) `PAUSE`/`RESUME`/`CANCEL`/`REPLACE` handling; (5) Escort Phase 9 gating & `/tracking/safety_request` handling; (6) RViz marker/path visualization.
  - **Đề xuất**: Không sửa ngay trong đợt cleanup nhỏ; nếu tiếp tục mở rộng, có thể tách thuần túy phần `rviz_visuals` helper và `GoalAttemptTracker` ra module riêng trong `airport_robot_voice/` để dễ đọc và test đơn vị.
  - **Rủi ro**: Trung bình–Cao nếu refactor vội (nhiều timer + callback bất đồng bộ tương tác chặt qua `self.state`).

- **ID: `C-02`** | **Package**: `person_tracking` | **File**: `config/navigation_guard.yaml` & `config/navigation_guard_test.yaml` vs `linorobot2/linorobot2_navigation/config/navigation.yaml`
  - **Mô tả**: Việc tách `navigation_guard.yaml` khỏi `navigation.yaml` là **chủ đích đúng đắn** (không gộp). Tuy nhiên sau khi `navigation.yaml` đã chuyển sang `MPPIController` (`motion_model: "DiffDrive"`), `navigation_guard.yaml:88-140` vẫn giữ cấu hình `dwb_core::DWBLocalPlanner` với thông số Mecanum Y (`min_vel_y: -0.5`, `max_vel_y: 0.5`) và `velocity_smoother` `[0.5, 0.5, 2.5]` (`:288-291`), đồng thời hardcode `amcl.initial_pose: (-1.0, -0.5)` (`:3-8`) của `turtlebot3_world` vào file launch cho robot thật (`escort_navigation_real.launch.py`). Ngoài ra cả 2 file đều có block `local_costmap.static_layer` bị bỏ khỏi danh sách `plugins`.
  - **Đề xuất**: Báo cáo chủ dự án để quyết định đồng bộ plugin/kinematics của `navigation_guard.yaml` sang MPPI DiffDrive (giữ nguyên cấu trúc file tách biệt và chuỗi Guard $\rightarrow$ Watchdog $\rightarrow$ Collision Monitor).

---

### Nhóm D — Phải giữ nguyên hoặc chưa đủ bằng chứng

- **ID: `D-01`** | **Package**: `person_tracking` | **File**: `person_tracking/velocity_guard.py`, `velocity_guard_node.py`, `velocity_watchdog.py`, `velocity_watchdog_node.py`, `passenger_lock_manager.py`, `launch/escort_navigation_real.launch.py`
  - **Lý do giữ nguyên**: Đây là toàn bộ chuỗi an toàn nhiều tầng độc lập (zero `/cmd_vel` ở perception/tracking, 7-check motion gate, steady-clock 20 Hz zeroing ở cả Guard và Watchdog, `SIGTERM`/`SIGINT` zero-Twist flush, fail-closed `ABORTING` latch, anti-replay stamp check, chặn map sim trên robot thật). **Tuyệt đối không được rút gọn hoặc gộp các tầng này.**
- **ID: `D-02`** | **Package**: `airport_robot_voice` | **File**: `intent_node.py:49`, `nav_command_node.py:116`, `stt_node.py:108`, `tts_node.py:84` (`declare_parameter('use_sim_time', False)`)
  - **Lý do giữ nguyên**: `use_sim_time` là tham số chuẩn của ROS 2, được `rclpy.Node` đọc nội bộ để chuyển `self.get_clock()` sang `/clock` trong Gazebo. AST báo "declared but not get_parameter" là đặc thù bình thường của ROS 2, **không phải dead code**.
- **ID: `D-03`** | **Package**: `airport_robot_voice` | **File**: `nav_command_node.py` (`GoalAttemptRecord`, `_revoked_attempts`, `_unresolved_attempts`, `_cancel_terminal_timers`, `on_goal_response`, `tts_policy`)
  - **Lý do giữ nguyên**: Bảo vệ race condition khi Nav2 chấp nhận goal muộn sau timeout/cancel/replace và chống gửi goal khi TTS chưa phát xong.
- **ID: `D-04`** | **Package**: `airport_robot_voice` & `person_tracking` | **File**: `stt_node.py` (Vosk fallback khi `stt_backend:=vosk`), `tts_node.py` (`log`, `espeak`, `aplay`), `faster_whisper_worker.py:191` (`UtteranceSegmenter.is_speaking`), `check_astra_hardware_topics.py`
  - **Lý do giữ nguyên**: Code dự phòng có chủ đích (`vosk`/`espeak`/`aplay`), property public của segmenter, hoặc công cụ kiểm tra phần cứng Astra thật chưa chạy được trong môi trường không có camera thật.
- **ID: `D-05`** | **Package**: `linorobot2` | **File**: `linorobot2/linorobot2_navigation/config/navigation.yaml`
  - **Lý do giữ nguyên**: Cấu hình MPPI DiffDrive do chủ dự án tinh chỉnh (`03debba`).
- **ID: `D-06`** | **Package**: `airport_robot_voice` | **File**: `rviz/voice_navigation.rviz` (`MPPI Local Trajectory` trỏ `/local_plan`, `Trajectories` trỏ `/marker`)
  - **Ghi nhận thực tế**: Trên Gazebo + MPPI (`ROS_DOMAIN_ID=44`), `/local_plan` có `Publisher count: 0` (MPPI dùng `/optimal_trajectory`), còn `/trajectories` tắt mặc định (`Enabled: false`). Chưa đủ bằng chứng xem chủ dự án có dùng `/local_plan` với plugin khác không, và nhiệm vụ này cấm sửa RViz — chỉ ghi nhận để chủ dự án cân nhắc đổi sang `/optimal_trajectory` khi cần debug MPPI.

---

## 3. Audit cấu hình, tài liệu & log tồn đọng

1. **File backup tracked trong Git**: `linorobot2/linorobot2_navigation/config/navigation.yaml.dwb.bak` (370 dòng) đang được track từ commit `03debba` — nên giữ ngoài Git hoặc đưa vào `.gitignore` khi chủ dự án dọn nhánh.
2. **Fixture đóng gói dư trong `person_tracking/setup.py:7-20`**: `fixtures/generate_fixtures.py`, `metadata.json`, `synthetic_empty.npz`, `synthetic_person_1.npz`, `synthetic_corrupted_depth.npz` và `fixtures/sample_fixture_bag/sample_fixture_bag_0.mcap` (61.6 MB) được cài đặt qua `setup.py` nhưng không có launch/node/test nào đọc (chỉ `fixtures/synthetic_video.mp4` được sử dụng).
3. **Log & Artifact trong `.agent/`**: 78 file log (20,461 dòng) — một số file lớn/lịch sử theo từng phase (`gazebo_verification_final.log` 326 KB, `phase_10/gate9_odom_peak_trace.csv` ~12.6k dòng trong git history). Không ảnh hưởng runtime, nhưng có thể lưu trữ/nén các log phase cũ sau khi chốt milestone.
4. **Cảnh báo `run_gate8_real_gazebo_nav2.py` / `execute_gate8_verification.sh:18-25`**: Vẫn còn `pkill -f gz` và `rm -f /dev/shm/fastrtps_*` ở script Phase 8 cũ (trong khi Phase 9–11 đã chuyển sang PGID tracking an toàn).

---

## 4. Bảng tổng hợp các phát hiện

| ID | Package | File:Line | Vấn đề | Bằng chứng | Phân loại | SLOC có thể giảm | Rủi ro | Đề xuất |
|---|---|---|---|---|---|---|---|---|
| `A-01` | `airport_robot_voice` | `launch/voice_assistant.launch.py:1-211` | Trùng 100% với `voice_system.launch.py` | `diff -u` trả về 0 dòng | **A** | ~180 | Rất thấp | Biến thành wrapper `IncludeLaunchDescription` |
| `A-02` | `person_tracking` | `person_tracker.py`, `person_follower.py`, `yolo_pose_node.py`, `control.py`, `tracking.launch.py`, `following.launch.py`, `person_tracking.yaml` | Prototype Phase 1/2 không còn dùng | 0 tham chiếu từ Phase 3–11 & test; `person_follower:34` pub thẳng `/cmd_vel` | **A** | ~598 | Thấp | Lưu trữ `legacy/` hoặc gỡ khỏi `setup.py`/`package.xml` |
| `A-03` | Cả 2 package | `nlu.py:19`, `multi_person_tracker.py:16,38`, `perception_source_node.py:31`, `person_detector_node.py:18`, `rear_camera_geometry.py:14`, `source_adapters.py:18`, `tracking_manager_node.py:19` | Import không sử dụng (+ `import uuid` trước shebang ở `perception_source_node.py:1`) | Kiểm tra AST & đọc file trực tiếp | **A** | ~10 | Không | Xóa import thừa |
| `A-04` | `person_tracking` | `source_adapters.py:267-311` | `generate_multi_person_scene` không có caller | Grep toàn workspace: 0 caller | **A** | ~40 | Thấp | Xóa nếu không dùng |
| `A-05` | `person_tracking` | `launch/gazebo_escort_integration.launch.py:19` | Biến `pkg_linorobot2_base` không sử dụng | Đọc toàn file launch | **A** | 1 | Không | Xóa biến thừa |
| `A-06` | `airport_robot_voice` | `nav_command_node.py` (12 vị trí) | Lặp `self._confirmed_path = None` dù `_clear_destination_visuals()` đã gọi `_clear_preview_path()` | Đọc `:426, :447` và 12 call sites | **A** | ~12 | Không | Bỏ gán dư |
| `B-01` | `airport_robot_voice` | `nav_command_node.py` (14 nhánh reset) | Lặp khối reset target + visuals; thiếu `_clear_destination_visuals` tại `:1910, :1925` | Đọc `:577-2505` | **B** | ~60 | Thấp–TB | Tạo helper `_reset_active_target()` |
| `B-02` | `airport_robot_voice` | `nav_command_node.py:1810-2130` | Trùng ~100 dòng giữa `on_goal_result` & `_on_delayed_cancel_terminal_result` | Đọc trực tiếp 2 hàm | **B** | ~85 | Trung bình | Tách `_handle_terminal_status_common()` |
| `B-03` | `airport_robot_voice` | `test/test_planner_resolution.py` | Lặp `QoSProfile`, `spin_all`, `on_tts_ack`, `MultiThreadedExecutor` | Đếm trực tiếp (5–7 lần mỗi mẫu) | **B** | ~190 (test) | Thấp | Gom helper/fixture cục bộ |
| `B-04` | `person_tracking` | `test/test_*.py` (9+ file) | Lặp mock observation/session, spin loop, zero-`/cmd_vel` audit, `_create_mock_camera_info` | Đọc 9+ file test | **B** | ~400 (test) | Thấp | Gom vào `conftest.py` / helper chung |
| `B-05` | Cả 2 package | `scripts/` & `test/verify_*`, `execute_*` | Nhiều script verification phase cũ trùng lặp với script mới hơn | So sánh trực tiếp các script Phase 5–11 | **B** | ~1,400 (script) | Thấp | Lưu trữ script phase cũ nếu không còn dùng |
| `B-06` | `person_tracking` | `test/benchmark_offline_pipeline.py:219`, `profile_detector_pid.py:82` | Bug `NameError` / sai key `"stamp"` trong script benchmark/profile | Đọc trực tiếp `:138-220`, `:82` | **B** | — | Thấp | Sửa hoặc đánh dấu deprecated |
| `C-01` | `airport_robot_voice` | `nav_command_node.py` (2,645 phys) | File quá lớn, 6 trách nhiệm trong 1 class | Thống kê AST (12 hàm $\ge 60$ dòng) | **C** | — | TB–Cao | Chỉ tách module khi cần mở rộng tiếp |
| `C-02` | `person_tracking` | `config/navigation_guard*.yaml` | Dùng DWB + Mecanum Y & hardcode sim `initial_pose` trong config real guard | Đối chiếu với `navigation.yaml` (MPPI DiffDrive) | **C** | — | Trung bình | Chủ dự án quyết định đồng bộ MPPI |
| `D-01` | `person_tracking` | `velocity_guard*`, `velocity_watchdog*`, `passenger_lock_manager.py`, `escort_navigation_real.launch.py` | Cơ chế an toàn nhiều tầng độc lập | Đọc chi tiết logic fail-closed & 20 Hz zeroing | **D** | 0 | Rất cao | **Tuyệt đối giữ nguyên** |
| `D-02..D-06` | Cả 2 package | `use_sim_time`, `GoalAttemptRecord`, Vosk/eSpeak fallback, `navigation.yaml` MPPI, `MPPI Local Trajectory` | Tham số ROS chuẩn / chống race / fallback / chưa đủ bằng chứng | Kiểm tra runtime & cấu trúc ROS 2 | **D** | 0 | Cao | **Giữ nguyên** |

---

## 5. Trả lời 12 câu hỏi tổng kết bàn giao

1. **Tổng số dòng production code hiện tại của từng package?**
   - `airport_robot_voice`: **5,322 physical lines / 4,439 SLOC** (7 file).
   - `person_tracking`: **4,564 physical lines / 3,628 SLOC** (16 file).
   - Tổng cộng: **9,886 physical lines / 8,067 SLOC**.
2. **Tổng số dòng test code?**
   - `airport_robot_voice`: **4,929 physical lines / 3,780 SLOC** (unit + integration pytest), cộng **2,954 phys / 2,344 SLOC** trong `scripts/`.
   - `person_tracking`: **9,034 physical lines / 6,820 SLOC** (unit + integration), cộng **2,543 phys / 2,015 SLOC** benchmark/diagnostic support.
   - Tổng cộng (gồm script kiểm thử): **19,460 physical lines / 14,959 SLOC** (riêng pytest `test_*.py` / `test/` integration: **13,963 phys / 10,600 SLOC**).
3. **Những file nào đang quá phức tạp?**
   - Đứng đầu là `airport_robot_voice/nav_command_node.py` (2,645 phys / 2,242 SLOC, 12 hàm $\ge 60$ dòng), tiếp theo là `faster_whisper_worker.py` (780 phys), `stt_node.py` (755 phys), `passenger_lock_manager.py` (646 phys, `process_observation` 232 dòng), `source_adapters.py` (639 phys), `nlu.py` (580 phys, `process` 225 dòng).
4. **Có bao nhiêu phát hiện thuộc nhóm A, B, C, D?**
   - Nhóm **A**: 6 phát hiện (`A-01` đến `A-06`).
   - Nhóm **B**: 6 phát hiện (`B-01` đến `B-06`).
   - Nhóm **C**: 2 phát hiện (`C-01`, `C-02`).
   - Nhóm **D**: 6 nhóm thành phần (`D-01` đến `D-06`).
5. **Có bao nhiêu đoạn code thực sự dư thừa đã được chứng minh?**
   - **6 nhóm** (`A-01`..`A-06`), nổi bật nhất là `voice_assistant.launch.py` trùng 100% (211 dòng), cụm prototype Phase 1/2 trong `person_tracking` (4 file Python + 2 launch + 1 YAML), 9 unused imports, hàm `generate_multi_person_scene`, biến `pkg_linorobot2_base`, và 12 dòng `_confirmed_path = None` lặp lại.
6. **Có thể giảm bao nhiêu dòng production code mà không đổi hành vi?**
   - Nếu chỉ dọn nhóm A an toàn + hợp nhất helper nhóm B (`B-01`, `B-02`): khoảng **~167 SLOC** trong phần code active (`~145 SLOC` từ `nav_command_node.py` + `~22 SLOC` từ `A-03`/`A-04` + `~180 SLOC` launch `A-01`), và thêm **~536 SLOC** nếu lưu trữ/gỡ cụm prototype Phase 1/2 (`A-02`).
7. **Có thể giảm bao nhiêu dòng test code bằng cách tái sử dụng?**
   - Khoảng **~550–670 SLOC** trong `test_*.py` (`B-03` + `B-04`), và **~1,200–1,600 SLOC** nếu lưu trữ các script verification phase cũ đã bị thay thế (`B-05`).
8. **Có những node, topic, service hoặc tham số nào không còn cần thiết?**
   - Node/launch cũ Phase 1/2: `person_tracker`, `person_follower`, `yolo_pose_node.py`, `tracking.launch.py`, `following.launch.py`, `person_tracking.yaml` (và `std_srvs`, `message_filters` trong `person_tracking/package.xml`).
   - Tham số YAML: block `local_costmap.static_layer` trong `navigation.yaml` & `navigation_guard*.yaml` (do `static_layer` không nằm trong `plugins`); các tham số DWB trong `navigation_guard.yaml` nếu chuyển sang MPPI.
9. **Những thành phần nào tuyệt đối không nên loại bỏ?**
   - Toàn bộ chuỗi an toàn `velocity_guard*`, `velocity_watchdog*`, `passenger_lock_manager.py`, `escort_navigation_real.launch.py` (`D-01`); `navigation_guard*.yaml` tách biệt với `navigation.yaml`; cơ chế `GoalAttemptRecord`/late-callback/TTS strict (`D-03`); `use_sim_time` (`D-02`); Vosk/eSpeak/aplay fallback (`D-04`); cấu hình MPPI DiffDrive (`D-05`).
10. **Những vấn đề nào chưa đủ bằng chứng để kết luận?**
    - Liệu chủ dự án có còn cần chạy lại prototype Phase 1/2 (`person_tracker`/`person_follower`) hay không; kế hoạch đồng bộ `navigation_guard.yaml` sang MPPI DiffDrive; việc giữ hay đổi topic `/local_plan` $\rightarrow$ `/optimal_trajectory` trong `voice_navigation.rviz`; và các nhánh phần cứng Astra thật chưa test được trong container không có camera.
11. **Nếu thực hiện cleanup thì nên ưu tiên theo thứ tự nào?**
    1. Dọn unused imports & biến thừa (`A-03`, `A-05`, `A-06`) — rủi ro bằng 0.
    2. Wrapper hóa `voice_assistant.launch.py` (`A-01`).
    3. Hợp nhất `_reset_active_target()` và `_handle_terminal_status_common()` trong `nav_command_node.py` (`B-01`, `B-02`).
    4. Gom fixture/helper dùng chung cho test (`B-03`, `B-04`).
    5. Lưu trữ/gỡ cụm prototype Phase 1/2 (`A-02`) và script verification cũ (`B-05`) sau khi chủ dự án xác nhận.
12. **Những test nào phải chạy lại sau mỗi nhóm thay đổi?**
    - Sau thay đổi Voice (`A-01`, `A-03`, `A-06`, `B-01`, `B-02`, `B-03`): `colcon build --packages-select airport_robot_voice`, `scripts/test_nlu.py`, toàn bộ `pytest airport_robot_voice/test/` (129 tests), và kiểm thử Gazebo `TC01–TC08`.
    - Sau thay đổi Person Tracking (`A-02`, `A-03`, `A-04`, `A-05`, `B-04`): `colcon build --packages-select person_tracking`, toàn bộ `pytest person_tracking/test/`, `verify_escort_real_launch.py` / `execute_phase9_verification.sh`.
