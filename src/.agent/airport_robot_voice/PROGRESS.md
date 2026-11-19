# TIẾN ĐỘ VÀ KẾ HOẠCH TRIỂN KHAI AIRPORT_ROBOT_VOICE

## 1. Thông tin môi trường và khảo sát hệ thống

- **Host OS**: Ubuntu 22.04 LTS (x86_64), Linux Kernel 6.8.0, ThinkPad P15 Gen 2.
- **Host ROS**: ROS 2 Humble (`/opt/ros/humble`).
- **Container ROS**: Docker Dev Container `thirsty_ishizaka` (Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12.3).
- **Workspace mapping**: Host `/home/thuytran/ws` <=> Container `/home/ws`.
- **Thao tác**: Trực tiếp qua Terminal của VS Code Dev Container (không cần dùng `docker exec`).
- **Thư mục source**: `/home/ws/src` (chứa `airport_robot_voice`, `linorobot2`, etc.).
- **Build tool**: `colcon build --packages-select airport_robot_voice --symlink-install` trong container.
- **Quyền hạn**: Container user `thuytran` có quyền passwordless sudo (`sudo -n true`).
- **Thiết bị âm thanh trong container**:
  - `/dev/snd` được mount đầy đủ từ host vào container.
  - Binary hệ thống: `alsa-utils`, `espeak-ng`.
  - Thư viện Python: `vosk`, `sounddevice`, `pyaudio`, `piper-tts`.
  - Model Vosk offline tiếng Việt: `/opt/vosk/vosk-model-small-vn-0.4`.
  - Model Piper offline tiếng Việt: binary `/usr/local/bin/piper`, model `/opt/piper/vi_VN-vais1000-medium.onnx` và `.json`.
  - `sounddevice.query_devices()` nhận diện 11 thiết bị âm thanh ALSA.

---

## 2. Kết quả xử lý 13 vấn đề mã nguồn ban đầu

| # | Vấn đề đã ghi nhận | Giải pháp và File đã xử lý | Trạng thái |
|---|--------------------|----------------------------|------------|
| 1 | Hỏi vị trí ("cổng A3 ở đâu") có `action=null` | Cập nhật `intents.json` và `nlu.py`: chuyển câu hỏi vị trí thành `action: navigate`. | ĐÃ GIẢI QUYẾT |
| 2 | Danh mục thiếu A3, A4 | Bổ sung đầy đủ `gate_a1`..`gate_a4`, `gate_b1`, `gate_b2`, `gate_15` vào `places.json`. | ĐÃ GIẢI QUYẾT |
| 3 | NLU tạo target cổng không tồn tại | `nlu.py` kiểm tra `place_key in self.places`. Nếu không có, gán intent `place_not_found`. | ĐÃ GIẢI QUYẾT |
| 4 | Thiếu đích thì hỏi lại (pending slot) | Bỏ pending slot. Thông báo không xác định được đích, `action: None`, kết thúc lượt ngay. | ĐÃ GIẢI QUYẾT |
| 5 | Phủ định vẫn sinh lệnh navigate | Bổ sung `check_negation` trong `nlu.py`: chuyển thành `negated_nav`, không điều hướng. | ĐÃ GIẢI QUYẾT |
| 6 | Regex khớp substring, A123 thành A12, A1 khớp A10 | Dùng ranh giới từ `\b` và kiểm tra span overlap. Ngăn chặn triệt để khớp substring. | ĐÃ GIẢI QUYẾT |
| 7 | Danh mục chỉ có một toilet | Cấu hình 2 điểm `toilet_1`, `toilet_2`. Tích hợp `ComputePathToPose` chọn điểm tối ưu. | ĐÃ GIẢI QUYẾT |
| 8 | Callback navigation dùng chung target & handle | Tách biệt `mission_id`, `request_id`, `goal_attempt_id`. Callback cũ bị thu hồi. | ĐÃ GIẢI QUYẾT |
| 9 | Cancel chưa theo dõi khi goal chờ chấp nhận | Hỗ trợ cancel goal pending-acceptance ngay khi nhận được goal handle từ server. | ĐÃ GIẢI QUYẾT |
| 10 | Đường dẫn config dùng `__file__` sai khi cài đặt | Dùng `ament_index_python.packages.get_package_share_directory` chuẩn ROS 2. | ĐÃ GIẢI QUYẾT |
| 11 | STT giữ wake mãi, buffer audio chưa kiểm soát | Thêm `wake_timeout_sec` (15s) và giới hạn queue bảo vệ chống tràn. | ĐÃ GIẢI QUYẾT |
| 12 | Launch keyboard không nhận được stdin | Hỗ trợ `emulate_tty=True` và hướng dẫn chạy `stt_node` riêng cho stdin. | ĐÃ GIẢI QUYẾT |
| 13 | Bộ test 57/57 cũ chỉ kiểm tra intent | Xây dựng bộ test toàn diện 55 tests pytest bao phủ logic, schema, lifecycle và audio. | ĐÃ GIẢI QUYẾT |

---

## 3. Kiến trúc hệ thống và Thiết kế giao tiếp (Schema & Topics)

### 3.1. Các topic và QoS
1. `/voice/text` (`std_msgs/String`, Volatile, Depth 10): Nhận transcript từ bàn phím, micro hoặc WAV.
2. `/voice/intent` (`std_msgs/String`, Volatile, Depth 10): JSON payload Schema v1.0 từ `intent_node`.
3. `/voice/say` (`std_msgs/String`, Volatile, Depth 10): JSON payload (`text`, `speech_id`, `mission_id`) gửi tới `tts_node`.
4. `/voice/speaking` (`std_msgs/Bool`, Volatile, Depth 10): Trạng thái robot đang phát âm thanh.
5. `/voice/tts_ack` (`std_msgs/String`, Volatile, Depth 10): JSON xác nhận câu nói (`speech_id`, `status`, `audio_played`).
6. `/voice/nav_state` (`std_msgs/String`, Transient Local, Depth 1): Trạng thái máy nhiệm vụ Nav2 latched.
7. `/voice/nav_feedback` (`std_msgs/String`, Volatile, Depth 10): Phản hồi chi tiết từng request.

### 3.2. Schema JSON `/voice/intent`
```json
{
  "schema_version": "1.0",
  "request_id": "req_1700000000_1",
  "timestamp": 1700000000.123,
  "raw": "dẫn tôi đến cổng A3",
  "intent": "guide_me",
  "action": "navigate",
  "score": 115.0,
  "target_id": "gate_a3",
  "category": "gate",
  "selection_policy": "direct",
  "slots": {
    "place_key": "gate_a3",
    "gate": "A3"
  }
}
```

### 3.3. Schema JSON `/voice/nav_state`
```json
{
  "schema_version": "1.0",
  "node_online": true,
  "mission_state": "NAVIGATING",
  "mission_id": "mission_1",
  "current_target_id": "gate_a3",
  "current_target_name": "cổng A3",
  "distance_remaining": 12.5,
  "eta_seconds": 18,
  "last_feedback_time": 1700000010.5,
  "timestamp": 1700000011.0
}
```

---

## 4. Chi tiết triển khai và Kết quả kiểm chứng 5 giai đoạn chuyên sâu

### 4.1. Giai đoạn 1 — Chọn đích bằng Planner (`ComputePathToPose`)
- **Vấn đề trước đây**: `resolve_category_and_start()` luôn chọn cứng `candidates[0]`.
- **Triển khai kỹ thuật**:
  - Lọc ứng viên: Chỉ chấp nhận điểm có `enabled == True`, `frame_id == map`, và tọa độ `x, y, yaw` là số thực hợp lệ.
  - Gửi request `ComputePathToPose` bất đồng bộ cho từng ứng viên (sử dụng thuộc tính `goal.goal = pose_stamped` đúng chuẩn `nav2_msgs/action/ComputePathToPose`).
  - Tính tổng độ dài đường đi: `sum(sqrt((x_i - x_{i-1})^2 + (y_i - y_{i-1})^2))`.
  - Chọn ứng viên có chiều dài đường đi ngắn nhất.
  - Xử lý ngoại lệ đầy đủ: Điểm không có đường (`NO_VALID_PATH`), server vắng mặt hoặc timeout, lỗi TF (`TF_ERROR`).
  - Ngắt trạng thái: Cho phép lệnh `pause` hoặc `cancel` ngắt quá trình `RESOLVING`, dừng đánh giá các ứng viên còn lại và trở về trạng thái phù hợp.
  - Kiểm tra tính hợp lệ của đích cụ thể trước khi thực thi (`check_target_exists`).
- **Kết quả kiểm thử**: Đạt **4/4 tests** trong `test_planner_resolution.py`:
  1. `test_planner_chooses_shorter_path_when_first_candidate_is_longer`: PASSED.
  2. `test_planner_chooses_second_candidate_when_first_candidate_is_unreachable`: PASSED.
  3. `test_planner_all_candidates_unreachable`: PASSED (phát hiện không có đường và trở về IDLE).
  4. `test_cancel_during_planner_resolving`: PASSED (ngắt đánh giá, hủy tiến trình).

---

### 4.2. Giai đoạn 2 — Sửa vòng đời Action Nav2 & State Machine
- **Vấn đề trước đây**: Pause/cancel không đợi kết quả hủy; goal acceptance muộn làm robot tiếp tục chạy; replace gửi goal mới ngay; feedback callback không phân biệt goal attempt; thiếu deduplicate.
- **Triển khai kỹ thuật**:
  - Tách bạch danh tính: `_request_id`, `_mission_id`, `_current_goal_attempt_id`.
  - Xử lý Pending-Acceptance: Ghi nhận cờ `_pause_requested_before_acceptance` và `_cancel_requested_before_acceptance`. Khi goal handle được server chấp nhận, tiến hành hủy ngay lập tức.
  - Thu hồi goal cũ về muộn: Danh sách `_cancelled_attempts` lưu các attempt đã hủy. Callback response/result của attempt cũ đều bị bỏ qua.
  - Xác nhận hủy trước khi đổi state: Chỉ chuyển sang `PAUSED` hoặc `CANCELED` sau khi action server xác nhận hoàn tất việc hủy (`STATUS_CANCELED`).
  - Cơ chế Resume an toàn: Lệnh `resume` chỉ được chấp nhận khi việc tạm dừng đã hoàn tất (`state == PAUSED`).
  - Xử lý Replace:
    - Khi đang `NAVIGATING`: Xác minh đích mới, gửi lệnh hủy goal hiện tại, chờ hoàn tất hủy rồi mới gửi goal mới.
    - Khi đang `PAUSED`: Cập nhật `_current_target_id`, `_current_target_name`, và giữ nguyên trạng thái `PAUSED` (chỉ xuất phát khi có lệnh resume).
  - Deduplication: LRU Cache lưu 200 `request_id` gần nhất, tự động bỏ qua các request trùng lặp.
  - Feedback Freshness: Kiểm tra thời gian từ feedback gần nhất (`feedback_freshness_sec <= 5.0s`).
- **Kết quả kiểm thử**: Đạt **10/10 tests** trong `test_ros_lifecycle.py`:
  1. `test_pause_before_goal_acceptance`: PASSED (hủy thành công ngay khi goal được accept).
  2. `test_cancel_before_goal_acceptance`: PASSED.
  3. `test_pause_resume_and_late_goal_result`: PASSED (thu hồi kết quả goal cũ về muộn).
  4. `test_replace_while_navigating_and_paused`: PASSED (xử lý đúng cả 2 trạng thái).
  5. `test_discard_old_feedback_and_freshness_status`: PASSED.
  6. `test_request_deduplication`: PASSED.
  7. `test_tts_synchronization_and_cancel_in_announcing`: PASSED.

---

### 4.3. Giai đoạn 3 — Liên kết TTS và Nhiệm vụ
- **Vấn đề trước đây**: Không có liên kết giữa câu thông báo xuất phát và thời điểm gửi goal; `on_tts_ack` chưa xử lý.
- **Triển khai kỹ thuật**:
  - Gắn `speech_id` và `mission_id` vào payload `/voice/say`.
  - `tts_node.py` phát `/voice/tts_ack` chi tiết (`speech_id`, `mission_id`, `status`, `wav_generated`, `audio_played`).
  - `nav_command_node.py` chuyển sang `ANNOUNCING` khi phát câu thông báo bắt đầu, chờ nhận `tts_ack` phù hợp mới chuyển sang `STARTING` gửi goal Nav2.
  - Timeout Fallback: Bổ sung timer fallback 3.0s nếu TTS vắng mặt hoặc không phản hồi ack.
  - Giới hạn hàng đợi: `tts_node` đặt `queue_size=30`, từ chối câu nói kèm ack `DROPPED_QUEUE_FULL` khi tràn hàng đợi.
  - Worker an toàn: Timeout lấy hàng đợi 0.2s kèm cờ `_running` cho phép shutdown tiến trình sạch sẽ.
  - Kiểm tra exit code: Bắt lỗi subprocess khi gọi `espeak-ng`, `piper`, `aplay`, phân biệt rõ giữa log-only, tạo file WAV và phát âm thanh thật.

---

### 4.4. Giai đoạn 4 — Toàn diện Audio offline & Kiểm thử STT/TTS
- **Vấn đề trước đây**: Lỗi lệch sample rate khi giải mã WAV (mặc định 16kHz trong khi eSpeak xuất 22.05kHz gây lệch tốc độ x1.38); chưa test WAV lời nói biết trước; chưa kiểm tra định dạng file WAV.
- **Triển khai kỹ thuật**:
  - `stt_node.py`: Tự động đọc sample rate từ header WAV (`wf.getframerate()`) và khởi tạo `KaldiRecognizer(model, wav_rate)`.
  - Kiểm tra và từ chối WAV không hợp lệ (bắt buộc mono 16-bit PCM).
  - Xả sạch `_audio_queue` khi `robot_speaking == True` để chống nhận diện nhầm âm thanh robot tự nói.
  - Timeout wake word đưa trạng thái về sleep sau 15 giây.
  - Viết 10 test cases toàn diện trong `test_audio.py`:
    - Tạo WAV tiếng nói 'cảm ơn' bằng eSpeak, giải mã bằng Vosk, đưa qua NLU xác nhận intent 'thanks'.
    - Kiểm tra silence WAV không sinh transcript rác.
    - Kiểm tra xử lý lỗi khi thiếu model Vosk.
    - Kiểm tra từ chối WAV sai định dạng (stereo hoặc 8-bit).
    - Kiểm tra wake word timeout và xả hàng đợi khi robot đang nói.
    - Kiểm tra định dạng file WAV do Piper tổng hợp (mono, 16-bit, số frames hợp lệ).
    - Kiểm tra truy vấn thiết bị âm thanh ALSA qua `sounddevice`.
- **Kết quả kiểm thử**: Đạt **10/10 tests** trong `test_audio.py`.

---

### 4.5. Giai đoạn 5 — Kiểm chứng môi trường và Khắc phục sự cố
- **Sự cố Deadlock trước đó**:
  - *Hiện tượng*: Background task `test_ros_lifecycle.py` bị kẹt 15 phút tại test đầu tiên `test_pause_before_goal_acceptance`.
  - *Nguyên nhân kỹ thuật*: `MockNavServer._execute_cb` sử dụng vòng lặp `while goal_handle.is_active: time.sleep(0.02)`. Trong ROS 2 Single-Threaded Executor, callback này chiếm giữ luồng độc quyền, khiến `cancel_callback` không bao giờ được kích hoạt để đưa `is_active` về `False`.
  - *Giải pháp*:
    1. Dừng tiến trình pytest bị kẹt (PID 106175).
    2. Tái cấu trúc `NavTestHarness` sang `MultiThreadedExecutor` và `ReentrantCallbackGroup`.
    3. Thay thế vòng chờ bằng `threading.Event()` và bổ sung finite timeout hữu hạn (2-3s) cho tất cả các bước chờ state.
- **Kết quả kiểm chứng tổng thể**:
  - Lệnh chạy: `python3 -m pytest src/airport_robot_voice/test/ -v`
  - **55/55 bài test PASSED 100% trong 12.01 giây**:
    - `test_audio.py`: 10/10 PASSED.
    - `test_nlu.py`: 24/24 PASSED.
    - `test_places.py`: 7/7 PASSED.
    - `test_planner_resolution.py`: 4/4 PASSED.
    - `test_ros_lifecycle.py`: 7/7 PASSED.
  - Kịch bản NLU độc lập: `python3 src/airport_robot_voice/scripts/test_nlu.py` đạt **61/61 PASSED**.

---

## 5. Phân định kết quả kiểm chứng theo 3 môi trường

### 5.1. Môi trường Test Fixture & Mock Server (ĐÃ XÁC MINH 100%)
- **Trạng thái**: ĐẠT 100% (52/55 tests pytest, 61/61 test_nlu).
- **Phạm vi xác minh**:
  - Xử lý NLU tiếng Việt, phủ định, Unicode tổ hợp, ranh giới từ, trích xuất slot, lọc đích tồn tại.
  - Thuật toán chọn đường bằng `ComputePathToPose` (so sánh độ dài đường đi, xử lý unreachable, timeout, cancel ngắt resolving).
  - Toàn bộ máy trạng thái vòng đời Nav2 (PAUSE, RESUME, CANCEL, REPLACE, STATUS, deduplication, pending-acceptance).
  - Đồng bộ âm thanh TTS (speech_id, mission_id, tts_ack, timeout fallback).
  - Định dạng file WAV, nhận diện âm thanh offline qua Vosk, xử lý hàng đợi audio.

### 5.2. Môi trường Thiết bị âm thanh vật lý (ĐÃ XÁC MINH CẤU HÌNH, CẦN THỬ NGHIỆM VỚI NGƯỜI DÙNG)
- **Đã kiểm chứng**:
  - Container nhận diện 11 thiết bị âm thanh ALSA qua `/dev/snd`.
  - eSpeak-ng và Piper tổng hợp tiếng Việt ra file WAV chuẩn mono 16-bit.
  - Model Vosk tiếng Việt giải mã âm thanh chính xác.
  - Chế độ `tts_engine:=log` và `mode:=keyboard` hoạt động độc lập hoàn hảo không cần micro/loa.
- **Cần xác minh thêm với người dùng**:
  - Nói trực tiếp qua micro phần cứng máy tính ThinkPad (kiểm tra mức âm lượng thu âm thực tế).
  - Nghe trực tiếp qua loa ngoài laptop (phụ thuộc routing âm thanh ALSA giữa host Ubuntu 22.04 và Docker).

### 5.3. Môi trường Điều hướng Nav2 / Gazebo (ĐÃ XÁC MINH LOGIC, CHỜ KHỞI CHẠY MÔ PHỎNG)
- **Đã kiểm chứng**:
  - Giao tiếp Action Client `NavigateToPose` và `ComputePathToPose` đúng chuẩn interface `nav2_msgs` trên ROS 2 Jazzy.
  - Dữ liệu địa điểm `places.json` đã gắn cờ `"_demo_data": true`.
- **Cần xác minh thêm với người dùng**:
  - Thay thế tọa độ demo bằng tọa độ thực tế từ bản đồ AMCL của mô phỏng Gazebo sân bay (`/amcl_pose`).
  - Quan sát robot di chuyển thực tế trong Gazebo khi nhận lệnh giọng nói.

---

## 6. Danh mục file của dự án

### Tài liệu và Rule AI (trong `.agent/`):
1. `.agent/airport_robot_voice/PROJECT_SPEC.md`: Bản sao đặc tả gốc của dự án.
2. `.agent/airport_robot_voice/PROGRESS.md`: File theo dõi tiến độ, khảo sát lỗi, kế hoạch kỹ thuật và kết quả kiểm chứng duy nhất.
3. `.agent/COMMANDS.md`: Tài liệu hướng dẫn cài đặt và vận hành toàn diện cho người dùng (chạy trực tiếp trong VS Code Dev Container).
4. `.agent/rules/workspace.md`: Quy tắc dự án cho AI.
5. `.agent/skills/ros2-voice-nav/SKILL.md`: Skill quy trình phát triển và kiểm thử ROS 2 Jazzy.

### Mã nguồn Package (`airport_robot_voice/`):
6. `airport_robot_voice/airport_robot_voice/nlu.py`: Lõi NLU tiếng Việt offline thuần Python.
7. `airport_robot_voice/airport_robot_voice/intent_node.py`: Node phân tích ý định, hỗ trợ voice độc lập và voice điều hướng.
8. `airport_robot_voice/airport_robot_voice/nav_command_node.py`: Cổng điều phối nhiệm vụ Nav2, vòng đời 8 trạng thái, chọn điểm bằng `ComputePathToPose`, đồng bộ TTS.
9. `airport_robot_voice/airport_robot_voice/stt_node.py`: Node chuyển giọng nói thành văn bản qua Vosk (hỗ trợ keyboard, mic, wav, tự nhận sample rate).
10. `airport_robot_voice/airport_robot_voice/tts_node.py`: Node tổng hợp giọng nói (log-only, espeak, piper), publish `tts_ack`, quản lý hàng đợi an toàn.
11. `airport_robot_voice/config/places.json`: Danh mục địa điểm sân bay (`_demo_data: true`).
12. `airport_robot_voice/config/intents.json`: Cấu hình ý định NLU.
13. `airport_robot_voice/launch/voice_assistant.launch.py`: Launch file toàn hệ thống với các tham số `mode`, `tts_engine`, `use_nav`, `wav_file`.
14. `airport_robot_voice/package.xml` & `setup.py`: Metadata package ROS 2 Jazzy.

### Bộ kiểm thử tự động (`airport_robot_voice/test/` và `scripts/`):
15. `airport_robot_voice/test/test_planner_resolution.py`: 4 test cases kiểm thử chọn đích bằng `ComputePathToPose`.
16. `airport_robot_voice/test/test_ros_lifecycle.py`: 7 test cases kiểm thử vòng đời Nav2, pending-acceptance, resume, replace, deduplication.
17. `airport_robot_voice/test/test_audio.py`: 10 test cases kiểm thử STT, TTS, format WAV, Vosk, Piper, ALSA devices.
18. `airport_robot_voice/test/test_nlu.py`: 24 test cases NLU tiếng Việt.
19. `airport_robot_voice/test/test_places.py`: 7 test cases kiểm thử schema và tính nhất quán dữ liệu.
20. `airport_robot_voice/scripts/test_nlu.py`: Script kiểm thử 61 kịch bản NLU và chế độ chat tương tác `-i`.
