# TIẾN ĐỘ VÀ KẾ HOẠCH TRIỂN KHAI AIRPORT_ROBOT_VOICE

## 1. Thông tin môi trường và khảo sát hệ thống (Giai đoạn 0)

- **Host OS**: Ubuntu 22.04 LTS (x86_64), Linux Kernel 6.8.0, ThinkPad P15 Gen 2.
- **Host ROS**: ROS 2 Humble (`/opt/ros/humble`).
- **Container ROS**: Docker Dev Container `thirsty_ishizaka` (Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12.3).
- **Workspace mapping**: Host `/home/thuytran/ws` <=> Container `/home/ws`.
- **Thư mục source**: `/home/ws/src` (chứa `airport_robot_voice`, `linorobot2`, etc.).
- **Build tool**: `colcon build --packages-select airport_robot_voice --symlink-install` trong container.
- **Quyền hạn**: Container user `thuytran` có quyền passwordless sudo (`sudo -n true`).
- **Mạng container**: Kết nối Internet hoạt động bình thường.
- **Thiết bị âm thanh trong container**:
  - `/dev/snd` được mount đầy đủ từ host vào container.
  - Đã cài đặt binary tiện ích: `alsa-utils`, `espeak-ng`.
  - Đã cài đặt thư viện Python: `vosk`, `sounddevice`, `pyaudio`.
  - Đã nạp Model Vosk tiếng Việt: `/opt/vosk/vosk-model-small-vn-0.4`.
  - Đã cài đặt TTS Piper: binary `/usr/local/bin/piper`, model `/opt/piper/vi_VN-vais1000-medium.onnx` và config `.json`.
  - `sounddevice.query_devices()` nhận diện 11 thiết bị âm thanh ALSA (Device 4: `sof-hda-dsp (hw:1,0)`, Device 8: `DMIC (hw:1,6)`).

---

## 2. Kết quả xử lý 13 vấn đề mã nguồn đã ghi nhận

| # | Vấn đề đã ghi nhận | Giải pháp và File đã xử lý | Trạng thái |
|---|--------------------|----------------------------|------------|
| 1 | Hỏi vị trí như "cổng A3 ở đâu" có `action=null` chưa dẫn đường | Cập nhật `intents.json` và `nlu.py`: các câu hỏi vị trí ("ở đâu", "chỗ nào", "tìm cổng") của địa điểm hợp lệ chuyển sang `action: navigate`. | ĐÃ GIẢI QUYẾT |
| 2 | Danh mục thiếu A3, A4 | Bổ sung đầy đủ `gate_a1`, `gate_a2`, `gate_a3`, `gate_a4`, `gate_b1`, `gate_b2`, `gate_15` vào `config/places.json`. | ĐÃ GIẢI QUYẾT |
| 3 | NLU tạo target mã cổng chưa tồn tại | Trong `nlu.py` (`extract_all_targets`, `process`), kiểm tra `place_key in self.places`. Nếu mã không tồn tại (ví dụ A99), chuyển sang intent `place_not_found`, báo lỗi cụ thể, không phát lệnh điều hướng. | ĐÃ GIẢI QUYẾT |
| 4 | Thiếu đích thì hệ thống hỏi lại (pending slot) | Loại bỏ hoàn toàn cơ chế pending slot trong `nlu.py`. Khi thiếu đích ("Dẫn tôi đến..."), thông báo ngắn *"Tôi chưa xác định được điểm đến"*, `action: None`, kết thúc lượt ngay. | ĐÃ GIẢI QUYẾT |
| 5 | Phủ định vẫn sinh lệnh navigate | Bổ sung hàm `check_negation` trong `nlu.py`: phát hiện "đừng dẫn...", "không dẫn...", "không đi..." -> chuyển thành `negated_nav`, không phát lệnh điều hướng. | ĐÃ GIẢI QUYẾT |
| 6 | Regex khớp substring, A123 bị cắt thành A12, alias A1 khớp A10 | Dùng ranh giới từ `\b` chặt chẽ trong regex cổng và kiểm tra span overlap khi dò alias trong `nlu.py`. A123 được nhận diện là mã không hợp lệ, alias `a1` không khớp `a10`. | ĐÃ GIẢI QUYẾT |
| 7 | Danh mục chỉ có một toilet, chưa chọn theo planner | Cấu hình category `toilet` với 2 điểm `toilet_1`, `toilet_2` trong `places.json`. `nav_command_node.py` tích hợp `ComputePathToPose` tính chiều dài đường đi để chọn điểm ngắn nhất. | ĐÃ GIẢI QUYẾT |
| 8 | Callback navigation dùng chung target name & goal handle | Quản lý vòng đời theo `mission_id` và `request_id`. Mọi callback (`on_goal_response`, `on_goal_result`) đều kiểm tra `mission_id == self._current_mission_id`. Callback cũ về muộn bị bỏ qua. | ĐÃ GIẢI QUYẾT |
| 9 | Cancel chưa theo dõi đầy đủ khi goal đang chờ chấp nhận | Hỗ trợ cancel ngay cả khi goal đang ở trạng thái pending/starting (`cancel_goal_async` được gửi khi handle sẵn sàng, hoặc reset state về IDLE). | ĐÃ GIẢI QUYẾT |
| 10 | Đường dẫn config dùng `__file__` sai khi cài đặt | Dùng `ament_index_python.packages.get_package_share_directory('airport_robot_voice')` với fallback trong cả `intent_node.py` và `nav_command_node.py`. | ĐÃ GIẢI QUYẾT |
| 11 | STT giữ wake state mãi, hàng đợi audio chưa kiểm soát | Thêm `wake_timeout_sec` (mặc định 15s) tự động trở về sleep trong `stt_node.py`. Hàng đợi audio đặt `maxsize=50` và bỏ qua chunk cũ khi đầy (overflow protection). | ĐÃ GIẢI QUYẾT |
| 12 | Launch keyboard không nhận được stdin | Hỗ trợ chạy từng node trong terminal riêng bằng `ros2 run` (chuẩn tương tác stdin), đồng thời cấu hình `emulate_tty=True` trong `voice_assistant.launch.py`. | ĐÃ GIẢI QUYẾT |
| 13 | Bộ test 57/57 cũ chỉ kiểm tra intent | Xây dựng bộ test mới 40 test cases pytest (NLU, places schema, ROS lifecycle, audio) và 61 kịch bản `test_nlu.py` bao phủ toàn diện cả `action`, `target`, `category`, `negation`, `ambiguity`, `offline info`. | ĐÃ GIẢI QUYẾT |

---

## 3. Kiến trúc hệ thống và Thiết kế giao tiếp (Schema & Topics)

### 3.1. Các topic và QoS

1. `/voice/text` (`std_msgs/String`, Volatile, Depth 10): Nhận transcript từ bàn phím, micro hoặc WAV.
2. `/voice/intent` (`std_msgs/String`, Volatile, Depth 10): JSON payload Schema v1.0 từ `intent_node`.
3. `/voice/say` (`std_msgs/String`, Volatile, Depth 10): Văn bản phát âm qua `tts_node`.
4. `/voice/speaking` (`std_msgs/Bool`, Volatile, Depth 10): Trạng thái đang phát âm của robot.
5. `/voice/tts_ack` (`std_msgs/String`, Volatile, Depth 10): JSON xác nhận câu nói đã hoàn thành.
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

## 4. Trạng thái thực hiện 6 giai đoạn

- [x] **Giai đoạn 0: Khảo sát và ghi quy tắc** (Hoàn thành)
  - Khảo sát hệ thống: ROS 2 Jazzy trong Docker `thirsty_ishizaka`, Python 3.12, workspace `/home/ws`.
  - Đặc tả gốc lưu tại `.agent/airport_robot_voice/PROJECT_SPEC.md`.
  - Rule dự án thiết lập tại `.agent/rules/workspace.md`.
  - Skill dự án thiết lập tại `.agent/skills/ros2-voice-nav/SKILL.md`.
  - Tiến độ theo dõi tại `.agent/airport_robot_voice/PROGRESS.md`.

- [x] **Giai đoạn 1: Package và chế độ voice độc lập** (Hoàn thành)
  - Hoàn thiện `package.xml`, `setup.py`, đường dẫn `get_package_share_directory`.
  - Thiết lập `config/places.json` demo: A1..A4, B1, B2, 15, checkin, 2 toilets (`toilet_1`, `toilet_2`), Wi-Fi, trạm sạc.
  - Hoàn thiện chế độ Voice độc lập: `intent_node` nhận diện đầy đủ khi `nav_command_node` không chạy.
  - Build thành công qua `colcon build --packages-select airport_robot_voice --symlink-install`.

- [x] **Giai đoạn 2: NLU và Dữ liệu** (Hoàn thành)
  - Hoàn thiện `airport_robot_voice/nlu.py`: xử lý dấu tiếng Việt, Unicode tổ hợp NFC/NFD, đọc số/mã ("a ba", "bê hai", "mười lăm", "a tư").
  - Xử lý ranh giới từ `\b` và span overlap: chống A123 bị cắt thành A12, alias A1 không khớp A10.
  - Kiểm tra tồn tại địa điểm trước khi phát lệnh: mã không có trong catalog chuyển sang `place_not_found`.
  - Nhận diện phủ định: "đừng dẫn..." -> `negated_nav`, "không dừng lại" -> `negated_pause`, "không cần nữa" -> `cancel_robot`.
  - Nhận diện đa đích: "A1 hay A2" -> `ambiguous_target`.
  - Nhận diện thiếu đích: "Dẫn tôi đến..." -> `missing_destination`, không dùng pending slot.
  - Chuyển câu hỏi vị trí ("ở đâu/chỗ nào") của địa điểm hợp lệ sang `navigate`.
  - Đạt 61/61 kịch bản kiểm thử tự động trong `scripts/test_nlu.py`.

- [x] **Giai đoạn 3: Navigation và Quản lý nhiệm vụ (NavCommandNode)** (Hoàn thành)
  - Xây dựng máy trạng thái đầy đủ: `IDLE`, `RESOLVING`, `ANNOUNCING`, `STARTING`, `NAVIGATING`, `PAUSING`, `PAUSED`, `CANCELING`.
  - Hỗ trợ đầy đủ vòng đời: PAUSE, RESUME, CANCEL, REPLACE (đổi đích), STATUS.
  - Tích hợp `ComputePathToPose` chọn điểm ngắn nhất cho category (`toilet`).
  - Quản lý `mission_id` và `request_id` loại trừ race condition khi callback về muộn.
  - Công bố trạng thái qua `/voice/nav_state` (QoS Transient Local).

- [x] **Giai đoạn 4: STT, TTS và Âm thanh offline trong Docker** (Hoàn thành)
  - `stt_node.py`: hỗ trợ mode `keyboard`, `mic`, `wav`. Đặt giới hạn hàng đợi audio (`audio_queue_size=50`), timeout wake word.
  - `tts_node.py`: hỗ trợ engine `log` (log-only), `espeak`, `piper`. Chuẩn hóa cách đọc mã cổng ("A3" -> "cổng a ba").
  - Cài đặt `alsa-utils`, `espeak-ng`, `vosk`, `sounddevice` trong container.
  - Tải model Vosk tiếng Việt vào `/opt/vosk/vosk-model-small-vn-0.4`.

- [x] **Giai đoạn 5: Tích hợp, Kiểm thử tổng thể và Bàn giao** (Hoàn thành)
  - Cập nhật `launch/voice_assistant.launch.py` hỗ trợ đầy đủ các tham số `mode`, `tts_engine`, `use_nav`, `wav_file`.
  - Hoàn thiện `README.md` bằng tiếng Việt với hướng dẫn lệnh copy/paste theo từng terminal trong container.
  - Chạy toàn bộ 40 test pytest và 61 kịch bản NLU đạt 100%.

---

## 5. Kết quả kiểm chứng chi tiết theo từng môi trường

### 5.1. Nhóm Test Fixture & Cô lập (ĐÃ XÁC MINH - ĐẠT 100%)
- **Lệnh thực thi**:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && python3 -m pytest src/airport_robot_voice/test/ -v
  ```
- **Kết quả**: 40/40 tests PASSED trong 4.20 giây.
  - `test_nlu.py`: 24 tests (A3, B2, 15, A123 word boundary, A10 alias, toilet category, specific toilet, phủ định, hủy, không pause, đa đích, thiếu đích, Wi-Fi, sạc, giờ, chào, cảm ơn, Unicode NFD, không dấu).
  - `test_places.py`: 7 tests (demo flag, required gates, categories, no duplicate aliases, offline services, no flight query, required action intents).
  - `test_ros_lifecycle.py`: 3 tests (IntentNode standalone mode, TtsNode log mode, NavCommandNode state publication and controls).
  - `test_audio.py`: 6 tests (speech normalization, log synthesis, espeak synthesis, missing model graceful handling, WAV file processing, sounddevice query).

### 5.2. Nhóm Thiết bị âm thanh trong Container (ĐÃ XÁC MINH MỘT PHẦN)
- **Đã kiểm chứng**:
  - `sounddevice.query_devices()`: Nhận diện 11 thiết bị ALSA (hw:1,0 HDA Analog, hw:1,6 DMIC).
  - `espeak-ng -v vi`: Đã kiểm chứng tổng hợp âm thanh tiếng Việt thành file WAV (`/tmp/test_espeak.wav`) thành công.
  - Model Vosk tiếng Việt: Đã nạp thành công từ `/opt/vosk/vosk-model-small-vn-0.4` và nhận dạng âm thanh từ file WAV.
  - TTS chế độ `log`: Hoạt động 100% không phụ thuộc loa ngoài.
- **Phần còn cần xác minh với người dùng**:
  - Nhập giọng nói trực tiếp qua micro laptop ThinkPad P15 Gen 2 (cần người dùng nói trực tiếp vào micro vật lý).
  - Phát âm thanh trực tiếp ra loa máy tính (phụ thuộc routing âm thanh PulseAudio/ALSA giữa host và container Docker).

### 5.3. Nhóm Điều hướng Nav2 / Gazebo (ĐÃ XÁC MINH LOGIC, CHỜ MÔ PHỎNG THỰC TẾ)
- **Đã kiểm chứng**:
  - Action clients `NavigateToPose` và `ComputePathToPose` hoạt động đúng interface ROS 2 Jazzy.
  - Máy trạng thái xử lý chuẩn xác các luồng ACCEPTED, REJECTED, ABORTED, CANCELED, PAUSE, RESUME, REPLACE, STATUS.
  - Chế độ Voice độc lập hoạt động độc lập khi Nav2 chưa khởi động.
- **Phần còn cần xác minh với người dùng**:
  - Robot di chuyển thực tế trên bản đồ Gazebo khi khởi động stack `linorobot2_gazebo` / `linorobot2_navigation`.

---

## 6. Danh sách các file thay đổi và tạo mới

### Tài liệu và Rule AI (tập trung trong `.agent/`):
1. `.agent/airport_robot_voice/PROJECT_SPEC.md`: Bản sao lưu đặc tả yêu cầu dự án.
2. `.agent/airport_robot_voice/PROGRESS.md`: File theo dõi tiến độ, khảo sát 13 lỗi, kế hoạch và kết quả kiểm thử duy nhất.
3. `.agent/rules/workspace.md`: Rule dùng chung cho dự án airport_robot_voice.
4. `.agent/skills/ros2-voice-nav/SKILL.md`: Skill quy trình phát triển và kiểm thử ROS 2 Jazzy voice/nav2 trong Docker.

### Source code và cấu hình Package (`airport_robot_voice/`):
5. `airport_robot_voice/package.xml`: Bổ sung `action_msgs`, `ament_index_python`, `pytest`.
6. `airport_robot_voice/setup.py`: Bổ sung `tests_require=['pytest']`.
7. `airport_robot_voice/config/places.json`: Cấu hình catalog demo đầy đủ (A1..A4, B1, B2, 15, checkin, 2 toilets, dịch vụ offline).
8. `airport_robot_voice/config/intents.json`: Loại bỏ query flight, bổ sung pause, resume, cancel, status, charging, hours.
9. `airport_robot_voice/launch/voice_assistant.launch.py`: Hỗ trợ đầy đủ tham số `mode`, `wav_file`, `tts_engine`, `use_nav`.
10. `airport_robot_voice/airport_robot_voice/nlu.py`: Lõi NLU tiếng Việt thuần Python hoàn thiện toàn bộ đặc tả.
11. `airport_robot_voice/airport_robot_voice/intent_node.py`: Hỗ trợ 2 mode Voice độc lập và Voice kết hợp Nav2, chống trùng lặp lời nói.
12. `airport_robot_voice/airport_robot_voice/nav_command_node.py`: Cổng điều phối nhiệm vụ Nav2 với máy trạng thái 8 bước, chống race condition, chọn điểm theo `ComputePathToPose`.
13. `airport_robot_voice/airport_robot_voice/stt_node.py`: Hỗ trợ 3 mode (keyboard, wav, mic), chống tràn buffer audio, timeout wake word.
14. `airport_robot_voice/airport_robot_voice/tts_node.py`: Hỗ trợ engine log-only, espeak, piper, chuẩn hóa cách đọc mã cổng, publish `tts_ack`.
15. `airport_robot_voice/scripts/test_nlu.py`: Script kiểm thử NLU 61 kịch bản và chat tương tác `-i`.
16. `airport_robot_voice/test/test_nlu.py`: Pytest suite cho NLU (24 tests).
17. `airport_robot_voice/test/test_places.py`: Pytest suite cho schema dữ liệu (7 tests).
18. `airport_robot_voice/test/test_ros_lifecycle.py`: Pytest suite cho ROS 2 node lifecycle và communication (3 tests).
19. `airport_robot_voice/test/test_audio.py`: Pytest suite cho âm thanh, STT, TTS và devices (6 tests).
20. `airport_robot_voice/README.md`: Hướng dẫn tổng quan và liên kết tới tài liệu vận hành chi tiết.
21. `.agent/COMMANDS.md`: Hướng dẫn cài đặt và vận hành toàn diện từ A-Z dành cho người mới (môi trường Docker, cài dependency, tải model offline Vosk & Piper, kết nối micro/loa thật, build và kịch bản chạy từng terminal).
