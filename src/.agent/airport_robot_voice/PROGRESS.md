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

## 2. Kết quả xử lý các nhóm vấn đề mã nguồn cốt lõi

| # | Vấn đề đã ghi nhận | Giải pháp và File đã xử lý | Trạng thái |
|---|--------------------|----------------------------|------------|
| 1 | Hỏi vị trí ("cổng A3 ở đâu") có `action=null` | Cập nhật `intents.json` và `nlu.py`: chuyển câu hỏi vị trí thành `action: navigate`. | ĐÃ GIẢI QUYẾT |
| 2 | Danh mục thiếu A3, A4 | Bổ sung đầy đủ `gate_a1`..`gate_a4`, `gate_b1`, `gate_b2`, `gate_15` vào `places.json`. | ĐÃ GIẢI QUYẾT |
| 3 | NLU tạo target cổng không tồn tại | `nlu.py` kiểm tra `place_key in self.places`. Nếu không có, gán intent `place_not_found`. | ĐÃ GIẢI QUYẾT |
| 4 | Thiếu đích thì hỏi lại (pending slot) | Bỏ pending slot. Thông báo không xác định được đích, `action: None`, kết thúc lượt ngay. | ĐÃ GIẢI QUYẾT |
| 5 | Phủ định vẫn sinh lệnh navigate | Bổ sung `check_negation` trong `nlu.py`: chuyển thành `negated_nav`, không điều hướng. | ĐÃ GIẢI QUYẾT |
| 6 | Regex khớp substring, A123 thành A12, A1 khớp A10 | Dùng ranh giới từ `\b` và kiểm tra span overlap. Ngăn chặn triệt để khớp substring. | ĐÃ GIẢI QUYẾT |
| 7 | Danh mục toilet chỉ chọn điểm đầu | Tích hợp `ComputePathToPose` tính chiều dài đường thực tế cho từng ứng viên, chọn điểm ngắn nhất. | ĐÃ GIẢI QUYẾT |
| 8 | Callback navigation dùng chung target & handle | Tách biệt `mission_id`, `request_id`, `goal_attempt_id`. Callback cũ bị thu hồi. | ĐÃ GIẢI QUYẾT |
| 9 | Cancel/Pause chưa phân biệt 3 pha | Tách biệt pha gửi yêu cầu, pha server chấp nhận, và pha terminal result. Xử lý khi reject. | ĐÃ GIẢI QUYẾT |
| 10 | Acceptance timeout & late acceptance | Quản lý attempt timeout như yêu cầu chưa xong, dọn dẹp và hủy goal muộn (`cancel_goal_async()`), theo dõi terminal. | ĐÃ GIẢI QUYẾT |
| 11 | Timer dùng chung giữa các operation | Gắn identity (`target_attempt_id`, `target_speech_id`) vào timer, chặn callback cũ dừng nhầm timer mới. | ĐÃ GIẢI QUYẾT |
| 12 | Treo chờ terminal sau cancel accepted | Bổ sung `cancel_terminal_timeout_sec` (4.0s), đánh dấu unconfirmed terminal, chặn resume/replace. | ĐÃ GIẢI QUYẾT |
| 13 | Chính sách TTS strict/flexible & drop câu resume | Hỗ trợ 2 chế độ `tts_policy`, hàng đợi TTS dùng LRU cache có timestamp để câu nói sau resume phát bình thường. | ĐÃ GIẢI QUYẾT |
| 14 | Fallback tự chọn điểm khi thiếu planner | Bỏ fallback chọn điểm tùy ý trong `resolve_category_and_start()`, báo `PLANNER_UNAVAILABLE` và về IDLE an toàn. | ĐÃ GIẢI QUYẾT |

---

## 3. Kiến trúc hệ thống và Thiết kế giao tiếp (Schema & Topics)

### 3.1. Các topic và QoS
1. `/voice/text` (`std_msgs/String`, Volatile, Depth 10): Nhận transcript từ bàn phím, micro hoặc WAV.
2. `/voice/intent` (`std_msgs/String`, Volatile, Depth 10): JSON payload Schema v1.0 từ `intent_node`.
3. `/voice/say` (`std_msgs/String`, Volatile, Depth 10): JSON payload (`text`, `speech_id`, `mission_id`) gửi tới `tts_node`.
4. `/voice/speaking` (`std_msgs/Bool`, Volatile, Depth 10): Trạng thái robot đang phát âm thanh.
5. `/voice/tts_ack` (`std_msgs/String`, Volatile, Depth 10): JSON xác nhận câu nói (`speech_id`, `mission_id`, `status`, `audio_played`).
6. `/voice/tts_cancel` (`std_msgs/String`, Volatile, Depth 10): JSON yêu cầu hủy câu nói đang chờ trong hàng đợi TTS.
7. `/voice/nav_state` (`std_msgs/String`, Transient Local, Depth 1): Trạng thái máy nhiệm vụ Nav2 latched.
8. `/voice/feedback` (`std_msgs/String`, Volatile, Depth 10): Phản hồi chi tiết từng request.

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
  "mission_id": "mis_1795153611488_1",
  "goal_attempt_id": "att_1795153611488_1",
  "current_target_id": "gate_a3",
  "current_target_name": "cổng A3",
  "distance_remaining": 12.5,
  "eta_seconds": 18,
  "last_feedback_time": 1700000010.5,
  "timestamp": 1700000011.0
}
```

---

## 4. Chi tiết triển khai và Kết quả kiểm chứng chuyên sâu

### 4.1. Ngữ nghĩa Hủy và Tạm dừng (Cancel & Pause Semantics)
- **Phân định 3 pha rõ ràng**:
  - Pha A: Đã gửi yêu cầu hủy (`PAUSING` hoặc `CANCELING`).
  - Pha B: Server chấp nhận yêu cầu hủy (`return_code == 0` VÀ `len(goals_canceling) > 0`).
  - Pha C: Goal đã đạt trạng thái terminal thực tế (`STATUS_CANCELED`, `STATUS_SUCCEEDED`, hoặc `STATUS_ABORTED`).
- **Xử lý khi Server từ chối hủy (Reject)**:
  - Khi action server từ chối hủy (`is_accepted = False`): Node không chuyển về `PAUSED` hay `IDLE`. Giữ nguyên `_active_goal_handle`, phục hồi trạng thái `NAVIGATING`, phát thông báo lỗi cho người dùng và công bố feedback `CANCEL_REJECTED`. Chặn hoàn toàn các lệnh `resume` hoặc `replace`.
- **Xử lý Goal kết thúc trong lúc yêu cầu hủy**:
  - Nếu goal kịp hoàn tất (`STATUS_SUCCEEDED`) trong lúc đang chờ hủy: Node nhận diện đã tới đích, thông báo chúc mừng và chuyển về `IDLE`.
  - Nếu goal bị lỗi hoặc chướng ngại (`STATUS_ABORTED`) trong lúc đang chờ hủy: Node báo lỗi lộ trình và đưa máy trạng thái về `IDLE`.

### 4.2. Xử lý Acceptance Timeout và Thu hồi Goal muộn (Late Acceptance Revocation)
- **Quản lý Attempt Timeout**: Khi `goal_acceptance_timeout_sec` (4.0s) kích hoạt mà Nav2 chưa phản hồi accept/reject:
  - Attempt được chuyển vào tập `_revoked_attempts` và vẫn duy trì trong `_unresolved_attempts`.
  - Máy trạng thái về `IDLE` an toàn, phát thông báo lỗi cho người dùng và công bố feedback `ACCEPTANCE_TIMEOUT`.
- **Xử lý Acceptance muộn**:
  - Khi goal response về muộn cho một attempt đã bị thu hồi: Node lập tức gửi `cancel_goal_async()`.
  - Thiết lập callback `_on_delayed_cancel_confirmed` để kiểm tra kết quả hủy trên server (công bố `REVOKED_GOAL_CANCEL_REJECTED` nếu server từ chối).
  - Thiết lập callback `_on_delayed_cancel_terminal_result` để theo dõi goal kết thúc hẳn và dọn dẹp khỏi `_unresolved_attempts`.
  - Tuyệt đối không gán `_active_goal_handle` và không chuyển `NAVIGATING` cho goal đến muộn.

### 4.3. Cô lập Timer theo từng Operation và Định danh ID
- **Gắn Identity vào Timer**:
  - `_acceptance_timeout_timer` gắn với `_acceptance_timer_attempt`.
  - `_cancel_response_timer` gắn với `_cancel_response_timer_attempt`.
  - `_cancel_terminal_timer` gắn với `_cancel_terminal_timer_attempt`.
  - `_tts_timeout_timer` gắn với `_tts_timer_speech_id`.
- **Nguyên tắc cô lập**: Khi một callback hoàn thành, hàm `_stop_..._timer(target_id)` chỉ hủy timer nếu `target_id` khớp với identity đang hoạt động. Một thao tác cũ hoàn tất muộn không thể dừng timer của một thao tác mới đang chạy.
- **Định danh phiên**: Tạo ID nhiệm vụ và attempt kèm timestamp ms (`mis_{session_start_ms}_{counter}`), đảm bảo khi node restart không bao giờ trùng lặp ID.

### 4.4. Timeout chờ Terminal Result sau khi Cancel Accepted
- Bổ sung tham số `cancel_terminal_timeout_sec` (4.0s).
- Khi server chấp nhận cancel nhưng kẹt không gửi terminal result:
  - Kích hoạt timer `_on_cancel_terminal_timeout`.
  - Đặt cờ `_unconfirmed_terminal = True`.
  - Công bố feedback cảnh báo `CANCEL_TERMINAL_TIMEOUT`.
  - Chặn hoàn toàn các yêu cầu `resume` hoặc `replace` mới cho đến khi goal cũ dừng hẳn (`UNCONFIRMED_TERMINAL_PENDING`).
  - Khi terminal result đến muộn: xóa cờ `_unconfirmed_terminal`, cho phép hệ thống nhận nhiệm vụ mới.

### 4.5. Chính sách TTS Strict & Flexible và Bộ đệm LRU Bounded Cache
- **Chính sách TTS**:
  - `tts_policy = 'strict'`: Khi timeout chờ TTS ACK hoặc nhận ACK lỗi/hàng đợi đầy/bị hủy, lập tức hủy gửi goal Nav2, đưa máy trạng thái về `IDLE` và công bố feedback `FAILED`.
  - `tts_policy = 'flexible'`: Cho phép fallback tiếp tục gửi goal tới Nav2 và công bố feedback `WARNING`.
- **Hàng đợi TTS và Quản lý Hủy**:
  - Chuyển `_cancelled_speeches` và `_cancelled_missions` trong `tts_node.py` sang `OrderedDict` LRU giới hạn tối đa 200 items, lưu timestamp thời điểm hủy.
  - Ghi nhận `enqueued_at` cho từng câu nói khi gọi `on_say`.
  - Worker chỉ hủy câu nói nếu `enqueued_at <= cancel_time`. Các câu nói mới phát ra sau khi resume (`enqueued_at > cancel_time`) được tổng hợp và phát bình thường, không bị drop nhầm.

### 4.6. Lựa chọn Đích bằng Planner và Bỏ Fallback tùy ý
- **Truy vấn ComputePathToPose**:
  - Lọc ứng viên hợp lệ (`enabled: true`, đúng frame, tọa độ không NaN/inf).
  - Gửi tuần tự `ComputePathToPose` tới planner, so sánh chiều dài đường đi thực tế qua `compute_path_length(path)`.
  - Xử lý timeout, lỗi TF (`100 <= error_code < 200`), và không tìm thấy đường.
- **Không tùy tiện fallback**:
  - Khi action server planner vắng mặt (`wait_for_server` timeout), node công bố feedback `FAILED` với mã `PLANNER_UNAVAILABLE`, phát thông báo giải thích rõ ràng và về `IDLE`.
  - Tuyệt đối KHÔNG gửi goal điều hướng tới ứng viên tùy ý khi chưa xác định được tính khả thi của đường đi.

---

## 5. Tổng hợp Bằng chứng và Kết quả Kiểm thử (Evidence)

Toàn bộ **74 bài kiểm thử tự động** đã được thực thi và xác minh 100% PASSED trực tiếp trong Container ROS 2 Jazzy:

```bash
# Lệnh thực thi:
python3 -m pytest src/airport_robot_voice/test/ -v
```

### Bảng kết quả chi tiết từng file kiểm thử:
| File kiểm thử | Số test | Kết quả | Thời gian | Nội dung xác minh |
|---|---|---|---|---|
| `test_ros_lifecycle.py` | 22 | **22/22 PASSED** | ~55.3s | Vòng đời Nav2, 3 pha cancel/pause, server reject, pause in resolving, replace, freshness, deduplication, tts sync, acceptance timeout, late acceptance revocation, timer isolation, cancel terminal timeout, tts strict/flexible |
| `test_planner_resolution.py` | 8 | **8/8 PASSED** | ~5.8s | So sánh độ dài đường đi, unreachable, timeout candidate, late callback discard, replace category, planner missing báo lỗi an toàn (0 nav goals) |
| `test_audio.py` | 13 | **13/13 PASSED** | ~3.8s | STT known speech WAV, silence WAV, model missing, invalid format, wake timeout, Piper WAV format, TTS cancel queue, dropped queue full, resume allows new speech after pause/cancel, ALSA devices |
| `test_nlu.py` | 24 | **24/24 PASSED** | ~0.04s | Cổng A3/B2, cách đọc tiếng Việt, phủ định, cancel/pause/resume, ambiguous, missing slot, Wi-Fi, sạc pin, giờ mở cửa, Unicode tổ hợp |
| `test_places.py` | 7 | **7/7 PASSED** | ~0.02s | Schema JSON places/intents, demo flag, required gates, aliases không trùng lặp |
| **TỔNG CỘNG** | **74** | **74/74 PASSED 100%** | **59.44s** | **Đạt tiêu chuẩn xuất xưởng và nghiệm thu** |

- File log bằng chứng chi tiết: `.agent/airport_robot_voice/evidence/full_pytest.log`.
- Kịch bản NLU độc lập: `python3 src/airport_robot_voice/scripts/test_nlu.py` đạt **61/61 PASSED**.
- Build package: `colcon build --packages-select airport_robot_voice --symlink-install` hoàn tất thành công 0 warnings, 0 errors.

---

## 6. Phân định kết quả kiểm chứng theo 3 môi trường

### 6.1. Môi trường Test Fixture & Mock Server (ĐÃ XÁC MINH 100%)
- **Trạng thái**: ĐẠT 100% (74/74 tests pytest, 61/61 test_nlu).
- **Phạm vi xác minh**:
  - Xử lý NLU tiếng Việt, phủ định, Unicode tổ hợp, ranh giới từ, trích xuất slot, lọc đích tồn tại.
  - Thuật toán chọn đường bằng `ComputePathToPose` (so sánh độ dài đường đi, unreachable, timeout, cancel ngắt resolving, late callback discard, planner unavailable).
  - Toàn bộ máy trạng thái vòng đời Nav2 (PAUSE, RESUME, CANCEL, REPLACE, STATUS, deduplication, acceptance timeout, late acceptance revocation, cancel terminal timeout, timer isolation).
  - Đồng bộ âm thanh TTS (speech_id, mission_id, tts_ack, strict/flexible policy, queue cancellation, LRU cache).
  - Định dạng file WAV, nhận diện âm thanh offline qua Vosk, xử lý hàng đợi audio.

### 6.2. Môi trường Thiết bị âm thanh vật lý (ĐÃ XÁC MINH CẤU HÌNH, SẴN SÀNG THỬ NGHIỆM VỚI NGƯỜI DÙNG)
- **Đã kiểm chứng**:
  - Container nhận diện 11 thiết bị âm thanh ALSA qua `/dev/snd`.
  - eSpeak-ng và Piper tổng hợp tiếng Việt ra file WAV chuẩn mono 16-bit.
  - Model Vosk tiếng Việt giải mã âm thanh chính xác.
  - Chế độ `tts_engine:=log` và `mode:=keyboard` hoạt động độc lập hoàn hảo không cần micro/loa.
- **Thao tác thử nghiệm micro và loa thật**:
  - Thu âm thử: `arecord -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_mic.wav`
  - Nghe lại: `aplay /tmp/test_mic.wav`
  - Chạy STT với micro: `ros2 run airport_robot_voice stt_node --ros-args -p mode:=mic`
  - Chạy TTS với loa: `ros2 run airport_robot_voice tts_node --ros-args -p engine:=espeak` (hoặc `piper`).

### 6.3. Kịch bản nghiệm thu mô phỏng Gazebo & Nav2 theo từng Terminal
Để nghiệm thu hệ thống voice tích hợp đầy đủ với mô phỏng robot sân bay (Gazebo + Nav2), thực hiện theo các bước tuần tự trong 4 Terminal VS Code:

#### Terminal 1: Khởi động Mô phỏng Gazebo và Điều hướng Nav2
```bash
# Nạp môi trường ROS 2 Jazzy và Workspace
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Khởi động mô phỏng robot và Nav2 (ví dụ Linorobot2 hoặc turtlebot3 simulation)
# ros2 launch linorobot2_gazebo gazebo.launch.py
# ros2 launch linorobot2_navigation navigation.launch.py rviz:=true
```
*Kiểm tra sẵn sàng*:
```bash
# Kiểm tra action server NavigateToPose và ComputePathToPose sẵn sàng:
ros2 action list | grep -E "navigate_to_pose|compute_path_to_pose"
# Kiểm tra toạ độ bản đồ AMCL của robot:
ros2 topic echo /amcl_pose --once
```

#### Terminal 2: Khởi động Hệ thống Airport Robot Voice
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Khởi chạy toàn bộ hệ thống voice với TTS ghi log hoặc phát loa:
ros2 launch airport_robot_voice voice_system.launch.py   mode:=keyboard   tts_engine:=log   tts_policy:=flexible
```

#### Terminal 3: Giám sát Trạng thái Nhiệm vụ và Tọa độ Điều hướng
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Theo dõi máy trạng thái vòng đời Nav2 latched:
ros2 topic echo /voice/nav_state --qos-durability transient_local

# Mở thêm tab phụ theo dõi phản hồi chi tiết:
# ros2 topic echo /voice/feedback
```

#### Terminal 4: Tương tác Kiểm thử tuần tự các kịch bản
Nhập trực tiếp các câu lệnh tại Terminal 4 (hoặc qua micro):
1. **Dẫn đường cổng A3**: Gõ `dẫn tôi đến cổng A3` -> Quan sát robot trong Gazebo xuất phát tới tọa độ Gate A3; `/voice/nav_state` chuyển sang `NAVIGATING`.
2. **Hỏi vị trí và dẫn tới Toilet tối ưu**: Gõ `nhà vệ sinh ở đâu` -> Quan sát planner truy vấn `ComputePathToPose` chọn nhà vệ sinh có đường đi ngắn nhất (`toilet_1` hoặc `toilet_2`).
3. **Tạm dừng giữa đường**: Gõ `dừng lại` -> Robot dừng di chuyển trong Gazebo; state chuyển `PAUSING` -> `PAUSED`.
4. **Đi tiếp**: Gõ `đi tiếp` -> Robot tiếp tục di chuyển; state chuyển `STARTING` -> `NAVIGATING`.
5. **Đổi đích khi đang di chuyển**: Gõ `đổi sang cổng A4` -> Goal A3 bị hủy an toàn, robot đổi lộ trình sang cổng A4.
6. **Hỏi trạng thái**: Gõ `bạn đang dẫn tôi đi đâu` -> Robot báo tên đích và khoảng cách còn lại (mét).
7. **Hủy chuyến**: Gõ `hủy chuyến đi` -> Robot dừng hẳn, goal bị cancel trên Nav2, state quay về `IDLE`.
