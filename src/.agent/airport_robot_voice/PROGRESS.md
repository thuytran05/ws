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
| 13 | Bộ test 57/57 cũ chỉ kiểm tra intent | Xây dựng bộ test toàn diện 65 tests pytest bao phủ logic, schema, lifecycle và audio. | ĐÃ GIẢI QUYẾT |

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
  "mission_id": "mission_1",
  "goal_attempt_id": "att_1_1700000000_1",
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
  - Khi action server từ chối hủy (`is_accepted = False`): Node không chuyển về `PAUSED` hay `IDLE`. Thay vào đó, node giữ nguyên `_active_goal_handle`, phục hồi trạng thái `NAVIGATING`, phát câu thông báo lỗi cho người dùng và công bố feedback `CANCEL_REJECTED`.
  - Robot tiếp tục theo dõi goal và chặn hoàn toàn các lệnh `resume` hoặc `replace` tạo chuyến mới khi goal cũ chưa giải quyết xong.
- **Xử lý Goal kết thúc trong lúc yêu cầu hủy**:
  - Nếu goal kịp hoàn tất (`STATUS_SUCCEEDED`) trong lúc đang chờ hủy: Node nhận diện đã tới đích, thông báo chúc mừng và chuyển về `IDLE`.
  - Nếu goal bị lỗi hoặc chướng ngại (`STATUS_ABORTED`) trong lúc đang chờ hủy: Node báo lỗi lộ trình và đưa máy trạng thái về `IDLE`.
- **Chuyển tiếp trạng thái an toàn**:
  - Cancel khi đang `PAUSING`: Nâng cấp trạng thái lên `CANCELING`.
  - Navigate / Replace khi đang `PAUSING` hoặc `CANCELING`: Lưu pending replace, đợi goal cũ đạt terminal result rồi mới khởi động đích mới.
  - Pending goal bị Reject từ chối acceptance: Báo lỗi `GOAL_REJECTED` và về `IDLE`.
  - Finite Timeout: Tham số hóa `cancel_timeout_sec` (3.0s) và `goal_acceptance_timeout_sec` (4.0s).

### 4.2. Hoàn thiện Phiên chọn đích bằng Planner (`ComputePathToPose`)
- **Phân lập phiên và định danh attempt**:
  - Mỗi phiên giải quyết category được cấp một `resolution_id` duy nhất (`res_{counter}_{timestamp}`).
  - Mỗi ứng viên trong phiên được gán `cand_attempt_id` (`cand_{res_id}_{idx}`).
  - Bộ điều phối kiểm tra nghiêm ngặt: Mọi callback response, result hay timer muộn của attempt cũ hoặc của phiên cũ đều bị loại bỏ ngay lập tức, không làm sai lệch chỉ số đánh giá hay ghi đè kết quả.
- **Lọc và kiểm tra ứng viên**:
  - Chỉ chấp nhận điểm có `enabled: true`, `frame_id == self.frame_id`, và tọa độ `x, y, yaw` là số thực hữu hạn (không NaN, không inf).
  - Tự động tìm kiếm members từ trường `category` trong `places.json` hoặc mảng `members` của nhóm.
- **Xử lý ngoại lệ Planner**:
  - Khi Planner action server vắng mặt: Phát thông báo giải thích rõ ràng `PLANNER_UNAVAILABLE_FALLBACK`, không tự tuyên bố là điểm gần nhất.
  - Phân biệt rõ ràng giữa lỗi TF (`100 <= error_code < 200`) và lỗi không tìm thấy đường đi (`NO_VALID_PATH`).
- **Đổi đích (Replace) sang Category**:
  - Khi nhận lệnh đổi đích sang nhóm điểm (`category: toilet`), tái sử dụng toàn bộ chu trình lọc và planner để tìm điểm tối ưu.
- **Tạm dừng và Tiếp tục khi đang Resolving**:
  - Lệnh `pause` khi đang `RESOLVING` sẽ hủy timer planner, lưu lại `_paused_resolution = {'category': ..., 'req_id': ...}` và chuyển sang `PAUSED`.
  - Khi nhận `resume`, hệ thống khôi phục tiến trình chọn đường cho category mà không bị nhầm lẫn toạ độ của chuyến đi trước.

### 4.3. Quản lý Hàng đợi TTS và Đồng bộ Hủy
- **Hủy câu nói chờ trong hàng đợi**:
  - Node TTS (`tts_node.py`) subscribe topic `/voice/tts_cancel`.
  - Khi nhận yêu cầu cancel hoặc replace, `nav_command_node` phát tín hiệu hủy các câu nói thuộc mission hiện tại.
  - Worker TTS kiểm tra trước khi tổng hợp và phát âm: Nếu câu nói đã bị hủy, bỏ qua việc phát âm thanh và phản hồi `/voice/tts_ack` với trạng thái `CANCELLED_BEFORE_PLAYBACK`.
- **Bảo vệ chống tràn hàng đợi**:
  - Giới hạn hàng đợi `queue_size=30`. Khi hàng đợi đầy, các câu nói mới bị từ chối và phản hồi `DROPPED_QUEUE_FULL`.

---

## 5. Tổng hợp Bằng chứng và Kết quả Kiểm thử (Evidence)

Toàn bộ 65 bài kiểm thử tự động đã được thực thi và xác minh trực tiếp trong Container ROS 2 Jazzy.

```bash
# Lệnh thực thi:
python3 -m pytest src/airport_robot_voice/test/ -v
```

### Bảng kết quả chi tiết từng file kiểm thử:
| File kiểm thử | Số test | Kết quả | Thời gian | Nội dung xác minh |
|---|---|---|---|---|
| `test_ros_lifecycle.py` | 14 | **14/14 PASSED** | ~37s | Vòng đời Nav2, 3 pha cancel/pause, server reject, pause in resolving, replace, freshness, deduplication, tts sync |
| `test_planner_resolution.py` | 8 | **8/8 PASSED** | ~5.8s | So sánh độ dài đường đi, unreachable, timeout candidate, late callback discard, replace category, planner missing |
| `test_audio.py` | 12 | **12/12 PASSED** | ~3.5s | STT known speech WAV, silence WAV, model missing, invalid format, wake timeout, Piper WAV format, TTS cancel queue, dropped queue full, ALSA devices |
| `test_nlu.py` | 24 | **24/24 PASSED** | ~0.04s | Cổng A3/B2, cách đọc tiếng Việt, phủ định, cancel/pause/resume, ambiguous, missing slot, Wi-Fi, sạc pin, giờ mở cửa, Unicode tổ hợp |
| `test_places.py` | 7 | **7/7 PASSED** | ~0.02s | Schema JSON places/intents, demo flag, required gates, aliases không trùng lặp |
| **TỔNG CỘNG** | **65** | **65/65 PASSED 100%** | **41.27s** | **Đạt tiêu chuẩn xuất xưởng và nghiệm thu** |

- File log bằng chứng chi tiết: `.agent/airport_robot_voice/evidence/full_pytest.log`.
- Kịch bản NLU độc lập: `python3 src/airport_robot_voice/scripts/test_nlu.py` đạt **61/61 PASSED**.

---

## 6. Phân định kết quả kiểm chứng theo 3 môi trường

### 6.1. Môi trường Test Fixture & Mock Server (ĐÃ XÁC MINH 100%)
- **Trạng thái**: ĐẠT 100% (65/65 tests pytest, 61/61 test_nlu).
- **Phạm vi xác minh**:
  - Xử lý NLU tiếng Việt, phủ định, Unicode tổ hợp, ranh giới từ, trích xuất slot, lọc đích tồn tại.
  - Thuật toán chọn đường bằng `ComputePathToPose` (so sánh độ dài đường đi, xử lý unreachable, timeout, cancel ngắt resolving, late callback discard).
  - Toàn bộ máy trạng thái vòng đời Nav2 (PAUSE, RESUME, CANCEL, REPLACE, STATUS, deduplication, pending-acceptance, server reject).
  - Đồng bộ âm thanh TTS (speech_id, mission_id, tts_ack, timeout fallback, queue cancellation).
  - Định dạng file WAV, nhận diện âm thanh offline qua Vosk, xử lý hàng đợi audio.

### 6.2. Môi trường Thiết bị âm thanh vật lý (ĐÃ XÁC MINH CẤU HÌNH, CẦN THỬ NGHIỆM VỚI NGƯỜI DÙNG)
- **Đã kiểm chứng**:
  - Container nhận diện 11 thiết bị âm thanh ALSA qua `/dev/snd`.
  - eSpeak-ng và Piper tổng hợp tiếng Việt ra file WAV chuẩn mono 16-bit.
  - Model Vosk tiếng Việt giải mã âm thanh chính xác.
  - Chế độ `tts_engine:=log` và `mode:=keyboard` hoạt động độc lập hoàn hảo không cần micro/loa.
- **Cần xác minh thêm với người dùng**:
  - Nói trực tiếp qua micro phần cứng máy tính ThinkPad (kiểm tra mức âm lượng thu âm thực tế).
  - Nghe trực tiếp qua loa ngoài laptop (phụ thuộc routing âm thanh ALSA giữa host Ubuntu 22.04 và Docker).

### 6.3. Môi trường Điều hướng Nav2 / Gazebo (ĐÃ XÁC MINH LOGIC, CHỜ KHỞI CHẠY MÔ PHỎNG)
- **Đã kiểm chứng**:
  - Giao tiếp Action Client `NavigateToPose` và `ComputePathToPose` đúng chuẩn interface `nav2_msgs` trên ROS 2 Jazzy.
  - Dữ liệu địa điểm `places.json` đã gắn cờ `"_demo_data": true`.
- **Cần xác minh thêm với người dùng**:
  - Thay thế tọa độ demo bằng tọa độ thực tế từ bản đồ AMCL của mô phỏng Gazebo sân bay (`/amcl_pose`).
  - Quan sát robot di chuyển thực tế trong Gazebo khi nhận lệnh giọng nói.
