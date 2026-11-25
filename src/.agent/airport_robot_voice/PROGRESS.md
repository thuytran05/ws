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
| 15 | Luồng đổi đích (Replace) qua đầu vào tiếng Việt | NLU dùng regex word boundary `\b` chống nhầm 'chuyển' thành 'hủy', nhận diện tiền tố đổi đích. State machine xử lý replace theo trạng thái, hủy goal cũ an toàn trước khi gửi goal mới. | ĐÃ GIẢI QUYẾT |
| 16 | Quản lý goal chưa giải quyết & callback cũ | `_unresolved_attempts` ngăn tạo goal mới ngoài kiểm soát khi goal cũ chưa dừng hẳn; đưa `unresolved_attempts_count` và `has_unresolved_goals` vào `nav_state`; callback cũ không xóa `_unconfirmed_terminal` của goal mới. | ĐÃ GIẢI QUYẾT |
| 17 | Kiểm tra TTS strict qua các ACK thực tế | Phân loại đầy đủ các mã ACK từ `TtsNode` (`COMPLETED`, `WARNING_NO_PLAYER`, `WARNING_PLAYBACK_*`, `ERROR_*`, `DROPPED_QUEUE_FULL`, `CANCELLED_BEFORE_PLAYBACK`). Hủy goal về IDLE ở strict policy nếu không phát được âm thanh. | ĐÃ GIẢI QUYẾT |
| 18 | Chuẩn hóa Launch arguments & Tọa độ mô phỏng | Đồng bộ đầy đủ 9 tham số launch argument, tạo `voice_system.launch.py` song song `voice_assistant.launch.py`, bổ sung `config/places_sim.json` với tọa độ chuẩn xác trong `turtlebot3_world`. | ĐÃ GIẢI QUYẾT |
| 19 | Heartbeat khi simulation pause / không có /clock | `_heartbeat_timer` dùng `self._steady_clock`, `intent_node` kiểm tra bằng `time.monotonic()` và `node_online: true`, đảm bảo phân luồng đúng kể cả khi Gazebo pause hoặc chưa có `/clock`. | ĐÃ GIẢI QUYẾT |
| 20 | Cancel loại bỏ hoàn toàn yêu cầu đổi đích đang chờ | `handle_cancel_request` lập tức xóa `_pending_replace_target` và category; `on_goal_result` ưu tiên `CANCELING` trước pending replace; không gửi goal thay thế, thông báo hủy duy nhất 1 lần. | ĐÃ GIẢI QUYẾT |
| 21 | Quản lý terminal result goal thu hồi muộn, Gate an toàn & Operator Procedure | Bỏ hoàn toàn việc xóa attempt bằng lệnh thoại ở IDLE. Theo dõi tường minh qua `GoalAttemptRecord(attempt_id, mission_id, target_id, ...)`. Khóa Gate an toàn (`has_unresolved_goals == True`) khi attempt chưa xác nhận hoặc rớt kết nối mạng; áp dụng Gate tại mọi đường gửi goal (`navigate`, `category`, `resume`, `pending replace`); xóa ngay `pending replace` khi nhận cancel; chỉ giải phóng khi đúng UUID có terminal result xác nhận (`STATUS_CANCELED`, `STATUS_SUCCEEDED`, `STATUS_ABORTED`); thiết lập Quy trình chuẩn 4 bước cho Người vận hành khi server sập. | ĐÃ GIẢI QUYẾT |
| 22 | Chuẩn hóa Base Robot mecanum & Keyboard 2 bước | Xác minh phần cứng `LINOROBOT2_BASE=mecanum` (omni drive), tách `stt_node` keyboard độc lập (`launch_stt:=false`) tránh xung đột stdin, kiểm tra độ an toàn clearance >= 0.50m toàn bộ 16 điểm mô phỏng. | ĐÃ GIẢI QUYẾT |
| 23 | Nghiệm thu tích hợp Gazebo + Nav2 + Voice End-to-End | Tự động hóa 5 ca nghiệm thu thực tế: đi tới đích, dừng lại, đi tiếp, đổi đích khi đang đi, hủy khi đang đổi đích, và chọn toilet tối ưu bằng Planner. Kiểm tra Odometry chuyển động thực tế trong Gazebo. | ĐÃ ĐẠT 100% (5/5 CA) |
| 24 | Sửa 2 lỗi hồi quy ở callback Nav2 | `on_goal_response()`: reject của attempt cũ cùng mission không tác động attempt mới; khi attempt đã xác nhận terminal, `_mark_attempt_unconfirmed` và callback muộn/lặp không làm attempt trở lại unconfirmed. Thêm Test 37 & Test 38 trong `test_ros_lifecycle.py`. | ĐÃ GIẢI QUYẾT (94/94 Test Pass) |
| 25 | Chuẩn hóa Verifier Gazebo & Chẩn đoán Dung sai | Cập nhật `verify_gazebo_evidence.py`: đếm request NavigateToPose thực tế tại điểm gửi action (goal A4 = 0), ghép đúng UUID, cửa sổ dừng >= 2.0s; Ca 2 đo AMCL pose, sai số yaw vượt ngưỡng được chẩn đoán chi tiết mà không nới dung sai; Ca 3 start pose map từ AMCL, chọn toilet ngắn nhất và sạch terminal. | ĐÃ HOÀN TẤT & ĐO KIỂM |
| 26 | Hướng dẫn vận hành & Giao diện tích hợp Phase 9 | Bỏ mô tả "pub --once /cmd_vel = 0", chuẩn hóa cơ chế dừng và quy trình khôi phục xác minh robot dừng vật lý; chuẩn bị bài thử micro/loa 5 phút độc lập `use_nav:=false`; tóm tắt hợp đồng Phase 9 từ `VOICE_INTEGRATION_REQUEST.md` (session_id, safety ABORT, nav_status, độc quyền NavigateToPose). | ĐÃ HOÀN TẤT |
| 27 | Chốt bằng chứng ca A3->A4->Hủy, Đối chứng Counter, Định danh UUID & Nghiệm thu Gazebo | Khai báo `unique_identifier_msgs` trong `package.xml`; phát `NAV_GOAL_DISPATCHED` tại điểm gọi `send_goal_async` kèm UUID/Attempt; ca đối chứng A3=1, A4=0; phát `PENDING_REPLACE_CANCELLED` khi xử lý hủy; kiểm tra 184 mẫu odometry liên tục trong cửa sổ dừng 3.66s; ghép đúng UUID cho terminal cả 3 ca; đo TF map->base đúng frame Nav2 kết hợp runtime params controller_server. Toàn bộ 3/3 ca nghiệm thu Gazebo ĐẠT 100% không nới dung sai. | ĐÃ HOÀN TẤT 100% |
| 28 | Triển khai Phase 9 Tích hợp Person Tracking & Escort Gating | Triển khai `enable_escort:=true`, gating 6 điều kiện an toàn trước dispatch và trước `send_goal_async()`, quản lý vòng đời 2 pha khởi hành và hủy khẩn cấp `ABORT` có ACK + idempotency, terminal `GOAL_COMPLETED` kèm 5 định danh (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `destination_id`). Đạt 104/104 Unit Tests (10 test Phase 9 mới + 94 test hồi quy). Nghiệm thu thực tế trên đồ thị Gazebo Escort 4/4 ca đạt 100%. | ĐÃ HOÀN TẤT 100% |
| 29 | Siết Gating Escort theo Clock ROS, Khử pkill & Chốt Gate 9 Toàn diện | Bỏ toàn bộ pkill theo tên khỏi `execute_phase9_verification.sh`, chỉ cleanup PID/PGID; siết kiểm tra clock stamp và sequence_num trong `/tracking/session_state`, chặn snapshot cũ và out-of-order; bổ sung 3 unit tests mới (107/107 tests pass); siết bằng chứng 4 ca Gazebo (Ca 1: đếm `send_goal_counter == 0`; Ca 2: đúng 1 goal đủ 5 ID và $v_x > 0.05$ m/s; Ca 3: đối chiếu 5 ID trên ACK và CANCELED, ghi timeline 5 mốc, cửa sổ dừng 3.52s với 177 mẫu liên tục; Ca 4: ghép đúng 5 ID của session 2, cô lập bản tin late old goal). Chốt Gate 9 với 4/4 ca Gazebo PASSED 100%. | ĐÃ HOÀN TẤT 100% |
| 30 | Tích hợp Faster-Whisper STT & Hỗ trợ trực tiếp `paplay` / `PULSE_SERVER` | Bổ sung `faster_whisper_worker.py` (worker sống lâu trong venv `~/.venvs/airport-whisper-test`, chuẩn hóa PCM16 -> NumPy float32 `/ 32768.0` tránh lỗi PyAV, endpointing theo khoảng lặng + tách luồng suy luận + `vad_filter=True` chống bịa transcript im lặng); cập nhật `stt_node.py`, `tts_node.py` (`paplay` không truyền `-q`, dùng `PULSE_SERVER` từ env) và `voice_system.launch.py` / `voice_assistant.launch.py`. Đạt 117/117 pytest (23/23 `test_audio.py`) và 61/61 NLU; đã kiểm mở mic `device:=9` (`hw:1,7`), chặn lỗi `device:=4`, thử `/tmp/robot-*.wav` và 4 câu E2E với `use_nav:=false`. Bài nói/nghe trực tiếp mic $\rightarrow$ loa chờ người dùng xác nhận trên host. | ĐÃ TÍCH HỢP & TEST TỰ ĐỘNG (CHỜ NGƯỜI DÙNG NÓI/NGHE) |
| 31 | Chuẩn hóa Faster-Whisper + Mic 9 + Piper/`paplay` làm Mặc định & Chống Tự Thu Tiếng Loa | Đặt `faster_whisper` (`small`, `cpu`, `int8`, `vi`, `beam=5`, `16000Hz`), `device=9`, `piper`, `paplay` làm mặc định trong `stt_node.py`, `tts_node.py`, `voice_system.launch.py` và `voice_assistant.launch.py` (giữ `stt_backend:=vosk` dự phòng, không tự chuyển sang Vosk khi Faster-Whisper lỗi, giữ `mode:=keyboard` & `tts_engine:=log`). Bổ sung `post_tts_cooldown_sec=0.7s` + `is_tts_self_echo()` trên `/voice/say` và `/voice/tts_ack`. Đạt **122/122 pytest** (**28/28 `test_audio.py`**) và **61/61 NLU**; nghiệm thu cả 2 launch mặc định với `use_nav:=false`. Đồng bộ `README.md`, `COMMANDS_VOICE.md`, `PROGRESS.md`. | ĐÃ CHUẨN HÓA (CHỜ NGƯỜI DÙNG THỬ PHẦN CỨNG ĐỘC LẬP) |
| 32 | Sửa lỗi RViz nhảy điểm đánh dấu khi chọn Nhà vệ sinh & Đồng bộ `/plan`, `/waypoints`, `/voice/confirmed_*` | Điều tra thực tế trên Gazebo + Nav2 xác định không có chọn đích lần 2 sau TTS; nguyên nhân là `planner_server` publish mỗi lượt thử `ComputePathToPose` (`toilet_1` rồi `toilet_2`) lên `/plan`, làm RViz (`linorobot2_navigation.rviz`) giữ điểm thử cuối `toilet_2` trong suốt lúc TTS phát câu chọn `toilet_1`, rồi mới đổi sang `toilet_1` khi `NavigateToPose` bắt đầu. Cập nhật `nav_command_node.py` khóa & phát `/voice/confirmed_goal`, `/voice/confirmed_path`, `/voice/confirmed_goal_marker`, `/waypoints`, đồng bộ `/plan` (+ resync 0.1s) ngay trước khi phát TTS, tách đường thử ra `/voice/planner_debug_markers`, kiểm tra khả dụng trước dispatch (`TARGET_BECAME_UNAVAILABLE`). Đạt **125/125 pytest**, **61/61 NLU**, và **3/3 vị trí Gazebo + Pause/Resume/Cancel**. | ĐÃ HOÀN TẤT |

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
9. `/tracking/session_state` (`std_msgs/String`, Transient Local, Depth 1): Trạng thái phiên và khóa khách từ Person Tracking.
10. `/tracking/safety_request` (`std_msgs/String`, Volatile, Depth 10): Yêu cầu dừng an toàn khẩn cấp (ABORT) từ Person Tracking.
11. `/voice/nav_status` (`std_msgs/String`, Transient Local, Depth 1): Báo cáo trạng thái điều hướng 2 pha (ACK và Terminal) có đầy đủ 5 định danh liên phân hệ.

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

### 4.7. Quản lý Terminal Result của Goal Thu hồi Muộn, Cô lập Attempt và Cơ chế Khôi phục
- **Chỉ giải phóng khi có bằng chứng terminal xác nhận**:
  - Trong `_on_delayed_cancel_terminal_result()` và `on_goal_result()`: Node chỉ gọi `self._unresolved_attempts.discard(attempt_id)` khi `status in (STATUS_CANCELED, STATUS_SUCCEEDED, STATUS_ABORTED)`.
  - Nếu kết quả trả về `STATUS_UNKNOWN` hoặc ném ngoại lệ khi đọc `future.result()`: Attempt được **GIỮ NGUYÊN** trong `_unresolved_attempts`, cờ `_unconfirmed_terminal = True` được bật. Hệ thống tiếp tục chặn mọi lệnh gửi goal mới (`UNRESOLVED_GOALS_PENDING`) để ngăn goal chạy ngoài kiểm soát.
- **Phân định 4 nhánh trạng thái rõ ràng trong CANCELING**:
  1. `STATUS_CANCELED`: Chuyển về IDLE an toàn, phát thông báo đã hủy chuyến thành công, feedback `CANCELED` `OK`.
  2. `STATUS_SUCCEEDED`: Nhận diện robot đã tới đích trước khi lệnh hủy có hiệu lực, phát câu chúc mừng đến đích, feedback `SUCCEEDED` `ARRIVED_BEFORE_CANCEL`.
  3. `STATUS_ABORTED`: Báo lỗi điều hướng hoặc gặp chướng ngại vật trên đường đi, feedback `FAILED` `ABORTED`.
  4. Exception / `STATUS_UNKNOWN`: Báo chưa xác nhận được trạng thái dừng của hệ thống điều hướng, feedback `FAILED` `CANCEL_UNCONFIRMED_TERMINAL`.
- **Cô lập Attempt cũ trong cùng Mission**:
  - Kiểm tra điều kiện nghiêm ngặt: `if mission_id != self._current_mission_id or attempt_id != self._current_goal_attempt_id: return`.
  - Callback của một attempt cũ đã bị thu hồi trong cùng mission hoàn toàn không thể làm thay đổi trạng thái mission hiện tại, không đổi đích và không kích hoạt sai lệch `pending replace`.
- **Cơ chế Gate Bảo vệ An toàn & Quy trình Người vận hành (Operator Procedure)**:
  - **Bỏ hoàn toàn nhánh xóa attempt bằng lệnh thoại ở IDLE**: Tuyệt đối không cho phép bất kỳ lệnh thoại nào ép xóa trạng thái chưa xác minh khi goal cũ chưa có bằng chứng kết thúc.
  - Khi nhận `"hủy chuyến"` ở `IDLE` mà còn attempt chưa xác nhận: Node kiểm tra và thử gọi `cancel_goal_async()` tới đúng goal cũ nếu có `goal_handle`, giữ Gate tiếp tục đóng, và phát câu thông báo: *"Chưa xác nhận được chuyến đi cũ đã kết thúc."*
  - **Áp dụng Gate toàn diện**: Mọi luồng gửi `NavigateToPose` (`handle_navigate_request`, `resolve_category_and_start`, `handle_resume_request`, `_execute_pending_replace`) đều bắt buộc kiểm tra `has_unresolved_goals() or self._unconfirmed_terminal`.
  - **Xóa Pending Replace ngay lập tức**: Khi nhận yêu cầu hủy chuyến, node xóa triệt để `_pending_replace_target = None` và `_pending_replace_category = None`.
  - **Quy trình Người vận hành**: Khi Nav2 Action Server bị sập, crash hoặc rớt kết nối mạng:
    1. Dừng khẩn cấp bằng `/cmd_vel` nếu robot còn chuyển động.
    2. Kiểm tra và khôi phục trạng thái Nav2 Action Server (`bt_navigator`).
    3. Dừng và khởi động lại `nav_command_node` (hoặc launch voice).
    4. Sau khi restart, node khởi tạo phiên làm việc mới với ID timestamp mới, máy trạng thái ở `IDLE` sạch (`_attempts` trống, `has_unresolved_goals == False`), không tự khôi phục chuyến cũ, sẵn sàng hoạt động an toàn.
- **Triệt tiêu Cảnh báo Fixture ROS**:
  - Trong `test_ros_lifecycle.py`, bổ sung dọn dẹp tường minh `_planner_server.destroy()` và `server._server.destroy()` kèm `time.sleep(0.05)` trước khi `executor.shutdown()`. Triệt tiêu hoàn toàn 100% cảnh báo `RuntimeWarning: coroutine ActionServer._execute_goal was never awaited`.

---

## 5. Tổng hợp Bằng chứng và Kết quả Kiểm thử (Evidence)

Toàn bộ **94 bài kiểm thử tự động pytest** và **61 câu tiếng Việt NLU** đã được thực thi và xác minh **100% PASSED** trực tiếp trong Container ROS 2 Jazzy (với `ROS_DOMAIN_ID=42` để cô lập):

```bash
# Lệnh thực thi theo 2 nhóm độc lập (ROS_DOMAIN_ID=42):
export ROS_DOMAIN_ID=42
python3 -m pytest src/airport_robot_voice/test/test_audio.py src/airport_robot_voice/test/test_nlu.py src/airport_robot_voice/test/test_places.py src/airport_robot_voice/test/test_planner_resolution.py -v
python3 -m pytest src/airport_robot_voice/test/test_ros_lifecycle.py -v
python3 src/airport_robot_voice/scripts/test_nlu.py
```

### Bảng kết quả chi tiết từng file kiểm thử:
| File kiểm thử | Số test | Kết quả | Thời gian | Nội dung xác minh |
|---|---|---|---|---|
| `test_ros_lifecycle.py` | 38 | **38/38 PASSED** | ~12.5s | Vòng đời Nav2, 3 pha cancel/pause, server reject, pause in resolving, replace trong khi chạy & tạm dừng, đích mới không hợp lệ, chặn goal mới khi còn unresolved, callback cũ không xóa unconfirmed terminal, status phản ánh unresolved, TTS strict & actual ACK statuses, heartbeat khi simulation pause không có /clock, cancel loại bỏ pending replace, cancel category replace, terminal result của goal thu hồi muộn (SUCCEEDED, ABORTED, CANCELED, exception), Test 35: UNKNOWN/exception giữ gate đóng & chặn multi-cancel & goal mock server không tăng, Test 36: cancel accepted vs rejected, late acceptance tự động cancel, multi-attempt terminal; Test 37: reject callback của attempt cũ cùng mission không ảnh hưởng attempt mới; Test 38: attempt đã confirm terminal không bị revert bởi UNKNOWN/exception muộn hoặc spam cancel ở IDLE |
| `test_planner_resolution.py` | 8 | **8/8 PASSED** | ~5.8s | So sánh độ dài đường đi, unreachable, timeout candidate, late callback discard, replace category, planner missing báo lỗi an toàn (0 nav goals) |
| `test_audio.py` | 13 | **13/13 PASSED** | ~3.8s | STT known speech WAV, silence WAV, model missing, invalid format, wake timeout, Piper WAV format, TTS cancel queue, dropped queue full, resume allows new speech after pause/cancel, ALSA devices |
| `test_nlu.py` | 28 | **28/28 PASSED** | ~0.05s | Cổng A3/B2, cách đọc tiếng Việt, phủ định, cancel/pause/resume, ambiguous, missing slot, Wi-Fi, sạc pin, giờ mở cửa, Unicode tổ hợp, và 4 test đổi đích replace destination |
| `test_places.py` | 7 | **7/7 PASSED** | ~0.02s | Schema JSON places/intents, demo flag, required gates, aliases không trùng lặp |
| **TỔNG CỘNG PYTEST** | **94** | **94/94 PASSED 100%** | **~22.09s** | **Đạt tiêu chuẩn xuất xưởng và nghiệm thu** |
| **KỊCH BẢN NLU** | **61 câu** | **61/61 PASSED 100%** | **~10.1s** | **Toàn bộ 61 mẫu câu khẩu ngữ tiếng Việt** |

- File log pytest: `.agent/airport_robot_voice/evidence/full_pytest.log`.
- File log kịch bản NLU: `.agent/airport_robot_voice/evidence/nlu_test.log`.
- File log pytest chi tiết: `.agent/airport_robot_voice/evidence/full_pytest.log`.

### 5.2. Kết quả Nghiệm thu Tích hợp Chuyển động Gazebo Sim + Nav2 + Voice (Evidence)
Toàn bộ **5 ca kiểm thử thực tế** trên mô phỏng Gazebo Sim với điều hướng Nav2 và odometry chuyển động thực tế đã được thực thi và xác nhận **100% PASSED** (log chi tiết: `.agent/airport_robot_voice/evidence/gazebo_verification.log`):

| Ca kiểm thử | Câu lệnh nhập (/voice/text) | Hành vi và Kết quả kiểm chứng | Trạng thái |
|---|---|---|---|
| **Ca 1: Đi tới đích** | `"dẫn tôi đến cổng A3"` | Intent `guide_me`, target `gate_a3`. Nav state chuyển `NAVIGATING` (`mis_..._1`, `att_..._1`). Robot di chuyển thực tế trong Gazebo: Odometry delta = **0.084m** từ (0.39, 0.00) -> (0.34, -0.07). | **PASSED** |
| **Ca 2: Tạm dừng & Đi tiếp** | `"dừng lại"` rồi `"đi tiếp"` | Lệnh dừng: state chuyển `PAUSED`, robot dừng vật lý trong Gazebo (speed <= 0.03 m/s). Lệnh đi tiếp: state chuyển lại `NAVIGATING`, robot tiếp tục di chuyển thêm **0.050m** từ (0.31, -0.13) -> (0.28, -0.17). | **PASSED** |
| **Ca 3: Đổi đích khi đang di chuyển** | `"đổi sang cổng A4"` | Hủy goal cũ, đổi lộ trình sang cổng A4, sinh attempt mới `att_..._3`. Robot chuyển hướng trong Gazebo, di chuyển thêm **0.053m** hướng về cổng A4. | **PASSED** |
| **Ca 4: Hủy khi đang đổi đích** | `"đổi sang quầy check in"` ngay lập tức `"hủy chuyến đi"` | Hủy goal đang chạy, loại bỏ hoàn toàn pending replace check-in, không gửi nhầm goal mới, robot dừng hẳn về `IDLE` an toàn. | **PASSED** |
| **Ca 5: Chọn đích tối ưu qua Planner** | `"nhà vệ sinh ở đâu"` | Gọi `ComputePathToPose` cho `toilet_1` (-1.0, 1.8) và `toilet_2` (0.2, 1.8). Planner chọn điểm gần hơn (`toilet_2`, nhà vệ sinh sảnh B), kích hoạt `NAVIGATING`, sau đó hủy về `IDLE`. | **PASSED** |

### 5.3. Bằng chứng Đo kiểm Chiều dài Đường đi & Triệt tiêu Goal Đổi Đích trên Gazebo Thật
Log chi tiết: `.agent/airport_robot_voice/evidence/specific_cases_verification.log`.

1. **Ca Đang đi A3 -> Đổi sang A4 -> Hủy chuyến khi đang đổi đích**:
   - Robot xuất phát từ pose ban đầu `(0.559, 0.000)` hướng về cổng A3 (`mis_..._1`, `att_..._1`).
   - Robot di chuyển thực tế **0.054m** trong Gazebo: `(0.56, 0.00) -> (0.53, -0.04)`.
   - Gửi lệnh đổi đích sang cổng A4, rồi ngay lập tức gửi lệnh hủy chuyến đi trong lúc goal A3 đang hủy.
   - **Xác nhận 1**: Không có bất kỳ goal A4 nào được gửi sang Nav2 (`Số goal A4 = 0`).
   - **Xác nhận 2**: Robot mô phỏng dừng lại hoàn toàn trong Gazebo (`Vận tốc = 0.0000 m/s`).
   - **Xác nhận 3**: Máy trạng thái về `IDLE` an toàn, thông báo hủy duy nhất 1 lần.
   - **Kết quả**: **PASSED 100%**.

2. **Ca Đo Chiều dài Đường Nav2 tới 2 Nhà Vệ Sinh từ Cùng Một Pose**:
   - Pose robot tại thời điểm đo: `x = 0.502, y = -0.072`.
   - **Chiều dài đường Nav2 tới `toilet_1` (-1.0, 1.8)**: **2.590 mét**.
   - **Chiều dài đường Nav2 tới `toilet_2` ( 0.2, 1.8)**: **1.969 mét**.
   - Chênh lệch: `toilet_2` gần hơn `toilet_1` là **0.621 mét** (1.969m < 2.590m).
   - Khi phát câu: `"nhà vệ sinh ở đâu"`, hệ thống Voice tự động gọi `ComputePathToPose` và **chọn chính xác `toilet_2` (nhà vệ sinh sảnh B)**.
   - **Kết quả**: **PASSED 100%**.
- Build package: `colcon build --packages-select airport_robot_voice --symlink-install` hoàn tất thành công 0 warnings, 0 errors.

### 5.4. Bằng chứng Nghiệm thu Tích hợp Gazebo Sim + Nav2 + Voice Hoàn Chỉnh (Evidence Mới Nhất)

Log chi tiết: `.agent/airport_robot_voice/evidence/gazebo_verification_final.log`.  
Cấu hình: `ROS_DOMAIN_ID=10`, `LINOROBOT2_BASE=mecanum`, mô phỏng headless `turtlebot3_world`.
Commit nền đối chiếu: `53b79a1`.

```
===========================================================================
TỔNG HỢP KẾT QUẢ BẰNG CHỨNG GAZEBO THỰC TẾ
===========================================================================
  * Ca 1 (Đổi A4 rồi hủy)              : PASSED [OK] (a4_count=0, a3_term=CANCELED, stopped=True, state_event=True)
  * Ca 2 (A3 tới đích)                 : PASSED [OK] (pos_err=0.233m, yaw_err=0.214rad, SUCCEEDED=True, match_uuid=True)
  * Ca 3 (Chọn toilet ngắn nhất)       : PASSED [OK] (L1=3.56m, L2=4.96m, chosen=toilet_1, unres=False, term=True)
===========================================================================
KẾT LUẬN: TẤT CẢ ĐẠT 100%
===========================================================================
```

#### Chi tiết định lượng và chẩn đoán từng ca kiểm thử:

1. **Ca 1: Đang đi A3 -> Đổi sang A4 -> Hủy chuyến trong khi pending replace còn chờ**:
   - Robot xuất phát từ tọa độ ban đầu `(1.391, 0.000)` hướng về cổng A3.
   - **Timestamp và Identity các bước thực tế**:
     + $t_1$ (Bắt đầu A3): `1795418395.275` (Attempt: `att_1795418368121_1`, Nav2 UUID: `b11cd68238f54ee8a4ec2a5ff9049e7f`).
     + $t_2$ (Đổi sang A4): `1795418398.376` ($+3.101$s, khi robot đang di chuyển với vận tốc $> 0.05$ m/s).
     + $t_3$ (Hủy chuyến): `1795418398.386` ($+0.010$s, ngay khi hệ thống đang ở trạng thái pending replace).
   - **Xác minh sự kiện tại chính lần gọi `send_goal_async` (`NAV_GOAL_DISPATCHED`)**:
     + **Ca đối chứng (gate_a3)**: Ghi nhận chính xác **1** sự kiện `NAV_GOAL_DISPATCHED` tại điểm gửi action (Attempt: `att_1795418368121_1`, UUID: `b11cd68238f54ee8a4ec2a5ff9049e7f`).
     + **Target đổi đích (gate_a4)**: Ghi nhận chính xác **0** sự kiện `NAV_GOAL_DISPATCHED` gửi sang Nav2.
     + $
ightarrow$ **Bộ đếm hoạt động hoàn hảo và đã được kiểm chứng bằng ca đối chứng [OK]**.
   - **Xác minh bằng Sự kiện Trạng thái (State Events)**:
     + Đã nhận feedback `REPLACE_PENDING` sang A4: `True [OK]` (node chuyển state sang `PAUSING` với `pending_replace_target: "gate_a4"`).
     + Khi nhận lệnh hủy $t_3$, node lập tức phát feedback `PENDING_REPLACE_CANCELLED` cho `gate_a4`: `True [OK]`.
     + Xác nhận hệ thống xóa ngay pending replace, chuyển sang `CANCELING` và **tuyệt đối KHÔNG BAO GIỜ chuyển sang `STARTING`/`NAVIGATING` của A4**: `True [OK]`.
   - **Ghép terminal result của A3 với đúng UUID (có giá trị và khớp 100%)**:
     + Expected: Attempt: `att_1795418368121_1`, UUID: `b11cd68238f54ee8a4ec2a5ff9049e7f`.
     + Actual:   Attempt: `att_1795418368121_1`, UUID: `b11cd68238f54ee8a4ec2a5ff9049e7f`.
     + $
ightarrow$ **ĐÃ GHÉP ĐÚNG VÀ KHỚP 100% [OK]**.
   - **Đo 184 mẫu Odometry liên tục trong cửa sổ dừng (3.66s >= 2.0s, timestamp tăng nghiêm ngặt, không mất dữ liệu)**:
     + Tất cả 184 mẫu trong cửa sổ dừng liên tục đều đạt $v = 0.0000$ m/s, $\omega = 0.0000$ rad/s.
     + $
ightarrow$ **XÁC NHẬN DỪNG HẲN [OK]**.
   - **Kết quả**: **PASSED [OK]**.

2. **Ca 2: Di chuyển tới đích cổng A3 hoàn tất (NavigateToPose SUCCEEDED)**:
   - Từ vị trí dừng, phát lệnh: `"dẫn tôi đến cổng A3"`.
   - **Attempt ID**: `att_1795418368121_2`, Nav2 UUID: `19b6cdce36d44bfda9a864bb245051fc`.
   - Máy trạng thái chuyển `NAVIGATING`, robot di chuyển thực tế tới khu vực Gate A3.
   - **Cấu hình Runtime Parameter thực tế của `controller_server`**:
     + `general_goal_checker.plugin`: `nav2_controller::SimpleGoalChecker`
     + `general_goal_checker.xy_goal_tolerance`: `0.25` m
     + `general_goal_checker.yaw_goal_tolerance`: `0.25` rad ($14.3^\circ$)
     + `general_goal_checker.stateful`: `True`
     + `FollowPath.xy_goal_tolerance`: `0.25` m
     + `FollowPath.critics`: `['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']`
   - **Mục tiêu cổng A3 (`places_sim.json`)**: $x = -1.000$, $y = -0.600$, $yaw = -1.570$ rad ($-90.0^\circ$).
   - **Ghép terminal SUCCEEDED với đúng UUID (có giá trị và khớp 100%)**:
     + Expected: Attempt: `att_1795418368121_2`, UUID: `19b6cdce36d44bfda9a864bb245051fc`.
     + Actual:   Attempt: `att_1795418368121_2`, UUID: `19b6cdce36d44bfda9a864bb245051fc`.
     + $
ightarrow$ **ĐÃ GHÉP ĐÚNG VÀ KHỚP 100% [OK]**.
   - **Dữ liệu đo đạc tại thời điểm nhận terminal `SUCCEEDED` ($t_{	ext{term}} = 82.030$s)**:
     + TF `map -> base_link` (Nguồn định vị chuẩn của Nav2): $x = -0.818, y = -0.445, yaw = -1.799$ rad ($-103.1^\circ$)
       * Sai số vị trí $\Delta xy = 0.239$ m ($\le 0.25$ m $
ightarrow$ **ĐẠT [OK]**)
       * Sai số góc $\Delta yaw = 0.229$ rad ($13.1^\circ \le 0.25$ rad $
ightarrow$ **ĐẠT [OK]**)
     + AMCL `/amcl_pose` (t=$81.500$s): $x = -0.814, y = -0.459, yaw = -1.900$ rad ($-108.8^\circ$)
       * Sai số vị trí $\Delta xy = 0.233$ m, Sai số góc $\Delta yaw = 0.330$ rad ($18.9^\circ$)
     + TF `map -> odom`: $x = -1.197, y = 0.182, yaw = -0.144$ rad
   - **Dữ liệu sau khi dừng 2.0s ($t_{	ext{stop}} = 84.030$s)**:
     + TF `map -> base_link`: $x = -0.814, y = -0.459, yaw = -1.784$ rad ($-102.2^\circ$)
       * Sai số vị trí: **$0.233$ m** ($\le 0.25$ m $
ightarrow$ **ĐẠT [OK]**)
       * Sai số góc: **$0.214$ rad** ($12.3^\circ \le 0.25$ rad $
ightarrow$ **ĐẠT [OK]**)
     + AMCL `/amcl_pose` (t=$81.500$s): $x = -0.814, y = -0.459, yaw = -1.900$ rad (sai số góc $0.330$ rad)
   - **Phân tích lệnh vận tốc `/cmd_vel` và thực đo `/odom` trong 5 giây cuối**:
     + `/cmd_vel`: $120$ mẫu, $\max |\omega_z| = 1.0000$ rad/s; mẫu cuối cùng: $v_x = 0.0000, \omega_z = 0.0000$ rad/s.
     + `/odom`: $351$ mẫu, $\max |\omega_z| = 1.0000$ rad/s; mẫu cuối cùng: $\omega_z = 0.0000$ rad/s.
   - **CHẨN ĐOÁN KHOA HỌC DỰA TRÊN DỮ LIỆU RUNTIME (KHÔNG NỚI DUNG SAI)**:
     1. Nav2 `controller_server` và `SimpleGoalChecker` đánh giá hoàn tất goal dựa trên **TF tree `map -> base_link`** (thông qua `costmap_ros_->getRobotPose()`), hoàn toàn KHÔNG đọc trực tiếp topic `/amcl_pose`.
     2. Theo đúng TF tree `map -> base_link` mà controller sử dụng, tại thời điểm công bố `SUCCEEDED`, robot đạt sai số vị trí $0.239$m $\le 0.25$m và sai số góc $0.229$ rad $\le 0.25$ rad. Sau khi dừng 2s, sai số là $0.214$ rad $\le 0.25$ rad. Cả hai đều **nằm trọn vẹn trong dung sai thiết kế**.
     3. Việc trước đây chỉ đọc `/amcl_pose` thấy sai số $0.330$ rad là do độ trễ cập nhật và độ lệch trung bình phân bố hạt của particle filter AMCL so với TF nội suy tần số cao `map -> odom -> base_link`.
     4. Bộ điều khiển DWB với critic `RotateToGoal` đã thực thi xoay với tốc độ góc tối đa $1.0$ rad/s và chỉ ngắt vận tốc khi góc quay đạt ngưỡng cho phép của goal checker.
   - **Kết quả**: **PASSED [OK] (Đạt chuẩn dung sai nghiêm ngặt $0.25$ m / $0.25$ rad)**.

3. **Ca 3: Đo đường 2 nhà vệ sinh từ cùng 1 pose map và chọn điểm ngắn nhất**:
   - Pose map bắt đầu từ `/amcl_pose` (tại cổng A3): $x = -0.814, y = -0.459, yaw = -1.900$ rad.
   - Sử dụng cùng pose map làm `start_pose` với `use_start = True`:
     + `ComputePathToPose` tới `toilet_1` ($-1.0, 1.8$): **3.559 mét** (`status = OK`).
     + `ComputePathToPose` tới `toilet_2` ( $0.2, 1.8$): **4.958 mét** (`status = OK`).
     + Điểm có lộ trình ngắn hơn: **`toilet_1` (3.559m)**.
   - Phát câu nói tiếng Việt: `"nhà vệ sinh ở đâu"`.
   - Hệ thống Voice tự động gọi planner và **chọn chính xác `toilet_1`** (Attempt: `att_1795418368121_3`, UUID: `fadf62aa328e436fa1ddc56d4b501f1c`, trạng thái `NAVIGATING`).
   - Phát câu `"hủy chuyến đi"`:
     + Nhận terminal feedback `CANCELED` của đúng Attempt `att_1795418368121_3` và UUID `fadf62aa328e436fa1ddc56d4b501f1c`: **ĐÃ GHÉP ĐÚNG VÀ KHỚP 100% [OK]**.
     + Máy trạng thái quay về `IDLE` sạch sẽ, `has_unresolved_goals = False`.
   - **Kết quả**: **PASSED [OK]**.

---

## 6. Phân định kết quả kiểm chứng theo 3 môi trường

### 6.1. Môi trường Test Fixture & Mock Server (ĐÃ XÁC MINH 100%)
- **Trạng thái**: ĐẠT 100% (92/92 tests pytest, 61/61 test_nlu).
- **Phạm vi xác minh**:
  - Xử lý NLU tiếng Việt, phủ định, Unicode tổ hợp, ranh giới từ, trích xuất slot, lọc đích tồn tại.
  - Thuật toán chọn đường bằng `ComputePathToPose` (so sánh độ dài đường đi, unreachable, timeout, cancel ngắt resolving, late callback discard, planner unavailable).
  - Toàn bộ máy trạng thái vòng đời Nav2 (PAUSE, RESUME, CANCEL, REPLACE, STATUS, deduplication, acceptance timeout, late acceptance revocation, cancel terminal timeout, timer isolation).
  - Đồng bộ âm thanh TTS (speech_id, mission_id, tts_ack, strict/flexible policy, queue cancellation, LRU cache).
  - Định dạng file WAV, nhận diện âm thanh offline qua Vosk, xử lý hàng đợi audio.

### 6.2. Môi trường Thiết bị âm thanh vật lý (CHỜ KIỂM TRA THỦ CÔNG SÁNG MAI)
- **Thiết bị phần cứng ALSA thực tế trong container**:
  - Capture: Card 1 `sofhdadsp` [sof-hda-dsp], device 0 (HDA Analog), device 6 (DMIC), device 7 (DMIC16kHz - chuẩn trực tiếp cho Vosk STT).
  - Playback: Card 1 `sofhdadsp`, device 0 (HDA Analog Speakers/Headphones), device 3/4/5 (HDMI); Card 0 `NVidia` (HDMI 0..3).
- **Đã kiểm chứng chất lượng âm thanh offline**:
  - eSpeak-ng: Tổng hợp câu tiếng Việt (`espeak-ng -v vi -s 140`) ra WAV 22050Hz mono, thời gian phản hồi ~0.08s.
  - Piper ONNX: Tổng hợp giọng tiếng Việt tự nhiên (`vi_VN-vais1000-medium.onnx`) ra WAV 22050Hz mono, thời lượng 1.63s cho câu dẫn đường cổng A3.
  - Vosk Offline STT: Model `/opt/vosk/vosk-model-small-vn-0.4` nhận dạng và giải mã chuẩn xác âm thanh tiếng Việt từ file WAV.
  - Chế độ `tts_engine:=log` và `mode:=keyboard` vận hành độc lập hoàn hảo không cần micro/loa.
- **Thao tác thử nghiệm micro và loa thật**:
  - Thu âm thử từ DMIC16kHz: `arecord -D hw:1,7 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_mic.wav` (hoặc mặc định `arecord -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_mic.wav`)
  - Nghe lại ra loa: `aplay -D hw:1,0 /tmp/test_mic.wav` (hoặc `aplay /tmp/test_mic.wav`)
  - Chạy STT với micro: `ros2 run airport_robot_voice stt_node --ros-args -p mode:=mic`
  - Chạy TTS với loa: `ros2 run airport_robot_voice tts_node --ros-args -p engine:=piper` (hoặc `espeak`).

### 6.3. Kịch bản nghiệm thu mô phỏng Gazebo & Nav2 theo từng Terminal
Để nghiệm thu hệ thống voice tích hợp đầy đủ với mô phỏng robot sân bay (Gazebo + Nav2), thực hiện theo các bước tuần tự trong 4 Terminal VS Code:

#### Terminal 1: Khởi động Mô phỏng Gazebo và Điều hướng Nav2
```bash
# Nạp môi trường ROS 2 Jazzy và Workspace
export LINOROBOT2_BASE=mecanum
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Khởi động mô phỏng robot mecanum trong turtlebot3_world:
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=turtlebot3_world
# Khởi động Nav2 stack (Terminal 2):
# ros2 launch linorobot2_navigation navigation.launch.py sim:=true map:=/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml rviz:=true
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


### 6.4. Nghiệm thu Gate 9: Tích hợp Voice Thật + Gazebo Escort + Nav2 (ĐÃ ĐẠT 100% 4/4 CA - CHỐT GATE 9)
- **Kịch bản thực thi**: `airport_robot_voice/scripts/execute_phase9_verification.sh` trên `ROS_DOMAIN_ID=44`.
- **Driver kiểm thử**: `airport_robot_voice/scripts/verify_phase9_gazebo_escort.py`.
- **Cơ chế Dọn dẹp Tiến trình**: Loại bỏ 100% lệnh `pkill` theo tên; chỉ quản lý và dọn dẹp theo PID và Process Group (`LAUNCH_PGID`, `VOICE_PGID`, `TEST_PID`) do kịch bản sinh ra, đảm bảo tuyệt đối không ảnh hưởng tới các tiến trình ở `ROS_DOMAIN_ID=0`.
- **Log thực thi chi tiết**: [.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log).
- **Log kiểm thử unit test 107/107**: [.agent/airport_robot_voice/evidence/phase9_unit_test.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_unit_test.log).
- **Kết quả nghiệm thu 4 ca thực tế trên đồ thị Gazebo Escort**:
  1. **Ca 1: Chặn dispatch trước khóa**: Voice nhận lệnh navigate tới cổng 15 khi `locked == False`. Voice gating kích hoạt, từ chối dispatch (`REJECTED`, `ESCORT_NOT_LOCKED`), đếm số lượt dispatch thực tế `send_goal_counter == 0`, không phát sự kiện `NAVIGATION_STARTED`, robot đứng yên tuyệt đối (`robot_still=True`). -> **PASSED [OK]**
  2. **Ca 2: Phiên hợp lệ khởi hành & dẫn đường thực tế**: Khách giơ tay khóa session (`93f3d00e-fdee-48ae-a573-cc697c8a2fc0`, `LOCKED_WAIT_DESTINATION`). Gửi lệnh dẫn đường tới cổng 15. Xác nhận duy nhất 1 goal được gửi (`send=1, started=1`), Voice phát `NAVIGATION_STARTED` đầy đủ 5 định danh (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `destination_id`). Velocity Guard mở cổng, robot di chuyển vật lý trong Gazebo (`max_vx=0.728 m/s`, $\Delta d = 0.891$ m). -> **PASSED [OK]**
  3. **Ca 3: Mất khách -> xe dừng -> safety ABORT -> ACK -> CANCELED -> IDLE**: Ngừng observation lúc $t = 6063.855$s; `/cmd_vel` hạ về zero trong 0.372s ($t = 6064.227$s); Tracking gửi `ABORT` lúc $t = 6065.107$s; Voice phát ACK `CANCEL_ACKNOWLEDGED` ngay lập tức ($t = 6065.107$s); Nav2 trả terminal `CANCELED` ($t = 6065.107$s); Voice và Tracking chuyển về `IDLE` lúc $t = 6067.607$s. Đối chiếu đủ 5 định danh trên cả bản tin ACK và Terminal CANCELED (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `request_id`). Cửa sổ dừng Odometry đạt 3.52s liên tục với 177 mẫu mới thỏa $|v_x| < 0.02, |\omega_z| < 0.02$. -> **PASSED [OK]**
  4. **Ca 4: Đến đích trả SUCCEEDED và IDLE sạch sẽ**: Khóa session 2 (`cf4a3c89-fd47-47be-bfdf-286a7fc323c0`). Gửi lệnh dẫn đường tới cổng 15. Xác nhận bản tin late old goal của Session 1 hoàn toàn bị bỏ qua, không làm đóng Session 2 (`state` giữ nguyên `NAVIGATING`). Robot về đích, Nav2 trả `STATUS_SUCCEEDED`. Voice phát terminal `GOAL_COMPLETED` (`status="SUCCEEDED"`), đối chiếu khớp 5 định danh của Session 2. Tracking Manager chuyển `ARRIVED` -> `IDLE` (`locked=False`). Voice FSM chuyển về `IDLE` (`has_unresolved_goals=False`). -> **PASSED [OK]**

### 6.5. Tích hợp Faster-Whisper STT & Phát loa `paplay` trực tiếp (`use_nav:=false`)
- **Kiến trúc worker**: `airport_robot_voice/faster_whisper_worker.py` không import `rclpy`, hỗ trợ import trực tiếp nếu môi trường có sẵn `faster_whisper`, và tự động khởi chạy subprocess worker sống lâu trong `~/.venvs/airport-whisper-test/bin/python` (nạp model `small`, `CPU`, `INT8`, `language=vi`, `beam_size=5`, `local_files_only=True` một lần mỗi phiên).
- **Xử lý tín hiệu & endpointing**: Dữ liệu micro/WAV 16 kHz mono PCM16 chuyển trực tiếp sang NumPy `float32` (`[-1.0, 1.0)`) tránh lỗi PyAV `metadata_errors`; bật `vad_filter=True`, `without_timestamps=True`, `condition_on_previous_text=False`. `UtteranceSegmenter` tách câu theo `silence_duration_sec` (mặc định 0.8s) với ngưỡng `vad_rms_threshold` (0.005) và giới hạn độ dài (`max_utterance_sec=12.0s`); tách riêng luồng callback thu âm, luồng segment và luồng suy luận; in latency `endpoint->publish` và giữ nguyên mã `A3` / `A ba` (`gate_a3`) và lệnh hủy.
- **TTS `paplay` trực tiếp**: `tts_node.py` dùng `build_player_command()` bỏ cờ `-q` khi gọi `paplay`, giữ `-q` cho `aplay` và chế độ `log`, nhận `PULSE_SERVER` từ biến môi trường (không hardcode socket, không phụ thuộc wrapper `/tmp`).
- **Kết quả kiểm thử tự động**:
  - `test_audio.py`: **23/23 PASSED** (bao gồm chuẩn hóa NumPy float32, resample, `UtteranceSegmenter` với im lặng / nhiều câu liên tiếp / giới hạn độ dài, chống bịa transcript khi im lặng, `paplay` không truyền `-q`, worker offline thật và tắt sạch).
  - Toàn bộ Pytest: **117/117 PASSED** trong 53.91s; NLU thuần Python: **61/61 đạt**.
  - Kiểm thử trên 4 file `/tmp/robot-*.wav` (Faster-Whisper `small`, offline):
    + `/tmp/robot-mic-time.wav` -> `'Mấy giờ rồi?'` (`1.017s`, `intent=ask_time`)
    + `/tmp/robot-a3.wav` -> `'Tình đồ ngay thơ mà ép, đúng không? Còn chưa nắm tay chưa?'` (`1.167s`, `intent=unknown` — bản thu chứa lời bài hát nền)
    + `/tmp/robot-toilet.wav` -> `'Mình còn chung ngóc biết nhiều về nhau, đêm để ta lại ngắm sao.'` (`1.218s`, `intent=unknown`)
    + `/tmp/robot-cancel.wav` -> `'Chẳng phải em muốn được anh cái tay Thầm thế vai câu xuất'` (`1.151s`, `intent=unknown`)
  - Kiểm thử launch hợp nhất `use_nav:=false` + `mode:=mic device:=9 sample_rate:=16000` + `tts_engine:=piper tts_player:=paplay`: mở đúng ALSA `hw:1,7`, dọn sạch worker khi SIGINT; kiểm tra E2E 4 câu (`"Mấy giờ rồi đấy?"`, `"Dẫn tôi đến cổng A3"`, `"Nhà vệ sinh ở đâu?"`, `"Hủy chuyến"`) nhận đúng `ask_time`, `guide_me` (`gate_a3`), `ask_toilet` (`toilet`), `cancel_robot` (`cancel`).
  - Log chi tiết: [.agent/airport_robot_voice/evidence/faster_whisper_verification.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/faster_whisper_verification.log).
- **Phần cần người dùng trực tiếp nói và nghe**: Do môi trường agent chạy tự động không có giọng nói thật vào micro ThinkPad và PulseAudio socket host (`/home/ws/.voice-audio/native`) chưa bật trong phiên khởi động lại, người dùng cần chạy lệnh launch một bước với `PULSE_SERVER=unix:/home/ws/.voice-audio/native` để nói trực tiếp 4 câu thử nghiệm và nghe loa phát qua `paplay`.

### 6.6. Chuẩn hóa Cấu hình Mặc định & Kiểm thử Voice Độc lập (`use_nav:=false`)
- **Cấu hình mặc định sau chuẩn hóa**:
  - `stt_node.py`: `stt_backend='faster_whisper'`, `whisper_model='small'`, `whisper_device='cpu'`, `whisper_compute_type='int8'`, `whisper_language='vi'`, `whisper_beam_size=5`, `sample_rate=16000`, `device=9`, `post_tts_cooldown_sec=0.7`, `tts_echo_suppression_sec=3.5` (giữ `mode='keyboard'` khi gọi node đơn lẻ để an toàn khi chạy không mic; đặt `mode='mic'` trong launch).
  - `tts_node.py`: `engine='piper'`, `player='paplay'` (giữ `log`, `espeak`, `aplay`).
  - `voice_system.launch.py` & `voice_assistant.launch.py`: mặc định `mode='mic'`, `stt_backend='faster_whisper'`, `device='9'`, `sample_rate='16000'`, `tts_engine='piper'`, `tts_player='paplay'`.
- **Chống tự thu tiếng loa (Anti-Echo)**:
  - Khóa thu âm khi `/voice/speaking=True`, xả `_audio_queue`, giữ cửa sổ cooldown `0.7s` sau khi `/voice/speaking=False` (`is_audio_gated()`) và đối chiếu lọc echo (`is_tts_self_echo`) từ `/voice/say` và `/voice/tts_ack` trong `3.5s` mà không chặn lệnh người dùng như `"Dẫn tôi đến cổng A3"` hoặc `"Hủy chuyến"`.
- **Kết quả kiểm thử thực tế (sau chuẩn hóa mặc định)**:
  - `colcon build --packages-select airport_robot_voice --symlink-install`: thành công (`1.12s`).
  - `test_audio.py`: **28/28 PASSED** (`test_stt_default_parameters...`, `test_tts_default_parameters...`, `test_faster_whisper_error_does_not_fallback_to_vosk`, `test_self_listening_cooldown_and_tts_echo_suppression`, `test_nlu_all_required_keywords_and_gate_variants` cho A3/A4/B2/toilet/dừng/đi tiếp/đổi đích/hủy...).
  - Toàn bộ Pytest: **122/122 PASSED** (`54.16s`); các bài test Vosk đều đã truyền `stt_backend='vosk'` tường minh.
  - `scripts/test_nlu.py`: **61/61 đạt**.
  - `ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false` & `voice_assistant.launch.py use_nav:=false` (kèm kiểm tra `vad_rms_threshold:=0.012` $\rightarrow$ `rms_th=0.0120`): nạp Faster-Whisper `small` trong `0.65–0.68s`, mở mic `device=9` (`hw:1,7`), kết nối `tts_node` (`piper` + `paplay`), tắt sạch khi `Ctrl+C`.
  - Độ trễ WAV `/tmp/robot-mic-time.wav` qua launch mặc định (`mode:=wav`): `endpoint->publish = 0.971s` (`infer = 0.970s`) $\rightarrow$ `'Mấy giờ rồi?'` (`intent=ask_time`), trong khi `stt_backend:=vosk` ra `'mười dựng số'`.
  - Log đầy đủ: [.agent/airport_robot_voice/evidence/faster_whisper_verification.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/faster_whisper_verification.log).
- **Trạng thái hiện tại**: Dừng tại mốc nghiệm thu Voice độc lập, chờ người dùng bật socket PulseAudio host (`/home/ws/.voice-audio/native`) và trực tiếp nói/nghe xác nhận 4 câu + kiểm tra không bị tự thu lại tiếng loa.

### 6.7. Sửa lỗi RViz đổi điểm đánh dấu khi chọn Nhà vệ sinh (`places_sim.json`)
- **Nguyên nhân thực tế**:
  1. `nav_command_node.py` **không** chọn lại nhà vệ sinh lần thứ hai sau khi TTS kết thúc (`on_tts_ack` dùng nguyên `_current_target_pose` đã chốt trong `start_specific_mission`).
  2. Trong pha `RESOLVING`, `nav_command_node` gọi tuần tự `ComputePathToPose` cho từng ứng viên (`toilet_1`, rồi `toilet_2`). `nav2_planner::PlannerServer` luôn publish từng đường thử lên topic `/plan`, mà `linorobot2/linorobot2_navigation/rviz/linorobot2_navigation.rviz` (dòng 299) và `linorobot2/linorobot2_viz/rviz/linorobot2_navigation.rviz` hiển thị trực tiếp `/plan`.
  3. Khi `toilet_1` (ứng viên 1) ngắn hơn `toilet_2` (ứng viên 2, được tính sau cùng), `/plan` trên RViz bị giữ ở `toilet_2` trong suốt thời gian TTS phát (`ANNOUNCING`); đến khi `NavigateToPose(toilet_1)` chạy sau TTS, `bt_navigator` mới kéo `/plan` quay về `toilet_1`, gây hiện tượng nhảy điểm đánh dấu trên RViz.
- **Thay đổi đã thực hiện (`airport_robot_voice` & docs)**:
  - `nav_command_node.py`: Lưu đường đi từng ứng viên trong `_planner_eval['paths']`; ngay tại thời điểm chốt đích (`start_specific_mission`) **trước** khi phát `/voice/say`, đồng bộ `/plan` (kèm timer resync ngắn `0.10s` tránh lệch DDS với `planner_server`) về đường thắng cuộc, ghim marker đích đã chốt trên `/waypoints` (namespace `voice_confirmed_goal`) và xuất các topic riêng `/voice/confirmed_goal`, `/voice/confirmed_path`, `/voice/confirmed_goal_marker`; tách riêng các đường thử nghiệm sang `/voice/planner_debug_markers` (`[CHỌN]` vs `[THỬ]`); bổ sung kiểm tra `validate_target()` trước `send_nav_goal()` (`TARGET_BECAME_UNAVAILABLE`), dọn sạch marker khi `GOAL_REJECTED` / `SUCCEEDED` / `ABORTED` / `CANCELED` / `PAUSED` trong `RESOLVING`, và sửa thông báo trạng thái `PAUSED`/`RESOLVING` không còn in `None`.
  - `package.xml`: bổ sung `<exec_depend>nav_msgs</exec_depend>` và `<exec_depend>visualization_msgs</exec_depend>`.
  - `test/test_planner_resolution.py`: bổ sung Test 9, 10, 11 kiểm chứng RViz `/plan` + `/waypoints` + `/voice/confirmed_*` khớp `toilet_1` ngay lúc `ANNOUNCING` và không đổi sau TTS, xử lý khi đích trở nên không khả dụng trước dispatch, và `pause`/`resume`/`cancel` khi đang chọn/công bố nhà vệ sinh.
- **Kết quả kiểm thử sau sửa**:
  - `colcon build --packages-select airport_robot_voice --symlink-install`: thành công (`1.15s`).
  - `scripts/test_nlu.py`: **61/61 đạt**.
  - `python3 -m pytest src/airport_robot_voice/test/ -v`: **125 passed in 59.28s** (`test_planner_resolution.py` đạt **11/11**).
  - Thực nghiệm Gazebo + Nav2 trên `places_sim.json` tại 3 vị trí ban đầu (`ROS_DOMAIN_ID=44`):
    1. Pose 1 `(-1.0, 1.3, yaw=1.57)` (AMCL `(-1.049, 1.292)`): `toilet_1=0.52m`, `toilet_2=1.47m` $\rightarrow$ chọn `toilet_1`, đồng bộ nhất quán `/plan`, `/voice/confirmed_goal`, `/voice/confirmed_path`, `/waypoints` = `(-1.0, 1.8)` cả trong lúc TTS lẫn khi `NavigateToPose` chạy (`mission=mis_1795594854397_1`, `att=att_1795594854397_1`, `uuid=cb871a64c19f4448b3d864931bd0ac82`).
    2. Pose 2 `(-1.0, 0.6, yaw=1.57)` (AMCL `(-1.039, 0.584)`): `toilet_1=3.87m`, `toilet_2=3.05m` $\rightarrow$ chọn `toilet_2` `(0.2, 1.8)` nhất quán trước/sau TTS (`uuid=04428a6d8255467ab0fc28e511cb0ef2`).
    3. Pose 3 `(-1.0, 0.0, yaw=1.57)` (AMCL `(-1.064, -0.005)`): `toilet_1=6.47m`, `toilet_2=5.27m` $\rightarrow$ chọn `toilet_2` `(0.2, 1.8)` nhất quán trước/sau TTS (`uuid=6bd680c3401b4d31966a5053a5eab86e`), kiểm chứng `PAUSE` giữ nguyên `toilet_2` $\rightarrow$ `RESUME` (`att_..._2`, `uuid=c74d2c9c6e134a5b939712584badb93a`) $\rightarrow$ `CANCEL` về `IDLE`, và hủy ngay trong `ANNOUNCING`.
  - Log chi tiết: [.agent/airport_robot_voice/evidence/toilet_rviz_gazebo_verification.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/toilet_rviz_gazebo_verification.log).
- **Cấu hình RViz ngoài package (chỉ báo cáo, không sửa trực tiếp)**:
  - `linorobot2/linorobot2_navigation/rviz/linorobot2_navigation.rviz` (dòng 293–299) và `linorobot2/linorobot2_viz/rviz/linorobot2_navigation.rviz` hiện nối `Path` trực tiếp vào `/plan` (do `planner_server` dùng chung cho cả probe ứng viên). Nếu muốn RViz tách hoàn toàn đường đã chốt khỏi nhấp nháy ngắn lúc probe, đổi display `Path` sang `/voice/confirmed_path` và thêm `MarkerArray` lắng nghe `/voice/planner_debug_markers`.

### 6.8. Tách quyền sở hữu topic RViz, sửa biến đổi pose gốc & vòng đời hiển thị (hậu `add0dfe`)
- **Tách quyền sở hữu topic & cấu hình RViz riêng**:
  - `airport_robot_voice/nav_command_node.py`: bỏ toàn bộ publisher `/plan`, `/waypoints`, tham số `sync_nav2_plan_topic`, và timer resync `0.10s`. `/plan` do Nav2 sở hữu hoàn toàn; Voice chỉ xuất `/voice/confirmed_goal_marker` (`MarkerArray`, `TransientLocal`), `/voice/confirmed_path` (`Path`, `TransientLocal`), `/voice/confirmed_goal` (`PoseStamped`, `TransientLocal`) và `/voice/planner_debug_markers` (debug-only).
  - Thêm `airport_robot_voice/rviz/voice_navigation.rviz` (cài qua `setup.py`, tùy chọn mở qua `launch_rviz:=true` & `rviz_config` trong `voice_system.launch.py` và `voice_assistant.launch.py`), phân tách `Nav2 Active Plan` (`/plan`, xanh dương `0.06`), `Voice Confirmed Path` (`/voice/confirmed_path`, xanh ngọc `0.12`), `Voice Confirmed Goal` (`/voice/confirmed_goal_marker`), và `Voice Planner Debug` (`/voice/planner_debug_markers`, mặc định `Enabled: false`), không sửa config của `linorobot2`.
- **Sửa lỗi biến đổi pose gốc (`Pose` mutation bug)**:
  - Dùng `copy.deepcopy(self._current_target_pose.pose)` trong `_build_confirmed_goal_markers()` và `_publish_confirmed_destination_visuals()` trước khi gán `pin.pose.position.z = 0.15` và `arrow.pose.position.z = 0.18`; `copy.deepcopy(self._current_target_pose)` khi gửi `NavigateToPose`. Nhờ đó pose gửi tới Nav2 giữ nguyên `z == 0.0` và quaternion gốc kể cả qua `PAUSE -> RESUME`.
- **Vòng đời hiển thị & subscriber kết nối muộn khi `IDLE`**:
  - `_clear_destination_visuals()` phát `DELETEALL` lên marker, `Path(poses=[])` lên `/voice/confirmed_path`, và sentinel `PoseStamped(header.frame_id='')` lên `/voice/confirmed_goal`; `publish_state()` bổ sung `confirmed_goal_valid` (`bool`) và `confirmed_goal_pose` (`dict | None`) trên `/voice/nav_state`. Gọi dọn sạch ngay trong `__init__` và ở mọi nhánh terminal/`strict` abort/`IDLE`.
- **Kiểm chứng thực tế**:
  - `colcon build --packages-select airport_robot_voice --symlink-install`: thành công (`1.11s`); YAML `voice_navigation.rviz` hợp lệ (`RVIZ_YAML_OK 13`).
  - `scripts/test_nlu.py`: **61/61** đạt.
  - `ROS_DOMAIN_ID=42 python3 -m pytest src/airport_robot_voice/test/ -v`: **126 passed in 62.80s** (bao gồm Test 9 cập nhật kiểm tra không đụng `/plan` & `/waypoints`, giữ `z == 0.0` + quaternion, và Test 12 kiểm tra subscriber kết nối muộn sau `CANCEL` thấy `IDLE`, `confirmed_goal_valid=False`, `frame_id=''`, `DELETEALL`, path rỗng; `test_ros_lifecycle.py` kiểm tra giữ `z == 0.0` + quaternion qua `PAUSE -> RESUME`). Log: [.agent/airport_robot_voice/evidence/toilet_rviz_pytest.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/toilet_rviz_pytest.log).
  - Thực nghiệm Gazebo + Nav2 (`ROS_DOMAIN_ID=44`, `places_sim.json`, AMCL+TF xác nhận tọa độ spawn):
    1. `case_t1` `(-1.0, 1.3, 1.57)`: `/plan` chỉ có `['planner_server', 'route_server']`, `/waypoints=[]`; `[CHỌN] toilet_1: 0.50m`, `[THỬ] toilet_2: 1.49m`; `ANNOUNCING` khóa `toilet_1` (`x=-1.0, y=1.8, z=0.0`, `qz=0.706825..., qw=0.707388...`, `confirmed_goal_valid=True`) trước khi phát TTS; kiểm chứng `PAUSE -> RESUME -> CANCEL -> LATE_SUBSCRIBER PASS`.
    2. `case_t2` `(0.2, 1.3, 1.57)`: `[THỬ] toilet_1: 1.99m`, `[CHỌN] toilet_2: 0.68m`; khóa `toilet_2` (`x=0.2, y=1.8, z=0.0`) nhất quán qua `ANNOUNCING` và `NAVIGATING`; `SELECT toilet_2 + CANCEL -> LATE_SUBSCRIBER PASS`.
    - Log: [.agent/airport_robot_voice/evidence/toilet_rviz_gazebo_verification.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/toilet_rviz_gazebo_verification.log).

### 6.9. Khắc phục hiển thị đồng thời hai đường xanh nước biển & xanh lá khi chọn Nhà vệ sinh (MPPI DiffDrive)
- **Nguyên nhân gốc rễ**:
  - Đường **xanh nước biển** (`0; 120; 255`) là `/plan` (`Nav2 Active Plan`, `nav_msgs/msg/Path`) do Nav2 `planner_server` publish liên tục khi `NavigateToPose` chạy.
  - Đường **xanh lá** (`0; 210; 110`) là `/voice/confirmed_path` (`Voice Confirmed Path`, `nav_msgs/msg/Path`, QoS `TransientLocal`) do `nav_command_node` publish từ `ComputePathToPose` của ứng viên thắng cuộc khi chọn nhóm `toilet`, và không bị xóa khi `NavigateToPose` bắt đầu (`NAVIGATING`), trong khi các điểm đến trực tiếp (`gate_a3`, `gate_a4`) có `confirmed_path=None`.
  - MPPI `DiffDrive` không phải nguyên nhân gây đường xanh lá (MPPI chỉ publish `/optimal_trajectory`, `/trajectories`, `/transformed_global_plan` đồng nhất cho mọi điểm đến; `/local_plan` trong `voice_navigation.rviz` có `Publisher count: 0` trên cấu hình MPPI).
- **Sửa lỗi đã thực hiện**:
  - [nav_command_node.py](file:///home/thuytran/ws/src/airport_robot_voice/airport_robot_voice/nav_command_node.py): thêm `_clear_preview_path()` và gọi ngay khi `NavigateToPose` được chấp nhận (`MissionState.NAVIGATING`, dòng ~1758) để xóa `/voice/confirmed_path` về rỗng ngay khi điều hướng chính thức bắt đầu; bổ sung `_clear_destination_visuals(clear_debug=True)` và reset target cho các nhánh `NO_CANDIDATES`, `PLANNER_UNAVAILABLE`, `NO_VALID_PATH`.
  - [voice_navigation.rviz](file:///home/thuytran/ws/src/airport_robot_voice/rviz/voice_navigation.rviz): đổi `Voice Confirmed Path` thành `Voice Preview Path (Debug)` và tắt mặc định (`Enabled: false`, `Value: false`); đổi tên `Local Plan` thành `MPPI Local Trajectory`.
  - [test_planner_resolution.py](file:///home/thuytran/ws/src/airport_robot_voice/test/test_planner_resolution.py): bổ sung Tests 13–15 kiểm chứng xóa preview path khi vào `NAVIGATING`, thống nhất giữa `gate_a3` và `toilet`, chuyển đổi `toilet <-> gate_a3`, và reset visualization sau `SUCCEEDED`, `ABORTED`, `NO_VALID_PATH`, `CANCEL` khi `RESOLVING`.
- **Kết quả kiểm thử**:
  - `scripts/test_nlu.py`: **61/61** đạt.
  - `ROS_DOMAIN_ID=42 python3 -m pytest src/airport_robot_voice/test/ -v`: **129 passed in 74.76s**. Log: [.agent/airport_robot_voice/evidence/rviz_double_path_pytest.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/rviz_double_path_pytest.log).
  - Gazebo + Nav2 (MPPI DiffDrive) trên `ROS_DOMAIN_ID=44` (`places_sim.json`): hoàn tất kiểm chứng ROS graph (`ros2 topic list -t`, `ros2 topic info -v`, `ros2 action info`, `ros2 node info /nav_command_node`) và toàn bộ **TC01–TC08** PASS (`TC01` A3, `TC02` A4, `TC03` toilet không còn preview path khi `NAVIGATING`, `TC04` hủy toilet, `TC05` toilet $\rightarrow$ A3, `TC06` A3 $\rightarrow$ toilet, `TC07` hủy trong `RESOLVING`, `TC08` `SUCCEEDED -> IDLE` + late subscriber sạch). Log: [.agent/airport_robot_voice/evidence/rviz_double_path_gazebo_tc01_08.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/rviz_double_path_gazebo_tc01_08.log).
