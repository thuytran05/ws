# PERSON TRACKING — ĐẶC TẢ HỢP ĐỒNG GIAO DIỆN CHÍNH THỨC (INTERFACES SPECIFICATION)

**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.0 — 23/11/2026 (Phase 1)  
**Trạng thái:** BẢN HỢP ĐỒNG ĐỀ XUẤT (Formal Proposal for Multi-Agent Integration)  
**Áp dụng:** Giao tiếp nội bộ Tracking, giao tiếp liên phân hệ với Voice (`airport_robot_voice`), và chuỗi vận tốc Nav2.

---

## 1. Nguyên tắc Phân quyền và Định danh (Correlation & Identity Principles)

Để đảm bảo hai agent hoạt động song song không bao giờ xung đột trạng thái hoặc bị kích hoạt bởi các bản tin cũ:

1. **Phân biệt rạch ròi 4 loại Định danh (Identifiers):**
   - `session_id` (UUIDv4, do `escort_tracking_manager` sinh): Định danh duy nhất cho một phiên phục vụ một hành khách cụ thể từ lúc giơ tay khóa thành công đến khi tới đích hoặc bị hủy.
   - `person_track_id` (Integer, do `passenger_perception` gán): ID nhận dạng quang học của thuật toán ByteTrack (ví dụ `3`). Tuyệt đối không dùng `target_id` chung chung để tránh nhầm với điểm đến!
   - `destination_id` / `destination_name` (String, do `airport_robot_voice` phân tích NLU): Định danh vị trí đích đến trong sân bay (ví dụ `"gate_1"`, `"restroom_2"`).
   - `mission_id` & `goal_attempt_id` (UUIDv4, do `airport_robot_voice` quản lý): Định danh phiên nhiệm vụ tổng thể và lần thử gửi goal cụ thể sang Nav2 Action Server.
   - `nav2_action_goal_id` (UUID 16 bytes của ROS 2 Action Goal): ID kỹ thuật do ActionClient của Nav2 sinh ra.
2. **Miền Thời gian (Clock Domain):**
   - Sử dụng chuẩn `builtin_interfaces/msg/Time` (sec + nanosec) lấy từ ROS Clock (`node->get_clock()->now()`).
   - Trong môi trường mô phỏng Gazebo (`use_sim_time = true`), toàn bộ các node đồng bộ theo topic `/clock`.
   - Các kiểm tra thời gian thực (watchdog timer, stream timeout) sử dụng thời gian đơn điệu `time.monotonic()` độc lập với việc dừng/nhảy của `/clock`.
3. **Phương thức Truyền tải (Transport Format):**
   - Để tương thích 100% với mã nguồn hiện hữu của `airport_robot_voice` (đang dùng `std_msgs/msg/String` bọc JSON serialize), các giao diện liên phân hệ bước đầu được triển khai qua **JSON String có schema chặt chẽ**.
   - Nếu bản tin bị lỗi cú pháp JSON hoặc thiếu trường bắt buộc, phía nhận phải ghi log cảnh báo và giữ nguyên trạng thái an toàn hiện tại, **tuyệt đối không crash node**.

---

## 2. Chi tiết Đặc tả Các Giao diện (Interface Specifications)

### 2.1. Giao diện Cấp phép: `/tracking/session_state`
- **Chủ quản (Producer/Owner):** `escort_tracking_manager`
- **Bên tiêu thụ (Consumer):** `airport_robot_voice/nav_command_node`
- **Kiểu ROS 2:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE
- **Tần số:** Phát khi có sự kiện chuyển trạng thái (Event-driven) và phát định kỳ nhịp tim 5 Hz (Heartbeat). Hạn sử dụng bản tin (`freshness_ttl`): 1.0 giây.
- **JSON Schema:**
  ```json
  {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "TrackingSessionState",
    "type": "object",
    "properties": {
      "schema_version": { "type": "string", "enum": ["1.0"] },
      "stamp": {
        "type": "object",
        "description": "Thời điểm phát nhịp tim (heartbeat) hiện tại theo ROS Clock",
        "properties": {
          "sec": { "type": "integer" },
          "nanosec": { "type": "integer" }
        },
        "required": ["sec", "nanosec"]
      },
      "last_observation_stamp": {
        "type": "object",
        "description": "Thời điểm chụp quang học thực tế của frame quan sát gần nhất được xử lý",
        "properties": {
          "sec": { "type": "integer" },
          "nanosec": { "type": "integer" }
        },
        "required": ["sec", "nanosec"]
      },
      "obs_fresh": {
        "type": "boolean",
        "description": "True nếu quan sát đến trong vòng 0.4s; False nếu mất frame hoặc quá hạn"
      },
      "sequence_num": { "type": "integer", "minimum": 0 },
      "session_id": { "type": "string", "format": "uuid" },
      "state": {
        "type": "string",
        "enum": ["IDLE", "LOCKED_WAIT_DESTINATION", "NAVIGATING", "WAITING_FOR_PASSENGER", "ABORTING", "ARRIVED", "FAULT"]
      },
      "locked": { "type": "boolean" },
      "person_track_id": { "type": "integer" },
      "person_distance_m": { "type": "number", "minimum": 0.0 },
      "depth_proven": { "type": "boolean" },
      "confidence": { "type": "number", "minimum": 0.0, "maximum": 1.0 },
      "reason_code": { "type": "string" }
    },
    "required": ["schema_version", "stamp", "sequence_num", "session_id", "state", "locked", "person_track_id", "confidence"]
  }
  ```
- **Ví dụ Payload:**
  ```json
  {
    "schema_version": "1.0",
    "stamp": { "sec": 1732320000, "nanosec": 500000000 },
    "last_observation_stamp": { "sec": 1732320000, "nanosec": 420000000 },
    "obs_fresh": true,
    "sequence_num": 42,
    "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "state": "LOCKED_WAIT_DESTINATION",
    "locked": true,
    "person_track_id": 3,
    "person_distance_m": 1.45,
    "depth_proven": false,
    "confidence": 0.94,
    "reason_code": "HAND_RAISE_HELD_NEAREST"
  }
  ```
- **Quy tắc Kiểm tra tại Voice (Gating Rule):**
  - Khi người dùng nói lệnh điểm đến: Voice kiểm tra bản tin `/tracking/session_state` mới nhất (tuổi bản tin heartbeat $\le 1.0s$).
  - Chỉ khi `locked == true` VÀ `state == "LOCKED_WAIT_DESTINATION"` VÀ `obs_fresh == true`: Voice mới chấp nhận điểm đến và gửi Goal sang Nav2.
  - Nếu `obs_fresh == false` (hoặc `reason_code == "OBSERVATION_STALE"`): Voice từ chối khởi hành vì mục tiêu bị mất dấu trước khi xuất phát.
  - Nếu `locked == false`: Voice từ chối bằng giọng nói (ví dụ: *"Xin quý khách vui lòng giơ tay để tôi nhận diện trước khi chọn điểm đến"*).

---

### 2.2. Giao diện Yêu cầu An toàn: `/tracking/safety_request`
- **Chủ quản (Producer/Owner):** `escort_tracking_manager`
- **Bên tiêu thụ (Consumer):** `airport_robot_voice/nav_command_node`
- **Kiểu ROS 2:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=10, Reliability=RELIABLE
- **Cơ chế:** Gửi ngay khi có sự kiện bất thường. Tự động gửi lại (retry) tối đa 3 lần sau mỗi 200 ms nếu chưa nhận được ACK từ Voice.
- **JSON Schema:**
  ```json
  {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "TrackingSafetyRequest",
    "type": "object",
    "properties": {
      "schema_version": { "type": "string", "enum": ["1.0"] },
      "stamp": {
        "type": "object",
        "properties": {
          "sec": { "type": "integer" },
          "nanosec": { "type": "integer" }
        },
        "required": ["sec", "nanosec"]
      },
      "request_id": { "type": "string", "format": "uuid" },
      "session_id": { "type": "string", "format": "uuid" },
      "command": { "type": "string", "enum": ["STOP", "WAIT", "ABORT", "RESUME"] },
      "reason_code": {
        "type": "string",
        "enum": ["TARGET_LOST_PROLONGED", "OUT_OF_FOV", "AMBIGUOUS_ID", "CAMERA_FREEZE", "EMERGENCY_STOP", "RESUME_CONFIRMED"]
      },
      "message": { "type": "string" }
    },
    "required": ["schema_version", "stamp", "request_id", "session_id", "command", "reason_code"]
  }
  ```
- **Hành vi Xử lý:**
  - `command == "ABORT"`: Voice lập tức gọi `cancel_goal_async` trên ActionClient của Nav2, chuyển trạng thái Voice sang CANCELING, và phát TTS giải thích lý do mất khách.
  - `command == "WAIT"`: Voice tạm dừng tiến trình (nếu hỗ trợ pause) hoặc giữ trạng thái chờ.
  - **Ràng buộc an toàn:** Phân hệ Tracking **KHÔNG CHỜ PHẢN HỒI** từ Voice để dừng xe. Ngay khi phát lệnh `ABORT` hoặc `WAIT`, `escort_velocity_guard` đã lập tức ép vận tốc về 0.

---

### 2.3. Giao diện Phản hồi Trạng thái Nav2 từ Voice: `/voice/nav_status`
- **Chủ quản (Producer/Owner):** `airport_robot_voice/nav_command_node`
- **Bên tiêu thụ (Consumer):** `escort_tracking_manager`
- **Kiểu ROS 2:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE
- **Cấu trúc Payload đề xuất:**
  ```json
  {
    "schema_version": "1.0",
    "stamp": { "sec": 1732320060, "nanosec": 987654321 },
    "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "mission_id": "a1b2c3d4-e5f6-7a8b-9c0d-1e2f3a4b5c6d",
    "goal_attempt_id": "11223344-5566-7788-99aa-bbccddeeff00",
    "destination_id": "gate_1",
    "status": "SUCCEEDED",
    "nav_event": "GOAL_COMPLETED",
    "is_terminal": true,
    "message": "Đã dẫn hành khách đến Cổng 1 an toàn."
  }
  ```
- **Xử lý Sự kiện Đặc biệt (Corner Cases):**
  1. **Kết quả đến muộn (Late arrival of old goal):** Nếu bản tin mang `session_id` cũ hoặc `is_terminal == true` nhưng không khớp phiên hiện tại, Tracking **hoàn toàn bỏ qua**, tránh việc một kết quả thành công cũ làm mở khóa hoặc xóa phiên mới.
  2. **Goal chưa xác nhận kết thúc (Unconfirmed Terminal):** Khi Tracking gửi yêu cầu `ABORT`, nếu Voice chỉ mới gửi ACK chấp nhận yêu cầu hủy mà chưa có xác nhận terminal (`is_terminal == true`) từ Nav2 Action Server:
     - Khách mới không được phép khóa!
     - Robot tuyệt đối không được cấp quyền di chuyển lại cho đến khi Nav2 hoàn toàn xác nhận Goal cũ đã hủy xong.

---

### 2.4. Giao diện Nội bộ: `/tracking/observation`
- **Chủ quản (Producer):** `person_detector` (`PersonDetectorNode`)
- **Bên tiêu thụ (Consumer):** `escort_tracking_manager` (Phase 4 & 6)
- **Kiểu ROS 2:** `std_msgs/msg/String` (JSON format với schema chuẩn)
- **QoS:** Depth=10, Reliability=RELIABLE
- **Tần số:** 15 Hz (đồng bộ theo camera đầu vào)
- **JSON Schema Chính Thức (Schema Version 1.0):**
  ```json
  {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "TrackingObservation",
    "type": "object",
    "properties": {
      "schema_version": { "type": "string", "enum": ["1.0"] },
      "stamp": {
        "type": "object",
        "properties": {
          "sec": { "type": "integer" },
          "nanosec": { "type": "integer" }
        },
        "required": ["sec", "nanosec"]
      },
      "frame_id": { "type": "string" },
      "receive_monotonic_time": { "type": "number" },
      "source_mode": { "type": "string", "enum": ["live_astra", "video_replay", "bag_replay", "synthetic_fixture"] },
      "source_session_id": { "type": "string" },
      "data_quality": {
        "type": "string",
        "enum": ["VALID_RGBD", "RGB_ONLY", "UNVERIFIED_ALIGNMENT", "DEGRADED_NO_INFO", "STALE", "DIMENSION_MISMATCH", "FRAME_ID_MISMATCH", "INVALID_DEPTH"]
      },
      "processing_time_ms": { "type": "number", "minimum": 0.0 },
      "device": { "type": "string" },
      "active_track_ids": { "type": "array", "items": { "type": "integer" } },
      "lost_track_ids": { "type": "array", "items": { "type": "integer" } },
      "candidates": {
        "type": "array",
        "items": {
          "type": "object",
          "properties": {
            "track_id": { "type": ["integer", "null"] },
            "is_confirmed": { "type": "boolean" },
            "is_ambiguous": { "type": "boolean" },
            "confidence": { "type": "number", "minimum": 0.0, "maximum": 1.0 },
            "bbox_xyxy": { "type": "array", "items": { "type": "number" }, "minItems": 4, "maxItems": 4 },
            "torso_depth_m": { "type": ["number", "null"], "minimum": 0.0 },
            "depth_valid": { "type": "boolean" },
            "center_offset_x_norm": { "type": "number", "minimum": -1.0, "maximum": 1.0 },
            "near_edge": { "type": "boolean" },
            "keypoints": { "type": "array", "items": { "type": "array", "minItems": 3, "maxItems": 3 } },
            "bearing_optical_rad": { "type": ["number", "null"], "description": "Góc lệch phương vị quang học theta_opt = atan2(u - cx, fx)" },
            "bearing_base_rad": { "type": ["number", "null"], "description": "Góc lệch phương vị hệ robot base_link [-pi, pi]" },
            "position_base": { "type": ["array", "null"], "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "[X, Y, Z] trong base_link (meters)" }
          },
          "required": ["is_confirmed", "is_ambiguous", "confidence", "bbox_xyxy", "depth_valid", "center_offset_x_norm", "near_edge"]
        }
      }
    },
    "required": ["schema_version", "stamp", "frame_id", "receive_monotonic_time", "source_mode", "data_quality", "active_track_ids", "lost_track_ids", "candidates"]
  }
  ```
- **Ý nghĩa & Ràng buộc Kỹ thuật của Các Trường:**
  1. `stamp`: Thời điểm camera chụp ảnh quang học (lấy từ Header của ảnh RGB).
  2. `receive_monotonic_time`: Thời điểm node nhận ảnh đo bằng clock monotonic máy chủ; hoàn toàn miễn nhiễm với hiện tượng pause/nhảy của `/clock`.
  3. `source_session_id`: UUID của phiên nguồn hiện tại; khi đổi nguồn hoặc video loop sang vòng lặp mới, phiên được cấp mới để cô lập triệt để dữ liệu cũ.
  4. `track_id` & `is_confirmed`: ID do ByteTrack duy trì liên tục qua các frame. Chỉ có giá trị trong cùng một `source_session_id`. `track_id = null` hoặc `is_confirmed = false` biểu thị người mới xuất hiện chưa đủ độ tin cậy.
  5. `lost_track_ids`: Danh sách các track ID từng xuất hiện trong frame trước nhưng biến mất ở frame hiện tại; là tín hiệu cơ sở để kích hoạt bộ đếm grace period (0.25s) và visibility timeout (0.4s/1.2s) của FSM.
  6. `is_ambiguous`: Đánh dấu `true` khi có từ 2 bounding box giao nhau với $IoU > 0.35$ (cắt ngang, che khuất một phần). Khi cờ này bật, FSM bị cấm khóa khách mới.
  7. `depth_valid`: **Quy tắc an toàn tối cao về đo khoảng cách**:
     - Chỉ được mang giá trị `true` khi `data_quality == VALID_RGBD` VÀ các pixel depth vùng thân hội tụ trong dải an toàn $[0.4m, 5.0m]$ với độ phân tán $MAD \le 0.25m$.
     - Khi `data_quality == UNVERIFIED_ALIGNMENT` (camera Astra thật chưa kiểm chứng căn chỉnh), trường `depth_valid` **BẮT BUỘC PHẢI LÀ `false`**. Trường `torso_depth_m` có thể mang giá trị thô để chẩn đoán nhưng thuật toán chọn người gần nhất ở Phase 4 tuyệt đối không được sử dụng.
  8. **Phân định phạm vi:** Các logic ngữ nghĩa cử chỉ (`hand_raised`, `raise_duration_sec`), thuật toán chọn người gần nhất (`nearest_candidate_selection`), và quản lý phiên phục vụ (`session_id`) được bàn giao cho **Phase 4** và **Phase 6** thực thi.

---

### 2.5. Giao diện Chuỗi Vận tốc: `cmd_vel_smoothed` -> `cmd_vel_guarded` -> `cmd_vel`
- **Kiểu bản tin:** `geometry_msgs/msg/Twist`
- **Luồng dữ liệu:**
  1. `cmd_vel_smoothed`: Do Nav2 `velocity_smoother` xuất ở 20 Hz.
  2. `cmd_vel_guarded`: Do `escort_velocity_guard` xuất ở 20 Hz sang Collision Monitor.
  3. `cmd_vel`: Do `collision_monitor` xuất ở 20 Hz tới Base Motor Controller / Gazebo Bridge.
- **Ràng buộc an toàn:** Collision Monitor luôn là khâu cuối cùng tiếp nhận lệnh để đảm bảo laser scan có quyền phủ quyết va chạm tối cao.
