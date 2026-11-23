# BẢN YÊU CẦU TÍCH HỢP GIAO DIỆN LIÊN PHÂN HỆ: TRACKING <-> VOICE (PHASE 6 SPECIFICATION)

**Gửi:** Kỹ sư / Agent phụ trách phân hệ `airport_robot_voice`  
**Từ:** Kỹ sư phụ trách phân hệ `person_tracking`  
**Dự án:** Robot dẫn đường hành khách sân bay (Airport Escort Robot) — ROS 2 Jazzy  
**Thời điểm cập nhật:** 23/11/2026 — Phase 6 Offline Implementation  
**Mục đích:** Chuẩn hóa toàn bộ hợp đồng giao tiếp, quan hệ định danh phiên/nhiệm vụ, cơ chế handshake xác nhận hai pha (ACK vs Terminal), và quy tắc kiểm soát an toàn trước khi tích hợp trực tiếp ở Phase 9.

---

## 1. Tôn chỉ Thiết kế và Phân quyền Tuyệt đối

1. **Voice là Chủ sở hữu DUY NHẤT của Nav2 Action Server:**
   - Phân hệ `person_tracking` **tuyệt đối không khởi tạo `ActionClient<NavigateToPose>`**, không gửi Goal và không gọi hàm `cancel_goal_async` trực tiếp lên Nav2.
   - Toàn bộ NLU điểm đến, khởi tạo Goal, vòng đời NavigateToPose, chọn đường ComputePathToPose, và tương tác âm thanh TTS do `nav_command_node.py` của phân hệ Voice toàn quyền sở hữu.
2. **Tracking là Cơ quan Duy nhất Giữ Lock và Giám sát An toàn Bám theo (Single Lock Authority):**
   - Phân hệ `person_tracking` phát hiện khách qua camera sau, nhận diện cử chỉ giơ tay để khóa khách, sinh `session_id`, giám sát độ mới quan sát (`obs_fresh`), và quản lý FSM bám theo.
   - Tracking ép dừng xe khi khách mất dấu quá thời hạn cho phép và gửi yêu cầu hủy an toàn sang Voice.
   - **Robot chỉ được phép khởi hành khi đã khóa đúng khách, quan sát còn mới và phiên trước đã kết thúc trọn vẹn.**

---

## 2. Chuẩn hóa Quan hệ giữa 5 Định danh (Correlation & Identity Architecture)

Để đảm bảo hai phân hệ hoạt động song song không bao giờ xung đột trạng thái hoặc bị kích hoạt bởi bản tin cũ:

| Tên Định danh | Kiểu dữ liệu | Phân hệ khởi tạo | Ý nghĩa & Vòng đời |
| :--- | :--- | :--- | :--- |
| **`session_id`** | UUIDv4 (String) | `escort_tracking_manager` | Định danh duy nhất cho một phiên phục vụ một hành khách cụ thể từ lúc giơ tay khóa thành công đến khi tới đích hoặc bị hủy. |
| **`mission_id`** | UUIDv4 (String) | `nav_command_node` | Định danh phiên nhiệm vụ dẫn đường tổng thể do Voice khởi tạo khi nhận diện xong điểm đến. |
| **`goal_attempt_id`**| UUIDv4 (String) | `nav_command_node` | Định danh mỗi lần thử gửi Goal cụ thể tới Nav2 Action Server. |
| **`nav2_uuid`** | UUID 16 bytes | Nav2 Action Client | ID kỹ thuật nội bộ của ROS 2 Action Goal handle. |
| **`request_id`** | UUIDv4 (String) | `escort_tracking_manager` | Định danh duy nhất cho từng yêu cầu an toàn (ví dụ lệnh hủy `ABORT`) để phục vụ bắt tay ACK và retry. |

**Quy tắc tương quan:**
- Một `session_id` của khách có thể chứa một `mission_id` của Voice.
- Một `mission_id` có thể trải qua một hoặc nhiều `goal_attempt_id` (nếu có cơ chế thử lại hoặc đổi đích).
- Mỗi lệnh `safety_request` (ABORT) sinh một `request_id` độc lập để Voice ACK chính xác.

---

## 3. Chi tiết Các Topic và Hợp đồng Giao tiếp

### 3.1. Giao diện Cấp phép: `/tracking/session_state`
- **Chủ quản:** `escort_tracking_manager`
- **Bên tiêu thụ:** `airport_robot_voice/nav_command_node`
- **Kiểu:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE
- **Tần số:** 5 Hz Heartbeat + Event-driven khi chuyển trạng thái.
- **Cấu trúc JSON Payload:**
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
    "depth_proven": true,
    "confidence": 0.94,
    "reason_code": "HAND_RAISE_HELD_NEAREST"
  }
  ```
- **Quy tắc Kiểm tra tại Voice (Gating Rule):**
  Khi người dùng phát âm lệnh điểm đến, Voice **chỉ được chấp nhận và gửi Goal sang Nav2** khi:
  1. `locked == true`
  2. `state == "LOCKED_WAIT_DESTINATION"`
  3. `obs_fresh == true` (nếu `obs_fresh == false` hoặc `reason_code == "PASSENGER_TEMPORARILY_UNSEEN"`: từ chối vì mất dấu trước xuất phát)
  4. Tuổi bản tin heartbeat $\le 1.0$ giây và Voice không còn goal cũ nào ở trạng thái chưa xác nhận terminal (`has_unresolved_goals == false`).

---

### 3.2. Giao diện Yêu cầu An toàn: `/tracking/safety_request`
- **Chủ quản:** `escort_tracking_manager`
- **Bên tiêu thụ:** `airport_robot_voice/nav_command_node`
- **Kiểu:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=10, Reliability=RELIABLE
- **Cơ chế Retry:** Phát ngay khi vào `ABORTING`. Tự động gửi lại tối đa 3 lần sau mỗi 200 ms (tổng 4 lần) nếu chưa nhận được ACK từ Voice.
- **Cấu trúc JSON Payload:**
  ```json
  {
    "schema_version": "1.0",
    "stamp": { "sec": 1732320005, "nanosec": 456789012 },
    "request_id": "8b0a1d63-3b1a-4f5e-9e7f-6d2c1e8a9b0c",
    "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "command": "ABORT",
    "reason_code": "VISIBILITY_TIMEOUT_EXCEEDED",
    "message": "Escort abort triggered: VISIBILITY_TIMEOUT_ABORT"
  }
  ```
- **Hành vi xử lý bắt buộc tại Voice:**
  1. Khi nhận bản tin, so sánh `session_id`. Nếu khớp với phiên hiện tại:
     - Gửi ngay bản tin **ACK** trên `/voice/nav_status` với `request_id` tương ứng và `nav_event: "CANCEL_ACKNOWLEDGED"`.
     - Lập tức gọi `cancel_goal_async` trên ActionClient của Nav2.
     - Phát câu thông báo TTS thông báo hành khách: *"Đã mất dấu hành khách, robot dừng hành trình dẫn đường."*

---

### 3.3. Giao diện Trạng thái Nav2 từ Voice: `/voice/nav_status`
- **Chủ quản:** `airport_robot_voice/nav_command_node`
- **Bên tiêu thụ:** `escort_tracking_manager`
- **Kiểu:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE

#### Quy định Tách Biệt Hai Pha (Handshake vs Terminal Result):
Phân hệ Tracking yêu cầu **tách riêng rẽ bản tin ACK và bản tin Kết quả Kết thúc (Terminal)**:

1. **Pha 1: ACK Tiếp nhận Yêu cầu Hủy (`is_terminal: false`)**
   ```json
   {
     "schema_version": "1.0",
     "stamp": { "sec": 1732320005, "nanosec": 470000000 },
     "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
     "request_id": "8b0a1d63-3b1a-4f5e-9e7f-6d2c1e8a9b0c",
     "status": "CANCELING",
     "nav_event": "CANCEL_ACKNOWLEDGED",
     "is_terminal": false,
     "message": "Voice đã tiếp nhận lệnh ABORT và đang yêu cầu Nav2 hủy Goal."
   }
   ```
   *Tác dụng:* Tracking ngừng gửi retry cho `request_id` này nhưng tiếp tục giữ trạng thái `ABORTING` (chưa cho phép phiên mới).

2. **Pha 2: Kết quả Kết thúc Goal (`is_terminal: true`)**
   ```json
   {
     "schema_version": "1.0",
     "stamp": { "sec": 1732320005, "nanosec": 980000000 },
     "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
     "mission_id": "a1b2c3d4-e5f6-7a8b-9c0d-1e2f3a4b5c6d",
     "goal_attempt_id": "11223344-5566-7788-99aa-bbccddeeff00",
     "destination_id": "gate_1",
     "status": "CANCELED",
     "nav_event": "GOAL_COMPLETED",
     "is_terminal": true,
     "message": "Nav2 Action Server đã xác nhận Goal hủy hoàn tất."
   }
   ```
   *Tác dụng:* Tracking xóa cờ `unconfirmed_terminal`, dọn dẹp phiên một lần duy nhất về `IDLE` sẵn sàng nhận khách mới.

3. **Pha Dẫn đường Thành công Đến Đích (`is_terminal: true`, `status: "SUCCEEDED"`)**
   ```json
   {
     "schema_version": "1.0",
     "stamp": { "sec": 1732320060, "nanosec": 123456789 },
     "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
     "status": "SUCCEEDED",
     "nav_event": "GOAL_COMPLETED",
     "is_terminal": true,
     "message": "Đã dẫn hành khách đến Cổng 1 an toàn."
   }
   ```
   *Tác dụng:* Tracking chuyển trạng thái `ARRIVED` -> dọn dẹp phiên về `IDLE`.

---

## 4. Xử lý Các Trường hợp Biên (Edge Cases)

1. **Bản tin đến muộn từ phiên cũ (Stale Goal Status):**
   Nếu Voice phát bản tin mang `session_id` cũ hoặc `is_terminal == true` nhưng không trùng `session_id` hiện tại của Tracking, Tracking **hoàn toàn bỏ qua**, không cho phép kết quả cũ làm xóa nhầm phiên đang chạy.
2. **Khởi hành Nghiêm ngặt (Strict Start Contract):**
   Bản tin `NAVIGATION_STARTED` từ Voice bắt buộc phải mang đầy đủ `session_id`, `mission_id`, và `goal_attempt_id`. Bỏ hoàn toàn fallback tự điền `session_id`. Nếu thiếu định danh hoặc `session_id` không khớp, Tracking từ chối khởi hành.
3. **Bắt tay ACK & Hạn chót Xác nhận Terminal (Handshake & Terminal Deadline):**
   - Khi nhận `safety_request` (ABORT), Voice gửi ACK `CANCEL_ACKNOWLEDGED` với cùng `request_id`, `session_id` và `goal_attempt_id`.
   - Tracking giữ nguyên `request_id` qua toàn bộ các lần retry (tối đa 3 lần sau mỗi 200 ms).
   - ACK **chỉ có tác dụng dừng gửi lại retry**, robot vẫn duy trì dừng vận tốc 0 (`ABORTING`).
   - Sau khi nhận ACK, Tracking kích hoạt bộ đếm hạn chót xác nhận Terminal (`terminal_timeout_sec = 3.0s`).
   - Nếu hết 3 lần retry không có ACK, HOẶC sau ACK quá 3.0s mà không nhận được bản tin Terminal: Tracking chuyển sang `FAULT`, đánh dấu `unconfirmed_terminal = true`, chốt dừng 0 và **chặn tuyệt đối không cho bất kỳ khách hàng nào khóa phiên mới**.
4. **Kiểm tra Tính hợp lệ của Kết quả Terminal:**
   - Trạng thái `status: "UNKNOWN"` bị từ chối tuyệt đối (không coi là kết thúc hợp lệ).
   - Chỉ chấp nhận các trạng thái chuẩn: `SUCCEEDED`, `CANCELED`, `ABORTED`.
   - `mission_id` và `goal_attempt_id` trong bản tin Terminal phải khớp với Goal đang được dẫn đường.
5. **Mất dấu tạm thời vs OUT_OF_FOV được xác nhận:**
   - Mất khách $< 0.25$s: Tracking giữ `NAVIGATING` (`LOCKED_TARGET_GRACE_PERIOD`), không gây xáo trộn.
   - Mất khách $> 0.40$s: Tracking chuyển `WAITING_FOR_PASSENGER` (vận tốc ép về 0).
   - Nếu khách quay lại trước 1.20s: Tracking tự phục hồi về `NAVIGATING` mà không cần hủy Goal.
   - Quá 1.20s: Lập tức `ABORT` sang Voice. Sau khi đã `ABORT`, việc khách quay lại **tuyệt đối không được mở khóa lại**.
   - Nếu khách ở gần mép ảnh (`near_edge == true`) rồi biến mất ở frame tiếp theo: Xác nhận `OUT_OF_FOV` $\implies$ lập tức chuyển sang `ABORTING` (`reason_code = "OUT_OF_FOV"`), không chờ qua grace period.

---

## 5. Danh mục Việc Cần Triển Khai từ Phía Phân hệ Voice (Phase 9 Checklist)

1. [ ] Thêm subscription lắng nghe `/tracking/session_state` trong `nav_command_node.py`.
2. [ ] Thêm logic kiểm tra gating khởi hành: `locked == true && state == "LOCKED_WAIT_DESTINATION" && obs_fresh == true`.
3. [ ] Thêm subscription lắng nghe `/tracking/safety_request` trong `nav_command_node.py`.
4. [ ] Triển khai hàm phản hồi ACK hai pha trên topic `/voice/nav_status`:
   - Gửi ngay ACK `CANCEL_ACKNOWLEDGED` kèm `request_id`.
   - Gửi `is_terminal: true` kèm `session_id` khi Nav2 xác nhận goal đã kết thúc.
5. [ ] Đảm bảo gán trường `session_id` từ `session_state` vào các bản tin trạng thái của Voice.
