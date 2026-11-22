# BẢN ĐỀ XUẤT TÍCH HỢP GIAO DIỆN GỬI PHÂN HỆ AIRPORT_ROBOT_VOICE

**Gửi:** Kỹ sư / Agent phụ trách phân hệ `airport_robot_voice`  
**Từ:** Kỹ sư phụ trách phân hệ `person_tracking`  
**Dự án:** Robot dẫn đường hành khách sân bay (Airport Escort Robot)  
**Thời điểm lập:** 23/11/2026 — Phase 1 Design  
**Mục đích:** Đề xuất thống nhất hợp đồng giao tiếp giữa Phân hệ Nhận diện Hành khách (Tracking) và Phân hệ Giọng nói / Quản lý Điều hướng (Voice) để chuẩn bị tích hợp ở Phase 9.

---

## 1. Tôn chỉ Thiết kế và Phân định Quyền hạn

1. **Voice là Chủ sở hữu DUY NHẤT của Nav2 Action:**
   - Phân hệ Tracking **tuyệt đối không khởi tạo `ActionClient<NavigateToPose>`**, không tự ý gửi Goal và không tự ý gọi lệnh Cancel trên Nav2.
   - Toàn bộ việc nhận diện điểm đến bằng NLU, gửi Goal cho Nav2, theo dõi tiến trình và hủy Goal đều do `nav_command_node.py` của phân hệ Voice toàn quyền đảm nhiệm.
2. **Tracking là Cổng An toàn và Quản lý Trạng thái Khách hàng:**
   - Phân hệ Tracking phát hiện người đi kèm qua camera sau, nhận diện cử chỉ giơ tay để khóa khách hàng, sinh `session_id` cho phiên, và quản lý cổng an toàn vận tốc (`escort_velocity_guard`).
   - Tracking cần phối hợp với Voice để đảm bảo: **Robot chỉ nhận lệnh điểm đến và di chuyển khi ĐÃ KHÓA ĐƯỢC HÀNH KHÁCH ở phía sau**.

---

## 2. Các Thay đổi Đề xuất từ Phía Phân hệ Voice (Tại Phase 9)

### 2.1. Lắng nghe Trạng thái Khóa Khách hàng: `/tracking/session_state`
- **Topic:** `/tracking/session_state`
- **Kiểu bản tin:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE
- **Cấu trúc JSON do Tracking phát hành:**
  ```json
  {
    "schema_version": "1.0",
    "stamp": { "sec": 1732320000, "nanosec": 123456789 },
    "sequence_num": 42,
    "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "state": "LOCKED_WAIT_DESTINATION",
    "locked": true,
    "person_track_id": 3,
    "person_distance_m": 1.45,
    "confidence": 0.94,
    "reason_code": "HAND_RAISE_HELD_NEAREST"
  }
  ```
- **Hành vi đề xuất thêm vào `nav_command_node.py`:**
  - Tạo subscription lắng nghe topic `/tracking/session_state`.
  - Khi nhận intent điểm đến từ người dùng (ví dụ: *"Dẫn tôi tới Cổng 1"*):
    - Kiểm tra `locked == true` và `state == "LOCKED_WAIT_DESTINATION"`.
    - **Nếu ĐÃ KHÓA:** Ghi nhận `session_id` vào phiên hiện tại, chấp nhận điểm đến và gửi Goal sang Nav2 như bình thường.
    - **Nếu CHƯA KHÓA (`locked == false`):** Từ chối tiếp nhận và phát TTS phản hồi:
      > *"Xin quý khách vui lòng đứng phía sau và giơ một tay để robot nhận diện trước khi chọn điểm đến."*

---

### 2.2. Tiếp nhận Yêu cầu Hủy khẩn cấp từ Tracking: `/tracking/safety_request`
- **Topic:** `/tracking/safety_request`
- **Kiểu bản tin:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=10, Reliability=RELIABLE
- **Cấu trúc JSON do Tracking phát hành:**
  ```json
  {
    "schema_version": "1.0",
    "stamp": { "sec": 1732320005, "nanosec": 456789012 },
    "request_id": "8b0a1d63-3b1a-4f5e-9e7f-6d2c1e8a9b0c",
    "session_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "command": "ABORT",
    "reason_code": "TARGET_LOST_PROLONGED",
    "message": "Không thấy hành khách trong hơn 1.2 giây"
  }
  ```
- **Hành vi đề xuất thêm vào `nav_command_node.py`:**
  - Khi nhận `command == "ABORT"` có `session_id` khớp với phiên đang dẫn đường:
    1. Gọi ngay `cancel_goal_async()` để hủy Goal đang chạy trên Nav2.
    2. Chuyển `mission_state` sang trạng thái hủy (`CANCELING` -> `IDLE`).
    3. Phát âm thanh TTS thông báo: *"Đã mất dấu hành khách, robot dừng hành trình dẫn đường."*
    4. (Lưu ý: Phía Tracking đã chủ động ép vận tốc motor về 0 ngay lập tức, Voice chỉ cần thực hiện dọn dẹp Goal trên Nav2 và tương tác âm thanh).

---

### 2.3. Phát hành Trạng thái Hoàn tất để Tracking Xóa Khóa: `/voice/nav_status`
- **Topic:** `/voice/nav_status` (hoặc bổ sung các trường vào `/voice/nav_state` hiện có)
- **Kiểu bản tin:** `std_msgs/msg/String` (JSON format)
- **QoS:** Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE
- **Cấu trúc JSON đề xuất:**
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
    "message": "Đã đến Cổng 1 thành công."
  }
  ```
- **Ý nghĩa:** Khi Voice phát bản tin có `status == "SUCCEEDED"` và `is_terminal == true` đúng với `session_id`, Tracking sẽ xác nhận đến đích, xóa ID khóa của khách, dọn dẹp phiên và đưa hệ thống về trạng thái `IDLE` sẵn sàng đón lượt khách mới.

---

## 3. Lộ trình Triển khai Phối hợp

- **Giai đoạn hiện tại (Phase 1–8):** Phân hệ Tracking sẽ sử dụng node giả lập `mock_voice_owner` bên trong package tracking để tự động hóa toàn bộ kiểm thử nội bộ mà **không yêu cầu Voice phải sửa đổi code ngay**.
- **Giai đoạn tích hợp chính thức (Phase 9):** Sau khi phân hệ Tracking hoàn thành các bài test độc lập, bản đề xuất này sẽ được chủ dự án phê duyệt để áp dụng chính thức vào mã nguồn của Voice.

*Rất mong nhận được phản hồi và thống nhất từ phía phân hệ Voice!*
