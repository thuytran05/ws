# Hợp đồng Giao diện ROS 2 cho Thin Web Client (`airport_robot_voice` + `person_tracking`)

Tài liệu này mô tả hợp đồng chính thức giữa backend ROS 2 (trong `ws`) và một **thin web client** (hoặc Web Gateway chuyển tiếp sang ROS 2). Toàn bộ quyết định về chuyến đi, điểm đến, khóa hành khách, điều hướng và an toàn thuộc về các node trong `ws`. Khi web client hoặc web gateway tắt/ngắt kết nối, robot vẫn tiếp tục hoặc tự hủy chuyến an toàn theo thiết kế.

---

## 1. Nguyên tắc kiến trúc & phân tách quyền sở hữu

| Thành phần | Quyền sở hữu | Ghi chú |
|---|---|---|
| **Nav2 `NavigateToPose` & `ComputePathToPose`** | `nav_command_node` (`airport_robot_voice`) | UI **không bao giờ** gọi action Nav2 trực tiếp. |
| **Danh mục điểm đến (`places_file`)** | `nav_command_node` & `intent_node` | Nguồn sự thật duy nhất từ `places_file` đang nạp; UI **chỉ đọc**, không tự tạo/sửa file catalog riêng. |
| **NLU tiếng Việt** | `intent_node` (`airport_robot_voice`) | Xử lý câu tiếng Việt qua `NluEngine` từ `/voice/text`. |
| **Khóa hành khách & phiên phục vụ** | `tracking_manager` (`person_tracking`) | Phát `/tracking/session_state`; **không** đưa web account/user ID vào ID hành khách. |
| **An toàn chuyển động** | `velocity_guard` → `velocity_watchdog` → `collision_monitor` | Chuỗi `/cmd_vel_smoothed` → `/tracking/cmd_vel_guarded` → `/tracking/cmd_vel_watchdog` → `/cmd_vel`. |

---

## 2. Phân biệt các định danh (Identifiers)

| Trường | Nguồn cấp | Ý nghĩa |
|---|---|---|
| `client_request_id` | Web Client / UI | Định danh duy nhất do UI tạo cho mỗi yêu cầu (vd: `"ui-req-1732710001-1"`). Dùng để đối soát phản hồi & chống gửi trùng (idempotency), kể cả khi yêu cầu bị từ chối trước khi tạo `mission_id`. |
| `request_id` | Nội bộ Voice / Safety | Khi đi qua `/voice/ui_request` (hoặc `/voice/text` có `client_request_id`), mặc định gán bằng `client_request_id`. Với lệnh hủy khẩn từ Tracking (`/tracking/safety_request`), `request_id` là mã abort riêng của `tracking_manager` (lưu thành `abort_request_id`, **không** ghi đè `client_request_id`). |
| `session_id` | `tracking_manager` | Định danh phiên khóa hành khách hiện tại (vd: `"sess-1732710000-1"`). |
| `mission_id` | `nav_command_node` | Định danh chuyến đi do Voice tạo khi bắt đầu nhiệm vụ (vd: `"mis_1732710005000_1"`). |
| `goal_attempt_id` | `nav_command_node` | Định danh mỗi lần gửi goal tới Nav2 (vd: `"goal_1732710006000_1"`). |
| `nav2_uuid` | Nav2 Action Server | UUID của goal trong Nav2 (chuỗi hex). |

---

## 3. Bảng tổng hợp Topic & QoS

Tất cả các topic đều dùng `std_msgs/msg/String` với payload JSON UTF-8.

| Topic | Chiều | QoS (Durability / Reliability / Depth) | Mục đích |
|---|---|---|---|
| `/voice/places_catalog` | Backend → UI | `TRANSIENT_LOCAL` / `RELIABLE` / `1` | Danh mục điểm đến & nhóm điểm **chỉ đọc** sinh trực tiếp từ `places_file` đang chạy. |
| `/voice/ui_status` | Backend → UI | `TRANSIENT_LOCAL` / `RELIABLE` / `1` | Trạng thái tổng hợp cho UI (trạng thái hiện tại, `active_mission`, `last_terminal_mission`, `recent_requests` tối đa 20 yêu cầu gần nhất, `supported_actions`). |
| `/voice/nav_state` | Backend → UI | `TRANSIENT_LOCAL` / `RELIABLE` / `1` | Heartbeat & trạng thái chi tiết của `nav_command_node` (gồm `client_request_id`, `last_terminal_mission`, `distance_remaining`, `eta_seconds`). |
| `/voice/nav_status` | Backend → UI | `TRANSIENT_LOCAL` / `RELIABLE` / `1` | Sự kiện vòng đời điều hướng (`NAVIGATION_STARTED`, `GOAL_COMPLETED`, `SUCCEEDED`, `CANCELED`, `ABORTED`). |
| `/tracking/session_state` | Backend → UI | `TRANSIENT_LOCAL` / `RELIABLE` / `1` | Trạng thái khóa hành khách (`state`, `locked`, `obs_fresh`, `session_id`, `reason_code`, `target_distance_m`, `target_bearing_rad`). |
| `/voice/ui_request` | UI → Backend | `VOLATILE` / `RELIABLE` / `10` | Cổng nhận lệnh chính thức từ UI (`navigate_id`, `text_nlu`, `cancel`, `replace_id`, `pause`, `resume`). |
| `/voice/text` | UI/STT → `intent_node` | `VOLATILE` / `RELIABLE` / `10` | Hỗ trợ cả chuỗi tiếng Việt thuần và JSON `{ "text", "client_request_id", "source": "ui" }`. UI ưu tiên gửi qua `/voice/ui_request` (`type: "text_nlu"`), node sẽ tự chuyển tiếp sang `/voice/text` để qua `intent_node` thật. |
| `/voice/feedback` | Backend → UI | `VOLATILE` / `RELIABLE` / `10` | Luồng sự kiện tức thời (`ANNOUNCING`, `STARTING`, `NAVIGATING`, `REJECTED`, `SUCCEEDED`, `FAILED`, `CANCELED`) có kèm `client_request_id` & `request_id`. |

> [!IMPORTANT]
> Các client kết nối muộn (late-join) hoặc kết nối lại (reconnect) **phải** subscribe `/voice/places_catalog`, `/voice/ui_status`, `/voice/nav_state`, và `/tracking/session_state` với `DurabilityPolicy.TRANSIENT_LOCAL` và `ReliabilityPolicy.RELIABLE` để nhận ngay snapshot mới nhất mà không cần chờ sự kiện mới.

---

## 4. Lấy danh mục điểm đến (Read-only Catalog)

Subscribe `/voice/places_catalog` (`TRANSIENT_LOCAL`, `RELIABLE`, `depth=1`).

Ví dụ payload:
```json
{
  "schema_version": "1.0",
  "read_only": true,
  "source_file": "places_sim.json",
  "default_frame_id": "map",
  "stamp": {"sec": 42, "nanosec": 120000000},
  "count": 10,
  "places": [
    {
      "id": "gate_15",
      "name": "Cổng số 15",
      "description": "Cổng khởi hành số 15",
      "category": "gate",
      "enabled": true,
      "frame_id": "map",
      "x": 2.0,
      "y": -0.5,
      "yaw": 0.0,
      "aliases": ["cổng 15", "cổng số 15", "gate 15", "cửa 15"]
    }
  ],
  "categories": [
    {
      "id": "toilet",
      "name": "Nhà vệ sinh",
      "description": "Chọn nhà vệ sinh gần nhất theo độ dài đường đi",
      "enabled": true,
      "members": ["toilet_north", "toilet_south"]
    }
  ]
}
```

- UI chỉ hiển thị các mục có `enabled: true` (hoặc làm mờ mục `enabled: false`).
- `x`, `y`, `yaw`, `frame_id` **chỉ dùng để vẽ bản đồ/ghim trên UI**; khi gửi lệnh điều hướng, UI **chỉ được gửi `destination_id`** (không gửi tọa độ thô).

---

## 5. Gửi yêu cầu từ UI (`/voice/ui_request`)

Mọi yêu cầu bắt buộc có `client_request_id` (chuỗi duy nhất phía client) và `type`.

### 5.1 Chọn điểm đến theo ID (`navigate_id`)
```json
{
  "client_request_id": "ui-req-001",
  "type": "navigate_id",
  "destination_id": "gate_15"
}
```
- **Chặn đổi đích ngoài ý muốn (Busy Guard):** Nếu robot đang không ở `IDLE` (vd: đang `NAVIGATING` hoặc `ANNOUNCING`), `navigate_id` sẽ bị từ chối với `event="REJECTED"`, `status="BUSY_MISSION_ACTIVE"` (trừ khi gửi `type: "replace_id"` hoặc đặt `allow_replace: true` tường minh).
- Nếu `enable_escort=true` và hành khách chưa khóa (`locked=false` hoặc `state != "LOCKED_WAIT_DESTINATION"`), yêu cầu bị từ chối với `event="REJECTED"`, `status="ESCORT_NOT_LOCKED"` (hoặc `ESCORT_NO_HEARTBEAT`, `ESCORT_OBSERVATION_STALE`, ...).

### 5.2 Gửi câu lệnh tiếng Việt qua NLU thật (`text_nlu`)
```json
{
  "client_request_id": "ui-req-002",
  "type": "text_nlu",
  "text": "Dẫn tôi tới cổng 15"
}
```
- `nav_command_node` chuyển tiếp sang `/voice/text` kèm `client_request_id`, `source: "ui"`, `allow_replace: false` để `intent_node` (`NluEngine`) phân tích và phát `/voice/intent`.
- Nếu câu không hợp lệ/bị phủ định (`unknown` / `negated`), `intent_node` phát `action: "nlu_rejected"` và UI nhận `event="REJECTED"`, `status="UNKNOWN_INTENT"` (hoặc `"NEGATED_COMMAND"`) khớp đúng `client_request_id`.

### 5.3 Hủy chuyến (`cancel`) & Đổi điểm đến (`replace_id`)
```json
{"client_request_id": "ui-req-003", "type": "cancel"}
```
```json
{"client_request_id": "ui-req-004", "type": "replace_id", "destination_id": "gate_14"}
```

### 5.4 Tạm dừng / Tiếp tục (`pause` / `resume`)
- Ở chế độ `enable_escort: false`: `pause` và `resume` hoạt động bình thường.
- Ở chế độ `enable_escort: true`: `pause` và `resume` **không được hỗ trợ** (vì hủy goal trên Nav2 sẽ kết thúc phiên hộ tống hiện tại); nếu gửi sẽ bị từ chối với `event="REJECTED"`, `status="PAUSE_UNSUPPORTED_IN_ESCORT"` / `"RESUME_UNSUPPORTED_IN_ESCORT"`. Danh sách hành động khả dụng được công bố động trong `supported_actions` của `/voice/ui_status`.

---

## 6. Chống gửi trùng (Idempotency) & Kết nối lại (Reconnect)

1. **Idempotency:** Nếu UI gửi lại đúng `client_request_id` đã xử lý trước đó (kể cả khi thành công, đang chạy, hay đã bị từ chối), backend **không** gửi goal thứ 2 tới Nav2 mà phát lại kết quả đã lưu trong cache trên `/voice/feedback` (có `"duplicate": true`) và cập nhật `/voice/ui_status`.
2. **Đọc kết quả chuyến đi cuối sau khi Reconnect:** Khi chuyến đi kết thúc (`SUCCEEDED`, `CANCELED`, `ABORTED`), `nav_command_node` quay về `mission_state: "IDLE"` nhưng **giữ lại** `last_terminal_mission` trên cả `/voice/ui_status` và `/voice/nav_state` (`TRANSIENT_LOCAL`), đồng thời giữ 20 yêu cầu gần nhất trong `recent_requests` của `/voice/ui_status`.

Ví dụ `/voice/ui_status` sau khi chuyến đi bị safety abort do mất dấu hành khách:
```json
{
  "schema_version": "1.0",
  "mission_state": "IDLE",
  "enable_escort": true,
  "supported_actions": ["navigate_id", "text_nlu", "cancel", "replace_id"],
  "active_mission": null,
  "last_terminal_mission": {
    "mission_id": "mis_1732710005000_1",
    "client_request_id": "ui-req-001",
    "request_id": "ui-req-001",
    "destination_id": "gate_15",
    "destination_name": "Cổng số 15",
    "session_id": "sess-1732710000-1",
    "goal_attempt_id": "goal_1732710006000_1",
    "nav2_uuid": "a1b2c3d4e5f6...",
    "status": "CANCELED",
    "reason": "ESCORT_SAFETY_ABORT",
    "abort_request_id": "abort_1732710020000_1",
    "finished_at": 1732710021.5
  },
  "recent_requests": [
    {
      "client_request_id": "ui-req-001",
      "action": "navigate",
      "event": "CANCELED",
      "status": "ESCORT_SAFETY_ABORT",
      "mission_id": "mis_1732710005000_1",
      "goal_attempt_id": "goal_1732710006000_1",
      "nav2_uuid": "a1b2c3d4e5f6...",
      "session_id": "sess-1732710000-1",
      "destination_id": "gate_15",
      "destination_name": "Cổng số 15",
      "abort_request_id": "abort_1732710020000_1"
    }
  ]
}
```
