# PERSON TRACKING — CÁC QUYẾT ĐỊNH KIẾN TRÚC KỸ THUẬT (ARCHITECTURAL DECISION RECORDS)

**Dự án:** Robot dẫn đường hành khách sân bay (Airport Escort Robot)  
**Phân hệ:** `person_tracking`  
**Phiên bản:** 1.0 — 23/11/2026 (Phase 1)  

---

## Danh mục Quyết định Kiến trúc (ADR Index)

- **ADR-01:** Phân quyền Khóa và Sinh Phiên Duy nhất tại `escort_tracking_manager`
- **ADR-02:** Vị trí Chèn Velocity Guard trong Chuỗi Vận tốc Nav2
- **ADR-03:** Cơ chế Ép Vận tốc 0 Chủ động (Active Continuous Zeroing) vs Watchdog Thụ động
- **ADR-04:** Tách bạch Chuyển động Ngang Nav2 (`linear.y` pass-through) và Hiệu chỉnh Ngang Tracking
- **ADR-05:** Ứng dụng Độ lệch Tuyệt đối Trung vị (MAD) Lọc Nhiễu Thân người
- **ADR-06:** Sử dụng Thời gian Đơn điệu (Monotonic Time) cho Bộ đếm An toàn Watchdog
- **ADR-07:** Định dạng JSON Chuẩn hóa cho Giao tiếp Liên phân hệ với Voice
- **ADR-08:** Chính sách Lựa chọn Mô hình Thị giác Ứng viên (YOLO Pose + ByteTrack)
- **ADR-09:** Khóa Chặn Khởi tạo Phiên mới khi Goal Cũ Chưa Xác nhận Terminal
- **ADR-10:** Quy ước Trục Tọa độ Quang học và Dấu Góc quay cho Camera Sau

---

### ADR-01: Phân quyền Khóa và Sinh Phiên Duy nhất tại `escort_tracking_manager`
- **Bối cảnh:** Khi nhiều node cùng xử lý (Perception phát hiện người, Manager điều phối, Voice nhận lệnh), nếu cả Perception và Manager đều tự ý sinh `session_id` hoặc chốt khóa, sẽ gây bất đồng bộ trạng thái khi có nhiều người hoặc mất frame.
- **Quyết định:**
  - `passenger_perception` chỉ đóng vai trò **Sensor/Observer**: nhận diện bounding boxes, keypoints, đo depth, tính thời gian giơ tay của từng ứng viên và xuất quan sát thô `/tracking/observation`.
  - `escort_tracking_manager` là **nơi duy nhất có quyền quyết định khóa (Sole Lock Authority)**: chọn người gần nhất, cấp phát `session_id`, quản lý FSM và xuất `/tracking/session_state`.
- **Hệ quả:** Tránh hoàn toàn race condition trong việc khóa khách; đảm bảo mỗi phiên dẫn đường gắn liền với một UUID duy nhất xuyên suốt.

---

### ADR-02: Vị trí Chèn Velocity Guard trong Chuỗi Vận tốc Nav2
- **Bối cảnh:** Nav2 stack của Linorobot2 bao gồm: `controller_server` -> `velocity_smoother` -> `collision_monitor` -> Base. Cần xác định vị trí đặt `escort_velocity_guard` để vừa can thiệp được vận tốc của Nav2 vừa không phá vỡ an toàn va chạm.
- **Quyết định:**
  - Chuỗi chuẩn bắt buộc: `Nav2 Controller -> Velocity Smoother -> Escort Velocity Guard -> Collision Monitor -> Base`.
  - Guard nhận `cmd_vel_smoothed` và xuất `cmd_vel_guarded`.
  - `collision_monitor` nhận `cmd_vel_guarded` và xuất `/cmd_vel` cuối cùng tới motor driver.
- **Hệ quả:** Collision Monitor luôn nằm sau Guard, đảm bảo cảm biến laser/lidar có quyền phủ quyết va chạm tối cao. Guard không bao giờ bypass Collision Monitor.

---

### ADR-03: Cơ chế Ép Vận tốc 0 Chủ động (Active Continuous Zeroing) vs Watchdog Thụ động
- **Bối cảnh:** Khảo sát cho thấy `command_timeout.py` của Gazebo chỉ phát 1 bản tin zero duy nhất khi topic `cmd_vel` im lặng >200ms. Nếu có nguồn phát lạ hoặc Nav2 tiếp tục gửi lệnh trễ sau abort, watchdog thụ động này hoàn toàn vô hiệu.
- **Quyết định:** Khi ở bất kỳ trạng thái không an toàn nào (`IDLE`, `LOCKED_WAIT_DESTINATION`, `WAITING_FOR_PASSENGER`, `ABORTING`, `FAULT`), `escort_velocity_guard` sẽ **liên tục phát các bản tin `Twist(0, 0, 0)` với tần số 20 Hz** vào chuỗi điều khiển.
- **Hệ quả:** Lệnh dừng chủ động liên tục ghi đè mọi dữ liệu sót, triệt tiêu nguy cơ robot giật cục hoặc tự chạy lại do lệnh trễ.

---

### ADR-04: Tách bạch Chuyển động Ngang Nav2 (`linear.y` pass-through) và Hiệu chỉnh Ngang Tracking
- **Bối cảnh:** Robot sân bay sử dụng khung gầm bánh Mecanum có khả năng dạt ngang (`linear.y`). Nav2 DWB Controller tính toán cả `linear.x` và `linear.y` để lách vật cản. Tuy nhiên, nếu tracking tự ý sinh thêm vận tốc `linear.y` để bám theo khách khi chưa kiểm chứng footprint, đuôi xe có thể quét trúng vật cản.
- **Quyết định:**
  1. Thành phần `linear.y` từ Nav2 được **cho phép đi qua nguyên vẹn (pass-through)** dưới sự giám sát của bộ giới hạn vận tốc toàn cục.
  2. Thành phần `linear.y` do Tracking tự sinh (tracking lateral correction) **mặc định TẮT (`0.0 m/s`)** cho tới khi hoàn tất kiểm thử động học ở Phase 7.
- **Hệ quả:** Bảo toàn khả năng dẫn đường linh hoạt của Nav2 trên xe Mecanum, đồng thời loại trừ rủi ro va quẹt do tracking tự sinh lệnh ngang.

---

### ADR-05: Ứng dụng Độ lệch Tuyệt đối Trung vị (MAD) Lọc Nhiễu Thân người
- **Bối cảnh:** Phép đo depth vùng thân người (torso patch) thường bị nhiễu do phông nền xung quanh (background clutter), người đi qua lại hoặc lỗ hổng phản xạ tia hồng ngoại.
- **Quyết định:** Sử dụng công thức Median Absolute Deviation (MAD):
  $$\text{MAD} = \text{median}(|\text{depth} - z|)$$
  với $z = \text{median}(\text{patch})$. Loại bỏ patch nếu $\text{MAD} > \max(0.10, z \times 0.08)$ hoặc số pixel hợp lệ $< 30\%$.
- **Hệ quả:** Đo khoảng cách chính xác, loại bỏ triệt để điểm ảnh phông nền, đảm bảo tính vững chắc trước nhiễu hạt của cảm biến Astra.

---

### ADR-06: Sử dụng Thời gian Đơn điệu (Monotonic Time) cho Bộ đếm An toàn Watchdog
- **Bối cảnh:** Trong môi trường ROS 2 có mô phỏng, topic `/clock` có thể bị tạm dừng (pause), nhảy lùi (jump backward khi reset simulation), hoặc bị trôi gián đoạn.
- **Quyết định:**
  - Header bản tin ROS và dữ liệu hình học vẫn dùng ROS Clock (`node->get_clock()->now()`).
  - Toàn bộ các bộ đếm watchdog (stream timeout, grace period, abort timeout) bên trong node phải đối chiếu với `time.monotonic()` của hệ thống để đo lường khoảng thời gian thực tế đã trôi qua.
- **Hệ quả:** Hệ thống an toàn không bị "đóng băng" vô thời hạn khi `/clock` bị pause hoặc lag.

---

### ADR-07: Định dạng JSON Chuẩn hóa cho Giao tiếp Liên phân hệ với Voice
- **Bối cảnh:** `airport_robot_voice` hiện đang sử dụng `std_msgs/msg/String` bọc chuỗi JSON serialize cho `/voice/nav_state` và `/voice/feedback`.
- **Quyết định:** Để tương thích ngay mà không bắt buộc tạo package custom ROS message mới ngoài allowlist, Tracking sử dụng chuỗi JSON có schema nghiêm ngặt cho `/tracking/session_state`, `/tracking/safety_request`, và `/voice/nav_status`.
- **Hệ quả:** Tích hợp nhanh, linh hoạt, dễ debug qua CLI (`ros2 topic echo`), không đòi hỏi biên dịch lại package message chung.

---

### ADR-08: Chính sách Lựa chọn Mô hình Thị giác Ứng viên (YOLO Pose + ByteTrack)
- **Bối cảnh:** Dự án đã có sẵn checkpoint `yolo26n-pose.pt` tại workspace root và thư viện `ultralytics 8.4`, `lap 0.5` trong container. Tuy nhiên hiệu năng inference thực tế trên GPU trong container chưa được benchmark chính thức.
- **Quyết định:**
  - Xác định YOLO Pose + ByteTrack là **mô hình ứng viên hàng đầu (Primary Candidate)** để tái sử dụng.
  - Chưa công bố đây là mô hình nghiệm thu cuối cùng hoặc khẳng định đạt 30 FPS cho tới khi tiến hành benchmark tải thực tế trong Dev Container ở Phase 3 và Phase 10.
- **Hệ quả:** Tôn trọng nguyên tắc kỹ thuật thực chứng, không khẳng định hiệu năng khi chưa đo đạc.

---

### ADR-09: Khóa Chặn Khởi tạo Phiên mới khi Goal Cũ Chưa Xác nhận Terminal
- **Bối cảnh:** Khi xảy ra sự cố mất khách, Tracking gửi yêu cầu `ABORT`. Voice gọi cancel goal trên Nav2. Nếu một hành khách khác lập tức giơ tay và hệ thống cho phép khởi tạo phiên mới trong khi Nav2 Goal cũ vẫn đang cancel dở dang, robot có thể bị xung đột hành trình nghiêm trọng.
- **Quyết định:** `escort_tracking_manager` từ chối mọi yêu cầu khóa khách mới cho tới khi Voice xác nhận Goal cũ đã hoàn toàn kết thúc (`is_terminal == true`).
- **Hệ quả:** Ngăn chặn triệt để hiện tượng chồng chéo hành trình và xung đột goal UUID.

---

### ADR-10: Quy ước Trục Tọa độ Quang học và Dấu Góc quay cho Camera Sau
- **Bối cảnh:** Camera gắn cố định nhìn về phía sau robot (`yaw = pi` so với `base_link`).
  - Hệ trục quang học camera (`camera_color_optical_frame`): $X$ sang phải ảnh, $Y$ xuống dưới ảnh, $Z$ đi thẳng ra xa theo trục nhìn.
  - Hệ trục thân xe (`base_link`): $X$ hướng về trước đầu xe, $Y$ hướng sang trái sườn xe, $Z$ hướng lên trời.
- **Quyết định:**
  - Vì camera quay về sau, một vật thể nằm ở nửa bên phải ảnh ($X_{\text{opt}} > 0$) sẽ nằm ở phía bên trái thân robot nhìn từ trước ($Y_{\text{base}} > 0$).
  - Để đưa khách từ bên phải ảnh trở về tâm ảnh, camera phải quay sang phải, tương ứng robot phải quay sang TRÁI (ngược chiều kim đồng hồ nhìn từ trên xuống -> $\text{angular.z} > 0$).
  - Công thức hiệu chỉnh góc quay có dấu:
    $$\Delta \omega_z = -K_{\text{yaw}} \cdot \text{clamp}(u_{\text{norm}}, -u_{\text{max}}, u_{\text{max}})$$
    trong đó $u_{\text{norm}} \in [-1, 1]$ là tọa độ tâm người chuẩn hóa.
- **Hệ quả:** Dấu điều khiển góc quay hoàn toàn chính xác cho camera sau, không gây hiện tượng robot quay ngược làm mất khách khỏi tầm nhìn.
