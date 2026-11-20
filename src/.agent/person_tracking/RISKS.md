# PERSON TRACKING — PHÂN TÍCH RỦI RO KỸ THUẬT VÀ AN TOÀN (RISKS & UNKNOWNS)

**Dự án:** Robot dẫn đường sân bay (ROS 2 Jazzy, Mecanum, Camera Astra sau)  
**Phân hệ:** `person_tracking`  
**Ngày lập:** 20/11/2026 — Phase 0  

---

## 1. Rủi ro kỹ thuật trọng yếu (Technical Risks)

### R01: Cấu hình mặc định Depth Registration của Astra Driver là `false`
- **Phát hiện:** Trong file launch gốc `ros2_astra_camera/astra_camera/launch/astra.launch.xml` dòng 5: `<arg name="depth_registration" default="false"/>`.
- **Hệ quả kỹ thuật:**
  - Khi `depth_registration = false`, depth image được phát hành trong frame `camera_depth_optical_frame` và **không được căn chỉnh pixel-to-pixel** với ảnh màu trong `camera_color_optical_frame`.
  - Node `person_tracker.py` cũ kiểm tra nghiêm ngặt `rgb_msg.header.frame_id == depth_msg.header.frame_id`, nếu không registration sẽ lập tức văng ngoại lệ `ValueError`.
  - Phép đo depth vùng thân người (`measure`) lấy tọa độ bounding box từ ảnh RGB áp trực tiếp lên ma trận depth; nếu chưa đăng ký, khoảng cách đo được sẽ bị lệch thị sai lớn (parallax error).
- **Biện pháp xử lý:**
  - Launch file cho camera thật bắt buộc phải truyền `depth_registration:=true`.
  - Trong code perception, thêm bộ kiểm tra tính tương thích: nếu nhận được `depth_registration = false` hoặc frame ID khác nhau, ghi nhận cảnh báo rõ ràng, hạ cờ tin cậy khoảng cách (`confidence = LOW`), và chuyển sang chế độ an toàn chứ không crash node.

### R02: Xung đột hướng lắp Camera trong Gazebo URDF vs Robot Thật
- **Phát hiện:** Trong `linorobot2_description/urdf/mecanum_properties.urdf.xacro` dòng 27:
  `<origin xyz="0.2375 0 0.06" rpy="0 0 0"/>` -> Camera mô phỏng trong Gazebo đang hướng ra **phía trước** (`rpy=0,0,0`). Trong khi đó, robot thật có camera Astra gắn cố định hướng ra **phía sau** (`yaw = pi` / 180°).
- **Ràng buộc tối cao:** Rule A.4 cấm tuyệt đối việc chỉnh sửa Gazebo/URDF/xacro.
- **Hệ quả kỹ thuật:** Nếu lấy TF từ Gazebo để tính toán điều khiển hướng cho camera sau, robot sẽ quay ngược hướng cần thiết, có nguy cơ làm mất khách khỏi FOV hoặc va chạm.
- **Biện pháp xử lý:**
  - Không mô phỏng camera trong Gazebo. Dùng dữ liệu ngoại tuyến (video replay, synthetic depth, fake observation generator) có metadata frame quang học chuẩn phía sau.
  - Thiết lập quy ước dấu hình học rõ ràng trong perception và guard:
    - Optical frame: X sang phải ảnh, Y xuống dưới ảnh, Z đi ra xa theo hướng nhìn của camera.
    - Với camera nhìn sau: X optical dương tương ứng phía bên trái thân robot (hướng nhìn tiến); khi khách lệch sang phải ảnh, robot phải điều chỉnh angular.z ngược chiều tương ứng để đưa khách về tâm ảnh.

### R03: Xung đột quyền điều khiển Nav2 (Nav2 Ownership & Escort Manager)
- **Phát hiện:** Code cũ `person_follower.py` tự ý publish thẳng vào `/cmd_vel` với tần số 20 Hz. Cùng lúc đó, Nav2 stack của `linorobot2` cũng phát lệnh vào chuỗi `cmd_vel`. Node `nav_command_node.py` của voice lại là bên nhận lệnh giọng nói.
- **Quy tắc đã chốt (D05):** VOICE là bên DUY NHẤT sở hữu `NavigateToPose`. Tracking tuyệt đối không gọi action Nav2 trực tiếp và không tự lái xe đi theo khách.
- **Hệ quả kỹ thuật:** Nếu tracking tự xuất `/cmd_vel`, hai luồng điều khiển sẽ giật cục, xung đột gia tốc, phá vỡ quỹ đạo toàn cục của Nav2.
- **Biện pháp xử lý:**
  - Khai tử hoàn toàn mô hình follower cũ.
  - Thay bằng `escort_velocity_guard`: chỉ đóng vai trò **cổng an toàn (safety guard)**. Nav2 vẫn dẫn đường chính. Guard chỉ can thiệp theo 3 mức: (1) Cho phép đi qua nguyên vẹn; (2) Giảm tốc độ tịnh tiến (clamp max_vel_x) và chỉnh nhẹ yaw (bounded angular.z trim) để giữ khách trong FOV; (3) Cắt vận tốc về 0 (hard stop) khi mất khách hoặc trạng thái không an toàn.

### R04: Vị trí chèn Velocity Guard trong chuỗi Nav2 & Collision Monitor
- **Phát hiện:** Trong `linorobot2_navigation/config/navigation.yaml`:
  - `velocity_smoother`: xuất `cmd_vel_smoothed`.
  - `collision_monitor`: nhận `cmd_vel_in_topic: "cmd_vel_smoothed"`, xuất `cmd_vel_out_topic: "cmd_vel"`.
- **Rủi ro:** Nếu Guard chèn SAU Collision Monitor (tức là Guard xuất thẳng vào `/cmd_vel`), mọi hiệu chỉnh vận tốc của Guard sẽ **bỏ qua (bypass)** lớp bảo vệ tránh va chạm của Collision Monitor. Điều này vi phạm nghiêm trọng tiêu chuẩn an toàn an ninh công nghiệp.
- **Hệ quả kỹ thuật:** Luồng chuẩn bắt buộc là: `Nav2 Controller -> Velocity Smoother -> Tracking Guard -> Collision Monitor -> Base`.
- **Ràng buộc:** Sửa `navigation.yaml` nằm ngoài allowlist ghi của agent tracking (Rule A.5).
- **Biện pháp xử lý:**
  - Trong Phase 7/8, đề xuất cấu hình remap trong launch của tracking hoặc xin người dùng phê duyệt cập nhật `navigation.yaml` (dòng 295: `cmd_vel_in_topic: "cmd_vel_guarded"`). Trong môi trường test offline/mock, kiểm thử tính toàn vẹn của chuỗi lệnh này.

### R05: Độ trễ hủy Goal của Voice & Lệnh sót (Stale / Delayed Commands)
- **Rủi ro:** Khi tracking xác nhận mất khách kéo dài (`visibility_abort_sec`), tracking gửi yêu cầu hủy (`ABORT`) sang voice. Việc gọi `cancel_goal_async` trên ROS 2 Action có thể mất 100–500 ms hoặc voice bị bận/chết.
- **Hệ quả:** Trong thời gian chờ Nav2 cancel, Nav2 controller vẫn có thể tiếp tục phát vận tốc lái robot đi tiếp, bỏ rơi hành khách.
- **Biện pháp xử lý (Fail-safe Watchdog):**
  - Velocity Guard có cơ chế **ngắt tức thì độc lập**: ngay khi trạng thái chuyển sang `ABORTING` hoặc mất khách quá `visibility_stop_sec`, Guard xuất ngay lệnh 0 vào motor mà không cần chờ voice gửi ACK.
  - Guard kích hoạt chốt khóa lỗi (`fault latch`), ngăn chặn mọi lệnh vận tốc cũ cho đến khi phiên mới được khởi tạo hợp lệ.

### R06: Chọn sai người khi nhiều người cùng giơ tay (Multi-person Ambiguity)
- **Phát hiện:** Node `person_tracker.py` cũ duyệt mảng `boxes` theo thứ tự ngẫu nhiên của detector; ai giơ tay đạt 2.0s trước thì người đó được chọn làm `selected_id`, bất kể xa hay gần.
- **Rủi ro:** Người đứng xa ở hậu cảnh giơ tay trước sẽ cướp quyền của người đứng gần ngay sau robot.
- **Biện pháp xử lý (D07):**
  - Thuật toán khóa ở Phase 4 phải tổng hợp tất cả các track ID giơ tay hợp lệ trong khung thời gian `raise_hand_hold_sec` (thử 1.5s).
  - So sánh khoảng cách Z (depth có kiểm chứng) của các ứng viên; chọn người có `Z` nhỏ nhất (gần camera nhất).
  - Nếu thiếu depth: chỉ dùng tỷ lệ diện tích bounding box làm ước lượng thứ cấp (`estimated_nearest`), gắn cờ giảm độ tin cậy; nếu có sự chênh lệch không đáng kể giữa 2 người (dưới 15%), chuyển trạng thái `AMBIGUOUS_GESTURE` và không khóa người nào để tránh chọn nhầm.

### R07: Trộn ID khi giao nhau / che khuất (ID Swap & Occlusion)
- **Rủi ro:** Khi hành khách đang được khóa bị một người khác đi ngang qua che khuất (crossing occlusion), ByteTrack có thể gán ID của người cắt ngang cho khách hoặc sinh ID mới.
- **Biện pháp xử lý:**
  - Áp dụng `perception_grace_sec` (0.25s) cho phép mất dấu vài frame mà không đổi ID.
  - Khi ID biến mất hoặc có nhiều ID trùng lặp, hạ độ tin cậy, không tự động re-ID sang người khác; duy trì ID phiên đã khóa. Nếu mất quá `visibility_stop_sec` (0.4s), giảm tốc dừng chờ; nếu mất quá `visibility_abort_sec` (1.2s), hủy phiên.

---

## 2. Phần chưa xác nhận và rào cản phần cứng (Unknowns & Blockers)

| Mục | Hiện trạng | Tác động | Kế hoạch giải quyết |
|---|---|---|---|
| **Camera Astra thật** | Không có tại chỗ, không kết nối. | Không thể đo trực tiếp FPS thật, nhiễu depth, méo quang học và độ trễ camera phần cứng. | Ghi nhận trạng thái `HARDWARE_VALIDATION_PENDING`. Sử dụng test video thực tế, rosbag2 và dữ liệu giả lập (synthetic fixtures). Chỉ nghiệm thu ở Phase 11 khi có phần cứng. |
| **Giao diện Voice chính thức** | `airport_robot_voice` chưa có subscriber cho tracking. | Tracking không thể tích hợp trực tiếp với voice node thật ở các phase đầu. | Thiết kế bản hợp đồng `INTERFACES.md`. Tạo node `mock_voice_owner` bên trong package tracking để tự động hóa test FSM và contract tests. Chờ agent voice cập nhật ở Phase 9. |
| **Môi trường chạy PyTorch/GPU trong Docker** | Cần xác minh dependencies PyTorch / Ultralytics trong container ROS 2 Jazzy. | Tracker cần GPU để đạt FPS thời gian thực. | Khảo sát và chuẩn bị phương án kiểm thử offline có CPU fallback; xác minh PyTorch trong container ở Phase 1 theo quy trình được phê duyệt. |
| **Chuyển động dạt ngang Mecanum (`linear.y`)** | Động học mecanum đã mở trong `navigation.yaml` (0.5 m/s), nhưng chưa có kiểm chứng thực tế về trượt bánh và footprint chướng ngại vật phía sau. | Nguy cơ va quẹt khi đi ngang ở góc hẹp. | Khóa `linear.y = 0.0` trong các phase đầu. Chỉ kích hoạt sau khi đã kiểm chứng an toàn và động học ở Phase 7. |

---

## 3. Thứ tự ưu tiên an toàn (Safety Priorities)

1. **Ưu tiên 1 (Tuyệt đối):** Dừng khẩn cấp và cắt vận tốc về 0 (Zero-velocity guard latch) khi:
   - Chưa khóa khách (`IDLE`).
   - Mất quan sát quá thời gian an toàn (`visibility_stop_sec`).
   - Nhận yêu cầu hủy hoặc lỗi stream dữ liệu (`FAULT`).
   - Lệnh Nav2 bị trễ quá hạn (`max_cmd_age_sec`).
2. **Ưu tiên 2:** Collision Monitor luôn nằm sau Guard trong chuỗi chấp hành vận tốc để đảm bảo quyền phủ quyết cuối cùng của radar/laser tránh va chạm.
3. **Ưu tiên 3:** Cô lập quyền điều khiển Nav2 — Voice là đơn vị duy nhất gửi/hủy `NavigateToPose`. Tracking không tạo lệnh cạnh tranh.
4. **Ưu tiên 4:** Không bao giờ tự động nối lại hành trình (resumption) sau khi phiên đã bị hủy. Mỗi lượt dẫn đường mới bắt buộc phải có một lượt giơ tay khóa mới với `session_id` mới.
