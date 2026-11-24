# PERSON TRACKING — CẨM NANG CHUẨN BỊ VÀ NGHIỆM THU PHASE 11
## Hướng Dẫn Vận Hành Phần Cứng Thực Tế: Camera Orbbec Astra & Robot Mecanum

- **Phân hệ:** `person_tracking`
- **Phiên bản:** 1.0 — 24/11/2026
- **Trạng thái phân hệ:** **SOFTWARE_READY / HARDWARE_PENDING**
- **Mục tiêu:** Cung cấp quy trình kiểm tra từng bước, bộ lệnh tái hiện, bảng tiêu chí Pass/Fail và công cụ chẩn đoán chuyên dụng sẵn sàng khi kết nối phần cứng thực tế (Camera Orbbec Astra Pro và khung gầm Robot Mecanum Linorobot2).
- **Quy tắc An toàn Cốt lõi (Safety Mandate):**
  > [!IMPORTANT]
  > **TÁCH BIỆT HOÀN TOÀN GIỮA KIỂM TRA TẠI CHỖ VÀ THỬ CHUYỂN ĐỘNG:**
  > 1. Toàn bộ các bước kiểm tra camera Astra, topic ROS, TF, depth registration, thuật toán nhận diện và watchdog phải được thực hiện khi **xe đứng yên hoặc kê bánh xe hổng khỏi sàn (Stationary Bench Testing)**.
  > 2. Thử nghiệm chuyển động chỉ được tiến hành sau khi hoàn thành toàn bộ kiểm tra tĩnh, **ở tốc độ thấp ($v_{x,\max} \le 0.30\text{ m/s}$)**, trong khu vực có kiểm soát, có người giám sát đi kèm và **công tắc E-stop vật lý đã được xác nhận hoạt động**.
  > 3. Tuyệt đối **không tự ý kích hoạt chuyển động** trong lượt chuẩn bị này.

---

## 1. QUY ƯỚC MÔI TRƯỜNG, ROS DOMAIN VÀ PHÂN QUYỀN

### 1.1 Quy ước ROS_DOMAIN_ID
- **Ghép nối hệ thống chính (Main System Integration):** Bắt buộc sử dụng `ROS_DOMAIN_ID=0` (đồng bộ với Voice, Nav2 và Base Controller).
- **Kiểm thử cô lập hoặc chẩn đoán riêng (Isolated Diagnostic Tests):** Sử dụng `ROS_DOMAIN_ID=43` (Perception/ROS runtime) hoặc `ROS_DOMAIN_ID=44` (Gazebo/Simulation baseline).

### 1.2 Phân lập Build Workspace
Luôn build và nạp môi trường phân hệ `person_tracking` cách ly:
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select person_tracking \
  --build-base build_tracking \
  --install-base install_tracking \
  --symlink-install
source /home/ws/install_tracking/setup.bash
```

### 1.3 Nguyên tắc Phân quyền Điều khiển
- **Voice Agent:** Chủ sở hữu **duy nhất** của Action Client `NavigateToPose`. Tracking tuyệt đối không tạo action client hay phát lệnh trực tiếp tới Nav2.
- **Tracking Manager:** Bên **duy nhất** cấp `session_id`, xác thực cử chỉ khóa khách và gửi `safety_request` (ABORT/STOP).
- **Velocity Guard & Watchdog:** Cổng an toàn độc lập chốt chặn vận tốc, ngắt zero ở 20 Hz độc lập khi mất quan sát hoặc mất lệnh, kẹp cứng $v_y = 0.0000\text{ m/s}$.

---

## 2. DANH SÁCH BƯỚC CẦN NGƯỜI DÙNG CÓ MẶT VÀ KẾT NỐI PHẦN CỨNG

Dưới đây là các thao tác vật lý bắt buộc cần sự có mặt của người vận hành:

| STT | Bước thao tác vật lý | Thiết bị liên quan | Mục đích & Lưu ý |
|---|---|---|---|
| **HW-01** | Cắm cáp USB Camera Astra | Orbbec Astra Pro / Robot Computer | Cắm vào cổng USB 3.0 (màu xanh) để đảm bảo băng thông RGB-D 30 FPS. |
| **HW-02** | Cắm cáp USB Microcontroller | Raspberry Pi Pico (micro-ROS) | Cắm vào cổng USB của Robot Computer (`/dev/ttyACM0` hoặc `/dev/ttyUSB0`). |
| **HW-03** | Kê kích bánh xe hổng sàn | Khung gầm Mecanum | Kê 4 bánh xe không chạm sàn trong toàn bộ Giai đoạn 1 (Stationary Test). |
| **HW-04** | Kiểm tra công tắc E-Stop | Nút dừng khẩn cấp vật lý | Nhấn thử E-stop để xác nhận mạch ngắt nguồn động cơ hoạt động ngay lập tức. |
| **HW-05** | Bố trí mốc khoảng cách | Thước dây & băng dính sàn | Dán vạch mốc tại $0.5\text{ m}, 1.0\text{ m}, 1.5\text{ m}, 2.0\text{ m}, 2.5\text{ m}, 3.0\text{ m}$ phía sau robot. |
| **HW-06** | Đóng vai hành khách / cắt ngang | Người giám sát & cộng sự | Người A đứng khóa session; Người B đi cắt ngang để kiểm chứng ca T06. |

---

## 3. GIAI ĐOẠN 1: KIỂM TRA TẠI CHỖ KHÔNG CHẠY XE (STATIONARY BENCH TESTING)

### 3.1 Kiểm tra Phần cứng và Driver Camera Astra
1. **Xác nhận kết nối thiết bị USB:**
   ```bash
   lsusb | grep -i -E "(orbbec|astra|2bc5)"
   ls -la /dev/video*
   ```
   *Yêu cầu:* Thấy xuất hiện thiết bị UVC Color (`2bc5:0501`) và OpenNI Depth (`2bc5:0403`).
2. **Kiểm tra udev rules:**
   Đảm bảo file `/etc/udev/rules.d/56-orbbec-usb.rules` đã được nạp để cấp quyền truy cập thiết bị không cần root:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. **Khởi chạy Astra Camera Driver:**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   ros2 launch astra_camera astra_pro.launch.xml depth_registration:=true
   ```
4. **Kiểm tra định dạng topic, encoding và QoS:**
   Chạy script chẩn đoán tự động:
   ```bash
   python3 /home/ws/src/person_tracking/test/check_astra_hardware_topics.py
   ```
   *Tiêu chí nghiệm thu:*
   - Topic RGB: `/camera/color/image_raw` (encoding: `rgb8` hoặc `bgr8`, tần số 20–30 Hz).
   - Topic Depth: `/camera/depth/image_raw` (encoding: `16UC1` đơn vị mm, tần số 20–30 Hz).
   - CameraInfo: `/camera/color/camera_info` (K ma trận hợp lệ, $f_x, f_y, c_x, c_y > 0$).
   - Đồng bộ thời gian: Chênh lệch timestamp giữa RGB và Depth $\Delta t = |t_{\text{rgb}} - t_{\text{depth}}| \le 0.060\text{ s}$.
   - Độ tuổi frame: $t_{\text{now}} - t_{\text{stamp}} \le 0.400\text{ s}$.
   - Căn chỉnh D2C (Hardware Alignment): Point cloud trên `/camera/depth_registered/points` hiển thị đúng biên dạng vật thể, không bị lệch pha giữa màu và độ sâu.

### 3.2 Kiểm tra Hướng Lắp Camera Sau và Xác nhận TF
1. **Đo đạc vị trí và góc lắp thực tế:**
   - Vị trí camera: Lắp phía sau xe ($x < 0$ so với tâm robot `base_link`).
   - Hướng nhìn: Hướng thẳng ra phía sau đuôi robot ($yaw \approx 180^\circ$ hoặc $\pi\text{ rad}$).
   - Đo chiều cao $z$ và góc nghiêng cúi (pitch) so với mặt phẳng ngang.
2. **Kiểm tra TF Tree:**
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_link
   ros2 run tf2_ros tf2_echo camera_link camera_color_optical_link
   ```
   *Yêu cầu:* TF tree liên tục, không đứt đoạn, frame_id chuẩn `camera_color_optical_link`.
3. **Đánh giá sai số khoảng cách bằng mốc thực tế (Ground-Truth Distance Benchmarks):**
   - Đặt tấm bia phẳng hoặc người đứng tại các mốc dán sàn: $0.5\text{ m}, 1.0\text{ m}, 1.5\text{ m}, 2.0\text{ m}, 2.5\text{ m}, 3.0\text{ m}$.
   - Đo khoảng cách tâm bounding box từ depth map và so sánh với thước mét thực tế:
     $$\text{Error} = |z_{\text{measured}} - z_{\text{ground\_truth}}|$$
   - *Bảng tiêu chí sai số đo:*
     - $0.5\text{ m} - 1.0\text{ m}$: Sai số $\le 2.0\%$ (chênh lệch $\le 2\text{ cm}$).
     - $1.0\text{ m} - 2.0\text{ m}$: Sai số $\le 3.0\%$ (chênh lệch $\le 6\text{ cm}$).
     - $2.0\text{ m} - 2.5\text{ m}$: Sai số $\le 5.0\%$ (chênh lệch $\le 12.5\text{ cm}$).
     - $> 2.5\text{ m}$: Bật cờ cảnh báo cự ly xa; $> 3.0\text{ m}$ đánh dấu `OUT_OF_RANGE`.

### 3.3 Khởi chạy Perception Pipeline & Profiling Tải Node Thật theo PID
1. **Khởi chạy Pipeline:**
   ```bash
   ros2 launch person_tracking perception_pipeline.launch.py \
     source_mode:=live_astra \
     model_path:=/home/ws/yolo26n-pose.pt \
     device:=cuda:0
   ```
2. **Chạy script Profile PID trong cửa sổ 30s–60s:**
   ```bash
   python3 /home/ws/src/person_tracking/test/profile_detector_pid.py --duration 45 --domain 0
   ```
   *Tiêu chí nghiệm thu:*
   - Đo đúng PID của tiến trình `PersonDetectorNode`.
   - CPU Usage: Đo liên tục qua cửa sổ 45s (mức trung bình danh định: $\le 45\%$ trên 1 core CPU).
   - RAM RSS: $\le 1900\text{ MB}$ (ổn định, không tăng lũy tiến).
   - GPU VRAM: $\le 300\text{ MB}$ (allocated).
   - Thông lượng thực tế qua DDS: $\ge 15.0\text{ FPS}$ (phù hợp với tốc độ phần cứng camera 20–30 Hz).
   - Tỷ lệ rơi frame (Dropped Frames): $\le 5.0\%$.

### 3.4 Thực thi Ca T06 (Người Đi Cắt Ngang Khách Đang Khóa)
1. **Thiết lập:** Người A đứng tại mốc $1.5\text{ m}$ giơ tay khóa session (`LOCKED_WAIT_DESTINATION`).
2. **Thực hiện cắt ngang:** Người B đi bộ cắt ngang khoảng không giữa camera robot và Người A.
3. **Quan sát telemetry trên `/tracking/observation`:**
   - ByteTrack gán track ID riêng cho Người B ($ID_B \ne ID_A$).
   - Khi Người B che khuất một phần Người A với $\text{IoU} > 0.35$: cờ `is_ambiguous = True` được kích hoạt.
   - Khi Người B đi qua hoàn toàn: cờ `is_ambiguous = False` tự động giải phóng, track $ID_A$ được duy trì ổn định.
   - *Tiêu chí Pass:* **ID Switch = 0**; hệ thống không bị nhảy khóa nhầm sang Người B.

### 3.5 Kiểm tra Watchdog Mất Lệnh Phần cứng trên Đế xe (Base Controller)
1. **Xác định vị trí Watchdog:**
   - Watchdog phần cứng nằm trong firmware micro-ROS trên microcontroller (Raspberry Pi Pico).
   - Hằng số timeout phần cứng $T_{\text{hw\_watchdog}}$ được cấu hình trong firmware (mặc định $200 - 500\text{ ms}$). Khi không nhận được bản tin `/cmd_vel` mới, microcontroller lập tức ngắt xung PWM điều khiển động cơ.
2. **Quy trình kiểm tra tĩnh (Bánh xe kê hổng sàn):**
   - Bật micro-ROS agent kết nối với microcontroller:
     ```bash
     ros2 launch linorobot2_bringup bringup.launch.py
     ```
   - Phát lệnh quay bánh xe thử nghiệm ở tốc độ chậm:
     ```bash
     ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
     ```
   - Quan sát bánh xe quay và **tự động dừng ngay lập tức** sau khoảng $0.2 - 0.5\text{ s}$.
   - *Tiêu chí Pass:* Bánh xe không bao giờ quay liên tục không kiểm soát khi publisher dừng phát lệnh.

---

## 4. GIAI ĐOẠN 2: QUY TRÌNH THỬ XE TỐC ĐỘ THẤP TẠI KHU VỰC CÓ KIỂM SOÁT (LOW-SPEED MOTION TESTING)

> [!CAUTION]
> **ĐIỀU KIỆN TIÊN QUYẾT BẮT BUỘC TRƯỚC KHI THẢ XE XUỐNG SÀN:**
> 1. Khu vực thử nghiệm: Hành lang hoặc phòng thử nghiệm biệt lập, sàn phẳng sạch, kích thước tối thiểu $10\text{ m} \times 3\text{ m}$, không có vật cản nguy hiểm, không có trẻ em hay người ngoài.
> 2. Đã kiểm tra E-stop vật lý: Nhấn nút E-stop ngắt động cơ tức thì.
> 3. Người giám sát (Supervisor) luôn đi bộ kèm sát cạnh robot trong tầm với tay tới nút E-stop hoặc cầm sẵn remote ngắt khẩn cấp.
> 4. Khóa giới hạn vận tốc an toàn: Thiết lập $v_{x,\max} = 0.30\text{ m/s}$ (thấp hơn trần danh định $0.50\text{ m/s}$ để đảm bảo phản ứng an toàn).

### 4.1 Khởi động Toàn bộ Hệ thống trên `ROS_DOMAIN_ID=0`
1. **Terminal 1 — Robot Base & Sensors:**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   ros2 launch linorobot2_bringup bringup.launch.py
   ```
2. **Terminal 2 — Navigation (Nav2):**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   ros2 launch linorobot2_navigation navigation.launch.py
   ```
3. **Terminal 3 — Astra Camera:**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   ros2 launch astra_camera astra_pro.launch.xml depth_registration:=true
   ```
4. **Terminal 4 — Tracking Escort Stack:**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   source /home/ws/install_tracking/setup.bash
   ros2 launch person_tracking perception_pipeline.launch.py source_mode:=live_astra &
   ros2 run person_tracking tracking_manager &
   ros2 run person_tracking velocity_guard --ros-args -p max_linear_x:=0.30 -p max_angular_z:=0.20 &
   ros2 run person_tracking velocity_watchdog &
   ```
5. **Terminal 5 — Voice Agent thật:**
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/jazzy/setup.bash
   ros2 run airport_robot_voice nav_command_node --ros-args -p enable_escort:=true
   ```

### 4.2 Các Bước Thử nghiệm Vận hành Có Người Giám sát
1. **Thử nghiệm 1 (Khóa và dẫn thẳng tốc độ thấp):**
   - Người giám sát đứng phía sau robot $1.5\text{ m}$, giơ tay $\ge 1.5\text{ s}$.
   - Nghe Voice phát xác nhận khóa thành công (`LOCKED_WAIT_DESTINATION`).
   - Ra lệnh giọng nói tới điểm đến gần (ví dụ "đi quầy thông tin", cự ly $3\text{ m}$).
   - Robot bắt đầu di chuyển về phía trước, người giám sát đi theo sau ở cự ly $1.5\text{ m}$.
   - *Yêu cầu:* Robot giữ vận tốc ổn định trong khoảng $0.20 - 0.30\text{ m/s}$, không giật cục.
2. **Thử nghiệm 2 (Khách tụt lại xa > 2.0m và > 2.5m):**
   - Đang dẫn đường, người giám sát cố tình dừng lại để khoảng cách tăng lên $2.2\text{ m}$, sau đó $> 2.5\text{ m}$.
   - *Yêu cầu:* Khi $> 2.0\text{ m}$, xe giảm tốc độ; khi $> 2.5\text{ m}$, xe dừng hẳn chờ; khi người giám sát bước lại gần $1.5\text{ m}$, xe tự động tiếp tục di chuyển mượt mà (hysteresis).
3. **Thử nghiệm 3 (Khách lệch sang mép ảnh):**
   - Người giám sát bước chếch sang trái/phải mép camera.
   - *Yêu cầu:* Robot điều chỉnh góc quay $\omega_z$ nhẹ hướng tâm để giữ khách trong tầm nhìn, không xoay đột ngột.
4. **Thử nghiệm 4 (Mất khách & Safety Abort):**
   - Người giám sát cố tình bước khuất sau vách tường:
     - Tại $t = 0.38\text{ s}$: Robot dập vận tốc về $0.0000\text{ m/s}$ (`WAITING_FOR_PASSENGER`).
     - Tại $t = 1.20\text{ s}$: Tracking gửi `safety_request` abort $\to$ Voice hủy goal Nav2 $\to$ robot dừng hẳn vĩnh viễn.
     - Người giám sát bước lại vào camera: **Robot tuyệt đối không tự chạy lại**, giữ nguyên trạng thái dừng.
5. **Thử nghiệm 5 (Collision Monitor ưu tiên dập laser):**
   - Đang dẫn đường, một chướng ngại vật xuất hiện phía trước đầu xe trong phạm vi $0.3\text{ m}$.
   - *Yêu cầu:* Collision Monitor lập tức dập `/cmd_vel` về 0, ưu tiên cao hơn lệnh của Guard.

---

## 5. GIAI ĐOẠN 3: ĐỐI CHIẾU ĐA LỚP VẬN TỐC & PHÉP ĐO CHUYỂN ĐỘNG ĐỘC LẬP

Để làm sáng tỏ triệt để hiện tượng vọt lố vận tốc trên `/odom` từng ghi nhận ở mô phỏng, tiến hành đối chiếu đồng thời 4 lớp số liệu:

```mermaid
flowchart TD
    Cmd["1. Lệnh điều khiển (/cmd_vel)"] --> Motor["Base Controller (Firmware)"]
    Motor --> OdomRaw["2. Odom thô (/odom/unfiltered)"]
    OdomRaw --> EKF["3. Odom lọc (/odom - EKF)"]
    GroundTruth["4. Đo vật lý độc lập (Thước Laser / Vạch bấm giờ)"] -.-> Compare["Bảng đối chiếu & kết luận vọt lố"]
    EKF -.-> Compare
    OdomRaw -.-> Compare
    Cmd -.-> Compare
```

### 5.1 Bố trí Phép đo Chuyển động Vật lý Độc lập (Independent Ground-Truth)
- **Phương pháp 1 (Vạch đo sàn cố định + Bấm giờ quang/điện tử):**
  - Đánh dấu 2 vạch đo cố định trên sàn cách nhau đúng $S = 3.000\text{ m}$.
  - Ghi lại thời gian robot đi qua giữa 2 vạch bằng đồng hồ bấm giờ độ chính xác cao: $t_{\text{start}}$ và $t_{\text{end}}$.
  - Tính vận tốc trung bình thực tế:
    $$v_{\text{ground\_truth}} = \frac{S}{t_{\text{end}} - t_{\text{start}}}$$
- **Phương pháp 2 (Máy đo khoảng cách Laser - Laser Distance Meter):**
  - Đặt máy đo khoảng cách laser cố định phía sau đuôi robot, nhắm vào mặt phẳng đuôi xe.
  - Ghi nhận độ dịch chuyển thực tế theo thời gian.

### 5.2 Bảng Mẫu Thu Thập và Đối Chiếu Vận Tốc Thực Tế
| Lượt chạy | Lệnh `/cmd_vel` max | Odom thô `/odom/unfiltered` max | Odom lọc `/odom` max | Vận tốc đo độc lập $v_{\text{ground\_truth}}$ | Đánh giá sai số EKF vs Thực tế |
|---|---|---|---|---|---|
| Lượt 1 | ... m/s | ... m/s | ... m/s | ... m/s | ... |
| Lượt 2 | ... m/s | ... m/s | ... m/s | ... m/s | ... |
| Lượt 3 | ... m/s | ... m/s | ... m/s | ... m/s | ... |

*Tiêu chí đánh giá:*
- Nếu $v_{\text{ground\_truth}} \le 0.30\text{ m/s}$ và `/odom/unfiltered` $\le 0.30\text{ m/s}$ trong khi `/odom` vọt lên $> 0.35\text{ m/s}$ $\implies$ Chứng minh thực nghiệm độc lập rằng đỉnh vận tốc là do bộ lọc EKF vọt lố số học, không phải chuyển động thực của robot.

---

## 6. BẢNG TIÊU CHÍ NGHIỆM THU PASS/FAIL PHASE 11 (HARDWARE ACCEPTANCE)

| Mã tiêu chí | Hạng mục kiểm tra | Điều kiện ĐẠT (PASS) | Điều kiện KHÔNG ĐẠT (FAIL) |
|---|---|---|---|
| **C-01** | Astra Stream & Format | Stream 20–30 Hz, depth 16UC1, RGB rgb8/bgr8, $\Delta t \le 0.06\text{ s}$ | Mất stream, sai encoding, lệch timestamp $> 0.06\text{ s}$ |
| **C-02** | D2C Hardware Alignment | Depth khớp hoàn toàn RGB, biên dạng vật thể không lệch | Biên dạng người bị bóng mờ hoặc lệch pha $> 5\text{ cm}$ |
| **C-03** | Sai số cự ly mốc thật | Sai số $\le 3\%$ ở $1.0 - 2.0\text{ m}$; $\le 5\%$ ở $2.0 - 2.5\text{ m}$ | Sai số $> 5\%$ trong phạm vi dẫn đường danh định |
| **C-04** | Tải CPU/RAM/VRAM PID thật | Throughput $\ge 15\text{ FPS}$, CPU $\le 45\%$, RAM $\le 2000\text{ MB}$ | Treo node, RAM tăng liên tục, throughput $< 10\text{ FPS}$ |
| **C-05** | Ca T06 Người cắt ngang | Giữ nguyên track ID Người A khi Người B cắt ngang (ID switch = 0) | Nhảy khóa sang Người B hoặc mất phiên vĩnh viễn |
| **C-06** | Watchdog phần cứng base | Bánh xe tự dừng quay trong $0.2 - 0.5\text{ s}$ khi ngắt `/cmd_vel` | Bánh xe tiếp tục quay không kiểm soát khi mất lệnh |
| **C-07** | Bám đuôi tốc độ thấp | Robot di chuyển mượt mà ở cự ly $1.5\text{ m}$, $v_x \le 0.30\text{ m/s}$ | Chạy giật cục, vọt tốc $> 0.30\text{ m/s}$ hoặc trôi ngang |
| **C-08** | Khoảng cách & Mép ảnh | Tụt $> 2.5\text{ m}$ xe dừng chờ; mép ảnh xe xoay chỉnh hướng tâm | Không dừng khi khách tụt lại, quay sai chiều |
| **C-09** | Mất khách & Safety Abort | Dập zero trong $0.4\text{ s}$, abort trong $1.2\text{ s}$, không tự chạy lại | Xe tiếp tục chạy khi mất người, tự chạy lại sau abort |
| **C-10** | E-Stop & Collision Stop | Nhấn E-stop hoặc laser cản $0.25\text{ m}$ $\to$ xe dừng lập tức | Xe không dừng khi bấm E-stop hoặc có cản trước đầu |

---

---

## 6.1 KẾT QUẢ PRE-CHECK PHẦN MỀM: SMOKE TEST GAZEBO SIMULATION (24/11/2026)

Trước khi tiến hành trên phần cứng thật, toàn bộ chuỗi bàn giao phần mềm giữa Voice, Tracking Manager, Velocity Guard, Watchdog và Nav2 đã được kiểm thử thông luồng độc lập trong mô phỏng Gazebo cô lập (`ROS_DOMAIN_ID=44`).

- **Bộ kiểm thử:** `src/person_tracking/test/verify_phase11_software_smoke.py`
- **Bộ thực thi:** `src/person_tracking/test/execute_phase11_software_smoke.sh`
- **File log bằng chứng:**
  * Tổng kết kết quả đo: [.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/phase11_software_smoke.log)
  * Log runtime Gazebo/Nav2: [.agent/person_tracking/evidence/phase_11/gazebo_smoke_runtime.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/gazebo_smoke_runtime.log)
  * Log runtime Voice node: [.agent/person_tracking/evidence/phase_11/voice_smoke_node.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_11/voice_smoke_node.log)

### Kết quả Đo lường Thực tế (3/3 Ca Đạt 100% PASS, Exit Code = 0):
1. **Ca 1 (Chặn dispatch trước khóa):**
   - Khi hành khách chưa khóa session (`locked = False`), Voice node tiếp nhận intent `navigate` và từ chối xử lý do escort gating: `NO_SESSION_ID`.
   - Kết quả: `dispatched=0, nav_started=0, rejected=True, robot_still=True` $\implies$ **PASSED [OK]**.
2. **Ca 2 (Khóa hợp lệ & Xe di chuyển):**
   - Bơm observation giả lập cử chỉ giơ tay $20	ext{ Hz}$. Tracking Manager khóa session sau $1.5	ext{ s}$ (`locked_session_id=05bdd991-5c1e-4efa-a723-5d2c904a02ea`, FSM: `LOCKED_WAIT_DESTINATION`).
   - Gửi voice intent: Voice node dispatch goal tới Nav2, phát bản tin `NAVIGATION_STARTED` đủ 5 định danh (`session_id`, `mission_id`, `goal_attempt_id`, `nav2_uuid`, `destination_id`).
   - Velocity Guard mở cổng, robot vật lý dịch chuyển trong Gazebo: $d = 0.054	ext{ m} > 0.05	ext{ m}$, $v_{x,\max} = 0.076	ext{ m/s} > 0.05	ext{ m/s}$.
   - Kết quả: `dispatched=1, has_5_ids=True, dist_moved=0.054m, max_vx=0.076m/s` $\implies$ **PASSED [OK]**.
3. **Ca 3 (Mất khách & Hủy goal Nav2):**
   - Ngừng phát `/tracking/observation` khi robot đang hành trình.
   - Velocity Guard kẹp lệnh vận tốc về $0.0	ext{ m/s}$ sau **$0.442	ext{ s}$** ($\le 0.65	ext{ s}$).
   - Tracking Manager phát bản tin `safety_request` abort sau **$1.225	ext{ s}$** ($\le 2.20	ext{ s}$).
   - Voice node nhận abort, gọi `cancel_goal_async` tới Nav2 action server, nhận kết quả terminal `CANCELED`, phát `nav_status` kết thúc khớp đúng `session_id`.
   - Robot dừng hẳn trên `/odom`, FSM trở về `IDLE` (`locked = False`).
   - Kết quả: `dt_zero=0.442s, dt_abort=1.225s, terminal_canceled=True, matching_goal=True, c3_stopped=True, fsm_idle=True` $\implies$ **PASSED [OK]**.

### Phân định Rõ ràng Giữa Phần Mềm và Phần Cứng:
| Hạng mục | Môi trường kiểm chứng | Trạng thái hiện tại | Điều kiện cần để hoàn tất |
|---|---|---|---|
| Logic FSM & Gating Voice | Gazebo + Fake Obs | **PASSED (100%)** | Đã sẵn sàng cho tích hợp hệ thống. |
| Điều phối Vận tốc Guard & Watchdog | Gazebo + Fake Obs | **PASSED (100%)** | Đã sẵn sàng cho tích hợp hệ thống. |
| Quy trình Hủy Goal Nav2 an toàn | Gazebo + Fake Obs | **PASSED (100%)** | Đã sẵn sàng cho tích hợp hệ thống. |
| Ca T06 (Người cắt ngang) | Python Logic / Synthetic | **PARTIAL / CODE_READY** | Chờ video hành khách thật tại hiện trường. |
| Driver Camera Orbbec Astra | Chưa có thiết bị | **HARDWARE_PENDING** | Chờ cắm cáp USB camera, kiểm tra FPS và D2C. |
| Đo đạc sai số cự ly mốc sàn | Chưa có thiết bị | **HARDWARE_PENDING** | Chờ dán mốc sàn thực tế $0.5	ext{ m} - 3.0	ext{ m}$. |
| Profile CPU/VRAM PID thật | Chưa có thiết bị | **HARDWARE_PENDING** | Chờ chạy node thật trên luồng video camera thật. |
| Watchdog mạch microcontroller | Chưa có thiết bị | **HARDWARE_PENDING** | Chờ cấp nguồn và cắm cáp USB mạch vi điều khiển. |
| Đối chiếu 4 lớp vận tốc $v_{\text{ground\_truth}}$ | Chưa có thiết bị | **HARDWARE_PENDING** | Chờ bấm giờ/thước laser thực địa khi cho xe chạy. |

## 7. DANH SÁCH PHÉP ĐO CÒN CHỜ (PENDING MEASUREMENTS)
Cho tới thời điểm người dùng kết nối thiết bị và chạy các bước trên, các phép đo sau đây được ghi nhận ở trạng thái **CHỜ ĐO TRÊN PHẦN CỨNG THỰC TẾ (PENDING HARDWARE MEASUREMENT)**:
1. Thông số nhiễu và độ lệch thực tế của camera Orbbec Astra Pro (FPS thật, dropout rate và profile CPU/VRAM theo PID liên tục).
2. Sai số cự ly thực tế trên các mốc khoảng cách dán sàn ($0.5\text{ m} - 3.0\text{ m}$).
3. Đo kiểm nghiệm thu ca T06 với video hành khách thật tại hiện trường.
4. Thông số timeout watchdog phần cứng thực tế trên mạch microcontroller.
5. Dữ liệu đối chiếu đồng thời giữa `/cmd_vel`, `/odom/unfiltered`, `/odom` và vận tốc đo độc lập $v_{\text{ground\_truth}}$.
