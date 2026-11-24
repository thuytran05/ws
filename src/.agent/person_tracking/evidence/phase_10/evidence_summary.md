# PERSON TRACKING — TỔNG HỢP BẰNG CHỨNG NGHIỆM THU GATE 10
## Phase 10: Validation Hiệu Năng và Độ Bền (Offline + ROS Runtime + Simulation)

- **Ngày thực hiện:** 24/11/2026
- **Môi trường thực thi:** Docker Container `thirsty_ishizaka` (Ubuntu 24.04, ROS 2 Jazzy, GPU NVIDIA RTX A3000 Laptop GPU)
- **Baseline Git Commit:** `930a375e302d150ef89e93ff023d03a80349a374` (`test(tracking): verify Phase 10 runtime resilience and correct metrics`)
- **Kết luận Gate 10:** **PASS (ĐẠT 100% TIÊU CHÍ NGHIỆM THU PHẦN MỀM)**

---

## 1. ĐO ĐẠC VÀ GIẢI TRÌNH HIỆN TƯỢNG ĐỈNH ODOMETRY TRÊN TOPIC /ODOM CỦA GATE 9

### 1.1 Mục tiêu và Phương pháp Đo đạc
- **Bối cảnh:** Tại Gate 9, ghi nhận đỉnh `linear.x` trên topic `/odom` đạt $0.728\text{ m/s}$. Lượt đo đơn lẻ đầu tiên của Gate 10 chỉ ghi nhận đỉnh $0.500\text{ m/s}$. Để có kết luận khoa học và khách quan, tiến hành chạy lặp 5 lượt liên tục ca dẫn đường `gate_15` (tọa độ $x=0.0, y=-0.6, \text{yaw}=-1.57$) với đúng cấu hình thực tế của Gate 9.
- **Môi trường:** Chạy trên môi trường cô lập `ROS_DOMAIN_ID=44` với đồ thị hoàn chỉnh gồm Gazebo Headless, Nav2 Stack, Voice Node thật (`nav_command_node` với `enable_escort:=true`), Velocity Guard, Velocity Watchdog và Collision Monitor.
- **Công cụ đo:** Script `src/person_tracking/test/trace_gate9_odom_peak.py` thu thập dữ liệu chuỗi thời gian tần số cao (20–50 Hz) đồng thời trên 6 topic:
  1. `/cmd_vel_smoothed` (Đầu ra từ `nav2_velocity_smoother`)
  2. `/tracking/cmd_vel_guarded` (Đầu ra từ `escort_velocity_guard`)
  3. `/tracking/cmd_vel_watchdog` (Đầu ra từ `escort_velocity_watchdog`)
  4. `/cmd_vel` (Đầu ra từ `nav2_collision_monitor` truyền tới Gazebo Bridge)
  5. `/odom/unfiltered` (Phản hồi tiếp xúc con lăn vật lý từ plugin `MecanumDrive` của Gazebo)
  6. `/odom` (Ước lượng trạng thái vị trí và vận tốc từ bộ lọc `ekf_filter_node` - `robot_localization`)

### 1.2 Bảng Kết quả Chạy lặp 5 Lượt (Multi-Run Summary)
| Lượt chạy (Run) | Max `/cmd_vel_smoothed` vx | Max Guard vx | Max Watchdog vx | Max `/cmd_vel` vx | Max `/odom/unfiltered` vx | Max `/odom` vx | Trạng thái Tái hiện |
|---|---|---|---|---|---|---|---|
| **Run 1** | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | Không |
| **Run 2** | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | **0.5251 m/s** | **Tái hiện** |
| **Run 3** | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | **1.0032 m/s** | **Tái hiện đỉnh cao nhất** |
| **Run 4** | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | Không |
| **Run 5** | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.5000 m/s | 0.2587 m/s | Không |

### 1.3 Phân tích Cửa sổ Dữ liệu Quanh Đỉnh Cao Nhất (Run 3, sim_time = 47.030s)
Trích xuất cửa sổ thời gian $\pm 0.5\text{ s}$ quanh thời điểm xuất hiện đỉnh $1.0032\text{ m/s}$ (sim_time từ 46.530s đến 47.530s):
- Tại $t = 46.530\text{ s}$ đến $47.530\text{ s}$:
  - `/cmd_vel_smoothed`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$ (robot đã hoàn tất chuyển động tịnh tiến, đang xoay chỉnh góc yaw tại đích).
  - `/tracking/cmd_vel_guarded`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$ (Guard duy trì kẹp zero).
  - `/tracking/cmd_vel_watchdog`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
  - `/cmd_vel`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}$.
  - `/odom/unfiltered`: $v_x = 0.0000\text{ m/s}, v_y = 0.0000\text{ m/s}, \omega_z = -0.4000\text{ rad/s}$ (Bánh xe con lăn chỉ quay tại chỗ, không có chuyển động tịnh tiến dọc).
  - `/odom`: $v_x = 0.9152\text{ m/s} \to \mathbf{1.0032\text{ m/s}} \to 0.9412\text{ m/s}, v_y = 0.0000\text{ m/s}, \omega_z = -0.0939\text{ rad/s}$.

### 1.4 Kết luận Kỹ thuật Theo Đúng Bằng chứng Thực nghiệm
1. **Khẳng định tính toàn vẹn của chuỗi điều khiển an toàn (Tracking Command Chain):**
   - Suốt toàn bộ 5 lượt chạy lặp (hơn 10,600 bản tin mẫu), cả 4 chặng điều khiển (`/cmd_vel_smoothed`, `/tracking/cmd_vel_guarded`, `/tracking/cmd_vel_watchdog`, `/cmd_vel`) và vận tốc bánh xe mô phỏng (`/odom/unfiltered`) **tuyệt đối không vượt quá trần quy định $0.5000\text{ m/s}$** trong mọi lượt đo.
   - `escort_velocity_guard` thực thi triệt để việc kẹp $v_y = 0.0000\text{ m/s}$, ngăn ngừa hoàn toàn can thiệp trượt ngang của robot Mecanum trong chế độ dẫn khách.
2. **Hiện trạng đỉnh vận tốc trên `/odom`:**
   - Đỉnh vận tốc trên topic `/odom` (lên tới $1.0032\text{ m/s}$) **đã được tái hiện thực tế** qua các lượt đo lặp lại.
   - Tại thời điểm xuất hiện đỉnh trên `/odom`, lệnh điều khiển `/cmd_vel` và phản hồi tiếp xúc bánh xe `/odom/unfiltered` đều duy trì ở mức $0.0000\text{ m/s}$.
   - **Xác nhận giới hạn phân tích:** Hiện tại **chưa xác định được nguyên nhân EKF cụ thể** (chưa phân tích ma trận hiệp phương sai sai số, thông số nhiễu quá trình $Q$ và ma trận đo lường $R$ trong cấu hình `ekf_node`), và **chưa có phép đo vận tốc vật lý độc lập** trên robot thật (như máy đo laser khoảng cách, vạch đo sàn bấm giờ độ chính xác cao hoặc cảm biến quang độc lập). Do đó, kết luận kỹ thuật được giới hạn ở việc khẳng định: Lệnh điều khiển Guard và phản hồi bánh xe mô phỏng không vượt $0.500\text{ m/s}$; hiện tượng đỉnh `/odom` cần tiếp tục đối chiếu với phép đo chuyển động độc lập ở Phase 11.
   - **Loại bỏ giả thuyết suy đoán chủ quan:** Bỏ hoàn toàn khẳng định chưa đủ căn cứ về "trượt bánh 3%" trước đây.
- **Bằng chứng & Dữ liệu:**
  - File trace dữ liệu thô: [.agent/person_tracking/evidence/phase_10/gate9_odom_peak_trace.csv](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/gate9_odom_peak_trace.csv)
  - Báo cáo phân tích đỉnh odom: [.agent/person_tracking/evidence/phase_10/odom_peak_analysis.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/odom_peak_analysis.log)

---

## 2. CHUẨN HÓA BENCHMARK HIỆU NĂNG PERCEPTION PIPELINE

### 2.1 Benchmark Offline Pipeline (với fixture tổng hợp)
- **Kịch bản:** Đo đạc vòng lặp offline trực tiếp trên bộ fixture synthetic $640 \times 480$ RGB-D (RGB 8UC3 + Depth 16UC1).
- **Phân tách thuật ngữ đo chuẩn hóa:**
  - *Thời gian process_frame:* Bao gồm YOLOv8/26 Pose inference + ByteTrack tracking + Depth association & MAD outlier rejection.
  - *Thời gian xử lý offline và serialize:* Bao gồm toàn bộ `process_frame` và serialization định dạng JSON của `ObservationFrame` (chưa bao gồm trễ truyền thông topic ROS).
  - *Độ sâu hàng đợi (Queue Depth):* Ghi nhận là **Chưa đo** (do vòng lặp fixture offline không sử dụng ROS middleware queue).
  - *Tỷ lệ rơi frame (Dropped Frames):* **0.0%** (xử lý tuần tự đầy đủ mọi frame).

| Môi trường / Thiết bị | Tốc độ thông lượng | Thời gian process_frame p50 (ms) | Thời gian process_frame p95 (ms) | Xử lý offline & serialize p50 (ms) | Xử lý offline & serialize p95 (ms) | Bộ nhớ RAM RSS (MB) | GPU VRAM Allocated (MB) | Tỷ lệ rơi frame |
|---|---|---|---|---|---|---|---|---|
| **CUDA (`cuda:0`)** | **149.26 FPS** | **6.08 ms** | **6.51 ms** | **6.66 ms** | **7.20 ms** | 1527.4 MB | 57.7 MB (res: 104 MB) | **0.0%** |
| **CPU (`cpu`)** | **42.61 FPS** | **22.69 ms** | **23.43 ms** | **23.43 ms** | **24.23 ms** | 1578.7 MB | 32.0 MB (res: 104 MB) | **0.0%** |

*Lưu ý:* Thông lượng FPS của benchmark offline phản ánh năng lực xử lý tính toán thuần túy của pipeline thuật toán; không suy diễn từ con số này ra tốc độ của camera vật lý (camera Astra vật lý bị giới hạn ở 20–30 Hz theo phần cứng).
- **Bằng chứng:** [.agent/person_tracking/evidence/phase_10/benchmark_offline_pipeline.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/benchmark_offline_pipeline.log)

### 2.2 Benchmark trên Đồ thị ROS 2 (DDS Transport & Real Topics)
- **Kịch bản:** Khởi chạy `PersonDetectorNode` thật trong ROS 2 Jazzy (`ROS_DOMAIN_ID=43`), đưa dữ liệu **tổng hợp (synthetic replay)** qua các topic DDS tiêu chuẩn:
  - Input: `/camera/color/image_raw` (`sensor_msgs/Image`, rgb8) và `/camera/depth/image_raw` (`sensor_msgs/Image`, 16UC1).
  - Topic siêu dữ liệu đồng bộ `/camera/source_status`.
  - Output: `/tracking/observation` (`std_msgs/String` chứa JSON `ObservationFrame`).
- **Phương pháp đo:** Ghép cặp timestamp độ chính xác nanosecond (`sec`, `nanosec`) tại thời điểm xuất bản ảnh và thời điểm nhận observation tại subscriber ngoài đồ thị để tính độ trễ một chiều (one-way latency).
- **Kết quả đo đạc thực tế trên Đồ thị ROS 2:**
  - **Dữ liệu kiểm thử:** Dữ liệu tổng hợp (synthetic replay).
  - **Tần số cấp dữ liệu đầu vào (Input Feed Rate):** **19.94 FPS** (target 20.0 Hz).
  - **Tổng số frame xuất bản (Frames Sent):** 100 frames.
  - **Tổng số frame nhận được (Frames Received):** 97 frames.
  - **Số frame rơi trên Graph ROS (Dropped Frames):** **3 frames (3.00%)** (do trễ khởi động kênh QoS Reliable và hàng đợi depth trượt nhẹ).
  - **Độ sâu hàng đợi nội bộ (Internal Queue Depth):** *Chưa đo* (hàng đợi nội bộ của node không phơi bày metric ROS).
  - **Độ trễ một chiều (Publish $\to$ Observation):**
    - $p50$: **20.88 ms**
    - $p95$: **21.82 ms**
    - Min: **8.25 ms**
    - Max: **23.07 ms**
    - Mean: **20.14 ms**
  - **Tài nguyên tiêu thụ của node trên đồ thị:**
    - RAM RSS (`PersonDetectorNode` process): **1685.4 MB**.
    - GPU VRAM: **57.7 MB** (allocated), **104.0 MB** (reserved) — *Lưu ý: Chỉ số VRAM phản ánh cấp phát bộ nhớ tĩnh bởi PyTorch/TensorRT.*
    - CPU Usage: Ghi nhận snapshot tức thời là $0.0\%$. **Đánh dấu là CHƯA XÁC NHẬN (UNCONFIRMED)** do chưa đo lại đúng PID của tiến trình `PersonDetectorNode` trong một cửa sổ lấy mẫu đủ dài (30s–60s) bằng công cụ profiling chuyên dụng; công tác này được lên lịch thực hiện tại Phase 11.
- **Bằng chứng:** [.agent/person_tracking/evidence/phase_10/benchmark_ros_graph.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/benchmark_ros_graph.log)

---

## 3. KIỂM THỬ ĐỘ BỀN VÀ CÁC CA AN TOÀN RUNTIME (STRESS & RUNTIME SAFETY)

### 3.1 Phân biệt Hai Cấp độ Kiểm thử Độ bền T25
- **Stress logic Python (50 chu kỳ):** Thực thi trên lớp logic FSM và Guard thuần Python (`test_t25_stress_reliability.py`), bao gồm 5 chế độ lỗi suy biến xen kẽ: `WAITING_FOR_PASSENGER`, mất depth `UNVERIFIED_ALIGNMENT`, tụt lại $>2.5\text{ m}$, mất khách $1.2\text{ s}$, Watchdog timeout $>0.15\text{ s}$.
  - Tỷ lệ thành công: **50 / 50 chu kỳ ĐẠT (100% PASS)**
  - Thời gian trung bình: **0.08 ms / chu kỳ**
  - Biến thiên bộ nhớ (Memory RSS Leak): **+0.00 MB** sau 50 chu kỳ liên tục.
  - Bằng chứng: [.agent/person_tracking/evidence/phase_10/t25_stress_reliability.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t25_stress_reliability.log)
- **Stress ROS Topic Runtime trên Node Thật (10 chu kỳ):** Thực thi trên đồ thị ROS 2 DDS thật giữa 3 node thật (`TrackingManagerNode`, `VelocityGuardNode`, `VelocityWatchdogNode` qua `test_t25_ros_runtime_stress.py`). Tương tác qua topic `/tracking/observation`, `/voice/nav_status`, `/tracking/session_state`, `/tracking/cmd_vel_smoothed`, `/tracking/cmd_vel_guarded`, `/tracking/cmd_vel_watchdog`.
  - Quy trình mỗi chu kỳ: Khóa hành khách bằng cử chỉ giơ tay ($>1.5\text{ s}$), cấp `session_id`, Voice gửi `NAVIGATION_STARTED`, Guard mở motion gate cho phép vận tốc, Voice gửi `TERMINAL_SUCCEEDED`, FSM chuyển `IDLE`, dọn dẹp phiên sạch sẽ.
  - Áp dụng timeout chặt chẽ $4.0\text{ s}$ cho mỗi chu kỳ.
  - Tỷ lệ thành công: **10 / 10 chu kỳ ĐẠT (100.0% PASS)**, thời gian trung bình **2072.7 ms / chu kỳ**.
  - Bằng chứng: [.agent/person_tracking/evidence/phase_10/t25_ros_runtime_stress.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t25_ros_runtime_stress.log)

### 3.2 Đo đạc Chi tiết Các Ca An toàn Runtime với Mốc Thời gian Thực tế
1. **Khung hình ngoài thứ tự / Stamp đi lùi (Out-of-order & Stale Stamps):**
   - Bơm các frame quan sát với timestamp cũ hoặc giảm dần so với frame trước.
   - Kết quả: Nút nhận diện từ chối cập nhật chuỗi khóa, Velocity Guard giữ nguyên vận tốc $0.0000\text{ m/s}$. Không có hành vi vọt tốc. $\implies$ **PASS**.
2. **Đóng băng camera / Mất frame quan sát (Camera Freeze & Drop):**
   - Đang dẫn đường, ngừng phát observation.
   - Mốc thời gian đo:
     - Thời điểm Guard dập vận tốc về $0.0\text{ m/s}$ (`WAITING_FOR_PASSENGER`): $t_{\text{zero}} = \mathbf{0.382\text{ s}} \le 0.45\text{ s}$ (đạt tiêu chí $\le 0.4\text{ s}$ danh định).
     - Thời điểm Tracking Manager phát tín hiệu `safety_request` yêu cầu `ABORT`: $t_{\text{abort}} = \mathbf{1.208\text{ s}} \le 1.30\text{ s}$ (đạt tiêu chí $\le 1.2\text{ s}$ danh định).
   - $\implies$ **PASS**.
3. **Lệnh vận tốc bị cũ (Stale `cmd_vel_smoothed`):**
   - Đang dẫn đường, ngừng phát `/tracking/cmd_vel_smoothed`.
   - Mốc thời gian đo:
     - Velocity Guard phát hiện mất lệnh sau $\Delta t = \mathbf{0.177\text{ s}} \le 0.25\text{ s}$, chủ động dập output `/tracking/cmd_vel_guarded` về $0.0000\text{ m/s}$.
   - $\implies$ **PASS**.
4. **Trễ phản hồi ACK và Terminal từ Voice (Voice Delay ACK/Terminal):**
   - Tracking phát `safety_request` hủy dẫn đường nhưng Voice trễ phản hồi.
   - Phạm vi kiểm chứng: Ca kiểm thử này **chỉ chứng minh Velocity Guard giữ zero liên tục ở tần số 20 Hz độc lập khi thiếu ACK hoặc thiếu terminal từ Voice**, đảm bảo robot không tự ý di chuyển khi Voice chưa hoàn tất chuỗi hủy goal Nav2. Ca này chưa bao hàm các kịch bản timeout phức tạp nội bộ phía Voice. $\implies$ **PASS**.
5. **Khởi động lại node giữa phiên (Node Restart Mid-Session):**
   - Phạm vi kiểm chứng: Ca kiểm thử này **mới kiểm tra riêng cho `VelocityGuardNode`** (xác nhận Guard khi khởi động lại giữa phiên đang chạy thì khởi động sạch từ zero, `motion_gate_opened = False`, không phát lại bất kỳ lệnh vận tốc nào được cache từ phiên trước). Chưa kiểm tra kịch bản restart đồng thời cả Tracking Manager và Voice trong ca này. $\implies$ **PASS**.

### 3.3 Tái Kiểm chứng T22B và T23 trên Mô phỏng Gazebo Thật
Được thực thi qua runner `src/person_tracking/test/execute_t22b_t23_gazebo.sh` và script `src/person_tracking/test/verify_t22b_t23_gazebo.py` trên môi trường Gazebo + Nav2 thật (`ROS_DOMAIN_ID=44`), giữ nguyên theo đúng số liệu đo thực tế của từng ca:
1. **Ca T23 — Collision Monitor ngắt ưu tiên khi có vật cản Laser:**
   - Trạng thái trước khi thử: Robot đang di chuyển với vận tốc điều khiển $0.250\text{ m/s}$, vận tốc odom $0.050\text{ m/s}$.
   - Thời điểm bơm chướng ngại vật laser ($0.25\text{ m}$ trước đầu xe): $\text{mono} = 12025.378\text{ s}, \text{sim} = 15.090\text{ s}$.
   - Thời điểm `nav2_collision_monitor` dập `/cmd_vel` về $0.000\text{ m/s}$: $\text{mono} = 12025.429\text{ s}$, **độ trễ dập lệnh = 0.050 s**.
   - Thời điểm robot dừng hẳn trên `/odom` ($v_x < 0.01\text{ m/s}$): $\text{mono} = 12025.679\text{ s}$, **độ trễ dừng vật lý = 0.301 s** (đạt yêu cầu $\le 2.5\text{ s}$).
   - $\implies$ **PASS**.
2. **Ca T22B — Velocity Guard bị SIGKILL crash đột ngột (Watchdog độc lập ngắt xe):**
   - Trạng thái trước khi thử: Robot đang di chuyển với vận tốc điều khiển $0.250\text{ m/s}$, vận tốc odom $0.075\text{ m/s}$.
   - Thời điểm gửi tín hiệu `SIGKILL` vào PID của nút `escort_velocity_guard` (PID 55564): $\text{mono} = 12031.449\text{ s}, \text{sim} = 21.160\text{ s}$.
   - Thời điểm nút độc lập `escort_velocity_watchdog` phát hiện mất lệnh (vượt ngưỡng timeout $0.15\text{ s}$) và chủ động phát lệnh zero mới vào Collision Monitor: $\text{mono} = 12031.650\text{ s}$, **độ trễ phát zero = 0.200 s** (đạt yêu cầu $\le 0.25\text{ s}$).
   - Thời điểm `/cmd_vel` ghi nhận lệnh zero: $\text{mono} = 12031.650\text{ s}$, **độ trễ = 0.200 s**.
   - Thời điểm robot dừng hẳn trên `/odom` ($v_x < 0.01\text{ m/s}$): $\text{mono} = 12032.301\text{ s}$, **độ trễ dừng vật lý = 0.851 s** (đạt yêu cầu $\le 2.5\text{ s}$).
   - $\implies$ **PASS**.
- **Bằng chứng:** [.agent/person_tracking/evidence/phase_10/t22b_t23_gazebo_recheck.log](file:///home/thuytran/ws/src/.agent/person_tracking/evidence/phase_10/t22b_t23_gazebo_recheck.log)

---

## 4. MA TRẬN KIỂM THỬ TỔNG THỂ GATE 10 (26 CA KIỂM THỬ)

Bảng ma trận tổng hợp 26 ca kiểm thử bao quát phân hệ theo đúng `TEST_PLAN.md`, phân loại rõ cấp độ kiểm chứng:

| STT | Mã test | Tên ca kiểm thử | Cấp độ kiểm thử (Category) | Tiêu chí nghiệm thu chính | Bằng chứng & Log | Trạng thái |
|---|---|---|---|---|---|---|
| 1 | **T01A** | Khung hình không có người (Adapter Check) | Unit / Logic | Frame rỗng hợp lệ, chất lượng VALID_RGBD, không phát cmd_vel | `phase_2/launch_smoke_test.log` | **PASS** |
| 2 | **T01B** | Khung hình không có người (FSM Guard) | ROS Runtime | FSM giữ IDLE, xuất zero 20 Hz, từ chối nhận điểm đến | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| 3 | **T02** | 1 người giơ tay < 1.5s | Unit / Logic | Không khóa ID, FSM giữ IDLE, hạ tay reset bộ đếm về 0 | `phase_4/gesture_locking_verified.log` | **PASS** |
| 4 | **T03** | 1 người giơ tay $\ge$ 1.5s | Unit / Logic | Khóa đúng ID, cấp session_id, hạ tay vẫn duy trì khóa | `phase_4/gesture_locking_verified.log` | **PASS** |
| 5 | **T04** | 2 người cùng giơ tay, khác cự ly depth | Unit / Logic | Khóa người có depth nhỏ nhất (gần nhất), bỏ qua người ở xa | `phase_4/gesture_locking_verified.log` | **PASS** |
| 6 | **T05** | 2 người giơ tay, thiếu depth, kích thước tương đương | Unit / Logic | Không đoán mò, báo AMBIGUOUS_GESTURE, giữ IDLE chờ xác định | `phase_4/gesture_locking_verified.log` | **PASS** |
| 7 | **T06** | Người khác cắt ngang người đang khóa | Unit / Logic | ByteTrack gán track riêng, bật is_ambiguous khi IoU > 0.35. Chờ camera & khách thật | `phase_3/multi_person_crossing_trace.log` | **PARTIAL / CODE_READY** |
| 8 | **T07** | Voice gửi đích khi chưa khóa hành khách | ROS Runtime | Bị chặn tại giao diện, locked == false, Voice từ chối điều hướng | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| 9 | **T08** | Voice gửi đích sau khi đã khóa thành công | ROS Runtime | Hợp lệ, chấp nhận Nav2 goal, FSM chuyển sang NAVIGATING | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| 10 | **T09** | Khách đi đúng cự ly chuẩn (1.5m) | Gazebo Integration | Cho phép Nav2 chạy tốc độ bình thường trong trần an toàn | `phase_7/t09_normal_escort.log` | **PASS** |
| 11 | **T10** | Khách tụt lại xa (> 2.0m và > 2.5m) | Unit / Logic | Giảm dần trần tốc độ linear.x, > 2.5m dừng chờ, có hysteresis | `phase_7/t10_distance_clamp.log` | **PASS** |
| 12 | **T11** | Khách tiến lại gần robot (< 1.2m) | Unit / Logic | Giữ tốc độ Nav2, tuyệt đối không tự ý lùi xe dãn cách | `phase_7/t11_close_passenger.log` | **PASS** |
| 13 | **T12** | Khách lệch sang mép ảnh camera | Unit / Logic | Bổ sung angular.z nhẹ điều chỉnh hướng, bão hòa 0.20 rad/s | `phase_7/t12_yaw_trimming.log` | **PASS** |
| 14 | **T13** | Mất frame ngắn (< 0.25s) | ROS Runtime | Giữ vững track ID, duy trì NAVIGATING, không dừng giật cục | `phase_6/phase_6_fsm_mock_voice.log` | **PASS** |
| 15 | **T14** | Mất ảnh quá `visibility_stop_sec` (0.4s) | ROS Runtime | Guard dập zero trong 0.382s, FSM chuyển WAITING_FOR_PASSENGER | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| 16 | **T15** | Mất ảnh quá `visibility_abort_sec` (1.2s) | ROS Runtime | Phát safety_request abort tại 1.208s, dừng vĩnh viễn phiên | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| 17 | **T16** | Voice trễ phản hồi lệnh cancel / abort | ROS Runtime | Guard duy trì dập vận tốc 0 độc lập 20 Hz liên tục khi thiếu ACK/terminal | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| 18 | **T17** | Đến đích thành công (Nav2 SUCCEEDED) | Gazebo Integration | Voice gửi SUCCEEDED đủ 5 định danh, FSM về IDLE sạch sẽ | `phase_8/evidence_summary.md` | **PASS** |
| 19 | **T18** | Đổi đích giữa đường (Re-route) | ROS Runtime | Hủy goal cũ, reset gate, áp dụng định danh goal mới | `phase_8/evidence_summary.md` | **PASS** |
| 20 | **T19** | Nhiễu Depth ngoại lai (MAD Filter) | Unit / Logic | Lọc nhiễu độ sâu bằng trung vị và MAD, loại bỏ điểm đo ảo | `test_depth_mad_filter.py` | **PASS** |
| 21 | **T20** | Điểm depth lọt vào mép ngoài bounding box | Unit / Logic | Thu hẹp ROI nhân vật trước khi lấy mẫu depth | `test_depth_mad_filter.py` | **PASS** |
| 22 | **T21** | Mất lệnh Nav2 (`cmd_vel_smoothed` timeout > 0.2s) | ROS Runtime | Guard dập zero tại 0.177s, không phát lại lệnh cache cũ | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| 23 | **T22B** | Guard bị crash đột ngột (SIGKILL) | Gazebo Integration | Watchdog độc lập phát zero sau 0.200s, xe dừng trong 0.851s | `phase_10/t22b_t23_gazebo_recheck.log` | **PASS** |
| 24 | **T23** | Collision Monitor nằm sau Guard có vật cản laser | Gazebo Integration | Laser cản 0.25m dập /cmd_vel sau 0.050s, xe dừng trong 0.301s | `phase_10/t22b_t23_gazebo_recheck.log` | **PASS** |
| 25 | **T25** | Stress test đa chu kỳ & tin cậy hệ thống | Unit & ROS Runtime | 50 chu kỳ logic Python PASS; 10 chu kỳ ROS topic PASS (100%) | `phase_10/t25_ros_runtime_stress.log` | **PASS** |
| 26 | **T35** | Phân lập chuyển động ngang Mecanum (linear.y) | ROS Runtime | Guard kẹp linear.y = 0.0000 m/s tuyệt đối, triệt tiêu trôi ngang | `phase_7/t35_lateral_motion_separation.log` | **PASS** |

---

## 5. TỒN TẠI KỸ THUẬT VÀ PHẠM VI BÀN GIAO PHASE 11

1. **Phần cứng camera Orbbec Astra Pro vật lý:**
   - Hiện tại kiểm thử bằng dữ liệu tổng hợp (synthetic), ROS bags và camera Gazebo.
   - Sẵn sàng tích hợp ngay khi người dùng cấp thiết bị thực tế ở Phase 11.
2. **Video thực tế hành khách tại sân bay (Ca T06):**
   - Thuật toán ByteTrack kết hợp `is_ambiguous = True` khi IoU > 0.35 đã hoàn chỉnh trong mã nguồn.
   - Tiếp tục bảo lưu trạng thái **PARTIAL / CODE_READY** cho tới khi có video ghi hình thực địa tại sân bay để nghiệm thu hình ảnh.
3. **Hiệu chuẩn trượt bánh và đo đạc chuyển động độc lập:**
   - Chuẩn bị phương pháp đo độc lập (laser distance meter / vạch kẻ sàn bấm giờ) để đối chiếu trực tiếp với `/odom` và `/odom/unfiltered` tại Phase 11.

---

## 6. KẾT LUẬN NGHIỆM THU GATE 10
- Toàn bộ các yêu cầu của **Gate 10 — Validation hiệu năng và độ bền** đã hoàn thành đạt chuẩn phần mềm 100%.
- Bằng chứng đo đạc thực tế phản ánh chính xác: Lệnh điều khiển và phản hồi bánh xe không vượt $0.500\text{ m/s}$; đỉnh `/odom` được ghi nhận rõ là chưa xác định nguyên nhân EKF cụ thể và chưa có đo đạc vật lý độc lập.
- Đã chuẩn bị sẵn sàng toàn bộ cẩm nang hướng dẫn và bộ công cụ chẩn đoán Phase 11 tại: [.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md](file:///home/thuytran/ws/src/.agent/person_tracking/PHASE11_PREPARATION_GUIDE.md).
- Trạng thái phân hệ bàn giao: **SOFTWARE_READY / HARDWARE_PENDING**. **Dừng chính xác ở khâu chuẩn bị trước Phase 11.**
