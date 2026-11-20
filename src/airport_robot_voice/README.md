# airport_robot_voice

Hệ thống trợ lý giọng nói (STT + NLU rule-based tiếng Việt + TTS) cho robot hỗ trợ hành khách sân bay trên nền tảng **ROS 2 Jazzy**. Hoạt động hoàn toàn **offline**, không phụ thuộc LLM hay API đám mây.

> [!TIP]
> **Tài liệu Cài đặt & Vận hành Chi tiết (Dành cho người mới)**:
> Vui lòng tham khảo file [COMMANDS_VOICE.md](file:///home/thuytran/ws/src/.agent/COMMANDS_VOICE.md) để biết đầy đủ từng bước từ cấu hình Docker, cài đặt dependency Python/ROS/Audio, tải model offline Vosk & Piper, kết nối micro/loa thật, build workspace và các lệnh chạy thử chi tiết theo từng Terminal.

---

## 1. Kiến trúc hệ thống

Hệ thống gồm 4 node chính giao tiếp qua ROS 2 topic và action:

```
[Người dùng]
     │ (Giọng nói / Bàn phím / WAV)
     ▼
[stt_node] ────── /voice/text ──────► [intent_node] ────── /voice/say ──────► [tts_node] ──► [Loa / Log]
(Vosk/ALSA)                              (NLU)             (std_msgs/String)   (Log/Piper/eSpeak)
                                           │
                                     /voice/intent
                                 (JSON Schema v1.0)
                                           │
                                           ▼
                                  [nav_command_node] ◄──── /voice/nav_state (QoS Transient Local)
                                     (Vòng đời Nav2)
                                           │
                                  NavigateToPose / ComputePathToPose
                                           │
                                           ▼
                                      [Nav2 Stack]
```

### Hai chế độ vận hành:
1. **Chế độ Voice độc lập (`use_nav:=false`)**:
   - Hoạt động ngay cả khi Nav2 chưa chạy.
   - Trích xuất chính xác `target_id` hoặc `category` và phát dữ liệu có cấu trúc ra `/voice/intent`.
   - `intent_node` trả lời câu hỏi thông tin (Wi-Fi, sạc pin, giờ mở cửa, chào hỏi, trợ giúp), và thông báo rõ: *"Đã xác định yêu cầu tới [điểm đến], nhưng hệ thống điều hướng chưa sẵn sàng"*.
2. **Chế độ Voice kết hợp Navigation (`use_nav:=true`)**:
   - `nav_command_node` đóng vai trò cổng điều phối nhiệm vụ và quản lý trạng thái di chuyển (IDLE, RESOLVING, ANNOUNCING, STARTING, NAVIGATING, PAUSING, PAUSED, CANCELING).
   - Tiếp nhận lệnh: `navigate` (dẫn đường), `pause` (tạm dừng), `resume` (đi tiếp), `cancel` (hủy chuyến), `replace` (đổi đích khi đang đi), `status` (hỏi vị trí/khoảng cách còn lại).
   - Chọn điểm tối ưu trong nhóm (ví dụ `toilet`) theo độ dài đường đi khả dụng từ `ComputePathToPose`.

---

## 2. Chuẩn bị môi trường

Mọi thao tác build và chạy được thực hiện trực tiếp trong terminal của VS Code (đang chạy trực tiếp trong môi trường ROS 2 Jazzy tại workspace `/home/ws`):

```bash
# 1. Source môi trường ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build package
cd /home/ws
colcon build --packages-select airport_robot_voice --symlink-install
source /home/ws/install/setup.bash
```

### Các dependency âm thanh đã được tích hợp sẵn:
- STT: `vosk` và `sounddevice` (`pip install --user vosk sounddevice`).
- Model Vosk tiếng Việt: `/opt/vosk/vosk-model-small-vn-0.4`.
- TTS eSpeak: `espeak-ng` (`sudo apt install alsa-utils espeak-ng`) hoặc chế độ `log` (không cần thiết bị âm thanh).
- TTS Piper: binary `/usr/local/bin/piper`, model `/opt/piper/vi_VN-vais1000-medium.onnx`.
- Thiết bị âm thanh: `/dev/snd` đã được mount trực tiếp vào container.

---

## 3. Hướng dẫn chạy chi tiết theo từng Terminal

> [!NOTE]
> Do bạn thao tác trực tiếp trên VS Code Dev Container, các Terminal mở trong VS Code (`Ctrl + ` ` hoặc `Ctrl + Shift + ` `) đều đã ở sẵn bên trong container tại `/home/ws`. Bạn **không cần dùng `docker exec`**, chỉ cần mở terminal và chạy trực tiếp các lệnh bên dưới.

### Cách 1: Chạy toàn bộ hệ thống bằng ROS 2 Launch

#### Chế độ A: Gõ bàn phím, log-only (Khuyến nghị khi test / không cần micro/loa)
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_assistant.launch.py mode:=keyboard tts_engine:=log use_nav:=false
```

#### Chế độ B: Gõ bàn phím kết hợp node Navigation (có quản lý action Nav2)
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_assistant.launch.py mode:=keyboard tts_engine:=log use_nav:=true
```

#### Chế độ C: Dùng micro thật và phát âm thanh qua eSpeak-ng
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_assistant.launch.py mode:=mic tts_engine:=espeak use_nav:=true
```

#### Chế độ D: Kiểm thử lặp lại bằng file âm thanh WAV
```bash
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_assistant.launch.py mode:=wav wav_file:=/tmp/test_a3.wav tts_engine:=log use_nav:=false
```

---

### Cách 2: Chạy riêng từng node trong từng Terminal (Dễ debug và tương tác stdin)

Mở các terminal độc lập trong VS Code (`Ctrl + Shift + ` `):

#### Terminal 1: TTS Node (Phát âm thanh hoặc ghi log)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice tts_node --ros-args -p engine:=log
```

#### Terminal 2: Intent Node (Lõi NLU hiểu ý định)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice intent_node
```

#### Terminal 3: Nav Command Node (Cổng điều phối Nav2)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice nav_command_node
```

#### Terminal 4: STT Node (Nhập văn bản từ bàn phím)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
```
*(Gõ câu thoại như `dẫn tôi đến cổng A3`, `quầy check in ở đâu`, `nhà vệ sinh`, `dừng lại`, `đi tiếp` rồi nhấn Enter).*

#### Terminal 5: Theo dõi trạng thái và sự kiện hệ thống (Monitor)
```bash
source /opt/ros/jazzy/setup.bash

# Xem câu trả lời phát ra loa
ros2 topic echo /voice/say

# Xem yêu cầu điều hướng có cấu trúc JSON
# ros2 topic echo /voice/intent

# Xem trạng thái nhiệm vụ Nav2 latched (Transient Local QoS)
# ros2 topic echo /voice/nav_state --qos-durability transient_local
```

---

## 4. Định dạng dữ liệu (Schema JSON)

### Topic `/voice/intent` (std_msgs/String)
Xuất phát từ `intent_node`:
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

### Topic `/voice/nav_state` (std_msgs/String, QoS Transient Local)
Công bố bởi `nav_command_node`:
```json
{
  "schema_version": "1.0",
  "node_online": true,
  "mission_state": "NAVIGATING",
  "mission_id": "mission_1",
  "current_target_id": "gate_a3",
  "current_target_name": "cổng A3",
  "distance_remaining": 12.5,
  "eta_seconds": 18,
  "last_feedback_time": 1700000010.5,
  "timestamp": 1700000011.0
}
```

---

## 5. Danh mục địa điểm (`config/places.json`) và Lấy tọa độ bản đồ

File `config/places.json` chứa dữ liệu demo được gắn cờ `"_demo_data": true`:
- Cổng bay: `gate_a1`, `gate_a2`, `gate_a3`, `gate_a4`, `gate_b1`, `gate_b2`, `gate_15`.
- Quầy làm thủ tục: `checkin`.
- Nhà vệ sinh (nhiều điểm cùng category `toilet`): `toilet_1` (sảnh A), `toilet_2` (sảnh B).
- Điểm tiện ích: `food_court`, `baggage`, `info_desk`, `atm`, `security`, `home` (trạm sạc).
- Thông tin dịch vụ: `wifi` (Airport-Free-WiFi), `charging` (sạc pin), `hours` (mở cửa 24/7).

### Cách lấy toạ độ thực tế trên bản đồ Nav2 / Gazebo:
1. Chạy Nav2 và RViz: Điều khiển robot tới vị trí cần lưu toạ độ.
2. Lấy vị trí:
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
3. Đọc `position.x`, `position.y` và tính góc yaw từ quaternion:
   ```python
   yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
   ```
4. Thêm mục vào `config/places.json` với cấu trúc:
   ```json
   "gate_c1": {
     "name": "cổng C1",
     "desc": "tầng 2, sảnh C",
     "aliases": ["c1", "c một", "cổng c1"],
     "category": "gate",
     "enabled": true,
     "x": 5.2, "y": -4.1, "yaw": 1.57,
     "frame_id": "map"
   }
   ```

---

## 6. Chạy kiểm thử tự động (Test Suite)

Package đi kèm bộ test toàn diện 41 kiểm thử pytest và 61 kịch bản NLU:

```bash
# 1. Chạy kiểm thử tự động NLU thuần Python (không cần ROS):
python3 src/airport_robot_voice/scripts/test_nlu.py

# 2. Chạy toàn bộ pytest suite trong môi trường ROS 2 Jazzy:
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
python3 -m pytest src/airport_robot_voice/test/ -v
```

---

## 7. Xử lý sự cố thường gặp (Troubleshooting)

| Sự cố | Nguyên nhân | Cách khắc phục |
|---|---|---|
| `Thiếu thư viện âm thanh` | Chưa cài `sounddevice`/`vosk` | Chạy `pip install --user --break-system-packages sounddevice vosk` hoặc dùng `mode:=keyboard`. |
| `Không nạp được model Vosk` | Thiếu thư mục model | Kiểm tra `/opt/vosk/vosk-model-small-vn-0.4` đã được tải về và giải nén đầy đủ. |
| `Hệ thống điều hướng chưa sẵn sàng` | `nav_command_node` hoặc Nav2 chưa chạy | Khởi động `nav_command_node` hoặc kiểm tra action server `navigate_to_pose`. |
| `Không có tiếng ra loa trong Docker` | Docker chưa cấu hình audio pipeline hoặc không có loa ngoài | Dùng `tts_engine:=log` (chế độ chỉ log, vẫn xác nhận và chạy thông suốt toàn bộ lifecycle). |
| `STT không nhận ký tự khi chạy launch keyboard` | Launch wrapper nuốt stdin | Chạy `stt_node` trực tiếp trong terminal riêng bằng lệnh `ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard`. |
