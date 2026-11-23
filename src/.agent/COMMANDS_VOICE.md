# HƯỚNG DẪN CÀI ĐẶT VÀ VẬN HÀNH AIRPORT_ROBOT_VOICE

Tài liệu này cung cấp toàn bộ quy trình thiết lập môi trường, chuẩn bị model offline, build, vận hành và kiểm thử package `airport_robot_voice` trên ROS 2 Jazzy.

---

## 1. Yêu cầu môi trường

### 1.1 Thông số hệ thống
- **Hệ điều hành**: Ubuntu 24.04 Noble (ROS 2 Jazzy Jalisco).
- **Môi trường thao tác**: Mở trực tiếp bằng VS Code Dev Container.
- **Python**: Python 3.12 (mặc định trong ROS 2 Jazzy).
- **Workspace**: `/home/ws` (mã nguồn package tại `/home/ws/src/airport_robot_voice`).

### 1.2 Môi trường thực thi trong VS Code Dev Container
> [!IMPORTANT]
> **Không cần dùng `docker exec`**: Do bạn đang thao tác trực tiếp trên VS Code đã kết nối vào container ROS 2 Jazzy, mọi Terminal mở trong VS Code (`Ctrl + ` ` hoặc `Ctrl + Shift + ` `) đều đã ở sẵn bên trong container tại thư mục workspace `/home/ws`. Tất cả các lệnh trong tài liệu này đều được gõ trực tiếp vào Terminal của VS Code.

### 1.3 Đường dẫn và cấu hình gắn kèm
- **Đường dẫn Workspace**: `/home/ws`.
- **Đường dẫn mã nguồn package**: `/home/ws/src/airport_robot_voice`.
- **Thiết bị gắn kèm**: Thư mục `/dev` và `/dev/snd` đã được mount sẵn vào container để hỗ trợ micro và loa.

---

## 2. Lấy mã nguồn và chuẩn bị môi trường

### 2.1 Kiểm tra mã nguồn và nhánh
Mở Terminal trong VS Code và kiểm tra:

```bash
cd /home/ws/src/airport_robot_voice
git checkout master
git status
```
*Dấu hiệu thành công*: Thấy nhánh `master` và cây thư mục package đầy đủ.

---

### 2.2 Mở Terminal làm việc trong VS Code
Trên thanh menu VS Code, chọn **Terminal -> New Terminal** (hoặc nhấn tổ hợp phím `Ctrl + Shift + ` `).
Dấu nhắc terminal hiển thị:
```bash
thuytran@thirsty_ishizaka:/home/ws$
```
Bạn sẵn sàng thực thi trực tiếp các lệnh tiếp theo mà không cần qua bất kỳ lệnh trung gian nào.

---

### 2.3 Cài đặt dependency hệ thống và âm thanh

```bash
# Cập nhật danh sách gói và cài đặt alsa-utils, espeak-ng, portaudio
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  alsa-utils \
  espeak-ng \
  libportaudio2 \
  python3-pyaudio \
  python3-pip
```
*Dấu hiệu thành công*: Lệnh `which arecord aplay espeak-ng` trả về đường dẫn nhị phân `/usr/bin/arecord`, `/usr/bin/aplay`, `/usr/bin/espeak-ng`.

---

### 2.4 Cài đặt thư viện Python (Tuân thủ PEP 668 trên Ubuntu 24.04)
Trên Ubuntu 24.04 (Noble), Python áp dụng quy chuẩn PEP 668 quản lý môi trường. Sử dụng cờ `--user --break-system-packages` để cài đặt cục bộ cho user `thuytran`:

```bash
# Cài đặt Vosk, SoundDevice và Piper TTS
pip install --user --break-system-packages \
  vosk \
  sounddevice \
  piper-tts
```

Tạo liên kết nhị phân cho `piper` để gọi trực tiếp từ CLI:
```bash
# Tạo symlink cho binary piper vào /usr/local/bin
sudo ln -sf ~/.local/bin/piper /usr/local/bin/piper
```

---

### 2.5 Kiểm tra dependencies sau khi cài
```bash
# Kiểm tra import Python
python3 -c "
for mod in ['vosk', 'sounddevice', 'pyaudio', 'piper', 'rclpy', 'nav2_msgs']:
    __import__(mod)
    print(f'  [OK] {mod}')
"
```
*Dấu hiệu thành công*: Cả 6 module đều in `[OK]`.

---

## 3. Chuẩn bị STT và TTS Offline

Hệ thống hoạt động **100% offline**, không cần mạng khi vận hành. Bước này chỉ cần kết nối Internet **một lần duy nhất** để tải model ban đầu.

### 3.1 Tải Model Vosk tiếng Việt (STT)
- **Nguồn**: Alphacephei Vosk Models (Chính thức).
- **Phiên bản**: `vosk-model-small-vn-0.4` (~43 MB).
- **Đường dẫn mặc định**: `/opt/vosk/vosk-model-small-vn-0.4`.

```bash
# Tạo thư mục và phân quyền
sudo mkdir -p /opt/vosk && sudo chown -R thuytran:thuytran /opt/vosk
cd /opt/vosk

# Tải và giải nén model Vosk
python3 -c '
import urllib.request, zipfile, os
url = "https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip"
zip_path = "/opt/vosk/model.zip"
print("Đang tải model Vosk...")
urllib.request.urlretrieve(url, zip_path)
print("Đang giải nén...")
with zipfile.ZipFile(zip_path, "r") as z:
    z.extractall("/opt/vosk")
os.remove(zip_path)
print("Hoàn tất:", os.listdir("/opt/vosk"))
'
```

*Kiểm tra*:
```bash
# Kiểm tra model Vosk
python3 -c "from vosk import Model; m = Model('/opt/vosk/vosk-model-small-vn-0.4'); print('Vosk model nạp thành công!')"
```
*Dấu hiệu thành công*: In ra thông số giải mã Vosk và dòng `Vosk model nạp thành công!`.

---

### 3.2 Tải Model Piper tiếng Việt (TTS giọng đọc tự nhiên)
- **Nguồn**: Rhasspy HuggingFace Piper Voices (Chính thức).
- **Phiên bản**: `vi_VN-vais1000-medium` (~61 MB ONNX).
- **Đường dẫn mặc định**: `/opt/piper/vi_VN-vais1000-medium.onnx`.

```bash
# Tạo thư mục và tải model Piper cùng cấu hình
sudo mkdir -p /opt/piper && sudo chown -R thuytran:thuytran /opt/piper
cd /opt/piper

curl -L -s -o vi_VN-vais1000-medium.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx

curl -L -s -o vi_VN-vais1000-medium.onnx.json \
  https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx.json

ls -lh /opt/piper
```

*Kiểm tra tổng hợp thử nghiệm Piper*:
```bash
# Thử phát âm một câu tiếng Việt ra file WAV
echo "Tôi sẽ dẫn bạn đến cổng a ba." | piper --model /opt/piper/vi_VN-vais1000-medium.onnx --output_file /tmp/test_piper.wav
ls -lh /tmp/test_piper.wav
```
*Dấu hiệu thành công*: File `/tmp/test_piper.wav` được tạo ra với dung lượng khoảng 70 KB.

---

### 3.3 Kiểm tra eSpeak-ng (TTS giọng máy siêu nhẹ)
```bash
# Thử tổng hợp qua eSpeak-ng
espeak-ng -v vi "Xin chào robot sân bay" -w /tmp/test_espeak.wav
ls -lh /tmp/test_espeak.wav
```
*Dấu hiệu thành công*: File `/tmp/test_espeak.wav` được tạo ra với dung lượng khoảng 67 KB.

---

## 4. Kết nối Micro và Loa

### 4.1 Cấu hình thiết bị Âm thanh
Môi trường Dev Container hiện tại đã được cấu hình sẵn các cờ mount thiết bị âm thanh (`--device /dev/snd -v /dev:/dev`), cho phép các node trong container truy cập trực tiếp vào card âm thanh và micro của máy.

### 4.2 Liệt kê và chọn thiết bị
Chạy lệnh kiểm tra phần cứng trong container:

```bash
# Liệt kê thiết bị thu âm (Capture)
arecord -l

# Liệt kê thiết bị phát âm (Playback)
aplay -l

# Liệt kê danh sách qua sounddevice
python3 -c "import sounddevice; print(sounddevice.query_devices())"
```

*Cách chọn thiết bị*:
- Nếu dùng micro mặc định của máy tính: để tham số `device:=-1`.
- Nếu dùng micro ngoài hoặc card USB: xem số index từ `sounddevice.query_devices()` và truyền `device:=<index>` (ví dụ `device:=4`).

### 4.3 Kiểm tra thu và phát âm thực tế
```bash
# Thu âm thử 3 giây từ DMIC16kHz (Card 1, Dev 7 - chuẩn trực tiếp cho Vosk STT trên ThinkPad):
arecord -D hw:1,7 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_record.wav
# Hoặc thu âm từ thiết bị mặc định:
arecord -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_record.wav

# Nghe lại file vừa thu ra loa Analog (Card 1, Dev 0):
aplay -D hw:1,0 /tmp/test_record.wav
# Hoặc phát qua thiết bị mặc định:
aplay /tmp/test_record.wav
```
*Lưu ý*: Nếu chạy trong môi trường không có loa ngoài hoặc container headless, sử dụng chế độ `tts_engine:=log` để kiểm thử toàn diện logic mà không phụ thuộc thiết bị âm thanh vật lý.

---

## 5. Build và chuẩn bị mỗi Terminal

Mỗi khi mở một Terminal mới vào container, biến môi trường chưa được nạp sẵn. Bắt buộc phải thực hiện các lệnh chuẩn bị:

### Các lệnh môi trường cần nạp trong MỖI Terminal mới:
```bash
# Chạy trong mọi Terminal mới mở
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

### Lệnh Build Package:
Thực hiện tại thư mục gốc workspace `/home/ws`:

```bash
# Thư mục hiện hành: /home/ws
cd /home/ws
colcon build --packages-select airport_robot_voice --symlink-install
source /home/ws/install/setup.bash
```
*Dấu hiệu thành công*: In ra `Finished <<< airport_robot_voice [x.xxs] Summary: 1 package finished`.

---

## 6. Chạy và kiểm thử hệ thống

### 6.1 Chạy bộ kiểm thử tự động

#### 1. Kiểm thử NLU thuần Python (không cần ROS):
```bash
# Thư mục: /home/ws
python3 src/airport_robot_voice/scripts/test_nlu.py
```
*Dấu hiệu thành công*: In ra `=== Kết quả kiểm thử: 61/61 đạt ===`.

#### 2. Chat thử tương tác NLU bằng bàn phím:
```bash
# Thư mục: /home/ws
python3 src/airport_robot_voice/scripts/test_nlu.py -i
```
*(Gõ thử: `dẫn tôi đến cổng a3`, `nhà vệ sinh ở đâu`, `đừng dẫn tôi đi đâu`, `hủy chuyến`, nhấn Ctrl+C để thoát).*

#### 3. Chạy toàn bộ 92 kiểm thử pytest trong ROS 2 Jazzy:
Để tránh tích tụ tài nguyên bộ nhớ của nhiều tiến trình action server trong một phiên test dài và cô lập môi trường, khuyến nghị dùng `ROS_DOMAIN_ID=42` và chạy theo 2 nhóm test độc lập:

```bash
# Thư mục: /home/ws
export ROS_DOMAIN_ID=42

# Nhóm 1: Kiểm thử Audio, NLU, Places Schema, và Planner Resolution (56 tests)
python3 -m pytest src/airport_robot_voice/test/test_audio.py src/airport_robot_voice/test/test_nlu.py src/airport_robot_voice/test/test_places.py src/airport_robot_voice/test/test_planner_resolution.py -v

# Nhóm 2: Kiểm thử Toàn diện Vòng đời ROS Lifecycle, Gate bảo vệ, Multi-Attempt & Late Revocation (36 tests)
python3 -m pytest src/airport_robot_voice/test/test_ros_lifecycle.py -v
```
*Dấu hiệu thành công*:
- Nhóm 1: `56 passed in ~9.6s` (13 tests audio, 28 tests NLU, 7 tests places schema, 8 tests planner resolution).
- Nhóm 2: `36 passed in ~12.6s` (36 tests: vòng đời Nav2, 3-phase cancel/pause, late acceptance revocation, unresolved goals gate blocking, end-to-end voice text replacement, cancel pending replace, delayed cancel terminal results, heartbeat with sim time, multi-attempt isolation, cancel acceptance vs rejection).
- Toàn bộ: **92/92 passed 100%**.

---

### 6.2 Khởi động trọn gói bằng Launch file (Khuyến nghị)

Package hỗ trợ cả hai tên launch file tương đương: `voice_system.launch.py` và `voice_assistant.launch.py`.

#### Bảng 9 tham số cấu hình Launch Argument:
| Tham số | Giá trị mặc định | Giá trị khả dụng | Ý nghĩa |
|---|---|---|---|
| `mode` | `keyboard` | `keyboard`, `mic`, `wav` | Nguồn đầu vào của giọng nói |
| `wav_file` | `""` | Đường dẫn file .wav | File âm thanh đầu vào khi `mode:=wav` |
| `tts_engine` | `log` | `log`, `espeak`, `piper` | Công cụ phát âm thanh (log-only, máy, tự nhiên) |
| `tts_policy` | `flexible` | `flexible`, `strict` | Chính sách: `strict` hủy goal nếu TTS lỗi/timeout; `flexible` tiếp tục gửi goal |
| `use_sim_time`| `false` | `true`, `false` | Đồng bộ thời gian mô phỏng (Gazebo /clock) |
| `places_file` | `places.json` | Đường dẫn file JSON | Cơ sở dữ liệu tọa độ các điểm đến |
| `vosk_model` | `/opt/vosk/vosk-model-small-vn-0.4` | Đường dẫn model | Model STT tiếng Việt |
| `use_nav` | `true` | `true`, `false` | Bật/tắt node điều hướng `nav_command_node` |
| `launch_stt` | `true` | `true`, `false` | Bật/tắt node nhận dạng giọng nói `stt_node` |

#### Lệnh khởi chạy tiêu biểu:

##### A. Chế độ Bàn phím với Hệ thống Voice (Khuyến nghị 2 bước rõ ràng):
Để tránh chạy 2 `stt_node` đồng thời hoặc tranh chấp stdin, sử dụng 2 Terminal riêng biệt:

- **Terminal 1: Khởi động hệ thống Voice (Tắt STT tự động)**:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
  ros2 launch airport_robot_voice voice_system.launch.py \
    launch_stt:=false \
    tts_engine:=log \
    tts_policy:=flexible
  ```

- **Terminal 2: Chạy đúng một node nhập liệu Bàn phím**:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
  ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
  ```
  *Cách tương tác*: Gõ các câu lệnh tiếng Việt tại Terminal 2 rồi nhấn `Enter`:
  - `dẫn tôi đến cổng A3` -> Bắt đầu dẫn đường tới cổng A3.
  - `đổi sang cổng A4` -> Đổi lộ trình sang cổng A4 khi đang di chuyển.
  - `dừng lại` -> Tạm dừng di chuyển.
  - `đi tiếp` -> Tiếp tục hành trình.
  - `hủy chuyến` -> Hủy chuyến đi hiện tại và đưa robot về IDLE.
  - `bạn đang dẫn tôi đi đâu` -> Hỏi trạng thái dẫn đường và khoảng cách.
  *Cách thoát sạch*: Nhấn `Ctrl + C` tại Terminal 2, sau đó `Ctrl + C` tại Terminal 1.

##### B. Chế độ Voice Độc lập (Voice Only, khi Nav2 chưa khởi động):
Khi cần kiểm thử NLU và trả lời thông tin offline mà chưa có Nav2:
- **Terminal 1: Khởi động hệ thống Voice không dùng Nav2**:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
  ros2 launch airport_robot_voice voice_system.launch.py \
    use_nav:=false \
    launch_stt:=false \
    tts_engine:=log
  ```
- **Terminal 2: Nhập liệu bàn phím**:
  ```bash
  source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
  ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
  ```
  *(Gõ thử: `wifi sân bay tên gì`, `sân bay mở cửa mấy giờ`, `sạc pin ở đâu`, `nhà vệ sinh ở đâu`).*

##### C. Chế độ Thực tế đầy đủ với Micro và Loa ngoài:
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py \
  mode:=mic \
  tts_engine:=piper \
  tts_policy:=flexible
```
---

### 6.3 Vận hành từng Node riêng biệt theo từng Terminal

Để quan sát chi tiết log của từng node, mở các Terminal tương tác riêng biệt:

#### Terminal 1: TTS Node (Phát tiếng nói hoặc ghi log)
```bash
# [Terminal 1]
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
# Chọn 1 trong các chế độ:
# Chế độ A: Log-only (Khuyến nghị, không cần loa)
ros2 run airport_robot_voice tts_node --ros-args -p engine:=log

# Chế độ B: eSpeak-ng (Phát ra loa thật)
# ros2 run airport_robot_voice tts_node --ros-args -p engine:=espeak

# Chế độ C: Piper ONNX (Giọng tự nhiên)
# ros2 run airport_robot_voice tts_node --ros-args -p engine:=piper
```

#### Terminal 2: Intent Node (Lõi NLU nhận diện ý định)
```bash
# [Terminal 2]
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice intent_node
```

#### Terminal 3: Nav Command Node (Quản lý nhiệm vụ và Nav2)
*(Khởi động khi muốn chạy chế độ kết hợp Navigation)*
```bash
# [Terminal 3]
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice nav_command_node
```

#### Terminal 4: STT Node (Nhập câu nói)
```bash
# [Terminal 4]
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
# Chế độ bàn phím:
ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard

# Hoặc chế độ micro thật:
# ros2 run airport_robot_voice stt_node --ros-args -p mode:=mic
```

#### Terminal 5: Giám sát Topic (Monitor)
```bash
# [Terminal 5]
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Lệnh theo dõi câu trả lời phát ra:
ros2 topic echo /voice/say

# Lệnh theo dõi yêu cầu điều hướng JSON:
# ros2 topic echo /voice/intent

# Lệnh theo dõi trạng thái nhiệm vụ Nav2 latched:
# ros2 topic echo /voice/nav_state --qos-durability transient_local
```

---

### 6.4 Bảng kịch bản kiểm tra chức năng giọng nói (Nhập tại Terminal 4)

| Kịch bản | Câu nhập | Phản hồi mong đợi | Hành vi hệ thống |
|---|---|---|---|
| **Dẫn đường cổng A3** | `dẫn tôi đến cổng A3` | *"Tôi sẽ dẫn bạn đến cổng A3"* | `intent: guide_me`, `action: navigate`, `target: gate_a3`. Gửi goal Nav2. |
| **Hỏi vị trí cổng** | `cổng A3 ở đâu` | *"Tôi sẽ dẫn bạn đến cổng A3"* | Chuyển câu hỏi vị trí thành lệnh dẫn đường. |
| **Cách đọc khác** | `cổng a ba`, `cổng bê hai` | *"Tôi sẽ dẫn bạn đến cổng A3/B2"* | Đọc chuẩn mã cổng tiếng Việt. |
| **Quầy làm thủ tục** | `cho tôi tới quầy check-in` | *"Tôi sẽ dẫn bạn đến quầy làm thủ tục"* | `target: checkin`. |
| **Nhà vệ sinh (nhóm)** | `nhà vệ sinh ở đâu` | *"Tôi sẽ tìm đường tới nhà vệ sinh phù hợp nhất"* | `category: toilet`, gọi `ComputePathToPose` chọn điểm ngắn nhất. |
| **Nhà vệ sinh cụ thể** | `nhà vệ sinh sảnh A` | *"Tôi sẽ dẫn bạn đến nhà vệ sinh sảnh A"* | `target: toilet_1`. |
| **Mã cổng không có** | `dẫn tôi đến cổng A99` | *"Địa điểm A99 chưa có trong danh mục"* | `intent: place_not_found`, không phát lệnh điều hướng. |
| **Thiếu điểm đến** | `dẫn tôi đến` | *"Tôi chưa xác định được điểm đến"* | `intent: missing_destination`, kết thúc lượt ngay. |
| **Nhiều điểm đến** | `cổng A1 hay A2` | *"Yêu cầu có nhiều điểm đến chưa rõ ràng"* | `intent: ambiguous_target`, yêu cầu người dùng chọn lại. |
| **Phủ định** | `đừng dẫn tôi đến cổng A1` | *"Tôi đã ghi nhận bạn không muốn dẫn đường"* | `intent: negated_nav`, không phát lệnh điều hướng. |
| **Hủy chuyến** | `không cần nữa`, `hủy chuyến` | *"Tôi đã hủy chuyến đi hiện tại"* | `action: cancel`, hủy goal Nav2 và dọn dẹp trạng thái. |
| **Tạm dừng** | `dừng lại`, `chờ tôi` | *"Tôi đã tạm dừng. Bạn bảo đi tiếp khi sẵn sàng"* | `action: pause`, dừng robot và giữ nguyên đích. |
| **Phủ định dừng** | `không dừng lại nhé` | *"Tôi tiếp tục di chuyển và không dừng lại"* | `intent: negated_pause`, không kích hoạt pause. |
| **Đi tiếp** | `đi tiếp`, `tiếp tục` | *"Chúng ta tiếp tục di chuyển"* | `action: resume`, gửi lại goal Nav2 từ vị trí hiện tại. |
| **Đổi đích** | `đổi sang cổng A4` *(khi đang đi)* | *"Đã đổi lộ trình, tôi sẽ dẫn bạn đến cổng A4"* | `action: replace`, hủy goal cũ và xuất phát tới đích mới. |
| **Hỏi trạng thái** | `bạn đang dẫn tôi đi đâu` | *"Tôi đang dẫn bạn đến [đích], còn khoảng [X] mét"* | `action: status`, đọc feedback từ Nav2. |
| **Thông tin Wi-Fi** | `wifi sân bay tên gì` | *"Sân bay có Wi-Fi miễn phí tên là Airport-Free-WiFi..."* | Trả lời offline, không di chuyển. |
| **Thông tin sạc pin** | `sạc điện thoại ở đâu` | *"Các điểm sạc điện thoại miễn phí bố trí tại..."* | Trả lời offline. |
| **Giờ mở cửa** | `sân bay mở cửa mấy giờ` | *"Các dịch vụ hành khách mở cửa liên tục 24/7"* | Trả lời offline. |

---

### 6.5 Kịch bản nghiệm thu tích hợp Gazebo & Nav2 theo từng Terminal

Workspace đã tích hợp sẵn package `linorobot2` (`linorobot2_gazebo` và `linorobot2_navigation`). Quy trình khởi động và nghiệm thu hoàn chỉnh theo từng Terminal:

#### Terminal 1: Khởi động Mô phỏng Robot trong Gazebo
```bash
# Thiết lập base robot mecanum (chuẩn xác theo cấu hình phần cứng workspace)
export LINOROBOT2_BASE=mecanum
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Khởi động mô phỏng với thế giới turtlebot3_world
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=turtlebot3_world
```

#### Terminal 2: Khởi động Hệ thống Điều hướng Nav2
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Khởi động Nav2 stack kèm bản đồ tương ứng và giao diện RViz2
ros2 launch linorobot2_navigation navigation.launch.py \
  sim:=true \
  map:=/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml \
  rviz:=true
```

*Kiểm tra sẵn sàng trước khi kết nối voice*:
```bash
# Kiểm tra các action server của Nav2 đã xuất hiện:
ros2 action list | grep -E "navigate_to_pose|compute_path_to_pose"

# Kiểm tra tọa độ AMCL của robot:
ros2 topic echo /amcl_pose --once
```

#### Terminal 3: Khởi động Airport Robot Voice (launch_stt:=false)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Khởi chạy voice với cờ sim_time, nạp places_sim.json và tách STT keyboard riêng:
ros2 launch airport_robot_voice voice_system.launch.py \
  launch_stt:=false \
  tts_engine:=log \
  tts_policy:=flexible \
  use_sim_time:=true \
  places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json
```

#### Terminal 4: Giám sát Trạng thái Điều hướng (/voice/nav_state)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 topic echo /voice/nav_state --qos-durability transient_local
```

#### Terminal 5: Nhập lệnh tương tác Bàn phím (STT Keyboard độc lập)
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
# Chạy node nhập liệu bàn phím duy nhất:
ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
```
*Chuỗi kịch bản nghiệm thu kiểm thử tuần tự*:
1. `dẫn tôi đến cổng A3` -> Robot nhận goal và bắt đầu di chuyển trong Gazebo; nav_state chuyển `NAVIGATING`.
2. `nhà vệ sinh ở đâu` -> Planner tính đường đến `toilet_1` (-1.0, 1.8) và `toilet_2` (0.2, 1.8), chọn điểm gần hơn (`toilet_2`).
3. `dừng lại` -> Robot dừng di chuyển hoàn toàn trong mô phỏng (/cmd_vel về 0); nav_state chuyển `PAUSING` -> `PAUSED`.
4. `đi tiếp` -> Robot tiếp tục hành trình tới đích; nav_state quay lại `NAVIGATING`.
5. `đổi sang cổng A4` -> Hủy an toàn goal A3, robot chuyển hướng sang cổng A4 (1.2, -0.6).
6. `hủy chuyến` (trong lúc đang đổi sang A4) -> Dừng hẳn ngay lập tức, hủy toàn bộ nhiệm vụ, không gửi goal A4, về `IDLE`.
7. `bạn đang dẫn tôi đi đâu` -> Robot báo tên đích và khoảng cách còn lại (mét).
8. `dẫn tôi đến cổng Z9` -> Báo địa điểm chưa có trong danh mục, không kích hoạt điều hướng.
9. `tôi không muốn đến cổng A1` -> Nhận diện câu phủ định, không kích hoạt điều hướng.
---

## 7. Cấu hình địa điểm (`places.json`)

Toàn bộ dữ liệu toạ độ và danh mục lưu tại `config/places.json`.

### 7.1 Cấu trúc địa điểm
```json
"gate_a3": {
  "name": "cổng A3",
  "desc": "cánh trái nhà ga, đối diện cổng A1",
  "aliases": ["a3", "a ba", "cổng a3", "cổng a ba"],
  "category": "gate",
  "enabled": true,
  "x": -2.0,
  "y": 10.0,
  "yaw": 1.57,
  "frame_id": "map"
}
```

### 7.2 Cách lấy toạ độ thực tế trên bản đồ Nav2
1. Khởi động bản đồ và RViz2.
2. Dùng công cụ `2D Pose Estimate` hoặc điều khiển robot tới vị trí mong muốn.
3. Chạy lệnh lấy toạ độ:
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
4. Đọc `position.x`, `position.y` và đổi quaternion sang góc yaw:
   ```python
   yaw = 2.0 * math.atan2(orientation.z, orientation.w)
   ```
5. Cập nhật các giá trị `x`, `y`, `yaw` vào `config/places.json`.

---

## 8. Xử lý lỗi thường gặp và Thiết lập máy mới

### 8.1 Các lỗi thường gặp và cách khắc phục

| Lỗi | Nguyên nhân | Lệnh chẩn đoán / Khắc phục |
|---|---|---|
| `No module named 'vosk'` | Chưa cài vosk vào user python | `pip install --user --break-system-packages vosk sounddevice` |
| `Không nạp được model Vosk` | Sai đường dẫn hoặc chưa giải nén | Kiểm tra `ls -la /opt/vosk/vosk-model-small-vn-0.4/model.cc` hoặc truyền `-p model_path:=/path/to/model` |
| `Hệ thống điều hướng chưa sẵn sàng` | `nav_command_node` hoặc Nav2 chưa chạy | Kiểm tra node: `ros2 node list` xem có `/nav_command_node` chưa |
| `piper: command not found` | Chưa có symlink trong PATH | Chạy `sudo ln -sf ~/.local/bin/piper /usr/local/bin/piper` |
| `ALSA lib pcm.c: cannot open` | Container thiếu thiết bị hoặc quyền audio | Thêm user vào nhóm audio: `sudo usermod -aG audio $USER` hoặc dùng `tts_engine:=log` |
| `Goal chưa xác nhận hoặc Action Server mất kết nối` | Action server rớt kết nối hoặc trả STATUS_UNKNOWN khi hủy goal | Hệ thống tự động đóng Gate bảo vệ an toàn, giữ nguyên attempt và chặn mọi goal mới. Xem Quy trình cho Người vận hành tại Mục 8.2 bên dưới. |

### 8.2 Quy trình cho Người vận hành (Operator Procedure) khi Action Server sập hoặc không lấy lại được kết quả

Khi Nav2 Action Server bị crash, rớt kết nối mạng hoặc không trả về kết quả terminal cho attempt cũ, `nav_command_node` sẽ **đóng Gate an toàn** (`has_unresolved_goals == True`). Mọi yêu cầu gửi goal mới qua lệnh thoại (`handle_navigate_request`, `resolve_category_and_start`, `handle_resume_request`, pending replace) đều bị chặn và robot sẽ thông báo: *"Chưa xác nhận được chuyến đi cũ đã kết thúc."*

> [!CAUTION]
> **Nguyên tắc an toàn**: Hệ thống **tuyệt đối không cho phép** bất kỳ câu lệnh thoại nào ép xóa trạng thái chưa xác minh ở `IDLE`. Chỉ người vận hành mới được phép can thiệp bằng quy trình chuẩn dưới đây.

#### Quy trình 4 bước cho Người vận hành:
1. **Bước 1 — Dừng khẩn cấp thực tế và Xác minh Robot dừng hẳn**:
   - **Lưu ý kỹ thuật quan trọng**: Tuyệt đối **không dùng** lệnh `ros2 topic pub --once /cmd_vel ...` như một cơ chế dừng khẩn cấp. Trên cấu hình robot Mecanum đang chạy với Nav2, `controller_server` liên tục tính toán và xuất topic `/cmd_vel` theo chu kỳ vòng lặp 10–20 Hz. Một bản tin `--once` sẽ ngay lập tức bị ghi đè bởi chu kỳ tiếp theo của controller nếu goal Nav2 vẫn đang active.
   - **Cơ chế dừng đã xác minh trên hệ thống**:
     + *Dừng phần mềm*: Gọi lệnh cancel goal thông qua ActionClient trên action `/navigate_to_pose`, hoặc chuyển lifecycle manager của Nav2 sang trạng thái inactive:
       ```bash
       # Hủy goal đang hoạt động qua Nav2 action CLI (nếu action server còn phản hồi):
       ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --feedback
       ```
     + *Dừng cưỡng bức / Khẩn cấp*: Nhấn công tắc E-stop vật lý trên thân robot nếu ở hiện trường; hoặc trong mô phỏng/terminal, phát lặp liên tục tần số cao (10–20 Hz) lệnh dừng vận tốc:
       ```bash
       # Phát lặp liên tục tần số 20 Hz cưỡng bức dừng motor:
       ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
       ```
   - **Xác minh robot dừng hẳn**: Trước khi thực hiện bất kỳ bước khôi phục nào, người vận hành phải kiểm tra vận tốc từ `/odom` để đảm bảo robot đã đứng yên hoàn toàn ($v \le 0.02$ m/s, $\omega \le 0.02$ rad/s):
     ```bash
     ros2 topic echo /odom --once | grep -E "linear|angular"
     ```

2. **Bước 2 — Kiểm tra và Xử lý triệt để Goal cũ trên Nav2**:
   - Kiểm tra xem `bt_navigator` và các action server của Nav2 còn hoạt động hay không:
     ```bash
     ros2 lifecycle get /bt_navigator
     ros2 action list | grep navigate_to_pose
     ```
   - Nếu Nav2 bị lỗi hoặc crash, khởi động lại stack Nav2 (`ros2 launch linorobot2_navigation ...`). Phải đảm bảo không còn goal mồ côi nào đang tiếp tục di chuyển bánh xe.

3. **Bước 3 — Khởi động lại Voice Command Node (Tránh pkill chung)**:
   - Trong môi trường phát triển song song (có các node khác như `person_tracking` đang chạy), **tuyệt đối tránh `pkill -f python3` hoặc `pkill -f node`**.
   - Chỉ dừng chính xác tiến trình `nav_command_node`:
     ```bash
     # Dừng chính xác tiến trình nav_command_node của package voice:
     pkill -f "airport_robot_voice/nav_command_node" || true
     
     # Khởi động lại node voice:
     ros2 run airport_robot_voice nav_command_node
     # (Hoặc khởi động lại toàn bộ voice qua voice_system.launch.py)
     ```

4. **Bước 4 — Xác minh phạm vi theo dõi sau khi Restart**:
   - Khi node khởi động lại, một phiên làm việc mới hoàn toàn được tạo với `_session_start_ms` mới (dựa trên epoch timestamp ms).
   - Máy trạng thái bắt đầu từ `IDLE` sạch, danh sách `_attempts` trống, cờ `_unconfirmed_terminal = False`.
   - > [!IMPORTANT]
     > **Phạm vi của Restart**: Việc khởi động lại Voice đưa máy trạng thái về `IDLE` **chỉ xác nhận và làm sạch bộ nhớ cục bộ trong phiên của node Voice mới**. Node mới không kế thừa attempt cũ, nhưng người vận hành **bắt buộc phải hoàn thành Bước 1 và Bước 2** để chắc chắn robot vật lý đã dừng và Nav2 không còn goal cũ trước khi ra lệnh dẫn đường mới.
   - Kiểm tra trạng thái sẵn sàng qua topic:
     ```bash
     ros2 topic echo /voice/nav_state --once --qos-durability transient_local
     ```
     *Dấu hiệu*: `mission_state: "IDLE"`, `has_unresolved_goals: false`, `node_online: true`. Hệ thống sẵn sàng nhận nhiệm vụ mới.

### 8.3 Dọn dẹp tiến trình an toàn trong Workspace dùng chung
```bash
# Chỉ dừng đúng các tiến trình thuộc package airport_robot_voice, không ảnh hưởng các package khác (person_tracking):
pkill -f "airport_robot_voice/nav_command_node|airport_robot_voice/intent_node|airport_robot_voice/tts_node|airport_robot_voice/stt_node" || true
```

### 8.4 Thiết lập lại toàn bộ từ đầu trên máy mới (Full Setup Script)
Trên một máy mới đã clone repo vào `~/ws/src/airport_robot_voice` và đang trong container ROS 2 Jazzy:

```bash
# 1. Cài đặt dependency hệ thống
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  alsa-utils espeak-ng libportaudio2 python3-pyaudio python3-pip

# 2. Cài đặt thư viện Python
pip install --user --break-system-packages vosk sounddevice piper-tts
sudo ln -sf ~/.local/bin/piper /usr/local/bin/piper

# 3. Tải model Vosk
sudo mkdir -p /opt/vosk && sudo chown -R $USER:$USER /opt/vosk && cd /opt/vosk
python3 -c 'import urllib.request, zipfile, os; urllib.request.urlretrieve("https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip", "m.zip"); zipfile.ZipFile("m.zip").extractall("/opt/vosk"); os.remove("m.zip")'

# 4. Tải model Piper
sudo mkdir -p /opt/piper && sudo chown -R $USER:$USER /opt/piper && cd /opt/piper
curl -L -s -o vi_VN-vais1000-medium.onnx https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx
curl -L -s -o vi_VN-vais1000-medium.onnx.json https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx.json

# 5. Build và test package
cd /home/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select airport_robot_voice --symlink-install
source /home/ws/install/setup.bash
python3 -m pytest src/airport_robot_voice/test/ -v
```

---

## 9. Bài thử nghiệm Micro và Loa thật (5 phút khi thức dậy)

Bài thử nghiệm này giúp bạn kiểm tra toàn diện chất lượng thu âm tiếng Việt từ Micro thực tế và phát tiếng nói ra Loa ngoài trong vòng **5 phút**, bắt đầu bằng **chế độ Voice độc lập (`use_nav:=false`)** trong không gian mạng ROS cô lập (`export ROS_DOMAIN_ID=42`), không phụ thuộc vào hệ thống điều hướng Nav2 hay Gazebo.

---

### 9.1 Kiểm tra thiết bị Âm thanh Động (SoundDevice Query)

Trước khi khởi chạy, chạy script Python sau trong bất kỳ Terminal nào để kiểm tra danh sách thiết bị micro và loa thực tế trên máy của bạn:

```bash
python3 -c "
import sounddevice as sd
print('=== DANH SÁCH THIẾT BỊ ÂM THANH KHẢ DỤNG ===')
devs = sd.query_devices()
for idx, d in enumerate(devs):
    in_ch = d.get('max_input_channels', 0)
    out_ch = d.get('max_output_channels', 0)
    kind = []
    if in_ch > 0: kind.append(f'MIC ({in_ch} ch)')
    if out_ch > 0: kind.append(f'SPEAKER ({out_ch} ch)')
    print(f'[{idx:02d}] {d["name"]} -> {", ".join(kind)}')
default_in, default_out = sd.default.device
print(f'Thiết bị mặc định: Input={default_in}, Output={default_out}')
"
```
*Lưu ý*: Mặc định node nhận diện giọng nói `stt_node` sẽ dùng `device:=-1` (thiết bị thu âm mặc định). Nếu muốn chọn đúng micro mong muốn, bạn chỉ cần truyền thêm tham số `-p device:=<index>` (ví dụ `-p device:=7`).

---

### 9.2 Thử nghiệm nhanh Thu âm và Nghe lại (WAV Check)

Để xác nhận phần cứng micro và loa hoạt động chuẩn trước khi chạy node ROS:
```bash
# 1. Thu âm 3 giây câu nói của bạn:
python3 -c "
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
print('Đang thu âm 3 giây... Hãy nói một câu vào micro!')
fs = 16000
rec = sd.rec(int(3 * fs), samplerate=fs, channels=1, dtype='int16')
sd.wait()
wav.write('/tmp/test_mic.wav', fs, rec)
print('Thu âm xong: /tmp/test_mic.wav')
"

# 2. Nghe lại trực tiếp qua loa:
python3 -c "
import sounddevice as sd
import scipy.io.wavfile as wav
fs, data = wav.read('/tmp/test_mic.wav')
print('Đang phát lại qua loa...')
sd.play(data, fs)
sd.wait()
print('Phát xong!')
"

# 3. Thử nhận dạng offline bằng mô hình Vosk tiếng Việt từ file vừa thu:
python3 -c "
from vosk import Model, KaldiRecognizer
import wave, json
wf = wave.open('/tmp/test_mic.wav', 'rb')
model = Model('/opt/vosk/vosk-model-small-vn-0.4')
rec = KaldiRecognizer(model, wf.getframerate())
while True:
    data = wf.readframes(4000)
    if len(data) == 0: break
    rec.AcceptWaveform(data)
res = json.loads(rec.FinalResult())
print('Vosk nhận diện được:', repr(res.get('text', '')))
"
```

---

### 9.3 Bố trí 3 Terminal trong VS Code (Chế độ Voice Độc lập, ROS_DOMAIN_ID=42)

Mở 3 Terminal riêng biệt trong VS Code (nhấn `Ctrl + Shift + \`` để mở terminal mới). Mỗi terminal chạy trong `ROS_DOMAIN_ID=42`:

#### Terminal 1: Khởi động Phát âm thanh TTS ra Loa thật
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Phát tiếng nói tự nhiên bằng Piper ONNX ra loa máy tính (hoặc thay bằng engine:=espeak):
ros2 run airport_robot_voice tts_node --ros-args -p engine:=piper
```
*(Node này sẽ nhận câu nói từ hệ thống, tổng hợp âm thanh và phát trực tiếp ra loa qua ALSA).*

#### Terminal 2: Khởi động Lõi Xử lý Ngôn ngữ Tự nhiên NLU
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Chạy Intent Node ở chế độ độc lập (không cần Nav2):
ros2 run airport_robot_voice intent_node
```
*(Node này phân tích ý định từ giọng nói, trả lời các thông tin offline và thông báo an toàn nếu yêu cầu dẫn đường khi Nav2 chưa bật).*

#### Terminal 3: Khởi động Nhận diện Giọng nói STT từ Micro thật
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Chạy Vosk STT lắng nghe micro phần cứng (sử dụng model tiếng Việt offline):
ros2 run airport_robot_voice stt_node --ros-args -p mode:=mic
```
*(Mặc định dùng micro hệ thống. Nếu muốn chỉ định index cụ thể: thêm `-p device:=<index>`).*

*(Tùy chọn) Terminal 4: Giám sát văn bản nhận diện thời gian thực*:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 topic echo /voice/text
```

---

### 9.2 Bảng 3 Câu Tiếng Việt Thử Nghiệm Tuần Tự (Kèm Bước Thao tác)

Ngồi trước máy tính, lần lượt nói vào Micro theo 3 câu dưới đây:

| STT | Câu bạn cần nói vào Micro | Transcript nhận diện (/voice/text) | Intent phân tích (/voice/intent) | Phản hồi mong đợi phát ra Loa | Bước thao tác của bạn |
|---|---|---|---|---|---|
| **Câu 1** | **"wifi sân bay tên gì"** | `"wifi sân bay tên gì"` | `intent: ask_wifi`<br>`action: None`<br>`target: None` | *"Sân bay có Wi-Fi miễn phí tên là Airport-Free-WiFi, kết nối không cần mật khẩu."* | **[NÓI]** Bạn nói rõ câu vào micro.<br>**[NGHE]** Xác nhận nghe thấy tiếng Piper phát câu thông tin Wi-Fi qua loa ngoài. |
| **Câu 2** | **"sân bay mở cửa mấy giờ"** | `"sân bay mở cửa mấy giờ"` | `intent: ask_hours`<br>`action: None`<br>`target: None` | *"Các dịch vụ hành khách tại nhà ga mở cửa liên tục 24/7."* | **[NÓI]** Bạn nói câu vào micro.<br>**[NGHE]** Xác nhận nghe tiếng loa phát rõ ràng *"mở cửa liên tục 24/7"*. |
| **Câu 3** | **"cổng A3 ở đâu"** | `"cổng a3 ở đâu"`<br>*(hoặc `"cổng a ba ở đâu"`)* | `intent: guide_me`<br>`action: navigate`<br>`target_id: gate_a3` | *"Đã xác định yêu cầu tới cổng A3, nhưng hệ thống điều hướng chưa sẵn sàng."* | **[NÓI]** Bạn nói câu hỏi vị trí cổng A3.<br>**[NGHE]** Xác nhận nghe tiếng loa báo đã nhận diện đúng cổng A3 và thông báo an toàn do đang chạy voice độc lập (`use_nav:=false`). |

---

### 9.3 Cách thoát sạch sau khi thử xong
Tại mỗi Terminal 1, 2, 3, nhấn tổ hợp phím **`Ctrl + C`** để dừng các node. Hệ thống giải phóng toàn bộ tài nguyên âm thanh ALSA.

---

### 9.4 Lệnh chạy lại toàn bộ Bộ Kiểm thử và Nghiệm thu Gazebo
Nếu muốn chạy lại bất kỳ lúc nào:
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# 1. Toàn bộ 92 bài test pytest (Cô lập ROS_DOMAIN_ID=42):
export ROS_DOMAIN_ID=42
python3 -m pytest src/airport_robot_voice/test/test_audio.py src/airport_robot_voice/test/test_nlu.py src/airport_robot_voice/test/test_places.py src/airport_robot_voice/test/test_planner_resolution.py -v
python3 -m pytest src/airport_robot_voice/test/test_ros_lifecycle.py -v

# 2. Kiểm thử 61 câu tiếng Việt NLU:
python3 src/airport_robot_voice/scripts/test_nlu.py

# 3. Toàn bộ 3 ca nghiệm thu chuyên sâu trên mô phỏng Gazebo Sim + Nav2:
# (Gồm: A3->A4->Hủy (v=0), A3 tới đích SUCCEEDED, Đo đường 2 Toilet & Chọn đúng điểm ngắn nhất)
bash src/airport_robot_voice/scripts/run_evidence_verification.sh
```


---

## 10. Giao diện Tích hợp Phân hệ Person Tracking (Chuẩn bị cho Phase 9)

Tài liệu tham chiếu thiết kế: `src/.agent/person_tracking/VOICE_INTEGRATION_REQUEST.md`.  
Dưới đây là bản tóm tắt các điểm then chốt đã thống nhất để chuẩn bị cho giai đoạn tích hợp chính thức tại **Phase 9**:

### 10.1 Phân định Quyền hạn và Trách nhiệm
1. **Voice là Chủ sở hữu DUY NHẤT của `NavigateToPose` Action**:
   - Node `nav_command_node.py` của phân hệ Voice toàn quyền khởi tạo `ActionClient<NavigateToPose>`, gửi Goal, theo dõi feedback và hủy Goal.
   - Phân hệ Person Tracking **tuyệt đối không** khởi tạo `ActionClient<NavigateToPose>`, không gửi goal và không gọi cancel trực tiếp trên Nav2.
2. **Tracking là Cổng An toàn và Quản lý Khóa Khách hàng**:
   - Tracking phát hiện hành khách qua camera sau, nhận diện cử chỉ giơ tay để khóa khách hàng, sinh `session_id`, và quản lý cổng an toàn vận tốc motor.

### 10.2 Hợp đồng Giao tiếp 3 Kênh giữa Voice và Tracking
1. **Kênh 1 — Trạng thái Khóa Khách hàng (`/tracking/session_state`)**:
   - **Kiểu bản tin**: `std_msgs/msg/String` (JSON), QoS: Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE.
   - **Trường dữ liệu**: `session_id`, `state` (`"LOCKED_WAIT_DESTINATION"`), `locked` (`true`/`false`), `confidence`, `stamp`.
   - **Hành vi Voice tại Phase 9**:
     - Lắng nghe topic `/tracking/session_state`.
     - Khi nhận intent dẫn đường từ người dùng:
       + Nếu `locked == true` và `state == "LOCKED_WAIT_DESTINATION"`: Ghi nhận `session_id` vào phiên hiện tại, chấp nhận điểm đến và gửi Goal cho Nav2.
       + Nếu `locked == false`: Từ chối dẫn đường, phát câu TTS: *"Xin quý khách vui lòng đứng phía sau và giơ một tay để robot nhận diện trước khi chọn điểm đến."*

2. **Kênh 2 — Yêu cầu Hủy khẩn cấp An toàn (`/tracking/safety_request`)**:
   - **Kiểu bản tin**: `std_msgs/msg/String` (JSON), QoS: Depth=10, Reliability=RELIABLE.
   - **Trường dữ liệu**: `request_id`, `session_id`, `command` (`"ABORT"`), `reason_code` (`"TARGET_LOST_PROLONGED"` khi mất dấu > 1.2s), `message`.
   - **Hành vi Voice tại Phase 9**:
     - Khi nhận bản tin `command == "ABORT"` có `session_id` khớp với phiên đang dẫn đường:
       + Lập tức gọi `cancel_goal_async()` hủy Goal đang chạy trên Nav2.
       + Chuyển trạng thái `mission_state` sang `CANCELING` -> `IDLE`.
       + Phát TTS thông báo: *"Đã mất dấu hành khách, robot dừng hành trình dẫn đường."*

3. **Kênh 3 — Phát hành Trạng thái Hoàn tất để Mở khóa (`/voice/nav_status`)**:
   - **Kiểu bản tin**: `std_msgs/msg/String` (JSON), QoS: Depth=1, Durability=TRANSIENT_LOCAL, Reliability=RELIABLE.
   - **Trường dữ liệu**: `session_id`, `mission_id`, `goal_attempt_id`, `destination_id`, `status` (`"SUCCEEDED"`), `nav_event` (`"GOAL_COMPLETED"`), `is_terminal` (`true`), `message`.
   - **Ý nghĩa**: Khi Voice phát bản tin hoàn tất có `status == "SUCCEEDED"` và `is_terminal == true` đúng với `session_id`, Tracking xác nhận khách đã đến nơi an toàn, xóa ID khóa, dọn dẹp phiên và đưa hệ thống về trạng thái sẵn sàng đón lượt khách mới.

### 10.3 Nguyên tắc Độc lập trong Giai đoạn Hiện tại
- Trong các Phase trước Phase 9, phân hệ Tracking sử dụng node mock nội bộ (`mock_voice_owner`) để tự kiểm thử.
- Phân hệ Voice **chưa nối trực tiếp code** vào các topic của Tracking trong lượt này để đảm bảo toàn bộ 94 bài test hiện tại và các kịch bản nghiệm thu độc lập không bị ảnh hưởng.
