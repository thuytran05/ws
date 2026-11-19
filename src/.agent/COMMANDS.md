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
# Thu âm thử 3 giây từ micro mặc định (16kHz, mono)
arecord -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_record.wav

# Nghe lại file vừa thu
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

#### 3. Chạy toàn bộ 65 kiểm thử pytest trong ROS 2 Jazzy:
```bash
# Thư mục: /home/ws
python3 -m pytest src/airport_robot_voice/test/ -v
```
*Dấu hiệu thành công*: `65 passed in ~41s` (bao gồm 12 tests audio, 24 tests NLU, 7 tests places schema, 8 tests planner resolution, 14 tests action lifecycle).

---

### 6.2 Vận hành hệ thống theo từng Terminal

Để vận hành hệ thống độc lập, mở 5 terminal tương tác vào container:

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

### 6.3 Các kịch bản kiểm tra chức năng (Nhập tại Terminal 4)

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

### 8.2 Dọn dẹp process khi kết thúc phiên
```bash
# Dừng tất cả các node của package đang chạy nền
pkill -f airport_robot_voice
```

### 8.3 Thiết lập lại toàn bộ từ đầu trên máy mới (Full Setup Script)
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
