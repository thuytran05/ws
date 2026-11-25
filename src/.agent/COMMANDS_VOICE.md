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

### 2.4 Cài đặt thư viện Python (Tách riêng venv Faster-Whisper, không cài đè ROS Python)
Trên Ubuntu 24.04 (Noble), ROS 2 Jazzy sử dụng `/usr/bin/python3`. Giữ nguyên môi trường này (chỉ cài `sounddevice`, `piper-tts`, `vosk` ở cấp user cho TTS/Audio/Vosk dự phòng), còn **Faster-Whisper 1.2.1** chạy qua worker sống lâu trong venv riêng `~/.venvs/airport-whisper-test` (**tuyệt đối không cài `rclpy` qua pip vào venv**):

```bash
# 1. Cài đặt SoundDevice, Piper TTS và Vosk dự phòng cho Python hệ thống (PEP 668)
pip install --user --break-system-packages \
  sounddevice \
  piper-tts \
  vosk
sudo ln -sf ~/.local/bin/piper /usr/local/bin/piper

# 2. Tạo venv riêng cho Faster-Whisper (mặc định được stt_node tự động dò)
python3 -m venv ~/.venvs/airport-whisper-test
~/.venvs/airport-whisper-test/bin/pip install --upgrade pip faster-whisper numpy

# 3. Cài paplay (pulseaudio-utils) nếu container chưa có
which paplay || sudo apt-get install -y pulseaudio-utils
```

---

### 2.5 Kiểm tra dependencies sau khi cài
```bash
# Kiểm tra Python ROS 2 Jazzy + Worker Faster-Whisper trong venv
python3 -c "
for mod in ['numpy', 'sounddevice', 'vosk', 'piper', 'rclpy', 'nav2_msgs']:
    __import__(mod)
    print(f'  [OK][ROS Python] {mod}')
"
~/.venvs/airport-whisper-test/bin/python -c "
import faster_whisper, numpy
print(f'  [OK][Whisper venv] faster_whisper={faster_whisper.__version__}, numpy={numpy.__version__}')
"
which piper paplay aplay arecord
```

---

## 3. Chuẩn bị STT và TTS Offline

Hệ thống hoạt động **100% offline**, không cần mạng khi vận hành. Bước này chỉ cần kết nối Internet **một lần duy nhất** để tải model ban đầu.

### 3.0 Chuẩn bị Model Faster-Whisper Offline (STT Chính Mặc định)
- **Thư viện**: `faster-whisper 1.2.1` trong `~/.venvs/airport-whisper-test`.
- **Cấu hình mặc định**: `model=small`, `device=cpu`, `compute_type=int8`, `language=vi`, `beam_size=5`, `local_files_only=True`.
- **Định dạng âm thanh**: WAV/micro 16 kHz mono PCM16 được đọc qua `wave`/`sounddevice`, chuẩn hóa sang NumPy `float32 / 32768.0` và truyền trực tiếp mảng vào `transcribe()` (tránh lỗi `av.open(..., metadata_errors=...)` của PyAV).

*(Chỉ cần Internet một lần để tải model vào cache `~/.cache/huggingface/hub`; sau đó hệ thống chạy hoàn toàn offline)*:
```bash
# Tải model mặc định 'small' (nếu máy chưa có cache):
~/.venvs/airport-whisper-test/bin/python -c "
from faster_whisper import WhisperModel
WhisperModel('small', device='cpu', compute_type='int8')
print('Đã tải cache Faster-Whisper small thành công!')
"

# Kiểm tra nạp offline (không kết nối mạng):
HF_HUB_OFFLINE=1 ~/.venvs/airport-whisper-test/bin/python -c "
from faster_whisper import WhisperModel
WhisperModel('small', device='cpu', compute_type='int8', local_files_only=True)
print('Faster-Whisper small offline OK!')
"
```
*(Nếu muốn thử `medium` hoặc `large-v3` sau này, tải trước vào cache như trên rồi truyền `whisper_model:=medium` hoặc `whisper_model:=large-v3`, `whisper_device:=cpu|cuda`, `whisper_compute_type:=int8|float16`).*

---

### 3.1 Tải Model Vosk tiếng Việt (STT Dự phòng — chỉ dùng khi chọn `stt_backend:=vosk`)
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

### 4.2 Liệt kê và chọn Micro (ThinkPad P15 Gen 2: mặc định `device:=9`, ALSA `hw:1,7`, 16000 Hz)
Chạy lệnh kiểm tra phần cứng trong container khi index thay đổi (sau reboot hoặc cắm mic USB):

```bash
# Liệt kê chỉ các thiết bị thu âm khả dụng cùng sample rate mặc định:
python3 -c "
import sounddevice as sd
for i, d in enumerate(sd.query_devices()):
    if d.get('max_input_channels', 0) > 0:
        print(f'[{i}] {d["name"]} | in={d["max_input_channels"]} | default_sr={int(d.get("default_samplerate", 0))}')
"

# Xem thêm bằng ALSA CLI:
arecord -l
```

*Cách chọn thiết bị trên ThinkPad P15 Gen 2*:
- **Mặc định đã đặt `device:=9`** (`sof-hda-dsp: - (hw:1,7)`, PCM mono 16-bit, `16000 Hz`).
- Lưu ý: index `4` (`hw:1,0`) trên ThinkPad sẽ báo `Invalid sample rate` nếu mở ở 16 kHz. Nếu index thay đổi sang số khác, truyền `device:=<index_mới>` vào launch file hoặc `-p device:=<index_mới>` cho `stt_node`. Dùng `device:=-1` nếu muốn theo thiết bị mặc định của hệ thống.

### 4.3 Kết nối Loa qua PulseAudio (`paplay`) & Khôi phục sau khi khởi động lại máy
Mặc định `tts_node` dùng `engine:=piper` và `player:=paplay` (lấy socket từ biến môi trường `PULSE_SERVER`, không hardcode trong mã nguồn).

**Nếu sau khi khởi động lại Ubuntu host mà container báo lỗi `pa_context_connect() failed: Connection refused` hoặc không thấy `/home/ws/.voice-audio/native`:**
1. **Trên Terminal Ubuntu Host (ngoài container)**, kiểm tra PulseAudio và mở lại Unix socket cho workspace `/home/thuytran/ws/.voice-audio/native`:
   ```bash
   # Trên HOST Ubuntu 22.04:
   mkdir -p ~/ws/.voice-audio
   pactl info | grep -E "Server Name|Default Sink"
   pactl list short modules | grep -F "socket=$HOME/ws/.voice-audio/native" || \
     pactl load-module module-native-protocol-unix auth-anonymous=1 socket=$HOME/ws/.voice-audio/native
   ls -la ~/ws/.voice-audio/native
   ```
2. **Trong Terminal Dev Container**, trỏ `PULSE_SERVER` và kiểm tra loa + mic thực tế:
   ```bash
   export PULSE_SERVER=unix:/home/ws/.voice-audio/native

   # Kiểm tra kết nối tới PulseAudio server của host:
   pactl info || paplay --version

   # Thu thử 3 giây từ mic ThinkPad hw:1,7 (16 kHz mono 16-bit):
   arecord -D hw:1,7 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_record.wav

   # Phát thử ra loa qua PulseAudio (paplay):
   paplay /tmp/test_record.wav
   ```
*Lưu ý*: Nếu kiểm thử tự động/headless hoặc trên Gazebo không cần loa thật, truyền `tts_engine:=log`.

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

#### 3. Chạy toàn bộ 122 kiểm thử pytest trong ROS 2 Jazzy (`ROS_DOMAIN_ID=42`):
```bash
# Thư mục: /home/ws
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Chỉ chạy nhóm Audio/STT/TTS (28 tests: Faster-Whisper NumPy float32, endpointing, anti-echo, paplay, Vosk backup):
python3 -m pytest src/airport_robot_voice/test/test_audio.py -v

# Chạy toàn bộ 122 tests (Audio, NLU, Places, Planner, Lifecycle, Phase 9 Escort):
python3 -m pytest src/airport_robot_voice/test/ -v
```
*Dấu hiệu thành công*: `28 passed` cho `test_audio.py` và **`122 passed`** cho toàn bộ suite (`test_audio.py` 28 + `test_nlu.py` 28 + `test_places.py` 7 + `test_planner_resolution.py` 8 + `test_ros_lifecycle.py` 38 + `test_phase9_escort_integration.py` 13).

---

### 6.2 Khởi động trọn gói bằng Launch file (Khuyến nghị)

Package hỗ trợ cả hai tên launch file tương đương: `voice_system.launch.py` và `voice_assistant.launch.py`.

#### Bảng tham số cấu hình Launch Argument (`voice_system.launch.py` / `voice_assistant.launch.py`):
| Tham số | Giá trị mặc định | Giá trị khả dụng | Ý nghĩa |
|---|---|---|---|
| `mode` | `mic` | `mic`, `keyboard`, `wav` | Nguồn đầu vào của giọng nói (`mic` mặc định cho launch) |
| `stt_backend` | `faster_whisper` | `faster_whisper`, `vosk` | Backend STT (`faster_whisper` mặc định, `vosk` dự phòng) |
| `wav_file` | `""` | Đường dẫn file .wav | File âm thanh đầu vào khi `mode:=wav` |
| `device` | `9` | `9`, `-1`, `0`, `1`, ... | Index micro `sounddevice` (`9` = ThinkPad DMIC `hw:1,7`, `-1` = mặc định) |
| `sample_rate` | `16000` | `16000`, `48000`, ... | Tần số lấy mẫu thu âm (Hz) |
| `vosk_model` | `/opt/vosk/vosk-model-small-vn-0.4` | Đường dẫn model | Model Vosk tiếng Việt offline |
| `whisper_model` | `small` | `tiny`, `base`, `small`, ... | Tên model Faster-Whisper (hoặc đường dẫn cục bộ) |
| `whisper_model_path` | `""` | Đường dẫn thư mục model CT2 | Ghi đè đường dẫn cục bộ tới model Faster-Whisper |
| `whisper_cache_dir` | `""` | Đường dẫn cache HF | Thư mục `download_root` chứa model cache offline |
| `whisper_python` | `""` | Đường dẫn Python venv | Python cho worker Faster-Whisper (rỗng = tự dò `~/.venvs/airport-whisper-test/bin/python`) |
| `whisper_device` | `cpu` | `cpu`, `cuda` | Thiết bị suy luận Faster-Whisper |
| `whisper_compute_type` | `int8` | `int8`, `float16`, `float32` | Kiểu lượng tử hóa CTranslate2 |
| `whisper_language` | `vi` | `vi`, `en`, ... | Ngôn ngữ nhận diện |
| `whisper_beam_size` | `5` | Số nguyên >= 1 | Beam size của Faster-Whisper |
| `whisper_local_files_only` | `true` | `true`, `false` | Chạy thuần offline từ cache/model đã chuẩn bị |
| `silence_duration_sec` | `0.8` | Số thực (giây) | Thời gian im lặng để kết thúc câu khi dùng mic Faster-Whisper |
| `vad_rms_threshold` | `0.005` | Số thực `float32` | Ngưỡng RMS phân biệt tiếng nói và nhiễu nền micro |
| `max_utterance_sec` | `12.0` | Số thực (giây) | Độ dài tối đa một câu nói |
| `post_tts_cooldown_sec` | `0.7` | Số thực (giây) | Thời gian khóa mic sau khi loa ngừng phát để xả đuôi PulseAudio/phòng |
| `tts_echo_suppression_sec` | `3.5` | Số thực (giây) | Cửa sổ lọc transcript phản hồi từ câu nói TTS vừa phát |
| `tts_engine` | `piper` | `piper`, `log`, `espeak` | Engine TTS mặc định (`piper` cho giọng thật, `log` cho test/Gazebo) |
| `tts_player` | `paplay` | `paplay`, `aplay`, `ffplay` | Trình phát cho Piper (`paplay` dùng `PULSE_SERVER` từ env, không truyền `-q`) |
| `piper_model` | `/opt/piper/vi_VN-vais1000-medium.onnx` | Đường dẫn `.onnx` | Model giọng đọc Piper tiếng Việt |
| `tts_policy` | `flexible` | `flexible`, `strict` | Chính sách: `strict` hủy goal nếu TTS lỗi; `flexible` tiếp tục |
| `use_sim_time` | `false` | `true`, `false` | Đồng bộ `/clock` cho Gazebo |
| `places_file` | `places.json` | Đường dẫn JSON | Danh mục tọa độ điểm đến |
| `use_nav` | `true` | `true`, `false` | Bật/tắt `nav_command_node` |
| `launch_stt` | `true` | `true`, `false` | Bật/tắt `stt_node` |
| `enable_escort` | `false` | `true`, `false` | Bật tích hợp Escort Tracking (Phase 9) |

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

##### C. Chế độ Thực tế mặc định (Micro ThinkPad 9 → Faster-Whisper → Piper → `paplay`):
```bash
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
export TZ=Asia/Ho_Chi_Minh
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py use_nav:=true
# (Với chế độ voice độc lập chưa bật Nav2, truyền use_nav:=false)
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
1. **Bước 1 — Dừng goal qua đường Voice đã kiểm chứng & Xác minh Robot dừng hẳn**:
   - **Lưu ý kỹ thuật quan trọng**:
     + Tuyệt đối **không dùng** lệnh `ros2 topic pub --once /cmd_vel ...` hoặc pub lặp zero vào `/cmd_vel` như một cơ chế dừng khẩn cấp. Trên cấu hình robot Mecanum đang chạy với Nav2, `controller_server` liên tục tính toán và xuất topic `/cmd_vel` theo chu kỳ vòng lặp 10–20 Hz. Một bản tin `--once` sẽ ngay lập tức bị ghi đè bởi chu kỳ tiếp theo của controller nếu goal Nav2 vẫn đang active; còn lệnh pub lặp tần số cao sẽ tranh chấp trực tiếp với controller gây giật xung đột trên motor.
     + Tuyệt đối **không dùng** lệnh `ros2 action send_goal /navigate_to_pose ... "{}"` vì lệnh đó sẽ gửi một goal rỗng mới tới Nav2 thay vì hủy goal cũ.
   - **Cơ chế dừng đã kiểm chứng trên hệ thống (Phần mềm / Mô phỏng)**:
     Sử dụng đường cancel goal chuẩn đã được tích hợp và kiểm chứng của hệ thống Voice:
     ```bash
     # Cách 1: Gửi lệnh thoại hủy chuyến qua topic text:
     ros2 topic pub --once /voice/text std_msgs/msg/String "{data: 'hủy chuyến'}"

     # Hoặc Cách 2: Phát trực tiếp intent hủy chuẩn Schema v1.0:
     ros2 topic pub --once /voice/intent std_msgs/msg/String \
       '{"schema_version": "1.0", "request_id": "req_op_cancel", "raw": "hủy chuyến", "intent": "cancel_robot", "action": "cancel"}'
     ```
     `nav_command_node` sẽ gọi `cancel_goal_async()` lên đúng goal handle đang hoạt động trên Nav2 và chuyển sang trạng thái `CANCELING`.
   - **Cơ chế dừng trên robot thật (Phần cứng)**:
     > [!WARNING]
     > **[CHỜ NGHIỆM THU PHẦN CỨNG]**: Các cơ chế dừng khẩn cấp vật lý trên robot thật (như nút nhấn E-stop cơ học ngắt nguồn motor, công tắc an toàn hoặc phanh điện từ) **chưa được xác minh thực tế** trên phần cứng của cấu hình này. Khi triển khai trên robot thật, phải thực hiện nghiệm thu riêng cho cơ chế E-stop phần cứng trước khi vận hành.
   - **Xác nhận terminal của đúng goal**:
     Giám sát topic `/voice/feedback` và `/voice/nav_state`:
     ```bash
     # Xác nhận feedback terminal của đúng goal attempt và UUID:
     ros2 topic echo /voice/feedback
     ```
     *Dấu hiệu xác nhận*: Nhận được bản tin `event: "CANCELED"`, `status: "OK"`, đúng `goal_attempt_id` và `nav2_uuid` của chuyến đi đang chạy.
     ```bash
     # Xác nhận trạng thái máy nhiệm vụ đã sạch goal:
     ros2 topic echo /voice/nav_state --once --qos-durability transient_local
     ```
     *Dấu hiệu xác nhận*: `mission_state: "IDLE"`, `has_unresolved_goals: false`, `current_target_id: null`.
   - **Xác nhận robot đã dừng hẳn**:
     Kiểm tra vận tốc thực tế từ `/odom` trong một cửa sổ dừng liên tục ít nhất 2 giây với timestamp tăng nghiêm ngặt:
     ```bash
     ros2 topic echo /odom
     ```
     *Dấu hiệu xác nhận*: Cả vận tốc tuyến tính $v = \sqrt{v_x^2 + v_y^2} \le 0.02$ m/s và vận tốc góc $|\omega_z| \le 0.02$ rad/s ở mọi mẫu trong cửa sổ dừng.

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
     > **Phạm vi của Restart**: Việc khởi động lại Voice đưa máy trạng thái về `IDLE` **chỉ làm sạch bộ nhớ cục bộ trong phiên của node Voice mới, tuyệt đối không chứng minh goal cũ trên Nav2 đã dừng**. Node mới không kế thừa attempt cũ, nhưng người vận hành **bắt buộc phải hoàn thành Bước 1 và Bước 2** để chắc chắn robot vật lý đã dừng và Nav2 không còn goal mồ côi trước khi ra lệnh dẫn đường mới.
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
  alsa-utils pulseaudio-utils espeak-ng libportaudio2 python3-pyaudio python3-pip python3-venv

# 2. Cài đặt thư viện Python hệ thống cho Piper / SoundDevice / Vosk dự phòng
pip install --user --break-system-packages sounddevice piper-tts vosk
sudo ln -sf ~/.local/bin/piper /usr/local/bin/piper

# 3. Cài đặt Faster-Whisper trong venv riêng & tải trước model 'small' cho chế độ offline
python3 -m venv ~/.venvs/airport-whisper-test
~/.venvs/airport-whisper-test/bin/pip install --upgrade pip faster-whisper numpy
~/.venvs/airport-whisper-test/bin/python -c "from faster_whisper import WhisperModel; WhisperModel('small', device='cpu', compute_type='int8')"

# 4. Tải model Piper tiếng Việt
sudo mkdir -p /opt/piper && sudo chown -R $USER:$USER /opt/piper && cd /opt/piper
curl -L -s -o vi_VN-vais1000-medium.onnx https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx
curl -L -s -o vi_VN-vais1000-medium.onnx.json https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/vi/vi_VN/vais1000/medium/vi_VN-vais1000-medium.onnx.json

# 5. (Dự phòng) Tải model Vosk
sudo mkdir -p /opt/vosk && sudo chown -R $USER:$USER /opt/vosk && cd /opt/vosk
python3 -c 'import urllib.request, zipfile, os; urllib.request.urlretrieve("https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip", "m.zip"); zipfile.ZipFile("m.zip").extractall("/opt/vosk"); os.remove("m.zip")'

# 6. Build và chạy bộ test package
cd /home/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select airport_robot_voice --symlink-install
source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=42
python3 src/airport_robot_voice/scripts/test_nlu.py
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
*Lưu ý*: Mặc định cả `voice_system.launch.py` lẫn `stt_node` đều đặt `device:=9` (micro DMIC ThinkPad `hw:1,7`, PCM 16 kHz) và `stt_backend:=faster_whisper`. Nếu index thiết bị trên máy khác với `9`, truyền `device:=<index>` (hoặc `-1` cho thiết bị mặc định hệ thống).

---

### 9.2 Thử nghiệm nhanh Thu âm và Nghe lại (WAV Check trên Mic 9 + PulseAudio `paplay`)

Để xác nhận phần cứng micro (index 9, `hw:1,7`) và loa (`paplay`) hoạt động chuẩn trước khi chạy node ROS:
```bash
# 1. Thu âm 3 giây từ mic ThinkPad hw:1,7 (device=9, 16 kHz mono 16-bit):
arecord -D hw:1,7 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_mic.wav

# 2. Phát lại ra loa qua PulseAudio socket host:
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
paplay /tmp/test_mic.wav

# 3. Thử nhận dạng offline bằng Faster-Whisper (small, CPU, int8, offline):
~/.venvs/airport-whisper-test/bin/python -c "
import wave, numpy as np
from faster_whisper import WhisperModel
with wave.open('/tmp/test_mic.wav', 'rb') as wf:
    audio = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
m = WhisperModel('small', device='cpu', compute_type='int8', local_files_only=True)
segs, _ = m.transcribe(audio, language='vi', beam_size=5, vad_filter=True, without_timestamps=True, condition_on_previous_text=False)
print('Faster-Whisper:', ' '.join(s.text.strip() for s in segs if s.text.strip()))
"

# (Tùy chọn dự phòng) Thử trên mô hình Vosk nếu cần đối chiếu:
python3 -c "
from vosk import Model, KaldiRecognizer
import wave, json
with wave.open('/tmp/test_mic.wav', 'rb') as wf:
    rec = KaldiRecognizer(Model('/opt/vosk/vosk-model-small-vn-0.4'), wf.getframerate())
    rec.AcceptWaveform(wf.readframes(wf.getnframes()))
    print('Vosk:', repr(json.loads(rec.FinalResult()).get('text', '')))
"
```

---

### 9.3 Khởi chạy nhanh 1 Lệnh (Khuyến nghị) hoặc Tách 3 Terminal Debug (`ROS_DOMAIN_ID=42`)

#### Cách 1 (Khuyến nghị — 1 lệnh duy nhất cho toàn bộ Voice độc lập):
```bash
export ROS_DOMAIN_ID=42
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
export TZ=Asia/Ho_Chi_Minh
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false
```

#### Cách 2 (Tách 3 Terminal riêng để quan sát log từng node):
Mở 3 Terminal riêng biệt trong VS Code (nhấn `Ctrl + Shift + \`` để mở terminal mới). Mỗi terminal đặt `export ROS_DOMAIN_ID=42`:

#### Terminal 1: Khởi động Phát âm thanh TTS ra Loa thật
```bash
export ROS_DOMAIN_ID=42
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Mặc định chạy Piper + paplay (hoặc thêm -p engine:=espeak / -p player:=aplay):
ros2 run airport_robot_voice tts_node
```
*(Node này tổng hợp âm thanh Piper và phát trực tiếp ra loa qua PulseAudio `paplay`).*

#### Terminal 2: Khởi động Lõi Xử lý Ngôn ngữ Tự nhiên NLU
```bash
export ROS_DOMAIN_ID=42
export TZ=Asia/Ho_Chi_Minh
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Chạy Intent Node ở chế độ độc lập (không cần Nav2):
ros2 run airport_robot_voice intent_node
```
*(Node này phân tích ý định từ giọng nói, trả lời các thông tin offline và thông báo an toàn nếu yêu cầu dẫn đường khi Nav2 chưa bật).*

#### Terminal 3: Khởi động Nhận diện Giọng nói STT từ Micro thật
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Mặc định chạy Faster-Whisper (small/cpu/int8/vi) trên mic 9 (hw:1,7), 16000 Hz:
ros2 run airport_robot_voice stt_node --ros-args -p mode:=mic
# Nếu muốn đổi sang Vosk dự phòng: -p stt_backend:=vosk
```
*(Mặc định `device:=9`, `stt_backend:=faster_whisper`. Nếu cần đổi mic: thêm `-p device:=<index>`).*

*(Tùy chọn) Terminal 4: Giám sát văn bản và intent thời gian thực*:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 topic echo /voice/text
# Hoặc xem intent & target_id: ros2 topic echo /voice/intent
```

---

### 9.4 Bảng Câu Tiếng Việt Thử Nghiệm Tuần Tự (Kèm Chống Tự Thu Tiếng Loa)

Ngồi trước máy tính, lần lượt nói rõ ràng vào Micro theo 4 câu dưới đây (mỗi câu cách nhau sau khi robot dứt lời để kiểm tra cả phản hồi lẫn chống echo):

| STT | Câu bạn cần nói vào Micro | Transcript (/voice/text) | Intent phân tích (/voice/intent) | Phản hồi mong đợi phát ra Loa (`use_nav:=false`) | Bước thao tác của bạn |
|---|---|---|---|---|---|
| **Câu 1** | **"Mấy giờ rồi đấy?"** | `"Mấy giờ rồi đấy?"` | `intent: ask_time`<br>`action: None`<br>`target: None` | *"Bây giờ là [HH] giờ [MM] phút."* | **[NÓI]** Nói vào micro.<br>**[NGHE]** Loa Piper đọc đúng giờ hiện tại và STT **không** nhận lại chính câu robot vừa đọc. |
| **Câu 2** | **"Dẫn tôi đến cổng A3"** | `"Dẫn tôi đến cổng A3"`<br>*(hoặc `"cổng a3 ở đâu"`)* | `intent: guide_me`<br>`action: navigate`<br>`target_id: gate_a3` | *"Đã xác định yêu cầu tới cổng A3, nhưng hệ thống điều hướng chưa sẵn sàng."* | **[NÓI]** Nói dẫn đường tới cổng A3.<br>**[NGHE]** Loa báo đã nhận cổng A3 nhưng chưa bật Nav2 (`use_nav:=false`). |
| **Câu 3** | **"Nhà vệ sinh ở đâu?"** | `"Nhà vệ sinh ở đâu?"` | `intent: ask_toilet`<br>`action: navigate`<br>`category: toilet` | *"Đã xác định yêu cầu tới nhà vệ sinh, nhưng hệ thống điều hướng chưa sẵn sàng."* | **[NÓI]** Hỏi vị trí nhà vệ sinh.<br>**[NGHE]** `/voice/intent` ra đúng `category: toilet` và loa đọc câu thông báo độc lập. |
| **Câu 4** | **"Hủy chuyến"** | `"Hủy chuyến"` | `intent: cancel_robot`<br>`action: cancel`<br>`target: None` | *"Hiện không có chuyến đi nào đang hoạt động."* | **[NÓI]** Ra lệnh hủy chuyến.<br>**[NGHE]** `/voice/intent` bắt đúng `action: cancel` và không bị lặp lại câu TTS qua mic. |

---

### 9.5 Cách thoát sạch sau khi thử xong
Nhấn **`Ctrl + C`** tại Terminal chạy `voice_system.launch.py` (hoặc tại mỗi Terminal 1, 2, 3 nếu chạy tách node). Hệ thống tự giải phóng micro ALSA và tắt sạch worker Faster-Whisper.

---

### 9.6 Lệnh chạy lại toàn bộ Bộ Kiểm thử và Nghiệm thu Gazebo
Nếu muốn chạy lại bất kỳ lúc nào:
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# 1. Toàn bộ 122 bài test pytest (Cô lập ROS_DOMAIN_ID=42):
export ROS_DOMAIN_ID=42
python3 -m pytest src/airport_robot_voice/test/ -v

# 2. Kiểm thử 61 câu tiếng Việt NLU:
python3 src/airport_robot_voice/scripts/test_nlu.py

# 3. Toàn bộ 3 ca nghiệm thu chuyên sâu trên mô phỏng Gazebo Sim + Nav2:
# (Gồm: A3->A4->Hủy (v=0), A3 tới đích SUCCEEDED, Đo đường 2 Toilet & Chọn đúng điểm ngắn nhất)
bash src/airport_robot_voice/scripts/run_evidence_verification.sh
```


---

## 10. Triển khai và Vận hành Tích hợp Phân hệ Person Tracking (Phase 9)

Tài liệu tham chiếu thiết kế: `src/.agent/person_tracking/INTERFACES.md` và `VOICE_INTEGRATION_REQUEST.md`.  
Phân hệ Voice đã hoàn thiện triển khai tích hợp chính thức tại **Phase 9**, hỗ trợ chuyển đổi linh hoạt qua cờ tham số `enable_escort`.

### 10.1 Phân định Quyền hạn và Trách nhiệm
1. **Voice là Chủ sở hữu DUY NHẤT của `NavigateToPose` Action**:
   - Node `nav_command_node.py` của phân hệ Voice toàn quyền khởi tạo `ActionClient<NavigateToPose>`, gửi Goal, theo dõi feedback và hủy Goal.
   - Phân hệ Person Tracking **tuyệt đối không** khởi tạo `ActionClient<NavigateToPose>`, không gửi goal và không gọi cancel trực tiếp trên Nav2.
2. **Tracking là Cơ quan Cấp Phiên và Quản lý Khóa Khách hàng**:
   - Tracking phát hiện hành khách qua camera, nhận diện cử chỉ giơ tay để khóa khách hàng, sinh `session_id`, và quản lý cổng an toàn vận tốc motor (Velocity Guard & Velocity Watchdog).

### 10.2 Tham số Cấu hình Chế độ Escort
| Tham số | Kiểu | Mặc định | Mô tả |
|---------|------|----------|-------|
| `enable_escort` | `bool` | `false` | Bật/tắt chế độ tích hợp Person Tracking. Khi `false`, node hoạt động độc lập (Standalone Voice). |
| `session_state_topic` | `string` | `/tracking/session_state` | Topic lắng nghe trạng thái phiên từ Tracking (QoS: Transient Local, depth 1). |
| `safety_request_topic` | `string` | `/tracking/safety_request` | Topic lắng nghe yêu cầu dừng an toàn (ABORT) từ Tracking (QoS: Volatile, depth 10). |
| `nav_status_topic` | `string` | `/voice/nav_status` | Topic phát trạng thái điều hướng 2 pha tới Tracking (QoS: Transient Local, depth 1). |
| `escort_heartbeat_timeout_sec` | `float` | `1.0` | Thời hạn tối đa cho phép của heartbeat từ Tracking. Quá thời hạn này sẽ chặn dispatch. |

### 10.3 Khởi chạy Chế độ Tích hợp Escort
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# Khởi chạy hệ thống voice hoàn chỉnh với chế độ Escort bật:
ros2 launch airport_robot_voice voice_system.launch.py \
    enable_escort:=true \
    mode:=keyboard \
    tts_engine:=log \
    tts_policy:=flexible

# Hoặc khởi chạy độc lập node nav_command_node:
ros2 run airport_robot_voice nav_command_node --ros-args \
    -p enable_escort:=true \
    -p use_sim_time:=true \
    -p places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json
```

### 10.4 Lệnh Kiểm thử và Nghiệm thu Phase 9
```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash

# 1. Chạy toàn bộ 107 tests pytest (gồm 13 bài test Phase 9 + 94 bài test hồi quy):
export ROS_DOMAIN_ID=42
python3 -m pytest src/airport_robot_voice/test/ -v

# 2. Chạy kịch bản nghiệm thu 4 ca thực tế trên đồ thị Gazebo Escort + Nav2:
bash src/airport_robot_voice/scripts/execute_phase9_verification.sh
```

### 10.5 Kết quả Nghiệm thu 4 Ca Thực tế trong Gazebo
- **File kịch bản**: `airport_robot_voice/scripts/verify_phase9_gazebo_escort.py`
- **File log vết thực thi**: [.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log](file:///home/thuytran/ws/src/.agent/airport_robot_voice/evidence/phase9_gazebo_escort_smoke.log)
- **Tóm tắt kết quả (ĐẠT 100% 4/4 CA)**:
  1. **Ca 1 (Chặn dispatch trước khóa)**: Gating chặn dispatch khi chưa khóa khách (`REJECTED`, `ESCORT_NO_SESSION_ID`), phát TTS cảnh báo, không gửi goal tới Nav2, robot đứng yên tuyệt đối. -> **PASSED [OK]**
  2. **Ca 2 (Khởi hành và dẫn đường thực tế)**: Khách khóa phiên thành công, Nav2 chấp nhận goal, Voice phát `NAVIGATION_STARTED` đủ 5 định danh, robot di chuyển vật lý trong Gazebo (`max_vx=0.500 m/s`, $\Delta d = 0.658$ m). -> **PASSED [OK]**
  3. **Ca 3 (Mất khách -> xe dừng -> safety ABORT -> ACK -> CANCELED -> IDLE)**: Mất quan sát, Velocity Guard hạ `cmd_vel` về 0, Tracking gửi `ABORT`, Voice phát non-terminal ACK `CANCEL_ACKNOWLEDGED`, gọi `cancel_goal_async()`, Nav2 trả `CANCELED`, Voice phát `GOAL_COMPLETED` terminal, robot dừng hẳn $\ge 2.0$s, Tracking trở về `IDLE`. -> **PASSED [OK]**
  4. **Ca 4 (Đến đích SUCCEEDED và IDLE sạch sẽ)**: Khóa Session 2, dẫn đường tới cổng 15, robot về đích, Nav2 trả `STATUS_SUCCEEDED`, Voice phát terminal `SUCCEEDED`, Tracking chuyển `ARRIVED` -> `IDLE`, Voice FSM về `IDLE` sạch sẽ. -> **PASSED [OK]**

---

## 11. Bộ Lệnh Vận hành Chuẩn Hóa (Mặc định Faster-Whisper + Mic 9 + Piper + `paplay`)

### 11.1 Test bằng Keyboard và TTS Log (không cần micro/loa)
```bash
# Terminal 1 (Voice không Nav2 hoặc thêm use_nav:=true nếu đã bật Nav2):
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py \
  use_nav:=false launch_stt:=false tts_engine:=log

# Terminal 2 (STT bàn phím độc lập):
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
```

### 11.2 Test Faster-Whisper bằng file WAV (lặp lại offline)
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py \
  use_nav:=false mode:=wav wav_file:=/tmp/robot-mic-time.wav tts_engine:=log
```

### 11.3 Test Micro và Loa thật độc lập — Lệnh Launch Mặc định (`use_nav:=false`)
*(Mặc định đã cấu hình sẵn `mode:=mic`, `stt_backend:=faster_whisper`, `whisper_model:=small`, `cpu`, `int8`, `vi`, `beam_size:=5`, `device:=9`, `sample_rate:=16000`, `tts_engine:=piper`, `tts_player:=paplay`, kèm chống tự thu tiếng loa `post_tts_cooldown_sec:=0.7` & `tts_echo_suppression_sec:=3.5`)*:

```bash
cd /home/ws && source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=42
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
export TZ=Asia/Ho_Chi_Minh

ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false
```

*(Terminal phụ theo dõi realtime trên `ROS_DOMAIN_ID=42`)*:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 topic echo /voice/text
# Hoặc xem JSON intent & target_id:
# ros2 topic echo /voice/intent
```

*4 câu nói thử nghiệm chuẩn trên micro (`use_nav:=false`)*:
1. **"Mấy giờ rồi đấy?"** $\rightarrow$ `intent=ask_time`, loa đọc `"Bây giờ là [HH] giờ [MM] phút."`.
2. **"Dẫn tôi đến cổng A3"** $\rightarrow$ `intent=guide_me`, `target_id=gate_a3`, loa đọc `"Đã xác định yêu cầu tới cổng A3, nhưng hệ thống điều hướng chưa sẵn sàng."`.
3. **"Nhà vệ sinh ở đâu?"** $\rightarrow$ `intent=ask_toilet`, `category=toilet`, loa đọc `"Đã xác định yêu cầu tới nhà vệ sinh, nhưng hệ thống điều hướng chưa sẵn sàng."`.
4. **"Hủy chuyến"** $\rightarrow$ `intent=cancel_robot`, `action=cancel`, loa đọc `"Hiện không có chuyến đi nào đang hoạt động."`.
*(Tiện kiểm tra thêm từ khóa `cổng A4`, `cổng B2`, `dừng lại`, `đi tiếp`, `đổi sang cổng A4` và thử nói ngay sau khi loa vừa đọc xong để kiểm tra không lặp).*

### 11.4 Test Voice + Nav2 trong Gazebo
```bash
# Terminal 1 (Gazebo turtlebot3_world):
export LINOROBOT2_BASE=mecanum ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=turtlebot3_world

# Terminal 2 (Nav2):
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch linorobot2_navigation navigation.launch.py sim:=true \
  map:=/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml rviz:=true

# Terminal 3A (Voice dùng Mic thật + Loa thật trên Gazebo):
export PULSE_SERVER=unix:/home/ws/.voice-audio/native TZ=Asia/Ho_Chi_Minh
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py \
  use_nav:=true use_sim_time:=true \
  places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json

# Hoặc Terminal 3B (Kiểm thử Gazebo bằng Keyboard + TTS log / chạy script nghiệm thu tự động):
# ros2 launch airport_robot_voice voice_system.launch.py use_nav:=true launch_stt:=false tts_engine:=log use_sim_time:=true places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json
# bash src/airport_robot_voice/scripts/run_evidence_verification.sh
```

### 11.5 Test Voice + `person_tracking` + Nav2 trong Gazebo (Escort Mode)
```bash
# Hoặc chạy trọn bộ smoke test Gate 9 cô lập trên ROS_DOMAIN_ID=44:
bash /home/ws/src/airport_robot_voice/scripts/execute_phase9_verification.sh

# Hoặc chạy từng terminal thủ công:
# Terminal 1: Khởi chạy Gazebo + Nav2 + Escort Tracking:
source /opt/ros/jazzy/setup.bash
[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash
[ -f /home/ws/install_tracking/setup.bash ] && source /home/ws/install_tracking/setup.bash
ros2 launch person_tracking gazebo_escort_integration.launch.py gui:=false enable_test_scan:=true enable_watchdog:=true

# Terminal 2: Khởi chạy Voice với enable_escort:=true:
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py \
  enable_escort:=true use_sim_time:=true \
  places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json \
  tts_engine:=log launch_stt:=false
# (Hoặc bỏ tts_engine:=log launch_stt:=false nếu muốn dùng trực tiếp micro + loa với enable_escort:=true)
```

### 11.6 Chuyển backend về Vosk hoặc thay đổi Model / Device
```bash
# Chuyển sang backend Vosk dự phòng (cùng mic 9, 16 kHz):
ROS_DOMAIN_ID=42 PULSE_SERVER=unix:/home/ws/.voice-audio/native \
  ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false stt_backend:=vosk

# Đổi model Faster-Whisper lớn hơn (ví dụ medium hoặc large-v3, GPU cuda float16):
ROS_DOMAIN_ID=42 PULSE_SERVER=unix:/home/ws/.voice-audio/native \
  ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false \
  whisper_model:=medium whisper_device:=cuda whisper_compute_type:=float16
```
