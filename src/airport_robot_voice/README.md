# airport_robot_voice

Hệ thống trợ lý giọng nói (STT + NLU rule-based tiếng Việt + TTS) cho robot hỗ trợ hành khách sân bay trên nền tảng **ROS 2 Jazzy**. Hoạt động hoàn toàn **offline**, không phụ thuộc LLM hay API đám mây.

- **STT chính (mặc định)**: `faster_whisper` (`model=small`, `cpu`, `int8`, `language=vi`, `beam_size=5`, `16000 Hz`, đọc PCM16 -> NumPy `float32` qua worker sống lâu trong `~/.venvs/airport-whisper-test`).
- **STT dự phòng**: `vosk` (`/opt/vosk/vosk-model-small-vn-0.4`), chỉ dùng khi truyền `stt_backend:=vosk`.
- **Micro mặc định (ThinkPad P15 Gen 2)**: `device:=9` (`hw:1,7`), mono 16-bit, `sample_rate:=16000`.
- **TTS mặc định (giọng nói thật)**: `piper` (`/opt/piper/vi_VN-vais1000-medium.onnx`) phát qua `paplay` (dùng `PULSE_SERVER` từ môi trường, ví dụ `unix:/home/ws/.voice-audio/native`), hỗ trợ `tts_engine:=log` / `espeak` khi test không cần loa.
- **Chống tự thu tiếng loa**: khóa thu trong lúc `/voice/speaking=True` + trễ xả đuôi loa `post_tts_cooldown_sec` (0.7s) + lọc echo theo `/voice/say` và `/voice/tts_ack`.

> [!TIP]
> **Tài liệu Cài đặt & Vận hành Đầy đủ**:
> Xem [COMMANDS_VOICE.md](file:///home/thuytran/ws/src/.agent/COMMANDS_VOICE.md) cho quy trình đầy đủ theo từng kịch bản: keyboard + log, WAV, micro/loa thật độc lập, Gazebo + Nav2, Gazebo + `person_tracking` + Nav2, chuyển về Vosk, cài máy mới và khôi phục PulseAudio sau reboot.

---

## 1. Kiến trúc hệ thống

```
[Người dùng]
     │ (Micro 16kHz / Bàn phím / WAV)
     ▼
[stt_node] ────── /voice/text ──────► [intent_node] ────── /voice/say ──────► [tts_node] ──► [paplay / aplay / Log]
(Faster-Whisper                         (NLU Offline)       (std_msgs/String)   (Piper / eSpeak / Log)
 mặc định; Vosk dự phòng)                 │                                            │
     ▲                               /voice/intent                                     │
     └────────── /voice/speaking & /voice/tts_ack (chống tự thu tiếng loa) ◄───────────┘
                                          │
                                          ▼
                                 [nav_command_node] ◄──── /voice/nav_state (Transient Local)
                                    (Vòng đời Nav2 & Escort Gating)
                                          │
                                 NavigateToPose / ComputePathToPose
                                          │
                                          ▼
                                     [Nav2 Stack]
```

### Hai chế độ vận hành:
1. **Chế độ Voice độc lập (`use_nav:=false`)**:
   - Hoạt động khi Nav2 chưa chạy; vẫn trích xuất `target_id` / `category` ra `/voice/intent` (ví dụ `gate_a3`, `toilet`) và trả lời thông tin offline (`ask_time`, `ask_wifi`, `ask_charging`, `ask_hours`).
2. **Chế độ Voice kết hợp Navigation (`use_nav:=true`)**:
   - `nav_command_node` quản lý vòng đời Nav2 (`navigate`, `pause`, `resume`, `cancel`, `replace`, `status`, chọn điểm gần nhất qua `ComputePathToPose` và hỗ trợ `enable_escort:=true` cho `person_tracking`).

---

## 2. Build Workspace

Thực hiện trong Terminal của VS Code Dev Container (ROS 2 Jazzy tại `/home/ws`):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ws
colcon build --packages-select airport_robot_voice --symlink-install
source /home/ws/install/setup.bash
```

### Dependencies & Model đã chuẩn bị:
- **Faster-Whisper offline**: venv `~/.venvs/airport-whisper-test` (`faster-whisper==1.2.1`, `numpy`), model `small` cached offline tại `~/.cache/huggingface/hub`.
- **Piper TTS offline**: binary `/usr/local/bin/piper`, model `/opt/piper/vi_VN-vais1000-medium.onnx` (+ `.onnx.json`), phát qua `paplay` (PulseAudio) hoặc `aplay`.
- **Vosk (dự phòng) & SoundDevice**: `vosk`, `sounddevice`, model `/opt/vosk/vosk-model-small-vn-0.4`.

---

## 3. Các lệnh chạy nhanh

Package cung cấp hai launch file tương đương: `voice_system.launch.py` và `voice_assistant.launch.py`.

### A. Lệnh launch mặc định: Micro → Faster‑Whisper → NLU → Piper → Loa (`use_nav:=false`)
*(Cấu hình mặc định đã đặt `mode:=mic`, `stt_backend:=faster_whisper`, `device:=9`, `sample_rate:=16000`, `tts_engine:=piper`, `tts_player:=paplay`)*:

```bash
cd /home/ws && source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=42
export PULSE_SERVER=unix:/home/ws/.voice-audio/native
export TZ=Asia/Ho_Chi_Minh

ros2 launch airport_robot_voice voice_system.launch.py use_nav:=false
```

### B. Chế độ Keyboard + TTS Log (không cần micro/loa)
```bash
# Terminal 1: Khởi động Voice (tắt STT tích hợp để tách stdin)
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 launch airport_robot_voice voice_system.launch.py launch_stt:=false tts_engine:=log use_nav:=false

# Terminal 2: Chạy STT keyboard
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ros2 run airport_robot_voice stt_node --ros-args -p mode:=keyboard
```

### C. Test Faster‑Whisper trên file WAV
```bash
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch airport_robot_voice voice_system.launch.py \
  mode:=wav wav_file:=/tmp/robot-mic-time.wav tts_engine:=log use_nav:=false
```

### D. Chuyển về backend Vosk dự phòng
```bash
ROS_DOMAIN_ID=42 ros2 launch airport_robot_voice voice_system.launch.py \
  use_nav:=false stt_backend:=vosk device:=9 sample_rate:=16000
```

### E. Liệt kê thiết bị micro nếu index thay đổi
```bash
python3 -c "import sounddevice as sd; [print(f'[{i}] {d[\"name\"]} (in={d[\"max_input_channels\"]}, sr={int(d[\"default_samplerate\"])})') for i, d in enumerate(sd.query_devices()) if d.get('max_input_channels', 0) > 0]"
```
*(Truyền `device:=<index_mới>` vào lệnh launch nếu index khác `9`).*

---

## 4. Kiểm thử tự động

```bash
cd /home/ws
source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=42

# 1. Kiểm thử NLU 61 câu tiếng Việt (thuần Python):
python3 src/airport_robot_voice/scripts/test_nlu.py

# 2. Kiểm thử Audio/STT/TTS (28 tests) và toàn bộ Pytest Suite (122 tests):
python3 -m pytest src/airport_robot_voice/test/test_audio.py -v
python3 -m pytest src/airport_robot_voice/test/ -v
```
