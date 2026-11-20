# PERSON TRACKING — WORKFLOW TRIỂN KHAI VÀ NGHIỆM THU

**Dự án:** Robot dẫn đường hành khách sân bay, ROS 2 Jazzy  
**Phân hệ:** `person_tracking`  
**Phiên bản tài liệu:** 1.0 — 20/11/2026  
**Trạng thái:** Đặc tả và kế hoạch; KHÔNG phải xác nhận đã triển khai/chạy thành công.  
**Áp dụng:** Chỉ trong workspace `ws` của người dùng; chủ yếu `~/ws/src/` trên host. Bên trong Docker, xác nhận đường dẫn bind-mount tương ứng trước khi thao tác (có thể là `/home/ws`).

> **QUY ƯỚC ĐIỀU HÀNH:** Đây là tài liệu gốc cho một cuộc hội thoại Antigravity chuyên trách tracking. Phải kiểm tra hiện trạng, hoàn thành từng giai đoạn và đưa bằng chứng trước khi chuyển bước. Không được tuyên bố đã kiểm thử phần cứng khi không có camera/robot thật.

---

## 0. Các quyết định của chủ dự án — bắt buộc tuân thủ

| Mã | Quyết định đã chốt | Hệ quả thiết kế |
|---|---|---|
| D01 | Chỉ đọc/viết/làm việc trên các file thuộc **workspace `ws`**. `~/ros2_ws` đã ngừng sử dụng. | Không truy cập hoặc chỉnh sửa `~/ros2_ws`; không sửa file ngoài `ws`. Toolchain ROS/driver hệ thống chỉ được dùng như công cụ chạy, không chỉnh sửa cài đặt hệ thống. |
| D02 | Giữ nguyên toàn bộ Gazebo, robot description và camera mô phỏng. | Không thêm/sửa cảm biến, plugin, bridge hoặc model Gazebo; chạy test Nav2 với cấu hình sẵn có. |
| D03 | Không có camera Astra thật tại nơi triển khai hiện tại. | Khảo sát package Astra **trong `ws`**, xác định interface từ source/launch/config, phân biệt rõ “được khai báo” với “đã đo lúc chạy”; dùng video/rosbag2/dữ liệu giả lập để test. |
| D04 | `airport_robot_voice` phát triển song song trong một conversation khác. | Không sửa bất kỳ file voice nào, kể cả tài liệu AI của voice; phối hợp bằng hợp đồng interface và yêu cầu gửi cho chủ dự án. |
| D05 | **Voice sở hữu Nav2 goal/action**; tracking **không gửi/hủy trực tiếp `NavigateToPose`**. | Tracking phát trạng thái an toàn và yêu cầu voice hủy/tạm ngừng; đường vận tốc cuối có cơ chế **dừng cưỡng bức** độc lập với độ trễ voice/Nav2. |
| D06 | Khi đến đích thành công, xóa khóa ID và đích, trở về `IDLE`. | Không dùng lại khách cũ cho lượt dẫn mới. |
| D07 | Khi nhiều người cùng giơ tay hợp lệ, ưu tiên người gần camera nhất. | Ưu tiên khoảng cách depth đã kiểm chứng; nếu depth không có, chỉ dùng heuristic 2D có gắn cờ giảm độ tin cậy và tránh chọn khi mơ hồ. |
| D08 | Mất người trong vài frame không được coi ngay là mất vĩnh viễn. | Có khoảng bỏ sót ngắn; khi kéo dài thì dừng chờ, sau đó hủy phiên. Các timeout là tham số cần benchmark. |
| D09 | Ưu tiên thực thi bằng GPU. | Kiểm tra CUDA, driver pass-through và framework sẵn có *trong môi trường đang chạy*; không tự sửa host/Docker bên ngoài vùng cho phép. |
| D10 | Camera cố định, hướng ra phía sau; robot mecanum. | Kiểm tra dấu góc quay, TF, `linear.x/y`, `angular.z` thực tế. Hành khách đi phía sau robot đi trước dẫn đường. |
| D11 | Ban đầu chỉ ROS 2 + terminal; web sẽ làm sau. | Ưu tiên CLI, topic/service/action, logs, rosbag, pytest/launch_testing. |
| D12 | Bảo vệ cấu trúc package giống `airport_robot_voice`. | Code ROS trong package chuẩn; toàn bộ tài liệu/skill AI trong `.agent`, không trộn code vào `.agent`. |

### 0.1. Điều gì *chưa được xác nhận*

- Vị trí package `person_tracking`, Astra, Nav2, firmware, twist mux và launch thật trong checkout local.
- Phiên bản/mẫu camera Astra, tên topic RGB/depth/CameraInfo/PointCloud2, frame ID, encoding, registration, QoS, hiệu năng và extrinsics.
- Có Docker GPU passthrough, runtime CUDA/PyTorch/TensorRT hay không; có camera/robot tại thời điểm chạy hay không.
- Chuỗi Nav2 `cmd_vel`/smoother/collision monitor/base và quyền điều khiển hiện có của `nav_command_node.py`.
- Định dạng giao tiếp voice, tên topic/service/action thật, interface báo hoàn thành goal.

**Không được đưa các mục này vào tài liệu như thể đã kiểm chứng.** Mọi tên topic dưới đây chỉ là tên *đề xuất* tới khi audit xác nhận.

---

## 1. Ranh giới file và quy tắc nhiều agent

### 1.1. Layout tương thích hiện trạng

```text
~/ws/                                # phạm vi thao tác duy nhất trên host
└── src/                              # thực tế do audit xác nhận
    ├── .agent/
    │   ├── airport_robot_voice/      # READ-ONLY cho agent tracking
    │   │   ├── PROJECT_SPEC.md
    │   │   ├── PROGRESS.md
    │   │   └── evidence/
    │   ├── person_tracking/          # agent tracking được tạo/sửa
    │   │   ├── PROJECT_SPEC.md
    │   │   ├── WORKFLOW.md
    │   │   ├── PROGRESS.md
    │   │   ├── INTERFACES.md
    │   │   ├── TEST_PLAN.md
    │   │   ├── DECISIONS.md
    │   │   └── evidence/
    │   ├── rules/
    │   │   └── workspace.md          # chỉ đọc; không ghi đè
    │   ├── skills/
    │   │   ├── ros2-voice-nav/        # chỉ đọc
    │   │   └── ros2-person-tracking/  # được tạo mới, phạm vi độc lập
    │   │       └── SKILL.md
    │   └── COMMANDS.md               # chỉ đọc, nếu tồn tại
    ├── airport_robot_voice/          # chỉ đọc, KHÔNG sửa
    ├── person_tracking/             # code, config, launch, test, model của tracking
    │   ├── ...                       # giữ cấu trúc package thật; KHÔNG áp đặt trước khi audit
    │   └── ...
    ├── <astra, nav2, robot, gazebo, firmware, ...>  # chỉ đọc
    ├── build/                         # chỉ đụng artifact tracking nếu chứng minh được cách ly
    ├── install/                       # chỉ đụng artifact tracking nếu chứng minh được cách ly
    └── log/                           # log tracking riêng, không xóa chung
```

**Lưu ý:** Cây được cung cấp có `build/` và `install/` bên trong `~/ws/src/`, khác bố cục ROS thông thường. Vì vậy **không được đoán colcon root**, không tự di chuyển/sửa cấu trúc đang chạy. Audit phải xác minh `pwd`, `readlink -f`, `git rev-parse --show-toplevel`, `colcon list`, `ros2 pkg prefix`, các mount/bind trong Docker trước khi build.

### 1.2. Allowlist ghi file cho agent tracking

- `~/ws/src/person_tracking/**` hoặc đường dẫn thật được audit xác nhận là package `person_tracking` thuộc `ws`.
- `~/ws/src/.agent/person_tracking/**`.
- `~/ws/src/.agent/skills/ros2-person-tracking/**` (chỉ skill mới của tracking).
- **Artifact build/log riêng tracking** nằm trong `ws`, chỉ sau khi kiểm tra không ảnh hưởng các package/agent khác; ưu tiên base riêng (`--build-base`, `--install-base`, `--log-base`) nếu khả thi với setup hiện tại.

Các đường dẫn khác trong `ws` **chỉ được đọc**, trừ khi người dùng phê duyệt rõ thao tác mở rộng phạm vi. Cấm đọc/ghi file ngoài `ws` bằng thao tác duyệt file, bao gồm `~/ros2_ws`; chỉ được sử dụng các lệnh ROS, Git, compiler và môi trường runtime có sẵn mà không cấu hình/sửa chúng bên ngoài `ws`.

**Không làm:** `git add .`, `git add -A`, `git reset --hard`, `git clean -fdx`, `git checkout`/`switch`/`rebase` trên shared worktree, `rm -rf build install log`, sửa `.gitignore`/`Dockerfile` chung, chạy `sudo apt`/`pip install` global, dịch chuyển package, thay đổi camera/Gazebo/firmware/voice. Xóa file tracking cũ chỉ sau khi audit, có danh sách lý do, xác nhận file thực sự chỉ thuộc tracking, và có khả năng khôi phục bằng Git.

### 1.3. Hai conversation chung một worktree

- Có thể nghiên cứu và viết code **song song ở các thư mục độc lập**.
- Không giao cho cả hai cùng lúc thực hiện thao tác Git thay đổi trạng thái repo hoặc build/clean cùng output. Một người/agent phụ trách Git ở thời điểm commit; stage bằng đường dẫn tường minh, xem `git diff --cached --name-only` trước commit.
- Khi phát hiện thay đổi bất ngờ trong voice, shared config hoặc code robot: **dừng thao tác ghi và báo cáo**, không rollback hay ghi đè thay đổi người khác.
- Không xóa các file `untracked` không phải do tracking tạo. Trước mỗi lượt làm việc: xem `git status --short`, kiểm tra owner phạm vi và các file đang biến đổi.
- Giao tiếp giữa agent bằng `.agent/person_tracking/INTERFACES.md` và bản tóm tắt đề xuất thay đổi cho agent voice. Agent tracking không có quyền tự sửa voice.

---

## 2. Hành vi hệ thống cần đạt

### 2.1. Luồng người dùng

```text
IDLE, robot đứng yên
  -> phát hiện nhiều người trong camera sau
  -> người giơ một tay đủ lâu (mặc định thử 1,5 s)
  -> chọn người hợp lệ gần camera nhất; LOCKED
  -> người có thể hạ tay; robot vẫn đứng yên
  -> voice nhận yêu cầu điểm đến hợp lệ
  -> voice kiểm tra trạng thái khóa và gửi goal cho Nav2
  -> NAVIGATING (robot đi trước, giữ hành khách trong khung ảnh sau)
  -> ARRIVED: voice báo goal thành công -> xóa ID/đích, IDLE
  -> LOST/ABORT: khóa vận tốc 0, yêu cầu voice hủy goal,
                  ngăn hành trình cũ tự chạy lại, xóa phiên, IDLE
```

Không dùng ID cũ để chọn lại khi mất người; khóa phải gắn với **một phiên** (`session_id`) duy nhất. Không tự đổi ID khi hai người đi giao nhau.

### 2.2. Khi đang dẫn khách

| Tình huống | Hành vi tối thiểu |
|---|---|
| Khách trong vùng giữa, khoảng cách phù hợp | Giữ vận tốc Nav2 sau lớp giới hạn an toàn; không tự tăng tốc để giãn khoảng cách. |
| Khách tụt xa, vẫn quan sát tốt | Giảm giới hạn tốc độ tịnh tiến, cần thì dừng chờ. |
| Khách lệch tâm | Điều chỉnh `angular.z` nhẹ có giới hạn (đã kiểm chứng dấu với camera sau); tránh phá tiến độ Nav2 hoặc gây lắc. |
| Khách gần mép ảnh hoặc đang qua cua | Ưu tiên giữ trong khung nhìn, giảm tốc/dừng; cơ chế đi ngang `linear.y` chỉ bật sau khi kiểm thử footprint/đường trống và động học. |
| Mất 1–vài frame đơn lẻ | Duy trì ID, không tự chọn người khác; áp dụng grace period ngắn có giới hạn và không cho phép sử dụng dữ liệu quá cũ để quay/đi nhanh. |
| Mất quan sát kéo dài | Trước hết dừng cưỡng bức, sau ngưỡng cancel thì phát yêu cầu hủy voice và lật trạng thái an toàn; không tự chạy lại khi ảnh trở lại. |
| Xác nhận người ra ngoài khung, đổi ID không chắc chắn, camera lỗi nặng | Dừng ngay; yêu cầu hủy phiên theo quy tắc fail-safe. |
| Nav2 báo đến đích | Xóa ID, điểm đến, timeout; về IDLE, vận tốc 0. |

### 2.3. Timeout dự kiến — **giá trị khởi đầu để hiệu chỉnh, không phải con số đã nghiệm thu**

| Tham số đề xuất | Giá trị khởi đầu | Ghi chú |
|---|---:|---|
| `raise_hand_hold_sec` | 1.5 s | Bộ đếm riêng cho từng track ổn định. |
| `perception_grace_sec` | 0.25 s | Bỏ sót vài frame: không đổi người, không đổi phiên. |
| `visibility_stop_sec` | 0.4 s | Chưa thấy người đến ngưỡng này: đầu ra vận tốc 0, còn giữ phiên để chờ. |
| `visibility_abort_sec` | 1.2 s | Mất tiếp: yêu cầu hủy qua voice, latch STOP; cần một lần khóa mới. |
| `max_observation_age_sec` | 0.4 s | Dữ liệu quá cũ không được tạo lệnh điều khiển mới. |
| `max_cmd_age_sec` | 0.2 s | Đầu vào Nav2 stale -> ra 0; hiệu chỉnh theo chu kỳ thật. |

Nếu FPS thấp hoặc latency cao thì đo lại thay vì tăng timeout tùy tiện. Dừng khi mất quan sát không phụ thuộc voice trả lời. Nếu đang ở gần mép và xác nhận mất khỏi FOV thì xử lý nhanh hơn timeout thường, nhưng **không đánh đồng 1 frame detector bỏ sót với người thực sự đã ra khỏi hình**.

---

## 3. Kiến trúc định hướng, trách nhiệm và giao diện

### 3.1. Thành phần do tracking sở hữu

1. **`passenger_perception`**: đọc RGB, (tùy chọn) depth và CameraInfo; phát hiện nhiều người, pose/keypoint, nhận giơ tay, track ID; tính tâm/độ lệch, chất lượng track, khoảng cách có confidence; trạng thái quan sát; debug overlay/metrics.
2. **`passenger_session` / `escort_tracking_manager`**: FSM cho khóa/giữ ID, phiên, mất dấu, reset/arrived và điều kiện cho phép tiếp tục; xuất sự kiện/mệnh lệnh an toàn sang voice; tuyệt đối **không sở hữu Nav2 action**.
3. **`escort_velocity_guard`**: nhận lệnh vận tốc Nav2 và trạng thái tracking; giảm tốc, chỉnh hướng rất hạn chế khi đủ điều kiện; ra 0 ở trạng thái không an toàn; watchdog, clamp và fault latch. Phải nằm trong chuỗi vận tốc phù hợp với Collision Monitor thực tế.
4. **`tracking_test_publisher`** (tool test chỉ khởi chạy khi test): phát dữ liệu quan sát/depth/goal status giả lập qua interface ROS, không ghi đè topic vận hành thật.

Các tên node là gợi ý; được đổi nếu audit code cho thấy giải pháp tốt hơn nhưng phải giải thích và cập nhật `INTERFACES.md`.

### 3.2. Ranh giới với `airport_robot_voice` và Nav2

```text
Camera Astra thật / video-replay / rosbag
                 |
                 v
      passenger_perception
                 |
        passenger_observation
                 v
    escort_tracking_manager <---- voice: nav state / goal result
                 |   |
    lock/permit  |   +--- abort/pause request ---> voice/nav_command_node
                 |                                  (sở hữu NavigateToPose)
                 v
          escort_velocity_guard <---- Nav2 cmd_vel đã qua smoothing (xác minh)
                 |
                 v
           Collision Monitor (phải xác minh đúng vị trí)
                 |
                 v
            mecanum base / firmware
```

**Không để hai bên cùng gửi/hủy `NavigateToPose`.** Voice phải gate yêu cầu dẫn đường theo trạng thái `LOCKED` và `session_id`, hủy goal khi tracking yêu cầu `ABORT` và báo `SUCCEEDED`/`CANCELED`/`FAILED` với đúng phiên. Tracking phải khóa xuất lệnh ra 0 kể cả voice/Nav2 chưa hủy xong. Nếu voice chưa hỗ trợ interface, giai đoạn tích hợp phải ghi `BLOCKED` hoặc kiểm thử bằng mock voice nav owner — không âm thầm đổi quyền quản lý goal.

### 3.3. Interface dự kiến — chỉ chốt tên sau audit

| Hướng | Trường/ý nghĩa tối thiểu | Chú ý |
|---|---|---|
| Tracking -> voice `tracking/session_state` | `session_id`, `state`, `locked`, `target_id`, `reason`, `stamp` | Có freshness/sequence; voice dùng để gate goal. |
| Tracking -> voice `tracking/safety_request` | `session_id`, `STOP/WAIT/ABORT`, `reason`, `request_id` | Cần acknowledgement và timeout; **STOP độc lập luôn có hiệu lực trên vận tốc**. |
| Voice -> tracking `voice/nav_status` | `session_id`, goal UUID, `ACCEPTED/EXECUTING/SUCCEEDED/CANCELED/FAILED`, `stamp` | Không chấp nhận trạng thái goal cũ, không chấp nhận `SUCCEEDED` sai session. |
| Perception -> manager `tracking/observation` | track ID, bbox, keypoint confidence, gesture, depth m, pixel offset, visibility, stamp | Có đơn vị, frame ID, quality, source mode. |
| Nav2 velocity -> guard | `Twist` hoặc `TwistStamped` **theo audit** | Tránh tự chuyển kiểu message hoặc ghi thẳng `/cmd_vel` trước khi biết subs/remap. |
| Guard -> đế | Vận tốc đã được kiểm tra bởi Collision Monitor | Collision Monitor luôn nhận được lệnh cuối và có quyền cắt. |

Thời gian đầu có thể dùng `std_msgs`/`std_srvs`/message sẵn có để test, nhưng với trạng thái có session và metadata phải thiết kế type đủ biểu đạt, tránh dùng `String` JSON mơ hồ cho interface chính. **Nếu cần tạo package message mới:** nằm trong vùng `person_tracking` dưới cấu trúc hợp lệ hoặc xin duyệt thay đổi allowlist, và phải xem xét giới hạn package ROS 2 khi khai báo interface. Không tạo trùng tên với giao diện voice đang tồn tại.

### 3.4. Điều kiện an toàn bắt buộc

- `NO LOCK`, `WAIT_DESTINATION`, `ABORTED`, `ARRIVED`, `ESTOP`, `DATA_STALE`, `NO_NAV_CMD` -> guard xuất zero, không pass-through lệnh cũ.
- Khi video mất/ROS topic ngắt/TF sai/depth sai: chọn chế độ an toàn rõ ràng; không lấy số đo lần trước dùng vô thời hạn.
- Nếu guard chết hẳn, firmware/base phải có **command watchdog** hoặc lớp độc lập chặn lệnh cũ. Phải kiểm tra đường lệnh thật trước khi thử robot.
- Nếu Navigation goal đã hủy nhưng còn residual cmd_vel, guard vẫn khóa zero cho tới khi phiên mới hợp lệ.
- Không biến việc điều chỉnh góc camera phía sau thành bộ điều khiển di chuyển trái hướng/đâm vào người.
- `linear.y` mecanum chỉ dùng khi biết dấu, footprint, tốc độ, vùng quét chướng ngại vật và collision monitor bao phủ chuyển động ngang.

---

## 4. Chiến lược kiểm thử không cần sửa Gazebo và không cần camera tại chỗ

### 4.1. Các nguồn đầu vào test

1. **Video RGB thực tế** (nhiều người, che khuất, giơ tay, người ra vào mép ảnh): dùng ROS node replay phát `Image` + `CameraInfo` giả lập hợp lý, có stamp, FPS, frame ID. Không ghi là ảnh từ camera Astra thật.
2. **Rosbag2 RGB + depth + CameraInfo** nếu người dùng đã có; replay giữ stamp và QoS phù hợp, kiểm thử sync/registration và ghi evidence.
3. **Synthetic RGB-D fixtures**: bản đồ depth có foreground/background và giá trị pixel có kiểm soát; kiểm tra median depth, invalid/NaN/zero, outlier, đơn vị m/mm, alignment. **Dữ liệu nhân tạo không chứng minh độ chính xác thực tế**.
4. **Fake observation ROS node**: phát track IDs, gesture, missed-frame, off-center, distances, timestamps và trạng thái voice giả; kiểm FSM và safety độc lập model.
5. **Mock Nav2 velocity / mock voice owner**: test guard không cần chạy robot/camera, gồm zero latch, cmd stale, cancellation race, goal UUID cũ.
6. **Gazebo sẵn có**: chạy riêng để test Nav2/mecanum/safety và xem robot dừng/quay/đi ngang qua các case injected perception; **không thêm camera vào Gazebo**.
7. **Camera Astra thật về sau**: test bắt buộc RGB-D topics, TF, latency, đo khoảng cách bằng vật chuẩn và nhiều người dưới ánh sáng/che khuất thực.

### 4.2. Hạn chế của từng loại test

- RGB-only video có thể đo giơ tay, tracking và lệch pixel; **không** chứng minh khoảng cách vật lý chính xác nếu không có depth thật.
- Depth synthetic chỉ kiểm thuật toán/ROS interfaces; không xác nhận calibration, noise, range, góc nhìn hay frame alignment thực tế.
- Chạy Gazebo với fake observations chỉ chứng minh pipeline điều khiển; không chứng minh perception ngoài đời.
- Nghiệm thu tổng cuối phải thực hiện trên robot thật/camera thật; trước đó ghi trạng thái **software complete / hardware validation pending**.

---

## 5. Workflow giai đoạn 0–11 (làm tuần tự; mỗi giai đoạn phải có Gate)

### PHASE 0 — Audit chỉ đọc: xác định hiện trạng thực tế

**Mục tiêu:** Có cơ sở bằng chứng để thiết kế, không đoán topic/TF/entry point.

**Công việc:**
- Xác nhận root/mount của `ws`, git root, branch, `git status`, cấu trúc `.agent` và package; check Docker Jazzy/Ubuntu mà không đọc/chỉnh ngoài `ws` bằng file operations.
- Dùng công cụ ROS/grep tìm **trong `ws`**: Astra package, launch, params, publisher ảnh RGB/depth/CameraInfo/point cloud, topic/service, frame ID, encoding, trường hợp depth registered, QoS, camera selection. Phân biệt static interface với live status (camera chưa kết nối).
- Audit package `person_tracking`: mã hiện có, dependency, test, model/checkpoint, tốc độ nếu có dữ liệu; xác định giữ/sửa/bỏ từng file với lý do. Không xóa trong phase này.
- Audit voice **read-only**: `nav_command_node.py`, cách gửi/cancel NavigateToPose, hiện có status/goal lifecycle gì. Chỉ đọc; không ghi.
- Audit Nav2/robot **read-only**: cmd_vel các khâu, smoother, collision monitor, base subscriber, message type, QoS, mecanum, TF camera -> base_link (nếu có), launch/remap.
- Kiểm tra đường chạy GPU qua các lệnh thông tin runtime nếu sẵn có; không tự cài thêm driver hoặc thư viện ngoài phạm vi.
- Báo cáo mâu thuẫn cấu trúc `~/ws/src/build|install` và colcon root; kiểm tra build outputs có thể cô lập.

**Deliverables trong `.agent/person_tracking/`:** `AUDIT.md`, `INVENTORY.md`, `RISKS.md` (có thể gộp), bản nháp `INTERFACES.md`, evidence command outputs ngắn kèm timestamps; updated `PROGRESS.md`.

**Gate 0:** Liệt kê được file path thật, phân loại “đã thấy trong code”/“đã chạy”/“chưa xác nhận”, kết luận hạn chế về camera và GPU. **Chưa viết lại code, chưa xóa file.** Báo cáo xin người dùng quyết định nếu cần thay đổi ranh giới file.

### PHASE 1 — Quy tắc, skill, thiết kế và kế hoạch kiểm thử

**Mục tiêu:** Tạo nền làm việc có thể tiếp tục qua nhiều phiên Antigravity.

**Công việc:**
- Tạo/đồng bộ `.agent/person_tracking/{PROJECT_SPEC,WORKFLOW,PROGRESS,INTERFACES,TEST_PLAN,DECISIONS}.md`, skill riêng tại `.agent/skills/ros2-person-tracking/SKILL.md`.
- Đọc quy tắc chung `.agent/rules/workspace.md` và `.agent/COMMANDS.md`, bổ sung **quy tắc riêng trong `.agent/person_tracking`**, không sửa file chung.
- Chọn kiến trúc từ kết quả audit, có sơ đồ dữ liệu, FSM, signal interface, QoS, latency, watchdog, điều kiện degrade và xử lý loss.
- Chọn model đa người đủ keypoint theo khả năng GPU thực tế. So sánh detector + tracker hiện có với pose model nhiều người; cân nhắc license, nguồn model, mức RAM/VRAM và latency. Chỉ giữ phần code thử hoạt động tốt; nếu định xóa thì báo diff trước.
- Tạo `TEST_PLAN.md` gồm case IDs, automation, gate; ghi cấu hình mẫu nhưng không hard-code topic chưa xác minh.

**Gate 1:** Có đặc tả có thể truy vết từng yêu cầu D01–D12; agent voice không bị sửa; giao diện dự kiến có owner & proposal; người dùng duyệt thiết kế trước code lớn.

### PHASE 2 — Source adapters, offline dataset và test infrastructure

**Mục tiêu:** Chạy được perception input độc lập với camera thật.

**Công việc:**
- Tạo adapter cấu hình cho live Astra, bag replay và video replay; không phụ thuộc topic name cố định.
- Chuẩn hóa `Image`, encoding, timestamp, CameraInfo, QoS; test sync RGB/depth, age và queue; robust với mất dữ liệu.
- Dựng fixtures gồm: 1 người, 3 người, cùng giơ tay, không ai giơ, người che người, đi gần mép, ID đổi, depth không hợp lệ, frame bị mất.
- Có ROS 2 launch chỉ chạy offline, không kích hoạt base/real Nav2 khi test perception.
- Ghi metadata dataset (nguồn, quyền sử dụng, độ phân giải, fps, phần nào RGB-only).

**Gate 2:** Có lệnh chạy lại được, pytest/ROS test pass; nhận dữ liệu ảo, ghi được topic/log/evidence; không sửa Gazebo.

### PHASE 3 — Multi-person perception, pose và tracking ID

**Mục tiêu:** Phát hiện nhiều người và ID nhất quán trong các cảnh kiểm thử.

**Công việc:**
- Chọn model dựa benchmark GPU khả dụng, không ràng buộc bắt buộc MoveNet SinglePose. SinglePose không đủ nếu chỉ suy luận một người/frame mà không có multi-person pipeline hợp lệ.
- Có confidence threshold, nonmax suppression nếu cần, track association, dropout logic, debug overlay, model version và FPS/latency thống kê.
- Chống trộn ID khi giao nhau/che khuất: nếu không chắc phải giảm confidence và tránh chuyển khóa sang người khác.
- Test nhiều người, người rời/đi vào ảnh, che khuất ngắn, fake ID switch.

**Gate 3:** Có biểu đồ/log/fixture chứng minh multi-person + ID theo thời gian; không dùng kết quả 1 frame để khẳng định identity.

### PHASE 4 — Giơ tay, chọn khách gần nhất và khóa phiên

**Mục tiêu:** IDLE -> LOCKED chỉ với cử chỉ hợp lệ.

**Công việc:**
- Cử chỉ một tay: cổ tay cao hơn vai cùng bên, keypoint confidence, đủ thời gian liên tục/ổn định, chống giơ tay thoáng qua; tùy chọn tỷ lệ normalized theo thân.
- Nếu nhiều người đồng thời đủ điều kiện, chọn **gần camera nhất**; ưu tiên depth hợp lệ và cùng frame. Nếu không có depth, có thể so proxy diện tích bbox với flag `estimated_nearest` nhưng khi gần nhau/che khuất/không chắc -> không khóa nhầm; báo ambiguity.
- Sau khóa, hạ tay không làm mất ID; khóa một lần cho session; robot không đi trước khi voice đưa đích hợp lệ.
- Test tránh khóa do nhầm khung xương, wrist occlusion, hai người cùng giơ, người gần hơn giơ muộn.

**Gate 4:** Log và overlay cho thấy chọn người đúng theo rule; lệnh start navigation khi `IDLE` bị chặn ở interface contract (mock voice).

### PHASE 5 — RGB-D, depth và hình học camera sau

**Mục tiêu:** Ước lượng khoảng cách và vị trí có confidence, biết khi nào dữ liệu không dùng được.

**Công việc:**
- Dựa source Astra đã audit: kiểm tra RGB/depth aligned hay không; nếu không, dùng CameraInfo, extrinsics, TF để project đúng hệ tọa độ; không lấy median pixel sai khung.
- Mẫu đo depth trên thân của đúng track ID, lọc bad values, median/trim, kiểm vùng valid, outlier, cự ly usable.
- Xác nhận optical frame vs `base_link`: camera nhìn sau làm thay đổi dấu góc. Test TF và thống nhất convention (+ lệch ảnh -> robot quay hướng nào).
- Sinh normalized pixel offset, nguy cơ gần mép, angular bearing (nếu calibration đủ); không dùng ước lượng metric giả trong RGB-only mode.
- Khi không có camera thật: synthetic unit tests, rosbag replay (nếu có); ghi `HARDWARE_VALIDATION_PENDING`.

**Gate 5:** Có test liên quan depth invalid/misaligned/out-of-range/TF invalid, dữ liệu báo valid/invalid và không âm thầm dùng số 0 như khoảng cách thật.

### PHASE 6 — Session FSM, timeout và giao tiếp voice

**Mục tiêu:** Vòng đời tracking deterministic, an toàn với event tới muộn.

**Trạng thái tối thiểu:** `IDLE`, `LOCKED_WAIT_DESTINATION`, `NAVIGATING`, `WAITING_FOR_PASSENGER`, `ABORTING`, `ARRIVED`, `FAULT` (có thể thêm trạng thái nhưng phải có transition table).

**Công việc:**
- State machine có session UUID/sequence, target ID, entry/exit actions, watchdog, accepted nav state và teardown idempotent.
- Kịch bản đến đích: voice thông báo `SUCCEEDED` của goal đúng session -> clear state, IDLE.
- Đánh mất vài frame -> grace; quá `visibility_stop` -> output zero chờ; quá `visibility_abort` -> yêu cầu voice hủy, latch STOP và clear khi đóng phiên an toàn.
- Voice cancel request phải idempotent, có ack hoặc timeout; nếu voice không ack vẫn giữ dừng. Tránh race acceptance muộn/goal cũ/response muộn làm bật lại xe.
- Nếu cần “WAIT nhưng không cancel” lâu hơn progress checker của Nav2, thống nhất chính sách voice pause/cancel/resubmit; **voice vẫn là goal owner**, tracking chỉ phát yêu cầu.
- Tạo **mock voice owner** trong `person_tracking` cho test khi agent voice chưa triển khai contract.

**Gate 6:** Unit/FSM và integration test pass cho every transition, duplicate/late events, stop/cancel/arrived, session isolation; không có trạng thái bị rơi vào cho phép lái ngoài ý muốn.

### PHASE 7 — Velocity guard, mecanum và watchdog

**Mục tiêu:** Robot dẫn theo đường Nav2 nhưng giới hạn vận tốc để không bỏ rơi/ra khỏi ảnh khách.

**Công việc:**
- Xác nhận kiểu cmd và vị trí guard theo Nav2 stack thật. Cơ chế điển hình: Nav2 -> smoother -> guard -> collision monitor -> base, **nhưng chỉ áp dụng sau khi audit**.
- Pass-through khi an toàn; distance-based speed limiting (không tăng tốc quá Nav2), yaw correction soft & saturated; giảm tốc khi camera edge, có hysteresis tránh dao động.
- Đặt lệnh zero khi chưa khóa, stale nav, stale tracking, stop/abort, goal complete; tránh ngắt nhịp bằng lệnh zero nếu trạng thái không rõ.
- Thử `linear.y=0` trước; bật mecanum lateral control riêng khi chứng minh collision/footprint coverage và tránh nguy hiểm quay ở cua.
- Quản lý tốc độ/acceleration, điều khiển quá độ, bounded yaw injection; trường hợp angular Nav2 đối nghịch safety phải có priority rõ.
- Test watchdog khi node crash, QoS mismatch, controller restart, disconnected topic.

**Gate 7:** Automated test mô phỏng mọi tín hiệu cho thấy zero trong deadline đặt ra, không ra vận tốc lớn hơn giới hạn, không tự tái khởi động khi nhận cmd cũ.

### PHASE 8 — End-to-end ROS 2 integration (không sửa Gazebo)

**Mục tiêu:** Nối tracking + mock voice / voice thật + Nav2 trên stack có sẵn.

**Công việc:**
- Tạo launch trong package tracking để tái sử dụng các launch hiện có qua include/remap nếu được; không sửa launch nguồn của các package khác.
- Dùng injection từ video/bag/fake perception trong khi Nav2/Gazebo tiếp tục như cũ; chứng minh cảnh: lock -> wait destination -> Nav2 start (mock hoặc voice real) -> slow/yaw/wait -> abort -> final zero.
- Xác nhận collision monitor nhận lệnh sau guard và vẫn kiểm ngang/động học; xác định lệnh đến base thực tế; nếu không thể nối mà không sửa file ngoài scope: báo `BLOCKED` và hướng dẫn chủ dự án giao thay đổi cho owner.
- Đo end-to-end latency, CPU/GPU utilization, stale timeout, log lifecycle; không chỉ dựa vào việc process chạy.

**Gate 8:** Demo được thực thi lại bằng command/launch rõ ràng và log, đạt tối thiểu 10–20 tình huống test quan trọng, bao gồm mất người và voice unavailable. Không sửa Gazebo.

### PHASE 9 — Giao tiếp song song với agent voice (đã được chủ dự án duyệt)

**Mục tiêu:** Có hợp đồng chính thức hai package mà không đụng chạm phạm vi của nhau.

**Công việc:**
- Chuyển cho agent voice bản `INTERFACES.md`: schema, sequence, QoS, session_id, pause/cancel acknowledgement, behavior khi voice nhận point đến lúc chưa LOCKED, SUCCEEDED, canceled.
- Agent voice tự cập nhật `nav_command_node.py` và interface trong phạm vi của họ. Tracking không tự sửa.
- Thực hiện contract tests từ bên tracking bằng mock; sau khi voice hoàn thành tiến hành smoke tests với voice.
- Điều kiện khởi hành: perception khóa đúng -> voice mới nhận/cho phép start. Không cho phép voice gửi goal mới lúc lỗi camera/target lost.

**Gate 9:** Cả hai agent có cùng một contract version; nav owner duy nhất là voice; test dừng/abort và late success thực sự pass. Nếu voice chưa sửa: ghi chờ tích hợp, không giả kết quả.

### PHASE 10 — Validation hiệu năng và độ bền (offline + mô phỏng chuyển động sẵn có)

**Mục tiêu:** Kiểm tải và tính an toàn, không chỉ demo happy path.

**Công việc:**
- GPU benchmark warm-up, FPS, p50/p95 latency, VRAM, queue depth, dropped frames, ROS timestamps.
- Stress test nhiều người, hand raise noise, camera freeze, TF thiếu, frame out-of-order, stale cmd, goal cancel delay, fast crossing, restart từng node, guard crash.
- Test 20+ case từ TEST_PLAN; kiểm regressions khi đổi model/config.
- Ghi rõ target acceptance performance theo số benchmark thật (không mặc định khẳng định 30 FPS nếu chưa đo).
- Kiểm safety priority: không có tracking -> không đi; không có voice ack -> không đi; restart session -> zero cho đến lock mới.

**Gate 10:** Test suite tái lập, log và lệnh chạy, known limitations, memory/GPU metrics và danh sách blocker phần cứng.

### PHASE 11 — Robot thật + Astra thật, nghiệm thu và bàn giao

**Chỉ làm khi có camera và robot thật.**

**Công việc:**
- Khởi chạy Astra driver theo source/launch đã audit; kiểm tra ROS stream bằng công cụ runtime, stamp/QoS/frame_id/encoding/registered depth/camera_info. Đo camera mount orientation, calibrate extrinsics/TF.
- Đo thật khoảng cách bằng mốc vật lý (có nhiều khoảng cách, người khác nhau, nhiều ánh sáng), kiểm sai số và giới hạn range.
- Test GPU inference chạy liên tục, nhiệt độ, dropouts, delay camera.
- Test tốc độ thấp tại khu vực có kiểm soát: lock, di chuyển thẳng, cua, người tụt xa, che khuất, mất người, dừng khẩn cấp, collision monitor, mecanum; có người giám sát và E-stop vật lý.
- Kiểm voice navigation owner + tracking safety phối hợp, không đẩy người vào vùng nguy hiểm, không cho robot đột ngột chạy lại sau abort.
- Ghi video/log/rosbag và bảng pass/fail, các tồn đọng; README hướng dẫn launch, setup và vận hành thực tế.

**Gate 11:** Chỉ báo **DONE/HARDWARE_VALIDATED** nếu đã đo nghiệm thu thật theo bộ test được duyệt. Khi không có camera thật thì trạng thái tối đa là **SOFTWARE_READY / HARDWARE_PENDING**.

---

## 6. Test matrix tối thiểu cần tự động hóa

| ID | Kiểm thử | Kết quả yêu cầu |
|---|---|---|
| T01 | Không có ai trong video | IDLE; lệnh zero; không nhận destination start. |
| T02 | Một người giơ tay < 1,5s | Không khóa. |
| T03 | Một người giơ tay >= 1,5s | Khóa đúng track, new session_id; hạ tay vẫn giữ. |
| T04 | Hai người cùng giơ, khác depth | Chọn người gần camera nhất với depth đáng tin. |
| T05 | Hai người giơ, không có depth, kích thước gần nhau | Không khẳng định nearest thật; không khóa tùy tiện nếu ambiguity. |
| T06 | Người khác cắt ngang người khóa | Không tự chuyển ID; giảm confidence/stop nếu không chắc. |
| T07 | Voice gửi đích trước khi có lock | Bị gate, không xuất lệnh đi. |
| T08 | Voice nhận đích sau lock | Goal được voice quản lý; tracking không gửi action trùng. |
| T09 | Người ổn định ở trung tâm | Cho Nav2 đi trong giới hạn đã thiết lập. |
| T10 | Người ở xa | Giảm tốc, có thể WAIT, không tăng tốc vượt Nav2. |
| T11 | Người tiến gần robot | Không chủ động lùi/đi nhanh để tạo khoảng cách. |
| T12 | Người lệch mép | Điều chỉnh hướng giới hạn; dấu đúng camera sau. |
| T13 | Bỏ sót 1–vài frame | Giữ ID/same session; không swap identity. |
| T14 | Mất ảnh lâu hơn stop timeout | Guard xuất zero ngay khi quá ngưỡng; chưa cần voice ack. |
| T15 | Mất ảnh lâu hơn abort timeout | Yêu cầu voice cancel, giữ zero; session không tự hồi sinh. |
| T16 | Voice cancel không phản hồi | Vẫn zero, có timeout/error, không resume. |
| T17 | Voice báo SUCCEEDED đúng goal/session | Clear lock/destination -> IDLE/zero. |
| T18 | SUCCEEDED/ACK muộn của goal cũ | Bỏ qua theo UUID/session/sequence. |
| T19 | Depth zero/NaN/out-of-range | Đánh dấu invalid, không dùng quyết định metric giả. |
| T20 | CameraInfo/TF mismatch | Không điều khiển dựa góc hoặc khoảng cách sai; fail-safe. |
| T21 | Nav2 cmd_vel stale | Output zero; không giữ lệnh cũ. |
| T22 | Guard restart hoặc crash | Không có chuyển động duy trì nguy hiểm; xác minh base watchdog. |
| T23 | Collision Monitor ở sau guard | Lệnh cuối (x/y/yaw) đều qua collision monitor. |
| T24 | Thử nghiệm Gazebo hiện có | Không sửa file Gazebo, chứng minh dừng/quay bằng injected observations. |
| T25 | ROS2 unit/launch tests lặp nhiều vòng | Kết quả ổn định, không test treo, có timeout và log. |

Đây là baseline; test mới bổ sung theo mỗi bug để tránh tái phát.

---

## 7. Quy tắc thực thi cho mỗi giai đoạn

**Mỗi lần người dùng nhắn “làm tiếp phase N”:**

1. Đọc lại `PROJECT_SPEC.md`, `WORKFLOW.md`, `PROGRESS.md`, `INTERFACES.md`, `DECISIONS.md`, skill và rules (tất cả trong `ws`).
2. Kiểm `git status --short`, xác định thay đổi ngoài phạm vi; không đụng file của agent voice.
3. Liệt kê đầu vào đã kiểm chứng, phần chưa rõ, test gate; chỉ triển khai **phase được yêu cầu**.
4. Chỉ sửa allowlist, không thay đổi chung; khi cần thay đổi bên ngoài phải báo rõ file + lý do và dừng phần phụ thuộc.
5. Chạy test ở phạm vi package, timeout rõ, log trong `.agent/person_tracking/evidence/<phase>/`; không đánh tráo mock thành hardware test.
6. Báo cáo `PASS/FAIL/BLOCKED/PARTIAL`, evidence path, changed files, lệnh tái hiện, performance, rủi ro, vấn đề cần chủ dự án hoặc agent voice xử lý.
7. Cập nhật `PROGRESS.md` với ngày giờ, commit hash (nếu có), status, test summary, next phase. **Không tự chuyển phase** nếu chưa nhận yêu cầu người dùng.

**Quy tắc xóa/sửa code thử nghiệm:** Chỉ trong `person_tracking`, sau audit có bảng giữ/bỏ, commit hoặc diff có thể khôi phục và không xóa dữ liệu người khác. Tránh xóa mã đang dùng chỉ vì muốn viết mới.

**Quy tắc báo cáo:** Không viết “đã test camera Astra thật” chỉ vì đã đọc package hoặc chạy fake topics. Không viết “Nav2 đã điều khiển an toàn” chỉ vì unit test pass. Mỗi nhận định phải phân biệt `source-static`, `ROS-runtime`, `offline-test`, `Gazebo`, `real-hardware`.

---

## 8. Mẫu PROGRESS và điều kiện hoàn tất dự án

Mỗi dòng phase trong `PROGRESS.md` dùng cấu trúc:

```md
## Phase N — <tên> — [NOT_STARTED | IN_PROGRESS | PASS | PARTIAL | BLOCKED]
- Ngày/giờ:
- Phạm vi file đã sửa:
- Bằng chứng: `.agent/person_tracking/evidence/phase_N/...`
- Kiểm thử: lệnh, số pass/failed/skipped, timeout, phiên ROS và GPU
- Không kiểm được: ...
- Vấn đề/rủi ro: ...
- Giao diện cần agent voice thực hiện: ...
- Chốt: ...
- Phase tiếp theo, chỉ làm sau lệnh của chủ dự án: ...
```

**Định nghĩa Done theo cấp:**

- **S0 — AUDITED:** Source package và giao diện được khảo sát có chứng cứ.
- **S1 — OFFLINE_PERCEPTION_READY:** Đạt replay/fake tests; chưa chứng minh camera thật.
- **S2 — SOFTWARE_INTEGRATED:** Chạy luồng tracking + mock/voice + Nav2 trên hệ sẵn có; vẫn không cam thật.
- **S3 — HARDWARE_VALIDATED:** Camera thật + robot thật hoàn thành gate phase 11.

Không dùng `DONE` chung nếu mới đạt S1/S2; cập nhật blockers trung thực.

---

## 9. Các câu lệnh khảo sát được phép dùng làm gợi ý (không phải bằng chứng đã chạy)

**Chỉ thực hiện trong `ws` và xác minh đường dẫn trước.**

```bash
cd ~/ws/src  # ví dụ: audit cần xác nhận đây có đúng là root build/colcon không
pwd
readlink -f .
git rev-parse --show-toplevel
git status --short
find . -maxdepth 3 -name package.xml -print
find . -maxdepth 5 \( -iname '*astra*' -o -iname '*person*track*' -o -iname '*nav*' \) -print
rg -n 'CameraInfo|image_raw|depth|pointcloud|cmd_vel|NavigateToPose' \
  --glob '!build/**' --glob '!install/**' --glob '!log/**' .
# Khi đã xác nhận đúng ROS/Docker env:
ros2 pkg list | grep -E 'astra|person|nav2'
ros2 topic list -t
ros2 action list -t
ros2 node list
ros2 topic info -v /cmd_vel  # CHỈ NẾU topic này thực sự tồn tại
# GPU: truy vấn runtime nếu công cụ đã có, không cài driver ngoài ws
nvidia-smi
```

Không chạy hàng loạt lệnh ROS khi daemon/stack không hoạt động rồi kết luận driver không có. Nếu camera chưa cắm, chỉ báo `static-driver-audit`; topic thực sự chỉ đo lại khi cắm thiết bị.

---

## 10. Tiêu chí để agent bắt đầu triển khai

- [ ] Xác nhận chỉ thao tác trong `ws` (host) và đúng bind-mount workspace (Docker).
- [ ] Đọc rules/skill/voice hiện có **chỉ đọc**.
- [ ] Chụp trạng thái Git trước khi thay đổi.
- [ ] Audit Astra, tracking, Nav2, GPU, workspace layout; chưa đụng Gazebo.
- [ ] Tạo tài liệu quản lý trong `.agent/person_tracking/` và skill tracking riêng.
- [ ] Đề xuất interface và milestone theo phase, không giao tracking quyền Nav2 action.
- [ ] Báo cáo phase 0/1, không tự nhảy sang code inference khi chưa nhận lệnh.

**Nguồn tham chiếu nội bộ ưu tiên:** Source code checkout `ws`, trạng thái ROS 2 Jazzy thực tế, những quyết định D01–D12 ở đầu văn bản. Mọi thông tin tham khảo upstream chỉ dùng để gợi ý; không thay thế bằng chứng từ chính workspace.
