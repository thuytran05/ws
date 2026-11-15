# person_tracking

ROS 2 Jazzy package for person selection using YOLO Pose, position measurement
using registered RGB/depth, and low-speed following using a rear-facing camera.

## Tên package, executable và node

| Thành phần | Tên |
|---|---|
| Package ament_python | `person_tracking` |
| Executable nhận diện và đo vị trí | `person_tracker` |
| Node nhận diện và đo vị trí | `/person_tracking/person_tracker` |
| Executable điều khiển | `person_follower` |
| Node điều khiển | `/person_tracking/person_follower` |
| Launch chỉ nhận diện và đo vị trí | `tracking.launch.py` |
| Launch nhận diện và điều khiển | `following.launch.py` |
| Cấu hình | `config/person_tracking.yaml` |

Hai launch dùng chung node tracker. Chỉ chạy một trong hai launch tại một thời
điểm. Camera được khởi động riêng bằng driver Astra hiện có.

## Giao tiếp ROS

| Topic/service | Kiểu | Nội dung |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Ảnh màu đầu vào |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth đã registration sang RGB |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Nội tại camera màu |
| `/person_tracking/image_annotated` | `sensor_msgs/msg/Image` | Ảnh kết quả |
| `/person_tracking/selected_id` | `std_msgs/msg/Int32` | ID khi chọn; -1 khi reset |
| `/person_tracking/target_pixel` | `std_msgs/msg/Int32MultiArray` | [ID, u, v], pixel tâm bounding box |
| `/person_tracking/target_position` | `geometry_msgs/msg/PointStamped` | Vị trí ước lượng trong frame quang học camera, mét |
| `/person_tracking/target_visible` | `std_msgs/msg/Bool` | Có mục tiêu với depth hợp lệ để điều khiển |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Vận tốc xe; follower xuất ở 20 Hz |
| `/person_tracking/reset` | `std_srvs/srv/Trigger` | Xóa chọn người và bỏ khóa lỗi |

Tracker chỉ publish dữ liệu nhận diện; follower subscribe vị trí và ID rồi
publish vận tốc. `selected_id` phát khi chọn/reset, không phải heartbeat.
`target_pixel` không có header và chỉ để xem/debug. Controller dùng
`target_position` có timestamp ảnh để phát hiện dữ liệu cũ.
NaN trong `target_position` nghĩa là không có vị trí hợp lệ, không được dùng
như một tọa độ. Đây là giao ước của hai node trong package này.

## Cài vào workspace đang có

Không cần tạo package khác. Thư mục gốc trong file ZIP là `person_tracking`;
đặt vào `/home/ws/src` để có `/home/ws/src/person_tracking/package.xml`.

Trước khi chép đè, sao lưu package cũ ra ngoài thư mục src. Ví dụ trong Docker:

```bash
mkdir -p /home/ws/backups
cp -a /home/ws/src/person_tracking /home/ws/backups/person_tracking_before_update_$(date +%Y%m%d_%H%M%S)
```

Sau đó chép các tệp mới vào `/home/ws/src/person_tracking`. File
`person_tracking/yolo_pose_node.py` cũ có thể giữ lại để đối chiếu, nhưng không
chạy nó đồng thời với tracker mới. Không đặt bản sao package cũ trong src,
tránh trùng tên package khi colcon tìm kiếm.

Build trong terminal đã có ROS:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ws
colcon build --symlink-install --packages-select person_tracking
source /home/ws/install/setup.bash
ros2 pkg executables person_tracking
```

Kết quả phải có hai executable mới:

```text
person_tracking person_tracker
person_tracking person_follower
```

Đây là package ament_python có resource marker, package.xml, setup.cfg,
console_scripts, launch và YAML. Không cần chạy Python bằng đường dẫn tệp.
Email maintainer trong metadata là placeholder `maintainer@example.com`;
thay bằng email của bạn nếu xuất bản package.

## Python và thư viện

Giữ venv hiện tại `/home/ws/.venv_yolo`. Launch argument `python_executable`
chọn rõ interpreter chạy tracker, nên không phụ thuộc vào việc `ros2` hay
`colcon` đang dùng shebang Python nào. Follower chỉ cần thư viện ROS hệ thống.

Kiểm tra venv:

```bash
/home/ws/.venv_yolo/bin/python -c "import rclpy, cv_bridge, message_filters, torch, ultralytics; print('OK')"
```

Nếu thiếu message_filters, cài gói ROS trong Docker:

```bash
sudo apt install ros-jazzy-message-filters
```

Không tạo lại venv hoặc nâng cấp numpy/torch khi chưa có lỗi. Nếu gói ROS đã
cài mà venv không import được, kiểm tra pyvenv.cfg và khả năng dùng system
site packages. Torch và Ultralytics dùng bản hiện có trong venv; package này
không tự cài hoặc thay đổi các phiên bản đó.

## Khởi động

Camera:

```bash
ros2 launch astra_camera astra.launch.xml \
  enable_point_cloud:=false enable_ir:=false depth_registration:=true
```

Chỉ nhận diện và đo vị trí:

```bash
ros2 launch person_tracking tracking.launch.py \
  python_executable:=/home/ws/.venv_yolo/bin/python
```

Mở `/person_tracking/image_annotated` trong RViz, chọn Best Effort nếu cần.
Giơ tay trên đầu khoảng 2 giây để chọn người. Ô xanh lam là vùng lấy depth;
khung xanh lá là người được chọn; Z là khoảng cách theo trục nhìn camera.
Đối chiếu bằng thước ở 1,0 / 1,5 / 2,0 m khi người đứng gần giữa ảnh.

Để chạy cả tracker và follower, dừng launch tracking rồi chạy:

```bash
ros2 launch person_tracking following.launch.py \
  python_executable:=/home/ws/.venv_yolo/bin/python
```

Launch này xuất trực tiếp vào `/cmd_vel`, không có topic thay tên.
Khi ESP32 được kết nối và subscribe đúng kiểu Twist, xe có thể chạy ngay nếu
mục tiêu đã được chọn. Nếu chưa có subscriber, có thể xem lệnh bằng:

```bash
ros2 topic echo /cmd_vel
```

Reset và giơ tay chọn lại:

```bash
ros2 service call /person_tracking/reset std_srvs/srv/Trigger '{}'
```

Mất ID đã chọn dù chỉ một khung, depth không đủ tin cậy hoặc ảnh quá cũ:
dừng/khóa chờ reset. ID cũ được giữ cho tới reset. Nếu controller mất dữ liệu
quá 0,40 s, nó cũng khóa chờ cùng thao tác reset trên tracker.

## Điều khiển và hình học

- Camera cố định, gần ngang, hướng thẳng ra sau xe, ảnh không lật gương.
- Xe theo hệ ROS: linear.x dương là tiến, angular.z dương là quay trái.
- Bản hiện tại dùng tiến/lùi và xoay; linear.y bằng 0. Chưa thêm đi ngang.
- Khoảng đặt 1,5 m là depth Z từ camera tới vùng thân, không phải khoảng cách
  từ mép xe hoặc khoảng cách Euclid. Khi người ở giữa ảnh và camera ngang,
  nó gần với khoảng cách theo phương ngang. Cần TF/bù góc nếu lắp camera nghiêng.
- Point x/y được chiếu trên tia qua tâm bounding box bằng Z đo ở vùng thân;
  đây là ước lượng phục vụ điều khiển, không phải tâm 3D chính xác cơ thể.
- Median vùng thân loại pixel zero/NaN và dữ liệu không hợp lệ, nhưng không
  loại được mọi trường hợp vật/người khác che vùng thân.

| Tình huống khi người gần giữa ảnh | Lệnh |
|---|---|
| Xa hơn 1,6 m | linear.x âm: xe lùi về người phía sau |
| Gần hơn 1,4 m | linear.x dương: xe tiến ra xa người |
| 1,4–1,6 m | Giảm tốc về 0 |
| Người ở bên phải ảnh | angular.z âm |
| Người ở bên trái ảnh | angular.z dương |
| Mất người hoặc depth lỗi | Dừng ngay, khóa chờ reset |

Giới hạn ban đầu: 0,10 m/s, 0,20 rad/s; gia tốc 0,10 m/s², 0,30 rad/s².
Vùng không chỉnh góc khoảng +/-2,86 độ. P khoảng cách là 0,20 s^-1.
Mức chỉnh xoay tăng phi tuyến theo độ lệch góc; tiến/lùi giảm khi người lệch
nhiều để ưu tiên xoay. Sửa thông số trong `config/person_tracking.yaml` rồi
khởi động lại launch. Đọc tham số thực tế đang dùng bằng `ros2 param get`.
Nếu không dùng symlink-install, cần build lại sau khi sửa YAML ở source.

## Giới hạn và mức kiểm chứng

Chưa có tránh vật cản hoặc ReID. ByteTrack vẫn có khả năng gán nhầm cùng ID
khi nhiều người che nhau. Chỉ thử trong vùng trống, tốc độ thấp.
Firmware cần watchdog riêng để dừng khi controller hoặc Docker bị tắt đột
ngột; Python không đảm bảo gửi lệnh dừng khi bị kill.

Capture age được kiểm tra sau inference; controller chạy riêng để YOLO chậm
không chặn timer dừng. 30 Hz ảnh camera chưa chắc là 30 FPS YOLO. Xem dòng
age trên ảnh và đo `/person_tracking/image_annotated` nếu nghi ngờ trễ.
Gesture chấp nhận gián đoạn tối đa 0,75 s; không phải 2 giây tuyệt đối liên tục.

Đã kiểm tra cú pháp, metadata package, entrypoint, launch/YAML và giao tiếp
giữa hai node; kiểm tra toán điều khiển và mô phỏng dừng/reset từ bản trước.
Chưa build/chạy với ROS, Astra hoặc xe thực ở môi trường tạo bộ tệp này.
