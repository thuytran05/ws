# ROS 2 Jazzy – Person tracking via Astra S RGB

## Install dependencies inside Docker

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y python3-venv ros-jazzy-cv-bridge ros-jazzy-std-srvs
python3 -m venv --system-site-packages /home/ws/.venv_yolo
touch /home/ws/.venv_yolo/COLCON_IGNORE
source /home/ws/.venv_yolo/bin/activate
python -m pip install -U pip
python -m pip install 'numpy==1.26.4' 'opencv-python==4.11.0.86' ultralytics
python -c 'import torch, cv_bridge, ultralytics; print("CUDA:", torch.cuda.is_available())'
```

The pip install may download large Torch/CUDA dependencies. Keep NumPy under 2 for ROS Jazzy cv_bridge compatibility.

## Install ROS package

Copy the bundled `person_tracking` directory into `/home/ws/src/` (next to linorobot2) and run:

```bash
cd /home/ws
colcon build --packages-select person_tracking --symlink-install
source /home/ws/install/setup.bash
```

## Run

Terminal A (Astra camera):
```bash
source /home/ws/install/setup.bash
ros2 launch astra_camera astra.launch.xml enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false
```

Terminal B (YOLO):
```bash
source /home/ws/install/setup.bash
source /home/ws/.venv_yolo/bin/activate
python /home/ws/src/person_tracking/person_tracking/yolo_pose_node.py
```

This explicit `python` method ensures the venv is used even if a `ros2 run` console script was built with the system Python executable.

Terminal C (view):
```bash
source /home/ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
Select `/person_tracking/image_annotated` or use RViz `Image` display.

## ROS API

- Subscribes `/camera/color/image_raw` with sensor data QoS
- Publishes `/person_tracking/image_annotated` (`sensor_msgs/msg/Image`, sensor data QoS)
- Publishes `/person_tracking/selected_id` (`std_msgs/msg/Int32`, emitted when selected and reset)
- Publishes `/person_tracking/target_pixel` (`std_msgs/msg/Int32MultiArray`, `[id, u, v]`, only when visible)
- Publishes `/person_tracking/target_visible` (`std_msgs/msg/Bool`, per processed frame)
- Service `/person_tracking/reset` (`std_srvs/srv/Trigger`)

Reset selection:
```bash
ros2 service call /person_tracking/reset std_srvs/srv/Trigger '{}'
```

Modify YOLO model, device and gesture threshold:
```bash
python /home/ws/src/person_tracking/person_tracking/yolo_pose_node.py --ros-args -p model:=yolo11s-pose.pt -p device:=cpu -p gesture_seconds:=2.0
```

## Limitations

- This is Stage 1 only. RGB pose tracking cannot provide actual 3D person location. You must register Depth to RGB and use camera intrinsics/TF to publish a metric pose in `base_link`.
- Track IDs can change or swap in crowds. A robust robot controller needs target re-identification, stale-data timeout, safety stop, and collision avoidance before motion.
- End-to-end input may be ~7–25 FPS according to the observed system. Gesture time is based on wall time and a max gap check, not a fixed count of 60 frames.
- This package does not publish `/cmd_vel` and will not command robot motion.
