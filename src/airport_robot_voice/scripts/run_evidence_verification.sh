#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID=10
export LINOROBOT2_BASE=mecanum
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

echo "========================================================================"
echo "KHỞI ĐỘNG THU THẬP BẰNG CHỨNG GAZEBO + NAV2 + AIRPORT ROBOT VOICE"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID | BASE=$LINOROBOT2_BASE"
echo "========================================================================"

CLEANUP_DONE=0
cleanup() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1
    echo "Dọn dẹp các tiến trình thử nghiệm..."
    pkill -f "linorobot2_gazebo|linorobot2_navigation|voice_system|nav_command_node|intent_node|tts_node|ros_gz_sim|ros_gz_bridge|gz sim|nav2_|ekf_node|robot_state_publisher" || true
    sleep 2
    pkill -9 -f "gz sim|ros_gz" || true
    echo "Đã dọn dẹp sạch sẽ toàn bộ tiến trình."
}
trap cleanup EXIT INT TERM

cleanup
CLEANUP_DONE=0

# 1. Khởi động Gazebo Sim headless
echo "[1/4] Khởi động Gazebo Sim (headless)..."
ros2 launch linorobot2_gazebo gazebo.launch.py \
    world_name:=turtlebot3_world \
    gui:=false \
    spawn_x:=0.5 spawn_y:=0.0 spawn_yaw:=0.0 > /tmp/gazebo_sim.log 2>&1 &
GZ_PID=$!

echo "Chờ Gazebo và Bridge sẵn sàng..."
READY=0
for i in $(seq 1 25); do
    if ros2 topic list 2>/dev/null | grep -q "/clock"; then
        READY=1
        echo "Gazebo /clock đã sẵn sàng sau ${i}s."
        break
    fi
    sleep 1
done

if [ "$READY" -ne 1 ]; then
    echo "LỖI: Gazebo không khởi động được trong 25s!"
    cat /tmp/gazebo_sim.log | tail -n 30
    exit 1
fi

# 2. Khởi động Nav2 Stack
echo "[2/4] Khởi động Nav2 Navigation Stack..."
ros2 launch linorobot2_navigation navigation.launch.py \
    sim:=true \
    rviz:=false \
    map:=/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml \
    initial_pose_x:=0.5 initial_pose_y:=0.0 initial_pose_yaw:=0.0 > /tmp/nav2.log 2>&1 &
NAV2_PID=$!

echo "Khởi tạo Initial Pose và kích hoạt AMCL..."
INITIAL_OK=0
for i in $(seq 1 25); do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" >/dev/null 2>&1 || true
    
    if timeout 2 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        INITIAL_OK=1
        echo "AMCL đã nhận Initial Pose và broadcast transform map->odom thành công sau ${i}s."
        break
    fi
    sleep 0.5
done

echo "Kích hoạt Lifecycle Manager Navigation (STARTUP = 0)..."
NAV_ACTIVE=0
for i in $(seq 1 30); do
    ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1 || true
    
    BT_STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)
    PLANNER_STATE=$(ros2 lifecycle get /planner_server 2>/dev/null || true)
    
    if echo "$BT_STATE" | grep -q "active" && echo "$PLANNER_STATE" | grep -q "active"; then
        NAV_ACTIVE=1
        echo "Nav2 bt_navigator và planner_server đã chuyển sang trạng thái ACTIVE sau ${i}s."
        break
    fi
    sleep 1
done

if [ "$NAV_ACTIVE" -ne 1 ]; then
    echo "CẢNH BÁO: bt_navigator chưa đạt active sau 30s! Kiểm tra log:"
    cat /tmp/nav2.log | grep -E "ERROR|WARN" | tail -n 20 || true
fi

sleep 2

# 3. Khởi động Airport Robot Voice System
echo "[3/4] Khởi động Airport Robot Voice System..."
ros2 launch airport_robot_voice voice_system.launch.py \
    launch_stt:=false \
    tts_engine:=log \
    tts_policy:=flexible \
    use_sim_time:=true \
    places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json > /tmp/voice.log 2>&1 &
VOICE_PID=$!

echo "Chờ Voice System sẵn sàng..."
VOICE_READY=0
for i in $(seq 1 15); do
    if ros2 topic list 2>/dev/null | grep -q "/voice/nav_state"; then
        VOICE_READY=1
        echo "Voice System sẵn sàng sau ${i}s."
        break
    fi
    sleep 1
done

if [ "$VOICE_READY" -ne 1 ]; then
    echo "LỖI: Voice System không sẵn sàng trong 15s!"
    cat /tmp/voice.log | tail -n 30
    exit 1
fi

sleep 2

# 4. Chạy kịch bản thu thập bằng chứng
echo "[4/4] Bắt đầu chạy kịch bản thu thập bằng chứng tự động..."
python3 /home/ws/src/airport_robot_voice/scripts/verify_gazebo_evidence.py

echo "THU THẬP BẰNG CHỨNG HOÀN TẤT THÀNH CÔNG!"
