#!/bin/bash
set -e

export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
source /home/ws/install_tracking/setup.bash

LOG_DIR="/home/ws/src/.agent/person_tracking/evidence/phase_8"
mkdir -p "$LOG_DIR"
RUNTIME_LOG="$LOG_DIR/gazebo_nav2_runtime.log"

echo "=== Starting Gazebo + Nav2 Integration Launch on ROS_DOMAIN_ID=43 ==="
ros2 launch person_tracking gazebo_escort_integration.launch.py gui:=false > "$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

cleanup() {
    echo "=== Shutting down launch process and clean up ==="
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "nav2" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "velocity_guard" 2>/dev/null || true
    pkill -9 -f "tracking_manager" 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

echo "Waiting for Nav2 & Gazebo nodes to come online (up to 30s)..."
for i in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -q "collision_monitor" && ros2 node list 2>/dev/null | grep -q "escort_velocity_guard"; then
        echo "Nodes detected online at i=$i! Waiting 5s for lifecycle activation..."
        sleep 5
        break
    fi
    sleep 1
done

echo "Current active ROS 2 nodes:"
ros2 node list || true

echo "=== Executing Gate 8 Verification Suite ==="
python3 /home/ws/src/person_tracking/test/run_gate8_real_gazebo_nav2.py

echo "=== Gate 8 Verification Script Finished Successfully ==="
