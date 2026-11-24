#!/bin/bash
set -e

export ROS_DOMAIN_ID=44
source /opt/ros/jazzy/setup.bash
if [ -f /home/ws/install/setup.bash ]; then
    source /home/ws/install/setup.bash
fi
if [ -f /home/ws/install_tracking/setup.bash ]; then
    source /home/ws/install_tracking/setup.bash
fi

LOG_DIR="/home/ws/src/.agent/airport_robot_voice/evidence"
mkdir -p "$LOG_DIR"
RUNTIME_LOG="$LOG_DIR/phase9_gazebo_runtime.log"
VOICE_LOG="$LOG_DIR/phase9_voice_node.log"

echo "=== Cleaning up any lingering processes on ROS_DOMAIN_ID=44 ==="
pkill -f "[e]scort_velocity_guard" 2>/dev/null || true
pkill -f "[e]scort_velocity_watchdog" 2>/dev/null || true
pkill -f "[e]scort_tracking_manager" 2>/dev/null || true
pkill -f "nav_command_node" 2>/dev/null || true
pkill -f "verify_phase9_gazebo_escort" 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f nav2 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 1

echo "=== Starting Gazebo + Nav2 + Escort Launch on ROS_DOMAIN_ID=44 ==="
set -m
ros2 launch person_tracking gazebo_escort_integration.launch.py gui:=false enable_test_scan:=true enable_watchdog:=true > "$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d " ")
echo "Launch PID: $LAUNCH_PID, PGID: $LAUNCH_PGID"
export LAUNCH_PID LAUNCH_PGID

cleanup() {
    echo "=== Shutting down processes... ==="
    if [ -n "$VOICE_PID" ]; then
        kill -TERM $VOICE_PID 2>/dev/null || true
    fi
    if [ -n "$LAUNCH_PGID" ]; then
        kill -TERM -$LAUNCH_PGID 2>/dev/null || true
        sleep 4
        kill -KILL -$LAUNCH_PGID 2>/dev/null || true
    fi
    pkill -f "[e]scort_velocity_guard" 2>/dev/null || true
    pkill -f "[e]scort_velocity_watchdog" 2>/dev/null || true
    pkill -f "[e]scort_tracking_manager" 2>/dev/null || true
    pkill -f "nav_command_node" 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

echo "Waiting for Gazebo, Nav2 & Escort nodes to come online (up to 60s)..."
NODES_READY=false
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "escort_velocity_guard" && \
       ros2 node list 2>/dev/null | grep -q "escort_velocity_watchdog" && \
       ros2 node list 2>/dev/null | grep -q "ekf_filter_node" && \
       ros2 node list 2>/dev/null | grep -q "bt_navigator"; then
        echo "Nodes detected online at i=$i! Waiting 5s for lifecycle activation..."
        sleep 5
        NODES_READY=true
        break
    fi
    sleep 1
done

if [ "$NODES_READY" != "true" ]; then
    echo "WARNING: Timeout waiting for Gazebo / Nav2 nodes!"
fi

echo "Current active ROS 2 nodes before voice:"
ros2 node list || true

echo "=== Starting Real Voice Node with enable_escort:=true ==="
ros2 run airport_robot_voice nav_command_node --ros-args \
    -p enable_escort:=true \
    -p use_sim_time:=true \
    -p places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json \
    > "$VOICE_LOG" 2>&1 &
VOICE_PID=$!
echo "Voice Node PID: $VOICE_PID"
sleep 3

echo "=== Running Phase 9 Verification Suite ==="
TEST_EXIT_CODE=0
python3 /home/ws/src/airport_robot_voice/scripts/verify_phase9_gazebo_escort.py || TEST_EXIT_CODE=$?

echo "Test driver finished with exit code: $TEST_EXIT_CODE"
if [ $TEST_EXIT_CODE -ne 0 ]; then
    echo "=== Phase 9 Verification FAILED (exit code $TEST_EXIT_CODE) ==="
    exit $TEST_EXIT_CODE
fi

echo "=== Phase 9 Verification Finished Successfully ==="
exit 0
