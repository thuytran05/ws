#!/bin/bash
set -e

export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash
if [ -f /home/ws/install/setup.bash ]; then
    source /home/ws/install/setup.bash
fi
if [ -f /home/ws/install_tracking/setup.bash ]; then
    source /home/ws/install_tracking/setup.bash
fi

LOG_DIR="/home/ws/src/.agent/person_tracking/evidence/phase_8"
mkdir -p "$LOG_DIR"
RUNTIME_LOG="$LOG_DIR/gazebo_nav2_runtime.log"

echo "=== Cleaning up any lingering processes on ROS_DOMAIN_ID=43 ==="
pkill -f "[e]scort_velocity_guard" 2>/dev/null || true
pkill -f "[e]scort_velocity_watchdog" 2>/dev/null || true
pkill -f "[e]scort_tracking_manager" 2>/dev/null || true
pkill -f "gate8_real_gazebo_test_driver" 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f nav2 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 1

echo "=== Starting Gazebo + Nav2 Integration Launch on ROS_DOMAIN_ID=43 ==="
set -m
ros2 launch person_tracking gazebo_escort_integration.launch.py gui:=false enable_test_scan:=true > "$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
echo "Launch PID: $LAUNCH_PID, PGID: $LAUNCH_PGID"
export LAUNCH_PID LAUNCH_PGID

cleanup() {
    echo "=== Shutting down launch process group -$LAUNCH_PGID ==="
    if [ -n "$LAUNCH_PGID" ]; then
        kill -TERM -$LAUNCH_PGID 2>/dev/null || true
        sleep 4
        kill -KILL -$LAUNCH_PGID 2>/dev/null || true
    fi
    pkill -f "[e]scort_velocity_guard" 2>/dev/null || true
    pkill -f "[e]scort_velocity_watchdog" 2>/dev/null || true
    pkill -f "[e]scort_tracking_manager" 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

echo "Waiting for Nav2 & Gazebo nodes to come online (up to 50s)..."
NODES_READY=false
for i in $(seq 1 50); do
    if ros2 node list 2>/dev/null | grep -q "escort_velocity_guard" && ros2 node list 2>/dev/null | grep -q "escort_velocity_watchdog" && ros2 node list 2>/dev/null | grep -q "ekf_filter_node"; then
        echo "Nodes detected online at i=$i! Waiting 5s for lifecycle activation..."
        sleep 5
        NODES_READY=true
        break
    fi
    sleep 1
done

if [ "$NODES_READY" != "true" ]; then
    echo "WARNING: Timeout waiting for collision_monitor and escort_velocity_guard nodes!"
fi

echo "Current active ROS 2 nodes:"
ros2 node list || true

echo "=== Executing Gate 8 Real Gazebo Suite ==="
TEST_EXIT_CODE=0
python3 /home/ws/src/person_tracking/test/run_gate8_real_gazebo_nav2.py || TEST_EXIT_CODE=$?

echo "Test driver finished with exit code: $TEST_EXIT_CODE"
if [ $TEST_EXIT_CODE -ne 0 ]; then
    echo "=== Gate 8 Verification Suite FAILED (exit code $TEST_EXIT_CODE) ==="
    exit $TEST_EXIT_CODE
fi

echo "=== Gate 8 Verification Script Finished Successfully ==="
exit 0
