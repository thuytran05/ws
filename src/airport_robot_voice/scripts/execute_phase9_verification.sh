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

LAUNCH_PID=""
LAUNCH_PGID=""
VOICE_PID=""
VOICE_PGID=""
TEST_PID=""

cleanup() {
    echo "=== Shutting down processes tracked by this run... ==="
    # 1. Stop test driver if still running
    if [ -n "$TEST_PID" ] && kill -0 "$TEST_PID" 2>/dev/null; then
        kill -TERM "$TEST_PID" 2>/dev/null || true
        wait "$TEST_PID" 2>/dev/null || true
    fi

    # 2. Stop voice node
    if [ -n "$VOICE_PGID" ] && kill -0 -"$VOICE_PGID" 2>/dev/null; then
        kill -TERM -"$VOICE_PGID" 2>/dev/null || true
    elif [ -n "$VOICE_PID" ] && kill -0 "$VOICE_PID" 2>/dev/null; then
        kill -TERM "$VOICE_PID" 2>/dev/null || true
    fi

    # 3. Stop launch process group
    if [ -n "$LAUNCH_PGID" ]; then
        kill -TERM -"$LAUNCH_PGID" 2>/dev/null || true
        for i in $(seq 1 5); do
            if kill -0 -"$LAUNCH_PGID" 2>/dev/null; then
                sleep 1
            else
                break
            fi
        done
        kill -KILL -"$LAUNCH_PGID" 2>/dev/null || true
    fi

    wait "$VOICE_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

echo "=== Starting Gazebo + Nav2 + Escort Launch on ROS_DOMAIN_ID=44 ==="
set -m
ros2 launch person_tracking gazebo_escort_integration.launch.py gui:=false enable_test_scan:=true enable_watchdog:=true > "$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d " ")
echo "Launch PID: $LAUNCH_PID, PGID: $LAUNCH_PGID"
export LAUNCH_PID LAUNCH_PGID

echo "Waiting for Gazebo, Nav2 & Escort nodes to come online (up to 80s)..."
NODES_READY=false
for i in $(seq 1 80); do
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
VOICE_PGID=$(ps -o pgid= -p $VOICE_PID | tr -d " ")
echo "Voice Node PID: $VOICE_PID, PGID: $VOICE_PGID"
sleep 3

echo "=== Running Phase 9 Verification Suite ==="
TEST_EXIT_CODE=0
python3 /home/ws/src/airport_robot_voice/scripts/verify_phase9_gazebo_escort.py &
TEST_PID=$!
wait $TEST_PID || TEST_EXIT_CODE=$?
TEST_PID=""

echo "Test driver finished with exit code: $TEST_EXIT_CODE"
if [ $TEST_EXIT_CODE -ne 0 ]; then
    echo "=== Phase 9 Verification FAILED (exit code $TEST_EXIT_CODE) ==="
    exit $TEST_EXIT_CODE
fi

echo "=== Phase 9 Verification Finished Successfully ==="
exit 0
