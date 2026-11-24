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

LOG_DIR="/home/ws/src/.agent/person_tracking/evidence/phase_11"
mkdir -p "$LOG_DIR"
BASE_LOG="$LOG_DIR/gazebo_base_runtime.log"
ESCORT_LOG="$LOG_DIR/escort_real_runtime.log"
VOICE_LOG="$LOG_DIR/voice_real_node.log"

BASE_PID=""
BASE_PGID=""
ESCORT_PID=""
ESCORT_PGID=""
VOICE_PID=""
VOICE_PGID=""
TEST_PID=""

cleanup() {
    echo "=== Shutting down processes tracked by this run... ==="
    if [ -n "$TEST_PID" ] && kill -0 "$TEST_PID" 2>/dev/null; then
        kill -TERM "$TEST_PID" 2>/dev/null || true
        wait "$TEST_PID" 2>/dev/null || true
    fi

    if [ -n "$VOICE_PGID" ] && kill -0 -"$VOICE_PGID" 2>/dev/null; then
        kill -TERM -"$VOICE_PGID" 2>/dev/null || true
    elif [ -n "$VOICE_PID" ] && kill -0 "$VOICE_PID" 2>/dev/null; then
        kill -TERM "$VOICE_PID" 2>/dev/null || true
    fi

    if [ -n "$ESCORT_PGID" ] && kill -0 -"$ESCORT_PGID" 2>/dev/null; then
        kill -TERM -"$ESCORT_PGID" 2>/dev/null || true
        for i in $(seq 1 5); do
            if kill -0 -"$ESCORT_PGID" 2>/dev/null; then sleep 1; else break; fi
        done
        kill -KILL -"$ESCORT_PGID" 2>/dev/null || true
    fi

    if [ -n "$BASE_PGID" ] && kill -0 -"$BASE_PGID" 2>/dev/null; then
        kill -TERM -"$BASE_PGID" 2>/dev/null || true
        for i in $(seq 1 5); do
            if kill -0 -"$BASE_PGID" 2>/dev/null; then sleep 1; else break; fi
        done
        kill -KILL -"$BASE_PGID" 2>/dev/null || true
    fi

    wait "$VOICE_PID" 2>/dev/null || true
    wait "$ESCORT_PID" 2>/dev/null || true
    wait "$BASE_PID" 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

echo "=== 1. Starting Isolated Gazebo Base Simulation on ROS_DOMAIN_ID=44 ==="
set -m
ros2 launch person_tracking gazebo_sim_base.launch.py gui:=false sim:=true > "$BASE_LOG" 2>&1 &
BASE_PID=$!
BASE_PGID=$(ps -o pgid= -p $BASE_PID | tr -d " ")
echo "Gazebo Base PID: $BASE_PID, PGID: $BASE_PGID"

echo "Waiting for Gazebo Base & EKF to come online (up to 40s)..."
BASE_READY=false
for i in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -q "ekf_filter_node" && ros2 topic list 2>/dev/null | grep -q "/clock"; then
        echo "Gazebo Base detected online at i=$i! Waiting 3s for clock stabilization..."
        sleep 3
        BASE_READY=true
        break
    fi
    sleep 1
done

if [ "$BASE_READY" != "true" ]; then
    echo "WARNING: Timeout waiting for Gazebo Base nodes!"
fi

echo "=== 2. Starting escort_navigation_real.launch.py (Nav2 + Guard + Watchdog) ==="
ros2 launch person_tracking escort_navigation_real.launch.py     use_sim_time:=true     map:=/home/ws/src/linorobot2/linorobot2_navigation/maps/turtlebot3_world.yaml     max_linear_x:=0.30     max_angular_z:=0.20     > "$ESCORT_LOG" 2>&1 &
ESCORT_PID=$!
ESCORT_PGID=$(ps -o pgid= -p $ESCORT_PID | tr -d " ")
echo "Escort Launch PID: $ESCORT_PID, PGID: $ESCORT_PGID"

echo "Waiting for Nav2 & Escort nodes to come online (up to 60s)..."
ESCORT_READY=false
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "escort_velocity_guard" &&        ros2 node list 2>/dev/null | grep -q "escort_velocity_watchdog" &&        ros2 node list 2>/dev/null | grep -q "collision_monitor" &&        ros2 node list 2>/dev/null | grep -q "bt_navigator"; then
        echo "Nav2 & Escort stack detected online at i=$i! Waiting 5s for lifecycle activation..."
        sleep 5
        ESCORT_READY=true
        break
    fi
    sleep 1
done

if [ "$ESCORT_READY" != "true" ]; then
    echo "WARNING: Timeout waiting for Nav2 / Escort stack nodes!"
fi

echo "=== 3. Starting Real Voice Node with enable_escort:=true ==="
ros2 run airport_robot_voice nav_command_node --ros-args     -p enable_escort:=true     -p use_sim_time:=true     -p places_file:=/home/ws/src/airport_robot_voice/config/places_sim.json     > "$VOICE_LOG" 2>&1 &
VOICE_PID=$!
VOICE_PGID=$(ps -o pgid= -p $VOICE_PID | tr -d " ")
echo "Voice Node PID: $VOICE_PID, PGID: $VOICE_PGID"
sleep 4

echo "=== 4. Running Phase 11 Escort Real Launch Verification Suite ==="
python3 /home/ws/src/person_tracking/test/verify_escort_real_launch.py
TEST_EXIT_CODE=$?

echo "Verification script exited with code $TEST_EXIT_CODE"
exit $TEST_EXIT_CODE
