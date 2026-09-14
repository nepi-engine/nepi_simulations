#!/bin/bash
#
# Integration test for the rigid dual-camera rework (generic_rover/model.sdf,
# camera_rig_controller.py, sim_bridge_node.py) and the driving/reset paths
# it shares infrastructure with. Not a unit test -- there is no mocking
# framework in this codebase for Gazebo/ROS state, so this spins up a real
# throwaway roscore + headless gzserver + the two sim-side nodes, drives real
# checks against them, and tears everything down. Safe to run repeatedly;
# never touches the real NEPI device.
#
# Usage: bash test_camera_rework.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

PASS=0
FAIL=0

pass() { echo "PASS: $1"; PASS=$((PASS+1)); }
fail() { echo "FAIL: $1"; FAIL=$((FAIL+1)); }

cleanup() {
  pkill -x gzclient >/dev/null 2>&1
  pkill -x gzserver >/dev/null 2>&1
  pkill -f "camera_rig_controller.py" >/dev/null 2>&1
  pkill -f "sim_bridge_node.py" >/dev/null 2>&1
  if [ "${STARTED_ROSCORE:-0}" = "1" ]; then
    pkill -x roscore >/dev/null 2>&1
    pkill -x rosmaster >/dev/null 2>&1
  fi
}
trap cleanup EXIT

source /opt/ros/noetic/setup.bash
export GAZEBO_MODEL_PATH="$SIM_DIR/models:${GAZEBO_MODEL_PATH:-}"

STARTED_ROSCORE=0
if ! rostopic list >/dev/null 2>&1; then
  nohup roscore > /tmp/test_camera_rework_roscore.log 2>&1 &
  disown
  STARTED_ROSCORE=1
  until rostopic list >/dev/null 2>&1; do sleep 1; done
fi

nohup rosrun gazebo_ros gzserver "$SIM_DIR/worlds/generic_rover.world" > /tmp/test_camera_rework_gzserver.log 2>&1 &
disown
until pgrep -x gzserver >/dev/null; do sleep 1; done
until rostopic list 2>/dev/null | grep -q "/rover/camera/image_raw"; do sleep 1; done
sleep 3

nohup python3 -u "$SIM_DIR/scripts/camera_rig_controller.py" > /tmp/test_camera_rework_camctl.log 2>&1 &
disown
nohup python3 -u "$SIM_DIR/scripts/sim_bridge_node.py" > /tmp/test_camera_rework_bridge.log 2>&1 &
disown
sleep 3

TOPICS="$(rostopic list 2>/dev/null)"

# Test 1: both onboard cameras are rigid links on the rover itself and publish.
if echo "$TOPICS" | grep -q "^/rover/camera/image_raw$"; then
  pass "first-person camera topic /rover/camera/image_raw exists"
else
  fail "first-person camera topic /rover/camera/image_raw missing"
fi
if echo "$TOPICS" | grep -q "^/rover/camera_chase/image_raw$"; then
  pass "chase camera topic /rover/camera_chase/image_raw exists"
else
  fail "chase camera topic /rover/camera_chase/image_raw missing"
fi

H1="$(timeout -s KILL 5 rostopic echo -n1 /rover/camera/image_raw/height 2>/dev/null | head -1 | tr -d '[:space:]')"
if [ "$H1" = "480" ]; then
  pass "first-person camera publishing real frames (height=480)"
else
  fail "first-person camera not publishing frames (got '$H1')"
fi
H2="$(timeout -s KILL 5 rostopic echo -n1 /rover/camera_chase/image_raw/height 2>/dev/null | head -1 | tr -d '[:space:]')"
if [ "$H2" = "480" ]; then
  pass "chase camera publishing real frames (height=480)"
else
  fail "chase camera not publishing frames (got '$H2')"
fi

# Test 2: regression guard -- the old standalone camera_rig model/topics
# must be gone now that both cameras are rigid links on generic_rover.
if echo "$TOPICS" | grep -q "^/camera_rig/camera/"; then
  fail "old standalone camera_rig model topics still present (rework incomplete)"
else
  pass "old standalone camera_rig model is gone (no /camera_rig/camera/* topics)"
fi

# Test 3: driving still works and doesn't introduce lateral/yaw drift.
rostopic pub -1 /rover/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.4, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' >/dev/null 2>&1
sleep 2
POS="$(timeout -s KILL 3 rostopic echo -n1 /rover/odom/pose/pose/position 2>/dev/null)"
rostopic pub -1 /rover/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' >/dev/null 2>&1
X_VAL="$(echo "$POS" | grep '^x:' | awk '{print $2}')"
Y_VAL="$(echo "$POS" | grep '^y:' | awk '{print $2}')"
if [ -n "$X_VAL" ] && python3 -c "import sys; sys.exit(0 if float('$X_VAL') > 0.5 else 1)" 2>/dev/null; then
  pass "rover drives forward on cmd_vel (x=$X_VAL)"
else
  fail "rover did not move forward as expected (x='$X_VAL')"
fi
if [ -n "$Y_VAL" ] && python3 -c "import sys; sys.exit(0 if abs(float('$Y_VAL')) < 0.1 else 1)" 2>/dev/null; then
  pass "no unexpected lateral/yaw drift while driving straight (y=$Y_VAL)"
else
  fail "unexpected lateral drift while driving straight (y='$Y_VAL')"
fi

# Test 4: camera_settings protocol still switches which onboard camera is
# relayed, and the relay keeps publishing in both modes (no crash on switch).
python3 - <<'EOF'
import socket, json, time
s = socket.create_connection(('127.0.0.1', 9023), timeout=3)
s.sendall((json.dumps({'type': 'camera_settings', 'view_mode': 'THIRD_PERSON'}) + '\n').encode())
time.sleep(0.5)
s.close()
EOF
sleep 1
MODE="$(rosparam get /sim/camera/view_mode 2>/dev/null | tr -d '[:space:]')"
if [ "$MODE" = "THIRD_PERSON" ]; then
  pass "camera_settings bridge protocol sets view_mode to THIRD_PERSON"
else
  fail "view_mode param did not update to THIRD_PERSON (got '$MODE')"
fi
FMT="$(timeout -s KILL 4 rostopic echo -n1 /camera_rig/image_compressed/format 2>/dev/null)"
if echo "$FMT" | grep -q "jpeg"; then
  pass "relay still publishing compressed frames in THIRD_PERSON mode"
else
  fail "relay stopped publishing after switching to THIRD_PERSON"
fi

python3 - <<'EOF'
import socket, json, time
s = socket.create_connection(('127.0.0.1', 9023), timeout=3)
s.sendall((json.dumps({'type': 'camera_settings', 'view_mode': 'FIRST_PERSON'}) + '\n').encode())
time.sleep(0.5)
s.close()
EOF
sleep 1
MODE="$(rosparam get /sim/camera/view_mode 2>/dev/null | tr -d '[:space:]')"
if [ "$MODE" = "FIRST_PERSON" ]; then
  pass "camera_settings bridge protocol switches view_mode back to FIRST_PERSON"
else
  fail "view_mode param did not switch back to FIRST_PERSON (got '$MODE')"
fi
FMT="$(timeout -s KILL 4 rostopic echo -n1 /camera_rig/image_compressed/format 2>/dev/null)"
if echo "$FMT" | grep -q "jpeg"; then
  pass "relay still publishing compressed frames back in FIRST_PERSON mode"
else
  fail "relay stopped publishing after switching back to FIRST_PERSON"
fi

# Test 5: RESET_SIM bridge command still teleports the rover back to spawn.
rostopic pub -1 /rover/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.4, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' >/dev/null 2>&1
sleep 2
rostopic pub -1 /rover/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' >/dev/null 2>&1
python3 - <<'EOF'
import socket, json, time
s = socket.create_connection(('127.0.0.1', 9023), timeout=3)
s.sendall((json.dumps({'type': 'reset'}) + '\n').encode())
time.sleep(1.0)
s.close()
EOF
sleep 1
POS2="$(timeout -s KILL 3 rostopic echo -n1 /rover/odom/pose/pose/position 2>/dev/null)"
X2="$(echo "$POS2" | grep '^x:' | awk '{print $2}')"
if [ -n "$X2" ] && python3 -c "import sys; sys.exit(0 if abs(float('$X2')) < 0.2 else 1)" 2>/dev/null; then
  pass "RESET_SIM bridge command teleports rover back to spawn (x=$X2)"
else
  fail "RESET_SIM did not return rover to spawn (x='$X2')"
fi

echo ""
echo "===================================="
echo "Results: $PASS passed, $FAIL failed"
echo "===================================="
[ "$FAIL" -eq 0 ]
