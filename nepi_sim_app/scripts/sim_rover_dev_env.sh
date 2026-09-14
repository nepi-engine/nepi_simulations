# NEPI generic-rover simulator dev VM environment -- source this from ~/.bashrc.
#
# Provides the one-command roscore + Gazebo + sim bridge launcher for the
# Universal Simulator Bridge Phase 1 workflow (see
# sim_container/ROVER_GAZEBO_BRIDGE_IMPL_PLAN.md, renamed 2026-08-05). Deliberately a separate command from
# sitl_gazebo (nepi_sitl_dev_env.sh): the ArduPilot workflow speaks
# MAVLink/FDM directly to SITL and never needs ROS on this VM, while this
# rover workflow is ROS-native (gazebo_ros_diff_drive / gazebo_ros_camera)
# and needs its own local roscore. The two are independent simulations that
# run at different times. Add to ~/.bashrc:
#
#   source /path/to/nepi_drones/sim_container/scripts/sim_rover_dev_env.sh
#
# Requires: gazebo11, ros-noetic-ros-base, ros-noetic-gazebo-ros-pkgs
# (installed 2026-07-22 -- see .claude/sessions/2026-07-22-universal-sim-phase1.md).

# Resolved at source time so the functions work from any cwd.
NEPI_DRONES_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Tiny liveness listener for the rover sim: listens on 127.0.0.1:<port> and
# replies ALIVE to any connection. Exists so the NEPI rbx_sim driver's
# discovery -- which runs on the remote NEPI device, not this VM -- can reach
# across the existing reverse SSH tunnel (nepi_tunnel forwards this port; see
# nepi_sitl_dev_env.sh) and confirm the sim stack is up. Not a ROS node on
# purpose: the two machines have separate ROS masters, so the device can never
# see this VM's /sim/heartbeat topic -- only a raw TCP port survives the
# tunnel. Same pattern as gz_reset_listener in nepi_sitl_dev_env.sh.
sim_heartbeat_listener() {
    local port="${1:-9022}"
    if pgrep -f "sim_heartbeat_listener.py $port" > /dev/null; then
        echo "sim heartbeat listener already running on 127.0.0.1:$port"
        return 0
    fi
    nohup python3 -u "$NEPI_DRONES_SIM_DIR/scripts/sim_heartbeat_listener.py" "$port" \
        > /tmp/sim_heartbeat_listener.log 2>&1 &
    disown
    echo "sim heartbeat listener started on 127.0.0.1:$port"
}

# Camera-rig follow controller (single-robot world): repositions the
# standalone camera_rig model onto the rover every frame via
# /gazebo/set_model_state. Previously had to be started by hand in a
# separate terminal (undocumented for this workflow -- only the ArduPilot
# variant had a bash function at all) -- until it's running, camera_rig just
# sits at its raw generic_rover.world spawn pose (2 2 1), which reads as
# "floating randomly away from the rover" once the rover drives off. Started
# here so the camera is on the rover by default, same as the heartbeat
# listener and tunnel above.
camera_rig_controller() {
    if pgrep -f "camera_rig_controller.py" > /dev/null; then
        echo "camera rig controller already running"
        return 0
    fi
    nohup python3 -u "$NEPI_DRONES_SIM_DIR/scripts/camera_rig_controller.py" \
        > /tmp/camera_rig_controller.log 2>&1 &
    disown
    echo "camera rig controller started"
}

# Multi-robot counterpart of camera_rig_controller (drives both camera_rig1
# and camera_rig2 in one process, mirroring RobotBridge's per-slot pattern).
camera_rig_controller_multi() {
    if pgrep -f "camera_rig_controller_multi.py" > /dev/null; then
        echo "camera rig controller (multi) already running"
        return 0
    fi
    nohup python3 -u "$NEPI_DRONES_SIM_DIR/scripts/camera_rig_controller_multi.py" \
        > /tmp/camera_rig_controller_multi.log 2>&1 &
    disown
    echo "camera rig controller (multi) started"
}

# Launches a local roscore (only if one isn't already up), then Gazebo with
# the generic rover world (via the gazebo_ros wrapper, which loads the ROS
# API plugin the diff-drive/camera plugins need), then runs
# sim_bridge_node.py in the foreground. Ctrl-C tears down Gazebo -- and the
# roscore too, but only if this function started it.
sim_rover_gazebo() {
    source /opt/ros/noetic/setup.bash
    export GAZEBO_MODEL_PATH="$NEPI_DRONES_SIM_DIR/models:$GAZEBO_MODEL_PATH"

    local started_roscore=0
    trap 'pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null; pkill -f sim_heartbeat_listener.py 2>/dev/null; pkill -f camera_rig_controller.py 2>/dev/null;
          if [ "$started_roscore" = "1" ]; then pkill -x roscore 2>/dev/null; pkill -x rosmaster 2>/dev/null; pkill -f "rosout/rosout" 2>/dev/null; fi;
          trap - INT TERM; return' INT TERM

    if ! rostopic list > /dev/null 2>&1; then
        echo "Starting local roscore..."
        nohup roscore > /tmp/sim_rover_roscore.log 2>&1 &
        disown
        started_roscore=1
        until rostopic list > /dev/null 2>&1; do
            sleep 1
        done
    else
        echo "roscore already running -- reusing it"
    fi

    echo "Starting Gazebo (generic_rover.world)..."
    rosrun gazebo_ros gazebo "$NEPI_DRONES_SIM_DIR/worlds/generic_rover.world" &

    echo "Waiting for Gazebo to finish loading..."
    until pgrep -x gzserver > /dev/null; do
        sleep 1
    done
    sleep 8

    sim_heartbeat_listener
    camera_rig_controller
    # The shared reverse tunnel to the NEPI device (defined in
    # nepi_sitl_dev_env.sh, which forwards this workflow's heartbeat port too).
    # Guarded in case only this file is sourced.
    if declare -F nepi_tunnel > /dev/null; then
        nepi_tunnel
    else
        echo "WARNING: nepi_tunnel not defined (source nepi_sitl_dev_env.sh) -- NEPI device cannot reach the sim"
    fi

    echo "Starting simulator bridge node (Ctrl-C to shut everything down)..."
    python3 "$NEPI_DRONES_SIM_DIR/scripts/sim_bridge_node.py"

    pkill -x gzclient 2>/dev/null
    pkill -x gzserver 2>/dev/null
    pkill -f sim_heartbeat_listener.py 2>/dev/null
    pkill -f camera_rig_controller.py 2>/dev/null
    if [ "$started_roscore" = "1" ]; then
        pkill -x roscore 2>/dev/null
        pkill -x rosmaster 2>/dev/null
        pkill -f "rosout/rosout" 2>/dev/null
    fi
    trap - INT TERM
}

# Manual test commands for the running generic-rover sim (sim_rover_gazebo).
# Talk directly to this VM's local /rover/cmd_vel + /rover/odom -- see
# sim_move.py's module docstring for why that's deliberately independent of
# the sim bridge / NEPI device relay. Only meaningful once Gazebo is up.
move() {
    source /opt/ros/noetic/setup.bash > /dev/null 2>&1
    python3 "$NEPI_DRONES_SIM_DIR/scripts/sim_move.py" "$@"
}

# Immediate stop -- in case a move overshoots or the rover is drifting and
# you don't want to wait for a command to finish.
stop() {
    source /opt/ros/noetic/setup.bash > /dev/null 2>&1
    rostopic pub -1 /rover/cmd_vel geometry_msgs/Twist \
        "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" > /dev/null
    echo "stopped"
}

testcommands() {
    cat <<'EOF'
Rover sim test commands (run after sim_rover_gazebo is up):

  move 10x              drive forward 10m (world x / east)
  move -5x               drive backward 5m
  move 5y                drive so as to end up 5m further north (world y)
  move 10x 5y            combined x+y move (turns to face the point, then drives)
  move 45yaw             turn in place 45 degrees (relative, left-positive)
  move 10x 45yaw         drive 10m forward, then turn 45 degrees
  move 10x, 5y, 3z, 45yaw   commas are fine -- z is accepted but ignored (ground rover)
  stop                   zero velocity immediately
  testcommands           show this list

Notes:
  - x/y are relative meters in the world frame, added to the rover's current
    position (not "forward/left" relative to which way it's currently facing).
  - yaw is relative degrees, applied as a final in-place turn after the drive.
  - These commands drive Gazebo directly and will fight the real NEPI device
    driver if it's actively connected and issuing commands at the same time --
    use them for standalone sim testing.
EOF
}

# Multi-robot variant of sim_rover_gazebo (Universal Simulator Bridge,
# Phase 4): launches generic_rover_multi.world (two independently-namespaced
# rover instances, /rover1 and /rover2), one heartbeat listener per robot
# (9022 for rover1, 9024 for rover2), and sim_bridge_multi_node.py (one
# bridge server per robot: 9023 for rover1, 9025 for rover2). Deliberately a
# separate command from sim_rover_gazebo, matching how that command was kept
# separate from the ArduPilot sitl_gazebo -- the single-robot workflow stays
# exactly as it was; run one or the other, not both (they share ports
# 9022/9023 and the Gazebo master port).
sim_rover_gazebo_multi() {
    source /opt/ros/noetic/setup.bash
    export GAZEBO_MODEL_PATH="$NEPI_DRONES_SIM_DIR/models:$GAZEBO_MODEL_PATH"

    local started_roscore=0
    trap 'pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null; pkill -f sim_heartbeat_listener.py 2>/dev/null; pkill -f camera_rig_controller_multi.py 2>/dev/null;
          if [ "$started_roscore" = "1" ]; then pkill -x roscore 2>/dev/null; pkill -x rosmaster 2>/dev/null; pkill -f "rosout/rosout" 2>/dev/null; fi;
          trap - INT TERM; return' INT TERM

    if ! rostopic list > /dev/null 2>&1; then
        echo "Starting local roscore..."
        nohup roscore > /tmp/sim_rover_roscore.log 2>&1 &
        disown
        started_roscore=1
        until rostopic list > /dev/null 2>&1; do
            sleep 1
        done
    else
        echo "roscore already running -- reusing it"
    fi

    echo "Starting Gazebo (generic_rover_multi.world)..."
    rosrun gazebo_ros gazebo "$NEPI_DRONES_SIM_DIR/worlds/generic_rover_multi.world" &

    echo "Waiting for Gazebo to finish loading..."
    until pgrep -x gzserver > /dev/null; do
        sleep 1
    done
    sleep 8

    # One heartbeat listener per robot slot (rover1 keeps the single-robot
    # workflow's 9022; rover2 gets 9024)
    sim_heartbeat_listener 9022
    sim_heartbeat_listener 9024
    camera_rig_controller_multi
    # The shared reverse tunnel to the NEPI device (defined in
    # nepi_sitl_dev_env.sh, which forwards both robots' port pairs).
    # Guarded in case only this file is sourced.
    if declare -F nepi_tunnel > /dev/null; then
        nepi_tunnel
    else
        echo "WARNING: nepi_tunnel not defined (source nepi_sitl_dev_env.sh) -- NEPI device cannot reach the sim"
    fi

    echo "Starting multi-robot simulator bridge node (Ctrl-C to shut everything down)..."
    python3 "$NEPI_DRONES_SIM_DIR/scripts/sim_bridge_multi_node.py"

    pkill -x gzclient 2>/dev/null
    pkill -x gzserver 2>/dev/null
    pkill -f sim_heartbeat_listener.py 2>/dev/null
    pkill -f camera_rig_controller_multi.py 2>/dev/null
    if [ "$started_roscore" = "1" ]; then
        pkill -x roscore 2>/dev/null
        pkill -x rosmaster 2>/dev/null
        pkill -f "rosout/rosout" 2>/dev/null
    fi
    trap - INT TERM
}
