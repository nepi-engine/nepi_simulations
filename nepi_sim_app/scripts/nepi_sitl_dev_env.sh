# NEPI ArduPilot SITL dev VM environment -- source this from ~/.bashrc.
#
# Provides the one-command Gazebo + ArduPilot SITL launcher used for testing
# the rbx_ardupilot driver against the real NEPI device's RUI (see
# docs/SIMULATOR_DEV_GUIDE.md). Add to ~/.bashrc:
#
#   source /path/to/nepi_drones/sim_container/scripts/nepi_sitl_dev_env.sh
#
# Requires: gazebo, ArduPilot's sim_vehicle.py on PATH, ~/ardupilot_gazebo
# world files, and autossh (`sudo apt-get install autossh`) for nepi_tunnel.

alias nepi_gazebo='gazebo ~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world'
alias nepi_sitl='sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map'
alias sitl='sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map'

# Resolved at source time so sitl_gazebo works from any cwd (camera-rig
# feature: needs nepi_drones/sim_container/models on GAZEBO_MODEL_PATH so
# `model://camera_rig` resolves -- see camera_rig_controller_ardupilot.py).
NEPI_DRONES_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Launches Gazebo, waits for it to fully load (so the ArduPilotPlugin's FDM
# socket is up before SITL starts sending), then runs SITL in the foreground
# so its MAVProxy prompt is right there in the terminal. Ctrl-C / `quit` in
# MAVProxy tears Gazebo down too.
# Tiny local trigger for "reset the sim": listens on 127.0.0.1:<port> and runs
# `gz world -o` (reset model poses only -- NOT -r/time, which crashes the
# connected ArduPilot SITL binary on a time discontinuity) on any connection.
# Exists so the NEPI RBX driver -- which runs on the remote NEPI device, not
# this VM -- can reach across the existing reverse SSH tunnel and trigger a
# Gazebo reset without its own SSH creds here. See gz_reset_listener.py
# (installed alongside this script; copy or symlink it to ~/.local/bin/).
gz_reset_listener() {
    local port="${1:-9021}"
    if pgrep -f "gz_reset_listener.py $port" > /dev/null; then
        echo "gz reset listener already running on 127.0.0.1:$port"
        return 0
    fi
    nohup python3 -u ~/.local/bin/gz_reset_listener.py "$port" > /tmp/gz_reset_listener.log 2>&1 &
    disown
    echo "gz reset listener started on 127.0.0.1:$port"
}

# Tiny local trigger for "get the whole sim stack running": listens on
# 127.0.0.1:<port> and fires sitl_gazebo_full() (idempotent -- only starts
# whatever isn't already up) on any connection. Exists so the NEPI device --
# which can't open a fresh connection into this VM, only reach ports this
# VM's own reverse tunnel already forwards back -- can get Gazebo/SITL/the
# camera-rig and AI-targeting controllers running without any SSH creds of
# its own. See sim_launch_listener.py (installed alongside this script; copy
# it to ~/.local/bin/). Started alongside gz_reset_listener from both
# sitl_gazebo and sitl_gazebo_full, so it's available whenever either has
# been run at least once -- there is no way to reach a genuinely cold VM
# (nothing started here yet) from the device side; something has to be
# launched here manually first.
sim_launch_listener() {
    local port="${1:-9028}"
    if pgrep -f "sim_launch_listener.py $port" > /dev/null; then
        echo "sim launch listener already running on 127.0.0.1:$port"
        return 0
    fi
    nohup python3 -u ~/.local/bin/sim_launch_listener.py "$port" "$NEPI_DRONES_SIM_DIR/scripts/nepi_sitl_dev_env.sh" \
        > /tmp/sim_launch_listener.log 2>&1 &
    disown
    echo "sim launch listener started on 127.0.0.1:$port"
}

# Keeps this VM linked to the real NEPI device so its RBX ArduPilot driver
# (which runs on the NEPI device, not here) can reach this VM's SITL/reset
# listener over their shared loopback. Persistent/idempotent on purpose --
# not tied to sitl_gazebo's lifecycle, so it's fine to leave running between
# SITL sessions. Forwards: 5771 (MAVProxy's dedicated --out port -- this is
# the one the RBX driver's discovery actually connects to; see sitl_gazebo
# below), 5760 (SITL's raw/primary port, forwarded for any other direct use --
# NOT used by driver discovery, which only tries 5771), 9021
# (gz_reset_listener, for the RESET_SIM RUI action), 9022
# (sim_heartbeat_listener, the rover-sim liveness port the rbx_sim driver's
# discovery probes -- see sim_rover_dev_env.sh; chosen as the next port in the
# 902x block of tiny sim-utility listeners started by gz_reset_listener's
# 9021, clear of the 576x MAVLink ports), 9023 (the rover-sim
# command/telemetry bridge served by sim_bridge_node.py -- the persistent
# JSON-lines connection rbx_sim_node.py holds open, next port in the same
# 902x block after the 9022 heartbeat), and 9024/9025 (the second rover's
# heartbeat/bridge pair for the multi-robot workflow's sim_rover_gazebo_multi
# -- see sim_rover_dev_env.sh; rover1 reuses 9022/9023, rover2 gets the next
# pair in the same 902x block, served by sim_heartbeat_listener.py 9024 and
# sim_bridge_multi_node.py), and 9026 (the ArduPilot camera-rig bridge --
# camera_rig_controller_ardupilot.py's own TCP JSON-lines server, next free
# port in the 902x block; carries ONLY camera settings in and compressed
# frames out, since MAVLink over 5771 already carries telemetry/commands for
# this driver), and 9027 (the ArduPilot simulated AI-targeting bridge --
# ai_targeting_controller_ardupilot.py's own TCP JSON-lines server, next free
# port in the 902x block after the camera bridge; outbound-only, streams
# synthetic range/azimuth/elevation for sim_ai_targeting_bridge_script.py to
# republish as Targets on the NEPI device, standing in for the
# app_ai_targeting app drone_follow_object_mission_script.py otherwise has no
# way to test against), and 9028 (sim_launch_listener -- lets the device
# trigger sitl_gazebo_full remotely to bring up whatever of this whole stack
# isn't already running, since the device has no other way to reach this VM),
# and 12222 (this VM's own sshd -- the sim connector app's launch/stop/install
# flow SSHes out from the device to 127.0.0.1:12222 as configured in each
# launch target's host/ssh_port in simulator_launch_targets.yaml, since that
# app drives gazebo_rover/gazebo_quadcopter directly by command rather than
# through sim_launch_listener's own narrower remote-trigger protocol).
# This one tunnel serves both
# simulation workflows (ArduPilot SITL and the generic rover sim).
# Uses autossh (not plain ssh) so the tunnel reconnects on its own whenever
# either side restarts -- a power-cycle of the NEPI device kills its sshd and
# drops a plain ssh tunnel for good, requiring a manual nepi_tunnel re-run.
# With autossh, it doesn't matter which of sitl_gazebo / the NEPI device
# comes up first or restarts later -- autossh just keeps retrying the
# connection until the other side is reachable again.
#
# NOTE: the tunnel is now normally owned by the nepi-tunnel systemd USER
# service (../systemd/nepi-tunnel.service), which starts it at boot and
# restarts it if it dies -- the two failure modes this function's own
# `nohup ... &` could never survive. Confirmed the hard way twice: a VM
# reboot silently dropped the tunnel, and every sim connector Deploy then
# failed with "connect to host 127.0.0.1 port 12222: Connection refused"
# until someone re-ran this by hand. This function is kept for manual/ad-hoc
# use and defers to the service when it's active, rather than starting a
# SECOND autossh that would race the first for the same forwards (the loser
# exits on ExitOnForwardFailure, but which one loses is a coin flip).
nepi_tunnel() {
    # NEPI_DEVICE_SSH_HOST/USER/PORT let this point at a real NEPI device
    # other than this one specific developer's own setup, with no edit to
    # this script -- "nepi"/"nepi"/2222 are the platform's own default
    # device username and dev-VM-facing sshd port (see nepi_setup), not a
    # personal value the way simulator_launcher.py's ssh_user "suraj" is,
    # but still worth overriding for a device configured differently. See
    # docs/SIM_VM_CONNECTION_SETUP.md for the full two-machine setup
    # walkthrough (also covers simulator_launcher.py's own equivalent
    # NEPI_SIM_VM_* variables for the other direction).
    local device_host="${NEPI_DEVICE_SSH_HOST:-nepi}"
    local device_user="${NEPI_DEVICE_SSH_USER:-nepi}"
    local device_port="${NEPI_DEVICE_SSH_PORT:-2222}"
    if systemctl --user is-active nepi-tunnel.service > /dev/null 2>&1; then
        echo "NEPI reverse tunnel is managed by the nepi-tunnel systemd service (already active)"
        echo "  status: systemctl --user status nepi-tunnel.service"
        echo "  restart: systemctl --user restart nepi-tunnel.service"
        return 0
    fi
    if pgrep -f "autossh.*R 5771:127.0.0.1:5771.*${device_user}@${device_host}" > /dev/null; then
        echo "NEPI reverse tunnel already running"
        return 0
    fi
    AUTOSSH_GATETIME=0 nohup autossh -M 0 -p "$device_port" -i ~/.ssh/nepi_default_ssh_key \
        -o ConnectTimeout=5 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
        -R 5760:127.0.0.1:5760 -R 5771:127.0.0.1:5771 -R 9021:127.0.0.1:9021 -R 9022:127.0.0.1:9022 -R 9023:127.0.0.1:9023 -R 9024:127.0.0.1:9024 -R 9025:127.0.0.1:9025 -R 9026:127.0.0.1:9026 -R 9027:127.0.0.1:9027 -R 9028:127.0.0.1:9028 -R 9041:127.0.0.1:9041 -R 9042:127.0.0.1:9042 -R 9046:127.0.0.1:9046 -R 9047:127.0.0.1:9047 -R 12222:127.0.0.1:22 \
        -N "${device_user}@${device_host}" > /tmp/nepi_tunnel.log 2>&1 &
    disown
    sleep 2
    if pgrep -f "autossh.*R 5771:127.0.0.1:5771.*${device_user}@${device_host}" > /dev/null; then
        echo "NEPI reverse tunnel started (auto-reconnecting via autossh)"
    else
        echo "NEPI reverse tunnel FAILED to start -- check /tmp/nepi_tunnel.log"
    fi
}

sitl_gazebo() {
    # Ctrl-C only signals SITL's own process group, not this shell, so we
    # can't rely on a trap firing when this function returns -- clean up
    # Gazebo explicitly after sim_vehicle.py exits (below), and also on
    # INT/TERM in case you cancel during the "waiting for Gazebo" phase.
    # Camera-rig feature: this workflow previously ran Gazebo standalone
    # (plain `gazebo`, no ROS at all -- MAVLink was the only channel). The
    # camera rig's libgazebo_ros_camera.so plugin and /gazebo/model_states
    # pose feed (read by camera_rig_controller_ardupilot.py) both require the
    # gazebo_ros API plugin, which requires a roscore. started_roscore tracks
    # whether THIS invocation started one, so it's only torn down if we
    # started it (matches sim_rover_gazebo's own convention in
    # sim_rover_dev_env.sh).
    local started_roscore=0
    trap 'pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null; pkill -f gz_reset_listener.py 2>/dev/null;
          if [ "$started_roscore" = "1" ]; then pkill -x roscore 2>/dev/null; pkill -x rosmaster 2>/dev/null; pkill -f "rosout/rosout" 2>/dev/null; fi;
          trap - INT TERM; return' INT TERM

    source /opt/ros/noetic/setup.bash
    export GAZEBO_MODEL_PATH="$NEPI_DRONES_SIM_DIR/models:$GAZEBO_MODEL_PATH"

    if ! rostopic list > /dev/null 2>&1; then
        echo "Starting local roscore..."
        nohup roscore > /tmp/sitl_gazebo_roscore.log 2>&1 &
        disown
        started_roscore=1
        until rostopic list > /dev/null 2>&1; do
            sleep 1
        done
    else
        echo "roscore already running -- reusing it"
    fi

    echo "Starting Gazebo..."
    rosrun gazebo_ros gazebo ~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world &

    echo "Waiting for Gazebo to finish loading..."
    until pgrep -x gzserver > /dev/null; do
        sleep 1
    done
    sleep 8

    gz_reset_listener
    sim_launch_listener
    nepi_tunnel

    # --out=tcpin:0.0.0.0:5771 gives MAVProxy a second, dedicated TCP port for
    # the NEPI RBX driver's mavros -- without it, MAVProxy alone occupies the
    # primary port 5760 as SITL's sole MAVLink client, and mavros can never
    # connect (the drone never shows up under Devices in the RUI). MAVProxy
    # still keeps 5760/--console/--map for you exactly as before.
    echo "Starting ArduPilot SITL..."
    sim_vehicle.py -v ArduCopter -f gazebo-iris --out=tcpin:0.0.0.0:5771 --console --map

    pkill -x gzclient 2>/dev/null
    pkill -x gzserver 2>/dev/null
    pkill -f gz_reset_listener.py 2>/dev/null
    if [ "$started_roscore" = "1" ]; then
        pkill -x roscore 2>/dev/null
        pkill -x rosmaster 2>/dev/null
        pkill -f "rosout/rosout" 2>/dev/null
    fi
    trap - INT TERM
}

# Camera-rig controller for the ArduPilot SITL sim (Universal Simulator
# Bridge camera feature, ArduPilot port). Not auto-started by sitl_gazebo --
# same manual-launch convention as the rover workflow's camera controllers
# (see sim_rover_dev_env.sh / camera_rig_controller.py). Run this in a
# separate terminal/screen session after sitl_gazebo is up.
camera_rig_controller_ardupilot() {
    source /opt/ros/noetic/setup.bash
    python3 "$NEPI_DRONES_SIM_DIR/scripts/camera_rig_controller_ardupilot.py"
}

# Simulated AI-targeting controller for the ArduPilot SITL sim (test
# scaffolding standing in for the missing app_ai_targeting app -- see
# drone_follow_object_mission_script.py's own "KNOWN GAP" docstring). Spawns
# a moving "chair" target object into the running Gazebo world and streams
# synthetic range/azimuth/elevation over its own TCP bridge (port 9027) for
# sim_ai_targeting_bridge_script.py (run as a NEPI automation script on the
# device) to republish as Targets. Not auto-started by sitl_gazebo -- same
# manual-launch convention as camera_rig_controller_ardupilot: run this in a
# separate terminal/screen session after sitl_gazebo is up.
ai_targeting_controller_ardupilot() {
    source /opt/ros/noetic/setup.bash
    python3 "$NEPI_DRONES_SIM_DIR/scripts/ai_targeting_controller_ardupilot.py"
}

# One-command, fully-detached equivalent of running sitl_gazebo plus both
# controllers in four separate terminals. sitl_gazebo itself stays
# interactive/foreground on purpose (it's meant to hand you MAVProxy's own
# console/map) -- this is for the opposite case: get the whole stack up and
# just leave it running, e.g. so a NEPI drone script's full requirement chain
# (RBX driver + camera feed + target-localization feed, see
# drone_follow_object_mission_script.py's ScriptDocs entry) is satisfied
# without babysitting multiple windows. Idempotent piece-by-piece -- safe to
# re-run any time, only starts whatever isn't already up. Logs for anything
# it starts land in /tmp/sitl_gazebo_full_*.log.
sitl_gazebo_full() {
    source /opt/ros/noetic/setup.bash
    export GAZEBO_MODEL_PATH="$NEPI_DRONES_SIM_DIR/models:$GAZEBO_MODEL_PATH"

    # Checked independently (not as one combined condition) -- Gazebo can be
    # up while SITL isn't (e.g. a prior SITL crash/exit left gzserver
    # running), and re-running `rosrun gazebo_ros gazebo` in that case is a
    # wasted, risky duplicate launch attempt: Gazebo's own singleton lock
    # blocks the second gzserver from actually binding, but the redundant
    # wrapper process still spins up and adds needless load. Confirmed live
    # (2026-08-11) this exact case happens after a headless SITL failure.
    if ! rostopic list > /dev/null 2>&1; then
        echo "Starting local roscore..."
        nohup roscore > /tmp/sitl_gazebo_full_roscore.log 2>&1 &
        disown
        until rostopic list > /dev/null 2>&1; do
            sleep 1
        done
    else
        echo "roscore already running -- reusing it"
    fi

    if pgrep -x gzserver > /dev/null; then
        echo "Gazebo already running -- reusing"
    else
        echo "Starting Gazebo..."
        nohup rosrun gazebo_ros gazebo ~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world \
            > /tmp/sitl_gazebo_full_gazebo.log 2>&1 &
        disown

        echo "Waiting for Gazebo to finish loading..."
        until pgrep -x gzserver > /dev/null; do
            sleep 1
        done
        sleep 8
    fi

    if pgrep -f "sim_vehicle.py -v ArduCopter" > /dev/null; then
        echo "ArduPilot SITL already running -- reusing"
    else
        # Same --out=tcpin:0.0.0.0:5771 dedicated port as sitl_gazebo, for the
        # same reason (mavros needs its own port alongside MAVProxy's
        # primary one on 5760) -- just without --console/--map, since
        # there's no terminal attached to show them to and they'd only try
        # (and fail) to open GUI windows under nohup anyway.
        #
        # --mavproxy-args="--daemon" is required, not optional, for a nohup'd
        # headless launch: without it, MavProxy's main loop calls
        # input_loop() (mavproxy.py's main()), which blocks reading from
        # stdin. Under nohup with no controlling terminal, stdin is
        # immediately at EOF, so MavProxy treats that as "user quit" and runs
        # its full unload-every-module-and-exit shutdown sequence within
        # seconds of starting -- ArduCopter itself boots fine (confirmed by
        # running it standalone), but MavProxy tears the whole session down
        # before mavros/the RBX driver ever gets a chance to connect, so
        # "Gazebo comes up but ArduPilot SITL never does" (confirmed live,
        # 2026-08-11). --daemon skips input_loop() entirely (mpstate.status
        # loop just sleeps instead of blocking on stdin), which is exactly
        # what a nohup'd/headless launch needs -- confirmed fixed live: with
        # this flag, ArduCopter reaches full EKF/AHRS-active boot and keeps
        # running indefinitely instead of exiting within ~5s.
        echo "Starting ArduPilot SITL..."
        nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --out=tcpin:0.0.0.0:5771 \
            --mavproxy-args="--daemon" \
            > /tmp/sitl_gazebo_full_sitl.log 2>&1 &
        disown
    fi

    gz_reset_listener
    sim_launch_listener
    nepi_tunnel

    if pgrep -f "camera_rig_controller_ardupilot.py" > /dev/null; then
        echo "camera_rig_controller_ardupilot already running"
    else
        echo "Starting camera_rig_controller_ardupilot..."
        nohup python3 "$NEPI_DRONES_SIM_DIR/scripts/camera_rig_controller_ardupilot.py" \
            > /tmp/sitl_gazebo_full_camera_rig.log 2>&1 &
        disown
    fi

    if pgrep -f "ai_targeting_controller_ardupilot.py" > /dev/null; then
        echo "ai_targeting_controller_ardupilot already running"
    else
        echo "Starting ai_targeting_controller_ardupilot..."
        nohup python3 "$NEPI_DRONES_SIM_DIR/scripts/ai_targeting_controller_ardupilot.py" \
            > /tmp/sitl_gazebo_full_ai_targeting.log 2>&1 &
        disown
    fi

    echo "sitl_gazebo_full: done -- check /tmp/sitl_gazebo_full_*.log if any piece didn't come up."
    echo "Remember: sim_ai_targeting_bridge_script.py still needs to be launched separately, as a NEPI script on the device itself, to actually produce the target_localizations topic."
}

# Alias for typos / muscle memory -- identical to sitl_gazebo.
gazebo_sitl() {
    sitl_gazebo "$@"
}
