#!/bin/bash

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# Entrypoint of the nepi_sim image. Runs ONE simulator service per container:
#
#   gazebo   nepi_gazebo_start.sh, then the nepi_gazebo service
#   mujoco   nepi_mujoco_start.sh, then the nepi_mujoco service
#   shell    an interactive shell with the simulator environment loaded
#   help     this usage text
#
# The start scripts are the same ones a native sim machine runs. They
# background the service and return, so this script then stays in the
# foreground for as long as the service runs, streams the service log to
# `docker logs`, and exits when the service does, which stops the container.
#
# `docker stop` (nepi_simulators_stop on the host) reaches this script as
# SIGTERM, forwarded by docker's --init process. It is handled with the same
# nepi_<sim>_stop the native commands use, which has the service stop
# everything it started. That can take up to 30 s, so nepi_simulators_stop gives
# docker stop a longer timeout than docker's 10 s default.
#
# NEPI storage is mounted by the host and bind-mounted in at
# /mnt/nepi_share_storage. This script does not mount it: it checks that it is
# there and stops with a clear message if it is not.

SIM=$1
SHARE=/mnt/nepi_share_storage

function usage(){
    echo "Usage: nepi_sim_entrypoint.sh gazebo|mujoco|shell|help"
    echo ""
    echo "Normally started by the host command nepi_simulators_start gazebo|mujoco"
    echo "(nepi_simulators), which mounts NEPI storage and passes the display, GPU and"
    echo "NEPI_IP through to this container."
}

source /opt/nepi_simulations/nepi_sim/docker/container/nepi_sim_container_utils

case "$SIM" in
    gazebo)
        RESOURCES_FOLDER=$NEPI_GAZEBO_RESOURCES_FOLDER
        PID_KEY=GAZEBO_PID
        ;;
    mujoco)
        RESOURCES_FOLDER=$NEPI_MUJOCO_RESOURCES_FOLDER
        PID_KEY=MUJOCO_PID
        ;;
    shell)
        exec bash -l
        ;;
    help|"")
        usage
        exit 0
        ;;
    *)
        usage
        exit 2
        ;;
esac

START_SCRIPT=${RESOURCES_FOLDER}/config/nepi_${SIM}_start.sh
SERVICE_SCRIPT=${RESOURCES_FOLDER}/config/nepi_${SIM}.sh
SERVICE_LOG=/tmp/nepi_${SIM}_service.log
CONFIG_FILE=${SHARE}/databases/sims/${SIM}/config/nepi_${SIM}_config.yaml

echo "########################"
echo "NEPI simulator container: ${SIM}"
echo "########################"


####################################
# 1. Preconditions: the device address, and NEPI storage from the host.

if [[ -z "$NEPI_IP" ]]; then
    echo "ERROR: NEPI_IP is not set. Start this container with nepi_simulators_start ${SIM} on the host,"
    echo "ERROR: or pass -e NEPI_IP=<device address>."
    exit 1
fi
echo "NEPI device: ${NEPI_IP} (share user ${NEPI_USER})"

if ! timeout 2 mountpoint -q "$SHARE"; then
    echo "ERROR: NEPI storage is not mounted at ${SHARE} in this container."
    echo "ERROR: Mount it on the host (nepi_simulators_mount) and start the simulator again."
    exit 1
fi
if [[ ! -w "$SHARE" ]]; then
    echo "WARNING: ${SHARE} is not writable by $(id -un) (uid $(id -u)) -- the device will"
    echo "WARNING: not see this simulator's state. Check the host mount's uid/gid options."
fi


####################################
# 2. Forget a process ID recorded by an earlier container. This container is
# new, so nothing from before it can still be running, but its PID numbers
# are reused from 1 up: a stale GAZEBO_PID can belong to an unrelated process
# here, and the Gazebo service adopts any recorded PID that is alive. Written
# through a temp file and rename, never yq -i (see update_yaml_value).

if [[ -f "$CONFIG_FILE" ]]; then
    tmp_file="${CONFIG_FILE}.svc.tmp"
    if yq e ".${PID_KEY} = 0" "$CONFIG_FILE" > "$tmp_file" && [[ -s "$tmp_file" ]]; then
        mv -f "$tmp_file" "$CONFIG_FILE"
    else
        rm -f "$tmp_file"
        echo "WARNING: could not reset ${PID_KEY} in ${CONFIG_FILE}"
    fi
fi


####################################
# 3. Display and rendering. Report what the container actually got, because
# a silent fallback to software rendering looks like a slow or broken
# simulator rather than a configuration problem.

if [[ "$SIM" == "gazebo" && -z "$DISPLAY" ]]; then
    echo "ERROR: the Gazebo service runs Gazebo with its GUI, which needs a display."
    echo "ERROR: Start it from a desktop session on the host, or use MuJoCo, which runs headless."
    exit 1
fi

if [[ -n "$DISPLAY" ]]; then
    renderer=$(timeout 10 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
    if [[ -z "$renderer" ]]; then
        echo "WARNING: cannot open display ${DISPLAY} from the container. Windows will not appear."
        echo "WARNING: Check that xauth is installed on the host and that DISPLAY is set there."
    else
        echo "OpenGL renderer: ${renderer}"
        case "${renderer,,}" in
            *llvmpipe*|*softpipe*|*software*)
                echo "WARNING: this is software rendering. The simulation will run, but slowly,"
                echo "WARNING: and camera images will lag. See the GPU notes in the setup guide."
                ;;
        esac
    fi
else
    echo "No display: running headless"
fi

if [[ "$SIM" == "mujoco" ]]; then
    if ! timeout 30 python3 -c "import os; os.environ['MUJOCO_GL'] = 'egl'; import mujoco; \
mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'), 64, 64).close()" >/dev/null 2>&1; then
        echo "WARNING: MuJoCo cannot render offscreen through EGL in this container, so there"
        echo "WARNING: will be no camera images. The host GPU is probably not passed through."
    fi
fi


####################################
# 4. Start the service, with the same start script a native machine runs.
# There is no share password to pass: the share is already mounted.

if ! bash "$START_SCRIPT"; then
    echo "ERROR: ${START_SCRIPT} failed"
    [[ -f "$SERVICE_LOG" ]] && tail -n 40 "$SERVICE_LOG"
    [[ "$SIM" == "gazebo" && -f /tmp/nepi_gazebo.log ]] && tail -n 40 /tmp/nepi_gazebo.log
    exit 1
fi

touch "$SERVICE_LOG"
tail -n +1 -F "$SERVICE_LOG" 2>/dev/null &
tail_pid=$!


####################################
# 5. Stay up while the service runs; stop it on docker stop.

# shellcheck disable=SC2329  # invoked through the trap below
function stop_container(){
    trap '' TERM INT
    echo "Container stop requested"
    "nepi_${SIM}_stop"
    kill "$tail_pid" 2>/dev/null
    exit 0
}
trap stop_container TERM INT

# A backgrounded sleep plus wait, rather than a plain sleep, so a TERM runs the
# trap straight away instead of after the sleep ends.
while pgrep -f "bash ${SERVICE_SCRIPT}" >/dev/null 2>&1; do
    sleep 2 &
    wait $!
done

echo "ERROR: the nepi_${SIM} service exited on its own -- stopping the container"
kill "$tail_pid" 2>/dev/null
exit 1
