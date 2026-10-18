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
## Redistributions in source code must retain this top-level comment block,
## Along with any License Check related code and checks.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# This script installs ArduPilot SITL, Gazebo 11 (Gazebo Classic), and the
# ArduPilot-Gazebo bridge plugin for local drone simulation.
#
# Revision notes (hardening pass):
#   - license/user check failures now actually stop the script (a bare `return`
#     is a no-op when the script is run with `bash`)
#   - sudo credentials are kept alive for the whole run
#   - failures are collected and reported at the end instead of always
#     printing "Setup Complete"
#   - ~/.bashrc edits are idempotent, and duplicate lines left by earlier
#     runs are cleaned up rather than added to
#   - TMPDIR=/tmp and the GAZEBO_*/OGRE path sanitising live in
#     ~/.nepi_gazebo_env, sourced from the END of ~/.bashrc so nothing
#     sourced earlier can override them
#   - the install ends by actually starting gzserver and reporting whether
#     it stayed up
#   - ~/gazebo re-staging backs up anything it would delete or overwrite

sudo -v

# Keep the sudo timestamp fresh: the ArduPilot prereqs plus the SITL and plugin
# builds easily outlast the default 15 minute sudo timeout, which would
# otherwise re-prompt mid-run. Stops when this script exits.
( while true; do sudo -n true 2>/dev/null; sleep 60; kill -0 "$$" 2>/dev/null || exit 0; done ) &
SUDO_KEEPALIVE_PID=$!
# IMPORTANT: disown it. Scripts this one sources (e.g. nepi_gazebo_bash_setup.sh)
# end with a bare `wait`, which waits for EVERY background child of the shell --
# including this never-ending loop -- and hangs the installer forever at
# "Sourcing updated bash files". A disowned job is not waited on.
disown "$SUDO_KEEPALIVE_PID" 2>/dev/null

stop_sudo_keepalive() {
    [[ -n "$SUDO_KEEPALIVE_PID" ]] && kill "$SUDO_KEEPALIVE_PID" 2>/dev/null
    SUDO_KEEPALIVE_PID=""
}
trap stop_sudo_keepalive EXIT

# Problems are collected here and summarised at the end.
FAILURES=()
note_failure() {
    echo "WARNING: $1"
    FAILURES+=("$1")
}

# Abort immediately -- for failures later steps cannot sensibly continue past.
fatal() {
    echo "ERROR: $1"
    echo "ERROR: aborting NEPI Gazebo setup"
    if (( ${#FAILURES[@]} )); then
        echo "Earlier problems:"
        printf '  - %s\n' "${FAILURES[@]}"
    fi
    exit 1
}

# ---- apt health -----------------------------------------------------------
# Every apt install in this script is a no-op when dpkg is left in a broken
# state by something that ran before it, and apt says so only in the middle of
# a long wall of output, so the install appears to succeed while installing
# nothing. The usual cause on a ROS machine is the Ubuntu/ROS packaging clash
# over catkin_pkg, rospkg and rosdistro: python3-catkin-pkg-modules and
# python3-catkin-pkg (and the rospkg/rosdistro pairs) ship the same files and
# neither declares a conflict, so dpkg refuses the unpack and leaves every ROS
# package that depends on them unconfigured.

# Names of packages dpkg has unpacked but not configured, or otherwise left
# half-installed. Empty output means dpkg is in a good state.
apt_broken_packages() {
    dpkg -l 2>/dev/null | awk 'NR>5 && $1 !~ /^(ii|rc|un)$/ {print $2}'
}

apt_is_healthy() {
    [[ -z "$(apt_broken_packages)" ]] || return 1
    # A dry run surfaces unmet dependencies without changing anything.
    if sudo apt-get -s install -y iputils-ping 2>&1 | grep -qE 'Unmet dependencies|^E: '; then
        return 1
    fi
    return 0
}

# Try to put dpkg back in a good state. Safe to call when nothing is wrong.
repair_apt() {
    echo "Repairing the package system"
    sudo dpkg --configure -a 2>/dev/null

    # The three packages at the centre of the ROS/Ubuntu file clash. Installing
    # them with --force-overwrite is the accepted workaround: the files are the
    # same modules either way, and every ROS package waiting behind them can
    # then be configured. apt-get download works even while dpkg is broken,
    # which apt-get install does not.
    local debdir="${HOME}/.nepi_gazebo_debs"
    rm -rf "$debdir"; mkdir -p "$debdir"
    ( cd "$debdir" && apt-get download \
        python3-catkin-pkg-modules python3-rospkg-modules python3-rosdistro-modules \
        >/dev/null 2>&1 )
    if compgen -G "${debdir}/*.deb" >/dev/null; then
        sudo dpkg -i --force-overwrite "${debdir}"/*.deb 2>/dev/null
    fi
    rm -rf "$debdir"

    sudo dpkg --configure -a 2>/dev/null
    sudo apt-get -y --fix-broken install 2>/dev/null
}

# Append a line to ~/.bashrc only if that exact line is not already there.
add_to_bashrc() {
    grep -qxF -- "$1" "${HOME}/.bashrc" 2>/dev/null || echo "$1" >> "${HOME}/.bashrc"
}

# Remove EVERY copy of a line from ~/.bashrc. Used to clear duplicates that
# earlier versions of this script appended on each run, and to clear legacy
# forms of a line before the current form is written.
remove_from_bashrc() {
    touch "${HOME}/.bashrc"
    grep -vxF -- "$1" "${HOME}/.bashrc" > "${HOME}/.bashrc.tmp.$$" ; true
    mv "${HOME}/.bashrc.tmp.$$" "${HOME}/.bashrc"
}

# Make a line the LAST thing in ~/.bashrc (removing any earlier copies), so it
# wins over anything sourced above it.
set_last_in_bashrc() {
    remove_from_bashrc "$1"
    echo "$1" >> "${HOME}/.bashrc"
}

# Write a block of lines exactly once, in order, at the end of ~/.bashrc:
# every existing copy of each line is removed first. Appending with
# add_to_bashrc cannot do this, because order matters here (gazebo's own
# setup.sh has to be sourced before the GAZEBO_* paths are extended) and
# because runs of earlier versions of this script may have left the lines
# scattered through the file in the wrong order.
rewrite_bashrc_block() {
    local line
    for line in "$@"; do
        remove_from_bashrc "$line"
    done
    for line in "$@"; do
        echo "$line" >> "${HOME}/.bashrc"
    done
}


# nepi_gazebo_bash_utils is sourced before the license/user checks below.
# nepi_license_check.sh calls ask_yes_no, which comes from nepi_setup's NEPI
# bash utils (sourced by .bashrc), not from nepi_gazebo_bash_utils.
SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname ${SCRIPT_FOLDER})/resources

# Pinned copy of the above, resolved here while the cwd is still the one the
# script was invoked from. BASH_SOURCE[0] is relative when the script is run
# as ./nepi_gazebo_setup.sh, so re-resolving it later -- after the cd's in
# steps 2/5/6 -- yields whatever directory we happen to be in instead of this
# one. Step 8 below needs the real path, so it uses this rather than
# recomputing.
NEPI_GAZEBO_SCRIPTS_DIR=${SCRIPT_FOLDER}

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
source $NEPI_UTILS_SOURCE


SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
LICENSE_CHECK_FILE=${SCRIPT_FOLDER}/nepi_license_check.sh
source $LICENSE_CHECK_FILE
if [[ "$?" -ne 0 ]]; then
    return 1 2>/dev/null || exit 1
fi


SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
USER_CHECK_FILE=${SCRIPT_FOLDER}/nepi_user_check.sh
source $USER_CHECK_FILE
if [[ "$?" -ne 0 ]]; then
    return 1 2>/dev/null || exit 1
fi


sudo apt-get install iputils-ping -y || note_failure "apt install iputils-ping"


if ! is_valid_internet; then
    echo "No Internet Connection Detected.  Connect and rerun this script"
    return 1 2>/dev/null || exit 1

else


    ######################################

    echo ""
    echo "########################"
    echo "NEPI GAZEBO SETUP"
    echo "########################"
    echo ""

    ARDUPILOT_FOLDER=${HOME}/ardupilot
    ARDUPILOT_GAZEBO_FOLDER=${HOME}/ardupilot_gazebo


    ####################################
    # 0. Make sure apt works before anything depends on it.

    echo ""
    echo "########"
    echo "Checking the package system"
    echo "########"

    if apt_is_healthy; then
        echo "Package system is healthy"
    else
        echo "dpkg is in a broken state -- nothing can be installed until it is fixed."
        echo "Packages left half-installed:"
        apt_broken_packages | sed 's/^/  /'
        repair_apt
        if apt_is_healthy; then
            echo "Package system repaired"
        else
            echo ""
            echo "Still broken after the repair attempt. Remaining packages:"
            apt_broken_packages | sed 's/^/  /'
            echo ""
            echo "Fix this by hand before re-running, starting with:"
            echo "  sudo dpkg --configure -a"
            echo "  sudo apt --fix-broken install"
            fatal "the package system is broken, so no install step here can succeed"
        fi
    fi


    ####################################
    # 1. Install yq (mikefarah/yq, the Go binary -- NOT the apt "yq" package,
    # which is a different, incompatible python-yq tool). update_yaml_value
    # in nepi_gazebo_bash_utils and nepi_gazebo.sh's own direct calls both
    # need this exact yq for the `env(VAR)` eval syntax they use.

    echo ""
    echo "########"
    echo "Installing yq"
    echo "########"

    sudo apt remove yq -y 2>/dev/null
    YQ_VERSION=v4.16.2
    YQ_ARCH=$(dpkg --print-architecture)
    wget https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -O ${HOME}/yq \
        && chmod +x ${HOME}/yq \
        && sudo mv ${HOME}/yq /usr/bin/yq \
        || note_failure "yq install failed -- nepi_gazebo needs the mikefarah yq binary"


    ####################################
    # 2. Clone ArduPilot and resolve the python/pip permission conflict

    echo ""
    echo "########"
    echo "Cloning ArduPilot"
    echo "########"

    sudo apt update || note_failure "apt update (ArduPilot step)"
    sudo apt install -y git || note_failure "apt install git"

    if [[ ! -d $ARDUPILOT_FOLDER ]]; then
        cd ${HOME} || fatal "cannot cd to ${HOME}"
        git clone https://github.com/ArduPilot/ardupilot.git \
            || fatal "git clone of ArduPilot failed"
    fi
    cd $ARDUPILOT_FOLDER || fatal "ArduPilot folder not found: ${ARDUPILOT_FOLDER}"

    # Stable Copter-4.5 branch supports Ubuntu 20.04's native Python 3.8.10
    git checkout Copter-4.5 \
        || note_failure "git checkout Copter-4.5 failed (local changes in ${ARDUPILOT_FOLDER}?) -- building whatever branch is checked out"
    git submodule update --init --recursive \
        || note_failure "ArduPilot submodule update failed"

    # A previous sudo pip install can leave ~/.local root-owned, which makes
    # install-prereqs-ubuntu.sh fail partway through with a permission error
    # and complain about missing launchpadlib deps -- fix both up front so
    # the installer only has to run once.
    sudo chown -R $(whoami):$(whoami) ${HOME}/.local
    sudo apt install -y python3-testresources || note_failure "apt install python3-testresources"

    set_last_in_bashrc 'export PATH=$HOME/.local/bin:$PATH'
    export PATH=${HOME}/.local/bin:$PATH

    Tools/environment_install/install-prereqs-ubuntu.sh -y
    prereqs_rc=$?

    # Reload profile to register build path changes
    if [[ -f ${HOME}/.profile ]]; then
        . ${HOME}/.profile
    fi

    # The prereqs script installs its apt packages and its pip packages in
    # separate phases, and a failure in the pip phase does not stop the
    # script or show up in its exit code -- which silently leaves the SITL
    # build unable to run ("you need to install empy with ..." from waf,
    # much later and far from the real cause). Verify the python modules the
    # build and sim_vehicle.py actually need, and repair rather than guess.
    if [[ $prereqs_rc -ne 0 ]]; then
        note_failure "install-prereqs-ubuntu.sh exited ${prereqs_rc}"
    fi

    missing_py_pkgs=""
    # module:pip-name -- the import name differs from the package name for
    # empy (em) and MAVProxy (MAVProxy is importable under its own name).
    for pair in em:empy==3.3.4 pymavlink:pymavlink MAVProxy:MAVProxy \
                serial:pyserial future:future lxml:lxml; do
        if ! python3 -c "import ${pair%%:*}" >/dev/null 2>&1; then
            missing_py_pkgs="${missing_py_pkgs} ${pair#*:}"
        fi
    done

    if [[ -n "$missing_py_pkgs" ]]; then
        echo "Python prerequisites missing after install-prereqs-ubuntu.sh:${missing_py_pkgs}"
        echo "Installing them directly"
        python3 -m pip install --user ${missing_py_pkgs} \
            || note_failure "pip install of:${missing_py_pkgs} failed -- the SITL build/sim_vehicle.py may not run"
    fi


    ####################################
    # 3. Add autotest tools to PATH so sim_vehicle.py runs anywhere

    # Append for future shells, AND export directly for this one -- sourcing
    # .bashrc here does nothing, since Ubuntu's stock .bashrc returns
    # immediately when $- has no 'i' (which is the case for this script).
    # Step 4 below calls sim_vehicle.py from this same shell, so without the
    # direct export it exits 127 and the EEPROM/parameter init silently
    # never happens.
    set_last_in_bashrc 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest'
    export PATH=$PATH:${ARDUPILOT_FOLDER}/Tools/autotest


    ####################################
    # 4. Build ArduCopter SITL and initialize vehicle EEPROM
    #
    # This used to be `timeout 900 sim_vehicle.py -w`, which hung the
    # installer: sim_vehicle.py launches the full interactive sim (an xterm
    # running arducopter plus a MAVProxy console on this terminal), and
    # timeout runs its child in a background process group, so the moment
    # MAVProxy touched the tty it was stopped by SIGTTIN/SIGTTOU -- leaving
    # the installer parked on "Waiting for heartbeat" for the full 15
    # minutes, with an xterm -hold window left behind even after. Most
    # visible on machines where ArduPilot was already built, since the build
    # then takes ~1s and the hang starts immediately.
    #
    # Instead: build with waf directly (runs to completion, no timeout to cut
    # it short), then run the SITL binary headless with -w just long enough
    # to see a heartbeat -- by then the wiped EEPROM has been rewritten with
    # the copter defaults -- and stop it.

    echo ""
    echo "########"
    echo "Building ArduCopter SITL and initializing parameters"
    echo "########"

    ARDUCOPTER_BIN=${ARDUPILOT_FOLDER}/build/sitl/bin/arducopter

    cd ${ARDUPILOT_FOLDER} || fatal "ArduPilot folder not found: ${ARDUPILOT_FOLDER}"
    # Judge the build by waf's exit status, not just by whether a binary exists
    # -- a stale binary from an earlier run would otherwise hide a failed build.
    waf_ok=0
    if ./waf configure --board sitl && ./waf copter; then
        waf_ok=1
    fi

    if [[ $waf_ok -ne 1 || ! -f $ARDUCOPTER_BIN ]]; then
        note_failure "the ArduCopter SITL build failed"
        echo "WARNING: re-run './waf configure --board sitl && ./waf copter' from ${ARDUPILOT_FOLDER}"
    else
        # eeprom.bin is written to the cwd, which is where sim_vehicle.py
        # expects it when later run from ArduCopter/
        cd ${ARDUPILOT_FOLDER}/ArduCopter || fatal "ArduCopter folder not found"
        rm -f eeprom.bin

        ${ARDUCOPTER_BIN} -S -w --model + --speedup 1 \
            --defaults ${ARDUPILOT_FOLDER}/Tools/autotest/default_params/copter.parm \
            --sim-address=127.0.0.1 -I0 </dev/null >/dev/null 2>&1 &
        sitl_pid=$!

        python3 - <<'EOF'
import sys, time
from pymavlink import mavutil
deadline = time.time() + 60
while True:
    try:
        conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760', autoreconnect=False)
        break
    except Exception:
        if time.time() > deadline:
            sys.exit(1)
        time.sleep(1)
sys.exit(0 if conn.wait_heartbeat(timeout=60) else 1)
EOF
        heartbeat_rc=$?

        kill $sitl_pid 2>/dev/null
        wait $sitl_pid 2>/dev/null

        if [[ $heartbeat_rc -eq 0 && -f eeprom.bin ]]; then
            echo "SITL parameter initialization complete"
        else
            note_failure "SITL parameter initialization did not complete"
            echo "WARNING: re-run 'sim_vehicle.py -w' from ${ARDUPILOT_FOLDER}/ArduCopter"
            echo "WARNING:   (once it reaches 'Waiting for heartbeat' / params load, Ctrl+C)"
        fi
    fi


    ####################################
    # 5. Install Gazebo 11 (Gazebo Classic)

    echo ""
    echo "########"
    echo "Installing Gazebo 11"
    echo "########"

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
        || note_failure "adding the Gazebo apt key failed"

    sudo apt update || note_failure "apt update (Gazebo step)"
    if ! sudo apt install -y gazebo11 libgazebo11-dev; then
        repair_apt
        sudo apt install -y gazebo11 libgazebo11-dev \
            || note_failure "apt install gazebo11 libgazebo11-dev failed -- the plugin build below will fail"
    fi


    ####################################
    # 5b. Install ROS Noetic with its Gazebo plugins
    #
    # The ROBOT_CONFIGS rover drives and sees through ROS Gazebo plugins
    # (libgazebo_ros_diff_drive, libgazebo_ros_openni_kinect), and
    # nepi_gazebo_bridge.py -- which connects it to its NEPI driver on the
    # device -- is a ROS node. Without ROS, Gazebo still runs but a rover
    # cannot be driven. nepi_gazebo_ros_setup (nepi_gazebo_bash_utils) is what
    # uses this at launch. Noetic is only packaged for Ubuntu 20.04 (focal).

    echo ""
    echo "########"
    echo "Installing ROS Noetic and the Gazebo ROS plugins"
    echo "########"

    if [[ "$(lsb_release -cs)" == "focal" ]]; then
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add - \
            || note_failure "adding the ROS apt key failed"
        sudo apt update || note_failure "apt update (ROS step)"
        # The catkin_pkg/rospkg/rosdistro file clash fires here on a fresh
        # machine, so a failure gets one repair-and-retry before it is
        # reported. Leaving it unrepaired would also break every apt call
        # after this point, not just ROS.
        if ! sudo apt install -y ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs python3-opencv python3-numpy; then
            repair_apt
            sudo apt install -y ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs python3-opencv python3-numpy \
                || note_failure "ROS Noetic install failed -- rover robots cannot be driven"
        fi
        if ! apt_is_healthy; then
            note_failure "dpkg is still in a broken state after the ROS install -- run 'sudo dpkg --configure -a' and re-run this script"
        fi
    else
        echo "WARNING: ROS Noetic needs Ubuntu 20.04 (focal); this is $(lsb_release -cs). Gazebo will run, but rover robots cannot be driven."
    fi


    ####################################
    # 6. Install the ArduPilot-Gazebo bridge plugin

    echo ""
    echo "########"
    echo "Installing the ArduPilot-Gazebo bridge plugin"
    echo "########"

    if [[ ! -d $ARDUPILOT_GAZEBO_FOLDER ]]; then
        cd ${HOME} || fatal "cannot cd to ${HOME}"
        git clone https://github.com/khancyr/ardupilot_gazebo.git \
            || fatal "git clone of ardupilot_gazebo failed"
    fi
    cd $ARDUPILOT_GAZEBO_FOLDER || fatal "ardupilot_gazebo folder not found: ${ARDUPILOT_GAZEBO_FOLDER}"

    mkdir -p build
    cd build || fatal "cannot enter ${ARDUPILOT_GAZEBO_FOLDER}/build"
    # Only install if the build actually succeeded.
    cmake .. && make -j$(nproc) && sudo make install \
        || note_failure "ardupilot_gazebo plugin build/install failed -- Gazebo will not be able to load the ArduPilot plugin"

    # Where Gazebo keeps the media it needs to start its render engine, in
    # particular the RTShaderLib shaders. gzserver aborts in
    # RTShaderSystem::Init() -- which looks like "terminate called after
    # throwing an instance of std::string", with no further explanation -- when
    # it cannot find them, and GAZEBO_RESOURCE_PATH REPLACES Gazebo's built-in
    # default rather than adding to it, so any value that leaves this directory
    # out breaks rendering even though the files are on disk. Found rather than
    # assumed, because the layout differs between Gazebo packages.
    GAZEBO_MEDIA_ROOT=""
    for _gz_dir in /usr/share/gazebo-11 /usr/share/gazebo /usr/local/share/gazebo-11; do
        if compgen -G "${_gz_dir}/media/rtshaderlib*" >/dev/null 2>&1; then
            GAZEBO_MEDIA_ROOT=$_gz_dir
            break
        fi
    done
    if [[ -z "$GAZEBO_MEDIA_ROOT" ]]; then
        _gz_found=$(find /usr/share /usr/local/share /opt -maxdepth 4 -type d -name 'rtshaderlib*' 2>/dev/null | head -1)
        if [[ -n "$_gz_found" ]]; then
            GAZEBO_MEDIA_ROOT=$(dirname "$(dirname "$_gz_found")")
        fi
    fi
    unset _gz_dir _gz_found

    if [[ -n "$GAZEBO_MEDIA_ROOT" ]]; then
        echo "Gazebo render media found in ${GAZEBO_MEDIA_ROOT}/media"
    else
        note_failure "Gazebo's RTShaderLib media could not be found -- gzserver will abort when it starts its render engine. Try: sudo apt install --reinstall gazebo11 gazebo11-common"
    fi

    # Clear the tilde forms earlier versions of this script wrote, so a
    # rerun does not leave both forms extending the same variable.
    remove_from_bashrc 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ardupilot_gazebo/models'
    remove_from_bashrc 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ardupilot_gazebo/worlds'

    # Written as one ordered block, with every previous copy removed first:
    # earlier versions appended these on every run, so a machine that ran the
    # installer several times ends up sourcing gazebo's setup.sh repeatedly
    # and extending GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH with the same
    # entries over and over. Step 9 below trims what is left at shell
    # startup; this keeps the file itself from growing in the first place.
    rewrite_bashrc_block \
        'source /usr/share/gazebo/setup.sh' \
        'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ardupilot_gazebo/models' \
        'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$HOME/ardupilot_gazebo/worlds'

    # NOTE: TMPDIR=/tmp and the path sanitising are persisted at the very end
    # of this script, after step 8, so nothing appended to ~/.bashrc later can
    # override them.

    # Same as step 3: the .bashrc appends above only reach future shells, so
    # export directly too for the rest of this run.
    if [[ -f /usr/share/gazebo/setup.sh ]]; then
        source /usr/share/gazebo/setup.sh
    else
        note_failure "/usr/share/gazebo/setup.sh not found -- is gazebo11 installed?"
    fi
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${ARDUPILOT_GAZEBO_FOLDER}/models
    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${ARDUPILOT_GAZEBO_FOLDER}/worlds
    export TMPDIR=/tmp


    ####################################
    # 7. Stage this repo's config/ENVIRONMENT_CONFIGS/ROBOT_CONFIGS folders under
    # ${HOME}/gazebo -- a plain, user-owned local copy (no sudo/chown
    # gymnastics needed, unlike the /mnt/nepi_storage seed this step used to
    # be -- see the note on step 8 below for why that moved elsewhere).
    # Keeps a copy of the simulation resources alongside ardupilot/
    # ardupilot_gazebo in the user's own home directory, independent of
    # wherever this nepi_gazebo repo checkout happens to live.

    echo ""
    echo "########"
    echo "Staging Gazebo simulation resources in ${HOME}/gazebo"
    echo "########"

    GAZEBO_HOME_FOLDER=${HOME}/gazebo

    # --delete keeps ~/gazebo an exact mirror of the repo, but would silently
    # remove any world/robot a user added by hand. --backup/--backup-dir moves
    # anything deleted or overwritten into a timestamped folder instead (rsync
    # only creates it if something actually needs backing up). The two quadcopter
    # model files are excluded: step 7 below regenerates them from ArduPilot's
    # iris_with_ardupilot on every run, so they must neither be mirrored from
    # the repo nor treated as stray files to delete/back up.
    GAZEBO_BACKUP_ROOT=${HOME}/gazebo_backup/$(date +%Y%m%d_%H%M%S)

    for folder in config ENVIRONMENT_CONFIGS ROBOT_CONFIGS; do
        echo "Copying ${RESOURCES_FOLDER}/${folder} to ${GAZEBO_HOME_FOLDER}/${folder}"
        mkdir -p ${GAZEBO_HOME_FOLDER}/${folder}
        rsync -ar --delete \
            --backup --backup-dir=${GAZEBO_BACKUP_ROOT}/${folder} \
            --exclude='quadcopter/model.sdf' --exclude='quadcopter/model.config' \
            ${RESOURCES_FOLDER}/${folder}/ ${GAZEBO_HOME_FOLDER}/${folder}/ \
            || note_failure "staging ${folder} into ${GAZEBO_HOME_FOLDER} failed"
    done

    if [[ -d ${GAZEBO_BACKUP_ROOT} ]]; then
        echo "NOTE: files replaced or removed while staging were saved to ${GAZEBO_BACKUP_ROOT}"
    fi

    # The quadcopter robot config's model files are ArduPilot's own
    # iris_with_ardupilot (ardupilot_gazebo, built above), copied in here
    # rather than kept in this repo: ardupilot_gazebo is GPL-3.0. Only the NEPI
    # capability file (ROBOT_CONFIGS/quadcopter/nepi_robot_config.yaml) lives
    # in this repo. Copied rather than symlinked as ROBOT_CONFIGS/README.md
    # suggests for upstream models, because the NEPI device lists robots from
    # its own storage copy of this folder, where a link into this machine's
    # home directory would not resolve. Runs after the rsync --delete above,
    # which would otherwise remove it again.
    IRIS_MODEL_FOLDER=${ARDUPILOT_GAZEBO_FOLDER}/models/iris_with_ardupilot
    if [[ -f ${IRIS_MODEL_FOLDER}/model.sdf ]]; then
        echo "Copying ${IRIS_MODEL_FOLDER} model files into ${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS/quadcopter"
        mkdir -p ${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS/quadcopter
        cp ${IRIS_MODEL_FOLDER}/model.config ${IRIS_MODEL_FOLDER}/model.sdf ${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS/quadcopter/ \
            || note_failure "copying the iris model into ROBOT_CONFIGS/quadcopter failed"
        # The service spawns every robot config under its folder name
        # (nepi_gazebo.sh's `gz model -m quadcopter`), not the SDF's own
        # <model name="iris_demo">, so the ArduPilotPlugin's IMU lookup by the
        # iris_demo-scoped name fails and the plugin aborts before it opens
        # its SITL ports (9002/9003) -- the quad spawns but can never fly. The
        # bare sensor name is what Gazebo's Model::SensorScopedName() takes,
        # and it matches whatever the model is named; iris_with_standoffs
        # carries the only imu_sensor. The rotor jointNames are already
        # model-relative and need nothing.
        sed -i 's|<imuName>[^<]*</imuName>|<imuName>imu_sensor</imuName>|' \
            ${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS/quadcopter/model.sdf
    else
        note_failure "${IRIS_MODEL_FOLDER} not found -- the quadcopter robot config will not be selectable"
    fi


    ####################################
    # 8. Persist this machine's bash environment (NEPI_MODE=REMOTE, the
    # nepi_gazebo_bash_utils functions above) into the user's own shell, so
    # every future shell has them without re-running this installer -- same
    # role remote_bash_setup.sh plays for remote_env_setup.sh in nepi_setup,
    # sourced here rather than duplicated inline for the same reason.
    #
    # Seeding NEPI storage itself (as opposed to the local ${HOME}/gazebo
    # copy in step 7 above) is deliberately NOT done here (a prior version
    # of this step rsync'd straight to
    # /mnt/nepi_storage/databases/sims/gazebo) -- in REMOTE mode that path
    # is never what actually gets read: nepistorage (see
    # nepi_gazebo_bash_utils) mounts the device's share at
    # /mnt/nepi_share_storage instead, and reaching it needs a password this
    # installer never collects. nepi_gazebo_sync.sh already does this
    # correctly (calls nepistorage with a password, then two-way rsyncs
    # against the mounted share) and nepi_gazebo_start.sh already runs it on
    # every start, so the device gets seeded the first time Gazebo starts,
    # not here.

    # The long sudo-heavy work is done; stop the keepalive before sourcing
    # anything else (belt and braces alongside the disown above).
    stop_sudo_keepalive

    BASH_SETUP_FILE=${NEPI_GAZEBO_SCRIPTS_DIR}/nepi_gazebo_bash_setup.sh
    if [[ ! -f "$BASH_SETUP_FILE" ]]; then
        echo "ERROR: bash setup script not found: ${BASH_SETUP_FILE}"
        echo "ERROR: NEPI Gazebo bash environment was NOT configured"
        FAILURES+=("NEPI Gazebo bash environment was not configured (${BASH_SETUP_FILE} missing)")
    else
        source $BASH_SETUP_FILE
    fi


    ####################################
    # 9. Last word on the shell environment Gazebo gets.
    #
    # Everything here is written into ~/.nepi_gazebo_env and sourced from the
    # LAST line of ~/.bashrc, so it runs after step 8's NEPI hooks and after
    # anything else in the file:
    #
    #   TMPDIR=/tmp -- NEPI's default storage path crashes Boost when used as
    #   the temp dir, and a NEPI hook may set TMPDIR to it.
    #
    #   Path sanitising -- gzserver reads GAZEBO_RESOURCE_PATH,
    #   GAZEBO_PLUGIN_PATH, GAZEBO_MODEL_PATH and OGRE_RESOURCE_PATH when it
    #   starts its render engine, and an empty element (from a leading or
    #   doubled ":"), a duplicate, or a directory that does not exist is a
    #   plausible cause of gzserver aborting there with a bare std::string
    #   exception. Repeated installer runs and repeated sourcing of gazebo's
    #   own setup.sh are how those entries accumulate. Trimming them costs
    #   nothing if they were not the problem.

    NEPI_GAZEBO_ENV_FILE=${HOME}/.nepi_gazebo_env

    cat > "$NEPI_GAZEBO_ENV_FILE" <<'NEPI_GAZEBO_ENV_EOF'
# NEPI Gazebo shell environment -- written by nepi_gazebo_setup.sh.
# Sourced from the last line of ~/.bashrc. Edits here are overwritten the
# next time the installer runs.

# Boost (and so Gazebo) aborts when the temp dir is NEPI's storage path.
export TMPDIR=/tmp

# Drop empty and duplicate entries from a colon-separated path variable, and
# (unless called with "keep-missing") entries whose directory does not exist.
_nepi_gz_clean_path() {
    local name=$1 keep=$2 entry out=""
    local IFS=:
    for entry in ${!name}; do
        [[ -z "$entry" ]] && continue
        if [[ "$keep" != "keep-missing" && ! -d "$entry" ]]; then
            continue
        fi
        case ":${out}:" in
            *":${entry}:"*) continue ;;
        esac
        out="${out:+${out}:}${entry}"
    done
    export "${name}=${out}"
}

for _nepi_gz_var in GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH GAZEBO_PLUGIN_PATH OGRE_RESOURCE_PATH; do
    _nepi_gz_clean_path "$_nepi_gz_var"
done
# LD_LIBRARY_PATH is only de-duplicated: a directory that does not exist yet
# is harmless there, and dropping it could break something else on the machine.
_nepi_gz_clean_path LD_LIBRARY_PATH keep-missing
unset _nepi_gz_var
NEPI_GAZEBO_ENV_EOF

    # Guarantee the render media directory is on GAZEBO_RESOURCE_PATH. This is
    # appended after the quoted block above so the discovered path is baked in
    # rather than re-resolved in every shell.
    if [[ -n "$GAZEBO_MEDIA_ROOT" ]]; then
        {
            echo ""
            echo "# Gazebo's render media (RTShaderLib). GAZEBO_RESOURCE_PATH replaces"
            echo "# Gazebo's built-in default instead of adding to it, so this has to be"
            echo "# present or gzserver aborts in RTShaderSystem::Init()."
            echo "case \":\${GAZEBO_RESOURCE_PATH}:\" in"
            echo "    *\":${GAZEBO_MEDIA_ROOT}:\"*) ;;"
            echo "    *) export GAZEBO_RESOURCE_PATH=\"${GAZEBO_MEDIA_ROOT}\${GAZEBO_RESOURCE_PATH:+:\${GAZEBO_RESOURCE_PATH}}\" ;;"
            echo "esac"
        } >> "$NEPI_GAZEBO_ENV_FILE"
    fi

    chmod 0644 "$NEPI_GAZEBO_ENV_FILE"

    # Clear the plain TMPDIR line earlier versions of this script wrote; the
    # env file above owns it now.
    remove_from_bashrc 'export TMPDIR=/tmp'
    set_last_in_bashrc 'if [ -f "$HOME/.nepi_gazebo_env" ]; then . "$HOME/.nepi_gazebo_env"; fi'

    # Apply it to this run too.
    source "$NEPI_GAZEBO_ENV_FILE"


    ####################################
    # 10. Check that Gazebo actually starts.
    #
    # Run the way the service runs it -- the real desktop session, DISPLAY
    # set, so the render engine is exercised -- because that is where it
    # fails when it fails. gzserver alone, no world of ours, no ROS: if this
    # cannot stay up for 15 seconds then nothing further in NEPI will work,
    # and it is far better to say so here than to have it surface later as
    # "Gazebo exited immediately after launch" from nepi_gazebo_start.

    echo ""
    echo "########"
    echo "Checking that Gazebo starts"
    echo "########"

    GAZEBO_CHECK_LOG=/tmp/nepi_gazebo_setup_check.log
    GAZEBO_CHECK_WORLD=/usr/share/gazebo-11/worlds/empty.world

    if [[ ! -f "$GAZEBO_CHECK_WORLD" ]]; then
        note_failure "Gazebo's own empty.world is missing -- gazebo11 is not properly installed"
    elif [[ -z "$DISPLAY" ]]; then
        echo "No DISPLAY set, so the render engine cannot be exercised -- skipping the check."
        echo "Run this from the desktop session to verify it:"
        echo "  gzserver --verbose ${GAZEBO_CHECK_WORLD}"
    else
        # A ~/.gazebo owned by root (left behind by a "sudo gazebo" at some
        # point) makes the render engine fail in the same place.
        if [[ -e "${HOME}/.gazebo" ]] && [[ ! -O "${HOME}/.gazebo" ]]; then
            echo "Fixing ownership of ${HOME}/.gazebo"
            sudo chown -R "$(id -un)":"$(id -gn)" "${HOME}/.gazebo"
        fi

        killall gzserver gzclient >/dev/null 2>&1
        sleep 2
        # timeout exits 124 when the command was still running at the end,
        # which is exactly the outcome wanted here.
        timeout 15 gzserver --verbose "$GAZEBO_CHECK_WORLD" >"$GAZEBO_CHECK_LOG" 2>&1
        gazebo_check_rc=$?
        killall gzserver >/dev/null 2>&1

        if [[ $gazebo_check_rc -eq 124 ]]; then
            echo "Gazebo started and stayed up"
        else
            note_failure "Gazebo could not start (exit ${gazebo_check_rc}) -- see ${GAZEBO_CHECK_LOG}"
            echo ""
            echo "Last lines of ${GAZEBO_CHECK_LOG}:"
            tail -5 "$GAZEBO_CHECK_LOG" 2>/dev/null
            echo ""
            echo "This is Gazebo failing on its own world, with nothing from NEPI involved,"
            echo "so it is a problem with this machine's Gazebo install or shell environment."
            if grep -q "terminate called after throwing an instance of 'std::__cxx11::basic_string" "$GAZEBO_CHECK_LOG" 2>/dev/null; then
                echo ""
                echo "That bare std::string abort is usually the render engine failing to find"
                echo "its media. Check, in this order:"
                echo "  ls -d /usr/share/gazebo-11/media/rtshaderlib*     # must exist"
                echo "  echo \$GAZEBO_RESOURCE_PATH                       # must contain its parent"
                echo "  ls -ld ~/.gazebo                                 # must be yours, not root's"
            fi
            echo ""
            echo "To find out exactly where it aborts:"
            echo "  sudo apt install -y gdb"
            echo "  gdb -batch -ex 'catch throw' -ex run -ex bt --args gzserver ${GAZEBO_CHECK_WORLD}"
        fi
    fi


    echo ""
    echo "##################################"
    if (( ${#FAILURES[@]} )); then
        echo "NEPI Gazebo Setup finished WITH PROBLEMS"
        echo "##################################"
        echo ""
        echo "These steps failed or were incomplete:"
        printf '  - %s\n' "${FAILURES[@]}"
        echo ""
        echo "Fix the above and re-run this script (it is safe to re-run)."
        echo "Open a NEW terminal (or run: source ~/.bashrc) before testing."
        echo ""
        return 1 2>/dev/null || exit 1
    fi
    echo "NEPI Gazebo Setup Complete"
    echo "##################################"
    echo ""
    echo "Open a NEW terminal (or run: source ~/.bashrc) so PATH, GAZEBO_* and"
    echo "TMPDIR are loaded."
    echo ""
    echo "To verify the install, run these in two separate terminals:"
    echo ""
    echo "  Terminal 1 (launch Gazebo with the Iris quadcopter world):"
    echo "    gazebo --verbose ${ARDUPILOT_GAZEBO_FOLDER}/worlds/iris_arducopter_runway.world"
    echo ""
    echo "  Terminal 2 (launch the SITL controller linked to the Gazebo model):"
    echo "    cd ${ARDUPILOT_FOLDER}/ArduCopter"
    echo "    sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
    echo ""

fi