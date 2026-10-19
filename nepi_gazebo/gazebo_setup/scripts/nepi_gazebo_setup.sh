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

sudo -v


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
    return
fi


SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
USER_CHECK_FILE=${SCRIPT_FOLDER}/nepi_user_check.sh
source $USER_CHECK_FILE
if [[ "$?" -ne 0 ]]; then
    return
fi


sudo apt-get install iputils-ping -y
wait


if ! is_valid_internet; then
    echo "No Internet Connection Detected.  Connect and rerun this script"

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
    wget https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -O ${HOME}/yq
    chmod +x ${HOME}/yq
    sudo mv ${HOME}/yq /usr/bin/yq


    ####################################
    # 2. Clone ArduPilot and resolve the python/pip permission conflict

    echo ""
    echo "########"
    echo "Cloning ArduPilot"
    echo "########"

    sudo apt update
    sudo apt install -y git

    if [[ ! -d $ARDUPILOT_FOLDER ]]; then
        cd ${HOME}
        git clone https://github.com/ArduPilot/ardupilot.git
    fi
    cd $ARDUPILOT_FOLDER

    # Stable Copter-4.5 branch supports Ubuntu 20.04's native Python 3.8.10
    git checkout Copter-4.5
    git submodule update --init --recursive

    # A previous sudo pip install can leave ~/.local root-owned, which makes
    # install-prereqs-ubuntu.sh fail partway through with a permission error
    # and complain about missing launchpadlib deps -- fix both up front so
    # the installer only has to run once.
    sudo chown -R $(whoami):$(whoami) ${HOME}/.local
    sudo apt install -y python3-testresources

    echo 'export PATH=$HOME/.local/bin:$PATH' >> ${HOME}/.bashrc
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
        echo "WARNING: install-prereqs-ubuntu.sh exited ${prereqs_rc}"
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
        python3 -m pip install --user ${missing_py_pkgs}
    fi


    ####################################
    # 3. Add autotest tools to PATH so sim_vehicle.py runs anywhere

    # Append for future shells, AND export directly for this one -- sourcing
    # .bashrc here does nothing, since Ubuntu's stock .bashrc returns
    # immediately when $- has no 'i' (which is the case for this script).
    # Step 4 below calls sim_vehicle.py from this same shell, so without the
    # direct export it exits 127 and the EEPROM/parameter init silently
    # never happens.
    echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ${HOME}/.bashrc
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

    cd ${ARDUPILOT_FOLDER}
    ./waf configure --board sitl && ./waf copter

    if [[ ! -f $ARDUCOPTER_BIN ]]; then
        echo "WARNING: the ArduCopter SITL binary was not built"
        echo "WARNING: re-run './waf configure --board sitl && ./waf copter' from ${ARDUPILOT_FOLDER}"
    else
        # eeprom.bin is written to the cwd, which is where sim_vehicle.py
        # expects it when later run from ArduCopter/
        cd ${ARDUPILOT_FOLDER}/ArduCopter
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
            echo "WARNING: SITL parameter initialization did not complete"
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
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    sudo apt update
    sudo apt install -y gazebo11 libgazebo11-dev


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
        wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -
        sudo apt update
        sudo apt install -y ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs python3-opencv python3-numpy
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
        cd ${HOME}
        git clone https://github.com/khancyr/ardupilot_gazebo.git
    fi
    cd $ARDUPILOT_GAZEBO_FOLDER

    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install

    echo 'source /usr/share/gazebo/setup.sh' >> ${HOME}/.bashrc
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ardupilot_gazebo/models' >> ${HOME}/.bashrc
    echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ardupilot_gazebo/worlds' >> ${HOME}/.bashrc

    # NEPI's default storage path crashes Boost when used as the temp dir
    echo 'export TMPDIR=/tmp' >> ${HOME}/.bashrc

    # Same as step 3: the .bashrc appends above only reach future shells, so
    # export directly too for the rest of this run.
    source /usr/share/gazebo/setup.sh
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

    for folder in config ENVIRONMENT_CONFIGS ROBOT_CONFIGS; do
        echo "Copying ${RESOURCES_FOLDER}/${folder} to ${GAZEBO_HOME_FOLDER}/${folder}"
        mkdir -p ${GAZEBO_HOME_FOLDER}/${folder}
        rsync -ar --delete ${RESOURCES_FOLDER}/${folder}/ ${GAZEBO_HOME_FOLDER}/${folder}/
    done

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
        cp ${IRIS_MODEL_FOLDER}/model.config ${IRIS_MODEL_FOLDER}/model.sdf ${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS/quadcopter/
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
        echo "WARNING: ${IRIS_MODEL_FOLDER} not found -- the quadcopter robot config will not be selectable"
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

    BASH_SETUP_FILE=${NEPI_GAZEBO_SCRIPTS_DIR}/nepi_gazebo_bash_setup.sh
    if [[ ! -f "$BASH_SETUP_FILE" ]]; then
        echo "ERROR: bash setup script not found: ${BASH_SETUP_FILE}"
        echo "ERROR: NEPI Gazebo bash environment was NOT configured"
    else
        source $BASH_SETUP_FILE
    fi


    echo ""
    echo "##################################"
    echo "NEPI Gazebo Setup Complete"
    echo "##################################"
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
