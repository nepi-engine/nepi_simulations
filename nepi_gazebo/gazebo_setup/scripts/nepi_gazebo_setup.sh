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


# nepi_gazebo_bash_utils must be sourced before the license/user checks
# below -- nepi_license_check.sh calls ask_yes_no, which it defines.
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
    # 4. Initialize vehicle EEPROM
    # sim_vehicle.py -w writes the mock EEPROM tables and default parameter
    # profiles and then idles -- normally you'd Ctrl+C once that settles;
    # timeout does the same thing non-interactively.

    echo ""
    echo "########"
    echo "Initializing ArduCopter SITL parameters"
    echo "########"

    cd ${ARDUPILOT_FOLDER}/ArduCopter

    # On a fresh checkout this call has to COMPILE ArduCopter SITL before it
    # can write anything, which takes far longer than the 60s this step used
    # to allow -- the timeout killed waf mid-build every time, leaving no
    # binary and no EEPROM. Allow enough time for the build, and note that
    # the exit code alone can't confirm success: timeout returns 124 both
    # when it interrupts the expected post-init idle AND when it cuts the
    # build short. Check for the artifacts instead.
    timeout 900 sim_vehicle.py -w

    if [[ -f ${ARDUPILOT_FOLDER}/build/sitl/bin/arducopter ]] \
       && find ${ARDUPILOT_FOLDER}/ArduCopter -maxdepth 2 -name 'eeprom.bin' | grep -q .; then
        echo "SITL parameter initialization complete"
    else
        echo "WARNING: SITL parameter initialization did not complete"
        [[ -f ${ARDUPILOT_FOLDER}/build/sitl/bin/arducopter ]] \
            || echo "WARNING:   the ArduCopter SITL binary was not built"
        echo "WARNING: re-run 'sim_vehicle.py -w' from ${ARDUPILOT_FOLDER}/ArduCopter"
        echo "WARNING:   (let it finish building, then Ctrl+C once it settles)"
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
    # 7. Stage this repo's config/ENVIRONMENT/SYSTEM folders under
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

    for folder in config ENVIRONMENT SYSTEM; do
        echo "Copying ${RESOURCES_FOLDER}/${folder} to ${GAZEBO_HOME_FOLDER}/${folder}"
        mkdir -p ${GAZEBO_HOME_FOLDER}/${folder}
        rsync -ar --delete ${RESOURCES_FOLDER}/${folder}/ ${GAZEBO_HOME_FOLDER}/${folder}/
    done


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
