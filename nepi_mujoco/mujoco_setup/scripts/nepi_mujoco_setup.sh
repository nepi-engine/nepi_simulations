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

# Installs MuJoCo for the NEPI Sim Connector on a sim machine, stages this
# repo's robot and world files in ${HOME}/mujoco, and adds the
# nepi_mujoco_start command to the user's shell.
#
# Run nepi_gazebo_setup.sh first on a new machine: it installs yq and sets up
# the NEPI bash utils (sync_yaml_files, NEPI_IP etc.) that the MuJoCo service
# also uses.

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname ${SCRIPT_FOLDER})/resources

# The license and user checks are shared with the Gazebo setup in this repo.
GAZEBO_SCRIPTS_FOLDER=$(cd -P "${SCRIPT_FOLDER}/../../../nepi_gazebo/gazebo_setup/scripts" 2>/dev/null && pwd)
if [[ -z "$GAZEBO_SCRIPTS_FOLDER" ]]; then
    echo "nepi_gazebo/gazebo_setup/scripts not found next to this repo's nepi_mujoco folder"
    return 1 2>/dev/null || exit 1
fi
source ${GAZEBO_SCRIPTS_FOLDER}/nepi_license_check.sh
if [[ "$?" -ne 0 ]]; then
    return 1 2>/dev/null || exit 1
fi
source ${GAZEBO_SCRIPTS_FOLDER}/nepi_user_check.sh
if [[ "$?" -ne 0 ]]; then
    return 1 2>/dev/null || exit 1
fi

echo ""
echo "########################"
echo "NEPI MUJOCO SETUP"
echo "########################"
echo ""


####################################
# 1. Install MuJoCo. Pinned to 3.2.3: the newest release for Ubuntu 20.04's
# Python 3.8, and the MjSpec API nepi_mujoco_sim.py is written against
# (later releases changed how specs are loaded). Installed per user, so it
# adds packages without touching the system's numpy or ROS Python.

MUJOCO_VERSION=3.2.3

echo "Installing mujoco==${MUJOCO_VERSION}"
python3 -m pip install --user mujoco==${MUJOCO_VERSION}
if ! python3 -c "import mujoco" >/dev/null 2>&1; then
    echo "ERROR: mujoco did not install -- see the pip output above"
    return 1 2>/dev/null || exit 1
fi
echo "mujoco $(python3 -c 'import mujoco; print(mujoco.__version__)') installed"

if ! command -v yq >/dev/null 2>&1; then
    echo "WARNING: yq is not installed. The MuJoCo service needs mikefarah's yq v4,"
    echo "WARNING: which nepi_gazebo_setup.sh installs."
fi

# Camera images: rendered offscreen through EGL and encoded with OpenCV. The
# simulation still runs without them, just with no images for the device.
if ! python3 -c "import cv2" >/dev/null 2>&1; then
    echo "WARNING: OpenCV for Python 3 is not installed, so there will be no camera images."
    echo "WARNING: Install it with: sudo apt install python3-opencv"
fi
if ! python3 -c "import os; os.environ['MUJOCO_GL'] = 'egl'; import mujoco; \
mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'), 64, 64).close()" >/dev/null 2>&1; then
    echo "WARNING: MuJoCo cannot render offscreen through EGL on this machine, so there"
    echo "WARNING: will be no camera images. Check the graphics driver (libEGL)."
fi


####################################
# 2. Stage this repo's config, world and robot folders in ${HOME}/mujoco,
# where the service reads them. nepi_mujoco_sync.sh keeps that copy and NEPI
# storage in step every time the service is started.

MUJOCO_HOME_FOLDER=${HOME}/mujoco

echo ""
echo "Staging MuJoCo simulation resources in ${MUJOCO_HOME_FOLDER}"
for folder in config ENVIRONMENT_CONFIGS ROBOT_CONFIGS; do
    echo "Copying ${RESOURCES_FOLDER}/${folder} to ${MUJOCO_HOME_FOLDER}/${folder}"
    mkdir -p ${MUJOCO_HOME_FOLDER}/${folder}
    rsync -ar --delete ${RESOURCES_FOLDER}/${folder}/ ${MUJOCO_HOME_FOLDER}/${folder}/
done


####################################
# 3. Add nepi_mujoco_start to every new shell: a copy of the bash utils in the
# home folder, pinned to this checkout, sourced from .bashrc.

NEPI_UTILS_FILE_DEST=${HOME}/.nepi_mujoco_bash_utils
cp ${RESOURCES_FOLDER}/bash/nepi_mujoco_bash_utils ${NEPI_UTILS_FILE_DEST}
echo "export NEPI_MUJOCO_RESOURCES_FOLDER='${RESOURCES_FOLDER}'" >> ${NEPI_UTILS_FILE_DEST}
chmod 664 ${NEPI_UTILS_FILE_DEST}

BASHRC=${HOME}/.bashrc
# -x: a commented-out block must not count as already installed.
if ! grep -qxF "##### Source NEPI MuJoCo Bash Utils #####" "$BASHRC"; then
    {
        echo ' '
        echo '##### Source NEPI MuJoCo Bash Utils #####'
        echo 'NEPI_MUJOCO_UTILS_FILE='${NEPI_UTILS_FILE_DEST}
        echo 'if [ -f ${NEPI_MUJOCO_UTILS_FILE} ]; then'
        echo '    . ${NEPI_MUJOCO_UTILS_FILE}'
        echo 'fi'
    } >> "$BASHRC"
fi
source ${NEPI_UTILS_FILE_DEST}


echo ""
echo "##################################"
echo "NEPI MuJoCo Setup Complete"
echo "##################################"
echo ""
echo "Start the MuJoCo service (open a new terminal first, or source ~/.bashrc):"
echo "    nepi_mujoco_start <nepi storage password>"
echo ""
echo "Then pick MuJoCo as the Simulator in the device's Sim Connector and Deploy."
echo ""
