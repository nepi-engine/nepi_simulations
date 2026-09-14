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

# NEPI Gazebo bash/environment setup -- this file's role in gazebo_setup is
# the same one nepi_setup's own <mode>_bash_setup.sh scripts play there
# (nepi_bash_setup.sh for SYSTEM, remote_bash_setup.sh for REMOTE,
# docker_bash_setup.sh for HOST): persist this machine's NEPI_MODE and bash
# utils functions into the user's own shell (~/.nepi_gazebo_bash_utils +
# .bashrc), so every future shell has them without re-running the installer.
# nepi_gazebo_setup.sh -- this file's counterpart to nepi_docker_init.sh /
# remote_env_setup.sh, the script that actually installs ArduPilot SITL,
# Gazebo11, and the bridge plugin -- sources this file near the end of its
# own run, the same point remote_env_setup.sh sources remote_bash_setup.sh
# in nepi_setup.
#
# NEPI_MODE is hardcoded to REMOTE below, never SYSTEM/HOST -- unlike
# nepi_setup, gazebo_setup has exactly one machine role: a person's own
# separate simulation machine, reaching the NEPI device's storage over the
# network via nepistorage()'s REMOTE branch (see nepi_gazebo_bash_utils),
# never the NEPI device itself. So, unlike nepi_bash_setup.sh/
# remote_bash_setup.sh/docker_bash_setup.sh -- one per mode, dispatched by
# nepisetup() based on whatever NEPI_MODE the machine was already set up
# with -- there is only this one file here.

sudo -v

export NEPI_MODE='REMOTE'

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname ${SCRIPT_FOLDER})/resources

# nepi_gazebo_bash_utils must be sourced before the license/user checks
# below -- nepi_license_check.sh calls ask_yes_no, which it defines.
NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
source $NEPI_UTILS_SOURCE
export NEPI_MODE='REMOTE'


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


echo ""
echo "########################"
echo "NEPI GAZEBO BASH SETUP"
echo "########################"
echo ""

sudo chown ${CONFIG_USER}:${CONFIG_USER} /home/${CONFIG_USER}


####################################
# 1. Persist nepi_gazebo_bash_utils into the user's home dir
# (~/.nepi_gazebo_bash_utils) -- same copy-then-pin-NEPI_MODE pattern
# nepi_bash_setup.sh/remote_bash_setup.sh use for their own
# ~/.nepi_bash_utils. This copy, not the in-repo original, is what .bashrc
# below sources on every new shell.

echo "Setting up NEPI Gazebo Bash Utils file"

NEPI_UTILS_FILE_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
NEPI_UTILS_FILE_DEST=/home/${CONFIG_USER}/.nepi_gazebo_bash_utils

sudo chown ${CONFIG_USER}:${CONFIG_USER} $NEPI_UTILS_FILE_SOURCE
sudo chmod 775 $NEPI_UTILS_FILE_SOURCE
sudo cp -p $NEPI_UTILS_FILE_SOURCE $NEPI_UTILS_FILE_DEST

# The copied file's own NEPI_MODE default logic (SYSTEM, unless NEPI_MODE is
# already one of SYSTEM/HOST/REMOTE when it's sourced) is for a machine that
# has never run this script. Once it has, pin REMOTE permanently by
# appending an override export -- every later shell that sources this copy
# gets REMOTE without depending on whatever NEPI_MODE happened to already be
# exported when that shell started.
echo "export NEPI_MODE='REMOTE'" | sudo tee -a $NEPI_UTILS_FILE_DEST > /dev/null

# Also pin where this machine's nepi_gazebo repo checkout actually lives.
# BASH_FOLDER (computed inside nepi_gazebo_bash_utils itself) resolves to
# $HOME once THIS copy is what gets sourced from .bashrc, not
# resources/bash in the checkout -- nepi_gazebo_start() (defined in that
# file) needs this override to find nepi_gazebo_start.sh from an ordinary
# new shell.
echo "export NEPI_GAZEBO_RESOURCES_FOLDER='${RESOURCES_FOLDER}'" | sudo tee -a $NEPI_UTILS_FILE_DEST > /dev/null

sudo chown ${CONFIG_USER}:${CONFIG_USER} $NEPI_UTILS_FILE_DEST
sudo chmod 664 $NEPI_UTILS_FILE_DEST


####################################
# 2. Wire ${CONFIG_USER}'s .bashrc to source that copy on every new shell
# (idempotent, marker-comment-guarded -- same "##### Source NEPI Aliases
# #####" idiom nepi_bash_setup.sh/remote_bash_setup.sh use, adapted here
# since gazebo_setup has no separate aliases file of its own to point at).

echo "Updating ${CONFIG_USER} user .bashrc file"

BASHRC=/home/${CONFIG_USER}/.bashrc
file=$BASHRC

if [[ ! -f "$file" ]]; then
    cp /etc/skel/.bashrc $file
fi

sudo chown ${CONFIG_USER}:${CONFIG_USER} $file
sudo chmod 775 $file

if ! grep -qF "##### Source NEPI Gazebo Bash Utils #####" "$file"; then
    echo ' ' | sudo tee -a $file > /dev/null
    echo '##### Source NEPI Gazebo Bash Utils #####' | sudo tee -a $file > /dev/null
    echo 'NEPI_GAZEBO_UTILS_FILE='${NEPI_UTILS_FILE_DEST} | sudo tee -a $file > /dev/null
    echo 'if [ -f ${NEPI_GAZEBO_UTILS_FILE} ]; then' | sudo tee -a $file > /dev/null
    echo '    . ${NEPI_GAZEBO_UTILS_FILE}' | sudo tee -a $file > /dev/null
    echo 'fi' | sudo tee -a $file > /dev/null
fi

sudo chown ${CONFIG_USER}:${CONFIG_USER} $file
sudo chmod 0664 $file

echo ""
echo "Sourcing updated bash files"
source $file
wait


echo ""
echo "################################# "
echo "NEPI GAZEBO BASH SETUP COMPLETE"
echo "################################# "
echo ""
