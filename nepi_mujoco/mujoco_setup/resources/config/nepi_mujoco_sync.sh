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

sudo -v

NEPISTORAGE_PASSWORD=$1

if [[ ! -n $CONFIG_USER ]]; then
    CONFIG_USER=$(id -un)
    if [[ ${CONFIG_USER} == 'root' ]]; then
        CONFIG_USER=$SUDO_USER
    fi
fi
if [[ ! -n $CONFIG_USER ]]; then
    if [[ -d "/home/nepihost" ]]; then
        CONFIG_USER=nepihost
    else
        CONFIG_USER=$(id -nu 1000)
    fi
fi
export CONFIG_USER=$CONFIG_USER

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${SCRIPT_FOLDER}")

MUJOCO_HOME_FOLDER=${HOME}/mujoco
# mkdir -p ${MUJOCO_HOME_FOLDER}

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_mujoco_bash_utils
if [[ -f "$NEPI_UTILS_SOURCE" ]]; then
    source "$NEPI_UTILS_SOURCE"
else
    echo "NEPI MuJoCo Bash Utils file not found at: ${NEPI_UTILS_SOURCE}"
    exit 1
fi

# nepi_mujoco_mount_storage, not nepi_setup's nepistorage: it uses the
# password and reuses an existing mount (see nepi_mujoco_bash_utils).
ORIG_PWD=$(pwd)
if nepi_mujoco_mount_storage "$NEPISTORAGE_PASSWORD"; then
    MUJOCO_SIM_FOLDER=$(pwd)/databases/sims/mujoco
    cd "$ORIG_PWD"
else
    echo "Failed to reach nepi_storage"
    return 1 2>/dev/null || exit 1
fi
sudo mkdir -p ${MUJOCO_SIM_FOLDER}

##################
# Fix folder owners

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${MUJOCO_HOME_FOLDER}
sudo chmod 0775 ${MUJOCO_HOME_FOLDER}
sudo chown 1000:1000 ${MUJOCO_SIM_FOLDER}
sudo chmod 0750 ${MUJOCO_SIM_FOLDER}


#############################
echo ""
echo "Updating MuJoCo Simulation Files and Folders"

#############################
# yaml files

SOURCE_PATH=${MUJOCO_SIM_FOLDER}/config
UPDATE_PATH=${MUJOCO_HOME_FOLDER}/config
CONFIG_FILENAME=nepi_mujoco_config.yaml

SOURCE_FILE=${SOURCE_PATH}/${CONFIG_FILENAME}
UPDATE_FILE=${UPDATE_PATH}/${CONFIG_FILENAME}

echo "Merging ${CONFIG_FILENAME} keys from ${UPDATE_PATH} into ${SOURCE_PATH}"
# Create the storage-side folder first, the same way the ENVIRONMENT_CONFIGS
# and ROBOT_CONFIGS blocks below do. Without this, the very first run on a device that
# has no databases/sims/mujoco/config yet fails at the cp ("No such file or
# directory") -- and since the config file is what nepi_mujoco_start.sh and
# nepi_mujoco.sh both read, that first run never gets off the ground.
# mkdir -p on MUJOCO_SIM_FOLDER alone (above) does not cover this subfolder.
if [[ ! -d $SOURCE_PATH ]]; then
    sudo mkdir -p $SOURCE_PATH
fi
if [[ ! -f $SOURCE_FILE ]]; then
    sudo cp $UPDATE_FILE $SOURCE_FILE
fi
sync_yaml_files $UPDATE_FILE $SOURCE_FILE

#################################
echo "Syncing ${CONFIG_FILENAME} from ${SOURCE_PATH} to ${UPDATE_PATH}"
sudo rsync -ar ${SOURCE_FILE} ${UPDATE_FILE}

sudo chown 1000:1000 ${SOURCE_PATH}
sudo chmod 775 ${SOURCE_PATH}

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${UPDATE_PATH}
sudo chmod 775 ${UPDATE_PATH}



#############################
# Sync MuJoCo environment / world files
#
# Both folders below sync with -u, so the NEWER copy of a file wins in either
# direction. (The Gazebo service's sync has no -u: the share's copy always
# wins there, so a file re-staged on this machine by the setup script would be
# overwritten by the share's older copy.) Deletes still do not sync: remove a
# file on both sides, or it comes back.

SOURCE_PATH=${MUJOCO_SIM_FOLDER}/ENVIRONMENT_CONFIGS
UPDATE_PATH=${MUJOCO_HOME_FOLDER}/ENVIRONMENT_CONFIGS

echo "Syncing files from ${SOURCE_PATH} to ${UPDATE_PATH}"
if [[ ! -d $SOURCE_PATH ]]; then
    sudo mkdir -p $SOURCE_PATH
fi
sudo rsync -aru ${SOURCE_PATH}/ ${UPDATE_PATH}/

echo "Syncing files from ${UPDATE_PATH} to ${SOURCE_PATH}"
sudo rsync -aru ${UPDATE_PATH}/ ${SOURCE_PATH}/

sudo chown 1000:1000 ${SOURCE_PATH}
sudo chmod 775 ${SOURCE_PATH}

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${UPDATE_PATH}
sudo chmod 775 ${UPDATE_PATH}


#############################
# Sync MuJoCo robot/vehicle model files

SOURCE_PATH=${MUJOCO_SIM_FOLDER}/ROBOT_CONFIGS
UPDATE_PATH=${MUJOCO_HOME_FOLDER}/ROBOT_CONFIGS

echo "Syncing files from ${SOURCE_PATH} to ${UPDATE_PATH}"
if [[ ! -d $SOURCE_PATH ]]; then
    sudo mkdir -p $SOURCE_PATH
fi
sudo rsync -aru ${SOURCE_PATH}/ ${UPDATE_PATH}/

echo "Syncing files from ${UPDATE_PATH} to ${SOURCE_PATH}"
sudo rsync -aru ${UPDATE_PATH}/ ${SOURCE_PATH}/

sudo chown 1000:1000 ${SOURCE_PATH}
sudo chmod 775 ${SOURCE_PATH}

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${UPDATE_PATH}
sudo chmod 775 ${UPDATE_PATH}


##################
# Fix folder owners

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${MUJOCO_HOME_FOLDER}
sudo chmod 0775 ${MUJOCO_HOME_FOLDER}
sudo chown 1000:1000 ${MUJOCO_SIM_FOLDER}
sudo chmod 0775 ${MUJOCO_SIM_FOLDER}
