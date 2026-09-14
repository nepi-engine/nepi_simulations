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

GAZEBO_HOME_FOLDER=${HOME}/gazebo
# mkdir -p ${GAZEBO_HOME_FOLDER}

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
if [[ -f "$NEPI_UTILS_SOURCE" ]]; then
    source "$NEPI_UTILS_SOURCE"
else
    echo "NEPI Gazebo Bash Utils file not found at: ${NEPI_UTILS_SOURCE}"
    exit 1
fi

ORIG_PWD=$(pwd)
if nepistorage "$NEPISTORAGE_PASSWORD"; then
    GAZEBO_SIM_FOLDER=$(pwd)/databases/sims/gazebo
    cd "$ORIG_PWD"
else
    echo "Failed to reach nepi_storage"
    return 1 2>/dev/null || exit 1
fi
sudo mkdir -p ${GAZEBO_SIM_FOLDER}

##################
# Fix folder owners

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${GAZEBO_HOME_FOLDER}
sudo chmod 0775 ${GAZEBO_HOME_FOLDER}
sudo chown 1000:1000 ${GAZEBO_SIM_FOLDER}
sudo chmod 0750 ${GAZEBO_SIM_FOLDER}


#############################
echo ""
echo "Updating Gazebo Simulation Files and Folders"

#############################
# yaml files

SOURCE_PATH=${GAZEBO_SIM_FOLDER}/config
UPDATE_PATH=${GAZEBO_HOME_FOLDER}/config
CONFIG_FILENAME=nepi_gazebo_config.yaml

SOURCE_FILE=${SOURCE_PATH}/${CONFIG_FILENAME}
UPDATE_FILE=${UPDATE_PATH}/${CONFIG_FILENAME}

echo "Merging ${CONFIG_FILENAME} keys from ${UPDATE_PATH} into ${SOURCE_PATH}"
# Create the storage-side folder first, the same way the ENVIRONMENT and
# SYSTEM blocks below do. Without this, the very first run on a device that
# has no databases/sims/gazebo/config yet fails at the cp ("No such file or
# directory") -- and since the config file is what nepi_gazebo_start.sh and
# nepi_gazebo.sh both read, that first run never gets off the ground.
# mkdir -p on GAZEBO_SIM_FOLDER alone (above) does not cover this subfolder.
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
# Sync Gazebo environment / world files

SOURCE_PATH=${GAZEBO_SIM_FOLDER}/ENVIRONMENT
UPDATE_PATH=${GAZEBO_HOME_FOLDER}/ENVIRONMENT

echo "Syncing files from ${SOURCE_PATH} to ${UPDATE_PATH}"
if [[ ! -d $SOURCE_PATH ]]; then
    sudo mkdir -p $SOURCE_PATH
fi
sudo rsync -ar ${SOURCE_PATH}/ ${UPDATE_PATH}/

echo "Syncing files from ${UPDATE_PATH} to ${SOURCE_PATH}"
sudo rsync -ar ${UPDATE_PATH}/ ${SOURCE_PATH}/

sudo chown 1000:1000 ${SOURCE_PATH}
sudo chmod 775 ${SOURCE_PATH}

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${UPDATE_PATH}
sudo chmod 775 ${UPDATE_PATH}


#############################
# Sync Gazebo robot/vehicle model files

SOURCE_PATH=${GAZEBO_SIM_FOLDER}/SYSTEM
UPDATE_PATH=${GAZEBO_HOME_FOLDER}/SYSTEM

echo "Syncing files from ${SOURCE_PATH} to ${UPDATE_PATH}"
if [[ ! -d $SOURCE_PATH ]]; then
    sudo mkdir -p $SOURCE_PATH
fi
sudo rsync -ar ${SOURCE_PATH}/ ${UPDATE_PATH}/

echo "Syncing files from ${UPDATE_PATH} to ${SOURCE_PATH}"
sudo rsync -ar ${UPDATE_PATH}/ ${SOURCE_PATH}/

sudo chown 1000:1000 ${SOURCE_PATH}
sudo chmod 775 ${SOURCE_PATH}

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${UPDATE_PATH}
sudo chmod 775 ${UPDATE_PATH}


##################
# Fix folder owners

sudo chown ${CONFIG_USER}:${CONFIG_USER} ${GAZEBO_HOME_FOLDER}
sudo chmod 0775 ${GAZEBO_HOME_FOLDER}
sudo chown 1000:1000 ${GAZEBO_SIM_FOLDER}
sudo chmod 0775 ${GAZEBO_SIM_FOLDER}
