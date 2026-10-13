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

# Entry point for the MuJoCo simulation subsystem on the sim machine. Syncs the
# robot and world folders and the config file with NEPI storage, then starts
# nepi_mujoco.sh (the poll-config service) in the background.
#
# Unlike nepi_gazebo_start.sh, this does not launch the simulation itself: the
# simulation starts when the device asks for it (Deploy in the Sim Connector).
# That keeps the service as the only thing that ever starts it.
#
# Safe to re-run: it skips starting the service if it is already running.
#
# Usage: nepi_mujoco_start.sh [NEPISTORAGE_PASSWORD]
# The password is passed to nepi_mujoco_sync.sh's mount and exported for the
# backgrounded service, whose own mount call reuses the share the sync mounted.

NEPISTORAGE_PASSWORD=$1

MUJOCO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${MUJOCO_FOLDER}")

MUJOCO_SERVICE=${MUJOCO_FOLDER}/nepi_mujoco.sh
MUJOCO_SYNC_SCRIPT=${MUJOCO_FOLDER}/nepi_mujoco_sync.sh

source "${RESOURCES_FOLDER}/bash/nepi_mujoco_bash_utils"

for required in "$MUJOCO_SERVICE" "$MUJOCO_SYNC_SCRIPT"; do
    if [[ ! -f "$required" ]]; then
        echo "Not found: ${required}"
        exit 1
    fi
done

if ! python3 -c "import mujoco" >/dev/null 2>&1; then
    echo "The mujoco Python package is not installed -- run nepi_mujoco_setup.sh first"
    exit 1
fi


####################################
# 1. Sync ENVIRONMENT_CONFIGS/ROBOT_CONFIGS and the config file with NEPI
# storage. Also seeds nepi_mujoco_config.yaml in storage the first time, and
# sets MUJOCO_SIM_FOLDER.

source "$MUJOCO_SYNC_SCRIPT" "$NEPISTORAGE_PASSWORD"
wait

if [[ ! -f "${MUJOCO_SIM_FOLDER}/config/nepi_mujoco_config.yaml" ]]; then
    echo "MuJoCo config file not found: ${MUJOCO_SIM_FOLDER}/config/nepi_mujoco_config.yaml"
    exit 1
fi


####################################
# 2. Start the service, if it is not already running.

if pgrep -f "bash ${MUJOCO_SERVICE}" >/dev/null 2>&1; then
    echo "nepi_mujoco service is already running"
else
    echo "Starting nepi_mujoco service"
    NEPISTORAGE_PASSWORD="$NEPISTORAGE_PASSWORD" nohup bash "$MUJOCO_SERVICE" >/tmp/nepi_mujoco_service.log 2>&1 &
    echo "nepi_mujoco service started with PID $! (log: /tmp/nepi_mujoco_service.log)"
fi
