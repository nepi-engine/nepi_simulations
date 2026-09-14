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

# Turnkey entry point for the Gazebo simulation subsystem. Launches Gazebo
# itself against the world staged in ENVIRONMENT (with SYSTEM added to
# GAZEBO_MODEL_PATH so any robot models staged there resolve), records its
# PID/state into nepi_gazebo_config.yaml (in NEPI storage -- see below),
# then launches nepi_gazebo.sh (the ongoing poll-config/act-on-change/
# report-status service) in the background. nepi_gazebo.sh's own startup
# already knows how to reattach to a GAZEBO_PID it finds alive in the
# config rather than relaunching -- this script is what puts that PID
# there in the first place.
#
# Safe to re-run: it skips launching whichever of Gazebo / the service is
# already running rather than starting a second copy.
#
# Usage: nepi_gazebo_start.sh [NEPISTORAGE_PASSWORD]
# The password is forwarded to nepi_gazebo_sync.sh's own mount below, and
# exported for the backgrounded nepi_gazebo.sh service too, so its own
# nepistorage call (see that script) doesn't have to prompt.

NEPISTORAGE_PASSWORD=$1

GAZEBO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${GAZEBO_FOLDER}")

# ENVIRONMENT/SYSTEM are read from ${HOME}/gazebo, not RESOURCES_FOLDER
# (this nepi_gazebo repo checkout) -- nepi_gazebo_setup.sh stages them there
# at install time, and nepi_gazebo_sync.sh (below) keeps that copy
# reconciled against NEPI storage on every start. RESOURCES_FOLDER remains
# just where the scripts themselves live. GAZEBO_CONFIG_FILE is resolved
# below instead, once nepi_gazebo_sync.sh has mounted NEPI storage and set
# GAZEBO_SIM_FOLDER -- this script and nepi_gazebo.sh both read/write that
# same storage-side file directly, not a local copy.
GAZEBO_HOME_FOLDER=${HOME}/gazebo
GAZEBO_ENVIRONMENT_FOLDER=${GAZEBO_HOME_FOLDER}/ENVIRONMENT
GAZEBO_SYSTEM_FOLDER=${GAZEBO_HOME_FOLDER}/SYSTEM

GAZEBO_SERVICE=${GAZEBO_FOLDER}/nepi_gazebo.sh

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
source "$NEPI_UTILS_SOURCE"

if [[ ! -f "$GAZEBO_SERVICE" ]]; then
    echo "Gazebo service not found: ${GAZEBO_SERVICE}"
    exit 1
fi

GAZEBO_SYNC_SCRIPT=${GAZEBO_FOLDER}/nepi_gazebo_sync.sh
if [[ ! -f "$GAZEBO_SYNC_SCRIPT" ]]; then
    echo "Gazebo sync script not found: ${GAZEBO_SYNC_SCRIPT}"
    exit 1
fi


####################################
# 1. Sync ENVIRONMENT/SYSTEM against NEPI storage before launching, so any
# changes made on the storage side (e.g. over the network share) are
# picked up here, and any local changes get pushed back out. Also seeds
# nepi_gazebo_config.yaml in storage (from the local template) if it isn't
# there yet, and mounts storage for the rest of this script -- see
# nepi_gazebo_sync.sh for what actually gets synced, and note it sets
# GAZEBO_SIM_FOLDER as a side effect, used just below.
#
# Unlike the old local-copy design, this is no longer a one-shot: both this
# script and nepi_gazebo.sh (step 3) read/write nepi_gazebo_config.yaml
# directly in NEPI storage, and nepi_gazebo.sh keeps that mount open and
# keeps polling it for its entire run -- a device-side request written
# after this point still reaches it on the very next poll.

source "$GAZEBO_SYNC_SCRIPT" "$NEPISTORAGE_PASSWORD"
wait

GAZEBO_CONFIG_FILE=${GAZEBO_SIM_FOLDER}/config/nepi_gazebo_config.yaml

if [[ ! -f "$GAZEBO_CONFIG_FILE" ]]; then
    echo "Gazebo config file not found: ${GAZEBO_CONFIG_FILE}"
    exit 1
fi


####################################
# 2. Launch Gazebo, if it isn't already running. nepi_gazebo.sh keeps its
# own copy of this same find-a-world/set-search-paths/launch logic for a
# later config-driven start -- kept inline in both places rather than a
# shared hook script, since it's short and each caller backgrounds it
# differently (this one direct via nohup, the service via its own poll
# loop).

# GAZEBO_CURRENT_ENVIRONMENT names the world to launch; it is the same
# selection the service honours, read straight from the config so both agree
# on which world is "current". Only when it is unset (or names something that
# is no longer staged) does this fall back to the first .world in the folder
# -- which is alphabetical, and therefore a guess. The fallback is recorded
# below so the guess happens at most once.
CURRENT_ENVIRONMENT=$(yq e '.GAZEBO_CURRENT_ENVIRONMENT // ""' "$GAZEBO_CONFIG_FILE" 2>/dev/null)

WORLD_FILE=""
if [[ -n "$CURRENT_ENVIRONMENT" ]]; then
    if [[ -f "${GAZEBO_ENVIRONMENT_FOLDER}/${CURRENT_ENVIRONMENT}" ]]; then
        WORLD_FILE=${GAZEBO_ENVIRONMENT_FOLDER}/${CURRENT_ENVIRONMENT}
    else
        echo "Selected environment '${CURRENT_ENVIRONMENT}' is not staged in ${GAZEBO_ENVIRONMENT_FOLDER} -- falling back"
    fi
fi

if [[ -z "$WORLD_FILE" ]]; then
    WORLD_FILE=$(ls ${GAZEBO_ENVIRONMENT_FOLDER}/*.world 2>/dev/null | head -n 1)
fi

if [[ -z "$WORLD_FILE" ]]; then
    echo "No .world file found in ${GAZEBO_ENVIRONMENT_FOLDER} -- stage one before running this script"
    exit 1
fi

# Record what was actually launched, so the config always reflects reality
# and the fallback above is not re-guessed on the next start.
if [[ "$CURRENT_ENVIRONMENT" != "$(basename "$WORLD_FILE")" ]]; then
    update_yaml_value GAZEBO_CURRENT_ENVIRONMENT "$(basename "$WORLD_FILE")" "$GAZEBO_CONFIG_FILE"
fi

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${GAZEBO_SYSTEM_FOLDER}
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${GAZEBO_ENVIRONMENT_FOLDER}

if pgrep -f "gazebo --verbose ${WORLD_FILE}" >/dev/null 2>&1; then
    echo "Gazebo is already running against ${WORLD_FILE}"
else
    echo "Launching Gazebo with world: ${WORLD_FILE}"
    nohup gazebo --verbose "$WORLD_FILE" >/tmp/nepi_gazebo.log 2>&1 &
    gazebo_pid=$!

    sleep 2
    if ! kill -0 "$gazebo_pid" 2>/dev/null; then
        echo "Gazebo exited immediately after launch -- see /tmp/nepi_gazebo.log"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "Gazebo exited immediately after launch" "$GAZEBO_CONFIG_FILE"
        exit 1
    fi

    echo "Gazebo running with PID ${gazebo_pid}"
    update_yaml_value GAZEBO_PID "$gazebo_pid" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_START 1 "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_STOP 0 "$GAZEBO_CONFIG_FILE"
    # Cleared directly rather than via update_yaml_value -- that function's
    # yq env() call errors on an empty value and leaves the field untouched,
    # which is the same quirk nepi_gazebo.sh works around in
    # clear_gazebo_last_error(). Going through it here would leave a stale
    # error from a previous failed run sitting in a now-successful start.
    yq e -i '.GAZEBO_LAST_ERROR = ""' "$GAZEBO_CONFIG_FILE"
fi


####################################
# 3. Launch the nepi_gazebo management service, if it isn't already running.
# It reads GAZEBO_PID/GAZEBO_STATE back out of the config on startup and
# reattaches to the Gazebo process above rather than relaunching it.

if pgrep -f "bash ${GAZEBO_SERVICE}" >/dev/null 2>&1; then
    echo "nepi_gazebo service is already running"
else
    echo "Starting nepi_gazebo service"
    NEPISTORAGE_PASSWORD="$NEPISTORAGE_PASSWORD" nohup bash "$GAZEBO_SERVICE" >/tmp/nepi_gazebo_service.log 2>&1 &
    echo "nepi_gazebo service started with PID $!"
fi
