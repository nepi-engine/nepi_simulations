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

# This script is the NEPI Gazebo Simulator Management Service. It runs on
# the VM/host machine and polls nepi_gazebo_config.yaml for start/stop/
# install requests written by the device side (see the per-request-file
# protocol referenced in that file's comments), and reports GAZEBO_STATE,
# GAZEBO_PID and GAZEBO_LAST_ERROR back into the same file so the device
# can observe progress.
#
# Modeled on nepi_docker.sh's poll-config / act-on-change / report-status
# loop, WITH the same "watch the file where it actually lives" approach
# that script uses (nepi_docker.sh runs out of /mnt/nepi_config/docker_cfg
# directly, no separate local staging copy) -- this service mounts NEPI
# storage once at startup below and polls GAZEBO_CONFIG_FILE there for its
# entire lifetime, so a device-written request (e.g. GAZEBO_STOP: 1) is
# picked up on the very next poll, with no re-sync/restart needed.
#
# Assumes nepi_gazebo_bash_utils is already sourced by whatever launches
# this script (same assumption nepi_docker.sh makes of nepi_bash_utils) --
# update_yaml_value and nepistorage are used as already-exported commands,
# not defined here. nepi_gazebo_start.sh, the normal way this service gets
# started, sources it before backgrounding this script, and also exports
# NEPISTORAGE_PASSWORD so the nepistorage call below doesn't need to prompt.

GAZEBO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

# ENVIRONMENT/SYSTEM (the staged .world/model files Gazebo itself reads)
# still come from ${HOME}/gazebo, not this script's own folder --
# nepi_gazebo_setup.sh stages them there at install time, and
# nepi_gazebo_sync.sh keeps that copy reconciled against NEPI storage on
# every start (see nepi_gazebo_start.sh, the normal way this service gets
# started). GAZEBO_CONFIG_FILE is different: it's resolved below, directly
# under NEPI storage, once that's mounted.
GAZEBO_HOME_FOLDER=${HOME}/gazebo
GAZEBO_ENVIRONMENT_FOLDER=${GAZEBO_HOME_FOLDER}/ENVIRONMENT
GAZEBO_SYSTEM_FOLDER=${GAZEBO_HOME_FOLDER}/SYSTEM

GAZEBO_CONFIG_LOAD_FILE=${GAZEBO_FOLDER}/load_gazebo_config.sh

# INSTALL_SCRIPT remains an unfilled extension point: dropped here rather
# than guessed, since the real install invocation depends on how this VM's
# Gazebo/ROS environment is laid out. There is no equivalent LAUNCH_SCRIPT
# -- launching Gazebo (find the staged .world file, extend
# GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH, run it) is small enough that this
# service and nepi_gazebo_start.sh each keep their own inline copy rather
# than sharing a hook script.
INSTALL_SCRIPT=${GAZEBO_ENVIRONMENT_FOLDER}/install_gazebo.sh

POLL_SECONDS=1
LAUNCH_SETTLE_SECONDS=1
# How long to let Gazebo wind down after a SIGINT before escalating.
STOP_GRACE_SECONDS=5

if [[ ! -f "$GAZEBO_CONFIG_LOAD_FILE" ]]; then
    echo "Load script not found: ${GAZEBO_CONFIG_LOAD_FILE}"
    exit 1
fi

if ! command -v update_yaml_value >/dev/null 2>&1; then
    echo "update_yaml_value is not defined -- source nepi_gazebo_bash_utils before running this script"
    exit 1
fi

if ! command -v nepistorage >/dev/null 2>&1; then
    echo "nepistorage is not defined -- source nepi_gazebo_bash_utils before running this script"
    exit 1
fi

# Mount NEPI storage once, for the life of this service, and watch the
# nepi_gazebo_config.yaml that lives there directly -- same pattern
# nepi_gazebo_sync.sh already uses (nepistorage cd's into the mount on
# success; capture that via pwd, then restore our own cwd). No local copy
# of this file is read or written anywhere below.
ORIG_PWD=$(pwd)
if nepistorage "$NEPISTORAGE_PASSWORD"; then
    GAZEBO_SIM_FOLDER=$(pwd)/databases/sims/gazebo
    cd "$ORIG_PWD"
else
    echo "Failed to reach NEPI storage -- cannot watch nepi_gazebo_config.yaml"
    exit 1
fi

GAZEBO_CONFIG_FILE=${GAZEBO_SIM_FOLDER}/config/nepi_gazebo_config.yaml

if [[ ! -f "$GAZEBO_CONFIG_FILE" ]]; then
    echo "Gazebo config file not found: ${GAZEBO_CONFIG_FILE}"
    exit 1
fi

# 0 = the install script ran and succeeded, 2 = there is no install script to
# run, anything else = it ran and failed. "No script" is deliberately NOT
# folded into success: INSTALL_SCRIPT is an unfilled extension point, so
# returning 0 for a missing file makes every install request report
# "completed" while installing nothing.
function run_install(){
    if [[ -f "$INSTALL_SCRIPT" ]]; then
        bash "$INSTALL_SCRIPT"
        return $?
    fi
    return 2
}

function is_gazebo_running(){
    [[ "$gazebo_pid" -ne 0 ]] && kill -0 "$gazebo_pid" 2>/dev/null
}

function clear_gazebo_last_error(){
    # update_yaml_value's yq env() call errors on an empty value ("Value
    # for env variable ... not provided in env()") -- that's a yq quirk
    # inside the shared function, not something callable around, so
    # clear this one field directly instead of going through it.
    yq e -i '.GAZEBO_LAST_ERROR = ""' "$GAZEBO_CONFIG_FILE"
}

# Same yq-empty-value quirk as above: clearing an UPDATE_* field back to ""
# cannot go through update_yaml_value.
function clear_yaml_field(){
    yq e -i '.'"$1"' = ""' "$GAZEBO_CONFIG_FILE"
    eval "export $1=''"
}

# Print the .world file to launch. GAZEBO_CURRENT_ENVIRONMENT names it when
# set; otherwise fall back to the first .world in the folder -- which is what
# this service always used to do, and is only a guess, since `ls` is
# alphabetical (a stray generic_rover.world would quietly outrank
# iris_arducopter_cmac.world). The caller records whatever this picks, so the
# guess happens at most once.
function resolve_world_file(){
    if [[ -n "$GAZEBO_CURRENT_ENVIRONMENT" ]]; then
        local named=${GAZEBO_ENVIRONMENT_FOLDER}/${GAZEBO_CURRENT_ENVIRONMENT}
        if [[ -f "$named" ]]; then
            echo "$named"
            return 0
        fi
        # Recorded but no longer present -- say so rather than silently
        # launching some other world.
        echo ""
        return 1
    fi
    ls ${GAZEBO_ENVIRONMENT_FOLDER}/*.world 2>/dev/null | head -n 1
}

# 0 = valid. Sets validate_error on failure.
function validate_environment_name(){
    local name=$1
    validate_error=""
    if [[ "$name" != *.world ]]; then
        validate_error="environment '${name}' is not a .world file"
        return 1
    fi
    if [[ ! -f "${GAZEBO_ENVIRONMENT_FOLDER}/${name}" ]]; then
        validate_error="environment '${name}' not found in ${GAZEBO_ENVIRONMENT_FOLDER}"
        return 1
    fi
    return 0
}

function validate_model_name(){
    local name=$1
    validate_error=""
    if [[ ! -d "${GAZEBO_SYSTEM_FOLDER}/${name}" ]]; then
        validate_error="model '${name}' not found in ${GAZEBO_SYSTEM_FOLDER}"
        return 1
    fi
    if [[ ! -f "${GAZEBO_SYSTEM_FOLDER}/${name}/model.config" ]]; then
        validate_error="model '${name}' has no model.config"
        return 1
    fi
    return 0
}

# Bring Gazebo down and report idle. No-op if nothing is running, so it is
# safe to call unconditionally (e.g. from the environment-change restart).
function stop_gazebo(){
    is_gazebo_running || return 0

    echo "Stopping Gazebo (pid ${gazebo_pid})"
    update_yaml_value GAZEBO_STATE "stopping" "$GAZEBO_CONFIG_FILE"

    # SIGINT, not the default SIGTERM: the `gazebo` binary is a wrapper
    # around gzserver and gzclient, and SIGINT is the signal it handles and
    # passes on to them. A SIGTERM kills only the wrapper -- gzserver and
    # gzclient survive, get reparented to init, and keep the simulator window
    # up and port 11345 held, while this service reports "idle".
    kill -INT "$gazebo_pid" 2>/dev/null

    # Reattached PIDs are not children of this shell, so `wait` cannot be
    # used to know when it is really gone -- poll instead, then sweep up
    # anything that ignored the signal.
    local _ survivors
    for _ in $(seq 1 "$STOP_GRACE_SECONDS"); do
        kill -0 "$gazebo_pid" 2>/dev/null || break
        sleep 1
    done

    # Children that outlived the wrapper (or a wrapper that never went down)
    # get a direct SIGINT, then SIGKILL as a last resort, so a stop always
    # actually stops.
    survivors=$(pgrep -f "gz(server|client) --verbose ${GAZEBO_ENVIRONMENT_FOLDER}/" 2>/dev/null)
    if [[ -n "$survivors" ]]; then
        echo "Gazebo children survived the wrapper -- signalling: ${survivors//$'\n'/ }"
        kill -INT $survivors 2>/dev/null
        sleep "$STOP_GRACE_SECONDS"
        survivors=$(pgrep -f "gz(server|client) --verbose ${GAZEBO_ENVIRONMENT_FOLDER}/" 2>/dev/null)
        [[ -n "$survivors" ]] && kill -KILL $survivors 2>/dev/null
    fi

    kill -0 "$gazebo_pid" 2>/dev/null && kill -KILL "$gazebo_pid" 2>/dev/null
    gazebo_pid=0
    update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_STATE "idle" "$GAZEBO_CONFIG_FILE"
    return 0
}

# Launch Gazebo against the selected world and report the outcome.
function start_gazebo(){
    local world_file
    world_file=$(resolve_world_file)

    if [[ -z "$world_file" ]]; then
        local why="no .world file found in ${GAZEBO_ENVIRONMENT_FOLDER}"
        [[ -n "$GAZEBO_CURRENT_ENVIRONMENT" ]] && \
            why="selected environment '${GAZEBO_CURRENT_ENVIRONMENT}' is missing from ${GAZEBO_ENVIRONMENT_FOLDER}"
        echo "Start requested but ${why}"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "$why" "$GAZEBO_CONFIG_FILE"
        return 1
    fi

    # Record what an empty selection fell back to, so the alphabetical guess
    # in resolve_world_file happens at most once and the device can see which
    # world is actually loaded.
    if [[ -z "$GAZEBO_CURRENT_ENVIRONMENT" ]]; then
        update_yaml_value GAZEBO_CURRENT_ENVIRONMENT "$(basename "$world_file")" "$GAZEBO_CONFIG_FILE"
    fi

    echo "Launching Gazebo with world: ${world_file}"
    update_yaml_value GAZEBO_STATE "starting" "$GAZEBO_CONFIG_FILE"

    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${GAZEBO_SYSTEM_FOLDER}
    export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${GAZEBO_ENVIRONMENT_FOLDER}
    gazebo --verbose "$world_file" &
    gazebo_pid=$!
    update_yaml_value GAZEBO_PID "$gazebo_pid" "$GAZEBO_CONFIG_FILE"

    sleep "$LAUNCH_SETTLE_SECONDS"
    if kill -0 "$gazebo_pid" 2>/dev/null; then
        update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
        return 0
    fi

    echo "Gazebo exited immediately after launch"
    update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_LAST_ERROR "Gazebo exited immediately after launch" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
    gazebo_pid=0
    return 1
}

echo ""
echo "##########################"
echo "*** STARTING NEPI GAZEBO SERVICE ***"
echo "##########################"
echo ""
echo "Watching config file: ${GAZEBO_CONFIG_FILE}"

source "$GAZEBO_CONFIG_LOAD_FILE" "$GAZEBO_CONFIG_FILE" > /dev/null 2>&1

gazebo_pid=0
if [[ "$GAZEBO_PID" -ne 0 ]] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
    echo "Reattaching to already-running Gazebo process ${GAZEBO_PID}"
    gazebo_pid=$GAZEBO_PID
elif [[ "$GAZEBO_STATE" == "running" || "$GAZEBO_STATE" == "starting" || "$GAZEBO_STATE" == "installing" ]]; then
    echo "Recorded state was '${GAZEBO_STATE}' but no matching process is running -- resetting to idle"
    update_yaml_value GAZEBO_STATE "idle" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
fi

while true; do
    source "$GAZEBO_CONFIG_LOAD_FILE" "$GAZEBO_CONFIG_FILE" > /dev/null 2>&1

    # Adopt a live GAZEBO_PID that someone else recorded -- nepi_gazebo_start.sh
    # launches Gazebo itself and writes the PID here, and it does that whether
    # or not this service was already running. Without this, a service sitting
    # idle keeps its own gazebo_pid at 0, reads the GAZEBO_START that
    # nepi_gazebo_start.sh also sets, concludes nothing is running, and starts
    # a SECOND Gazebo on top of the one that is already up.
    if [[ "$gazebo_pid" -eq 0 && "$GAZEBO_PID" -ne 0 ]] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        echo "Adopting Gazebo process ${GAZEBO_PID} recorded in the config"
        gazebo_pid=$GAZEBO_PID
    fi

    # The request flags ARE the request: setting any of them to 1 -- by the
    # device, or by hand in the config file -- is picked up on the next poll,
    # with no companion timestamp to keep in step. This used to be gated on a
    # GAZEBO_LAST_UPDATED that had to be bumped alongside the flag, which made
    # a lone "GAZEBO_STOP: 0 -> 1" edit silently do nothing. That gate was
    # redundant: the clear-on-act below is what actually stops a request from
    # re-firing, so dropping it costs nothing and makes hand-editing work the
    # way the file reads.
    if [[ "$GAZEBO_INSTALL" -eq 1 || "$GAZEBO_STOP" -eq 1 || "$GAZEBO_START" -eq 1 \
          || -n "$GAZEBO_UPDATE_ENVIRONMENT" || -n "$GAZEBO_UPDATE_MODEL" ]]; then
        clear_gazebo_last_error
        install_requested=0

        # Request flags are cleared here, by the service, once the action
        # they name has actually been carried out -- same convention
        # nepi_docker.sh uses for its own request flags (e.g.
        # NEPI_UPDATE_CONFIG, NEPI_EXPAND_FS): do the work (with an
        # "-ing"/GAZEBO_STATE update while it's in flight), then zero the
        # flag once the outcome is known. Left uncleared, the flag would
        # still read 1 on the very next poll and re-run the same action.
        if [[ "$GAZEBO_INSTALL" -eq 1 ]]; then
            # Remember that this pass handled an install: update_yaml_value
            # re-exports the key it writes, so GAZEBO_INSTALL becomes 0 in
            # this shell the moment the flag is cleared below, and cannot be
            # tested afterwards.
            install_requested=1

            echo "Install requested -- running install step"
            update_yaml_value GAZEBO_STATE "installing" "$GAZEBO_CONFIG_FILE"
            run_install
            install_rc=$?
            update_yaml_value GAZEBO_INSTALL 0 "$GAZEBO_CONFIG_FILE"

            if [[ $install_rc -eq 0 ]]; then
                echo "Install step completed"
            elif [[ $install_rc -eq 2 ]]; then
                # Reported rather than swallowed -- the device asked for an
                # install and did not get one.
                echo "No install script at ${INSTALL_SCRIPT} -- nothing was installed"
                update_yaml_value GAZEBO_LAST_ERROR "no install script at ${INSTALL_SCRIPT}" "$GAZEBO_CONFIG_FILE"
            else
                echo "Install step failed (exit ${install_rc})"
                update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
                update_yaml_value GAZEBO_LAST_ERROR "install step failed (exit ${install_rc})" "$GAZEBO_CONFIG_FILE"
                sleep "$POLL_SECONDS"
                continue
            fi
        fi

        # Selection changes are handled BEFORE start/stop, so that a request
        # pairing a new environment with GAZEBO_START uses the new world
        # rather than the outgoing one.
        if [[ -n "$GAZEBO_UPDATE_ENVIRONMENT" ]]; then
            requested_env=$GAZEBO_UPDATE_ENVIRONMENT
            clear_yaml_field GAZEBO_UPDATE_ENVIRONMENT

            if validate_environment_name "$requested_env"; then
                if [[ "$requested_env" == "$GAZEBO_CURRENT_ENVIRONMENT" ]]; then
                    echo "Environment '${requested_env}' is already current -- nothing to do"
                else
                    echo "Environment change accepted: '${GAZEBO_CURRENT_ENVIRONMENT}' -> '${requested_env}'"
                    update_yaml_value GAZEBO_CURRENT_ENVIRONMENT "$requested_env" "$GAZEBO_CONFIG_FILE"

                    # A running gzserver cannot swap worlds in place, so the
                    # only way to make this take effect now is a full cycle.
                    if is_gazebo_running; then
                        echo "Restarting Gazebo against the new environment"
                        stop_gazebo
                        start_gazebo
                    fi
                fi
            else
                echo "Environment change rejected: ${validate_error}"
                update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
            fi
        fi

        if [[ -n "$GAZEBO_UPDATE_MODEL" ]]; then
            requested_model=$GAZEBO_UPDATE_MODEL
            clear_yaml_field GAZEBO_UPDATE_MODEL

            if validate_model_name "$requested_model"; then
                echo "Model change accepted: '${GAZEBO_CURRENT_MODEL}' -> '${requested_model}'"
                update_yaml_value GAZEBO_CURRENT_MODEL "$requested_model" "$GAZEBO_CONFIG_FILE"
            else
                echo "Model change rejected: ${validate_error}"
                update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
            fi
        fi

        if [[ "$GAZEBO_STOP" -eq 1 ]]; then
            stop_gazebo
            update_yaml_value GAZEBO_STOP 0 "$GAZEBO_CONFIG_FILE"
        elif [[ "$GAZEBO_START" -eq 1 ]]; then
            if ! is_gazebo_running; then
                start_gazebo
            else
                # Already running (e.g. reattached after a service
                # restart) -- reaffirm state in case the install step
                # above left it on "installing".
                update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
            fi
            update_yaml_value GAZEBO_START 0 "$GAZEBO_CONFIG_FILE"
        fi

        # An install requested on its own -- GAZEBO_START left at 0, which the
        # config documents as a valid way to pre-install without launching --
        # reaches neither branch above, so nothing would move GAZEBO_STATE off
        # "installing" and it would stay pinned there for good. Resolve it to
        # whatever is actually true now.
        if [[ "$install_requested" -eq 1 && "$GAZEBO_STATE" == "installing" ]]; then
            if is_gazebo_running; then
                update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
            else
                update_yaml_value GAZEBO_STATE "idle" "$GAZEBO_CONFIG_FILE"
            fi
        fi
    fi

    # Catch a crash even without a new device request.
    if [[ "$gazebo_pid" -ne 0 ]] && ! kill -0 "$gazebo_pid" 2>/dev/null; then
        echo "Gazebo process ${gazebo_pid} is no longer running"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "Gazebo process exited unexpectedly" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
        gazebo_pid=0
    fi

    update_yaml_value GAZEBO_LAST_POLL "$(date +%s)" "$GAZEBO_CONFIG_FILE"

    sleep "$POLL_SECONDS"
done
