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
# the VM/host machine and polls nepi_gazebo_config.yaml for start/stop and
# environment/robot config-selection requests written by the device side (see
# the per-request-file protocol referenced in that file's comments), and
# reports GAZEBO_STATE,
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

# ENVIRONMENT_CONFIGS/ROBOT_CONFIGS (the staged .world/model files Gazebo
# itself reads) still come from ${HOME}/gazebo, not this script's own folder --
# nepi_gazebo_setup.sh stages them there at install time, and
# nepi_gazebo_sync.sh keeps that copy reconciled against NEPI storage on
# every start (see nepi_gazebo_start.sh, the normal way this service gets
# started). GAZEBO_CONFIG_FILE is different: it's resolved below, directly
# under NEPI storage, once that's mounted.
#
# Renamed 2026-09-18 from ENVIRONMENT/SYSTEM, matching the config keys these
# back (GAZEBO_*_ENVIRONMENT_CONFIG / GAZEBO_*_ROBOT_CONFIG) so the folder a
# name is validated against is obvious from the key that carries it.
GAZEBO_HOME_FOLDER=${HOME}/gazebo
GAZEBO_ENVIRONMENT_CONFIGS_FOLDER=${GAZEBO_HOME_FOLDER}/ENVIRONMENT_CONFIGS
GAZEBO_ROBOT_CONFIGS_FOLDER=${GAZEBO_HOME_FOLDER}/ROBOT_CONFIGS

GAZEBO_CONFIG_LOAD_FILE=${GAZEBO_FOLDER}/load_gazebo_config.sh

# There is no LAUNCH_SCRIPT hook -- launching Gazebo (find the staged .world
# file, extend GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH, run it) is small enough
# that this service and nepi_gazebo_start.sh each keep their own inline copy
# rather than sharing one.
#
# The install step (GAZEBO_INSTALL / INSTALL_SCRIPT / run_install) was removed
# 2026-09-18. It had never worked: INSTALL_SCRIPT pointed at an
# install_gazebo.sh that nothing in nepi_simulations ships, so every install
# request recorded "no install script at ..." and installed nothing, while the
# device side's is_installed() unconditionally reported True so the Install
# button never appeared anyway. Dependencies are installed by
# nepi_gazebo_setup.sh, out of band from this service.

POLL_SECONDS=1
LAUNCH_SETTLE_SECONDS=1
# How long to let Gazebo wind down after a SIGINT before escalating.
STOP_GRACE_SECONDS=5

# Robot configs are spawned into the RUNNING world with `gz model`, not baked
# into the .world file. Two reasons, both learned here already:
#
#   * A model name that originates from a world-file <include> tag hits a real
#     Gazebo caching quirk on its first respawn -- confirmed live 2026-09-09
#     against the camera rigs, which went silently dead after one FOV change
#     until they were moved to runtime spawning. See the surviving note in
#     iris_arducopter_cmac.world. Spawning from the start means no name is ever
#     include-derived, so that quirk cannot bite.
#   * It makes environment and robot genuinely independent: N scenery worlds x
#     M robots, instead of one .world per combination.
#
# gzserver's spawn service is not up the instant the process starts, so the
# spawn is retried rather than gated on a separate readiness probe -- a failed
# spawn is self-correcting on the next attempt, and one that never succeeds
# reports itself through GAZEBO_LAST_ERROR.
#
# CRITICAL, measured against Gazebo 11.15.1 (2026-09-18): `gz model` exits 0 no
# matter what happens. It exits 0 for a spawn whose SDF file does not exist,
# for a spawn whose SDF produces no model at all, for -i on a model that is not
# there, and for -d on a model that is not there. Its exit status therefore
# carries NO information and must never be used to decide whether an operation
# worked -- robot_config_is_spawned below reads the OUTPUT instead, which is
# the only usable signal. This is the same "reported success while doing
# nothing" trap kill_all_gazebo was rewritten to escape.
#
# Both spawn and delete are also ASYNCHRONOUS: gz returns long before gzserver
# has finished, with a delete measured taking well over 2s on a live server. So
# every operation here is confirmed by polling for the model's actual presence,
# never by the call returning.
SPAWN_ATTEMPTS=10
SPAWN_RETRY_SECONDS=2
DESPAWN_ATTEMPTS=10

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

# Print the .world file to launch. GAZEBO_CURRENT_ENVIRONMENT_CONFIG names it
# when set; otherwise fall back to the first .world in the folder -- which is
# what this service always used to do, and is only a guess, since `ls` is
# alphabetical (a stray generic_rover.world would quietly outrank
# iris_arducopter_cmac.world). The caller records whatever this picks, so the
# guess happens at most once.
function resolve_world_file(){
    if [[ -n "$GAZEBO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
        local named=${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}/${GAZEBO_CURRENT_ENVIRONMENT_CONFIG}
        if [[ -f "$named" ]]; then
            echo "$named"
            return 0
        fi
        # Recorded but no longer present -- say so rather than silently
        # launching some other world.
        echo ""
        return 1
    fi
    ls ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}/*.world 2>/dev/null | head -n 1
}

# 0 = valid. Sets validate_error on failure.
function validate_environment_config_name(){
    local name=$1
    validate_error=""
    if [[ "$name" != *.world ]]; then
        validate_error="environment config '${name}' is not a .world file"
        return 1
    fi
    if [[ ! -f "${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}/${name}" ]]; then
        validate_error="environment config '${name}' not found in ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}"
        return 1
    fi
    return 0
}

function validate_robot_config_name(){
    local name=$1
    validate_error=""
    if [[ ! -d "${GAZEBO_ROBOT_CONFIGS_FOLDER}/${name}" ]]; then
        validate_error="robot config '${name}' not found in ${GAZEBO_ROBOT_CONFIGS_FOLDER}"
        return 1
    fi
    if [[ ! -f "${GAZEBO_ROBOT_CONFIGS_FOLDER}/${name}/model.config" ]]; then
        validate_error="robot config '${name}' has no model.config"
        return 1
    fi
    # model.sdf is what actually gets spawned, so a config without one is
    # rejected here rather than validating clean and then failing at spawn
    # time, when the operator has already been told the change was accepted.
    if [[ ! -f "$(robot_config_sdf "$name")" ]]; then
        validate_error="robot config '${name}' has no model.sdf to spawn"
        return 1
    fi
    return 0
}

function robot_config_sdf(){
    echo "${GAZEBO_ROBOT_CONFIGS_FOLDER}/${1}/model.sdf"
}

# True when a model of this name exists in the running world. Reads gz's OUTPUT
# rather than its exit status, for the reason spelled out at SPAWN_ATTEMPTS: the
# exit status is always 0 and says nothing. Also correctly reports absent when
# gzserver is not running at all (gz returns immediately in that case rather
# than blocking, so this is safe to call from the poll loop unguarded).
function robot_config_is_spawned(){
    gz model -m "$1" -i 2>&1 | grep -q '^name:'
}

# Spawn a robot config into the running world. The spawned model takes the
# config's own folder name -- `gz model -m` overrides whatever <model name> the
# SDF itself carries -- which is what despawn_robot_config deletes by and what
# GAZEBO_CURRENT_ROBOT_CONFIG records.
#
# The SDF must be a real model definition: a file whose root is a bare
# <include> spawns nothing at all (and still exits 0), while wrapping that
# <include> in an outer <model> to satisfy the parser double-loads the included
# model's plugins. See ROBOT_CONFIGS/README.md for both findings and for how to
# offer a model that lives elsewhere on the machine.
function spawn_robot_config(){
    local name=$1
    local sdf
    sdf=$(robot_config_sdf "$name")

    local attempt
    for attempt in $(seq 1 "$SPAWN_ATTEMPTS"); do
        gz model -m "$name" -f "$sdf" >/dev/null 2>&1
        # Confirm, never trust: the call above reports success unconditionally,
        # and the spawn it requested completes asynchronously afterwards.
        sleep "$SPAWN_RETRY_SECONDS"
        if robot_config_is_spawned "$name"; then
            echo "Spawned robot config '${name}'"
            return 0
        fi
        # Re-issued only once the model is confirmed absent, so a slow spawn
        # cannot turn into a duplicate.
    done

    echo "Failed to spawn robot config '${name}' after ${SPAWN_ATTEMPTS} attempts"
    update_yaml_value GAZEBO_LAST_ERROR "failed to spawn robot config '${name}'" "$GAZEBO_CONFIG_FILE"
    return 1
}

# Print the robot config to spawn, or "" when there is nothing usable.
#
# Mirrors resolve_world_file deliberately, including the fallback: an empty
# GAZEBO_CURRENT_ROBOT_CONFIG means "no explicit choice", not "no robot", so
# the first valid folder is used and the caller records it. Without that, a
# fresh config (every key still "") brings up an EMPTY world -- which is what
# happened the moment robots moved out of the .world files, and reads as a
# broken simulator rather than as an unmade choice.
#
# A recorded-but-now-invalid name returns "" rather than silently substituting
# some other robot, same as resolve_world_file refuses to quietly launch a
# different world.
function resolve_robot_config(){
    if [[ -n "$GAZEBO_CURRENT_ROBOT_CONFIG" ]]; then
        if validate_robot_config_name "$GAZEBO_CURRENT_ROBOT_CONFIG"; then
            echo "$GAZEBO_CURRENT_ROBOT_CONFIG"
            return 0
        fi
        echo ""
        return 1
    fi

    local dir name
    for dir in "${GAZEBO_ROBOT_CONFIGS_FOLDER}"/*/; do
        [[ -d "$dir" ]] || continue
        name=$(basename "$dir")
        if validate_robot_config_name "$name"; then
            echo "$name"
            return 0
        fi
    done
    echo ""
}

# Make sure the selected robot is actually in the running world. Idempotent, so
# it is safe on any path that might already have spawned it.
#
# This is the ONE place a robot gets put into a world, which matters because
# Gazebo can be launched two ways: by start_gazebo below, and inline by
# nepi_gazebo_start.sh at boot. That second path does not call start_gazebo at
# all -- it runs its own `gazebo --verbose` and then sets GAZEBO_START, which
# this service picks up and routes here. Duplicating spawn logic into that
# script instead would mean two copies to keep in step.
function ensure_robot_spawned(){
    local name
    name=$(resolve_robot_config)

    if [[ -z "$name" ]]; then
        if [[ -n "$GAZEBO_CURRENT_ROBOT_CONFIG" ]]; then
            echo "Selected robot config '${GAZEBO_CURRENT_ROBOT_CONFIG}' is not usable: ${validate_error}"
            update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
        else
            echo "No usable robot config in ${GAZEBO_ROBOT_CONFIGS_FOLDER} -- world will have no robot"
        fi
        return 1
    fi

    # Record a fallback pick so the guess happens at most once, and so the
    # device can see which robot is actually loaded.
    if [[ "$name" != "$GAZEBO_CURRENT_ROBOT_CONFIG" ]]; then
        echo "No robot config selected -- using '${name}'"
        update_yaml_value GAZEBO_CURRENT_ROBOT_CONFIG "$name" "$GAZEBO_CONFIG_FILE"
    fi

    robot_config_is_spawned "$name" && return 0
    spawn_robot_config "$name"
}

# Remove a spawned robot config, and wait until it is actually gone. Waiting
# matters: a swap spawns the replacement straight after this returns, and a
# delete still in flight would briefly leave both in the world.
#
# Not an error when the model was never there -- the first swap of a session
# has nothing to remove, and start_gazebo's own spawn path calls nothing here.
function despawn_robot_config(){
    local name=$1
    [[ -z "$name" ]] && return 0
    robot_config_is_spawned "$name" || return 0

    gz model -m "$name" -d >/dev/null 2>&1

    local attempt
    for attempt in $(seq 1 "$DESPAWN_ATTEMPTS"); do
        if ! robot_config_is_spawned "$name"; then
            echo "Removed robot config '${name}'"
            return 0
        fi
        sleep "$SPAWN_RETRY_SECONDS"
    done

    echo "Robot config '${name}' was still present after ${DESPAWN_ATTEMPTS} delete checks"
    update_yaml_value GAZEBO_LAST_ERROR "could not remove robot config '${name}'" "$GAZEBO_CONFIG_FILE"
    return 1
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
    survivors=$(pgrep -f "gz(server|client) --verbose ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}/" 2>/dev/null)
    if [[ -n "$survivors" ]]; then
        echo "Gazebo children survived the wrapper -- signalling: ${survivors//$'\n'/ }"
        kill -INT $survivors 2>/dev/null
        sleep "$STOP_GRACE_SECONDS"
        survivors=$(pgrep -f "gz(server|client) --verbose ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}/" 2>/dev/null)
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
        local why="no .world file found in ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}"
        [[ -n "$GAZEBO_CURRENT_ENVIRONMENT_CONFIG" ]] && \
            why="selected environment config '${GAZEBO_CURRENT_ENVIRONMENT_CONFIG}' is missing from ${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}"
        echo "Start requested but ${why}"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "$why" "$GAZEBO_CONFIG_FILE"
        return 1
    fi

    # Record what an empty selection fell back to, so the alphabetical guess
    # in resolve_world_file happens at most once and the device can see which
    # world is actually loaded.
    if [[ -z "$GAZEBO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
        update_yaml_value GAZEBO_CURRENT_ENVIRONMENT_CONFIG "$(basename "$world_file")" "$GAZEBO_CONFIG_FILE"
    fi

    echo "Launching Gazebo with world: ${world_file}"
    update_yaml_value GAZEBO_STATE "starting" "$GAZEBO_CONFIG_FILE"

    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${GAZEBO_ROBOT_CONFIGS_FOLDER}
    export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${GAZEBO_ENVIRONMENT_CONFIGS_FOLDER}
    gazebo --verbose "$world_file" &
    gazebo_pid=$!
    update_yaml_value GAZEBO_PID "$gazebo_pid" "$GAZEBO_CONFIG_FILE"

    sleep "$LAUNCH_SETTLE_SECONDS"
    if kill -0 "$gazebo_pid" 2>/dev/null; then
        update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
        # Worlds are scenery only -- the robot is spawned in here, so every
        # fresh gzserver needs the current selection put back. A spawn failure
        # is reported (inside ensure_robot_spawned) but deliberately does NOT
        # fail the launch: an empty world the operator can still pick a robot
        # into beats tearing down a Gazebo that came up fine.
        ensure_robot_spawned
        return 0
    fi

    echo "Gazebo exited immediately after launch"
    update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_LAST_ERROR "Gazebo exited immediately after launch" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
    gazebo_pid=0
    return 1
}

# Validate a newly-staged selection without applying it, so a typo is reported
# the moment it is made rather than only once the operator asks to apply.
#
# An invalid stage is cleared, so it cannot sit pending and surprise the next
# apply with a value that was already known bad. A valid one is deliberately
# LEFT in place: staging is inert, and the value has to survive until
# apply_staged_configs consumes it.
#
# The *_checked memos exist because a staged value persists across polls by
# design. Without them this would re-validate, and re-log, once a second for as
# long as something is staged.
staged_env_checked=""
staged_robot_checked=""

function validate_staged_configs(){
    if [[ "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG" != "$staged_env_checked" ]]; then
        staged_env_checked=$GAZEBO_UPDATE_ENVIRONMENT_CONFIG
        if [[ -n "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG" ]]; then
            if validate_environment_config_name "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG"; then
                echo "Environment config staged: '${GAZEBO_UPDATE_ENVIRONMENT_CONFIG}' (applies on start)"
            else
                echo "Staged environment config rejected: ${validate_error}"
                update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
                clear_yaml_field GAZEBO_UPDATE_ENVIRONMENT_CONFIG
                staged_env_checked=""
            fi
        fi
    fi

    if [[ "$GAZEBO_UPDATE_ROBOT_CONFIG" != "$staged_robot_checked" ]]; then
        staged_robot_checked=$GAZEBO_UPDATE_ROBOT_CONFIG
        if [[ -n "$GAZEBO_UPDATE_ROBOT_CONFIG" ]]; then
            if validate_robot_config_name "$GAZEBO_UPDATE_ROBOT_CONFIG"; then
                echo "Robot config staged: '${GAZEBO_UPDATE_ROBOT_CONFIG}' (applies on start)"
            else
                echo "Staged robot config rejected: ${validate_error}"
                update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
                clear_yaml_field GAZEBO_UPDATE_ROBOT_CONFIG
                staged_robot_checked=""
            fi
        fi
    fi
}

# Promote staged selections into the CURRENT_* pair, and report what changed so
# the caller can pick the cheapest way to make it real.
#
# Sets applied_env_changed / applied_robot_changed / applied_previous_robot,
# which are globals rather than a return value because bash functions can only
# return a status, and the caller needs three facts. They are reset here on
# every call, so a stale value from an earlier apply can never leak into a
# later decision.
applied_env_changed=0
applied_robot_changed=0
applied_previous_robot=""

function apply_staged_configs(){
    applied_env_changed=0
    applied_robot_changed=0
    applied_previous_robot=$GAZEBO_CURRENT_ROBOT_CONFIG

    # Re-validated at apply time, not trusted from staging: the folder is
    # synced from NEPI storage independently of this service, so a config that
    # validated when staged can be gone by the time it is applied.
    if [[ -n "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG" ]]; then
        if validate_environment_config_name "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG"; then
            if [[ "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG" != "$GAZEBO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
                applied_env_changed=1
                echo "Applying environment config: '${GAZEBO_CURRENT_ENVIRONMENT_CONFIG}' -> '${GAZEBO_UPDATE_ENVIRONMENT_CONFIG}'"
            fi
            update_yaml_value GAZEBO_CURRENT_ENVIRONMENT_CONFIG "$GAZEBO_UPDATE_ENVIRONMENT_CONFIG" "$GAZEBO_CONFIG_FILE"
        else
            echo "Staged environment config no longer valid at apply time: ${validate_error}"
            update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
        fi
        clear_yaml_field GAZEBO_UPDATE_ENVIRONMENT_CONFIG
        staged_env_checked=""
    fi

    if [[ -n "$GAZEBO_UPDATE_ROBOT_CONFIG" ]]; then
        if validate_robot_config_name "$GAZEBO_UPDATE_ROBOT_CONFIG"; then
            if [[ "$GAZEBO_UPDATE_ROBOT_CONFIG" != "$GAZEBO_CURRENT_ROBOT_CONFIG" ]]; then
                applied_robot_changed=1
                echo "Applying robot config: '${GAZEBO_CURRENT_ROBOT_CONFIG}' -> '${GAZEBO_UPDATE_ROBOT_CONFIG}'"
            fi
            update_yaml_value GAZEBO_CURRENT_ROBOT_CONFIG "$GAZEBO_UPDATE_ROBOT_CONFIG" "$GAZEBO_CONFIG_FILE"
        else
            echo "Staged robot config no longer valid at apply time: ${validate_error}"
            update_yaml_value GAZEBO_LAST_ERROR "$validate_error" "$GAZEBO_CONFIG_FILE"
        fi
        clear_yaml_field GAZEBO_UPDATE_ROBOT_CONFIG
        staged_robot_checked=""
    fi
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
    # "installing" is legacy -- the install step was removed 2026-09-18, but a
    # config file written before that can still carry it, and it must not pin
    # the service to a state nothing will ever clear.
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

    # Selections are STAGED, not applied. A non-empty UPDATE_* is a pending
    # choice and nothing more: it is validated here for immediate feedback, but
    # the running simulation is untouched until GAZEBO_START says to apply.
    #
    # Changed 2026-09-18, from "a non-empty UPDATE_* IS the request, acted on
    # next poll". The old behaviour meant an operator browsing the environment
    # dropdown restarted Gazebo on every pick, before they had decided
    # anything. Staging lets robot and environment be chosen in any order and
    # applied together, as one action, when asked for.
    validate_staged_configs

    # The request flags ARE the request: setting one to 1 -- by the device, or
    # by hand in the config file -- is picked up on the next poll, with no
    # companion timestamp to keep in step. The clear-on-act below is what stops
    # a request re-firing, so nothing else has to be edited for the service to
    # notice one.
    if [[ "$GAZEBO_STOP" -eq 1 || "$GAZEBO_START" -eq 1 ]]; then
        clear_gazebo_last_error

        if [[ "$GAZEBO_STOP" -eq 1 ]]; then
            # Stop wins over a start set in the same tick, and deliberately
            # leaves staged selections alone -- stopping is not a decision to
            # discard what was picked.
            stop_gazebo
            update_yaml_value GAZEBO_STOP 0 "$GAZEBO_CONFIG_FILE"
        elif [[ "$GAZEBO_START" -eq 1 ]]; then
            # This is the apply step: promote whatever is staged, then make the
            # world match it by the cheapest route that actually works.
            apply_staged_configs

            if ! is_gazebo_running; then
                # Nothing running: a plain start already loads the current
                # world and spawns the current robot, so both kinds of change
                # are covered for free.
                start_gazebo
            elif [[ "$applied_env_changed" -eq 1 ]]; then
                # A running gzserver cannot swap worlds in place. The restart
                # re-spawns the current robot, so a simultaneous robot change
                # is carried by this one cycle and needs no swap of its own.
                echo "Restarting Gazebo against the new environment config"
                stop_gazebo
                start_gazebo
            elif [[ "$applied_robot_changed" -eq 1 ]]; then
                # Robot-only change against a world that is staying: just swap
                # the spawned model, no restart.
                despawn_robot_config "$applied_previous_robot"
                spawn_robot_config "$GAZEBO_CURRENT_ROBOT_CONFIG"
            else
                # Already running and nothing changed -- reaffirm state so a
                # stale value cannot stick.
                #
                # Also make sure the robot is actually there. This is the path
                # nepi_gazebo_start.sh's own inline launch lands on: it starts
                # Gazebo itself, sets GAZEBO_START, and leaves this service to
                # reattach to the PID it recorded. Without this the boot path
                # would come up with a scenery-only world and no robot, which
                # is exactly what happened when robots moved out of the .world
                # files.
                update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
                ensure_robot_spawned
            fi

            update_yaml_value GAZEBO_START 0 "$GAZEBO_CONFIG_FILE"
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
