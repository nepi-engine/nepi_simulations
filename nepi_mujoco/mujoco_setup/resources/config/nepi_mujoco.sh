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

# The NEPI MuJoCo Simulator Management Service. Runs on the sim machine and
# polls nepi_mujoco_config.yaml in NEPI storage for start/stop requests and
# staged robot/world selections written by the device's Sim Connector app,
# then reports MUJOCO_STATE, MUJOCO_PID and MUJOCO_LAST_ERROR back into the
# same file. The same poll / act / report loop as nepi_gazebo.sh, which has the
# longer write-ups of why each part works the way it does.
#
# Simpler than the Gazebo service in one way: the simulation is a single
# process (nepi_mujoco_sim.py) that loads the world and attaches the robot when
# it starts, so there is no spawning into a running world. Changing the robot
# or the world means restarting that process, which takes about a second.
#
# Assumes nepi_mujoco_mount_storage (nepi_mujoco_bash_utils) and NEPI_IP (the
# NEPI bash utils sourced by .bashrc) are already defined. NEPI_IP is where the
# simulation's bridge sends the robot's telemetry. nepi_mujoco_start.sh, the
# normal way this service gets started, sources the MuJoCo utils and mounts
# NEPI storage before backgrounding this script, so the mount call below finds
# the share already mounted and neither prompts nor remounts. It also exports
# NEPISTORAGE_PASSWORD, used only if the share is not mounted when this starts.

MUJOCO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

# The robot and world files the simulation reads come from ${HOME}/mujoco,
# which nepi_mujoco_setup.sh stages and nepi_mujoco_sync.sh keeps reconciled
# with NEPI storage. The config file itself is read straight from storage.
MUJOCO_HOME_FOLDER=${HOME}/mujoco
MUJOCO_ENVIRONMENT_CONFIGS_FOLDER=${MUJOCO_HOME_FOLDER}/ENVIRONMENT_CONFIGS
MUJOCO_ROBOT_CONFIGS_FOLDER=${MUJOCO_HOME_FOLDER}/ROBOT_CONFIGS

MUJOCO_CONFIG_LOAD_FILE=${MUJOCO_FOLDER}/load_mujoco_config.sh
MUJOCO_SIM_SCRIPT=${MUJOCO_FOLDER}/nepi_mujoco_sim.py
MUJOCO_SIM_LOG=/tmp/nepi_mujoco_sim.log

# A robot whose nepi_robot_config.yaml names nepi_driver: rbx_sim gets the
# simulation's bridge to the device's RBX_SIM driver, which listens on this
# port, UDP (the same port as the Gazebo service's bridge).
MUJOCO_BRIDGE_PORT=9023
MUJOCO_BRIDGE_DRIVER=rbx_sim

POLL_SECONDS=1
# Long enough for the Python import and model compile to fail, if they will.
LAUNCH_SETTLE_SECONDS=3
# How long to let the simulation wind down after a SIGINT before a SIGKILL.
STOP_GRACE_SECONDS=5

if [[ ! -f "$MUJOCO_CONFIG_LOAD_FILE" ]]; then
    echo "Load script not found: ${MUJOCO_CONFIG_LOAD_FILE}"
    exit 1
fi

if [[ ! -f "$MUJOCO_SIM_SCRIPT" ]]; then
    echo "Simulation script not found: ${MUJOCO_SIM_SCRIPT}"
    exit 1
fi

if ! command -v nepi_mujoco_mount_storage >/dev/null 2>&1; then
    echo "nepi_mujoco_mount_storage is not defined -- source nepi_mujoco_bash_utils before running this script"
    exit 1
fi

# Mount NEPI storage once, for the life of this service, and watch the config
# file there directly (same as nepi_gazebo.sh).
ORIG_PWD=$(pwd)
if nepi_mujoco_mount_storage "$NEPISTORAGE_PASSWORD"; then
    MUJOCO_SIM_FOLDER=$(pwd)/databases/sims/mujoco
    cd "$ORIG_PWD"
else
    echo "Failed to reach NEPI storage -- cannot watch nepi_mujoco_config.yaml"
    exit 1
fi

MUJOCO_CONFIG_FILE=${MUJOCO_SIM_FOLDER}/config/nepi_mujoco_config.yaml

if [[ ! -f "$MUJOCO_CONFIG_FILE" ]]; then
    echo "MuJoCo config file not found: ${MUJOCO_CONFIG_FILE}"
    exit 1
fi

# Every config write goes through here, never `yq -i`: the config lives on the
# device's CIFS share, where yq -i truncates the file and copies into it, and
# the device then reads an empty file for 10-20 ms. A temp file beside the
# config, renamed over it, is never seen half-written. (Same as nepi_gazebo.sh.)
function write_yaml_expr(){
    local yaml_file=$2
    local tmp_file="${yaml_file}.svc.tmp"
    if yq e "$1" "$yaml_file" > "$tmp_file" && [[ -s "$tmp_file" ]]; then
        mv -f "$tmp_file" "$yaml_file"
    else
        rm -f "$tmp_file"
        return 1
    fi
}

function update_yaml_value(){
    export UPDATE_YAML_KEY=$1
    export UPDATE_YAML_VAL=$2
    export UPDATE_YAML_FILE=$3

    # env() parses the value as YAML, so text such as an error message with
    # ": " in it fails to write. Numbers keep env() so they stay numbers;
    # everything else is written as a plain string with strenv().
    if [[ "$UPDATE_YAML_VAL" =~ ^-?[0-9]+$ ]]; then
        write_yaml_expr '.'"$UPDATE_YAML_KEY"' = env(UPDATE_YAML_VAL)' "$UPDATE_YAML_FILE"
    else
        write_yaml_expr '.'"$UPDATE_YAML_KEY"' = strenv(UPDATE_YAML_VAL)' "$UPDATE_YAML_FILE"
    fi

    eval "export $UPDATE_YAML_KEY=\"$UPDATE_YAML_VAL\""
}

# yq's env() errors on an empty value, so clearing a field uses a literal.
function clear_yaml_field(){
    write_yaml_expr '.'"$1"' = ""' "$MUJOCO_CONFIG_FILE"
    eval "export $1=''"
}

function is_mujoco_running(){
    [[ "$mujoco_pid" -ne 0 ]] && kill -0 "$mujoco_pid" 2>/dev/null
}

# True when this PID is a nepi_mujoco_sim.py process, so a recorded PID that
# the OS has since given to something else is never adopted.
function is_sim_process(){
    [[ "$1" -ne 0 ]] && kill -0 "$1" 2>/dev/null && \
        tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q "nepi_mujoco_sim.py"
}

# 0 = valid. Sets validate_error on failure.
function validate_environment_config_name(){
    local name=$1
    validate_error=""
    if [[ "$name" != *.xml ]]; then
        validate_error="environment config '${name}' is not an .xml file"
        return 1
    fi
    if [[ ! -f "${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}/${name}" ]]; then
        validate_error="environment config '${name}' not found in ${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}"
        return 1
    fi
    return 0
}

function validate_robot_config_name(){
    local name=$1
    validate_error=""
    if [[ ! -d "${MUJOCO_ROBOT_CONFIGS_FOLDER}/${name}" ]]; then
        validate_error="robot config '${name}' not found in ${MUJOCO_ROBOT_CONFIGS_FOLDER}"
        return 1
    fi
    if [[ ! -f "${MUJOCO_ROBOT_CONFIGS_FOLDER}/${name}/model.xml" ]]; then
        validate_error="robot config '${name}' has no model.xml"
        return 1
    fi
    return 0
}

# Print the world file to load. An empty selection falls back to the first
# .xml in the folder; a recorded name that is gone prints "" rather than
# silently loading a different world.
function resolve_world_file(){
    if [[ -n "$MUJOCO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
        if validate_environment_config_name "$MUJOCO_CURRENT_ENVIRONMENT_CONFIG"; then
            echo "${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}/${MUJOCO_CURRENT_ENVIRONMENT_CONFIG}"
            return 0
        fi
        echo ""
        return 1
    fi
    ls ${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}/*.xml 2>/dev/null | head -n 1
}

# Print the robot config to load, with the same fallback rule.
function resolve_robot_config(){
    if [[ -n "$MUJOCO_CURRENT_ROBOT_CONFIG" ]]; then
        if validate_robot_config_name "$MUJOCO_CURRENT_ROBOT_CONFIG"; then
            echo "$MUJOCO_CURRENT_ROBOT_CONFIG"
            return 0
        fi
        echo ""
        return 1
    fi
    local dir name
    for dir in "${MUJOCO_ROBOT_CONFIGS_FOLDER}"/*/; do
        [[ -d "$dir" ]] || continue
        name=$(basename "$dir")
        if validate_robot_config_name "$name"; then
            echo "$name"
            return 0
        fi
    done
    echo ""
}

function robot_config_driver(){
    local capabilities=${MUJOCO_ROBOT_CONFIGS_FOLDER}/${1}/nepi_robot_config.yaml
    [[ -f "$capabilities" ]] || return 0
    yq e '.nepi_driver // ""' "$capabilities" 2>/dev/null
}

function fail_start(){
    echo "Start failed: $1"
    update_yaml_value MUJOCO_STATE "failed" "$MUJOCO_CONFIG_FILE"
    update_yaml_value MUJOCO_LAST_ERROR "$1" "$MUJOCO_CONFIG_FILE"
    update_yaml_value MUJOCO_PID 0 "$MUJOCO_CONFIG_FILE"
    mujoco_pid=0
}

# Start the simulation with the current world and robot, and report the outcome.
function start_mujoco(){
    local world_file robot
    world_file=$(resolve_world_file)
    if [[ -z "$world_file" ]]; then
        if [[ -n "$MUJOCO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
            fail_start "selected environment config '${MUJOCO_CURRENT_ENVIRONMENT_CONFIG}' is missing from ${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}"
        else
            fail_start "no .xml world file found in ${MUJOCO_ENVIRONMENT_CONFIGS_FOLDER}"
        fi
        return 1
    fi
    robot=$(resolve_robot_config)
    if [[ -z "$robot" ]]; then
        if [[ -n "$MUJOCO_CURRENT_ROBOT_CONFIG" ]]; then
            fail_start "selected robot config '${MUJOCO_CURRENT_ROBOT_CONFIG}' is not usable: ${validate_error}"
        else
            fail_start "no robot config with a model.xml found in ${MUJOCO_ROBOT_CONFIGS_FOLDER}"
        fi
        return 1
    fi

    # Record fallback picks, so the guess happens at most once and the device
    # can see what is actually loaded.
    if [[ -z "$MUJOCO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
        update_yaml_value MUJOCO_CURRENT_ENVIRONMENT_CONFIG "$(basename "$world_file")" "$MUJOCO_CONFIG_FILE"
    fi
    if [[ -z "$MUJOCO_CURRENT_ROBOT_CONFIG" ]]; then
        update_yaml_value MUJOCO_CURRENT_ROBOT_CONFIG "$robot" "$MUJOCO_CONFIG_FILE"
    fi

    local viewer_mode=auto
    [[ "$MUJOCO_VIEWER" == "0" ]] && viewer_mode=off

    # The bridge to the device, for a robot the device drives (same rule as
    # the Gazebo service's sync_robot_bridge).
    local bridge_args=()
    local bridge_note="no bridge"
    if [[ "$(robot_config_driver "$robot")" == "$MUJOCO_BRIDGE_DRIVER" ]]; then
        if [[ -n "${NEPI_IP%%/*}" ]]; then
            bridge_args=(--device-host "${NEPI_IP%%/*}" --port "$MUJOCO_BRIDGE_PORT")
            bridge_note="bridge to ${NEPI_IP%%/*}:${MUJOCO_BRIDGE_PORT}"
        else
            echo "NEPI_IP is not set -- '${robot}' will run without its bridge, so the device cannot drive it"
        fi
    fi

    echo "Starting MuJoCo: world '$(basename "$world_file")', robot '${robot}', viewer ${viewer_mode}, ${bridge_note}"
    update_yaml_value MUJOCO_STATE "starting" "$MUJOCO_CONFIG_FILE"
    python3 -u "$MUJOCO_SIM_SCRIPT" --world-file "$world_file" \
        --robot-dir "${MUJOCO_ROBOT_CONFIGS_FOLDER}/${robot}" --viewer "$viewer_mode" \
        "${bridge_args[@]}" >>"$MUJOCO_SIM_LOG" 2>&1 &
    mujoco_pid=$!
    update_yaml_value MUJOCO_PID "$mujoco_pid" "$MUJOCO_CONFIG_FILE"

    sleep "$LAUNCH_SETTLE_SECONDS"
    if kill -0 "$mujoco_pid" 2>/dev/null; then
        update_yaml_value MUJOCO_STATE "running" "$MUJOCO_CONFIG_FILE"
        return 0
    fi

    # Pass the log's last line on, so the device shows the real reason (a
    # model load error, or a Python error such as mujoco not installed), not
    # just "exited".
    local reason
    reason=$(grep -v '^[[:space:]]*$' "$MUJOCO_SIM_LOG" 2>/dev/null | tail -n 1 | sed 's/^NEPI_MUJOCO_SIM: //')
    fail_start "MuJoCo exited right after start${reason:+: ${reason}} (see ${MUJOCO_SIM_LOG})"
    return 1
}

# Stop the simulation and report idle. No-op if nothing is running.
function stop_mujoco(){
    is_mujoco_running || return 0

    echo "Stopping MuJoCo (pid ${mujoco_pid})"
    update_yaml_value MUJOCO_STATE "stopping" "$MUJOCO_CONFIG_FILE"
    kill -INT "$mujoco_pid" 2>/dev/null

    # An adopted PID is not a child of this shell, so poll rather than wait.
    local _
    for _ in $(seq 1 "$STOP_GRACE_SECONDS"); do
        kill -0 "$mujoco_pid" 2>/dev/null || break
        sleep 1
    done
    kill -0 "$mujoco_pid" 2>/dev/null && kill -KILL "$mujoco_pid" 2>/dev/null

    mujoco_pid=0
    update_yaml_value MUJOCO_PID 0 "$MUJOCO_CONFIG_FILE"
    update_yaml_value MUJOCO_STATE "idle" "$MUJOCO_CONFIG_FILE"
    return 0
}

# Check newly staged selections right away, so a typo is reported when it is
# made. Invalid ones are cleared; valid ones wait for MUJOCO_START. The
# *_checked memos stop a staged value being re-checked every poll.
staged_env_checked=""
staged_robot_checked=""

function validate_staged_configs(){
    if [[ "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG" != "$staged_env_checked" ]]; then
        staged_env_checked=$MUJOCO_UPDATE_ENVIRONMENT_CONFIG
        if [[ -n "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG" ]]; then
            if validate_environment_config_name "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG"; then
                echo "Environment config staged: '${MUJOCO_UPDATE_ENVIRONMENT_CONFIG}' (applies on start)"
            else
                echo "Staged environment config rejected: ${validate_error}"
                update_yaml_value MUJOCO_LAST_ERROR "$validate_error" "$MUJOCO_CONFIG_FILE"
                clear_yaml_field MUJOCO_UPDATE_ENVIRONMENT_CONFIG
                staged_env_checked=""
            fi
        fi
    fi

    if [[ "$MUJOCO_UPDATE_ROBOT_CONFIG" != "$staged_robot_checked" ]]; then
        staged_robot_checked=$MUJOCO_UPDATE_ROBOT_CONFIG
        if [[ -n "$MUJOCO_UPDATE_ROBOT_CONFIG" ]]; then
            if validate_robot_config_name "$MUJOCO_UPDATE_ROBOT_CONFIG"; then
                echo "Robot config staged: '${MUJOCO_UPDATE_ROBOT_CONFIG}' (applies on start)"
            else
                echo "Staged robot config rejected: ${validate_error}"
                update_yaml_value MUJOCO_LAST_ERROR "$validate_error" "$MUJOCO_CONFIG_FILE"
                clear_yaml_field MUJOCO_UPDATE_ROBOT_CONFIG
                staged_robot_checked=""
            fi
        fi
    fi
}

# Promote staged selections into CURRENT_*. Sets applied_changed=1 when either
# one changed, since either change needs a restart here.
applied_changed=0

function apply_staged_configs(){
    applied_changed=0
    if [[ -n "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG" ]]; then
        # Re-checked at apply time: the folder is synced independently of
        # this service, so a file valid when staged can be gone now.
        if validate_environment_config_name "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG"; then
            if [[ "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG" != "$MUJOCO_CURRENT_ENVIRONMENT_CONFIG" ]]; then
                applied_changed=1
                echo "Applying environment config: '${MUJOCO_CURRENT_ENVIRONMENT_CONFIG}' -> '${MUJOCO_UPDATE_ENVIRONMENT_CONFIG}'"
            fi
            update_yaml_value MUJOCO_CURRENT_ENVIRONMENT_CONFIG "$MUJOCO_UPDATE_ENVIRONMENT_CONFIG" "$MUJOCO_CONFIG_FILE"
        else
            echo "Staged environment config no longer valid at apply time: ${validate_error}"
            update_yaml_value MUJOCO_LAST_ERROR "$validate_error" "$MUJOCO_CONFIG_FILE"
        fi
        clear_yaml_field MUJOCO_UPDATE_ENVIRONMENT_CONFIG
        staged_env_checked=""
    fi

    if [[ -n "$MUJOCO_UPDATE_ROBOT_CONFIG" ]]; then
        if validate_robot_config_name "$MUJOCO_UPDATE_ROBOT_CONFIG"; then
            if [[ "$MUJOCO_UPDATE_ROBOT_CONFIG" != "$MUJOCO_CURRENT_ROBOT_CONFIG" ]]; then
                applied_changed=1
                echo "Applying robot config: '${MUJOCO_CURRENT_ROBOT_CONFIG}' -> '${MUJOCO_UPDATE_ROBOT_CONFIG}'"
            fi
            update_yaml_value MUJOCO_CURRENT_ROBOT_CONFIG "$MUJOCO_UPDATE_ROBOT_CONFIG" "$MUJOCO_CONFIG_FILE"
        else
            echo "Staged robot config no longer valid at apply time: ${validate_error}"
            update_yaml_value MUJOCO_LAST_ERROR "$validate_error" "$MUJOCO_CONFIG_FILE"
        fi
        clear_yaml_field MUJOCO_UPDATE_ROBOT_CONFIG
        staged_robot_checked=""
    fi
}

echo ""
echo "##########################"
echo "*** STARTING NEPI MUJOCO SERVICE ***"
echo "##########################"
echo ""
echo "Watching config file: ${MUJOCO_CONFIG_FILE}"

source "$MUJOCO_CONFIG_LOAD_FILE" "$MUJOCO_CONFIG_FILE" > /dev/null 2>&1

# Reattach to a simulation left running by an earlier run of this service.
mujoco_pid=0
if is_sim_process "$MUJOCO_PID"; then
    echo "Reattaching to already-running MuJoCo process ${MUJOCO_PID}"
    mujoco_pid=$MUJOCO_PID
elif [[ "$MUJOCO_STATE" == "running" || "$MUJOCO_STATE" == "starting" || "$MUJOCO_STATE" == "stopping" ]]; then
    echo "Recorded state was '${MUJOCO_STATE}' but no simulation is running -- resetting to idle"
    update_yaml_value MUJOCO_STATE "idle" "$MUJOCO_CONFIG_FILE"
    update_yaml_value MUJOCO_PID 0 "$MUJOCO_CONFIG_FILE"
fi

# nepi_mujoco_stop (nepi_mujoco_bash_utils) stops this service with a SIGTERM;
# the simulation goes down with it through stop_mujoco, the same as a device
# stop. Same as nepi_gazebo.sh's stop_service.
function stop_service(){
    trap '' TERM INT
    echo "Service stop requested -- stopping the simulation"
    stop_mujoco
    echo "Service stopped"
    exit 0
}
trap stop_service TERM INT

while true; do
    source "$MUJOCO_CONFIG_LOAD_FILE" "$MUJOCO_CONFIG_FILE" > /dev/null 2>&1

    validate_staged_configs

    if [[ "$MUJOCO_STOP" -eq 1 || "$MUJOCO_START" -eq 1 ]]; then
        write_yaml_expr '.MUJOCO_LAST_ERROR = ""' "$MUJOCO_CONFIG_FILE"

        if [[ "$MUJOCO_STOP" -eq 1 ]]; then
            # Stop wins over a start in the same tick, and leaves staged
            # selections alone.
            stop_mujoco
            update_yaml_value MUJOCO_STOP 0 "$MUJOCO_CONFIG_FILE"
        elif [[ "$MUJOCO_START" -eq 1 ]]; then
            apply_staged_configs
            if ! is_mujoco_running; then
                start_mujoco
            elif [[ "$applied_changed" -eq 1 ]]; then
                echo "Restarting MuJoCo with the new selection"
                stop_mujoco
                start_mujoco
            else
                # Already running what was asked for -- reaffirm the state.
                update_yaml_value MUJOCO_STATE "running" "$MUJOCO_CONFIG_FILE"
            fi
            update_yaml_value MUJOCO_START 0 "$MUJOCO_CONFIG_FILE"
        fi
    fi

    # Catch a crash even without a new device request.
    if [[ "$mujoco_pid" -ne 0 ]] && ! kill -0 "$mujoco_pid" 2>/dev/null; then
        echo "MuJoCo process ${mujoco_pid} is no longer running"
        update_yaml_value MUJOCO_STATE "failed" "$MUJOCO_CONFIG_FILE"
        update_yaml_value MUJOCO_LAST_ERROR "MuJoCo exited unexpectedly (see ${MUJOCO_SIM_LOG})" "$MUJOCO_CONFIG_FILE"
        update_yaml_value MUJOCO_PID 0 "$MUJOCO_CONFIG_FILE"
        mujoco_pid=0
    fi

    update_yaml_value MUJOCO_LAST_POLL "$(date +%s)" "$MUJOCO_CONFIG_FILE"

    sleep "$POLL_SECONDS"
done
