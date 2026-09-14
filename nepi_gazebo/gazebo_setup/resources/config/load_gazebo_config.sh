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


# Gazebo config loader: exports the key/values of nepi_gazebo_config.yaml as
# shell environment variables, with automatic backup-on-success and
# restore-from-backup-on-failure.
#
# Usage: source load_gazebo_config.sh [CONFIG_FILE]
#   CONFIG_FILE defaults to nepi_gazebo_config.yaml alongside this script.

CONFIG_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

LOAD_SCRIPT=${CONFIG_FOLDER}/load_gazebo_config.py

CONFIG_FILE="${1:-${CONFIG_FOLDER}/nepi_gazebo_config.yaml}"
BACKUP_FILE="${CONFIG_FILE}.bak"

if [[ -z "$CONFIG_FILE" || ! -f "$CONFIG_FILE" ]]; then
    echo "Config file not found: ${CONFIG_FILE}"
    return 1 2>/dev/null || exit 1
fi

if [[ ! -f "$LOAD_SCRIPT" ]]; then
    echo "Load script not found ${LOAD_SCRIPT}"
    return 1 2>/dev/null || exit 1
fi

echo "Loading config file ${CONFIG_FILE}"

success=0
# One KEY=VALUE per line from load_gazebo_config.py (see its own comment) --
# read line-by-line via process substitution (not a `| while`, which would
# run the loop in a subshell and lose $success once it exits) and export
# each as a single argument, splitting only on the FIRST "=" so a value
# containing spaces (GAZEBO_LAST_ERROR messages, in practice) survives
# intact instead of being word-split into several bogus entries.
while IFS= read -r entry; do
    [[ -z "$entry" ]] && continue
    export "${entry%%=*}=${entry#*=}"
done < <(python3 $LOAD_SCRIPT $CONFIG_FILE)
echo "Load returned success=${success}"

if [[ $success -eq 2 ]]; then
    echo "Config file was invalid but values were recovered from backup"
    echo "Restoring config file from backup..."
    cp "$BACKUP_FILE" "$CONFIG_FILE"
elif [[ $success -ne 1 ]]; then
    success=0
    echo "Config file failed to load"
    echo "Checking for backup file..."

    if [[ -f "$BACKUP_FILE" ]]; then
        echo "Backup file exists, restoring config file"
        cp "$BACKUP_FILE" "$CONFIG_FILE"
        success=0
        while IFS= read -r entry; do
            [[ -z "$entry" ]] && continue
            export "${entry%%=*}=${entry#*=}"
        done < <(python3 $LOAD_SCRIPT $CONFIG_FILE)
        echo "Backup load returned success=${success}"
        if [[ "$success" -ne 1 ]]; then
            echo "Failed to load config file from backup"
            return 1 2>/dev/null || exit 1
        fi
    else
        echo "Backup file does not exist"
        return 1 2>/dev/null || exit 1
    fi
fi

if [[ $success -eq 1 ]]; then
    echo "Backing up config file..."
    cp "$CONFIG_FILE" "$BACKUP_FILE"
fi

echo "Finished load process"
