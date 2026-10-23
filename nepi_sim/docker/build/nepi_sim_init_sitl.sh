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

# Image build step: write ArduCopter's default parameters into eeprom.bin, the
# same way step 4 of nepi_gazebo_setup.sh does on a native machine. The SITL
# binary runs headless with -w (wipe) just long enough to send a heartbeat --
# by then the wiped EEPROM has been rewritten with the copter defaults -- and
# is stopped. eeprom.bin is written to the cwd, ArduCopter/, which is where
# sim_vehicle.py looks for it when the Gazebo service runs it from there.
#
# Unlike the native step, a failure here fails the image build: an image
# without initialized parameters would only show it much later, as a
# quadcopter that never arms.

ARDUPILOT_FOLDER=${HOME}/ardupilot
ARDUCOPTER_BIN=${ARDUPILOT_FOLDER}/build/sitl/bin/arducopter

if [[ ! -x "$ARDUCOPTER_BIN" ]]; then
    echo "ERROR: ArduCopter SITL binary not found at ${ARDUCOPTER_BIN}"
    exit 1
fi

cd "${ARDUPILOT_FOLDER}/ArduCopter" || exit 1
rm -f eeprom.bin

"$ARDUCOPTER_BIN" -S -w --model + --speedup 1 \
    --defaults "${ARDUPILOT_FOLDER}/Tools/autotest/default_params/copter.parm" \
    --sim-address=127.0.0.1 -I0 </dev/null >/tmp/nepi_sim_init_sitl.log 2>&1 &
sitl_pid=$!

python3 - <<'EOF'
import sys, time
from pymavlink import mavutil
deadline = time.time() + 60
while True:
    try:
        conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760', autoreconnect=False)
        break
    except Exception:
        if time.time() > deadline:
            sys.exit(1)
        time.sleep(1)
sys.exit(0 if conn.wait_heartbeat(timeout=60) else 1)
EOF
heartbeat_rc=$?

kill "$sitl_pid" 2>/dev/null
wait "$sitl_pid" 2>/dev/null

if [[ $heartbeat_rc -ne 0 || ! -f eeprom.bin ]]; then
    echo "ERROR: SITL parameter initialization did not complete"
    cat /tmp/nepi_sim_init_sitl.log
    exit 1
fi

rm -f /tmp/nepi_sim_init_sitl.log
echo "SITL parameter initialization complete"
