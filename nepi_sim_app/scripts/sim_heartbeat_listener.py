#!/usr/bin/env python3
"""Tiny liveness pinger for the generic-rover Gazebo simulation.

Name kept for history -- this is a CLIENT now, not a listener. Every
CHECK_INTERVAL_SEC it checks whether gzserver is ACTUALLY still running
(see gzserver_is_alive() below) and, only if so, dials out to the NEPI
device's own heartbeat listener (rbx_sim_discovery.py's
_startHeartbeatListener) and sends ALIVE. Deliberately NOT a ROS node: the
remote NEPI device and this dev VM run separate ROS masters, so the NEPI
rbx_sim driver's discovery cannot see this VM's ROS graph (no
/sim/heartbeat topic) -- a raw TCP ping stands in instead.

2026-09-08 -- DIRECTION REVERSED (see rbx_sim_discovery.py's own comment
for the full reasoning): this used to be a server the device dialed into,
relying on a reverse SSH tunnel to make that inbound connection possible.
That requires the VM to accept an unsolicited inbound connection, which a
very common real setup -- Windows + WSL2 -- blocks by default even once
mirrored networking makes the VM LAN-addressable (Windows Firewall's
Public-profile default), and asking every operator to add a firewall
exception doesn't scale. Outbound is never blocked, so this now dials the
device instead of waiting to be dialed -- no tunnel, no firewall config, on
any OS.

Checks gzserver's real liveness (2026-08-26, unchanged by the above)
rather than unconditionally pinging ALIVE just because this process itself
is running -- this pinger and gzserver are separate processes, started
together by launch_command but with no guaranteed teardown coupling after
that: a gzserver crash, an operator manually killing just gzserver/
gzclient (e.g. to debug something), or any stop path that doesn't happen
to hit this process's own PID all leave it running and happily able to lie
"ALIVE" forever. Reported live: "even though the rover is killed in
gazebo, it still shows it in robots... this is a recurring issue" --
rbx_sim_discovery.py's own liveness check (checkForSimDevice) is only as
honest as whether a ping actually arrives, so a stale "still running"
belief here means a stale robot entry that never clears from Devices ->
Robots no matter how long gzserver has been gone.

Started and stopped by the sim_heartbeat_listener function in
sim_rover_dev_env.sh as part of sim_rover_gazebo, alongside roscore, Gazebo,
and sim_bridge_node.py -- so a ping arriving USED TO mean only "the port
answers"; now (both before and after the direction reversal) it means the
stack was launched at some point AND gzserver is still actually alive right
now. Modeled on gz_reset_listener.py (same pattern, ArduPilot workflow).
"""

import os
import socket
import subprocess
import sys
import threading
import time

DEFAULT_PORT = 9022

# The NEPI device's own reachable address -- same env var and same default
# ("nepi", meant to resolve via whatever ~/.ssh/config / /etc/hosts entry
# the operator's one-time device SSH setup already created) as
# nepi_tunnel()'s device_host in nepi_sitl_dev_env.sh, reused rather than
# inventing a second variable for the same machine.
DEVICE_HOST = os.environ.get('NEPI_DEVICE_SSH_HOST', 'nepi')

# How often to ping when alive. Well under rbx_sim_discovery.py's
# HEARTBEAT_LISTEN_TIMEOUT_SEC (6s) so one dropped ping or one slow
# connection attempt doesn't read as "gone".
PING_INTERVAL_SEC = 2.0

SIM_ALIVE_PING = b'ALIVE\n'

# World-file substring gazserver_is_alive() requires in a candidate
# process's OWN argv -- scoped to this specific world, not a bare
# "gzserver", so an unrelated Gazebo instance (a different launch target)
# can't produce a false ALIVE here. Matches gazebo_rover's own
# ready_check_command in simulator_launch_targets.yaml.
GZSERVER_WORLD_FILE_MARKER = 'generic_rover.world'

# How often the background thread below re-checks gzserver's liveness --
# fast enough that a genuine gzserver death is reflected within about a
# second (matching this listener's whole reason for existing, see the
# module docstring's "stale ALIVE reply" story), cheap enough that a burst
# of near-simultaneous heartbeat probes never triggers more than one pgrep
# fork per second.
CHECK_INTERVAL_SEC = 1.0

_alive_lock = threading.Lock()
_alive = False


def gzserver_is_alive():
    # `pgrep -f 'gzserver.*generic_rover.world'` (the original approach here)
    # matches the FULL COMMAND LINE of every process as one regex -- and the
    # launch_command wrapper script that starts gzserver in the first place
    # matches its OWN pattern: its own text contains "gzserver" (from its own
    # `pgrep -x gzserver` guard) followed somewhere later by
    # "generic_rover.world" (from the actual launch command), so the whole
    # multi-hundred-character wrapper script counts as a match. Since that
    # wrapper stays alive (blocked in its own `wait`) for as long as ANY of
    # gzserver/this listener/camera_rig_controller/sim_bridge_node are still
    # running, this made a real gzserver crash or an operator closing just
    # the Gazebo window invisible to this check for as long as those
    # siblings kept running -- confirmed live (2026-09-01): "closed the
    # gazebo app but it still shows... on robots" while gzserver was
    # genuinely gone and only the wrapper + this process + its two siblings
    # were still up. `pgrep -x gzserver` (matching the bare process NAME,
    # not a full-cmdline regex -- the same check the wrapper script's own
    # guard already uses) can never match a bash process, so this reads each
    # exact-name candidate's own argv directly instead of trusting pgrep's
    # own substring search across the whole command line.
    try:
        result = subprocess.run(
            ['pgrep', '-x', 'gzserver'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
        )
        if result.returncode != 0:
            return False
        for pid in result.stdout.split():
            try:
                with open('/proc/' + pid + '/cmdline', 'rb') as f:
                    cmdline = f.read().decode('utf-8', errors='replace')
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
            if GZSERVER_WORLD_FILE_MARKER in cmdline:
                return True
        return False
    except Exception:
        return False


def _livenessLoop():
    global _alive
    while True:
        result = gzserver_is_alive()
        with _alive_lock:
            _alive = result
        time.sleep(CHECK_INTERVAL_SEC)


def _pingOnce(port):
    with _alive_lock:
        alive = _alive
    if not alive:
        # Send nothing at all this cycle -- rbx_sim_discovery.py's
        # checkForSimDevice reads "no recent ping" as "not alive", the same
        # conclusion a failed/empty reply used to produce, so simply
        # skipping the dial is enough; no need to connect just to say
        # nothing.
        return
    try:
        with socket.create_connection((DEVICE_HOST, port), timeout = 3) as sock:
            sock.sendall(SIM_ALIVE_PING)
    except Exception:
        # Device listener not up yet / momentarily unreachable -- harmless,
        # matches every other reconnect-style loop in this codebase (e.g.
        # sim_bridge_node.py's own dial loop); just try again next cycle.
        pass


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    threading.Thread(target=_livenessLoop, daemon=True).start()
    print(f"sim_heartbeat_listener pinging {DEVICE_HOST}:{port} every {PING_INTERVAL_SEC}s", flush=True)
    while True:
        _pingOnce(port)
        time.sleep(PING_INTERVAL_SEC)


if __name__ == '__main__':
    main()
