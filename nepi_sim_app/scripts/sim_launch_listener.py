#!/usr/bin/env python3
"""Tiny local trigger for launching the full SITL/Gazebo sim stack.

Listens on 127.0.0.1:<port> and, on every connection, fires sitl_gazebo_full()
(see nepi_sitl_dev_env.sh) as a detached subprocess, then replies OK/ERR
immediately -- it does NOT wait for the stack to actually finish coming up
(sitl_gazebo_full takes 15-20s and itself backgrounds several processes), it
only confirms the trigger was accepted. sitl_gazebo_full is idempotent and
checks each piece (Gazebo/SITL, gz_reset_listener, nepi_tunnel, the camera-rig
and AI-targeting controllers) individually, so this one listener correctly
handles "nothing running yet", "some pieces already up", and "everything
already up" the same way.

Exists so the NEPI device -- which runs sim_ai_targeting_bridge_script.py, on
the other side of the same reverse SSH tunnel gz_reset_listener already uses
-- can get the whole sim stack running without needing its own SSH creds back
to this VM. Requires this VM to have run sitl_gazebo/sitl_gazebo_full at least
once already (this listener is started by those functions, not at boot) --
from a genuinely cold VM with nothing running at all, there's nothing here yet
to receive the trigger.

Installed to ~/.local/bin/sim_launch_listener.py on the dev VM and launched by
the sim_launch_listener function in nepi_sitl_dev_env.sh (see that file) --
NOTE this script gets copied there, so it can't find nepi_sitl_dev_env.sh
relative to its own location once installed. The caller (the bash function)
passes that script's real path explicitly as the second argument instead.

Usage: sim_launch_listener.py [port] [path-to-nepi_sitl_dev_env.sh]
"""

import os
import socket
import subprocess
import sys

DEFAULT_PORT = 9028

# Fallback for manual/direct invocation without the second argument -- matches
# this file's own known repo location. The bash function always passes the
# real path explicitly (see sim_launch_listener() in nepi_sitl_dev_env.sh), so
# this is only hit when running this script by hand for a quick test.
DEFAULT_DEV_ENV_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nepi_sitl_dev_env.sh')


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    dev_env_sh = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_DEV_ENV_SH
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    print(f"sim_launch_listener listening on 127.0.0.1:{port}, using {dev_env_sh}", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            subprocess.Popen(
                ['bash', '-c', f'source "{dev_env_sh}" && sitl_gazebo_full'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            reply = b'OK triggered\n'
        except Exception as exc:
            reply = ('ERR ' + str(exc) + '\n').encode()
        try:
            conn.sendall(reply)
        finally:
            conn.close()


if __name__ == '__main__':
    main()
