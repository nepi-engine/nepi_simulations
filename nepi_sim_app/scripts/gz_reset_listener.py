#!/usr/bin/env python3
"""Tiny local trigger for resetting the running Gazebo sim.

Listens on 127.0.0.1:<port> and runs `gz world -w default -o` (reset model
poses only) on every connection received, then replies OK/ERR. Deliberately
NOT `-r`/`--reset-all`: that also resets sim time, and ArduPilot's SITL binary
-- actively connected via the FDM socket -- sees that as a time discontinuity
and crashes. Resetting poses alone leaves sim time monotonic and ArduPilot
stays alive.

Exists so the NEPI RBX ArduPilot driver -- which runs on the remote NEPI
device, not this VM -- can reach across the existing reverse SSH tunnel and
trigger a Gazebo reset without needing its own SSH credentials back to here.

Installed to ~/.local/bin/gz_reset_listener.py on the dev VM and launched by
the gz_reset_listener function in nepi_sitl_dev_env.sh (see that file).
"""

import socket
import subprocess
import sys

DEFAULT_PORT = 9021


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    print(f"gz_reset_listener listening on 127.0.0.1:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            result = subprocess.run(
                ['gz', 'world', '-w', 'default', '-o'],
                capture_output=True, text=True, timeout=10,
            )
            reply = b'OK\n' if result.returncode == 0 else (b'ERR\n' + result.stderr.encode())
        except Exception as exc:
            reply = ('ERR\n' + str(exc)).encode()
        try:
            conn.sendall(reply)
        finally:
            conn.close()


if __name__ == '__main__':
    main()
