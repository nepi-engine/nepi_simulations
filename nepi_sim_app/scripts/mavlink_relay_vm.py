#!/usr/bin/env python3
"""VM-side half of a plain TCP relay that makes ArduCopter SITL's MAVLink
output reachable at 127.0.0.1:5771 on the NEPI device again, WITHOUT a
reverse SSH tunnel.

Why this exists: rbx_ardupilot_discovery.py's SITL path (sitl_addr_list=
['127.0.0.1'], sitl_tcp_port_list=['5771']) and its launchSitlDeviceNode's
own mavros fcu_url ("tcp://127.0.0.1:5771") were both written assuming
SITL runs locally alongside the RBX driver, or is made to look that way by
a reverse SSH tunnel forwarding the VM's real 5771 back to the device's own
loopback (see docs/SIM_VM_CONNECTION_SETUP.md's pre-2026-09 setup). Once
every bridge in this app switched to dialing OUT from the VM to the device
instead (see sim_bridge_node.py's own module docstring for that direction
reversal), nothing forwarded 5771 anymore -- confirmed live (2026-09-08):
rbx_ardupilot_discovery's own log showed "Did not find TCP device on ip
address: 127.0.0.1 port: 5771" in an endless loop the whole time a real
quadcopter sim was up and running fine on the VM, so no RBX device (and
therefore no images, no Settings, nothing) ever appeared for it -- the
generic ArduPilot driver was never told anything changed.

This script is the VM side of the fix, matching the same "VM dials device"
direction every other bridge here already uses: connects to ArduCopter
SITL's own dedicated MAVLink --out port (5771, launched by
simulator_launch_targets.yaml's gazebo_quadcopter launch_command) and
dials OUT to the device's own SitlMavlinkRelay (see
rbx_ardupilot_discovery.py), which re-exposes whatever arrives as a plain
local TCP server on the device's own 127.0.0.1:5771 -- exactly the address
discovery and mavros already expect, so neither needed to change at all.

Reconnects both ends on any failure, same retry shape sim_bridge_node.py's
own bridgeServerLoop already uses -- SITL/the device relay can each come
and go independently of this script's own lifetime.
"""
import argparse
import select
import socket
import sys
import time

DEFAULT_DEVICE_HOST = 'nepi'
DEFAULT_DEVICE_PORT = 9031
DEFAULT_SITL_HOST = '127.0.0.1'
DEFAULT_SITL_PORT = 5771
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5


def log(msg):
  print('mavlink_relay_vm: ' + msg, flush=True)


def pump(a, b):
  # Plain bidirectional byte relay -- MAVLink is opaque to this script, no
  # framing/parsing needed, just move bytes both ways until either side
  # closes or errors.
  a.setblocking(False)
  b.setblocking(False)
  while True:
    r, _, x = select.select([a, b], [], [a, b], 5.0)
    if x:
      return
    for s in r:
      try:
        data = s.recv(4096)
      except BlockingIOError:
        continue
      except Exception:
        return
      if not data:
        return
      dst = b if s is a else a
      try:
        dst.sendall(data)
      except Exception:
        return


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--device-host', default=DEFAULT_DEVICE_HOST)
  parser.add_argument('--device-port', type=int, default=DEFAULT_DEVICE_PORT)
  parser.add_argument('--sitl-host', default=DEFAULT_SITL_HOST)
  parser.add_argument('--sitl-port', type=int, default=DEFAULT_SITL_PORT)
  args = parser.parse_args()

  log('starting, will bridge %s:%d <-> %s:%d' %
      (args.sitl_host, args.sitl_port, args.device_host, args.device_port))

  while True:
    sitl_conn = None
    device_conn = None
    try:
      sitl_conn = socket.create_connection((args.sitl_host, args.sitl_port),
                                            timeout=CONNECT_TIMEOUT_SEC)
      device_conn = socket.create_connection((args.device_host, args.device_port),
                                              timeout=CONNECT_TIMEOUT_SEC)
      sitl_conn.settimeout(None)
      device_conn.settimeout(None)
      log('connected both ends, relaying')
      pump(sitl_conn, device_conn)
      log('link dropped, reconnecting')
    except Exception as e:
      log('connect/relay failed: ' + str(e))
    finally:
      for c in (sitl_conn, device_conn):
        if c is not None:
          try:
            c.close()
          except Exception:
            pass
    time.sleep(RECONNECT_INTERVAL_SEC)


if __name__ == '__main__':
  main()
