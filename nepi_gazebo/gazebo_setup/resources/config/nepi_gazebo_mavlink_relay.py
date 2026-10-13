#!/usr/bin/env python3
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

# NEPI Gazebo MAVLink relay. Runs on the VM next to ArduPilot SITL and
# connects SITL's MAVLink output to the NEPI device's RBX_ARDUPILOT_SITL
# driver (nepi_drivers' rbx_ardupilot_discovery.py / rbx_ardupilot_node.py).
#
# Started and stopped by nepi_gazebo.sh, never by hand: the service runs one
# of these, together with SITL, while Gazebo is up with a robot config whose
# nepi_robot_config.yaml names nepi_driver: rbx_ardupilot, and stops both with
# Gazebo or when the robot changes.
#
# A plain bidirectional TCP byte relay -- MAVLink is opaque here. It connects
# to SITL's dedicated MAVLink output (MAVProxy --out=tcpin:0.0.0.0:5771) and
# dials OUT to the device's SitlMavlinkRelay on port 9031, which re-exposes
# the link on the device's own 127.0.0.1:5771, where ArduPilot discovery
# probes for it and mavros connects. This side always connects, so the VM
# never has to accept an inbound connection -- the same direction the rover
# bridge (nepi_gazebo_bridge.py) uses.
#
# Carried over unchanged from sim_container's mavlink_relay_vm.py, which the
# old gazebo_quadcopter launch target ran. Either end can come and go
# independently of this script; both are reconnected on any failure.

import argparse
import select
import socket
import time

DEFAULT_DEVICE_PORT = 9031
DEFAULT_SITL_HOST = '127.0.0.1'
DEFAULT_SITL_PORT = 5771
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5

LOG_PREFIX = 'nepi_gazebo_mavlink_relay: '


def log(msg):
  print(LOG_PREFIX + msg, flush=True)


def pump(a, b):
  # Move bytes both ways until either side closes or errors.
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
  parser = argparse.ArgumentParser(description='NEPI Gazebo MAVLink relay (SITL -> NEPI device)')
  parser.add_argument('--device-host', required=True)
  parser.add_argument('--device-port', type=int, default=DEFAULT_DEVICE_PORT)
  parser.add_argument('--sitl-host', default=DEFAULT_SITL_HOST)
  parser.add_argument('--sitl-port', type=int, default=DEFAULT_SITL_PORT)
  args = parser.parse_args()

  log('starting, will relay %s:%d <-> %s:%d' %
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
