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

# NEPI Gazebo camera relay for the ArduPilot quadcopter. Runs on the VM and
# connects nepi_gazebo_quad_cameras.py's camera server (127.0.0.1:9026 here)
# to the device RBX_ARDUPILOT_SITL driver's CameraBridgeDeviceRelay (device
# port 9032), which re-exposes it on the device's own 127.0.0.1:9026 where
# the driver's camera thread connects. A plain bidirectional TCP byte relay:
# images flow to the device, camera settings flow back.
#
# Started and stopped by nepi_gazebo.sh, never by hand, alongside SITL for a
# robot config whose nepi_robot_config.yaml names nepi_driver: rbx_ardupilot.
# This side always connects, so the VM never has to accept an inbound
# connection -- the same direction as nepi_gazebo_bridge.py and
# nepi_gazebo_mavlink_relay.py.
#
# Carried over unchanged from sim_container's camera_bridge_relay_vm.py.
# Either end can come and go; both are reconnected on any failure.

import argparse
import select
import socket
import time

DEFAULT_DEVICE_PORT = 9032
DEFAULT_CAMERA_HOST = '127.0.0.1'
DEFAULT_CAMERA_PORT = 9026
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5

LOG_PREFIX = 'nepi_gazebo_camera_relay: '


def log(msg):
  print(LOG_PREFIX + msg, flush=True)


def pump(a, b):
  a.setblocking(False)
  b.setblocking(False)
  while True:
    r, _, x = select.select([a, b], [], [a, b], 5.0)
    if x:
      return
    for s in r:
      try:
        data = s.recv(65536)
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
  parser = argparse.ArgumentParser(description='NEPI Gazebo camera relay (quad cameras -> NEPI device)')
  parser.add_argument('--device-host', required=True)
  parser.add_argument('--device-port', type=int, default=DEFAULT_DEVICE_PORT)
  parser.add_argument('--camera-host', default=DEFAULT_CAMERA_HOST)
  parser.add_argument('--camera-port', type=int, default=DEFAULT_CAMERA_PORT)
  args = parser.parse_args()

  log('starting, will relay %s:%d <-> %s:%d' %
      (args.camera_host, args.camera_port, args.device_host, args.device_port))

  while True:
    camera_conn = None
    device_conn = None
    try:
      camera_conn = socket.create_connection((args.camera_host, args.camera_port),
                                             timeout=CONNECT_TIMEOUT_SEC)
      device_conn = socket.create_connection((args.device_host, args.device_port),
                                             timeout=CONNECT_TIMEOUT_SEC)
      camera_conn.settimeout(None)
      device_conn.settimeout(None)
      log('connected both ends, relaying')
      pump(camera_conn, device_conn)
      log('link dropped, reconnecting')
    except Exception as e:
      log('connect/relay failed: ' + str(e))
    finally:
      for c in (camera_conn, device_conn):
        if c is not None:
          try:
            c.close()
          except Exception:
            pass
    time.sleep(RECONNECT_INTERVAL_SEC)


if __name__ == '__main__':
  main()
