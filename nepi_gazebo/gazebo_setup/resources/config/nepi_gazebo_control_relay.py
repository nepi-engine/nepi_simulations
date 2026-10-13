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

# NEPI Gazebo control relay for the ArduPilot quadcopter. Runs on the VM and
# carries the device RBX_ARDUPILOT_SITL driver's one-shot control signals to
# their VM-side listeners:
#
#   reset          device 9034 -> 127.0.0.1:9021  nepi_gazebo_reset_listener.py
#   teardown       device 9035 -> 127.0.0.1:9029  (follow-mission demo only)
#   start_trigger  device 9036 -> 127.0.0.1:9030  (follow-mission demo only)
#
# The device ports match RESET_SIM/TEARDOWN/START_TRIGGER_DEVICE_RELAY in
# rbx_ardupilot_node.py. Only reset has a listener under the service today;
# the other two wait idle and log a failed dial if ever triggered, until the
# AI-targeting controller they belong to is carried over too.
#
# Started and stopped by nepi_gazebo.sh, never by hand, alongside SITL for a
# robot config whose nepi_robot_config.yaml names nepi_driver: rbx_ardupilot.
#
# Event-driven, not a continuous pump: each relay holds one connection to the
# device and only dials its local listener once the device sends TRIGGER\n,
# the device-side sign that a real request (e.g. reset_sim()) just arrived.
# The local listeners act on the connection itself, so dialing them on a
# reconnect timer would fire a reset nobody asked for.
#
# Carried over unchanged from sim_container's sim_control_relay_vm.py.

import argparse
import select
import socket
import threading
import time

TRIGGER_MARKER = b'TRIGGER\n'

# (label, local_host, local_port, device_port)
RELAYS = [
    ('reset', '127.0.0.1', 9021, 9034),
    ('teardown', '127.0.0.1', 9029, 9035),
    ('start_trigger', '127.0.0.1', 9030, 9036),
]
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5

LOG_PREFIX = 'nepi_gazebo_control_relay'


def log(label, msg):
  print('%s[%s]: %s' % (LOG_PREFIX, label, msg), flush=True)


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


def relay_loop(label, local_host, local_port, device_host, device_port):
  log(label, 'starting, will wait on device:%d and only touch %s:%d on an actual trigger' %
      (device_port, local_host, local_port))
  while True:
    device_conn = None
    try:
      device_conn = socket.create_connection((device_host, device_port),
                                             timeout=CONNECT_TIMEOUT_SEC)
      device_conn.settimeout(None)
      log(label, 'connected to device relay, waiting for a trigger')
      marker = device_conn.recv(len(TRIGGER_MARKER))
      if not marker:
        log(label, 'device relay closed with no trigger, reconnecting')
        continue
      if marker != TRIGGER_MARKER:
        log(label, 'unexpected data instead of a trigger marker, dropping: %r' % marker)
        continue

      log(label, 'trigger received, dialing the local listener now')
      local_conn = socket.create_connection((local_host, local_port),
                                            timeout=CONNECT_TIMEOUT_SEC)
      local_conn.settimeout(None)
      try:
        pump(device_conn, local_conn)
      finally:
        local_conn.close()
      log(label, 'trigger handled, waiting for the next one')
    except Exception as e:
      log(label, 'connect/relay failed: ' + str(e))
    finally:
      if device_conn is not None:
        try:
          device_conn.close()
        except Exception:
          pass
    time.sleep(RECONNECT_INTERVAL_SEC)


def main():
  parser = argparse.ArgumentParser(description='NEPI Gazebo control relay (quadcopter one-shot signals)')
  parser.add_argument('--device-host', required=True)
  args = parser.parse_args()

  threads = []
  for label, local_host, local_port, relay_device_port in RELAYS:
    t = threading.Thread(
        target=relay_loop,
        args=(label, local_host, local_port, args.device_host, relay_device_port),
        daemon=True,
    )
    t.start()
    threads.append(t)

  for t in threads:
    t.join()


if __name__ == '__main__':
  main()
