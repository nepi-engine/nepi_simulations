#!/usr/bin/env python3
"""VM-side half of a plain TCP relay that makes ai_targeting_controller_
ardupilot.py's own bridge server (127.0.0.1:9027 on the VM) reachable at
127.0.0.1:9027 on the NEPI device again, WITHOUT a reverse SSH tunnel.

Same root cause and same fix shape as mavlink_relay_vm.py and
camera_bridge_relay_vm.py (see either script's own docstring for the full
writeup) -- confirmed live (2026-09-08) that this exact port was flatly
refused from the device, meaning sim_ai_targeting_bridge_script.py's own
bridgeLoop had never once actually connected, so
drone_follow_object_mission_script.py's target_localizations feed (and the
RUI's own Peripheral Status check for it) could never have worked
regardless of anything else in the follow pipeline.

Connects to ai_targeting_controller_ardupilot.py's own bridge server
(127.0.0.1:9027, already listening on 0.0.0.0 for exactly this) as a plain
client, and dials OUT to the device's own AiTargetingDeviceRelay (see
sim_ai_targeting_bridge_script.py), which re-exposes whatever arrives as a
plain local TCP server on the device's own 127.0.0.1:9027 -- exactly the
address BRIDGE_HOST/BRIDGE_PORT there already expects, so that script
needed no changes to its own connect logic at all.
"""
import argparse
import select
import socket
import time

DEFAULT_DEVICE_HOST = 'nepi'
DEFAULT_DEVICE_PORT = 9033
DEFAULT_BRIDGE_HOST = '127.0.0.1'
DEFAULT_BRIDGE_PORT = 9027
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5


def log(msg):
  print('ai_targeting_relay_vm: ' + msg, flush=True)


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
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--device-host', default=DEFAULT_DEVICE_HOST)
  parser.add_argument('--device-port', type=int, default=DEFAULT_DEVICE_PORT)
  parser.add_argument('--bridge-host', default=DEFAULT_BRIDGE_HOST)
  parser.add_argument('--bridge-port', type=int, default=DEFAULT_BRIDGE_PORT)
  args = parser.parse_args()

  log('starting, will bridge %s:%d <-> %s:%d' %
      (args.bridge_host, args.bridge_port, args.device_host, args.device_port))

  while True:
    bridge_conn = None
    device_conn = None
    try:
      bridge_conn = socket.create_connection((args.bridge_host, args.bridge_port),
                                              timeout=CONNECT_TIMEOUT_SEC)
      device_conn = socket.create_connection((args.device_host, args.device_port),
                                              timeout=CONNECT_TIMEOUT_SEC)
      bridge_conn.settimeout(None)
      device_conn.settimeout(None)
      log('connected both ends, relaying')
      pump(bridge_conn, device_conn)
      log('link dropped, reconnecting')
    except Exception as e:
      log('connect/relay failed: ' + str(e))
    finally:
      for c in (bridge_conn, device_conn):
        if c is not None:
          try:
            c.close()
          except Exception:
            pass
    time.sleep(RECONNECT_INTERVAL_SEC)


if __name__ == '__main__':
  main()
