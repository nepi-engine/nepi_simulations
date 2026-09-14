#!/usr/bin/env python3
"""VM-side half of a plain TCP relay that makes camera_rig_controller_
ardupilot.py's own bridge server (127.0.0.1:9026 on the VM) reachable at
127.0.0.1:9026 on the NEPI device again, WITHOUT a reverse SSH tunnel.

Same root cause and same fix shape as mavlink_relay_vm.py (see that
script's own docstring for the full writeup) -- confirmed live
(2026-09-08) that this exact port was flatly refused from the device
(`Connection refused`), meaning rbx_ardupilot_node.py's own cameraBridgeLoop
had never once actually connected since every sim_connector bridge
switched to dialing OUT from the VM to the device instead of relying on a
reverse SSH tunnel to make a VM-local port look local on the device. No
image data (or environment commands, added alongside this fix) could ever
have reached that node regardless of anything else in the pipeline.

Connects to camera_rig_controller_ardupilot.py's own bridge server
(127.0.0.1:9026, already listening on 0.0.0.0 for exactly this) as a
plain client, and dials OUT to the device's own CameraBridgeDeviceRelay
(see rbx_ardupilot_node.py), which re-exposes whatever arrives as a plain
local TCP server on the device's own 127.0.0.1:9026 -- exactly the address
ArdupilotNode.CAMERA_BRIDGE_HOST/PORT already expects, so that node needed
no changes to its own connect logic at all.
"""
import argparse
import select
import socket
import time

DEFAULT_DEVICE_HOST = 'nepi'
DEFAULT_DEVICE_PORT = 9032
DEFAULT_CAMERA_HOST = '127.0.0.1'
DEFAULT_CAMERA_PORT = 9026
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5


def log(msg):
  print('camera_bridge_relay_vm: ' + msg, flush=True)


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
  parser.add_argument('--camera-host', default=DEFAULT_CAMERA_HOST)
  parser.add_argument('--camera-port', type=int, default=DEFAULT_CAMERA_PORT)
  args = parser.parse_args()

  log('starting, will bridge %s:%d <-> %s:%d' %
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
