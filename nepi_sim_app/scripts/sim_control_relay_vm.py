#!/usr/bin/env python3
"""VM-side half of three small plain TCP relays that make RESET_SIM, the
follow-mission script's TEARDOWN, and its START_TRIGGER connections all
reachable on the NEPI device again, WITHOUT a reverse SSH tunnel.

Same root cause and same fix shape as mavlink_relay_vm.py /
camera_bridge_relay_vm.py (see those scripts' own docstrings for the full
writeup): rbx_ardupilot_node.py's reset_sim() and
drone_follow_object_mission_script.py's own TEARDOWN/START_TRIGGER
connections all still dialed their own loopback (127.0.0.1:<port>) assuming
a reverse SSH tunnel would forward it back to this VM. MAVLink and the
camera bridge were already fixed to dial OUT from the VM to the device
instead (rbx_ardupilot_node.py's CameraBridgeDeviceRelay /
rbx_ardupilot_discovery.py's SitlMavlinkRelay) -- these three one-shot
control signals never got the same treatment, so in any setup without a
live reverse tunnel (this dev VM's ad-hoc test rig included), "Reset Sim"
only ever disarmed and the chair never spawned/despawned on its own.

UNLIKE those two continuous streams, gz_reset_listener.py and
ai_targeting_controller_ardupilot.py's teardown/start-trigger listeners are
one-shot: the mere act of a client connecting IS the whole request, with no
payload ever exchanged (rbx_ardupilot_node.py's reset_sim() itself only
connects and reads a reply). A first cut of this script held one persistent
connection to each real local service and pumped bytes continuously, which
meant its own reconnect loop re-triggered the real service every
~RECONNECT_INTERVAL_SEC regardless of whether the device had actually asked
for anything -- confirmed live, resetting the sim's pose on a timer with
nobody pressing Reset Sim. Fixed to be event-driven instead: this script
holds one persistent connection to the device's DeviceSideRelay (see
rbx_ardupilot_node.py), and only dials the real local service (127.0.0.1:
9021/9029/9030) once that connection actually delivers
DeviceSideRelay.TRIGGER_MARKER -- the explicit signal that a genuine local
client (reset_sim(), or the follow script's own teardown/start-trigger
connect) just landed on the device side.
"""
import argparse
import select
import socket
import threading
import time

DEFAULT_DEVICE_HOST = 'nepi'
TRIGGER_MARKER = b'TRIGGER\n'

# (label, local_host, local_port, device_port) -- local_port/device_port
# match RESET_SIM_DEVICE_RELAY/TEARDOWN_DEVICE_RELAY/START_TRIGGER_DEVICE_RELAY
# in rbx_ardupilot_node.py exactly.
RELAYS = [
    ('reset', '127.0.0.1', 9021, 9034),
    ('teardown', '127.0.0.1', 9029, 9035),
    ('start_trigger', '127.0.0.1', 9030, 9036),
]
RECONNECT_INTERVAL_SEC = 2.0
CONNECT_TIMEOUT_SEC = 5


def log(label, msg):
  print('sim_control_relay_vm[%s]: %s' % (label, msg), flush=True)


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
      # Block here -- no bytes flow until the device's own local one-shot
      # listener (127.0.0.1:9021/9029/9030 ON THE DEVICE) actually accepts a
      # genuine client and DeviceSideRelay._relayLocalConn sends the marker.
      # This is the whole point: nothing below this line runs speculatively.
      marker = device_conn.recv(len(TRIGGER_MARKER))
      if not marker:
        log(label, 'device relay closed with no trigger, reconnecting')
        continue
      if marker != TRIGGER_MARKER:
        log(label, 'unexpected data instead of a trigger marker, dropping: %r' % marker)
        continue

      log(label, 'trigger received, dialing the real local service now')
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
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--device-host', default=DEFAULT_DEVICE_HOST)
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
