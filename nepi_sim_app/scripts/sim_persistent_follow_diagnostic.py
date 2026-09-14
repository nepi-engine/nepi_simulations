#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Standalone follow-chair diagnostic, run directly on the dev VM against
# ArduPilot SITL over raw pymavlink -- a stand-in for
# drone_follow_object_mission_script.py for whenever the actual NEPI
# device isn't reachable/plugged in. Uses the EXACT same standoff-radius
# setpoint math as move_to_object_callback() in the real mission script
# (range/azimuth/elevation from ai_targeting_controller_ardupilot.py's
# bridge on port 9027 -> body-frame x/y/z), so it's a reasonable proxy for
# "does the follow logic itself converge" independent of anything in the
# NEPI RBX driver/device stack. NOTE: TARGET_OFFSET_GOAL_M below is left at
# this diagnostic's own original 0.1m default; it is intentionally
# independent of drone_follow_object_mission_script.py's own
# TARGET_OFFSET_GOAL_M setting (2.5m as of this session) -- update both if
# you want them to match for a given test.
#
# Runs indefinitely (no cycle cap, no auto-land) until killed (SIGINT/
# SIGTERM) -- prints ground-truth distance-to-chair throughout each cycle
# via Gazebo's /gazebo/model_states, so a real follow session can be
# watched live and left running unattended.
#
# Confirmed live 2026-08-25: the original version of this script had no
# bridge-reconnect logic, which was the actual cause of one real bug --
# the drone silently stopping following after the AI-targeting bridge
# socket got stuck/EOF'd once (nothing ever reopened it). Hardened here
# with reconnect-on-EOF/error/staleness, mirroring
# sim_ai_targeting_bridge_script.py's own bridgeLoop() pattern -- worth
# keeping this fix in mind if any OTHER bridge-consuming script in this
# repo needs the same hardening.
#
# Usage (on the dev VM, with sitl_gazebo/sitl_gazebo_full and
# ai_targeting_controller_ardupilot.py already running):
#   python3 sim_persistent_follow_diagnostic.py
# Ctrl-C or SIGTERM to stop (leaves the drone airborne where it is, does
# not land/RTL on exit -- land manually via QGroundControl/MAVProxy or
# RESET_SIM if needed afterward).

import json
import math
import socket
import time
import signal
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from pymavlink import mavutil

TAKEOFF_HEIGHT_M = 10.0
TARGET_OFFSET_GOAL_M = 0.1
TRIGGER_RESET_DELAY_S = 5
GOTO_TIMEOUT_S = 30
GOTO_MAX_ERROR_M = 2.0
VEHICLE_MODEL_NAME = 'iris_demo'
TARGET_MODEL_NAME = 'sim_target_chair'

_positions = {}
def _model_states_cb(msg):
    for n in (VEHICLE_MODEL_NAME, TARGET_MODEL_NAME):
        if n in msg.name:
            idx = msg.name.index(n)
            p = msg.pose[idx].position
            _positions[n] = (p.x, p.y, p.z)

_shutdown = False
def _handle_sig(signum, frame):
    global _shutdown
    _shutdown = True
signal.signal(signal.SIGTERM, _handle_sig)
signal.signal(signal.SIGINT, _handle_sig)

rospy.init_node('persistent_follow_mission_diag', anonymous=True, disable_signals=True)
rospy.Subscriber('/gazebo/model_states', ModelStates, _model_states_cb)
time.sleep(1.0)

print("Connecting to SITL on 127.0.0.1:5771 ...", flush=True)
m = mavutil.mavlink_connection('tcp:127.0.0.1:5771', source_system=250)
m.wait_heartbeat(timeout=10)
print(f"Heartbeat from system {m.target_system}", flush=True)

def wait_hb(timeout=10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            return hb
    return None

print("Setting GUIDED mode...", flush=True)
mode_id = m.mode_mapping()['GUIDED']
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
time.sleep(1)

armed = False
for attempt in range(6):
    print(f"Arming (attempt {attempt+1})...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                             1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 6:
        msg = m.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print("  STATUSTEXT:", msg.text, flush=True)
        if msg.get_type() == 'HEARTBEAT' and bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
    if armed:
        break
    print("  Not armed yet, waiting for EKF to settle...", flush=True)
    time.sleep(5)

print("Armed:", armed, flush=True)
if not armed:
    print("Could not arm -- exiting", flush=True)
    sys.exit(1)

print(f"Taking off to {TAKEOFF_HEIGHT_M} m...", flush=True)
m.mav.command_long_send(m.target_system, m.target_component,
                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                         0, 0, 0, 0, 0, 0, TAKEOFF_HEIGHT_M)
t0 = time.time()
while time.time() - t0 < 40:
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        alt = msg.relative_alt / 1000.0
        if alt > TAKEOFF_HEIGHT_M - 1.0:
            break
print("Takeoff complete.", flush=True)

# Bridge connection is held as mutable state (not a bare module-level
# socket) so it can be transparently dropped and reconnected -- the earlier
# version had no reconnect logic at all, which is exactly what caused the
# drone to silently stop following (the socket got stuck/EOF'd once and
# nothing ever reopened it). Mirrors sim_ai_targeting_bridge_script.py's own
# bridgeLoop reconnect pattern.
_bridge = {'sock': None, 'buf': b'', 'last_data_t': 0.0}
STALE_CONN_SEC = 3.0  # server pushes at 5 Hz -- 3s of total silence means the link is dead

def _connect_bridge():
    print("Connecting to AI-targeting bridge on 127.0.0.1:9027 ...", flush=True)
    try:
        s = socket.create_connection(('127.0.0.1', 9027), timeout=5)
        s.settimeout(0.2)
        _bridge['sock'] = s
        _bridge['buf'] = b''
        _bridge['last_data_t'] = time.time()
        print("Bridge connected.", flush=True)
    except Exception as e:
        print(f"Bridge connect failed: {e}", flush=True)
        _bridge['sock'] = None

def latest_target():
    if _bridge['sock'] is None:
        _connect_bridge()
        if _bridge['sock'] is None:
            return None

    last = None
    got_any = False
    try:
        while True:
            data = _bridge['sock'].recv(4096)
            if not data:
                print("Bridge connection closed (EOF) -- will reconnect", flush=True)
                try:
                    _bridge['sock'].close()
                except Exception:
                    pass
                _bridge['sock'] = None
                return None
            got_any = True
            _bridge['buf'] += data
    except socket.timeout:
        pass
    except Exception as e:
        print(f"Bridge recv error ({e}) -- will reconnect", flush=True)
        try:
            _bridge['sock'].close()
        except Exception:
            pass
        _bridge['sock'] = None
        return None

    if got_any:
        _bridge['last_data_t'] = time.time()
    elif time.time() - _bridge['last_data_t'] > STALE_CONN_SEC:
        print(f"No data from bridge in {STALE_CONN_SEC}s -- forcing reconnect", flush=True)
        try:
            _bridge['sock'].close()
        except Exception:
            pass
        _bridge['sock'] = None
        return None

    buf = _bridge['buf']
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if line.strip():
            try:
                last = json.loads(line)
            except Exception:
                pass
    _bridge['buf'] = buf
    return last

_connect_bridge()

def dist_to_chair():
    if VEHICLE_MODEL_NAME in _positions and TARGET_MODEL_NAME in _positions:
        dx, dy, dz = (_positions[TARGET_MODEL_NAME][i] - _positions[VEHICLE_MODEL_NAME][i] for i in range(3))
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    return None

_last_heartbeat_t = time.time()

def check_mavlink_alive():
    global _last_heartbeat_t
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if hb:
        _last_heartbeat_t = time.time()
        return True
    if time.time() - _last_heartbeat_t > 10:
        print("WARNING: no MAVLink heartbeat in 10s -- SITL/Gazebo may have "
              "crashed (e.g. a VM suspend/resume time jump). Will keep "
              "retrying; restart the sim stack if this persists.", flush=True)
    return False

cycle = 0
print("Entering permanent follow loop -- will run until killed.", flush=True)
while not _shutdown:
    cycle += 1
    check_mavlink_alive()
    print(f"\n--- Follow cycle {cycle} ---", flush=True)
    target = None
    t0 = time.time()
    while target is None and time.time() - t0 < 5 and not _shutdown:
        target = latest_target()
        if target is None:
            time.sleep(0.2)
    if _shutdown:
        break
    if target is None:
        print("No target data received -- retrying", flush=True)
        time.sleep(TRIGGER_RESET_DELAY_S)
        continue

    name = target.get('target_name', '')
    range_m = target.get('range_m', -999.0)
    az_d = target.get('azimuth_deg', -999.0)
    el_d = target.get('elevation_deg', -999.0)
    print(f"Target: name={name} range_m={range_m:.2f} az={az_d:.2f} el={el_d:.2f}", flush=True)

    if name != 'chair' or range_m == -999:
        print("Target invalid, skipping actions", flush=True)
        time.sleep(1)
        continue

    setpoint_range_m = range_m - TARGET_OFFSET_GOAL_M
    sp_x = setpoint_range_m * math.cos(math.radians(az_d))
    sp_y = setpoint_range_m * math.sin(math.radians(az_d))
    sp_z = -setpoint_range_m * math.sin(math.radians(el_d))
    print(f"Setpoint (body frame): x={sp_x:.2f} y={sp_y:.2f} z={sp_z:.2f}", flush=True)

    d0 = dist_to_chair()
    if d0:
        print(f"Ground-truth distance to chair BEFORE goto: {d0:.2f} m", flush=True)

    type_mask = 0x0DF8
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, type_mask,
        sp_x, sp_y, sp_z,
        0, 0, 0, 0, 0, 0, 0, 0)

    t0 = time.time()
    while time.time() - t0 < GOTO_TIMEOUT_S and not _shutdown:
        time.sleep(1)
        d = dist_to_chair()
        if d is not None:
            print(f"  t+{time.time()-t0:4.1f}s  ground-truth distance to chair: {d:.2f} m", flush=True)
            if d < GOTO_MAX_ERROR_M:
                print("  Within GOTO_MAX_ERROR_M -- goto considered complete", flush=True)
                break

    print(f"Delaying next trigger for {TRIGGER_RESET_DELAY_S}s", flush=True)
    for _ in range(TRIGGER_RESET_DELAY_S * 5):
        if _shutdown:
            break
        time.sleep(0.2)

print("\nShutdown signal received -- exiting follow loop (drone left as-is, not landed).", flush=True)
