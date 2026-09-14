#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Raw pymavlink altitude-hold probe, bypassing mavros/the NEPI RBX driver
# entirely -- run directly on the dev VM (where ArduPilot SITL lives), not
# on the NEPI device.
#
# Why this exists: isolates whether an altitude-tracking anomaly is coming
# from ArduPilot/Gazebo itself vs. from NEPI's own driver code, by arming,
# taking off, and then holding one exact altitude via a continuously
# resent SET_POSITION_TARGET_GLOBAL_INT at the same 50Hz rate
# rbx_ardupilot_node.py's sendGotoCommandLoop() uses -- with NOTHING else
# in the command path. If altitude still drifts here, it rules out any
# NEPI-side cause. Confirmed live 2026-08-27/28 this session: it does drift
# under some conditions, pointing at ArduPilot SITL / Gazebo real-time
# behavior on a loaded VM rather than a driver bug.
#
# IMPORTANT CONSTRAINT: ArduPilot SITL's dedicated port (127.0.0.1:5771,
# see nepi_sitl_dev_env.sh's --out=tcpin:0.0.0.0:5771) is a tcpin listener
# that hands its one connection slot to whichever client connects first --
# it does NOT multiplex multiple simultaneous clients. If the real
# mavros/RBX driver bridge (reached from the NEPI device over the reverse
# SSH tunnel) is already connected, this script's own connect attempt will
# just hang waiting for a socket that never gets serviced. Either run this
# BEFORE the device's RBX_ARDUPILOT driver connects, or temporarily disable
# that driver (drivers_mgr update_driver_state RBX_ARDUPILOT false) first.
#
# Usage (on the dev VM, with ArduPilot SITL already running via
# sitl_gazebo/sitl_gazebo_full and nothing else holding port 5771):
#   python3 sim_raw_mavlink_altitude_probe.py

import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('tcp:127.0.0.1:5771', source_system=250)
m.wait_heartbeat()
print("heartbeat from sys %d comp %d" % (m.target_system, m.target_component))

def set_mode(mode_name):
    mode_id = m.mode_mapping()[mode_name]
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

def wait_ack(cmd_name="", timeout=5):
    msg = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    print(cmd_name, "ack:", msg)

print("Setting GUIDED...")
set_mode('GUIDED')
time.sleep(1)

print("Arming...")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
wait_ack("arm")
time.sleep(1)

print("Taking off to 10m...")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, 10)
wait_ack("takeoff")

# wait for climb to ~10m relative alt
t0 = time.time()
while time.time() - t0 < 25:
    gpi = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if gpi:
        relalt = gpi.relative_alt / 1000.0
        print("climbing... relalt=%.2f" % relalt)
        if relalt > 9.5:
            break

msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
start_lat = msg.lat
start_lon = msg.lon
current_alt_amsl_m = msg.alt / 1000.0
current_relalt = msg.relative_alt / 1000.0
print("Reached takeoff. alt_amsl=%.2f relalt=%.2f" % (current_alt_amsl_m, current_relalt))

# Now hold this EXACT altitude via continuous SET_POSITION_TARGET_GLOBAL_INT,
# same as NEPI driver does, and watch whether it drifts on its own.
target_alt_amsl_m = current_alt_amsl_m
type_mask = 0b0000111111111000  # position only

print("Holding altitude %.2f AMSL via continuous SET_POSITION_TARGET_GLOBAL_INT for 30s..." % target_alt_amsl_m)
t0 = time.time()
results = []
next_send = time.time()
while time.time() - t0 < 30:
    now = time.time()
    if now >= next_send:
        m.mav.set_position_target_global_int_send(
            int((now - t0) * 1000),
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            start_lat, start_lon, target_alt_amsl_m,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        next_send = now + 0.02  # 50Hz like the NEPI driver
    gpi = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if gpi:
        results.append((round(now - t0, 2), gpi.alt / 1000.0, gpi.relative_alt / 1000.0))

print("t, alt_amsl, relative_alt  (target relalt=%.2f)" % current_relalt)
for r in results[::10]:
    print(r)
print("last 10:")
for r in results[-10:]:
    print(r)
