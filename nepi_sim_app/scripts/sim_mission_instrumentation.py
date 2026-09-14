#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Single-process ArduPilot SITL mission instrumentation (device-side, run
# directly on the NEPI device via ssh -- not a NEPI automation script).
#
# Why this exists: diagnosing RBX/ArduPilot SITL issues by chaining separate
# `rostopic echo`/`rostopic hz` calls across multiple ssh round-trips produces
# timestamps from different clocks/tool invocations that don't line up, and
# each call only samples a narrow window -- easy to miss a low-rate event or
# misattribute which mission phase a symptom happened during. This instead
# subscribes to everything relevant in ONE rospy process on ONE shared clock
# for the whole run, so the resulting timeline can be read back afterward and
# trusted: e.g. "did setpoint_position/global actually receive traffic during
# corner 1?" or "was rel_alt still climbing when errors_current went stale?"
# both become simple queries against the same events list instead of separate,
# hard-to-correlate captures.
#
# Confirmed useful for exactly this session's real bugs: caught rel_alt
# climbing well past a correctly-and-continuously-sent altitude setpoint
# (an ArduPilot SITL altitude-tracking issue, not a NEPI bug), and caught
# setpoint_position/global going completely silent during a run where
# takeoff itself had failed (ruling out a "dead code" theory that looked
# plausible from a single narrow rostopic hz sample).
#
# Usage (on the device):
#   python3 sim_mission_instrumentation.py [duration_sec=170] [outfile=/tmp/instrument_mission.json]
#
# Start this BEFORE launching the mission script under test so the whole
# LAUNCH sequence is captured, then inspect outfile's JSON array of
# {"t": <seconds since start>, "kind": ..., ...fields} events afterward,
# e.g.:
#   python3 -c "import json; [print(e) for e in json.load(open('/tmp/instrument_mission.json')) if e['kind']=='rel_alt']"
#
# NS/MAV below assume the ardupilot_sitl device name used throughout this
# repo's SITL setup (see nepi_sitl_dev_env.sh) -- edit if testing against a
# differently-named RBX device.

import rospy, time, json, sys
from mavros_msgs.msg import GlobalPositionTarget
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from nepi_interfaces.msg import DeviceRBXStatus, DeviceRBXInfo

NS = "/nepi/device1/ardupilot_sitl"
MAV = "/nepi/device1/mavlink_sitl"

events = []
t0 = time.time()
last_process = [None]
last_errors = [None]

def log(kind, **fields):
    events.append({"t": round(time.time() - t0, 3), "kind": kind, **fields})

def status_cb(msg):
    errs = (round(msg.errors_current.x_m, 3), round(msg.errors_current.y_m, 3), round(msg.errors_current.z_m, 3))
    changed = msg.process_current != last_process[0] or errs != last_errors[0]
    last_process[0] = msg.process_current
    last_errors[0] = errs
    if changed:
        log("status", ready=msg.ready, process=msg.process_current, errors_xyz=list(errs))

last_info = [None]
def info_cb(msg):
    key = (msg.state, msg.mode)
    if key != last_info[0]:
        last_info[0] = key
        log("info", state=msg.state, mode=msg.mode)

def setpt_global_cb(msg):
    log("setpoint_position_global", lat=msg.pose.position.latitude,
        lon=msg.pose.position.longitude, alt=msg.pose.position.altitude)

def setpt_local_cb(msg):
    log("setpoint_position_local", x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z)

last_relalt = [None, 0.0]
def relalt_cb(msg):
    now = time.time()
    if last_relalt[0] is None or abs(msg.data - last_relalt[0]) > 0.02 or now - last_relalt[1] > 1.0:
        last_relalt[0] = msg.data
        last_relalt[1] = now
        log("rel_alt", alt=round(msg.data, 3))

rospy.init_node("instrument_mission", anonymous=True)

rospy.Subscriber(NS + "/rbx/status", DeviceRBXStatus, status_cb)
rospy.Subscriber(NS + "/rbx/info", DeviceRBXInfo, info_cb)
rospy.Subscriber(MAV + "/setpoint_position/global", GeoPoseStamped, setpt_global_cb)
rospy.Subscriber(MAV + "/setpoint_position/local", PoseStamped, setpt_local_cb)
rospy.Subscriber(MAV + "/global_position/rel_alt", Float64, relalt_cb)

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 170.0
end_time = time.time() + duration
rate = rospy.Rate(20)
while not rospy.is_shutdown() and time.time() < end_time:
    rate.sleep()

outfile = sys.argv[2] if len(sys.argv) > 2 else "/tmp/instrument_mission.json"
with open(outfile, "w") as f:
    json.dump(events, f)
print("wrote %d events to %s" % (len(events), outfile))
