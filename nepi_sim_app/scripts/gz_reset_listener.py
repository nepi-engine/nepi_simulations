#!/usr/bin/env python3
"""Tiny local trigger for resetting the running Gazebo sim.

Listens on 127.0.0.1:<port> and, on every connection received, calls
/gazebo/reset_world then re-asserts the quadcopter's own spawn pose via
/gazebo/set_model_state, replying OK/ERR.

Rewritten 2026-09-14 (was a plain `gz world -w default -o` CLI call, no ROS
involved at all) -- reported live: "resetting the sim for the quadcopter
simply kills all the motors [instead of teleporting home]." `-o` only ever
resets model POSES, never velocity, unlike sim_bridge_node.py's own
resetRover() (the rover's proven, working RESET_SIM path) which uses
/gazebo/reset_world specifically because it resets "pose, linear/angular
velocity, AND every joint's own position/velocity" in one call (Gazebo's own
Model::Reset()/Joint::Reset(), not something rebuilt field-by-field here). A
quadcopter reset while it still has real forward/climb velocity landed back
at the spawn pose but kept moving on the very next physics tick under that
same residual velocity -- arriving in the same instant reset_sim's own
force-disarm cuts power, so the whole sequence looked like "the motors just
died" rather than "it's now sitting still at home." reset_world fixes that
by zeroing velocity too, matching the rover exactly.

Deliberately still NOT /gazebo/reset_simulation (which also resets sim time
-- ArduPilot's SITL binary, actively connected via the FDM socket, sees that
as a discontinuity and crashes, the same reason the original -o-only
approach avoided -r/--reset-all). /gazebo/reset_world resets every model's
pose/velocity/joint state but leaves sim time running, exactly like
sim_bridge_node.py's own comment describes for the identical rover call.

The explicit /gazebo/set_model_state re-assertion afterward is the same
belt-and-suspenders reassertion resetRover() itself does: reset_world's own
model-state application can race a Gazebo physics step already in flight,
so this is a defensive re-send of the same target pose, not the primary
mechanism.

Exists so the NEPI RBX ArduPilot driver -- which runs on the remote NEPI
device, not this VM -- can reach across the existing reverse SSH tunnel and
trigger a Gazebo reset without needing its own SSH credentials back to here.

Installed to ~/.local/bin/gz_reset_listener.py on the dev VM and launched by
the gz_reset_listener function in nepi_sitl_dev_env.sh (see that file), and
by gazebo_quadcopter's own launch_command in simulator_launch_targets.yaml.
"""

import socket
import sys

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState

DEFAULT_PORT = 9021
# Matches every other VM-side script's own hardcoded constant for this model
# (camera_rig_controller_ardupilot.py, ai_targeting_controller_ardupilot.py,
# sim_persistent_follow_diagnostic.py all use the identical name).
VEHICLE_MODEL_NAME = 'iris_demo'
RESET_WORLD_SERVICE = '/gazebo/reset_world'
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
GAZEBO_SERVICE_WAIT_SEC = 5.0


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    rospy.init_node('gz_reset_listener', anonymous=True, disable_signals=True)
    model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)
    # One-time wait at startup, not per-connection -- reset_world existing
    # (or not) doesn't change while this process is alive, so there is no
    # reason to re-block every single RESET_SIM click on a fresh service
    # lookup the way sim_bridge_node.py's own per-call wait_for_service does
    # (that one has to, since a rover session can spawn/tear down the world
    # repeatedly across its own lifetime; this listener is started fresh
    # alongside a single Gazebo instance each launch).
    try:
        rospy.wait_for_service(RESET_WORLD_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
    except Exception as exc:
        print(f"gz_reset_listener: {RESET_WORLD_SERVICE} not available yet "
              f"({exc}); will keep trying per-request", flush=True)
    reset_world = rospy.ServiceProxy(RESET_WORLD_SERVICE, Empty)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Explicit None, NOT left as the default -- rospy.init_node() above sets
    # a PROCESS-GLOBAL socket.setdefaulttimeout(60), which this socket would
    # otherwise silently inherit. Found live 2026-09-15: this listener kept
    # crashing with "socket.timeout: timed out" on srv.accept() after ~60s
    # idle (no reset requested in that window), meaning Reset Sim would work
    # right after this script started but silently stop working again a
    # minute later with nothing to explain it. Same gotcha SitlMavlinkRelay/
    # CameraBridgeDeviceRelay's own srv.settimeout(None) already guards
    # against in rbx_ardupilot_node.py -- this script just never had it.
    srv.settimeout(None)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    print(f"gz_reset_listener listening on 127.0.0.1:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            reset_world()
            state = ModelState()
            state.model_name = VEHICLE_MODEL_NAME
            state.pose.orientation.w = 1.0
            state.reference_frame = 'world'
            model_state_pub.publish(state)
            reply = b'OK\n'
        except Exception as exc:
            reply = ('ERR\n' + str(exc)).encode()
        try:
            conn.sendall(reply)
        finally:
            conn.close()


if __name__ == '__main__':
    main()
