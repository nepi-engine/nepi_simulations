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

# NEPI Gazebo reset listener for the ArduPilot quadcopter. Runs on the VM and
# answers the device RBX_ARDUPILOT_SITL driver's RESET_SIM: every connection
# on its port calls /gazebo/reset_world, re-asserts the vehicle's spawn pose,
# and replies OK (or ERR plus the reason). The device's reset_sim() then
# force-disarms over MAVLink itself; nothing here touches ArduPilot SITL.
#
# Started and stopped by nepi_gazebo.sh, never by hand, alongside SITL for a
# robot config whose nepi_robot_config.yaml names nepi_driver: rbx_ardupilot.
# The device reaches it through nepi_gazebo_control_relay.py (device port 9034
# -> this port), so the VM never accepts a connection from the network.
#
# reset_world, not reset_simulation: it zeroes every model's pose, velocity
# and joints (a quad reset mid-flight must stop, not keep its momentum) but
# leaves sim time running -- resetting sim time breaks SITL's FDM link to the
# ArduPilotPlugin. The set_model_state afterwards re-sends the same target in
# case reset_world raced a physics step already in flight.
#
# Carried over from sim_container's gz_reset_listener.py; the vehicle model
# name is now an argument (the service spawns each robot config under its
# folder name) instead of the old hardcoded 'iris_demo'.

import argparse
import socket

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState

DEFAULT_PORT = 9021
RESET_WORLD_SERVICE = '/gazebo/reset_world'
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
GAZEBO_SERVICE_WAIT_SEC = 5.0

LOG_PREFIX = 'nepi_gazebo_reset_listener: '


def main():
    parser = argparse.ArgumentParser(description='NEPI Gazebo reset listener (quadcopter RESET_SIM)')
    parser.add_argument('--model', required=True,
                        help='Gazebo model name of the vehicle to put back at the origin')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('nepi_gazebo_reset_listener', anonymous=True, disable_signals=True)
    model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)
    # Once at startup: reset_world's availability doesn't change while this
    # process (started alongside one Gazebo instance) is alive.
    try:
        rospy.wait_for_service(RESET_WORLD_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
    except Exception as exc:
        print(LOG_PREFIX + RESET_WORLD_SERVICE + " not available yet (" + str(exc) +
              "); will keep trying per request", flush=True)
    reset_world = rospy.ServiceProxy(RESET_WORLD_SERVICE, Empty)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # rospy.init_node() sets a process-global socket.setdefaulttimeout(60),
    # which would make accept() time out after a minute with no reset.
    srv.settimeout(None)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', args.port))
    srv.listen(1)
    print(LOG_PREFIX + "listening on 127.0.0.1:" + str(args.port) + " for model '" + args.model + "'",
          flush=True)
    while not rospy.is_shutdown():
        conn, _ = srv.accept()
        try:
            reset_world()
            state = ModelState()
            state.model_name = args.model
            state.pose.orientation.w = 1.0
            state.reference_frame = 'world'
            model_state_pub.publish(state)
            reply = b'OK\n'
            print(LOG_PREFIX + "reset done", flush=True)
        except Exception as exc:
            reply = ('ERR\n' + str(exc)).encode()
            print(LOG_PREFIX + "reset failed: " + str(exc), flush=True)
        try:
            conn.sendall(reply)
        finally:
            conn.close()


if __name__ == '__main__':
    main()
