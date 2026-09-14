#!/usr/bin/env python3
"""VM-terminal test tool: drive the generic-rover Gazebo sim by a relative
(x, y, yaw) offset, e.g. `move 10x` or `move 10x 5y 45yaw`.

Talks directly to this VM's local /rover/cmd_vel + /rover/odom -- NOT
through the sim bridge / NEPI_CMD_VEL_TOPIC relay. That's deliberate: if
the real rbx_sim_node.py driver is connected from the NEPI device, its
gotoControlCb is already sending a fresh velocity command on every 20Hz
tick (see that file's comment on why a single authoritative sender
matters); a second sender on the same relayed topic would race it exactly
like the motor-ratio bug found earlier this session. Publishing straight
to /rover/cmd_vel makes this a standalone Gazebo-physics test tool, used
when nothing else is actively driving the rover.

x/y are world ENU-frame meters (matches rbx_sim_node.py's gotoPosition
convention: x = east, y = north), added to the rover's current position.
yaw is relative degrees, applied as a final in-place turn after the
drive phase. z is accepted but ignored (ground rover).

Controller shape (turn-toward-bearing, drive when aligned, final yaw) and
gains mirror rbx_sim_node.py's gotoControlCb so a manual test here behaves
the same as a real RBX goto_position/goto_pose command would.
"""

import math
import re
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_VEL_TOPIC = '/rover/cmd_vel'
ODOM_TOPIC = '/rover/odom'

CONTROLLER_RATE_HZ = 20
GOTO_KP_LIN = 0.5
GOTO_KP_ANG = 1.5
GOTO_TURN_GATE_RAD = math.radians(30.0)
MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADPS = math.radians(45.0)
DIST_TOL_M = 0.05
YAW_TOL_RAD = math.radians(1.0)
TIMEOUT_SEC = 60.0

TOKEN_RE = re.compile(r'^(-?\d+(?:\.\d+)?)(x|y|z|yaw)$', re.IGNORECASE)


def normalizeAngle(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def parseArgs(argv):
    dx = dy = dyaw_deg = 0.0
    saw_z = False
    for raw in argv:
        tok = raw.strip().rstrip(',')
        if not tok:
            continue
        m = TOKEN_RE.match(tok)
        if not m:
            sys.exit("sim_move: can't parse '%s' -- expected e.g. 10x, -5y, 45yaw"
                      % raw)
        val = float(m.group(1))
        axis = m.group(2).lower()
        if axis == 'x':
            dx += val
        elif axis == 'y':
            dy += val
        elif axis == 'z':
            saw_z = True
        elif axis == 'yaw':
            dyaw_deg += val
    if saw_z:
        sys.stderr.write("sim_move: z ignored -- ground rover can't move vertically\n")
    return dx, dy, dyaw_deg


class OneShotMover(object):
    def __init__(self):
        self.pose = None
        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odomCb)

    def odomCb(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pos.x, pos.y, yaw)

    def waitForOdom(self, timeout=5.0):
        start = time.time()
        while self.pose is None and time.time() - start < timeout:
            time.sleep(0.05)
        if self.pose is None:
            sys.exit("sim_move: no /rover/odom received -- is sim_rover_gazebo running?")

    def run(self, dx, dy, dyaw_deg):
        self.waitForOdom()
        x0, y0, yaw0 = self.pose
        target_x = x0 + dx
        target_y = y0 + dy
        target_yaw = yaw0 + math.radians(dyaw_deg) if dyaw_deg != 0.0 else None

        rate = rospy.Rate(CONTROLLER_RATE_HZ)
        start = time.time()
        while not rospy.is_shutdown() and time.time() - start < TIMEOUT_SEC:
            cur_x, cur_y, cur_yaw = self.pose
            dxr = target_x - cur_x
            dyr = target_y - cur_y
            dist = math.hypot(dxr, dyr)

            lin = 0.0
            ang = 0.0
            if dist > DIST_TOL_M:
                bearing_err = normalizeAngle(math.atan2(dyr, dxr) - cur_yaw)
                ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * bearing_err))
                if abs(bearing_err) < GOTO_TURN_GATE_RAD:
                    lin = max(0.0, min(MAX_LINEAR_MPS, GOTO_KP_LIN * dist))
            elif target_yaw is not None:
                yaw_err = normalizeAngle(target_yaw - cur_yaw)
                if abs(yaw_err) > YAW_TOL_RAD:
                    ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * yaw_err))
                else:
                    break
            else:
                break

            cmd = Twist()
            cmd.linear.x = lin
            cmd.angular.z = ang
            self.pub.publish(cmd)
            rate.sleep()

        # Re-send the stop a few times: this is a one-shot script (nothing
        # keeps re-asserting after it exits), so a single dropped/last
        # message could latch a nonzero velocity in Gazebo's diff-drive
        # plugin. See gotoControlCb's comment in rbx_sim_node.py for the
        # same reasoning.
        for _ in range(5):
            self.pub.publish(Twist())
            rate.sleep()

        cur_x, cur_y, cur_yaw = self.pose
        print("sim_move: done -- now at x=%.2f y=%.2f yaw=%.1fdeg"
              % (cur_x, cur_y, math.degrees(cur_yaw)))


def main():
    argv = sys.argv[1:]
    if not argv:
        sys.exit("usage: move 10x [5y] [3z] [45yaw]  (run `testcommands` for the full list)")
    dx, dy, dyaw_deg = parseArgs(argv)
    rospy.init_node('sim_move', anonymous=True, disable_signals=True)
    OneShotMover().run(dx, dy, dyaw_deg)


if __name__ == '__main__':
    main()
