#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Multi-robot simulator-side bridge (Universal Simulator Bridge, Phase 4).
# Serves a fixed set of robot slots against the generic_rover_multi.world
# Gazebo simulation: one TCP command/telemetry server per robot, each wired
# to that robot's own Gazebo namespace. A deliberately separate file from the
# single-robot sim_bridge_node.py -- that file is the verified single-rover
# path (sim_rover_gazebo) and stays untouched; this one is launched only by
# sim_rover_gazebo_multi. Two slots is the point of this phase (prove the
# per-robot-port pattern side by side), not arbitrary N.
#
# Per-slot protocol is identical to sim_bridge_node.py's (the remote
# rbx_sim_node.py client is shared): newline-delimited JSON both ways on one
# persistent connection per slot -- commands in ({"linear_x", "angular_z"}),
# odometry telemetry out pushed at a fixed rate from the latest
# /rover<N>/odom. One difference: commands publish straight to
# /rover<N>/cmd_vel instead of bouncing through the single-robot file's
# /nepi/sim/cmd_vel relay topic -- that relay exists for the single-robot
# world's VM-local testing convention and has no per-slot equivalent.
#
# Each slot's heartbeat listener stays a separate sim_heartbeat_listener.py
# process (started per port by sim_rover_gazebo_multi), same as the
# single-robot workflow -- reachability of a slot's heartbeat port means the
# sim stack is up, and the remote discovery probes it before connecting here.
#
# Camera-rover-multi phase addition: each slot's RobotBridge mirrors the
# single-robot sim_bridge_node.py's camera-relay lines on its own TCP
# connection -- {"type":"camera_settings",...} in (from that slot's own
# remote rbx_sim_node.py instance) applied to that slot's own ROS param
# namespace (/sim/camera/roverN/*, matching camera_rig_controller_multi.py's
# ROBOT_SLOTS table), and {"type":"image",...} out relayed straight through
# from that slot's own camera_rig_controller_multi.py compressed-image topic
# (/camera_rigN/image_compressed). Same division of labor as the
# single-robot file: the camera controller owns compression/rate throttling,
# this node only owns the per-slot network relay.

import base64
import json
import math
import socket
import threading
import time

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from gazebo_msgs.msg import ModelState

PKG_NAME = 'SIM_BRIDGE_MULTI'  # Use in display menus
FILE_TYPE = 'NODE'

NODE_NAME = 'sim_bridge_multi_node'

HEARTBEAT_TOPIC = '/sim/heartbeat'
HEARTBEAT_FRAME_ID = 'gazebo_simulation'
HEARTBEAT_INTERVAL_SEC = 1.0

TELEMETRY_RATE_HZ = 10.0

# RESET_SIM target topic, shared by all slots (rospy publishers key on
# model_name in the message itself, not the topic).
MODEL_STATE_TOPIC = '/gazebo/set_model_state'

# Robot slots: Gazebo namespace (from generic_rover_multi.world) + bridge
# port per robot. Ports follow the 902x sim-utility block numbering
# (9021 gz reset, 9022 rover1 heartbeat, 9023 rover1 bridge, 9024 rover2
# heartbeat, 9025 rover2 bridge -- heartbeat ports are served by separate
# sim_heartbeat_listener.py processes, listed here only for the comment
# trail). rover1 keeps the single-robot workflow's 9022/9023 pair so the
# remote discovery's slot table covers both workflows with one list.
# model_name/spawn_pose match generic_rover_multi.world's per-slot <model
# name>/<pose> exactly -- RESET_SIM's target for this slot.
ROBOT_SLOTS = [
    {'name': 'rover1', 'gazebo_ns': '/rover1', 'bridge_port': 9023,
     'camera_compressed_topic': '/camera_rig1/image_compressed',
     'camera_param_ns': '/sim/camera/rover1',
     'model_name': 'rover1', 'spawn_pose': (0.0, 2.0, 0.0)},
    {'name': 'rover2', 'gazebo_ns': '/rover2', 'bridge_port': 9025,
     'camera_compressed_topic': '/camera_rig2/image_compressed',
     'camera_param_ns': '/sim/camera/rover2',
     'model_name': 'rover2', 'spawn_pose': (0.0, -2.0, 0.0)},
]


#########################################
# Per-Robot Bridge Class
#########################################

class RobotBridge:
  """One robot slot: TCP command/telemetry server on its own port, wired to
  its own Gazebo topic namespace."""

  def __init__(self, name, gazebo_ns, bridge_port, camera_compressed_topic,
               camera_param_ns, model_name, spawn_pose):
    self.name = name
    self.bridge_port = bridge_port
    self.log_prefix = PKG_NAME + " [" + name + "]: "
    self.model_name = model_name
    self.spawn_pose = spawn_pose

    self.param_view_mode = camera_param_ns + '/view_mode'
    self.param_offset_x = camera_param_ns + '/offset_x'
    self.param_offset_y = camera_param_ns + '/offset_y'
    self.param_offset_z = camera_param_ns + '/offset_z'

    self.cmd_pub = rospy.Publisher(gazebo_ns + '/cmd_vel', Twist, queue_size=1)
    self.odom_sub = rospy.Subscriber(gazebo_ns + '/odom', Odometry, self.odomCb)
    self.model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)
    # Camera-rover-multi feature: this slot's own camera_rig_controller_multi.py
    # instance owns compression/rate throttling on its own topic; this bridge
    # only relays whatever arrives, straight through to this slot's client.
    self.image_sub = rospy.Subscriber(camera_compressed_topic, CompressedImage,
                                      self.imageCompressedCb)

    # Latest odom snapshot for the telemetry push loop, and the single
    # active bridge client for this slot (one robot, one remote node)
    self.latest_telemetry = None
    self.client_conn = None
    self.client_lock = threading.Lock()

    # Wall-clock threads, same rationale as sim_bridge_node.py: with
    # /use_sim_time set, a ROS timer slows/stops with the sim, and the
    # telemetry push doubles as connection liveness
    self.server_thread = threading.Thread(target=self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()
    self.telemetry_thread = threading.Thread(target=self.telemetryPushLoop)
    self.telemetry_thread.daemon = True
    self.telemetry_thread.start()

    rospy.loginfo(self.log_prefix + "Bridge server on 127.0.0.1:" +
                  str(bridge_port) + " for " + gazebo_ns)

  def odomCb(self, msg):
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Planar rover: yaw is all the remote node needs (math, not tf, to keep
    # this node's dependencies minimal)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    self.latest_telemetry = {
      'x': pos.x,
      'y': pos.y,
      'yaw': yaw,
      'linear_x': msg.twist.twist.linear.x,
      'angular_z': msg.twist.twist.angular.z,
      'stamp': msg.header.stamp.to_sec(),
    }

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy sets a process-global socket.setdefaulttimeout(60), which
    # accept() applies to every accepted connection. The command stream is
    # legitimately idle for long stretches, so a recv timeout must not be
    # treated as client death -- clear the timeout and block instead; a real
    # disconnect still unblocks recv with EOF, and a half-open client is
    # caught by the telemetry push failing.
    srv.settimeout(None)
    srv.bind(('0.0.0.0', self.bridge_port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    while not rospy.is_shutdown():
      try:
        conn, _ = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      rospy.loginfo(self.log_prefix + "Bridge client connected")
      with self.client_lock:
        self.client_conn = conn
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      rospy.loginfo(self.log_prefix + "Bridge client disconnected")

  def serveClient(self, conn):
    # Blocking recv loop on this slot's one active client: newline-delimited
    # JSON commands in, published to this robot's own cmd_vel topic. Returns
    # (back to accept) on client close or any error.
    buf = b''
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception as e:
        rospy.logwarn(self.log_prefix + "Bridge client recv error: " + repr(e))
        return
      if not data:
        rospy.loginfo(self.log_prefix + "Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          cmd = json.loads(line)
        except Exception as e:
          rospy.logwarn(self.log_prefix + "Bad bridge command line: " + str(e))
          continue
        # Dispatch by key presence, not a mandatory "type" tag: the existing
        # velocity command shape ({"linear_x","angular_z"}) predates this and
        # is left untouched. Only the new camera_settings shape carries a
        # "type" field. Same convention as sim_bridge_node.py.
        if cmd.get('type') == 'camera_settings':
          self.applyCameraSettings(cmd)
          continue
        if cmd.get('type') == 'reset':
          self.resetRobot()
          continue
        twist = Twist()
        twist.linear.x = float(cmd.get('linear_x', 0.0))
        twist.angular.z = float(cmd.get('angular_z', 0.0))
        self.cmd_pub.publish(twist)

  def resetRobot(self):
    # Stop first so the teleported pose isn't immediately fought by this
    # slot's diff-drive plugin still applying the last commanded velocity.
    self.cmd_pub.publish(Twist())
    state = ModelState()
    state.model_name = self.model_name
    state.pose.position.x = self.spawn_pose[0]
    state.pose.position.y = self.spawn_pose[1]
    state.pose.position.z = self.spawn_pose[2]
    state.pose.orientation.w = 1.0
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)

  def applyCameraSettings(self, cmd):
    # From this slot's own remote rbx_sim_node.py RBX settings mechanism
    # (view_mode/offset_x/y/z), pushed here because this VM has no other
    # visibility into the remote device's process state. Stored under this
    # slot's own param namespace so camera_rig_controller_multi.py's matching
    # slot instance -- and only that instance -- picks it up.
    rospy.set_param(self.param_view_mode, str(cmd.get('view_mode', 'FIRST_PERSON')))
    rospy.set_param(self.param_offset_x, float(cmd.get('offset_x', 0.0)))
    rospy.set_param(self.param_offset_y, float(cmd.get('offset_y', 0.0)))
    rospy.set_param(self.param_offset_z, float(cmd.get('offset_z', 0.0)))

  def imageCompressedCb(self, msg):
    # Relayed straight through to whichever client is connected to this slot
    # right now; dropped silently if none is, same as the single-robot file.
    line = {
      'type': 'image',
      'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
      'stamp': msg.header.stamp.to_sec(),
    }
    self.sendLineToClient(line)

  def sendLineToClient(self, line_dict):
    with self.client_lock:
      conn = self.client_conn
    if conn is None:
      return
    try:
      conn.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      rospy.logwarn_throttle(5.0, self.log_prefix + "Failed to send line to client: " + str(e))
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass

  def telemetryPushLoop(self):
    interval = 1.0 / TELEMETRY_RATE_HZ
    while not rospy.is_shutdown():
      time.sleep(interval)
      if self.latest_telemetry is None:
        continue
      self.sendLineToClient(self.latest_telemetry)


#########################################
# Node Class
#########################################

class SimBridgeMultiNode:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.heartbeat_pub = rospy.Publisher(HEARTBEAT_TOPIC, Header, queue_size=1)
    self.robot_bridges = [RobotBridge(slot['name'], slot['gazebo_ns'],
                                      slot['bridge_port'],
                                      slot['camera_compressed_topic'],
                                      slot['camera_param_ns'],
                                      slot['model_name'], slot['spawn_pose'])
                          for slot in ROBOT_SLOTS]

    # Wall-clock thread, not rospy.Timer (see RobotBridge comment)
    self.heartbeat_thread = threading.Thread(target=self.heartbeatLoop)
    self.heartbeat_thread.daemon = True
    self.heartbeat_thread.start()

    rospy.loginfo(PKG_NAME + ": Multi-robot Simulator Bridge Node initialized "
                  "(" + str(len(self.robot_bridges)) + " robot slots)")

  def run(self):
    """Block until ROS shutdown, servicing the per-robot subscribers."""
    rospy.spin()

  def heartbeatLoop(self):
    while not rospy.is_shutdown():
      hdr = Header()
      hdr.stamp = rospy.Time.now()
      hdr.frame_id = HEARTBEAT_FRAME_ID
      self.heartbeat_pub.publish(hdr)
      time.sleep(HEARTBEAT_INTERVAL_SEC)


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = SimBridgeMultiNode()
  node.run()
