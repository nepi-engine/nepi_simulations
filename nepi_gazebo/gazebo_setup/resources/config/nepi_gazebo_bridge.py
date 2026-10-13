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

# NEPI Gazebo rover bridge. Runs on the VM next to Gazebo and connects the
# spawned rover to the NEPI device's RBX_SIM driver (nepi_drivers'
# rbx_sim_node.py), which the Sim Connector app starts on the device.
#
# Started and stopped by nepi_gazebo.sh, never by hand: the service runs one
# of these while Gazebo is up with a robot config whose nepi_robot_config.yaml
# names nepi_driver: rbx_sim, and stops it with Gazebo or when the robot
# changes. It never starts, stops or respawns anything in Gazebo itself -- the
# service owns the simulator and the spawned model.
#
# The two machines run separate ROS masters, so this is a plain ROS node on
# the VM side and plain sockets across the network, to the device's
# bridge_port on both UDP and TCP. This side always speaks first (sends
# telemetry, opens the TCP connection), so the VM never has to accept an
# inbound connection. Wire format, one JSON object per message (newline-
# delimited on TCP):
#
#   UDP  VM -> device  {"type":"telemetry","seq","x","y","yaw","linear_x","angular_z","stamp"}
#                      10 Hz, from /rover/odom.
#   UDP  device -> VM  {"type":"velocity","seq","linear_x","linear_y","angular_z"}
#                      20 Hz from the device, always. Republished on
#                      /rover/cmd_vel. If it stops for WATCHDOG_SEC the rover
#                      is zeroed -- a lost device or link stops the robot
#                      instead of leaving it on its last command, which the
#                      diff-drive plugin would otherwise hold forever.
#   TCP  VM -> device  {"type":"image","camera","format":"jpeg","data":"<base64>","stamp"}
#                      The four camera views, 10 Hz each.
#   TCP  device -> VM  {"type":"reset"}  back to the start pose.
#
# Replaces sim_container's sim_bridge_node.py + camera_rig_controller.py for
# the service. Their idle hold, reset and camera encoding are carried over
# unchanged; their Gazebo management (restarting gzserver, spawning their own
# rover, respawning it for camera or dimension edits, the obstacle toggle) is
# not, because the service does all of that now.

import argparse
import base64
import json
import math
import os
import socket
import threading
import time

import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState

NODE_NAME = 'nepi_gazebo_bridge'
LOG_PREFIX = 'NEPI_GAZEBO_BRIDGE: '

FACTORY_PORT = 9023

CMD_VEL_TOPIC = '/rover/cmd_vel'
ODOM_TOPIC = '/rover/odom'
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
RESET_WORLD_SERVICE = '/gazebo/reset_world'

# The generic_rover model's own camera plugin topics (ROBOT_CONFIGS/
# generic_rover/model.sdf): robot = the first-person camera, scene = the
# chase camera, each with a colour and a 32FC1-metres depth image.
CAMERA_TOPICS = {
  'robot_color': '/rover/camera/image_raw',
  'scene_color': '/rover/camera_chase/image_raw',
  'robot_depth': '/rover/camera/depth/image_raw',
  'scene_depth': '/rover/camera_chase/depth/image_raw',
}

TELEMETRY_RATE_HZ = 10.0
CONTROL_RATE_HZ = 20.0
IMAGE_RATE_HZ = 10.0
JPEG_QUALITY = 65
DEPTH_MIN_RANGE_M = 0.3
DEPTH_MAX_RANGE_M = 15.0

# The device sends every 50 ms, so 250 ms is several missed packets.
WATCHDOG_SEC = 0.25
# A seq this far behind the last one accepted is a restarted sender, not a
# late packet (50 packets is 2.5 s of commands).
SEQ_RESTART_GAP = 50
CONNECT_RETRY_SEC = 2.0
RESET_WORLD_WAIT_SEC = 2.0


def imgmsg_to_cv2(msg):
  # Enough of cv_bridge for the encodings the rover's cameras publish, so
  # this runs without it.
  if msg.encoding in ('rgb8', 'bgr8'):
    img = np.frombuffer(msg.data, dtype = np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
  if msg.encoding == '32FC1':
    return np.frombuffer(msg.data, dtype = np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
  if msg.encoding == '16UC1':
    # millimetres
    return np.frombuffer(msg.data, dtype = np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width] / 1000.0
  raise ValueError("unsupported image encoding " + str(msg.encoding))


def depth_to_color_img(depth_img):
  """Colorize a depth frame in metres: close = blue, far = red (JET)."""
  depth_img = np.nan_to_num(np.asarray(depth_img, dtype = np.float32), nan = DEPTH_MAX_RANGE_M,
                            posinf = DEPTH_MAX_RANGE_M, neginf = DEPTH_MAX_RANGE_M)
  clipped = np.clip(depth_img, DEPTH_MIN_RANGE_M, DEPTH_MAX_RANGE_M)
  scaled = ((clipped - DEPTH_MIN_RANGE_M) *
            (255.0 / (DEPTH_MAX_RANGE_M - DEPTH_MIN_RANGE_M))).astype(np.uint8)
  return cv2.applyColorMap(scaled, cv2.COLORMAP_JET)


def yaw_from_quaternion(q):
  return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GazeboBridge(object):

  def __init__(self, device_host, port, model_name):
    self.device_host = device_host
    self.device_ip = socket.gethostbyname(device_host)
    self.port = int(port)
    self.model_name = model_name

    self.lock = threading.Lock()
    self.latest_telemetry = None
    self.tx_seq = 0
    self.rx_seq = None
    self.last_cmd = None
    self.last_cmd_time = 0.0
    self.watchdog_tripped = False
    self.held_pose = None
    self.tcp_conn = None
    self.tcp_send_lock = threading.Lock()
    self.latest_images = dict()
    self.image_lock = threading.Lock()

    self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 1)
    self.model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size = 1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, self.odomCb, queue_size = 1)
    for view, topic in CAMERA_TOPICS.items():
      rospy.Subscriber(topic, Image, self.imageCb, callback_args = view, queue_size = 1, buff_size = 2 ** 24)

    # One UDP socket for both directions: telemetry goes out from it, and the
    # device answers to the address it came from.
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.udp_sock.settimeout(0.5)

    # Wall-clock threads, not rospy.Timer: a ROS timer follows sim time when
    # /use_sim_time is set, and would slow or stop with the simulation.
    for target in (self.telemetryLoop, self.udpReceiveLoop, self.controlLoop,
                   self.tcpConnectLoop, self.imageLoop):
      thread = threading.Thread(target = target)
      thread.daemon = True
      thread.start()

    rospy.loginfo(LOG_PREFIX + "bridging model '" + model_name + "' to device " +
                  device_host + " (" + self.device_ip + ") port " + str(self.port) + " UDP+TCP")

  #######################
  # ROS callbacks

  def odomCb(self, msg):
    pos = msg.pose.pose.position
    with self.lock:
      self.latest_telemetry = {
        'x': pos.x,
        'y': pos.y,
        'yaw': yaw_from_quaternion(msg.pose.pose.orientation),
        'linear_x': msg.twist.twist.linear.x,
        'angular_z': msg.twist.twist.angular.z,
        'stamp': msg.header.stamp.to_sec(),
      }

  def imageCb(self, msg, view):
    with self.image_lock:
      self.latest_images[view] = msg

  #######################
  # UDP: telemetry out, velocity in

  def telemetryLoop(self):
    interval = 1.0 / TELEMETRY_RATE_HZ
    while not rospy.is_shutdown():
      with self.lock:
        telemetry = self.latest_telemetry
        if telemetry is not None:
          self.tx_seq += 1
          packet = dict(telemetry, type = 'telemetry', seq = self.tx_seq)
      if telemetry is not None:
        try:
          self.udp_sock.sendto(json.dumps(packet).encode(), (self.device_ip, self.port))
        except Exception as e:
          rospy.logwarn_throttle(10.0, LOG_PREFIX + "telemetry send failed: " + str(e))
      time.sleep(interval)

  def udpReceiveLoop(self):
    while not rospy.is_shutdown():
      try:
        data, addr = self.udp_sock.recvfrom(65535)
      except socket.timeout:
        continue
      except Exception:
        # Before the first send the socket has no local port yet, and recv on
        # it can fail on some platforms; the next telemetry send fixes that.
        time.sleep(0.1)
        continue
      if addr[0] != self.device_ip:
        continue
      try:
        msg = json.loads(data.decode())
      except Exception:
        continue
      if msg.get('type') != 'velocity':
        continue
      seq = msg.get('seq')
      with self.lock:
        # Stale or duplicated -- unless it is far behind, which is a restarted
        # device driver counting from 1 again, not a late packet.
        if seq is not None and self.rx_seq is not None and self.rx_seq - SEQ_RESTART_GAP < seq <= self.rx_seq:
          continue
        if seq is not None:
          self.rx_seq = seq
        self.last_cmd = (float(msg.get('linear_x', 0.0)), float(msg.get('linear_y', 0.0)),
                         float(msg.get('angular_z', 0.0)))
        self.last_cmd_time = time.monotonic()

  def controlLoop(self):
    interval = 1.0 / CONTROL_RATE_HZ
    while not rospy.is_shutdown():
      with self.lock:
        cmd = self.last_cmd
        fresh = cmd is not None and (time.monotonic() - self.last_cmd_time) < WATCHDOG_SEC
      if not fresh:
        if cmd is not None and not self.watchdog_tripped:
          rospy.logwarn(LOG_PREFIX + "no velocity command from the device for " +
                        str(WATCHDOG_SEC) + "s -- stopping the rover")
        self.watchdog_tripped = cmd is not None
        cmd = (0.0, 0.0, 0.0)
      elif self.watchdog_tripped:
        rospy.loginfo(LOG_PREFIX + "velocity commands resumed")
        self.watchdog_tripped = False
      self.applyCommand(cmd)
      time.sleep(interval)

  def applyCommand(self, cmd):
    twist = Twist()
    twist.linear.x, twist.linear.y, twist.angular.z = cmd
    self.cmd_pub.publish(twist)
    # A zero command means "hold position", not just "no command". Even with
    # cmd_vel exactly zero the four-wheel rover drifts slowly (ODE's solver
    # never settles the wheel contacts at exactly zero), so the pose is held
    # at a FIXED anchor captured when the rover stopped. Re-reading odom each
    # tick instead would lock in each tick's creep and drift anyway.
    if cmd == (0.0, 0.0, 0.0):
      if self.held_pose is None:
        with self.lock:
          telemetry = self.latest_telemetry
        if telemetry is not None:
          self.held_pose = {'x': telemetry['x'], 'y': telemetry['y'], 'yaw': telemetry['yaw']}
      self.holdStill()
    else:
      self.held_pose = None

  def holdStill(self):
    if self.held_pose is None:
      return
    state = ModelState()
    state.model_name = self.model_name
    state.pose.position.x = self.held_pose['x']
    state.pose.position.y = self.held_pose['y']
    state.pose.orientation.z = math.sin(self.held_pose['yaw'] / 2.0)
    state.pose.orientation.w = math.cos(self.held_pose['yaw'] / 2.0)
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)

  def resetRover(self):
    # Stop first so the reset pose isn't immediately fought by the last
    # commanded velocity, then put the rover back where the service spawns
    # it (the world origin).
    rospy.loginfo(LOG_PREFIX + "reset requested")
    with self.lock:
      self.last_cmd = None
    self.cmd_pub.publish(Twist())
    try:
      rospy.wait_for_service(RESET_WORLD_SERVICE, timeout = RESET_WORLD_WAIT_SEC)
      rospy.ServiceProxy(RESET_WORLD_SERVICE, Empty)()
    except Exception as e:
      rospy.logwarn(LOG_PREFIX + RESET_WORLD_SERVICE + " failed, resetting the rover pose only: " + str(e))
    state = ModelState()
    state.model_name = self.model_name
    state.pose.orientation.w = 1.0
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)
    self.held_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

  #######################
  # TCP: images out, one-shot commands in

  def tcpConnectLoop(self):
    while not rospy.is_shutdown():
      try:
        conn = socket.create_connection((self.device_ip, self.port), timeout = 5)
      except Exception as e:
        rospy.logwarn_throttle(10.0, LOG_PREFIX + "data connection to " + self.device_ip + ":" +
                               str(self.port) + " failed: " + str(e))
        time.sleep(CONNECT_RETRY_SEC)
        continue
      # rospy sets a process-wide default socket timeout; commands on this
      # connection are sporadic, so a quiet socket is not a dead one.
      conn.settimeout(None)
      conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
      rospy.loginfo(LOG_PREFIX + "data connection up to " + self.device_ip + ":" + str(self.port))
      with self.tcp_send_lock:
        self.tcp_conn = conn
      self.tcpReadLoop(conn)
      with self.tcp_send_lock:
        if self.tcp_conn is conn:
          self.tcp_conn = None
      try:
        conn.close()
      except Exception:
        pass
      rospy.logwarn(LOG_PREFIX + "data connection lost -- retrying in " + str(CONNECT_RETRY_SEC) + "s")
      time.sleep(CONNECT_RETRY_SEC)

  def tcpReadLoop(self, conn):
    buf = b''
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception:
        data = b''
      if not data:
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          msg = json.loads(line)
        except Exception:
          continue
        if msg.get('type') == 'reset':
          self.resetRover()

  def sendLine(self, line_dict):
    with self.tcp_send_lock:
      conn = self.tcp_conn
      if conn is None:
        return
      try:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception:
        # tcpReadLoop sees the same dead socket and reconnects.
        pass

  def imageLoop(self):
    interval = 1.0 / IMAGE_RATE_HZ
    while not rospy.is_shutdown():
      with self.image_lock:
        images = dict(self.latest_images)
        self.latest_images = dict()
      for view, msg in images.items():
        try:
          cv_img = imgmsg_to_cv2(msg)
          if view.endswith('_depth'):
            cv_img = depth_to_color_img(cv_img)
          ok, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
          if not ok:
            raise ValueError("JPEG encode failed")
        except Exception as e:
          rospy.logwarn_throttle(10.0, LOG_PREFIX + view + " frame dropped: " + str(e))
          continue
        self.sendLine({'type': 'image', 'camera': view, 'format': 'jpeg',
                       'data': base64.b64encode(encoded.tobytes()).decode('ascii'),
                       'stamp': msg.header.stamp.to_sec()})
      time.sleep(interval)


def main():
  parser = argparse.ArgumentParser(description = 'NEPI Gazebo rover bridge')
  parser.add_argument('--device-host', default = os.environ.get('NEPI_IP', ''),
                      help = 'NEPI device address (default: $NEPI_IP)')
  parser.add_argument('--port', type = int, default = FACTORY_PORT)
  parser.add_argument('--model', required = True, help = 'spawned model name (the robot config folder)')
  args, _ros_args = parser.parse_known_args()
  device_host = args.device_host.split('/')[0]
  if not device_host:
    raise SystemExit("no device address -- pass --device-host or set NEPI_IP")

  rospy.init_node(NODE_NAME)
  GazeboBridge(device_host, args.port, args.model)
  rospy.spin()


if __name__ == '__main__':
  main()
