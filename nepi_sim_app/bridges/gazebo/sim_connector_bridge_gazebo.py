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

# Gazebo bridge for nepi_app_sim_connector (Phase 1, MULTI_SIMULATOR_INTEGRATION_PLAN.md).
#
# This is the first bridge script written against the *new*, generic
# device_if_sim.py/sim_connector_app_node.py contract -- distinct from, and not
# a replacement for, the existing RBX_SIM driver (rbx_sim_node.py +
# sim_bridge_node.py), which talks to the same Gazebo rover through the older
# RBXRobotIF path. Both can point at the same running Gazebo instance, but not
# at the same time -- see the "A/B sanity check" note in the plan doc, Phase 1.
#
# Plain ROS node (Gazebo's own interface is ROS-native), zero nepi_sdk
# dependency -- runs on the sim VM only. Dials sim_connector_app_node.py's
# well-known TCP port as a client and speaks the exact newline-delimited-JSON
# wire protocol documented at the top of that file. Reuses generic_rover's
# already-proven closed-loop goto controller math from rbx_sim_node.py
# (proportional gains, turn-in-place gate, tolerance) rather than re-deriving
# it -- see MULTI_SIMULATOR_INTEGRATION_PLAN.md Phase 1, step 2.
#
# Why the goto controller lives HERE and not in sim_connector_app_node.py:
# device_if_sim.py's own scope note says its goto*Cb methods are thin
# delegators with no convergence wait -- "The connected simulator's own bridge
# or vehicle model owns reaching the setpoint." A rover has no onboard
# autopilot to delegate to, so this bridge script is that "simulator's own
# bridge."

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
import tf.transformations
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030

ODOM_TOPIC = "/rover/odom"
CMD_VEL_TOPIC = "/rover/cmd_vel"

# Two cameras, matching the driver-level scene_camera/robot_camera contract in
# docs/SIMULATION_INTERFACE_SPEC.md and the ground_robot_2_wheel robot config's
# available_camera_view_modes (SCENE_CAMERA/ROBOT_CAMERA) already in
# sim_connector_app_params.yaml -- both cameras exist unmodified in
# generic_rover.world already (an onboard camera plus the movable chase rig).
ROBOT_CAMERA_TOPIC = "/rover/camera/image_raw"
SCENE_CAMERA_TOPIC = "/rover/camera_chase/image_raw"
ROBOT_CAMERA_SENSOR_NAME = "gazebo_rover/robot_camera"
SCENE_CAMERA_SENSOR_NAME = "gazebo_rover/scene_camera"
VIEW_MODE_TO_SENSOR_NAME = {
    "ROBOT_CAMERA": ROBOT_CAMERA_SENSOR_NAME,
    "SCENE_CAMERA": SCENE_CAMERA_SENSOR_NAME,
}
FACTORY_VIEW_MODE = "ROBOT_CAMERA"

ROVER_MODEL_NAME = "generic_rover_demo"
SET_MODEL_STATE_SERVICE = "/gazebo/set_model_state"
SPAWN_MODEL_SERVICE = "/gazebo/spawn_sdf_model"
DELETE_MODEL_SERVICE = "/gazebo/delete_model"
GAZEBO_SERVICE_WAIT_SEC = 5.0
OBSTACLE_COURSE_MODEL_NAME = "obstacle_course"
OBSTACLE_COURSE_SDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "models",
    "obstacle_course", "model.sdf")

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60

CONTROLLER_RATE_HZ = 20.0
GOTO_KP_LIN = 0.5
GOTO_KP_ANG = 1.5
GOTO_TURN_GATE_RAD = math.radians(30.0)
GOTO_TOL_M = 0.3
GOTO_TOL_RAD = math.radians(3.0)
MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADPS = math.radians(45.0)

MOTOR_MAX_LINEAR_MPS = 0.5
MOTOR_WHEEL_BASE_M = 0.4


class GazeboSimConnectorBridge:

  def __init__(self, app_host, app_port):
    self.app_host = app_host
    self.app_port = app_port

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0

    self.goto_lock = threading.Lock()
    self.goto_target = None       # dict(x_m, y_m, yaw_deg or None)
    self.motor_ratios = [0.0, 0.0]

    self.home_lock = threading.Lock()
    self.home_x_m = 0.0
    self.home_y_m = 0.0

    self.frame_lock = threading.Lock()
    self.latest_frames = {ROBOT_CAMERA_SENSOR_NAME: None, SCENE_CAMERA_SENSOR_NAME: None}
    self.view_mode = FACTORY_VIEW_MODE
    self.active_image_topic = ""

    self.obstacle_course_spawned = False
    try:
      with open(OBSTACLE_COURSE_SDF_PATH, "r") as f:
        self.obstacle_course_sdf = f.read()
    except Exception as e:
      rospy.logwarn("sim_connector_bridge_gazebo: obstacle course SDF not loaded: %s", str(e))
      self.obstacle_course_sdf = None

    self.sock = None
    self.sock_lock = threading.Lock()

    rospy.init_node("sim_connector_bridge_gazebo", anonymous = False)

    self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, self.odomCb, queue_size = 1)
    rospy.Subscriber(ROBOT_CAMERA_TOPIC, Image, self.imageCb, callback_args = ROBOT_CAMERA_SENSOR_NAME,
                     queue_size = 1)
    rospy.Subscriber(SCENE_CAMERA_TOPIC, Image, self.imageCb, callback_args = SCENE_CAMERA_SENSOR_NAME,
                     queue_size = 1)

    threading.Thread(target = self.bridgeLoop, daemon = True).start()
    rospy.Timer(rospy.Duration(1.0 / CONTROLLER_RATE_HZ), self.controlTickCb)

    rospy.loginfo("sim_connector_bridge_gazebo: connecting to %s:%d",
                  self.app_host, self.app_port)
    rospy.spin()

  #**********************
  # Gazebo-side ROS callbacks

  def odomCb(self, msg):
    q = msg.pose.pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    with self.pose_lock:
      self.x_m = msg.pose.pose.position.x
      self.y_m = msg.pose.pose.position.y
      self.yaw_rad = yaw
      self.lin_mps = msg.twist.twist.linear.x
      self.ang_radps = msg.twist.twist.angular.z

  def imageCb(self, msg, sensor_name):
    try:
      arr = np.frombuffer(msg.data, dtype = np.uint8).reshape(msg.height, msg.width, -1)
      if msg.encoding in ("rgb8",):
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
      ok, encoded = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
      if ok:
        with self.frame_lock:
          self.latest_frames[sensor_name] = encoded.tobytes()
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo: bad camera frame (%s): %s",
                             sensor_name, str(e))

  #**********************
  # Closed-loop goto controller -- proportional gains, turn-in-place gate,
  # reused from rbx_sim_node.py's already-verified gotoControlCb.

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def controlTickCb(self, event):
    with self.goto_lock:
      target = self.goto_target
      motor_ratios = list(self.motor_ratios)

    lin = 0.0
    ang = 0.0
    with self.pose_lock:
      cur_x, cur_y, cur_yaw = self.x_m, self.y_m, self.yaw_rad

    if target is not None:
      dx = target["x_m"] - cur_x
      dy = target["y_m"] - cur_y
      dist = math.hypot(dx, dy)
      if dist > GOTO_TOL_M:
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw)
        ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(MAX_LINEAR_MPS, GOTO_KP_LIN * dist))
      else:
        yaw_err = 0.0
        if target["yaw_deg"] is not None:
          yaw_err = self.normalizeAngle(math.radians(target["yaw_deg"]) - cur_yaw)
        if abs(yaw_err) > GOTO_TOL_RAD:
          ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * yaw_err))
        else:
          with self.goto_lock:
            self.goto_target = None
          rospy.loginfo("sim_connector_bridge_gazebo: goto target reached")
          if self.sock is not None:
            self.sendLine(self.sock, {"type": "goto_result", "success": True})
    elif any(motor_ratios):
      left, right = motor_ratios[0], motor_ratios[1]
      lin = (left + right) / 2.0 * MOTOR_MAX_LINEAR_MPS
      ang = (right - left) / MOTOR_WHEEL_BASE_M * MOTOR_MAX_LINEAR_MPS

    twist = Twist()
    twist.linear.x = lin
    twist.angular.z = ang
    self.cmd_vel_pub.publish(twist)

  #**********************
  # sim_connector_app_node.py TCP client

  def bridgeLoop(self):
    while not rospy.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.app_host, self.app_port),
                                        timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception as e:
        rospy.logwarn_throttle(10.0, "sim_connector_bridge_gazebo: connect failed: %s", str(e))
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      rospy.loginfo("sim_connector_bridge_gazebo: connected to %s:%d",
                    self.app_host, self.app_port)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (sock, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while not rospy.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          continue
        except Exception:
          data = b""
        if not data:
          break
        buf += data
        while b"\n" in buf:
          line, buf = buf.split(b"\n", 1)
          if line.strip():
            self.processLineFromApp(line)

      sender_stop.set()
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      rospy.logwarn("sim_connector_bridge_gazebo: connection lost, retrying in %.0fs",
                    RECONNECT_INTERVAL_SEC)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    last_image = 0.0
    while not stop_event.is_set() and not rospy.is_shutdown():
      now = time.time()
      self.sendLine(sock, self.buildTelemetryLine())

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        self.sendLine(sock, {"type": "sensor_topics", "topics": [
            {"topic_name": ROBOT_CAMERA_SENSOR_NAME, "msg_type": "sensor_msgs/Image"},
            {"topic_name": SCENE_CAMERA_SENSOR_NAME, "msg_type": "sensor_msgs/Image"},
        ]})
        self.sendLine(sock, {"type": "environment_options", "options": ["obstacle_course"]})
        last_announce = now

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        # An explicit topic selection (set_active_image_topic) takes priority
        # over the camera_view_mode rig setting -- the topic selector is the
        # mechanism the RUI's image viewer actually renders from.
        active_sensor_name = (self.active_image_topic or
                              VIEW_MODE_TO_SENSOR_NAME.get(self.view_mode, ROBOT_CAMERA_SENSOR_NAME))
        with self.frame_lock:
          frame = self.latest_frames.get(active_sensor_name)
        if frame is not None:
          self.sendLine(sock, {
              "type": "image",
              "topic_name": active_sensor_name,
              "data": base64.b64encode(frame).decode("ascii"),
              "stamp": now,
          })
        last_image = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def buildTelemetryLine(self):
    with self.pose_lock:
      x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      lin_mps, ang_radps = self.lin_mps, self.ang_radps
    return {
        "x_m": x_m, "y_m": y_m, "z_m": 0.0,
        "yaw_deg": math.degrees(yaw_rad),
        "x_m_per_sec": lin_mps * math.cos(yaw_rad),
        "y_m_per_sec": lin_mps * math.sin(yaw_rad),
        "yaw_deg_per_sec": math.degrees(ang_radps),
    }

  def sendLine(self, sock, line_dict):
    # Locked around the actual send, not just around self.sock's assignment --
    # controlTickCb (a ROS timer, its own thread) can now send goto_result
    # concurrently with senderLoop's own sends on the same socket; without
    # this, two interleaved sendall() calls could tear a JSON line in half on
    # the wire. self.sock's connect/disconnect assignment already used this
    # same lock; this just extends it to cover concurrent senders too.
    with self.sock_lock:
      try:
        sock.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception as e:
        rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo: send failed: %s", str(e))

  #**********************
  # Commands from sim_connector_app_node.py

  def processLineFromApp(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo: bad line from app: %s", str(e))
      return
    msg_type = msg.get("type")
    if msg_type == "motor_control":
      self.handleMotorControl(msg)
    elif msg_type == "goto_position":
      self.handleGotoPosition(msg)
    elif msg_type == "goto_pose":
      self.handleGotoPose(msg)
    elif msg_type == "go_home":
      self.handleGoHome()
    elif msg_type == "go_stop":
      self.handleGoStop()
    elif msg_type == "setup_action":
      self.handleSetupAction(msg)
    elif msg_type == "set_active_image_topic":
      self.active_image_topic = msg.get("topic_name", "")
    elif msg_type == "camera_settings":
      self.view_mode = msg.get("view_mode", FACTORY_VIEW_MODE)
      rospy.loginfo("sim_connector_bridge_gazebo: view mode set to %s", self.view_mode)
    elif msg_type == "environment_option":
      if msg.get("option") == "obstacle_course":
        self.setObstacleCourse(bool(msg.get("enabled", True)))
    elif msg_type == "robot_config":
      rospy.loginfo("sim_connector_bridge_gazebo: robot_config selected: %s", msg.get("config"))
    else:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo: unhandled command type: %s",
                             str(msg_type))

  def handleMotorControl(self, msg):
    ind = int(msg.get("motor_ind", -1))
    ratio = float(msg.get("speed_ratio", 0.0))
    if ind < 0 or ind >= len(self.motor_ratios):
      return
    with self.goto_lock:
      self.motor_ratios[ind] = max(0.0, min(1.0, ratio))

  def handleGotoPosition(self, msg):
    with self.pose_lock:
      cur_x, cur_y = self.x_m, self.y_m
    with self.goto_lock:
      self.goto_target = {
          "x_m": cur_x + float(msg.get("x_meters", 0.0)),
          "y_m": cur_y + float(msg.get("y_meters", 0.0)),
          "yaw_deg": msg.get("yaw_deg"),
      }
      self.motor_ratios = [0.0, 0.0]

  def handleGotoPose(self, msg):
    # Ground rover: only yaw is achievable. Hold position, turn in place.
    with self.pose_lock:
      cur_x, cur_y = self.x_m, self.y_m
    with self.goto_lock:
      self.goto_target = {"x_m": cur_x, "y_m": cur_y, "yaw_deg": msg.get("yaw_deg", 0.0)}
      self.motor_ratios = [0.0, 0.0]

  def handleGoHome(self):
    with self.home_lock:
      home_x, home_y = self.home_x_m, self.home_y_m
    with self.goto_lock:
      self.goto_target = {"x_m": home_x, "y_m": home_y, "yaw_deg": None}
      self.motor_ratios = [0.0, 0.0]

  def handleGoStop(self):
    with self.goto_lock:
      self.goto_target = None
      self.motor_ratios = [0.0, 0.0]

  def handleSetupAction(self, msg):
    # Matches ground_robot_2_wheel's setup_actions list in
    # sim_connector_app_params.yaml (RESET, RETURN_HOME) -- not RESET_SIM/
    # RETURN_HOME's old RBX_SIM spelling.
    action = msg.get("action")
    if action == "RESET":
      self.resetSimPose()
    elif action == "RETURN_HOME":
      self.handleGoHome()

  def resetSimPose(self):
    try:
      rospy.wait_for_service(SET_MODEL_STATE_SERVICE, timeout = GAZEBO_SERVICE_WAIT_SEC)
      set_state = rospy.ServiceProxy(SET_MODEL_STATE_SERVICE, SetModelState)
      state = ModelState()
      state.model_name = ROVER_MODEL_NAME
      state.pose.orientation.w = 1.0
      set_state(state)
      with self.goto_lock:
        self.goto_target = None
        self.motor_ratios = [0.0, 0.0]
      rospy.loginfo("sim_connector_bridge_gazebo: reset rover to spawn pose")
    except Exception as e:
      rospy.logwarn("sim_connector_bridge_gazebo: reset failed: %s", str(e))

  def setObstacleCourse(self, enabled):
    # Reused verbatim pattern from sim_bridge_node.py's own setObstacleCourse
    # -- same model/services, same spawn-once/delete-once guard. The
    # device_if_sim contract only ever sends enabled=True (see
    # sim_connector_app_node.py's setEnvironmentOption comment), so this is a
    # one-way toggle for now, same limitation the contract itself documents.
    if self.obstacle_course_sdf is None:
      rospy.logwarn("sim_connector_bridge_gazebo: obstacle course SDF not loaded, ignoring")
      return
    if enabled == self.obstacle_course_spawned:
      return
    try:
      if enabled:
        rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout = GAZEBO_SERVICE_WAIT_SEC)
        spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
        resp = spawn(OBSTACLE_COURSE_MODEL_NAME, self.obstacle_course_sdf, "", Pose(), "world")
        if resp.success:
          self.obstacle_course_spawned = True
          rospy.loginfo("sim_connector_bridge_gazebo: obstacle course spawned")
        else:
          rospy.logwarn("sim_connector_bridge_gazebo: obstacle course spawn failed: %s",
                        resp.status_message)
      else:
        rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout = GAZEBO_SERVICE_WAIT_SEC)
        delete = rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)
        resp = delete(OBSTACLE_COURSE_MODEL_NAME)
        if resp.success:
          self.obstacle_course_spawned = False
          rospy.loginfo("sim_connector_bridge_gazebo: obstacle course removed")
    except Exception as e:
      rospy.logwarn("sim_connector_bridge_gazebo: obstacle course toggle failed: %s", str(e))


def main():
  import argparse
  parser = argparse.ArgumentParser(description = __doc__)
  parser.add_argument("--host", default = DEFAULT_APP_HOST,
                      help = "Host running sim_connector_app_node.py")
  parser.add_argument("--port", type = int, default = DEFAULT_APP_PORT,
                      help = "sim_connector_app_node.py bridge listen port")
  args, _ = parser.parse_known_args()
  GazeboSimConnectorBridge(args.host, args.port)


if __name__ == "__main__":
  main()
