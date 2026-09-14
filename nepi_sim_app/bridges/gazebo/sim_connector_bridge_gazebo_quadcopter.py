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

# Gazebo+ArduCopter-SITL bridge for nepi_app_sim_connector, quadcopter path.
#
# NOT an extension of sim_connector_bridge_gazebo.py -- that bridge drives a
# wheeled rover through a closed-loop Twist controller of its own; a
# quadcopter under ArduCopter SITL already HAS a flight controller doing
# that job, so this bridge's role is entirely different: translate the
# sim_connector wire protocol into MAVLink guided-mode commands and back,
# never drive motors or attitude directly. Reached only through
# gazebo_rover's own launch_target_overrides (see resolve_launch_target in
# simulator_launcher.py) when the operator's selected robot config is
# flight_robot_4_motor -- "Gazebo" stays the one thing picked in the RUI;
# this is the hidden target that combination actually launches.
#
# Camera: reuses the EXISTING camera_rig_controller_ardupilot.py process
# (launched alongside this bridge by gazebo_quadcopter's launch_command) --
# that script already owns positioning/publishing the chase-cam rig
# (/camera_rig/camera/image_raw) for the separate RBX ArduPilot SITL dev
# flow. This bridge only SUBSCRIBES to that already-proven feed rather than
# re-deriving chase-cam pose math a second time. v1 scope: only that one
# feed exists for this target, so both SCENE_CAMERA and ROBOT_CAMERA view
# modes map to it -- a real body-mounted camera would mean editing the
# third-party iris_with_ardupilot model (outside this repo, at
# ~/ardupilot_gazebo/models/, not reproducible via this repo's own deploy
# path), out of scope here.
#
# Environment: reuses environment_models.py's shared EnvironmentModelSpawner
# (same module camera_rig_controller_ardupilot.py and sim_bridge_node.py
# already use) -- was a no-op stub with has_environment_controls hardcoded
# false in sim_connector_app_params.yaml (v1 scope note, since removed);
# flipped true once this handler existed to back it, matching the rover's
# own has_environment_controls: true.
#
# Coordinate/axis conventions (best-effort, worth confirming against a real
# flight since sign errors here would show up as "moves the wrong way" not
# a crash): sim_connector_app_node.py's NavPose contract uses ENU
# (x=East, y=North, z=Up; see FACTORY frame_nav). MAVLink's LOCAL_POSITION_NED
# and ATTITUDE are NED (x=North, y=East, z=Down) with yaw measured clockwise
# from North. Every NED<->ENU conversion below is exactly: swap x/y, negate
# z, and yaw_enu_deg = 90 - yaw_ned_deg (normalized to [-180, 180]) -- the
# standard NED->ENU relationship, not something derived per-field.

import argparse
import base64
import json
import math
import os
import socket
import sys
import threading
import time

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pymavlink import mavutil

# environment_models.py lives in sim_container/scripts/, a sibling of this
# file's own bridges/gazebo/ directory, not on sys.path by default the way
# a script run from scripts/ itself gets its own directory added
# automatically -- added explicitly so the plain `import environment_models`
# below (same module camera_rig_controller_ardupilot.py and sim_bridge_node.py
# already use for exactly this) resolves here too.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
import environment_models

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030
DEFAULT_MAVLINK_CONN = "tcp:127.0.0.1:5772"

# The chase-cam rig camera_rig_controller_ardupilot.py already publishes --
# see module docstring for why this bridge doesn't own positioning it.
SCENE_CAMERA_TOPIC = "/camera_rig/camera/image_raw"
SCENE_CAMERA_SENSOR_NAME = "gazebo_quadcopter/scene_camera"
VIEW_MODE_TO_SENSOR_NAME = {
    "SCENE_CAMERA": SCENE_CAMERA_SENSOR_NAME,
    "ROBOT_CAMERA": SCENE_CAMERA_SENSOR_NAME,
}
FACTORY_VIEW_MODE = "SCENE_CAMERA"

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60

MAVLINK_HEARTBEAT_TIMEOUT_SEC = 60.0
MAVLINK_STREAM_RATE_HZ = 10
MAVLINK_ACK_TIMEOUT_SEC = 3.0

DEFAULT_TAKEOFF_ALT_M = 10.0
# ArduCopter rejects arm/takeoff until its EKF/GPS pre-arm checks pass --
# normal and expected for the first ~15-30s of simulated time after SITL
# starts, not a fault. Retried rather than failing immediately on the first
# rejection.
ARM_RETRY_ATTEMPTS = 12
ARM_RETRY_INTERVAL_SEC = 3.0


def wrap_deg(deg):
  while deg > 180.0:
    deg -= 360.0
  while deg < -180.0:
    deg += 360.0
  return deg


def ned_yaw_deg_to_enu(yaw_ned_deg):
  return wrap_deg(90.0 - yaw_ned_deg)


def enu_yaw_deg_to_ned(yaw_enu_deg):
  return wrap_deg(90.0 - yaw_enu_deg)


class GazeboQuadcopterSimConnectorBridge:

  def __init__(self, app_host, app_port, mavlink_conn_str):
    self.app_host = app_host
    self.app_port = app_port

    self.telemetry_lock = threading.Lock()
    self.have_telemetry = False
    self.x_m = 0.0          # ENU east
    self.y_m = 0.0          # ENU north
    self.z_m = 0.0          # ENU up (relative alt)
    self.yaw_deg = 0.0      # ENU convention
    self.x_m_per_sec = 0.0
    self.y_m_per_sec = 0.0
    self.yaw_deg_per_sec = 0.0
    self.latitude = 0.0
    self.longitude = 0.0

    self.home_lock = threading.Lock()
    self.home_set = False

    self.frame_lock = threading.Lock()
    self.latest_frame = None
    self.view_mode = FACTORY_VIEW_MODE
    self.active_image_topic = ""

    self.mav_lock = threading.Lock()
    self.master = None
    self.mavlink_conn_str = mavlink_conn_str

    self.env_spawner = environment_models.EnvironmentModelSpawner(
        log_prefix = "sim_connector_bridge_gazebo_quadcopter")

    self.sock = None
    self.sock_lock = threading.Lock()

    rospy.init_node("sim_connector_bridge_gazebo_quadcopter", anonymous = False)

    self.image_bridge = CvBridge()
    rospy.Subscriber(SCENE_CAMERA_TOPIC, Image, self.imageCb, queue_size = 1)

    threading.Thread(target = self.mavlinkLoop, daemon = True).start()
    threading.Thread(target = self.bridgeLoop, daemon = True).start()

    rospy.loginfo("sim_connector_bridge_gazebo_quadcopter: connecting to app at %s:%d, "
                  "MAVLink at %s", self.app_host, self.app_port, self.mavlink_conn_str)
    rospy.spin()

  #**********************
  # MAVLink connection to ArduCopter SITL -- one persistent connection,
  # reconnected on drop, same reconnect-loop shape as the TCP bridge to
  # sim_connector_app_node.py below.

  def mavlinkLoop(self):
    while not rospy.is_shutdown():
      try:
        master = mavutil.mavlink_connection(self.mavlink_conn_str)
        master.wait_heartbeat(timeout = MAVLINK_HEARTBEAT_TIMEOUT_SEC)
      except Exception as e:
        rospy.logwarn_throttle(10.0, "sim_connector_bridge_gazebo_quadcopter: "
                               "MAVLink connect failed: %s", str(e))
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      rospy.loginfo("sim_connector_bridge_gazebo_quadcopter: MAVLink heartbeat from "
                    "system %d component %d", master.target_system, master.target_component)
      self.requestStreams(master)
      with self.mav_lock:
        self.master = master

      # Blocking receive loop -- exits (and triggers a reconnect) only when
      # recv_match itself raises, which is what a real link drop looks like
      # with pymavlink's TCP transport.
      try:
        while not rospy.is_shutdown():
          msg = master.recv_match(blocking = True, timeout = SOCKET_TIMEOUT_SEC)
          if msg is None:
            continue
          self.handleMavlinkMessage(msg)
      except Exception as e:
        rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: MAVLink link lost: %s", str(e))

      with self.mav_lock:
        self.master = None
      time.sleep(RECONNECT_INTERVAL_SEC)

  def requestStreams(self, master):
    # The classic REQUEST_DATA_STREAM message -- deprecated upstream but
    # still the most broadly-supported way to get ArduCopter SITL streaming
    # position/attitude at a useful rate without per-message SET_MESSAGE_INTERVAL
    # calls (SITL's own default stream rates are too slow for 10 Hz telemetry).
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
      master.mav.request_data_stream_send(
          master.target_system, master.target_component,
          stream_id, MAVLINK_STREAM_RATE_HZ, 1)

  def handleMavlinkMessage(self, msg):
    msg_type = msg.get_type()
    if msg_type == "LOCAL_POSITION_NED":
      # NED -> ENU: x_m(east)=NED.y, y_m(north)=NED.x, z_m(up)=-NED.z.
      with self.telemetry_lock:
        self.x_m = msg.y
        self.y_m = msg.x
        self.z_m = -msg.z
        self.x_m_per_sec = msg.vy
        self.y_m_per_sec = msg.vx
        self.have_telemetry = True
    elif msg_type == "ATTITUDE":
      with self.telemetry_lock:
        self.yaw_deg = ned_yaw_deg_to_enu(math.degrees(msg.yaw))
        self.yaw_deg_per_sec = -math.degrees(msg.yawspeed)
        self.have_telemetry = True
    elif msg_type == "GLOBAL_POSITION_INT":
      with self.telemetry_lock:
        self.latitude = msg.lat / 1e7
        self.longitude = msg.lon / 1e7
        self.have_telemetry = True

  #**********************
  # Guided-mode command helpers

  def setModeGuided(self, master):
    try:
      mode_id = master.mode_mapping()["GUIDED"]
    except KeyError:
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: GUIDED mode not offered by this build")
      return False
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    return True

  def setMode(self, master, mode_name, fallback_mode_name = None):
    try:
      mode_id = master.mode_mapping()[mode_name]
    except KeyError:
      if fallback_mode_name is not None:
        rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: %s mode not offered, "
                      "falling back to %s", mode_name, fallback_mode_name)
        return self.setMode(master, fallback_mode_name)
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: %s mode not offered by this build",
                    mode_name)
      return False
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    return True

  def commandAccepted(self, master, command):
    # Best-effort ack check -- logged, never blocks the caller beyond
    # MAVLINK_ACK_TIMEOUT_SEC, matching this project's general "fail loud,
    # don't hang" convention rather than trusting a fire-and-forget send.
    ack = master.recv_match(type = "COMMAND_ACK", blocking = True, timeout = MAVLINK_ACK_TIMEOUT_SEC)
    if ack is None:
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: no ACK for command %d", command)
      return False
    if ack.command != command:
      return True  # a different command's ack arrived first; don't misreport
    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: command %d rejected (result %d)",
                    command, ack.result)
      return False
    return True

  def armAndTakeoff(self, alt_m):
    # Handles both TAKEOFF and LAUNCH -- see handleSetupAction's own comment
    # for why a multirotor doesn't get a distinct behavior for each.
    def attempt():
      with self.mav_lock:
        master = self.master
      if master is None:
        return False
      self.setModeGuided(master)
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
          1, 0, 0, 0, 0, 0, 0)
      if not self.commandAccepted(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
        return False
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
          0, 0, 0, 0, 0, 0, alt_m)
      return self.commandAccepted(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def retryLoop():
      for i in range(ARM_RETRY_ATTEMPTS):
        if attempt():
          rospy.loginfo("sim_connector_bridge_gazebo_quadcopter: armed and taking off to %.1fm", alt_m)
          return
        time.sleep(ARM_RETRY_INTERVAL_SEC)
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: could not arm/takeoff after %d attempts "
                    "-- SITL's EKF/GPS pre-arm checks may still be settling; try TAKEOFF again shortly",
                    ARM_RETRY_ATTEMPTS)

    # Arm can legitimately take the full retry window (waiting on SITL's own
    # EKF/GPS convergence) -- run off the ROS callback/recv thread so a slow
    # arm never blocks telemetry or the next incoming command.
    threading.Thread(target = retryLoop, daemon = True).start()

  #**********************
  # sim_connector_app_node.py TCP client -- same shape as
  # sim_connector_bridge_gazebo.py's own bridgeLoop/senderLoop.

  def bridgeLoop(self):
    while not rospy.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.app_host, self.app_port), timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception as e:
        rospy.logwarn_throttle(10.0, "sim_connector_bridge_gazebo_quadcopter: connect failed: %s", str(e))
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      rospy.loginfo("sim_connector_bridge_gazebo_quadcopter: connected to %s:%d",
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
      rospy.logwarn("sim_connector_bridge_gazebo_quadcopter: connection lost, retrying in %.0fs",
                    RECONNECT_INTERVAL_SEC)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    last_image = 0.0
    while not stop_event.is_set() and not rospy.is_shutdown():
      now = time.time()
      with self.telemetry_lock:
        have = self.have_telemetry
      if have:
        self.sendLine(sock, self.buildTelemetryLine())

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        self.sendLine(sock, {"type": "sensor_topics", "topics": [
            {"topic_name": SCENE_CAMERA_SENSOR_NAME, "msg_type": "sensor_msgs/Image"},
        ]})
        self.sendLine(sock, {"type": "environment_options",
                             "options": environment_models.list_environment_models()})
        last_announce = now

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        active_sensor_name = self.active_image_topic or VIEW_MODE_TO_SENSOR_NAME.get(
            self.view_mode, SCENE_CAMERA_SENSOR_NAME)
        with self.frame_lock:
          frame = self.latest_frame if active_sensor_name == SCENE_CAMERA_SENSOR_NAME else None
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
    with self.telemetry_lock:
      return {
          "x_m": self.x_m, "y_m": self.y_m, "z_m": self.z_m,
          "yaw_deg": self.yaw_deg,
          "x_m_per_sec": self.x_m_per_sec, "y_m_per_sec": self.y_m_per_sec,
          "yaw_deg_per_sec": self.yaw_deg_per_sec,
          "latitude": self.latitude, "longitude": self.longitude,
          "altitude_m": self.z_m,
      }

  def sendLine(self, sock, line_dict):
    try:
      sock.sendall((json.dumps(line_dict) + "\n").encode())
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo_quadcopter: send failed: %s", str(e))

  def imageCb(self, msg):
    try:
      cv_img = self.image_bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
      ok, encoded = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
      if ok:
        with self.frame_lock:
          self.latest_frame = encoded.tobytes()
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo_quadcopter: bad camera frame: %s", str(e))

  #**********************
  # Commands from sim_connector_app_node.py

  def processLineFromApp(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo_quadcopter: bad line from app: %s", str(e))
      return
    msg_type = msg.get("type")
    with self.mav_lock:
      master = self.master
    if msg_type == "goto_position":
      self.handleGotoPosition(master, msg)
    elif msg_type == "goto_pose":
      self.handleGotoPose(master, msg)
    elif msg_type == "goto_location":
      self.handleGotoLocation(master, msg)
    elif msg_type == "go_home":
      self.handleGoHome(master)
    elif msg_type == "go_stop":
      self.handleGoStop(master)
    elif msg_type == "setup_action":
      self.handleSetupAction(master, msg)
    elif msg_type == "motor_control":
      # ArduCopter's own flight controller owns motor mixing/stabilization --
      # forwarding a raw per-motor speed_ratio around it would fight the
      # autopilot rather than fly the vehicle. Declined on purpose, not
      # merely unhandled.
      pass
    elif msg_type == "set_active_image_topic":
      self.active_image_topic = msg.get("topic_name", "")
    elif msg_type == "camera_settings":
      self.view_mode = msg.get("view_mode", FACTORY_VIEW_MODE)
    elif msg_type == "environment_option":
      self.handleEnvironmentOption(msg)
    elif msg_type == "robot_config":
      rospy.loginfo("sim_connector_bridge_gazebo_quadcopter: robot_config selected: %s",
                    msg.get("config"))
    else:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_gazebo_quadcopter: unhandled command type: %s",
                             str(msg_type))

  def handleGotoPosition(self, master, msg):
    if master is None:
      return
    # x_meters/y_meters/z_meters are ENU offsets from the CURRENT position
    # (matches the rover bridge's own handleGotoPosition semantics) --
    # MAV_FRAME_LOCAL_OFFSET_NED expresses exactly that natively, so there is
    # no need to track "current position" here at all; ArduCopter resolves
    # the offset against its own current position.
    #
    # NOT the textbook ENU->NED swap (north=y, east=x, down=-z) -- confirmed
    # live (2026-09-08) with two isolated single-axis goto_position tests
    # (pure x_meters, then pure y_meters, vehicle allowed to settle between
    # each) that this SITL/world's own ArduCopter EKF north/east reference is
    # NOT aligned with Gazebo's world frame the textbook formula assumes: a
    # pure +x_meters (intended East) request landed as +Gazebo-X (correct
    # axis) but ONLY when sent as the NED "north" field, and a pure
    # +y_meters (intended North) request landed as +Gazebo-Y but only when
    # sent as the NED "east" field negated. I.e. actual_world = (north_sent,
    # -east_sent), not the assumed (east_sent, north_sent) -- solved by
    # inverting: north_sent = x_meters, east_sent = -y_meters. This is the
    # same class of misalignment (and the same fix shape) as
    # rbx_ardupilot_node.py's setTeleopVelocity fix from the same day, but a
    # separate bug in a separate code path -- that one goes through mavros;
    # this bridge talks raw MAVLink directly and needed its own, independent
    # fix. down=-z_meters is unaffected (vertical, no rotation ambiguity).
    north = float(msg.get("x_meters", 0.0))
    east = -float(msg.get("y_meters", 0.0))
    down = -float(msg.get("z_meters", 0.0))
    yaw_deg = msg.get("yaw_deg")
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    if yaw_deg is None:
      type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
      yaw_rad = 0.0
    else:
      yaw_rad = math.radians(enu_yaw_deg_to_ned(float(yaw_deg)))
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, type_mask,
        north, east, down, 0, 0, 0, 0, 0, 0, yaw_rad, 0)

  def handleGotoPose(self, master, msg):
    # Multirotor equivalent of the rover bridge's own "Ground rover: only
    # yaw is achievable" limitation: ArduCopter's flight controller owns
    # roll/pitch stabilization, and guided mode has no sensible "hold this
    # roll/pitch while hovering" request -- only yaw, at zero position
    # offset (hold current position, rotate to face yaw_deg).
    if master is None:
      return
    yaw_rad = math.radians(enu_yaw_deg_to_ned(float(msg.get("yaw_deg", 0.0))))
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, type_mask,
        0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rad, 0)

  def handleGotoLocation(self, master, msg):
    if master is None:
      return
    lat_int = int(float(msg.get("lat", 0.0)) * 1e7)
    lon_int = int(float(msg.get("long", 0.0)) * 1e7)
    alt_m = float(msg.get("altitude_meters", DEFAULT_TAKEOFF_ALT_M))
    yaw_deg = msg.get("yaw_deg")
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    if yaw_deg is None:
      type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
      yaw_rad = 0.0
    else:
      yaw_rad = math.radians(enu_yaw_deg_to_ned(float(yaw_deg)))
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
        lat_int, lon_int, alt_m, 0, 0, 0, 0, 0, 0, yaw_rad, 0)

  def handleEnvironmentOption(self, msg):
    # Wire format matches setEnvironmentOptionCb's own contract in
    # device_if_sim.py: {"option": "<name>", "enabled": <bool>}. enabled=True
    # spawns/switches to that named model (EnvironmentModelSpawner deletes
    # whatever was previously active first, only one is ever live); enabled=
    # False only matters when it names the CURRENTLY active model (switching
    # to a different option already implies deactivating the old one), and
    # returns to flat ground.
    option = msg.get("option")
    enabled = bool(msg.get("enabled", True))
    if enabled:
      self.env_spawner.set_active_model(option)
    elif self.env_spawner.is_spawned(option):
      self.env_spawner.set_active_model(None)

  def handleGoHome(self, master):
    if master is None:
      return
    self.setMode(master, "RTL")

  def handleGoStop(self, master):
    # BRAKE halts immediately in place; not every ArduCopter build offers it
    # (older/stripped firmware), so LOITER (hold position, softer stop) is
    # the fallback rather than silently doing nothing.
    if master is None:
      return
    self.setMode(master, "BRAKE", fallback_mode_name = "LOITER")

  def handleSetupAction(self, master, msg):
    # setup_actions for flight_robot_4_motor are [TAKEOFF, LAUNCH] (see
    # sim_connector_app_params.yaml) -- both do the same arm+GUIDED+climb
    # sequence. Unlike a fixed-wing (hand/catapult LAUNCH is a materially
    # different action from powered TAKEOFF), a multirotor has no
    # comparable distinct "launch" step -- offering the same safe, known
    # behavior under both buttons beats inventing a second, untested one.
    action = msg.get("action")
    if action in ("TAKEOFF", "LAUNCH"):
      self.armAndTakeoff(DEFAULT_TAKEOFF_ALT_M)


def main():
  parser = argparse.ArgumentParser(description = __doc__)
  parser.add_argument("--host", default = DEFAULT_APP_HOST,
                      help = "Host running sim_connector_app_node.py")
  parser.add_argument("--port", type = int, default = DEFAULT_APP_PORT,
                      help = "sim_connector_app_node.py bridge listen port")
  parser.add_argument("--mavlink", default = DEFAULT_MAVLINK_CONN,
                      help = "pymavlink connection string for ArduCopter SITL's dedicated output port")
  args, _ = parser.parse_known_args()
  GazeboQuadcopterSimConnectorBridge(args.host, args.port, args.mavlink)


if __name__ == "__main__":
  main()
