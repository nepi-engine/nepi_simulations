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

# Webots bridge for nepi_app_sim_connector (Phase 2, MULTI_SIMULATOR_INTEGRATION_PLAN.md).
#
# Runs as a Webots ROBOT CONTROLLER (launched by Webots itself, declared via
# the world file's `controller "sim_connector_bridge_webots"` field) -- NOT a
# ROS node. Zero ROS/nepi_sdk dependency, using only Webots' own native Python
# Controller API (`from controller import Robot`) plus a plain TCP socket to
# sim_connector_app_node.py -- the same "prefer no ROS on the simulator side"
# shape as demo_bridge_client.py and the PyBullet/WPILib bridges in this plan,
# and the one genuinely simulator-agnostic proof point of the three ROS-capable
# sims done so far (Gazebo is a ROS node; this and later ones are not).
#
# Reuses the exact same closed-loop goto controller shape as
# sim_connector_bridge_gazebo.py (proportional gains, turn-in-place gate) --
# not re-derived, since the control law doesn't depend on which physics engine
# is underneath, only the actuator interface does (Twist-publish there,
# per-wheel RotationalMotor.setVelocity() here).
#
# Robot: sim_container/bridges/webots/worlds/sim_connector_rover.wbt, adapted
# from Webots' own tutorial 4-wheel robot (wheel1/wheel3 = left side,
# wheel2/wheel4 = right side) plus GPS/InertialUnit/Camera devices added for
# the sim_connector NavPose/image contract. One camera only (unlike Gazebo's
# two) -- SCENE_CAMERA/ROBOT_CAMERA both map to it here; this bridge's
# environment_option and the RESET setup action are honest no-ops (logged, not
# silently dropped) since this world has no obstacle-course model and this
# Robot node is not a Supervisor (cannot teleport itself) -- see the Phase 2
# write-up in the plan doc for why those two are deliberately out of scope
# here rather than half-implemented.

import base64
import json
import math
import socket
import threading
import time

import numpy as np
import cv2

from controller import Robot

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030

CAMERA_SENSOR_NAME = "webots_rover/camera"

WHEEL_RADIUS_M = 0.04
WHEEL_TRACK_M = 0.12
MAX_WHEEL_RADPS = 8.0

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60

GOTO_KP_LIN = 0.5
GOTO_KP_ANG = 1.5
GOTO_TURN_GATE_RAD = math.radians(30.0)
GOTO_TOL_M = 0.05
GOTO_TOL_RAD = math.radians(3.0)
MAX_LINEAR_MPS = 0.3
MAX_ANGULAR_RADPS = math.radians(60.0)

MOTOR_MAX_LINEAR_MPS = 0.3


class WebotsSimConnectorBridge:

  def __init__(self, app_host, app_port):
    self.app_host = app_host
    self.app_port = app_port

    self.robot = Robot()
    self.timestep = int(self.robot.getBasicTimeStep())

    # wheel1/wheel3 = left (anchor y=+0.06), wheel2/wheel4 = right (y=-0.06) --
    # matches the .wbt file's HingeJoint anchors exactly.
    self.left_motors = [self.robot.getDevice("wheel1"), self.robot.getDevice("wheel3")]
    self.right_motors = [self.robot.getDevice("wheel2"), self.robot.getDevice("wheel4")]
    for m in self.left_motors + self.right_motors:
      m.setPosition(float("inf"))  # velocity-control mode
      m.setVelocity(0.0)

    self.gps = self.robot.getDevice("gps")
    self.gps.enable(self.timestep)
    self.imu = self.robot.getDevice("imu")
    self.imu.enable(self.timestep)
    self.camera = self.robot.getDevice("camera")
    self.camera.enable(self.timestep)

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0
    self._last_x, self._last_y, self._last_t = 0.0, 0.0, None

    self.goto_lock = threading.Lock()
    self.goto_target = None
    self.motor_ratios = [0.0, 0.0]

    self.home_lock = threading.Lock()
    self.home_x_m = 0.0
    self.home_y_m = 0.0

    self.frame_lock = threading.Lock()
    self.latest_frame = None
    self.active_image_topic = ""

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.bridgeLoop, daemon = True).start()

    print("sim_connector_bridge_webots: controller started, connecting to %s:%d" %
          (self.app_host, self.app_port), flush = True)

  #**********************
  # Webots simulation-step loop -- runs on the MAIN thread, as Webots requires.

  def run(self):
    while self.robot.step(self.timestep) != -1:
      self.updatePoseFromSensors()
      self.controlTick()

  def updatePoseFromSensors(self):
    x, y, _z = self.gps.getValues()
    roll, pitch, yaw = self.imu.getRollPitchYaw()
    now = self.robot.getTime()
    lin_mps = 0.0
    ang_radps = 0.0
    if self._last_t is not None:
      dt = now - self._last_t
      if dt > 1e-6:
        lin_mps = math.hypot(x - self._last_x, y - self._last_y) / dt
        ang_radps = self.normalizeAngle(yaw - self.yaw_rad) / dt
    self._last_x, self._last_y, self._last_t = x, y, now
    with self.pose_lock:
      self.x_m, self.y_m, self.yaw_rad = x, y, yaw
      self.lin_mps, self.ang_radps = lin_mps, ang_radps

    if now - getattr(self, "_last_image_capture", -999.0) >= 1.0 / IMAGE_RATE_HZ:
      self._last_image_capture = now
      self.captureFrame()

  def captureFrame(self):
    try:
      width, height = self.camera.getWidth(), self.camera.getHeight()
      raw = self.camera.getImage()
      if raw is None:
        return
      arr = np.frombuffer(raw, dtype = np.uint8).reshape((height, width, 4))
      bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
      ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
      if ok:
        with self.frame_lock:
          self.latest_frame = encoded.tobytes()
    except Exception as e:
      print("sim_connector_bridge_webots: bad camera frame: %s" % str(e), flush = True)

  #**********************
  # Closed-loop goto controller -- same shape as sim_connector_bridge_gazebo.py

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def controlTick(self):
    with self.goto_lock:
      target = self.goto_target
      motor_ratios = list(self.motor_ratios)
    with self.pose_lock:
      cur_x, cur_y, cur_yaw = self.x_m, self.y_m, self.yaw_rad

    lin = 0.0
    ang = 0.0
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
          print("sim_connector_bridge_webots: goto target reached", flush = True)
          if self.sock is not None:
            self.sendLine(self.sock, {"type": "goto_result", "success": True})
    elif any(motor_ratios):
      lin = (motor_ratios[0] + motor_ratios[1]) / 2.0 * MOTOR_MAX_LINEAR_MPS
      ang = (motor_ratios[1] - motor_ratios[0]) / WHEEL_TRACK_M * MOTOR_MAX_LINEAR_MPS

    left_radps = (lin - ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    right_radps = (lin + ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    left_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, left_radps))
    right_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, right_radps))
    for m in self.left_motors:
      m.setVelocity(left_radps)
    for m in self.right_motors:
      m.setVelocity(right_radps)

  #**********************
  # sim_connector_app_node.py TCP client (background thread)

  def bridgeLoop(self):
    while True:
      sock = None
      try:
        sock = socket.create_connection((self.app_host, self.app_port),
                                        timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception as e:
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      print("sim_connector_bridge_webots: connected to %s:%d" % (self.app_host, self.app_port),
            flush = True)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (sock, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while True:
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
      print("sim_connector_bridge_webots: connection lost, retrying in %.0fs" %
            RECONNECT_INTERVAL_SEC, flush = True)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    last_image = 0.0
    while not stop_event.is_set():
      now = time.time()
      self.sendLine(sock, self.buildTelemetryLine())

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        self.sendLine(sock, {"type": "sensor_topics", "topics": [
            {"topic_name": CAMERA_SENSOR_NAME, "msg_type": "sensor_msgs/Image"},
        ]})
        self.sendLine(sock, {"type": "environment_options", "options": []})
        last_announce = now

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        with self.frame_lock:
          frame = self.latest_frame
        if frame is not None:
          self.sendLine(sock, {
              "type": "image",
              "topic_name": CAMERA_SENSOR_NAME,
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
    # Locked around the actual send, not just self.sock's assignment -- the
    # goto-control thread can now send goto_result concurrently with the
    # main sender loop's own sends on the same socket.
    with self.sock_lock:
      try:
        sock.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  #**********************
  # Commands from sim_connector_app_node.py

  def processLineFromApp(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      print("sim_connector_bridge_webots: bad line from app: %s" % str(e), flush = True)
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
      pass  # Single camera on this world -- nothing to switch between.
    elif msg_type == "environment_option":
      print("sim_connector_bridge_webots: environment_option not supported on this world, ignoring",
            flush = True)
    elif msg_type == "robot_config":
      print("sim_connector_bridge_webots: robot_config selected: %s" % msg.get("config"),
            flush = True)

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
    action = msg.get("action")
    if action == "RETURN_HOME":
      self.handleGoHome()
    elif action == "RESET":
      # This Robot node is not a Supervisor, so it cannot teleport itself --
      # an honest, documented gap (see module docstring), not a silent no-op.
      print("sim_connector_bridge_webots: RESET not supported (robot is not a Supervisor)",
            flush = True)


def main():
  import argparse
  parser = argparse.ArgumentParser(description = __doc__)
  parser.add_argument("--host", default = DEFAULT_APP_HOST)
  parser.add_argument("--port", type = int, default = DEFAULT_APP_PORT)
  args, _ = parser.parse_known_args()
  bridge = WebotsSimConnectorBridge(args.host, args.port)
  bridge.run()


if __name__ == "__main__":
  main()
