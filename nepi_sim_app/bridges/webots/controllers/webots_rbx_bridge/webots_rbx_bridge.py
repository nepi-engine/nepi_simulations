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

# Webots bridge for rbx_webots_node.py -- the RBX-driver path, NOT the generic
# sim_connector path (that one is sim_connector_bridge_webots.py, kept intact
# and unmodified as its own separate controller/world pair).
#
# Runs as a Webots ROBOT CONTROLLER (launched by Webots itself, declared via
# rbx_rover.wbt's `controller "webots_rbx_bridge"` field). Speaks the SAME
# simple wire protocol sim_bridge_node.py does for Gazebo (see that file):
# raw velocity in ({"linear_x","angular_z"}), bare telemetry + relayed camera
# frames out, plus {"type":"camera_settings"/"reset"/"environment_option"}
# handled as documented no-ops below -- NOT the generic sim_connector
# protocol's goto_position/motor_control/sensor_topics/goto_result messages.
#
# This bridge is deliberately SIMPLER than sim_connector_bridge_webots.py: the
# RBX driver (rbx_webots_node.py) runs its own closed-loop goto controller and
# only ever sends raw velocity downstream, exactly like sim_bridge_node.py does
# for rbx_gazebo_node.py -- so there is no goto-target/proportional-control
# logic in this file at all, only sensor reading and direct velocity-to-wheel
# conversion.
#
# SERVER, not client: rbx_webots_node.py connects TO this bridge (matching
# sim_bridge_node.py's server role), the reverse of
# sim_connector_bridge_webots.py's dial-out-to-the-app model. Also serves a
# tiny heartbeat port on a second listening socket (matching
# sim_heartbeat_listener.py exactly) since rbx_webots_discovery.py needs a
# real ALIVE-reply probe, not just a successful connect, for the same reason
# documented there (a forwarded port can accept a connection even when
# nothing real is listening on the far end).
#
# Robot: sim_container/bridges/webots/worlds/rbx_rover.wbt -- a copy of
# sim_connector_rover.wbt with only the controller field changed, same
# wheel1-4/GPS/IMU/Camera devices. One camera only, same as that world --
# SCENE_CAMERA/ROBOT_CAMERA both resolve to it, handled entirely on the
# rbx_webots_node.py side (this bridge doesn't know or care about camera
# naming, it just relays whatever frame it captures). RESET and
# environment_option are honest no-ops here for the same reasons documented
# in sim_connector_bridge_webots.py: this Robot node is not a Supervisor, and
# this world has no obstacle-course model.

import base64
import json
import math
import socket
import sys
import threading
import time

import numpy as np
import cv2

from controller import Robot

DEFAULT_HEARTBEAT_PORT = 9041
DEFAULT_BRIDGE_PORT = 9046
ALIVE_REPLY = b'ALIVE\n'

WHEEL_RADIUS_M = 0.04
WHEEL_TRACK_M = 0.12
MAX_WHEEL_RADPS = 8.0

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60


class WebotsRbxBridge:

  def __init__(self, heartbeat_port, bridge_port):
    self.heartbeat_port = heartbeat_port
    self.bridge_port = bridge_port

    self.robot = Robot()
    self.timestep = int(self.robot.getBasicTimeStep())

    # wheel1/wheel3 = left (anchor y=+0.06), wheel2/wheel4 = right (y=-0.06) --
    # matches the .wbt file's HingeJoint anchors exactly, same grouping
    # sim_connector_bridge_webots.py already uses for this same robot body.
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

    # Commanded velocity, set directly by the RBX driver's own closed-loop
    # controller -- no goto-target/proportional-control state here at all,
    # unlike sim_connector_bridge_webots.py (see module docstring).
    self.cmd_lock = threading.Lock()
    self.cmd_linear_x = 0.0
    self.cmd_angular_z = 0.0

    self.frame_lock = threading.Lock()
    self.latest_frame = None

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.heartbeatLoop, daemon = True).start()
    threading.Thread(target = self.bridgeServerLoop, daemon = True).start()

    print("webots_rbx_bridge: controller started, heartbeat on 127.0.0.1:%d, "
          "bridge on 127.0.0.1:%d" % (self.heartbeat_port, self.bridge_port), flush = True)

  #**********************
  # Webots simulation-step loop -- runs on the MAIN thread, as Webots requires.

  def run(self):
    while self.robot.step(self.timestep) != -1:
      self.updatePoseFromSensors()
      self.applyCommandedVelocity()

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
      print("webots_rbx_bridge: bad camera frame: %s" % str(e), flush = True)

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def applyCommandedVelocity(self):
    # No goto math here -- rbx_webots_node.py already computed lin/ang and
    # sends it every control tick (including (0,0) when idle), the same
    # self-healing-against-dropped-packets design sim_bridge_node.py relies on
    # for Gazebo. This just converts to per-wheel velocity and applies it.
    with self.cmd_lock:
      lin, ang = self.cmd_linear_x, self.cmd_angular_z

    left_radps = (lin - ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    right_radps = (lin + ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    left_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, left_radps))
    right_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, right_radps))
    for m in self.left_motors:
      m.setVelocity(left_radps)
    for m in self.right_motors:
      m.setVelocity(right_radps)

  #**********************
  # Heartbeat listener -- matches sim_heartbeat_listener.py exactly.

  def heartbeatLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', self.heartbeat_port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    while True:
      try:
        conn, _ = srv.accept()
        try:
          conn.sendall(ALIVE_REPLY)
        except Exception:
          pass
        finally:
          conn.close()
      except Exception as e:
        print("webots_rbx_bridge: heartbeat listener error: %s" % str(e), flush = True)

  #**********************
  # rbx_webots_node.py TCP server -- matches sim_bridge_node.py's server role
  # (the RBX node dials in, not the other way around).

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', self.bridge_port))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    while True:
      conn, _ = srv.accept()
      conn.settimeout(SOCKET_TIMEOUT_SEC)
      with self.sock_lock:
        self.sock = conn
      print("webots_rbx_bridge: rbx node connected", flush = True)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (conn, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while True:
        try:
          data = conn.recv(4096)
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
            self.processLineFromNode(line)

      sender_stop.set()
      with self.sock_lock:
        self.sock = None
      try:
        conn.close()
      except Exception:
        pass
      print("webots_rbx_bridge: rbx node disconnected, waiting for reconnect", flush = True)

  def senderLoop(self, conn, stop_event):
    last_image = 0.0
    while not stop_event.is_set():
      now = time.time()
      self.sendLine(conn, self.buildTelemetryLine())

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        with self.frame_lock:
          frame = self.latest_frame
        if frame is not None:
          self.sendLine(conn, {
              "type": "image",
              "data": base64.b64encode(frame).decode("ascii"),
          })
        last_image = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def buildTelemetryLine(self):
    with self.pose_lock:
      x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      lin_mps, ang_radps = self.lin_mps, self.ang_radps
    # Matches sim_bridge_node.py's bare-telemetry shape exactly: x/y/yaw plus
    # linear_x/angular_z, no "type" key (a line with no type key IS telemetry,
    # per rbx_webots_node.py's processBridgeLine dispatch).
    return {
        "x": x_m, "y": y_m, "yaw": yaw_rad,
        "linear_x": lin_mps, "angular_z": ang_radps,
    }

  def sendLine(self, conn, line_dict):
    # Locked around the actual send, not just self.sock's assignment -- same
    # reasoning as every other bridge in this project: two threads (this
    # sender loop and nothing else here, since there's no separate goto-result
    # sender) could otherwise interleave sendall() calls on the same socket.
    with self.sock_lock:
      try:
        conn.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  #**********************
  # Commands from rbx_webots_node.py

  def processLineFromNode(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      print("webots_rbx_bridge: bad line from node: %s" % str(e), flush = True)
      return
    if not isinstance(msg, dict):
      return
    if 'linear_x' in msg and 'type' not in msg:
      with self.cmd_lock:
        self.cmd_linear_x = float(msg.get('linear_x', 0.0))
        self.cmd_angular_z = float(msg.get('angular_z', 0.0))
      return
    msg_type = msg.get("type")
    if msg_type == "camera_settings":
      pass  # Single camera on this world -- nothing to switch between.
    elif msg_type == "reset":
      # This Robot node is not a Supervisor, so it cannot teleport itself --
      # an honest, documented gap (see module docstring), not a silent drop.
      print("webots_rbx_bridge: reset not supported (robot is not a Supervisor)", flush = True)
    elif msg_type == "environment_option":
      print("webots_rbx_bridge: environment_option not supported on this world, ignoring",
            flush = True)


def main():
  heartbeat_port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_HEARTBEAT_PORT
  bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BRIDGE_PORT
  bridge = WebotsRbxBridge(heartbeat_port, bridge_port)
  bridge.run()


if __name__ == "__main__":
  main()
