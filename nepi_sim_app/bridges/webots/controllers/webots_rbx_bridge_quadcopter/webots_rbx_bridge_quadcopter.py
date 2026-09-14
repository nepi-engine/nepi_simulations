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

# Webots bridge for rbx_webots_quadcopter_node.py -- see
# docs/WEBOTS_QUADCOPTER_DRIVER_PLAN.md for why this is a plain Supervisor-
# velocity-injected body (no rotor aerodynamics, no ArduPilot SITL). New file,
# not a shared one with the rover's webots_rbx_bridge.py: the motion model is
# materially different (3D Supervisor setVelocity, body-frame command rotated
# by yaw into world frame) and this world's Robot node IS a Supervisor
# (rbx_rover.wbt's is not), so RESET actually teleports for real here instead
# of being a documented no-op.
#
# Wire protocol, matching webots_rbx_bridge.py's simple-protocol shape but
# extended to 3D: command in is {"linear_x","linear_y","linear_z","angular_z"}
# (body-frame velocity, rotated into world frame here before being applied --
# rbx_webots_quadcopter_node.py's own closed-loop goto controller computes
# body-frame lin/ang the same way the rover driver's does and sends it every
# control tick, including zeros when idle, the same self-healing-against-
# dropped-packets design every bridge in this project already relies on).
# Telemetry out is bare (no "type" key) {"x","y","z","yaw","linear_x",
# "linear_y","linear_z","angular_z"} -- z is new vs. the rover's telemetry,
# a real axis this time instead of always zero.
#
# SERVER, not client (matches webots_rbx_bridge.py's role, the reverse of
# sim_connector_bridge_webots.py's dial-out model). Also serves a heartbeat
# port matching sim_heartbeat_listener.py's ALIVE-reply contract.

import base64
import json
import math
import socket
import sys
import threading
import time

import numpy as np
import cv2

from controller import Supervisor

DEFAULT_HEARTBEAT_PORT = 9042
DEFAULT_BRIDGE_PORT = 9047
ALIVE_REPLY = b'ALIVE\n'

MAX_LINEAR_MPS = 2.0
MAX_VERTICAL_MPS = 1.5
MAX_ANGULAR_RADPS = math.radians(90.0)

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60


class WebotsRbxBridgeQuadcopter:

  def __init__(self, heartbeat_port, bridge_port):
    self.heartbeat_port = heartbeat_port
    self.bridge_port = bridge_port

    # Supervisor, not plain Robot -- needed for direct velocity injection
    # (setVelocity) and a real RESET (teleport back to spawn pose), neither
    # of which a plain Robot controller can do to its own node.
    self.robot = Supervisor()
    self.timestep = int(self.robot.getBasicTimeStep())
    self.self_node = self.robot.getSelf()

    # Spawn pose, captured once at startup -- what RESET_SIM teleports back
    # to. Reading it here (rather than hardcoding the .wbt's own translation)
    # means this stays correct even if the world file's spawn point changes.
    self.spawn_translation = list(self.self_node.getField("translation").getSFVec3f())
    self.spawn_rotation = list(self.self_node.getField("rotation").getSFRotation())

    self.camera = self.robot.getDevice("camera")
    self.camera.enable(self.timestep)

    # Kinematic position state -- WE are the authority on where this body
    # is (see module docstring: no Physics node, direct field writes every
    # step), not a sensor reading. Initialized from the spawn pose so the
    # very first telemetry line before any command arrives reports the real
    # starting position, not (0,0,0).
    self.pose_lock = threading.Lock()
    self.x_m = self.spawn_translation[0]
    self.y_m = self.spawn_translation[1]
    self.z_m = self.spawn_translation[2]
    self.yaw_rad = 0.0
    self.lin_x_mps = 0.0
    self.lin_y_mps = 0.0
    self.lin_z_mps = 0.0
    self.ang_radps = 0.0

    # Commanded velocity, BODY FRAME -- set directly by the RBX driver's own
    # closed-loop controller, no goto-target/proportional-control state here
    # at all (same division of responsibility as webots_rbx_bridge.py).
    self.cmd_lock = threading.Lock()
    self.cmd_linear_x = 0.0
    self.cmd_linear_y = 0.0
    self.cmd_linear_z = 0.0
    self.cmd_angular_z = 0.0

    self.frame_lock = threading.Lock()
    self.latest_frame = None

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.heartbeatLoop, daemon = True).start()
    threading.Thread(target = self.bridgeServerLoop, daemon = True).start()

    print("webots_rbx_bridge_quadcopter: controller started, heartbeat on "
          "127.0.0.1:%d, bridge on 127.0.0.1:%d" % (self.heartbeat_port, self.bridge_port),
          flush = True)

  #**********************
  # Webots simulation-step loop -- runs on the MAIN thread, as Webots requires.
  # Position is integrated and written every physics step (not a slower timer
  # thread) purely for smoothness -- there is no gravity to fight now (see
  # module docstring: no Physics node), unlike an earlier version of this
  # file that used Supervisor.setVelocity() on a Physics-enabled body, which
  # fell straight to the floor and then ignored every subsequent command.

  def run(self):
    while self.robot.step(self.timestep) != -1:
      self.applyCommandedVelocity()
      if self.robot.getTime() - getattr(self, "_last_image_capture", -999.0) >= 1.0 / IMAGE_RATE_HZ:
        self._last_image_capture = self.robot.getTime()
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
      print("webots_rbx_bridge_quadcopter: bad camera frame: %s" % str(e), flush = True)

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def applyCommandedVelocity(self):
    # Body-frame command -> world-frame velocity (rotate x/y by current yaw;
    # z and yaw-rate need no rotation) -> integrated directly into position
    # and written to the translation/rotation fields every step. No goto math
    # here -- see module docstring, the RBX driver already computed this and
    # sends it every control tick including (0,0,0,0) when idle.
    with self.cmd_lock:
      cx, cy, cz, cw = (self.cmd_linear_x, self.cmd_linear_y,
                        self.cmd_linear_z, self.cmd_angular_z)

    cx = max(-MAX_LINEAR_MPS, min(MAX_LINEAR_MPS, cx))
    cy = max(-MAX_LINEAR_MPS, min(MAX_LINEAR_MPS, cy))
    cz = max(-MAX_VERTICAL_MPS, min(MAX_VERTICAL_MPS, cz))
    cw = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, cw))

    dt = self.timestep / 1000.0

    with self.pose_lock:
      yaw = self.yaw_rad
      cos_y, sin_y = math.cos(yaw), math.sin(yaw)
      world_vx = cx * cos_y - cy * sin_y
      world_vy = cx * sin_y + cy * cos_y

      self.x_m += world_vx * dt
      self.y_m += world_vy * dt
      self.z_m += cz * dt
      self.yaw_rad = self.normalizeAngle(yaw + cw * dt)
      self.lin_x_mps, self.lin_y_mps, self.lin_z_mps, self.ang_radps = (
        world_vx, world_vy, cz, cw)
      x_m, y_m, z_m, yaw_rad = self.x_m, self.y_m, self.z_m, self.yaw_rad

    # Only yaw rate is ever commanded/written -- no real roll/pitch attitude
    # here, same simplification the rover driver already makes for its own
    # single rotation axis. Quaternion-free: R2023a's SFRotation field takes
    # axis+angle directly, and a pure yaw rotation is just axis (0,0,1).
    self.self_node.getField("translation").setSFVec3f([x_m, y_m, z_m])
    self.self_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, yaw_rad])

  def resetToSpawn(self):
    # Real teleport -- this world's Robot node IS a Supervisor, unlike the
    # rover's, so this actually works rather than being a documented no-op.
    with self.pose_lock:
      self.x_m, self.y_m, self.z_m = self.spawn_translation
      self.yaw_rad = 0.0
      self.lin_x_mps = self.lin_y_mps = self.lin_z_mps = self.ang_radps = 0.0
    self.self_node.getField("translation").setSFVec3f(self.spawn_translation)
    self.self_node.getField("rotation").setSFRotation(self.spawn_rotation)

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
        print("webots_rbx_bridge_quadcopter: heartbeat listener error: %s" % str(e), flush = True)

  #**********************
  # rbx_webots_quadcopter_node.py TCP server -- matches webots_rbx_bridge.py's
  # server role (the RBX node dials in, not the other way around).

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
      print("webots_rbx_bridge_quadcopter: rbx node connected", flush = True)

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
      print("webots_rbx_bridge_quadcopter: rbx node disconnected, waiting for reconnect",
            flush = True)

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
      x_m, y_m, z_m, yaw_rad = self.x_m, self.y_m, self.z_m, self.yaw_rad
      lin_x, lin_y, lin_z, ang_radps = (self.lin_x_mps, self.lin_y_mps,
                                        self.lin_z_mps, self.ang_radps)
    # No "type" key -> telemetry, matching webots_rbx_bridge.py's convention
    # (rbx_webots_quadcopter_node.py's processBridgeLine dispatches on that
    # absence the same way rbx_webots_node.py's already does).
    return {
        "x": x_m, "y": y_m, "z": z_m, "yaw": yaw_rad,
        "linear_x": lin_x, "linear_y": lin_y, "linear_z": lin_z, "angular_z": ang_radps,
    }

  def sendLine(self, conn, line_dict):
    # Locked around the actual send, not just self.sock's assignment -- same
    # thread-safety reasoning documented across every other bridge in this
    # project (senderLoop and the recv loop's own error paths both touch
    # this socket).
    with self.sock_lock:
      if conn is not self.sock:
        return
      try:
        conn.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  def processLineFromNode(self, line):
    try:
      cmd = json.loads(line)
    except Exception as e:
      print("webots_rbx_bridge_quadcopter: bad line from rbx node: %s" % str(e), flush = True)
      return
    cmd_type = cmd.get("type")
    if cmd_type is None:
      # No "type" key -> velocity command (matches webots_rbx_bridge.py's
      # convention for the rover).
      with self.cmd_lock:
        self.cmd_linear_x = float(cmd.get("linear_x", 0.0))
        self.cmd_linear_y = float(cmd.get("linear_y", 0.0))
        self.cmd_linear_z = float(cmd.get("linear_z", 0.0))
        self.cmd_angular_z = float(cmd.get("angular_z", 0.0))
    elif cmd_type == "reset":
      self.resetToSpawn()
    elif cmd_type == "camera_settings":
      # Documented no-op, same reasoning as webots_rbx_bridge.py's own: one
      # fixed Camera device, no repositionable rig to move.
      pass
    elif cmd_type == "environment_option":
      # Documented no-op -- this world has no obstacle-course model, same
      # gap as the rover's world.
      pass
    else:
      print("webots_rbx_bridge_quadcopter: unrecognized command type: %s" % str(cmd_type),
            flush = True)


#########################################
# Main
#########################################

if __name__ == "__main__":
  heartbeat_port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_HEARTBEAT_PORT
  bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BRIDGE_PORT
  bridge = WebotsRbxBridgeQuadcopter(heartbeat_port, bridge_port)
  bridge.run()
