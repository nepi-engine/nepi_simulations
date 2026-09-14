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

# Simulated AI-targeting controller, ArduPilot SITL port (test scaffolding for
# drone_follow_object_mission_script.py's "KNOWN GAP" -- no app_ai_targeting
# app exists in this workspace). New file, not an edit to
# camera_rig_controller_ardupilot.py: this one originates synthetic detection
# data (range/azimuth/elevation to a moving target) rather than repositioning
# a camera, and owns its own Gazebo model/bridge port, per this project's
# convention of a separate file per distinct simulator/workflow.
#
# What this does:
#   - Spawns a small static "chair" stand-in model (sim_target_chair) into
#     the running Gazebo world via /gazebo/spawn_sdf_model (idempotent --
#     skipped if already present, same guard sim_bridge_node.py uses for
#     obstacle_course).
#   - Drives that model along a slow circle via /gazebo/set_model_state (the
#     same live-repositioning technique already proven jitter-free for
#     camera_rig) -- a moving target actually exercises "follow", not just
#     "fly to a fixed point".
#   - Reads the drone's live ground-truth pose from /gazebo/model_states
#     (filtered by VEHICLE_MODEL_NAME, identical modelStatesCb pattern to
#     camera_rig_controller_ardupilot.py).
#   - Computes range/azimuth/elevation from the drone to the target in the
#     SAME body-frame convention drone_follow_object_mission_script.py's
#     move_to_object_callback already assumes: X forward, Y right, Z down,
#     azimuth_deg positive = target to the right of the nose, elevation_deg
#     positive = target above the drone's current altitude. Range-gated only
#     (no camera-FOV frustum math) -- outside MAX_DETECTION_RANGE_M the
#     target is reported "not detected" via the Target.msg range_m sentinel
#     (-999), matching the mission script's own check.
#   - Serves this as newline-delimited JSON on its own TCP bridge (port 9027,
#     next free slot in the 902x sim-utility block after the camera bridge's
#     9026) for the NEPI device's sim_ai_targeting_bridge_script.py to
#     consume across the existing reverse tunnel -- no new tunnel/credentials
#     needed, just one more -R forward in nepi_tunnel() (see
#     nepi_sitl_dev_env.sh).
#
# Not auto-started by sitl_gazebo() (the older manual nepi_sitl_dev_env.sh
# dev workflow) -- manual-launch convention there matching
# camera_rig_controller_ardupilot (run in a separate terminal/screen after
# sitl_gazebo is up); see ai_targeting_controller_ardupilot() in
# nepi_sitl_dev_env.sh.
#
# IS auto-started by the Sim Connector app's own Deploy button, though
# (added 2026-09-03) -- simulator_launch_targets.yaml's gazebo_quadcopter
# launch_command now starts this alongside camera_rig_controller_ardupilot.py
# and the SITL/bridge processes. Found live: clicking Deploy produced a
# flying drone with nothing to detect at all -- this file's own guard
# already killed a stray instance from the older dev workflow but nothing
# ever started a fresh one through Deploy, so the two launch paths
# (Deploy vs. the manual dev workflow) had silently diverged.

import json
import math
import os
import socket
import threading
import time

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

PKG_NAME = 'AI_TARGETING_CONTROLLER_ARDUPILOT'
NODE_NAME = 'ai_targeting_controller_ardupilot'

MODEL_STATES_TOPIC = '/gazebo/model_states'
VEHICLE_MODEL_NAME = 'iris_demo'
TARGET_MODEL_NAME = 'sim_target_chair'
TARGET_NAME = 'chair'  # matches drone_follow_object_mission_script.py's default TARGET_TO_FOLLOW
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
SPAWN_MODEL_SERVICE = '/gazebo/spawn_sdf_model'
DELETE_MODEL_SERVICE = '/gazebo/delete_model'
GAZEBO_SERVICE_WAIT_SEC = 5
# See spawnTargetModelRetryLoop's own comment for why this is a loop, not a
# single attempt.
SPAWN_RETRY_INTERVAL_SEC = 3.0

SDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'models', 'sim_target_chair', 'model.sdf')

# Target motion: slow circle around a base point well within flight range of
# the drone's world-origin spawn point and its TAKEOFF_HEIGHT_M (10m) --
# horizontal distance to the circle's center is ~8m, so 3D range after
# takeoff is comfortably inside MAX_DETECTION_RANGE_M below.
CIRCLE_CENTER_X = 8.0
CIRCLE_CENTER_Y = 0.0
CIRCLE_Z = 0.5
CIRCLE_RADIUS_M = 2.5
CIRCLE_PERIOD_SEC = 50.0  # ~0.3 m/s -- slow enough for the mission script's
                          # ~5s (TRIGGER_RESET_DELAY_S) correction cadence to visibly keep pace

# Detection range gate -- see module docstring; deliberately simpler than a
# camera-FOV frustum check.
MAX_DETECTION_RANGE_M = 20.0

# Bridge streaming rate -- independent of the internal control tick, matching
# camera_rig_controller_ardupilot.py's own CONTROL_RATE_HZ/IMAGE_RATE_HZ split.
# CONTROL_RATE_HZ was 20.0 -- visibly steppy/jittery motion (user-reported
# 2026-08-25), since each /gazebo/set_model_state teleport is a discrete
# 50ms jump with nothing interpolating between them. Raised into the
# 45-60 Hz range requested so consecutive teleports are close enough
# together to read as smooth continuous motion.
CONTROL_RATE_HZ = 50.0
TARGET_STREAM_RATE_HZ = 5.0

BRIDGE_PORT = 9027

# Teardown trigger -- added 2026-08-26 so the real mission script
# (drone_follow_object_mission_script.py, via its cleanup_actions()) can ask
# the sim to make the chair actually disappear on stop, not just leave it
# frozen in place. Next free port in the 902x sim-utility block after the
# AI-targeting bridge's own 9027. Shuts this whole node down after
# despawning (rather than looping to accept further connections) so a
# later launch-trigger cleanly respawns a fresh controller + chair, instead
# of this instance quietly running forever with no target to report.
TEARDOWN_PORT = 9029

# Start trigger -- added 2026-09-09, requested live: "the chair object seems
# to be around even though the follow object script isnt on. it should only
# spawn in if that gets selected on." This node launches unconditionally
# alongside gazebo_quadcopter (see simulator_launch_targets.yaml), so before
# this change the chair existed for the entire lifetime of the quadcopter
# sim regardless of whether drone_follow_object_mission_script.py -- or any
# other follow-mission script -- was ever started; only a script's own
# cleanup_actions() teardown (TEARDOWN_PORT above) ever made it disappear,
# so a session where the script never ran at all still showed the chair the
# whole time. Symmetric with TEARDOWN_PORT (9030 is the next free slot --
# 9021-9029 are all already assigned in this directory's other scripts, and
# 9031-9033 are the device-side relay ports mavlink_relay_vm.py/
# camera_bridge_relay_vm.py/ai_targeting_relay_vm.py already claim). Spawning
# is now deferred until exactly one client
# (drone_follow_object_mission_script.py's own __init__) connects here -- see
# startTriggerServerLoop.
START_TRIGGER_PORT = 9030


class AiTargetingControllerArdupilot:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.start_time = time.time()

    self.pose_lock = threading.Lock()
    self.drone_x = 0.0
    self.drone_y = 0.0
    self.drone_z = 0.0
    self.drone_yaw = 0.0
    self.have_pose = False

    self.client_lock = threading.Lock()
    self.client_conn = None

    # Spawning itself is now deferred to startTriggerServerLoop (see
    # START_TRIGGER_PORT's own comment) -- this used to start
    # spawnTargetModelRetryLoop unconditionally right here, which is what
    # kept the chair alive for the entire quadcopter sim session regardless
    # of whether a follow-mission script ever ran.
    self.start_trigger_thread = threading.Thread(target = self.startTriggerServerLoop)
    self.start_trigger_thread.daemon = True
    self.start_trigger_thread.start()

    self.state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size = 1)
    self.model_states_sub = rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, self.modelStatesCb)

    self.control_timer = rospy.Timer(rospy.Duration(1.0 / CONTROL_RATE_HZ), self.controlCb)
    self.stream_timer = rospy.Timer(rospy.Duration(1.0 / TARGET_STREAM_RATE_HZ), self.streamTargetCb)

    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    self.teardown_thread = threading.Thread(target = self.teardownServerLoop)
    self.teardown_thread.daemon = True
    self.teardown_thread.start()

    rospy.loginfo(PKG_NAME + ": Target '" + TARGET_NAME + "' circling center (" +
                  str(CIRCLE_CENTER_X) + "," + str(CIRCLE_CENTER_Y) + "), radius " +
                  str(CIRCLE_RADIUS_M) + "m, period " + str(CIRCLE_PERIOD_SEC) + "s")
    rospy.loginfo(PKG_NAME + ": Targeting bridge server on 127.0.0.1:" + str(BRIDGE_PORT))
    rospy.loginfo(PKG_NAME + ": Start-trigger listener on 127.0.0.1:" + str(START_TRIGGER_PORT) +
                  " (target not spawned until triggered)")
    rospy.loginfo(PKG_NAME + ": Teardown listener on 127.0.0.1:" + str(TEARDOWN_PORT))

  def run(self):
    """Block until ROS shutdown, servicing the control/stream timers and the
    bridge server thread."""
    rospy.spin()

  def spawnTargetModelRetryLoop(self):
    while not rospy.is_shutdown():
      if self.spawnTargetModel():
        return
      time.sleep(SPAWN_RETRY_INTERVAL_SEC)

  def spawnTargetModel(self):
    """Returns True once the model is confirmed spawned (or already
    present), False if this attempt should be retried."""
    try:
      with open(SDF_PATH, 'r') as f:
        target_sdf = f.read()
    except Exception as e:
      rospy.logerr(PKG_NAME + ": Failed to read target SDF at " + SDF_PATH + ": " + str(e))
      return False
    try:
      rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout = GAZEBO_SERVICE_WAIT_SEC)
      spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
      initial_pose = Pose()
      initial_pose.position.x = CIRCLE_CENTER_X + CIRCLE_RADIUS_M
      initial_pose.position.y = CIRCLE_CENTER_Y
      initial_pose.position.z = CIRCLE_Z
      resp = spawn(TARGET_MODEL_NAME, target_sdf, '', initial_pose, 'world')
      if resp.success:
        rospy.loginfo(PKG_NAME + ": Target model spawned")
        return True
      if 'already exist' in resp.status_message.lower():
        # Already-spawned from a prior run of this node is the common case
        # (Gazebo keeps running across node restarts) -- not fatal, the
        # existing model is reused as-is.
        rospy.loginfo(PKG_NAME + ": Target model already exists, reusing: " +
                      resp.status_message)
        return True
      rospy.logwarn(PKG_NAME + ": Target model spawn failed, will retry: " + resp.status_message)
      return False
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Target model spawn service call failed, will retry: " + str(e))
      return False

  def startTriggerServerLoop(self):
    """Single-shot: waits for exactly one start trigger (sent by
    drone_follow_object_mission_script.py's own __init__, or any other
    follow-mission script wired to SIM_START_PORT the same way) before the
    target model is spawned at all -- see START_TRIGGER_PORT's own comment.
    Mirrors teardownServerLoop's accept-once shape, but does NOT shut this
    node down afterward: receiving a start trigger is the BEGINNING of this
    node's useful life, not the end. Spawning stays backgrounded with
    retries (spawnTargetModelRetryLoop, unchanged) rather than a single
    blocking attempt, for the same gzserver-not-ready-yet race
    spawnTargetModelRetryLoop's own comment already documents."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.settimeout(None)
    try:
      srv.bind(('0.0.0.0', START_TRIGGER_PORT))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
      srv.listen(1)
    except Exception as e:
      rospy.logerr(PKG_NAME + ": Could not bind start-trigger listener on 127.0.0.1:" +
                   str(START_TRIGGER_PORT) + ": " + str(e))
      return
    try:
      conn, _ = srv.accept()
    except Exception:
      return
    rospy.loginfo(PKG_NAME + ": Start triggered -- spawning target")
    try:
      conn.sendall(b'OK\n')
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Start-trigger response failed: " + str(e))
    finally:
      try:
        conn.close()
      except Exception:
        pass
      try:
        srv.close()
      except Exception:
        pass
    self.spawn_thread = threading.Thread(target = self.spawnTargetModelRetryLoop)
    self.spawn_thread.daemon = True
    self.spawn_thread.start()

  def despawnTargetModel(self):
    try:
      rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout = GAZEBO_SERVICE_WAIT_SEC)
      delete = rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)
      resp = delete(TARGET_MODEL_NAME)
      if resp.success:
        rospy.loginfo(PKG_NAME + ": Target model despawned")
      else:
        rospy.logwarn(PKG_NAME + ": Target model despawn failed: " + resp.status_message)
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Target model despawn service call failed: " + str(e))

  def teardownServerLoop(self):
    """Single-shot: accept exactly one teardown trigger, despawn the
    target, then shut this whole node down -- see TEARDOWN_PORT's own
    comment for why a full shutdown (not just despawn-and-keep-running)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.settimeout(None)
    try:
      srv.bind(('0.0.0.0', TEARDOWN_PORT))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
      srv.listen(1)
    except Exception as e:
      rospy.logerr(PKG_NAME + ": Could not bind teardown listener on 127.0.0.1:" +
                   str(TEARDOWN_PORT) + ": " + str(e))
      return
    try:
      conn, _ = srv.accept()
    except Exception:
      return
    rospy.loginfo(PKG_NAME + ": Teardown triggered -- despawning target and shutting down")
    try:
      self.despawnTargetModel()
      conn.sendall(b'OK\n')
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Teardown response failed: " + str(e))
    finally:
      try:
        conn.close()
      except Exception:
        pass
      try:
        srv.close()
      except Exception:
        pass
    rospy.signal_shutdown("teardown requested")

  def modelStatesCb(self, msg):
    try:
      idx = msg.name.index(VEHICLE_MODEL_NAME)
    except ValueError:
      return
    pos = msg.pose[idx].position
    q = msg.pose[idx].orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with self.pose_lock:
      self.drone_x = pos.x
      self.drone_y = pos.y
      self.drone_z = pos.z
      self.drone_yaw = yaw
      self.have_pose = True

  def currentTargetPosition(self):
    """Analytic target position along its slow circle -- commanded directly,
    not read back from Gazebo, same convention camera_rig_controller_ardupilot.py
    uses for its own externally-driven pose (self.drone_x/y/z there)."""
    t = time.time() - self.start_time
    theta = 2.0 * math.pi * (t / CIRCLE_PERIOD_SEC)
    tx = CIRCLE_CENTER_X + CIRCLE_RADIUS_M * math.cos(theta)
    ty = CIRCLE_CENTER_Y + CIRCLE_RADIUS_M * math.sin(theta)
    tz = CIRCLE_Z
    return tx, ty, tz

  def controlCb(self, timer_event):
    tx, ty, tz = self.currentTargetPosition()
    state = ModelState()
    state.model_name = TARGET_MODEL_NAME
    state.pose.position.x = tx
    state.pose.position.y = ty
    state.pose.position.z = tz
    state.pose.orientation.w = 1.0
    state.reference_frame = 'world'
    self.state_pub.publish(state)

  def computeTargetReport(self):
    """Returns (target_name, range_m, azimuth_deg, elevation_deg, detected)
    in the body-frame convention move_to_object_callback assumes: X forward,
    Y right, Z down; azimuth positive = right of nose, elevation positive =
    target above the drone's current altitude."""
    with self.pose_lock:
      if not self.have_pose:
        return TARGET_NAME, -999.0, -999.0, -999.0, False
      drone_x = self.drone_x
      drone_y = self.drone_y
      drone_z = self.drone_z
      drone_yaw = self.drone_yaw

    tx, ty, tz = self.currentTargetPosition()
    dx = tx - drone_x
    dy = ty - drone_y
    dz = tz - drone_z

    # Rotate the world-frame offset into the drone's body frame by -yaw
    # (world Z-up, yaw CCW from +X -- standard Gazebo/ROS convention, same
    # as modelStatesCb's own yaw derivation above).
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    body_forward = dx * cos_y + dy * sin_y
    body_right = dx * sin_y - dy * cos_y

    horiz_dist = math.hypot(body_forward, body_right)
    range_m = math.sqrt(horiz_dist * horiz_dist + dz * dz)

    if range_m > MAX_DETECTION_RANGE_M:
      return TARGET_NAME, -999.0, -999.0, -999.0, False

    azimuth_deg = math.degrees(math.atan2(body_right, body_forward))
    elevation_deg = math.degrees(math.atan2(dz, horiz_dist)) if horiz_dist > 1e-6 else 0.0

    return TARGET_NAME, range_m, azimuth_deg, elevation_deg, True

  def streamTargetCb(self, timer_event):
    target_name, range_m, azimuth_deg, elevation_deg, detected = self.computeTargetReport()
    line = {
      'type': 'target',
      'target_name': target_name,
      'range_m': range_m,
      'azimuth_deg': azimuth_deg,
      'elevation_deg': elevation_deg,
      'detected': detected,
      # Wall-clock, not rospy.Time.now() -- matches currentTargetPosition()'s own
      # time.time() usage and avoids any sim-time/use_sim_time subtlety on this
      # purely-informational field (controlCb, which is confirmed firing via the
      # target's own visible motion, uses time.time() throughout for the same reason).
      'stamp': time.time(),
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
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Failed to send line to client: " + str(e))
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy's process-global socket.setdefaulttimeout(60) (set by init_node())
    # applies to accepted connections too -- this channel is one-way/outbound
    # only (no inbound commands expected), so clear it rather than treat a
    # long idle gap as client death; a real disconnect still unblocks recv
    # with EOF, and the send loop independently detects a dead client via its
    # own sendall failure (same reasoning as camera_rig_controller_ardupilot.py).
    srv.settimeout(None)
    srv.bind(('0.0.0.0', BRIDGE_PORT))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    while not rospy.is_shutdown():
      try:
        conn, _ = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      rospy.loginfo(PKG_NAME + ": Bridge client connected")
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
      rospy.loginfo(PKG_NAME + ": Bridge client disconnected")

  def serveClient(self, conn):
    # Outbound-only channel -- still need to detect client disconnect, so
    # keep reading (and discarding) until EOF or error.
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Bridge client recv error: " + repr(e))
        return
      if not data:
        rospy.loginfo(PKG_NAME + ": Bridge client closed connection (EOF)")
        return


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = AiTargetingControllerArdupilot()
  node.run()
