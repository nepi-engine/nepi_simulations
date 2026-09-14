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

# Simulator-side bridge entry point (Universal Simulator Bridge, Phase 1/3a).
# Runs against this dev VM's own local roscore -- it is a plain ROS node with
# no nepi_sdk dependency, since the NEPI SDK is not installed on the sim VM.
# Publishes a liveness heartbeat, relays NEPI-namespace velocity commands
# to the Gazebo diff-drive plugin's topic, and (Phase 3a) dials out to the
# remote NEPI device's cross-machine command/telemetry bridge port: the
# device's rbx_sim_node.py cannot see this VM's ROS graph (separate
# masters), so this side reaches it over a plain TCP socket instead.
#
# 2026-09-08 -- DIRECTION REVERSED: this used to LISTEN (0.0.0.0:BRIDGE_PORT)
# and rbx_sim_node.py dialed in, forwarded by a reverse SSH tunnel when the
# VM had no direct route. That requires the VM to accept an unsolicited
# inbound connection, which a very common real setup -- Windows + WSL2 --
# blocks by default even with mirrored networking (Windows Firewall's
# Public-profile default), and per-machine firewall exceptions don't scale.
# Outbound is never blocked, so this VM now dials the device
# (DEVICE_HOST:BRIDGE_PORT) and rbx_sim_node.py listens instead -- no
# tunnel, no firewall config, on any OS. Protocol is unchanged: newline-
# delimited JSON both ways on one persistent connection: commands in
# ({"linear_x", "angular_z"} -> /nepi/sim/cmd_vel, feeding the existing
# relay), odometry out (pushed at a fixed rate from the latest /rover/odom
# -- push, not poll, keeps the far side a bare line reader and avoids a
# round-trip per sample).
#
# Camera-rover feature addition: two more line shapes on the same socket,
# distinguished from the above by key presence rather than a mandatory "type"
# tag (kept backward compatible with the already-verified velocity/telemetry
# shapes above, which carry none):
#   in  -- {"type":"camera_settings","offset_x":...,"scene_offset_x":...}
#           from rbx_sim_node.py's settings mechanism -- camera_offset_x/y/z
#           (robot view) and scene_offset_x/y/z (scene view), applied by
#           editing generic_rover/model.sdf's camera_link/camera_link_chase
#           <pose> and respawning the rover (see applyCameraSettings/
#           respawnRoverWithCameraOffsets below). No view_mode field any
#           more (2026-08-18): both camera views used to be relayed on ONE
#           topic, switched by a view_mode RBX setting pushed here as a
#           ROS param for camera_rig_controller.py to read -- reworked so
#           both are always-live, separately-named topics instead (see that
#           file's own module docstring for the full reasoning), leaving
#           nothing left for this node to forward but the offsets.
#   out -- {"type":"image","camera":"robot_color"|"scene_color"|"robot_depth"|
#           "scene_depth","format":"jpeg","data":"<base64>","stamp":...}
#           relayed straight through from camera_rig_controller.py's own
#           four always-live /camera_rig/*/image_compressed topics (it owns
#           the Gazebo-facing compression/throttling; this node only owns
#           the network relay, same division of labor as the existing odom
#           -> telemetry path). The "camera" field is what lets
#           rbx_sim_node.py route each frame to the matching one of its own
#           published ROS topics.
#
# RESET_SIM go-action addition: a third line shape, {"type":"reset"} in, no
# reply out. Unlike ArduPilot's RESET_SIM (which reaches gz_reset_listener.py
# directly over its own socket), this rover has no autopilot/FDM in the way,
# so the reset is just an instant /gazebo/set_model_state teleport of the
# rover model back to its generic_rover.world spawn pose -- the same
# non-blocking-topic mechanism camera_rig_controller.py already uses to move
# the follow-cam smoothly, confirmed there to not fight the physics solver.
#
# OBSTACLE_COURSE_ON/OFF setup-action addition: a fourth line shape,
# {"type":"obstacle_course","enabled":bool} in, no reply out. Swapping the
# whole world file would mean tearing down and relaunching gzserver (drops
# every existing ROS connection, heavyweight for what's meant to be a live
# RUI toggle), so instead this spawns/deletes the standalone
# models/obstacle_course/model.sdf (chicane walls + ramp bump) into the
# already-running generic_rover.world session via the stock
# /gazebo/spawn_sdf_model and /gazebo/delete_model services -- the same
# geometry `generic_rover_obstacle_course.world` includes for standalone
# testing, single source of truth, just spawned live instead of baked into
# the world file. `sim_rover_gazebo` (the default "basic room" command) still
# always launches plain generic_rover.world with no obstacles.
#
# Manual per-motor control (RBX_EXTERNAL_HARDWARE_INTERFACES.md worked
# example, section 6) is implemented entirely on the rbx_sim_node.py side:
# it folds the left/right-ratio-to-Twist conversion into the SAME
# continuously-running 20Hz control loop that already sends goto/idle
# velocity commands (gotoControlCb), then sends the result over this
# bridge's existing velocity-command shape. A one-shot motor_cmd message
# here would have been immediately overwritten by that loop's next
# (0,0)-when-idle tick, since it always sends every 20Hz regardless of
# whether a goto is active -- confirmed live during development. No new
# bridge message shape needed as a result.

import base64
import json
import math
import os
import re
import socket
import subprocess
import threading
import time

import rospy

from std_msgs.msg import Header
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties

# Sibling module in this same scripts/ directory -- see its own docstring
# (the obstacle_course spawn/delete pattern used to be hand-copied here and
# in sim_connector_bridge_gazebo.py; this is the shared, generalized version,
# see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 7).
import environment_models

PKG_NAME = 'SIM_BRIDGE'  # Use in display menus
FILE_TYPE = 'NODE'

NODE_NAME = 'sim_bridge_node'

HEARTBEAT_TOPIC = '/sim/heartbeat'
HEARTBEAT_FRAME_ID = 'gazebo_simulation'
HEARTBEAT_INTERVAL_SEC = 1.0

NEPI_CMD_VEL_TOPIC = '/nepi/sim/cmd_vel'
GAZEBO_CMD_VEL_TOPIC = '/rover/cmd_vel'
GAZEBO_ODOM_TOPIC = '/rover/odom'

# Command/telemetry bridge port: next free port after the 9022 heartbeat
# port in the 902x sim-utility block (9021 gz reset, 9022 heartbeat), clear
# of the 576x MAVLink ports. This is now the port rbx_sim_node.py LISTENS
# on -- see BRIDGE_RECONNECT_INTERVAL_SEC's comment for why.
BRIDGE_PORT = 9023
# The NEPI device's own reachable address to dial for the bridge
# connection -- same env var and default ("nepi", meant to resolve via
# whatever ~/.ssh/config / /etc/hosts entry the operator's one-time device
# SSH setup already created) as nepi_tunnel()'s device_host in
# nepi_sitl_dev_env.sh and sim_heartbeat_listener.py's DEVICE_HOST, reused
# rather than inventing a second variable for the same machine.
DEVICE_HOST = os.environ.get('NEPI_DEVICE_SSH_HOST', 'nepi')
BRIDGE_RECONNECT_INTERVAL_SEC = 2.0
TELEMETRY_RATE_HZ = 10.0

# camera_rig_controller.py's four always-live compressed topics (color +
# colorized depth view, for each of robot/scene) -- see the module
# docstring above for why all are relayed simultaneously now instead of a
# depth_map_enabled toggle swapping one topic's content.
ROBOT_COLOR_COMPRESSED_TOPIC = '/camera_rig/robot_color/image_compressed'
SCENE_COLOR_COMPRESSED_TOPIC = '/camera_rig/scene_color/image_compressed'
ROBOT_DEPTH_COMPRESSED_TOPIC = '/camera_rig/robot_depth/image_compressed'
SCENE_DEPTH_COMPRESSED_TOPIC = '/camera_rig/scene_depth/image_compressed'

# RESET_SIM target: generic_rover.world's containing <model> name and its
# (unmodified, default) spawn pose -- world origin, identity orientation.
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
ROVER_MODEL_NAME = 'generic_rover_demo'
# Name a customized-offset respawn switches to -- see
# respawnRoverWithCameraOffsets' own comment for why reusing ROVER_MODEL_NAME
# itself is not safe.
ROVER_MODEL_NAME_CUSTOM = 'generic_rover_demo_custom'

SPAWN_MODEL_SERVICE = '/gazebo/spawn_sdf_model'
DELETE_MODEL_SERVICE = '/gazebo/delete_model'
GET_WORLD_PROPERTIES_SERVICE = '/gazebo/get_world_properties'
# Resets every model in the world (poses, linear/angular velocities, AND
# each joint's own position/velocity) back to its spawn state -- see
# resetRover's own comment for why this replaced a pose-only teleport.
# Deliberately reset_world, not reset_simulation: the latter also zeroes
# sim time, which is fine for this static-environment rover world but is
# the exact hazard gz_reset_listener.py's own docstring documents for the
# ArduPilot quadcopter target (a time discontinuity crashes SITL's FDM
# link) -- reset_world avoids that class of problem entirely by leaving
# sim time alone, and this bridge is rover-only regardless.
RESET_WORLD_SERVICE = '/gazebo/reset_world'
# Poll budget for confirming a DeleteModel has actually taken effect before
# respawning under the same name -- see respawnRoverWithCameraOffsets' own
# comment for why a fixed sleep wasn't reliable.
DELETE_CONFIRM_TIMEOUT_SEC = 5.0
DELETE_CONFIRM_POLL_INTERVAL_SEC = 0.1
# Poll budget for confirming the OLD model's own camera plugins have
# actually unadvertised their ROS services (a separate, later signal than
# DELETE_CONFIRM_TIMEOUT_SEC's own get_world_properties check -- see
# _waitForOldCameraServicesGone's own comment). Same poll interval as the
# deletion check above; the timeout is shorter because this is the tail end
# of an already-confirmed deletion, not a wait for the deletion itself --
# if the plugins haven't let go within 2s of the model itself being gone,
# something is genuinely stuck and waiting longer wouldn't help.
CAMERA_SERVICE_TEARDOWN_TIMEOUT_SEC = 2.0
OLD_CAMERA_SERVICE_NAMES = ('/rover/camera/set_parameters', '/rover/camera_chase/set_parameters')

# camera_offset_*/scene_offset_* settings (rbx_sim_node.py's own robot-view /
# scene-view camera offset controls, sent here in a camera_settings line):
# applied by editing generic_rover/model.sdf's camera_link / camera_link_chase
# <pose> and respawning the whole rover model.
#
# Why a respawn rather than moving the camera at runtime -- every runtime
# option was tried live on this VM (Gazebo 11.15.1) and rejected:
#   - A cross-model fixed joint (spawning the camera as its own model welded
#     to generic_rover_demo::base_link) is SILENTLY IGNORED. Confirmed: the
#     spawn reports success, but the welded link stayed at its spawn pose
#     while the rover drove 13 m away.
#   - /gazebo/set_model_configuration and /gazebo/set_link_state (to drive a
#     3-DOF prismatic camera-mount chain built for this) both report success
#     while moving nothing -- tried with and without physics paused, and with
#     every joint-name scoping variant get_model_properties actually reports.
#   - gazebo_ros_joint_pose_trajectory (the one plugin that DID move the
#     joints) fights the physics engine's own integration of an unactuated
#     joint and drove all six camera joint states to nan within a few ticks --
#     usable for a kinematic-only model, not for links riding on a
#     physically-simulated rover.
# The two cameras stay genuinely rigid (fixed joints, zero per-tick follow
# lag) between offset changes, which is the property worth keeping; the
# respawn is the one moment they are not, and it is instant. This is viable
# specifically because generic_rover/model.sdf's diff_drive plugin uses
# <odometrySource>world</odometrySource> -- odom is read back from Gazebo's
# own model pose, so it resumes correctly after the model is recreated with
# no encoder state to lose.
ROVER_MODEL_SDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'models',
    'generic_rover', 'model.sdf')
# Same directory-relative-to-this-file convention as ROVER_MODEL_SDF_PATH
# above, for the world file _recoverDeadGzserver relaunches gzserver
# against -- see that method's own comment for why gzserver sometimes
# needs a real restart rather than just another model respawn.
GAZEBO_WORLD_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'worlds',
    'generic_rover.world')
# Raw (uncompressed) camera topics Gazebo's own camera plugins publish
# directly -- camera_rig_controller.py subscribes to these and republishes
# the compressed six-topic set this bridge relays over the wire (see the
# module docstring); checked directly here (not the compressed relay)
# since a dead sensor is a Gazebo-side problem, independent of whether
# camera_rig_controller.py itself is still running.
ROBOT_RAW_IMAGE_TOPIC = '/rover/camera/image_raw'
SCENE_RAW_IMAGE_TOPIC = '/rover/camera_chase/image_raw'
# Watchdog constants (see cameraWatchdogCb's own comment for the failure
# this recovers from -- a camera sensor silently stops publishing some
# seconds after a respawn, with everything else about it -- model,
# services -- still looking perfectly healthy, so only "did a frame
# actually arrive recently" catches it). CAMERA_DEAD_THRESHOLD_SEC is
# generous relative to the ~15Hz these publish at normally (see
# generic_rover/model.sdf's own update_rate) so a merely-busy Gazebo isn't
# mistaken for a dead one. RESPAWN_GRACE_SEC covers a respawn's own brief,
# normal gap in both topics (delete, wait, spawn) -- confirmed live this
# alone can be several seconds, well past one CAMERA_WATCHDOG_PERIOD_SEC
# tick, so the watchdog must not fire mid-respawn.
CAMERA_WATCHDOG_PERIOD_SEC = 3.0
# 3.0s, not the original 8.0 -- confirmed live (2026-09-08) against a real
# ~15Hz stream (frames arriving every ~67ms) that a genuinely dead/degraded
# camera has to be caught well before 8s of silence feels laggy to whoever's
# watching the RUI's live feed. 3s is still ~45 missed frames' worth of
# margin above normal jitter, so a merely-busy Gazebo tick still isn't
# mistaken for a dead one.
CAMERA_DEAD_THRESHOLD_SEC = 3.0
RESPAWN_GRACE_SEC = 12.0
# Settle time after gzserver's own process exists before trusting its
# services/models -- same value and reasoning as every existing launch
# script's own post-boot sleep (nepi_sitl_dev_env.sh, sim_rover_dev_env.sh,
# simulator_launch_targets.yaml's gazebo_rover launch_command all use 8s
# here, for the same "process exists" vs "actually finished loading the
# world and initializing its ROS API plugin" gap).
GAZEBO_RESTART_SETTLE_SEC = 8.0
GAZEBO_PROCESS_WAIT_SEC = 15.0
# Matches generic_rover/model.sdf's own hard-coded camera_link/camera_link_chase
# poses exactly, and rbx_sim_node.py's own FACTORY_SETTINGS for the same ten
# values -- see applied_camera_offsets' own comment for why this matters.
# Widened (2026-09-03) from 6 values (position only) to 10 (position + yaw +
# tilt/pitch per camera) -- reported live: "yaw and tilt should also be
# editable." Roll is not one of the ten: it stays fixed at 0 for both
# cameras, matching generic_rover/model.sdf's own convention, where neither
# camera has ever had roll. camera_link_chase's own factory tilt reproduces
# its hard-coded downward-look pitch (see rbx_sim_node.py's
# FACTORY_SCENE_TILT_DEG, the single source of truth for that value in
# degrees -- duplicated here as a literal only because this file cannot
# import that class; computed the same way, atan2(FACTORY_SCENE_OFFSET_Z,
# -FACTORY_SCENE_OFFSET_X), not hardcoded, so the two can't drift apart
# again the way they did before 2026-09-08); camera_link's factory tilt is
# 0 (no rotation in the stock model at all).
#
# scene_offset_x/y/z's own three values changed (2026-09-04) from the
# absolute mount pose (-2.5, 0.0, 1.65) to 0.0/0.0/0.0 -- requested live:
# "for the scene view cam, the 0 0 0 point should be set to wherever the
# cam is by default, not where the center of the robot is, so its easier
# for viewers to refer off that." scene_offset_x/y/z is now a DELTA from
# the factory mount point (FACTORY_SCENE_OFFSET_X/Y/Z below, added back in
# by respawnRoverWithCameraOffsets before writing the actual SDF pose),
# not an absolute rover-frame coordinate -- see rbx_sim_node.py's own
# matching FACTORY_SCENE_OFFSET_X/Y/Z comment, the single source of truth
# for these three values (duplicated here as literals for the same reason
# FACTORY_SCENE_TILT_DEG already is: this file cannot import that class).
# camera_offset_x/y/z (robot view) changed the same way (2026-09-08,
# requested live: "the 0 0 0 position for the robot view cam offset should
# also be where it is by default on the rover... make it like what it is
# for the scene view so its not confusing") -- see rbx_sim_node.py's own
# matching FACTORY_CAMERA_OFFSET_X/Y/Z comment, the single source of truth.
FACTORY_SCENE_OFFSET_X = -2.5
FACTORY_SCENE_OFFSET_Y = 0.0
FACTORY_SCENE_OFFSET_Z = 1.65
FACTORY_CAMERA_OFFSET_X = 0.2
FACTORY_CAMERA_OFFSET_Y = 0.0
FACTORY_CAMERA_OFFSET_Z = 0.65
# Shared horizontal FOV for both cameras -- see rbx_sim_node.py's
# FACTORY_CAMERA_FOV_DEG, the single source of truth (duplicated here as a
# literal for the same reason as the constants above). Runtime-adjustable
# (2026-09-08, requested live: "changing fov settings doesn't seem to do
# anything") via the same respawn mechanism as the pose offsets.
FACTORY_CAMERA_FOV_DEG = 80.0
FACTORY_CAMERA_OFFSETS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          math.degrees(math.atan2(FACTORY_SCENE_OFFSET_Z, -FACTORY_SCENE_OFFSET_X)),
                          FACTORY_CAMERA_FOV_DEG)
# Matches a <link name="LINKNAME"> immediately followed by its own <pose>
# element, capturing everything up to (group 1) and including "</pose>"
# (implicitly, via the non-capturing replacement below) -- unlike the
# previous 6-value version, this no longer preserves any existing rotation:
# ALL SIX pose components (x y z roll pitch yaw) are now supplied by
# respawnRoverWithCameraOffsets, roll always 0.0.
# How long applyCameraSettings waits for more changes before actually
# respawning -- see camera_respawn_timer's own comment for why this exists.
# Long enough to absorb a multi-field edit (offset + recomputed yaw/tilt,
# three separate Setting updates a few hundred ms apart in practice) into
# one respawn; short enough that a single manual edit still feels immediate.
CAMERA_RESPAWN_DEBOUNCE_SEC = 0.6
CAMERA_LINK_POSE_RE = {
  'camera_link': re.compile(
      r'(<link name="camera_link">\s*<pose>)\s*'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s+'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*(</pose>)'),
  'camera_link_chase': re.compile(
      r'(<link name="camera_link_chase">\s*<pose>)\s*'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s+'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*(</pose>)'),
}
# Same shape as CAMERA_LINK_POSE_RE, for the <horizontal_fov> each camera's
# own <camera name="..."> block carries -- both rover_camera (robot view)
# and rover_camera_chase (scene view) get the SAME fov_deg value, matching
# generate_model_sdf.py's own single shared camera_horizontal_fov_deg field.
CAMERA_FOV_RE = {
  'rover_camera': re.compile(
      r'(<camera name="rover_camera">\s*<horizontal_fov>)'
      r'[-0-9.eE]+(</horizontal_fov>)'),
  'rover_camera_chase': re.compile(
      r'(<camera name="rover_camera_chase">\s*<horizontal_fov>)'
      r'[-0-9.eE]+(</horizontal_fov>)'),
}
GAZEBO_SERVICE_WAIT_SEC = 5.0


#########################################
# Node Class
#########################################

class SimBridgeNode:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.heartbeat_pub = rospy.Publisher(HEARTBEAT_TOPIC, Header, queue_size=1)
    self.gazebo_cmd_pub = rospy.Publisher(GAZEBO_CMD_VEL_TOPIC, Twist, queue_size=1)
    self.nepi_cmd_pub = rospy.Publisher(NEPI_CMD_VEL_TOPIC, Twist, queue_size=1)
    self.cmd_sub = rospy.Subscriber(NEPI_CMD_VEL_TOPIC, Twist, self.cmdCb)
    self.odom_sub = rospy.Subscriber(GAZEBO_ODOM_TOPIC, Odometry, self.odomCb)
    # Camera-rover feature: camera_rig_controller.py owns compression/rate
    # throttling on its own topics; this node only relays whatever arrives,
    # from all six simultaneously-live feeds.
    self.robot_color_sub = rospy.Subscriber(ROBOT_COLOR_COMPRESSED_TOPIC, CompressedImage,
                                            self.robotColorImageCompressedCb)
    self.scene_color_sub = rospy.Subscriber(SCENE_COLOR_COMPRESSED_TOPIC, CompressedImage,
                                            self.sceneColorImageCompressedCb)
    self.robot_depth_sub = rospy.Subscriber(ROBOT_DEPTH_COMPRESSED_TOPIC, CompressedImage,
                                            self.robotDepthImageCompressedCb)
    self.scene_depth_sub = rospy.Subscriber(SCENE_DEPTH_COMPRESSED_TOPIC, CompressedImage,
                                            self.sceneDepthImageCompressedCb)
    self.model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)

    # Camera-sensor-death watchdog -- see cameraWatchdogCb's own comment for
    # what this recovers from. Persistent subscribers (not a repeated
    # rospy.wait_for_message check) just tracking "when did a frame last
    # arrive" -- cheap, and avoids creating/tearing down a temporary
    # subscriber every check cycle. last_respawn_time gates the watchdog
    # off entirely for RESPAWN_GRACE_SEC after any respawn, since a
    # respawn's own delete+spawn cycle is a normal, if brief, gap in both
    # topics that must never be mistaken for the sensors actually dying.
    self.last_robot_raw_frame_time = time.time()
    self.last_scene_raw_frame_time = time.time()
    self.last_respawn_time = 0.0
    self.robot_raw_sub = rospy.Subscriber(ROBOT_RAW_IMAGE_TOPIC, Image,
                                          lambda msg: setattr(self, 'last_robot_raw_frame_time', time.time()))
    self.scene_raw_sub = rospy.Subscriber(SCENE_RAW_IMAGE_TOPIC, Image,
                                          lambda msg: setattr(self, 'last_scene_raw_frame_time', time.time()))
    self.camera_watchdog_timer = rospy.Timer(rospy.Duration(CAMERA_WATCHDOG_PERIOD_SEC), self.cameraWatchdogCb)

    # Environment model spawn/delete-by-name, generalized from the old
    # single-hardcoded-obstacle_course toggle -- see environment_models.py.
    # Tracks which model (if any) Gazebo actually has spawned so repeated
    # same-value settings (e.g. a stale RUI double-click) don't send a
    # doomed second spawn (name collision) or delete (already gone).
    self.env_spawner = environment_models.EnvironmentModelSpawner(log_prefix = PKG_NAME)

    # Rover model SDF, read once here for the same reason as the obstacle
    # course file above (the file's structure never changes at runtime -- only
    # the two camera <pose> values get substituted per offset change, see
    # applyCameraSettings). self.applied_camera_offsets tracks the last offsets
    # actually baked into the live model, so a camera_settings line carrying
    # the same offsets as last time (e.g. a redundant resend triggered by
    # Nepi_IF_SimLauncher's own robot-config re-send fix) does not trigger a
    # pointless respawn.
    #
    # Initialized to FACTORY_CAMERA_OFFSETS, NOT None -- found live
    # (2026-08-18) as the actual root cause of a duplicate-plugin-load Gazebo
    # bug hitting essentially every single deploy: rbx_sim_node.py's
    # bridgeLoop unconditionally sends its current camera settings once on
    # every fresh connect (see that method's own comment), and with this
    # starting as None, that FIRST sync message -- even carrying nothing but
    # untouched factory-default offsets -- always failed the "already
    # applied" dedup check and triggered a real respawnRoverWithCameraOffsets
    # call. That respawn's delete+spawn raced the world file's own initial
    # <include> spawn (still mid-plugin-init at ~2-3s into boot), producing
    # "Tried to advertise a service that is already advertised" errors and,
    # observed live, both camera topics ending up with zero active
    # publishers. Starting this at the actual factory values (verified to
    # exactly match rbx_sim_node.py's own FACTORY_SETTINGS -- scene_offset_x/
    # y/z is 0.0 in both now, meaning "no delta from the factory
    # camera_link_chase mount point", not a byte-identical pose string
    # anymore since FACTORY_SCENE_OFFSET_X/Y/Z's own comment; camera_link's
    # own values are still the real absolute pose, unaffected by that
    # change) means an UNCUSTOMIZED deploy -- the common case -- now
    # legitimately skips this first respawn entirely. A deploy with
    # genuinely customized offsets still respawns once, which is correct:
    # that respawn is real work this mechanism exists to do.
    try:
      with open(ROVER_MODEL_SDF_PATH, 'r') as f:
        self.rover_sdf_template = f.read()
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Failed to read rover SDF at " +
                    ROVER_MODEL_SDF_PATH + ": " + str(e) +
                    " -- camera offset changes will be ignored")
      self.rover_sdf_template = None
    self.applied_camera_offsets = FACTORY_CAMERA_OFFSETS

    # Which Gazebo model name is CURRENTLY live -- starts as ROVER_MODEL_NAME
    # (the world file's own <include> spawns it under this name at boot) but
    # permanently switches to ROVER_MODEL_NAME_CUSTOM the first time
    # respawnRoverWithCameraOffsets actually respawns with customized
    # offsets. Confirmed live (2026-08-18) as necessary, not cosmetic: Gazebo
    # Classic appears to cache a world-file <include>-sourced model's
    # geometry keyed by its instance name, so a LATER spawn_sdf_model call
    # reusing that exact name silently reuses the ORIGINAL cached geometry
    # regardless of the new SDF text provided -- confirmed directly (delete
    # + respawn "generic_rover_demo" with a modified camera pose left the
    # live link at its old pose every time; the identical delete+respawn
    # sequence under a name that never came from an <include> applied the
    # new pose correctly, every time, including on repeated reuse of that
    # SAME non-<include> name). All later Gazebo calls that need "whichever
    # rover model is live right now" (RESET_SIM, holdStill, further
    # respawns) must read this instead of the ROVER_MODEL_NAME constant.
    self.rover_model_name = ROVER_MODEL_NAME

    # Latest odom snapshot for the telemetry push loop, and the single
    # active bridge client (one robot, one remote node -- serve one
    # connection at a time; a reconnect is picked up after the dead one
    # is detected and torn down).
    self.latest_telemetry = None
    self.client_conn = None
    self.client_lock = threading.Lock()

    # Debounces respawnRoverWithCameraOffsets -- see applyCameraSettings's
    # own comment for why this exists (2026-09-08, confirmed live: rapid
    # multi-field camera edits, e.g. the RUI's "Lock Scene Camera To Robot"
    # sending an offset plus a recomputed yaw+tilt as three separate Setting
    # updates within about a second, raced multiple overlapping
    # delete+spawn cycles against each other and crashed gzserver outright,
    # not just the milder "already advertised" duplicate-registration
    # symptom a single stray extra respawn produces).
    self.pending_camera_offsets = None
    self.camera_respawn_timer = None
    self.camera_respawn_lock = threading.Lock()
    # Serializes actual respawnRoverWithCameraOffsets EXECUTION, a separate
    # concern from camera_respawn_lock above (which only serializes
    # SCHEDULING). Confirmed live (2026-09-08) that the debounce alone isn't
    # enough: a fresh connect's own initial sendCameraSettings() (resyncing
    # whatever settings were last persisted -- can differ from this VM's
    # own FACTORY_CAMERA_OFFSETS sentinel) and an operator's own settings
    # edit moments later each schedule their OWN timer; the debounce window
    # (0.6s) is far shorter than one delete+wait-for-services-gone+spawn
    # cycle actually takes, so the second timer fires and starts a SECOND
    # respawn while the first is still mid-flight on its own thread -- two
    # concurrent delete/spawn cycles racing each other is exactly what
    # produces "already advertised" and, worse, silently dead camera
    # topics afterward. Held for the full body of
    # respawnRoverWithCameraOffsets so a second respawn genuinely waits for
    # the first to finish rather than running alongside it.
    self.camera_respawn_inflight_lock = threading.Lock()
    # Guards against _recoverDeadGzserver's own re-applying respawn
    # detecting ANOTHER dead camera and recursing into a second recovery --
    # see that check's own comment.
    self._recovering_gzserver = False

    # Idle-hold anchor for holdStill() below -- captured ONCE when cmd_vel
    # first goes to exactly zero, then re-asserted unchanged on every
    # subsequent idle tick. Deliberately NOT re-read from self.latest_telemetry
    # each time: an earlier version did that and still drifted, because
    # re-anchoring to "whatever odom says right now" just locks in that
    # tick's tiny residual-velocity creep instead of preventing it -- the
    # anchor has to stay fixed across the whole idle period to actually hold
    # position, not merely re-zero velocity every tick.
    self.held_pose = None

    # Wall-clock thread, not rospy.Timer: with /use_sim_time set (the
    # gazebo_ros launcher sets it), a ROS timer tracks sim time -- it slows
    # with the real-time factor and stops entirely if the sim is paused,
    # which would falsely read as "simulator dead" to a liveness consumer.
    self.heartbeat_thread = threading.Thread(target=self.heartbeatLoop)
    self.heartbeat_thread.daemon = True
    self.heartbeat_thread.start()

    # Bridge server + telemetry push threads (wall-clock for the same
    # reason as the heartbeat: the push doubles as connection liveness).
    self.server_thread = threading.Thread(target=self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()
    self.telemetry_thread = threading.Thread(target=self.telemetryPushLoop)
    self.telemetry_thread.daemon = True
    self.telemetry_thread.start()

    rospy.loginfo(PKG_NAME + ": Simulator Bridge Node initialized")
    rospy.loginfo(PKG_NAME + ": Heartbeat on " + HEARTBEAT_TOPIC)
    rospy.loginfo(PKG_NAME + ": Relaying " + NEPI_CMD_VEL_TOPIC +
                  " -> " + GAZEBO_CMD_VEL_TOPIC)
    rospy.loginfo(PKG_NAME + ": Bridge server on 127.0.0.1:" +
                  str(BRIDGE_PORT))

  def run(self):
    """Block until ROS shutdown, servicing the heartbeat timer and the
    command relay subscriber."""
    rospy.spin()

  def heartbeatLoop(self):
    while not rospy.is_shutdown():
      hdr = Header()
      hdr.stamp = rospy.Time.now()
      hdr.frame_id = HEARTBEAT_FRAME_ID
      self.heartbeat_pub.publish(hdr)
      time.sleep(HEARTBEAT_INTERVAL_SEC)

  def cmdCb(self, msg):
    self.gazebo_cmd_pub.publish(msg)
    # rbx_sim_node.py's gotoControlCb sends a fresh velocity command every
    # control tick regardless of active/idle, defaulting to (0,0) when there's
    # no goto in progress -- a zero Twist here means "hold position", not just
    # "no active command yet". See holdStill() for why that needs enforcing
    # at the model level, not just left to the diff-drive plugin's wheel motors.
    if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
      if self.held_pose is None:
        self.held_pose = self.captureCurrentPose()
      self.holdStill()
    else:
      # Actively driving again -- drop any stale anchor so the next time
      # cmd_vel returns to zero, a fresh one gets captured at wherever the
      # rover actually stopped, not wherever it was idle before this move.
      self.held_pose = None

  def captureCurrentPose(self):
    if self.latest_telemetry is None:
      return None
    return {
      'x': self.latest_telemetry['x'],
      'y': self.latest_telemetry['y'],
      'yaw': self.latest_telemetry['yaw'],
    }

  def odomCb(self, msg):
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Planar rover: yaw is all the remote scaffold needs (math, not tf,
    # to keep this node's dependencies minimal)
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

  def robotColorImageCompressedCb(self, msg):
    self.imageCompressedCb(msg, 'robot_color')

  def sceneColorImageCompressedCb(self, msg):
    self.imageCompressedCb(msg, 'scene_color')

  def robotDepthImageCompressedCb(self, msg):
    self.imageCompressedCb(msg, 'robot_depth')

  def sceneDepthImageCompressedCb(self, msg):
    self.imageCompressedCb(msg, 'scene_depth')

  def imageCompressedCb(self, msg, camera):
    # Relayed straight through to whichever client is connected right now;
    # dropped silently if none is (matches the existing "no client" behavior
    # of sendVelocityCmd's counterpart on the remote-device side). "camera"
    # tags which of the four always-live feeds this frame came from, so
    # rbx_sim_node.py can route it to the matching one of its own published
    # ROS topics. "format" is relayed too (camera_rig_controller.py's own
    # msg.format, always 'jpeg' now that the raw depth_map feeds are gone).
    line = {
      'type': 'image',
      'camera': camera,
      'format': msg.format,
      'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
      'stamp': msg.header.stamp.to_sec(),
    }
    self.sendLineToClient(line)

  def sendLineToClient(self, line_dict):
    # Holds client_lock across the actual sendall, not just the self.client_conn
    # read -- imageCompressedCb (image subscriber thread) and telemetryPushLoop
    # (its own thread) both write to the same TCP stream socket. Without one
    # lock serializing the writes themselves, two sendall calls landing at
    # once can interleave their bytes on the wire, corrupting the
    # newline-delimited JSON stream (the receiver's json.loads then silently
    # drops the mangled line -- see rbx_sim_node.py's processBridgeLine).
    with self.client_lock:
      conn = self.client_conn
      if conn is None:
        return
      try:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        rospy.logwarn_throttle(5.0, PKG_NAME + ": Failed to send line to client: " + str(e))
        if self.client_conn is conn:
          self.client_conn = None
        # shutdown() before close(): the accept loop's recv() (a different
        # thread) is almost certainly blocked reading this exact socket --
        # closing a fd out from under a thread blocked in recv() on it does
        # not reliably unblock that recv() on Linux. Without this, the
        # accept loop can stay wedged and refuse the client's next
        # reconnect attempt indefinitely. See camera_rig_controller_
        # ardupilot.py's sendLineToClient for the same fix and full
        # reasoning (found while chasing the quadcopter camera's
        # flicker-in-and-out bug; this rover bridge has the identical
        # send-thread/recv-thread split, so the same fix applies here).
        try:
          conn.shutdown(socket.SHUT_RDWR)
        except Exception:
          pass
        try:
          conn.close()
        except Exception:
          pass

  def resetRover(self):
    # Stop first so the reset pose/velocity isn't immediately fought by the
    # diff-drive plugin still applying the last commanded velocity.
    self.gazebo_cmd_pub.publish(Twist())
    # ModelState only ever covered the rover's own TOP-LEVEL pose+twist --
    # it never touched each wheel joint's own velocity, which the
    # diff-drive plugin's per-wheel PID drives independently. Reported live
    # (2026-09-02): "if the rover is a little discombobulated... it doesn't
    # stabilize it, it continues to glitch around" -- a rover that had been
    # tumbling/spinning kept its wheels' own residual angular velocities
    # through a ModelState-only reset, so they kept driving the
    # freshly-repositioned body for the next several physics steps instead
    # of actually being at rest. /gazebo/reset_world resets EVERY model to
    # its spawn state -- pose, linear/angular velocity, AND every joint's
    # own position/velocity (Gazebo's own Model::Reset()/Joint::Reset(),
    # not something this bridge has to reconstruct field-by-field) -- so
    # this is a genuine full reset, not a best-effort approximation of one.
    # Best-effort or not, the call itself is guarded: a missing/slow
    # service must not crash the bridge's own command-handling thread.
    try:
      rospy.wait_for_service(RESET_WORLD_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      rospy.ServiceProxy(RESET_WORLD_SERVICE, Empty)()
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": /gazebo/reset_world failed (falling back to "
                    "pose-only reset): " + str(e))
    # Also explicitly re-publish the target pose via ModelState -- reset_world
    # already lands the rover at its spawn pose, but this stays as the
    # fallback path if that service call above failed, and as a defensive
    # belt-and-suspenders re-assertion either way (matches the exact target
    # held_pose is set to just below).
    state = ModelState()
    state.model_name = self.rover_model_name
    state.pose.orientation.w = 1.0
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)
    # Set the idle anchor DIRECTLY to the reset target (origin), not None --
    # found live (2026-08-18) as the actual reason RESET_SIM appeared to do
    # nothing. rbx_sim_node.py's gotoControlCb sends a fresh (0,0) idle
    # cmd_vel at 20Hz regardless of activity, and cmdCb only re-captures
    # held_pose from live telemetry when it is None (see cmdCb's own
    # comment). Clearing it to None here left a race: an idle tick landing
    # before Gazebo's own physics step had processed this ModelState publish
    # would re-capture the STALE pre-reset position from /rover/odom into
    # held_pose, and the very next holdStill() tick would republish that
    # stale position via set_model_state -- silently undoing the reset
    # within milliseconds, consistently enough to look like RESET_SIM simply
    # didn't work. Setting held_pose to the actual reset target here closes
    # that window: any holdStill() tick landing during the race now
    # reasserts the CORRECT already-reset pose instead of re-reading
    # (possibly stale) telemetry.
    self.held_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

  def holdStill(self):
    # Confirmed live (get_joint_properties on all 4 wheel joints, with
    # cmd_vel verified rock-solid at exactly zero) that the rover still drifts
    # slowly in position and yaw even fully idle: each wheel joint's
    # velocity-controlled motor settles to a small residual angular velocity
    # instead of exactly zero, and the four residuals aren't even
    # symmetric side-to-side. Four simultaneous wheel-ground friction
    # contacts against a perfectly flat plane is a slightly over-determined
    # constraint problem for ODE's iterative "quick" solver, which doesn't
    # converge to exact zero every step; that tiny per-step error integrates
    # into a real, slowly-growing position/heading drift over time even
    # though the commanded velocity never leaves zero. Bumping solver
    # iterations made this worse, not better (tested live), and there's no
    # persistent-SDF equivalent of the ODE auto-disable-bodies knob to fall
    # back on.
    #
    # Re-asserts self.held_pose -- a FIXED anchor captured once when cmd_vel
    # first went to zero (see cmdCb), not a fresh read of self.latest_telemetry
    # each call. An earlier version re-read live telemetry every tick and
    # still drifted just as much: re-anchoring to "whatever odom says right
    # now" only locks in that tick's residual-velocity creep as the new
    # baseline instead of preventing it, so the drift kept accumulating one
    # tiny confirmed step at a time. Holding one fixed value across the whole
    # idle period is what actually stops it. Same non-blocking
    # /gazebo/set_model_state mechanism resetRover (above) and
    # camera_rig_controller.py's follow-cam already use, confirmed elsewhere
    # in this codebase not to fight the physics solver.
    if self.held_pose is None:
      return
    state = ModelState()
    state.model_name = self.rover_model_name
    state.pose.position.x = self.held_pose['x']
    state.pose.position.y = self.held_pose['y']
    yaw = self.held_pose['yaw']
    state.pose.orientation.z = math.sin(yaw / 2.0)
    state.pose.orientation.w = math.cos(yaw / 2.0)
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)

  def applyCameraSettings(self, cmd):
    # offset_x/y/z/yaw/tilt (robot view), scene_offset_x/y/z/yaw/tilt
    # (scene/chase view), and fov_deg are optional in this wire message:
    # absent on any deployment still running an older rbx_sim_node.py that
    # predates yaw/tilt/fov. get() with None sentinels, then bail without
    # touching anything already applied, rather than defaulting to 0.0/some
    # fixed FOV and silently snapping the cameras the first time an old
    # sender's message arrives.
    keys = ('offset_x', 'offset_y', 'offset_z', 'offset_yaw', 'offset_tilt',
            'scene_offset_x', 'scene_offset_y', 'scene_offset_z',
            'scene_offset_yaw', 'scene_offset_tilt', 'fov_deg')
    if any(cmd.get(k) is None for k in keys):
      return
    try:
      offsets = tuple(float(cmd[k]) for k in keys)
    except (TypeError, ValueError) as e:
      rospy.logwarn(PKG_NAME + ": Ignoring malformed camera offsets: " + str(e))
      return
    if offsets == self.applied_camera_offsets:
      return  # Already live -- e.g. a redundant resend of the same offsets.
    # Debounced, not respawned immediately -- see CAMERA_RESPAWN_DEBOUNCE_SEC's
    # own comment: a multi-field edit (e.g. "Lock Scene Camera To Robot"
    # sending an offset plus a recomputed yaw+tilt as three separate Setting
    # updates in quick succession) used to trigger one respawn PER update,
    # racing overlapping delete+spawn cycles against each other and
    # crashing gzserver outright (confirmed live 2026-09-08). Each call here
    # just replaces the pending offsets and restarts the timer, so only the
    # LAST state in a burst actually respawns, once, after things settle.
    with self.camera_respawn_lock:
      self.pending_camera_offsets = offsets
      if self.camera_respawn_timer is not None:
        self.camera_respawn_timer.cancel()
      self.camera_respawn_timer = threading.Timer(
          CAMERA_RESPAWN_DEBOUNCE_SEC, self.respawnPendingCameraOffsets)
      self.camera_respawn_timer.daemon = True
      self.camera_respawn_timer.start()

  def respawnPendingCameraOffsets(self):
    with self.camera_respawn_lock:
      offsets = self.pending_camera_offsets
      self.camera_respawn_timer = None
    if offsets is not None and offsets != self.applied_camera_offsets:
      self.respawnRoverWithCameraOffsets(offsets)

  def _waitForOldCameraServicesGone(self):
    # get_world_properties dropping old_name from its model list (the check
    # respawnRoverWithCameraOffsets already does before calling this) fires
    # as soon as Gazebo's OWN bookkeeping removes the model -- confirmed
    # live (2026-09-08) that this is EARLIER than the model's
    # libgazebo_ros_openni_kinect.so plugin instances actually deregistering
    # their own /rover/camera/set_parameters and /rover/camera_chase/
    # set_parameters services from the ROS master. Spawning the replacement
    # before that finishes hits "Tried to advertise a service that is
    # already advertised" for the NEW plugin instances, and (unlike a
    # cosmetic log warning) that failure silently kills the new camera
    # topics' own publishers -- confirmed live: both /rover/camera/image_raw
    # and /rover/camera_chase/image_raw went from a healthy publisher to
    # zero after exactly this race. Polls the ROS master's own system state
    # (the actual source of truth for "is this service still registered",
    # not a proxy for it) rather than sleeping a guessed fixed delay.
    # Best-effort: proceeds after CAMERA_SERVICE_TEARDOWN_TIMEOUT_SEC even
    # if a service is still listed, same "don't block forever" reasoning as
    # the model-deletion poll above -- spawning anyway is still better than
    # never spawning at all, and this is the rare case, not the common one.
    master = rospy.get_master()
    deadline = time.time() + CAMERA_SERVICE_TEARDOWN_TIMEOUT_SEC
    while time.time() < deadline:
      try:
        _, _, services = master.getSystemState()[2]
        registered = {name for name, _nodes in services}
      except Exception:
        break
      if not any(name in registered for name in OLD_CAMERA_SERVICE_NAMES):
        return
      time.sleep(DELETE_CONFIRM_POLL_INTERVAL_SEC)

  def respawnRoverWithCameraOffsets(self, offsets):
    # Thin wrapper: see camera_respawn_inflight_lock's own comment for why
    # this needs to be a genuine mutex around the whole respawn, not just
    # the debounce that decides whether to call this at all. A second
    # caller blocks here until the first respawn (delete, wait for the old
    # plugins' services to really be gone, spawn) has completely finished,
    # instead of running concurrently with it on a second Timer thread.
    with self.camera_respawn_inflight_lock:
      self._respawnRoverWithCameraOffsetsLocked(offsets)

  def _respawnRoverWithCameraOffsetsLocked(self, offsets):
    if self.rover_sdf_template is None:
      rospy.logwarn(PKG_NAME + ": No rover SDF loaded, cannot apply camera offsets")
      return
    if offsets == self.applied_camera_offsets:
      # Re-checked here (applyCameraSettings/respawnPendingCameraOffsets
      # both already checked this before scheduling/calling in) because a
      # call queued up waiting on camera_respawn_inflight_lock can go stale
      # while it waits: the respawn that just finished ahead of it may have
      # already applied these exact same offsets. Without this, a blocked
      # duplicate call still ran a full, pointless second respawn the
      # instant the lock freed up.
      return
    (off_x, off_y, off_z, off_yaw_deg, off_tilt_deg,
     scene_x, scene_y, scene_z, scene_yaw_deg, scene_tilt_deg, fov_deg) = offsets

    # scene_x/y/z arrive as a DELTA from the factory chase-cam mount point,
    # not an absolute rover-frame coordinate -- see FACTORY_SCENE_OFFSET_X/Y/Z's
    # own comment. Add the mount point back in here, the one place that
    # actually needs the real absolute pose (the SDF <pose> element itself);
    # everywhere else in this app (Settings, the RUI) keeps working in the
    # delta.
    scene_x = scene_x + FACTORY_SCENE_OFFSET_X
    scene_y = scene_y + FACTORY_SCENE_OFFSET_Y
    scene_z = scene_z + FACTORY_SCENE_OFFSET_Z
    # off_x/y/z (robot view) is the same delta-from-mount-point convention
    # now -- see FACTORY_CAMERA_OFFSET_X/Y/Z's own comment.
    off_x = off_x + FACTORY_CAMERA_OFFSET_X
    off_y = off_y + FACTORY_CAMERA_OFFSET_Y
    off_z = off_z + FACTORY_CAMERA_OFFSET_Z

    sdf = self.rover_sdf_template
    sdf, n1 = CAMERA_LINK_POSE_RE['camera_link'].subn(
        lambda m: m.group(1) + ("%.6f %.6f %.6f 0 %.6f %.6f " %
            (off_x, off_y, off_z, math.radians(off_tilt_deg), math.radians(off_yaw_deg))) + m.group(2), sdf)
    sdf, n2 = CAMERA_LINK_POSE_RE['camera_link_chase'].subn(
        lambda m: m.group(1) + ("%.6f %.6f %.6f 0 %.6f %.6f " %
            (scene_x, scene_y, scene_z, math.radians(scene_tilt_deg), math.radians(scene_yaw_deg))) + m.group(2), sdf)
    fov_rad = math.radians(fov_deg)
    sdf, n3 = CAMERA_FOV_RE['rover_camera'].subn(
        lambda m: m.group(1) + ("%.7f" % fov_rad) + m.group(2), sdf)
    sdf, n4 = CAMERA_FOV_RE['rover_camera_chase'].subn(
        lambda m: m.group(1) + ("%.7f" % fov_rad) + m.group(2), sdf)
    if n1 != 1 or n2 != 1 or n3 != 1 or n4 != 1:
      # A structural change to generic_rover/model.sdf (renamed link, reordered
      # pose/fov) could make one of these regexes stop matching -- fail loudly
      # rather than silently spawning the rover with its OLD/default camera
      # poses or FOV, which would look exactly like "the setting doesn't do
      # anything".
      rospy.logerr(PKG_NAME + ": Camera substitution matched " + str(n1) +
                   "/1 camera_link, " + str(n2) + "/1 camera_link_chase, " +
                   str(n3) + "/1 rover_camera fov, " + str(n4) +
                   "/1 rover_camera_chase fov -- refusing to respawn with an "
                   "unverified model")
      return

    # Capture the rover's current pose so the respawn doesn't teleport it back
    # to the world origin -- odometrySource=world means Gazebo's own model pose
    # IS the odom source, so this is the one piece of state that must survive.
    pose = self.captureCurrentPose()
    initial_pose = Pose()
    if pose is not None:
      initial_pose.position.x = pose['x']
      initial_pose.position.y = pose['y']
      initial_pose.orientation.z = math.sin(pose['yaw'] / 2.0)
      initial_pose.orientation.w = math.cos(pose['yaw'] / 2.0)
    else:
      initial_pose.orientation.w = 1.0

    # Stop first, same reasoning as resetRover: an in-flight cmd_vel would
    # otherwise be applied to the new model the instant it exists.
    self.gazebo_cmd_pub.publish(Twist())
    old_name = self.rover_model_name
    # Always respawn under the dedicated non-<include> name, whether this is
    # the first customization or a later one -- see ROVER_MODEL_NAME_CUSTOM's
    # own comment (self.rover_model_name) for the full root-cause writeup.
    # Confirmed live that reusing THIS name across repeated respawns is
    # safe (unlike ROVER_MODEL_NAME itself) -- the caching quirk is specific
    # to a name that originated from the world file's own <include>.
    new_name = ROVER_MODEL_NAME_CUSTOM
    try:
      rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)(old_name)
      # Poll get_world_properties until old_name is actually gone from the
      # model list instead of guessing a fixed delay -- DeleteModel
      # returning does not guarantee Gazebo's own (asynchronous) deletion,
      # and this model's plugins (gazebo_ros_camera x2, diff_drive)
      # unadvertising their ROS services/topics, have actually finished yet.
      rospy.wait_for_service(GET_WORLD_PROPERTIES_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      get_world_props = rospy.ServiceProxy(GET_WORLD_PROPERTIES_SERVICE, GetWorldProperties)
      deadline = time.time() + DELETE_CONFIRM_TIMEOUT_SEC
      deleted = False
      while time.time() < deadline:
        if old_name not in get_world_props().model_names:
          deleted = True
          break
        time.sleep(DELETE_CONFIRM_POLL_INTERVAL_SEC)
      if not deleted:
        rospy.logerr(PKG_NAME + ": Respawn with new camera offsets failed: " +
                     old_name + " still present " +
                     str(DELETE_CONFIRM_TIMEOUT_SEC) + "s after DeleteModel")
        return
      # get_world_properties no longer listing old_name is NOT the same as
      # its camera plugins having actually finished unadvertising their own
      # ROS services -- confirmed live (2026-09-08): a SECOND respawn
      # shortly after a first one hit "Tried to advertise a service that is
      # already advertised" for /rover/camera/set_parameters and
      # /rover/camera_chase/set_parameters, then left BOTH camera topics
      # with zero publishers (this method's own success log still fired --
      # the SpawnModel call itself succeeds, it's the new plugins' own
      # service registration that silently loses the naming collision).
      # Actually wait for those two specific services to disappear from the
      # ROS master's registry -- the real signal the old plugins are gone,
      # not a proxy for it -- before spawning the replacement.
      self._waitForOldCameraServicesGone()
      rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
      resp = spawn(new_name, sdf, '', initial_pose, 'world')
      if not resp.success:
        rospy.logerr(PKG_NAME + ": Respawn with new camera offsets failed: " +
                     resp.status_message)
        return
    except Exception as e:
      rospy.logerr(PKG_NAME + ": Respawn with new camera offsets failed: " + str(e))
      return

    self.rover_model_name = new_name
    self.applied_camera_offsets = offsets
    self.held_pose = None  # Stale anchor from before the respawn -- see resetRover.
    rospy.loginfo(PKG_NAME + ": Applied camera offsets, robot=(%.2f,%.2f,%.2f,yaw=%.1f,tilt=%.1f) "
                  "scene=(%.2f,%.2f,%.2f,yaw=%.1f,tilt=%.1f), fov=%.1fdeg, model now '%s'"
                  % (offsets + (new_name,)))
    # Camera sensor health after this respawn is checked by the periodic
    # watchdog (cameraWatchdogCb), not here -- see that method's own
    # comment for why a check performed immediately after this respawn
    # succeeds is not a reliable signal (confirmed live 2026-09-08: a
    # sensor can publish normally for the first several seconds after a
    # respawn and then stop, well after any check placed right here would
    # have already returned "fine").
    self.last_respawn_time = time.time()

  def _gzserverAlive(self):
    return subprocess.call(['pgrep', '-x', 'gzserver'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0

  def cameraWatchdogCb(self, event):
    # Confirmed live (2026-09-08): a second-or-later respawn in the same
    # gzserver process can leave a camera sensor's ROS plumbing looking
    # perfectly healthy (model exists, its /set_parameters service
    # responds, image_raw has a registered /gazebo publisher) while the
    # sensor has silently stopped actually rendering -- and this can take
    # several seconds AFTER the respawn to manifest, so a one-time check
    # performed right after a respawn succeeds is not a reliable signal (it
    # can catch the sensor still working, seconds before it stops). This
    # periodic watchdog is the actual fix: track "time since a frame last
    # arrived" continuously, independent of when/why a respawn happened,
    # and recover whenever that goes stale for too long.
    # Re-tested live (2026-09-08) after the camera_respawn_inflight_lock +
    # _waitForOldCameraServicesGone() fixes above: three sequential
    # respawns (fov 100 -> 80 -> 50, ~20s apart) all left both camera
    # topics publishing a healthy steady ~15Hz afterward, cross-checked via
    # rostopic echo, not just this class's own subscriber timestamps. The
    # earlier "still degraded after those fixes" read came from `rostopic
    # hz`, which was independently confirmed broken in this environment (it
    # reports "no new messages" even against /clock, which is always
    # alive) -- so that read was a tooling false positive, not a real
    # ongoing degradation. This watchdog stays as a safety net regardless
    # (per the user's own call, given a race here is genuinely hard to
    # fully rule out), just tuned tighter now that real data says a healthy
    # camera never sits idle anywhere near CAMERA_DEAD_THRESHOLD_SEC.
    now = time.time()
    if now - self.last_respawn_time < RESPAWN_GRACE_SEC:
      return  # a respawn's own delete+spawn gap, not a real failure
    robot_stale = now - self.last_robot_raw_frame_time > CAMERA_DEAD_THRESHOLD_SEC
    scene_stale = now - self.last_scene_raw_frame_time > CAMERA_DEAD_THRESHOLD_SEC
    if not (robot_stale or scene_stale):
      return
    if self._recovering_gzserver:
      # Already inside a recovery's own re-applying respawn (see
      # _recoverDeadGzserver's own final call) -- if a supposedly-fresh
      # gzserver STILL can't hold a camera sensor, restarting it again is
      # unlikely to help and risks looping forever. Leave it broken and
      # loud rather than silently spin.
      rospy.logerr_throttle(CAMERA_WATCHDOG_PERIOD_SEC,
                            PKG_NAME + ": Cameras still dead after a gzserver recovery attempt "
                            "-- not retrying again automatically")
      return
    rospy.logerr(PKG_NAME + ": Camera watchdog: no frame on " +
                 ("robot " if robot_stale else "") + ("scene " if scene_stale else "") +
                 "view for over " + str(CAMERA_DEAD_THRESHOLD_SEC) + "s -- restarting gzserver to recover")
    self._recoverDeadGzserver()

  def _recoverDeadGzserver(self):
    # Sets the re-entrancy guard for the WHOLE recovery (not just the final
    # re-apply-respawn step) -- the kill+relaunch+settle sequence below can
    # take 15-20+ seconds, comfortably longer than one
    # CAMERA_WATCHDOG_PERIOD_SEC tick, and last_robot_raw_frame_time/
    # last_scene_raw_frame_time stay stale (no gzserver running to publish
    # anything) for that whole window -- without the guard covering this
    # entire method, cameraWatchdogCb would see the same staleness and try
    # to start a SECOND concurrent recovery mid-recovery.
    self._recovering_gzserver = True
    try:
      self._recoverDeadGzserverImpl()
    finally:
      self._recovering_gzserver = False

  def _recoverDeadGzserverImpl(self):
    """Kills and relaunches gzserver/gzclient against the same world file,
    then re-applies whatever this bridge currently believes should be
    live -- the environment model (if any) and the current camera
    offsets/FOV -- since a fresh gzserver reloads generic_rover.world from
    scratch (the world file's own <include>, back under ROVER_MODEL_NAME,
    with none of this session's customization). roscore, this bridge
    itself, camera_rig_controller.py, and the rest of the VM-side stack
    are untouched -- only gzserver/gzclient restart, so this is seconds,
    not a full redeploy, and the device-side connection this bridge holds
    never drops.

    Best-effort throughout: logs and returns on any step failing rather
    than raising, matching this whole file's own "a bridge command failure
    should never crash the node" convention -- an operator still sees the
    dead cameras and can fall back to a fresh Deploy if this recovery
    itself doesn't pan out.
    """
    subprocess.call(['pkill', '-x', 'gzclient'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.call(['pkill', '-x', 'gzserver'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + GAZEBO_SERVICE_WAIT_SEC
    while time.time() < deadline and self._gzserverAlive():
      time.sleep(0.2)
    if self._gzserverAlive():
      rospy.logerr(PKG_NAME + ": gzserver recovery failed -- old process would not die")
      return

    try:
      subprocess.Popen(['rosrun', 'gazebo_ros', 'gazebo', GAZEBO_WORLD_PATH],
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
      rospy.logerr(PKG_NAME + ": gzserver recovery failed to relaunch: " + str(e))
      return

    deadline = time.time() + GAZEBO_PROCESS_WAIT_SEC
    while time.time() < deadline and not self._gzserverAlive():
      time.sleep(0.2)
    if not self._gzserverAlive():
      rospy.logerr(PKG_NAME + ": gzserver recovery failed -- new process never started")
      return
    time.sleep(GAZEBO_RESTART_SETTLE_SEC)

    # Fresh world -- back to the <include>-spawned default name and no
    # environment model, regardless of what this bridge thought was true a
    # moment ago.
    self.rover_model_name = ROVER_MODEL_NAME
    if self.env_spawner.spawned_name is not None:
      name = self.env_spawner.spawned_name
      self.env_spawner.spawned_name = None
      self.env_spawner.set_active_model(name)

    # Force the respawn even though offsets will equal self.
    # applied_camera_offsets (that's the whole point: re-apply the same
    # customization gzserver just forgot) -- reset the sentinel first so
    # _respawnRoverWithCameraOffsetsLocked's own already-applied guard
    # doesn't skip it.
    offsets_to_reapply = self.applied_camera_offsets
    self.applied_camera_offsets = None
    rospy.loginfo(PKG_NAME + ": gzserver recovered, re-applying camera offsets/FOV")
    # _recovering_gzserver is already True here -- set by the outer
    # _recoverDeadGzserver wrapper for this whole method's duration, not
    # re-set locally (see that wrapper's own comment for why it needs to
    # cover more than just this one call).
    self._respawnRoverWithCameraOffsetsLocked(offsets_to_reapply)

  def bridgeServerLoop(self):
    # Name kept for history -- this is a dial-out reconnect loop now, not a
    # server accept loop (see module docstring's 2026-09-08 note). Mirrors
    # the retry shape rbx_sim_node.py's OLD bridgeLoop used before the
    # direction reversed: connect, serve until disconnect/error, sleep,
    # retry -- so the device's rbx_sim node (or this VM's own sim stack) can
    # restart independently of the other and the connection just re-forms.
    while not rospy.is_shutdown():
      try:
        conn = socket.create_connection((DEVICE_HOST, BRIDGE_PORT), timeout = 5)
      except Exception as e:
        rospy.logwarn_throttle(10, PKG_NAME + ": Bridge connect to " + DEVICE_HOST +
                               ":" + str(BRIDGE_PORT) + " failed: " + str(e))
        time.sleep(BRIDGE_RECONNECT_INTERVAL_SEC)
        continue
      # rospy sets a process-global socket.setdefaulttimeout(60), which
      # create_connection above already applied to the connect itself --
      # clear it for the life of the connection. The command stream is
      # legitimately idle for long stretches (commands are sporadic), so a
      # recv timeout must not be treated as client death -- a real
      # disconnect still unblocks recv with EOF, and a half-open peer is
      # caught by the 10 Hz telemetry push failing.
      conn.settimeout(None)
      rospy.loginfo(PKG_NAME + ": Connected to device bridge at " + DEVICE_HOST +
                    ":" + str(BRIDGE_PORT))
      with self.client_lock:
        self.client_conn = conn
      # Tell the device which environment models exist on this VM right
      # away, same "push state on connect" instinct as rbx_sim_node.py's own
      # camera-settings resync -- see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md
      # section 5.6 (this is the only window rbx_sim_node.py's capabilities
      # get to learn about scanned models, since RBX capabilities don't
      # live-refresh after construction).
      self.sendLineToClient({'type': 'environment_options',
                            'options': environment_models.list_environment_models()})
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      rospy.logwarn(PKG_NAME + ": Bridge connection lost -- retrying in " +
                    str(BRIDGE_RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(BRIDGE_RECONNECT_INTERVAL_SEC)
      try:
        conn.close()
      except Exception:
        pass
      rospy.loginfo(PKG_NAME + ": Bridge client disconnected")

  def serveClient(self, conn):
    # Blocking recv loop on the one active client: newline-delimited JSON
    # commands in. Returns (back to accept) on client close or any error,
    # so the remote node can restart independently and reconnect.
    buf = b''
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Bridge client recv error: " + repr(e))
        return
      if not data:
        rospy.loginfo(PKG_NAME + ": Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          cmd = json.loads(line)
        except Exception as e:
          rospy.logwarn(PKG_NAME + ": Bad bridge command line: " + str(e))
          continue
        # Dispatch by key presence, not a mandatory "type" tag: the existing
        # velocity command shape ({"linear_x","angular_z"}) predates this and
        # is left untouched. Only the new camera_settings shape carries a
        # "type" field.
        if cmd.get('type') == 'camera_settings':
          self.applyCameraSettings(cmd)
          continue
        if cmd.get('type') == 'reset':
          self.resetRover()
          continue
        if cmd.get('type') == 'environment':
          self.env_spawner.set_active_model(cmd.get('model_name'))
          continue
        twist = Twist()
        twist.linear.x = float(cmd.get('linear_x', 0.0))
        twist.angular.z = float(cmd.get('angular_z', 0.0))
        self.nepi_cmd_pub.publish(twist)

  def telemetryPushLoop(self):
    interval = 1.0 / TELEMETRY_RATE_HZ
    while not rospy.is_shutdown():
      time.sleep(interval)
      if self.latest_telemetry is None:
        continue
      # Same client_lock-around-sendall rationale as sendLineToClient above --
      # this loop and imageCompressedCb's sendLineToClient both write the one
      # client socket from different threads and must not interleave.
      with self.client_lock:
        conn = self.client_conn
        if conn is None:
          continue
        try:
          conn.sendall((json.dumps(self.latest_telemetry) + '\n').encode())
        except Exception as e:
          # Dead client: closing here unblocks serveClient's recv, which
          # returns the server loop to accept for the reconnect
          rospy.logwarn(PKG_NAME + ": Telemetry push failed, dropping client: " + repr(e))
          if self.client_conn is conn:
            self.client_conn = None
          try:
            conn.close()
          except Exception:
            pass


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = SimBridgeNode()
  node.run()
