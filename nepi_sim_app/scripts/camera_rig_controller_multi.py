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

# Multi-robot camera-rig follow controller (Universal Simulator Bridge,
# camera-rover-multi phase). Runs one CameraRigController instance per robot
# slot in a single process, mirroring sim_bridge_multi_node.py's RobotBridge
# pattern: each instance follows its own robot's own /roverN/odom, reads its
# own view-mode/offset ROS params from a distinct per-slot namespace
# (/sim/camera/roverN/*, not the single-robot file's shared /sim/camera/*),
# repositions its own camera_rig<N> Gazebo instance via /gazebo/set_model_state,
# and publishes its own compressed frames on a distinct topic
# (/camera_rig<N>/image_compressed) so sim_bridge_multi_node.py's per-slot
# RobotBridge can subscribe to exactly one robot's frames with no cross-talk.
#
# A deliberately separate file from the single-robot camera_rig_controller.py
# -- new file rather than extending it, matching the same rationale
# sim_bridge_multi_node.py used for sim_bridge_node.py: zero regression risk
# to the verified single-robot path, and the two workflows are launched
# independently (sim_rover_gazebo vs. sim_rover_gazebo_multi, never both).
#
# One mechanism per instance, same as the single-robot controller: target
# camera position is always rover_position + offset rotated into the rover's
# current heading; only the orientation logic differs between FIRST_PERSON
# (facing forward) and THIRD_PERSON (a real look-at). See
# camera_rig_controller.py's module docstring for the full design rationale
# (set_model_state topic vs. service, compression-in-this-node reasoning,
# param-based cross-process settings handoff) -- unchanged here, just
# parameterized per slot.

import math
import threading

import cv2
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from gazebo_msgs.msg import ModelState
from cv_bridge import CvBridge

PKG_NAME = 'CAMERA_RIG_CONTROLLER_MULTI'
NODE_NAME = 'camera_rig_controller_multi'

# Pose-follow update rate: same as the single-robot controller.
CONTROL_RATE_HZ = 20.0
IMAGE_RATE_HZ = 7.0
JPEG_QUALITY = 60

# Matches rbx_sim_node.py's FACTORY_SETTINGS for these (kept in sync by eye,
# same convention as the single-robot controller).
DEFAULT_VIEW_MODE = 'FIRST_PERSON'
DEFAULT_OFFSET_X = 0.2
DEFAULT_OFFSET_Y = 0.0
DEFAULT_OFFSET_Z = 0.5

# Robot slots: odom topic, camera_rig Gazebo model + image topic, and ROS
# param namespace all keyed by robot name -- matches
# sim_bridge_multi_node.py's ROBOT_SLOTS table (same rover1/rover2 naming,
# same generic_rover_multi.world source of truth).
ROBOT_SLOTS = [
    {
        'name': 'rover1',
        'odom_topic': '/rover1/odom',
        'model_name': 'camera_rig1',
        'image_topic': '/camera_rig1/camera/image_raw',
        'compressed_topic': '/camera_rig1/image_compressed',
        'param_ns': '/sim/camera/rover1',
    },
    {
        'name': 'rover2',
        'odom_topic': '/rover2/odom',
        'model_name': 'camera_rig2',
        'image_topic': '/camera_rig2/camera/image_raw',
        'compressed_topic': '/camera_rig2/image_compressed',
        'param_ns': '/sim/camera/rover2',
    },
]

MODEL_STATE_TOPIC = '/gazebo/set_model_state'


#########################################
# Per-Robot Camera Rig Controller
#########################################

class CameraRigController:
  """One robot slot: follows its own rover's odom, repositions its own
  camera_rig instance, and relays its own compressed frames."""

  def __init__(self, name, odom_topic, model_name, image_topic,
               compressed_topic, param_ns):
    self.name = name
    self.model_name = model_name
    self.log_prefix = PKG_NAME + " [" + name + "]: "

    self.param_view_mode = param_ns + '/view_mode'
    self.param_offset_x = param_ns + '/offset_x'
    self.param_offset_y = param_ns + '/offset_y'
    self.param_offset_z = param_ns + '/offset_z'

    self.bridge = CvBridge()
    self.state_lock = threading.Lock()
    self.rover_x = 0.0
    self.rover_y = 0.0
    self.rover_yaw = 0.0
    self.have_odom = False

    self.image_lock = threading.Lock()
    self.latest_cv_img = None

    self.state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)
    self.compressed_pub = rospy.Publisher(compressed_topic, CompressedImage,
                                          queue_size=1)

    self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odomCb)
    self.image_sub = rospy.Subscriber(image_topic, Image, self.imageCb)

    self.control_timer = rospy.Timer(rospy.Duration(1.0 / CONTROL_RATE_HZ), self.controlCb)
    self.image_timer = rospy.Timer(rospy.Duration(1.0 / IMAGE_RATE_HZ), self.imagePublishCb)

    rospy.loginfo(self.log_prefix + "Following " + odom_topic + " -> " +
                  model_name + " via " + MODEL_STATE_TOPIC)

  def odomCb(self, msg):
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with self.state_lock:
      self.rover_x = pos.x
      self.rover_y = pos.y
      self.rover_yaw = yaw
      self.have_odom = True

  def imageCb(self, msg):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
      rospy.logwarn_throttle(5.0, self.log_prefix + "Image conversion failed: " + str(e))
      return
    with self.image_lock:
      self.latest_cv_img = cv_img

  def getViewSettings(self):
    view_mode = rospy.get_param(self.param_view_mode, DEFAULT_VIEW_MODE)
    offset_x = float(rospy.get_param(self.param_offset_x, DEFAULT_OFFSET_X))
    offset_y = float(rospy.get_param(self.param_offset_y, DEFAULT_OFFSET_Y))
    offset_z = float(rospy.get_param(self.param_offset_z, DEFAULT_OFFSET_Z))
    return view_mode, offset_x, offset_y, offset_z

  def controlCb(self, timer_event):
    with self.state_lock:
      if not self.have_odom:
        return
      rover_x = self.rover_x
      rover_y = self.rover_y
      rover_yaw = self.rover_yaw

    view_mode, off_x, off_y, off_z = self.getViewSettings()

    cos_y = math.cos(rover_yaw)
    sin_y = math.sin(rover_yaw)
    world_dx = off_x * cos_y - off_y * sin_y
    world_dy = off_x * sin_y + off_y * cos_y

    cam_x = rover_x + world_dx
    cam_y = rover_y + world_dy
    cam_z = off_z

    if view_mode == 'THIRD_PERSON':
      dx = rover_x - cam_x
      dy = rover_y - cam_y
      dz = 0.0 - cam_z
      horiz_dist = math.hypot(dx, dy)
      cam_yaw = math.atan2(dy, dx)
      cam_pitch = math.atan2(dz, horiz_dist) if horiz_dist > 1e-6 else 0.0
    else:
      cam_yaw = rover_yaw
      cam_pitch = 0.0

    qx, qy, qz, qw = self.eulerToQuat(0.0, cam_pitch, cam_yaw)

    state = ModelState()
    state.model_name = self.model_name
    state.pose.position.x = cam_x
    state.pose.position.y = cam_y
    state.pose.position.z = cam_z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.reference_frame = 'world'
    self.state_pub.publish(state)

  def eulerToQuat(self, roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

  def imagePublishCb(self, timer_event):
    with self.image_lock:
      cv_img = self.latest_cv_img
    if cv_img is None:
      return
    ok, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
      rospy.logwarn_throttle(5.0, self.log_prefix + "JPEG encode failed")
      return
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'jpeg'
    msg.data = encoded.tobytes()
    self.compressed_pub.publish(msg)


#########################################
# Node Class
#########################################

class CameraRigControllerMultiNode:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.controllers = [CameraRigController(slot['name'], slot['odom_topic'],
                                            slot['model_name'], slot['image_topic'],
                                            slot['compressed_topic'], slot['param_ns'])
                        for slot in ROBOT_SLOTS]

    rospy.loginfo(PKG_NAME + ": Multi-robot camera rig controller initialized "
                  "(" + str(len(self.controllers)) + " robot slots)")

  def run(self):
    """Block until ROS shutdown, servicing each slot's control/image timers."""
    rospy.spin()


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = CameraRigControllerMultiNode()
  node.run()
