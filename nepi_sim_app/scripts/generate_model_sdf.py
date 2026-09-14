#!/usr/bin/env python3
"""Renders a model's model.sdf from its dimensions.yaml -- the curated-fields
half of the robot/environment dimension-editing feature (raw-SDF-upload is the
other half, and bypasses this script entirely by writing model.sdf directly).

Each dimensions.yaml holds only INDEPENDENT parameters (wheel radius, track
width, corridor width, etc.) -- everything derivable from those (wheel poses,
base_link height, the diff-drive plugin's wheelDiameter/wheelSeparation, ramp
geometry) is computed here, once, so the two can never drift out of sync the
way the original hand-authored generic_rover/model.sdf's 8 separate wheel-
radius literals and obstacle_course/model.sdf's 3 separate "6.0" corridor-width
literals could.

Standalone script -- no rospy import, safe to run by hand or invoked remotely
(see sim_connector_app_node.py's push-then-generate flow, api/simulator_launcher.py)
over the same ssh channel already used for launch/stop commands.

Usage:
    python3 generate_model_sdf.py <model_name> [--models-dir DIR]

<model_name> is one of: generic_rover, obstacle_course,
aerial_obstacle_course, custom_obstacles. Reads
<models_dir>/<model_name>/dimensions.yaml and writes
<models_dir>/<model_name>/model.sdf.
"""

import argparse
import math
import os
import sys

import yaml

DEFAULT_MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")

ROVER_DEFAULT_DIMENSIONS = {
    "wheel_radius_m": 0.1,
    "wheel_width_m": 0.05,
    "track_width_m": 0.34,
    "wheelbase_m": 0.3,
    "chassis_length_m": 0.4,
    "chassis_width_m": 0.3,
    "chassis_height_m": 0.1,
    # base_link's own mass -- added 2026-09-03, requested live: "add a weight
    # parameter to the robot in the sim connector, which can affect some of
    # its physics on how hard it sticks to the ground and is able to be
    # affected by things." 5.0 kg matches base_link's own previously-
    # hardcoded mass exactly, so an unconfigured robot config's physics are
    # unchanged. buildRoverSdf recomputes base_link's inertia FROM this value
    # (standard rectangular-prism formula, using chassis_length_m/width_m/
    # height_m) rather than leaving inertia hardcoded -- a mass change with
    # stale inertia would itself reintroduce the instability this same pass
    # also fixed (see wheelAcceleration's own comment below). Wheel mass/
    # inertia are unaffected -- out of scope, this is the robot's OVERALL
    # weight/ground-stick, not a per-wheel property.
    "weight_kg": 5.0,
    # Shared by both camera sensors below (onboard + chase) -- matches
    # sim_connector_app_node.py's own CAMERA_HORIZONTAL_FOV_DEG default, the
    # value it reports to the RUI/targeting math for pixel-to-degree
    # conversion. Editing this field (same curated-fields path as every
    # other rover dimension) changes what Gazebo actually renders AND what
    # gets reported, so the two can never drift out of sync with each
    # other.
    "camera_horizontal_fov_deg": 80.0,
    # "Wheel independence" (crab-steering): each wheel gets its own vertical
    # steering joint (a "steer hub" between base_link and the wheel) driven
    # to point the commanded travel direction, so the rover can translate
    # sideways/diagonally without its body yawing -- requested live
    # (2026-09-04): "the robot (like a rover) can move to the side without
    # its base moving, where only the wheels need to move a certain
    # direction. the base can still face the same way... if its disabled,
    # it will just work normally." 0 (default) = today's exact skid-steer
    # behavior, byte-for-byte unchanged SDF output -- see buildRoverSdf's
    # own branch on this field and docs/ROVER_WHEEL_INDEPENDENCE_PLAN.md.
    "wheel_independence_enabled": 0.0,
}

OBSTACLE_COURSE_DEFAULT_DIMENSIONS = {
    "course_start_x_m": 2.0,
    "corridor_width_m": 6.0,
    "wall_length_m": 22.0,
    "wall_thickness_m": 0.2,
    "wall_height_m": 1.0,
    "baffle_a_x_m": 8.0,
    "baffle_b_x_m": 14.0,
    "baffle_gap_m": 0.4,
    "baffle_thickness_m": 0.2,
    "ramp_start_x_m": 18.0,
    "ramp_rise_m": 0.35,
    "ramp_angle_deg": 9.97,
    "ramp_plateau_length_m": 1.0,
}

# A sequence of square gate frames a drone flies up-and-through in order --
# independent params only, same convention as the two courses above: each
# gate's exact frame geometry (four box segments forming a hollow square) is
# derived in buildAerialObstacleCourseSdf, not hand-placed per gate, so
# gate_count alone controls how long the course is.
AERIAL_OBSTACLE_COURSE_DEFAULT_DIMENSIONS = {
    "course_start_x_m": 3.0,
    "gate_count": 4,
    "gate_spacing_m": 6.0,
    "gate_opening_width_m": 2.0,
    "gate_opening_height_m": 2.0,
    "gate_frame_thickness_m": 0.15,
    "gate_base_height_m": 2.0,
    "gate_height_step_m": 1.0,
}


def loadDimensions(model_name, models_dir, defaults):
    path = os.path.join(models_dir, model_name, "dimensions.yaml")
    dims = dict(defaults)
    if os.path.exists(path):
        with open(path, "r") as f:
            loaded = yaml.safe_load(f) or {}
        dims.update(loaded)
    # A value the operator typed through the RUI round-trips as plain YAML
    # text, which is not type-preserving -- a field can land here as the
    # STRING '0.1' instead of the float 0.1 (confirmed live: a string-typed
    # wheel_radius_m crashed buildRoverSdf's own
    # wheel_radius + chassis_h/2.0 with "can only concatenate str to str").
    # Coerce every field back to a real number here, once, rather than
    # requiring every builder function to defensively re-coerce everything
    # it reads -- only for keys this model's OWN defaults declare numeric,
    # so a non-numeric field (custom_obstacles' own 'obstacles' list, or the
    # reserved '_environment_model' string) passes through untouched.
    for key, default_value in defaults.items():
        if isinstance(default_value, (int, float)) and key in dims:
            try:
                dims[key] = float(dims[key])
            except (TypeError, ValueError):
                pass
    return dims


# ---------------------------------------------------------------------------
# generic_rover
# ---------------------------------------------------------------------------

ROVER_WHEELS = [
    ("front_left_wheel", 1, 1),
    ("front_right_wheel", 1, -1),
    ("rear_left_wheel", -1, 1),
    ("rear_right_wheel", -1, -1),
]


def buildRoverSdf(dims):
    wheel_radius = dims["wheel_radius_m"]
    wheel_width = dims["wheel_width_m"]
    track_width = dims["track_width_m"]
    wheelbase = dims["wheelbase_m"]
    chassis_l = dims["chassis_length_m"]
    chassis_w = dims["chassis_width_m"]
    chassis_h = dims["chassis_height_m"]
    camera_horizontal_fov_rad = math.radians(dims["camera_horizontal_fov_deg"])
    weight_kg = dims["weight_kg"]
    # Standard solid-rectangular-prism inertia formula, using the chassis'
    # own length/width/height -- recomputed from weight_kg (not left at the
    # old hardcoded 0.0417/0.0708/0.1042 literals) so mass and inertia can
    # never drift out of sync with each other, the same reasoning
    # camera_horizontal_fov_deg's own comment above already applies to
    # keeping two related values from silently disagreeing. This also fixes
    # a pre-existing bug: those literals were only ever correct for the
    # factory 5.0 kg / 0.4x0.3x0.1 m chassis -- editing chassis dimensions
    # alone (mass unchanged) already went stale before this change.
    base_ixx = weight_kg * (chassis_w ** 2 + chassis_h ** 2) / 12.0
    base_iyy = weight_kg * (chassis_l ** 2 + chassis_h ** 2) / 12.0
    base_izz = weight_kg * (chassis_l ** 2 + chassis_w ** 2) / 12.0

    # base_link sits at wheel-axle height so the wheels touch the ground
    # plane when the model spawns at z = 0 -- see generic_rover/model.sdf's
    # own long-standing comment for why this replaced a 2-wheel+caster design.
    base_z = wheel_radius + chassis_h / 2.0
    x_off = wheelbase / 2.0
    y_off = track_width / 2.0

    # Camera x/z offset from base_link (0.2, 0.5) is a fixed spec value, NOT
    # derived from wheelbase_m/track_width_m -- it just happens to be close
    # to wheelbase/2 at the factory defaults, which is a coincidence, not a
    # relationship. Only the z placement needs re-deriving when chassis/wheel
    # dimensions move base_link's own absolute height; x stays fixed.
    camera_x = 0.2
    camera_rel_z = 0.5
    camera_z = base_z + camera_rel_z
    mast_len = camera_rel_z - chassis_h / 2.0
    mast_z = chassis_h / 2.0 + mast_len / 2.0

    wheel_independence_enabled = bool(dims["wheel_independence_enabled"])

    wheel_links = []
    hub_links = []
    left_joint_tags = []
    right_joint_tags = []
    for name, x_sign, y_sign in ROVER_WHEELS:
        x = x_sign * x_off
        y = y_sign * y_off
        if wheel_independence_enabled:
            # Real locomotion comes entirely from libgazebo_ros_planar_move
            # in this mode (kinematic body-frame velocity, no wheel-ground
            # traction involved at all) -- ground friction on the wheels
            # serves no locomotion purpose here, only a chance of dragging
            # against the body's own planar_move motion (the wheel is still
            # in ground contact while being kinematically translated).
            # Lowered as a precaution kept from an earlier debugging pass
            # (mu=1.5/mu2=0.2, tuned for the disabled case's skid-steer
            # turning) before the real steering bug -- a missing
            # robotNamespace on nepi_crab_steer_plugin's own cmd_vel
            # subscriber, see docs/ROVER_WHEEL_INDEPENDENCE_PLAN.md's "what
            # went wrong" section -- was found; not re-verified against
            # default friction since the passing live test below already
            # ran with this value in place.
            wheel_links.append(_roverWheelLink(name, x, y, wheel_radius, wheel_width, mu=0.05, mu2=0.05))
            hub_links.append(_roverWheelHubLink(name, x, y, wheel_radius))
        else:
            wheel_links.append(_roverWheelLink(name, x, y, wheel_radius, wheel_width))
        if y_sign > 0:
            left_joint_tags.append(f"      <leftJoint>{name}_joint</leftJoint>")
        else:
            right_joint_tags.append(f"      <rightJoint>{name}_joint</rightJoint>")

    left_joint_tags = "\n".join(left_joint_tags)
    right_joint_tags = "\n".join(right_joint_tags)

    wheel_separation = track_width
    wheel_diameter = wheel_radius * 2.0

    # Disabled (default): EXACTLY today's plugin, unchanged. Enabled:
    # libgazebo_ros_planar_move (stock Gazebo plugin, real body-frame x/y/yaw
    # kinematics from the same cmd_vel Twist -- unlike diff_drive it natively
    # understands linear.y) drives the actual motion, and
    # nepi_crab_steer_plugin (nepi_gazebo_plugins package, this repo's own)
    # purely animates the 4 wheel corners to visually steer+spin in a way
    # consistent with that motion. Two independent cmd_vel subscribers on
    # one topic is fine.
    if wheel_independence_enabled:
        wheel_plugin_entries = "\n".join(
            f'      <wheel steerJoint="{name}_steer_joint" spinJoint="{name}_joint"/>'
            for name, _x, _y in ROVER_WHEELS
        )
        drive_plugin_block = f"""    <plugin name="planar_move_controller" filename="libgazebo_ros_planar_move.so">
      <robotNamespace>/rover</robotNamespace>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>30.0</odometryRate>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>

    <plugin name="crab_steer_controller" filename="libnepi_crab_steer_plugin.so">
      <robotNamespace>/rover</robotNamespace>
      <commandTopic>cmd_vel</commandTopic>
      <wheelRadius>{wheel_radius}</wheelRadius>
{wheel_plugin_entries}
    </plugin>"""
    else:
        drive_plugin_block = f"""    <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <robotNamespace>/rover</robotNamespace>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <robotBaseFrame>base_link</robotBaseFrame>
      <!-- Repeated leftJoint/rightJoint tags: gazebo_ros_diff_drive drives
           every listed left-side joint identically and every right-side
           joint identically, exactly the skid-steer pattern the front/rear
           axle pairs need. -->
{left_joint_tags}
{right_joint_tags}
      <wheelSeparation>{wheel_separation:.6f}</wheelSeparation>
      <wheelDiameter>{wheel_diameter:.6f}</wheelDiameter>
      <wheelTorque>25.0</wheelTorque>
      <!-- 0.0 here means UNLIMITED (libgazebo_ros_diff_drive's own
           documented meaning: no cap on how fast the plugin's internal PID
           target can change per step), reported live: "if you change
           motor controls too rapidly, ex: putting to 100% and then -100%
           after 5 seconds, it starts going crazy and randomly glitching
           out." A 100% to -100% command was a literal step-function
           velocity reversal with the full 25 N*m of wheelTorque applied
           instantly to chase it, exactly the kind of single-timestep delta
           that diverges ODE's constraint solver. Capped instead of left
           unlimited; RESET_SIM "helping" was never fixing the physics,
           just teleporting the model back to clear the diverged state
           (see rbx_sim_node.py's resetSimAction), this fixes the actual
           cause. -->
      <wheelAcceleration>3.0</wheelAcceleration>
      <updateRate>30.0</updateRate>
      <publishOdomTF>true</publishOdomTF>
      <publishWheelTF>false</publishWheelTF>
      <publishWheelJointState>false</publishWheelJointState>
      <legacyMode>false</legacyMode>
    </plugin>"""

    return f"""<?xml version='1.0'?>
<sdf version="1.6">
  <model name="generic_rover">

    <!-- Chassis. Origin sits at wheel-axle height ({base_z:.4f} m =
         wheel_radius_m + chassis_height_m/2) so the wheels
         (r = {wheel_radius} m) touch the ground plane when the model spawns
         at z = 0. Generated by generate_model_sdf.py from dimensions.yaml,
         edit that file (or the curated-fields UI in Sim Connector), not this
         one directly, unless using the raw-SDF-upload escape hatch. -->
    <link name="base_link">
      <pose>0 0 {base_z:.6f} 0 0 0</pose>
      <inertial>
        <mass>{weight_kg}</mass>
        <inertia>
          <ixx>{base_ixx:.6f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{base_iyy:.6f}</iyy>
          <iyz>0</iyz>
          <izz>{base_izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="base_collision">
        <geometry>
          <box>
            <size>{chassis_l} {chassis_w} {chassis_h}</size>
          </box>
        </geometry>
      </collision>
      <visual name="base_visual">
        <geometry>
          <box>
            <size>{chassis_l} {chassis_w} {chassis_h}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.3 0.6 1</ambient>
          <diffuse>0.1 0.3 0.6 1</diffuse>
        </material>
      </visual>
      <!-- Camera mast (visual only) up to the camera_link offset. -->
      <visual name="camera_mast_visual">
        <pose>{camera_x:.6f} 0 {mast_z:.6f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.01</radius>
            <length>{mast_len:.6f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Four wheels, identical geometry, mirrored front/rear (x) and left/
         right (y) from track_width_m/wheelbase_m. Anisotropic friction
         (mu2 << mu) is required, not cosmetic: a 4-wheel skid-steer can only
         yaw by slipping all four contact patches sideways, and isotropic
         friction made that physically impossible (confirmed live: commanding
         angular.z = 1.0 rad/s for 10s rotated the rover 2.3 degrees instead
         of the expected 573 -- forward drive looked fine, which is why this
         stayed hidden; it breaks turning and every goto/autonomous command
         that needs to change heading). fdir1 is the wheel's rolling
         direction (local X survives the link's -90deg X rotation), so mu is
         longitudinal traction and mu2 is lateral; 0.2 lateral keeps enough
         grip that the rover doesn't slide under its own weight while still
         allowing the skid a turn requires. -->
{"".join(wheel_links)}{"".join(hub_links)}
{_roverJoints(wheel_independence_enabled)}

    <!-- Camera at (0.2, 0.0, 0.5) relative to base_link -- x offset is a
         fixed spec value (not a curated field), z offset (0.5m above
         base_link) likewise fixed; base_link's own absolute height is what
         moves when chassis/wheel dimensions change, so camera_z is re-derived
         from base_z + 0.5, not hardcoded. -->
    <link name="camera_link">
      <pose>{camera_x:.6f} 0 {camera_z:.6f} 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00001</iyy>
          <iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry>
          <box>
            <size>0.04 0.08 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <!-- type="depth" + libgazebo_ros_openni_kinect.so is Gazebo Classic's
           standard RGBD sensor -- publishes the same image_raw as before
           PLUS a 32FC1 raw depth image from this same link. -->
      <sensor type="depth" name="rover_camera_sensor">
        <update_rate>15.0</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <camera name="rover_camera">
          <horizontal_fov>{camera_horizontal_fov_rad:.7f}</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>100.0</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <updateRate>15.0</updateRate>
          <cameraName>rover/camera</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <depthImageInfoTopicName>depth/camera_info</depthImageInfoTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.05</pointCloudCutoff>
          <pointCloudCutoffMax>100.0</pointCloudCutoffMax>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
      </sensor>
    </link>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>

    <!-- Third-person/chase camera, rigidly welded onto the rover at a FIXED
         body-frame offset (-2.5, 0.0, 1.65) -- matches FACTORY_SCENE_OFFSET_X/Y/Z
         in rbx_sim_node.py/sim_bridge_node.py, independent of the curated
         dimensions above -- not re-derived, since it is its own spec value,
         not something computed from chassis/wheel geometry. pitch =
         atan2(1.65, 2.5) = 0.5834 rad is the fixed tilt that keeps the rover
         framed from directly behind and above. -->
    <link name="camera_link_chase">
      <pose>-2.5 0 1.65 0 0.5833730 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00001</iyy>
          <iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="camera_chase_visual">
        <geometry>
          <box>
            <size>0.06 0.1 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.2 0.8 0.2 1</diffuse>
        </material>
      </visual>
      <sensor type="depth" name="rover_camera_chase_sensor">
        <update_rate>15.0</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <camera name="rover_camera_chase">
          <horizontal_fov>{camera_horizontal_fov_rad:.7f}</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>100.0</far>
          </clip>
        </camera>
        <plugin name="camera_chase_controller" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <updateRate>15.0</updateRate>
          <cameraName>rover/camera_chase</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <depthImageInfoTopicName>depth/camera_info</depthImageInfoTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <frameName>camera_link_chase</frameName>
          <pointCloudCutoff>0.05</pointCloudCutoff>
          <pointCloudCutoffMax>100.0</pointCloudCutoffMax>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
      </sensor>
    </link>

    <joint name="camera_joint_chase" type="fixed">
      <parent>base_link</parent>
      <child>camera_link_chase</child>
    </joint>

{drive_plugin_block}

  </model>
</sdf>
"""


def _roverJoints(wheel_independence_enabled=False):
    if not wheel_independence_enabled:
        return "".join(
            f"""    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>

"""
            for name, _x, _y in ROVER_WHEELS
        )
    # Two-joint chain per wheel instead of one: a new steering revolute (Z
    # axis, base_link -> {name}_hub) plus the SAME spin joint the disabled
    # case above uses (Y axis, same name -- so left_joint_tags/
    # right_joint_tags built in buildRoverSdf still reference a real joint
    # even though this branch never actually uses them), just re-parented
    # from base_link to the new hub link. nepi_crab_steer_plugin drives both
    # by name -- see buildRoverSdf's own plugin block.
    return "".join(
        f"""    <joint name="{name}_steer_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}_hub</child>
      <axis>
        <xyz>0 0 1</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>

    <joint name="{name}_joint" type="revolute">
      <parent>{name}_hub</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>

"""
        for name, _x, _y in ROVER_WHEELS
    )


def _roverWheelLink(name, x, y, radius, width, mu=1.5, mu2=0.2):
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} {radius:.6f} -1.5707963 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.00135</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00135</iyy>
          <iyz>0</iyz>
          <izz>0.0025</izz>
        </inertia>
      </inertial>
      <collision name="{name}_collision">
        <geometry>
          <cylinder>
            <radius>{radius:.6f}</radius>
            <length>{width:.6f}</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu2}</mu2>
              <fdir1>1 0 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="{name}_visual">
        <geometry>
          <cylinder>
            <radius>{radius:.6f}</radius>
            <length>{width:.6f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>

"""


# Wheel-independence-only steering hub -- sits between base_link and a wheel
# link, at the same (x, y, wheel_radius) point the wheel's own joint used to
# attach directly to base_link at. Negligible mass/inertia, same convention
# camera_link already uses for its own non-structural helper links (this
# repo has no true massless/zero-inertia link -- ODE needs a real, if tiny,
# inertia for numerical stability). No visual/collision of its own: it's a
# kinematic bookkeeping link, not something meant to be seen.
def _roverWheelHubLink(name, x, y, z):
    return f"""    <link name="{name}_hub">
      <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00001</iyy>
          <iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
    </link>

"""


# ---------------------------------------------------------------------------
# obstacle_course
# ---------------------------------------------------------------------------

def _wallLink(name, x, y, length, thickness, height, color):
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{color}</name>
          </script>
        </material>
      </visual>
    </link>

"""


def buildObstacleCourseSdf(dims):
    course_start_x = dims["course_start_x_m"]
    corridor_width = dims["corridor_width_m"]
    wall_length = dims["wall_length_m"]
    wall_thickness = dims["wall_thickness_m"]
    wall_height = dims["wall_height_m"]
    baffle_a_x = dims["baffle_a_x_m"]
    baffle_b_x = dims["baffle_b_x_m"]
    baffle_gap = dims["baffle_gap_m"]
    baffle_thickness = dims["baffle_thickness_m"]
    ramp_start_x = dims["ramp_start_x_m"]
    ramp_rise = dims["ramp_rise_m"]
    ramp_angle_deg = dims["ramp_angle_deg"]
    plateau_length = dims["ramp_plateau_length_m"]

    half_corridor = corridor_width / 2.0
    # Walls run along the corridor starting at course_start_x_m (clear of the
    # rover's origin spawn/turn radius), centered at start + wall_length/2.
    wall_center_x = course_start_x + wall_length / 2.0

    # Baffle: hangs from one wall down to a clear drive-through gap on the
    # other side -- length = half_corridor - gap, centered between the wall
    # and the gap edge.
    baffle_len = half_corridor - baffle_gap
    baffle_a_y = half_corridor - baffle_len / 2.0
    baffle_b_y = -(half_corridor - baffle_len / 2.0)

    # Ramp: rise_m and angle_deg are the two independent knobs (how high, how
    # steep) -- run and box (hypotenuse) length are derived, not hand-tuned.
    angle_rad = math.radians(ramp_angle_deg)
    run = ramp_rise / math.tan(angle_rad)
    box_len = ramp_rise / math.sin(angle_rad)
    ramp_z = ramp_rise / 2.0
    plateau_z = ramp_rise

    ramp_up_x = ramp_start_x + run / 2.0
    plateau_x = ramp_up_x + run / 2.0 + plateau_length / 2.0
    ramp_down_x = plateau_x + plateau_length / 2.0 + run / 2.0

    walls = _wallLink("left_wall", wall_center_x, half_corridor, wall_length, wall_thickness, wall_height, "Gazebo/Orange")
    walls += _wallLink("right_wall", wall_center_x, -half_corridor, wall_length, wall_thickness, wall_height, "Gazebo/Orange")

    baffles = _wallLink("baffle_a", baffle_a_x, baffle_a_y, baffle_thickness, baffle_len, wall_height, "Gazebo/Orange")
    baffles += _wallLink("baffle_b", baffle_b_x, baffle_b_y, baffle_thickness, baffle_len, wall_height, "Gazebo/Orange")
    # baffle box axes are swapped (thin in x, long in y) vs the wall helper's
    # (long in x, thin in y) -- _wallLink's (length, thickness) params map
    # directly since baffles pass (thickness, length) in that order above.

    ramp = f"""    <link name="ramp_up">
      <pose>{ramp_up_x:.6f} 0 {ramp_z:.6f} 0 {-angle_rad:.6f} 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>{box_len:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{box_len:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script>
        </material>
      </visual>
    </link>

    <link name="ramp_plateau">
      <pose>{plateau_x:.6f} 0 {plateau_z:.6f} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>{plateau_length:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{plateau_length:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script>
        </material>
      </visual>
    </link>

    <link name="ramp_down">
      <pose>{ramp_down_x:.6f} 0 {ramp_z:.6f} 0 {angle_rad:.6f} 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>{box_len:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{box_len:.6f} {corridor_width:.6f} 0.12</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script>
        </material>
      </visual>
    </link>

"""

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="obstacle_course">
    <!-- Static: pure world geometry, no dynamics needed. Generated by
         generate_model_sdf.py from dimensions.yaml -- edit that file (or the
         curated-fields UI in Sim Connector), not this one directly, unless
         using the raw-SDF-upload escape hatch. -->
    <static>true</static>

    <!-- Course runs along +x inside a {corridor_width:.2f}m-wide corridor
         (y = +/-{half_corridor:.2f}): two boundary walls, a two-baffle
         chicane forcing a weave (gap = {baffle_gap}m), then a ramp-up/
         plateau/ramp-down bump (rise = {ramp_rise}m over {run:.3f}m run,
         {ramp_angle_deg}deg) to climb over. -->

{walls}{baffles}{ramp}  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# aerial_obstacle_course
# ---------------------------------------------------------------------------

def _boxLink(name, x, y, z, size_x, size_y, size_z, color):
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{color}</name>
          </script>
        </material>
      </visual>
    </link>

"""


def _gateFrameLinks(index, center_x, center_z, opening_width, opening_height, thickness, color):
    # Hollow square frame in the y-z plane (thin along x, the flight-through
    # direction) -- top/bottom bars span the full outer width so the four
    # bars' corners overlap cleanly, left/right bars fill exactly the
    # opening_height gap between them.
    outer_width = opening_width + 2.0 * thickness
    prefix = f"gate_{index}"
    links = _boxLink(f"{prefix}_top", center_x, 0.0, center_z + opening_height / 2.0 + thickness / 2.0,
                      thickness, outer_width, thickness, color)
    links += _boxLink(f"{prefix}_bottom", center_x, 0.0, center_z - opening_height / 2.0 - thickness / 2.0,
                       thickness, outer_width, thickness, color)
    links += _boxLink(f"{prefix}_left", center_x, opening_width / 2.0 + thickness / 2.0, center_z,
                       thickness, thickness, opening_height, color)
    links += _boxLink(f"{prefix}_right", center_x, -(opening_width / 2.0 + thickness / 2.0), center_z,
                       thickness, thickness, opening_height, color)
    return links


def buildAerialObstacleCourseSdf(dims):
    course_start_x = dims["course_start_x_m"]
    gate_count = int(dims["gate_count"])
    gate_spacing = dims["gate_spacing_m"]
    opening_width = dims["gate_opening_width_m"]
    opening_height = dims["gate_opening_height_m"]
    thickness = dims["gate_frame_thickness_m"]
    base_height = dims["gate_base_height_m"]
    height_step = dims["gate_height_step_m"]

    gates = ""
    for i in range(gate_count):
        center_x = course_start_x + i * gate_spacing
        center_z = base_height + i * height_step
        gates += _gateFrameLinks(i, center_x, center_z, opening_width, opening_height, thickness, "Gazebo/Red")

    final_height = base_height + max(gate_count - 1, 0) * height_step

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="aerial_obstacle_course">
    <!-- Static: pure world geometry, no dynamics needed. Generated by
         generate_model_sdf.py from dimensions.yaml -- edit that file (or the
         curated-fields UI in Sim Connector), not this one directly, unless
         using the raw-SDF-upload escape hatch. -->
    <static>true</static>

    <!-- {gate_count} square gate frames along +x starting at
         course_start_x_m={course_start_x}m, spaced {gate_spacing}m apart,
         each opening {opening_width}m x {opening_height}m, climbing from
         {base_height}m up to {final_height}m in {height_step}m steps -- a
         drone flies up-and-through each gate in order. -->

{gates}  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# custom_obstacles -- unlike every model above, this one has no fixed set of
# curated fields. dimensions.yaml holds a single 'obstacles' list, and each
# entry is one independently add/remove/edit-able obstacle (the RUI's
# "Custom Obstacles" environment config builds this list interactively).
# Each obstacle needs only a 'type' key naming which OBSTACLE_BUILDERS
# function renders it; every other field is that type's own, with a sane
# fallback via dict.get() if the RUI ever posts a partial entry.
# ---------------------------------------------------------------------------

CUSTOM_OBSTACLES_DEFAULT_DIMENSIONS = {
    "obstacles": [],
}


def _obstacleWallLink(name, obstacle):
    x = float(obstacle.get("x", 0.0))
    y = float(obstacle.get("y", 0.0))
    yaw_deg = float(obstacle.get("yaw_deg", 0.0))
    length = max(float(obstacle.get("length_m", 2.0)), 0.01)
    thickness = max(float(obstacle.get("thickness_m", 0.2)), 0.01)
    height = max(float(obstacle.get("height_m", 1.0)), 0.01)
    yaw_rad = math.radians(yaw_deg)
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 {yaw_rad:.6f}</pose>
      <collision name="collision">
        <geometry><box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{length:.6f} {thickness:.6f} {height:.6f}</size></box></geometry>
        <material>
          <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Orange</name></script>
        </material>
      </visual>
    </link>

"""


def _obstacleCircleLink(name, obstacle):
    x = float(obstacle.get("x", 0.0))
    y = float(obstacle.get("y", 0.0))
    radius = max(float(obstacle.get("radius_m", 0.5)), 0.01)
    height = max(float(obstacle.get("height_m", 1.0)), 0.01)
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{radius:.6f}</radius><length>{height:.6f}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{radius:.6f}</radius><length>{height:.6f}</length></cylinder></geometry>
        <material>
          <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script>
        </material>
      </visual>
    </link>

"""


def _obstacleTriangleLink(name, obstacle):
    # A wedge: local vertices (0, depth/2), (0, -depth/2), (base, 0) --
    # pointing in local +x -- extruded from z=0 to height via SDF's
    # <polyline> geometry (Gazebo Classic 9+ / SDF 1.6; no native <triangle>
    # primitive exists, so this is the correct, real way to build one,
    # not an approximation).
    x = float(obstacle.get("x", 0.0))
    y = float(obstacle.get("y", 0.0))
    yaw_deg = float(obstacle.get("yaw_deg", 0.0))
    base = max(float(obstacle.get("base_m", 1.0)), 0.01)
    depth = max(float(obstacle.get("depth_m", 1.0)), 0.01)
    height = max(float(obstacle.get("height_m", 1.0)), 0.01)
    yaw_rad = math.radians(yaw_deg)
    points = (
        f"<point>0 {depth / 2.0:.6f}</point>"
        f"<point>0 {-depth / 2.0:.6f}</point>"
        f"<point>{base:.6f} 0</point>"
    )
    geometry = f"<polyline>{points}<height>{height:.6f}</height></polyline>"
    return f"""    <link name="{name}">
      <pose>{x:.6f} {y:.6f} 0 0 0 {yaw_rad:.6f}</pose>
      <collision name="collision">
        <geometry>{geometry}</geometry>
      </collision>
      <visual name="visual">
        <geometry>{geometry}</geometry>
        <material>
          <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Blue</name></script>
        </material>
      </visual>
    </link>

"""


OBSTACLE_TYPE_BUILDERS = {
    "wall": _obstacleWallLink,
    "circle": _obstacleCircleLink,
    "triangle": _obstacleTriangleLink,
}


def buildCustomObstaclesSdf(dims):
    obstacles = dims.get("obstacles", [])
    if not isinstance(obstacles, list):
        obstacles = []
    links = ""
    for i, obstacle in enumerate(obstacles):
        if not isinstance(obstacle, dict):
            continue
        obstacle_type = str(obstacle.get("type", ""))
        builder = OBSTACLE_TYPE_BUILDERS.get(obstacle_type)
        if builder is None:
            continue
        links += builder(f"obstacle_{i}_{obstacle_type}", obstacle)

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="custom_obstacles">
    <!-- Static: pure world geometry, no dynamics needed. Generated by
         generate_model_sdf.py from dimensions.yaml's own 'obstacles' list --
         unlike every other model here, this one has no fixed curated field
         set; edit the list via the "Custom Obstacles" environment config in
         Sim Connector (or hand-edit dimensions.yaml), not this file
         directly, unless using the raw-SDF-upload escape hatch.

         {len(obstacles)} obstacle(s) in this course. -->
{links}  </model>
</sdf>
"""


BUILDERS = {
    "generic_rover": (buildRoverSdf, ROVER_DEFAULT_DIMENSIONS),
    "obstacle_course": (buildObstacleCourseSdf, OBSTACLE_COURSE_DEFAULT_DIMENSIONS),
    "aerial_obstacle_course": (buildAerialObstacleCourseSdf, AERIAL_OBSTACLE_COURSE_DEFAULT_DIMENSIONS),
    "custom_obstacles": (buildCustomObstaclesSdf, CUSTOM_OBSTACLES_DEFAULT_DIMENSIONS),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_name", choices=sorted(BUILDERS.keys()))
    parser.add_argument("--models-dir", default=DEFAULT_MODELS_DIR)
    args = parser.parse_args()

    builder, defaults = BUILDERS[args.model_name]
    dims = loadDimensions(args.model_name, args.models_dir, defaults)
    sdf_text = builder(dims)

    out_path = os.path.join(args.models_dir, args.model_name, "model.sdf")
    with open(out_path, "w") as f:
        f.write(sdf_text)
    print("Wrote " + out_path + " from dimensions: " + str(dims))


if __name__ == "__main__":
    sys.exit(main())
