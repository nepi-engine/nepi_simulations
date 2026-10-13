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

# The MuJoCo simulation process that nepi_mujoco.sh starts and stops. MuJoCo
# is a library, not a separate simulator program, so this one process owns the
# physics loop, the viewer window, and the bridge to the robot's NEPI driver on
# the device.
#
# The world and the robot are separate files, combined here at launch: the
# world is an MJCF file from ENVIRONMENT_CONFIGS (scenery, lights, and the
# <option> physics settings), and the robot is ROBOT_CONFIGS/<name>/model.xml,
# whose first worldbody body is attached into the world with MjSpec. That is
# the MuJoCo counterpart of Gazebo's "spawn the robot into a scenery world",
# and it is why changing either one means restarting this process: a compiled
# MuJoCo model cannot gain or lose bodies.
#
# Viewer: opened when --viewer is "auto" and a display is available. With no
# display, or --viewer off, the simulation runs headless. Closing the window
# does not stop the simulation -- the robot stays up for the device, and only
# the service (Kill on the device) ends it.
#
# Learned with the old sim_container MuJoCo bridge: launch_passive segfaults if
# the viewer is closed before the first step. The loop below steps straight
# away, so that cannot happen here.
#
# Bridge (--device-host): the same wire protocol as the Gazebo service's
# nepi_gazebo_bridge.py, so the device runs the same RBX_SIM driver
# (rbx_sim_node.py) for either simulator. This side always speaks first, so the
# sim machine never has to accept an inbound connection:
#
#   UDP  sim -> device  {"type":"telemetry","seq","x","y","yaw","linear_x","angular_z","stamp"}
#                       10 Hz: world position (m), heading (rad), forward speed
#                       (m/s) and turn rate (rad/s).
#   UDP  device -> sim  {"type":"velocity","seq","linear_x","linear_y","angular_z"}
#                       20 Hz from the device. Turned into wheel speeds through
#                       the robot's mujoco_drive section (nepi_robot_config.yaml).
#                       If it stops for WATCHDOG_SEC the robot is stopped, so a
#                       lost device or link never leaves it driving.
#   TCP  sim -> device  {"type":"image","camera","format":"jpeg","data":"<base64>","stamp"}
#                       robot_color / robot_depth / scene_color / scene_depth,
#                       IMAGE_RATE_HZ each, rendered from the robot's
#                       robot_camera and scene_camera. Depth is colorized
#                       (close = blue, far = red), as Gazebo's bridge sends it.
#   TCP  device -> sim  {"type":"reset"}  back to the start pose.
#
# Camera rendering runs on its own thread with its own offscreen (EGL) OpenGL
# context, from a copy of the robot's pose, so it never holds up the physics
# loop or competes with the viewer window's context. It only renders while the
# device is connected.

import argparse
import base64
import json
import math
import os
import signal
import socket
import threading
import time

# Offscreen camera rendering through EGL, which needs no display and stays
# separate from the viewer window's GLFW context. Must be set before mujoco is
# imported; an explicit MUJOCO_GL in the environment still wins.
os.environ.setdefault('MUJOCO_GL', 'egl')

import cv2
import mujoco
import numpy as np
import yaml

# How often the viewer is redrawn. The physics runs at the world's own
# timestep regardless.
VIEWER_SYNC_HZ = 60.0
# How often the physics loop applies the latest velocity command and takes a
# telemetry snapshot. Wheel targets hold between ticks, so this does not need
# to match the physics timestep.
CONTROL_RATE_HZ = 50.0
TELEMETRY_RATE_HZ = 10.0
# The device sends every 50 ms, so 250 ms is several missed packets.
WATCHDOG_SEC = 0.25
# A velocity seq this far behind the last one accepted is a restarted driver,
# not a late packet (50 packets is 2.5 s of commands).
SEQ_RESTART_GAP = 50
FACTORY_PORT = 9023
CONNECT_RETRY_SEC = 2.0

# Camera images, matching what nepi_gazebo_bridge.py sends: the view names
# rbx_sim_node.py publishes, the MuJoCo camera each is rendered from, and the
# same size, rate and JPEG quality.
CAMERA_VIEWS = (('robot', 'robot_camera'), ('scene', 'scene_camera'))
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_RATE_HZ = 10.0
JPEG_QUALITY = 65
DEPTH_MIN_RANGE_M = 0.3
DEPTH_MAX_RANGE_M = 15.0
ROBOT_CAPABILITIES_FILENAME = 'nepi_robot_config.yaml'
LOG_PREFIX = "NEPI_MUJOCO_SIM: "


def log(msg):
    print(LOG_PREFIX + msg, flush = True)


def load_model(world_file, robot_dir):
    """Compile the world with the robot attached. Returns (model, robot_body_name)."""
    robot_file = os.path.join(robot_dir, 'model.xml')
    world = mujoco.MjSpec()
    world.from_file(world_file)
    robot = mujoco.MjSpec()
    robot.from_file(robot_file)

    robot_root = robot.worldbody.first_body()
    if robot_root is None:
        raise ValueError(robot_file + " has no body in its worldbody to attach")
    robot_body_name = robot_root.name

    # Attached at the world origin; the robot file's own body pos sets its
    # starting height.
    frame = world.worldbody.add_frame()
    frame.attach_body(robot_root, '', '')
    return world.compile(), robot_body_name


def load_drive_config(robot_dir):
    """The robot's mujoco_drive section from its nepi_robot_config.yaml, or None."""
    path = os.path.join(robot_dir, ROBOT_CAPABILITIES_FILENAME)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        capabilities = yaml.safe_load(f) or dict()
    return capabilities.get('mujoco_drive')


class SkidSteerDrive(object):
    """Velocity command -> wheel speeds for a skid-steer robot. See the
    mujoco_drive comments in the robot's nepi_robot_config.yaml for why the
    effective track is wider than the real one."""

    def __init__(self, model, drive_config):
        self.wheel_radius = float(drive_config['wheel_radius_m'])
        self.effective_track = float(drive_config['effective_track_m'])
        # model.actuator() raises KeyError for a name the model does not have.
        self.left_ids = [model.actuator(name).id for name in drive_config['left_actuators']]
        self.right_ids = [model.actuator(name).id for name in drive_config['right_actuators']]

    def apply(self, data, linear_x, angular_z):
        # linear_y (sideways) is not possible for a skid-steer robot and is
        # ignored.
        half_turn = angular_z * self.effective_track / 2.0
        data.ctrl[self.left_ids] = (linear_x - half_turn) / self.wheel_radius
        data.ctrl[self.right_ids] = (linear_x + half_turn) / self.wheel_radius


def make_drive(model, drive_config):
    drive_type = drive_config.get('type', '')
    if drive_type == 'skid_steer':
        return SkidSteerDrive(model, drive_config)
    raise ValueError("unsupported mujoco_drive type '" + str(drive_type) + "'")


class DeviceBridge(object):
    """UDP and TCP link to the device's RBX_SIM driver. Threads only move
    packets in and out under the lock; the physics loop is the only thing that
    touches the MuJoCo model, through update_telemetry, command and the reset
    callback it registers."""

    def __init__(self, device_host, port, reset_callback = None):
        self.device_ip = socket.gethostbyname(device_host)
        self.port = int(port)
        self.reset_callback = reset_callback
        self.lock = threading.Lock()
        self.running = True
        self.telemetry = None
        self.tx_seq = 0
        self.rx_seq = None
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.watchdog_tripped = False
        self.tcp_conn = None
        self.tcp_send_lock = threading.Lock()

        # One UDP socket both ways: telemetry goes out from it, and the device
        # answers to the address it came from.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        for target in (self.telemetryLoop, self.receiveLoop, self.tcpConnectLoop):
            thread = threading.Thread(target = target)
            thread.daemon = True
            thread.start()
        log("bridging to device " + device_host + " (" + self.device_ip + ") port " +
            str(self.port) + " over UDP and TCP")

    def update_telemetry(self, telemetry):
        with self.lock:
            self.telemetry = telemetry

    def command(self):
        """The latest (linear_x, angular_z), or zeros once commands have
        stopped for WATCHDOG_SEC."""
        with self.lock:
            cmd = self.last_cmd
            fresh = cmd is not None and (time.monotonic() - self.last_cmd_time) < WATCHDOG_SEC
        if not fresh:
            if cmd is not None and not self.watchdog_tripped:
                log("no velocity command from the device for " + str(WATCHDOG_SEC) +
                    "s -- stopping the robot")
            self.watchdog_tripped = cmd is not None
            return 0.0, 0.0
        if self.watchdog_tripped:
            log("velocity commands resumed")
            self.watchdog_tripped = False
        return cmd

    def clear_command(self):
        """Forget the last velocity command, so a reset is not undone by it."""
        with self.lock:
            self.last_cmd = None

    def has_data_connection(self):
        with self.tcp_send_lock:
            return self.tcp_conn is not None

    def send_line(self, line_dict):
        """Send one message over TCP. False when there is no connection or the
        send failed (tcpReadLoop then sees the dead socket and reconnects)."""
        with self.tcp_send_lock:
            conn = self.tcp_conn
            if conn is None:
                return False
            try:
                conn.sendall((json.dumps(line_dict) + '\n').encode())
                return True
            except Exception:
                return False

    def telemetryLoop(self):
        interval = 1.0 / TELEMETRY_RATE_HZ
        while self.running:
            with self.lock:
                telemetry = self.telemetry
                if telemetry is not None:
                    self.tx_seq += 1
                    packet = dict(telemetry, type = 'telemetry', seq = self.tx_seq)
            if telemetry is not None:
                try:
                    self.sock.sendto(json.dumps(packet).encode(), (self.device_ip, self.port))
                except Exception as e:
                    if self.running:
                        log("telemetry send failed: " + str(e))
                        time.sleep(1.0)
            time.sleep(interval)

    def receiveLoop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                # Before the first send the socket has no local port yet, and
                # recv on it can fail; the next telemetry send fixes that.
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
                # Stale or duplicated -- unless far behind, which is a
                # restarted driver counting from 1 again, not a late packet.
                if (seq is not None and self.rx_seq is not None and
                        self.rx_seq - SEQ_RESTART_GAP < seq <= self.rx_seq):
                    continue
                if seq is not None:
                    self.rx_seq = seq
                self.last_cmd = (float(msg.get('linear_x', 0.0)), float(msg.get('angular_z', 0.0)))
                self.last_cmd_time = time.monotonic()

    def tcpConnectLoop(self):
        # The device's driver listens; this side connects, and reconnects
        # whenever the connection drops (a restarted driver, a lost link).
        last_warned = 0.0
        while self.running:
            try:
                conn = socket.create_connection((self.device_ip, self.port), timeout = 5)
            except Exception as e:
                if time.monotonic() - last_warned > 10.0:
                    log("data connection to " + self.device_ip + ":" + str(self.port) +
                        " failed (" + str(e) + ") -- retrying")
                    last_warned = time.monotonic()
                time.sleep(CONNECT_RETRY_SEC)
                continue
            # Resets arrive rarely, so a quiet socket is not a dead one.
            conn.settimeout(None)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            with self.tcp_send_lock:
                self.tcp_conn = conn
            log("data connection up to " + self.device_ip + ":" + str(self.port))
            self.tcpReadLoop(conn)
            with self.tcp_send_lock:
                if self.tcp_conn is conn:
                    self.tcp_conn = None
            try:
                conn.close()
            except Exception:
                pass
            if self.running:
                log("data connection lost -- retrying in " + str(CONNECT_RETRY_SEC) + "s")
                time.sleep(CONNECT_RETRY_SEC)

    def tcpReadLoop(self, conn):
        buf = b''
        while self.running:
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
                if msg.get('type') == 'reset' and self.reset_callback is not None:
                    self.reset_callback()

    def close(self):
        self.running = False
        with self.tcp_send_lock:
            conn = self.tcp_conn
            self.tcp_conn = None
        for sock in (self.sock, conn):
            if sock is None:
                continue
            # shutdown first: a thread blocked in recv on it is not woken by
            # close alone.
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass


def depth_to_color_img(depth_m):
    """Colorize a depth frame in metres: close = blue, far = red (JET), the
    same mapping nepi_gazebo_bridge.py uses."""
    depth_m = np.nan_to_num(np.asarray(depth_m, dtype = np.float32), nan = DEPTH_MAX_RANGE_M,
                            posinf = DEPTH_MAX_RANGE_M, neginf = DEPTH_MAX_RANGE_M)
    clipped = np.clip(depth_m, DEPTH_MIN_RANGE_M, DEPTH_MAX_RANGE_M)
    scaled = ((clipped - DEPTH_MIN_RANGE_M) *
              (255.0 / (DEPTH_MAX_RANGE_M - DEPTH_MIN_RANGE_M))).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_JET)


class CameraStreamer(object):
    """Renders the robot's cameras and sends them to the device over the
    bridge's TCP connection. Runs on its own thread with its own offscreen
    OpenGL context, rendering a private copy of the model's state built from
    pose_source, so the physics loop and the viewer are never held up by it."""

    def __init__(self, model, bridge, pose_source):
        self.model = model
        self.bridge = bridge
        # Callable returning (qpos copy, simulated time) -- see SimRunner.poseSnapshot.
        self.pose_source = pose_source
        self.running = True
        self.thread = None
        self.views = [(view, camera) for view, camera in CAMERA_VIEWS
                      if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera) >= 0]
        if not self.views:
            log("the robot has no robot_camera or scene_camera -- no camera images")
            return
        log("camera views: " + ", ".join(view + "_color/" + view + "_depth" for view, _ in self.views))
        self.thread = threading.Thread(target = self.run)
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        renderer = None
        local = mujoco.MjData(self.model)
        interval = 1.0 / IMAGE_RATE_HZ
        try:
            while self.running:
                # No connection, no consumer: skip the rendering entirely.
                if not self.bridge.has_data_connection():
                    time.sleep(0.5)
                    continue
                if renderer is None:
                    # Created here, so its OpenGL context belongs to this thread.
                    try:
                        renderer = mujoco.Renderer(self.model, height = IMAGE_HEIGHT, width = IMAGE_WIDTH)
                    except Exception as e:
                        log("could not start offscreen rendering (" + str(e) + ") -- no camera images")
                        return
                started = time.monotonic()
                qpos, stamp = self.pose_source()
                local.qpos[:] = qpos
                mujoco.mj_forward(self.model, local)
                for view, camera in self.views:
                    renderer.disable_depth_rendering()
                    renderer.update_scene(local, camera = camera)
                    color = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                    renderer.enable_depth_rendering()
                    renderer.update_scene(local, camera = camera)
                    depth = depth_to_color_img(renderer.render())
                    for suffix, img in (('_color', color), ('_depth', depth)):
                        ok, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                        if ok:
                            self.bridge.send_line({
                                'type': 'image', 'camera': view + suffix, 'format': 'jpeg',
                                'data': base64.b64encode(encoded.tobytes()).decode('ascii'),
                                'stamp': stamp})
                delay = interval - (time.monotonic() - started)
                if delay > 0:
                    time.sleep(delay)
        finally:
            if renderer is not None:
                renderer.close()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout = 2.0)


class SimRunner(object):

    def __init__(self, model, robot_body_name, viewer_mode, bridge = None, drive = None):
        self.model = model
        self.data = mujoco.MjData(model)
        self.robot_body_id = model.body(robot_body_name).id
        self.bridge = bridge
        self.drive = drive
        self.viewer = None
        self.running = True
        self.reset_requested = False
        # A reset zeroes data.time; this keeps the total for the exit log.
        self.time_before_resets = 0.0
        # mj_objectVelocity's output buffer: [angular xyz, linear xyz].
        self.velocity = np.zeros(6)
        # The robot pose the camera streamer renders from, refreshed by the
        # physics loop at CONTROL_RATE_HZ.
        self.pose_lock = threading.Lock()
        self.pose_qpos = self.data.qpos.copy()
        self.pose_time = 0.0
        # Offscreen rendering needs a framebuffer at least as big as the
        # images, whatever the world file's <visual><global> says.
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, IMAGE_WIDTH)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, IMAGE_HEIGHT)
        if viewer_mode == 'auto':
            self.openViewer()
        self.streamer = CameraStreamer(model, bridge, self.poseSnapshot) if bridge is not None else None

    def openViewer(self):
        if not os.environ.get('DISPLAY'):
            log("no DISPLAY -- running headless")
            return
        try:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        except Exception as e:
            log("could not open the viewer (" + str(e) + ") -- running headless")
            self.viewer = None
            return
        # Start framed on the robot rather than on the world origin.
        with self.viewer.lock():
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.robot_body_id
            self.viewer.cam.distance = 3.0
            self.viewer.cam.elevation = -20.0
            self.viewer.cam.azimuth = 180.0
        log("viewer open")

    def stop(self, *_):
        self.running = False

    def requestReset(self):
        # Called from the bridge's TCP thread; the physics loop does the reset.
        self.reset_requested = True

    def poseSnapshot(self):
        with self.pose_lock:
            return self.pose_qpos.copy(), self.pose_time

    def resetToStart(self):
        # Back to the pose the model starts in, standing still. The last
        # velocity command is dropped too, or it would drive straight off again
        # until the device's next command arrives.
        self.time_before_resets += self.data.time
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if self.bridge is not None:
            self.bridge.clear_command()
        log("reset to the start pose")

    def controlTick(self):
        """Apply the device's latest command and hand the bridge a telemetry
        snapshot. Runs in the physics loop, so the model is never read or
        written from another thread."""
        if self.bridge is None:
            return
        linear_x, angular_z = self.bridge.command()
        if self.drive is not None:
            self.drive.apply(self.data, linear_x, angular_z)

        body = self.robot_body_id
        w, x, y, z = self.data.xquat[body]
        # Forward speed is in the robot's own frame; the turn rate is about
        # the world's vertical axis.
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body,
                                 self.velocity, 1)
        forward_speed = self.velocity[3]
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body,
                                 self.velocity, 0)
        turn_rate = self.velocity[2]
        self.bridge.update_telemetry({
            'x': float(self.data.xpos[body][0]),
            'y': float(self.data.xpos[body][1]),
            'yaw': math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
            'linear_x': float(forward_speed),
            'angular_z': float(turn_rate),
            'stamp': float(self.data.time),
        })
        with self.pose_lock:
            self.pose_qpos[:] = self.data.qpos
            self.pose_time = float(self.data.time)

    def run(self):
        timestep = self.model.opt.timestep
        sync_interval = 1.0 / VIEWER_SYNC_HZ
        control_every = max(1, int(round((1.0 / CONTROL_RATE_HZ) / timestep)))
        steps = 0
        next_step = time.monotonic()
        next_sync = next_step
        while self.running:
            if self.reset_requested:
                self.reset_requested = False
                self.resetToStart()
            if steps % control_every == 0:
                self.controlTick()
            mujoco.mj_step(self.model, self.data)
            steps += 1

            now = time.monotonic()
            if self.viewer is not None and now >= next_sync:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    log("viewer window closed -- simulation keeps running headless")
                    self.viewer = None
                next_sync = now + sync_interval

            # Real time: one simulated timestep per timestep of wall clock.
            # Falling well behind (a stalled machine) resets the schedule
            # rather than running fast to catch up.
            next_step += timestep
            delay = next_step - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.5:
                next_step = time.monotonic()

        # Streamer first: it renders from this process's model and sends over
        # the bridge.
        if self.streamer is not None:
            self.streamer.stop()
        if self.viewer is not None:
            self.viewer.close()
        if self.bridge is not None:
            self.bridge.close()


def main():
    parser = argparse.ArgumentParser(description = "NEPI MuJoCo simulation process")
    parser.add_argument('--world-file', required = True,
                        help = 'MJCF world file from ENVIRONMENT_CONFIGS')
    parser.add_argument('--robot-dir', required = True,
                        help = 'ROBOT_CONFIGS/<name> folder holding model.xml')
    parser.add_argument('--viewer', choices = ['auto', 'off'], default = 'auto',
                        help = 'auto: open the window when a display is available')
    parser.add_argument('--device-host', default = '',
                        help = "NEPI device to bridge the robot to; no bridge when empty")
    parser.add_argument('--port', type = int, default = FACTORY_PORT,
                        help = "the device driver's bridge port (UDP)")
    args = parser.parse_args()

    try:
        model, robot_body_name = load_model(args.world_file, args.robot_dir)
        drive = None
        if args.device_host:
            drive_config = load_drive_config(args.robot_dir)
            if drive_config is None:
                log("robot has no mujoco_drive section -- its velocity commands will be ignored")
            else:
                drive = make_drive(model, drive_config)
    except Exception as e:
        # One line, so nepi_mujoco.sh can report it as the start error.
        log("failed to load the model: " + str(e).replace('\n', ' '))
        # os._exit, not sys.exit: MuJoCo 3.2.3 segfaults while freeing a spec
        # whose XML failed to parse, which would bury this clean exit code.
        os._exit(1)

    log("loaded world '" + os.path.basename(args.world_file) + "' with robot '" +
        os.path.basename(os.path.normpath(args.robot_dir)) + "' (" + str(model.nbody) +
        " bodies, " + str(model.nu) + " actuators, timestep " + str(model.opt.timestep) + " s)")

    bridge = DeviceBridge(args.device_host, args.port) if args.device_host else None
    runner = SimRunner(model, robot_body_name, args.viewer, bridge, drive)
    if bridge is not None:
        bridge.reset_callback = runner.requestReset
    signal.signal(signal.SIGINT, runner.stop)
    signal.signal(signal.SIGTERM, runner.stop)
    runner.run()
    log("stopped after %.1f s of simulated time" % (runner.time_before_resets + runner.data.time))


if __name__ == '__main__':
    main()
