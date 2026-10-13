# NEPI Gazebo Setup

This tutorial sets up a Gazebo simulation machine that a NEPI device can drive.
When you finish, you will pick a robot and an environment in the NEPI RUI,
press **Deploy**, and fly or drive the simulated robot through NEPI exactly as
you would a real one: the same Devices -> Robots page, the same RBX controls,
the same image views.

Two robots ship with it:

| Robot | What drives it | Appears on the device as |
|---|---|---|
| **4-Wheel Rover** (`generic_rover`) | NEPI's RBX_SIM driver, over a small bridge | `sim_rover` |
| **Quadcopter** (`quadcopter`) | ArduPilot SITL (ArduCopter 4.5) through mavros | `ardupilot_sitl` |

and three environments: **Flat**, **Obstacle Course** and **Aerial Obstacle
Course**. You can add your own robots and environments (see
[Adding robots and environments](#adding-robots-and-environments)).


################################################################
### How it fits together

```
 Sim machine (Ubuntu 20.04)                          NEPI device
 ---------------------------                         -----------
 nepi_gazebo service  <---- watches ----  nepi_gazebo_config.yaml  <---- written by the
   - Gazebo 11                         (on the device's shared         Sim Connector app
   - rover bridge                       nepi_storage folder,           (Deploy / Kill)
   - ArduPilot SITL                     mounted on the sim machine)
   - camera / reset helpers
        |
        +---- sim machine dials out to the device ---->  RBX drivers
              (telemetry, commands, images, MAVLink)      (sim_rover / ardupilot_sitl)
```

The sim machine mounts the device's `nepi_storage` share. The device's Sim
Connector app writes requests (start, stop, which robot, which world) into a
config file on that share, and the **nepi_gazebo service** on the sim machine
polls it once a second and acts on it. Every network connection between the
two machines is opened from the sim machine to the device, so the sim machine
never has to accept an inbound connection.

**NOTE:** The service's scripts run from your `nepi_simulations` checkout, not
from a copy. Keep the checkout where you cloned it. If you move it, rerun the
bash setup step (step 3 below).


################################################################
### Requirements

**Sim machine**
- Ubuntu 20.04 (focal). ROS Noetic, which the robots need to move, is only
  installed on 20.04; on other releases Gazebo runs but nothing can be driven.
- A desktop session. The service runs Gazebo with its GUI.
- Several GB of free disk (ArduPilot alone is about 2 GB once built), plus
  internet access during setup.
- `sudo` access.

**NEPI device**
- A running NEPI device reachable from the sim machine, with its
  `nepi_storage` folder shared (the standard NEPI Samba share).
- The **Sim Connector** app and the simulation RBX drivers installed. These
  come from the `nepi_drones` repo, deployed onto the device's NEPI source and
  built.

**Network**

The sim machine connects out to the device on these ports. If there is a
firewall between them, allow these from the sim machine to the device:

| Port | Protocol | Used for |
|---|---|---|
| 445 | TCP | Mounting the device's `nepi_storage` share |
| 9023 | TCP + UDP | Rover: telemetry, commands and camera images |
| 9031 | TCP | Quadcopter: MAVLink to the ArduPilot driver |
| 9032 | TCP | Quadcopter: camera images |
| 9034 | TCP | Quadcopter: RESET_SIM |

Everything else (Gazebo, SITL on 5760/5771, the ArduPilot plugin on UDP
9002/9003, the quad camera and reset helpers on 9026/9021) stays local to the
sim machine.


################################################################
### 1. Set up the NEPI remote environment

The Gazebo setup builds on NEPI's remote environment, which provides the
device address (`NEPI_IP`), the NEPI user, the share mounting tools and the
helper commands the Gazebo scripts call. If this machine is not already set up
as a NEPI remote system, follow **NEPI Remote Setup** in the `nepi_setup` repo
first:

```
sudo apt update && sudo apt install -y git
cd /home/${USER}
git clone https://github.com/nepi-engine/nepi_setup.git
bash /home/${USER}/nepi_setup/scripts/remote_env_setup.sh
source ~/.bashrc
```

Then check that the sim machine can see the device:

```
echo $NEPI_IP
ping -c 3 ${NEPI_IP%%/*}
```

`NEPI_IP` should be your device's address (the factory default is
`192.168.179.103`). If it is wrong, fix it in `~/.nepi_bash_utils` and open a
new terminal.

**NOTE:** Without this step the Gazebo installer stops at its internet check
with "No Internet Connection Detected", even when the machine is online,
because the check it calls comes from the NEPI remote environment.


################################################################
### 2. Clone the simulations repo

```
cd /home/${USER}
git clone https://github.com/nepi-engine/nepi_simulations.git
```


################################################################
### 3. Run the Gazebo setup script

```
bash /home/${USER}/nepi_simulations/nepi_gazebo/gazebo_setup/scripts/nepi_gazebo_setup.sh
```

It asks for your sudo password, then runs unattended. Expect it to take a
while: it clones and builds ArduPilot and a Gazebo plugin from source. It:

1. Installs `yq` (v4.16.2), which the service uses to edit its config file.
2. Clones ArduPilot into `~/ardupilot` (branch `Copter-4.5`) and installs its
   prerequisites.
3. Adds `~/ardupilot/Tools/autotest` to your `PATH`.
4. Builds ArduCopter SITL and initializes its parameters (`eeprom.bin`).
5. Installs Gazebo 11 and ROS Noetic (`ros-base` plus the Gazebo ROS packages).
6. Clones and builds the ArduPilot Gazebo plugin into `~/ardupilot_gazebo`.
7. Stages the robots, environments and config into `~/gazebo`, including the
   quadcopter model, which is copied from ArduPilot's `iris_with_ardupilot`
   (it is GPL licensed, so it is not kept in this repo).
8. Installs the NEPI Gazebo bash commands (`nepi_gazebo_start` and friends)
   and hooks them into `~/.bashrc`.

When it finishes, **open a new terminal** so the new `PATH` and commands are
loaded.

Optional check, independent of NEPI: in two terminals,

```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```
```
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

The quadcopter should appear and respond to MAVProxy commands. Close both
before continuing.

**NOTE:** The setup script appends to `~/.bashrc` each time it runs. If you
rerun it, you can delete the duplicate `PATH`/`GAZEBO_*` lines it adds.


################################################################
### 4. Start the Gazebo service

In a new terminal on the sim machine:

```
nepi_gazebo_start
```

It asks for your sudo password and, to mount the device's share, the
`nepi_storage` password (the NEPI user's share password; `nepi` on a factory
device). You can pass the share password instead of typing it,
`nepi_gazebo_start <password>`, but it then lands in your shell history.

You should see:

```
Mounting shared drive at: //192.168.179.103/nepi_storage to /mnt/nepi_share_storage
Updating Gazebo Simulation Files and Folders
...
Launching Gazebo with world: /home/<you>/gazebo/ENVIRONMENT_CONFIGS/<world>
Gazebo running with PID ...
Starting nepi_gazebo service
nepi_gazebo service started with PID ...
```

and a Gazebo window with a robot in it. The service now runs in the
background; you can close the terminal.

On the first start this also seeds the device: it creates
`databases/sims/gazebo/` in the device's `nepi_storage` and copies the
config, robots and environments there. Until that has happened once, the
device lists Gazebo as "Not configured on this deployment".


################################################################
### 5. Set up the NEPI device

On the device, in the RUI:

1. Make sure the **Sim Connector** app is running.
2. **For the quadcopter only:** on the Drivers page, enable
   **Ardupilot Autopilot (SITL)** (`RBX_ARDUPILOT_SITL`). Leave its connection
   on `SITL`. You only do this once. The rover needs no driver setting; the
   Sim Connector app starts and stops its driver itself.


################################################################
### 6. Drive the rover

1. Open the **Sim Connector** app.
2. Set **Simulator** to **Gazebo**, **Robot Config** to **4-Wheel Rover**, and
   **Environment Config** to any world.
3. Press **Deploy**. Gazebo restarts with that world and robot; this can take
   up to a minute.
4. Go to **Devices -> Robots** and select **sim_rover**. Its camera views,
   NavPose and RBX controls are live.
5. Try **GoTo Position**: forward 3 m. The rover turns toward the target and
   drives there.

**NOTE:** GoTo counts a move as done once the robot is within **Max (m)** of
the target (GoTo Error Bounds, factory value 2 m), and the rover stops when it
is inside half of that. A move shorter than that does nothing. For short
moves, lower Max (m) first (for example to 0.5) and press Enter.

**RESET** in Setup Actions puts the rover back at the origin.


################################################################
### 7. Fly the quadcopter

1. In **Sim Connector**, set **Robot Config** to **Quadcopter** and press
   **Deploy**.
2. Wait for the vehicle to come up. On the sim machine the service starts
   ArduPilot SITL once the quad is in the world, and on the device the
   ArduPilot driver finds it and starts **ardupilot_sitl**. Allow about a
   minute.
3. Go to **Devices -> Robots** and select **ardupilot_sitl**. It starts in
   **DISARM** / **STABILIZE**.
4. Wait for SITL's GPS and EKF to settle before arming, usually 30 to 60
   seconds after it starts. If the device log is open, wait for
   `FCU: EKF3 IMU0 is using GPS`.
5. Set **Mode** to **GUIDED**, set **State** to **ARM**, then send the
   **TAKEOFF** setup action. The quad climbs to 5 m (the `takeoff_height_m`
   setting).
6. Once airborne, **Autonomous Ready** turns green. Send **GoTo Position**,
   for example forward 5 m.

The quad has four camera views (robot and scene, color and depth), from two
small camera models the service attaches to it. **RESET_SIM** puts the quad
back at the origin and disarms it.

If arming is refused, the device log shows ArduPilot's reason on a line
starting `PreArm:`.


################################################################
### Stopping and restarting

**Stop the simulation:** press **Kill** in Sim Connector. Gazebo, the robot's
driver bridge and SITL all stop; the nepi_gazebo service keeps running, ready
for the next Deploy.

**Restart the service** (needed after you change robots or environments, or
update the repo):

```
pkill -f "gazebo_setup/resources/config/nepi_gazebo\.sh"
nepi_gazebo_start
```

A running Gazebo is left alone and picked up again by the new service.

**NOTE:** There is no separate stop command for the service yet. Killing it
does not stop Gazebo, SITL or the ROS master; use **Kill** in the RUI first if
you want everything down.


################################################################
### Adding robots and environments

Robots live in `ROBOT_CONFIGS/` and environments (`.world` files) in
`ENVIRONMENT_CONFIGS/`. The device lists whatever is in the copies on its
share; the sim machine spawns from its copies in `~/gazebo`.

- A robot is a folder with `model.config`, `model.sdf` and, optionally,
  `nepi_robot_config.yaml`, which says what the robot can do and which NEPI
  driver controls it. See
  [ROBOT_CONFIGS/README.md](resources/ROBOT_CONFIGS/README.md) for every field
  and the rules a robot model must follow.
- An environment is a `.world` file. Its name in the RUI comes from the file
  name: `my_test_area.world` shows as "My Test Area".

**How the copies stay in step.** Each `nepi_gazebo_start` syncs the share and
`~/gazebo` in both directions, with two rules to keep in mind:

- **The share wins.** If a file differs, the share's copy overwrites
  `~/gazebo`'s. Make edits on the share
  (`/mnt/nepi_share_storage/databases/sims/gazebo/` while mounted), or in
  both places.
- **Deletes do not sync.** To remove a robot or world, delete it from the
  share and from `~/gazebo`, or it comes back.

New items appear on the device within a few seconds. The sim machine picks
them up only when the service restarts (see above).


################################################################
### Where to look when something goes wrong

All logs are in `/tmp` on the sim machine:

| Log | What it covers |
|---|---|
| `nepi_gazebo_service.log` | The service; also Gazebo's own output when the service started it |
| `nepi_gazebo.log` | Gazebo, when `nepi_gazebo_start` launched it |
| `nepi_gazebo_bridge.log` | Rover bridge |
| `nepi_gazebo_sitl.log`, `ArduCopter.log` | ArduPilot SITL and the vehicle |
| `nepi_gazebo_mavlink_relay.log` | Quadcopter MAVLink to the device |
| `nepi_gazebo_quad_cameras.log`, `nepi_gazebo_camera_relay.log` | Quadcopter cameras |
| `nepi_gazebo_reset_listener.log`, `nepi_gazebo_control_relay.log` | Quadcopter RESET_SIM |
| `nepi_gazebo_roscore.log` | ROS master |

The service's current state is in the config file on the share:

```
cat /mnt/nepi_share_storage/databases/sims/gazebo/config/nepi_gazebo_config.yaml
```

`GAZEBO_STATE` (`idle`, `starting`, `running`, `stopping`, `failed`) and
`GAZEBO_LAST_ERROR` say what it last did; `GAZEBO_LAST_POLL` should change
every second while the service is alive.

**Common problems**

- **The RUI says the Gazebo service isn't running.** The device has not seen
  the service update its config for 20 seconds. Check it is running
  (`pgrep -af nepi_gazebo.sh`) and start it with `nepi_gazebo_start`.

- **Gazebo is listed as "Not configured on this deployment".** The share has
  never been seeded. Run `nepi_gazebo_start` once on the sim machine.

- **The rover appears but will not move.** Check `nepi_gazebo_bridge.log` for
  a connection to the device, and that the device is reachable on port 9023.

- **The quad spawns but never becomes ardupilot_sitl on the device.** Check
  that **Ardupilot Autopilot (SITL)** is enabled on the device's Drivers page,
  and that `nepi_gazebo_mavlink_relay.log` shows `connected both ends,
  relaying`.

- **The quad sits still and the log says `imu_sensor ... not found, abort
  ArduPilot plugin`.** The quad's `model.sdf` still has ArduPilot's original,
  model-scoped `imuName`. The setup script rewrites it to
  `<imuName>imu_sensor</imuName>`; if the share holds an older copy, the share
  wins. Fix it in both `~/gazebo/ROBOT_CONFIGS/quadcopter/model.sdf` and the
  share's copy.

- **A Deploy reports it started with a different world or robot than you
  chose.** The device can see a file the sim machine does not have yet.
  Restart the service so it syncs from the share.

- **The first quadcopter spawn is slow.** Its model includes Gazebo's
  `gimbal_small_2d`, which Gazebo may download from its online model database
  the first time it is used.

- **"MuJoCo reports running but its service isn't responding".** The MuJoCo
  service's status file still says `running` from an earlier session. It is
  harmless to Gazebo; it clears when the MuJoCo service next runs.

- **"Edited robot and environment dimensions were not sent to the sim
  machine".** The Gazebo service does not accept dimension edits from the RUI
  yet; the robot spawns at its model's own size.
