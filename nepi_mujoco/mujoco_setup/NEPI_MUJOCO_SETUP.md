# NEPI MuJoCo Setup

This tutorial sets up a MuJoCo simulation machine that a NEPI device can drive.
When you finish, you will pick a robot and an environment in the NEPI RUI,
press **Deploy**, and drive the simulated robot through NEPI exactly as you
would a real one: the same Devices -> Robots page, the same RBX controls, the
same image views.

MuJoCo is the lighter of NEPI's two simulators. It needs no ROS, installs in
minutes, and runs headless if the machine has no display. What ships with it
today:

| Robot | What drives it | Appears on the device as |
|---|---|---|
| **4-Wheel Rover** (`generic_rover`, skid steer) | NEPI's RBX_SIM driver, over a bridge built into the simulator | `sim_rover` |

and one environment, **Flat**. You can add your own robots and environments
(see [Adding robots and environments](#adding-robots-and-environments)). For a
quadcopter or the obstacle-course worlds, use the Gazebo simulator
([NEPI_GAZEBO_SETUP.md](../../nepi_gazebo/gazebo_setup/NEPI_GAZEBO_SETUP.md)).


################################################################
### How it fits together

```
 Sim machine (Ubuntu 20.04)                          NEPI device
 ---------------------------                         -----------
 nepi_mujoco service  <---- watches ----  nepi_mujoco_config.yaml  <---- written by the
   - nepi_mujoco_sim.py                (on the device's shared         Sim Connector app
     (physics, viewer, cameras,         nepi_storage folder,           (Deploy / Kill)
      device bridge)                    mounted on the sim machine)
        |
        +---- sim machine dials out to the device ---->  RBX_SIM driver
              (telemetry, commands, images, reset)        (sim_rover)
```

The sim machine mounts the device's `nepi_storage` share. The device's Sim
Connector app writes requests (start, stop, which robot, which world) into a
config file on that share, and the **nepi_mujoco service** on the sim machine
polls it once a second and acts on it. The simulation itself is a single
Python process: it steps the physics in real time, shows the viewer, renders
the camera views, and talks to the device. Every network connection between
the two machines is opened from the sim machine, so the sim machine never has
to accept an inbound connection.

**NOTE:** The service's scripts run from your `nepi_simulations` checkout, not
from a copy. Keep the checkout where you cloned it. If you move it, rerun the
setup script (step 4 below). The MuJoCo setup also reuses two scripts from the
`nepi_gazebo` folder of the same checkout, so keep the whole repo, even if you
only use MuJoCo.

**NOTE:** Gazebo and MuJoCo can both be set up on one sim machine, but only one
runs at a time: both drive the device's single `sim_rover` driver on the same
port. Deploying one in the RUI stops the other first.


################################################################
### Requirements

**Sim machine**
- Ubuntu 20.04 (focal). MuJoCo is pinned to 3.2.3, the newest release for
  20.04's Python 3.8.
- A graphics driver with EGL support, for the camera views. A desktop session
  is optional: with one, the MuJoCo viewer window opens; without one, the
  simulator runs headless and the camera views still work.
- Internet access during setup, and `sudo` access.

**NEPI device**
- A running NEPI device reachable from the sim machine, with its
  `nepi_storage` folder shared (the standard NEPI Samba share).
- The **Sim Connector** app and the RBX_SIM driver installed. These come from
  the `nepi_drones` repo, deployed onto the device's NEPI source and built.

**Network**

The sim machine connects out to the device on these ports. If there is a
firewall between them, allow these from the sim machine to the device:

| Port | Protocol | Used for |
|---|---|---|
| 445 | TCP | Mounting the device's `nepi_storage` share |
| 9023 | TCP + UDP | Rover: telemetry, commands, camera images and reset |


################################################################
### 1. Set up the NEPI remote environment

The MuJoCo service builds on NEPI's remote environment, which provides the
device address (`NEPI_IP`), the NEPI user, the share mount and the helper
commands the MuJoCo scripts call. If this machine is not already set up as a
NEPI remote system, follow **NEPI Remote Setup** in the `nepi_setup` repo
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


################################################################
### 2. Clone the simulations repo

Skip this if you already cloned it for Gazebo.

```
cd /home/${USER}
git clone https://github.com/nepi-engine/nepi_simulations.git
```


################################################################
### 3. Install the system packages

If this machine already went through the Gazebo setup, `yq` is installed and
you only need the apt line below.

The service edits its config file with `yq` version 4 (mikefarah's Go
binary, not the apt `yq` package, which is a different tool):

```
sudo apt remove -y yq 2>/dev/null
wget https://github.com/mikefarah/yq/releases/download/v4.16.2/yq_linux_$(dpkg --print-architecture) -O ~/yq
chmod +x ~/yq && sudo mv ~/yq /usr/bin/yq
yq --version
```

The simulator needs pip, OpenCV and PyYAML:

```
sudo apt install -y python3-pip python3-opencv python3-yaml
```


################################################################
### 4. Run the MuJoCo setup script

```
bash /home/${USER}/nepi_simulations/nepi_mujoco/mujoco_setup/scripts/nepi_mujoco_setup.sh
```

It takes a minute or two. It:

1. Installs MuJoCo 3.2.3 for your user (`pip install --user mujoco==3.2.3`),
   leaving the system Python packages alone, and checks that it imports.
2. Checks for `yq`, OpenCV and a working EGL renderer, and warns about any
   that are missing.
3. Stages the robots, environments and config into `~/mujoco`.
4. Installs the `nepi_mujoco_start` command and hooks it into `~/.bashrc`.

Read the end of its output. If it warns that the EGL render test failed, the
simulator will run but send no camera images until the graphics driver is
fixed.

When it finishes, **open a new terminal** so `nepi_mujoco_start` is loaded.


################################################################
### 5. Start the MuJoCo service

In a new terminal on the sim machine:

```
nepi_mujoco_start
```

It asks for your sudo password and, to mount the device's share, the
`nepi_storage` password (the NEPI user's share password; `nepi` on a factory
device). You should see:

```
Mounting shared drive at: //192.168.179.103/nepi_storage to /mnt/nepi_share_storage
Updating MuJoCo Simulation Files and Folders
...
Starting nepi_mujoco service
nepi_mujoco service started with PID ... (log: /tmp/nepi_mujoco_service.log)
```

**NOTE:** Right now the service mounts the share a second time as it starts,
so a second `Password for nepi@//...` prompt appears in the terminal after
the start finishes. Type the password there too; the service waits at that
prompt until you do. Passing the password as an argument
(`nepi_mujoco_start <password>`) does not avoid either prompt yet.

Unlike Gazebo, nothing opens yet: the service waits for the device. The
simulation starts when you press **Deploy** in the RUI.

On the first start this also seeds the device: it creates
`databases/sims/mujoco/` in the device's `nepi_storage` and copies the config,
robots and environments there. Until that has happened once, the device does
not offer MuJoCo as a simulator.

**NOTE:** If the Gazebo service is also running on this machine, starting the
MuJoCo service remounts the shared drive under it. Start both services one
after the other, not while a simulation is running.


################################################################
### 6. Set up the NEPI device

On the device, in the RUI, make sure the **Sim Connector** app is running.
There is no driver to enable: the Sim Connector app starts and stops the
rover's driver itself.


################################################################
### 7. Drive the rover

1. Open the **Sim Connector** app.
2. Set **Simulator** to **MuJoCo**, **Robot Config** to **4-Wheel Rover**, and
   **Environment Config** to **Flat**.
3. Press **Deploy**. If Gazebo is running, it is stopped first. On the sim
   machine the MuJoCo viewer opens, following the rover.
4. Go to **Devices -> Robots** and select **sim_rover**. Its camera views
   (robot and scene, color and depth), NavPose and RBX controls are live.
5. Try **GoTo Position**: forward 3 m. The rover turns toward the target and
   drives there.

**NOTE:** GoTo counts a move as done once the robot is within **Max (m)** of
the target (GoTo Error Bounds, factory value 2 m), and the rover stops when it
is inside half of that. A move shorter than that does nothing. For short
moves, lower Max (m) first (for example to 0.5) and press Enter.

**RESET** in Setup Actions puts the rover back at its start pose.

**The viewer window.** Closing it does not stop the simulation; the rover
keeps running headless until you press **Kill**. To always run headless, set
`MUJOCO_VIEWER: 0` in the config file on the share
(`/mnt/nepi_share_storage/databases/sims/mujoco/config/nepi_mujoco_config.yaml`);
it takes effect at the next Deploy.


################################################################
### Stopping and restarting

**Stop the simulation:** press **Kill** in Sim Connector. The simulation stops;
the nepi_mujoco service keeps running, ready for the next Deploy.

**Restart the service** (needed after you add robots or environments, or
update the repo):

```
pkill -f "mujoco_setup/resources/config/nepi_mujoco\.sh"
nepi_mujoco_start
```

A running simulation is left alone and picked up again by the new service.

**NOTE:** There is no separate stop command for the service yet. Killing it
does not stop a running simulation; use **Kill** in the RUI first if you want
everything down. While the service is down, the device stops the rover's
driver after about 20 seconds.


################################################################
### Adding robots and environments

Robots live in `ROBOT_CONFIGS/` and environments in `ENVIRONMENT_CONFIGS/`.
The device lists whatever is in the copies on its share; the sim machine loads
its copies in `~/mujoco`. When you Deploy, the simulator attaches the robot
into the world and compiles the two together, so changing either restarts the
simulation (it takes about a second).

- **An environment** is a MuJoCo XML file. Physics options (`<option>`,
  integrator, timestep) belong here, not in the robot. Its name in the RUI
  comes from the file name: `my_test_area.xml` shows as "My Test Area".
- **A robot** is a folder with `model.xml` and, optionally,
  `nepi_robot_config.yaml`. See
  [ROBOT_CONFIGS/README.md](resources/ROBOT_CONFIGS/README.md) for every
  field. The rules that catch people out:
  - Only the **first body under `<worldbody>`** is used. It is attached at the
    world origin, and its `pos` sets the start height.
  - Name the cameras **`robot_camera`** and **`scene_camera`**, or the device
    gets no images.
  - No `<include>`, meshes or textures yet: their paths would resolve against
    the world's folder, not the robot's.
  - To drive it from NEPI, set `nepi_driver: rbx_sim` and add a
    `mujoco_drive` section naming the left and right wheel actuators. Only
    `skid_steer` is supported.
  - `effective_track_m` must be measured, not copied from the CAD track width.
    A skid-steer robot turns slower than its geometry predicts; the shipped
    rover uses 1.36 m against a real 0.34 m track. Command a turn rate,
    measure the actual one, and scale until they match.

**How the copies stay in step.** Each `nepi_mujoco_start` syncs the share and
`~/mujoco`:

- **Robots and environments:** the newer copy of each file wins, in both
  directions. Edit in either place; the edit you made last is kept.
- **The config file:** the share's copy always wins.
- **Deletes do not sync.** To remove a robot or world, delete it from the
  share and from `~/mujoco`, or it comes back.

New items appear on the device within a few seconds. The sim machine picks
them up only when the service restarts (see above).


################################################################
### Where to look when something goes wrong

All logs are in `/tmp` on the sim machine:

| Log | What it covers |
|---|---|
| `nepi_mujoco_service.log` | The service: starts, stops, rejected selections. Overwritten each time the service starts |
| `nepi_mujoco_sim.log` | The simulator: model loading, viewer, device bridge, cameras. Appended across runs; lines start `NEPI_MUJOCO_SIM:` |

The service's current state is in the config file on the share:

```
cat /mnt/nepi_share_storage/databases/sims/mujoco/config/nepi_mujoco_config.yaml
```

`MUJOCO_STATE` (`idle`, `starting`, `running`, `stopping`, `failed`) and
`MUJOCO_LAST_ERROR` say what it last did; `MUJOCO_LAST_POLL` should change
every second while the service is alive.

**Common problems**

- **The RUI says the MuJoCo service isn't running.** The device has not seen
  the service update its config for 20 seconds. Check it is running
  (`pgrep -af nepi_mujoco.sh`); if the terminal is still showing the second
  password prompt, the service is waiting there. Otherwise start it with
  `nepi_mujoco_start`.

- **MuJoCo is not offered as a simulator.** The share has never been seeded.
  Run `nepi_mujoco_start` once on the sim machine.

- **Deploy fails straight away.** `MUJOCO_LAST_ERROR` and the end of
  `nepi_mujoco_sim.log` give the reason. A robot or world XML that does not
  load is reported as `failed to load the model: ...`; a missing Python
  package (for example OpenCV) shows as an import error.

- **No camera images.** Check `nepi_mujoco_sim.log` for `could not start
  offscreen rendering` (the EGL driver) or `no robot_camera or scene_camera`
  (the robot's camera names).

- **The rover appears but will not move.** Check that its
  `nepi_robot_config.yaml` has `nepi_driver: rbx_sim` and a `mujoco_drive`
  section, and that `nepi_mujoco_sim.log` shows the bridge connecting to the
  device on port 9023.

- **A Deploy reports it started with a different world or robot than you
  chose.** The device can see a file the sim machine does not have yet.
  Restart the service so it syncs from the share.

- **"Gazebo reports running but its service isn't responding".** Gazebo's
  status file still says `running` from an earlier session, so it cannot be
  stopped from the RUI. It is harmless to MuJoCo; it clears when the Gazebo
  service next runs.
