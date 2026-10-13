# MuJoCo ROBOT_CONFIGS

Each folder here is one robot the Sim Connector can deploy into MuJoCo. The folder name is the robot config name (for example `generic_rover`).

A folder needs:

| File | Purpose |
| --- | --- |
| `model.xml` | The robot as MJCF. Required: the service only offers folders that have one. |
| `nepi_robot_config.yaml` | Display name and capabilities, with the same fields as the Gazebo service's `ROBOT_CONFIGS` (see `nepi_gazebo/gazebo_setup/resources/ROBOT_CONFIGS/README.md`), plus the MuJoCo-only `mujoco_drive` section below. Optional; without it the robot renders no controls. |

## Driving it from the device

`nepi_driver: rbx_sim` in `nepi_robot_config.yaml` does two things:

- The service starts the simulation with its bridge to the device (UDP port 9023, the address in `NEPI_IP`).
- The device's Sim Connector app runs the RBX_SIM driver, so the robot appears in Devices -> Robots.

The `mujoco_drive` section says how the driver's velocity commands (forward m/s, turn rad/s) become wheel speeds:

```yaml
mujoco_drive:
  type: skid_steer          # the only type so far
  wheel_radius_m: 0.1
  effective_track_m: 1.36   # measured, not the wheel track -- see below
  left_actuators: [front_left_motor, rear_left_motor]
  right_actuators: [front_right_motor, rear_right_motor]
```

A skid-steer robot turns by sliding its wheels sideways, so it turns much slower than plain differential drive predicts. `effective_track_m` corrects for that. For `generic_rover` the real 0.34 m track gave about 25% of the commanded turn rate, and 1.36 m (4x) gave 97-107%. Measure it again for a new robot: command a turn rate, compare it with the rate the robot actually reaches, and scale the track by their ratio.

## Writing `model.xml`

- `nepi_mujoco_sim.py` attaches the **first body under `<worldbody>`** into the selected world, at the world origin. Put everything the robot is made of inside that one body, and set the body's `pos` to its starting height.
- Motors go in `<actuator>`. Actuators that drive the robot's own joints come along with it when it is attached.
- Physics settings (`<option>`: timestep, integrator, gravity) belong to the **world** file, not here. An attached robot does not bring its own.
- Name the cameras `robot_camera` (on the robot) and `scene_camera` (chase view). Those two are rendered and sent to the device, in colour and depth, as the robot's camera views; a robot without them has no images.
- Keep the model self-contained for now: no `<include>`, meshes or texture files. Asset paths would resolve against the world file's folder after attaching.

## Changing a robot

The robot is compiled into the MuJoCo model together with the world, so a new robot selection restarts the simulation. A restart takes about a second.

The share syncs both ways each time the service is started (`nepi_mujoco_sync.sh`), and the newer copy of a file wins. Deletes do not sync, so to rename or remove a robot, change it both in `~/mujoco/ROBOT_CONFIGS` and in NEPI storage's `databases/sims/mujoco/ROBOT_CONFIGS`, or the old folder comes back.
