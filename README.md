# nepi_simulations

Sim-machine setup for driving simulated robots from a NEPI device. Each
simulator here installs on a separate Ubuntu machine (the sim machine), runs a
small service that watches a config file in the device's NEPI storage, and
starts or stops the simulation when the device's Sim Connector app asks for it.
The device then drives the simulated robot through its normal RBX driver, the
same way it drives a real one.

Two simulators are supported:

| Simulator | Robots | Environments | Setup guide |
|---|---|---|---|
| **Gazebo** (Classic 11, with ROS Noetic) | 4-Wheel Rover, Quadcopter (ArduPilot SITL) | Flat, Obstacle Course, Aerial Obstacle Course | [NEPI_GAZEBO_SETUP.md](nepi_gazebo/gazebo_setup/NEPI_GAZEBO_SETUP.md) |
| **MuJoCo** (no ROS needed) | 4-Wheel Rover | Flat | [NEPI_MUJOCO_SETUP.md](nepi_mujoco/mujoco_setup/NEPI_MUJOCO_SETUP.md) |

Both can be set up on one sim machine, but only one runs at a time: both drive
the device's single `sim_rover` driver on the same port, and deploying one in
the RUI stops the other first. MuJoCo's setup reuses scripts from the
`nepi_gazebo` folder, so keep the whole repo even if you only use MuJoCo.

After setup, start a service with `nepi_gazebo_start` or `nepi_mujoco_start`,
and stop it, along with everything it started, with `nepi_gazebo_stop` or
`nepi_mujoco_stop`.
