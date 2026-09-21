# ROBOT_CONFIGS

One subfolder per selectable robot. `nepi_gazebo.sh` spawns the selected one
into the **running** world, so these are not referenced by any `.world` file --
files in `ENVIRONMENT_CONFIGS/` are scenery only.

A robot config is a plain Gazebo model directory, optionally plus one NEPI file:

```
ROBOT_CONFIGS/<name>/
  model.config            # required -- the service validates its presence
  model.sdf               # required -- this is what actually gets spawned
  nepi_robot_config.yaml  # optional -- what the robot CAN DO (see below)
  ...                     # meshes, dimensions.yaml, anything else it needs
```

## nepi_robot_config.yaml

Without it the folder is still selectable and still spawns; it just reports no
capabilities, so the RUI renders no controls for it. That is the same safe
all-flags-off state the factory default profile uses, not a broken one.

With it, dropping the folder on the share is genuinely all that is needed: the
robot appears in the selector, spawns, and renders the right controls, with no
edit on the device side.

It is a separate file from `model.config` on purpose. That one is Gazebo's own
manifest with its own schema, and upstream models ship their own -- a symlinked
upstream model could not be given NEPI keys there at all.

Fields (every one optional; anything omitted is off / zero / empty):

| Field | Meaning |
|---|---|
| `display_name` | Shown in the selector. Defaults to the folder name. |
| `description` | Free text. |
| `wheel_count`, `motor_count` | Integers. `motor_count > 0` enables motor controls. |
| `has_goto_position`, `has_goto_pose`, `has_goto_location` | Which goto surfaces are commandable. |
| `has_go_home`, `has_set_home`, `has_go_stop` | Home / stop controls. |
| `setup_actions`, `go_actions` | Named action lists, e.g. `RESET`, `TAKEOFF`. |
| `has_camera_view_control`, `available_camera_view_modes` | Camera view switching. |
| `has_environment_controls` | Whether environment controls are offered. |

`gazebo_robot_config` is NOT a field here: a folder always maps to itself, and
the app fills that in. It exists only on a capability profile in
`sim_connector_vehicle_configs.yaml` on the device, which is the other way a
robot can be offered.

### When a device-side profile already claims the folder

A capability profile in `sim_connector_vehicle_configs.yaml` can point at a
folder through its own `gazebo_robot_config` (that is how `4-Wheel Rover` maps
to `generic_rover`). When that happens the robot is listed **once**, under the
profile's name and key -- not twice under two names with different
capabilities.

The folder still wins on capabilities. Anything in its
`nepi_robot_config.yaml` is merged over the profile's fields, so editing the
robot on the share is enough and the profile does not have to be kept in sync
by hand. The profile keeps its KEY, because other config points at profiles by
name (a launch target's `default_robot_config`, for instance) and renaming
would break them. A claimed folder with no capabilities file changes nothing,
so a plain model directory never strips capabilities off a profile that
already describes it.

The folder name is the identity: it is what goes in
`GAZEBO_CURRENT_ROBOT_CONFIG`, what the spawned model is named in the world
(`gz model -m <folder name>` overrides whatever `<model name>` the SDF carries),
and what the despawn-on-swap looks for.

## model.sdf must be a real model, never an `<include>` wrapper

It is tempting to make a config that just pulls in a model installed elsewhere:

```xml
<!-- DO NOT DO THIS -->
<sdf version="1.6">
  <model name="whatever">
    <include><uri>model://something_installed</uri></include>
  </model>
</sdf>
```

Two independent reasons not to:

1. **It double-loads the included model's plugins.** Found live 2026-08-18
   under Gazebo Classic 11, in `generic_rover.world`: wrapping an `<include>`
   in an outer `<model>` element produced "Tried to advertise a service that is
   already advertised" plus the DiffDrive plugin's startup lines twice per
   launch, and left both camera topics with *no* active publisher despite the
   model existing in `get_world_properties`. That note names it as the likely
   mechanism behind the long-reported "can't see some of the cameras in the
   robot viewer". `<include>`'s own `<name>` element is the documented-correct
   way to rename an included model, and does not re-instantiate it -- but that
   only helps inside a world file, not here, because:

2. **A bare `<include>` at SDF root does not spawn at all.** `gz model -f` on
   such a file fails with "Could not find model tag in SDFormat file" and spawns
   nothing, while still exiting 0 (verified against Gazebo 11.15.1 on
   2026-09-18). So the wrapper cannot simply be unwrapped.

Between the two, there is no safe wrapper shape for `gz model -f`. Put the real
model definition here instead. To offer a model that is installed elsewhere on
the machine (an upstream one from `ardupilot_gazebo`, say), symlink the whole
directory rather than wrapping it -- `model://` references inside it still
resolve through `GAZEBO_MODEL_PATH`:

```
ln -s /usr/share/gazebo-11/models/iris_with_ardupilot  ROBOT_CONFIGS/iris_with_ardupilot
```

## Beware: gz exit status is meaningless

`gz model` exits 0 whatever happens -- for a spawn whose SDF file is missing,
for one that produces no model, and for `-i`/`-d` on a model that is not there.
`nepi_gazebo.sh` therefore confirms every spawn and delete by polling for the
model's actual presence, and anything else scripting against `gz` should too.
