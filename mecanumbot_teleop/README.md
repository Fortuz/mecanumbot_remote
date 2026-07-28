# mecanumbot_teleop

ROS 2 teleoperation package for controlling the mecanumbot base and accessory actuators.

This package provides two executable nodes:

- `mecanumbot_keyboard`
- `mecanumbot_joystick`

Both publish the same pair of topics, so only one should be run at a time.

## Node: `mecanumbot_keyboard`

### Publishers

| Topic              | Data type                          | Function                                                                     |
| ------------------ | ---------------------------------- | ---------------------------------------------------------------------------- |
| /cmd_vel           | geometry_msgs/msg/Twist            | Publishes velocity commands from keyboard inputs (x/y linear and z angular). |
| /cmd_accessory_pos | mecanumbot_msgs/msg/AccessMotorCmd | Publishes camera/gripper position commands from keyboard keys.               |

### Key map

| Key        | Effect                                                  |
| ---------- | ------------------------------------------------------- |
| `w` / `x`  | Increase / decrease linear x velocity (forward / back). |
| `a` / `d`  | Increase / decrease linear y velocity (left / right).   |
| `q` / `e`  | Increase / decrease angular z velocity (rotate).        |
| `i` / `k`  | Camera (neck) up / down.                                |
| `j` / `l`  | Open / close gripper.                                   |
| space, `s` | Force stop — zeroes both target and control velocities. |
| `Ctrl-C`   | Quit; a zero `Twist` is published on the way out.       |

### Behavior

- Uses raw terminal key capture (TTY) for low-latency control; each keypress steps the
  target velocity rather than setting it absolutely.
- Applies velocity ramp profiling (`make_simple_profile`, half a step per iteration) and
  clamps to the limits below.
- Publishes only on a handled keypress, then sleeps 10 ms.

### Limits and positions

| Constant               | Value                              |
| ---------------------- | ---------------------------------- |
| Max linear velocity    | 0.26                               |
| Max angular velocity   | 1.82                               |
| Linear step size       | 0.01                               |
| Angular step size      | 0.05                               |
| Camera position range  | 2.0 to 8.6                         |
| Gripper position range | 1.6 to 8.54 (front / neutral 5.12) |

## Node: `mecanumbot_joystick`

### Publishers

| Topic              | Data type                          | Function                                                     |
| ------------------ | ---------------------------------- | ------------------------------------------------------------ |
| /cmd_vel           | geometry_msgs/msg/Twist            | Publishes base velocity commands derived from joystick axes. |
| /cmd_accessory_pos | mecanumbot_msgs/msg/AccessMotorCmd | Publishes accessory positions from joystick button actions.  |

### Subscribers

| Topic | Data type           | Processing                                                                             |
| ----- | ------------------- | -------------------------------------------------------------------------------------- |
| joy   | sensor_msgs/msg/Joy | Reads controller axes/buttons, applies deadzone and mappings, updates command targets. |

### Button map (Xbox-style defaults)

| Input       | Effect                                   |
| ----------- | ---------------------------------------- |
| Left stick  | Linear x (axis 1) and linear y (axis 0). |
| Right stick | Angular z (axis 3).                      |
| `Y` / `A`   | Camera to max / mid position.            |
| `X` / `B`   | Open / close gripper.                    |
| `Back`      | Emergency stop — zeroes all velocities.  |

### Parameters

| Parameter                                  | Default                     | Function                                          |
| ------------------------------------------ | --------------------------- | ------------------------------------------------- |
| `axes.linear_x`                            | `1`                         | Joy axis index for linear x.                      |
| `axes.linear_y`                            | `0`                         | Joy axis index for linear y.                      |
| `axes.angular`                             | `3`                         | Joy axis index for angular z.                     |
| `buttons.A` / `.B` / `.X` / `.Y` / `.Back` | `0` / `1` / `2` / `3` / `6` | Button indices in `joy.buttons`.                  |
| `sensitivity.deadzone`                     | `0.05`                      | Axis values below this are treated as zero.       |
| `smoothing.lin_step`                       | `0.01`                      | Linear ramp step (currently unused — see below).  |
| `smoothing.ang_step`                       | `0.05`                      | Angular ramp step (currently unused — see below). |
| `max_lin_vel`                              | `0.234`                     | Axis value `1.0` maps to this linear velocity.    |
| `max_ang_vel`                              | `1.092`                     | Axis value `1.0` maps to this angular velocity.   |
| `publish_hz`                               | `3.0`                       | Timer rate for publishing both topics.            |

### Behavior

- Axis and button indices are parameters, so other controller layouts can be used
  without code changes; out-of-range indices are read safely as `0`.
- Axes map linearly onto the velocity limits after the deadzone is applied.
- Ramp smoothing is commented out in `timer_publish` — the control velocity is set
  directly from the target, so the `smoothing.*` parameters currently have no effect.
- Both topics are republished on every timer tick, not only when the joystick moves.

## File functions

| File or folder                              | Function                                                                                |
| ------------------------------------------- | --------------------------------------------------------------------------------------- |
| mecanumbot_teleop/script/teleop_keyboard.py | Keyboard teleoperation executable implementation.                                       |
| mecanumbot_teleop/script/teleop_joystick.py | Joystick teleoperation executable implementation.                                       |
| launch/launch_joy_teleop.launch.py          | Starts `joy_node` (`/dev/input/js0`, deadzone 0.05) and `mecanumbot_joystick` together. |
| setup.py                                    | Installs launch files and both executable entry points.                                 |

## Run

```bash
ros2 run mecanumbot_teleop mecanumbot_keyboard
ros2 launch mecanumbot_teleop launch_joy_teleop.launch.py
```
