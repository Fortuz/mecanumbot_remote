# mecanumbot_teleop

ROS 2 teleoperation package for controlling the mecanumbot base and accessory actuators.

This package provides one executable node:

- `mecanumbot_keyboard`

It publishes the same pair of topics as `mecanumbot_joy` on the robot, so only one
of them should be driving at a time.

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
| `i` / `k`  | Camera (neck) up to 8.6 / down to the mid position 5.3. |
| `j` / `l`  | Open / close gripper (fixed preset positions).          |
| space, `s` | Force stop — zeroes both target and control velocities. |
| `Ctrl-C`   | Quit; a zero `Twist` is published on the way out.       |

### Behavior

- Uses raw terminal key capture (TTY) for low-latency control; each keypress steps the
  target velocity rather than setting it absolutely.
- Applies velocity ramp profiling (`make_simple_profile`, half a step per iteration) and
  clamps to the limits below.
- Publishes only on a handled keypress — the `Twist` and the current accessory
  command together, whichever key it was — then sleeps 10 ms.
- The accessory keys jump to preset positions rather than stepping; the command
  starts at camera 8.6 and both grippers at 5.12.

### Limits and positions

| Constant               | Value                              |
| ---------------------- | ---------------------------------- |
| Max linear velocity    | 0.26                               |
| Max angular velocity   | 1.82                               |
| Linear step size       | 0.01                               |
| Angular step size      | 0.05                               |
| Camera position range  | 2.0 to 8.6 (keys reach 8.6 and 5.3) |
| Gripper position range | 1.6 to 8.54 (front / neutral 5.12) |

## Joystick teleoperation has moved

The `mecanumbot_joystick` node used to live here. It is now
`mecanumbot_joy` in the `mecanumbot` repository, running **on the robot**
rather than on the operator PC.

It is not a straight port. The old node hardcoded which button did what and
only let you remap the *indices*; the replacement is driven by a YAML profile
in `mecanumbot_description/config/joystick/`, editable from the robot's web
GUI, and it also replaced the separate controller/mapping stack that used to
live in `mecanumbot_gui`.

```bash
# on the robot
ros2 launch mecanumbot_joy joy_teleop.launch.py
```

This package keeps the keyboard node, which still runs from the operator PC.

## File functions

| File or folder                              | Function                                                                                |
| ------------------------------------------- | --------------------------------------------------------------------------------------- |
| mecanumbot_teleop/script/teleop_keyboard.py | Keyboard teleoperation executable implementation. |
| setup.py                                    | Installs the `mecanumbot_keyboard` entry point.   |

## Run

```bash
ros2 run mecanumbot_teleop mecanumbot_keyboard
```
