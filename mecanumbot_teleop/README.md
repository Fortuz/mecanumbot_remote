# mecanumbot_teleop

ROS 2 teleoperation package for controlling mecanumbot base and accessory actuators.

This package provides two executable nodes:
- mecanumbot_keyboard
- mecanumbot_joystick

## Node: mecanumbot_keyboard

### Publishers
| Topic | Data type | Function |
|---|---|---|
| /cmd_vel | geometry_msgs/msg/Twist | Publishes velocity commands from keyboard inputs (x/y linear and z angular). |
| /cmd_accessory_pos | mecanumbot_msgs/msg/AccessMotorCmd | Publishes camera/gripper position commands from keyboard keys. |



### Behavior
- Uses raw terminal key capture (TTY) for low-latency control.
- Applies velocity ramp profiling and max velocity constraints.
- Supports emergency stop with space or s.

## Node: mecanumbot_joystick

### Publishers
| Topic | Data type | Function |
|---|---|---|
| /cmd_vel | geometry_msgs/msg/Twist | Publishes base velocity commands derived from joystick axes. |
| /cmd_accessory_pos | mecanumbot_msgs/msg/AccessMotorCmd | Publishes accessory positions from joystick button actions. |

### Subscribers
| Topic | Data type | Processing |
|---|---|---|
| joy | sensor_msgs/msg/Joy | Reads controller axes/buttons, applies deadzone and mappings, updates command targets. |


### Behavior
- Parameterized axis/button mapping for controller compatibility.
- Includes deadzone and velocity limits.
- Back button triggers emergency stop.
- Timer-based command publishing at configurable rate.

## File functions

| File or folder | Function |
|---|---|
| mecanumbot_teleop/script/teleop_keyboard.py | Keyboard teleoperation executable implementation. |
| mecanumbot_teleop/script/teleop_joystick.py | Joystick teleoperation executable implementation. |
| launch/launch_joy_teleop.launch.py | Starts joy_node and mecanumbot_joystick together. |
| setup.py | Installs launch files and both executable entry points. |
