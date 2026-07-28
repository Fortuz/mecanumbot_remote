# mecanumbot_remote

Remote-compute ROS 2 packages for controlling mecanumbot from an external PC.
Both packages here are operator-side tools: they talk to the robot over the ROS 2
graph and are not meant to run on the robot's Jetson.

## Packages in this repository

| Package | Type | Purpose |
| --- | --- | --- |
| `mecanumbot_teleop` | Python (`ament_python`) | Manual control nodes — keyboard and joystick — publishing `/cmd_vel` and `/cmd_accessory_pos`. |
| `mecanumbot_ledgui` | Python (`ament_python`) | Tkinter desktop GUI that drives the robot LEDs through the `set_led_status` / `get_led_status` services. |

## Executables

| Executable | Package | ROS node name | Interfaces |
| --- | --- | --- | --- |
| `mecanumbot_keyboard` | `mecanumbot_teleop` | `mecanumbot_keyboard` | Publishes `/cmd_vel`, `/cmd_accessory_pos`. |
| `mecanumbot_joystick` | `mecanumbot_teleop` | `mecanumbot_joystick` | Subscribes `joy`; publishes `/cmd_vel`, `/cmd_accessory_pos`. |
| `mecanumbot_ledgui` | `mecanumbot_ledgui` | `mecanumbot_ledgui_client` | Service client for `set_led_status`, `get_led_status`. |

No nodes are implemented at the repository root — it only groups the two packages.

## Dependencies

Both packages depend on `rclpy` and `mecanumbot_msgs`. `mecanumbot_teleop` also uses
`geometry_msgs` and `sensor_msgs/Joy` (the joystick launch file starts the standard
`joy` package's `joy_node`). `mecanumbot_ledgui` needs a working Tkinter install
(`python3-tk`) and a display.

## Build

```bash
cd ~/Documents/mecanumbot_ws
colcon build --symlink-install --packages-select mecanumbot_teleop mecanumbot_ledgui
source install/setup.bash
```

## Quick run examples

```bash
# keyboard teleop
ros2 run mecanumbot_teleop mecanumbot_keyboard

# joystick teleop (also starts joy_node on /dev/input/js0)
ros2 launch mecanumbot_teleop launch_joy_teleop.launch.py

# LED GUI
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py
```

The LED GUI needs `mecanumbot_led`'s service node running on the robot, otherwise
the status bar reports the services as unavailable.

## Repository structure

| Path | Function |
| --- | --- |
| `mecanumbot_teleop/` | Keyboard and joystick teleoperation nodes plus the joy launch file. |
| `mecanumbot_ledgui/` | LED GUI node, its launch file, and the saved-preset JSON. |
| `LICENSE` | Apache-2.0. |

See each package's own README for the full interface tables and key/button mappings.
