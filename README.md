# mecanumbot_remote

Remote-compute ROS 2 packages for controlling and extending mecanumbot from an external PC.

## Folder functions

| Folder | Function |
|------|----------|
| mecanumbot_teleop | Manual control nodes (keyboard and joystick) publishing cmd_vel and accessory commands. |
| mecanumbot_ledgui | Desktop GUI node for LED control using ROS services. |
| mecanumbot_sensorprocess_smart | LiDAR people detection and tracking node using DR-SPAAM. |

## Node interfaces at repository root

### Publishers

| Topic | Data type | Function |
|-------|-----------|----------|
| None | - | No nodes are implemented in the repository root folder. |

### Subscribers

| Topic | Data type | Processing |
|-------|-----------|------------|
| None | - | No nodes are implemented in the repository root folder. |

### Services handled

| Service | Type | Behavior |
|---------|------|----------|
| None | - | No nodes are implemented in the repository root folder. |

## Quick run examples

```bash
# keyboard teleop
ros2 run mecanumbot_teleop mecanumbot_keyboard

# joystick teleop
ros2 launch mecanumbot_teleop launch_joy_teleop.launch.py

# LED GUI
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py

# people detection (included in mecanumbot_bringup's launch_external launch process - no need to start if that runs)
ros2 launch mecanumbot_sensorprocess_smart mecanumbot_peopledetect.launch.py
```