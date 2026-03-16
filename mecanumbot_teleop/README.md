# mecanumbot_teleop

ROS2 teleoperation package for Mecanumbot.

This package provides two teleop entry points:

- `mecanumbot_keyboard`: keyboard-driven teleoperation (TTY-based)
- `mecanumbot_joystick`: gamepad/joystick teleoperation (uses `sensor_msgs/Joy`)

Both teleop nodes publish drive commands to `/cmd_vel` and accessory position commands to `/cmd_accessory_pos`.

---

## 🧩 What this package does

- Reads user input (keyboard or joystick)
- Converts it into:
  - `geometry_msgs/Twist` messages (for driving the mecanum base)
  - `mecanumbot_msgs/AccessMotorCmd` messages (for controlling camera and gripper positions)

It does **not** directly drive the motors; those are handled by other packages on the robot that subscribe to these topics.

---

## ✅ Requirements

- ROS 2 (tested with Humble)
- A running ROS 2 workspace with the following packages built and sourced:
  - `mecanumbot_msgs`
  - `mecanumbot_core` (or whatever node subscribes to `/cmd_vel` and `/cmd_accessory_pos`)
  - `joy` (for joystick support)

---

## 🚀 Installation

From your ROS 2 workspace root (e.g., `~/dev_ws`):

```bash
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

> 🔧 If you are using the `joy` node (joystick), make sure your controller is connected and visible as `/dev/input/js0`.

---

## ▶️ Running keyboard teleop

Start the teleop script and then use keyboard keys to drive and operate accessories.

```bash
ros2 run mecanumbot_teleop mecanumbot_keyboard
```

### Keyboard controls

```
Control Your TurtleBot3 friend - Mecanumbot !
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease linear x velocity
q/e : decrease/increase angular velocity
a/d : decrease/increase linear y velocity

Moving accessories:
        i    
   j    k    l

i/k : cam up/down
j/l : open/close gripper

space key, s : force stop

CTRL-C to quit
```

---

## ▶️ Running joystick teleop

### Option A: via launch file (recommended)

```bash
ros2 launch mecanumbot_teleop launch_joy_teleop.launch.py
```

### Option B: manually (when you want custom `joy_node` settings)

```bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.05
ros2 run mecanumbot_teleop mecanumbot_joystick
```

### Joystick button mapping (Xbox-style controller)

- **A (button 0)**: camera mid/down
- **B (button 1)**: close gripper
- **X (button 2)**: open gripper
- **Y (button 3)**: camera up
- **Back (button 6)**: emergency stop (zero velocities)

The default axis mapping is:
- **Left stick up/down** → linear X (forward/backward)
- **Left stick left/right** → linear Y (strafing)
- **Right stick left/right** → angular Z (rotation)

---

## 📡 Topics

| Topic | Type | Description |
|------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Mecanum base velocity commands |
| `/cmd_accessory_pos` | `mecanumbot_msgs/AccessMotorCmd` | Camera/gripper position commands |
| `/joy` | `sensor_msgs/Joy` | Joystick input (only for joystick teleop) |

---

## ⚙️ Parameters (joystick teleop)

You can override these at runtime using ROS 2 parameter remapping or a YAML file.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `axes.linear_x` | `1` | Axis index for forward/backward control |
| `axes.linear_y` | `0` | Axis index for strafing control |
| `axes.angular` | `3` | Axis index for rotation control |
| `buttons.A` | `0` | Open camera / mid camera (Xbox A button) |
| `buttons.B` | `1` | Close gripper | 
| `buttons.X` | `2` | Open gripper |
| `buttons.Y` | `3` | Camera up |
| `buttons.Back` | `6` | Emergency stop |
| `sensitivity.deadzone` | `0.05` | Deadzone threshold for joystick axes |
| `smoothing.lin_step` | `0.01` | Linear velocity smoothing step |
| `smoothing.ang_step` | `0.05` | Angular velocity smoothing step |
| `max_lin_vel` | `0.234` | Max linear velocity (m/s) |
| `max_ang_vel` | `1.092` | Max angular velocity (rad/s) |

---

## 🛠️ Notes & Troubleshooting

- If you run keyboard teleop from a terminal and it seems unresponsive, make sure the terminal has focus and is not in another mode (e.g., a pager).
- If your joystick is not detected, verify it shows up under `/dev/input/js*` and that `joy_node` has permission to read it.

---

## 📄 License

This package is licensed under Apache 2.0 (see `package.xml`).
