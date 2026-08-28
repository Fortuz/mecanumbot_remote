# mecanumbot_remote

ROS 2 packages for the parts of mecanumbot that are **not** the robot's own
onboard stack: the operator PC that drives it, and the off-board server that
does work too heavy for the Orin. Both directions cross the same boundary — the
robot's ROS graph on one side, something else on the other — which is why they
live in one repository.

## Packages in this repository

| Package | Type | Runs on | Purpose |
| --- | --- | --- | --- |
| `mecanumbot_teleop` | Python (`ament_python`) | operator PC | Keyboard teleoperation, publishing `/cmd_vel` and `/cmd_accessory_pos`. Joystick control moved onto the robot as `mecanumbot_joy`. |
| `mecanumbot_ledgui` | Python (`ament_python`) | operator PC | Tkinter desktop GUI that drives the robot LEDs through the `set_led_status` / `get_led_status` services. |
| `mecanumbot_deep3r` | Python (`ament_python`) | robot (Jetson) | Off-board 3D reconstruction: streams camera JPEGs to the CUT3R server on the cluster and republishes the returned point cloud. |

`mecanumbot_deep3r` is the odd one out in where it runs — it is a node on the
robot — but it is remote-compute in the sense that matters here: it does no
perception and no control of its own, it moves data across the robot ↔ cluster
boundary and translates what comes back into ROS types. Anything else that
offloads work to the server — a detector, a VLM, a planner — belongs here too,
so that the set of things that stop working when the link is down is a single
directory rather than a memory.

## Executables

| Executable | Package | ROS node name | Interfaces |
| --- | --- | --- | --- |
| `mecanumbot_keyboard` | `mecanumbot_teleop` | `mecanumbot_keyboard` | Publishes `/cmd_vel`, `/cmd_accessory_pos`. |
| `mecanumbot_ledgui` | `mecanumbot_ledgui` | `mecanumbot_ledgui_client` | Service client for `set_led_status`, `get_led_status`. |
| `mecanumbot_deep3r_client_node` | `mecanumbot_deep3r` | `mecanumbot_deep3r_client_node` | Subscribes `camera/image_raw/compressed`; publishes `deep3r/points`, `deep3r/pose`. |

No nodes are implemented at the repository root — it only groups the three packages.

## The server link is not a detail

The server itself is not here. It lives in
[`RoboCamStreamProcessing`](https://github.com/hubaycsenge/RoboCamStreamProcessing)
on the cluster, and `mecanumbot_deep3r` is only the robot's end of the
conversation.

The robot is on the lab WiFi behind NAT; the server is on the cluster. Measured
2026-08-28: **the robot cannot reach the server's address directly.** The node
therefore talks to `tcp://127.0.0.1:<port>` and depends on a tunnel the robot
dials out itself, installed as a systemd unit from
`RoboCamStreamProcessing/link/`. Read that directory's README before debugging
anything here — a node that looks hung is usually a tunnel that is down.

## Dependencies

All three packages depend on `rclpy`. `mecanumbot_teleop` and `mecanumbot_ledgui`
also need `mecanumbot_msgs`; `mecanumbot_teleop` uses `geometry_msgs`, and
`mecanumbot_ledgui` needs a working Tkinter install (`python3-tk`) and a display.

`mecanumbot_deep3r` needs `pyzmq`, which is **not** declared in its `package.xml`
because `rosdep` will not install it here (the same under-declaration the rest of
the workspace has — see the root `CLAUDE.md`). Install it by hand on the robot:

```bash
pip3 install pyzmq
```

The reconstruction model itself is not a dependency of anything in this repo. It
runs on the cluster; nothing here imports torch.

## Build

```bash
cd ~/Documents/mecanumbot_ws
colcon build --symlink-install --packages-select mecanumbot_teleop mecanumbot_ledgui mecanumbot_deep3r
source install/setup.bash
```

## Quick run examples

```bash
# keyboard teleop
ros2 run mecanumbot_teleop mecanumbot_keyboard

# LED GUI
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py

# off-board reconstruction (on the robot, tunnel up first)
ros2 launch mecanumbot_deep3r deep3r.launch.py
```

The LED GUI needs `mecanumbot_led`'s service node running on the robot, otherwise
the status bar reports the services as unavailable.

Joystick control is no longer run from here. It is `mecanumbot_joy` on the robot,
configured through YAML profiles in `mecanumbot_description/config/joystick/` and
editable from the robot's web GUI at `http://<robot>:8080/joystick`.

## Repository structure

| Path | Function |
| --- | --- |
| `mecanumbot_teleop/` | Keyboard teleoperation node. |
| `mecanumbot_ledgui/` | LED GUI node, its launch file, and the saved-preset JSON. |
| `mecanumbot_deep3r/` | CUT3R client node, its launch file, and `config/deep3r.yaml`. |
| `LICENSE` | Apache-2.0. |

See each package's own README for the full interface tables and key/button mappings.
