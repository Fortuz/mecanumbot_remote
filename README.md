# mecanumbot_remote

ROS 2 packages for the parts of mecanumbot that run **off** the robot: the
operator PC that drives it, and the simulator that stands in for it. Nothing
here is ever started on the Orin — that is the line the repository draws.

## Packages in this repository

| Package | Type | Runs on | Purpose |
| --- | --- | --- | --- |
| `mecanumbot_teleop` | Python (`ament_python`) | operator PC | Keyboard teleoperation, publishing `/cmd_vel` and `/cmd_accessory_pos`. Joystick control moved onto the robot as `mecanumbot_joy`. |
| `mecanumbot_ledgui` | Python (`ament_python`) | operator PC | Tkinter desktop GUI that drives the robot LEDs through the `set_led_status` / `get_led_status` services. |
| `mecanumbot_sim` | Python (`ament_python`) | development machine | The simulator. Two interchangeable backends (MuJoCo, Gazebo Sim) presenting exactly the interface the real `mecanumbot_io_node` presents, so perception and behaviour run unchanged; plus YAML scenarios with dead-reckoned actors, a Nav2 shim, and the detection/behaviour evaluators. Holds the largest block of real unit tests in the workspace. |

`mecanumbot_sim` came from the `mecanumbot` repository, which is the onboard
Jetson stack. It never belonged there: a simulator is the one thing that by
definition does not run on the robot, and keeping it next to the serial driver
made the onboard repo look like it shipped MuJoCo to the Orin. Here it sits with
the other packages a developer runs on their own machine.

## Executables

| Executable | Package | ROS node name | Interfaces |
| --- | --- | --- | --- |
| `mecanumbot_keyboard` | `mecanumbot_teleop` | `mecanumbot_keyboard` | Publishes `/cmd_vel`, `/cmd_accessory_pos`. |
| `mecanumbot_ledgui` | `mecanumbot_ledgui` | `mecanumbot_ledgui_client` | Service client for `set_led_status`, `get_led_status`. |
| `mecanumbot_sim_mujoco_io_node` | `mecanumbot_sim` | `mecanumbot_sim_io_node` | MuJoCo backend. Publishes `opencr_state`, `scan`; subscribes `cmd_vel`, `cmd_accessory_pos`. |
| `mecanumbot_sim_gz_io_node` | `mecanumbot_sim` | `mecanumbot_sim_io_node` | Gazebo Sim backend, same interface. Only one backend runs at a time. |
| `mecanumbot_sim_oracle_subject_node` | `mecanumbot_sim` | same | Ground-truth `subject_pose`, standing in for the detector. |
| `mecanumbot_sim_nav_shim_node` | `mecanumbot_sim` | same | Serves `navigate_to_pose` / `navigate_through_poses` so trees run without Nav2. |
| `mecanumbot_sim_visualization_node` | `mecanumbot_sim` | same | RViz markers for actors, props, detections, verdicts. |
| `mecanumbot_sim_detector_debug_node` | `mecanumbot_sim` | same | Detection-to-truth association markers. |
| `mecanumbot_sim_behavior_evaluator_node` | `mecanumbot_sim` | same | Publishes `/sim/behavior_evaluation`. |
| `mecanumbot_sim_detection_evaluator_node` | `mecanumbot_sim` | same | Publishes `/sim/detection_evaluation`. |

No nodes are implemented at the repository root — it only groups the three
packages.

## The simulation entry point is in another repository

`mecanumbot_sim` holds every node, model, world, scenario and evaluator, but the
launch file that starts them, `sim.launch.py`, stays in `mecanumbot_bringup` with
all the other launch orchestration. So a simulated run needs both repositories
checked out:

```bash
ros2 launch mecanumbot_bringup sim.launch.py                      # MuJoCo, drivers only
ros2 launch mecanumbot_bringup sim.launch.py mode:=perception scenario:=human_near_wall
ros2 launch mecanumbot_bringup sim.launch.py mode:=behaviour condition:=Doglike
ros2 launch mecanumbot_bringup sim.launch.py backend:=gazebo      # Gazebo instead of MuJoCo
```

`mecanumbot_sim/launch/gz_backend.launch.py` is a fragment that `sim.launch.py`
includes, not something to launch directly.

## Off-board compute is no longer here

`mecanumbot_deep3r` — the robot's client for the CUT3R reconstruction server —
moved out to [`mecanumbot_server`](https://github.com/Fortuz/mecanumbot_server).
It ran *on* the robot, which made it the one package here that contradicted the
rule this repository now keeps. Anything else that offloads work to the cluster
belongs in `mecanumbot_server` too.

## Dependencies

All three packages depend on `rclpy`. `mecanumbot_teleop` and `mecanumbot_ledgui`
also need `mecanumbot_msgs`; `mecanumbot_teleop` uses `geometry_msgs`, and
`mecanumbot_ledgui` needs a working Tkinter install (`python3-tk`) and a display.

`mecanumbot_sim` needs the `mujoco` pip package for the MuJoCo backend and
`ros-humble-ros-gz` for the Gazebo one, neither declared in its `package.xml`
(the same under-declaration the rest of the workspace has — see the root
`CLAUDE.md`). Install what the backend you actually use needs:

```bash
pip3 install mujoco                     # MuJoCo backend
sudo apt install ros-humble-ros-gz      # Gazebo backend
```

## Build

```bash
cd ~/Documents/mecanumbot_ws
colcon build --symlink-install --packages-select mecanumbot_teleop mecanumbot_ledgui mecanumbot_sim
source install/setup.bash
```

## Quick run examples

```bash
# keyboard teleop
ros2 run mecanumbot_teleop mecanumbot_keyboard

# LED GUI
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py

# simulation (see above for the full argument matrix)
ros2 launch mecanumbot_bringup sim.launch.py
```

The LED GUI needs `mecanumbot_led`'s service node running on the robot, otherwise
the status bar reports the services as unavailable.

Joystick control is no longer run from here. It is `mecanumbot_joy` on the robot,
configured through YAML profiles in `mecanumbot_description/config/joystick/` and
editable from the robot's web GUI at `http://<robot>:8080/joystick`.

## Tests

`mecanumbot_sim` carries the largest block of real unit tests in the workspace —
four files over the scenarios, the gait figure and both evaluators, all pure
Python and runnable without a ROS graph, which makes them the fastest feedback
loop available:

```bash
python3 -m pytest mecanumbot_sim/test/test_sim_scenarios.py -v
```

See the root `CLAUDE.md` for why `colcon test` and `test_flake8` are red on this
machine regardless of the code.

## Repository structure

| Path | Function |
| --- | --- |
| `mecanumbot_teleop/` | Keyboard teleoperation node. |
| `mecanumbot_ledgui/` | LED GUI node, its launch file, and the saved-preset JSON. |
| `mecanumbot_sim/` | Simulator: both backends, the sim nodes, MJCF, Gazebo world and models, scenarios, and the unit tests. |
| `LICENSE` | Apache-2.0. |

See each package's own README for the full interface tables and key/button
mappings; `mecanumbot_sim/README.md` is the long one — read it before changing
anything in the simulator.
