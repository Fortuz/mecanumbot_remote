# mecanumbot_ledgui

ROS 2 desktop GUI package for controlling the robot LEDs through service calls.

The GUI is the operator-side counterpart of `mecanumbot_led`, which runs on the robot
and forwards the values over serial to the Arduino Nano LED controller. Without that
node running, the GUI starts but reports the services as unavailable.

Executable: `mecanumbot_ledgui` — ROS node name `mecanumbot_ledgui_client`.

## Node: `mecanumbot_ledgui_client`

### Service clients

| Service          | Type                               | Used by                                                      |
| ---------------- | ---------------------------------- | ------------------------------------------------------------ |
| `set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | **Send to Robot** — pushes the four corner mode/color pairs. |
| `get_led_status` | `mecanumbot_msgs/srv/GetLedStatus` | **Get from Robot** — reads current state into the table.     |

Both are called from worker threads via `call_async`, so the Tk event loop stays
responsive. `Get from Robot` also runs once at startup to populate the table.

### GUI layout

| Area                           | Function                                                                  |
| ------------------------------ | ------------------------------------------------------------------------- |
| Status bar                     | Reports whether both services are reachable; **Re-check** re-probes them. |
| Current Configuration          | One row per corner (FL, FR, BL, BR) with a color and a mode dropdown.     |
| Saved Configurations           | List of named presets; clicking one loads it into the table.              |
| Save / Delete                  | Store the current table under a name, or remove the selected preset.      |
| Get from Robot / Send to Robot | Read the robot's current LED state, or apply the table to the robot.      |

### Behavior

- Presets are read from and written to `configs/led_configs.json` in the package's
  **share** directory (resolved with `get_package_share_directory`), not the source
  tree — build with `--symlink-install` if you want saved presets to persist in the
  repository.
- Saving under an existing name asks before overwriting.
- The service request order is `fl`, `fr`, `br`, `bl`, matching `SetLedStatus.srv`.

## LED value reference

The dropdowns expose the values below. Note the GUI only offers the first four modes;
the LED firmware also supports `FAST_BLINK` (5) and `SLOW_BLINK` (6), which are
documented in the `mecanumbot_led` README and are reachable only by calling the
service directly.

| Mode         | Value |
| ------------ | ----- |
| `WAVE_RIGHT` | `1`   |
| `WAVE_LEFT`  | `2`   |
| `PULSE`      | `3`   |
| `SOLID`      | `4`   |

| Color    | Value |
| -------- | ----- |
| `BLACK`  | `0`   |
| `WHITE`  | `1`   |
| `GREEN`  | `2`   |
| `RED`    | `3`   |
| `BLUE`   | `4`   |
| `CYAN`   | `5`   |
| `PINK`   | `6`   |
| `YELLOW` | `7`   |

## File functions

| File or folder                         | Function                                               |
| -------------------------------------- | ------------------------------------------------------ |
| mecanumbot_ledgui/mecanumbot_ledgui.py | GUI application and ROS 2 service client logic.        |
| launch/mecanumbot_ledgui.launch.py     | Launches the GUI node executable.                      |
| configs/led_configs.json               | Persistent user-saved LED presets.                     |
| setup.py                               | Installs launch/config assets and console entry point. |

## Run

```bash
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py
```

Requires Tkinter (`python3-tk`) and a display.
