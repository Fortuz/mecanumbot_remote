# mecanumbot_ledgui

ROS 2 desktop GUI package for controlling robot LEDs through service calls.

## Node: mecanumbot_ledgui_client

### Behavior

- Creates ROS service clients for set_led_status (mecanumbot_msgs/srv/SetLedStatus) and get_led_status (mecanumbot_msgs/srv/GetLedStatus).
- Provides a Tkinter GUI to edit per-corner mode/color (FL, FR, BL, BR).
- Loads/saves named LED presets from configs/led_configs.json in package share.
- Runs async calls for service requests to keep UI responsive.

## File functions

| File or folder | Function |
|----------------|----------|
| mecanumbot_ledgui/mecanumbot_ledgui.py | GUI application and ROS2 service client logic. |
| launch/mecanumbot_ledgui.launch.py | Launches the GUI node executable. |
| configs/led_configs.json | Persistent user-saved LED configurations. |
| setup.py | Installs launch/config assets and console entry point. |
