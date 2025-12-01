from launch import LaunchDescription
from launch_ros.actions import Node
joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    output='screen',
    parameters=[{
        'dev': '/dev/input/js0',  # <-- your joystick device
        'deadzone': 0.05
    }]
)
teleop_node = Node(
    package = 'mecanumbot_teleop',
    executable = 'mecanumbot_joystick',
    name = 'joystick_teleop_node'
)
def generate_launch_description():
    return LaunchDescription([
        joy_node,
        teleop_node
    ])
