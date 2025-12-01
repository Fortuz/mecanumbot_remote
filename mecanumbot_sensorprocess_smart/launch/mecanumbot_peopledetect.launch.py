import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

yaml_file = os.path.join(
        get_package_share_directory('mecanumbot_sensorprocess_smart'),
        'config',
        'lidar_peopledetect_config.yaml'
    )
print(yaml_file)
def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace="mecanumbot",
            package="mecanumbot_sensorprocess_smart",
            executable="mecanumbot_lidar_detect_people",
            name="mecanumbot_lidar_detect_people",  # must match YAML top-level key
            output="screen",
            parameters=[yaml_file]
)
    ])