import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the deep3r client against the tunnelled server port."""
    config = os.path.join(
        get_package_share_directory("mecanumbot_deep3r"), "config", "deep3r.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "server", default_value="tcp://127.0.0.1:5555",
            description="ZeroMQ endpoint; the local end of the forward tunnel.",
        ),
        DeclareLaunchArgument(
            "client_path", default_value="~/robocam_client.py",
            description="Deployed robocam_client.py, or the directory holding it.",
        ),
        Node(
            namespace="mecanumbot",
            package="mecanumbot_deep3r",
            executable="mecanumbot_deep3r_client_node",
            # Must match the YAML top-level key, which is the name the node
            # registers for itself; naming it after the executable silently
            # drops every parameter in the file.
            name="mecanumbot_deep3r_client_node",
            output="screen",
            parameters=[
                config,
                {"server": LaunchConfiguration("server")},
                {"client_path": LaunchConfiguration("client_path")},
            ],
        ),
    ])
