import os

import yaml
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="visual", executable="camera", output="screen"),
            # Node(package="visual", executable="decomp_test", output="screen"),
        ]
    )
