import os

import yaml
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory("navigation"), "config", "navigation.yaml")

with open(nav_config, "r") as f:
    params = yaml.safe_load(f)

print(params)

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="navigation", executable="navigation", output="screen"),
            Node(package="navigation", executable="rover", output="screen"),

            # Node(package="navigation", executable="navigation", output="screen", parameters=[comm_params]),
            # Node(package="navigation", executable="rover", output="screen", parameters=[comm_params]),
        ]
    )