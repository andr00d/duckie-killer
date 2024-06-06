import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_publisher',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='yolo_detector',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen'
        ),
        Node(
            package='mode_publisher',
            executable='mode_publisher',
            name='mode_publisher',
            output='screen'
        ),
        Node(
            package='navigator',
            executable='navigator',
            name='navigator',
            output='screen'
        ),
        Node(
            package='guard_follower',
            executable='guard_follower',
            name='guard_follower',
            output='screen'
        ),
    ])

