from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'smartboat_control'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'avoidance_pid.yaml'
    )

    avoidance_pid_node = Node(
        package=package_name,
        executable='avoidance_pid_node',
        name='avoidance_pid',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        avoidance_pid_node,
    ])
