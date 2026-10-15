import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'smartboat_px4_bridge'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'cmd_vel_to_mavlink.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='cmd_vel_to_mavlink_node',
            name='cmd_vel_to_mavlink',
            output='screen',
            parameters=[config_file],
        )
    ])
