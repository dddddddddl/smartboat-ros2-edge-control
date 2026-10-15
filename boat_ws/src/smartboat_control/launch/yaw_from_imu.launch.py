from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'smartboat_control'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'yaw_from_imu.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='yaw_from_imu_node',
            name='yaw_from_imu',
            output='screen',
            parameters=[config_file],
        )
    ])
