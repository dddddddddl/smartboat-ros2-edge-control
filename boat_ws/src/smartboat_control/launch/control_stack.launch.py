from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'smartboat_control'
    package_share = get_package_share_directory(package_name)

    yaw_config = os.path.join(
        package_share,
        'config',
        'yaw_from_mavlink.yaml'
    )

    yaw_from_mavlink_node = Node(
        package=package_name,
        executable='yaw_from_mavlink_node',
        name='yaw_from_mavlink',
        output='screen',
        parameters=[yaw_config],
    )

    return LaunchDescription([
        yaw_from_mavlink_node,
    ])
