import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'smartboat_px4_bridge'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'cmd_vel_to_mavlink_shore_test.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_output',
            default_value='false',
            description=(
                'Explicitly enable low-power MAVLink output after completing '
                'the shore-test safety checklist.'
            ),
        ),
        Node(
            package=package_name,
            executable='cmd_vel_to_mavlink_node',
            name='cmd_vel_to_mavlink_shore_test',
            output='screen',
            parameters=[
                config_file,
                {
                    'output_enabled': ParameterValue(
                        LaunchConfiguration('enable_output'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
