from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('smartboat_drivers'),
        'config',
        'sr73f_radar.yaml',
    ])

    return LaunchDescription([
        Node(
            package='smartboat_drivers',
            executable='sr73f_radar_node',
            name='sr73f_radar',
            output='screen',
            parameters=[params],
        ),
    ])
