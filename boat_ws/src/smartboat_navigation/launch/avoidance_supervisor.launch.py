from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('smartboat_navigation'),
        'config',
        'avoidance_supervisor.yaml',
    ])

    return LaunchDescription([
        Node(
            package='smartboat_navigation',
            executable='avoidance_supervisor_node',
            name='avoidance_supervisor',
            output='screen',
            parameters=[params],
        ),
    ])
