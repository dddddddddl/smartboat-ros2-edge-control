from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('smartboat_navigation'),
        'config',
        'avoidance_executor.yaml',
    ])
    return LaunchDescription([
        Node(
            package='smartboat_navigation',
            executable='avoidance_executor_node',
            name='avoidance_executor',
            output='screen',
            parameters=[config],
        ),
    ])
