from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def package_config(package, filename):
    return PathJoinSubstitution([
        FindPackageShare(package),
        'config',
        filename,
    ])


def generate_launch_description():
    log_level = PythonExpression([
        "'debug' if '",
        LaunchConfiguration('debug'),
        "'.lower() in ('1', 'true', 'yes', 'on') else 'warn'",
    ])
    radar_node = Node(
        package='smartboat_drivers',
        executable='sr73f_radar_node',
        name='sr73f_radar',
        output='screen',
        parameters=[package_config('smartboat_drivers', 'sr73f_radar.yaml')],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('enable_radar')),
        respawn=True,
        respawn_delay=2.0,
    )
    ewasr_node = Node(
        package='smartboat_perception',
        executable='ewasr_fusion_node',
        name='ewasr_fusion',
        output='screen',
        parameters=[package_config('smartboat_perception', 'ewasr_fusion.yaml')],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('enable_ewasr_perception')),
        respawn=True,
        respawn_delay=2.0,
    )
    dashboard_node = Node(
        package='smartboat_perception',
        executable='dashboard_stream_node',
        name='dashboard_stream',
        output='screen',
        parameters=[
            package_config('smartboat_perception', 'dashboard_stream.yaml'),
            {
                'stream_server': ParameterValue(
                    LaunchConfiguration('dashboard_stream_server'),
                    value_type=str,
                ),
                'stream_port': ParameterValue(
                    LaunchConfiguration('dashboard_stream_port'),
                    value_type=int,
                ),
                'stream_suffix': ParameterValue(
                    LaunchConfiguration('dashboard_stream_suffix'),
                    value_type=str,
                ),
                'stream_fps': ParameterValue(
                    LaunchConfiguration('dashboard_stream_fps'),
                    value_type=int,
                ),
                'stream_width': ParameterValue(
                    LaunchConfiguration('dashboard_stream_width'),
                    value_type=int,
                ),
                'stream_height': ParameterValue(
                    LaunchConfiguration('dashboard_stream_height'),
                    value_type=int,
                ),
                'stream_bitrate': ParameterValue(
                    LaunchConfiguration('dashboard_stream_bitrate'),
                    value_type=str,
                ),
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('enable_dashboard_stream')),
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Set maintained edge-node logging to debug.',
        ),
        DeclareLaunchArgument(
            'enable_mavlink',
            default_value='true',
            description='Start the installed MAVLink forwarding helper.',
        ),
        DeclareLaunchArgument(
            'enable_yaw_from_mavlink',
            default_value='true',
            description='Start the ROS 2 MAVLink yaw/ground-speed reader.',
        ),
        DeclareLaunchArgument(
            'enable_radar',
            default_value='true',
            description='Start the self-configuring SR73F SocketCAN node.',
        ),
        DeclareLaunchArgument(
            'enable_ewasr_perception',
            default_value='true',
            description='Start D435i/EWaSR safety perception.',
        ),
        DeclareLaunchArgument(
            'enable_dashboard_stream',
            default_value='true',
            description=(
                'Start dashboard/SRT output independently from safety perception.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_avoidance_supervisor',
            default_value='true',
            description='Start radar/camera avoidance safety assessment.',
        ),
        DeclareLaunchArgument(
            'enable_avoidance_executor',
            default_value='true',
            description='Start production mission/manual avoidance execution.',
        ),
        DeclareLaunchArgument(
            'enable_px4_cmd_bridge',
            default_value='true',
            description=(
                'Start the activation-gated production cmd_vel bridge.'
            ),
        ),
        DeclareLaunchArgument(
            'dashboard_stream_server',
            default_value='111.229.242.175',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_port',
            default_value='10000',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_suffix',
            default_value='',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_fps',
            default_value='15',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_width',
            default_value='1280',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_height',
            default_value='960',
        ),
        DeclareLaunchArgument(
            'dashboard_stream_bitrate',
            default_value='1800k',
        ),
        Node(
            package='smartboat_drivers',
            executable='water_quality_node',
            name='water_quality',
            output='screen',
            parameters=[package_config('smartboat_mission', 'edge_nodes.yaml')],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        ewasr_node,
        dashboard_node,
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([
                    FindPackagePrefix('smartboat_mission'),
                    'lib',
                    'smartboat_mission',
                    'mavlink_autoconnect',
                ]),
                '--log-level',
                log_level,
            ],
            name='mavlink_autoconnect',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_mavlink')),
            respawn=True,
            respawn_delay=5.0,
        ),
        radar_node,
        Node(
            package='smartboat_control',
            executable='yaw_from_mavlink_node',
            name='yaw_from_mavlink',
            output='screen',
            parameters=[
                package_config('smartboat_control', 'yaw_from_mavlink.yaml')
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(
                LaunchConfiguration('enable_yaw_from_mavlink')
            ),
        ),
        Node(
            package='smartboat_navigation',
            executable='avoidance_supervisor_node',
            name='avoidance_supervisor',
            output='screen',
            parameters=[
                package_config(
                    'smartboat_navigation',
                    'avoidance_supervisor.yaml',
                )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(
                LaunchConfiguration('enable_avoidance_supervisor')
            ),
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package='smartboat_navigation',
            executable='avoidance_executor_node',
            name='avoidance_executor',
            output='screen',
            parameters=[
                package_config(
                    'smartboat_navigation',
                    'avoidance_executor.yaml',
                )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(
                LaunchConfiguration('enable_avoidance_executor')
            ),
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package='smartboat_px4_bridge',
            executable='cmd_vel_to_mavlink_node',
            name='cmd_vel_to_mavlink',
            output='screen',
            parameters=[
                package_config(
                    'smartboat_px4_bridge',
                    'cmd_vel_to_mavlink.yaml',
                )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(
                LaunchConfiguration('enable_px4_cmd_bridge')
            ),
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
