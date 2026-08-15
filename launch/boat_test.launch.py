"""Minimal MAVROS measurement stack for dock/boat characterization tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch only the FCU and request the streams needed by boat_test."""
    nav_share = FindPackageShare('mhseals_nav')
    fcu_url = LaunchConfiguration('fcu_url')
    mavros_params = LaunchConfiguration('mavros_params_file')

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[mavros_params, {
            'fcu_url': fcu_url,
            'gcs_url': LaunchConfiguration('gcs_url'),
            'use_sim_time': False,
        }],
        remappings=[
            ('/mavros/local_position/odom', '/odom/mavros'),
            ('/mavros/imu/data', '/imu/raw'),
            ('/mavros/global_position/global', '/gps/fix'),
        ],
    )

    # MAVLink LOCAL_POSITION_NED, HIGHRES_IMU, and GPS_RAW_INT.
    rates = ((32, 50.0), (105, 50.0), (24, 10.0))
    rate_requests = [
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/mavros/set_message_interval',
                'mavros_msgs/srv/MessageInterval',
                f'{{message_id: {message_id}, message_rate: {rate}}}',
            ],
            output='screen',
        )
        for message_id, rate in rates
    ]

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@127.0.0.1:14550'),
        DeclareLaunchArgument(
            'mavros_params_file',
            default_value=PathJoinSubstitution(
                [nav_share, 'config', 'mavros.yaml'])),
        mavros,
        TimerAction(period=5.0, actions=rate_requests),
    ])
