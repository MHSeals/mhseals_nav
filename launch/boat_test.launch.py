"""Shared boat-test measurements; the hardware dashboard owns physical tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('mhseals_nav')
    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'odom.launch.py'])),
        launch_arguments={
            'sim': 'false',
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'mavros_params_file': LaunchConfiguration('mavros_params_file'),
        }.items())
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'sensors.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_sensors')),
        launch_arguments={
            'sim': 'false',
            'sensors_ignore': LaunchConfiguration('sensors_ignore'),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@127.0.0.1:14550'),
        DeclareLaunchArgument('mavros_params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'mavros.yaml'])),
        DeclareLaunchArgument('start_sensors', default_value='false',
                             description='Start local sensor drivers'),
        DeclareLaunchArgument('sensors_ignore', default_value='camera',
                             description='Camera normally runs on the Jetson'),
        odom,
        sensors,
    ])
