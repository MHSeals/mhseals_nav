"""The only launch file the Orin runs: camera, perception and the lidar pair."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(name, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mhseals_nav'), 'launch', f'{name}.launch.py'])),
        launch_arguments=arguments.items())


def generate_launch_description():
    return LaunchDescription([
        include('camera'),
        include('objects', backend='zed', sim='false'),
        include('sensors', sim='false'),
    ])
