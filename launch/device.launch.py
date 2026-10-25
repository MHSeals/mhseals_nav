"""One launch file for any computer: the enabled parts of its device profile.

ros2 launch mhseals_nav device.launch.py device:=jetson [only:=camera,objects] [overrides:='{...}']

Profiles live in config/devices/<device>.yaml (see mhseals_nav/device_profile.py). Each part is
an include of a launch file in this package, in its own scope, or one ``ros2 run`` process.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.device_profile import launch_arguments, load, select


def parts(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    profile = load(value('device'), value('device_deployment_file'),
                   value('device_startup_file'), value('overrides'))
    actions = []
    for name, spec in select(profile, value('only')):
        if 'run' in spec:
            actions.append(ExecuteProcess(cmd=['ros2', 'run', *spec['run']], name=name,
                                          output='screen'))
            continue
        source = PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mhseals_nav'), 'launch', spec['launch']]))
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            source, launch_arguments=launch_arguments(profile, spec).items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', description='Device profile in config/devices/'),
        DeclareLaunchArgument('only', default_value='',
                              description='Comma-separated parts to start; empty starts all'),
        DeclareLaunchArgument('overrides', default_value='',
                              description='Session layer: JSON in the device profile schema'),
        DeclareLaunchArgument('device_deployment_file', default_value='',
                              description='Deployment layer; empty finds $ROS_WS/config/devices'),
        DeclareLaunchArgument('device_startup_file', default_value='',
                              description='Operator layer; empty finds ~/.config/mhseals/devices'),
        OpaqueFunction(function=parts),
    ])
