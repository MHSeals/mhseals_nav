"""Explicit boat server set: no implicit docking, route server or SLAM."""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

from mhseals_nav.layers import package_path


def navigation_nodes(context):
    from mhseals_nav.startup_projection import compile_navigation, SERVERS
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    parameters = compile_navigation(
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('hardware_config').perform(context),
        sim=LaunchConfiguration('sim').perform(context))
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    with handle:
        yaml.safe_dump(parameters, handle)
    source = handle.name
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda _: os.unlink(source) if os.path.exists(source) else None)]))
    servers = [(package, name) for package, name in SERVERS
               if name in parameters]
    nodes = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
            if executable == 'controller_server':
                # RViz uses /local_plan for the controller's active path;
                # Nav2 calls the same stream /received_global_plan.
                remaps.append(('received_global_plan', 'local_plan'))
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', 'cmd_vel')]
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[source],
            remappings=remaps, output='screen'))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[source]))
    return ([cleanup] if cleanup else []) + nodes


def generate_launch_description():
    share = str(package_path())
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('hardware_config', default_value='',
                              description='Hardware state YAML whose characterization.nav2_limits sets '
                                          'the Nav2 limits; empty = $MHSEALS_HARDWARE_CONFIG, else '
                                          '~/.config/mhseals/hardware.yaml (absent: nav2_params.yaml)'),
        OpaqueFunction(function=navigation_nodes),
    ])

