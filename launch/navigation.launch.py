"""Navigation: where to go. Nav2 and, on request, RTAB-Map mapping.

ros2 launch mhseals_nav navigation.launch.py [preset:=pool|lake] [mapping:=true] [nav2:=false]

nav2     the explicit server set (no docking or route server): its limits are the hardware state's
         characterization.nav2_limits, clipped to config/nav2_ceilings.yaml; preset merges
         config/nav2_presets/<preset>.yaml over params_file. It commands motion (/cmd_vel).
mapping  RTAB-Map on /points and the camera's relays (perception mapping:=true); it publishes /map,
         never TF (the EKFs own it). nav2:=false mapping:=true maps without navigating.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

from mhseals_nav.layers import package_path


def mapping_node(context, sim):
    """RTAB-Map; localization TF stays the EKFs'."""
    camera = f"/{LaunchConfiguration('camera_name').perform(context)}_camera"
    return Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
                parameters=[LaunchConfiguration('rtabmap_params_file').perform(context),
                            {'use_sim_time': sim}],
                remappings=[('/rgb/image', f'{camera}/rgb/image'),
                            ('/depth/image', f'{camera}/depth/image'),
                            ('/rgb/camera_info', f'{camera}/camera_info'),
                            ('/scan_cloud', '/points')])


def nodes(context):
    from mhseals_nav.layers import boolean
    sim = boolean(LaunchConfiguration('sim').perform(context))
    found = navigation_nodes(context) if boolean(LaunchConfiguration('nav2').perform(context)) else []
    if boolean(LaunchConfiguration('mapping').perform(context)):
        found.append(mapping_node(context, sim))
    return found


def navigation_nodes(context):
    """Nav2's servers and their lifecycle manager, from one compiled parameter file."""
    from mhseals_nav.startup_projection import compile_navigation, SERVERS
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    parameters = compile_navigation(
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('hardware_config').perform(context),
        sim=LaunchConfiguration('sim').perform(context),
        preset=LaunchConfiguration('preset').perform(context))
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
        DeclareLaunchArgument('sim', default_value='false', description='Use the simulator clock'),
        DeclareLaunchArgument('nav2', default_value='true', description='Start Nav2 (it commands motion)'),
        DeclareLaunchArgument('mapping', default_value='false', description='Start RTAB-Map mapping'),
        DeclareLaunchArgument('camera_name', default_value='front',
                              description="Camera whose /<camera_name>_camera/* relays RTAB-Map reads"),
        DeclareLaunchArgument('rtabmap_params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'rtabmap.yaml'])),
        DeclareLaunchArgument('preset', default_value='',
                              description='config/nav2_presets/<preset>.yaml merged over params_file, e.g. '
                                          'pool or lake; measured velocities only lower its own, its accelerations win; empty: none'),
        DeclareLaunchArgument('hardware_config', default_value='',
                              description='Hardware state YAML whose characterization.nav2_limits sets '
                                          'the Nav2 limits; empty = $MHSEALS_HARDWARE_CONFIG, else '
                                          '~/.config/mhseals/hardware.yaml (absent: nav2_params.yaml)'),
        OpaqueFunction(function=nodes),
    ])

