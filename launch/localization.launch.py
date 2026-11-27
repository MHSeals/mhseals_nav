"""Localization: where the boat is. The robot TF, the two EKFs and navsat_transform, and their pose source.

ros2 launch mhseals_nav localization.launch.py [source:=fcu|none] [sim:=true]

source picks what feeds the EKFs /odom/mavros, /imu/raw and /gps/fix:
  fcu   MAVROS, its relays and the message-rate setup (the boat; with sim:=true, ArduPilot SITL).
  none  nothing here: a simulator publishes them, or MAVROS is already up and only the EKFs and TF restart.
With source fcu the EKFs wait ekf_delay_included (config/odometry_startup.yaml) for MAVROS; else
ekf_delay_standalone. ~/.config/mhseals/ekf.yaml (ekf_overlay_file) overrides the EKF configs per node.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mhseals_nav import endpoint_configuration, mavros_configuration, odometry_configuration
from mhseals_nav.layers import boolean, config_path, package_path
from mhseals_nav.localization_configuration import compile_localization
from mhseals_nav.robot_geometry import description_xml

SOURCES = ('fcu', 'none')


def arguments():
    """(name, default, description) of every argument; empty strings inherit the YAML layers."""
    config = str(config_path())
    found = [
        ('source', 'fcu', 'What feeds the EKFs: fcu (MAVROS) or none (a simulator, or MAVROS already up)'),
        ('sim', 'false', 'Use the simulator clock'),
        ('urdf_file', str(package_path('description', 'blastoise', 'blastoise.xacro')),
         'Robot description (.xacro or .urdf)'),
        ('mavros_params_file', f'{config}/mavros.yaml', 'MAVROS parameters'),
        ('mavros_setup_params_file', f'{config}/mavros_setup.yaml', 'FCU message rates requested on connect'),
        ('ekf_local_config_file', f'{config}/ekf_local.yaml', 'Local EKF (odom -> base_link)'),
        ('ekf_global_config_file', f'{config}/ekf_global.yaml', 'Global EKF (map -> odom)'),
        ('navsat_transform_config_file', f'{config}/navsat_transform.yaml', 'navsat_transform'),
        ('ekf_overlay_file', '', 'Per-node EKF overrides; empty uses ~/.config/mhseals/ekf.yaml if present'),
        ('fcu_url', '', 'FCU connection URL; empty finds the Cube (config/endpoint_startup.yaml)'),
        ('gcs_url', endpoint_configuration.UNSET_GCS, 'Ground control URL; empty disables it'),
        ('endpoint_context', 'standalone', 'Endpoint discovery context in config/endpoint_startup.yaml'),
    ]
    found += [(name, '', 'Override of config/mavros_setup.yaml; empty inherits it')
              for name in mavros_configuration.ALIASES]
    found += [(name, '', 'Override of config/odometry_startup.yaml; empty inherits it')
              for name in (*odometry_configuration.ALIASES, *odometry_configuration.LAYER_ARGUMENTS)]
    found += [(name, '', 'Endpoint YAML layer') for name in endpoint_configuration.LAYER_ARGUMENTS]
    return found


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def odometry(context):
    """The odometry layers; the EKF delay context follows the source (MAVROS needs time to connect)."""
    source = value(context, 'source')
    if source not in SOURCES:
        raise ValueError(f'source must be one of {", ".join(SOURCES)}, not {source!r}')
    return odometry_configuration.resolve_odometry(
        deployment_file=value(context, 'odometry_deployment_file'),
        operator_file=value(context, 'odometry_startup_file'),
        aliases={key: value(context, key) for key in odometry_configuration.ALIASES},
        context='included' if source == 'fcu' else 'standalone')


def fcu_nodes(context, settings, sim):
    """MAVROS, the topic relays and the message-rate setup."""
    endpoints = endpoint_configuration.resolve_endpoints(
        deployment_file=value(context, 'endpoint_deployment_file'),
        operator_file=value(context, 'endpoint_startup_file'),
        aliases={name: value(context, name) for name in ('fcu_url', 'gcs_url')},
        context=value(context, 'endpoint_context'))
    parameters, _ = endpoints.mavros_parameters(sim)
    setup_file = value(context, 'mavros_setup_params_file')
    rates = mavros_configuration.resolve_setup(
        setup_file, {name: value(context, name) for name in mavros_configuration.ALIASES})
    return [
        # An FCU reboot drops the USB device and MAVROS exits; mavros_setup re-requests the rates when
        # the respawned node reconnects.
        Node(package='mavros', executable='mavros_node', output='screen', respawn=True,
             respawn_delay=2.0, parameters=[value(context, 'mavros_params_file'), parameters]),
        # MAVROS plugins are nodes of their own and ignore remappings, so the relays give the stack its
        # stable topics. The adapter reads global_position (ArduPilot's local_position odometry stays
        # silent), rotates by the FCU attitude, fills the covariance and republishes the fix as /gps/fix.
        Node(package='mhseals_nav', executable='mavros_odometry', name='mavros_odom_adapter',
             output='screen', parameters=[settings.adapter_parameters]),
        # /mavros/imu/data -> /imu/raw, the heading held on the gyro while thrust current bends
        # the compass (heading_guard.py). Respawned: every EKF needs /imu/raw.
        Node(package='mhseals_nav', executable='heading_guard', name='heading_guard',
             output='screen', respawn=True, respawn_delay=1.0,
             parameters=[settings.guard_parameters]),
        Node(package='mhseals_nav', executable='mavros_setup', name='mavros_setup', output='screen',
             parameters=[setup_file, rates]),
    ]


def description_node(context, settings, sim):
    """robot_state_publisher: base_link -> every sensor frame, from the xacro (the vessel's lidar mount wins)."""
    path = value(context, 'urdf_file')
    extension = os.path.splitext(path.lower())[1]
    if extension == '.xacro':
        description = description_xml(path)
    elif extension == '.urdf':
        with open(path) as handle:
            description = handle.read()
    else:
        raise ValueError(f'urdf_file: a .xacro or .urdf, not {path}')
    return Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                parameters=[{'robot_description': description, 'use_sim_time': sim,
                             'publish_frequency': settings.values['robot_state_publish_frequency']}])


def ekf_nodes(context, settings, sim):
    """navsat_transform, then the local (odom -> base_link) and global (map -> odom) EKFs."""
    bases = {name: value(context, argument) for name, argument in (
        ('ekf_local', 'ekf_local_config_file'), ('ekf_global', 'ekf_global_config_file'),
        ('navsat_transform', 'navsat_transform_config_file'))}
    compiled = compile_localization(bases, value(context, 'ekf_overlay_file'), sim)

    def node(name, executable, remappings):
        return Node(package='robot_localization', executable=executable, name=name, output='screen',
                    parameters=[compiled.parameters(name)], remappings=remappings)

    navsat = node('navsat_transform', 'navsat_transform_node', [
        ('imu', '/imu/raw'), ('imu/data', '/imu/raw'),   # the ROS 2 subscription name, and the ROS 1 one
        ('gps/fix', '/gps/fix'),
        ('odometry/filtered', '/odom/global'),   # the standard dual-EKF flow: global feeds navsat
        ('odometry/gps', '/odom/gps')])
    filters = [node('ekf_local', 'ekf_node', [('odometry/filtered', '/odom/local')]),
               node('ekf_global', 'ekf_node', [('odometry/filtered', '/odom/global')])]
    if settings.delay <= 0:
        return [navsat, *filters]
    return [navsat, TimerAction(period=settings.delay, actions=filters)]


def nodes(context):
    settings = odometry(context)   # every layer is checked before any node is built
    sim = boolean(value(context, 'sim'))
    source = fcu_nodes(context, settings, sim) if value(context, 'source') == 'fcu' else []
    return [*source, description_node(context, settings, sim), *ekf_nodes(context, settings, sim)]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, default, text in arguments()],
        OpaqueFunction(function=nodes),
    ])
