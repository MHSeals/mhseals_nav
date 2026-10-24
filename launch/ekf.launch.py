"""The two EKFs and navsat_transform, launchable on their own.

Kept apart from odom.launch.py so they can restart without touching MAVROS.
Values in ekf_overlay_file (keyed by node name) override the packaged configs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.localization_configuration import compile_localization, simulation_clock
from mhseals_nav.odometry_configuration import ALIASES, from_launch, LAYER_ARGUMENTS


def ekf_nodes(context):
    settings = from_launch(context)
    sim = simulation_clock(LaunchConfiguration('sim').perform(context))
    overlay = LaunchConfiguration('ekf_overlay_file').perform(context)

    bases = {name: LaunchConfiguration(argument).perform(context) for name, argument in (
        ('ekf_local', 'ekf_local_config_file'), ('ekf_global', 'ekf_global_config_file'),
        ('navsat_transform', 'navsat_transform_config_file'))}
    compiled = compile_localization(bases, overlay, sim)

    def node(name, executable, remappings):
        return Node(
            package='robot_localization', executable=executable, name=name, output='screen',
            parameters=[compiled.parameters(name)], remappings=remappings)

    navsat = node('navsat_transform', 'navsat_transform_node', [
        ('imu', '/imu/raw'),  # robot_localization ROS 2 subscription name
        ('imu/data', '/imu/raw'),
        ('gps/fix', '/gps/fix'),
        # The global EKF can initialize from /odom/local before the first GPS
        # correction; this is the standard dual-EKF data flow.
        ('odometry/filtered', '/odom/global'),
        ('odometry/gps', '/odom/gps'),
    ])
    local = node('ekf_local', 'ekf_node',
                 [('odometry/filtered', '/odom/local')])
    global_ = node('ekf_global', 'ekf_node',
                   [('odometry/filtered', '/odom/global')])
    # odom.launch passes a delay so the EKFs wait for MAVROS; alone it is 0.
    delay = settings.delay
    if delay <= 0:
        return [navsat, local, global_]
    return [navsat, TimerAction(period=delay, actions=[local, global_])]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config'])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='')
          for name in (*LAYER_ARGUMENTS, *ALIASES) if name != 'ekf_delay'],
        DeclareLaunchArgument('ekf_startup_context', default_value='standalone'),
        DeclareLaunchArgument('sim', default_value='true', description='Use the simulation clock'),
        DeclareLaunchArgument('ekf_delay', default_value='',
                              description='Seconds to hold the EKFs back (odom.launch sets 6 for MAVROS)'),
        DeclareLaunchArgument('ekf_overlay_file', default_value='',
                              description='Empty inherits optional ~/.config/mhseals/ekf.yaml; '
                                          'an explicit path must exist'),
        DeclareLaunchArgument('navsat_transform_config_file',
                              default_value=PathJoinSubstitution([config, 'navsat_transform.yaml'])),
        DeclareLaunchArgument('ekf_local_config_file',
                              default_value=PathJoinSubstitution([config, 'ekf_local.yaml'])),
        DeclareLaunchArgument('ekf_global_config_file',
                              default_value=PathJoinSubstitution([config, 'ekf_global.yaml'])),
        OpaqueFunction(function=ekf_nodes),
    ])
