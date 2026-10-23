"""The two EKFs and navsat_transform, launchable on their own.

Kept apart from odom.launch.py so they can restart without touching MAVROS.
Values in ekf_overlay_file (keyed by node name) override the packaged configs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.navigation_config import ekf_overlay


def ekf_nodes(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    overlay = LaunchConfiguration('ekf_overlay_file').perform(context)

    def node(name, executable, config, remappings):
        # Later entries win, so the overlay replaces a whole key such as odom0_config.
        return Node(
            package='robot_localization', executable=executable, name=name, output='screen',
            parameters=[LaunchConfiguration(config).perform(context),
                        ekf_overlay(overlay, name), {'use_sim_time': sim}],
            remappings=remappings)

    navsat = node('navsat_transform', 'navsat_transform_node', 'navsat_transform_config_file', [
        ('imu', '/imu/raw'),  # robot_localization ROS 2 subscription name
        ('imu/data', '/imu/raw'),
        ('gps/fix', '/gps/fix'),
        # The global EKF can initialize from /odom/local before the first GPS
        # correction; this is the standard dual-EKF data flow.
        ('odometry/filtered', '/odom/global'),
        ('odometry/gps', '/odom/gps'),
    ])
    local = node('ekf_local', 'ekf_node', 'ekf_local_config_file',
                 [('odometry/filtered', '/odom/local')])
    global_ = node('ekf_global', 'ekf_node', 'ekf_global_config_file',
                   [('odometry/filtered', '/odom/global')])
    return [navsat, TimerAction(period=6.0, actions=[local, global_])]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config'])
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='Use the simulation clock'),
        DeclareLaunchArgument('ekf_overlay_file', default_value='~/.config/mhseals/ekf.yaml',
                              description='Per-node parameter overlay; ignored when missing'),
        DeclareLaunchArgument('navsat_transform_config_file',
                              default_value=PathJoinSubstitution([config, 'navsat_transform.yaml'])),
        DeclareLaunchArgument('ekf_local_config_file',
                              default_value=PathJoinSubstitution([config, 'ekf_local.yaml'])),
        DeclareLaunchArgument('ekf_global_config_file',
                              default_value=PathJoinSubstitution([config, 'ekf_global.yaml'])),
        OpaqueFunction(function=ekf_nodes),
    ])
