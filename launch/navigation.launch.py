"""Explicit boat server set: no implicit docking, route server or SLAM."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare('mhseals_nav')
    rewrites = {
        f'{name}.{name}.ros__parameters.obstacle_layer.enabled':
            LaunchConfiguration('use_lidar')
        for name in ('local_costmap', 'global_costmap')}
    for key, filename in (
            ('default_nav_to_pose_bt_xml', 'nav_to_pose.xml'),
            ('default_nav_through_poses_bt_xml', 'nav_replan_recov.xml')):
        rewrites[f'bt_navigator.ros__parameters.{key}'] = PathJoinSubstitution(
            [share, 'behavior_trees', filename])
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'), root_key='',
        param_rewrites=rewrites,
        convert_types=True)
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    clock = ParameterValue(LaunchConfiguration('sim'), value_type=bool)
    nodes = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', 'cmd_vel')]
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[params,
                        {'use_sim_time': clock, 'enable_stamped_cmd_vel': False}],
            remappings=remaps, output='screen'))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'use_sim_time': clock,
                     'node_names': [name for _, name in servers]}]))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        LogInfo(msg='LIDAR DISABLED: no range-obstacle avoidance; visual supervision required.',
                condition=UnlessCondition(LaunchConfiguration('use_lidar'))),
    ] + nodes)
