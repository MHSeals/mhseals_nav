"""Explicit boat server set: no implicit docking, route server or SLAM."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    nodes = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', LaunchConfiguration('cmd_vel_topic'))]
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': clock, 'enable_stamped_cmd_vel': False}],
            remappings=remaps, output='screen'))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'use_sim_time': clock,
                     'node_names': [name for _, name in servers]}]))
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/nav/cmd_vel'),
    ] + nodes)
