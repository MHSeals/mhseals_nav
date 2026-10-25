"""Perception on one host: a camera backend, the world-frame tracker and the perception costmap."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BACKENDS = {'zed': 'zed_backend', 'sim': 'sim_backend'}


def nodes(context):
    sim = LaunchConfiguration('sim').perform(context).lower()
    if sim not in ('true', 'false'):
        raise ValueError('sim must be true or false')
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in BACKENDS:
        raise ValueError(f'backend must be one of {sorted(BACKENDS)}')
    clock = {'use_sim_time': sim == 'true'}

    def config(name):
        return PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', name])

    executable = BACKENDS[backend]
    return [
        Node(package='mhseals_nav', executable=executable, output='screen',
             parameters=[config(f'{executable}.yaml'), clock]),
        Node(package='mhseals_nav', executable='object_tracker', output='screen',
             parameters=[config('tracker.yaml'), clock]),
        # /perception/costmap and /perception/costmap_updates. A standalone Jazzy
        # Costmap2DROS never creates a bond, so the manager must not wait for one.
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='costmap',
             namespace='perception', output='screen',
             parameters=[config('perception_costmap.yaml'), clock]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_perception', namespace='perception', output='screen',
             parameters=[{'autostart': True, 'node_names': ['costmap'],
                          'bond_timeout': 0.0, **clock}]),
        # /perception/nearest_obstacle for the velocity controller's drift observer.
        Node(package='mhseals_nav', executable='nearest_obstacle', output='screen',
             parameters=[config('nearest_obstacle.yaml'), clock]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='Use /clock'),
        DeclareLaunchArgument('backend', default_value='sim',
                              description='Camera backend: zed or sim'),
        OpaqueFunction(function=nodes),
    ])
