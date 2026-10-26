"""Perception on one host: a camera backend, the world-frame tracker and the perception costmap."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BACKENDS = {'zed': 'zed_backend', 'sim': 'sim_backend'}
# Sim: /points on the default graph, the camera's body-axes link and its camera_info.
SIM_FUSION = {'points_transport': 'direct', 'camera_axes': 'body',
              'camera_info_topic': '/front_camera/camera_info'}


def fusion_config():
    """config/lidar_fusion.yaml beside this launch directory (source tree or share)."""
    import yaml
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / 'config/lidar_fusion.yaml'
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


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
    fusion = fusion_config()
    tracker = [config('tracker.yaml'), clock]
    fused = []
    if fusion['enabled']:
        # The tracker's only input then: respawn, and the depth term moves into the covariance.
        tracker.append({'observations_topic': fusion['output_topic'], 'meas.depth_term': False})
        fused = [Node(package='mhseals_nav', executable='lidar_fusion', output='screen',
                      respawn=True, respawn_delay=1.0,
                      parameters=[config('lidar_fusion.yaml'), clock,
                                  *([SIM_FUSION] if sim == 'true' else [])])]
    return [
        Node(package='mhseals_nav', executable=executable, output='screen',
             parameters=[config(f'{executable}.yaml'), clock]),
        *fused,
        Node(package='mhseals_nav', executable='object_tracker', output='screen',
             parameters=tracker),
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
