"""The simulator's link (sim profile only): the Unity ROS-TCP endpoint, which publishes the boat's sensors.

ros2 launch mhseals_nav sim_bridge.launch.py [ros_port:=10000]

It stands in for every driver the boat has (MAVROS, the ZED, the Velodyne), so the sim profile runs
localization with source:=none and perception with camera:=sim lidar:=sim. With config/lidar.yaml
tune.smear, lidar_smear spreads each instantaneous scan over a VLP16 sweep (/points/instant -> /points).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mhseals_nav import endpoint_configuration
from mhseals_nav.layers import config_path, load_strict


def smear_enabled():
    """config/lidar.yaml tune.smear (default off)."""
    path = config_path('lidar.yaml')
    return bool(load_strict(path.read_text(), path)['tune']['smear'])


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    endpoints = endpoint_configuration.resolve_endpoints(
        deployment_file=value('endpoint_deployment_file'), operator_file=value('endpoint_startup_file'),
        aliases={name: value(name) for name in ('ros_ip', 'ros_port')})
    smear = smear_enabled()
    endpoint = Node(package='ros_tcp_endpoint', executable='default_server_endpoint', output='screen',
                    parameters=[endpoints.unity_parameters],
                    remappings=[('/points', '/points/instant')] if smear else [])
    return [endpoint, *([Node(package='mhseals_nav', executable='lidar_smear', output='screen')]
                        if smear else [])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ros_ip', default_value='',
                              description='Endpoint address; empty inherits config/endpoint_startup.yaml'),
        DeclareLaunchArgument('ros_port', default_value='',
                              description='Endpoint port; empty inherits config/endpoint_startup.yaml'),
        *[DeclareLaunchArgument(name, default_value='', description='Endpoint YAML layer')
          for name in endpoint_configuration.LAYER_ARGUMENTS],
        OpaqueFunction(function=nodes),
    ])
