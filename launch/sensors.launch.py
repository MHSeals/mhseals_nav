from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.lidar_configuration import resolve_lidar

# Large-data transport for the lidar pair only; see config/fastdds_large_data.xml.
LARGE_DATA_PROFILE = PathJoinSubstitution(
    [FindPackageShare('mhseals_nav'), 'config', 'fastdds_large_data.xml'])
large_data_env = {'FASTRTPS_DEFAULT_PROFILES_FILE': LARGE_DATA_PROFILE,
                  'FASTDDS_DEFAULT_PROFILES_FILE': LARGE_DATA_PROFILE}

def launch_boolean(value):
    """ROS launch boolean vocabulary, validated before constructing nodes."""
    normalized = value.strip().lower()
    if normalized not in ('true', '1', 'false', '0'):
        raise ValueError(f'Invalid sim boolean: {value!r}')
    return normalized in ('true', '1')


def sensor_nodes(context):
    sim = launch_boolean(LaunchConfiguration('sim').perform(context))
    ignored = LaunchConfiguration('sensors_ignore').perform(context).split(',')
    launch_configurations = {name: LaunchConfiguration(name) for name in (
        'ros_ip', 'ros_port')}
    if sim:
        return [Node(package='ros_tcp_endpoint', executable='default_server_endpoint',
                     output='screen', parameters=[{
                         'ROS_IP': launch_configurations['ros_ip'],
                         'ROS_TCP_PORT': launch_configurations['ros_port']}])]
    if 'lidar' in [name.strip() for name in ignored]:
        return []
    selected = LaunchConfiguration('lidar_params_file').perform(context)
    deployment = LaunchConfiguration('lidar_deployment_file').perform(context)
    aliases = {name: LaunchConfiguration(name).perform(context)
               for name in ('velodyne_ip', 'velodyne_port')}
    lidar = resolve_lidar(selected, deployment, aliases, required_frame=
                          LaunchConfiguration('lidar_required_frame').perform(context))
    # Validate the complete pair before constructing either physical node.
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        # The lidar pair uses the large-data transport; TF and odometry stay on
        # the default transport. See config/fastdds_large_data.xml.
        additional_env=large_data_env,
        parameters=[lidar['driver']]
    )

    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform',
        output='screen',
        additional_env=large_data_env,
        parameters=[lidar['transform']],
        remappings=[('velodyne_points', '/points')]
    )

    return [velodyne_driver_node, velodyne_pointcloud_node,
            Node(package='mhseals_nav', executable='lidar_relay', output='screen')]


def generate_launch_description():
    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('lidar_params_file', '', 'Optional selected lidar YAML over package/deployment defaults'),
        ('lidar_deployment_file', '', 'Optional deployment lidar YAML layer'),
        ('lidar_required_frame', '', 'Internal full-robot fixed frame contract'),
        ('velodyne_ip', '', 'Explicit device IP override; empty inherits YAML'),
        ('velodyne_port', '', 'Explicit UDP port override; empty inherits YAML'),
        ('ros_ip', '127.0.0.1', 'IP address of the Unity ROS TCP Endpoint'),
        ('ros_port', '6767', 'Port of the Unity ROS TCP Endpoint'),
        ('sensors_ignore', '', 'Comma-separated list of sensors to ignore, e.g. "lidar"'),
    ]
    
    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    return LaunchDescription(declare_arguments + [OpaqueFunction(function=sensor_nodes)])
