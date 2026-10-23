from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Large-data transport for the lidar pair only; see config/fastdds_large_data.xml.
LARGE_DATA_PROFILE = PathJoinSubstitution(
    [FindPackageShare('mhseals_nav'), 'config', 'fastdds_large_data.xml'])
large_data_env = {'FASTRTPS_DEFAULT_PROFILES_FILE': LARGE_DATA_PROFILE,
                  'FASTDDS_DEFAULT_PROFILES_FILE': LARGE_DATA_PROFILE}

def sensor_ignore(sensor):
    return UnlessCondition(
        PythonExpression(
            ["'true' == '", LaunchConfiguration('sim'), "' or '", sensor, 
             "' in [s.strip() for s in '", LaunchConfiguration('sensors_ignore'), "'.split(',')]"]
        )
    )

def generate_launch_description():
    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('velodyne_ip', '192.168.0.201', 'IP address of the Velodyne LiDAR'),
        ('velodyne_port', '2368', 'UDP port for Velodyne LiDAR packets'),
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

    # Velodyne driver (skip if "lidar" is in sensors_ignore)
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        # The lidar pair uses the large-data transport; TF and odometry stay on
        # the default transport. See config/fastdds_large_data.xml.
        additional_env=large_data_env,
        parameters=[{
            'model': 'VLP16',
            'frame_id': 'lidar_link',
            'device_ip': launch_configurations['velodyne_ip'],
            'port': launch_configurations['velodyne_port'],
            'rpm': 600.0
        }],
        condition=sensor_ignore('lidar')
    )

    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform',
        output='screen',
        additional_env=large_data_env,
        parameters=[{
            'model': 'VLP16',
            'frame_id': 'lidar_link',
            'calibration': PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'
            ])
        }],
        remappings=[('velodyne_points', '/points')],
        condition=sensor_ignore('lidar')
    )

    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': launch_configurations['ros_ip'],
            'ROS_TCP_PORT': launch_configurations['ros_port']
        }],
        condition=IfCondition(launch_configurations['sim'])
    )

    nodes = [ 
        velodyne_driver_node, 
        velodyne_pointcloud_node, 
        Node(package='mhseals_nav', executable='lidar_relay',
             output='screen', condition=sensor_ignore('lidar')),
        ros_tcp_endpoint
    ]

    return LaunchDescription(declare_arguments + nodes)
