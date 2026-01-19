from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sensor_ignore = lambda sensor: UnlessCondition(
        PythonExpression(
            ["'true' == '", LaunchConfiguration('sim'), "' or '", sensor, 
             "' in [s.strip() for s in '", LaunchConfiguration('sensors_ignore'), "'.split(',')]"]
        )
    )

    mhseals_nav_dir = FindPackageShare('mhseals_nav')
    zed_wrapper_dir = FindPackageShare('zed_wrapper')

    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('velodyne_ip', '192.168.1.201', 'IP address of the Velodyne LiDAR'),
        ('velodyne_port', '2368', 'UDP port for Velodyne LiDAR packets'),
        ('zed_camera_name', 'zed2i', 'ZED camera model'),
        ('ros_ip', '192.168.1.167', 'IP address of the Unity ROS TCP Endpoint'),
        ('ros_port', '10000', 'Port of the Unity ROS TCP Endpoint'),
        ('sensors_ignore', '', 'Comma-separated list of sensors to ignore, e.g. "camera,lidar"'),

        ('zed_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'zed2i.yaml']), 'ZED camera YAML config'),
        ('virtual_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'virtual.yaml']), 'Virtual sensor config'),
        ('common_mono_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'common_mono.yaml']), 'Common mono camera config'),
        ('common_stereo_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'common_stereo.yaml']), 'Common stereo camera config'),
        ('custom_object_detection_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'custom_object_detection.yaml']), 'Custom object detection config'),
        ('custom_object_detection_model_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'best.onnx']), 'Custom object detection model'),
        ('object_detection_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'object_detection.yaml']), 'Object detection YAML config'),
        ('ffmpeg_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ffmpeg.yaml']), 'FFMPEG video config')
    ]
    
    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([zed_wrapper_dir, 'launch', 'zed_camera.launch.py'])
        ),
        launch_arguments={
            'camera_model': launch_configurations['zed_camera_name'],
            'camera_name': 'zed',
            'common_config_path': launch_configurations['common_stereo_config_file'],
            'zed_id_path': launch_configurations['zed_config_file'],
            'ffmpeg_config_path': launch_configurations['ffmpeg_config_file'],
            'object_detection_config_path': launch_configurations['object_detection_config_file'],
            'custom_object_detection_config_path': launch_configurations['custom_object_detection_config_file'],
            'custom_onnx_file': launch_configurations['custom_object_detection_model_file'],
        }.items(),
        condition=sensor_ignore('camera')
    )

    # Velodyne driver (skip if "lidar" is in sensors_ignore)
    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_link',
            'device_ip': launch_configurations['velodyne_ip'],
            'port': launch_configurations['velodyne_port'],
            'rpm': 600.0
        }],
        condition=sensor_ignore('lidar')
    )

    velodyne_pointcloud = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform',
        output='screen',
        parameters=[{
            'model': 'VLP16',
            'frame_id': 'lidar_link',
            'calibration': '/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml'
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

    nodes = [zed_launch, velodyne_driver, velodyne_pointcloud, ros_tcp_endpoint]

    return LaunchDescription(declare_arguments + nodes)