from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')
    zed_wrapper_dir = FindPackageShare('zed_wrapper')

    use_sim_time = LaunchConfiguration('use_sim_time')

    velodyne_ip = LaunchConfiguration('velodyne_ip')
    velodyne_port = LaunchConfiguration('velodyne_port')

    zed_camera_name = LaunchConfiguration('zed_camera_name')

    zed_config_file = LaunchConfiguration('zed_config_file')
    virtual_config_file = LaunchConfiguration('virtual_config_file')
    common_mono_file = LaunchConfiguration('common_mono_file')
    common_stereo_file = LaunchConfiguration('common_stereo_file')
    custom_object_detection_file = LaunchConfiguration('custom_object_detection_file')
    object_detection_file = LaunchConfiguration('object_detection_file')
    ffmpeg_file = LaunchConfiguration('ffmpeg_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip',
        default_value='192.168.1.201',
        description='IP address of the Velodyne LiDAR'
    )

    velodyne_port_arg = DeclareLaunchArgument(
        'velodyne_port',
        default_value='2368',
        description='UDP port for Velodyne LiDAR packets'
    )

    zed_camera_name_arg = DeclareLaunchArgument(
        'zed_camera_name',
        default_value='zed2i',
        description='ZED camera model'
    )

    zed_config_arg = DeclareLaunchArgument(
        'zed_config_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'zed2i.yaml'
        ]),
        description='ZED main configuration file'
    )

    virtual_config_arg = DeclareLaunchArgument(
        'virtual_config_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'virtual.yaml'
        ]),
        description='Virtual ZED configuration'
    )

    common_mono_arg = DeclareLaunchArgument(
        'common_mono_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'common_mono.yaml'
        ])
    )

    common_stereo_arg = DeclareLaunchArgument(
        'common_stereo_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'common_stereo.yaml'
        ])
    )

    custom_object_detection_arg = DeclareLaunchArgument(
        'custom_object_detection_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'custom_object_detection.yaml'
        ])
    )

    object_detection_arg = DeclareLaunchArgument(
        'object_detection_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'object_detection.yaml'
        ])
    )

    ffmpeg_arg = DeclareLaunchArgument(
        'ffmpeg_file',
        default_value=PathJoinSubstitution([
            mhseals_nav_dir, 'config', 'ffmpeg.yaml'
        ])
    )

    # TODO: Add topic remappings for rtabmap
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                zed_wrapper_dir, 'launch', 'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': zed_camera_name,
            'camera_name': 'zed',
            'common_config_path': common_stereo_file,
            'zed_id_path': zed_config_file,
            'ffmpeg_config_path': ffmpeg_file,
            'object_detection_config_path': object_detection_file,
            'custom_object_detection_config_path': custom_object_detection_file,
        }.items(),
        condition=UnlessCondition(use_sim_time)
    )

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_link',
            'device_ip': velodyne_ip,
            'port': velodyne_port,
            'rpm': 600.0
        }],
        condition=UnlessCondition(use_sim_time)
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
        remappings=[
            ('velodyne_points', '/points')
        ],
        condition=UnlessCondition(use_sim_time)
    )

    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '192.168.1.167',
            'ROS_TCP_PORT': 10000
        }],
        condition=IfCondition(use_sim_time)
    )

    return LaunchDescription([
        use_sim_time_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        zed_camera_name_arg,
        zed_config_arg,
        virtual_config_arg,
        common_mono_arg,
        common_stereo_arg,
        custom_object_detection_arg,
        object_detection_arg,
        ffmpeg_arg,
        zed_launch,
        velodyne_driver,
        velodyne_pointcloud,
        ros_tcp_endpoint
    ])