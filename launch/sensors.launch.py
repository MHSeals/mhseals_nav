from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = FindPackageShare('mhseals_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    velodyne_ip = LaunchConfiguration('velodyne_ip', default='192.168.1.201')
    zed_camera_name = LaunchConfiguration('zed_camera_name', default='zed2i')

    zed_config_file = LaunchConfiguration('zed_config_file')
    virtual_config_file = LaunchConfiguration('virtual_config_file')
    custom_mono_file = LaunchConfiguration('custom_mono_file')
    custom_stereo_file = LaunchConfiguration('custom_stereo_file')
    custom_object_detection_file = LaunchConfiguration('custom_object_detection_file')
    ffmpeg_file = LaunchConfiguration('ffmpeg_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock if true'
    )
    zed_config_arg = DeclareLaunchArgument(
        'zed_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'zed2i.yaml']),
        description='Path to the main ZED config file'
    )
    virtual_config_arg = DeclareLaunchArgument(
        'virtual_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'virtual.yaml']),
        description='Path to virtual ZED config'
    )
    custom_mono_arg = DeclareLaunchArgument(
        'custom_mono_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'custom_mono.yaml'])
    )
    custom_stereo_arg = DeclareLaunchArgument(
        'custom_stereo_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'custom_stereo.yaml'])
    )
    custom_object_detection_arg = DeclareLaunchArgument(
        'custom_object_detection_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'custom_object_detection.yaml'])
    )
    ffmpeg_arg = DeclareLaunchArgument(
        'ffmpeg_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'ffmpeg.yaml'])
    )

    zed_node = Node(
        package='zed_ros2_wrapper',
        executable='zed_wrapper_node',
        name='zed_camera',
        output='screen',
        parameters=[
            zed_config_file,
            virtual_config_file,
            custom_mono_file,
            custom_stereo_file,
            custom_object_detection_file,
            ffmpeg_file,
            {'camera_name': zed_camera_name, 'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/zed2i/zed_node/rgb/image_rect_color', '/camera/color/image_raw'),
            ('/zed2i/zed_node/depth/depth_registered', '/camera/depth/image_rect_raw'),
            ('/zed2i/zed_node/imu/data', '/imu/data'),
            ('/zed2i/zed_node/odom', '/odom')
        ],
        condition=UnlessCondition(use_sim_time)
    )

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        parameters=[{'frame_id': 'velodyne', 'model': 'VLP16', 'device_ip': velodyne_ip}],
        condition=UnlessCondition(use_sim_time)
    )

    velodyne_pointcloud = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        name='velodyne_convert',
        output='screen',
        remappings=[('velodyne_points', '/points')],
        condition=UnlessCondition(use_sim_time)
    )

    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        parameters=[
            {'ROS_IP': '192.168.1.167'},
            {'ROS_TCP_PORT': 10000}
        ],
        condition=IfCondition(use_sim_time)
    )

    return LaunchDescription([
        use_sim_time_arg,
        zed_config_arg,
        virtual_config_arg,
        custom_mono_arg,
        custom_stereo_arg,
        custom_object_detection_arg,
        ffmpeg_arg,
        zed_node,
        velodyne_driver,
        velodyne_pointcloud,
        ros_tcp_endpoint
    ])