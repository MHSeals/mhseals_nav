from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    fcu_url = LaunchConfiguration('fcu_url')
    mavros_params_file = LaunchConfiguration('mavros_params_file')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    dlio_yaml_file = LaunchConfiguration('dlio_yaml_file')
    dlio_params_file = LaunchConfiguration('dlio_params_file')
    urdf_file = LaunchConfiguration('urdf_file')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    zed_camera_name = LaunchConfiguration('zed_camera_name')
    zed_config_file = LaunchConfiguration('zed_config_file')
    virtual_config = LaunchConfiguration('virtual_config')
    common_stereo_file = LaunchConfiguration('common_stereo_file')
    common_mono_file = LaunchConfiguration('common_mono_file')
    object_detection_file = LaunchConfiguration('object_detection_file')
    custom_object_detection_file = LaunchConfiguration('custom_object_detection_file')
    ffmpeg_file = LaunchConfiguration('ffmpeg_file')

    velodyne_ip = LaunchConfiguration('velodyne_ip')
    velodyne_port = LaunchConfiguration('velodyne_port')

    ros_ip = LaunchConfiguration('ros_ip')
    ros_port = LaunchConfiguration('ros_port')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='tcp://127.0.0.1:5762', description='MAVLink connection URL'
    )
    mavros_params_arg = DeclareLaunchArgument(
        'mavros_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'mavros.yaml']),
        description='Path to MAVROS parameters file'
    )
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'ekf.yaml']),
        description='Path to EKF parameters file'
    )
    dlio_yaml_arg = DeclareLaunchArgument(
        'dlio_yaml_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'dlio.yaml']),
        description='Path to DLIO main YAML'
    )
    dlio_params_arg = DeclareLaunchArgument(
        'dlio_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'dlio_params.yaml']),
        description='Path to DLIO parameters YAML'
    )
    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'description', 'omni_boat', 'omni_boat.xacro']),
        description='Path to robot URDF file'
    )
    rtabmap_params_arg = DeclareLaunchArgument(
        'rtabmap_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'rtabmap.yaml']),
        description='Path to RTAB-Map parameters file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'nav2_params.yaml']),
        description='Path to Nav2 parameters file'
    )

    # Sensor arguments
    zed_camera_name_arg = DeclareLaunchArgument(
        'zed_camera_name', default_value='zed2i', description='ZED camera model'
    )
    zed_config_file_arg = DeclareLaunchArgument(
        'zed_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'zed2i.yaml']),
        description='Path to ZED config'
    )
    virtual_config_arg = DeclareLaunchArgument(
        'virtual_config',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'virtual.yaml'])
    )
    common_stereo_file_arg = DeclareLaunchArgument(
        'common_stereo_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'common_stereo.yaml'])
    )
    common_mono_file_arg = DeclareLaunchArgument(
        'common_mono_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'common_mono.yaml'])
    )
    object_detection_file_arg = DeclareLaunchArgument(
        'object_detection_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'object_detection.yaml'])
    )
    custom_object_detection_file_arg = DeclareLaunchArgument(
        'custom_object_detection_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'custom_object_detection.yaml'])
    )
    ffmpeg_file_arg = DeclareLaunchArgument(
        'ffmpeg_file',
        default_value=PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'ffmpeg.yaml'])
    )

    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip', default_value='192.168.1.201', description='IP address of Velodyne LiDAR'
    )
    velodyne_port_arg = DeclareLaunchArgument(
        'velodyne_port', default_value='2368', description='UDP port of Velodyne LiDAR'
    )

    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip', default_value='192.168.1.167', description='IP address of the Unity ROS TCP Endpoint'
    )
    ros_port_arg = DeclareLaunchArgument(
        'ros_port', default_value='10000', description='Port of the Unity ROS TCP Endpoint'
    )

    # Include launches
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'launch', 'sensors.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'velodyne_ip': velodyne_ip,
            'velodyne_port': velodyne_port,
            'zed_camera_name': zed_camera_name,
            'zed_config': zed_config_file,
            'virtual_config': virtual_config,
            'common_mono': common_mono_file,
            'common_stereo': common_stereo_file,
            'custom_object_detection': custom_object_detection_file,
            'object_detection': object_detection_file,
            'ffmpeg': ffmpeg_file,
            'ros_ip': ros_ip,
            'ros_port': ros_port
        }.items()
    )

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'launch', 'odom.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'fcu_url': fcu_url,
            'mavros_params_file': mavros_params_file,
            'rtabmap_params': rtabmap_params_file,
            'ekf_config_file': ekf_config_file,
            'dlio_yaml_file': dlio_yaml_file,
            'dlio_params_file': dlio_params_file,
            'urdf_file': urdf_file

        }.items()
    )

    delayed_odom_launch = TimerAction(
        period=2.0,
        actions=[odom_launch]
    )

    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'launch', 'slam_nav.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rtabmap_params': rtabmap_params_file,
            'nav2_params': nav2_params_file
        }.items()
    )

    delayed_slam_nav_launch = TimerAction(
        period=5.0,
        actions=[slam_nav_launch]
    )

    return LaunchDescription([
        use_sim_time_arg,
        fcu_url_arg,
        mavros_params_arg,
        ekf_config_arg,
        dlio_yaml_arg,
        dlio_params_arg,
        urdf_arg,
        rtabmap_params_arg,
        nav2_params_arg,

        zed_camera_name_arg,
        zed_config_file_arg,
        virtual_config_arg,
        common_stereo_file_arg,
        common_mono_file_arg,
        object_detection_file_arg,
        custom_object_detection_file_arg,
        ffmpeg_file_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        ros_ip_arg,
        ros_port_arg,

        sensors_launch,
        delayed_odom_launch,
        delayed_slam_nav_launch
    ])