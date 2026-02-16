from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        # General
        ('sim', 'true', 'Use simulation time'),

        # MAVROS / FCU
        ('fcu_url', 'tcp://127.0.0.1:5762', 'Flight control unit connection URL'),
        ('gcs_url', 'udp://127.0.0.1:14550', 'Ground control station connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS parameters file'),

        # Localization / Odometry
        ('ekf_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf.yaml']), 'Path to EKF configuration file'),
        ('dlio_yaml_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']), 'Path to DLIO main YAML'),
        ('dlio_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio_params.yaml']), 'Path to DLIO parameters YAML'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF file'),

        # SLAM / Navigation
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTAB-Map parameters file'),
        ('nav2_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'nav2_params.yaml']), 'Path to Nav2 parameters file'),

        # Sensors
        ('zed_camera_name', 'zed2i', 'ZED camera model'),
        ('camera_name', 'zed', 'ZED camera link name'),
        ('zed_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'zed2i.yaml']), 'Path to ZED config'),
        ('virtual_config', PathJoinSubstitution([mhseals_nav_dir, 'config', 'virtual.yaml']), 'Virtual sensor config'),
        ('common_stereo_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'common_stereo.yaml']), 'Common stereo config'),
        ('common_mono_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'common_mono.yaml']), 'Common mono config'),
        ('object_detection_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'object_detection.yaml']), 'Object detection config'),
        ('custom_object_detection_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'custom_object_detection.yaml']), 'Custom object detection config'),
        ('ffmpeg_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ffmpeg.yaml']), 'FFMPEG config'),
        ('velodyne_ip', '192.168.1.201', 'Velodyne LiDAR IP address'),
        ('velodyne_port', '2368', 'Velodyne UDP port'),
        ('ros_ip', '192.168.1.167', 'ROS TCP Endpoint IP'),
        ('ros_port', '10000', 'ROS TCP Endpoint port'),
        ('sensors_ignore', '', 'Comma-separated list of sensors to ignore, e.g. "camera,lidar"')
    ]

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        lc = LaunchConfiguration(name)
        launch_configurations[name] = lc
        declare_arguments.append(DeclareLaunchArgument(name, default_value=default_value, description=description))

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'sensors.launch.py'])
        ]),
        launch_arguments={k: v for k, v in launch_configurations.items()}.items()
    )

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'odom.launch.py'])
        ]),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'fcu_url': launch_configurations['fcu_url'],
            'gcs_url': launch_configurations['gcs_url'],
            'mavros_params_file': launch_configurations['mavros_params_file'],
            'rtabmap_params': launch_configurations['rtabmap_params_file'],
            'ekf_config_file': launch_configurations['ekf_config_file'],
            'dlio_yaml_file': launch_configurations['dlio_yaml_file'],
            'dlio_params_file': launch_configurations['dlio_params_file'],
            'urdf_file': launch_configurations['urdf_file']
        }.items()
    )

    delayed_odom_launch = TimerAction(period=2.0, actions=[odom_launch])

    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'slam_nav.launch.py'])
        ]),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'rtabmap_params': launch_configurations['rtabmap_params_file'],
            'nav2_params': launch_configurations['nav2_params_file']
        }.items()
    )

    delayed_slam_nav_launch = TimerAction(period=5.0, actions=[slam_nav_launch])

    return LaunchDescription(declare_arguments + [
        sensors_launch,
        delayed_odom_launch,
        delayed_slam_nav_launch
    ])