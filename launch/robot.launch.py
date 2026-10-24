from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from mhseals_nav.navigation_config import persistent_limits_path
from mhseals_nav.odometry_configuration import ALIASES, LAYER_ARGUMENTS
from mhseals_nav.startup_snapshot import snapshot_root, snapshot_scope

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        # General
        ('config_candidate', '', 'Exact owner-prepared startup candidate'),
        ('config_root', snapshot_root(), 'Local immutable startup artifact root'),
        ('config_scope', snapshot_scope(), 'Session identity; reuse for partial restart'),
        ('sim', 'true', 'Use simulation time'),
        ('use_lidar', 'true', 'Enable lidar costmap; false requires supervised operation'),
        ('enable_slam', 'false', 'Enable optional RTABMap lidar-cloud mapping'),

        # MAVROS / FCU
        ('fcu', 'true', 'Start MAVROS; false when the simulator publishes the FCU topics'),
        ('fcu_url', '', 'Flight control unit connection URL'),
        ('gcs_url', 'udp://127.0.0.1:14550', 'Ground control station connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS parameters file'),

        ('mavros_setup_params_file',
         PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros_setup.yaml']),
         'Selected desired message-rate startup parameters'),
        ('mavros_odom_rate', '', 'Explicit odometry rate override; empty uses YAML'),
        ('mavros_imu_rate', '', 'Explicit attitude rate override for IDs 30 and 31'),
        ('mavros_gps_rate', '', 'Explicit GPS rate override; empty uses YAML'),
        ('mavros_rc_rate', '', 'Explicit RC rate override; empty uses YAML'),

        # Localization / Odometry
        ('ekf_local_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_local.yaml']), 'Local EKF config'),
        ('ekf_global_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_global.yaml']), 'Global EKF config'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF file'),

        # SLAM / Navigation
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTAB-Map parameters file'),
        ('nav2_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'nav2_params.yaml']), 'Path to Nav2 parameters file'),

        ('limits_file', persistent_limits_path(),
         'Persistent Nav2 overlay; empty disables overlay'),

        # Sensors
        ('camera_name', 'front', 'ZED camera namespace (see camera.launch.py)'),
        ('lidar_params_file', '', 'Optional selected lidar YAML over package/deployment defaults'),
        ('lidar_deployment_file',
         PathJoinSubstitution([mhseals_nav_dir, 'config', 'lidar_robot.yaml']),
         'Explicit historical full-robot endpoint deployment layer'),
        ('velodyne_ip', '', 'Explicit device IP override; empty inherits YAML'),
        ('velodyne_port', '', 'Explicit UDP port override; empty inherits YAML'),
        ('ros_ip', '127.0.0.1', 'ROS TCP Endpoint IP'),
        ('ros_port', '6767', 'ROS TCP Endpoint port'),
        ('sensors_ignore', '', 'Comma-separated list of sensors to ignore, e.g. "camera,lidar"')
    ]

    launch_args += [(name, '', 'Explicit startup selection; empty inherits YAML')
                    for name in (*LAYER_ARGUMENTS, *ALIASES)]
    launch_args += [
        ('ekf_startup_context', 'included', 'EKF startup delay context'),
        ('ekf_overlay_file', '~/.config/mhseals/ekf.yaml', 'Localization overlay'),
        ('navsat_transform_config_file',
         PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']),
         'Navsat parameters'),
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
        launch_arguments={**launch_configurations, 'lidar_required_frame': 'lidar_link'}.items()
    )

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'odom.launch.py'])
        ]),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'fcu': launch_configurations['fcu'],
            'fcu_url': launch_configurations['fcu_url'],
            'gcs_url': launch_configurations['gcs_url'],
            'mavros_params_file': launch_configurations['mavros_params_file'],
            **{name: launch_configurations[name] for name in (
                'mavros_setup_params_file', 'mavros_odom_rate', 'mavros_imu_rate',
                'mavros_gps_rate', 'mavros_rc_rate', *LAYER_ARGUMENTS, *ALIASES,
                'ekf_startup_context', 'ekf_overlay_file', 'navsat_transform_config_file')},
            'rtabmap_params_file': launch_configurations['rtabmap_params_file'],
            'ekf_local_config_file': launch_configurations['ekf_local_config_file'],
            'ekf_global_config_file': launch_configurations['ekf_global_config_file'],
            'urdf_file': launch_configurations['urdf_file'],
            'camera_name': launch_configurations['camera_name']
        }.items()
    )

    delayed_odom_launch = TimerAction(period=2.0, actions=[odom_launch])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [mhseals_nav_dir, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'config_candidate': launch_configurations['config_candidate'],
            'config_root': launch_configurations['config_root'],
            'config_scope': launch_configurations['config_scope'],
            'params_file': launch_configurations['nav2_params_file'],
            'limits_file': launch_configurations['limits_file'],
            'use_lidar': launch_configurations['use_lidar'],
        }.items())

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [mhseals_nav_dir, 'launch', 'slam.launch.py'])),
        condition=IfCondition(launch_configurations['enable_slam']),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'camera_name': launch_configurations['camera_name'],
            'rtabmap_params_file': launch_configurations['rtabmap_params_file'],
        }.items())

    objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [mhseals_nav_dir, 'launch', 'objects.launch.py'])),
        launch_arguments={
            'sim': launch_configurations['sim'],
            'objects_topic': ['/', launch_configurations['camera_name'],
                              '/zed_node/obj_det/objects'],
        }.items())

    return LaunchDescription(declare_arguments + [
        sensors_launch,
        delayed_odom_launch,
        TimerAction(period=5.0, actions=[navigation_launch]),
        slam_launch,
        objects_launch,
    ])
