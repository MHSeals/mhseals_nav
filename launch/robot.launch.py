from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from mhseals_nav.endpoint_configuration import (
    boolean as endpoint_boolean,
    from_launch as endpoint_from_launch, LAYER_ARGUMENTS as ENDPOINT_LAYERS, UNSET_GCS)
from mhseals_nav.navigation_config import persistent_limits_path
from mhseals_nav.odometry_configuration import ALIASES, LAYER_ARGUMENTS

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        # General
        ('sim', 'true', 'Use simulation time'),
        ('use_lidar', 'true', 'Enable lidar costmap; false requires supervised operation'),
        ('enable_slam', 'false', 'Enable optional RTABMap lidar-cloud mapping'),
        ('command_mux', 'false',
         'A command_mux arbitrates /cmd_vel; starts the stale-costmap watchdog that asks it to hold'),

        # MAVROS / FCU
        ('fcu', 'true', 'Start MAVROS; false when the simulator publishes the FCU topics'),
        ('fcu_url', '', 'Flight control unit connection URL'),
        ('gcs_url', UNSET_GCS, 'Ground control station connection URL'),
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
        ('lidar_deployment_file', '', 'Optional deployment lidar YAML layer'),
        ('velodyne_ip', '', 'Explicit device IP override; empty inherits YAML'),
        ('velodyne_port', '', 'Explicit UDP port override; empty inherits YAML'),
        ('ros_ip', '', 'ROS TCP Endpoint IP'),
        ('ros_port', '', 'ROS TCP Endpoint port'),
        ('sensors_ignore', '', 'Comma-separated list of sensors to ignore, e.g. "camera,lidar"')
    ]

    launch_args += [(name, '', 'Explicit startup selection; empty inherits YAML')
                    for name in (*LAYER_ARGUMENTS, *ALIASES)]
    launch_args += [
        ('ekf_startup_context', 'included', 'EKF startup delay context'),
        ('ekf_overlay_file', '', 'Empty inherits optional standard localization overlay'),
        ('navsat_transform_config_file',
         PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']),
         'Navsat parameters'),
    ]

    launch_args += [(name, '', 'Selected endpoint startup YAML layer') for name in ENDPOINT_LAYERS]
    launch_args.append(('endpoint_context', 'full_robot', 'Endpoint defaults context'))

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        lc = LaunchConfiguration(name)
        launch_configurations[name] = lc
        declare_arguments.append(DeclareLaunchArgument(name, default_value=default_value, description=description))

    # On the boat the Orin runs sensors and perception (device.launch.py device:=jetson).
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'sensors.launch.py'])
        ]),
        condition=IfCondition(launch_configurations['sim']),
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
                *ENDPOINT_LAYERS, 'endpoint_context', 'ros_ip', 'ros_port',
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
            'params_file': launch_configurations['nav2_params_file'],
            'limits_file': launch_configurations['limits_file'],
            'use_lidar': launch_configurations['use_lidar'],
            'command_mux': launch_configurations['command_mux'],
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
        condition=IfCondition(launch_configurations['sim']),
        launch_arguments={'sim': 'true', 'backend': 'sim'}.items())

    return LaunchDescription(declare_arguments + [
        OpaqueFunction(function=endpoint_preflight),
        sensors_launch,
        delayed_odom_launch,
        TimerAction(period=5.0, actions=[navigation_launch]),
        slam_launch,
        objects_launch,
    ])


def endpoint_preflight(context):
    """Validate endpoint closure before any included launch can create a child."""
    endpoint_from_launch(context)
    endpoint_boolean(LaunchConfiguration('sim').perform(context))
    endpoint_boolean(LaunchConfiguration('fcu').perform(context))
    return []
