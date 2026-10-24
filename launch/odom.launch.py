from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import xacro
import os

from mhseals_nav.endpoint_configuration import (
    boolean as endpoint_boolean,
    from_launch as endpoint_from_launch, LAYER_ARGUMENTS as ENDPOINT_LAYERS, UNSET_GCS)

from mhseals_nav.odometry_configuration import ALIASES, from_launch, LAYER_ARGUMENTS

def load_urdf(context):
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    _, ext = os.path.splitext(urdf_path.lower())

    if ext == '.xacro':
        doc = xacro.process_file(urdf_path)
        robot_description = doc.toxml() # type: ignore
    elif ext == '.urdf':
        with open(urdf_path, 'r') as infp:
            robot_description = infp.read()
    else:
        raise RuntimeError(f"Unsupported robot description file type: {ext}")

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('sim'),
            'publish_frequency': from_launch(context).values['robot_state_publish_frequency']
        }],
    )]

def create_mavros_node(context):
    endpoints = endpoint_from_launch(context)
    sim = endpoint_boolean(LaunchConfiguration('sim').perform(context))
    parameters, observation = endpoints.mavros_parameters(sim)

    return [Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            LaunchConfiguration('mavros_params_file'),
            {
                **parameters
            }
        ]
    )]

def create_mavros_setup_node(context):
    from mhseals_nav.mavros_configuration import ALIASES, resolve_setup
    path = LaunchConfiguration('mavros_setup_params_file').perform(context)
    overrides = {name: LaunchConfiguration(name).perform(context) for name in ALIASES}
    parameters = resolve_setup(path, overrides)
    return [Node(
        package='mhseals_nav',
        executable='mavros_setup',
        name='mavros_setup',
        parameters=[path, parameters],
        output='screen'
    )]

def create_rtabmap_odom_node(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    return [Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_params_file, {'use_sim_time': sim}],
        remappings=[
            ("/rgb/image",f"/{camera_name}_camera/rgb/image"),
            ("/depth/image", f"/{camera_name}_camera/depth/image"),
            ("/rgb/camera_info", f"/{camera_name}_camera/camera_info")
        ]
    )]

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('fcu', 'true', 'Start MAVROS and its relays; false when a simulator publishes '
                        '/odom/mavros, /imu/raw and /gps/fix directly'),
        ('use_imu_filter', 'false', 'Start imu_filter_madgwick (/imu/filtered); nothing consumes it'),
        ('robot_state_publish_frequency', '', 'Rate at which robot state is published'),
        ('fcu_url', '', 'Flight control unit connection URL'),
        ('gcs_url', UNSET_GCS, 'Ground control service connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS params'),
        ('mavros_setup_params_file',
         PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros_setup.yaml']),
         'Selected desired message-rate startup parameters'),
        ('mavros_odom_rate', '', 'Requested MAVROS odometry rate'),
        ('mavros_velocity_stddev', '',
         'Body velocity uncertainty in m/s (FCU mode; GPS mode uses the receiver vel_acc)'),
        ('mavros_velocity_source', '',
         'Body velocity from the GPS receiver (gps) or the FCU EKF (fcu)'),
        ('mavros_imu_rate', '', 'Requested MAVROS IMU rate'),
        ('mavros_gps_rate', '', 'Requested MAVROS GPS rate'),
        ('mavros_rc_rate', '', 'Requested MAVROS RC_CHANNELS rate'),
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTABMap params'),
        ('start_ekf', 'true', 'Start the EKFs and navsat_transform (false when ekf.launch.py runs on its own)'),
        ('ekf_overlay_file', '', 'Empty inherits optional standard overlay; explicit path must exist'),
        ('navsat_transform_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']), 'Path to navsat_transform params'),
        ('ekf_local_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_local.yaml']), 'Path to local EKF config'),
        ('ekf_global_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_global.yaml']), 'Path to global EKF config'),
        ('dlio_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']), 'Path to DLIO params'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF/XACRO'),
        ('camera_name', 'front', 'Name of camera')
    ]

    launch_args += [(name, '', 'Explicit startup override; empty inherits YAML')
                    for name in (*LAYER_ARGUMENTS, *ALIASES)
                    if name not in {item[0] for item in launch_args}]
    launch_args.append(('ekf_startup_context', 'included', 'EKF startup delay context'))

    launch_args += [(name, '', 'Selected endpoint startup YAML layer') for name in ENDPOINT_LAYERS]
    launch_args += [(name, '', 'Explicit endpoint alias; empty inherits YAML')
                    for name in ('ros_ip', 'ros_port')]
    launch_args.append(('endpoint_context', 'standalone', 'Historical endpoint defaults context'))

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    return LaunchDescription(declare_arguments + [OpaqueFunction(function=odometry_nodes)])


def odometry_nodes(context):
    settings = from_launch(context)  # fail every selected startup layer before Node construction
    endpoint_from_launch(context)
    endpoint_boolean(LaunchConfiguration('sim').perform(context))
    endpoint_boolean(LaunchConfiguration('fcu').perform(context))
    mhseals_nav_dir = FindPackageShare('mhseals_nav')
    launch_configurations = {name: LaunchConfiguration(name) for name in (
        'sim', 'fcu', 'use_imu_filter', 'start_ekf', 'dlio_params_file',
        'navsat_transform_config_file', 'ekf_local_config_file', 'ekf_global_config_file',
        'ekf_overlay_file', *LAYER_ARGUMENTS, *ALIASES, 'ekf_startup_context')}
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[launch_configurations['dlio_params_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ('pointcloud', '/points'),
            ('imu', '/imu/filtered'),
            ('odom', '/odom/dlio'),
            ('pose', '/dlio/pose'),
            ('path', '/dlio/path'),
            ('kf_pose', '/dlio/keyframes/pose'),
            ('kf_cloud', '/dlio/keyframes/cloud'),
            ('deskewed', '/dlio/deskewed')
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[launch_configurations['dlio_params_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[('keyframes', '/dlio/pointcloud/keyframe')],
    )

    rtabmap_odom_node = OpaqueFunction(function=create_rtabmap_odom_node)

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        condition=IfCondition(launch_configurations['use_imu_filter']),
        output="screen",
        parameters=[{"use_mag": False, "publish_tf": False, "use_sim_time": launch_configurations['sim']}],
        remappings=[
            ("imu/data_raw", "/imu/raw"),
            ("imu/data", "/imu/filtered")
        ],
    )

    mavros_node = OpaqueFunction(function=create_mavros_node)

    # MAVROS plugins are dynamically-created nodes and do not inherit the
    # remappings passed to mavros_node. Relays preserve the stable hardware /
    # simulation topic contract consumed by the rest of the stack.
    mavros_topic_relays = [
        Node(
            package='mhseals_nav',
            executable='mavros_odometry',
            name='mavros_odom_adapter',
            # ArduPilot publishes GLOBAL_POSITION_INT through MAVROS's
            # global_position plugin.  The local_position plugin's odometry
            # topic is advertised on some FCUs but remains silent, which
            # leaves the EKF predicting without fresh translational motion.
            # This source carries ENU velocity despite its base_link child label.
            # Rotate with its own FCU attitude and replace unknown covariance.
            parameters=[settings.adapter_parameters],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='mavros_imu_relay',
            arguments=['/mavros/imu/data', '/imu/raw'],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='mavros_gps_relay',
            arguments=['/mavros/global_position/raw/fix', '/gps/fix'],
            output='screen'
        ),
    ]

    set_mavros_message_rate = OpaqueFunction(function=create_mavros_setup_node)

    robot_state_publisher_node = OpaqueFunction(function=load_urdf)

    # The EKFs and navsat live in ekf.launch.py so they can be restarted alone,
    # with MAVROS left up. start_ekf:=false leaves them to a separate launch.
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mhseals_nav_dir, 'launch', 'ekf.launch.py'])),
        condition=IfCondition(launch_configurations['start_ekf']),
        launch_arguments={
            **{name: launch_configurations[name] for name in (
                'sim', 'navsat_transform_config_file', 'ekf_local_config_file',
                'ekf_global_config_file', 'ekf_overlay_file')},
            **{name: launch_configurations[name] for name in
               (*LAYER_ARGUMENTS, *ALIASES, 'ekf_startup_context')},
        }.items())

    # Everything that talks to the flight controller. The EKFs only need the
    # topics, so a simulator can publish them and run without ArduPilot.
    flight_controller = GroupAction(
        condition=IfCondition(launch_configurations['fcu']),
        actions=[mavros_node, *mavros_topic_relays, set_mavros_message_rate])

    return [flight_controller, imu_filter_node, ekf, robot_state_publisher_node]
