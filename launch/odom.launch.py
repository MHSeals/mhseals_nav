from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

import os

from mhseals_nav.robot_geometry import description_xml

from mhseals_nav.endpoint_configuration import (
    from_launch as endpoint_from_launch, LAYER_ARGUMENTS as ENDPOINT_LAYERS, UNSET_GCS)
from mhseals_nav.layers import boolean as endpoint_boolean

from mhseals_nav.odometry_configuration import ALIASES, from_launch, LAYER_ARGUMENTS

def load_urdf(context):
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    _, ext = os.path.splitext(urdf_path.lower())

    if ext == '.xacro':
        # The vessel profile's lidar mount, when it gives one, wins over the xacro's.
        robot_description = description_xml(urdf_path)
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
        # An FCU reboot drops the USB device and MAVROS exits; mavros_setup re-requests rates
        # when the respawned node reports a connection.
        respawn=True,
        respawn_delay=2.0,
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

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'false', 'Run in simulation mode'),
        ('fcu', 'true', 'Start MAVROS and its relays; false when a simulator publishes '
                        '/odom/mavros, /imu/raw and /gps/fix directly'),
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
        ('start_ekf', 'true', 'Start the EKFs and navsat_transform (false when ekf.launch.py runs on its own)'),
        ('ekf_overlay_file', '', 'Empty inherits optional standard overlay; explicit path must exist'),
        ('navsat_transform_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']), 'Path to navsat_transform params'),
        ('ekf_local_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_local.yaml']), 'Path to local EKF config'),
        ('ekf_global_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_global.yaml']), 'Path to global EKF config'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF/XACRO'),
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
        'sim', 'fcu', 'start_ekf',
        'navsat_transform_config_file', 'ekf_local_config_file', 'ekf_global_config_file',
        'ekf_overlay_file', *LAYER_ARGUMENTS, *ALIASES, 'ekf_startup_context')}
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
            # It also republishes /mavros/global_position/raw/fix as /gps/fix in gps_link.
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

    return [flight_controller, ekf, robot_state_publisher_node]
