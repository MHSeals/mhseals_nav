from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition

from glob import glob
import xacro
import os

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
            'publish_frequency': float(LaunchConfiguration('robot_state_publish_frequency').perform(context))
        }],
    )]

def create_mavros_node(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    fcu_url_override = LaunchConfiguration('fcu_url').perform(context)

    if fcu_url_override != '':
        fcu_url = fcu_url_override
    elif sim:
        fcu_url = 'tcp://127.0.0.1:5762'
    else:
        # ttyACM numbering changes with USB enumeration order; by-id does not.
        # The baud rate is ignored on USB CDC.
        cube = sorted(glob('/dev/serial/by-id/usb-CubePilot_CubeOrange+_*-if00'))
        fcu_url = f'serial://{cube[0] if cube else "/dev/ttyACM0"}:115200'

    return [Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            LaunchConfiguration('mavros_params_file'),
            {
                'use_sim_time': sim,
                'fcu_url': fcu_url,
                'gcs_url': LaunchConfiguration('gcs_url')
            }
        ]
    )]

def create_mavros_setup_node(context):
    # The FCU forgets message intervals when it reboots, so mavros_setup repeats
    # them on reconnect. GPS_RTK (127) is never emitted by a DroneCAN Here4.
    rates = [
        (33, LaunchConfiguration('mavros_odom_rate').perform(context)),   # global_position/local
        (30, LaunchConfiguration('mavros_imu_rate').perform(context)),    # ATTITUDE
        (31, LaunchConfiguration('mavros_imu_rate').perform(context)),    # imu/data
        (24, LaunchConfiguration('mavros_gps_rate').perform(context)),    # gps1/raw
        (27, '10.0'),   # RAW_IMU: compass 1 as imu/mag; Rover's compass runs at 10 Hz
        (1, '2.0'),     # SYS_STATUS: sensor health
        (193, '2.0'),   # EKF_STATUS_REPORT: flags and compass variance
    ]
    return [Node(
        package='mhseals_nav',
        executable='mavros_setup',
        parameters=[{'message_rates': [f'{i}:{hz}' for i, hz in rates]}],
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
        ('robot_state_publish_frequency', '100.0', 'Rate at which robot state is published'),
        ('fcu_url', '', 'Flight control unit connection URL'),
        ('gcs_url', 'udp://@127.0.0.1:14550', 'Ground control service connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS params'),
        ('mavros_odom_rate', '20.0', 'Requested MAVROS odometry rate'),
        ('mavros_velocity_stddev', '0.3', 'Body velocity uncertainty in m/s (FCU mode; GPS mode uses the receiver vel_acc)'),
        ('mavros_velocity_source', 'gps', 'Body velocity from the GPS receiver (gps) or the FCU EKF (fcu)'),
        ('mavros_imu_rate', '20.0', 'Requested MAVROS IMU rate'),
        ('mavros_gps_rate', '5.0', 'Requested MAVROS GPS rate'),
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTABMap params'),
        ('navsat_transform_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']), 'Path to navsat_transform params'),
        ('ekf_local_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_local.yaml']), 'Path to local EKF config'),
        ('ekf_global_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_global.yaml']), 'Path to global EKF config'),
        ('dlio_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']), 'Path to DLIO params'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF/XACRO'),
        ('camera_name', 'front', 'Name of camera')
    ]

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

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
        output="screen",
        parameters=[{"use_mag": False, "publish_tf": False, "use_sim_time": launch_configurations['sim']}],
        remappings=[
            ("imu/data_raw", "/imu/raw"),
            ("imu/data", "/imu/filtered")
        ],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[
            launch_configurations['navsat_transform_config_file'],
            {'use_sim_time': launch_configurations['sim']}
        ],
        remappings=[
            ('imu', '/imu/raw'),  # robot_localization ROS 2 subscription name
            ('imu/data', '/imu/raw'),
            ('gps/fix', '/gps/fix'),
            # Use the global EKF reference so odometry/gps is expressed in map.
            # The global EKF can initialize from /odom/local before the first
            # GPS correction; this is the standard dual-EKF data flow.
            ('odometry/filtered', '/odom/global'),
            ('odometry/gps', '/odom/gps')
        ]
    )
    
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[launch_configurations['ekf_local_config_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ('odometry/filtered', '/odom/local')
        ]
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[launch_configurations['ekf_global_config_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ('odometry/filtered', '/odom/global')
        ]
    )

    ekf_local_node_delayed = TimerAction(
        period=6.0,
        actions=[ekf_local_node]
    )
    
    ekf_global_node_delayed = TimerAction(
        period=6.0,
        actions=[ekf_global_node]
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
            parameters=[{'velocity_stddev': ParameterValue(
                LaunchConfiguration('mavros_velocity_stddev'), value_type=float),
                'velocity_source': LaunchConfiguration('mavros_velocity_source')}],
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

    # Everything that talks to the flight controller. The EKFs only need the
    # topics, so a simulator can publish them and run without ArduPilot.
    flight_controller = GroupAction(
        condition=IfCondition(launch_configurations['fcu']),
        actions=[mavros_node, *mavros_topic_relays, set_mavros_message_rate])

    return LaunchDescription(
        declare_arguments + [
            # dlio_map_node,
            # dlio_odom_node,
            # rtabmap_odom_node,
            flight_controller,
            imu_filter_node,
            navsat_transform_node,
            ekf_local_node_delayed,
            ekf_global_node_delayed,
            robot_state_publisher_node,
        ]
    )
