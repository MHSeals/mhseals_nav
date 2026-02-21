from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition

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
        fcu_url = 'serial:///dev/ttyACM0:57600'

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
        ],
    )]

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('robot_state_publish_frequency', '100.0', 'Rate at which robot state is published'),
        ('fcu_url', '', 'Flight control unit connection URL'),
        ('gcs_url', 'udp://@127.0.0.1:14550', 'Ground control service connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS params'),
        ('mavros_odom_rate', '100.0', 'Rate at which mavros publishes odom data'),
        ('mavros_imu_rate', '100.0', 'Rate at which mavros publishes imu data'),
        ('mavros_gps_rate', '15.0', 'Rate at which mavros publishes gps data'),
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTABMap params'),
        ('navsat_transform_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'navsat_transform.yaml']), 'Path to navsat_transform params'),
        ('ekf_local_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_local.yaml']), 'Path to local EKF config'),
        ('ekf_global_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf_global.yaml']), 'Path to global EKF config'),
        ('dlio_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']), 'Path to DLIO params'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'blastoise', 'blastoise.xacro']), 'Path to robot URDF/XACRO')
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
            ('odom', '/odom/lidar'),
            ('pose', '/lidar/pose'),
            ('path', '/lidar/path'),
            ('kf_pose', '/lidar/keyframes/pose'),
            ('kf_cloud', '/lidar/keyframes/cloud'),
            ('deskewed', '/lidar/deskewed')
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[launch_configurations['dlio_params_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[('keyframes', '/dlio/pointcloud/keyframe')],
    )

    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[launch_configurations['rtabmap_params_file'], {'use_sim_time': launch_configurations['sim']}],
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{"use_mag": False, "publish_tf": False, "use_sim_time": launch_configurations['sim']}],
        remappings=[
            ("imu/data_raw", "/imu/data_raw"),
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
            ('imu/data', '/imu/filtered'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odom/local'),
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

    set_mavros_message_rate = TimerAction(
        period=5.0,
        actions=[
            OpaqueFunction(
                function=lambda context: [
                    # Local position / odometry
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            '/mavros/set_message_interval',
                            'mavros_msgs/srv/MessageInterval',
                            f'{{message_id: 32, message_rate: {LaunchConfiguration("mavros_odom_rate").perform(context)}}}'
                        ],
                        output='screen'
                    ),
                    # High-rate IMU
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            '/mavros/set_message_interval',
                            'mavros_msgs/srv/MessageInterval',
                            f'{{message_id: 27, message_rate: {LaunchConfiguration("mavros_imu_rate").perform(context)}}}'
                        ],
                        output='screen'
                    ),
                    # GPS fix
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            '/mavros/set_message_interval',
                            'mavros_msgs/srv/MessageInterval',
                            f'{{message_id: 24, message_rate: {LaunchConfiguration("mavros_gps_rate").perform(context)}}}'
                        ],
                        output='screen'
                    )
                ]
            )
        ]
    )

    robot_state_publisher_node = OpaqueFunction(function=load_urdf)

    return LaunchDescription(
        declare_arguments + [
            # dlio_map_node,
            dlio_odom_node,
            mavros_node,
            rtabmap_odom_node,
            imu_filter_node,
            navsat_transform_node,
            ekf_local_node_delayed,
            ekf_global_node_delayed,
            set_mavros_message_rate,
            robot_state_publisher_node
        ]
    )