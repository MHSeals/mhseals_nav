from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
            'use_sim_time': LaunchConfiguration('sim')
        }],
        condition=UnlessCondition(PythonExpression(["'true' == '", LaunchConfiguration('sim'), "'"]))
    )]

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'true', 'Run in simulation mode'),
        ('use_sim_time', 'true', 'Use simulation clock if true'),
        ('fcu_url', 'tcp://127.0.0.1:5762', 'Flight control unit connection URL'),
        ('gcs_url', 'udp://127.0.0.1:14550', 'Ground control service connection URL'),
        ('mavros_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']), 'Path to MAVROS params'),
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTABMap params'),
        ('ekf_config_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf.yaml']), 'Path to EKF config'),
        ('dlio_yaml_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']), 'Path to DLIO main YAML'),
        ('dlio_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio_params.yaml']), 'Path to DLIO params'),
        ('urdf_file', PathJoinSubstitution([mhseals_nav_dir, 'description', 'omni_boat', 'omni_boat.xacro']), 'Path to robot URDF/XACRO')
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
        parameters=[launch_configurations['dlio_yaml_file'], launch_configurations['dlio_params_file']],
        remappings=[
            ('pointcloud', '/points'),
            ('imu', '/imu/data'),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[launch_configurations['dlio_yaml_file'], launch_configurations['dlio_params_file']],
        remappings=[('keyframes', 'dlio/odom_node/pointcloud/keyframe')],
    )

    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[launch_configurations['rtabmap_params_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera_info')
        ],
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{"use_mag": False, "use_sim_time": launch_configurations['sim']}],
        remappings=[("imu/data_raw", "/mavros/imu/data")],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[launch_configurations['ekf_config_file'], {'use_sim_time': launch_configurations['sim']}],
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[launch_configurations['mavros_params_file'],
                    {'use_sim_time': launch_configurations['sim'],
                     'fcu_url': launch_configurations['fcu_url'],
                     'gcs_url': launch_configurations['gcs_url']}],
    )

    robot_state_publisher_node = OpaqueFunction(function=load_urdf)

    return LaunchDescription(
        declare_arguments + [
            # dlio_odom_node,
            # dlio_map_node,
            rtabmap_odom_node,
            imu_filter_node,
            ekf_node,
            mavros_node,
            robot_state_publisher_node
        ]
    )