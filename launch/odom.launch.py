from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_urdf(context):
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )]

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fcu_url = LaunchConfiguration('fcu_url')
    mavros_params_file = LaunchConfiguration('mavros_params_file')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    dlio_yaml_file = LaunchConfiguration('dlio_yaml_file')
    dlio_params_file = LaunchConfiguration('dlio_params_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='tcp://127.0.0.1:5762',
        description='MAVLink connection URL'
    )

    mavros_params_arg = DeclareLaunchArgument(
        'mavros_params_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'config', 'mavros.yaml']),
        description='Path to MAVROS parameters file'
    )

    rtabmap_params_arg = DeclareLaunchArgument(
        'rtabmap_params_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']),
        description='Path to RTAB-Map parameters file'
    )

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'config', 'ekf.yaml']),
        description='Path to EKF configuration file'
    )

    dlio_yaml_arg = DeclareLaunchArgument(
        'dlio_yaml_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio.yaml']),
        description='Path to DLIO main YAML'
    )

    dlio_params_arg = DeclareLaunchArgument(
        'dlio_params_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'config', 'dlio_params.yaml']),
        description='Path to DLIO parameters YAML'
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([mhseals_nav_dir, 'description', 'omni_boat.urdf']),
        description='Path to robot URDF file'
    )

    # Nodes
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_file, dlio_params_file],
        remappings=[
            ('pointcloud', '/points'),
            ('imu', '/imu/data'),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ]
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_file, dlio_params_file],
        remappings=[('keyframes', 'dlio/odom_node/pointcloud/keyframe')]
    )

    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera_info')
        ]
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{"use_mag": False, "use_sim_time": use_sim_time}],
        remappings=[("imu/data_raw", "/mavros/imu/data")]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}]
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_params_file, {'use_sim_time': use_sim_time, 'fcu_url': fcu_url}]
    )
    
    robot_state_publisher_node = OpaqueFunction(function=load_urdf)

    return LaunchDescription([
        use_sim_time_arg,
        fcu_url_arg,
        mavros_params_arg,
        rtabmap_params_arg,
        ekf_config_arg,
        dlio_yaml_arg,
        dlio_params_arg,
        urdf_arg,
        # dlio_odom_node,
        # dlio_map_node,
        rtabmap_odom_node,
        imu_filter_node,
        ekf_node,
        mavros_node,
        robot_state_publisher_node
    ])