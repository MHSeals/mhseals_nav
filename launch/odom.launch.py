from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("mhseals_nav")
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")
    
    dlio_yaml_path = os.path.join(pkg_share, "config", "dlio.yaml")
    dlio_params_yaml_path = os.path.join(pkg_share, "config", "dlio_params.yaml")

    use_sim_time = LaunchConfiguration('use_sim_time')
    mavros_params = LaunchConfiguration('mavros_params')
    fcu_url = LaunchConfiguration('fcu_url', default='tcp://127.0.0.1:5762')
    rtabmap_params = LaunchConfiguration('rtabmap_params')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time.'
    )

    rtabmap_params_arg = DeclareLaunchArgument(
        'rtabmap_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('mhseals_nav'),
            'config',
            'rtabmap.yaml'
        ])
    )

    mavros_params_arg = DeclareLaunchArgument(
        'mavros_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('mhseals_nav'),
            'config',
            'mavros.yaml'
        ]),
        description='Path to MAVROS parameters file'
    )

    dlio_odom = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
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
    
    dlio_map = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera_info')
        ]
    )
    
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{"use_mag": False}],
        remappings=[("imu/data_raw", "/mavros/imu/data")],
    )
    
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            mavros_params,
            {'use_sim_time': use_sim_time, 'fcu_url': fcu_url}
        ]
    )

    urdf_file = os.path.join(pkg_share, 'description', 'omni_boat.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        mavros_params_arg,
        rtabmap_params_arg,
        # dlio_odom,
        # dlio_map,
        rtabmap_odom,
        imu_filter,
        ekf,
        mavros,
        robot_state_publisher
    ])
