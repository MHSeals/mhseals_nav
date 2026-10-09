"""Optional RTABMap mapping; localization TF remains owned by the EKFs."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_rtabmap_slam_node(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    return [Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params_file, {'use_sim_time': sim}],
        remappings=[
            ("/rgb/image", f"/{camera_name}_camera/rgb/image"),
            ("/depth/image", f"/{camera_name}_camera/depth/image"),
            ("/rgb/camera_info", f"/{camera_name}_camera/camera_info")
        ]
    )]


def generate_launch_description():
    share = FindPackageShare('mhseals_nav')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='front'),
        DeclareLaunchArgument('rtabmap_params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'rtabmap.yaml'])),
        OpaqueFunction(function=create_rtabmap_slam_node),
    ])
