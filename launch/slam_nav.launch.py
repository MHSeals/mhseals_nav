import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_params = LaunchConfiguration('rtabmap_params')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    rtabmap_params_arg = DeclareLaunchArgument(
        'rtabmap_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('mhseals_nav'),
            'config',
            'rtabmap.yaml'
        ])
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('mhseals_nav'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    twist_converter = Node(
        package='mhseals_nav',
        executable='twist_converter',
        name='twist_converter',
        output='screen'
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params, {'use_sim_time': use_sim_time}],
        remappings=[
           ("/rgb/image", "/camera/image_raw"), 
           ("/depth/image", "/camera/depth/image_rect_raw"),
           ("/rgb/camera_info", "/camera/camera_info"),
        ]
    )


    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_params_arg,
        nav2_bringup_launch,
        twist_converter,
        rtabmap_slam
    ])