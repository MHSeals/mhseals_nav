from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_args = [
        ('sim', 'true', 'Use simulation time'),
        ('rtabmap_params', PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'rtabmap.yaml']), 'Path to RTAB-Map parameters file'),
        ('nav2_params', PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config', 'nav2_params.yaml']), 'Path to Nav2 parameters file')
    ]

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Nodes / launch includes
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': launch_configurations['sim'],
            'params_file': launch_configurations['nav2_params']
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
        parameters=[launch_configurations['rtabmap_params'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ("/rgb/image", "/camera/image_raw"),
            ("/depth/image", "/camera/depth/image_rect_raw"),
            ("/rgb/camera_info", "/camera/camera_info"),
        ]
    )

    return LaunchDescription(declare_arguments + [
        nav2_bringup_launch,
        twist_converter,
        rtabmap_slam
    ])