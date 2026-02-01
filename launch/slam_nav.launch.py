from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'true', 'Use simulation time'),
        ('nav2_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'nav2_params.yaml']), 'Path to Nav2 parameters file'),
        ('rtabmap_params_file', PathJoinSubstitution([mhseals_nav_dir, 'config', 'rtabmap.yaml']), 'Path to RTABMap params')
    ]

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    nav2_bringup_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': launch_configurations['sim'],
                    'params_file': launch_configurations['nav2_params_file']
                }.items()
            )
        ]
    )

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[launch_configurations['rtabmap_params_file'], {'use_sim_time': launch_configurations['sim']}],
        remappings=[
            ("/rgb/image", "/camera/image_raw"),
            ("/depth/image", "/camera/depth/image_rect_raw"),
            ("/rgb/camera_info", "/camera/camera_info"),
        ]
    )

    twist_converter_node = Node(
        package='mhseals_nav',
        executable='twist_converter',
        name='twist_converter',
        output='screen'
    )

    return LaunchDescription(declare_arguments + [
        nav2_bringup_launch,
        rtabmap_slam_node,
        twist_converter_node
    ])