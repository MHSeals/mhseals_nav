from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def create_rtabmap_slam_node(context):
    sim = bool(LaunchConfiguration('sim').perform(context))
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
    mhseals_nav_dir = FindPackageShare('mhseals_nav')

    launch_args = [
        ('sim', 'true', 'Use simulation time'),
        ('camera_name', 'front', 'Name of camera'),
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

    rtabmap_slam_node = OpaqueFunction(function=create_rtabmap_slam_node)

    thruster_allocation = Node(
        package='mhseals_nav',
        executable='thruster_allocation',
        name='thruster_allocation',
        output='screen'
    )

    # twist_converter_node = Node(
    #     package='mhseals_nav',
    #     executable='twist_converter',
    #     name='twist_converter',
    #     output='screen'
    # )

    return LaunchDescription(declare_arguments + [
        thruster_allocation, 
        nav2_bringup_launch,
        rtabmap_slam_node,
        # twist_converter_node
    ])