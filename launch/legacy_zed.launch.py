"""JP5 / SDK 4: calibrated ZED RGB-D, external buoy inference and rosbridge."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('mhseals_nav')
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='front'),
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('model_file', default_value=PathJoinSubstitution(
            [share, 'config', 'epoch60.onnx'])),
        DeclareLaunchArgument('engine_cache',
                              default_value='/usr/local/zed/resources/astro_buoys'),
        DeclareLaunchArgument('tile_size', default_value='720',
                              description='Central detail crop size; 0 disables crops'),
        GroupAction(actions=[
            SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                PathJoinSubstitution([share, 'config', 'legacy_camera_dds.xml'])),
            SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE',
                PathJoinSubstitution([share, 'config', 'legacy_camera_dds.xml'])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
                launch_arguments={
                    'camera_model': LaunchConfiguration('camera_model'),
                    'camera_name': LaunchConfiguration('camera_name'),
                    'publish_tf': 'false', 'publish_map_tf': 'false',
                    'ros_params_override_path': PathJoinSubstitution(
                        [share, 'config', 'legacy_zed.yaml']),
                }.items()),
            Node(package='mhseals_nav', executable='legacy_buoy_detector',
                 parameters=[{
                     'model_file': LaunchConfiguration('model_file'),
                     'engine_cache': LaunchConfiguration('engine_cache'),
                     'camera_topic': ['/', LaunchConfiguration('camera_name'), '/zed_node'],
                     'tile_size': LaunchConfiguration('tile_size'),
                 }], output='screen'),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rosbridge_server'), 'launch',
                'rosbridge_websocket_launch.xml']))),
        ]),
    ])
