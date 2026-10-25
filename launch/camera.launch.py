"""ZED wrapper plus its named, lazy relays; the jetson device profile starts it."""

import os
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.camera_configuration import from_launch
FASTDDS_NS = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'


def pinned_profile(profile, address):
    """The profile itself, or a temp copy whose UDPv4 transport uses only `address` and loopback.

    Same pin as ros_net in astro_dock's dev.bashrc: without it Fast DDS advertises
    docker0 and other interfaces, and discovery works while data goes nowhere.
    """
    if not address:
        return profile
    ET.register_namespace('', FASTDDS_NS)
    tree = ET.parse(profile)
    for udp in tree.iter(f'{{{FASTDDS_NS}}}transport_descriptor'):
        if udp.findtext(f'{{{FASTDDS_NS}}}type') == 'UDPv4':
            allowed = ET.SubElement(udp, f'{{{FASTDDS_NS}}}interfaceWhiteList')
            # Loopback too, as ros_net does: same-host peers on the default profile
            # announce 127.0.0.1 and are unreachable without it.
            for allow in (address, '127.0.0.1'):
                ET.SubElement(allowed, f'{{{FASTDDS_NS}}}address').text = allow
    descriptor, path = tempfile.mkstemp(prefix='camera_dds_', suffix='.xml')
    os.close(descriptor)
    tree.write(path, xml_declaration=True, encoding='UTF-8')
    return path


def dds_env(context):
    profile = pinned_profile(
        PathJoinSubstitution([FindPackageShare('mhseals_nav'), 'config',
                              'camera_dds.xml']).perform(context),
        os.environ.get('ASTRO_DDS_ADDRESS'))
    return [SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile),
            SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', profile)]


def relay(name, executable, arguments):
    """Node arguments for a topic_tools relay that idles until someone subscribes."""
    return dict(package='topic_tools', executable=executable, name=name,
                arguments=arguments, parameters=[{'lazy': True}], output='screen')


def relays(camera_name, slam, preview_hz):
    zed = f'/{camera_name}/zed_node'
    out = f'/{camera_name}_camera'
    found = [
        relay(f'{camera_name}_info_relay', 'relay',
              [f'{zed}/rgb/color/rect/camera_info', f'{out}/camera_info']),
        relay(f'{camera_name}_preview_throttle', 'throttle',
              ['messages', f'{zed}/rgb/color/rect/image/compressed', str(preview_hz),
               f'{out}/preview/compressed']),
    ]
    if slam:
        found += [
            relay(f'{camera_name}_rgb_relay', 'relay',
                  [f'{zed}/rgb/color/rect/image', f'{out}/rgb/image']),
            relay(f'{camera_name}_depth_relay', 'relay',
                  [f'{zed}/depth/depth_registered', f'{out}/depth/image']),
        ]
    return found


def relay_nodes(context, settings):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    return [Node(**kwargs) for kwargs in relays(camera_name, slam, settings.preview_hz)]


def camera_startup(context):
    settings = from_launch(context)
    environment = dds_env(context)
    share = FindPackageShare('mhseals_nav')
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            # The EKFs and the robot description own every TF edge except the
            # wrapper's static camera subtree.
            'publish_tf': 'false', 'publish_map_tf': 'false', 'publish_imu_tf': 'false',
            'publish_urdf': 'true',
            'custom_object_detection_config_path': PathJoinSubstitution(
                [share, 'config', 'custom_object_detection.yaml']),
            'ros_params_override_path': PathJoinSubstitution(
                [share, 'config', 'zed_boat.yaml']),
        }.items())
    # The ZED IMU and magnetometer as ekf_local's second yaw reference (on the ODROID).
    reference = Node(
        package='mhseals_nav', executable='zed_imu_reference', name='zed_imu_reference',
        output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'zed_imu_reference.yaml']),
            {'camera_name': LaunchConfiguration('camera_name')}])
    return [GroupAction([*environment, wrapper, *relay_nodes(context, settings), reference])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_deployment_file', default_value='',
                              description='Explicit camera startup deployment YAML'),
        DeclareLaunchArgument('camera_startup_file', default_value='',
                              description='Explicit camera startup operator YAML'),
        DeclareLaunchArgument('preview_hz', default_value='',
                              description='Explicit preview throttle rate; empty inherits YAML'),
        DeclareLaunchArgument('camera_name', default_value='front',
                              description='ZED namespace; topics are /<camera_name>/zed_node/*'),
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Also relay raw RGB and depth for RTAB-Map'),
        # Large camera frames need this profile; it applies to this launch only and
        # keeps the boat's interface pin and 1400-byte messages.
        OpaqueFunction(function=camera_startup),
    ])
