"""Perception: what is around the boat. Its sensors, the object tracker and the perception costmap.

ros2 launch mhseals_nav perception.launch.py [camera:=zed|sim|none] [lidar:=velodyne|sim|none] [sim:=true]

camera  zed   the ZED wrapper (its own detector) with named lazy relays, and zed_backend
        sim   no driver: sim_backend turns the simulator's detections into observations
        none  no camera and no backend; the tracker and costmap still run
lidar   velodyne  the Velodyne driver pair on the large-data transport, and lidar_relay onto /points
        sim       no driver: the simulator publishes /points (see sim_bridge.launch.py)
        none      no lidar
Every choice but none also starts /perception/nearby. The tracker, lidar_fusion (config/lidar_fusion.yaml)
and the costmap (/perception/costmap, the only way the lidar reaches Nav2) run whatever the sensors.
"""
import os
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mhseals_nav.camera_configuration import resolve_camera
from mhseals_nav.layers import boolean, config_path
from mhseals_nav.lidar_configuration import resolve_lidar
from mhseals_nav.yaml_params import footprint, hull, params_file

CAMERAS = ('zed', 'sim', 'none')
LIDARS = ('velodyne', 'sim', 'none')
BACKENDS = {'zed': ('zed_backend', None), 'sim': ('sim_backend', 'sim_backend.yaml')}   # (node, params)
# Sim: /points on the default graph, the camera's body-axes link and its camera_info; crane's IMU runs at
# 19-20 Hz, healthy next to the 4 Hz FCU stream the attitude gate is for.
SIM_FUSION = {'points_transport': 'direct', 'camera_axes': 'body',
              'camera_info_topic': '/front_camera/camera_info', 'attitude_min_hz': 15.0}
FASTDDS_NS = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'

ARGUMENTS = [
    ('camera', 'zed', 'Camera: zed, sim or none'),
    ('lidar', 'velodyne', 'Lidar: velodyne, sim or none'),
    ('sim', 'false', 'Use the simulator clock'),
    ('camera_name', 'front', 'ZED namespace; topics are /<camera_name>/zed_node/*'),
    ('camera_model', 'zed2i', 'ZED model'),
    ('mapping', 'false', 'Also relay the raw RGB and depth images for RTAB-Map'),
    ('preview_rate', '', 'Preview throttle rate; empty inherits config/camera.yaml'),
    ('camera_deployment_file', '', 'Camera YAML deployment layer'),
    ('camera_startup_file', '', 'Camera YAML operator layer'),
    ('lidar_params_file', '', 'Lidar YAML over the package and deployment layers'),
    ('lidar_deployment_file', '', 'Lidar YAML deployment layer'),
    ('lidar_required_frame', '', 'Frame the lidar must publish in; empty: no check'),
    ('velodyne_ip', '', 'Lidar address; empty inherits the YAML layers, then the vessel profile'),
    ('velodyne_port', '', 'Lidar UDP port; empty inherits the YAML layers'),
]


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def choice(context, name, allowed):
    found = value(context, name)
    if found not in allowed:
        raise ValueError(f'{name} must be one of {", ".join(allowed)}, not {found!r}')
    return found


# Camera

def pinned_profile(profile, address):
    """The profile itself, or a temp copy whose UDPv4 transport uses only `address` and loopback.

    Same pin as ros_net in astro_dock's dev.bashrc: without it Fast DDS advertises docker0 and other
    interfaces, and discovery works while data goes nowhere.
    """
    if not address:
        return profile
    ET.register_namespace('', FASTDDS_NS)
    tree = ET.parse(profile)
    for udp in tree.iter(f'{{{FASTDDS_NS}}}transport_descriptor'):
        if udp.findtext(f'{{{FASTDDS_NS}}}type') == 'UDPv4':
            allowed = ET.SubElement(udp, f'{{{FASTDDS_NS}}}interfaceWhiteList')
            # Loopback too, as ros_net does: same-host peers on the default profile announce
            # 127.0.0.1 and are unreachable without it.
            for allow in (address, '127.0.0.1'):
                ET.SubElement(allowed, f'{{{FASTDDS_NS}}}address').text = allow
    descriptor, path = tempfile.mkstemp(prefix='camera_dds_', suffix='.xml')
    os.close(descriptor)
    tree.write(path, xml_declaration=True, encoding='UTF-8')
    return path


def relay(name, executable, arguments):
    """A topic_tools relay that idles until someone subscribes."""
    return Node(package='topic_tools', executable=executable, name=name, arguments=arguments,
                parameters=[{'lazy': True}], output='screen')


def camera_relays(camera_name, mapping, preview_rate):
    """The canonical /<camera_name>_camera/* topics; consumers never bind to the wrapper's names."""
    zed, out = f'/{camera_name}/zed_node', f'/{camera_name}_camera'
    found = [relay(f'{camera_name}_info_relay', 'relay',
                   [f'{zed}/rgb/color/rect/camera_info', f'{out}/camera_info']),
             relay(f'{camera_name}_preview_throttle', 'throttle',
                   ['messages', f'{zed}/rgb/color/rect/image/compressed', str(preview_rate),
                    f'{out}/preview/compressed'])]
    if mapping:
        found += [relay(f'{camera_name}_rgb_relay', 'relay',
                        [f'{zed}/rgb/color/rect/image', f'{out}/rgb/image']),
                  relay(f'{camera_name}_depth_relay', 'relay',
                        [f'{zed}/depth/depth_registered', f'{out}/depth/image'])]
    return found


def zed_camera(context):
    """The ZED wrapper and its relays, in a scope with the large-frame DDS profile (camera only)."""
    settings = resolve_camera(deployment_file=value(context, 'camera_deployment_file'),
                              operator_file=value(context, 'camera_startup_file'),
                              preview_rate=value(context, 'preview_rate'))
    profile = pinned_profile(str(config_path('camera_dds.xml')), os.environ.get('ASTRO_DDS_ADDRESS'))
    camera_name = value(context, 'camera_name')
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('zed_wrapper'), '/launch/zed_camera.launch.py']),
        launch_arguments={
            'camera_model': value(context, 'camera_model'), 'camera_name': camera_name,
            # The EKFs and the robot description own every TF edge but the wrapper's static camera subtree.
            'publish_tf': 'false', 'publish_map_tf': 'false', 'publish_imu_tf': 'false',
            'publish_urdf': 'true',
            'custom_object_detection_config_path': str(config_path('custom_object_detection.yaml')),
            'ros_params_override_path': str(config_path('zed_boat.yaml')),
            # The model ships in this package's share, wherever it is installed.
            'param_overrides': f'object_detection.custom_onnx_file:={config_path("epoch60.onnx")}',
        }.items())
    return [GroupAction(scoped=True, actions=[
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', profile),
        wrapper, *camera_relays(camera_name, boolean(value(context, 'mapping')), settings.preview_rate)])]


# Lidar

def large_data_env():
    """The Velodyne pair's transport (config/fastdds_large_data.xml); TF and odometry keep the default."""
    profile = str(config_path('fastdds_large_data.xml'))
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': profile, 'FASTDDS_DEFAULT_PROFILES_FILE': profile}


def velodyne(context):
    """The driver and pointcloud pair (address, model and rpm from the vessel profile) and lidar_relay."""
    lidar = resolve_lidar(value(context, 'lidar_params_file'), value(context, 'lidar_deployment_file'),
                          {name: value(context, name) for name in ('velodyne_ip', 'velodyne_port')},
                          required_frame=value(context, 'lidar_required_frame'))
    return [Node(package='velodyne_driver', executable='velodyne_driver_node', name='velodyne_driver',
                 output='screen', additional_env=large_data_env(), parameters=[lidar['driver']]),
            Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
                 name='velodyne_transform', output='screen', additional_env=large_data_env(),
                 parameters=[lidar['transform']], remappings=[('velodyne_points', '/points')]),
            # One reliable /points writer on the default transport, downsampled, for Nav2 and RViz.
            Node(package='mhseals_nav', executable='lidar_relay', output='screen')]


# Objects

def objects(camera, sim):
    """The camera's backend, lidar_fusion when enabled, the tracker, the costmap and nearest_obstacle."""
    clock = {'use_sim_time': sim}
    rectangle = hull()
    found = []
    if camera in BACKENDS:
        executable, backend_file = BACKENDS[camera]
        found.append(Node(package='mhseals_nav', executable=executable, output='screen',
                          parameters=[*([str(config_path(backend_file))] if backend_file else []), clock]))
    tracker = [str(config_path('tracker.yaml')), clock]
    if params_file('lidar_fusion.yaml')['enabled']:
        # The tracker's only input then: respawn, and the depth term moves into the covariance.
        tracker.append({'observations_topic': params_file('lidar_fusion.yaml')['output_topic'],
                        'meas.depth_term': False})
        found.append(Node(package='mhseals_nav', executable='lidar_fusion', output='screen',
                          respawn=True, respawn_delay=1.0,
                          parameters=[str(config_path('lidar_fusion.yaml')), clock,
                                      *([SIM_FUSION] if sim else [])]))
    return [
        *found,
        Node(package='mhseals_nav', executable='object_tracker', output='screen', parameters=tracker),
        # /perception/costmap(_updates). A standalone Jazzy Costmap2DROS never creates a bond, so the
        # manager must not wait for one.
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='costmap',
             namespace='perception', output='screen',
             parameters=[str(config_path('perception_costmap.yaml')),
                         {'footprint': footprint(rectangle), 'footprint_padding': rectangle['padding']},
                         clock]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_perception', namespace='perception', output='screen',
             parameters=[{'autostart': True, 'node_names': ['costmap'], 'bond_timeout': 0.0, **clock}]),
        # /perception/nearest_obstacle for the velocity controller's drift observer.
        Node(package='mhseals_nav', executable='nearest_obstacle', output='screen',
             parameters=[str(config_path('nearest_obstacle.yaml')), clock]),
    ]


def nodes(context):
    camera = choice(context, 'camera', CAMERAS)
    lidar = choice(context, 'lidar', LIDARS)
    sim = boolean(value(context, 'sim'))
    found = zed_camera(context) if camera == 'zed' else []
    found += velodyne(context) if lidar == 'velodyne' else []
    if lidar != 'none':   # /perception/nearby: /points summarized by sector (config/lidar.yaml costmap)
        found.append(Node(package='mhseals_nav', executable='nearby', output='screen',
                          parameters=[{'use_sim_time': sim}]))
    return found + objects(camera, sim)


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, default, text in ARGUMENTS],
        OpaqueFunction(function=nodes),
    ])
