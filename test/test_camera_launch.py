"""Check the camera host's launch description without a camera or the wrapper's nodes."""

import importlib.util
import re
from pathlib import Path

from mhseals_nav.camera_configuration import resolve_camera
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
    OpaqueFunction, SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('camera_launch', ROOT / 'launch/camera.launch.py')
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


def default_rate():
    return resolve_camera(ROOT / 'config/camera_startup.yaml').preview_hz


def wrapper_arguments():
    try:
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('zed_wrapper'),
                      'launch/zed_camera.launch.py').read_text()
    except Exception:
        pytest.skip('zed_wrapper is not installed')
    return set(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)'", source))


@pytest.fixture(autouse=True)
def fake_share(monkeypatch):
    """mhseals_nav need not be installed to check the paths the launch builds."""
    from launch_ros.substitutions import FindPackageShare
    monkeypatch.setattr(camera, 'from_launch', lambda context: resolve_camera(
        ROOT / 'config/camera_startup.yaml',
        deployment_file=context.launch_configurations.get('camera_deployment_file', ''),
        operator_file=context.launch_configurations.get('camera_startup_file', ''),
        preview_hz=context.launch_configurations.get('preview_hz', '')))
    real = FindPackageShare.perform
    monkeypatch.setattr(FindPackageShare, 'perform', lambda self, context: (
        '/share/mhseals_nav' if 'mhseals_nav' in perform_substitutions(context, self.package)
        else real(self, context)))


def performed(context, value):
    if isinstance(value, str):
        return value
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def group_actions():
    description = camera.generate_launch_description()
    declared = {e.name: e for e in description.entities if isinstance(e, DeclareLaunchArgument)}
    context = LaunchContext()
    for argument in declared.values():
        argument.execute(context)
    startup = next(e for e in description.entities if isinstance(e, OpaqueFunction))
    group = startup.execute(context)[0]
    assert isinstance(group, GroupAction)
    return declared, group.get_sub_entities()


def test_one_config_point_with_defaults():
    declared, _ = group_actions()
    defaults = {name: perform_substitutions(LaunchContext(), arg.default_value)
                for name, arg in declared.items()}
    assert defaults == {'camera_name': 'front', 'camera_model': 'zed2i', 'slam': 'false',
                        'camera_deployment_file': '', 'camera_startup_file': '', 'preview_hz': ''}


def test_wrapper_include_loads_our_files_and_publishes_no_pose_tf():
    _, entities = group_actions()
    include = next(e for e in entities if isinstance(e, IncludeLaunchDescription))
    context = LaunchContext()
    for name, value in (('camera_name', 'front'), ('camera_model', 'zed2i')):
        context.launch_configurations[name] = value
    passed = {performed(context, key): performed(context, value)
              for key, value in include.launch_arguments}
    for key in ('publish_tf', 'publish_map_tf', 'publish_imu_tf'):
        assert passed[key] == 'false'
    assert passed['publish_urdf'] == 'true'
    assert passed['camera_name'] == 'front'
    assert passed['ros_params_override_path'].endswith('/config/zed_boat.yaml')
    assert passed['custom_object_detection_config_path'].endswith(
        '/config/custom_object_detection.yaml')


def test_wrapper_include_passes_only_arguments_the_wrapper_declares():
    _, entities = group_actions()
    include = next(e for e in entities if isinstance(e, IncludeLaunchDescription))
    passed = {performed(LaunchContext(), key) for key, _ in include.launch_arguments}
    assert passed <= wrapper_arguments()


def dds_variables(monkeypatch, address):
    if address:
        monkeypatch.setenv('ASTRO_DDS_ADDRESS', address)
    else:
        monkeypatch.delenv('ASTRO_DDS_ADDRESS', raising=False)
    variables = camera.dds_env(LaunchContext())
    return [perform_substitutions(LaunchContext(), v.value) for v in variables]


def test_camera_scoped_dds_profile_is_used_as_is_without_a_boat_address(monkeypatch):
    values = dds_variables(monkeypatch, None)
    assert values == ['/share/mhseals_nav/config/camera_dds.xml'] * 2


def test_camera_dds_profile_whitelists_the_boat_address(monkeypatch):
    import xml.etree.ElementTree as ET
    from launch_ros.substitutions import FindPackageShare
    monkeypatch.setattr(FindPackageShare, 'perform', lambda self, context: str(ROOT))
    values = dds_variables(monkeypatch, '192.168.8.20')
    assert values[0] == values[1] != str(ROOT / 'config/camera_dds.xml')
    ns = '{%s}' % camera.FASTDDS_NS
    udp = [d for d in ET.parse(values[0]).iter(ns + 'transport_descriptor')
           if d.findtext(ns + 'type') == 'UDPv4']
    assert udp
    for d in udp:
        assert [a.text for a in d.iter(ns + 'address')] == ['192.168.8.20']
        assert d.findtext(ns + 'maxMessageSize') == '1400'


def test_boat_relays_are_named_and_carry_no_raw_images():
    found = {n['name']: (n['executable'], n['arguments'])
             for n in camera.relays('front', False, default_rate())}
    assert found == {
        'front_info_relay': ('relay', [
            '/front/zed_node/rgb/color/rect/camera_info', '/front_camera/camera_info']),
        'front_preview_throttle': ('throttle', [
            'messages', '/front/zed_node/rgb/color/rect/image/compressed', '2.0',
            '/front_camera/preview/compressed']),
    }


def test_slam_adds_named_raw_relays():
    boat = {n['name'] for n in camera.relays('front', False, default_rate())}
    slam = {n['name']: n['arguments'][-1] for n in camera.relays('front', True, default_rate())}
    assert set(slam) - boat == {'front_rgb_relay', 'front_depth_relay'}
    assert slam['front_rgb_relay'] == '/front_camera/rgb/image'
    assert slam['front_depth_relay'] == '/front_camera/depth/image'


def test_relay_names_are_unique_and_follow_camera_name():
    names = [n['name'] for n in camera.relays('rear', True, default_rate())]
    assert len(names) == len(set(names))
    assert all(name.startswith('rear_') for name in names)


def test_every_relay_is_lazy():
    assert all(n['parameters'] == [{'lazy': True}]
               for n in camera.relays('front', True, default_rate()))


def test_hardware_sensors_launch_starts_no_camera():
    """The camera host runs camera.launch.py; sensors.launch.py is lidar and the sim endpoint."""
    for name in ('sensors', 'robot'):
        text = (ROOT / f'launch/{name}.launch.py').read_text()
        for dead in ('zed_wrapper', 'common_config_path', 'zed_id_path', 'ffmpeg_config_path',
                     'custom_onnx_file', 'common_stereo', 'ffmpeg.yaml', 'zed2i.yaml'):
            assert dead not in text, (name, dead)
