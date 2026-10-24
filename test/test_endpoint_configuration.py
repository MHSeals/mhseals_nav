"""Endpoint layers and fixed discovery policies, without sockets or real devices."""
from hashlib import sha256
from pathlib import Path
import sys
from types import ModuleType

from mhseals_nav.endpoint_configuration import discover_device, resolve_endpoints, validate_url
import pytest
import yaml

ROOT = Path(__file__).parents[1]
BASE = ROOT / 'config/endpoint_startup.yaml'


def selected(tmp_path, data, filename='operator.yaml'):
    path = tmp_path / filename
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize('context,baud,strategy', [
    ('standalone', 115200, 'odom_cube_if00'), ('full_robot', 115200, 'odom_cube_if00'),
    ('tui', 57600, 'tui_stable_hints')])
def test_default_contexts(tmp_path, context, baud, strategy):
    result = resolve_endpoints(BASE, context=context)
    parameters, observation = result.mavros_parameters(False, tmp_path)
    assert parameters == {'fcu_url': f'serial:///dev/ttyACM0:{baud}',
                          'gcs_url': 'udp://@127.0.0.1:14550', 'use_sim_time': False}
    assert observation['discovery'] == strategy and observation['fallback'] is True
    assert result.mavros_parameters(True, tmp_path)[0]['fcu_url'] == 'tcp://127.0.0.1:5762'
    assert result.unity_parameters == {'ROS_IP': '127.0.0.1', 'ROS_TCP_PORT': 6767}
    assert type(result.unity_parameters['ROS_TCP_PORT']) is int


def test_layer_alias_precedence_identity_and_disable(tmp_path):
    deployment = selected(tmp_path, {'unity': {'port': 7000}}, 'deployment.yaml')
    operator = selected(tmp_path, {'unity': {'port': 8000}, 'fcu_url': 'tcp://host:9000'})
    raw = operator.read_bytes()
    result = resolve_endpoints(BASE, deployment, operator,
                               {'ros_port': '9000', 'gcs_url': '', 'fcu_url': ''})
    assert result.unity_parameters['ROS_TCP_PORT'] == 9000
    assert result.mavros_parameters(False, tmp_path)[0]['gcs_url'] == ''
    assert result.mavros_parameters(False, tmp_path)[0]['fcu_url'] == 'tcp://host:9000'
    assert result.overrides == {'ros_port': 9000, 'gcs_url': ''}
    assert result.inputs[-1]['sha256'] == sha256(raw).hexdigest()
    assert result.inputs[-1]['path'] == str(operator.resolve())
    assert operator.read_bytes() == raw
    with pytest.raises(TypeError):
        result.values['unity']['port'] = 1


@pytest.mark.parametrize('data', [
    {'unity': {'port': True}}, {'unity': {'port': '7000'}}, {'unity': {'port': 0}},
    {'unity': {'port': 65536}}, {'unity': {'ip': 'localhost'}}, {'unity': None},
    {'simulation_fcu_url': 'udp://127.0.0.1:14550'}, {'fcu_url': None},
    {'serial_fallback': 'relative'}, {'contexts': {'tui': {'baud': True}}},
    {'contexts': {'tui': {'baud': 2147483648}}}, {'contexts': {'tui': {'baud': 0}}},
    {'contexts': {'tui': {'discovery': 'shell-command'}}}, {'unknown': 1}])
def test_invalid_layers_cannot_be_hidden(tmp_path, data):
    path = selected(tmp_path, data)
    with pytest.raises((ValueError, yaml.YAMLError)):
        resolve_endpoints(BASE, path, aliases={'ros_port': '9000', 'fcu_url': 'tcp://host:1'})


@pytest.mark.parametrize('raw', [
    'unity: {port: 7000, port: 8000}', 'unity: &x {port: 7000}\ncontexts: *x',
    'unity: {<<: {port: 7000}}', '[]', 'null'])
def test_ambiguous_yaml_refuses(tmp_path, raw):
    path = tmp_path / 'bad.yaml'
    path.write_text(raw)
    with pytest.raises((ValueError, yaml.YAMLError)):
        resolve_endpoints(BASE, operator_file=path)


@pytest.mark.parametrize('url', [
    'tcp://localhost:5760', 'tcp-l://0.0.0.0:5760', 'udp://:14555@host:14550',
    'udp-b://@', 'udp-pb://0.0.0.0:1@host:65535', 'serial:///dev/ttyUSB0:2147483647',
    'serial-hwfc:///dev/ttyACM0:57600?ids=1,2', '/dev/ttyUSB0:57600'])
def test_source_supported_bounded_url_forms(url):
    validate_url(url)


@pytest.mark.parametrize('url', [
    '', 'udp://127.0.0.1:14550', 'udp://@host:65536', 'tcp://host:0',
    'serial:///dev/ttyACM0:2147483648', 'tcp://host:1\n', 'http://host:80',
    'serial:///dev/ttyUSB0:1?ids=256,2'])
def test_invalid_uri_no_silent_correction(url):
    with pytest.raises(ValueError):
        validate_url(url)


def test_fixed_discovery_equivalence(tmp_path):
    by_id = tmp_path / 'serial/by-id'
    by_id.mkdir(parents=True)
    device = tmp_path / 'ttyACM1'
    device.touch()
    unknown = by_id / 'unknown-device'
    unknown.symlink_to(device)
    assert discover_device('odom_cube_if00', tmp_path) is None
    assert discover_device('tui_stable_hints', tmp_path) is None
    fallback = tmp_path / 'ttyUSB0'
    fallback.touch()
    assert discover_device('tui_stable_hints', tmp_path) == fallback
    hinted = by_id / 'usb-Pixhawk'
    hinted.symlink_to(device)
    assert discover_device('tui_stable_hints', tmp_path) == hinted
    cube = by_id / 'usb-CubePilot_CubeOrange+_test-if00'
    cube.symlink_to(device)
    assert discover_device('odom_cube_if00', tmp_path) == cube
    assert discover_device('tui_stable_hints', tmp_path) == cube


def test_installed_share_from_unrelated_cwd(tmp_path, monkeypatch):
    share = tmp_path / 'installed/share/mhseals_nav'
    (share / 'config').mkdir(parents=True)
    (share / 'config/endpoint_startup.yaml').write_bytes(BASE.read_bytes())
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.chdir(tmp_path)
    assert resolve_endpoints().inputs[0]['path'] == str(share / 'config/endpoint_startup.yaml')
    with pytest.raises(FileNotFoundError):
        resolve_endpoints(operator_file=tmp_path / 'missing')


def test_actual_launch_adapter_preserves_explicit_alias_provenance(tmp_path, monkeypatch):
    from mhseals_nav.endpoint_configuration import from_launch
    package = ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(ROOT)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    substitutions = ModuleType('launch.substitutions')

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]
    substitutions.LaunchConfiguration = Configuration
    monkeypatch.setitem(sys.modules, 'launch.substitutions', substitutions)
    operator = selected(tmp_path, {'unity': {'port': 7999}})
    result = from_launch({'endpoint_deployment_file': '', 'endpoint_startup_file': str(operator),
                          'endpoint_context': 'tui', 'fcu_url': 'tcp://operator:1234',
                          'gcs_url': '__inherit__', 'ros_ip': '', 'ros_port': ''})
    assert result.context == 'tui'
    assert result.overrides == {'fcu_url': 'tcp://operator:1234'}
    assert result.unity_parameters['ROS_TCP_PORT'] == 7999
    assert result.mavros_parameters(False, tmp_path)[0]['fcu_url'] == 'tcp://operator:1234'


@pytest.mark.parametrize('url', [
    'tcp://host:１２３４', 'serial:///dev/ttyACM0:５７６００',
    'udp://@host:14550/?ids=１,２', 'tcp://host:١٢٣٤'])
def test_unicode_uri_numbers_refuse_in_selected_layer(tmp_path, url):
    """Keep URI numeric grammar compatible with byte-preserved std::stoi input."""
    path = selected(tmp_path, {'fcu_url': url})
    with pytest.raises(ValueError):
        resolve_endpoints(BASE, operator_file=path)
