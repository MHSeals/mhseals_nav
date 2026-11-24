"""device.launch.py and the device profiles in config/devices/."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.substitutions import FindPackageShare
from mhseals_nav import device_profile
import pytest

ROOT = Path(__file__).parents[1]


@pytest.fixture(autouse=True)
def source_tree(monkeypatch, tmp_path):
    monkeypatch.setattr(device_profile, 'share', lambda: ROOT)
    monkeypatch.setattr(FindPackageShare, 'perform', lambda self, context: str(ROOT))
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'config'))
    return tmp_path


def expand(**arguments):
    """{part: (launch file or run argv, arguments)} that device.launch.py starts."""
    path = ROOT / 'launch/device.launch.py'
    spec = importlib.util.spec_from_file_location('device_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = {'only': '', 'overrides': '', 'device_deployment_file': '',
                'device_startup_file': ''}
    context.launch_configurations.update({**defaults, **arguments})

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    found = {}
    for action in module.parts(context):
        if isinstance(action, ExecuteProcess):
            found[text(action.name)] = ([text(part) for part in action.cmd], {})
            continue
        assert isinstance(action, GroupAction)
        (include,) = [entity for entity in action.get_sub_entities()
                      if isinstance(entity, IncludeLaunchDescription)]
        include.launch_description_source.get_launch_description(context)  # the file exists
        found[Path(include.launch_description_source.location).name] = (
            None, {text(k): text(v) for k, v in include.launch_arguments})
    return found


def test_jetson_runs_camera_zed_perception_and_the_lidar():
    assert expand(device='jetson') == {
        'camera.launch.py': (None, {'sim': 'false', 'camera_model': 'zed2i'}),
        'objects.launch.py': (None, {'sim': 'false', 'backend': 'zed'}),
        'sensors.launch.py': (None, {'sim': 'false'}),   # the lidar part
        'host_diagnostic': (['ros2', 'run', 'mhseals_nav', 'host_diagnostic'], {}),
    }


def test_localization_is_one_part_with_the_ekfs_on_the_boat_and_in_the_sim():
    assert expand(device='odroid', only='localization') == {
        'odom.launch.py': (None, {'sim': 'false', 'fcu': 'true'})}
    assert expand(device='sim', only='localization') == {'odom.launch.py': (None, {
        'sim': 'true', 'fcu': 'false', 'ekf_startup_context': 'standalone'})}


def test_a_run_part_is_one_ros2_run():
    assert expand(device='operator') == {
        'rtk': (['ros2', 'run', 'mhseals_nav', 'rtk', 'base'], {})}


def test_sim_extends_both_computers_with_its_bridge_for_the_camera_and_lidar():
    profile = device_profile.load('sim')
    assert profile['requires'] == []
    assert [name for name, _ in device_profile.select(profile)] == [
        'localization', 'mapping', 'navigation', 'mission', 'objects', 'sim_bridge']
    assert profile['parts']['objects']['args'] == {'backend': 'sim'}
    # crane_ml passes the endpoint's port as this part's ros_port.
    bridge = expand(device='sim', only='sim_bridge',
                    overrides='{"parts": {"sim_bridge": {"args": {"ros_port": "10001"}}}}')
    assert bridge == {'sensors.launch.py': (None, {'sim': 'true', 'ros_port': '10001'})}


def test_device_parts_lists_what_only_takes():
    from mhseals_nav.device_parts import lines
    assert [line.split('\t')[0] for line in lines('odroid')] == [
        'localization', 'mapping', 'navigation', 'mission']
    assert lines('operator') == ['rtk\tros2 run mhseals_nav rtk base: rtk_base']
    assert {line.split('\t')[0] for line in lines()} == {
        'localization', 'mapping', 'navigation', 'mission', 'camera', 'objects', 'lidar',
        'host_diagnostic', 'rtk', 'sim_bridge'}


def test_overrides_are_the_session_layer():
    overrides = '{"parts": {"navigation": {"args": {"hardware_config": "/x/hardware.yaml"}}}}'
    parts = expand(device='odroid', only='navigation', overrides=overrides)
    assert parts['navigation.launch.py'][1] == {
        'sim': 'false', 'hardware_config': '/x/hardware.yaml'}


@pytest.mark.parametrize('layer', ['{"parts": {"sensors": {"args": {"ros_port": "1"}}}}',
                                   '{"parts": {"sensors": {"enabled": false}}}'])
def test_overrides_for_a_part_the_profile_lacks_name_it(layer):
    with pytest.raises(ValueError, match=r"^overrides: no part 'sensors' in sim \(parts: "):
        device_profile.load('sim', overrides=layer)


def test_deployment_and_operator_layers_apply_in_order(source_tree):
    deployment = source_tree / 'ws' / 'config' / 'devices'
    operator = source_tree / 'config' / 'mhseals' / 'devices'
    for directory, backend in ((deployment, 'one'), (operator, 'two')):
        directory.mkdir(parents=True)
        layer = f'parts: {{objects: {{args: {{backend: {backend}}}}}}}\n'
        (directory / 'jetson.yaml').write_text(layer)
    assert device_profile.load('jetson')['parts']['objects']['args'] == {'backend': 'two'}
    assert device_profile.load('jetson', overrides={'parts': {'camera': {'enabled': False}}})[
        'parts']['camera']['enabled'] is False


def test_a_layer_replaces_launch_with_run_and_merges_args_per_key():
    base = {'requires': ['a'], 'args': {'x': 1},
            'parts': {'p': {'launch': 'odom.launch.py', 'args': {'k': 1, 'j': 2}}}}
    run = device_profile.merge(base, {'parts': {'p': {'run': ['pkg', 'exe']}}})
    assert run['parts']['p'] == {'run': ['pkg', 'exe'], 'args': {'k': 1, 'j': 2}}
    args = device_profile.merge(base, {'args': {'y': 2}, 'parts': {'p': {'args': {'k': 3}}}})
    assert args == {'requires': ['a'], 'args': {'x': 1, 'y': 2},
                    'parts': {'p': {'launch': 'odom.launch.py', 'args': {'k': 3, 'j': 2}}}}
    assert base['parts']['p'] == {'launch': 'odom.launch.py', 'args': {'k': 1, 'j': 2}}


def test_every_shipped_profile_loads():
    assert device_profile.available() == ['jetson', 'odroid', 'operator', 'sim']
    for name in device_profile.available():
        assert device_profile.load(name)['parts']


def test_no_launch_file_here_starts_the_thruster_driver_or_the_mux():
    for path in (ROOT / 'launch').glob('*.launch.py'):
        text = path.read_text()
        assert "'thruster_pwm_node'" not in text and "executable='command_mux'" not in text, path


@pytest.mark.parametrize('overrides, message', [
    ({'parts': {'navigation': {'kind': 'sensing'}}}, 'can command motion; set kind: commander'),
    ({'parts': {'localization': {'launch': 'nowhere.launch.py'}}}, 'no nowhere.launch.py'),
    # A path to a commander launch must not pass as sensing.
    ({'parts': {'localization': {'launch': './navigation.launch.py'}}}, r'<name>\.launch\.py'),
    ({'parts': {'localization': {'launch': '../config/../launch/navigation.launch.py'}}},
     r'<name>\.launch\.py'),
    ({'parts': {'localization': {'launch': '/opt/x/task.launch.py'}}}, r'<name>\.launch\.py'),
    ({'parts': {'localization': {'launch': 'navigation.launch.xml'}}}, r'<name>\.launch\.py'),
    ({'parts': {'pwm': {'run': ['mhseals_hardware', 'thruster_pwm_node']}}}, 'drive.launch.py'),
    ({'parts': {'mux': {'run': ['mhseals_hardware', 'command_mux']}}}, 'drive.launch.py'),
    ({'parts': {'odd': {'launch': 'odom.launch.py', 'run': ['mhseals_nav', 'x']}}},
     'exactly one of launch or run'),
    ({'parts': {'gps': {'run': ['some_driver', 'rtk']}}}, "another package's"),
    ({'parts': {'localization': {'enabled': 'no'}}}, 'true or false'),
    ({'parts': {'localization': {'colour': 'red'}}}, 'keys from'),
    ({'extends': ['jetson']}, 'keys from'),
    ({'args': {'sim': ['x']}}, 'string, number or boolean'),
    ({'parts': {'localization': {'with': 'mapping'}}}, 'with: list of names'),
    ({'parts': {'localization': {'with': ['ghost']}}}, "'ghost' is not another part"),
    ({'parts': {'localization': {'with': ['localization']}}}, "'localization' is not another part"),
    ({'parts': {'localization': {'with': ['navigation']}}}, 'can command motion'),
    ({'parts': {'localization': {'with': ['mapping']}, 'mapping': {'with': ['localization']}}},
     'brings parts of its own'),
])
def test_bad_profiles_are_refused(overrides, message):
    with pytest.raises(ValueError, match=message):
        device_profile.load('odroid', overrides=overrides)


def test_check_refuses_a_one_element_run_of_another_package():
    with pytest.raises(ValueError, match="another package's"):
        device_profile.check({'parts': {'x': {'run': ['some_driver']}}}, 'p')


def test_only_refuses_unknown_and_disabled_parts():
    profile = device_profile.load('sim')
    with pytest.raises(ValueError, match="no part 'ghost'"):
        device_profile.select(profile, 'ghost')
    with pytest.raises(ValueError, match="'camera' is disabled"):
        device_profile.select(profile, 'camera')


def test_bad_json_overrides_and_unknown_profiles_are_refused():
    with pytest.raises(ValueError, match='overrides: not JSON'):
        device_profile.load('jetson', overrides='{parts')
    with pytest.raises(ValueError, match="no device profile 'xavier'"):
        device_profile.load('xavier')


def test_an_extends_cycle_is_refused(source_tree, monkeypatch):
    root = source_tree / 'pkg'
    (root / 'config' / 'devices').mkdir(parents=True)
    (root / 'launch').mkdir()
    (root / 'config' / 'devices' / 'a.yaml').write_text('extends: [b]\n')
    (root / 'config' / 'devices' / 'b.yaml').write_text('extends: [a]\n')
    monkeypatch.setattr(device_profile, 'share', lambda: root)
    with pytest.raises(ValueError, match='extends itself: a -> b -> a'):
        device_profile.load('a')


def test_the_sim_has_no_host_diagnostic():
    assert 'host_diagnostic' not in dict(device_profile.select(device_profile.load('sim')))

