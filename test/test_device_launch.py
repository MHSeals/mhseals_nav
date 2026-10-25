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
        'camera.launch.py': (None, {'sim': 'false'}),
        'objects.launch.py': (None, {'sim': 'false', 'backend': 'zed'}),
        'sensors.launch.py': (None, {'sim': 'false'}),
        'host_diagnostic': (['ros2', 'run', 'mhseals_testing', 'host_diagnostic'], {}),
    }


def test_only_starts_the_named_parts():
    assert list(expand(device='odroid', only='ekf')) == ['ekf.launch.py']
    assert expand(device='odroid', only='ekf')['ekf.launch.py'][1] == {
        'sim': 'false', 'ekf_startup_context': 'included'}


def test_a_run_part_is_one_ros2_run():
    assert expand(device='operator') == {
        'rtk_base': (['ros2', 'run', 'mhseals_testing', 'rtk_base'], {}),
        'operator_heartbeat': (['ros2', 'run', 'mhseals_testing', 'operator_heartbeat'], {})}


def test_sim_extends_both_computers_without_the_camera():
    profile = device_profile.load('sim')
    assert profile['requires'] == []
    assert [name for name, _ in device_profile.select(profile)] == [
        'odometry', 'ekf', 'mapping', 'navigation', 'mission', 'watchdogs', 'objects', 'sensors']
    assert device_profile.launch_arguments(profile, profile['parts']['odometry']) == {
        'sim': 'true', 'fcu': 'false', 'start_ekf': 'false'}
    assert profile['parts']['objects']['args'] == {'backend': 'sim'}


def test_overrides_are_the_session_layer():
    overrides = '{"parts": {"navigation": {"args": {"limits_file": "/x/nav2.yaml"}}}}'
    parts = expand(device='odroid', only='navigation', overrides=overrides)
    assert parts['navigation.launch.py'][1] == {
        'sim': 'false', 'command_mux': 'true', 'limits_file': '/x/nav2.yaml'}


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
    ({'parts': {'ekf': {'launch': 'nowhere.launch.py'}}}, 'no nowhere.launch.py'),
    # A path to a commander launch must not pass as sensing.
    ({'parts': {'ekf': {'launch': './navigation.launch.py'}}}, r'<name>\.launch\.py'),
    ({'parts': {'ekf': {'launch': '../config/../launch/navigation.launch.py'}}},
     r'<name>\.launch\.py'),
    ({'parts': {'ekf': {'launch': '/opt/x/task.launch.py'}}}, r'<name>\.launch\.py'),
    ({'parts': {'ekf': {'launch': 'navigation.launch.xml'}}}, r'<name>\.launch\.py'),
    ({'parts': {'pwm': {'run': ['mhseals_hardware', 'thruster_pwm_node']}}}, 'with arming'),
    ({'parts': {'mux': {'run': ['mhseals_hardware', 'command_mux']}}}, 'start with arming'),
    ({'parts': {'odd': {'nodes': ['x']}}}, 'exactly one of launch or run'),
    ({'parts': {'ekf': {'enabled': 'no'}}}, 'true or false'),
    ({'parts': {'ekf': {'colour': 'red'}}}, 'keys from'),
    ({'extends': ['jetson']}, 'keys from'),
    ({'args': {'sim': ['x']}}, 'string, number or boolean'),
])
def test_bad_profiles_are_refused(overrides, message):
    with pytest.raises(ValueError, match=message):
        device_profile.load('odroid', overrides=overrides)


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


def watchdog_nodes(**arguments):
    """{node name: parameters} that watchdogs.launch.py starts with ``arguments``."""
    path = ROOT / 'launch/watchdogs.launch.py'
    spec = importlib.util.spec_from_file_location('watchdogs_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = {'sim': 'false', 'require_operator': '', 'fence_frame': '', 'fence_points': '',
                'fcu_watchdog': 'true'}
    context.launch_configurations.update({**defaults, **arguments})
    assert len(module.watchdogs(context)) in (2, 3)          # the launch builds the nodes
    return dict(module.watchdog_parameters(lambda name: {**defaults, **arguments}[name]))


def test_the_boat_runs_all_three_watchdogs_on_their_yaml():
    arguments = expand(device='odroid', only='watchdogs')['watchdogs.launch.py'][1]
    assert arguments == {'sim': 'false'}
    nodes = watchdog_nodes(**arguments)
    assert list(nodes) == ['link_watchdog', 'fence_watchdog', 'fcu_watchdog']
    assert all(parameters == {'use_sim_time': False} for parameters in nodes.values())


def test_the_sim_watchdogs_need_no_operator_or_fcu_and_fence_a_map_box():
    arguments = expand(device='sim', only='watchdogs')['watchdogs.launch.py'][1]
    nodes = watchdog_nodes(**arguments)
    assert list(nodes) == ['link_watchdog', 'fence_watchdog']
    assert nodes['link_watchdog'] == {'use_sim_time': True, 'require_operator': False}
    fence = nodes['fence_watchdog']
    assert fence['fence_frame'] == 'map' and fence['fence_file'] == ''   # an operator file cannot move it
    from mhseals_nav.fence_watchdog import inside, polygon
    box = polygon('map', fence['fence_points'])
    assert inside((0.0, 0.0), box) and inside((12.0, -14.0), box) and not inside((-77.0, -76.0), box)
    assert 'host_diagnostic' not in dict(device_profile.select(device_profile.load('sim')))


@pytest.mark.parametrize('arguments, message', [
    ({'fence_points': '[1, 2, x'}, 'fence_points: not a list'),
    ({'fence_points': '[1, true]'}, 'flat list of numbers'),
    ({'require_operator': 'maybe'}, 'require_operator: invalid launch boolean: maybe'),
])
def test_bad_watchdog_arguments_are_refused(arguments, message):
    with pytest.raises(ValueError, match=message):
        watchdog_nodes(**arguments)
