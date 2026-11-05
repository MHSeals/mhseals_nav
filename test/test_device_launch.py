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
        'sensors.launch.py': (None, {'sim': 'false'}),
        'host_diagnostic': (['ros2', 'run', 'mhseals_nav', 'host_diagnostic'], {}),
    }


def test_only_starts_the_named_parts():
    assert list(expand(device='odroid', only='ekf')) == ['ekf.launch.py']
    assert expand(device='odroid', only='ekf')['ekf.launch.py'][1] == {
        'sim': 'false', 'ekf_startup_context': 'included'}


def test_the_boat_watchdogs_start_with_odometry_on_the_boat():
    parts = expand(device='odroid', only='odometry')
    assert list(parts) == ['odom.launch.py', 'watchdogs.launch.py']
    assert parts['watchdogs.launch.py'][1] == {'sim': 'false'}     # no operator or lease asked
    profile = device_profile.load('odroid')
    names = [name for name, _ in device_profile.select(profile, 'odometry,watchdogs')]
    assert names == ['odometry', 'watchdogs']                      # once, never a second copy
    assert [name for name, _ in device_profile.select(profile, 'ekf')] == ['ekf']
    off = device_profile.load('odroid', overrides={'parts': {'watchdogs': {'enabled': False}}})
    assert [name for name, _ in device_profile.select(off, 'odometry')] == ['odometry']


def test_the_sim_starts_odometry_without_its_watchdogs():
    profile = device_profile.load('sim')
    assert [name for name, _ in device_profile.select(profile, 'odometry')] == ['odometry']
    assert list(expand(device='sim', only='odometry')) == ['odom.launch.py']


def test_a_run_part_is_one_ros2_run():
    assert expand(device='operator') == {
        'rtk_base': (['ros2', 'run', 'mhseals_testing', 'rtk_base'], {}),
        'operator_heartbeat': (['ros2', 'run', 'mhseals_nav', 'operator_heartbeat'], {})}


def test_sim_extends_both_computers_without_the_camera():
    profile = device_profile.load('sim')
    assert profile['requires'] == []
    assert [name for name, _ in device_profile.select(profile)] == [
        'odometry', 'ekf', 'mapping', 'navigation', 'mission', 'watchdogs', 'objects', 'sensors']
    assert device_profile.launch_arguments(profile, profile['parts']['odometry']) == {
        'sim': 'true', 'fcu': 'false', 'start_ekf': 'false'}
    assert profile['parts']['objects']['args'] == {'backend': 'sim'}


def test_approach_guard_enforces_in_sim_and_only_logs_on_the_boat():
    """The sim profile is the one place enforce is set; the boat keeps watchdogs.yaml's false."""
    from mhseals_nav.yaml_params import config_path
    import yaml
    sim, boat = (device_profile.load(name) for name in ('sim', 'odroid'))
    assert device_profile.launch_arguments(sim, sim['parts']['navigation']) == {
        'sim': 'true', 'command_mux': 'true', 'approach_enforce': 'true'}
    assert 'approach_enforce' not in device_profile.launch_arguments(
        boat, boat['parts']['navigation'])
    tune = yaml.safe_load(config_path('watchdogs.yaml').read_text())['tune']
    assert tune['approach_guard']['enforce'] is False


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
    ({'parts': {'beat': {'run': ['mhseals_testing', 'operator_heartbeat']}}}, "another package's"),
    ({'parts': {'gps': {'run': ['some_driver', 'rtk_base']}}}, "another package's"),
    ({'parts': {'ekf': {'enabled': 'no'}}}, 'true or false'),
    ({'parts': {'ekf': {'colour': 'red'}}}, 'keys from'),
    ({'extends': ['jetson']}, 'keys from'),
    ({'args': {'sim': ['x']}}, 'string, number or boolean'),
    ({'parts': {'odometry': {'with': 'watchdogs'}}}, 'with: list of names'),
    ({'parts': {'odometry': {'with': ['ghost']}}}, "'ghost' is not another part"),
    ({'parts': {'odometry': {'with': ['odometry']}}}, "'odometry' is not another part"),
    ({'parts': {'odometry': {'with': ['navigation']}}}, 'can command motion'),
    ({'parts': {'ekf': {'with': ['odometry']}}}, 'brings parts of its own'),
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
    defaults = {'sim': 'false', 'require_operator': '', 'fence_file': '', 'fcu_watchdog': 'true'}
    context.launch_configurations.update({**defaults, **arguments})
    assert len(module.watchdogs(context)) in (3, 4)          # the launch builds the nodes
    return dict(module.watchdog_parameters(lambda name: {**defaults, **arguments}[name]))


def test_the_boat_runs_all_four_watchdogs_on_their_yaml():
    arguments = expand(device='odroid', only='watchdogs')['watchdogs.launch.py'][1]
    assert arguments == {'sim': 'false'}
    nodes = watchdog_nodes(**arguments)
    assert list(nodes) == ['link_watchdog', 'fence_watchdog', 'fcu_watchdog',
                           'ekf_input_watchdog']
    assert all(parameters == {'use_sim_time': False} for parameters in nodes.values())


def test_the_sim_watchdogs_need_no_operator_or_fcu_and_fence_a_map_box():
    arguments = expand(device='sim', only='watchdogs')['watchdogs.launch.py'][1]
    nodes = watchdog_nodes(**arguments)
    assert list(nodes) == ['link_watchdog', 'fence_watchdog', 'ekf_input_watchdog']
    assert nodes['ekf_input_watchdog'] == {'use_sim_time': True}
    assert nodes['link_watchdog'] == {'use_sim_time': True, 'require_operator': False}
    fence = nodes['fence_watchdog']
    # The package's crane box, never the operator's file.
    assert fence == {'use_sim_time': True, 'fence_file': 'fences/crane.yaml'}
    from mhseals_nav.fence import inside, load_fence, polygon
    box = polygon('map', load_fence(fence['fence_file'])[0]['points'])
    assert inside((0.0, 0.0), box) and inside((12.0, -14.0), box)
    assert not inside((-77.0, -76.0), box)
    assert 'host_diagnostic' not in dict(device_profile.select(device_profile.load('sim')))


@pytest.mark.parametrize('arguments, message', [
    ({'require_operator': 'maybe'}, 'require_operator: invalid launch boolean: maybe'),
])
def test_bad_watchdog_arguments_are_refused(arguments, message):
    with pytest.raises(ValueError, match=message):
        watchdog_nodes(**arguments)
