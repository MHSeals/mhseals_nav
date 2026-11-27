"""device.launch.py and the device profiles in config/devices/, run in the launch harness."""

from pathlib import Path

from launch_harness import harness, included  # noqa: F401
from mhseals_nav import device_profile, endpoint_configuration
from mhseals_nav.lidar_configuration import resolve_lidar
import pytest
import test_lidar_configuration as calibration

ROOT = Path(__file__).parents[1]


@pytest.fixture(autouse=True)
def source_tree(monkeypatch, tmp_path):
    monkeypatch.setattr(device_profile, 'share', lambda: ROOT)
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'config'))
    return tmp_path


@pytest.fixture
def expand(harness):  # noqa: F811
    """Give [(launch file or run argv, arguments)] that device.launch.py starts."""
    def start(**arguments):
        found = []
        for record in harness.run('device.launch.py', **arguments):
            if record.kind == 'ExecuteProcess':
                assert record['cmd'][:2] == ['ros2', 'run'] and record['name']
                found.append((record['cmd'], {}))
            else:
                name, passed = included(record)
                assert (ROOT / 'launch' / name).is_file()
                found.append((name, passed))
        return found
    return start


def test_jetson_runs_perception_on_the_zed_and_the_velodyne(expand):
    assert expand(device='jetson') == [
        ('perception.launch.py', {'sim': 'false', 'camera': 'zed', 'camera_model': 'zed2i',
                                  'lidar': 'velodyne'}),
        (['ros2', 'run', 'mhseals_nav', 'host_diagnostic'], {}),
    ]


def test_odroid_runs_localization_on_the_fcu_mapping_navigation_and_the_mission(expand):
    assert expand(device='odroid') == [
        ('localization.launch.py', {'sim': 'false', 'source': 'fcu'}),
        ('navigation.launch.py', {'sim': 'false', 'nav2': 'false', 'mapping': 'true'}),
        ('navigation.launch.py', {'sim': 'false'}),
        ('mission.launch.py', {'sim': 'false'}),
    ]


def test_localization_takes_the_fcu_on_the_boat_and_nothing_in_the_sim(expand):
    assert expand(device='odroid', only='localization') == [
        ('localization.launch.py', {'sim': 'false', 'source': 'fcu'})]
    assert expand(device='sim', only='localization') == [
        ('localization.launch.py', {'sim': 'true', 'source': 'none'})]


def test_the_sim_perceives_the_simulator_through_its_bridge(expand):
    assert expand(device='sim', only='perception') == [('perception.launch.py', {
        'sim': 'true', 'camera': 'sim', 'camera_model': 'zed2i', 'lidar': 'sim'})]
    # crane_ml passes the endpoint's port as this part's ros_port.
    bridge = expand(device='sim', only='sim_bridge',
                    overrides='{"parts": {"sim_bridge": {"args": {"ros_port": "10001"}}}}')
    assert bridge == [('sim_bridge.launch.py', {'sim': 'true', 'ros_port': '10001'})]


def test_a_run_part_is_one_ros2_run(expand):
    assert expand(device='operator') == [(['ros2', 'run', 'mhseals_nav', 'rtk', 'base'], {})]


def test_sim_extends_both_computers_and_adds_its_bridge():
    profile = device_profile.load('sim')
    assert profile['requires'] == []
    assert [name for name, _ in device_profile.select(profile)] == [
        'localization', 'mapping', 'navigation', 'mission', 'perception', 'sim_bridge']
    assert profile['parts']['localization']['args'] == {'source': 'none'}
    assert profile['parts']['perception']['launch'] == 'perception.launch.py'   # jetson's
    assert profile['parts']['host_diagnostic']['enabled'] is False


@pytest.mark.parametrize('device', ['jetson', 'odroid', 'operator', 'sim'])
def test_every_part_starts_with_arguments_its_launch_file_declares(
        harness, monkeypatch, tmp_path, device):  # noqa: F811
    """Each part's own args are declared by its file, and the file runs with them.

    With no task the mission part starts nothing.
    """
    localization = harness.load('localization.launch.py')
    monkeypatch.setattr(localization, 'description_xml', lambda path: '<robot/>')   # no xacro
    share = calibration.share.__wrapped__(tmp_path)
    monkeypatch.setattr(harness.load('perception.launch.py'), 'resolve_lidar',
                        lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    real = endpoint_configuration.discover_device
    monkeypatch.setattr(endpoint_configuration, 'discover_device',
                        lambda strategy, _=None: real(strategy, tmp_path))
    profile = device_profile.load(device)
    for part, spec in device_profile.select(profile):
        if 'run' in spec:
            continue
        undeclared = set(spec.get('args', {})) - set(harness.declared(spec['launch']))
        assert not undeclared, f'{device}.{part}: {spec["launch"]} declares no {undeclared}'
        arguments = device_profile.launch_arguments(profile, spec)
        assert harness.run(spec['launch'], **arguments) or spec['launch'] == 'mission.launch.py'


def test_device_parts_lists_what_only_takes():
    from mhseals_nav.device_parts import lines
    assert [line.split('\t')[0] for line in lines('odroid')] == [
        'localization', 'mapping', 'navigation', 'mission']
    (rtk,) = lines('operator')
    assert rtk.startswith('rtk\tRTK corrections')
    assert rtk.endswith('(ros2 run mhseals_nav rtk base)')
    assert {line.split('\t')[0] for line in lines()} == {
        'localization', 'mapping', 'navigation', 'mission', 'perception', 'host_diagnostic',
        'rtk', 'sim_bridge'}
    # One line per way a part starts, with the devices that start it so.
    localization = [line for line in lines() if line.startswith('localization\t')]
    assert [line.rsplit('(', 1)[1] for line in localization] == [
        'odroid; localization.launch.py source:=fcu)', 'sim; localization.launch.py source:=none)']
    assert lines('--devices') == [
        'jetson\tperception, host_diagnostic',
        'odroid\tlocalization, mapping, navigation, mission', 'operator\trtk',
        'sim\tlocalization, mapping, navigation, mission, perception, sim_bridge']


def test_overrides_are_the_session_layer(expand):
    overrides = '{"parts": {"navigation": {"args": {"hardware_config": "/x/hardware.yaml"}}}}'
    assert expand(device='odroid', only='navigation', overrides=overrides) == [(
        'navigation.launch.py', {'sim': 'false', 'hardware_config': '/x/hardware.yaml'})]


@pytest.mark.parametrize('layer', ['{"parts": {"lidar": {"args": {"ros_port": "1"}}}}',
                                   '{"parts": {"camera": {"enabled": false}}}'])
def test_overrides_for_a_part_the_profile_lacks_name_it(layer):
    with pytest.raises(ValueError, match=r"^overrides: no part '(lidar|camera)' in sim \(parts: "):
        device_profile.load('sim', overrides=layer)


def test_deployment_and_operator_layers_apply_in_order(source_tree):
    deployment = source_tree / 'ws' / 'config' / 'devices'
    operator = source_tree / 'config' / 'mhseals' / 'devices'
    for directory, camera in ((deployment, 'sim'), (operator, 'none')):
        directory.mkdir(parents=True)
        layer = f'parts: {{perception: {{args: {{camera: {camera}}}}}}}\n'
        (directory / 'jetson.yaml').write_text(layer)
    assert device_profile.load('jetson')['parts']['perception']['args'] == {
        'camera': 'none', 'camera_model': 'zed2i', 'lidar': 'velodyne'}
    assert device_profile.load('jetson', overrides={'parts': {'perception': {'enabled': False}}})[
        'parts']['perception']['enabled'] is False


def test_a_layer_replaces_launch_with_run_and_merges_args_per_key():
    base = {'requires': ['a'], 'args': {'x': 1},
            'parts': {'p': {'launch': 'localization.launch.py', 'args': {'k': 1, 'j': 2}}}}
    run = device_profile.merge(base, {'parts': {'p': {'run': ['pkg', 'exe']}}})
    assert run['parts']['p'] == {'run': ['pkg', 'exe'], 'args': {'k': 1, 'j': 2}}
    args = device_profile.merge(base, {'args': {'y': 2}, 'parts': {'p': {'args': {'k': 3}}}})
    assert args == {'requires': ['a'], 'args': {'x': 1, 'y': 2},
                    'parts': {'p': {'launch': 'localization.launch.py', 'args': {'k': 3, 'j': 2}}}}
    assert base['parts']['p'] == {'launch': 'localization.launch.py', 'args': {'k': 1, 'j': 2}}


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
    ({'parts': {'localization': {'launch': '/opt/x/mission.launch.py'}}}, r'<name>\.launch\.py'),
    ({'parts': {'localization': {'launch': 'navigation.launch.xml'}}}, r'<name>\.launch\.py'),
    ({'parts': {'pwm': {'run': ['mhseals_hardware', 'thruster_pwm_node']}}}, 'drive.launch.py'),
    ({'parts': {'mux': {'run': ['mhseals_hardware', 'command_mux']}}}, 'drive.launch.py'),
    ({'parts': {'odd': {'launch': 'localization.launch.py', 'run': ['mhseals_nav', 'x']}}},
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
    with pytest.raises(ValueError, match="'host_diagnostic' is disabled"):
        device_profile.select(profile, 'host_diagnostic')


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


@pytest.mark.parametrize('args,refused', [
    ({'nav2': 'false', 'mapping': 'true'}, False), ({'nav2': False}, False),
    ({'nav2': '0'}, False),
    ({}, True), ({'nav2': 'true'}, True), ({'mapping': 'true'}, True)])
def test_navigation_without_nav2_is_sensing_and_with_it_a_commander(args, refused):
    """A navigation part needs kind: commander unless nav2 is off (mapping only)."""
    layer = {'parts': {'survey': {'launch': 'navigation.launch.py', 'args': args}}}
    if refused:
        with pytest.raises(ValueError, match='survey: navigation.launch.py can command motion'):
            device_profile.load('odroid', overrides=layer)
    else:
        assert 'kind' not in device_profile.load('odroid', overrides=layer)['parts']['survey']
    commander = {'parts': {'survey': {**layer['parts']['survey'], 'kind': 'commander'}}}
    assert device_profile.load('odroid', overrides=commander)['parts']['survey']


def test_the_mapping_part_is_sensing_so_another_part_may_bring_it():
    profile = device_profile.load('odroid', overrides={
        'parts': {'localization': {'with': ['mapping']}}})
    assert [name for name, _ in device_profile.select(profile, 'localization')] == [
        'localization', 'mapping']
    assert not device_profile.commands(profile['parts']['mapping'])
    assert device_profile.commands(profile['parts']['navigation'])
    assert device_profile.commands(profile['parts']['mission'])
    assert device_profile.commands({'launch': 'mission.launch.py', 'args': {'nav2': 'false'}})
