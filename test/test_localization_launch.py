"""localization.launch.py: the pose source, the robot TF and the EKFs, in the launch harness."""
from pathlib import Path
import shutil

from launch_harness import executables, harness, named  # noqa: F401
from mhseals_nav import endpoint_configuration as endpoint
from mhseals_nav import mavros_configuration, odometry_configuration
import pytest
from test_mavros_configuration import selected
import yaml

ROOT = Path(__file__).parents[1]
FCU = ['mavros_node', 'mavros_odometry', 'relay', 'mavros_setup']
CORE = ['robot_state_publisher', 'navsat_transform_node', 'ekf_node', 'ekf_node']


@pytest.fixture
def run(harness, tmp_path, monkeypatch):  # noqa: F811
    """Start localization.launch.py with a plain URDF (no xacro needed).

    FCU discovery looks under run.dev (empty), never this machine's /dev.
    """
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="test"/>')
    dev = tmp_path / 'dev'
    dev.mkdir()
    real = endpoint.discover_device
    monkeypatch.setattr(endpoint, 'discover_device', lambda strategy, _=None: real(strategy, dev))

    def start(**arguments):
        return harness.run('localization.launch.py', **{'urdf_file': str(urdf), **arguments})
    start.harness, start.dev = harness, dev
    return start


def layer(tmp_path, text, name='operator.yaml'):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def test_source_fcu_adds_mavros_its_relays_and_setup_before_the_ekfs(run):
    fcu = run()
    assert executables(fcu) == FCU + CORE
    mavros = named(fcu, 'mavros_node')
    # An FCU reboot drops the USB device and MAVROS exits; it must come back.
    assert mavros['respawn'] is True and mavros['respawn_delay'] == 2.0
    assert mavros['parameters'][0] == str(ROOT / 'config/mavros.yaml')
    assert named(fcu, 'mavros_imu_relay')['arguments'] == ['/mavros/imu/data', '/imu/raw']
    assert named(fcu, 'mavros_odom_adapter')['executable'] == 'mavros_odometry'
    assert executables(run(source='none')) == CORE


def test_the_ekfs_and_navsat_use_the_dual_ekf_topics(run):
    nodes = run(source='none')
    navsat = dict(named(nodes, 'navsat_transform')['remappings'])
    assert navsat['odometry/filtered'] == '/odom/global'   # the global EKF feeds navsat
    assert navsat['imu'] == navsat['imu/data'] == '/imu/raw'
    assert (navsat['gps/fix'], navsat['odometry/gps']) == ('/gps/fix', '/odom/gps')
    assert named(nodes, 'ekf_local')['remappings'] == [('odometry/filtered', '/odom/local')]
    assert named(nodes, 'ekf_global')['remappings'] == [('odometry/filtered', '/odom/global')]


@pytest.mark.parametrize('sim', ['true', 'false'])
@pytest.mark.parametrize('source', ['fcu', 'none'])
def test_sim_reaches_every_clock_as_a_boolean(run, sim, source):
    nodes = run(source=source, sim=sim)
    clocks = [named(nodes, 'robot_state_publisher')['parameters'][0]['use_sim_time']]
    clocks += [named(nodes, name)['parameters'][0]['use_sim_time']
               for name in ('ekf_local', 'ekf_global', 'navsat_transform')]
    if source == 'fcu':
        clocks.append(named(nodes, 'mavros_node')['parameters'][1]['use_sim_time'])
    assert clocks == [sim == 'true'] * len(clocks)


@pytest.mark.parametrize('source,delay', [('fcu', 7.8), ('none', 1.2)])
def test_ekf_delay_follows_the_source(run, tmp_path, source, delay):
    """MAVROS needs time to connect: with source fcu the EKFs wait ekf_delay_included."""
    operator = layer(tmp_path, 'ekf_delay_standalone: 1.2\nekf_delay_included: 7.8\n')
    nodes = run(source=source, odometry_startup_file=operator)
    assert [n.timer for n in nodes if n['executable'] == 'ekf_node'] == [delay, delay]
    assert named(nodes, 'navsat_transform').timer is None
    assert [n.timer for n in run(source=source, odometry_startup_file=operator,
                                 ekf_delay='3.5') if n['executable'] == 'ekf_node'] == [3.5, 3.5]


def test_packaged_delay_is_six_seconds_after_mavros_and_none_without(run):
    assert {n.timer for n in run() if n['executable'] == 'ekf_node'} == {6.0}
    assert {n.timer for n in run(source='none')} == {None}


@pytest.mark.parametrize('key,value,target', [
    ('mavros_velocity_source', 'fcu', 'velocity_source'),
    ('mavros_velocity_stddev', .8, 'velocity_stddev'),
    ('mavros_velocity_max_age', 2.3, 'gps_velocity_max_age'),
    ('mavros_velocity_min_stddev', .2, 'gps_velocity_min_stddev'),
    ('mavros_velocity_no_fix_stddev', 3., 'velocity_stddev_nofix'),
    ('mavros_velocity_min_fix_type', 5, 'gps_velocity_min_fix_type'),
    ('mavros_velocity_max_acc', .5, 'gps_velocity_max_acc'),
])
def test_each_odometry_yaml_field_reaches_the_adapter(run, tmp_path, key, value, target):
    nodes = run(odometry_startup_file=layer(tmp_path, f'{key}: {value}\n'))
    assert named(nodes, 'mavros_odom_adapter')['parameters'][0][target] == value


@pytest.mark.parametrize('alias', [False, True])
def test_integer_spelled_values_reach_the_nodes_as_doubles(run, tmp_path, alias):
    operator = layer(tmp_path, 'robot_state_publish_frequency: 50\nmavros_velocity_stddev: 1\n')
    aliases = {'robot_state_publish_frequency': '60', 'mavros_velocity_stddev': '2'}
    nodes = run(odometry_startup_file=operator, **(aliases if alias else {}))
    frequency = named(nodes, 'robot_state_publisher')['parameters'][0]['publish_frequency']
    stddev = named(nodes, 'mavros_odom_adapter')['parameters'][0]['velocity_stddev']
    assert (type(frequency), type(stddev)) == (float, float)
    assert (frequency, stddev) == ((60., 2.) if alias else (50., 1.))


@pytest.mark.parametrize('arguments', [
    {'source': 'sitl'}, {'sim': 'yes'}, {'sim': ''}, {'sim': 'garbage'},
    {'mavros_velocity_stddev': 'false'}, {'ekf_delay': 'nan'}])
def test_invalid_arguments_fail_before_any_node(run, arguments):
    with pytest.raises(ValueError):
        run(**arguments)
    assert run.harness.built == []


def test_an_invalid_odometry_layer_fails_before_any_node(run, tmp_path):
    with pytest.raises(ValueError):
        run(odometry_startup_file=layer(tmp_path, 'mavros_velocity_stddev: false\n'))
    with pytest.raises(OSError):
        run(odometry_startup_file=str(tmp_path / 'missing.yaml'))
    assert run.harness.built == []


@pytest.mark.parametrize('name', ['ekf_global', 'navsat_transform'])
@pytest.mark.parametrize('bad', ['invalid_double', 'graph', 'duplicate', 'missing'])
def test_an_invalid_ekf_base_fails_before_any_ekf(run, tmp_path, name, bad):
    """An overlay cannot conceal an invalid base."""
    raw = (ROOT / 'config' / (name + '.yaml')).read_text()
    raw = {'invalid_double': raw.replace('frequency: 20.0', 'frequency: .nan'),
           'graph': raw.replace('map_frame: map', 'map_frame: other'),
           'duplicate': raw.replace('frequency: 20.0', 'frequency: 20.0\n    frequency: 30.0'),
           'missing': None}[bad]
    base = tmp_path / (name + '.yaml')
    if raw is not None:
        base.write_text(raw)
    overlay = layer(tmp_path, name + ': {map_frame: map}', 'overlay.yaml')
    with pytest.raises((ValueError, OSError)):
        run(source='none', ekf_overlay_file=overlay, **{name + '_config_file': str(base)})
    assert not [n for n in run.harness.built if n['package'] == 'robot_localization']


@pytest.mark.parametrize('field', ['odom0_queue_size', 'imu0_queue_size'])
@pytest.mark.parametrize('value', [2**63, 2**100, 0, True, 1.0])
def test_an_invalid_queue_size_fails(run, tmp_path, field, value):
    overlay = layer(tmp_path, yaml.safe_dump({'ekf_local': {field: value}}), 'overlay.yaml')
    with pytest.raises(ValueError, match='positive signed int64'):
        run(source='none', ekf_overlay_file=overlay)


def test_ekf_parameters_are_validated_copies_and_later_edits_reach_the_next_launch(run, tmp_path):
    overlay = tmp_path / 'overlay.yaml'
    overlay.write_text('ekf_global: {frequency: 37, transform_time_offset: -0.2}')
    old = named(run(source='none', ekf_overlay_file=str(overlay)),
                'ekf_global')['parameters'][0]
    assert old['frequency'] == 37. and type(old['frequency']) is float
    assert old['use_sim_time'] is False   # sim defaults to false: the boat never waits for /clock
    overlay.write_text('ekf_global: {frequency: 43}')
    assert old['frequency'] == 37.   # a node is never handed a path that re-reads new bytes
    fresh = named(run(source='none', ekf_overlay_file=str(overlay)), 'ekf_global')
    assert fresh['parameters'][0]['frequency'] == 43.


def test_installed_share_defaults_outside_the_source_tree(run, installed, tmp_path, monkeypatch):
    shutil.copytree(ROOT / 'config', installed / 'config', dirs_exist_ok=True)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('HOME', str(tmp_path))
    declared = run.harness.declared('localization.launch.py')
    assert declared['ekf_overlay_file'] == ''
    assert declared['ekf_local_config_file'] == str(installed / 'config/ekf_local.yaml')
    nodes = run(source='none')
    ekfs = [n for n in nodes if n['package'] == 'robot_localization']
    assert len(ekfs) == 3
    assert all(n['parameters'][0]['use_sim_time'] is False for n in ekfs)


def test_robot_description_follows_the_urdf_file(run, tmp_path):
    operator = layer(tmp_path, 'robot_state_publish_frequency: 42.0\n')
    robot = named(run(source='none', odometry_startup_file=operator), 'robot_state_publisher')
    assert robot['parameters'][0]['robot_description'] == '<robot name="test"/>'
    assert robot['parameters'][0]['publish_frequency'] == 42.
    with pytest.raises(ValueError, match='.xacro or .urdf'):
        run(source='none', urdf_file=str(tmp_path / 'robot.sdf'))


def test_the_packaged_xacro_gives_the_lidar_link(harness):  # noqa: F811
    """Needs xacro (skips without ROS)."""
    nodes = harness.run('localization.launch.py', source='none')
    description = named(nodes, 'robot_state_publisher')['parameters'][0]['robot_description']
    assert 'lidar_link' in description


def test_every_alias_argument_defaults_empty_so_the_yaml_decides(run):
    declared = run.harness.declared('localization.launch.py')
    for name in (*mavros_configuration.ALIASES, *odometry_configuration.ALIASES,
                 *odometry_configuration.LAYER_ARGUMENTS, *endpoint.LAYER_ARGUMENTS, 'fcu_url'):
        assert declared[name] == '', name
    assert declared['gcs_url'] == endpoint.UNSET_GCS
    assert (declared['source'], declared['sim']) == ('fcu', 'false')


# MAVROS message rates (mavros_setup)

@pytest.mark.parametrize('selection', ['', '/missing/rates.yaml', 'invalid'])
def test_a_bad_rate_selection_fails_before_mavros_setup(run, tmp_path, selection):
    if selection == 'invalid':
        selection = layer(tmp_path, 'mavros_setup: []', 'invalid.yaml')
    with pytest.raises((ValueError, OSError)):
        run(mavros_setup_params_file=selection)
    assert 'mavros_setup' not in executables(run.harness.built)


def test_an_invalid_rate_alias_fails_before_mavros_setup(run, tmp_path):
    with pytest.raises(ValueError):
        run(mavros_setup_params_file=selected(tmp_path, ['31:7']), mavros_imu_rate='nan')
    assert 'mavros_setup' not in executables(run.harness.built)


@pytest.mark.parametrize('alias,ids', list(mavros_configuration.ALIASES.items()))
def test_every_rate_alias_overrides_its_messages_and_unset_keeps_them(run, tmp_path, alias, ids):
    rates = selected(tmp_path, ['33:7', '30:8', '31:9', '24:4', '65:6'])

    def requested(**arguments):
        setup = named(run(mavros_setup_params_file=rates, **arguments), 'mavros_setup')
        assert setup['parameters'][0] == rates
        return dict(entry.split(':') for entry in setup['parameters'][1]['message_rates'])
    before = requested()
    assert requested(**{alias: '13.5'}) == dict(before, **{str(i): '13.5' for i in ids})


# FCU endpoint (MAVROS)

@pytest.mark.parametrize('data,sim,context,expected', [
    ({'fcu_url': 'tcp://explicit:9000'}, False, 'standalone', 'tcp://explicit:9000'),
    ({'simulation_fcu_url': 'tcp://simulation:9001'}, True, 'standalone',
     'tcp://simulation:9001'),
    ({'serial_fallback': '/dev/selected'}, False, 'standalone',
     'serial:///dev/selected:115200'),
    ({'contexts': {'standalone': {'baud': 70000}}}, False, 'standalone',
     'serial:///dev/ttyACM0:70000'),
    ({'contexts': {'full_robot': {'baud': 70001}}}, False, 'full_robot',
     'serial:///dev/ttyACM0:70001'),
    ({'contexts': {'tui': {'baud': 70002}}}, False, 'tui', 'serial:///dev/ttyACM0:70002'),
])
def test_each_fcu_yaml_field_reaches_mavros(run, tmp_path, data, sim, context, expected):
    """Endpoint selection owns the FCU URL without opening the chosen transport."""
    nodes = run(endpoint_startup_file=layer(tmp_path, yaml.safe_dump(data)),
                endpoint_context=context, sim=str(sim).lower())
    parameters = named(nodes, 'mavros_node')['parameters'][1]
    assert parameters['fcu_url'] == expected
    assert parameters['use_sim_time'] is sim


def test_the_fcu_and_gcs_arguments_win_over_the_yaml(run):
    parameters = named(run(fcu_url='udp://:14540@', gcs_url=''), 'mavros_node')['parameters'][1]
    assert (parameters['fcu_url'], parameters['gcs_url']) == ('udp://:14540@', '')


@pytest.mark.parametrize('context', endpoint.CONTEXTS)
def test_each_context_can_disable_the_gcs(run, tmp_path, context):
    operator = layer(tmp_path, yaml.safe_dump({'contexts': {context: {'gcs_url': ''}}}))
    nodes = run(endpoint_startup_file=operator, endpoint_context=context, sim='true')
    assert named(nodes, 'mavros_node')['parameters'][1]['gcs_url'] == ''


@pytest.mark.parametrize('context', endpoint.CONTEXTS)
def test_each_context_discovery_strategy_finds_its_device(run, tmp_path, context):
    operator = layer(tmp_path, yaml.safe_dump({'contexts': {context: {
        'discovery': 'tui_stable_hints'}}}))
    (run.dev / 'ttyUSB0').touch()
    nodes = run(endpoint_startup_file=operator, endpoint_context=context)
    assert named(nodes, 'mavros_node')['parameters'][1]['fcu_url'] == (
        f'serial://{run.dev}/ttyUSB0:' + str(endpoint.resolve_endpoints(
            operator_file=operator, context=context).values['contexts'][context]['baud']))


@pytest.mark.parametrize('text', [
    'unity: {port: false}\n',   # an unrelated late field still blocks the endpoint
    "fcu_url: 'tcp://host:１２３４'\n", "fcu_url: 'serial:///dev/ttyACM0:５７６００'\n",
    "fcu_url: 'udp://@host:14550/?ids=１,２'\n"])
def test_an_invalid_endpoint_layer_fails_before_mavros(run, tmp_path, text):
    with pytest.raises(ValueError):
        run(endpoint_startup_file=layer(tmp_path, text), sim='true')
    assert run.harness.built == []
