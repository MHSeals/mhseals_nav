"""Actual launch bodies, inert constructors and installed-share selection."""
from mhseals_nav.odometry_configuration import resolve_odometry
import pytest
from test_odometry_startup_launch import context, factories, ROOT


def launch(calls):
    def settings():
        return resolve_odometry(ROOT / 'config/odometry_startup.yaml')
    return factories('ekf.launch.py', settings, calls)


@pytest.mark.parametrize('name', ['ekf_global', 'navsat_transform'])
@pytest.mark.parametrize('bad', ['invalid_double', 'graph', 'duplicate', 'missing'])
def test_late_invalid_base_or_masked_graph_constructs_nothing(tmp_path, name, bad):
    selected = tmp_path / (name + '.yaml')
    raw = (ROOT / 'config' / (name + '.yaml')).read_text()
    if bad == 'invalid_double':
        raw = raw.replace('frequency: 20.0', 'frequency: .nan')
    elif bad == 'graph':
        raw = raw.replace('map_frame: map', 'map_frame: other')
    elif bad == 'duplicate':
        raw = raw.replace('frequency: 20.0', 'frequency: 20.0\n    frequency: 30.0')
    if bad != 'missing':
        selected.write_text(raw)
    overlay = tmp_path / 'overlay.yaml'
    overlay.write_text(name + ': {map_frame: map}')  # cannot conceal an invalid base
    supplied = dict(context(), ekf_overlay_file=str(overlay),
                    **{name + '_config_file': str(selected)})
    calls = []
    with pytest.raises((ValueError, OSError)):
        launch(calls)['ekf_nodes'](supplied)
    assert calls == []  # both Node and TimerAction use this collector


@pytest.mark.parametrize('clock', ['garbage', '1', 'yes', ''])
def test_invalid_clock_before_construction(clock):
    calls = []
    with pytest.raises(ValueError):
        launch(calls)['ekf_nodes'](dict(context(), sim=clock))
    assert calls == []


def test_factory_parameters_are_validated_copies_and_fresh_changes_propagate(tmp_path):
    overlay = tmp_path / 'selected.yaml'
    overlay.write_text('ekf_global: {frequency: 37, transform_time_offset: -0.2}')
    supplied = dict(context(), ekf_overlay_file=str(overlay), sim='false')
    calls = []
    launch(calls)['ekf_nodes'](supplied)
    old = next(c for c in calls if c.get('name') == 'ekf_global')['parameters'][0]
    assert old['frequency'] == 37. and type(old['frequency']) is float
    assert old['use_sim_time'] is False
    assert all(isinstance(c['parameters'][0], dict) for c in calls if 'parameters' in c)
    overlay.write_text('ekf_global: {frequency: 43}')
    assert old['frequency'] == 37.  # Node is not handed a path that re-reads new bytes
    calls = []
    launch(calls)['ekf_nodes'](supplied)
    assert next(c for c in calls if c.get('name') == 'ekf_global')[
        'parameters'][0]['frequency'] == 43.


def test_full_robot_forwards_empty_default_and_selected_inputs(tmp_path):
    calls = []
    robot = factories('robot.launch.py', lambda: None, calls)
    declared = {}
    robot.update(LaunchDescription=lambda x: x,
                 DeclareLaunchArgument=lambda name, **kw: declared.update(
                     {name: kw['default_value']}),
                 persistent_limits_path=lambda: '', snapshot_root=lambda: '',
                 snapshot_scope=lambda: '')
    robot['generate_launch_description']()
    assert declared['ekf_overlay_file'] == ''
    odom = next(dict(c['launch_arguments']) for c in calls if 'launch_arguments' in c
                and 'urdf_file' in dict(c['launch_arguments']))
    supplied = context()
    overlay = tmp_path / 'robot.yaml'
    overlay.write_text('navsat_transform: {delay: 4}')
    supplied['ekf_overlay_file'] = str(overlay)
    forwarded = {key: value.perform(supplied) for key, value in odom.items()
                 if key in supplied}
    calls = []
    launch(calls)['ekf_nodes'](dict(supplied, **forwarded))
    assert next(c for c in calls if c.get('name') == 'navsat_transform')[
        'parameters'][0]['delay'] == 4.


def test_installed_share_defaults_outside_cwd(tmp_path, monkeypatch):
    """Execute actual declaration defaults resolving an explicit installed share."""
    import shutil
    share = tmp_path / 'installed/share/mhseals_nav'
    shutil.copytree(ROOT / 'config', share / 'config')
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('HOME', str(tmp_path))

    class Join:
        def __init__(self, parts): self.parts = parts

        def perform(self, ctx):
            return '/'.join(p.perform(ctx) if hasattr(p, 'perform') else str(p)
                            for p in self.parts)

    declared = {}
    calls = []
    factory = launch(calls)
    factory.update(FindPackageShare=lambda name: str(share), PathJoinSubstitution=Join,
                   DeclareLaunchArgument=lambda name, **kw: declared.update(
                     {name: kw['default_value']}),
                   LaunchDescription=lambda x: x)
    factory['generate_launch_description']()
    assert declared['ekf_overlay_file'] == ''
    supplied = {key: value.perform({}) if hasattr(value, 'perform') else value
                for key, value in declared.items()}
    calls.clear()
    factory['ekf_nodes'](supplied)
    assert len([c for c in calls if 'parameters' in c]) == 3
    assert all(c['parameters'][0]['use_sim_time'] is True for c in calls if 'parameters' in c)
