"""Literal expected consumer inputs for authoritative pure lowering."""
from copy import deepcopy
from pathlib import Path

from mhseals_nav.navigation_config import load_overlay, validated_merge_limits
from mhseals_nav.startup_projection import compile_navigation
import pytest
import yaml


@pytest.fixture
def base():
    path = Path(__file__).parents[1] / 'config/nav2_params.yaml'
    return yaml.safe_load(path.read_text())


def patch():
    return {'controller_server': {'ros__parameters': {
        'controller_frequency': 20, 'FollowPath': {
            'vx_max': .6, 'vy_max': .12, 'wz_max': .2}}}}


def test_nondefault_complete_consumer_fixture(base):
    before = deepcopy(base)
    result = validated_merge_limits(base, patch(), {})
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['model_dt'] == .05
    assert [follow[k] for k in ('vx_std', 'vy_std', 'wz_std')] == [.1, .07, .15]
    assert follow['prune_distance'] == 3.0
    assert follow['rotate_to_heading_angular_vel'] == .2
    assert follow['max_angular_accel'] == .5
    smoother = result['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [.6, .12, .2]
    assert smoother['min_velocity'] == [-.1, -.12, -.2]
    assert result['planner_server'] == base['planner_server']
    assert base == before


@pytest.mark.parametrize('key,value', [('model_dt', .1), ('wz_std', .3),
                                       ('prune_distance', 9.)])
def test_conflicting_explicit_derived_value_refused(base, key, value):
    explicit = patch()
    explicit['controller_server']['ros__parameters']['FollowPath'][key] = value
    with pytest.raises(ValueError, match='conflicts with canonical lowering'):
        validated_merge_limits(base, explicit, {})


@pytest.mark.parametrize('section,key', [
    ('velocity_smoother', 'max_velocity'), ('behavior_server', 'max_rotational_vel'),
    ('controller_server', 'rotate_to_heading_angular_vel')])
def test_derived_limit_in_an_overlay_is_refused(base, section, key):
    explicit = patch()
    params = explicit.setdefault(section, {}).setdefault('ros__parameters', {})
    (params.setdefault('FollowPath', {}) if section == 'controller_server' else params)[key] = .1
    with pytest.raises(ValueError, match='unknown parameter'):
        validated_merge_limits(base, explicit, {})


def test_smoother_optional_absence(base):
    del base['velocity_smoother']
    result = validated_merge_limits(base, patch(), {})
    assert 'velocity_smoother' not in result


def test_follow_path_change_reaches_every_derived_limit(base):
    result = validated_merge_limits(base, patch(), {})
    behavior = result['behavior_server']['ros__parameters']
    assert (behavior['max_rotational_vel'], behavior['rotational_acc_lim']) == (.2, .5)
    for key in ('vx_max', 'ax_max', 'az_max'):
        missing = deepcopy(base)
        del missing['controller_server']['ros__parameters']['FollowPath'][key]
        with pytest.raises(ValueError, match=f'FollowPath.{key}: required'):
            validated_merge_limits(missing, {}, {})


def test_required_base_is_checked_without_an_overlay(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('controller_server: []')
    with pytest.raises(ValueError, match='required mapping'):
        compile_navigation(path, '', sim='true')


@pytest.mark.parametrize('name', ['FollowPath', 'Custom'])
def test_selected_non_mppi_plugin_does_not_receive_mppi_fields(base, tmp_path, name):
    controller = base['controller_server']['ros__parameters']
    del controller['FollowPath']
    custom = {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
              'desired_linear_vel': .3}
    controller[name] = custom
    controller['controller_plugins'] = [name]
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(base))
    selected = compile_navigation(path, '', sim='true')
    assert selected['controller_server']['ros__parameters'][name] == custom
    assert 'model_dt' not in selected['controller_server']['ros__parameters'][name]


def test_renamed_selected_mppi_uses_its_plugin_contract(base):
    controller = base['controller_server']['ros__parameters']
    controller['Boat'] = controller.pop('FollowPath')
    controller['controller_plugins'] = ['Boat']
    result = validated_merge_limits(base, {'controller_server': {'ros__parameters': {
        'controller_frequency': 20}}}, {})
    assert result['controller_server']['ros__parameters']['Boat']['model_dt'] == .05


def _compile_text(tmp_path, text):
    path = tmp_path / 'strict.yaml'
    path.write_text(text)
    return path


@pytest.mark.parametrize('text, message', [
    ('a: .inf\n', 'Non-finite number'),
    ('a: .nan\n', 'Non-finite number'),
    ('a: 1\na: 2\n', 'Duplicate key'),
    ('a: &x {b: 1}\nc: {<<: *x}\n', 'aliases are unsupported'),
    ('a: {<<: {b: 1}}\n', 'Merge keys'),
])
def test_strict_loader_names_file_and_line(tmp_path, text, message):
    path = _compile_text(tmp_path, text)
    for call in (lambda: load_overlay(path, strict=True),
                 lambda: compile_navigation(path, '', sim='true')):
        with pytest.raises(ValueError, match=message) as error:
            call()
        assert str(path) in str(error.value) and 'line ' in str(error.value)


def test_nav2_limits_come_from_the_hardware_state_file(tmp_path):
    """characterization.nav2_limits sets MPPI's limits (and what derives from them); absent, the base stays."""
    root = Path(__file__).resolve().parents[1]
    params = root / 'config/nav2_params.yaml'
    state = tmp_path / 'hardware.yaml'
    state.write_text(yaml.safe_dump({'characterization': {'nav2_limits': {
        'velocity': [0.5, 0.2, 0.4], 'acceleration': [0.4, 0.3, 0.6]}}}))
    nav = compile_navigation(params, state, sim='true')
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    assert (follow['vx_max'], follow['vy_max'], follow['wz_max']) == (0.5, 0.2, 0.4)
    assert (follow['ax_max'], follow['ay_max'], follow['az_max'], follow['ax_min']) == (0.4, 0.3, 0.6, -0.4)
    assert nav['velocity_smoother']['ros__parameters']['max_velocity'] == [0.5, 0.2, 0.4]
    base = yaml.safe_load(params.read_text())['controller_server']['ros__parameters']['FollowPath']
    state.write_text('characterization: {}\n')
    unchanged = compile_navigation(params, state, sim='true')['controller_server']['ros__parameters']['FollowPath']
    assert unchanged['vx_max'] == base['vx_max']
    absent = compile_navigation(params, tmp_path / 'none.yaml', sim='true')['controller_server']['ros__parameters']
    assert absent['FollowPath']['vx_max'] == base['vx_max']


def test_nav2_limits_above_a_ceiling_are_clipped_and_malformed_refused(tmp_path, capsys):
    root = Path(__file__).resolve().parents[1]
    ceilings = yaml.safe_load((root / 'config/nav2_ceilings.yaml').read_text())
    state = tmp_path / 'hardware.yaml'
    state.write_text('characterization: {nav2_limits: {velocity: [9.0, 0.2, 0.4], acceleration: [0.4, 0.3, 9.0]}}\n')
    nav = compile_navigation(root / 'config/nav2_params.yaml', state, sim='true')
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    assert (follow['vx_max'], follow['az_max'], follow['vy_max']) == (ceilings['vx_max'], ceilings['az_max'], 0.2)
    warning = capsys.readouterr().err.strip().splitlines()
    assert len(warning) == 1 and 'vx_max 9->' in warning[0] and 'az_max 9->' in warning[0]
    state.write_text('characterization: {nav2_limits: {velocity: [0.5, 0.2], acceleration: [0.4, 0.3, 0.6]}}\n')
    with pytest.raises(ValueError, match='hardware.yaml'):
        compile_navigation(root / 'config/nav2_params.yaml', state, sim='true')


def test_missing_params_file_names_the_path(tmp_path):
    missing = tmp_path / 'absent.yaml'
    with pytest.raises(FileNotFoundError, match='absent.yaml'):
        compile_navigation(missing, '', sim='true')


def test_both_costmaps_get_the_hull_footprint_with_its_padding():
    """config/hull.yaml is the one hull: the compile writes it into both Nav2 costmaps."""
    root = Path(__file__).resolve().parents[1]
    hull = yaml.safe_load((root / 'config/hull.yaml').read_text())
    x, y = hull['half_length'], hull['half_width']
    nav = compile_navigation(root / 'config/nav2_params.yaml', '', sim='true')
    for name in ('local_costmap', 'global_costmap'):
        params = nav[name][name]['ros__parameters']
        assert yaml.safe_load(params['footprint']) == [[x, y], [x, -y], [-x, -y], [-x, y]]
        assert params['footprint_padding'] == hull['padding']
    raw = yaml.safe_load((root / 'config/nav2_params.yaml').read_text())
    assert 'footprint' not in raw['local_costmap']['local_costmap']['ros__parameters']


def test_effective_limits_are_what_the_launch_resolves(tmp_path):
    """The fast sim's limits: the launch's own compile, the state file's nav2_limits and ceilings applied."""
    from mhseals_nav.startup_projection import effective_limits
    root = Path(__file__).resolve().parents[1]
    params = root / 'config/nav2_params.yaml'
    limits = tmp_path / 'hardware.yaml'
    limits.write_text(yaml.safe_dump({'characterization': {'nav2_limits': {
        'velocity': [0.5, 0.3, 0.4], 'acceleration': [0.3, 0.2, 0.5]}}}))
    resolved = effective_limits(params, limits)
    launched = compile_navigation(params, limits, sim='true',
                                  share=None)
    follow = launched['controller_server']['ros__parameters']['FollowPath']
    smoother = launched['velocity_smoother']['ros__parameters']
    assert resolved['velocity'] == {'max': smoother['max_velocity'],
                                    'min': smoother['min_velocity']}
    assert resolved['accel'] == {'max': smoother['max_accel'], 'decel': smoother['max_decel']}
    # The overlay and what derives from it; above a ceiling, both refuse alike.
    assert resolved['controller']['vx_max'] == 0.5 and resolved['velocity']['max'][0] == 0.5
    assert resolved['controller']['vy_max'] == 0.3 == resolved['velocity']['max'][1]
    assert resolved['controller']['ay_max'] == 0.2
    assert resolved['rotation_shim']['rotate_to_heading_angular_vel'] == follow['wz_max'] == 0.4
    assert resolved['goal_checkers'] == {'general_goal_checker': {'xy': 0.4, 'yaw': 0.5},
                                         'precise_goal_checker': {'xy': 0.3, 'yaw': 0.25}}
    assert resolved['progress'] == {'radius': 0.1, 'time': 20.0}
    assert resolved['passed_goal_radius'] == 1.0
    hull = yaml.safe_load((root / 'config/hull.yaml').read_text())
    assert {(abs(x), abs(y)) for x, y in resolved['footprint']} == {
        (hull['half_length'], hull['half_width'])}
    camera = yaml.safe_load((root / 'config/camera.yaml').read_text())['backend']['fov']
    assert resolved['camera'] == {'hfov_deg': pytest.approx(110.0), 'range_m': camera['range']}
    assert str(limits) in resolved['sources'] and str(params) in resolved['sources']
    assert {str(root / 'config/hull.yaml'), str(root / 'config/camera.yaml')} <= set(
        resolved['sources'])
    limits.write_text('characterization: {nav2_limits: {velocity: [0.5, 9.0, 0.4], acceleration: [0.3, 0.2, 0.5]}}\n')
    ceiling = yaml.safe_load((root / 'config/nav2_ceilings.yaml').read_text())['vy_max']
    assert effective_limits(params, limits)['controller']['vy_max'] == ceiling     # clipped, not refused


def test_the_pool_preset_merges_over_the_base_and_caps_the_measured_limits(tmp_path, capsys):
    """preset:=pool: its keys over nav2_params.yaml, its footprint over hull.yaml's, and the measured
    nav2_limits lowered to its own limits (never raised)."""
    params = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'
    state = tmp_path / 'hardware.yaml'
    state.write_text(yaml.safe_dump({'characterization': {'nav2_limits': {
        'velocity': [0.5, 0.1, 0.4], 'acceleration': [0.4, 0.3, 0.6]}}}))
    nav = compile_navigation(params, state, sim='true', preset='pool')
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    assert (follow['vx_max'], follow['vy_max'], follow['wz_max']) == (0.3, 0.1, 0.4)
    assert 'the pool preset, clipped: vx_max 0.5->0.3' in capsys.readouterr().err
    assert follow['visualize'] is True and follow['time_steps'] * follow['model_dt'] == pytest.approx(2.0)
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    assert (local['width'], local['inflation_layer']['inflation_radius'], local['footprint_padding']) == (5, 0.3, 0.05)
    assert yaml.safe_load(local['footprint'])[2] == [-0.76, -0.45]
    with pytest.raises(FileNotFoundError, match='have: lake, pool'):
        compile_navigation(params, state, sim='true', preset='river')
