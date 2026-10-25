from pathlib import Path

from mhseals_nav.navigation_config import derived, load_overlay, merge_limits


def test_limits_use_current_planner_and_horizon_without_mutating_base():
    base = {'planner_server': {'ros__parameters': {'plugin': 'current'}},
            'controller_server': {'ros__parameters': {'FollowPath': {
                'model_dt': .1, 'time_steps': 40, 'prune_distance': 3., 'vx_min': -.2}}}}
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 2.}}}}
    merged = merge_limits(base, limits)
    assert merged['planner_server'] == base['planner_server']
    follow = merged['controller_server']['ros__parameters']['FollowPath']
    assert follow['prune_distance'] == 9.6
    assert follow['vx_min'] == -.2
    assert 'vx_max' not in base['controller_server']['ros__parameters']['FollowPath']


def test_current_reverse_cap_and_shorter_horizon_win_over_old_limits():
    base = {'controller_server': {'ros__parameters': {'FollowPath': {
                'vx_min': -.1, 'model_dt': .1, 'time_steps': 10, 'prune_distance': 1.}}},
            'velocity_smoother': {'ros__parameters': {'min_velocity': [-.1, -.5, -.5]}}}
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {
                  'vx_max': 1., 'vx_min': -.2, 'prune_distance': 9.6}}},
              'velocity_smoother': {'ros__parameters': {'min_velocity': [-.2, -.4, -.3]}}}
    result = merge_limits(base, limits)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_min'] == -.1 and follow['prune_distance'] == 1.2
    assert result['velocity_smoother']['ros__parameters']['min_velocity'][0] == -.1
    assert limits['velocity_smoother']['ros__parameters']['min_velocity'][0] == -.2


def test_sampling_noise_never_exceeds_a_measured_axis_limit():
    base = {'controller_server': {'ros__parameters': {'FollowPath': {
                'vx_std': .15, 'vy_std': .1, 'wz_std': .25, 'model_dt': .1, 'time_steps': 48,
                'prune_distance': 3.}}}}
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {
                  'vx_max': .3, 'vy_max': .05, 'wz_max': .4}}}}
    follow = merge_limits(base, limits)['controller_server']['ros__parameters']['FollowPath']
    assert (follow['vx_std'], follow['vy_std'], follow['wz_std']) == (.15, .05, .25)
    assert follow['prune_distance'] == 3.0  # max(3.0, 1.2 x .3 x 4.8) keeps the base floor


def test_derived_ties_smoother_and_model_dt_to_mppi():
    config = {'controller_server': {'ros__parameters': {'controller_frequency': 20., 'FollowPath': {
                  'model_dt': .1, 'time_steps': 10, 'vx_max': .5, 'vx_min': -.2,
                  'vy_max': .1, 'wz_max': .3, 'prune_distance': 0.}}},
              'velocity_smoother': {'ros__parameters': {
                  'max_velocity': [9., 9., 9.], 'min_velocity': [-9., -9., -9.]}}}
    result = derived(config)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['model_dt'] == .05 and follow['prune_distance'] == .3
    smoother = result['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [.5, .1, .3]
    assert smoother['min_velocity'] == [-.2, -.1, -.3]
    assert config['velocity_smoother']['ros__parameters']['max_velocity'] == [9., 9., 9.]


def test_empty_and_comment_only_overlays_are_no_overlay(tmp_path):
    base = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 1.0}}}}
    for text in ('', '# nothing measured yet\n'):
        overlay = tmp_path / 'nav2.yaml'
        overlay.write_text(text)
        assert load_overlay(overlay) == {}
        merged = merge_limits(base, load_overlay(overlay))
        assert merged['controller_server']['ros__parameters']['FollowPath']['vx_max'] == 1.0


def test_navigation_launch_uses_the_shared_compiler():
    launch = Path(__file__).parents[1] / 'launch' / 'navigation.launch.py'
    source = launch.read_text()
    assert 'compile_navigation(' in source
    assert 'select_snapshot' not in source
    assert 'measured = yaml.safe_load' not in source


def test_overlay_values_the_merge_overrides_are_warned_about(caplog):
    base = {'controller_server': {'ros__parameters': {'FollowPath': {
                'model_dt': .05, 'time_steps': 56, 'prune_distance': 3., 'vx_min': -.1}}}}
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {
                'vx_max': .4, 'vx_min': -.9, 'prune_distance': 9.}}}}
    with caplog.at_level('WARNING'):
        merge_limits(base, limits)
    assert 'FollowPath.vx_min' in caplog.text
    assert 'FollowPath.prune_distance' in caplog.text
    assert 'FollowPath.vx_max' not in caplog.text


def test_non_numeric_or_zero_limits_are_reported_not_raised(caplog):
    for follow, controller in (({'vx_max': '0.4'}, {}), ({'vx_max': .4}, {'controller_frequency': 0})):
        config = {'controller_server': {'ros__parameters': {'FollowPath': follow, **controller}}}
        with caplog.at_level('ERROR'):
            assert derived(config) == config
    assert 'is not a number' in caplog.text and 'must be positive' in caplog.text


def test_derived_ties_smoother_accel_to_mppi():
    config = {'controller_server': {'ros__parameters': {'FollowPath': {
                  'ax_max': .1, 'ax_min': -.2, 'ay_max': .15, 'az_max': .4}}},
              'velocity_smoother': {'ros__parameters': {
                  'max_accel': [.3, .3, .5], 'max_decel': [-.3, -.3, -.5]}}}
    smoother = derived(config)['velocity_smoother']['ros__parameters']
    assert smoother['max_accel'] == [.1, .15, .4]
    assert smoother['max_decel'] == [-.2, -.15, -.4]


def test_derived_ties_shim_rotation_to_mppi():
    follow = {'wz_max': .2, 'az_max': .3, 'rotate_to_heading_angular_vel': .3, 'max_angular_accel': .5}
    config = {'controller_server': {'ros__parameters': {'FollowPath': follow}}}
    shim = derived(config)['controller_server']['ros__parameters']['FollowPath']
    assert (shim['rotate_to_heading_angular_vel'], shim['max_angular_accel']) == (.2, .3)


def overlay_params(**params):
    base = {'controller_server': {'ros__parameters': {'controller_frequency': 10., 'FollowPath': {
        'vx_max': .4, 'model_dt': .1, 'time_steps': 40, 'prune_distance': 3.}}},
        'velocity_smoother': {'ros__parameters': {'max_velocity': [.4, .15, .3]}}}
    controller = {k: v for k, v in params.items() if k != 'vx_max'}
    follow = {'vx_max': params['vx_max']} if 'vx_max' in params else {}
    limits = {'controller_server': {'ros__parameters': {**controller, 'FollowPath': follow}}}
    return base, merge_limits(base, limits)


def test_integer_overlay_values_launch_as_their_float_forms():
    assert overlay_params(vx_max=1, controller_frequency=20) == overlay_params(vx_max=1., controller_frequency=20.)
    merged = overlay_params(vx_max=1)[1]
    assert merged['velocity_smoother']['ros__parameters']['max_velocity'] == [1., .15, .3]
    assert type(merged['controller_server']['ros__parameters']['FollowPath']['vx_max']) is float


def test_non_numeric_overlay_values_are_dropped_not_handed_to_nav2(caplog):
    with caplog.at_level('ERROR'):
        base, merged = overlay_params(vx_max='0.4', controller_frequency=True)
    assert merged == merge_limits(base, {})
    assert 'not a number' in caplog.text


def test_non_finite_zero_and_negative_overlay_values_keep_the_base(caplog):
    for params in ({'vx_max': float('nan')}, {'vx_max': -.4}, {'vx_max': float('inf')},
                   {'controller_frequency': 0}, {'controller_frequency': -5}):
        caplog.clear()
        with caplog.at_level('ERROR'):
            base, merged = overlay_params(**params)
        assert merged == merge_limits(base, {}), params
        assert 'out of range' in caplog.text


def follow_path(**params):
    base, _ = overlay_params()
    limits = {'controller_server': {'ros__parameters': {'FollowPath': params}}}
    return merge_limits(base, limits)['controller_server']['ros__parameters']['FollowPath']


def test_integral_float_for_an_integer_key_launches_as_int_and_others_keep_the_base(caplog):
    steps = follow_path(time_steps=56.0)['time_steps']
    assert steps == 56 and type(steps) is int
    for value in (56.5, float('nan'), '56', True):
        caplog.clear()
        with caplog.at_level('ERROR'):
            assert follow_path(time_steps=value)['time_steps'] == 40
        assert 'not an integer' in caplog.text


def test_zero_minimum_forbids_reversing_and_reaches_the_smoother(caplog):
    base = {'controller_server': {'ros__parameters': {'FollowPath': {
                'vx_min': -.1, 'model_dt': .1, 'time_steps': 10, 'prune_distance': 1.}}},
            'velocity_smoother': {'ros__parameters': {'min_velocity': [-.1, -.5, -.5]}}}
    for zero in (0, 0.):
        limits = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_min': zero}}}}
        with caplog.at_level('ERROR'):
            result = merge_limits(base, limits)
        assert result['controller_server']['ros__parameters']['FollowPath']['vx_min'] == 0.
        assert result['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0.
    assert 'out of range' not in caplog.text
