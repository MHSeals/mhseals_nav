from mhseals_nav.navigation_config import derived, merge_limits


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
