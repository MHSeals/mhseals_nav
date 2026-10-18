from mhseals_nav.navigation_config import merge_limits


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
