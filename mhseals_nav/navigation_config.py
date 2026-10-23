"""Merge measured limits with the current navigation parameters at launch."""
from copy import deepcopy


def merge_limits(base, limits):
    result = deepcopy(base)
    def merge(dst, src):
        for key, value in src.items():
            if isinstance(value, dict):
                merge(dst.setdefault(key, {}), value)
            else:
                dst[key] = deepcopy(value)
    merge(result, limits)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    original = base['controller_server']['ros__parameters']['FollowPath']
    if 'vx_min' in original and 'vx_min' in follow:
        follow['vx_min'] = max(original['vx_min'], follow['vx_min'])
    smoother = result.get('velocity_smoother', {}).get('ros__parameters', {})
    reverse = base.get('velocity_smoother', {}).get('ros__parameters', {}).get('min_velocity')
    if reverse and smoother.get('min_velocity'):
        smoother['min_velocity'][0] = max(reverse[0], smoother['min_velocity'][0])
    # Sampling noise above an axis limit would clip most samples.
    for axis in ('vx', 'vy', 'wz'):
        if f'{axis}_std' in follow and f'{axis}_max' in follow:
            follow[f'{axis}_std'] = min(follow[f'{axis}_std'], follow[f'{axis}_max'])
    # Recompute against today's MPPI horizon, including after base tuning changes.
    if 'vx_max' in follow:
        follow['prune_distance'] = max(original.get('prune_distance', 0.), round(
            1.2 * follow['vx_max'] * follow.get('time_steps', 56) * follow.get('model_dt', .05), 2))
    return result
