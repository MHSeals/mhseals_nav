"""Merge measured limits with the current navigation parameters at launch."""
from copy import deepcopy
from pathlib import Path

import yaml


def deep_merge(dst, src):
    """Recursively copy src onto dst; lists and scalars replace."""
    for key, value in src.items():
        if isinstance(value, dict):
            deep_merge(dst.setdefault(key, {}), value)
        else:
            dst[key] = deepcopy(value)
    return dst


def load_overlay(path):
    """Read a user YAML overlay; a missing or empty file is no overlay."""
    try:
        return yaml.safe_load(Path(path).expanduser().read_text()) or {}
    except FileNotFoundError:
        return {}


def ekf_overlay(path, node):
    """Parameters the overlay sets for one EKF/navsat node, flat or under ros__parameters."""
    section = load_overlay(path).get(node) or {}
    return section.get('ros__parameters', section)


def derived(config, original=None):
    """Return config with the values that follow from others recomputed.

    The smoother passes what MPPI may command, MPPI advances one model_dt per
    control cycle, and the plan must reach the whole horizon. original is the
    current tuning whose prune_distance is a floor.
    """
    result = deepcopy(config)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    follow = controller.get('FollowPath')
    if follow is None:
        return result
    original = (original or config)['controller_server']['ros__parameters']['FollowPath']
    if 'controller_frequency' in controller:
        follow['model_dt'] = round(1 / controller['controller_frequency'], 6)
    # Sampling noise above an axis limit would clip most samples.
    for axis in ('vx', 'vy', 'wz'):
        if f'{axis}_std' in follow and f'{axis}_max' in follow:
            follow[f'{axis}_std'] = min(follow[f'{axis}_std'], follow[f'{axis}_max'])
    if 'vx_max' in follow:
        follow['prune_distance'] = max(original.get('prune_distance', 0.), round(
            1.2 * follow['vx_max'] * follow.get('time_steps', 56) * follow.get('model_dt', .05), 2))
    smoother = result.get('velocity_smoother', {}).get('ros__parameters')
    if smoother is not None:
        for key, limits in (('max_velocity', ('vx_max', 'vy_max', 'wz_max')),
                            ('min_velocity', ('vx_min', '-vy_max', '-wz_max'))):
            for index, name in enumerate(limits):
                sign, name = (-1, name[1:]) if name[0] == '-' else (1, name)
                if name in follow and key in smoother:
                    smoother[key][index] = sign * follow[name]
    return result


def merge_limits(base, limits):
    result = deep_merge(deepcopy(base), limits)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    original = base['controller_server']['ros__parameters']['FollowPath']
    if 'vx_min' in original and 'vx_min' in follow:
        follow['vx_min'] = max(original['vx_min'], follow['vx_min'])
    return derived(result, base)
