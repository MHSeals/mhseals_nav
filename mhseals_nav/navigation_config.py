"""Merge measured limits with the current navigation parameters at launch."""
import logging
from copy import deepcopy
from math import isfinite
from pathlib import Path

import yaml

log = logging.getLogger(__name__)


def deep_merge(dst, src):
    """Recursively copy src onto dst; lists and scalars replace."""
    for key, value in src.items():
        if isinstance(value, dict):
            deep_merge(dst.setdefault(key, {}), value)
        else:
            dst[key] = deepcopy(value)
    return dst


def load_overlay(path):
    """Read a user YAML overlay; a missing, empty, unparsable or non-mapping file is no overlay."""
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text()) or {}
    except FileNotFoundError:
        return {}
    except (OSError, yaml.YAMLError, UnicodeDecodeError) as error:
        log.error('Ignoring overlay %s: %s', path, error)
        return {}
    if not isinstance(data, dict):
        log.error('Ignoring overlay %s: root is %s, not a mapping', path, type(data).__name__)
        return {}
    return data


EKF_NODES = {'ekf_local', 'ekf_global', 'navsat_transform'}


def ekf_overlay(path, node, base_path=None):
    """Parameters the overlay sets for one EKF/navsat node, flat or under ros__parameters.

    With base_path (the packaged config) values are typed like it, so an int
    frequency stays a float and NaN is dropped; robot_localization aborts on both.
    """
    overlay = load_overlay(path)
    for key in overlay.keys() - EKF_NODES:
        log.warning('Ignoring overlay key %r in %s: use node-keyed sections (%s), not /**',
                    key, path, ', '.join(sorted(EKF_NODES)))
    section = overlay.get(node) or {}
    if not isinstance(section, dict):
        log.error('Ignoring overlay section %s in %s: not a mapping', node, path)
        return {}
    params = section.get('ros__parameters', section)
    if not isinstance(params, dict):
        return {}
    if base_path is None:
        return params
    base = load_overlay(base_path).get('/**', {}).get('ros__parameters', {})
    return typed_like(base, params, keep_sign=False)


def valid_numbers(controller, follow):
    """True when the values derived() computes from are numbers and the frequency is positive."""
    values = {'controller_frequency': controller.get('controller_frequency', 1)}
    for key in ('vx_max', 'time_steps', 'model_dt', 'vx_std', 'vy_std', 'wz_std', 'vy_max', 'wz_max'):
        if key in follow:
            values[key] = follow[key]
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            log.error('Not deriving navigation limits: %s = %r is not a number', key, value)
            return False
    if values['controller_frequency'] <= 0:
        log.error('Not deriving navigation limits: controller_frequency must be positive')
        return False
    return True


def derived(config, original=None):
    """Return config with the values that follow from others recomputed.

    The smoother passes what MPPI may command, the rotation shim turns no faster than it, MPPI advances one model_dt per
    control cycle, and the plan must reach the whole horizon. original is the
    current tuning whose prune_distance is a floor.
    """
    result = deepcopy(config)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    follow = controller.get('FollowPath')
    if follow is None:
        return result
    original = (original or config)['controller_server']['ros__parameters']['FollowPath']
    if not valid_numbers(controller, follow):
        return result
    if 'controller_frequency' in controller:
        follow['model_dt'] = round(1 / controller['controller_frequency'], 6)
    # Sampling noise above an axis limit would clip most samples.
    for axis in ('vx', 'vy', 'wz'):
        if f'{axis}_std' in follow and f'{axis}_max' in follow:
            follow[f'{axis}_std'] = min(follow[f'{axis}_std'], follow[f'{axis}_max'])
    if 'vx_max' in follow:
        follow['prune_distance'] = max(original.get('prune_distance', 0.), round(
            1.2 * follow['vx_max'] * follow.get('time_steps', 56) * follow.get('model_dt', .05), 2))
    # The shim's look-ahead and closed-loop accel assume the rates MPPI is held to.
    for shim, limit in (('rotate_to_heading_angular_vel', 'wz_max'), ('max_angular_accel', 'az_max')):
        if shim in follow and limit in follow:
            follow[shim] = follow[limit]
    smoother = result.get('velocity_smoother', {}).get('ros__parameters')
    if smoother is not None:
        for key, limits in (('max_velocity', ('vx_max', 'vy_max', 'wz_max')),
                            ('min_velocity', ('vx_min', '-vy_max', '-wz_max')),
                            ('max_accel', ('ax_max', 'ay_max', 'az_max')),
                            ('max_decel', ('ax_min', '-ay_max', '-az_max'))):
            for index, name in enumerate(limits):
                sign, name = (-1, name[1:]) if name[0] == '-' else (1, name)
                if name in follow and key in smoother:
                    smoother[key][index] = sign * follow[name]
    return result


def warn_overridden(limits, result, path=()):
    """Log each overlay value that the merge or derivation replaced with another."""
    for key, value in limits.items():
        if isinstance(value, dict):
            warn_overridden(value, result.get(key, {}), path + (key,))
        elif result.get(key) != value:
            log.warning('Overlay %s = %r is overridden to %r (derived from other limits or capped by the base)',
                        '.'.join(path + (key,)), value, result.get(key))


def wrong_sign(default, value):
    """True when value is on the other side of zero than default; zero only fits a default that is <= 0 (a minimum)."""
    return value * default < 0 or (default > 0 and value == 0)


def typed_like(base, limits, path=(), keep_sign=True):
    """Copy of limits with ints widened to float where base holds a float.

    Non-numbers, non-finite values and values whose sign differs from the base
    (or is zero where the base is positive) are dropped, so the base value stays.

    keep_sign=False skips the sign check for parameters that may be negative.

    Where base holds an int, an integral float becomes an int and any other
    non-integer is dropped.

    Nav2 rejects a params file whose parameter changes type, so a hand-edited
    `vx_max: 1` must not reach it as an integer.
    """
    result = {}
    for key, value in limits.items():
        default = base.get(key) if isinstance(base, dict) else None
        if isinstance(value, dict):
            result[key] = typed_like(default, value, path + (key,), keep_sign)
        elif isinstance(default, float) and not isinstance(default, bool):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                log.error('Ignoring overlay %s = %r: not a number', '.'.join(path + (key,)), value)
                continue
            if not isfinite(value) or (keep_sign and wrong_sign(default, value)):
                log.error('Ignoring overlay %s = %r: not finite or out of range', '.'.join(path + (key,)), value)
                continue
            result[key] = float(value)
        elif isinstance(default, int) and not isinstance(default, bool):
            if isinstance(value, float) and isfinite(value) and value == int(value):
                value = int(value)
            if isinstance(value, bool) or not isinstance(value, int):
                log.error('Ignoring overlay %s = %r: not an integer', '.'.join(path + (key,)), value)
                continue
            result[key] = value
        else:
            result[key] = value
    return result


def merge_limits(base, limits):
    """Overlay limits onto base, recompute derived keys and warn about overlay values that did not stick."""
    limits = typed_like(base, limits)
    result = deep_merge(deepcopy(base), limits)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    original = base['controller_server']['ros__parameters']['FollowPath']
    if 'vx_min' in original and 'vx_min' in follow:
        follow['vx_min'] = max(original['vx_min'], follow['vx_min'])
    result = derived(result, base)
    warn_overridden(limits, result)
    return result
