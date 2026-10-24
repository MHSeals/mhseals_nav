"""Merge measured limits with the current navigation parameters at launch."""
import logging
import os
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


class StrictOverlayLoader(yaml.SafeLoader):
    """Do not discard ambiguous user assignments during dictionary construction."""
    def construct_mapping(self, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            if key_node.tag == 'tag:yaml.org,2002:merge':
                raise ValueError(f'Merge keys are unsupported at {key_node.start_mark}')
            key = self.construct_object(key_node, deep=deep)
            if key in result:
                raise ValueError(f'Duplicate key {key!r} at {key_node.start_mark}')
            result[key] = self.construct_object(value_node, deep=deep)
        return result

    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise ValueError('YAML aliases are unsupported in navigation overlays')
        return super().compose_node(parent, index)


def load_overlay(path, *, strict=False):
    """Read an optional overlay; strict mode rejects invalid existing documents."""
    try:
        text = Path(path).expanduser().read_text()
        data = yaml.load(text, Loader=StrictOverlayLoader if strict else yaml.SafeLoader)
        if data is None and (not strict or yaml.compose(text) is None):
            data = {}
    except FileNotFoundError:
        return {}
    except (OSError, yaml.YAMLError, UnicodeDecodeError) as error:
        if strict:
            raise ValueError(f'Invalid overlay {path}: {error}') from error
        log.error('Ignoring overlay %s: %s', path, error)
        return {}
    if not isinstance(data, dict):
        if strict:
            raise ValueError(f'Invalid overlay {path}: root must be a mapping')
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
    follow = result.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath')
    original = base.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath')
    if isinstance(original, dict) and isinstance(follow, dict) and 'vx_min' in original and 'vx_min' in follow:
        follow['vx_min'] = max(original['vx_min'], follow['vx_min'])
    result = derived(result, base)
    warn_overridden(limits, result)
    return result


def validate_navigation_overlay(base, overlay, path=()):
    """Reject malformed requested values before deriving or constructing Nav2 nodes.

    The selected base is the supported parameter structure. This does not claim
    upstream dynamic-parameter support or infer undocumented enum options.
    """
    if not isinstance(overlay, dict):
        raise ValueError(f"{'.'.join(path)}: expected mapping")
    for key, value in overlay.items():
        location = '.'.join((*path, str(key)))
        if key not in base:
            raise ValueError(f'{location}: unknown parameter in selected base')
        default = base[key]
        if isinstance(default, dict):
            validate_navigation_overlay(default, value, (*path, str(key)))
        elif isinstance(default, bool):
            if not isinstance(value, bool):
                raise ValueError(f'{location}: expected boolean')
        elif isinstance(default, (int, float)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(f'{location}: expected finite number')
            if isinstance(default, int) and value != int(value):
                raise ValueError(f'{location}: expected integer')
            if wrong_sign(default, value):
                raise ValueError(f'{location}: out of range')
            if key == 'waypoint_pause_duration' and value < 0:
                raise ValueError(f'{location}: must be nonnegative')
            if key == 'mode' and path and path[-1] == 'PathAngleCritic' and value not in (0, 1, 2):
                raise ValueError(f'{location}: expected mode 0, 1 or 2')
            if len(path) >= 2 and path[-2] == 'deadband_velocity' and value < 0:
                raise ValueError(f'{location}: deadband must be nonnegative')
        elif isinstance(default, list):
            if not isinstance(value, list):
                raise ValueError(f'{location}: expected list')
            # Motion vectors have a fixed axis count; plugin lists remain variable.
            if key in ('max_velocity', 'min_velocity', 'max_accel', 'max_decel', 'deadband_velocity'):
                if len(value) != 3:
                    raise ValueError(f'{location}: expected three axes')
            if default:
                for index, item in enumerate(value):
                    template = default[min(index, len(default) - 1)]
                    validate_navigation_overlay({'item': template}, {'item': item}, (*path, str(key), str(index)))
        elif isinstance(default, str):
            if not isinstance(value, str):
                raise ValueError(f'{location}: expected string')
            # Nav2 velocity smoother's documented closed/open feedback contract.
            if key == 'feedback' and value not in ('OPEN_LOOP', 'CLOSED_LOOP'):
                raise ValueError(f'{location}: expected OPEN_LOOP or CLOSED_LOOP')


def validated_merge_limits(base, limits):
    validate_navigation_overlay(base, limits)
    def normalized(template, value):
        if isinstance(value, dict):
            return {key: normalized(template[key], item) for key, item in value.items()}
        if isinstance(value, list) and template:
            return [normalized(template[min(index, len(template) - 1)], item)
                    for index, item in enumerate(value)]
        if isinstance(template, float):
            return float(value)
        if isinstance(template, int) and not isinstance(template, bool):
            return int(value)
        return value
    return merge_limits(base, normalized(base, limits))


def persistent_limits_path():
    """Match the existing TUI overlay selection until shared snapshots replace it."""
    override = os.environ.get('MHSEALS_NAV2')
    if override:
        return str(Path(override).expanduser())
    return str(Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'mhseals/nav2.yaml')
