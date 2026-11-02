"""A node's YAML from the installed share as defaults, with launch overrides on top.

``watchdog_config`` is the one reader of config/watchdogs.yaml: defaults, the watchdog's section
and its tune: entries, then overrides, strictly (an unknown key is an error naming it). Keys that
lost their unit suffix are still read from overrides for one release, with a warning.
"""

import math
from pathlib import Path

import yaml

WATCHDOGS = 'watchdogs.yaml'
ROS_PARAMETERS = ('use_sim_time', 'start_type_description_service')   # every node has them
# Old override key -> (new key, conversion); dropped after one release.
RENAMED = {'max_age_s': ('max_age', None), 'command_window_s': ('command_window', None),
           'mux_state_max_age_s': ('mux_state_max_age', None), 'repeat_s': ('repeat', None),
           'rate_hz': ('rate', None), 'margin_m': ('margin', None),
           'outward_speed_mps': ('outward_speed', None),
           'outward_angle_deg': ('outward_angle', math.radians),
           'fence_rate_hz': ('fence_rate', None), 'anchor_max_speed_mps': ('anchor_max_speed', None),
           'max_tracks_age_s': ('max_tracks_age', None), 'max_stamp_age_s': ('max_stamp_age', None),
           'max_state_age_s': ('max_state_age', None), 'max_imu_age_s': ('max_imu_age', None),
           'max_gps_age_s': ('max_gps_age', None), 'max_odom_age_s': ('max_odom_age', None),
           'half_length_m': ('half_length', None), 'half_width_m': ('half_width', None),
           'trigger_m': ('trigger', None), 'move_m': ('move', None), 'min_age_s': ('min_age', None),
           'memory_s': ('memory', None), 'warmup_s': ('warmup', None),
           'cluster_m': ('cluster', None), 'miss_s': ('miss', None), 'crop_m': ('crop', None),
           'confirm_s': ('confirm', None), 'recent_s': ('recent', None),
           'max_costmap_age_s': ('max_costmap_age', None), 'max_tf_age_s': ('max_tf_age', None)}


def nested(flat):
    """{'a.b': 1} -> {'a': {'b': 1}}."""
    out = {}
    for key, value in flat.items():
        *parents, leaf = key.split('.')
        node = out
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = value
    return out


def merge(base, over):
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def node_config(node, filename):
    """Defaults from share/mhseals_nav/config/<filename>, then the node's parameters.

    The node must be built with ``automatically_declare_parameters_from_overrides=True``.
    """
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('mhseals_nav')) / 'config' / filename
    base = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    flat = {name: p.value for name, p in node._parameters.items() if name != 'use_sim_time'}
    return merge(base, nested(flat))


def config_path(filename):
    """config/<filename> of this source tree, else of the installed share."""
    path = Path(__file__).resolve().parents[1] / 'config' / filename
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config' / filename
    return path


def renamed(flat, warn=lambda message: None):
    """``flat`` with old suffixed keys under their new names (converted), each one warned."""
    out = {}
    for key, value in flat.items():
        *parents, leaf = key.split('.')
        if leaf in RENAMED:
            new, convert = RENAMED[leaf]
            warn(f'{key} is deprecated, read as {new} (units are SI with radians); '
                 'rename it before the next release')
            key, value = '.'.join([*parents, new]), convert(value) if convert else value
        out[key] = value
    return out


def watchdog_config(name, overrides=None, warn=lambda message: None, path=None):
    """The watchdog ``name``'s settings: defaults < its section < tune.<name> < ``overrides``.

    ``overrides`` is flat ({'course.corners': [...]}) as ROS parameters are. Strict: the file
    must have the section, and an override must name a key the section has.
    """
    from mhseals_nav.navigation_config import load_strict
    path = Path(path) if path else config_path(WATCHDOGS)
    doc = load_strict(path.read_text(), path)
    if not isinstance(doc, dict) or not isinstance(doc.get(name), dict):
        raise ValueError(f'{path}: no {name} section')
    cfg = merge(merge(dict(doc.get('defaults') or {}), doc[name]),
                (doc.get('tune') or {}).get(name) or {})
    over = nested(renamed(overrides or {}, warn))
    unknown = sorted(set(over) - set(cfg))
    if unknown:
        raise ValueError(f"{path}: {name} has no {', '.join(unknown)}")
    return merge(cfg, over)


def node_watchdog_config(node, name):
    """``watchdog_config`` with the node's parameters (built with automatically_declare...)."""
    flat = {key: p.value for key, p in node._parameters.items()
            if key not in ROS_PARAMETERS and not key.startswith('qos_overrides.')}
    return watchdog_config(name, flat, node.get_logger().warning)
