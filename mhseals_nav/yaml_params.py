"""A node's YAML from the installed share as defaults, with launch overrides on top.

``section_config`` reads one node's section of a subsystem file (watchdogs.yaml, camera.yaml):
its defaults, the section and its tune: entries, then overrides, strictly (an unknown key is an
error naming it). Keys that lost their unit suffix are still read from overrides for one release,
with a warning.
"""

import math
from pathlib import Path

import yaml

from mhseals_nav.layers import deep_merge, load_strict, nested

WATCHDOGS = 'watchdogs.yaml'
HULL = 'hull.yaml'
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
           'max_costmap_age_s': ('max_costmap_age', None), 'max_tf_age_s': ('max_tf_age', None),
           'hfov_deg': ('hfov', math.radians), 'range_m': ('range', None)}


def node_config(node, filename, base=None):
    """Return ``base`` < share/mhseals_nav/config/<filename> < the node's parameters.

    The node must be built with ``automatically_declare_parameters_from_overrides=True``.
    """
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('mhseals_nav')) / 'config' / filename
    cfg = deep_merge(deep_merge({}, base or {}),
                     yaml.safe_load(path.read_text())['/**']['ros__parameters'])
    flat = {name: p.value for name, p in node._parameters.items() if name != 'use_sim_time'}
    return deep_merge(cfg, nested(flat))


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


def hull(path=None):
    """Return config/hull.yaml, the one statement of the hull's size.

    The rectangle about base_link (half_length, half_width) and the padding Nav2's costmaps
    add around it.
    """
    path = Path(path) if path else config_path(HULL)
    return load_strict(path.read_text(), path)


def rectangle(path=None):
    """The hull rectangle of config/hull.yaml ({half_length, half_width}), the nodes' default."""
    h = hull(path)
    return {'half_length': h['half_length'], 'half_width': h['half_width']}


def footprint(rectangle):
    """Nav2's footprint string of a hull rectangle (unpadded)."""
    x, y = rectangle['half_length'], rectangle['half_width']
    return str([[x, y], [x, -y], [-x, -y], [-x, y]])


def camera_fov(path=None):
    """config/camera.yaml backend.fov: {hfov (rad), range (m)}, the one camera FOV."""
    return section_config('camera.yaml', 'backend', path=path)['fov']


def watchdog_config(name, overrides=None, warn=lambda message: None, path=None, base=None):
    """The watchdog ``name``'s section of config/watchdogs.yaml (see ``section_config``)."""
    return section_config(WATCHDOGS, name, overrides, warn, path, base)


def section_config(filename, name, overrides=None, warn=lambda message: None, path=None,
                   base=None):
    """Return section ``name`` of config/<filename>, ``base`` under its defaults.

    base < defaults < the section < tune.<name> < overrides.

    ``overrides`` is flat ({'lidar.rpm': 600}) as ROS parameters are. Strict: the file
    must have the section, and an override must name a key the section has.
    """
    path = Path(path) if path else config_path(filename)
    doc = load_strict(path.read_text(), path)
    if not isinstance(doc, dict) or not isinstance(doc.get(name), dict):
        raise ValueError(f'{path}: no {name} section')
    cfg = {}
    for layer in (base, doc.get('defaults'), doc[name], (doc.get('tune') or {}).get(name)):
        deep_merge(cfg, layer or {})
    over = nested(renamed(overrides or {}, warn))
    unknown = sorted(set(over) - set(cfg))
    if unknown:
        raise ValueError(f"{path}: {name} has no {', '.join(unknown)}")
    return deep_merge(cfg, over)


def node_section_config(node, filename, name, base=None):
    """``section_config`` with the node's parameters (built with automatically_declare...)."""
    flat = {key: p.value for key, p in node._parameters.items()
            if key not in ROS_PARAMETERS and not key.startswith('qos_overrides.')}
    return section_config(filename, name, flat, node.get_logger().warning, base=base)


def node_watchdog_config(node, name, base=None):
    return node_section_config(node, WATCHDOGS, name, base)
