"""A node's YAML from the package config/ (layers.config_path), with launch overrides on top.

``section_config`` reads one node's section of a subsystem file (camera.yaml):
its defaults, the section and its tune: entries, then overrides, strictly (an unknown key is an
error naming it). Keys that lost their unit suffix are still read from overrides for one release,
with a warning.
"""

import math
from pathlib import Path

from mhseals_nav.layers import config_path, deep_merge, load_strict, nested

HULL = 'hull.yaml'
ROS_PARAMETERS = ('use_sim_time', 'start_type_description_service')   # every node has them
# Old override key -> (new key, conversion); dropped after one release.
RENAMED = {'max_age_s': ('max_age', None), 'rate_hz': ('rate', None),
           'half_length_m': ('half_length', None), 'half_width_m': ('half_width', None),
           'hfov_deg': ('hfov', math.radians), 'range_m': ('range', None)}


def params_file(name):
    """config/<name>'s ros__parameters (``layers.config_path``), read strictly."""
    path = config_path(name)
    return load_strict(path.read_text(), path)['/**']['ros__parameters']


def node_config(node, filename, base=None):
    """Return ``base`` < config/<filename> < the node's parameters.

    The node must be built with ``automatically_declare_parameters_from_overrides=True``.
    """
    cfg = deep_merge(deep_merge({}, base or {}), params_file(filename))
    flat = {name: p.value for name, p in node._parameters.items() if name != 'use_sim_time'}
    return deep_merge(cfg, nested(flat))


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
