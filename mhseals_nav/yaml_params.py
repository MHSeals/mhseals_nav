"""A node's YAML from the installed share as defaults, with launch overrides on top."""

from pathlib import Path

import yaml


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
