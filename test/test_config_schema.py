"""The TUI schema describes real keys: each resolves in the package YAML within its range."""
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
SCHEMA = yaml.safe_load((ROOT / 'config/schema.yaml').read_text())
FILES = {'controller_server': 'nav2_params', 'velocity_smoother': 'nav2_params',
         'mavros_setup': 'mavros_setup', 'lidar': 'lidar'}
R2_UNITS = {'s', 'Hz', 'm', 'm/s', 'rad', 'rad/s', 'm/s^2', 'count', 'bool', 'enum', 'frame',
            'topic', 'path', '1'}
# Not in the R2 list yet; the astro lane owns docs/config-conventions.md.
PENDING_R2 = {'rad/s^2', 'ID:Hz', 'm/s, m/s, rad/s', 'm/s^2, m/s^2, rad/s^2'}
# Settable in a layer, but defaulted outside the package YAML (boat-network.env LIDAR_IP).
NO_PACKAGE_DEFAULT = {'lidar.endpoint.ip'}
TYPES = {'float': (int, float), 'int': (int,), 'bool': (bool,), 'str': (str,),
         'list': (list,)}


def lookup(key):
    head = key.split('.')[0]
    node = yaml.safe_load((ROOT / 'config' / (FILES[head] + '.yaml')).read_text())
    parts = key.split('.')
    if head in ('controller_server', 'velocity_smoother', 'mavros_setup'):
        parts = [head, 'ros__parameters', *parts[1:]]
    for part in parts:
        node = node[part]
    return node


@pytest.mark.parametrize('key', SCHEMA)
def test_entry_matches_package_default(key):
    entry = SCHEMA[key]
    assert set(entry) - {'range'} == {'type', 'units', 'tier', 'doc'}
    assert isinstance(entry['units'], str) and entry['units'] in R2_UNITS | PENDING_R2
    assert entry['tier'] in ('live', 'reconfigure', 'restart', 'new-session')
    if key in NO_PACKAGE_DEFAULT:
        with pytest.raises(KeyError):
            lookup(key)
        return
    value = lookup(key)
    assert isinstance(value, TYPES[entry['type']]) and not (
        entry['type'] != 'bool' and isinstance(value, bool))
    if 'range' in entry:
        low, high = entry['range']
        assert low <= value <= high
