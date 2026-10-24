"""
Strict, pure startup selection of MAVROS desired message rates.

MAVROS MessageInterval uses Hz: zero restores the FCU default and -1 disables.
No request result or observed delivery is inferred by this resolver.
"""
import math
from pathlib import Path
import re

import yaml

from .navigation_config import StrictOverlayLoader

ALIASES = {'mavros_odom_rate': (33,), 'mavros_imu_rate': (30, 31),
           'mavros_gps_rate': (24,), 'mavros_rc_rate': (65,)}


def validate_rates(entries):
    """Validate the ROS string-array contract without coercing IDs or duplicates."""
    if not isinstance(entries, list) or not entries:
        raise ValueError('message_rates must be a nonempty list of ID:Hz strings')
    result = []
    seen = set()
    for entry in entries:
        if not isinstance(entry, str) or entry.count(':') != 1:
            raise ValueError('message_rates entries must be ID:Hz strings')
        message_id, text = entry.split(':')
        if not re.fullmatch(r'0|[1-9][0-9]*', message_id):
            raise ValueError('message ID must be a decimal integer')
        message_id = int(message_id)
        if message_id > 0xffffff or message_id in seen:
            raise ValueError('message ID out of MAVLink 2 range or duplicated')
        try:
            hz = float(text)
        except (ValueError, TypeError) as error:
            raise ValueError('rate must be numeric Hz') from error
        if not math.isfinite(hz) or (hz < 0 and hz != -1):
            raise ValueError('rate must be finite positive Hz, 0 reset, or -1 disable')
        seen.add(message_id)
        result.append((message_id, hz))
    return result


def rates_from_yaml(raw):
    """Read only the consumed per-node schema; unsupported settings fail closed."""
    data = yaml.load(raw, Loader=StrictOverlayLoader)
    if not isinstance(data, dict) or set(data) != {'mavros_setup'}:
        raise ValueError('expected only mavros_setup')
    node = data['mavros_setup']
    if not isinstance(node, dict) or set(node) != {'ros__parameters'}:
        raise ValueError('expected only mavros_setup.ros__parameters')
    params = node['ros__parameters']
    if not isinstance(params, dict) or set(params) != {'message_rates'}:
        raise ValueError('expected only message_rates')
    return validate_rates(params['message_rates'])


def resolve_setup(path, overrides=None):
    """Resolve selected bytes then explicit aliases, validating before construction."""
    if not path:
        raise ValueError('mavros_setup_params_file must select an existing file')
    rates = dict(rates_from_yaml(Path(path).expanduser().read_text()))
    for alias, text in (overrides or {}).items():
        if alias not in ALIASES:
            raise ValueError(f'unknown rate alias: {alias}')
        if text == '':
            continue
        if not isinstance(text, str):
            raise ValueError('launch rate overrides must be strings')
        for message_id in ALIASES[alias]:
            rates[message_id] = validate_rates([f'{message_id}:{text}'])[0][1]
    entries = [f'{message_id}:{hz:g}' for message_id, hz in rates.items()]
    validate_rates(entries)
    return {'message_rates': entries}


def installed_setup_defaults():
    """Load the same package authority for direct executable startup."""
    from ament_index_python.packages import get_package_share_directory
    return resolve_setup(Path(get_package_share_directory('mhseals_nav')) /
                         'config' / 'mavros_setup.yaml')
