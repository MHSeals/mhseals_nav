"""
Strict, pure startup selection of MAVROS desired message rates.

MAVROS MessageInterval uses Hz: zero restores the FCU default and -1 disables.
No request result or observed delivery is inferred by this resolver.
"""
import math
from pathlib import Path
import re
from types import MappingProxyType


from .navigation_config import load_strict

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


POLICY_FIELDS = {
    'retry_attempts': 'integer', 'retry_delay': 'nonnegative',
    'response_timeout': 'positive', 'service_wait': 'positive',
    'reboot_dedupe_window': 'positive', 'verify_period': 'positive',
    'rc_window': 'positive', 'rc_min_hz': 'positive', 'rc_min_channels': 'integer',
    'rc_grace': 'nonnegative', 'rc_timer': 'positive', 'fcu_sysid': 'sysid',
}


def validate_policy_value(key, value):
    rule = POLICY_FIELDS[key]
    try:
        finite = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        finite = False
    if (not finite
            or (rule in ('integer', 'sysid') and type(value) is not int)
            or (rule == 'sysid' and value > 255)
            or value < 0 or (rule != 'nonnegative' and value == 0)):
        raise ValueError('invalid policy: ' + key)
    return value if rule in ('integer', 'sysid') else float(value)


def validate_setup(params, *, complete=True):
    """Validate every supplied layer before overrides; no Python policy defaults."""
    allowed = {'message_rates', *POLICY_FIELDS}
    if (not isinstance(params, dict) or set(params) - allowed
            or (complete and set(params) != allowed)):
        raise ValueError('invalid or incomplete MAVROS setup parameters')
    if 'message_rates' in params:
        validate_rates(params['message_rates'])
    for key, rule in POLICY_FIELDS.items():
        if key not in params:
            continue
        validate_policy_value(key, params[key])
    return MappingProxyType({
        key: tuple(value) if key == 'message_rates' else validate_policy_value(key, value)
        for key, value in params.items()})


def parameters_from_yaml(raw, path='<mavros_setup>'):
    """Read only the consumed per-node schema; unsupported settings fail closed."""
    data = load_strict(raw, path)
    if not isinstance(data, dict) or set(data) != {'mavros_setup'}:
        raise ValueError('expected only mavros_setup')
    node = data['mavros_setup']
    if not isinstance(node, dict) or set(node) != {'ros__parameters'}:
        raise ValueError('expected only mavros_setup.ros__parameters')
    params = node['ros__parameters']
    validate_setup(params, complete=False)
    return params


def rates_from_yaml(raw):
    return validate_rates(parameters_from_yaml(raw)['message_rates'])


def resolve_setup(path, overrides=None):
    """Resolve selected bytes then explicit aliases, validating before construction."""
    if not path:
        raise ValueError('mavros_setup_params_file must select an existing file')
    params = installed_setup_defaults()
    selected = parameters_from_yaml(Path(path).expanduser().read_text(), path)
    params.update(selected)
    validate_setup(params)
    rates = dict(validate_rates(params['message_rates']))
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
    params['message_rates'] = entries
    normalized = validate_setup(params)
    return {key: list(value) if key == 'message_rates' else value
            for key, value in normalized.items()}


def installed_setup_defaults():
    """Load the same package authority for direct executable startup."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('mhseals_nav')) / 'config' / 'mavros_setup.yaml'
    params = parameters_from_yaml(path.read_text(), path)
    normalized = validate_setup(params)
    return {key: list(value) if key == 'message_rates' else value
            for key, value in normalized.items()}
