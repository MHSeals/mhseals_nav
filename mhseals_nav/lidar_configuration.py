"""Resolve startup-only lidar layers and paired driver/transform projections."""
from collections.abc import Mapping
from hashlib import sha256
from ipaddress import IPv4Address
import math
from pathlib import Path


from .navigation_config import load_strict

KEYS = ('lidar.model', 'lidar.frame', 'lidar.rpm', 'lidar.calibration',
        'lidar.endpoint.ip', 'lidar.endpoint.port')
CORRECTIONS = {'dist_correction', 'dist_correction_x', 'dist_correction_y',
               'focal_distance', 'focal_slope', 'horiz_offset_correction',
               'rot_correction', 'vert_correction', 'vert_offset_correction'}


def package_share(package):
    """Resolve only explicitly named installed packages, without ROS discovery."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def positive(value, name):
    """Reject booleans/coercion and nonfinite or nonpositive scalar values."""
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite positive numeric')


def validate_present(values):
    """Reject every supplied leaf before precedence can hide invalid input."""
    for key, value in values.items():
        if key == 'lidar.model':
            if value != 'VLP16':
                raise ValueError('only the VLP16 model is supported')
        elif key == 'lidar.frame':
            if (not isinstance(value, str) or not value or value.strip() != value or
                    any(char.isspace() for char in value)):
                raise ValueError('lidar frame must be nonempty and contain no whitespace')
        elif key == 'lidar.rpm':
            positive(value, 'rpm')
        elif key == 'lidar.calibration':
            validate_reference(value)
        elif key == 'lidar.endpoint.ip':
            if not isinstance(value, str):
                raise ValueError('lidar IP must be an IPv4 string')
            IPv4Address(value)
        elif key == 'lidar.endpoint.port':
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError('lidar UDP port must be an integer in 1..65535')
        else:
            raise ValueError(f'unsupported lidar leaf: {key}')


def validate_values(values):
    """Validate the complete supported projection after all layers resolve."""
    if set(values) != set(KEYS):
        raise ValueError('complete VLP16 lidar configuration required')
    validate_present(values)


def validate_calibration(raw, path='<calibration>'):
    """Accept the bounded VLP16db correction schema and all sixteen unique lasers."""
    data = load_strict(raw, path)
    if not isinstance(data, dict) or set(data) != {
            'lasers', 'num_lasers', 'distance_resolution'}:
        raise ValueError('unsupported calibration schema')
    if type(data['num_lasers']) is not int or data['num_lasers'] != 16:
        raise ValueError('VLP16 calibration requires sixteen lasers')
    positive(data['distance_resolution'], 'distance_resolution')
    lasers = data['lasers']
    if not isinstance(lasers, list) or len(lasers) != 16:
        raise ValueError('VLP16 calibration requires sixteen correction rows')
    ids = set()
    for row in lasers:
        if not isinstance(row, dict) or set(row) != CORRECTIONS | {'laser_id'}:
            raise ValueError('unsupported calibration laser fields')
        laser_id = row['laser_id']
        if type(laser_id) is not int or laser_id not in range(16) or laser_id in ids:
            raise ValueError('invalid or duplicate calibration laser ID')
        ids.add(laser_id)
        if any(type(row[key]) not in (int, float) or not math.isfinite(row[key])
               for key in CORRECTIONS):
            raise ValueError('calibration corrections must be finite numbers')
    return data


def parse_lidar(raw, path='<lidar>'):
    """Load one strict layer into validated dotted keys; unknown keys are refused."""
    data = load_strict(raw, path)
    if not isinstance(data, dict) or set(data) != {'lidar'} or not isinstance(data['lidar'], dict):
        raise ValueError('expected a single lidar mapping')
    values = {}
    for key, value in data['lidar'].items():
        if key == 'endpoint' and isinstance(value, dict):
            values.update({'lidar.endpoint.' + name: item for name, item in value.items()})
        else:
            values['lidar.' + key] = value
    validate_present(values)
    return values


def _layer(path):
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    values = parse_lidar(raw, path)
    return values, {'path': str(path), 'sha256': sha256(raw).hexdigest()}


def validate_reference(reference):
    """Validate the finite calibration reference schema without reading a file."""
    if not isinstance(reference, Mapping):
        raise ValueError('calibration must be a file or package/path mapping')
    if set(reference) == {'package', 'path'}:
        package, relative = reference['package'], reference['path']
        if (not isinstance(package, str) or not package or '/' in package or
                not isinstance(relative, str) or not relative or Path(relative).is_absolute() or
                '..' in Path(relative).parts):
            raise ValueError('invalid package calibration reference')
    elif (set(reference) != {'file'} or not isinstance(reference['file'], str) or
          not reference['file']):
        raise ValueError('unsupported calibration reference')


def _calibration(reference, source, share):
    validate_reference(reference)
    if 'package' in reference:
        path = Path(share(reference['package'])) / reference['path']
    else:
        path = Path(reference['file']).expanduser()
        if not path.is_absolute():
            path = Path(source).parent / path
    path = path.resolve(strict=True)
    raw = path.read_bytes()
    validate_calibration(raw, path)
    return {'path': str(path), 'sha256': sha256(raw).hexdigest()}


def resolve_lidar(selected='', deployment='', overrides=None, *, share=package_share,
                  required_frame='', fallback_ip=''):
    """Compile fallback_ip < defaults < deployment < operator file < explicit endpoint aliases.

    The package YAML has no address: ``fallback_ip`` is LIDAR_IP from boat-network.env.
    """
    paths = [Path(share('mhseals_nav')) / 'config/lidar.yaml']
    paths += [path for path in (deployment, selected) if path != '']
    values, origins, sources = {}, {}, []
    if fallback_ip:
        values['lidar.endpoint.ip'] = fallback_ip
        origins['lidar.endpoint.ip'] = 'boat-network.env LIDAR_IP'
    for path in paths:
        layer, identity = _layer(path)
        values.update(layer)
        origins.update(dict.fromkeys(layer, identity['path']))
        sources.append(identity)
    for alias, value in (overrides or {}).items():
        if alias not in ('velodyne_ip', 'velodyne_port') or not isinstance(value, str):
            raise ValueError('unknown or untyped lidar endpoint alias')
        if value == '':
            continue
        if alias == 'velodyne_port':
            if not value.isascii() or not value.isdecimal():
                raise ValueError('explicit lidar port must be a decimal integer')
            values['lidar.endpoint.port'] = int(value)
        else:
            values['lidar.endpoint.ip'] = value
        origins['lidar.endpoint.' + alias[len('velodyne_'):]] = '<explicit>'
    if 'lidar.endpoint.ip' not in values:
        raise ValueError('no lidar address: set LIDAR_IP in the vessel network env '
                         '($ASTRO_NETWORK_ENV, legacy $ROS_WS/config/boat-network.env) '
                         'or pass velodyne_ip:=')
    validate_values(values)
    if required_frame and values['lidar.frame'] != required_frame:
        raise ValueError('full robot lidar frame must match fixed URDF lidar_link contract')
    calibration = _calibration(values['lidar.calibration'], origins['lidar.calibration'], share)
    pair = {'model': values['lidar.model'], 'frame_id': values['lidar.frame']}
    return {'driver': dict(pair, rpm=float(values['lidar.rpm']),
                           device_ip=values['lidar.endpoint.ip'],
                           port=values['lidar.endpoint.port']),
            'transform': dict(pair, calibration=calibration['path']),
            'calibration': calibration, 'sources': sources, 'origins': origins}
