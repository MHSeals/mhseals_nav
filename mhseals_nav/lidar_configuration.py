"""Resolve startup-only lidar layers and paired driver/transform projections."""
from collections.abc import Mapping
from hashlib import sha256
from ipaddress import IPv4Address
import math
from pathlib import Path

from mhseals_config.resolver import Field, parse, resolve

import yaml

from .navigation_config import StrictOverlayLoader

FIELDS = {'lidar.model': Field(str), 'lidar.frame': Field(str),
          'lidar.rpm': Field(float), 'lidar.calibration': Field(dict),
          'lidar.endpoint.ip': Field(str), 'lidar.endpoint.port': Field(int)}
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


def validate_values(values):
    """Validate the finite supported VLP16 projection, not firmware qualification."""
    if set(values) != set(FIELDS) or values['lidar.model'] != 'VLP16':
        raise ValueError('complete VLP16 lidar configuration required')
    frame = values['lidar.frame']
    if not frame or frame.strip() != frame or any(char.isspace() for char in frame):
        raise ValueError('lidar frame must be nonempty and contain no whitespace')
    positive(values['lidar.rpm'], 'rpm')
    IPv4Address(values['lidar.endpoint.ip'])
    port = values['lidar.endpoint.port']
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('lidar UDP port must be an integer in 1..65535')


def validate_calibration(raw):
    """Accept the bounded VLP16db correction schema and all sixteen unique lasers."""
    data = yaml.load(raw, Loader=StrictOverlayLoader)
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


def parse_lidar(raw, source, layer):
    """Parse located strict leaves using the core layer contract."""
    # The core parser has exact scalar types; RPM deliberately supports int/float.
    data = yaml.load(raw, Loader=StrictOverlayLoader)
    rpm = data.get('lidar', {}).get('rpm') if isinstance(data, dict) and isinstance(
        data.get('lidar'), dict) else None
    fields = dict(FIELDS)
    if rpm is not None:
        positive(rpm, 'rpm')
        fields['lidar.rpm'] = Field(type(rpm))
    return parse(raw.decode(), source, layer, fields)


def _layer(path, layer):
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    return parse_lidar(raw, str(path), layer), {'path': str(path), 'sha256': sha256(raw).hexdigest(), 'layer': layer}


def _calibration(reference, source, share):
    if not isinstance(reference, Mapping):
        raise ValueError('calibration must be a file or package/path mapping')
    if set(reference) == {'package', 'path'}:
        package, relative = reference['package'], reference['path']
        if (not isinstance(package, str) or not package or '/' in package or
                not isinstance(relative, str) or Path(relative).is_absolute() or
                '..' in Path(relative).parts):
            raise ValueError('invalid package calibration reference')
        path = Path(share(package)) / relative
    elif set(reference) == {'file'} and isinstance(reference['file'], str) and reference['file']:
        path = Path(reference['file']).expanduser()
        if not path.is_absolute():
            path = Path(source).parent / path
    else:
        raise ValueError('unsupported calibration reference')
    path = path.resolve(strict=True)
    raw = path.read_bytes()
    validate_calibration(raw)
    return {'path': str(path), 'sha256': sha256(raw).hexdigest()}


def resolve_lidar(selected='', deployment='', overrides=None, *, share=package_share,
                  required_frame=''):
    """Compile defaults < deployment < selected < explicit endpoint aliases."""
    sources, layers = [], []
    paths = [(Path(share('mhseals_nav')) / 'config/lidar.yaml', 'defaults')]
    paths += [(path, layer) for path, layer in ((deployment, 'deployment'),
                                               (selected, 'operator')) if path != '']
    for path, layer in paths:
        assignments, identity = _layer(path, layer)
        layers.append(assignments)
        sources.append(identity)
    endpoint = {}
    for alias, value in (overrides or {}).items():
        if alias not in ('velodyne_ip', 'velodyne_port') or not isinstance(value, str):
            raise ValueError('unknown or untyped lidar endpoint alias')
        if value == '':
            continue
        if alias == 'velodyne_port':
            if not value.isascii() or not value.isdecimal():
                raise ValueError('explicit lidar port must be a decimal integer')
            endpoint['port'] = int(value)
        else:
            endpoint['ip'] = value
    if endpoint:
        layers.append(parse(yaml.safe_dump({'lidar': {'endpoint': endpoint}}),
                            '<explicit-endpoint-aliases>', 'session', FIELDS))
    resolution = resolve(layers)
    values = resolution.effective
    validate_values(values)
    if required_frame and values['lidar.frame'] != required_frame:
        raise ValueError('full robot lidar frame must match fixed URDF lidar_link contract')
    calibration_source = resolution.origins['lidar.calibration'][-1].source
    calibration = _calibration(values['lidar.calibration'], calibration_source, share)
    pair = {'model': values['lidar.model'], 'frame_id': values['lidar.frame']}
    return {'driver': dict(pair, rpm=float(values['lidar.rpm']),
                           device_ip=values['lidar.endpoint.ip'], port=values['lidar.endpoint.port']),
            'transform': dict(pair, calibration=calibration['path']),
            'calibration': calibration, 'sources': sources, 'resolution': resolution}
