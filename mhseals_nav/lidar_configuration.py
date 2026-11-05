"""Resolve startup-only lidar layers and paired driver/transform projections.

Vessel facts come from the vessel profile (``profile_lidar``): the address, the model and the rpm
the sensor is set to, and the mount when the profile gives one (``profile_mount``, laid over the
description's lidar_* properties by robot_geometry; without it the description's mount stands).
config/lidar.yaml's ``driver:`` holds the rest; deployment and selected layers (``driver:``
overrides) and the velodyne_ip/velodyne_port aliases go on top. The model and rpm are the
profile's alone: a layer that still sets them is used only without a profile, with a warning.
"""
from collections.abc import Mapping
from ipaddress import IPv4Address
import logging
import math
from pathlib import Path
import string

import yaml

from . import boat_network, layers
from .layers import load_strict

log = logging.getLogger(__name__)
SECTIONS = {'tune', 'driver', 'costmap', 'fusion'}      # config/lidar.yaml
VESSEL = ('lidar.model', 'lidar.rpm')                   # the profile's alone

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
    """Load one strict layer's ``driver:`` into validated dotted keys; unknown keys are refused.

    A layer is lidar.yaml or an overlay of it; an old overlay's single ``lidar:`` mapping is read
    as ``driver:`` for one release, with a warning.
    """
    data = load_strict(raw, path)
    if isinstance(data, dict) and set(data) == {'lidar'}:
        log.warning('%s: the lidar: root is deprecated, write driver: (as config/lidar.yaml)', path)
        data = {'driver': data['lidar']}
    if not isinstance(data, dict) or not set(data) <= SECTIONS or not isinstance(
            data.get('driver', {}), dict):
        raise ValueError(f'expected a mapping of {", ".join(sorted(SECTIONS))} with a driver mapping')
    values = {}
    for key, value in data.get('driver', {}).items():
        if key == 'endpoint' and isinstance(value, dict):
            values.update({'lidar.endpoint.' + name: item for name, item in value.items()})
        else:
            values['lidar.' + key] = value
    validate_present(values)
    return values


def _layer(path):
    path, raw, digest = layers.read(path)
    return parse_lidar(raw, path), {'path': str(path), 'sha256': digest}


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
    path, raw, digest = layers.read(path)
    validate_calibration(raw, path)
    return {'path': str(path), 'sha256': digest}


def profile_facts(network_env=None):
    """The vessel profile's merged ``lidar:`` facts and its network env ({}, None without one).

    The vessel's directory holds its network env (``boat_network.path()``). From the profile it
    ``extends`` down to the vessel, each profile's lidar.yaml and then its profile.yaml ``lidar:``
    lay over the facts so far.
    """
    env = Path(network_env) if network_env else boat_network.path()
    if env is None or not env.is_file():
        return {}, None
    chain, here = [], env.parent
    while here is not None and len(chain) < 8 and here not in [d for d, _ in chain]:
        profile = _mapping(here / 'profile.yaml')
        chain.insert(0, (here, profile))
        here = here.parent / profile['extends'] if profile.get('extends') else None
    facts = {}
    for directory, profile in chain:
        facts.update(_mapping(directory / 'lidar.yaml'))
        facts.update(profile.get('lidar') or {})
    return facts, env


def profile_lidar(network_env=None):
    """The vessel profile's lidar facts as dotted keys ({} without a vessel profile).

    The address, the model and the rpm (``profile_facts``); ``$NAME`` values are read from the
    network env.
    """
    facts, env = profile_facts(network_env)
    if env is None:
        return {}
    address = string.Template(str(facts.get('address') or '')).safe_substitute(
        boat_network.values(env))
    found = {'lidar.endpoint.ip': address if address and '$' not in address else None,
             'lidar.model': facts.get('model'), 'lidar.rpm': facts.get('rpm')}
    return {key: value for key, value in found.items() if value is not None}


def profile_mount(network_env=None):
    """The vessel profile's lidar mount as xacro property overrides ({} when it gives none).

    ``lidar: {mount: {xyz: [x, y, z], rpy: [roll, pitch, yaw]}}``, either part alone, in the
    frame of the description's lidar_x/lidar_y/lidar_z/lidar_rpy properties (m and rad;
    blastoise.xacro: x, y from the datum, z above the sensor rail's foot). The description is
    the geometry source; where the profile gives a mount, the profile wins (robot_geometry).
    """
    mount = profile_facts(network_env)[0].get('mount')
    if mount is None:
        return {}
    if not isinstance(mount, dict) or not mount or not set(mount) <= {'xyz', 'rpy'}:
        raise ValueError('vessel profile lidar mount must be a mapping of xyz and/or rpy')
    for name, value in mount.items():
        if (not isinstance(value, list) or len(value) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            raise ValueError(f'vessel profile lidar mount {name} must be three finite numbers')
    properties = {}
    if 'xyz' in mount:
        properties.update(zip(('lidar_x', 'lidar_y', 'lidar_z'), map(float, mount['xyz'])))
    if 'rpy' in mount:
        properties['lidar_rpy'] = ' '.join(str(float(v)) for v in mount['rpy'])
    return properties


def _mapping(path):
    """A profile file's mapping, {} when it is missing or not one."""
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def resolve_lidar(selected='', deployment='', overrides=None, *, share=package_share,
                  required_frame='', profile=None):
    """Compile profile < defaults < deployment < operator file < explicit endpoint aliases.

    ``profile`` is the vessel's lidar facts (``profile_lidar()`` when None); the package YAML
    has no address, model or rpm. The model and rpm stay the profile's whatever a layer says.
    """
    profile = profile_lidar() if profile is None else dict(profile)
    validate_present(profile)
    paths = [Path(share('mhseals_nav')) / 'config/lidar.yaml']
    paths += [path for path in (deployment, selected) if path != '']
    values, origins, sources = dict(profile), dict.fromkeys(profile, 'vessel profile'), []
    for path in paths:
        layer, identity = _layer(path)
        for key in set(layer) & set(VESSEL):
            log.warning('%s: %s comes from the vessel profile; %s', identity['path'], key,
                        'ignored' if key in profile else 'used, as there is no profile value')
            if key in profile:
                del layer[key]
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
    missing = [key.split('.')[-1] for key in VESSEL if key not in values]
    if missing:
        raise ValueError(f"no lidar {', '.join(missing)}: the vessel profile's lidar.yaml "
                         '(hw.sh profile; its directory holds $ASTRO_NETWORK_ENV) sets them')
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
