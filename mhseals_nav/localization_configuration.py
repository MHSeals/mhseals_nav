"""Strict fresh-start inputs for the shipped robot_localization 3.8.3 graph.

Fusion vectors follow x/y/z, roll/pitch/yaw, linear velocities, angular
velocities, accelerations. Covariance and additional sensor slots are unsupported.
"""
from dataclasses import dataclass
from hashlib import sha256
import math
from pathlib import Path
from types import MappingProxyType

import yaml

from .navigation_config import StrictOverlayLoader

NODES = ('ekf_local', 'ekf_global', 'navsat_transform')
FRAMES = {'map_frame': 'map', 'odom_frame': 'odom', 'base_link_frame': 'base_link'}
GRAPH = {
    'ekf_local': dict(FRAMES, world_frame='odom', odom0='/odom/mavros', imu0='/imu/raw'),
    'ekf_global': dict(FRAMES, world_frame='map', odom0='/odom/mavros', imu0='/imu/raw',
                       odom1='/odom/gps'),
    'navsat_transform': dict(FRAMES),
}
BOOLEANS = {
    'ekf_local': {'two_d_mode', 'publish_tf', 'publish_acceleration', 'print_diagnostics',
                  'predict_to_current_time', 'odom0_differential', 'odom0_relative',
                  'imu0_differential', 'imu0_relative', 'imu0_remove_gravitational_acceleration'},
    'ekf_global': {'two_d_mode', 'publish_tf', 'publish_acceleration', 'print_diagnostics',
                   'predict_to_current_time', 'odom0_differential', 'odom1_differential',
                   'imu0_differential', 'imu0_relative', 'imu0_remove_gravitational_acceleration'},
    'navsat_transform': {'zero_altitude', 'broadcast_utm_transform', 'publish_filtered_gps',
                        'use_odometry_yaw', 'wait_for_datum'},
}
DOUBLES = {name: {'frequency', 'sensor_timeout', 'transform_time_offset'} for name in NODES[:2]}
DOUBLES['navsat_transform'] = {'frequency', 'delay', 'magnetic_declination_radians', 'yaw_offset'}
VECTORS = {'ekf_local': {'odom0_config', 'imu0_config'},
           'ekf_global': {'odom0_config', 'imu0_config', 'odom1_config'}, 'navsat_transform': set()}
QUEUES = {'ekf_local': {'odom0_queue_size', 'imu0_queue_size'},
          'ekf_global': set(), 'navsat_transform': set()}


class Loader(StrictOverlayLoader):
    """Also reject nonstring keys before constructing dictionaries."""

    def construct_mapping(self, node, deep=False):
        if any(key.tag != 'tag:yaml.org,2002:str' for key, _ in node.value):
            raise ValueError('localization YAML requires string keys and no merges')
        return super().construct_mapping(node, deep=deep)


def validate(name, params):
    """Validate supplied fields independently before precedence can mask a fault."""
    allowed = set(GRAPH[name]) | BOOLEANS[name] | DOUBLES[name] | VECTORS[name] | QUEUES[name]
    if not isinstance(params, dict) or set(params) - allowed:
        raise ValueError('unsupported localization fields for ' + name)
    checked = {}
    for key, value in params.items():
        if key in GRAPH[name]:
            if type(value) is not str or value != GRAPH[name][key]:
                raise ValueError('unsupported fixed graph identity: ' + name + '.' + key)
        elif key in BOOLEANS[name]:
            if type(value) is not bool:
                raise ValueError('expected actual boolean: ' + key)
        elif key in VECTORS[name]:
            if type(value) is not list or len(value) != 15 or any(type(v) is not bool for v in value):
                raise ValueError('fusion vector must contain exactly 15 actual booleans: ' + key)
            value = tuple(value)
        elif key in QUEUES[name]:
            if type(value) is not int or value <= 0:
                raise ValueError('queue size must be a positive integer: ' + key)
        else:
            try:
                valid = type(value) in (int, float) and math.isfinite(value)
            except OverflowError:
                valid = False
            if (not valid or key == 'frequency' and value <= 0
                    or key in ('delay', 'sensor_timeout') and value < 0):
                raise ValueError('invalid localization double: ' + key)
            value = float(value)
        checked[key] = value
    return checked


def parse_overlay(raw):
    """An empty document inherits; node nulls and mixed wrappers do not."""
    data = yaml.load(raw, Loader=Loader)
    if data is None and yaml.compose(raw) is None:
        return {}
    if not isinstance(data, dict) or set(data) - set(NODES):
        raise ValueError('localization overlay must name only the three supported nodes')
    result = {}
    for name, section in data.items():
        if not isinstance(section, dict):
            raise ValueError('localization node section must be a mapping')
        if 'ros__parameters' in section:
            if set(section) != {'ros__parameters'}:
                raise ValueError('mixed localization wrapper')
            section = section['ros__parameters']
        result[name] = validate(name, section)
    return result


@dataclass(frozen=True)
class LocalizationConfiguration:
    nodes: object
    inputs: tuple
    use_sim_time: bool

    def parameters(self, name):
        """A fresh ROS-compatible projection, never the original mutable YAML file."""
        return dict({k: list(v) if isinstance(v, tuple) else v
                     for k, v in self.nodes[name].items()}, use_sim_time=self.use_sim_time)


def simulation_clock(value):
    """No unknown clock spelling silently selects wall time."""
    if type(value) is str and value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    raise ValueError('sim must be true or false')


def compile_localization(base_paths, overlay_selection='', use_sim_time=False):
    """Compile the complete closure before any launch object is constructed."""
    if type(use_sim_time) is not bool or set(base_paths) != set(NODES):
        raise ValueError('three explicit bases and an actual boolean clock required')
    nodes, inputs = {}, []

    def read(path, kind, explicit=True):
        path = Path(path).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        inputs.append(MappingProxyType(dict(path=str(path), sha256=sha256(raw).hexdigest(),
                                             kind=kind, explicit=explicit)))
        return raw

    for name in NODES:
        data = yaml.load(read(base_paths[name], name), Loader=Loader)
        if (not isinstance(data, dict) or set(data) != {'/**'}
                or not isinstance(data['/**'], dict) or set(data['/**']) != {'ros__parameters'}):
            raise ValueError('base must contain only /**.ros__parameters')
        nodes[name] = validate(name, data['/**']['ros__parameters'])
        if not set(GRAPH[name]) <= set(nodes[name]):
            raise ValueError('base must state complete fixed graph identities: ' + name)
    explicit = overlay_selection != ''
    path = Path(overlay_selection).expanduser() if explicit else Path.home() / '.config/mhseals/ekf.yaml'
    try:
        raw = read(path, 'overlay', explicit)
    except FileNotFoundError:
        if explicit or path.is_symlink():
            raise
        raw = b''
        inputs.append(MappingProxyType(dict(path=str(path), sha256=None,
                                             kind='overlay-absent', explicit=False)))
    for name, values in parse_overlay(raw).items():
        nodes[name].update(values)
    return LocalizationConfiguration(
        MappingProxyType({name: MappingProxyType(params) for name, params in nodes.items()}),
        tuple(inputs), use_sim_time)
