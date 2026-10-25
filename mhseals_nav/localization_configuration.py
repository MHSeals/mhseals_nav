"""
Strict fresh-start inputs for the shipped robot_localization 3.8.3 graph.

Fusion vectors follow x/y/z, roll/pitch/yaw, linear velocities, angular
velocities, accelerations. Covariance and additional sensor slots are unsupported.
"""
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from types import MappingProxyType

import yaml

from .navigation_config import StrictOverlayLoader
from .startup_rules import Rule

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
           'ekf_global': {'odom0_config', 'imu0_config', 'odom1_config'},
           'navsat_transform': set()}
QUEUES = {'ekf_local': {'odom0_queue_size', 'imu0_queue_size'},
          'ekf_global': set(), 'navsat_transform': set()}


class Loader(StrictOverlayLoader):
    """Also reject nonstring keys before constructing dictionaries."""

    def construct_mapping(self, node, deep=False):
        if any(key.tag != 'tag:yaml.org,2002:str' for key, _ in node.value):
            raise ValueError('localization YAML requires string keys and no merges')
        return super().construct_mapping(node, deep=deep)


def field_rule(name, key):
    """Rules are the compiler schema."""
    if key in GRAPH[name]:
        return Rule('string', choices=(GRAPH[name][key],))
    if key in BOOLEANS[name]:
        return Rule('boolean')
    if key in VECTORS[name]:
        return Rule('sequence', size=15)
    if key in QUEUES[name]:
        return Rule('integer', minimum=1, maximum=2**63 - 1)
    if key in DOUBLES[name]:
        return Rule('number',
                    minimum=0 if key in ('frequency', 'delay', 'sensor_timeout') else None,
                    exclusive=key == 'frequency')
    raise ValueError('unsupported localization field: ' + name + '.' + key)


def validate(name, params):
    """Validate supplied fields independently before precedence can mask a fault."""
    allowed = set(GRAPH[name]) | BOOLEANS[name] | DOUBLES[name] | VECTORS[name] | QUEUES[name]
    if not isinstance(params, dict) or set(params) - allowed:
        raise ValueError('unsupported localization fields for ' + name)
    return {key: field_rule(name, key).check(value) for key, value in params.items()}


def parse_base(raw, name):
    """Each selected base must preserve the canonical fixed graph independently."""
    data = yaml.load(raw, Loader=Loader)
    if (not isinstance(data, dict) or set(data) != {'/**'}
            or not isinstance(data['/**'], dict) or set(data['/**']) != {'ros__parameters'}):
        raise ValueError('base must contain only /**.ros__parameters')
    params = validate(name, data['/**']['ros__parameters'])
    if not set(GRAPH[name]) <= set(params):
        raise ValueError('base must state complete fixed graph identities: ' + name)
    return params


def parse_overlay(raw):
    """Inherit for empty documents; reject node nulls and mixed wrappers."""
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
    unverified_consumers: tuple = (
        'navsat_transform.map_frame', 'navsat_transform.odom_frame',
        'navsat_transform.base_link_frame', 'ekf_local.predict_to_current_time',
        'ekf_global.predict_to_current_time')

    def parameters(self, name):
        """Return a fresh ROS-compatible projection, never the mutable YAML file."""
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
        selection = str(path)
        path = Path(path).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        inputs.append(MappingProxyType({
            'path': str(path), 'sha256': sha256(raw).hexdigest(),
            'kind': kind, 'explicit': explicit, 'selection': selection}))
        return raw

    for name in NODES:
        nodes[name] = parse_base(read(base_paths[name], name), name)
    explicit = overlay_selection != ''
    path = (Path(overlay_selection).expanduser() if explicit else
            Path.home() / '.config/mhseals/ekf.yaml')
    try:
        raw = read(path, 'overlay', explicit)
    except FileNotFoundError:
        if explicit or path.is_symlink():
            raise
        raw = b''
        inputs.append(MappingProxyType({
            'path': str(path), 'sha256': None, 'kind': 'overlay-absent', 'explicit': False}))
    for name, values in parse_overlay(raw).items():
        nodes[name].update(values)
    return LocalizationConfiguration(
        MappingProxyType({name: MappingProxyType(params) for name, params in nodes.items()}),
        tuple(inputs), use_sim_time)
