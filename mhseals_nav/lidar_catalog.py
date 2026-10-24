"""Read-only finite lidar startup inventory; no live sensor qualification."""
from hashlib import sha256

from mhseals_config.catalog import DescriptorBundle
from mhseals_config.resolver import resolve

import yaml

from .catalog import _locations
from .lidar_configuration import parse_lidar, resolve_lidar, validate_values
from .navigation_config import StrictOverlayLoader

REVISION = '8ad272e9bd1cd4ddca5dafb39ea9dfbb2075dc07'
VALIDATOR = 'lidar.validate_values'
COMPILER = 'lidar.resolve_lidar'
REGISTRY = {VALIDATOR: validate_values, COMPILER: resolve_lidar}


def lidar_catalog(raw, *, resource, policy_raw, consumer_versions=None):
    """Read complete installed baseline bytes; layer origins stay in resolver."""
    validate_values(resolve([parse_lidar(raw, resource, 'defaults')]).effective)
    policy = yaml.load(policy_raw, Loader=StrictOverlayLoader)
    if not isinstance(policy, dict) or policy.get('schema_version') != 1:
        raise ValueError('unsupported lidar catalog policy')
    source, digest = 'lidar-package-defaults', sha256(raw).hexdigest()
    locations = _locations(raw)
    closure = ['/lidar/' + key for key in ('model', 'frame', 'rpm', 'calibration', 'endpoint')]
    consumers = ('velodyne_driver', 'velodyne_transform', 'sensor_factory')
    pins = dict.fromkeys(consumers, 'unknown')
    leaves = []
    dependencies = ['installed-lidar-defaults', 'selected-deployment-and-operator-layers',
                    'validated-calibration-path-and-sha256', 'TF-URDF-contract-pending-M06']

    def add(key, kind, units, consumers, outputs):
        pointer = '/lidar/' + key
        leaves.append({
            'canonical_key': 'lidar.' + key, 'aliases': [],
            'default_ref': {'source_id': source, 'yaml_pointer': pointer, 'path': resource,
                            'line': locations[pointer], 'source_hash': digest},
            'value': {'type': kind, 'dimensions': [], 'units': units,
                      'constraint_ref': VALIDATOR, 'validator_ref': COMPILER, 'inputs': closure},
            'authoring': dict(policy['authoring'], destination=resource, pointer=pointer,
                              layers=['package', 'deployment', 'operator', 'session'],
                              absence='next-allowed-layer', scopes=['baseline', 'session'],
                              controlling_keys=[]),
            'consumers': [dict(policy['consumer'], id=cid, version=pins[cid],
                               parameters=outputs, compiler_ref=COMPILER, inputs=closure,
                               outputs=outputs, dependencies=dependencies) for cid in consumers],
            'safety': policy['safety'],
            'evidence': [dict(policy['evidence'], recipe_ref=VALIDATOR, consumer=cid,
                              version=pins[cid], required_type='startup-inputs/unverified')
                         for cid in consumers],
            'coverage': [{'source': source, 'consumer': cid, 'status': 'startup-supported',
                          'revision': REVISION, 'path': 'launch/sensors.launch.py', 'line': 41,
                          'consume_site': 'sensor_nodes paired projection; startup only'}
                         for cid in consumers]})

    pair = ['velodyne_driver', 'velodyne_transform']
    add('model', 'string', 'supported-model-enum', pair, ['model'])
    add('frame', 'string', 'ROS-frame; standalone-local/full-robot-fixed', pair, ['frame_id'])
    add('rpm', 'number', 'revolutions/minute', ['velodyne_driver'], ['rpm'])
    add('calibration', 'mapping', 'package-or-file-ref; resolved-path-and-sha256',
        ['velodyne_transform'], ['calibration'])
    add('endpoint', 'mapping', 'IPv4-address/UDP-port', ['velodyne_driver'], ['device_ip', 'port'])
    # Inventoried adjacent launch fields have no claimed YAML/default authority yet.
    for key, parameters in (('sensors.selection', ['sim', 'sensors_ignore']),
                            ('sensors.sim_endpoint', ['ros_ip', 'ros_port'])):
        leaves.append({
            'canonical_key': key, 'aliases': [],
            'consumers': [dict(policy['consumer'], id='sensor_factory', version='unknown',
                               parameters=parameters, inputs=parameters, outputs=parameters,
                               compiler_ref='unsupported', dependencies=['launch-arguments'])],
            'coverage': [{'source': 'launch/sensors.launch.py', 'consumer': 'sensor_factory',
                          'status': 'unsupported', 'revision': REVISION,
                          'path': 'launch/sensors.launch.py', 'line': 26,
                          'consume_site': 'existing launch-only selection/simulation endpoint'}]})
    data = {'schema_version': 1, 'owner_package': 'mhseals_nav', 'revision': REVISION,
            'consumer_versions': pins,
            'default_sources': [{'id': source, 'resource': resource, 'sha256': digest}],
            'leaves': leaves}
    return DescriptorBundle.read(data, {source: raw}, REGISTRY, dict(consumer_versions or {}))
