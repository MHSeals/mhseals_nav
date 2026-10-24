"""
Read-only Nav2 inventory from explicitly selected installed parameter bytes.

Plugin build contracts are unresolved; no ROS discovery or live authority.
The existing navigation compiler is the sole validator/lowering implementation.
"""
from hashlib import sha256

from mhseals_config.catalog import DescriptorBundle
import yaml

from .mavros_configuration import rates_from_yaml, resolve_setup, validate_rates
from .navigation_config import (LOWERING_INPUTS, mppi_controller, StrictOverlayLoader,
                                validate_navigation_base, validated_merge_limits)

REVISION = 'fbebfbc83148da71b08d420e0ab21def6883e852'
COMPILER = 'navigation.validated_merge_limits'
VALIDATOR = 'navigation.validate_navigation_base'
RECIPE = 'navigation.compiler-parity'
REGISTRY = {COMPILER: validated_merge_limits, VALIDATOR: validate_navigation_base}
# Recipe checks compilation only; upstream plugin behavior requires separate evidence.
REGISTRY[RECIPE] = validated_merge_limits
MAVROS_REVISION = '79992f4'
REGISTRY.update({'mavros.validate_rates': validate_rates,
                 'mavros.resolve_setup': resolve_setup,
                 'mavros.selected-rate-validation': rates_from_yaml})


def _pointer(parts):
    return '/' + '/'.join(str(p).replace('~', '~0').replace('/', '~1') for p in parts)


def _locations(raw):
    result = {}

    def walk(node, parts=()):
        result[_pointer(parts)] = node.start_mark.line + 1
        if isinstance(node, yaml.MappingNode):
            for key, value in node.value:
                walk(value, (*parts, key.value))
    walk(yaml.compose(raw, Loader=yaml.SafeLoader))
    return result


def navigation_catalog(raw, *, resource, policy_raw, consumer_versions=None):
    """
    Resolve catalog defaults from selected installed bytes, never source-tree paths.

    ``resource`` is package-relative (e.g. config/nav2_params.yaml). Version IDs
    are checked against declared pins; no upstream build has yet been qualified.
    The returned bundle is immutable inventory. Requested/effective provenance
    belongs to the existing compiler/snapshot, not to descriptor defaults.
    """
    policy = yaml.load(policy_raw, Loader=StrictOverlayLoader)
    if (set(policy) != {'schema_version', 'owner_package', 'revision', 'consumer_version',
                        'authoring', 'consumer', 'safety', 'evidence'}
            or policy['schema_version'] != 1):
        raise ValueError('unknown navigation catalog policy')
    base = yaml.load(raw, Loader=StrictOverlayLoader)
    validate_navigation_base(base)
    controller = base['controller_server']['ros__parameters']
    name = mppi_controller(controller)
    locations, digest = _locations(raw), sha256(raw).hexdigest()
    versions = dict(consumer_versions or {})
    ids = ['controller_server']
    if name:
        ids.append('mppi')
        if controller[name]['plugin'] == 'nav2_rotation_shim_controller::RotationShimController':
            ids.append('rotation_shim')
        if 'velocity_smoother' in base:
            ids.append('velocity_smoother')
    if 'behavior_server' in base:
        ids.append('behavior_server')
    pins = {cid: policy['consumer_version'] for cid in ids}
    cp = ('controller_server', 'ros__parameters')
    fp = (*cp, name) if name else ()
    inputs = [_pointer((*cp, 'controller_frequency'))]
    if name:
        inputs += [_pointer((*fp, key)) for key in LOWERING_INPUTS
                   if key != 'controller_frequency' and key in controller[name]]
    outputs = list(inputs)
    derived = ('model_dt', 'vx_std', 'vy_std', 'wz_std', 'prune_distance',
               'rotate_to_heading_angular_vel', 'max_angular_accel')
    if name:
        outputs += [_pointer((*fp, key)) for key in derived if key in controller[name]]
    smoother_keys = ('max_velocity', 'min_velocity', 'max_accel', 'max_decel')
    if name and 'velocity_smoother' in base:
        outputs += [_pointer(('velocity_smoother', 'ros__parameters', key))
                    for key in smoother_keys
                    if key in base['velocity_smoother']['ros__parameters']]
    # De-duplicate input/output overlap without changing deterministic order.
    outputs = list(dict.fromkeys(outputs))
    leaves = []

    def add(parts, consumers, *, controlling=()):
        pointer = _pointer(parts)
        value = base
        for part in parts:
            value = value[part]
        kind = ('sequence' if isinstance(value, list) else
                'integer' if type(value) is int else 'number')
        independent = parts[0] == 'behavior_server'
        closure_in = [pointer] if independent else inputs
        closure_out = [pointer] if independent else outputs
        leaves.append({
            'canonical_key': 'navigation' + pointer, 'aliases': [],
            'default_ref': {'source_id': 'selected-nav2', 'yaml_pointer': pointer,
                             'path': resource, 'line': locations[pointer], 'source_hash': digest},
            'value': {'type': kind, 'dimensions': [len(value)] if isinstance(value, list) else [],
                      'units': 'domain-defined', 'constraint_ref': VALIDATOR,
                      'validator_ref': COMPILER, 'inputs': closure_in},
            'authoring': dict(policy['authoring'], pointer=pointer,
                              layers=(['derived'] if controlling else
                                      ['package', 'operator', 'session']),
                              scopes=[] if controlling else ['baseline', 'session'],
                              controlling_keys=list(controlling)),
            'consumers': [dict(policy['consumer'], id=cid, version=pins[cid], parameters=[pointer],
                               compiler_ref=COMPILER, inputs=closure_in, outputs=closure_out,
                               dependencies=['selected-nav2-yaml']) for cid in consumers],
            'safety': policy['safety'],
            'evidence': [dict(policy['evidence'], consumer=cid, version=pins[cid])
                         for cid in consumers],
            'coverage': [{'source': 'selected-nav2', 'consumer': cid, 'status': 'traced',
                          'revision': REVISION, 'path': 'mhseals_nav/navigation_config.py',
                          'line': 346 if independent else 133,
                          'consume_site': ('independent behavior_server' if independent else
                                           'derived/lower_navigation')}
                         for cid in consumers]})

    add((*cp, 'controller_frequency'), ['controller_server'])
    if name:
        consumers = ['mppi'] + (['rotation_shim'] if 'rotation_shim' in ids else [])
        for key in dict.fromkeys((*LOWERING_INPUTS[1:], *derived)):
            if key in controller[name]:
                controls = {
                    'model_dt': (*cp, 'controller_frequency'),
                    'rotate_to_heading_angular_vel': (*fp, 'wz_max'),
                    'max_angular_accel': (*fp, 'az_max'),
                }
                controlling = ['navigation' + _pointer(controls[key])] if key in controls else []
                add((*fp, key), consumers, controlling=controlling)
        if 'velocity_smoother' in ids:
            for key in smoother_keys:
                if key in base['velocity_smoother']['ros__parameters']:
                    add(('velocity_smoother', 'ros__parameters', key), ['velocity_smoother'],
                        controlling=['navigation' + p for p in inputs])
    behavior = base.get('behavior_server', {}).get('ros__parameters', {})
    for key in ('max_rotational_vel', 'min_rotational_vel', 'rotational_acc_lim'):
        if key in behavior:
            add(('behavior_server', 'ros__parameters', key), ['behavior_server'])
    data = {'schema_version': 1, 'owner_package': policy['owner_package'],
            'revision': policy['revision'],
            'consumer_versions': pins,
            'default_sources': [{'id': 'selected-nav2', 'resource': resource,
                                 'sha256': digest}],
            'leaves': leaves}
    return DescriptorBundle.read(data, {'selected-nav2': raw}, REGISTRY, versions)


def mavros_catalog(raw, *, resource, policy_raw, consumer_versions=None):
    """Describe selected rate bytes, with requested/unverified evidence only."""
    rates_from_yaml(raw)
    policy = yaml.load(policy_raw, Loader=StrictOverlayLoader)
    if (not isinstance(policy, dict) or set(policy) != {
            'schema_version', 'owner_package', 'revision', 'consumer_version',
            'authoring', 'consumer', 'safety', 'evidence'} or policy['schema_version'] != 1):
        raise ValueError('unknown MAVROS catalog policy')
    # Keep the existing package policy shape and unknown safety/version blockers.
    pointer = '/mavros_setup/ros__parameters/message_rates'
    digest = sha256(raw).hexdigest()
    validator = 'mavros.validate_rates'
    compiler = 'mavros.resolve_setup'
    recipe = 'mavros.selected-rate-validation'
    source = 'selected-mavros-setup'
    consumer = 'mavros_setup'
    version = 'unknown'
    leaf = {
        'canonical_key': 'mavros.intervals', 'aliases': [],
        'default_ref': {'source_id': source, 'yaml_pointer': pointer, 'path': resource,
                        'line': _locations(raw)[pointer], 'source_hash': digest},
        'value': {'type': 'sequence', 'dimensions': [len(rates_from_yaml(raw))],
                  'units': 'ID:Hz; positive=request, 0=reset, -1=disable',
                  'constraint_ref': validator, 'validator_ref': validator, 'inputs': [pointer]},
        'authoring': dict(policy['authoring'], destination=resource, pointer=pointer,
                          layers=['package', 'deployment', 'profile', 'session'],
                          absence='required', scopes=['baseline', 'session'], controlling_keys=[]),
        'consumers': [dict(policy['consumer'], id=consumer, version=version,
                           parameters=[pointer], compiler_ref=compiler, inputs=[pointer],
                           outputs=['/mavros/set_message_interval'],
                           dependencies=['selected-mavros-setup-yaml', 'FCU-target-system-1',
                                         'autopilot-component-1', 'connection-and-boot-epoch'])],
        'safety': policy['safety'],
        'evidence': [dict(policy['evidence'], recipe_ref=recipe, consumer=consumer,
                          version=version, required_type='requested/unverified')],
        'coverage': [{'source': source, 'consumer': consumer, 'status': 'startup-supported',
                      'revision': MAVROS_REVISION, 'path': 'launch/odom.launch.py',
                      'line': 71,
                      'consume_site': 'create_mavros_setup_node; reconnect request_rates'}]}
    data = {'schema_version': 1, 'owner_package': 'mhseals_nav', 'revision': MAVROS_REVISION,
            'consumer_versions': {consumer: version},
            'default_sources': [{'id': source, 'resource': resource, 'sha256': digest}],
            'leaves': [leaf]}
    return DescriptorBundle.read(data, {source: raw}, REGISTRY, dict(consumer_versions or {}))
