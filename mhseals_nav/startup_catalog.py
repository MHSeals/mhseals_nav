"""Explicit startup inventories; raw defaults never stand for compiled or live values."""
from hashlib import sha256

from mhseals_config.catalog import DescriptorBundle
import yaml

from .catalog import _locations, _pointer
from . import localization_configuration as loc
from . import odometry_configuration as odom

REVISION = 'b30394bcfa083b62eadd0296ea64006dd6d29fd5+startup-catalog-v1'
ODOMETRY_RESOURCE = 'config/odometry_startup.yaml'
LOCALIZATION_RESOURCES = {name: 'config/' + name + '.yaml' for name in loc.NODES}
UNSUPPORTED = {
    'localization.covariance': {'reason': 'covariance matrices have no supported compiler schema',
                              'revision': REVISION, 'family': 'localization'},
    'localization.additional_sensor_slots': {
        'reason': 'only the explicit GRAPH and fusion-vector schema is supported',
        'revision': REVISION, 'family': 'localization'},
}
UNVERIFIED = frozenset(loc.LocalizationConfiguration.__dataclass_fields__[
    'unverified_consumers'].default)


def schema_fields():
    """Canonical fields mapped to their one raw default pointer and actual rule."""
    result = {'odometry.' + key: (ODOMETRY_RESOURCE, _pointer((key,)), odom.field_rule(key))
              for key in odom.KEYS}
    for name in loc.NODES:
        keys = (set(loc.GRAPH[name]) | loc.BOOLEANS[name] | loc.DOUBLES[name]
                | loc.VECTORS[name] | loc.QUEUES[name])
        for key in keys:
            result[f'localization.{name}.{key}'] = (
                LOCALIZATION_RESOURCES[name], _pointer(('/**', 'ros__parameters', key)),
                loc.field_rule(name, key))
    return result


# Semantic units describe parameter values, independently of validation bounds.
# A fusion mask is dimensionless: its booleans select states with mixed units.
ODOMETRY_UNITS = {
    'robot_state_publish_frequency': 'Hz', 'mavros_velocity_source': 'dimensionless',
    'mavros_velocity_stddev': 'm/s', 'mavros_velocity_max_age': 's',
    'mavros_velocity_min_stddev': 'm/s', 'mavros_velocity_no_fix_stddev': 'm/s',
    'ekf_delay_standalone': 's', 'ekf_delay_included': 's',
}
LOCALIZATION_DOUBLE_UNITS = {
    'frequency': 'Hz', 'sensor_timeout': 's', 'transform_time_offset': 's',
    'delay': 's', 'magnetic_declination_radians': 'rad', 'yaw_offset': 'rad',
}


def field_units(key):
    """Owned semantic units; unknown is reserved for fields lacking unit evidence."""
    if key.startswith('odometry.'):
        return ODOMETRY_UNITS[key.removeprefix('odometry.')]
    _, name, field = key.split('.', 2)
    if field in loc.GRAPH[name]:
        return 'frame' if field.endswith('_frame') else 'topic'
    if field in loc.BOOLEANS[name] or field in loc.VECTORS[name]:
        return 'dimensionless'
    if field in loc.QUEUES[name]:
        return 'count'
    if field in loc.DOUBLES[name]:
        return LOCALIZATION_DOUBLE_UNITS[field]
    raise ValueError('unsupported startup field: ' + key)


def override_routes():
    """Existing launch aliases select fields/context, never additional default leaves."""
    fields = schema_fields()
    result = {key: ('odometry.' + key,) for key in odom.ALIASES if key != 'ekf_delay'}
    result['ekf_delay'] = ('odometry.ekf_delay_standalone', 'odometry.ekf_delay_included')
    result['ekf_startup_context'] = result['ekf_delay']
    for selector in odom.LAYER_ARGUMENTS:
        result[selector] = tuple(sorted(key for key in fields if key.startswith('odometry.')))
    for name in loc.NODES:
        result[name + '_config_file'] = tuple(sorted(
            key for key in fields if key.startswith('localization.' + name + '.')))
    result['ekf_overlay_file'] = tuple(sorted(k for k in fields if k.startswith('localization.')))
    return result


RULES = {key: rule for key, (_, _, rule) in schema_fields().items()}
REGISTRY = {'odometry.validate': odom.validate, 'odometry.resolve': odom.resolve_odometry,
            'localization.validate': loc.validate, 'localization.compile': loc.compile_localization}
REGISTRY.update({'startup.rule.' + key: rule.check for key, rule in RULES.items()})


def constraint_details(reference):
    """Describe a resolvable constraint using the validator's own rule object."""
    prefix = 'startup.rule.'
    if not reference.startswith(prefix) or reference[len(prefix):] not in RULES:
        raise ValueError('unknown startup constraint reference')
    return RULES[reference[len(prefix):]].describe()


def _leaves(raw):
    data = yaml.load(raw, Loader=loc.Loader)
    result = set()

    def walk(value, parts=()):
        if isinstance(value, dict):
            for key, child in value.items():
                walk(child, (*parts, key))
        else:
            result.add(_pointer(parts))
    walk(data)
    return result


def audit_startup_catalog(resources, descriptors, dispositions=None, *, routes=None):
    """Bidirectional bounded YAML/compiler/override coverage; no generic AST claims."""
    fields = schema_fields()
    dispositions = dispositions or {}
    for key, value in dispositions.items():
        if (not isinstance(value, dict) or set(value) != {'reason', 'revision', 'family'}
                or not all(type(v) is str and v for v in value.values())
                or value['family'] not in ('odometry', 'localization')):
            raise ValueError('disposition requires reason, revision and accountable family')
        if key not in fields and not (isinstance(key, tuple) and len(key) == 2
                                      and key[0] in resources
                                      and key[1] in _leaves(resources[key[0]])):
            raise ValueError('unknown disposition field or YAML leaf: ' + str(key))
    seen, pointers = set(), set()
    for leaf in descriptors:
        key = leaf['canonical_key']
        if key not in fields or key in seen or key in dispositions:
            raise ValueError('stale, duplicate or ambiguous startup descriptor: ' + key)
        path, pointer, _ = fields[key]
        ref = leaf['default_ref']
        if (ref['path'] != path or ref['yaml_pointer'] != pointer or path not in resources
                or ref['source_id'] != 'startup-' + path.removeprefix('config/').removesuffix('.yaml')
                or ref['line'] != _locations(resources[path]).get(pointer)
                or sha256(resources[path]).hexdigest() != ref['source_hash']
                or pointer not in _leaves(resources[path])):
            raise ValueError('startup default pointer/hash mismatch: ' + key)
        seen.add(key)
        pointers.add((path, pointer))
    if seen | (dispositions.keys() & fields.keys()) != fields.keys():
        raise ValueError('compiler field missing descriptor or justified disposition')
    for key in dispositions:
        if key in fields:
            path, pointer, _ = fields[key]
            pointers.add((path, pointer))
        else:
            pointers.add(key)
    actual = {(path, pointer) for path, raw in resources.items() for pointer in _leaves(raw)}
    if pointers != actual:
        raise ValueError('orphan YAML leaf or missing source default')
    expected = override_routes()
    routes = expected if routes is None else routes
    if set(routes) != set(expected) or any(tuple(routes[key]) != expected[key] for key in expected):
        raise ValueError('unaccounted or unsupported launch override route')
    if any(target not in fields for targets in routes.values() for target in targets):
        raise ValueError('override targets unknown compiler field')
    return True


def _bundle(resources, family, policy, consumer_versions):
    fields = schema_fields()
    leaves, sources, pins = [], [], {}
    for path, raw in resources.items():
        source = 'startup-' + path.removeprefix('config/').removesuffix('.yaml')
        digest, locations = sha256(raw).hexdigest(), _locations(raw)
        sources.append({'id': source, 'resource': path, 'sha256': digest})
        closure = sorted(resource + pointer for resource, pointer, _ in fields.values()
                         if resource in resources)
        for key, (resource, pointer, rule) in sorted(fields.items()):
            if resource != path:
                continue
            if family == 'odometry':
                field = key.removeprefix('odometry.')
                consumer = ('robot_state_publisher' if field == 'robot_state_publish_frequency' else
                            'ekf_startup_timer' if field.startswith('ekf_delay_') else
                            'mavros_odom_adapter')
                outputs = ([name for name, value in odom.ADAPTER_PARAMETERS.items() if value == field]
                           or ['publish_frequency' if consumer == 'robot_state_publisher' else 'period'])
                site = ('odom.load_urdf -> robot_state_publisher.publish_frequency' if
                        consumer == 'robot_state_publisher' else
                        'ekf.ekf_nodes -> TimerAction.period; explicit context selects delay' if
                        consumer == 'ekf_startup_timer' else
                        'odom.odometry_nodes and mavros_odometry.main -> adapter parameter declarations')
                destination = ('explicit odometry_deployment_file / odometry_startup_file; '
                               'package < deployment < operator < supplied launch aliases')
                layers, status = ['package', 'deployment', 'operator'], 'startup-supported'
                absence = 'next-allowed-layer'
            else:
                _, consumer, field = key.split('.', 2)
                outputs = [field]
                unverified = consumer + '.' + field in UNVERIFIED
                status = ('discovered' if unverified else 'fixed-invariant' if
                          field in loc.GRAPH[consumer] else 'startup-supported')
                site = ('ekf.ekf_nodes -> compiled.parameters; local graph invariant where applicable; '
                        'upstream consumption unverified' if
                        unverified else 'ekf.ekf_nodes -> compiled.parameters startup projection only')
                destination = (consumer + '_config_file replaces packaged base; explicit '
                               'ekf_overlay_file or optional ~/.config/mhseals/ekf.yaml node overlay')
                layers, absence = ['package', 'operator'], 'domain-defined'
            pins[consumer] = 'unknown'
            compiler = 'odometry.resolve' if family == 'odometry' else 'localization.compile'
            reference = 'startup.rule.' + key
            leaves.append({
                'canonical_key': key, 'aliases': [],
                'default_ref': {'source_id': source, 'yaml_pointer': pointer, 'path': path,
                                'line': locations[pointer], 'source_hash': digest},
                'value': {'type': rule.kind, 'dimensions': [rule.size] if rule.size else [],
                          'units': field_units(key),
                          'constraint_ref': reference, 'validator_ref': compiler, 'inputs': closure},
                'authoring': dict(policy['authoring'], destination=destination, pointer=pointer,
                                  layers=layers, absence=absence, scopes=['baseline'],
                                  reset='unsupported',
                                  controlling_keys=[]),
                'consumers': [dict(policy['consumer'], id=consumer, version='unknown',
                                   parameters=outputs, compiler_ref=compiler, inputs=closure,
                                   outputs=outputs, dependencies=['fresh-start/restart-required; runtime restart unsupported',
                                    'complete startup input closure',
                                    'selected base/layer paths and hashes; context/aliases separate'])],
                'safety': policy['safety'],
                'evidence': [dict(policy['evidence'], recipe_ref=reference, consumer=consumer,
                                  version='unknown', required_type='consumer-unverified' if
                                  status == 'discovered' else 'startup-projection-only')],
                'coverage': [{'source': source, 'consumer': consumer, 'status': status,
                              'revision': REVISION, 'path': 'launch/ekf.launch.py' if
                              family == 'localization' or consumer == 'ekf_startup_timer' else
                              'launch/odom.launch.py',
                              'line': (29 if family == 'localization' else
                                       48 if consumer == 'ekf_startup_timer' else
                                       35 if consumer == 'robot_state_publisher' else 209),
                              'consume_site': site}],
            })
    data = {'schema_version': 1, 'owner_package': 'mhseals_nav', 'revision': REVISION,
            'consumer_versions': pins, 'default_sources': sources, 'leaves': leaves}
    raw_resources = {source['id']: resources[source['resource']] for source in sources}
    return DescriptorBundle.read(data, raw_resources, REGISTRY, dict(consumer_versions or {}))


def startup_catalogs(resources, *, policy_raw, consumer_versions=None):
    """Inventory all four complete package resources; explicit compile evidence stays separate."""
    if set(resources) != {ODOMETRY_RESOURCE, *LOCALIZATION_RESOURCES.values()}:
        raise ValueError('four complete startup resources required')
    odometry = yaml.load(resources[ODOMETRY_RESOURCE], Loader=loc.Loader)
    odom.validate(odometry)
    if set(odometry) != odom.KEYS:
        raise ValueError('complete odometry defaults required')
    for name, path in LOCALIZATION_RESOURCES.items():
        loc.parse_base(resources[path], name)
        expected = {pointer for resource, pointer, _ in schema_fields().values() if resource == path}
        if _leaves(resources[path]) != expected:
            raise ValueError('complete localization package defaults required: ' + name)
    policy = yaml.load(policy_raw, Loader=loc.Loader)
    if (not isinstance(policy, dict) or policy.get('schema_version') != 1
            or not {'authoring', 'consumer', 'safety', 'evidence'} <= policy.keys()):
        raise ValueError('unsupported startup catalog policy')
    result = {}
    for family, subset in (('odometry', {ODOMETRY_RESOURCE: resources[ODOMETRY_RESOURCE]}),
                            ('localization', {path: resources[path]
                                              for path in LOCALIZATION_RESOURCES.values()})):
        bundle = _bundle(subset, family, policy, consumer_versions)
        raw = {source['id']: subset[source['resource']]
               for source in bundle.descriptor['default_sources']}
        result[family] = (bundle, raw, REGISTRY)
    audit_startup_catalog(resources, [leaf for bundle, _, _ in result.values()
                                    for leaf in bundle.descriptor['leaves']])
    return result


def constraint_markdown(resources):
    """Generate read-only documentation from the same rules and actual default resources."""
    lines = ['# Startup catalog constraints (generated)', '',
             'Generated by `startup_catalog.constraint_markdown` from compiler Rule objects and '
             'owned semantic unit metadata and the four authoritative package YAML resources. Do not edit this table as schema.', '',
             'Units describe values: dimensionless includes booleans, source selectors and fusion masks; '
             'frame/topic identify names and count identifies queue entries. Unknown is reserved for '
             'missing unit evidence, not unitless values. Constraint prose stays separate.\n\n'
             'Every row is read-only: fresh-start/restart-required, no runtime authority. '
             'Numeric bounds are schema limits, not physical qualification. Consumer versions '
             'remain unknown. Selected compiled values and path/hash/layer/context evidence '
             'remain separate from these raw defaults.', '',
             'Odometry precedence: package < explicit deployment < explicit operator < supplied '
             'launch aliases. `ekf_delay` targets the context-selected delay field, not two defaults. '
             'No implicit home or session startup layer exists. Localization selected bases replace '
             'their respective packaged bases; explicit overlay or optional `~/.config/mhseals/ekf.yaml` '
             'overlays node fields. Null/unknown fields refuse; arrays replace whole; omission inherits '
             'only at an existing layer. No runtime writer or reset adapter is provided.', '',
             'Covariance matrices and additional sensor slots remain unsupported. Fusion-vector '
             'ordering is x/y/z, roll/pitch/yaw, linear velocities, angular velocities, accelerations. '
             'Navsat frames are local graph invariants whose upstream consumption is unverified; '
             'both EKF predict_to_current_time consumers are also unverified. Projection is not '
             'runtime behavior qualification.', '',
             '| Field | Raw default | Units | Compiler constraint | Consumer coverage |',
             '| --- | --- | --- | --- | --- |']
    for key, (path, pointer, rule) in sorted(schema_fields().items()):
        value = yaml.load(resources[path], Loader=loc.Loader)
        for part in pointer[1:].split('/'):
            value = value[part.replace('~1', '/').replace('~0', '~')]
        rule.check(value)
        consumer_field = key.removeprefix('localization.')
        status = ('discovered/unverified' if consumer_field in UNVERIFIED else
                  'fixed-invariant' if key.startswith('localization.') and
                  key.split('.', 2)[2] in loc.GRAPH[key.split('.', 2)[1]] else
                  'startup projection only')
        description = rule.describe().replace('|', '\\|')
        lines.append(f"| `{key}` | `{value!r}` | {field_units(key)} | {description} | {status} |")
    lines += ['', 'Raw source SHA256 (defaults remain attributed only to these bytes):', '']
    lines += [f"- `{path}`: `{sha256(raw).hexdigest()}`" for path, raw in sorted(resources.items())]
    return '\n'.join(lines) + '\n'
