"""Bidirectional finite schema coverage, source provenance and compiler evidence separation."""
from hashlib import sha256
from pathlib import Path

import pytest
import yaml

from mhseals_nav import localization_configuration as loc, odometry_configuration as odom
from mhseals_nav import owner_catalog as owner, startup_catalog as catalog

ROOT = Path(__file__).parents[1]


@pytest.fixture
def resources():
    return {path: (ROOT / path).read_bytes() for path in
            (catalog.ODOMETRY_RESOURCE, *catalog.LOCALIZATION_RESOURCES.values())}


def bundles(resources):
    return catalog.startup_catalogs(resources, policy_raw=(ROOT / 'config/catalog.yaml').read_bytes())


def leaves(resources):
    return [owner._plain(leaf) for bundle, _, _ in bundles(resources).values()
            for leaf in bundle.descriptor['leaves']]


def test_every_default_exact_source_pointer_type_and_shared_rule(resources):
    fields = catalog.schema_fields()
    rows = [row for bundle, _, _ in bundles(resources).values() for row in bundle.rows]
    assert len(rows) == len(fields) == 66
    for row in rows:
        descriptor = row.descriptor
        key = descriptor['canonical_key']
        path, pointer, rule = fields[key]
        value = yaml.safe_load(resources[path])
        for part in pointer[1:].split('/'):
            value = value[part.replace('~1', '/').replace('~0', '~')]
        assert row.default == (tuple(value) if isinstance(value, list) else value)
        assert descriptor['default_ref']['source_hash'] == sha256(resources[path]).hexdigest()
        assert descriptor['default_ref']['line'] == catalog._locations(resources[path])[pointer]
        assert descriptor['value']['type'] == rule.kind
        assert descriptor['value']['units'] == 'unknown'
        assert catalog.constraint_details(descriptor['value']['constraint_ref']) == rule.describe()
        assert not row.writable and 'runtime-authority-not-provided' in row.blockers
        assert descriptor['consumers'][0]['candidate_tier'] == 'unsupported'
        assert any('restart-required' in dep for dep in
                   descriptor['consumers'][0]['dependencies'])
    assert catalog.audit_startup_catalog(resources, leaves(resources))


def test_known_constraints_and_unverified_consumer_claims(resources):
    rows = {row.descriptor['canonical_key']: row for bundle, _, _ in bundles(resources).values()
            for row in bundle.rows}
    assert catalog.RULES['odometry.mavros_velocity_source'].choices == ('gps', 'fcu')
    for key in odom.KEYS - {'mavros_velocity_source'}:
        rule = catalog.RULES['odometry.' + key]
        assert rule.minimum == 0 and rule.maximum is None
        assert rule.exclusive == (not key.startswith('ekf_delay_'))
    for name in loc.NODES:
        for key in loc.GRAPH[name]:
            assert catalog.RULES[f'localization.{name}.{key}'].choices == (loc.GRAPH[name][key],)
        for key in loc.VECTORS[name]:
            assert catalog.RULES[f'localization.{name}.{key}'].size == 15
        for key in loc.QUEUES[name]:
            rule = catalog.RULES[f'localization.{name}.{key}']
            assert (rule.minimum, rule.maximum) == (1, 2**63 - 1)
    for key in catalog.UNVERIFIED:
        row = rows['localization.' + key]
        assert row.descriptor['coverage'][0]['status'] == 'discovered'
        assert 'inventory-coverage:discovered' in row.blockers
        assert row.descriptor['evidence'][0]['required_type'] == 'consumer-unverified'
    assert set(catalog.UNSUPPORTED) == {'localization.covariance',
                                        'localization.additional_sensor_slots'}


@pytest.mark.parametrize('fault', ['orphan-yaml', 'compiler-key', 'stale-descriptor',
                                  'removed-source-key', 'hash', 'override', 'covariance'])
def test_bidirectional_mutations_refuse(resources, monkeypatch, fault):
    descriptors = leaves(resources)
    path = catalog.ODOMETRY_RESOURCE
    if fault == 'orphan-yaml':
        resources[path] += b'orphan: 3.0\n'
    elif fault == 'compiler-key':
        monkeypatch.setattr(odom, 'KEYS', odom.KEYS | {'new_compiler_key'})
    elif fault in ('stale-descriptor', 'covariance'):
        descriptors[0]['canonical_key'] = ('odometry.removed' if fault == 'stale-descriptor' else
                                          'localization.ekf_local.process_noise_covariance')
    elif fault == 'removed-source-key':
        data = yaml.safe_load(resources[path])
        del data['mavros_velocity_stddev']
        resources[path] = yaml.safe_dump(data).encode()
    elif fault == 'hash':
        descriptors[0]['default_ref']['source_hash'] = '0' * 64
    elif fault == 'override':
        routes = catalog.override_routes()
        routes['unknown_alias'] = ('odometry.ekf_delay_included',)
        with pytest.raises(ValueError, match='override'):
            catalog.audit_startup_catalog(resources, descriptors, routes=routes)
        return
    if fault in ('orphan-yaml', 'removed-source-key'):
        for leaf in descriptors:
            if leaf['default_ref']['path'] == path:
                leaf['default_ref']['source_hash'] = sha256(resources[path]).hexdigest()
    with pytest.raises(ValueError):
        catalog.audit_startup_catalog(resources, descriptors)


def test_missing_descriptor_requires_accountable_disposition(resources):
    descriptors = leaves(resources)
    removed = descriptors.pop()['canonical_key']
    with pytest.raises(ValueError):
        catalog.audit_startup_catalog(resources, descriptors)
    with pytest.raises(ValueError, match='disposition'):
        catalog.audit_startup_catalog(resources, descriptors, {removed: {'reason': 'unknown'}})
    disposition = {'reason': 'explicit bounded audit fixture: schema inventoried elsewhere',
                   'revision': catalog.REVISION, 'family': removed.split('.')[0]}
    assert catalog.audit_startup_catalog(resources, descriptors, {removed: disposition})


def test_selected_compiler_values_keep_actual_source_hashes_separate(resources, tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    operator = tmp_path / 'operator.yaml'
    operator.write_text('mavros_velocity_stddev: 2\nekf_delay_included: 7.25\n')
    selected = odom.resolve_odometry(ROOT / catalog.ODOMETRY_RESOURCE, operator_file=operator,
                                     aliases={'ekf_delay': '8.5'}, context='included')
    assert selected.delay == 8.5 and selected.values['mavros_velocity_stddev'] == 2.0
    assert selected.inputs[-1]['path'] == str(operator)
    assert selected.inputs[-1]['sha256'] == sha256(operator.read_bytes()).hexdigest()
    overlay = tmp_path / 'ekf.yaml'
    overlay.write_text('ekf_local: {frequency: 41}\n')
    compiled = loc.compile_localization({name: ROOT / path for name, path in
                                         catalog.LOCALIZATION_RESOURCES.items()}, str(overlay))
    assert compiled.parameters('ekf_local')['frequency'] == 41.0
    assert compiled.inputs[-1]['sha256'] == sha256(overlay.read_bytes()).hexdigest()
    result = bundles(resources)
    assert result['odometry'][0].field('odometry.mavros_velocity_stddev').default == yaml.safe_load(resources[catalog.ODOMETRY_RESOURCE])['mavros_velocity_stddev']
    assert result['localization'][0].field('localization.ekf_local.frequency').default == 30.0
    assert result['odometry'][0].field('odometry.mavros_velocity_stddev').descriptor[
        'default_ref']['source_hash'] == sha256(resources[catalog.ODOMETRY_RESOURCE]).hexdigest()


def test_source_union_retains_existing_three_families(resources, monkeypatch):
    monkeypatch.setattr(owner, 'package_share', lambda name: ROOT)
    union = owner.load_catalog()
    assert len(union.descriptor['default_sources']) == 7
    assert set(owner.FAMILIES) == {'nav2', 'mavros', 'lidar', 'odometry', 'localization'}
    assert union.field('lidar.rpm').default == 600.0
    assert union.field('mavros.intervals').default[0] == '33:20'
    assert union.field('navigation/controller_server/ros__parameters/controller_frequency')
    assert union.field('odometry.ekf_delay_included').default == 6.0
    assert union.field('localization.navsat_transform.map_frame').default == 'map'
    assert all(not row.writable for row in union.rows)


def test_generated_constraint_guide_is_current_and_complete(resources):
    guide = catalog.constraint_markdown(resources)
    assert (ROOT / 'docs/startup-catalog-constraints.md').read_text() == guide
    for key in catalog.schema_fields():
        assert '`' + key + '`' in guide
    assert 'exactly 15 actual booleans' in guide
    assert '9223372036854775807' in guide
    assert 'upstream consumption is unverified' in guide


def test_orphan_yaml_requires_explicit_accountable_unsupported_disposition(resources):
    descriptors = leaves(resources)
    path = catalog.ODOMETRY_RESOURCE
    resources[path] += b'future_unsupported: 2\n'
    for leaf in descriptors:
        if leaf['default_ref']['path'] == path:
            leaf['default_ref']['source_hash'] = sha256(resources[path]).hexdigest()
    with pytest.raises(ValueError, match='orphan'):
        catalog.audit_startup_catalog(resources, descriptors)
    assert catalog.audit_startup_catalog(resources, descriptors, {
        (path, '/future_unsupported'): {'reason': 'fixture: unsupported startup field; compiler refuses',
                                        'revision': catalog.REVISION, 'family': 'odometry'}})
    with pytest.raises(ValueError):
        bundles(resources)


def test_override_mapping_closes_existing_aliases_and_layer_selectors():
    routes = catalog.override_routes()
    assert set(odom.ALIASES) | set(odom.LAYER_ARGUMENTS) <= routes.keys()
    assert {name + '_config_file' for name in loc.NODES} <= routes.keys()
    assert routes['ekf_delay'] == routes['ekf_startup_context'] == (
        'odometry.ekf_delay_standalone', 'odometry.ekf_delay_included')
    assert all(target in catalog.schema_fields() for targets in routes.values() for target in targets)


def test_incomplete_known_localization_base_is_not_a_complete_catalog_default(resources):
    path = catalog.LOCALIZATION_RESOURCES['ekf_local']
    data = yaml.safe_load(resources[path])
    del data['/**']['ros__parameters']['frequency']
    resources[path] = yaml.safe_dump(data).encode()
    with pytest.raises(ValueError, match='complete localization'):
        bundles(resources)
