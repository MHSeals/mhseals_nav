"""Installed complete owner inventory and collision refusal, without live claims."""
from hashlib import sha256
from pathlib import Path
import sys

from mhseals_config.catalog import DescriptorBundle
from mhseals_config.resolver import resolve
from mhseals_nav import owner_catalog as owner
from mhseals_nav.lidar_configuration import parse_lidar
import pytest

SHARE = Path(sys.prefix) / 'share/mhseals_nav'


@pytest.fixture
def installed(monkeypatch):
    """Use normal installed package resources even on a host without ament."""
    assert 'site-packages' in owner.__file__
    monkeypatch.setattr(owner, 'package_share', lambda package: SHARE)
    return SHARE


def families(share):
    """Build explicit validated family inputs using installed bytes."""
    policy = (share / 'config/catalog.yaml').read_bytes()
    result = {}
    for name, filename, loader, bindings in (
            ('nav2', 'nav2_params.yaml', owner.navigation_catalog, owner.REGISTRY),
            ('mavros', 'mavros_setup.yaml', owner.mavros_catalog, owner.REGISTRY),
            ('lidar', 'lidar.yaml', owner.lidar_catalog, owner.LIDAR_REGISTRY)):
        raw = (share / 'config' / filename).read_bytes()
        bundle = loader(raw, resource='config/' + filename, policy_raw=policy)
        source = bundle.descriptor['default_sources'][0]['id']
        result[name] = (bundle, {source: raw}, dict(bindings))
    from mhseals_nav.startup_catalog import (
        startup_catalogs, ODOMETRY_RESOURCE, LOCALIZATION_RESOURCES)
    resources = {path: (share / path).read_bytes()
                 for path in (ODOMETRY_RESOURCE, *LOCALIZATION_RESOURCES.values())}
    result.update(startup_catalogs(resources, policy_raw=policy))
    return result


def mutate(inputs, family, edit):
    """Provide a structurally valid family whose union identity may conflict."""
    bundle, resources, registry = inputs[family]
    data = owner._plain(bundle.descriptor)
    edit(data)
    inputs[family] = (DescriptorBundle.read(data, resources, registry, {}),
                      resources, registry)


def test_complete_deterministic_installed_provider(installed):
    """One owner retains every family, revision, source hash and blocker."""
    first = owner.load_catalog()
    inputs = families(installed)
    assert first == owner.combine_catalogs(dict(reversed(list(inputs.items()))))
    assert len(first.rows) == sum(len(item[0].rows) for item in inputs.values())
    assert first.field('mavros.intervals').default[0] == '33:20'
    assert first.field('lidar.rpm').default == 600.0
    assert first.field('navigation/controller_server/ros__parameters/controller_frequency')
    for family, (bundle, _, _) in inputs.items():
        assert family + '@' + bundle.descriptor['revision'] in first.descriptor['revision']
    assert len(first.descriptor['default_sources']) == 7
    assert first.field('odometry.ekf_delay_included').default == 6.0
    assert first.field('localization.ekf_local.frequency').default == 30.0
    guide = installed / 'docs/startup-catalog-constraints.md'
    assert guide.is_file() and 'exactly 15 actual booleans' in guide.read_text()
    for key in ('odometry.mavros_velocity_source', 'localization.navsat_transform.map_frame'):
        row = first.field(key)
        ref = row.descriptor['default_ref']
        assert ref['source_hash'] == sha256((installed / ref['path']).read_bytes()).hexdigest()
    assert all(not row.writable for row in first.rows)
    assert all('runtime-authority-not-provided' in row.blockers for row in first.rows)


def test_explicit_selected_bytes_and_separate_lidar_origins(installed):
    """Requested Nav2/MAVROS bytes do not relabel lidar runtime layers as defaults."""
    nav = (installed / 'config/nav2_params.yaml').read_bytes()
    nav = nav.replace(b'controller_frequency: 10.0', b'controller_frequency: 21.0')
    mav = (installed / 'config/mavros_setup.yaml').read_bytes().replace(b'27:10', b'27:7')
    bundle = owner.load_catalog(nav2=('config/requested-nav.yaml', nav),
                                mavros=('config/requested-mav.yaml', mav))
    row = bundle.field('navigation/controller_server/ros__parameters/controller_frequency')
    assert row.default == 21.0
    assert row.descriptor['default_ref']['source_hash'] == sha256(nav).hexdigest()
    assert row.descriptor['default_ref']['path'] == 'config/requested-nav.yaml'
    row = bundle.field('mavros.intervals')
    assert row.default[4] == '27:7'
    assert row.descriptor['default_ref']['source_hash'] == sha256(mav).hexdigest()
    assert row.descriptor['default_ref']['path'] == 'config/requested-mav.yaml'
    assert row.descriptor['evidence'][0]['required_type'] == 'requested/unverified'
    raw = (installed / 'config/lidar.yaml').read_bytes()
    selected = resolve([parse_lidar(raw, 'installed/lidar.yaml', 'defaults'),
                        parse_lidar(b'lidar: {rpm: 450.0}', 'deployment.yaml', 'deployment')])
    assert selected.effective['lidar.rpm'] == 450.0
    assert selected.origins['lidar.rpm'][-1].source == 'deployment.yaml'
    assert bundle.field('lidar.rpm').default == 600.0
    assert bundle.field('lidar.rpm').descriptor['default_ref']['path'] == 'config/lidar.yaml'


@pytest.mark.parametrize('kind', ['alias', 'canonical', 'source', 'registry', 'version'])
def test_conflicting_identities_refuse(installed, kind):
    """Never overwrite a family identity; identical shared callables alone may share."""
    inputs = families(installed)
    if kind in ('alias', 'canonical'):
        key = inputs['nav2'][0].rows[0].descriptor['canonical_key']

        def edit(data):
            if kind == 'alias':
                data['leaves'][0]['aliases'] = [key]
            else:
                data['leaves'][0]['canonical_key'] = key
        mutate(inputs, 'lidar', edit)
    elif kind == 'version':
        mutate(inputs, 'lidar', lambda data: data['consumer_versions'].update(
            {'controller_server': 'conflicting'}))
    elif kind == 'registry':
        inputs['mavros'][2]['navigation.validated_merge_limits'] = lambda: None
    else:
        bundle, resources, registry = inputs['lidar']
        data = owner._plain(bundle.descriptor)
        old = data['default_sources'][0]['id']
        data['default_sources'][0]['id'] = 'selected-nav2'
        for leaf in data['leaves']:
            if 'default_ref' in leaf:
                leaf['default_ref']['source_id'] = 'selected-nav2'
        resources = {'selected-nav2': resources[old]}
        inputs['lidar'] = (DescriptorBundle.read(data, resources, registry, {}),
                           resources, registry)
    with pytest.raises(ValueError, match='conflicting'):
        owner.combine_catalogs(inputs)


@pytest.mark.parametrize('filename', [
    'catalog.yaml', 'nav2_params.yaml', 'mavros_setup.yaml', 'lidar.yaml', 'odometry_startup.yaml',
    'ekf_local.yaml', 'ekf_global.yaml', 'navsat_transform.yaml'])
def test_missing_malformed_and_invalid_family_fail_visibly(
        installed, monkeypatch, tmp_path, filename):
    """Required family failures cannot return a partial inventory labeled complete."""
    inputs = families(installed)
    del inputs['lidar']
    with pytest.raises(ValueError, match='complete'):
        owner.combine_catalogs(inputs)
    empty = families(installed)
    mutate(empty, 'lidar', lambda data: data.update(leaves=[]))
    with pytest.raises(ValueError, match='empty required family'):
        owner.combine_catalogs(empty)
    for kind in ('hash', 'type'):
        inputs = families(installed)
        bundle, resources, registry = inputs['lidar']
        if kind == 'hash':
            resources['lidar-package-defaults'] += b'\n'
        else:
            data = owner._plain(bundle.descriptor)
            data['leaves'][2]['value']['type'] = 'boolean'
            inputs['lidar'] = (DescriptorBundle(data, ()), resources, registry)
        with pytest.raises(ValueError):
            owner.combine_catalogs(inputs)
    config = tmp_path / 'config'
    config.mkdir()
    for name in ('catalog.yaml', 'nav2_params.yaml', 'mavros_setup.yaml', 'lidar.yaml',
                 'odometry_startup.yaml', 'ekf_local.yaml', 'ekf_global.yaml',
                 'navsat_transform.yaml'):
        if name != filename:
            (config / name).write_bytes((installed / 'config' / name).read_bytes())
    monkeypatch.setattr(owner, 'package_share', lambda package: tmp_path)
    with pytest.raises(FileNotFoundError):
        owner.load_catalog()
    (config / filename).write_text('[]')
    with pytest.raises(ValueError):
        owner.load_catalog()


def test_installed_nondefault_compiler_evidence_does_not_replace_raw_defaults(installed, tmp_path, monkeypatch):
    from mhseals_nav.odometry_configuration import resolve_odometry
    from mhseals_nav.localization_configuration import compile_localization, NODES
    monkeypatch.setenv('HOME', str(tmp_path))
    operator, overlay = tmp_path / 'startup.yaml', tmp_path / 'overlay.yaml'
    operator.write_text('ekf_delay_included: 9.0\n')
    overlay.write_text('ekf_local: {frequency: 37}\n')
    selected = resolve_odometry(installed / 'config/odometry_startup.yaml',
                                operator_file=operator, context='included')
    projected = compile_localization({name: installed / ('config/' + name + '.yaml')
                                      for name in NODES}, str(overlay))
    assert selected.delay == 9.0 and projected.parameters('ekf_local')['frequency'] == 37.0
    assert selected.inputs[-1]['sha256'] == sha256(operator.read_bytes()).hexdigest()
    assert projected.inputs[-1]['sha256'] == sha256(overlay.read_bytes()).hexdigest()
    baseline = owner.load_catalog()
    assert baseline.field('odometry.ekf_delay_included').default == 6.0
    assert baseline.field('localization.ekf_local.frequency').default == 30.0
