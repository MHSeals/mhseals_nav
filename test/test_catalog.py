"""Pure installed-resource descriptor/compiler parity, without ROS/plugin claims."""
from copy import deepcopy
from hashlib import sha256
import os
from pathlib import Path

from mhseals_nav.catalog import COMPILER, navigation_catalog, REGISTRY
from mhseals_nav.navigation_config import validated_merge_limits
import pytest
import yaml

# Gate sets this to the archived installed share resource, not checkout data.
RESOURCE = Path(os.environ.get('NAV_CATALOG_RESOURCE',
                               Path(__file__).parents[1] / 'config/nav2_params.yaml'))


POLICY = RESOURCE.with_name('catalog.yaml').read_bytes()


def bundle(base):
    return navigation_catalog(yaml.safe_dump(base).encode(),
                              resource='config/selected.yaml', policy_raw=POLICY)


def test_reference_and_changed_resource():
    raw = RESOURCE.read_bytes()
    first = navigation_catalog(raw, resource='config/nav2_params.yaml', policy_raw=POLICY)
    row = first.field('navigation/controller_server/ros__parameters/controller_frequency')
    params = yaml.safe_load(raw)['controller_server']['ros__parameters']
    assert row.default == params['controller_frequency']
    assert row.descriptor['default_ref']['source_hash'] == sha256(raw).hexdigest()
    line = raw.splitlines()[row.descriptor['default_ref']['line'] - 1]
    assert line.lstrip().startswith(b'controller_frequency:')
    changed = yaml.safe_load(raw)
    changed['controller_server']['ros__parameters']['controller_frequency'] = 20.0
    second = bundle(changed)
    assert second.rows[0].default == 20.0
    assert (second.descriptor['default_sources'][0]['sha256'] !=
            first.descriptor['default_sources'][0]['sha256'])
    assert all(not r.writable for r in first.rows)
    assert any('unresolved-consumer-version' in b for b in row.blockers)


@pytest.mark.parametrize('smoother', [True, False])
def test_renamed_nondefault_compiler_closure(smoother):
    base = yaml.safe_load(RESOURCE.read_bytes())
    controller = base['controller_server']['ros__parameters']
    controller['Renamed'] = controller.pop('FollowPath')
    controller['controller_plugins'] = ['Renamed']
    if not smoother:
        base.pop('velocity_smoother')
    inventory = bundle(base)
    explicit = {'controller_server': {'ros__parameters': {
        'controller_frequency': 20.0,
        'Renamed': {'vx_max': 0.6, 'vy_max': 0.12, 'wz_max': 0.2}}}}
    effective = REGISTRY[COMPILER](base, explicit)
    assert effective == validated_merge_limits(base, explicit)
    assert effective['controller_server']['ros__parameters']['Renamed']['model_dt'] == 0.05
    assert effective['behavior_server'] == base['behavior_server']
    model = inventory.field('navigation/controller_server/ros__parameters/Renamed/model_dt')
    assert model.descriptor['authoring']['scopes'] == ()
    assert model.descriptor['authoring']['controlling_keys']
    output = model.descriptor['consumers'][0]['outputs']
    assert any('velocity_smoother' in key for key in output) == smoother
    if smoother:
        velocity = effective['velocity_smoother']['ros__parameters']['max_velocity']
        assert velocity == [0.6, 0.12, 0.2]
    bad = deepcopy(explicit)
    bad['controller_server']['ros__parameters']['Renamed']['model_dt'] = 0.1
    with pytest.raises(ValueError, match='conflicts'):
        REGISTRY[COMPILER](base, bad)
    with pytest.raises(ValueError, match='unknown parameter'):
        REGISTRY[COMPILER](base, {'controller_server': {'ros__parameters': {'unknown': 1}}})
    with pytest.raises(ValueError, match='unknown catalog leaf'):
        inventory.field('navigation/controller_server/ros__parameters/Renamed/unknown')


def test_unknown_versions_and_non_mppi():
    raw = RESOURCE.read_bytes()
    with pytest.raises(ValueError, match='incompatible consumer version'):
        navigation_catalog(raw, resource='config/nav2_params.yaml', policy_raw=POLICY,
                           consumer_versions={'mppi': 'unqualified-build'})
    base = yaml.safe_load(raw)
    base['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'other::Controller'
    inventory = bundle(base)
    assert not any('/FollowPath/' in r.descriptor['canonical_key'] for r in inventory.rows)
    explicit = {'controller_server': {'ros__parameters': {'controller_frequency': 20.0}}}
    effective = REGISTRY[COMPILER](base, explicit)
    assert (effective['controller_server']['ros__parameters']['FollowPath'] ==
            base['controller_server']['ros__parameters']['FollowPath'])


@pytest.mark.parametrize('name', ['FollowPath', 'Renamed'])
def test_always_derived_shim_metadata(name):
    """Match shim authoring relationships to canonical compiler conflicts."""
    base = yaml.safe_load(RESOURCE.read_bytes())
    controller = base['controller_server']['ros__parameters']
    controller[name] = controller.pop('FollowPath')
    controller['controller_plugins'] = [name]
    inventory = bundle(base)
    prefix = 'navigation/controller_server/ros__parameters/' + name + '/'
    for output, source in [('rotate_to_heading_angular_vel', 'wz_max'),
                           ('max_angular_accel', 'az_max')]:
        authoring = inventory.field(prefix + output).descriptor['authoring']
        assert authoring['layers'] == ('derived',)
        assert authoring['scopes'] == ()
        assert authoring['controlling_keys'] == (prefix + source,)
        explicit = {'controller_server': {'ros__parameters': {name: {source: 0.2}}}}
        effective = REGISTRY[COMPILER](base, explicit)
        assert effective['controller_server']['ros__parameters'][name][output] == 0.2
        explicit['controller_server']['ros__parameters'][name][output] = 0.1
        with pytest.raises(ValueError, match='conflicts with canonical lowering'):
            REGISTRY[COMPILER](base, explicit)


def test_consumer_credit_per_key():
    base = yaml.safe_load(RESOURCE.read_bytes())
    inventory = bundle(base)
    prefix = 'navigation/controller_server/ros__parameters/FollowPath/'

    def ids(key):
        return [c['id'] for c in inventory.field(prefix + key).descriptor['consumers']]

    assert ids('rotate_to_heading_angular_vel') == ['rotation_shim']
    assert ids('vx_max') == ['mppi']
    spin = 'behavior_server/ros__parameters/max_rotational_vel'
    assert not any(spin in r.descriptor['canonical_key'] for r in inventory.rows)
