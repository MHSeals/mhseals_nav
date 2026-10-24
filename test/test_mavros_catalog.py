"""MAVROS descriptor defaults come from selected bytes, never a parallel table."""
from hashlib import sha256
from pathlib import Path

from mhseals_nav.catalog import mavros_catalog

import pytest

ROOT = Path(__file__).parents[1]
POLICY = (ROOT / 'config/catalog.yaml').read_bytes()


def test_named_family_selected_defaults_and_evidence():
    """Named family selected defaults and evidence."""
    raw = (ROOT / 'config/mavros_setup.yaml').read_bytes()
    bundle = mavros_catalog(raw, resource='config/mavros_setup.yaml', policy_raw=POLICY)
    row = bundle.field('mavros.intervals')
    assert row.default == ('33:20', '30:50', '31:50', '24:5', '27:10', '1:2', '193:2', '65:25')
    assert not row.writable
    assert row.descriptor['default_ref']['source_hash'] == sha256(raw).hexdigest()
    assert row.descriptor['evidence'][0]['required_type'] == 'requested/unverified'
    assert 'runtime-authority-not-provided' in row.blockers
    changed = raw.replace(b'27:10', b'27:7')
    second = mavros_catalog(changed, resource='config/custom.yaml', policy_raw=POLICY)
    assert second.field('mavros.intervals').default[4] == '27:7'
    with pytest.raises(ValueError, match='incompatible'):
        mavros_catalog(raw, resource='config/mavros_setup.yaml', policy_raw=POLICY,
                       consumer_versions={'mavros_setup': 'unqualified-build'})


def test_descriptor_reuses_strict_schema():
    """Descriptor reuses strict schema."""
    raw = b'mavros_setup: {ros__parameters: {message_rates: ["1:2", "1:3"]}}'
    with pytest.raises(ValueError):
        mavros_catalog(raw, resource='config/custom.yaml', policy_raw=POLICY)


def test_policy_catalog_uses_selected_consumed_values():
    raw = (ROOT / 'config/mavros_setup.yaml').read_bytes()
    bundle = mavros_catalog(raw.replace(b'rc_min_hz: 10.0', b'rc_min_hz: 3.0'),
                            resource='custom.yaml', policy_raw=POLICY)
    assert bundle.field('mavros.rc_monitor.rc_min_hz').default == 3.
    assert bundle.field('mavros.request_policy.retry_attempts').default == 3
    assert bundle.field('mavros.connection_identity.fcu_sysid').default == 1


@pytest.mark.parametrize('identity,output,dependency,line', [
    ('mavros.request_policy.retry_attempts', '/mavros/set_message_interval',
     'MAVROS-request-target-external', 214),
    ('mavros.rc_monitor.rc_min_hz', 'rc-rate-diagnostic-log', '/mavros/rc/in', 260),
    ('mavros.connection_identity.fcu_sysid', '/fcu/ekf_status', '/uas1/mavlink_source', 224),
])
def test_policy_family_closure_and_exact_implementation_provenance(identity, output, dependency, line):
    raw = (ROOT / 'config/mavros_setup.yaml').read_bytes()
    row = mavros_catalog(raw, resource='config/mavros_setup.yaml', policy_raw=POLICY).field(identity)
    consumer = row.descriptor['consumers'][0]
    assert output in consumer['outputs']
    assert dependency in consumer['dependencies']
    if not identity.startswith('mavros.request_policy.'):
        assert '/mavros/set_message_interval' not in consumer['outputs']
        assert 'MAVROS-request-target-external' not in consumer['dependencies']
    coverage = row.descriptor['coverage'][0]
    assert coverage['revision'] == '27fae9069aa839671f25f2f3a08e49713394be70'
    assert coverage['path'] == 'mhseals_nav/mavros_setup.py'
    assert coverage['line'] == line
    assert not row.writable
    assert row.descriptor['evidence'][0]['required_type'] == 'requested/unverified'
