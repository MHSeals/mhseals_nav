"""MAVROS descriptor defaults come from selected bytes, never a parallel table."""
from hashlib import sha256
from pathlib import Path

from mhseals_nav.catalog import mavros_catalog
import pytest

ROOT = Path(__file__).parents[1]
POLICY = (ROOT / 'config/catalog.yaml').read_bytes()


def test_named_family_selected_defaults_and_evidence():
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
    raw = b'mavros_setup: {ros__parameters: {message_rates: ["1:2", "1:3"]}}'
    with pytest.raises(ValueError):
        mavros_catalog(raw, resource='config/custom.yaml', policy_raw=POLICY)
