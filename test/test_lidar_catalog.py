"""Lidar descriptor/default parity and honest unsupported adjacent inventory."""
from pathlib import Path

from mhseals_nav.lidar_catalog import lidar_catalog

import pytest

ROOT = Path(__file__).parents[1]


def test_finite_named_catalog_uses_selected_baseline_bytes():
    """No independent numeric defaults exist in package metadata."""
    raw = (ROOT / 'config/lidar.yaml').read_bytes()
    policy = (ROOT / 'config/catalog.yaml').read_bytes()
    first = lidar_catalog(raw, resource='config/lidar.yaml', policy_raw=policy)
    assert len(first.rows) == 7
    assert first.field('lidar.rpm').default == 600.
    assert first.field('lidar.frame').default == 'lidar_link'
    assert len(first.field('lidar.model').descriptor['consumers']) == 2
    assert all(not row.writable for row in first.rows)
    assert first.field('lidar.calibration').descriptor['evidence'][0][
        'required_type'] == 'startup-inputs/unverified'
    assert 'missing-default-authority' in first.field('sensors.selection').blockers
    second = lidar_catalog(raw.replace(b'600.0', b'450.0'),
                           resource='config/custom.yaml', policy_raw=policy)
    assert second.field('lidar.rpm').default == 450.
    assert second.descriptor['default_sources'] != first.descriptor['default_sources']
    with pytest.raises(ValueError, match='incompatible'):
        lidar_catalog(raw, resource='config/lidar.yaml', policy_raw=policy,
                      consumer_versions={'velodyne_driver': 'unqualified'})
