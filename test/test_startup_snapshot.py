"""Immutable startup selection tests; no ROS runtime evidence is inferred."""
import json
from pathlib import Path

from mhseals_nav.startup_snapshot import select_snapshot
import pytest

ROOT = Path(__file__).parents[1]


def inputs(tmp_path):
    base = tmp_path / 'base.yaml'
    base.write_text((ROOT / 'config/nav2_params.yaml').read_text())
    limits = tmp_path / 'limits.yaml'
    limits.write_text('controller_server:\n  ros__parameters:\n'
                      '    controller_frequency: 20\n    FollowPath:\n'
                      '      vx_max: 0.6\n      vy_max: 0.12\n'
                      '      wz_max: 0.2\n')
    return base, limits


def test_bootstrap_and_scope_adoption(tmp_path):
    base, limits = inputs(tmp_path)
    root = tmp_path / 'snapshots'
    first = select_snapshot(root, 'session-one', base, limits,
                            sim='true', use_lidar='false')
    params = first['parameters']['controller_server']['ros__parameters']
    # Independent literal fixture, not a second call to the derivation helper.
    assert params['controller_frequency'] == 20.0
    assert params['FollowPath']['model_dt'] == .05
    assert [params['FollowPath'][k] for k in (
        'vx_max', 'vy_max', 'wz_max')] == [.6, .12, .2]
    assert all(s['sha256'] for s in first['baseline']['sources'])
    assert first['baseline']['sources'][1]['origins'][0]['line'] == 3
    assert 'invocation' not in first['baseline']
    limits.write_text('controller_server: []')
    same = select_snapshot(root, 'session-one', base, limits,
                           sim='false', use_lidar='true')
    assert same == first
    limits.unlink()
    fresh = select_snapshot(root, 'session-two', base, limits,
                            sim='false', use_lidar='true')
    assert fresh['invocation'] == {'sim': False, 'use_lidar': True}
    assert fresh['baseline']['sources'][1]['sha256'] is None
    receipt = json.loads((root / 'session-one/initializing/receipt.json').read_text())
    assert set(receipt['verified']) == {'nav2-baseline', 'nav2-compiler'}
    assert first['verification'] == 'pure-compiler-output; ROS unobserved'


@pytest.mark.parametrize('relative', [
    'active.json', 'initializing/nav2-compiler.json',
    'initializing/candidate.json'])
def test_corrupt_required_snapshot_refuses(tmp_path, relative):
    base, limits = inputs(tmp_path)
    root = tmp_path / 'snapshots'
    select_snapshot(root, 'session', base, limits, sim='1', use_lidar='0')
    (root / 'session' / relative).write_text('{}')
    with pytest.raises((ValueError, KeyError, TypeError)):
        select_snapshot(root, 'session', base, limits, sim='1', use_lidar='0')


def test_missing_required_base_and_invalid_overlay_do_not_reserve(tmp_path):
    base, limits = inputs(tmp_path)
    root = tmp_path / 'snapshots'
    limits.write_text('controller_server: []')
    with pytest.raises(ValueError):
        select_snapshot(root, 'bad', base, limits, sim='true', use_lidar='true')
    assert not (root / 'bad/initializing').exists()
    with pytest.raises(FileNotFoundError):
        select_snapshot(root, 'missing', tmp_path / 'missing.yaml', limits,
                        sim='true', use_lidar='true')
