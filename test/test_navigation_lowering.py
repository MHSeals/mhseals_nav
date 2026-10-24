"""Literal expected consumer inputs for authoritative pure lowering."""
from copy import deepcopy
from pathlib import Path

from mhseals_nav.navigation_config import validated_merge_limits
import pytest
import yaml


@pytest.fixture
def base():
    path = Path(__file__).parents[1] / 'config/nav2_params.yaml'
    return yaml.safe_load(path.read_text())


def patch():
    return {'controller_server': {'ros__parameters': {
        'controller_frequency': 20, 'FollowPath': {
            'vx_max': .6, 'vy_max': .12, 'wz_max': .2}}}}


def test_nondefault_complete_consumer_fixture(base):
    before = deepcopy(base)
    result = validated_merge_limits(base, patch())
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['model_dt'] == .05
    assert [follow[k] for k in ('vx_std', 'vy_std', 'wz_std')] == [.15, .1, .2]
    assert follow['prune_distance'] == 3.0
    assert follow['rotate_to_heading_angular_vel'] == .2
    assert follow['max_angular_accel'] == .5
    smoother = result['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [.6, .12, .2]
    assert smoother['min_velocity'] == [-.1, -.12, -.2]
    assert result['behavior_server'] == base['behavior_server']
    assert result['planner_server'] == base['planner_server']
    assert base == before


@pytest.mark.parametrize('key,value', [('model_dt', .1), ('wz_std', .3),
                                       ('rotate_to_heading_angular_vel', .4),
                                       ('prune_distance', 9.)])
def test_conflicting_explicit_derived_value_refused(base, key, value):
    explicit = patch()
    explicit['controller_server']['ros__parameters']['FollowPath'][key] = value
    with pytest.raises(ValueError, match='conflicts with canonical lowering'):
        validated_merge_limits(base, explicit)


def test_smoother_conflict_and_optional_absence(base):
    explicit = patch()
    explicit['velocity_smoother'] = {'ros__parameters': {
        'max_velocity': [.2, .1, .1]}}
    with pytest.raises(ValueError, match='conflicts with canonical lowering'):
        validated_merge_limits(base, explicit)
    del base['velocity_smoother']
    result = validated_merge_limits(base, patch())
    assert 'velocity_smoother' not in result


def test_plugin_map_and_behavior_limit_are_independent(base):
    base['behavior_server']['ros__parameters']['max_rotational_vel'] = .7
    result = validated_merge_limits(base, patch())
    assert result['behavior_server']['ros__parameters']['max_rotational_vel'] == .7
    controller = base['controller_server']['ros__parameters']
    controller['Custom'] = controller.pop('FollowPath')
    controller['controller_plugins'] = ['Custom']
    result = validated_merge_limits(base, {})
    assert result['controller_server'] == base['controller_server']


def test_required_base_is_checked_without_an_overlay(tmp_path):
    from mhseals_nav.startup_snapshot import select_snapshot
    path = tmp_path / 'bad.yaml'
    path.write_text('controller_server: []')
    with pytest.raises(ValueError, match='required mapping'):
        select_snapshot(tmp_path / 'snapshots', 'new', path, '',
                        sim='true', use_lidar='true')
