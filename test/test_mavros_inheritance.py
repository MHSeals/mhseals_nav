"""Installed complete defaults precede independently checked partial selections."""
from pathlib import Path

from mhseals_nav.mavros_configuration import installed_setup_defaults, resolve_setup
import pytest
import yaml

from test_mavros_policy import inert_main

ROOT = Path(__file__).parents[1]


def layer(tmp_path, params):
    """Write raw selected parameters without expanding package defaults."""
    path = tmp_path / 'partial.yaml'
    path.write_text(yaml.safe_dump({'mavros_setup': {'ros__parameters': params}}))
    return path


def test_rate_only_selection_matches_actual_direct_node(tmp_path, monkeypatch):
    """Rate replacement and direct ROS parameter precedence retain all policy."""
    partial = {'message_rates': ['31:17', '27:3']}
    resolved = resolve_setup(layer(tmp_path, partial))
    direct = inert_main(monkeypatch, partial)
    assert direct.params == resolved
    assert resolved['message_rates'] == partial['message_rates']
    assert '33:20' not in resolved['message_rates']
    explicit = resolve_setup(layer(tmp_path, partial), {'mavros_imu_rate': '19'})
    assert explicit['message_rates'] == ['31:19', '27:3', '30:19']
    assert explicit['retry_attempts'] == installed_setup_defaults()['retry_attempts']


def test_policy_only_selection_inherits_rates(tmp_path):
    """Omitted rates inherit the canonical sequence intact."""
    baseline = installed_setup_defaults()
    resolved = resolve_setup(layer(tmp_path, {'retry_delay': .25}))
    assert resolved == dict(baseline, retry_delay=.25)


@pytest.mark.parametrize('bad', [None, True, -1, float('nan')])
def test_invalid_selected_field_cannot_be_masked(tmp_path, bad):
    """Aliases cannot conceal invalid supplied policy values."""
    with pytest.raises(ValueError):
        resolve_setup(layer(tmp_path, {'retry_delay': bad}), {'mavros_imu_rate': '19'})


@pytest.mark.parametrize('change', ['invalid', 'incomplete', 'missing'])
def test_bad_installed_baseline_cannot_be_masked(tmp_path, monkeypatch, change):
    """Even a complete valid selected file cannot repair a broken baseline."""
    from ament_index_python import packages
    selected = layer(tmp_path, installed_setup_defaults())
    baseline = tmp_path / 'installed' / 'config' / 'mavros_setup.yaml'
    baseline.parent.mkdir(parents=True)
    data = yaml.safe_load((ROOT / 'config/mavros_setup.yaml').read_text())
    params = data['mavros_setup']['ros__parameters']
    if change == 'invalid':
        params['retry_attempts'] = True
    if change == 'incomplete':
        del params['retry_attempts']
    if change != 'missing':
        baseline.write_text(yaml.safe_dump(data))
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda name: str(baseline.parents[1]))
    with pytest.raises((ValueError, OSError)):
        resolve_setup(selected)


@pytest.mark.parametrize('raw', [
    'mavros_setup: {ros__parameters: {retry_delay: null}}',
    'mavros_setup: {ros__parameters: {retry_delay: 1, retry_delay: 2}}',
    'mavros_setup: {ros__parameters: {future_policy: 1}}',
])
def test_partial_layer_remains_strict(tmp_path, raw):
    """Partial support never weakens null, duplicate or unknown-key rejection."""
    path = tmp_path / 'invalid.yaml'
    path.write_text(raw)
    with pytest.raises((ValueError, yaml.YAMLError)):
        resolve_setup(path)
