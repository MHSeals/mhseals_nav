"""ROS-free startup layer boundary and immutable input identity."""
from pathlib import Path

from mhseals_nav.odometry_configuration import resolve_odometry

import pytest

BASE = Path(__file__).parents[1] / 'config/odometry_startup.yaml'


def test_defaults_and_precedence(tmp_path, monkeypatch):
    deployment = tmp_path / 'deployment.yaml'
    operator = tmp_path / 'operator.yaml'
    deployment.write_text('mavros_velocity_stddev: 0.7\n')
    operator.write_text('mavros_velocity_stddev: 0.9\n')
    monkeypatch.chdir(tmp_path)
    result = resolve_odometry(BASE, deployment, operator, {'mavros_velocity_stddev': ''})
    assert result.values['mavros_velocity_stddev'] == .9
    assert result.delay == 0
    assert resolve_odometry(BASE, context='included').delay == 6
    explicit = resolve_odometry(BASE, deployment, operator,
                                {'mavros_velocity_stddev': '1.2'})
    assert explicit.adapter_parameters['velocity_stddev'] == 1.2
    assert [x['layer'] for x in result.inputs] == ['package', 'deployment', 'operator']
    assert all(len(x['sha256']) == 64 and Path(x['path']).is_absolute() for x in result.inputs)
    with pytest.raises(TypeError):
        result.values['mavros_velocity_stddev'] = 4


@pytest.mark.parametrize('raw', [
    'mavros_velocity_stddev: true', 'mavros_velocity_stddev: .nan',
    'mavros_velocity_stddev: 0', 'ekf_delay_included: -1',
    'mavros_velocity_source: other', 'unknown: 1', '[]',
    'mavros_velocity_stddev: 1\nmavros_velocity_stddev: 2',
    'mavros_velocity_stddev: &x 1\nekf_delay_included: *x'])
def test_every_layer_validated_before_precedence(tmp_path, raw):
    selected = tmp_path / 'bad.yaml'
    selected.write_text(raw)
    with pytest.raises((ValueError, __import__('yaml').YAMLError)):
        resolve_odometry(BASE, selected, aliases={'mavros_velocity_stddev': '1'})


def test_explicit_missing_and_boolean_alias_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        resolve_odometry(BASE, operator_file=tmp_path / 'missing')
    with pytest.raises(ValueError):
        resolve_odometry(BASE, aliases={'ekf_delay': True})


def test_installed_share_lookup_from_unrelated_directory(tmp_path, monkeypatch):
    import sys
    import types
    share = tmp_path / 'installed/share/mhseals_nav'
    (share / 'config').mkdir(parents=True)
    (share / 'config/odometry_startup.yaml').write_bytes(BASE.read_bytes())
    package = types.ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    monkeypatch.chdir(tmp_path)
    assert resolve_odometry().inputs[0]['path'] == str(share / 'config/odometry_startup.yaml')
