"""Pure layered lidar inputs and calibration identity; no sensor resources."""
from pathlib import Path

from mhseals_nav.lidar_configuration import CORRECTIONS, resolve_lidar, validate_calibration

import pytest

import yaml

ROOT = Path(__file__).parents[1]


@pytest.fixture
def share(tmp_path):
    """Supply an inert installed calibration package and real canonical defaults."""
    path = tmp_path / 'params/VLP16db.yaml'
    path.parent.mkdir()
    path.write_text(yaml.safe_dump({
        'num_lasers': 16, 'distance_resolution': .002,
        'lasers': [dict.fromkeys(CORRECTIONS, 0.) | {'laser_id': i} for i in range(16)]}))
    return lambda package: ROOT if package == 'mhseals_nav' else tmp_path


def overlay(tmp_path, data):
    """Write a selected layer retaining explicit nulls and types."""
    path = tmp_path / 'selected.yaml'
    path.write_text(yaml.safe_dump({'lidar': data}))
    return str(path)


def test_defaults_deployment_selection_and_explicit_aliases(share, tmp_path):
    """Preserve both historical endpoints as visible, separately hashed layers."""
    standalone = resolve_lidar(share=share)
    robot = resolve_lidar(deployment=ROOT / 'config/lidar_robot.yaml', share=share)
    assert standalone['driver']['device_ip'] == '192.168.0.201'
    assert robot['driver']['device_ip'] == '192.168.1.201'
    selected = overlay(tmp_path, {'endpoint': {'ip': '10.1.2.3'}, 'rpm': 420})
    custom = resolve_lidar(selected, ROOT / 'config/lidar_robot.yaml',
                           {'velodyne_port': '1234', 'velodyne_ip': ''}, share=share)
    assert custom['driver']['device_ip'] == '10.1.2.3'
    assert custom['driver']['port'] == 1234
    assert custom['driver']['rpm'] == 420.
    assert len(custom['sources']) == 3
    assert custom['calibration']['sha256']


@pytest.mark.parametrize('patch', [{'model': 'HDL64'}, {'frame': ''}, {'rpm': True},
    {'rpm': float('nan')}, {'rpm': 0}, {'rpm': -2}, {'endpoint': {'port': True}},
    {'endpoint': {'port': 0}}, {'endpoint': {'port': 65536}}, {'endpoint': {'ip': 'bad'}},
    {'unknown': 3}, {'calibration': {'file': '/missing'}}])
def test_invalid_selected_layer_refuses(share, tmp_path, patch):
    """Invalid selections never fall back to installed defaults."""
    with pytest.raises((ValueError, OSError)):
        resolve_lidar(overlay(tmp_path, patch), share=share)


def test_duplicate_and_missing_selected_refuse(share, tmp_path):
    """Reject duplicate explicit keys and nonexistent nonempty selections."""
    path = tmp_path / 'duplicate.yaml'
    path.write_text('lidar: {rpm: 200, rpm: 300}')
    with pytest.raises(ValueError):
        resolve_lidar(path, share=share)
    with pytest.raises(OSError):
        resolve_lidar(tmp_path / 'missing', share=share)


def test_calibration_schema_refuses_incompatible_shape():
    """Malformed and other-model calibration cannot reach transform construction."""
    for raw in ('[]', '{}', 'num_lasers: 32', 'num_lasers: 16\nnum_lasers: 16'):
        with pytest.raises(ValueError):
            validate_calibration(raw)


def test_custom_calibration_relative_origin_and_content_identity(share, tmp_path):
    """Relative calibration follows its authoring layer and records exact bytes."""
    from hashlib import sha256
    raw = (share('velodyne_pointcloud') / 'params/VLP16db.yaml').read_bytes()
    path = tmp_path / 'custom.yaml'
    path.write_bytes(raw)
    selected = overlay(tmp_path, {'calibration': {'file': 'custom.yaml'}})
    result = resolve_lidar(selected, overrides={'velodyne_port': '1234'}, share=share)
    assert result['transform']['calibration'] == str(path)
    assert result['calibration'] == {'path': str(path), 'sha256': sha256(raw).hexdigest()}
    origins = result['resolution'].origins
    assert origins['lidar.calibration'][-1].source == selected
    assert origins['lidar.endpoint.port'][-1].layer == 'session'
    path.write_bytes(raw.replace(b'distance_resolution: 0.002', b'distance_resolution: 0.003'))
    assert resolve_lidar(selected, share=share)['calibration']['sha256'] != sha256(raw).hexdigest()


@pytest.mark.parametrize('alias,value', [('velodyne_port', '1.5'), ('velodyne_port', '-1'),
    ('velodyne_port', '65536'), ('velodyne_ip', 'not-an-IP'), ('unknown', '')])
def test_invalid_explicit_endpoint_refuses(share, alias, value):
    """Explicit aliases share the final typed endpoint validator."""
    with pytest.raises(ValueError):
        resolve_lidar(overrides={alias: value}, share=share)


def test_calibration_duplicate_laser_and_nonfinite_refuse(share):
    """A readable YAML calibration must still satisfy the bounded structural contract."""
    path = share('velodyne_pointcloud') / 'params/VLP16db.yaml'
    data = yaml.safe_load(path.read_bytes())
    data['lasers'][1]['laser_id'] = 0
    with pytest.raises(ValueError):
        validate_calibration(yaml.safe_dump(data))
    data['lasers'][1]['laser_id'] = 1
    data['lasers'][0]['vert_correction'] = float('inf')
    with pytest.raises(ValueError):
        validate_calibration(yaml.safe_dump(data))
