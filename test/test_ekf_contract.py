"""Static measurement contracts: do not fuse fields a sensor cannot measure."""
from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]


def params(name):
    return yaml.safe_load((ROOT / 'config' / name).read_text())['/**']['ros__parameters']


def test_all_measurement_masks_have_fifteen_booleans():
    for name in ('ekf_local.yaml', 'ekf_global.yaml'):
        for key, mask in params(name).items():
            if key.endswith('_config'):
                assert len(mask) == 15, (name, key, len(mask))
                assert all(type(value) is bool for value in mask)


def test_gps_only_supplies_planar_position():
    mask = params('ekf_global.yaml')['odom1_config']
    assert [index for index, enabled in enumerate(mask) if enabled] == [0, 1]


def test_fc_heading_not_magnetometer_free_reintegration():
    local = params('ekf_local.yaml')
    assert local['imu0'] == '/imu/raw'  # MAVROS /imu/data, despite legacy topic name
    assert local['imu0_config'][5] and local['imu0_config'][11]


def test_no_duplicate_absolute_position_in_local_or_global_reference():
    for name in ('ekf_local.yaml', 'ekf_global.yaml'):
        assert not any(params(name)['odom0_config'][:3])


def test_nav2_rate_and_tf_contract():
    local, global_ = params('ekf_local.yaml'), params('ekf_global.yaml')
    nav2 = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    controller_hz = nav2['controller_server']['ros__parameters']['controller_frequency']
    assert local['frequency'] >= controller_hz
    assert global_['frequency'] >= controller_hz
    assert local['world_frame'] == 'odom'
    assert global_['world_frame'] == 'map'
    for config in (local, global_):
        assert config['publish_tf']
        assert not any(config.get(key) for key in ('odom2', 'odom3'))


def test_mavros_imu_frame_is_scoped_to_plugin():
    config = yaml.safe_load((ROOT / 'config/mavros.yaml').read_text())
    assert 'frame_id' not in config['/**']['ros__parameters']
    assert config['/**/imu']['ros__parameters']['frame_id'] == 'base_link'
