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


def test_global_filter_fuses_the_raw_sources_not_the_local_filters_output():
    config = params('ekf_global.yaml')
    assert config['odom0'] == '/odom/mavros'
    assert config['odom0_config'][6:9] == [True, True, False]
    assert not config['odom0_config'][11]  # /odom/mavros does not measure yaw rate
    assert config['imu0_config'][11]


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


def test_mavros_global_defaults_do_not_relabel_sensor_frames():
    config = yaml.safe_load((ROOT / 'config/mavros.yaml').read_text())
    assert 'frame_id' not in config['/**']['ros__parameters']


def test_mavros_relays_preserve_sensor_abstraction_at_safe_rates():
    launch = (ROOT / 'launch/odom.launch.py').read_text()
    for source, destination in (
        ('/mavros/imu/data', '/imu/raw'),
        ('/mavros/global_position/raw/fix', '/gps/fix'),
    ):
        assert f"arguments=['{source}', '{destination}']" in launch
    assert "executable='mavros_odometry'" in launch
    assert "executable='mavros_setup'" in launch
    assert "(33, LaunchConfiguration('mavros_odom_rate')" in launch
    assert "(127," not in launch and "(128," not in launch  # a Here4 never emits GPS_RTK
    assert "('mavros_odom_rate', '20.0'" in launch
    assert "('mavros_imu_rate', '20.0'" in launch
    assert "('mavros_gps_rate', '5.0'" in launch
    assert '/dev/serial/by-id/usb-CubePilot_CubeOrange+_*-if00' in launch


def test_mavros_loads_only_the_plugins_the_stack_uses():
    config = yaml.safe_load((ROOT / 'config/mavros.yaml').read_text())
    plugins = config['/**']['ros__parameters']['plugin_allowlist']
    assert {'sys_status', 'imu', 'global_position', 'gps_status', 'rc_io',
            'setpoint_velocity'} <= set(plugins)


def test_ekfs_launch_on_their_own_and_odom_includes_them_unless_disabled():
    ekf = (ROOT / 'launch/ekf.launch.py').read_text()
    assert "('odometry/filtered', '/odom/global')" in ekf.split('navsat =', 1)[1].split('local =', 1)[0]
    assert "('odometry/filtered', '/odom/local')" in ekf
    assert ekf.count("'ekf_node'") == 2
    odom = (ROOT / 'launch/odom.launch.py').read_text()
    assert "('start_ekf', 'true'" in odom and "'ekf.launch.py'" in odom
    assert 'ekf_node' not in odom and 'navsat_transform_node' not in odom


def test_overlay_overrides_one_node_and_a_missing_file_is_no_overlay(tmp_path):
    from mhseals_nav.navigation_config import ekf_overlay
    path = tmp_path / 'ekf.yaml'
    path.write_text('ekf_local:\n  frequency: 40.0\nekf_global:\n  ros__parameters:\n    two_d_mode: false\n')
    assert ekf_overlay(str(path), 'ekf_local') == {'frequency': 40.0}
    assert ekf_overlay(str(path), 'ekf_global') == {'two_d_mode': False}
    assert ekf_overlay(str(path), 'navsat_transform') == {}
    assert ekf_overlay(str(tmp_path / 'none.yaml'), 'ekf_local') == {}
