"""Static measurement contracts: do not fuse fields a sensor cannot measure."""
from pathlib import Path

import pytest
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


def test_zed_imu_is_a_yaw_rate_and_its_magnetometer_a_heading():
    """The ZED orientation yaw is relative, so only its gyro and mag heading are fused."""
    local, global_ = params('ekf_local.yaml'), params('ekf_global.yaml')
    assert local['imu1'] == '/imu/zed'
    assert [i for i, on in enumerate(local['imu1_config']) if on] == [11]
    for config, key in ((local, 'imu2'), (global_, 'imu1')):
        assert config[key] == '/imu/zed_heading'
        assert [i for i, on in enumerate(config[key + '_config']) if on] == [5]
    assert local['two_d_mode'] and local['frequency'] == 100.0


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


def test_mavros_relays_preserve_sensor_abstraction_at_safe_rates(tmp_path):
    launch = (ROOT / 'launch/odom.launch.py').read_text()
    assert "arguments=['/mavros/imu/data', '/imu/raw']" in launch
    # /gps/fix comes from the odometry adapter, relabelled to the antenna's frame.
    assert "'/mavros/global_position/raw/fix', '/gps/fix'" not in launch
    adapter = (ROOT / 'mhseals_nav/mavros_odometry.py').read_text()
    assert "'/mavros/global_position/raw/fix'" in adapter and "'/gps/fix'" in adapter
    assert "executable='mavros_odometry'" in launch
    assert "executable='mavros_setup'" in launch
    from mhseals_nav.mavros_configuration import ALIASES, resolve_setup
    parameters = resolve_setup(ROOT / 'config/mavros_setup.yaml')
    rates = {int(message_id): float(hz)
             for message_id, hz in (entry.split(':') for entry in parameters['message_rates'])}
    # 31 feeds /imu/raw at 100 Hz once SCHED_LOOP_RATE is 200; 30 is the 25 Hz fallback.
    assert rates == {33: 20., 30: 25., 31: 100., 24: 5., 27: 10., 1: 2., 193: 2., 65: 25.}
    assert 127 not in rates and 128 not in rates  # a Here4 never emits GPS_RTK
    assert ALIASES == {'mavros_odom_rate': (33,), 'mavros_imu_rate': (30, 31),
                       'mavros_gps_rate': (24,), 'mavros_rc_rate': (65,)}
    for alias in ALIASES:
        assert f"('{alias}', ''" in launch  # absence must preserve the YAML authority
    from mhseals_nav.endpoint_configuration import resolve_endpoints
    endpoints = resolve_endpoints(ROOT / 'config/endpoint_startup.yaml')
    authority = yaml.safe_load((ROOT / 'config/endpoint_startup.yaml').read_text())
    selected = authority['contexts']['standalone']
    assert selected['discovery'] == 'odom_cube_if00'
    fallback, observation = endpoints.mavros_parameters(False, tmp_path)
    assert fallback['fcu_url'] == (
        f"serial://{authority['serial_fallback']}:{selected['baud']}")
    assert observation['fallback'] is True
    by_id = tmp_path / 'serial/by-id'
    by_id.mkdir(parents=True)
    for name in ('usb-CubePilot_CubeOrange+_z-if00',
                 'usb-CubePilot_CubeOrange+_a-if00',
                 'usb-CubePilot_CubeOrange+_0-if01'):
        (by_id / name).touch()
    found, observation = endpoints.mavros_parameters(False, tmp_path)
    expected = by_id / 'usb-CubePilot_CubeOrange+_a-if00'
    assert found['fcu_url'] == f"serial://{expected}:{selected['baud']}"
    assert observation['path'] == str(expected) and observation['fallback'] is False


def test_imu_orientation_stdev_is_a_heading_sigma_not_the_mavros_default():
    """The EKF yaw comes from /imu/raw; MAVROS's 1 rad default left it near 12.7 deg."""
    config = yaml.safe_load((ROOT / 'config/mavros.yaml').read_text())
    stdev = config['/mavros/imu']['ros__parameters']['orientation_stdev']
    assert 0.02 <= stdev <= 0.2


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
    with pytest.raises(FileNotFoundError):
        ekf_overlay(str(tmp_path / 'none.yaml'), 'ekf_local')


def test_bad_overlay_refuses_instead_of_falling_back(tmp_path):
    from mhseals_nav.navigation_config import ekf_overlay
    for text in ('ekf_local: [unclosed', '- a\n- b\n', 'ekf_local: 5\n', 'ekf_local:\n  ros__parameters: 3\n'):
        path = tmp_path / 'ekf.yaml'
        path.write_text(text)
        with pytest.raises((ValueError, yaml.YAMLError)):
            ekf_overlay(str(path), 'ekf_local')


def test_ekf_delay_is_zero_standalone_and_six_seconds_from_odom():
    ekf = (ROOT / 'launch/ekf.launch.py').read_text()
    odom = (ROOT / 'launch/odom.launch.py').read_text()
    from mhseals_nav.odometry_configuration import resolve_odometry
    assert "DeclareLaunchArgument('ekf_delay', default_value=''" in ekf
    defaults = ROOT / 'config/odometry_startup.yaml'
    assert resolve_odometry(defaults, context='standalone').delay == 0.0
    assert resolve_odometry(defaults, context='included').delay == 6.0
    assert 'period=6.0' not in ekf
    assert "('ekf_startup_context', 'included'" in odom
    assert "'ekf_delay': '6.0'" not in odom


def test_wildcard_overlay_refuses_instead_of_silently_applying_nothing(tmp_path):
    from mhseals_nav.navigation_config import ekf_overlay
    path = tmp_path / 'ekf.yaml'
    path.write_text('/**:\n  ros__parameters:\n    frequency: 40.0\n')
    with pytest.raises(ValueError):
        ekf_overlay(str(path), 'ekf_local')


def test_overlay_normalizes_doubles_and_refuses_invalid_values(tmp_path):
    from mhseals_nav.navigation_config import ekf_overlay
    base = ROOT / 'config/ekf_local.yaml'
    path = tmp_path / 'ekf.yaml'
    path.write_text('ekf_local:\n  frequency: 30\n  sensor_timeout: .nan\n  transform_time_offset: -0.1\n')
    with pytest.raises(ValueError):
        ekf_overlay(str(path), 'ekf_local', str(base))
    path.write_text('ekf_local: {frequency: 30, transform_time_offset: -0.1}')
    params = ekf_overlay(str(path), 'ekf_local', str(base))
    assert params == {'frequency': 30.0, 'transform_time_offset': -0.1}
    assert isinstance(params['frequency'], float)
