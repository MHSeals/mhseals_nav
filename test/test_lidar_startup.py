"""Actual sensor and full-robot construction with inert launch objects."""
from pathlib import Path

from mhseals_nav.lidar_configuration import resolve_lidar

import pytest

import test_lidar_configuration as calibration_fixtures
from test_lidar_configuration import overlay

from test_robot_overlay import robot_factory

from test_sensor_startup import factory

ROOT = Path(__file__).parents[1]


@pytest.fixture
def share(tmp_path):
    """Reuse the calibrated installed-share fixture without an imported fixture name."""
    return calibration_fixtures.share.__wrapped__(tmp_path)


def context():
    """Represent unset launch arguments, not copied sensor defaults."""
    return {'sim': 'false', 'sensors_ignore': '', 'lidar_params_file': '',
            'lidar_deployment_file': '', 'lidar_required_frame': '',
            'velodyne_ip': '', 'velodyne_port': ''}


def sensor_factory(share):
    """Bind actual sensor factory to resolver using the inert installed fixture."""
    namespace = factory(lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    namespace['large_data_env'] = {'transport': 'preserved'}
    return namespace


@pytest.mark.parametrize('explicit', ['', '192.168.0.201'])
def test_robot_selected_layer_and_direct_equivalent(share, tmp_path, explicit):
    """Nondefault values survive the actual robot include and sensor factory."""
    selected = overlay(tmp_path, {'rpm': 450, 'endpoint': {'ip': '10.0.1.8', 'port': 9999}})
    entities = robot_factory()
    values = {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}
    assert values['velodyne_ip'] == values['velodyne_port'] == ''
    assert values['lidar_params_file'] == ''
    values.update(lidar_params_file=selected, velodyne_ip=explicit, sim='false')
    values['lidar_deployment_file'] = str(ROOT / 'config/lidar_robot.yaml')
    include = next(e for e in entities if hasattr(e, 'launch_arguments'))
    forwarded = {key: value.perform(values) if hasattr(value, 'perform') else value
                 for key, value in include.launch_arguments}
    namespace = sensor_factory(share)
    robot = namespace['sensor_nodes'](forwarded)
    direct_context = dict(context(), lidar_params_file=selected,
                          lidar_deployment_file=values['lidar_deployment_file'],
                          velodyne_ip=explicit, lidar_required_frame='lidar_link')
    direct = namespace['sensor_nodes'](direct_context)
    assert robot == direct
    driver, transform = [node['parameters'][0] for node in robot[:2]]
    assert driver['device_ip'] == (explicit or '10.0.1.8')
    assert driver['port'] == 9999 and driver['rpm'] == 450.
    assert driver['model'] == transform['model'] == 'VLP16'
    assert driver['frame_id'] == transform['frame_id'] == 'lidar_link'
    assert 'calibration' not in driver and 'rpm' not in transform
    assert robot[0]['additional_env'] == robot[1]['additional_env'] == {'transport': 'preserved'}
    assert robot[1]['remappings'] == [('velodyne_points', '/points')]


@pytest.mark.parametrize('selection', ['missing', 'invalid', 'duplicate'])
def test_invalid_selection_before_any_node(share, tmp_path, selection):
    """Reject bad requested bytes before either half of the pair is constructed."""
    path = tmp_path / selection
    if selection != 'missing':
        path.write_text('lidar: []' if selection == 'invalid' else 'lidar: {rpm: 1, rpm: 2}')
    namespace = sensor_factory(share)
    namespace['Node'] = lambda **kwargs: pytest.fail('constructed physical Node before validation')
    with pytest.raises((OSError, ValueError)):
        namespace['sensor_nodes'](dict(context(), lidar_params_file=str(path)))


def test_frame_is_sensor_local_but_full_robot_refuses(share, tmp_path):
    """Full robot's fixed URDF cannot silently diverge from cloud frames."""
    selected = overlay(tmp_path, {'frame': 'custom_lidar'})
    namespace = sensor_factory(share)
    standalone = namespace['sensor_nodes'](dict(context(), lidar_params_file=selected))
    assert all(n['parameters'][0]['frame_id'] == 'custom_lidar' for n in standalone[:2])
    namespace['Node'] = lambda **kwargs: pytest.fail('constructed incompatible physical Node')
    with pytest.raises(ValueError, match='URDF'):
        namespace['sensor_nodes'](dict(context(), lidar_params_file=selected,
                                       lidar_required_frame='lidar_link'))
    urdf = (ROOT / 'description/blastoise/blastoise.xacro').read_text()
    assert '<child link="lidar_link"/>' in urdf


@pytest.mark.parametrize('sim,ignored', [('true', ''), ('false', ' lidar ')])
def test_unused_lidar_selection_does_not_resolve(share, sim, ignored):
    """Simulation and ignored lidar retain their bounded previous closure."""
    namespace = factory(lambda *args, **kwargs: pytest.fail('resolved unused physical lidar'))
    nodes = namespace['sensor_nodes'](dict(context(), sim=sim, sensors_ignore=ignored,
                                         lidar_params_file='/missing'))
    assert [n['package'] for n in nodes] == (['ros_tcp_endpoint'] if sim == 'true' else [])


def test_actual_standalone_argument_defaults_are_unset():
    """Standalone launch declaration does not insert endpoint values over YAML."""
    entities = factory()['generate_launch_description']()
    defaults = {e['name']: e['default_value'] for e in entities if 'name' in e}
    assert all(defaults[key] == '' for key in (
        'lidar_params_file', 'lidar_deployment_file', 'lidar_required_frame',
        'velodyne_ip', 'velodyne_port'))


def test_corrupt_calibration_refuses_before_nodes(share):
    """Validate consumed calibration bytes before constructing the physical pair."""
    path = share('velodyne_pointcloud') / 'params/VLP16db.yaml'
    path.write_text('lasers: []')
    namespace = sensor_factory(share)
    namespace['Node'] = lambda **kwargs: pytest.fail('constructed unvalidated calibration node')
    with pytest.raises(ValueError):
        namespace['sensor_nodes'](context())
