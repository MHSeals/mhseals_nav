"""boat-network.env: the one parser for the boat LAN's addresses, and the lidar's LIDAR_IP."""
import pytest

from mhseals_nav import boat_network
from test_lidar_configuration import share  # noqa: F401
from test_sensor_startup import factory


def test_values_read_like_the_shell(tmp_path):
    env = tmp_path / 'boat-network.env'
    env.write_text('# comment\nLIDAR_IP=192.168.0.201  # sensor\n'
                   'STATIC_LEASES="aa=blastoise-odroid=192.168.0.152 bb=blastoise-jetson=192.168.0.153"\n'
                   'BROKEN="unclosed\nEMPTY=\n')
    assert boat_network.value('LIDAR_IP', env) == '192.168.0.201'
    assert boat_network.value('BROKEN', env) == boat_network.value('EMPTY', env) == ''
    assert boat_network.value('MISSING', env) == ''
    assert boat_network.leases(env) == {'blastoise-odroid': '192.168.0.152',
                                        'blastoise-jetson': '192.168.0.153'}


def test_the_workspace_file_and_a_missing_one(tmp_path, monkeypatch):
    monkeypatch.undo()      # the real path(), not the conftest stand-in
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert boat_network.path() == tmp_path / 'config' / 'boat-network.env'
    assert boat_network.value('LIDAR_IP') == ''
    monkeypatch.delenv('ROS_WS')
    assert boat_network.path() is None and boat_network.values() == {}


def context(**values):
    return {'sim': 'false', 'sensors_ignore': '', 'lidar_params_file': '', 'lidar_deployment_file': '',
            'lidar_required_frame': '', 'velodyne_ip': '', 'velodyne_port': '', **values}


def test_the_lidar_launch_takes_lidar_ip_and_refuses_without_one(share, boat_network_env):
    from mhseals_nav.lidar_configuration import resolve_lidar
    namespace = factory(lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    boat_network_env.write_text('LIDAR_IP=10.9.8.7\n')
    assert namespace['sensor_nodes'](context())[0]['parameters'][0]['device_ip'] == '10.9.8.7'
    explicit = namespace['sensor_nodes'](context(velodyne_ip='10.1.1.1'))
    assert explicit[0]['parameters'][0]['device_ip'] == '10.1.1.1'
    boat_network_env.write_text('')
    namespace['Node'] = lambda **kwargs: pytest.fail('constructed a lidar node without an address')
    with pytest.raises(ValueError, match='LIDAR_IP'):
        namespace['sensor_nodes'](context())
