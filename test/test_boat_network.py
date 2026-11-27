"""boat-network.env: the one parser for the boat LAN's addresses, and the lidar's LIDAR_IP."""
from launch_harness import harness  # noqa: F401
from mhseals_nav import boat_network
import pytest
from test_lidar_configuration import share  # noqa: F401


def test_values_read_like_the_shell(tmp_path):
    env = tmp_path / 'boat-network.env'
    env.write_text('# comment\nLIDAR_IP=192.168.0.201  # sensor\n'
                   'STATIC_LEASES="aa=blastoise-odroid=192.168.0.152 '
                   'bb=blastoise-jetson=192.168.0.153"\n'
                   'BROKEN="unclosed\nEMPTY=\n')
    assert boat_network.value('LIDAR_IP', env) == '192.168.0.201'
    assert boat_network.value('BROKEN', env) == boat_network.value('EMPTY', env) == ''
    assert boat_network.value('MISSING', env) == ''
    assert boat_network.leases(env) == {'blastoise-odroid': '192.168.0.152',
                                        'blastoise-jetson': '192.168.0.153'}


def test_the_workspace_file_and_a_missing_one(tmp_path, monkeypatch):
    monkeypatch.undo()      # the real path(), not the conftest stand-in
    monkeypatch.delenv('ASTRO_NETWORK_ENV', raising=False)
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert boat_network.path() == tmp_path / 'config' / 'boat-network.env'
    assert boat_network.value('LIDAR_IP') == ''
    monkeypatch.delenv('ROS_WS')
    assert boat_network.path() is None and boat_network.values() == {}


def test_the_vessel_network_env_wins_over_the_workspace_file(tmp_path, monkeypatch, capsys):
    monkeypatch.undo()
    legacy = tmp_path / 'config' / 'boat-network.env'
    legacy.parent.mkdir()
    legacy.write_text('LIDAR_IP=10.0.0.1\n')
    vessel = tmp_path / 'network.env'
    vessel.write_text('LIDAR_IP=10.0.0.2\n')
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    monkeypatch.setenv('ASTRO_NETWORK_ENV', str(vessel))
    assert boat_network.path() == vessel and boat_network.value('LIDAR_IP') == '10.0.0.2'
    assert capsys.readouterr().err == ''
    monkeypatch.delenv('ASTRO_NETWORK_ENV')
    assert boat_network.value('LIDAR_IP') == '10.0.0.1'


def test_a_missing_vessel_network_env_warns_once_and_never_falls_back(
        tmp_path, monkeypatch, capsys):
    monkeypatch.undo()
    legacy = tmp_path / 'config' / 'boat-network.env'
    legacy.parent.mkdir()
    legacy.write_text('LIDAR_IP=10.0.0.1\n')
    gone = tmp_path / 'gone.env'
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    monkeypatch.setenv('ASTRO_NETWORK_ENV', str(gone))
    assert boat_network.path() == gone and boat_network.values() == {}
    boat_network.path()
    err = capsys.readouterr().err
    assert err.count('ASTRO_NETWORK_ENV') == 1 and str(gone) in err


def test_the_lidar_launch_takes_lidar_ip_and_refuses_without_one(
        harness, share, boat_network_env, monkeypatch):  # noqa: F811
    from mhseals_nav.lidar_configuration import resolve_lidar
    perception = harness.load('perception.launch.py')
    monkeypatch.setattr(perception, 'resolve_lidar',
                        lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))

    def driver(**arguments):
        nodes = harness.run('perception.launch.py', camera='none', **arguments)
        return nodes[0]['parameters'][0]['device_ip']
    boat_network_env.write_text('LIDAR_IP=10.9.8.7\n')
    assert driver() == '10.9.8.7'
    assert driver(velodyne_ip='10.1.1.1') == '10.1.1.1'
    boat_network_env.write_text('')
    harness.built.clear()
    with pytest.raises(ValueError, match='LIDAR_IP'):
        driver()
    assert harness.built == []
